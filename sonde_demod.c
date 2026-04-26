// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sonde_demod.c: Radiosonde (RS41) GFSK demodulator
//
// RS41 protocol implementation per Vaisala RS41 specification:
//   IQ samples (2.4 MSPS) → FM discriminator → decimation → lowpass filter
//   → PLL bit clock recovery at 4800 baud → syncword detection (0x086BCA11)
//   → frame extraction (320 bytes) → XOR de-whitening (offset 8)
//   → Reed-Solomon RS(255,231) error correction (2 interleaved codewords)
//   → CRC-16 CCITT subblock verification → GPS/PTU data parsing
//
// Reed-Solomon: GF(2^8) with primitive polynomial x^8+x^4+x^3+x^2+1 (0x11D)
//   Two interleaved RS blocks, each with 24 parity bytes (t=12 correction)
//   Berlekamp-Massey + Chien search + Forney algorithm
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// Sub-block format: ID(1) + LEN(1) + data(LEN) + CRC-16(2)
//   CRC-16 CCITT: poly=0x1021, init=0xFFFF, covers data bytes only
//
// Based on RS41 protocol analysis by rs1729 and radiosonde_auto_rx project.

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "sonde_demod.h"

// ======================== Constants ========================

// Decimation: 2.4 MHz → ~57 kHz (factor 42)
#define SONDE_DECIM       42
#define SONDE_IF_RATE     (2400000 / SONDE_DECIM)  // ~57142 Hz
#define SONDE_SPS         ((double)SONDE_IF_RATE / SONDE_BAUD_RATE)  // ~11.9

// RS41 syncword: 0x08 0x6B 0xCA 0x11 (after Manchester/bit-inversion)
static const uint8_t RS41_SYNC[] = { 0x08, 0x6B, 0xCA, 0x11 };
#define RS41_SYNC_BITS    32

// Syncword correlation threshold (out of 32 bits)
// Require at least 30/32 matching bits to reduce false positives
#define SYNC_THRESHOLD    30

// RS41 XOR whitening mask (64 bytes, repeating)
// Applied with offset 8 (frame data starts after 8-byte sync header)
static const uint8_t rs41_whitening[] = {
    0x96, 0x83, 0x3E, 0x51, 0xB1, 0x49, 0x08, 0x98,
    0x32, 0x05, 0x59, 0x0E, 0xF9, 0x44, 0xC6, 0x26,
    0x21, 0x60, 0xC2, 0xEA, 0x79, 0x5D, 0x6D, 0xA1,
    0x54, 0x69, 0x47, 0x0C, 0xDC, 0xE8, 0x5C, 0xF1,
    0xF7, 0x76, 0x82, 0x7F, 0x07, 0x99, 0xA2, 0x2C,
    0x93, 0x7C, 0x30, 0x63, 0xF5, 0x10, 0x2E, 0x61,
    0xD0, 0xBC, 0xB4, 0xB6, 0x06, 0xAA, 0xF4, 0x23,
    0x78, 0x6E, 0x3B, 0xAE, 0xBF, 0x7B, 0x4C, 0xC1
};
#define RS41_WHITENING_LEN 64

// RS41 frame structure
#define RS41_RS_PARITY    48       // 48 bytes of RS parity (24 per codeword)
#define RS41_DATA_START   48       // Data subblocks start after RS parity

// RS41 subblock IDs (from rs1729 RS/rs41mod.c)
#define RS41_BLOCK_STATUS  0x79    // Frame counter, serial, calibration
#define RS41_BLOCK_MEAS    0x7A    // PTU measurements (temperature, humidity)
#define RS41_BLOCK_GPSPOS  0x7B    // GPS ECEF position + velocity + numSats
#define RS41_BLOCK_GPSINFO 0x7C    // GPS week, iTOW, satellite list
#define RS41_BLOCK_GPSRAW  0x7D    // GPS pseudorange/doppler (raw)
#define RS41_BLOCK_EMPTY   0x76    // Zero padding

// FM discriminator gain
#define FM_GAIN           0.8f

// Lowpass filter for FM output
#define SONDE_LPF_TAPS    17

// PLL bit clock recovery constants
#define PLL_BW            0.01f    // Loop bandwidth (fraction of baud)
#define PLL_DAMP          0.707f   // Damping factor

// ======================== GF(2^8) Arithmetic ========================
// Primitive polynomial: x^8 + x^4 + x^3 + x^2 + 1 = 0x11D

#define GF_POLY 0x11D

static uint8_t gf_exp[512];  // Double-sized for modular reduction
static uint8_t gf_log[256];
static int gf_initialized = 0;

static void gf_init(void)
{
    if (gf_initialized) return;
    int x = 1;
    for (int i = 0; i < 255; i++) {
        gf_exp[i] = (uint8_t)x;
        gf_log[x] = (uint8_t)i;
        x <<= 1;
        if (x & 0x100) x ^= GF_POLY;
    }
    for (int i = 255; i < 512; i++)
        gf_exp[i] = gf_exp[i - 255];
    gf_log[0] = 0;
    gf_initialized = 1;
}

static inline uint8_t gf_mul(uint8_t a, uint8_t b)
{
    if (a == 0 || b == 0) return 0;
    return gf_exp[gf_log[a] + gf_log[b]];
}

static inline uint8_t gf_div(uint8_t a, uint8_t b)
{
    if (b == 0 || a == 0) return 0;
    return gf_exp[(gf_log[a] + 255 - gf_log[b]) % 255];
}

static inline uint8_t gf_pow(uint8_t a, int n)
{
    if (a == 0) return 0;
    return gf_exp[(gf_log[a] * n) % 255];
}

// ======================== Reed-Solomon RS(255,231) Decoder ========================

// Compute syndromes S[0..2t-1] = R(alpha^i)
// Returns 1 if all syndromes are zero (no errors)
static int rs_syndromes(const uint8_t *data, int len, uint8_t *syn)
{
    int all_zero = 1;
    for (int i = 0; i < RS_NROOTS; i++) {
        uint8_t s = 0;
        for (int j = 0; j < len; j++)
            s = gf_mul(s, gf_exp[i]) ^ data[j];
        syn[i] = s;
        if (s != 0) all_zero = 0;
    }
    return all_zero;
}

// Berlekamp-Massey: find error locator polynomial sigma(x)
static int rs_berlekamp_massey(const uint8_t *syn, uint8_t *sigma)
{
    uint8_t C[RS_NROOTS + 1];
    uint8_t B[RS_NROOTS + 1];
    memset(C, 0, sizeof(C));
    memset(B, 0, sizeof(B));
    C[0] = 1;
    B[0] = 1;
    int L = 0, m = 1;
    uint8_t b = 1;

    for (int n = 0; n < RS_NROOTS; n++) {
        uint8_t d = syn[n];
        for (int i = 1; i <= L; i++)
            d ^= gf_mul(C[i], syn[n - i]);

        if (d == 0) {
            m++;
        } else if (2 * L <= n) {
            uint8_t T[RS_NROOTS + 1];
            memcpy(T, C, sizeof(T));
            uint8_t coef = gf_div(d, b);
            for (int i = m; i <= RS_NROOTS; i++)
                C[i] ^= gf_mul(coef, B[i - m]);
            L = n + 1 - L;
            memcpy(B, T, sizeof(B));
            b = d;
            m = 1;
        } else {
            uint8_t coef = gf_div(d, b);
            for (int i = m; i <= RS_NROOTS; i++)
                C[i] ^= gf_mul(coef, B[i - m]);
            m++;
        }
    }

    memcpy(sigma, C, (RS_NROOTS + 1) * sizeof(uint8_t));
    return L;
}

// Chien search: find error locations
// Search all 255 nonzero GF elements.
// sigma(alpha^m) = 0  =>  array position j = (m + n - 1) % 255
static int rs_chien_search(const uint8_t *sigma, int deg, int n,
                           int *positions, int *roots)
{
    int count = 0;
    for (int m = 0; m < 255; m++) {
        uint8_t val = 1;
        for (int j = 1; j <= deg; j++)
            val ^= gf_mul(sigma[j], gf_exp[(m * j) % 255]);
        if (val == 0) {
            int pos = (m + n - 1) % 255;
            if (pos >= 0 && pos < n) {
                positions[count] = pos;
                roots[count] = m;
                count++;
                if (count >= deg) break;
            }
        }
    }
    return count;
}

// Forney algorithm: compute error magnitudes
// roots[k] = Chien search index m where sigma(alpha^m) = 0
// Xi_inv = alpha^m, X_k = Xi_inv^(-1) = alpha^(255-m)
static void rs_forney(const uint8_t *syn, const uint8_t *sigma, int deg,
                      const int *roots, int num_errors, uint8_t *magnitudes)
{
    // Error evaluator: omega(x) = S(x)*sigma(x) mod x^2t
    uint8_t omega[RS_NROOTS];
    memset(omega, 0, sizeof(omega));
    for (int i = 0; i < RS_NROOTS; i++) {
        uint8_t v = 0;
        for (int j = 0; j <= deg && j <= i; j++)
            v ^= gf_mul(sigma[j], syn[i - j]);
        omega[i] = v;
    }

    for (int k = 0; k < num_errors; k++) {
        uint8_t Xi_inv = gf_exp[roots[k]];  // alpha^m

        // Evaluate omega(Xi_inv)
        uint8_t O = 0;
        uint8_t xi_pow = 1;
        for (int i = 0; i < RS_NROOTS; i++) {
            O ^= gf_mul(omega[i], xi_pow);
            xi_pow = gf_mul(xi_pow, Xi_inv);
        }

        // Evaluate sigma'(Xi_inv) — formal derivative (odd-degree terms only)
        uint8_t Sp = 0;
        xi_pow = 1;
        for (int i = 1; i <= deg; i += 2) {
            Sp ^= gf_mul(sigma[i], xi_pow);
            xi_pow = gf_mul(xi_pow, gf_mul(Xi_inv, Xi_inv));
        }

        if (Sp != 0) {
            uint8_t X_k = gf_exp[(255 - roots[k]) % 255];  // Xi_inv^(-1)
            magnitudes[k] = gf_mul(X_k, gf_div(O, Sp));
        } else {
            magnitudes[k] = 0;
        }
    }
}

// Full RS decode: correct errors in data[0..len-1]
// Returns number of errors corrected, or -1 if uncorrectable
static int rs_decode(uint8_t *data, int len)
{
    uint8_t syn[RS_NROOTS];

    if (rs_syndromes(data, len, syn))
        return 0;  // No errors

    uint8_t sigma[RS_NROOTS + 1];
    int deg = rs_berlekamp_massey(syn, sigma);

    if (deg > RS_T)
        return -1;  // Too many errors

    int positions[RS_T], roots[RS_T];
    int num_errors = rs_chien_search(sigma, deg, len, positions, roots);

    if (num_errors != deg)
        return -1;  // Chien search didn't find all roots

    uint8_t magnitudes[RS_T];
    rs_forney(syn, sigma, deg, roots, num_errors, magnitudes);

    for (int i = 0; i < num_errors; i++) {
        if (positions[i] >= 0 && positions[i] < len)
            data[positions[i]] ^= magnitudes[i];
    }

    return num_errors;
}

// Apply RS to RS41 frame (two interleaved RS codewords)
// Frame layout after sync: [RS_parity(48)][data(272)]
// CW1: parity bytes 0-23  + data at even positions (48,50,52,...)
// CW2: parity bytes 24-47 + data at odd positions (49,51,53,...)
static int rs41_rs_correct(uint8_t *frame, int len)
{
    if (len < 64) return -1;

    int data_len = len - RS41_RS_PARITY;
    int data_per_cw = (data_len + 1) / 2;
    int cw_len = 24 + data_per_cw;

    uint8_t *cw1 = calloc((unsigned)cw_len, 1);
    uint8_t *cw2 = calloc((unsigned)cw_len, 1);
    if (!cw1 || !cw2) { free(cw1); free(cw2); return -1; }

    // CW1: first 24 parity bytes + data at even offsets from byte 48
    memcpy(cw1, frame, 24);
    for (int i = 0; i < data_per_cw; i++) {
        int src = RS41_RS_PARITY + 2 * i;
        if (src < len) cw1[24 + i] = frame[src];
    }

    // CW2: second 24 parity bytes + data at odd offsets from byte 49
    memcpy(cw2, frame + 24, 24);
    for (int i = 0; i < data_per_cw; i++) {
        int src = RS41_RS_PARITY + 2 * i + 1;
        if (src < len) cw2[24 + i] = frame[src];
    }

    int err0 = rs_decode(cw1, cw_len);
    int err1 = rs_decode(cw2, cw_len);

    // Write back corrected data
    if (err0 >= 0) {
        memcpy(frame, cw1, 24);
        for (int i = 0; i < data_per_cw; i++) {
            int dst = RS41_RS_PARITY + 2 * i;
            if (dst < len) frame[dst] = cw1[24 + i];
        }
    }
    if (err1 >= 0) {
        memcpy(frame + 24, cw2, 24);
        for (int i = 0; i < data_per_cw; i++) {
            int dst = RS41_RS_PARITY + 2 * i + 1;
            if (dst < len) frame[dst] = cw2[24 + i];
        }
    }

    free(cw1);
    free(cw2);

    if (err0 < 0 || err1 < 0) return -1;
    return err0 + err1;
}

// ======================== CRC-16 CCITT ========================
// Polynomial: 0x1021, init: 0xFFFF, no reflection
// Used to verify each RS41 sub-block

static uint16_t crc16_table[256];
static int crc16_init_done = 0;

static void crc16_init(void)
{
    if (crc16_init_done) return;
    for (int i = 0; i < 256; i++) {
        uint16_t crc = (uint16_t)(i << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc = (uint16_t)(crc << 1);
        }
        crc16_table[i] = crc;
    }
    crc16_init_done = 1;
}

static uint16_t crc16_ccitt(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++)
        crc = (uint16_t)((crc << 8) ^ crc16_table[((crc >> 8) ^ data[i]) & 0xFF]);
    return crc;
}

// Verify CRC of an RS41 subblock
// Format: ID(1) + LEN(1) + data(LEN) + CRC(2)
// CRC covers data bytes only (ID and LEN excluded), stored little-endian
static bool rs41_check_block_crc(const uint8_t *block, int block_total_len)
{
    if (block_total_len < 4) return false;
    uint16_t stored_crc = (uint16_t)(block[block_total_len - 2] |
                                     (block[block_total_len - 1] << 8));
    uint16_t calc_crc = crc16_ccitt(block + 2, block_total_len - 4);
    return (stored_crc == calc_crc);
}

// ======================== Whitening ========================
// XOR de-whitening with offset 8 (sync header is 8 bytes before frame data)

static void rs41_dewhiten(uint8_t *frame, int len)
{
    for (int i = 0; i < len; i++)
        frame[i] ^= rs41_whitening[(i + 8) % RS41_WHITENING_LEN];
}

// ======================== Utility ========================

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static int32_t read_i32_le(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

// ECEF to Lat/Lon/Alt (WGS84, Bowring's iterative method, 5 iterations)
static void ecef_to_lla(double x, double y, double z,
                        double *lat_out, double *lon_out, double *alt_out)
{
    const double a = 6378137.0;          // WGS84 semi-major axis
    const double e2 = 0.00669437999014;  // WGS84 first eccentricity squared

    double lon = atan2(y, x);
    double p = sqrt(x * x + y * y);
    double lat = atan2(z, p * (1.0 - e2));

    for (int i = 0; i < 5; i++) {
        double sin_lat = sin(lat);
        double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);
        lat = atan2(z + e2 * N * sin_lat, p);
    }

    double sin_lat = sin(lat);
    double cos_lat = cos(lat);
    double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);
    double alt;
    if (fabs(cos_lat) > 1e-10)
        alt = (p / cos_lat) - N;
    else
        alt = fabs(z) / fabs(sin_lat) - N * (1.0 - e2);

    *lat_out = lat * 180.0 / M_PI;
    *lon_out = lon * 180.0 / M_PI;
    *alt_out = alt;
}

// ======================== State ========================

struct sonde_state {
    sonde_config_t config;

    // FM discriminator state
    int      prev_di;
    int      prev_dq;

    // Decimation
    int      decim_count;
    float    decim_accum_i;
    float    decim_accum_q;

    // FM output lowpass filter
    float    lpf_buf[SONDE_LPF_TAPS];
    int      lpf_idx;
    float    lpf_coeff[SONDE_LPF_TAPS];

    // DC removal
    float    dc_avg;

    // PLL bit clock recovery
    float    bit_clock;
    float    bit_freq;       // Samples per bit (PLL-adjusted)
    float    bit_phase;      // PLL phase accumulator
    float    bit_accum;
    float    prev_sample;
    float    prev_prev_sample;
    int      bit_samples;

    // Syncword detection
    uint32_t shift_reg;

    // Frame assembly
    int      in_frame;
    int      frame_bit_count;
    uint8_t  frame_buf[SONDE_FRAME_LEN + 4];
    int      frame_byte_idx;
    int      frame_bit_idx;

    // Stats
    sonde_stats_t stats;
};

// ======================== Create / Destroy ========================

struct sonde_state *sonde_create(const sonde_config_t *config)
{
    struct sonde_state *s = calloc(1, sizeof(struct sonde_state));
    if (!s) return NULL;

    s->config = *config;

    // Initialize GF(2^8) tables and CRC-16 table
    gf_init();
    crc16_init();

    // Initialize lowpass filter (Blackman window, cutoff ~3 kHz / IF_rate)
    double fc = 3000.0 / SONDE_IF_RATE;
    int M = SONDE_LPF_TAPS - 1;
    double sum = 0;
    for (int i = 0; i < SONDE_LPF_TAPS; i++) {
        double n = i - M / 2.0;
        double h = (fabs(n) < 1e-10) ? (2.0 * fc) : (sin(2.0 * M_PI * fc * n) / (M_PI * n));
        double w = 0.42 - 0.5 * cos(2.0 * M_PI * i / M) + 0.08 * cos(4.0 * M_PI * i / M);
        s->lpf_coeff[i] = (float)(h * w);
        sum += s->lpf_coeff[i];
    }
    for (int i = 0; i < SONDE_LPF_TAPS; i++)
        s->lpf_coeff[i] /= (float)sum;

    // Initialize PLL bit clock
    s->bit_freq = (float)SONDE_SPS;
    s->bit_clock = 0;
    s->bit_phase = 0;
    s->shift_reg = 0;
    s->in_frame = 0;

    fprintf(stderr, "Sonde: RS41 decoder, freq=%.3f MHz, sr=%.0f, decim=%d, IF=%d, sps=%.1f\n",
            config->center_freq / 1e6, config->sample_rate,
            SONDE_DECIM, SONDE_IF_RATE, SONDE_SPS);
    fprintf(stderr, "Sonde: Reed-Solomon RS(255,231) t=%d, CRC-16 CCITT, sync threshold %d/32\n",
            RS_T, SYNC_THRESHOLD);

    return s;
}

void sonde_destroy(struct sonde_state *state)
{
    free(state);
}

void sonde_get_stats(struct sonde_state *state, sonde_stats_t *stats)
{
    *stats = state->stats;
}

// ======================== RS41 Frame Parser ========================

static void sonde_parse_frame(struct sonde_state *state)
{
    uint8_t *frame = state->frame_buf;
    int len = state->frame_byte_idx;

    if (len < 64) return;  // Need RS parity + at least some data

    state->stats.frames_detected++;

    // Step 1: De-whiten (with offset 8 for sync header)
    rs41_dewhiten(frame, len);

    // Step 2: Reed-Solomon error correction (two interleaved codewords)
    int rs_errors = rs41_rs_correct(frame, len);
    if (rs_errors < 0) {
        // Uncorrectable RS errors — discard frame entirely
        state->stats.rs_uncorrectable++;
        return;
    }
    if (rs_errors > 0) {
        state->stats.rs_corrected += (uint64_t)rs_errors;
    }

    // Step 3: Parse subblocks with CRC verification
    // Subblocks start after RS parity (48 bytes) + frame type byte (1)
    // Format: ID(1) + LEN(1) + data(LEN) + CRC(2)
    sonde_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    snprintf(msg.type, sizeof(msg.type), "RS41");
    msg.freq = (float)(state->config.center_freq / 1e6);
    msg.rs_errors = rs_errors;

    int pos = RS41_DATA_START + 1; // Skip RS parity + frame-type byte
    bool got_serial = false;

    while (pos + 3 < len) {
        uint8_t block_id = frame[pos];
        uint8_t block_len = frame[pos + 1];

        // Validate block boundaries
        int block_total = 2 + (int)block_len + 2; // ID + LEN + data + CRC
        if (block_len == 0 || pos + block_total > len)
            break;

        uint8_t *data = &frame[pos + 2];

        // Verify CRC-16 CCITT for this subblock
        if (!rs41_check_block_crc(&frame[pos], block_total)) {
            state->stats.crc_errors++;
            pos += block_total;
            continue;  // Skip this block — CRC mismatch
        }

        switch (block_id) {
        case RS41_BLOCK_STATUS:
            // Serial at data[2..9], frame counter at data[0..1]
            if (block_len >= 10) {
                msg.frame_num = (int)read_u16_le(data);
                int serial_len = 0;
                for (int i = 0; i < 8 && i < SONDE_ID_LEN - 1; i++) {
                    char c = (char)data[2 + i];
                    if (c >= 0x20 && c < 0x7F) {
                        msg.serial[serial_len++] = c;
                    } else {
                        break;
                    }
                }
                msg.serial[serial_len] = '\0';
                got_serial = (serial_len > 0);
            }
            break;

        case RS41_BLOCK_GPSPOS:
            // GPS ECEF position block (0x7B)
            // ECEF X,Y,Z at data[0..11] as int32 (cm)
            // Velocity vX,vY,vZ at data[12..17] as int16 (cm/s)
            // numSats at data[18]
            if (block_len >= 12) {
                int32_t ecef_x = read_i32_le(&data[0]);
                int32_t ecef_y = read_i32_le(&data[4]);
                int32_t ecef_z = read_i32_le(&data[8]);

                double x = ecef_x / 100.0;
                double y = ecef_y / 100.0;
                double z = ecef_z / 100.0;

                // Sanity check: ECEF coordinates must be on/near Earth
                // Earth radius ~6371 km, max altitude ~50 km
                double r = sqrt(x * x + y * y + z * z);
                if (r > 6300000.0 && r < 6500000.0) {
                    ecef_to_lla(x, y, z, &msg.lat, &msg.lon, &msg.alt);
                    // Validate: lat in [-90,90], lon in [-180,180], alt in [-1000,50000]
                    if (fabs(msg.lat) <= 90.0 && fabs(msg.lon) <= 180.0 &&
                        msg.alt > -1000.0 && msg.alt < 50000.0) {
                        msg.valid_pos = true;
                    }
                }

                // Velocity
                if (block_len >= 18 && msg.valid_pos) {
                    int16_t vx = (int16_t)read_u16_le(&data[12]);
                    int16_t vy = (int16_t)read_u16_le(&data[14]);
                    int16_t vz = (int16_t)read_u16_le(&data[16]);
                    double dvx = vx / 100.0;
                    double dvy = vy / 100.0;
                    double dvz = vz / 100.0;
                    msg.vel_h = sqrt(dvx * dvx + dvy * dvy);
                    msg.heading = atan2(dvy, dvx) * 180.0 / M_PI;
                    if (msg.heading < 0) msg.heading += 360.0;
                    msg.vel_v = dvz;
                }

                // Number of satellites
                if (block_len >= 19) {
                    msg.satellites = data[18];
                }
            }
            break;

        case RS41_BLOCK_GPSINFO:
            // GPS week + iTOW at data[0..5] (informational, not parsed to msg)
            break;

        case RS41_BLOCK_MEAS:
            // PTU measurements: temperature ADC readings
            // Not calibrated (would need per-sonde calibration blocks)
            break;

        case RS41_BLOCK_GPSRAW:
        case RS41_BLOCK_EMPTY:
            // Skip
            break;

        default:
            // Unknown block — skip (but CRC was OK)
            break;
        }

        pos += block_total;
    }

    // Only emit message if we got a valid serial AND passed all checks
    if (!got_serial) return;

    state->stats.frames_decoded++;

    if (state->config.callback)
        state->config.callback(&msg, state->config.callback_ctx);
}

// ======================== Bit Processing ========================

static void sonde_process_bit(struct sonde_state *state, int bit)
{
    state->shift_reg = (state->shift_reg << 1) | (unsigned)(bit & 1);

    if (!state->in_frame) {
        // Check for RS41 syncword (4 bytes = 32 bits)
        uint32_t expected = ((uint32_t)RS41_SYNC[0] << 24) |
                            ((uint32_t)RS41_SYNC[1] << 16) |
                            ((uint32_t)RS41_SYNC[2] << 8)  |
                            RS41_SYNC[3];

        uint32_t diff = state->shift_reg ^ expected;
        int errors = 0;
        uint32_t d = diff;
        while (d) { d &= d - 1; errors++; }

        // Also check inverted sync (RS41 can be received with inverted polarity)
        uint32_t diff_inv = state->shift_reg ^ ~expected;
        int errors_inv = 0;
        d = diff_inv;
        while (d) { d &= d - 1; errors_inv++; }

        int match = 32 - errors;
        int match_inv = 32 - errors_inv;

        if (match >= SYNC_THRESHOLD || match_inv >= SYNC_THRESHOLD) {
            state->in_frame = 1;
            state->frame_bit_count = 0;
            state->frame_byte_idx = 0;
            state->frame_bit_idx = 0;
            memset(state->frame_buf, 0, sizeof(state->frame_buf));
        }
    } else {
        // Assemble frame byte (LSB first, per RS41 standard)
        state->frame_buf[state->frame_byte_idx] |=
            (uint8_t)((bit & 1) << state->frame_bit_idx);
        state->frame_bit_idx++;

        if (state->frame_bit_idx >= 8) {
            state->frame_bit_idx = 0;
            state->frame_byte_idx++;
        }

        state->frame_bit_count++;

        if (state->frame_byte_idx >= SONDE_FRAME_LEN) {
            sonde_parse_frame(state);
            state->in_frame = 0;
        }
    }
}

// ======================== IQ Processing ========================

void sonde_process(struct sonde_state *state, const uint8_t *iq_data, unsigned len)
{
    unsigned samples = len / 2;
    state->stats.samples_processed += samples;

    for (unsigned i = 0; i < samples; i++) {
        int ci = (int)iq_data[i * 2]     - 128;
        int cq = (int)iq_data[i * 2 + 1] - 128;

        // Decimate
        state->decim_accum_i += (float)ci;
        state->decim_accum_q += (float)cq;
        state->decim_count++;

        if (state->decim_count >= SONDE_DECIM) {
            float di = state->decim_accum_i / SONDE_DECIM;
            float dq = state->decim_accum_q / SONDE_DECIM;
            state->decim_accum_i = 0;
            state->decim_accum_q = 0;
            state->decim_count = 0;

            // FM discriminator
            float fm = atan2f(dq * (float)state->prev_di - di * (float)state->prev_dq,
                              di * (float)state->prev_di + dq * (float)state->prev_dq);
            fm *= FM_GAIN / (float)M_PI;

            state->prev_di = (int)di;
            state->prev_dq = (int)dq;

            // DC removal (slow IIR)
            state->dc_avg = 0.999f * state->dc_avg + 0.001f * fm;
            fm -= state->dc_avg;

            // Lowpass filter
            state->lpf_buf[state->lpf_idx] = fm;
            state->lpf_idx = (state->lpf_idx + 1) % SONDE_LPF_TAPS;

            float filtered = 0;
            for (int k = 0; k < SONDE_LPF_TAPS; k++)
                filtered += state->lpf_buf[(state->lpf_idx + k) % SONDE_LPF_TAPS]
                            * state->lpf_coeff[k];

            // PLL bit clock recovery with timing error feedback
            state->bit_clock += 1.0f;
            state->bit_samples++;
            state->prev_prev_sample = state->prev_sample;
            state->prev_sample = filtered;
            state->bit_accum += filtered;

            if (state->bit_clock >= state->bit_freq) {
                state->bit_clock -= state->bit_freq;

                // Bit decision
                int bit = (state->bit_accum >= 0) ? 1 : 0;

                // Mueller-Müller timing error
                float on_time = state->bit_accum / (float)(state->bit_samples > 0 ? state->bit_samples : 1);
                float timing_error = state->prev_sample * (on_time >= 0 ? 1.0f : -1.0f)
                                   - on_time * (state->prev_prev_sample >= 0 ? 1.0f : -1.0f);

                // PLL loop filter (proportional + integral)
                float Kp = 2.0f * PLL_DAMP * PLL_BW;
                float Ki = PLL_BW * PLL_BW;
                state->bit_phase += Ki * timing_error;
                state->bit_freq += Kp * timing_error + state->bit_phase;

                // Clamp frequency to ±5% of nominal
                float nominal = (float)SONDE_SPS;
                if (state->bit_freq < nominal * 0.95f) state->bit_freq = nominal * 0.95f;
                if (state->bit_freq > nominal * 1.05f) state->bit_freq = nominal * 1.05f;

                state->bit_accum = 0;
                state->bit_samples = 0;

                sonde_process_bit(state, bit);
            }
        }
    }
}
