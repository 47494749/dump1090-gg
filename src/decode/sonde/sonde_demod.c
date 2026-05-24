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

// Decimation: 2.4 MHz → 150 kHz (factor 16)
// Lower decimation than RS41-only to support M10's higher baud rate
#define SONDE_SAMPLE_RATE 2400000
#define SONDE_DECIM       16
#define SONDE_IF_RATE     (SONDE_SAMPLE_RATE / SONDE_DECIM)  // 150000 Hz
#define SONDE_SPS         ((double)SONDE_IF_RATE / SONDE_BAUD_RATE)  // 31.25

// RS41 syncword: 0x08 0x6B 0xCA 0x11 (after Manchester/bit-inversion)
static const uint8_t RS41_SYNC[] = { 0x08, 0x6B, 0xCA, 0x11 };
#define RS41_SYNC_BITS    32

// Syncword correlation threshold (out of 32 bits)
// Require at least 30/32 matching bits to reduce false positives
#define SYNC_THRESHOLD    30

// ---- DFM (GRAW DFM-06/09/17) ----
// Manchester-2 at 2500 baud → 5000 symbols/s
#define DFM_BAUD_RATE     2500
#define DFM_SYM_RATE      5000
#define DFM_SPS           ((float)SONDE_IF_RATE / DFM_SYM_RATE)  // 30.0
#define DFM_FRAME_BITS    280      // Total decoded bits per frame
#define DFM_SYNC_BITS     30       // Raw Manchester sync length
#define DFM_SYNC_MASK     0x3FFFFFFFU
#define DFM_SYNC_THRESHOLD 26      // Min matching bits (out of 30)
// DFM raw Manchester sync "10011010100110010101101001010101" as shift-reg value
#define DFM_SYNC_WORD     0x26A65695U
// Frame structure offsets (decoded bit positions)
#define DFM_HEAD  0       // 16 bits
#define DFM_CONF  16      // 56 bits (7 × Hamming(8,4))
#define DFM_DAT1  72      // 104 bits (13 × Hamming(8,4))
#define DFM_DAT2  176     // 104 bits (13 × Hamming(8,4))

// ---- M10 (Meteomodem M10/M10+/M20) ----
// Manchester differential at 9615 baud → 19230 symbols/s
#define M10_BAUD_RATE     9615
#define M10_SYM_RATE      19230
#define M10_SPS           ((float)SONDE_IF_RATE / M10_SYM_RATE)  // 7.8
#define M10_FRAME_LEN     101      // 0x64+1 bytes
#define M10_AUX_LEN       20
#define M10_SYNC_BITS     32
#define M10_SYNC_THRESHOLD 30
// M10 raw Manchester sync "10011001100110010100110010011001"
#define M10_SYNC_WORD     0x99994C99U
// M10 type bytes
#define M10_TYPE_M10      0x9F
#define M10_TYPE_M10PLUS  0xAF
#define M10_TYPE_M20      0x20
#define M10_TYPE_M2K2     0x8F
// M10 GPS offsets (Trimble Copernicus II)
#define M10_POS_TOW       0x0A
#define M10_POS_LAT       0x0E
#define M10_POS_LON       0x12
#define M10_POS_ALT       0x16
#define M10_POS_VE        0x04
#define M10_POS_VN        0x06
#define M10_POS_VU        0x08
#define M10_POS_SATS      0x1E
#define M10_POS_UTC       0x1F
#define M10_POS_WEEK      0x20
#define M10_POS_SN        0x5D
#define M10_POS_CNT       0x62
#define M10_POS_CHECK     0x63
// M10+ GPS offsets (Gtop)
#define M10P_POS_LAT      0x04
#define M10P_POS_LON      0x08
#define M10P_POS_ALT      0x0C
#define M10P_POS_VE       0x0F
#define M10P_POS_VN       0x11
#define M10P_POS_VU       0x13
#define M10P_POS_TIME     0x15
#define M10P_POS_DATE     0x18

// ---- Per-frequency channelizer ----
// Each protocol can be on a different frequency within the SDR bandwidth.
// NCO shifts each signal to baseband before decimation + FM discriminator.
// Offsets from center_freq in Hz (positive = above center)
#define SONDE_DFM_OFFSET   50000.0   // DFM at center + 50 kHz
#define SONDE_M10_OFFSET   100000.0  // M10 at center + 100 kHz
#define SONDE_NCO_NORM     1024      // Renormalize NCO phasor every N samples

// FM discriminator gain
#define FM_GAIN           0.8f

// Lowpass filter for FM output (real-valued, after FM discriminator)
#define SONDE_LPF_TAPS    17

// IQ lowpass filter for channel isolation (complex, before FM discriminator)
// Rejects cross-channel interference from signals at ±50 kHz offset
#define SONDE_IQ_TAPS     33

// Per-channel signal processing state
struct sonde_channel {
    // NCO (complex oscillator for frequency shifting)
    float nco_i, nco_q;      // current phasor (unit complex number)
    float nco_di, nco_dq;    // rotation per raw sample
    int   nco_count;          // sample counter for normalization

    // Decimation accumulators (shares decim_count with main)
    float decim_accum_i, decim_accum_q;

    // IQ lowpass filter (complex, at IF rate, for channel isolation)
    float iq_buf_i[SONDE_IQ_TAPS];
    float iq_buf_q[SONDE_IQ_TAPS];
    int   iq_idx;
    float iq_coeff[SONDE_IQ_TAPS];

    // FM discriminator state
    float prev_di, prev_dq;

    // Raw (unfiltered) FM discriminator state for IQ filter bypass test
    float prev_di_raw, prev_dq_raw;
    float fm_raw_nofilt;  // FM computed from raw decimated IQ (no IQ filter)

    // DC removal
    float dc_avg;
    float dc_alpha;  // IIR coefficient (e.g. 0.999 slow, 0.99 fast)

    // LPF state (shares coefficients with main)
    float lpf_buf[SONDE_LPF_TAPS];
    int   lpf_idx;

    // Pre-LPF FM output (DC-removed, before lowpass)
    float fm_raw;
};

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

// PLL bit clock recovery constants
#define PLL_BW            0.002f   // Loop bandwidth (fraction of baud)
#define PLL_DAMP          0.707f   // Damping factor
#define PLL_DISABLE       1        // PLL disabled — fixed clock, more stable

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

// Compute syndromes S[i] = sum_j data[j] * alpha^(i*j)  (ascending polynomial)
// Returns 1 if all syndromes are zero (no errors)
static int rs_syndromes(const uint8_t *data, int len, uint8_t *syn)
{
    int all_zero = 1;
    for (int i = 0; i < RS_NROOTS; i++) {
        uint8_t s = 0;
        for (int j = len - 1; j >= 0; j--)
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
// sigma(alpha^m) = 0  =>  error at position j = (255 - m) % 255
static int rs_chien_search(const uint8_t *sigma, int deg, int n,
                           int *positions, int *roots)
{
    int count = 0;
    for (int m = 0; m < 255; m++) {
        uint8_t val = 1;
        for (int j = 1; j <= deg; j++)
            val ^= gf_mul(sigma[j], gf_exp[(m * j) % 255]);
        if (val == 0) {
            int pos = (255 - m) % 255;
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
// RS41 frame layout (our frame_buf, after 8-byte header consumed by sync):
//   [0..47]:   RS parity (48 bytes): CW1=[0..23], CW2=[24..47] (contiguous, NOT interleaved)
//   [48..311]: Data (264 bytes): CW1=even positions (48,50,...,310), CW2=odd (49,51,...,311)
//   [312..319]: Extra bytes beyond valid frame (ignored)
// Codeword layout for RS decoder: [parity(24) | data(132)] = 156 bytes
static int rs41_rs_correct(uint8_t *frame, int len)
{
    if (len < 64) return -1;
    gf_init();

    // RS41 frame is two interleaved RS(255,231) codewords:
    //   CW0 = frame[0,2,4,...,310] (156 even-indexed bytes)
    //   CW1 = frame[1,3,5,...,311] (156 odd-indexed bytes)
    // Each codeword: first 24 bytes = parity, next 132 bytes = data
    int cw_len = 156;
    int frame_rs_len = 312;  // 2 × 156
    if (frame_rs_len > len) frame_rs_len = len & ~1;  // round down to even
    cw_len = frame_rs_len / 2;

    uint8_t *cw0 = calloc((uint32_t)cw_len, 1);
    uint8_t *cw1 = calloc((uint32_t)cw_len, 1);
    if (!cw0 || !cw1) { free(cw0); free(cw1); return -1; }

    // De-interleave: even bytes → CW0, odd bytes → CW1
    for (int i = 0; i < cw_len; i++) {
        cw0[i] = frame[2 * i];
        cw1[i] = frame[2 * i + 1];
    }

    int err0 = rs_decode(cw0, cw_len);
    int err1 = rs_decode(cw1, cw_len);

    // Write back corrected bytes
    if (err0 >= 0) {
        for (int i = 0; i < cw_len; i++)
            frame[2 * i] = cw0[i];
    }
    if (err1 >= 0) {
        for (int i = 0; i < cw_len; i++)
            frame[2 * i + 1] = cw1[i];
    }

    free(cw0);
    free(cw1);

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
// CRC covers ID + LEN + data (everything except the 2-byte CRC itself), stored little-endian
static bool __attribute__((unused)) rs41_check_block_crc(const uint8_t *block, int block_total_len)
{
    if (block_total_len < 4) return false;
    uint16_t stored_crc = (uint16_t)(block[block_total_len - 2] |
                                     (block[block_total_len - 1] << 8));
    uint16_t calc_crc = crc16_ccitt(block, block_total_len - 2);
    return (stored_crc == calc_crc);
}

// ======================== Whitening ========================
// XOR de-whitening with offset 8 (frame data starts at position 8 in full packet)
// Reference: rs1729/rs41mod.c uses mask[byte_count % 64] where byte_count starts at FRAMESTART=8

static void __attribute__((unused)) rs41_dewhiten(uint8_t *frame, int len)
{
    for (int i = 0; i < len; i++)
        frame[i] ^= rs41_whitening[(i + 8) % RS41_WHITENING_LEN];
}

// ======================== Utility ========================

static uint16_t __attribute__((unused)) read_u16_le(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static int32_t __attribute__((unused)) read_i32_le(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

// ECEF to Lat/Lon/Alt (WGS84, Bowring's iterative method, 5 iterations)
static void __attribute__((unused)) ecef_to_lla(double x, double y, double z,
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

// ======================== DFM Hamming(8,4) ========================

static const uint8_t dfm_H[4][8] = {
    {0,1,1,1,1,0,0,0},
    {1,0,1,1,0,1,0,0},
    {1,1,0,1,0,0,1,0},
    {1,1,1,0,0,0,0,1}
};
static const uint8_t dfm_He[8] = {0x7, 0xB, 0xD, 0xE, 0x8, 0x4, 0x2, 0x1};

// Check/correct one Hamming(8,4) codeword (8 bits → 4 data + 4 parity)
// Returns: 0=no error, 1=corrected 1-bit error, -1=uncorrectable
static int dfm_hamming_check(uint8_t code[8])
{
    uint8_t syndrom[4];
    uint32_t synval;
    for (int i = 0; i < 4; i++) {
        syndrom[i] = 0;
        for (int j = 0; j < 8; j++)
            syndrom[i] ^= dfm_H[i][j] & code[j];
    }
    synval = (uint32_t)((syndrom[0]<<3) | (syndrom[1]<<2) | (syndrom[2]<<1) | syndrom[3]);
    if (synval == 0) return 0;
    for (int j = 0; j < 8; j++) {
        if (synval == dfm_He[j]) {
            code[j] ^= 1;
            return 1;
        }
    }
    return -1;
}

// Deinterleave DFM bit block: str[L*8] → block[L*8]
static void dfm_deinterleave(const uint8_t *str, int L, uint8_t *block)
{
    for (int j = 0; j < 8; j++)
        for (int i = 0; i < L; i++)
            block[8*i+j] = str[L*j+i];
}

// Hamming decode L codewords from interleaved data, extract data nibbles
// Returns: 0=OK, >0=corrected bits, -1=uncorrectable
static int dfm_hamming_decode(const uint8_t *raw_bits, int L, uint8_t *data_bits)
{
    uint8_t block[13*8]; // max L=13
    int ret = 0;
    dfm_deinterleave(raw_bits, L, block);
    for (int i = 0; i < L; i++) {
        int ec = dfm_hamming_check(&block[8*i]);
        if (ec > 0) ret |= (1 << i);
        if (ec < 0) return -1;
        for (int j = 0; j < 4; j++)
            data_bits[4*i+j] = block[8*i+j]; // systematic: data in bits 0..3
    }
    return ret;
}

// Convert DFM bit array to integer (big endian, MSB first)
static uint32_t dfm_bits2val(const uint8_t *bits, int len)
{
    uint32_t val = 0;
    for (int j = 0; j < len && j < 32; j++)
        val |= ((uint32_t)(bits[j] & 1)) << (len-1-j);
    return val;
}

// ======================== M10 Checksum ========================

static int m10_update_check(int c, uint8_t b)
{
    int c0, c1, t, t6, t7, s;
    c1 = c & 0xFF;
    b = (uint8_t)((b >> 1) | ((b & 1) << 7));
    b ^= (b >> 2) & 0xFF;
    t6 = (c & 1) ^ ((c>>2) & 1) ^ ((c>>4) & 1);
    t7 = ((c>>1) & 1) ^ ((c>>3) & 1) ^ ((c>>5) & 1);
    t = (c & 0x3F) | (t6 << 6) | (t7 << 7);
    s = (c >> 7) & 0xFF;
    s ^= (s >> 2) & 0xFF;
    c0 = b ^ t ^ s;
    return ((c1 << 8) | c0) & 0xFFFF;
}

static int m10_checksum(const uint8_t *msg, int len)
{
    int cs = 0;
    for (int i = 0; i < len; i++)
        cs = m10_update_check(cs, msg[i]);
    return cs & 0xFFFF;
}

// Convert M10 decoded bits (0/1 uint8_t) to bytes (big endian)
static void m10_bits2bytes(const uint8_t *bits, uint8_t *bytes, int nbytes)
{
    for (int bp = 0; bp < nbytes; bp++) {
        int val = 0;
        for (int i = 0; i < 8; i++) {
            if (bits[bp * 8 + 7 - i]) val |= (1 << i);
        }
        bytes[bp] = (uint8_t)val;
    }
}

// GPS Week+Seconds to calendar date (from rs1729)
static void m10_gps2date(int week, int tow, int *year, int *month, int *day)
{
    long GpsDays = (long)week * 7 + (tow / 86400);
    long Mjd = 44244 + GpsDays;
    long J = Mjd + 2468570;
    long C = 4 * J / 146097;
    J = J - (146097 * C + 3) / 4;
    long Y = 4000 * (J + 1) / 1461001;
    J = J - 1461 * Y / 4 + 31;
    long M = 80 * J / 2447;
    *day = (int)(J - 2447 * M / 80);
    J = M / 11;
    *month = (int)(M + 2 - 12 * J);
    *year = (int)(100 * (C - 49) + Y + J);
}

// ======================== State ========================

struct sonde_state {
    sonde_config_t config;

    // FM discriminator state
    float    prev_di;
    float    prev_dq;

    // Decimation
    int      decim_count;
    float    decim_accum_i;
    float    decim_accum_q;

    // FM output lowpass filter
    float    lpf_buf[SONDE_LPF_TAPS];
    int      lpf_idx;
    float    lpf_coeff[SONDE_LPF_TAPS];

    // DFM dedicated lowpass filter (narrower cutoff for 2500 baud signal)
    float    dfm_lpf_coeff[SONDE_LPF_TAPS];

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
    float    last_bit_mag;   // DEBUG: magnitude of last bit decision

    // Syncword detection
    uint32_t shift_reg;

    // Lookback bit buffer (circular) — stores raw bits for extraction BEFORE sync
    #define LOOKBACK_BITS  4096   // Must be power of 2
    uint8_t  bit_buf[LOOKBACK_BITS / 8];  // packed bits, 512 bytes
    int      bit_buf_pos;         // Current write position (bit index)

    // Frame assembly
    int      in_frame;
    int      invert_bits;       // 1 if inverted-polarity sync detected
    int      frame_bit_count;
    uint8_t  frame_buf[SONDE_FRAME_LEN + 4];
    int      frame_byte_idx;
    int      frame_bit_idx;

    // Stats
    sonde_stats_t stats;

    // === DFM demodulator ===
    struct {
        float    bit_clock;
        float    bit_freq;     // SPS at Manchester symbol rate
        float    bit_accum;
        float    prev_filtered; // for zero-crossing bit clock recovery
        int      bit_samples;
        uint32_t shift_reg;
        int      in_frame;
        int      invert;
        int      sym_count;    // 0=first half, 1=second half (data)
        int      frame_pos;    // current decoded bit position
        uint8_t  frame_bits[DFM_FRAME_BITS + 8];
        // GPS data accumulator (across fr_id 0-8)
        double   lat, lon, alt;
        double   horiV, dir, vertV;
        int      frnr;
        int      year, month, day;
        int      hour, minute;
        float    sek;
        int      nSV;
        int      posmode;
        uint16_t got_mask;     // bitmask of received fr_id (0-8)
        char     serial[16];
        int      serial_valid;
        // Serial number detection
        uint32_t SN;
        uint32_t SN_prev;
        uint32_t SN6;
        uint32_t SN6_prev;
        uint8_t  sonde_typ;
        uint8_t  nul_ch;
        uint8_t  max_ch;
        uint8_t  sn_ch;
        uint16_t chX[2];
        uint8_t  chXbit;
    } dfm;

    // === M10 demodulator ===
    struct {
        float    bit_clock;
        float    bit_freq;     // SPS at Manchester symbol rate
        float    bit_accum;
        float    prev_filtered; // for zero-crossing bit clock recovery
        int      bit_samples;
        float    last_accum;   // accumulator value of last chip decision
        uint32_t shift_reg;
        int      in_frame;
        int      invert;
        int      sym_count;    // 0=first half, 1=second half
        int      prev_sym;     // previous symbol for differential decode
        float    first_chip_accum; // accumulator of first chip for pair decision
        int      frame_pos;    // current decoded bit position
        uint8_t  frame_bits[(M10_FRAME_LEN + M10_AUX_LEN) * 8 + 8];
        uint8_t  frame_bytes[M10_FRAME_LEN + M10_AUX_LEN + 4];
        // Burst gate
        int      burst_active; // IQ-power burst gate flag
        float    iq_pwr_fast;  // fast exponential IQ power estimate
        int      burst_chips;  // chips since burst start
        // AFC (automatic frequency correction)
        int      afc_done;     // 1 = AFC applied
    } m10;

    // Per-frequency channels (DFM and M10 on different frequencies)
    struct sonde_channel dfm_ch;
    struct sonde_channel m10_ch;
};

// ======================== Channel Demod Helper ========================
// Process one decimated block through IQ filter + FM discriminator + DC removal + LPF
// The IQ lowpass filter isolates this channel from signals at other offsets.
// Returns the lowpass-filtered FM output.

static inline float channel_demod(struct sonde_channel *ch, const float *lpf_coeff)
{
    float di_raw = ch->decim_accum_i / SONDE_DECIM;
    float dq_raw = ch->decim_accum_q / SONDE_DECIM;
    ch->decim_accum_i = 0;
    ch->decim_accum_q = 0;

    // Compute raw (unfiltered) FM for IQ filter bypass test
    {
        float fm_nf = atan2f(dq_raw * ch->prev_di_raw - di_raw * ch->prev_dq_raw,
                             di_raw * ch->prev_di_raw + dq_raw * ch->prev_dq_raw);
        fm_nf *= FM_GAIN / (float)M_PI;
        ch->prev_di_raw = di_raw;
        ch->prev_dq_raw = dq_raw;
        ch->fm_raw_nofilt = fm_nf;
    }

    // Complex IQ lowpass filter for channel isolation
    // Rejects signals at ±50 kHz (cross-channel interference)
    ch->iq_buf_i[ch->iq_idx] = di_raw;
    ch->iq_buf_q[ch->iq_idx] = dq_raw;
    ch->iq_idx = (ch->iq_idx + 1) % SONDE_IQ_TAPS;

    float di = 0, dq = 0;
    for (int k = 0; k < SONDE_IQ_TAPS; k++) {
        int idx = (ch->iq_idx + k) % SONDE_IQ_TAPS;
        di += ch->iq_buf_i[idx] * ch->iq_coeff[k];
        dq += ch->iq_buf_q[idx] * ch->iq_coeff[k];
    }

    // FM discriminator (atan2 of cross/dot product)
    float fm = atan2f(dq * ch->prev_di - di * ch->prev_dq,
                      di * ch->prev_di + dq * ch->prev_dq);
    fm *= FM_GAIN / (float)M_PI;
    ch->prev_di = di;
    ch->prev_dq = dq;

    // DC removal (per-channel IIR)
    ch->dc_avg = ch->dc_alpha * ch->dc_avg + (1.0f - ch->dc_alpha) * fm;
    fm -= ch->dc_avg;

    // Store pre-LPF FM output
    ch->fm_raw = fm;

    // Lowpass filter
    ch->lpf_buf[ch->lpf_idx] = fm;
    ch->lpf_idx = (ch->lpf_idx + 1) % SONDE_LPF_TAPS;

    float filtered = 0;
    for (int k = 0; k < SONDE_LPF_TAPS; k++)
        filtered += ch->lpf_buf[(ch->lpf_idx + k) % SONDE_LPF_TAPS] * lpf_coeff[k];

    return filtered;
}

// ======================== Create / Destroy ========================

struct sonde_state *sonde_create(const sonde_config_t *config)
{
    struct sonde_state *s = calloc(1, sizeof(struct sonde_state));
    if (!s) return NULL;

    s->config = *config;

    // Initialize GF(2^8) tables and CRC-16 table
    gf_init();
    crc16_init();

    // Initialize lowpass filter (Blackman window, cutoff ~30 kHz / IF_rate)
    // Must pass M10 Manchester at 19230 sym/s; integrate-and-dump does final filtering
    double fc = 30000.0 / SONDE_IF_RATE;
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

    // Initialize DFM dedicated LPF (Blackman window, cutoff ~8 kHz for 2500 baud)
    // DFM signal bandwidth ~5 kHz; 8 kHz cutoff gives margin for crystal error
    {
        double fc_dfm = 8000.0 / SONDE_IF_RATE;
        double sum_dfm = 0;
        for (int i = 0; i < SONDE_LPF_TAPS; i++) {
            double n = i - M / 2.0;
            double h = (fabs(n) < 1e-10) ? (2.0 * fc_dfm) : (sin(2.0 * M_PI * fc_dfm * n) / (M_PI * n));
            double w = 0.42 - 0.5 * cos(2.0 * M_PI * i / M) + 0.08 * cos(4.0 * M_PI * i / M);
            s->dfm_lpf_coeff[i] = (float)(h * w);
            sum_dfm += s->dfm_lpf_coeff[i];
        }
        for (int i = 0; i < SONDE_LPF_TAPS; i++)
            s->dfm_lpf_coeff[i] /= (float)sum_dfm;
    }

    // Initialize PLL bit clock
    s->bit_freq = (float)SONDE_SPS;
    s->bit_clock = 0;
    s->bit_phase = 0;
    s->shift_reg = 0;
    s->in_frame = 0;

    // Initialize DFM demodulator
    s->dfm.bit_freq = DFM_SPS;
    s->dfm.posmode = 2;

    // Initialize M10 demodulator
    s->m10.bit_freq = M10_SPS;

    // Initialize DFM channel NCO (downconvert +50 kHz to baseband)
    {
        double phase_step = -2.0 * M_PI * SONDE_DFM_OFFSET / SONDE_SAMPLE_RATE;
        s->dfm_ch.nco_i  = 1.0f;
        s->dfm_ch.nco_q  = 0.0f;
        s->dfm_ch.nco_di = (float)cos(phase_step);
        s->dfm_ch.nco_dq = (float)sin(phase_step);
        s->dfm_ch.dc_alpha = 0.99f;  // DC removal (τ=100 samples, f3dB=239Hz)
    }

    // Initialize M10 channel NCO (downconvert -50 kHz to baseband)
    {
        double phase_step = -2.0 * M_PI * SONDE_M10_OFFSET / SONDE_SAMPLE_RATE;
        s->m10_ch.nco_i  = 1.0f;
        s->m10_ch.nco_q  = 0.0f;
        s->m10_ch.nco_di = (float)cos(phase_step);
        s->m10_ch.nco_dq = (float)sin(phase_step);
        s->m10_ch.dc_alpha = 0.95f;  // DC removal (τ=20 samples, f3dB=1194Hz)
    }

    // Initialize DFM channel IQ lowpass filter (15 kHz cutoff for 5000 sym/s Manchester)
    // Signal bandwidth ~5 kHz, plus RTL-SDR crystal error margin
    {
        double fc = 15000.0 / SONDE_IF_RATE;
        int M = SONDE_IQ_TAPS - 1;
        double fsum = 0;
        for (int i = 0; i < SONDE_IQ_TAPS; i++) {
            double n = i - M / 2.0;
            double h = (fabs(n) < 1e-10) ? (2.0 * fc) : (sin(2.0 * M_PI * fc * n) / (M_PI * n));
            double w = 0.42 - 0.5 * cos(2.0 * M_PI * i / M) + 0.08 * cos(4.0 * M_PI * i / M);
            s->dfm_ch.iq_coeff[i] = (float)(h * w);
            fsum += s->dfm_ch.iq_coeff[i];
        }
        for (int i = 0; i < SONDE_IQ_TAPS; i++)
            s->dfm_ch.iq_coeff[i] /= (float)fsum;
    }

    // Initialize M10 channel IQ lowpass filter (45 kHz cutoff for 19230 sym/s Manchester)
    // Signal bandwidth ~15 kHz, plus up to 30 kHz RTL-SDR crystal error (~75 ppm)
    {
        double fc = 45000.0 / SONDE_IF_RATE;
        int M = SONDE_IQ_TAPS - 1;
        double fsum = 0;
        for (int i = 0; i < SONDE_IQ_TAPS; i++) {
            double n = i - M / 2.0;
            double h = (fabs(n) < 1e-10) ? (2.0 * fc) : (sin(2.0 * M_PI * fc * n) / (M_PI * n));
            double w = 0.42 - 0.5 * cos(2.0 * M_PI * i / M) + 0.08 * cos(4.0 * M_PI * i / M);
            s->m10_ch.iq_coeff[i] = (float)(h * w);
            fsum += s->m10_ch.iq_coeff[i];
        }
        for (int i = 0; i < SONDE_IQ_TAPS; i++)
            s->m10_ch.iq_coeff[i] /= (float)fsum;
    }

    fprintf(stderr, "Sonde: multi-protocol channelizer (RS41/DFM/M10)\n");
    fprintf(stderr, "  freq=%.3f MHz, sr=%.0f, decim=%d, IF=%d Hz, IQ taps=%d\n",
            config->center_freq / 1e6, config->sample_rate,
            SONDE_DECIM, SONDE_IF_RATE, SONDE_IQ_TAPS);
    fprintf(stderr, "  RS41: %d baud, sps=%.1f, offset=0 Hz\n",
            SONDE_BAUD_RATE, (float)SONDE_SPS);
    fprintf(stderr, "  DFM:  %d baud, sps=%.1f, offset=%+.0f Hz, IQ fc=7kHz\n",
            DFM_BAUD_RATE, DFM_SPS, SONDE_DFM_OFFSET);
    fprintf(stderr, "  M10:  %d baud, sps=%.1f, offset=%+.0f Hz, IQ fc=25kHz\n",
            M10_BAUD_RATE, M10_SPS, SONDE_M10_OFFSET);

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

    if (len < 64) return;

    state->stats.frames_detected++;

    // Bit-reverse all bytes: our capture assembles LSB-first but RS41 protocol is MSB-first
    // (rs41mod assembles MSB-first natively, so it doesn't need this step)
    static const uint8_t bitrev[256] = {
        0x00,0x80,0x40,0xC0,0x20,0xA0,0x60,0xE0,0x10,0x90,0x50,0xD0,0x30,0xB0,0x70,0xF0,
        0x08,0x88,0x48,0xC8,0x28,0xA8,0x68,0xE8,0x18,0x98,0x58,0xD8,0x38,0xB8,0x78,0xF8,
        0x04,0x84,0x44,0xC4,0x24,0xA4,0x64,0xE4,0x14,0x94,0x54,0xD4,0x34,0xB4,0x74,0xF4,
        0x0C,0x8C,0x4C,0xCC,0x2C,0xAC,0x6C,0xEC,0x1C,0x9C,0x5C,0xDC,0x3C,0xBC,0x7C,0xFC,
        0x02,0x82,0x42,0xC2,0x22,0xA2,0x62,0xE2,0x12,0x92,0x52,0xD2,0x32,0xB2,0x72,0xF2,
        0x0A,0x8A,0x4A,0xCA,0x2A,0xAA,0x6A,0xEA,0x1A,0x9A,0x5A,0xDA,0x3A,0xBA,0x7A,0xFA,
        0x06,0x86,0x46,0xC6,0x26,0xA6,0x66,0xE6,0x16,0x96,0x56,0xD6,0x36,0xB6,0x76,0xF6,
        0x0E,0x8E,0x4E,0xCE,0x2E,0xAE,0x6E,0xEE,0x1E,0x9E,0x5E,0xDE,0x3E,0xBE,0x7E,0xFE,
        0x01,0x81,0x41,0xC1,0x21,0xA1,0x61,0xE1,0x11,0x91,0x51,0xD1,0x31,0xB1,0x71,0xF1,
        0x09,0x89,0x49,0xC9,0x29,0xA9,0x69,0xE9,0x19,0x99,0x59,0xD9,0x39,0xB9,0x79,0xF9,
        0x05,0x85,0x45,0xC5,0x25,0xA5,0x65,0xE5,0x15,0x95,0x55,0xD5,0x35,0xB5,0x75,0xF5,
        0x0D,0x8D,0x4D,0xCD,0x2D,0xAD,0x6D,0xED,0x1D,0x9D,0x5D,0xDD,0x3D,0xBD,0x7D,0xFD,
        0x03,0x83,0x43,0xC3,0x23,0xA3,0x63,0xE3,0x13,0x93,0x53,0xD3,0x33,0xB3,0x73,0xF3,
        0x0B,0x8B,0x4B,0xCB,0x2B,0xAB,0x6B,0xEB,0x1B,0x9B,0x5B,0xDB,0x3B,0xBB,0x7B,0xFB,
        0x07,0x87,0x47,0xC7,0x27,0xA7,0x67,0xE7,0x17,0x97,0x57,0xD7,0x37,0xB7,0x77,0xF7,
        0x0F,0x8F,0x4F,0xCF,0x2F,0xAF,0x6F,0xEF,0x1F,0x9F,0x5F,0xDF,0x3F,0xBF,0x7F,0xFF
    };
    for (int i = 0; i < len; i++)
        frame[i] = bitrev[frame[i]];

    // Step 1: RS error correction on raw (whitened) frame — RS parity was computed
    // on whitened data, so RS must operate BEFORE dewhitening
    crc16_init();
    int rs_errors = rs41_rs_correct(frame, len);

    // Step 2: Dewhiten the corrected frame
    uint8_t dw[SONDE_FRAME_LEN];
    for (int i = 0; i < len; i++)
        dw[i] = frame[i] ^ rs41_whitening[(i + 8) % RS41_WHITENING_LEN];

    fprintf(stderr, "Sonde: frame #%lu len=%d rs=%d\n",
            (unsigned long)state->stats.frames_detected, len, rs_errors);

    if (rs_errors < 0) {
        state->stats.rs_uncorrectable++;
    } else if (rs_errors > 0) {
        state->stats.rs_corrected += (uint64_t)rs_errors;
    }

    // Step 3: Parse subblocks (starting at pos 49 = after RS parity + frame type byte)
    // Format: ID(1) + LEN(1) + data(LEN) + CRC(2)
    // CRC is DATA-only: crc16_ccitt(data, LEN)
    sonde_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    snprintf(msg.type, sizeof(msg.type), "RS41");
    msg.freq = (float)(state->config.center_freq / 1e6);
    msg.rs_errors = rs_errors;

    int pos = 49;  // subblocks start at byte 49 (after 48 RS parity + 1 frame type)
    bool got_serial = false;
    int crc_ok_count = 0;

    while (pos + 4 <= len) {
        uint8_t blk_id = dw[pos];
        uint8_t blk_len = dw[pos + 1];
        if (blk_len < 1 || blk_len > 200) break;
        int blk_total = 2 + (int)blk_len + 2;
        if (pos + blk_total > len) break;

        // CRC-16 CCITT over data only (not ID+LEN)
        uint16_t stored = (uint16_t)(dw[pos + blk_total - 2] | (dw[pos + blk_total - 1] << 8));
        uint16_t calc = crc16_ccitt(&dw[pos + 2], blk_len);
        if (calc != stored) {
            // CRC failed — try next position (might be bit errors)
            pos++;
            continue;
        }
        crc_ok_count++;
        uint8_t *data = &dw[pos + 2];

        switch (blk_id) {
        case RS41_BLOCK_STATUS:  // 0x79
            if (blk_len >= 10) {
                msg.frame_num = (int)read_u16_le(data);
                int serial_len = 0;
                for (int i = 0; i < 8 && i < SONDE_ID_LEN - 1; i++) {
                    char c = (char)data[2 + i];
                    if (c >= 0x20 && c < 0x7F)
                        msg.serial[serial_len++] = c;
                    else
                        break;
                }
                msg.serial[serial_len] = '\0';
                got_serial = (serial_len > 0);
                fprintf(stderr, "  STATUS: frame=%d serial=%s\n", msg.frame_num, msg.serial);
            }
            break;

        case RS41_BLOCK_GPSPOS:  // 0x7B
            if (blk_len >= 12) {
                int32_t ecef_x = read_i32_le(data);
                int32_t ecef_y = read_i32_le(&data[4]);
                int32_t ecef_z = read_i32_le(&data[8]);
                double x = ecef_x / 100.0;
                double y = ecef_y / 100.0;
                double z = ecef_z / 100.0;
                double r = sqrt(x * x + y * y + z * z);
                if (r > 6300000.0 && r < 6500000.0) {
                    ecef_to_lla(x, y, z, &msg.lat, &msg.lon, &msg.alt);
                    if (fabs(msg.lat) <= 90.0 && fabs(msg.lon) <= 180.0 &&
                        msg.alt > -1000.0 && msg.alt < 50000.0) {
                        msg.valid_pos = true;
                        fprintf(stderr, "  GPSPOS: lat=%.5f lon=%.5f alt=%.1f\n",
                                msg.lat, msg.lon, msg.alt);
                    }
                }
                if (blk_len >= 18 && msg.valid_pos) {
                    int16_t vx = (int16_t)read_u16_le(&data[12]);
                    int16_t vy = (int16_t)read_u16_le(&data[14]);
                    int16_t vz = (int16_t)read_u16_le(&data[16]);
                    msg.vel_h = sqrt((vx/100.0)*(vx/100.0) + (vy/100.0)*(vy/100.0));
                    msg.heading = atan2(vy/100.0, vx/100.0) * 180.0 / M_PI;
                    if (msg.heading < 0) msg.heading += 360.0;
                    msg.vel_v = vz / 100.0;
                }
                if (blk_len >= 19)
                    msg.satellites = data[18];
            }
            break;

        case RS41_BLOCK_MEAS:   // 0x7A
        case RS41_BLOCK_GPSINFO: // 0x7C
        case RS41_BLOCK_GPSRAW: // 0x7D
        case RS41_BLOCK_EMPTY:  // 0x76
        default:
            break;
        }
        pos += blk_total;
    }

    fprintf(stderr, "  CRC_OK blocks: %d\n", crc_ok_count);

    if (got_serial) {
        state->stats.frames_decoded++;
        if (state->config.callback)
            state->config.callback(&msg, state->config.callback_ctx);
    }
}

// ======================== DFM Frame Parser ========================

// Extract serial number info from DFM CONF block
static void dfm_conf_out(struct sonde_state *state, const uint8_t *conf_bits)
{
    uint8_t conf_id = (uint8_t)dfm_bits2val(conf_bits, 4);

    // Track null channel for type detection
    if (conf_id > 4 && dfm_bits2val(conf_bits + 8, 20) == 0)
        state->dfm.nul_ch = (uint8_t)dfm_bits2val(conf_bits, 8);

    if (conf_id > 5 && conf_id > state->dfm.max_ch)
        state->dfm.max_ch = conf_id;

    if (conf_id <= 5) return;
    if (conf_id != (state->dfm.nul_ch >> 4) + 1 &&
        conf_id != state->dfm.max_ch) return;

    uint8_t sn2_ch = (uint8_t)dfm_bits2val(conf_bits, 8);
    uint8_t sn_ch = (sn2_ch >> 4) & 0xF;

    if ((state->dfm.nul_ch & 0x58) == 0x58) {
        // DFM-06 style: 24-bit hex serial
        uint32_t SN6 = dfm_bits2val(conf_bits + 4, 24);
        if (SN6 == state->dfm.SN6_prev && SN6 != 0) {
            state->dfm.sonde_typ = sn_ch;
            snprintf(state->dfm.serial, sizeof(state->dfm.serial),
                     "D%1X%06X", (unsigned)(sn_ch & 0xF), (unsigned)SN6);
            state->dfm.serial_valid = 1;
        }
        state->dfm.SN6_prev = SN6;
    }
    else if ((sn2_ch & 0xF) == 0xC || (sn2_ch & 0xF) == 0x0) {
        // DFM-09/17: decimal serial assembled from two halves
        uint32_t val = dfm_bits2val(conf_bits + 8, 20);
        uint8_t hl = val & 0xF;
        if (hl >= 2) return;

        if (state->dfm.sn_ch != sn_ch) {
            state->dfm.sn_ch = sn_ch;
            state->dfm.chXbit = 0;
            state->dfm.chX[0] = 0;
            state->dfm.chX[1] = 0;
        }
        state->dfm.chX[hl] = (uint16_t)((val >> 4) & 0xFFFF);
        state->dfm.chXbit |= (uint8_t)(1 << hl);

        if (state->dfm.chXbit == 3) {
            uint32_t SN = ((uint32_t)state->dfm.chX[0] << 16) | state->dfm.chX[1];
            if (SN == state->dfm.SN_prev || state->dfm.SN_prev == 0) {
                state->dfm.SN = SN;
                state->dfm.sonde_typ = sn_ch;
                snprintf(state->dfm.serial, sizeof(state->dfm.serial),
                         "D%1X%06u", (unsigned)(sn_ch & 0xF), (unsigned)SN);
                state->dfm.serial_valid = 1;
            }
            state->dfm.SN_prev = SN;
            state->dfm.chXbit = 0;
        }
    }
}

// Extract GPS data from DFM DAT block (called for each fr_id)
static void dfm_dat_out(struct sonde_state *state, const uint8_t *dat_bits, int fr_id)
{
    if (fr_id < 0 || fr_id > 8) return;

    if (fr_id == 0) {
        int mode = (int)dfm_bits2val(dat_bits + 16, 8);
        if (mode > 1 && mode < 5) state->dfm.posmode = mode;
        state->dfm.frnr = (int)dfm_bits2val(dat_bits + 24, 8);
        state->dfm.got_mask |= (1 << 0);
    }

    if (state->dfm.posmode <= 2) {
        if (fr_id == 1) {
            int msek = (int)dfm_bits2val(dat_bits + 32, 16);
            state->dfm.sek = msek / 1000.0f;
            state->dfm.got_mask |= (1 << 1);
        }
        if (fr_id == 2) {
            int32_t lat = (int32_t)dfm_bits2val(dat_bits, 32);
            state->dfm.lat = lat / 1e7;
            int16_t dvv = (int16_t)(uint16_t)dfm_bits2val(dat_bits + 32, 16);
            state->dfm.horiV = dvv / 1e2;
            state->dfm.got_mask |= (1 << 2);
        }
        if (fr_id == 3) {
            int32_t lon = (int32_t)dfm_bits2val(dat_bits, 32);
            state->dfm.lon = lon / 1e7;
            uint16_t d = (uint16_t)(dfm_bits2val(dat_bits + 32, 16) & 0xFFFF);
            state->dfm.dir = d / 1e2;
            state->dfm.got_mask |= (1 << 3);
        }
        if (fr_id == 4) {
            int32_t alt = (int32_t)dfm_bits2val(dat_bits, 32);
            state->dfm.alt = alt / 1e2;
            int16_t dvv = (int16_t)(uint16_t)dfm_bits2val(dat_bits + 32, 16);
            state->dfm.vertV = dvv / 1e2;
            state->dfm.got_mask |= (1 << 4);
        }
    } else {
        // posmode 3 or 4
        if (fr_id == 0) {
            int msek = (int)dfm_bits2val(dat_bits, 16);
            state->dfm.sek = msek / 1000.0f;
            int16_t dvv = (int16_t)(uint16_t)dfm_bits2val(dat_bits + 32, 16);
            state->dfm.horiV = dvv / 1e2;
        }
        if (fr_id == 1) {
            int32_t lat = (int32_t)dfm_bits2val(dat_bits, 32);
            state->dfm.lat = lat / 1e7;
            uint16_t d = (uint16_t)(dfm_bits2val(dat_bits + 32, 16) & 0xFFFF);
            state->dfm.dir = d / 1e2;
            state->dfm.got_mask |= (1 << 2);
        }
        if (fr_id == 2) {
            int32_t lon = (int32_t)dfm_bits2val(dat_bits, 32);
            state->dfm.lon = lon / 1e7;
            int16_t dvv = (int16_t)(uint16_t)dfm_bits2val(dat_bits + 32, 16);
            state->dfm.vertV = dvv / 1e2;
            state->dfm.got_mask |= (1 << 3);
        }
        if (fr_id == 3) {
            int32_t alt = (int32_t)dfm_bits2val(dat_bits, 32);
            state->dfm.alt = alt / 1e2;
            state->dfm.got_mask |= (1 << 4);
        }
    }

    if (fr_id == 8) {
        state->dfm.year   = (int)dfm_bits2val(dat_bits, 12);
        state->dfm.month  = (int)dfm_bits2val(dat_bits + 12, 4);
        state->dfm.day    = (int)dfm_bits2val(dat_bits + 16, 5);
        state->dfm.hour   = (int)dfm_bits2val(dat_bits + 21, 5);
        state->dfm.minute = (int)dfm_bits2val(dat_bits + 26, 6);
        state->dfm.nSV    = (int)dfm_bits2val(dat_bits + 32, 8);
        state->dfm.got_mask |= (1 << 8);
    }
}

// Report DFM position when complete set of fr_id received
static void dfm_report(struct sonde_state *state)
{
    // Need lat(2), lon(3), alt(4), date(8) at minimum
    uint16_t need = (1<<2) | (1<<3) | (1<<4) | (1<<8);
    if ((state->dfm.got_mask & need) != need) return;

    if (fabs(state->dfm.lat) < 0.001 && fabs(state->dfm.lon) < 0.001) return;
    if (fabs(state->dfm.lat) > 90.0 || fabs(state->dfm.lon) > 180.0) return;
    if (state->dfm.alt < -1000.0 || state->dfm.alt > 50000.0) return;

    sonde_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    snprintf(msg.type, sizeof(msg.type), "DFM");
    msg.lat = state->dfm.lat;
    msg.lon = state->dfm.lon;
    msg.alt = state->dfm.alt;
    msg.vel_h = state->dfm.horiV;
    msg.heading = state->dfm.dir;
    msg.vel_v = state->dfm.vertV;
    msg.frame_num = state->dfm.frnr;
    msg.satellites = state->dfm.nSV;
    msg.freq = (float)(state->config.center_freq / 1e6);
    msg.valid_pos = true;
    msg.rs_errors = 0;

    if (state->dfm.serial_valid)
        snprintf(msg.serial, sizeof(msg.serial), "%s", state->dfm.serial);
    else
        snprintf(msg.serial, sizeof(msg.serial), "DFM-????");

    fprintf(stderr, "DFM: [%3d] %04d-%02d-%02d %02d:%02d:%04.1f "
            "lat=%.5f lon=%.5f alt=%.1f vH=%.1f D=%.1f vV=%.1f sats=%d %s\n",
            state->dfm.frnr,
            state->dfm.year, state->dfm.month, state->dfm.day,
            state->dfm.hour, state->dfm.minute, state->dfm.sek,
            state->dfm.lat, state->dfm.lon, state->dfm.alt,
            state->dfm.horiV, state->dfm.dir, state->dfm.vertV,
            state->dfm.nSV, msg.serial);

    state->stats.frames_decoded++;
    if (state->config.callback)
        state->config.callback(&msg, state->config.callback_ctx);

    state->dfm.got_mask = 0;
}

// Parse a complete DFM frame (280 decoded bits)
static void dfm_parse_frame(struct sonde_state *state)
{
    uint8_t *frame = state->dfm.frame_bits;

    // Decode CONF: 56 bits at DFM_CONF → 7 Hamming codewords → 28 data bits
    uint8_t conf_data[7*4];
    int ret_conf = dfm_hamming_decode(&frame[DFM_CONF], 7, conf_data);

    // Decode DAT1: 104 bits at DFM_DAT1 → 13 codewords → 52 data bits
    uint8_t dat1_data[13*4];
    int ret_dat1 = dfm_hamming_decode(&frame[DFM_DAT1], 13, dat1_data);

    // Decode DAT2: 104 bits at DFM_DAT2 → 13 codewords → 52 data bits
    uint8_t dat2_data[13*4];
    int ret_dat2 = dfm_hamming_decode(&frame[DFM_DAT2], 13, dat2_data);

    // Count correctable/uncorrectable blocks per section
    int conf_ok = (ret_conf >= 0) ? 7 : 0;
    int dat1_ok = (ret_dat1 >= 0) ? 13 : 0;
    int dat2_ok = (ret_dat2 >= 0) ? 13 : 0;
    // Show real data bits from CONF area (positions 16-23)
    fprintf(stderr, "DFM: frame hamming conf=%d dat1=%d dat2=%d (ok=%d/%d/%d) "
            "conf_bits=%d%d%d%d%d%d%d%d dat1=%d%d%d%d%d%d%d%d\n",
            ret_conf, ret_dat1, ret_dat2, conf_ok, dat1_ok, dat2_ok,
            frame[DFM_CONF],frame[DFM_CONF+1],frame[DFM_CONF+2],frame[DFM_CONF+3],
            frame[DFM_CONF+4],frame[DFM_CONF+5],frame[DFM_CONF+6],frame[DFM_CONF+7],
            frame[DFM_DAT1],frame[DFM_DAT1+1],frame[DFM_DAT1+2],frame[DFM_DAT1+3],
            frame[DFM_DAT1+4],frame[DFM_DAT1+5],frame[DFM_DAT1+6],frame[DFM_DAT1+7]);

    if (ret_conf == -1 && ret_dat1 == -1 && ret_dat2 == -1)
        return;

    state->stats.frames_detected++;

    if (ret_conf != -1)
        dfm_conf_out(state, conf_data);

    if (ret_dat1 != -1) {
        int fr_id = (int)dfm_bits2val(dat1_data + 48, 4);
        dfm_dat_out(state, dat1_data, fr_id);
        if (fr_id == 8) dfm_report(state);
    }
    if (ret_dat2 != -1) {
        int fr_id = (int)dfm_bits2val(dat2_data + 48, 4);
        dfm_dat_out(state, dat2_data, fr_id);
        if (fr_id == 8) dfm_report(state);
    }
}

// ======================== M10 Frame Parser ========================

static void m10_parse_frame(struct sonde_state *state)
{
    uint8_t *f = state->m10.frame_bytes;
    state->stats.frames_detected++;

    uint8_t type = f[1];
    fprintf(stderr, "M10: frame captured, f[0]=0x%02X type=0x%02X f[2]=0x%02X f[3]=0x%02X f[4]=0x%02X f[5]=0x%02X\n",
            f[0], type, f[2], f[3], f[4], f[5]);
    if (type != M10_TYPE_M10 && type != M10_TYPE_M10PLUS &&
        type != M10_TYPE_M2K2) {
        fprintf(stderr, "M10: unknown type 0x%02X, skipping\n", type);
        return; // M20 not implemented
    }

    // Determine aux length and checksum position
    int flen = f[0];
    int auxlen = 0;
    if (flen != 0x64) {
        auxlen = flen - 0x64;
        if (auxlen < 0 || auxlen > M10_AUX_LEN) auxlen = 0;
    }
    int check_pos = M10_POS_CHECK + auxlen;

    // Verify checksum
    int cs_stored = (f[check_pos] << 8) | f[check_pos + 1];
    int cs_calc = m10_checksum(f, check_pos);
    if (cs_stored != cs_calc) {
        fprintf(stderr, "M10: checksum FAIL stored=0x%04X calc=0x%04X\n", (unsigned)cs_stored, (unsigned)cs_calc);
        return;
    }
    fprintf(stderr, "M10: checksum OK!\n");

    sonde_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.freq = (float)(state->config.center_freq / 1e6);
    msg.rs_errors = 0;

    if (type == M10_TYPE_M10 || type == M10_TYPE_M2K2) {
        snprintf(msg.type, sizeof(msg.type), "M10");

        // Trimble GPS: big-endian 32-bit fields
        double B60B60 = (double)(1<<30) / 90.0; // 2^32/360

        int32_t tow_ms = ((int32_t)f[M10_POS_TOW]<<24) | ((int32_t)f[M10_POS_TOW+1]<<16) |
                         ((int32_t)f[M10_POS_TOW+2]<<8) | f[M10_POS_TOW+3];

        int32_t lat_raw = ((int32_t)f[M10_POS_LAT]<<24) | ((int32_t)f[M10_POS_LAT+1]<<16) |
                          ((int32_t)f[M10_POS_LAT+2]<<8) | f[M10_POS_LAT+3];
        msg.lat = lat_raw / B60B60;

        int32_t lon_raw = ((int32_t)f[M10_POS_LON]<<24) | ((int32_t)f[M10_POS_LON+1]<<16) |
                          ((int32_t)f[M10_POS_LON+2]<<8) | f[M10_POS_LON+3];
        msg.lon = lon_raw / B60B60;

        int32_t alt_raw = ((int32_t)f[M10_POS_ALT]<<24) | ((int32_t)f[M10_POS_ALT+1]<<16) |
                          ((int32_t)f[M10_POS_ALT+2]<<8) | f[M10_POS_ALT+3];
        msg.alt = alt_raw / 1000.0;

        // Velocity (0.005 m/s = 1/200 per count)
        int16_t vE = (int16_t)(((uint16_t)f[M10_POS_VE]<<8) | f[M10_POS_VE+1]);
        int16_t vN = (int16_t)(((uint16_t)f[M10_POS_VN]<<8) | f[M10_POS_VN+1]);
        int16_t vU = (int16_t)(((uint16_t)f[M10_POS_VU]<<8) | f[M10_POS_VU+1]);
        double ve = vE / 200.0;
        double vn = vN / 200.0;
        msg.vel_h = sqrt(ve*ve + vn*vn);
        msg.heading = atan2(ve, vn) * 180.0 / M_PI;
        if (msg.heading < 0) msg.heading += 360.0;
        msg.vel_v = vU / 200.0;

        msg.satellites = f[M10_POS_SATS];

        int gpsweek = (f[M10_POS_WEEK]<<8) | f[M10_POS_WEEK+1];
        if (gpsweek < 1304) gpsweek += 1024;
        int gpssec = tow_ms / 1000;
        int year, month, day;
        m10_gps2date(gpsweek, gpssec, &year, &month, &day);
        (void)year; (void)month; (void)day;
        msg.frame_num = gpssec;

    } else if (type == M10_TYPE_M10PLUS) {
        snprintf(msg.type, sizeof(msg.type), "M10+");

        // Gtop GPS
        int32_t lat_raw = ((int32_t)f[M10P_POS_LAT]<<24) | ((int32_t)f[M10P_POS_LAT+1]<<16) |
                          ((int32_t)f[M10P_POS_LAT+2]<<8) | f[M10P_POS_LAT+3];
        msg.lat = lat_raw / 1e6;

        int32_t lon_raw = ((int32_t)f[M10P_POS_LON]<<24) | ((int32_t)f[M10P_POS_LON+1]<<16) |
                          ((int32_t)f[M10P_POS_LON+2]<<8) | f[M10P_POS_LON+3];
        msg.lon = lon_raw / 1e6;

        int32_t alt_raw = ((int32_t)f[M10P_POS_ALT]<<16) | ((int32_t)f[M10P_POS_ALT+1]<<8) |
                          f[M10P_POS_ALT+2];
        if (alt_raw & 0x800000) alt_raw -= 0x1000000;
        msg.alt = alt_raw / 1e2;

        int16_t vE = (int16_t)(((uint16_t)f[M10P_POS_VE]<<8) | f[M10P_POS_VE+1]);
        int16_t vN = (int16_t)(((uint16_t)f[M10P_POS_VN]<<8) | f[M10P_POS_VN+1]);
        int16_t vU = (int16_t)(((uint16_t)f[M10P_POS_VU]<<8) | f[M10P_POS_VU+1]);
        msg.vel_h = sqrt((vE/1e2)*(vE/1e2) + (vN/1e2)*(vN/1e2));
        msg.heading = atan2(vE/1e2, vN/1e2) * 180.0 / M_PI;
        if (msg.heading < 0) msg.heading += 360.0;
        msg.vel_v = vU / 1e2;

        msg.frame_num = f[M10_POS_CNT];
    }

    // Serial number (5 bytes at SN offset)
    {
        unsigned sn34 = (unsigned)f[M10_POS_SN+3] | ((unsigned)f[M10_POS_SN+4] << 8);
        snprintf(msg.serial, sizeof(msg.serial), "%1X%02u%1X%1u%04u",
                 (unsigned)((f[M10_POS_SN+2]>>4)&0xF), (unsigned)(f[M10_POS_SN+2]&0xFF),
                 (unsigned)(f[M10_POS_SN]&0xF),
                 (sn34>>13)&0x7, sn34&0x1FFF);
    }

    // Sanity check
    if (fabs(msg.lat) > 90.0 || fabs(msg.lon) > 180.0) return;
    if (msg.alt < -1000.0 || msg.alt > 50000.0) return;
    if (fabs(msg.lat) < 0.001 && fabs(msg.lon) < 0.001) return;

    msg.valid_pos = true;

    fprintf(stderr, "M10: lat=%.5f lon=%.5f alt=%.1f vH=%.1f D=%.1f vV=%.1f sats=%d SN=%s [OK]\n",
            msg.lat, msg.lon, msg.alt, msg.vel_h, msg.heading, msg.vel_v,
            msg.satellites, msg.serial);

    state->stats.frames_decoded++;
    if (state->config.callback)
        state->config.callback(&msg, state->config.callback_ctx);
}
// ======================== Bit Processing ========================

static void sonde_process_bit(struct sonde_state *state, int bit)
{
    int raw_bit = bit & 1;

    // Always store raw bit in lookback buffer
    int byte_pos = state->bit_buf_pos / 8;
    int bit_pos = state->bit_buf_pos % 8;
    if (bit_pos == 0)
        state->bit_buf[byte_pos] = 0;
    state->bit_buf[byte_pos] |= (uint8_t)(raw_bit << bit_pos);
    state->bit_buf_pos = (state->bit_buf_pos + 1) & (LOOKBACK_BITS - 1);

    // Shift register for sync detection (always active)
    state->shift_reg = (state->shift_reg << 1) | (uint32_t)raw_bit;

    if (state->in_frame) {
        // Still capturing post-sync bits (for the forward-capture path)
        int b = raw_bit ^ state->invert_bits;
        state->frame_buf[state->frame_byte_idx] |=
            (uint8_t)(b << state->frame_bit_idx);
        state->frame_bit_idx++;
        if (state->frame_bit_idx >= 8) {
            state->frame_bit_idx = 0;
            state->frame_byte_idx++;
        }
        state->frame_bit_count++;
        if (state->frame_byte_idx >= SONDE_FRAME_LEN) {
            // Forward capture complete
            state->in_frame = 0;
            sonde_parse_frame(state);

            // NRZI disabled for debug
        }
        return;
    }

    // Check for RS41 syncword (4 bytes = 32 bits)
    uint32_t expected = ((uint32_t)RS41_SYNC[0] << 24) |
                        ((uint32_t)RS41_SYNC[1] << 16) |
                        ((uint32_t)RS41_SYNC[2] << 8)  |
                        RS41_SYNC[3];

    // Also check correct RS41 sync (from rs1729 reference: 0x10B6CA11 LSB-first → shift reg 0x086D5388)
    uint32_t expected2 = 0x086D5388;

    uint32_t diff = state->shift_reg ^ expected;
    int errors = 0;
    uint32_t d = diff;
    while (d) { d &= d - 1; errors++; }

    uint32_t diff_inv = state->shift_reg ^ ~expected;
    int errors_inv = 0;
    d = diff_inv;
    while (d) { d &= d - 1; errors_inv++; }

    // Check reference sync too
    uint32_t diff2 = state->shift_reg ^ expected2;
    int errors2 = 0;
    d = diff2;
    while (d) { d &= d - 1; errors2++; }
    uint32_t diff2_inv = state->shift_reg ^ ~expected2;
    int errors2_inv = 0;
    d = diff2_inv;
    while (d) { d &= d - 1; errors2_inv++; }
    int match2 = 32 - errors2;
    int match2_inv = 32 - errors2_inv;

    int match = 32 - errors;
    int match_inv = 32 - errors_inv;

    // Report near-matches on the correct sync (threshold 28+ only to reduce noise)
    (void)match2;
    (void)match2_inv;

    if (match >= SYNC_THRESHOLD || match_inv >= SYNC_THRESHOLD) {
        state->invert_bits = (match_inv > match) ? 1 : 0;

        // Set up forward capture
        state->in_frame = 1;
        state->frame_bit_count = 0;
        state->frame_byte_idx = 0;
        state->frame_bit_idx = 0;
        memset(state->frame_buf, 0, sizeof(state->frame_buf));
    }
}

// ======================== DFM Symbol Processing ========================

static void dfm_process_sym(struct sonde_state *state, int sym)
{
    int raw_sym = sym & 1;

    // Update shift register at Manchester symbol rate (5000 sym/s)
    state->dfm.shift_reg = ((state->dfm.shift_reg << 1) | (uint32_t)raw_sym) & DFM_SYNC_MASK;

    if (state->dfm.in_frame) {
        // Manchester-2 decode: data = second symbol of each pair
        state->dfm.sym_count++;
        if (state->dfm.sym_count == 2) {
            state->dfm.sym_count = 0;
            int bit = state->dfm.invert ? (raw_sym ^ 1) : raw_sym;
            state->dfm.frame_bits[state->dfm.frame_pos] = (uint8_t)bit;
            state->dfm.frame_pos++;
            if (state->dfm.frame_pos >= DFM_FRAME_BITS) {
                state->dfm.in_frame = 0;
                dfm_parse_frame(state);
            }
        }
        return;
    }

    // Check for DFM sync (30 bits)
    uint32_t diff = state->dfm.shift_reg ^ DFM_SYNC_WORD;
    int errors = 0;
    uint32_t d = diff;
    while (d) { d &= d - 1; errors++; }

    uint32_t diff_inv = state->dfm.shift_reg ^ (~DFM_SYNC_WORD & DFM_SYNC_MASK);
    int errors_inv = 0;
    d = diff_inv;
    while (d) { d &= d - 1; errors_inv++; }

    int match = DFM_SYNC_BITS - errors;
    int match_inv = DFM_SYNC_BITS - errors_inv;

    if (match >= DFM_SYNC_THRESHOLD || match_inv >= DFM_SYNC_THRESHOLD) {
        state->dfm.invert = (match_inv > match) ? 1 : 0;
        state->dfm.in_frame = 1;
        state->dfm.sym_count = 0;
        state->dfm.frame_pos = 15; // 30 raw sync symbols = 15 decoded bits (header)
        memset(state->dfm.frame_bits, 0, sizeof(state->dfm.frame_bits));
        fprintf(stderr, "DFM: SYNC match=%d%s reg=0x%08X\n",
                match_inv > match ? match_inv : match,
                state->dfm.invert ? " inv" : "",
                state->dfm.shift_reg);

        // DFM independent AFC: correct NCO frequency from DC offset
        // Only apply from HIGH-QUALITY sync to avoid noise false positives
        {
            int best_match = (match_inv > match) ? match_inv : match;
            if (!state->m10.afc_done && best_match >= 28) {
                float dc = state->dfm_ch.dc_avg;
                if (fabsf(dc) > 0.03f) {  // Only apply if significant offset
                    state->m10.afc_done = 1;  // Set global AFC flag
                    float freq_err = dc * SONDE_IF_RATE / (2.0f * FM_GAIN);
                    // Correct DFM NCO
                    float dfm_off = (float)SONDE_DFM_OFFSET + freq_err;
                    float ps = -2.0f * (float)M_PI * dfm_off / SONDE_SAMPLE_RATE;
                    state->dfm_ch.nco_di = cosf(ps);
                    state->dfm_ch.nco_dq = sinf(ps);
                    // Correct M10 NCO (same crystal error)
                    float m10_off = (float)SONDE_M10_OFFSET + freq_err;
                    ps = -2.0f * (float)M_PI * m10_off / SONDE_SAMPLE_RATE;
                    state->m10_ch.nco_di = cosf(ps);
                    state->m10_ch.nco_dq = sinf(ps);
                    fprintf(stderr, "DFM: AFC freq_err=%.0f Hz, DFM_off=%.0f M10_off=%.0f\n",
                            freq_err, dfm_off, m10_off);
                }
            }
        }
    }
}

// ======================== M10 Symbol Processing ========================

static void m10_process_sym(struct sonde_state *state, int sym)
{
    int raw_sym = sym & 1;

    // M10_DECODE diagnostic statics
    static uint8_t m10_dbg_chips[64];
    static float m10_dbg_accum[64];
    static int m10_dbg_idx = 0;
    static int m10_dbg_done = 0;
    // M10_RAW diagnostic: raw (no Manchester) decode
    static uint8_t m10_raw_bits[2048];
    static int m10_raw_pos = 0;
    static int m10_raw_done = 0;

    // Update shift register at Manchester symbol rate (19230 sym/s)
    state->m10.shift_reg = (state->m10.shift_reg << 1) | (uint32_t)raw_sym;

    if (state->m10.in_frame) {
        // M10_DECODE diagnostic: capture raw chips and accum
        if (!m10_dbg_done && m10_dbg_idx < 64) {
            m10_dbg_chips[m10_dbg_idx] = (uint8_t)raw_sym;
            m10_dbg_accum[m10_dbg_idx] = state->m10.last_accum;
            m10_dbg_idx++;
            if (m10_dbg_idx == 64) {
                m10_dbg_done = 1;
                fprintf(stderr, "M10_DECODE: raw 64 chips: ");
                for (int j = 0; j < 64; j++) fprintf(stderr, "%d", m10_dbg_chips[j]);
                fprintf(stderr, "\n");
                fprintf(stderr, "M10_DECODE: accum 64: ");
                for (int j = 0; j < 64; j++)
                    fprintf(stderr, "%.2f ", m10_dbg_accum[j]);
                fprintf(stderr, "\n");
                // Pair decision: (accum_c1 - accum_c2) > 0 → coded=1
                fprintf(stderr, "M10_DECODE: pair_diff: ");
                for (int j = 0; j < 64; j += 2)
                    fprintf(stderr, "%.2f ", m10_dbg_accum[j] - m10_dbg_accum[j+1]);
                fprintf(stderr, "\n");
                // Pair decision → coded bits → differential → raw bits
                {
                    int prev = 0;
                    fprintf(stderr, "M10_DECODE: decoded:   ");
                    for (int j = 0; j < 64; j += 2) {
                        int coded = ((m10_dbg_accum[j] - m10_dbg_accum[j+1]) >= 0) ? 1 : 0;
                        int raw = coded ^ prev;
                        prev = raw;
                        fprintf(stderr, "%d", raw);
                    }
                    fprintf(stderr, "\n");
                }
            }
        }

        state->m10.sym_count++;
        if (state->m10.sym_count == 1) {
            // First chip of Manchester pair: save accumulator for pair decision
            state->m10.first_chip_accum = state->m10.last_accum;
        }
        if (state->m10.sym_count == 2) {
            state->m10.sym_count = 0;
            // M10 differential Manchester decode using PAIR DECISION:
            // TX: coded = prev_raw XOR raw_bit, chips = (coded, coded^1)
            // For coded=1: chips are (+dev, -dev) → accum_c1 > 0, accum_c2 < 0
            // For coded=0: chips are (-dev, +dev) → accum_c1 < 0, accum_c2 > 0
            // Pair decision: (accum_c1 - accum_c2) > 0 → coded=1
            float pair_diff = state->m10.first_chip_accum - state->m10.last_accum;
            if (state->m10.invert) pair_diff = -pair_diff;
            int coded_bit;
            if (fabsf(state->m10.first_chip_accum - state->m10.last_accum) > 1.0f) {
                // Anomalous pair (freq step/transient): first chip is more reliable
                float chip_val = state->m10.invert ? -state->m10.first_chip_accum
                                                   : state->m10.first_chip_accum;
                coded_bit = (chip_val >= 0) ? 1 : 0;
            } else {
                coded_bit = (pair_diff >= 0) ? 1 : 0;
            }
            // Differential decode: raw_bit = coded_bit XOR prev_raw_bit
            int decoded = coded_bit ^ state->m10.prev_sym;
            state->m10.prev_sym = decoded;
            state->m10.frame_bits[state->m10.frame_pos] = (uint8_t)decoded;
            state->m10.frame_pos++;

            int target = M10_FRAME_LEN * 8;
            if (state->m10.frame_pos >= target) {
                state->m10.in_frame = 0;
                m10_bits2bytes(state->m10.frame_bits, state->m10.frame_bytes,
                               M10_FRAME_LEN);
                int cs = m10_checksum(state->m10.frame_bytes, M10_POS_CHECK);
                int cs_stored = (state->m10.frame_bytes[M10_POS_CHECK] << 8) |
                                 state->m10.frame_bytes[M10_POS_CHECK + 1];
                if (cs == cs_stored) {
                    fprintf(stderr, "M10: checksum OK (differential decode)\n");
                    m10_parse_frame(state);
                } else {
                    // Try differential error correction: a coded-bit error at
                    // position K inverts all decoded bits from K onward.
                    // TX frequency settling can cause 1-2 errors in first pairs.
                    int corrected = 0;
                    uint8_t trial_bytes[M10_FRAME_LEN + M10_AUX_LEN + 2];

                    // Single coded-bit error: flip bits K..end
                    for (int K = 1; K < 16 && K < target && !corrected; K++) {
                        for (int j = K; j < target; j++)
                            state->m10.frame_bits[j] ^= 1;
                        m10_bits2bytes(state->m10.frame_bits, trial_bytes,
                                       M10_FRAME_LEN);
                        int tcs = m10_checksum(trial_bytes, M10_POS_CHECK);
                        int tcs_s = (trial_bytes[M10_POS_CHECK] << 8) |
                                     trial_bytes[M10_POS_CHECK + 1];
                        if (tcs == tcs_s) {
                            memcpy(state->m10.frame_bytes, trial_bytes,
                                   M10_FRAME_LEN);
                            fprintf(stderr, "M10: checksum OK (1-error correction "
                                    "at bit %d)\n", K);
                            m10_parse_frame(state);
                            corrected = 1;
                        }
                        // Undo flip
                        for (int j = K; j < target; j++)
                            state->m10.frame_bits[j] ^= 1;
                    }

                    // Double coded-bit error: flip bits K1..K2-1
                    for (int K1 = 1; K1 < 12 && !corrected; K1++) {
                        for (int K2 = K1 + 1; K2 <= 16 && K2 < target && !corrected; K2++) {
                            for (int j = K1; j < K2; j++)
                                state->m10.frame_bits[j] ^= 1;
                            m10_bits2bytes(state->m10.frame_bits, trial_bytes,
                                           M10_FRAME_LEN);
                            int tcs = m10_checksum(trial_bytes, M10_POS_CHECK);
                            int tcs_s = (trial_bytes[M10_POS_CHECK] << 8) |
                                         trial_bytes[M10_POS_CHECK + 1];
                            if (tcs == tcs_s) {
                                memcpy(state->m10.frame_bytes, trial_bytes,
                                       M10_FRAME_LEN);
                                fprintf(stderr, "M10: checksum OK (2-error correction "
                                        "at bits %d,%d)\n", K1, K2);
                                m10_parse_frame(state);
                                corrected = 1;
                            }
                            // Undo flip
                            for (int j = K1; j < K2; j++)
                                state->m10.frame_bits[j] ^= 1;
                        }
                    }

                    if (!corrected) {
                        // Try full inversion (polarity ambiguity)
                        uint8_t alt_bits[(M10_FRAME_LEN + M10_AUX_LEN) * 8 + 8];
                        for (int j = 0; j < target; j++)
                            alt_bits[j] = state->m10.frame_bits[j] ^ 1;
                        m10_bits2bytes(alt_bits, state->m10.frame_bytes,
                                       M10_FRAME_LEN);
                        int cs2 = m10_checksum(state->m10.frame_bytes,
                                              M10_POS_CHECK);
                        int cs2_stored = (state->m10.frame_bytes[M10_POS_CHECK] << 8) |
                                          state->m10.frame_bytes[M10_POS_CHECK + 1];
                        if (cs2 == cs2_stored) {
                            fprintf(stderr, "M10: checksum OK (inverted polarity)\n");
                            m10_parse_frame(state);
                        } else {
                            fprintf(stderr, "M10: checksum FAIL cs=0x%04X/0x%04X "
                                    "f[0]=0x%02X f[1]=0x%02X\n",
                                    (unsigned)cs, (unsigned)cs_stored,
                                    (unsigned)state->m10.frame_bytes[0],
                                    (unsigned)state->m10.frame_bytes[1]);
                        }
                    }
                }
            }
        }

        // Collect every chip for multi-method diagnostic
        {
            int cur_raw = state->m10.invert ? (raw_sym ^ 1) : raw_sym;
            if (!m10_raw_done && m10_raw_pos < 1616) {
                m10_raw_bits[m10_raw_pos++] = (uint8_t)cur_raw;
                if (m10_raw_pos >= 1616) {
                    m10_raw_done = 1;
                    int nbytes = (M10_FRAME_LEN + M10_AUX_LEN);
                    uint8_t tbits[1024], tbytes[128];
                    // Method 1: Raw (no Manchester) - first 808 chips
                    m10_bits2bytes(m10_raw_bits, tbytes, nbytes);
                    fprintf(stderr, "M10_TRY raw:  f[0]=0x%02X type=0x%02X f[2]=0x%02X f[3]=0x%02X f[4]=0x%02X f[5]=0x%02X\n",
                            tbytes[0], tbytes[1], tbytes[2], tbytes[3], tbytes[4], tbytes[5]);
                    // Method 2: Standard Manchester (2nd chip = data)
                    for (int j = 0; j < nbytes * 8; j++) tbits[j] = m10_raw_bits[j*2+1];
                    m10_bits2bytes(tbits, tbytes, nbytes);
                    fprintf(stderr, "M10_TRY std:  f[0]=0x%02X type=0x%02X f[2]=0x%02X f[3]=0x%02X f[4]=0x%02X f[5]=0x%02X\n",
                            tbytes[0], tbytes[1], tbytes[2], tbytes[3], tbytes[4], tbytes[5]);
                    // Method 3: Manchester-2 (2nd chip inverted = data)
                    for (int j = 0; j < nbytes * 8; j++) tbits[j] = m10_raw_bits[j*2+1] ^ 1;
                    m10_bits2bytes(tbits, tbytes, nbytes);
                    fprintf(stderr, "M10_TRY inv:  f[0]=0x%02X type=0x%02X f[2]=0x%02X f[3]=0x%02X f[4]=0x%02X f[5]=0x%02X\n",
                            tbytes[0], tbytes[1], tbytes[2], tbytes[3], tbytes[4], tbytes[5]);
                    // Method 4: Differential (prev != cur → 1)
                    for (int j = 0; j < nbytes * 8; j++)
                        tbits[j] = (m10_raw_bits[j*2] != m10_raw_bits[j*2+1]) ? 1 : 0;
                    m10_bits2bytes(tbits, tbytes, nbytes);
                    fprintf(stderr, "M10_TRY diff: f[0]=0x%02X type=0x%02X f[2]=0x%02X f[3]=0x%02X f[4]=0x%02X f[5]=0x%02X\n",
                            tbytes[0], tbytes[1], tbytes[2], tbytes[3], tbytes[4], tbytes[5]);
                    // Method 5: Differential inverted
                    for (int j = 0; j < nbytes * 8; j++)
                        tbits[j] = (m10_raw_bits[j*2] == m10_raw_bits[j*2+1]) ? 1 : 0;
                    m10_bits2bytes(tbits, tbytes, nbytes);
                    fprintf(stderr, "M10_TRY dinv: f[0]=0x%02X type=0x%02X f[2]=0x%02X f[3]=0x%02X f[4]=0x%02X f[5]=0x%02X\n",
                            tbytes[0], tbytes[1], tbytes[2], tbytes[3], tbytes[4], tbytes[5]);
                    // Method 6: Phase-shifted Standard (sync triggered 1 chip early)
                    for (int j = 0; j < nbytes * 8; j++) tbits[j] = m10_raw_bits[j*2+2];
                    m10_bits2bytes(tbits, tbytes, nbytes);
                    fprintf(stderr, "M10_TRY sft:  f[0]=0x%02X type=0x%02X f[2]=0x%02X f[3]=0x%02X f[4]=0x%02X f[5]=0x%02X\n",
                            tbytes[0], tbytes[1], tbytes[2], tbytes[3], tbytes[4], tbytes[5]);
                    // Method 7: Phase-shifted Inverted
                    for (int j = 0; j < nbytes * 8; j++) tbits[j] = m10_raw_bits[j*2+2] ^ 1;
                    m10_bits2bytes(tbits, tbytes, nbytes);
                    fprintf(stderr, "M10_TRY sfti: f[0]=0x%02X type=0x%02X f[2]=0x%02X f[3]=0x%02X f[4]=0x%02X f[5]=0x%02X\n",
                            tbytes[0], tbytes[1], tbytes[2], tbytes[3], tbytes[4], tbytes[5]);
                    // Method 8: Differential Manchester (2nd chip → XOR accumulate, initial prev=0)
                    {
                        int prev = 0;
                        for (int j = 0; j < M10_FRAME_LEN * 8; j++) {
                            int coded_inv = m10_raw_bits[j*2+1];
                            int decoded = (coded_inv ^ 1) ^ prev;
                            tbits[j] = (uint8_t)decoded;
                            prev = decoded;
                        }
                        m10_bits2bytes(tbits, tbytes, M10_FRAME_LEN);
                        fprintf(stderr, "M10_TRY dman: f[0]=0x%02X type=0x%02X f[2]=0x%02X f[3]=0x%02X f[4]=0x%02X f[5]=0x%02X\n",
                                tbytes[0], tbytes[1], tbytes[2], tbytes[3], tbytes[4], tbytes[5]);
                    }
                }
            }
        }
        return;
    }

    // Check for M10 sync (32 bits)
    uint32_t diff = state->m10.shift_reg ^ M10_SYNC_WORD;
    int errors = 0;
    uint32_t d = diff;
    while (d) { d &= d - 1; errors++; }

    uint32_t diff_inv = state->m10.shift_reg ^ ~M10_SYNC_WORD;
    int errors_inv = 0;
    d = diff_inv;
    while (d) { d &= d - 1; errors_inv++; }

    int match = M10_SYNC_BITS - errors;
    int match_inv = M10_SYNC_BITS - errors_inv;

    if ((match >= M10_SYNC_THRESHOLD || match_inv >= M10_SYNC_THRESHOLD) &&
        state->m10.iq_pwr_fast > 200.0f) {
        state->m10.invert = (match_inv > match) ? 1 : 0;
        state->m10.in_frame = 1;
        state->m10.sym_count = 0;
        state->m10.frame_pos = 0;
        state->m10.prev_sym = 0; // Initial state for differential decode
        state->m10.bit_clock = 0; // Reset clock for clean first-chip alignment
        state->m10.bit_accum = 0; // Clear accumulator for first data chip
        memset(state->m10.frame_bits, 0, sizeof(state->m10.frame_bits));
        m10_dbg_idx = 0;  // Reset diagnostic capture
        m10_dbg_done = 0;
        m10_raw_pos = 0;  // Reset raw diagnostic
        m10_raw_done = 0;
        fprintf(stderr, "M10: SYNC match=%d%s reg=0x%08X bc=%.2f\n",
                match_inv > match ? match_inv : match,
                state->m10.invert ? " inv" : "",
                state->m10.shift_reg,
                state->m10.bit_clock);

        // AFC: measure crystal error from M10 DC offset and correct all NCOs
        if (!state->m10.afc_done) {
            float dc = state->m10_ch.dc_avg;
            if (fabsf(dc) > 0.05f) {  // Only apply if significant offset
                state->m10.afc_done = 1;
                float freq_err = dc * SONDE_IF_RATE / (2.0f * FM_GAIN);
                // Correct M10 NCO
                float m10_off = SONDE_M10_OFFSET + freq_err;
                float ps = -2.0f * (float)M_PI * m10_off / SONDE_SAMPLE_RATE;
                state->m10_ch.nco_di = cosf(ps);
                state->m10_ch.nco_dq = sinf(ps);
                // Correct DFM NCO (same crystal error affects all channels)
                float dfm_off = SONDE_DFM_OFFSET + freq_err;
                ps = -2.0f * (float)M_PI * dfm_off / SONDE_SAMPLE_RATE;
                state->dfm_ch.nco_di = cosf(ps);
                state->dfm_ch.nco_dq = sinf(ps);
                fprintf(stderr, "M10: AFC freq_err=%.0f Hz, M10_off=%.0f DFM_off=%.0f\n",
                        freq_err, m10_off, dfm_off);
            }
        }
    }
}

// ======================== IQ Processing ========================

void sonde_process(struct sonde_state *state, const uint8_t *iq_data, uint32_t len)
{
    uint32_t samples = len / 2;
    state->stats.samples_processed += samples;

    for (uint32_t i = 0; i < samples; i++) {
        float fi = (float)((int)iq_data[i * 2]     - 128);
        float fq = (float)((int)iq_data[i * 2 + 1] - 128);

        // ---- RS41 channel: no NCO (center frequency) ----
        state->decim_accum_i += fi;
        state->decim_accum_q += fq;
        state->decim_count++;

        // ---- DFM channel: NCO downconvert +50 kHz to baseband ----
        {
            struct sonde_channel *ch = &state->dfm_ch;
            float mi = fi * ch->nco_i - fq * ch->nco_q;
            float mq = fq * ch->nco_i + fi * ch->nco_q;
            ch->decim_accum_i += mi;
            ch->decim_accum_q += mq;
            // Advance NCO phasor
            float ni = ch->nco_i * ch->nco_di - ch->nco_q * ch->nco_dq;
            float nq = ch->nco_i * ch->nco_dq + ch->nco_q * ch->nco_di;
            ch->nco_i = ni;
            ch->nco_q = nq;
        }

        // ---- M10 channel: NCO downconvert -50 kHz to baseband ----
        {
            struct sonde_channel *ch = &state->m10_ch;
            float mi = fi * ch->nco_i - fq * ch->nco_q;
            float mq = fq * ch->nco_i + fi * ch->nco_q;
            ch->decim_accum_i += mi;
            ch->decim_accum_q += mq;
            // Advance NCO phasor
            float ni = ch->nco_i * ch->nco_di - ch->nco_q * ch->nco_dq;
            float nq = ch->nco_i * ch->nco_dq + ch->nco_q * ch->nco_di;
            ch->nco_i = ni;
            ch->nco_q = nq;
        }

        // ---- Periodic NCO normalization (prevent amplitude drift) ----
        if ((++state->dfm_ch.nco_count & (SONDE_NCO_NORM - 1)) == 0) {
            float norm;
            norm = 1.0f / sqrtf(state->dfm_ch.nco_i * state->dfm_ch.nco_i +
                                state->dfm_ch.nco_q * state->dfm_ch.nco_q);
            state->dfm_ch.nco_i *= norm;
            state->dfm_ch.nco_q *= norm;

            norm = 1.0f / sqrtf(state->m10_ch.nco_i * state->m10_ch.nco_i +
                                state->m10_ch.nco_q * state->m10_ch.nco_q);
            state->m10_ch.nco_i *= norm;
            state->m10_ch.nco_q *= norm;
        }

        if (state->decim_count >= SONDE_DECIM) {
            // ======== RS41 channel (center freq, same as before) ========
            float di = state->decim_accum_i / SONDE_DECIM;
            float dq = state->decim_accum_q / SONDE_DECIM;
            state->decim_accum_i = 0;
            state->decim_accum_q = 0;
            state->decim_count = 0;

            // FM discriminator
            float fm = atan2f(dq * state->prev_di - di * state->prev_dq,
                              di * state->prev_di + dq * state->prev_dq);
            fm *= FM_GAIN / (float)M_PI;

            state->prev_di = di;
            state->prev_dq = dq;

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
                state->last_bit_mag = state->bit_accum >= 0 ? state->bit_accum : -state->bit_accum;

                // Mueller-Müller timing error
                float on_time = state->bit_accum / (float)(state->bit_samples > 0 ? state->bit_samples : 1);
                float timing_error = state->prev_sample * (on_time >= 0 ? 1.0f : -1.0f)
                                   - on_time * (state->prev_prev_sample >= 0 ? 1.0f : -1.0f);

#if PLL_DISABLE
                // Fixed bit clock — no PLL adjustment
                (void)timing_error;
                (void)on_time;
#else
                // PLL loop filter — only track during frame capture
                if (state->in_frame) {
                    float Kp = 2.0f * PLL_DAMP * PLL_BW;
                    float Ki = PLL_BW * PLL_BW;
                    state->bit_phase += Ki * timing_error;
                    state->bit_freq = (float)SONDE_SPS + Kp * timing_error + state->bit_phase;

                    // Clamp frequency to ±0.5% of nominal
                    float nominal = (float)SONDE_SPS;
                    if (state->bit_freq < nominal * 0.995f) state->bit_freq = nominal * 0.995f;
                    if (state->bit_freq > nominal * 1.005f) state->bit_freq = nominal * 1.005f;
                } else {
                    // Between frames: reset to nominal
                    state->bit_freq = (float)SONDE_SPS;
                    state->bit_phase = 0;
                    (void)timing_error;
                    (void)on_time;
                }
#endif

                state->bit_accum = 0;
                state->bit_samples = 0;

                sonde_process_bit(state, bit);
            }

            // ======== DFM channel (+50 kHz) ========
            float dfm_filtered = channel_demod(&state->dfm_ch, state->dfm_lpf_coeff);

            // Zero-crossing bit clock recovery for DFM
            if (state->dfm.prev_filtered * dfm_filtered < 0) {
                float phase_err = state->dfm.bit_clock;
                if (phase_err > state->dfm.bit_freq * 0.5f)
                    phase_err -= state->dfm.bit_freq;
                state->dfm.bit_clock -= 0.3f * phase_err;
                if (state->dfm.bit_clock < 0)
                    state->dfm.bit_clock += state->dfm.bit_freq;
            }
            state->dfm.prev_filtered = dfm_filtered;

            state->dfm.bit_clock += 1.0f;
            state->dfm.bit_accum += dfm_filtered;
            state->dfm.bit_samples++;

            if (state->dfm.bit_clock >= state->dfm.bit_freq) {
                state->dfm.bit_clock -= state->dfm.bit_freq;
                int sym = (state->dfm.bit_accum >= 0) ? 1 : 0;
                state->dfm.bit_accum = 0;
                state->dfm.bit_samples = 0;
                dfm_process_sym(state, sym);
            }

            // ======== M10 channel (+100 kHz) ========
            float m10_filtered = channel_demod(&state->m10_ch, state->lpf_coeff);
            // Use IQ-filtered + LPF signal for M10 (proper channel isolation)
            float m10_fm = m10_filtered;

            // IQ power tracking (diagnostic only, no burst gating)
            {
                float m10_iq_inst = state->m10_ch.prev_di * state->m10_ch.prev_di +
                                    state->m10_ch.prev_dq * state->m10_ch.prev_dq;
                state->m10.iq_pwr_fast = 0.95f * state->m10.iq_pwr_fast +
                                         0.05f * m10_iq_inst;
            }

            // M10 runs continuously (no burst gate) — like DFM
            {
                // Zero-crossing bit clock recovery
                if (state->m10.prev_filtered * m10_fm < 0) {
                    float phase_err = state->m10.bit_clock;
                    if (phase_err > state->m10.bit_freq * 0.5f)
                        phase_err -= state->m10.bit_freq;
                    // Gain schedule: reduced tracking during data
                    float gain;
                    if (state->m10.in_frame)
                        gain = 0.05f;  // Low gain during Manchester data (track drift)
                    else
                        gain = 0.3f;   // Normal tracking (preamble + sync)
                    state->m10.bit_clock -= gain * phase_err;
                    if (state->m10.bit_clock < 0)
                        state->m10.bit_clock += state->m10.bit_freq;
                }
                state->m10.prev_filtered = m10_fm;

                state->m10.bit_clock += 1.0f;
                state->m10.bit_accum += m10_fm;
                state->m10.bit_samples++;

                // Per-sample FM dump for first 64 IF samples after sync
                {
                    static float fm_samp[64];
                    static int fm_samp_idx = 0;
                    static int fm_samp_active = 0;
                    static int fm_samp_done = 0;
                    if (state->m10.in_frame && !fm_samp_done) {
                        if (!fm_samp_active) {
                            fm_samp_active = 1;
                            fm_samp_idx = 0;
                        }
                        if (fm_samp_idx < 64) {
                            fm_samp[fm_samp_idx++] = m10_fm;
                        }
                        if (fm_samp_idx == 64) {
                            fm_samp_done = 1;
                            fprintf(stderr, "M10_FM64 bc=%.2f: ", state->m10.bit_clock);
                            for (int k = 0; k < 64; k++)
                                fprintf(stderr, "%.4f ", fm_samp[k]);
                            fprintf(stderr, "\n");
                        }
                    }
                    if (!state->m10.in_frame) {
                        fm_samp_active = 0;
                    }
                }

                if (state->m10.bit_clock >= state->m10.bit_freq) {
                    state->m10.bit_clock -= state->m10.bit_freq;
                    int sym = (state->m10.bit_accum >= 0) ? 1 : 0;
                    state->m10.last_accum = state->m10.bit_accum;
                    // Per-chip timing diagnostic: first 16 chips after sync
                    {
                        static float chip_bc[16];
                        static int chip_nsamp[16];
                        static float chip_acc[16];
                        static int chip_diag_idx = 0;
                        static int chip_diag_done = 0;
                        if (state->m10.in_frame && !chip_diag_done) {
                            if (chip_diag_idx < 16) {
                                chip_bc[chip_diag_idx] = state->m10.bit_clock;
                                chip_nsamp[chip_diag_idx] = state->m10.bit_samples;
                                chip_acc[chip_diag_idx] = state->m10.bit_accum;
                                chip_diag_idx++;
                            }
                            if (chip_diag_idx == 16) {
                                chip_diag_done = 1;
                                fprintf(stderr, "M10_CHIPS bc: ");
                                for (int k = 0; k < 16; k++)
                                    fprintf(stderr, "%.2f ", chip_bc[k]);
                                fprintf(stderr, "\nM10_CHIPS ns: ");
                                for (int k = 0; k < 16; k++)
                                    fprintf(stderr, "%d ", chip_nsamp[k]);
                                fprintf(stderr, "\nM10_CHIPS ac: ");
                                for (int k = 0; k < 16; k++)
                                    fprintf(stderr, "%.3f ", chip_acc[k]);
                                fprintf(stderr, "\n");
                            }
                        }
                        if (!state->m10.in_frame) {
                            chip_diag_idx = 0;
                            chip_diag_done = 0;
                        }
                    }
                    state->m10.bit_accum = 0;
                    state->m10.bit_samples = 0;
                    state->m10.burst_chips++;

                    // Burst chip dump: first 256 chips of each burst
                    {
                        static uint8_t bdump[256];
                        static float   baccum[256];
                        int bc = state->m10.burst_chips - 1;
                        if (bc < 256) {
                            bdump[bc] = (uint8_t)sym;
                            baccum[bc] = state->m10.last_accum;
                        }
                        if (bc == 255) {
                            fprintf(stderr, "M10_BURST: first 256 chips (hex bytes MSB-first):\n");
                            for (int i = 0; i < 32; i++) {
                                uint8_t bv = 0;
                                for (int b = 0; b < 8; b++)
                                    bv = (bv << 1) | bdump[i * 8 + b];
                                fprintf(stderr, "%02X ", bv);
                                if (i % 16 == 15) fprintf(stderr, "\n");
                            }
                            fprintf(stderr, "M10_BURST accum[0..31]: ");
                            for (int i = 0; i < 32; i++)
                                fprintf(stderr, "%.2f ", baccum[i]);
                            fprintf(stderr, "(dc=%.4f)\n", state->m10_ch.dc_avg);
                            fprintf(stderr, "M10_BURST accum[32..63]: ");
                            for (int i = 32; i < 64; i++)
                                fprintf(stderr, "%.2f ", baccum[i]);
                            fprintf(stderr, "\n");
                        }
                    }

                    m10_process_sym(state, sym);
                }
            }

            // ======== Channel power diagnostic (every 5 sec) ========
            {
                static float rs41_pwr = 0, dfm_pwr = 0, m10_pwr = 0;
                static float dfm_iq_pwr = 0, m10_iq_pwr = 0;
                static int pwr_cnt = 0;
                rs41_pwr += filtered * filtered;
                dfm_pwr += dfm_filtered * dfm_filtered;
                m10_pwr += m10_filtered * m10_filtered;
                float dip = state->dfm_ch.prev_di, dqp = state->dfm_ch.prev_dq;
                dfm_iq_pwr += dip * dip + dqp * dqp;
                float mip = state->m10_ch.prev_di, mqp = state->m10_ch.prev_dq;
                m10_iq_pwr += mip * mip + mqp * mqp;
                pwr_cnt++;
                if (pwr_cnt >= 5 * SONDE_IF_RATE) {
                    fprintf(stderr, "POWER: RS41 fm=%.4f  DFM fm=%.4f iq=%.4f  M10 fm=%.4f iq=%.4f\n",
                            sqrtf(rs41_pwr / pwr_cnt),
                            sqrtf(dfm_pwr / pwr_cnt), sqrtf(dfm_iq_pwr / pwr_cnt),
                            sqrtf(m10_pwr / pwr_cnt), sqrtf(m10_iq_pwr / pwr_cnt));
                    rs41_pwr = dfm_pwr = m10_pwr = 0;
                    dfm_iq_pwr = m10_iq_pwr = 0;
                    pwr_cnt = 0;
                }
            }
        }
    }
}
