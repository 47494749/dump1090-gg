// Part of dump1090-gg-light
//
// fanet_decode.c: FANET+ LoRa decoder implementation
//
// Implements a LoRa CSS (Chirp Spread Spectrum) demodulator for FANET+
// packets on 868.2 MHz. Uses dechirping via complex multiplication with
// a conjugate base chirp, then FFT peak detection to extract symbols.
//
// LoRa PHY parameters for FANET:
//   SF=7, BW=250kHz, Sync word=0xF1
//   Preamble: 8 upchirps + 2 sync symbols + 2.25 downchirps (SFD)
//   Explicit header: always CR=4/8 for header block
//   Payload CR: as specified in header (4/5 to 4/8)
//
// Reference: https://github.com/3s1d/fanet-stm32/blob/master/Src/fanet/radio/protocol.txt
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <limits.h>
#include <pthread.h>

#include "fanet_decode.h"
#include "msg_queue.h"

// ======================== Internal constants ========================

#define LORA_N          (1 << FANET_SF)          // 128 FFT bins
#define MAX_MSG_QUEUE   16
#define PI              3.14159265358979323846
#define FANET_NAME_CACHE_SIZE  64

// ======================== Name cache entry ========================

typedef struct {
    uint8_t  manufacturer;
    uint16_t id;
    char     name[32];
    uint64_t last_seen;
} fanet_name_entry_t;

// ======================== Decoder state ========================

struct fanet_state {
    uint32_t sample_rate;
    uint32_t samples_per_symbol;

    // Base chirp references
    float   *dechirp_i;
    float   *dechirp_q;
    float   *downchirp_i;
    float   *downchirp_q;

    // Accumulation buffer for one symbol
    float   *sym_buf_i;
    float   *sym_buf_q;
    int32_t      sym_buf_pos;

    // FFT output
    float   *fft_mag;

    // Preamble detection state
    int32_t      preamble_count;
    int32_t      last_peak_bin;
    int32_t      preamble_bin_offset;
    bool     in_packet;
    int32_t      packet_sym_idx;

    // Sync word + SFD tracking
    int32_t      sync_symbols[2];
    int32_t      sfd_count;
    bool     sfd_adjust;        // true when 0.25-symbol SFD offset needs correction
    float    freq_offset;       // estimated CFO in bins (fractional)

    // Packet assembly
    uint8_t  raw_symbols[256];
    int32_t      raw_sym_count;
    float    packet_power;

    // Output queue (thread-safe, C-opaque handle)
    msg_queue_t out_queue;

    // Name cache
    fanet_name_entry_t name_cache[FANET_NAME_CACHE_SIZE];
    int32_t                name_cache_count;
    pthread_mutex_t    name_mutex;

    // Statistics
    fanet_stats_t stats;
};

// ======================== FFT (radix-2 DIT, N=128) ========================

static void fft128(float *re, float *im)
{
    const int32_t N = LORA_N;

    // Bit-reversal permutation
    for (int32_t i = 1, j = 0; i < N; i++) {
        int32_t bit = N >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) {
            float tmp;
            tmp = re[i]; re[i] = re[j]; re[j] = tmp;
            tmp = im[i]; im[i] = im[j]; im[j] = tmp;
        }
    }

    // Cooley-Tukey butterfly
    for (int32_t len = 2; len <= N; len <<= 1) {
        double ang = -2.0 * PI / len;
        float wn_r = (float)cos(ang);
        float wn_i = (float)sin(ang);
        for (int32_t i = 0; i < N; i += len) {
            float w_r = 1.0f, w_i = 0.0f;
            for (int32_t j = 0; j < len / 2; j++) {
                float u_r = re[i + j];
                float u_i = im[i + j];
                float v_r = re[i + j + len/2] * w_r - im[i + j + len/2] * w_i;
                float v_i = re[i + j + len/2] * w_i + im[i + j + len/2] * w_r;
                re[i + j] = u_r + v_r;
                im[i + j] = u_i + v_i;
                re[i + j + len/2] = u_r - v_r;
                im[i + j + len/2] = u_i - v_i;
                float new_w_r = w_r * wn_r - w_i * wn_i;
                float new_w_i = w_r * wn_i + w_i * wn_r;
                w_r = new_w_r;
                w_i = new_w_i;
            }
        }
    }
}

// ======================== Chirp generation ========================

static void generate_dechirp(fanet_state_t *s)
{
    int32_t N = (int32_t)s->samples_per_symbol;
    double bw = (double)FANET_BW;
    double T = (double)N / s->sample_rate;
    double chirp_rate = bw / T;

    for (int32_t n = 0; n < N; n++) {
        double t = (double)n / s->sample_rate;
        double phase = 2.0 * PI * (-bw/2.0 * t + chirp_rate/2.0 * t * t);
        s->dechirp_i[n] = (float)cos(phase);
        s->dechirp_q[n] = (float)sin(phase);
    }

    for (int32_t n = 0; n < N; n++) {
        double t = (double)n / s->sample_rate;
        double phase = 2.0 * PI * (bw/2.0 * t - chirp_rate/2.0 * t * t);
        s->downchirp_i[n] = (float)cos(phase);
        s->downchirp_q[n] = (float)sin(phase);
    }
}

// ======================== Symbol demodulation ========================

static int32_t demodulate_symbol(fanet_state_t *s, const float *buf_i, const float *buf_q,
                             float *peak_power)
{
    int32_t N = (int32_t)s->samples_per_symbol;
    int32_t fft_n = LORA_N;

    float re[LORA_N], im[LORA_N];
    memset(re, 0, sizeof(re));
    memset(im, 0, sizeof(im));

    int32_t fold = N / fft_n;
    if (fold < 1) fold = 1;

    for (int32_t k = 0; k < N; k++) {
        float r = buf_i[k] * s->dechirp_i[k] + buf_q[k] * s->dechirp_q[k];
        float i = buf_q[k] * s->dechirp_i[k] - buf_i[k] * s->dechirp_q[k];
        int32_t bin = (k / fold) % fft_n;
        re[bin] += r;
        im[bin] += i;
    }

    fft128(re, im);

    float max_mag = 0;
    int32_t peak_bin = 0;
    for (int32_t k = 0; k < fft_n; k++) {
        float mag = re[k] * re[k] + im[k] * im[k];
        if (mag > max_mag) {
            max_mag = mag;
            peak_bin = k;
        }
    }

    // Parabolic interpolation on magnitude to resolve +/-1 bin ambiguity
    {
        int32_t left  = (peak_bin - 1 + fft_n) % fft_n;
        int32_t right = (peak_bin + 1) % fft_n;
        float alpha = sqrtf(re[left]*re[left]   + im[left]*im[left]);
        float beta  = sqrtf(max_mag);
        float gamma = sqrtf(re[right]*re[right] + im[right]*im[right]);
        float denom = alpha - 2.0f * beta + gamma;
        if (fabsf(denom) > 1e-6f) {
            float delta = 0.5f * (alpha - gamma) / denom;
            if (delta > 0.5f)
                peak_bin = right;
            else if (delta < -0.5f)
                peak_bin = left;
        }
    }

    if (peak_power) *peak_power = max_mag;
    return peak_bin;
}

static bool is_downchirp(fanet_state_t *s, const float *buf_i, const float *buf_q, int32_t *dc_peak_out)
{
    int32_t N = (int32_t)s->samples_per_symbol;
    int32_t fft_n = LORA_N;
    float re[LORA_N], im[LORA_N];
    memset(re, 0, sizeof(re));
    memset(im, 0, sizeof(im));

    int32_t fold = N / fft_n;
    if (fold < 1) fold = 1;

    for (int32_t k = 0; k < N; k++) {
        float r = buf_i[k] * s->downchirp_i[k] + buf_q[k] * s->downchirp_q[k];
        float i = buf_q[k] * s->downchirp_i[k] - buf_i[k] * s->downchirp_q[k];
        int32_t bin = (k / fold) % fft_n;
        re[bin] += r;
        im[bin] += i;
    }

    fft128(re, im);

    float max_mag = 0;
    int32_t peak_bin = 0;
    for (int32_t k = 0; k < fft_n; k++) {
        float mag = re[k] * re[k] + im[k] * im[k];
        if (mag > max_mag) {
            max_mag = mag;
            peak_bin = k;
        }
    }

    if (dc_peak_out) *dc_peak_out = peak_bin;
    // Empirically, for real downchirps the relationship is:
    //   (dc_peak + preamble_bin_offset) % N ≈ 107
    // This fixed offset arises from the correlation geometry between
    // upchirp demod (demodulate_symbol) and downchirp detection.
    int32_t sum = (peak_bin + s->preamble_bin_offset) % fft_n;
    int32_t diff = abs(sum - 107);
    if (diff > fft_n / 2) diff = fft_n - diff;
    return (diff <= 3);
}

// ======================== LoRa whitening sequence (from gr-lora_sdr) ========================
// This is the empirically determined LoRa whitening sequence.
// The CRC bytes are NOT whitened.

static const uint8_t lora_whitening_seq[255] = {
    0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE1, 0xC2, 0x85, 0x0B, 0x17, 0x2F, 0x5E, 0xBC, 0x78, 0xF1, 0xE3,
    0xC6, 0x8D, 0x1A, 0x34, 0x68, 0xD0, 0xA0, 0x40, 0x80, 0x01, 0x02, 0x04, 0x08, 0x11, 0x23, 0x47,
    0x8E, 0x1C, 0x38, 0x71, 0xE2, 0xC4, 0x89, 0x12, 0x25, 0x4B, 0x97, 0x2E, 0x5C, 0xB8, 0x70, 0xE0,
    0xC0, 0x81, 0x03, 0x06, 0x0C, 0x19, 0x32, 0x64, 0xC9, 0x92, 0x24, 0x49, 0x93, 0x26, 0x4D, 0x9B,
    0x37, 0x6E, 0xDC, 0xB9, 0x72, 0xE4, 0xC8, 0x90, 0x20, 0x41, 0x82, 0x05, 0x0A, 0x15, 0x2B, 0x56,
    0xAD, 0x5B, 0xB6, 0x6D, 0xDA, 0xB5, 0x6B, 0xD6, 0xAC, 0x59, 0xB2, 0x65, 0xCB, 0x96, 0x2C, 0x58,
    0xB0, 0x61, 0xC3, 0x87, 0x0F, 0x1F, 0x3E, 0x7D, 0xFB, 0xF6, 0xED, 0xDB, 0xB7, 0x6F, 0xDE, 0xBD,
    0x7A, 0xF5, 0xEB, 0xD7, 0xAE, 0x5D, 0xBA, 0x74, 0xE8, 0xD1, 0xA2, 0x44, 0x88, 0x10, 0x21, 0x43,
    0x86, 0x0D, 0x1B, 0x36, 0x6C, 0xD8, 0xB1, 0x63, 0xC7, 0x8F, 0x1E, 0x3C, 0x79, 0xF3, 0xE7, 0xCE,
    0x9C, 0x39, 0x73, 0xE6, 0xCC, 0x98, 0x31, 0x62, 0xC5, 0x8B, 0x16, 0x2D, 0x5A, 0xB4, 0x69, 0xD2,
    0xA4, 0x48, 0x91, 0x22, 0x45, 0x8A, 0x14, 0x29, 0x52, 0xA5, 0x4A, 0x95, 0x2A, 0x54, 0xA9, 0x53,
    0xA7, 0x4E, 0x9D, 0x3B, 0x77, 0xEE, 0xDD, 0xBB, 0x76, 0xEC, 0xD9, 0xB3, 0x67, 0xCF, 0x9E, 0x3D,
    0x7B, 0xF7, 0xEF, 0xDF, 0xBF, 0x7E, 0xFD, 0xFA, 0xF4, 0xE9, 0xD3, 0xA6, 0x4C, 0x99, 0x33, 0x66,
    0xCD, 0x9A, 0x35, 0x6A, 0xD4, 0xA8, 0x51, 0xA3, 0x46, 0x8C, 0x18, 0x30, 0x60, 0xC1, 0x83, 0x07,
    0x0E, 0x1D, 0x3A, 0x75, 0xEA, 0xD5, 0xAA, 0x55, 0xAB, 0x57, 0xAF, 0x5F, 0xBE, 0x7C, 0xF9, 0xF2,
    0xE5, 0xCA, 0x94, 0x28, 0x50, 0xA1, 0x42, 0x84, 0x09, 0x13, 0x27, 0x4F, 0x9F, 0x3F, 0x7F
};

static void lora_dewhiten(uint8_t *data, int32_t len)
{
    for (int32_t i = 0; i < len && i < 255; i++) {
        data[i] ^= lora_whitening_seq[i];
    }
}

// ======================== CRC-16 (LoRa) ========================

static uint16_t crc16_lora(const uint8_t *data, int32_t len)
{
    uint16_t crc = 0x0000;
    for (int32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int32_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// ======================== SHA-1 ========================

typedef struct {
    uint32_t state[5];
    uint64_t bit_count;
    uint8_t  buffer[64];
    size_t   buffer_len;
} sha1_ctx_t;

static uint32_t sha1_rotl32(uint32_t v, uint32_t shift)
{
    return (v << shift) | (v >> (32 - shift));
}

static void sha1_transform(uint32_t state[5], const uint8_t block[64])
{
    uint32_t w[80];
    for (int32_t i = 0; i < 16; i++) {
        w[i] = ((uint32_t)block[i * 4] << 24)
             | ((uint32_t)block[i * 4 + 1] << 16)
             | ((uint32_t)block[i * 4 + 2] << 8)
             | (uint32_t)block[i * 4 + 3];
    }
    for (int32_t i = 16; i < 80; i++) {
        w[i] = sha1_rotl32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }

    uint32_t a = state[0];
    uint32_t b = state[1];
    uint32_t c = state[2];
    uint32_t d = state[3];
    uint32_t e = state[4];

    for (int32_t i = 0; i < 80; i++) {
        uint32_t f;
        uint32_t k;
        if (i < 20) {
            f = (b & c) | ((~b) & d);
            k = 0x5A827999u;
        } else if (i < 40) {
            f = b ^ c ^ d;
            k = 0x6ED9EBA1u;
        } else if (i < 60) {
            f = (b & c) | (b & d) | (c & d);
            k = 0x8F1BBCDCu;
        } else {
            f = b ^ c ^ d;
            k = 0xCA62C1D6u;
        }

        uint32_t temp = sha1_rotl32(a, 5) + f + e + k + w[i];
        e = d;
        d = c;
        c = sha1_rotl32(b, 30);
        b = a;
        a = temp;
    }

    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
}

static void sha1_init(sha1_ctx_t *ctx)
{
    ctx->state[0] = 0x67452301u;
    ctx->state[1] = 0xEFCDAB89u;
    ctx->state[2] = 0x98BADCFEu;
    ctx->state[3] = 0x10325476u;
    ctx->state[4] = 0xC3D2E1F0u;
    ctx->bit_count = 0;
    ctx->buffer_len = 0;
}

static void sha1_update(sha1_ctx_t *ctx, const uint8_t *data, size_t len)
{
    ctx->bit_count += (uint64_t)len * 8u;

    while (len > 0) {
        size_t take = 64 - ctx->buffer_len;
        if (take > len)
            take = len;
        memcpy(ctx->buffer + ctx->buffer_len, data, take);
        ctx->buffer_len += take;
        data += take;
        len -= take;

        if (ctx->buffer_len == 64) {
            sha1_transform(ctx->state, ctx->buffer);
            ctx->buffer_len = 0;
        }
    }
}

static void sha1_final(sha1_ctx_t *ctx, uint8_t digest[20])
{
    ctx->buffer[ctx->buffer_len++] = 0x80;
    if (ctx->buffer_len > 56) {
        while (ctx->buffer_len < 64)
            ctx->buffer[ctx->buffer_len++] = 0;
        sha1_transform(ctx->state, ctx->buffer);
        ctx->buffer_len = 0;
    }

    while (ctx->buffer_len < 56)
        ctx->buffer[ctx->buffer_len++] = 0;

    for (int32_t i = 7; i >= 0; i--) {
        ctx->buffer[ctx->buffer_len++] = (uint8_t)(ctx->bit_count >> (i * 8));
    }

    sha1_transform(ctx->state, ctx->buffer);

    for (int32_t i = 0; i < 5; i++) {
        digest[i * 4] = (uint8_t)(ctx->state[i] >> 24);
        digest[i * 4 + 1] = (uint8_t)(ctx->state[i] >> 16);
        digest[i * 4 + 2] = (uint8_t)(ctx->state[i] >> 8);
        digest[i * 4 + 3] = (uint8_t)ctx->state[i];
    }
}

static bool fanet_get_signature_key(const uint8_t **key, int32_t *key_len)
{
    *key = NULL;
    *key_len = 0;
    return false;
}

static bool fanet_verify_signature(const uint8_t *packet, const uint8_t *payload,
                                   int32_t payload_len, const uint8_t *key, int32_t key_len,
                                   uint32_t signature)
{
    uint8_t pseudo[4] = {
        (uint8_t)(packet[0] & 0x3F),
        packet[1],
        packet[2],
        packet[3]
    };
    uint8_t digest[20];
    sha1_ctx_t ctx;

    sha1_init(&ctx);
    sha1_update(&ctx, pseudo, sizeof(pseudo));
    if (payload_len > 0)
        sha1_update(&ctx, payload, (size_t)payload_len);
    sha1_update(&ctx, key, (size_t)key_len);
    sha1_final(&ctx, digest);

    uint32_t expected = (uint32_t)digest[0]
                      | ((uint32_t)digest[1] << 8)
                      | ((uint32_t)digest[2] << 16)
                      | ((uint32_t)digest[3] << 24);
    return expected == signature;
}

// ======================== Gray code (LoRa-specific) ========================
// LoRa gray demapping per gr-lora_sdr fft_demod_impl.cc + gray_mapping_impl.cc:
//   1. Subtract 1 mod N:  s = (bin - 1) mod 2^SF
//   2. Divide for reduced rate (header): s = s / 4
//   3. Gray demap: result = s ^ (s >> 1)  [binary_to_gray formula]

static uint8_t gray_demap(uint8_t bin, int32_t sf, int32_t reduced_rate)
{
    int32_t N = 1 << sf;
    int32_t s = ((int32_t)bin - 1 + N) & (N - 1);  // subtract 1 mod N
    if (reduced_rate) s >>= 2;              // divide by 4 for header
    return (uint8_t)(s ^ (s >> 1));         // gray demap
}

// ======================== LoRa FEC: Deinterleaving ========================
// Reference: gr-lora_sdr deinterleaver_impl.cc
// Matrix is cw_len rows × sf_app columns
// Formula: deinter[mod(i - j - 1, sf_app)][i] = inter[i][j]
//
// For header: sf_app = SF - 2 (reduced rate)
// For payload: sf_app = SF

static void deinterleave(const uint8_t *symbols, int32_t num_symbols,
                         int32_t sf_app, int32_t cw_len,
                         uint8_t *codewords, int32_t *num_codewords)
{
    *num_codewords = 0;
    int32_t blocks = num_symbols / cw_len;

    for (int32_t blk = 0; blk < blocks; blk++) {
        // Build interleaved binary matrix: cw_len rows × sf_app columns
        // Each symbol provides sf_app bits (MSB first) — matches gr-lora_sdr int2bool()
        bool inter_bin[8][12]; // max cw_len=8, max sf_app=12
        memset(inter_bin, 0, sizeof(inter_bin));

        for (int32_t i = 0; i < cw_len; i++) {
            int32_t sym_idx = blk * cw_len + i;
            if (sym_idx >= num_symbols) break;
            uint8_t sym = symbols[sym_idx];
            for (int32_t j = 0; j < sf_app; j++) {
                // MSB first: bit j is bit (sf_app-1-j)
                inter_bin[i][j] = (sym >> (sf_app - 1 - j)) & 1;
            }
        }

        // Deinterleave: deinter[mod(i-j-1, sf_app)][i] = inter[i][j]
        bool deinter_bin[12][8]; // max sf_app=12, max cw_len=8
        memset(deinter_bin, 0, sizeof(deinter_bin));

        for (int32_t i = 0; i < cw_len; i++) {
            for (int32_t j = 0; j < sf_app; j++) {
                int32_t row = ((i - j - 1) % sf_app + sf_app) % sf_app;
                deinter_bin[row][i] = inter_bin[i][j];
            }
        }

        // Convert binary rows to codeword bytes — matches gr-lora_sdr bool2int()
        for (int32_t i = 0; i < sf_app; i++) {
            uint8_t cw = 0;
            for (int32_t j = 0; j < cw_len; j++) {
                cw |= (deinter_bin[i][j] << (cw_len - 1 - j));
            }
            codewords[(*num_codewords)++] = cw;
        }
    }
}

// ======================== LoRa FEC: Hamming decode ========================
// Reference: gr-lora_sdr hamming_dec_impl.cc
// Codeword bits (MSB first): [d3 d2 d1 d0 p0 p1 p2 p3]
// (for CR=4/8, cw_len=8)
//
// Syndrome computation:
//   s0 = cw[0]^cw[1]^cw[2]^cw[4]   (d3^d2^d1^p0)
//   s1 = cw[1]^cw[2]^cw[3]^cw[5]   (d2^d1^d0^p1)
//   s2 = cw[0]^cw[1]^cw[3]^cw[6]   (d3^d2^d0^p2)
//
// Data nibble output (reversed): {cw[3], cw[2], cw[1], cw[0]}

static uint8_t hamming_decode_nibble(uint8_t cw, int32_t cr)
{
    int32_t cw_len = cr + 4;
    // Extract bits (MSB first in cw_len bits)
    bool bits[8];
    for (int32_t i = 0; i < cw_len; i++) {
        bits[i] = (cw >> (cw_len - 1 - i)) & 1;
    }

    switch (cr) {
    case 4: {
        // CR 4/8: full Hamming(7,4) + overall parity
        // Overall parity check
        int32_t parity = 0;
        for (int32_t i = 0; i < 8; i++) parity ^= bits[i];

        // Syndrome
        int32_t s0 = bits[0] ^ bits[1] ^ bits[2] ^ bits[4];
        int32_t s1 = bits[1] ^ bits[2] ^ bits[3] ^ bits[5];
        int32_t s2 = bits[0] ^ bits[1] ^ bits[3] ^ bits[6];
        int32_t syndrome = s0 + (s1 << 1) + (s2 << 2);

        if (parity) { // odd number of errors → correct single bit
            switch (syndrome) {
            case 5: bits[0] ^= 1; break; // d3
            case 7: bits[1] ^= 1; break; // d2
            case 3: bits[2] ^= 1; break; // d1
            case 6: bits[3] ^= 1; break; // d0
            // parity bit errors: 1,2,4 → no data correction needed
            }
        }
        break;
    }
    case 3: {
        // CR 4/7: Hamming(7,4) without extra parity
        int32_t s0 = bits[0] ^ bits[1] ^ bits[2] ^ bits[4];
        int32_t s1 = bits[1] ^ bits[2] ^ bits[3] ^ bits[5];
        int32_t s2 = bits[0] ^ bits[1] ^ bits[3] ^ bits[6];
        int32_t syndrome = s0 + (s1 << 1) + (s2 << 2);

        switch (syndrome) {
        case 5: bits[0] ^= 1; break;
        case 7: bits[1] ^= 1; break;
        case 3: bits[2] ^= 1; break;
        case 6: bits[3] ^= 1; break;
        }
        break;
    }
    case 2: {
        // CR 4/6: parity detection only (2 parity bits), no correction
        // s0 = bits[0]^bits[1]^bits[2]^bits[4]
        // s1 = bits[1]^bits[2]^bits[3]^bits[5]
        // No correction applied
        break;
    }
    case 1: {
        // CR 4/5: single parity bit, detection only
        break;
    }
    }

    // Data nibble: bits[3] bits[2] bits[1] bits[0] (reversed order)
    uint8_t nibble = (bits[3] << 3) | (bits[2] << 2) | (bits[1] << 1) | bits[0];
    return nibble;
}

// ======================== Coordinate decoding ========================

static double fanet_decode_lat(const uint8_t *p)
{
    int32_t raw = (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)(int8_t)p[2] << 16));
    return raw / 93206.0;
}

static double fanet_decode_lon(const uint8_t *p)
{
    int32_t raw = (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)(int8_t)p[2] << 16));
    return raw / 46603.0;
}

// Compressed coordinate decode (2-byte relative, reference-based)
// Reference: FANET protocol.txt
static double fanet_decode_compressed(const uint8_t *p, double ref_coord)
{
    uint16_t raw = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    bool odd = (raw >> 15) & 1;
    int16_t sub_deg_int = (int16_t)((raw & 0x7FFF) | ((raw & 0x4000) << 1)); // sign-extend 15 bits
    double sub_deg = sub_deg_int / 32767.0;

    double ref_rounded = round(ref_coord);
    bool ref_isodd = ((int32_t)ref_rounded) & 1;

    if (ref_isodd != odd) {
        double mysub_deg = ref_coord - ref_rounded;
        if (sub_deg > mysub_deg)
            ref_rounded -= 1.0;
        else
            ref_rounded += 1.0;
    }

    return ref_rounded + sub_deg;
}

// ======================== Scaled value decode helpers ========================

static float decode_scaled_unsigned(uint8_t byte, float resolution)
{
    float val = (byte & 0x7F) * resolution;
    if (byte & 0x80) val *= 5.0f;
    return val;
}

static float decode_scaled_signed(uint8_t byte, float resolution, float scale_factor)
{
    int8_t raw = (int8_t)((byte & 0x7F) | ((byte & 0x40) ? 0x80 : 0));
    float val = raw * resolution;
    if (byte & 0x80) val *= scale_factor;
    return val;
}

// ======================== FANET packet parsing ========================

static bool parse_fanet_packet(const uint8_t *payload, int32_t len, fanet_message_t *msg)
{
    if (len < 4) return false;

    // ---- MAC Header ----
    // Byte 0: bit7=extended header, bit6=forward, bit5-0=type
    uint8_t byte0 = payload[0];
    msg->extended_header = (byte0 >> 7) & 1;
    msg->forward = (byte0 >> 6) & 1;
    msg->type = (fanet_msg_type_t)(byte0 & 0x3F);

    // Bytes 1-3: Source address (manufacturer:1 + id:2 LE)
    msg->src_manufacturer = payload[1];
    msg->src_id = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);

    int32_t hdr_len = 4;

    // Extended header defaults
    msg->unicast = false;
    msg->signature_present = false;
    msg->signature_checked = false;
    msg->signature_valid = false;
    msg->geo_forwarded = false;
    msg->ack_type = 0;
    msg->dst_manufacturer = 0;
    msg->dst_id = 0;
    msg->signature = 0;

    if (msg->extended_header) {
        if (len < 5) return false;
        uint8_t ext = payload[4];
        msg->ack_type = (ext >> 6) & 3;
        msg->unicast = (ext >> 5) & 1;
        msg->signature_present = (ext >> 4) & 1;
        msg->geo_forwarded = (ext >> 3) & 1;
        hdr_len = 5;

        if (msg->unicast) {
            if (len < 8) return false;
            msg->dst_manufacturer = payload[5];
            msg->dst_id = (uint16_t)payload[6] | ((uint16_t)payload[7] << 8);
            hdr_len = 8;
        }

        if (msg->signature_present) {
            if (len < hdr_len + 4) return false;
            msg->signature = (uint32_t)payload[hdr_len]
                           | ((uint32_t)payload[hdr_len + 1] << 8)
                           | ((uint32_t)payload[hdr_len + 2] << 16)
                           | ((uint32_t)payload[hdr_len + 3] << 24);
            hdr_len += 4;
        }
    }

    // Application payload
    int32_t raw_plen = len - hdr_len;
    if (raw_plen < 0) raw_plen = 0;
    const uint8_t *p = payload + hdr_len;

    if (msg->signature_present) {
        const uint8_t *key = NULL;
        int32_t key_len = 0;
        if (fanet_get_signature_key(&key, &key_len)) {
            msg->signature_checked = true;
            msg->signature_valid = fanet_verify_signature(payload, p, raw_plen,
                                                          key, key_len, msg->signature);
        }
    }

    int32_t plen = raw_plen;
    if (plen > FANET_MAX_PAYLOAD) plen = FANET_MAX_PAYLOAD;
    if (plen > 0) memcpy(msg->payload, payload + hdr_len, plen);
    msg->payload_len = (uint8_t)plen;

    switch (msg->type) {
    case FANET_TYPE_ACK:
        break;

    case FANET_TYPE_TRACKING:
        if (plen < 6) break;
        msg->tracking.latitude = fanet_decode_lat(p);
        msg->tracking.longitude = fanet_decode_lon(p + 3);
        msg->tracking.position_valid = true;

        if (plen >= 8) {
            uint16_t type_alt = (uint16_t)p[6] | ((uint16_t)p[7] << 8);
            msg->tracking.online_tracking = (type_alt >> 15) & 1;
            msg->tracking.aircraft_type = (fanet_aircraft_type_t)((type_alt >> 12) & 0x07);
            bool alt_scale = (type_alt >> 11) & 1;
            msg->tracking.altitude = (int32_t)(type_alt & 0x07FF);
            if (alt_scale) msg->tracking.altitude *= 4;
        }

        if (plen >= 9)
            msg->tracking.speed = decode_scaled_unsigned(p[8], 0.5f);

        if (plen >= 10)
            msg->tracking.climb = decode_scaled_signed(p[9], 0.1f, 5.0f);

        if (plen >= 11)
            msg->tracking.heading = p[10] * (360.0f / 256.0f);

        msg->tracking.turn_rate = NAN;
        msg->tracking.qne_offset = INT_MIN;

        if (plen >= 12)
            msg->tracking.turn_rate = decode_scaled_signed(p[11], 0.25f, 4.0f);

        if (plen >= 13) {
            int8_t raw = (int8_t)((p[12] & 0x7F) | ((p[12] & 0x40) ? 0x80 : 0));
            msg->tracking.qne_offset = (int32_t)raw;
            if (p[12] & 0x80) msg->tracking.qne_offset *= 4;
        }
        break;

    case FANET_TYPE_NAME:
        if (plen > 0) {
            int32_t nlen = plen > 31 ? 31 : plen;
            memcpy(msg->name, p, nlen);
            msg->name[nlen] = '\0';
        }
        break;

    case FANET_TYPE_MESSAGE:
        if (plen >= 1) {
            msg->message.subtype = p[0];
            int32_t tlen = plen - 1;
            if (tlen > (int32_t)sizeof(msg->message.text) - 1)
                tlen = (int32_t)sizeof(msg->message.text) - 1;
            if (tlen > 0) memcpy(msg->message.text, p + 1, tlen);
            msg->message.text[tlen > 0 ? tlen : 0] = '\0';
        }
        break;

    case FANET_TYPE_SERVICE: {
        if (plen < 1) break;
        uint8_t header = p[0];
        int32_t idx = 1;

        msg->weather.is_gateway = (header >> 7) & 1;
        msg->weather.remote_cfg = (header >> 2) & 1;
        msg->weather.has_position = false;
        msg->weather.has_temp = false;
        msg->weather.has_wind = false;
        msg->weather.has_humidity = false;
        msg->weather.has_pressure = false;
        msg->weather.has_soc = false;
        msg->weather.state_of_charge = -1;
        msg->weather.latitude = NAN;
        msg->weather.longitude = NAN;

        // Extended header (bit 0)
        if (header & 0x01) {
            if (idx < plen) idx++;
        }

        // Position (if any data bits 6-1 set)
        bool has_data = (header & 0x7E) != 0;
        if (has_data && idx + 6 <= plen) {
            msg->weather.latitude = fanet_decode_lat(p + idx);
            msg->weather.longitude = fanet_decode_lon(p + idx + 3);
            msg->weather.has_position = true;
            idx += 6;
        }

        // Temperature (bit 6)
        if ((header & 0x40) && idx < plen) {
            msg->weather.temperature = ((int8_t)p[idx]) * 0.5f;
            msg->weather.has_temp = true;
            idx++;
        }

        // Wind (bit 5): heading + speed + gusts
        if ((header & 0x20) && idx + 3 <= plen) {
            msg->weather.wind_heading = p[idx] * (360.0f / 256.0f);
            idx++;
            msg->weather.wind_speed = decode_scaled_unsigned(p[idx], 0.2f);
            idx++;
            msg->weather.wind_gust = decode_scaled_unsigned(p[idx], 0.2f);
            idx++;
            msg->weather.has_wind = true;
        }

        // Humidity (bit 4)
        if ((header & 0x10) && idx < plen) {
            msg->weather.humidity = p[idx] * 0.4f;
            msg->weather.has_humidity = true;
            idx++;
        }

        // Barometric pressure (bit 3): 2 bytes, 10 Pa, offset 430 hPa
        if ((header & 0x08) && idx + 2 <= plen) {
            uint16_t press_raw = (uint16_t)p[idx] | ((uint16_t)p[idx+1] << 8);
            msg->weather.pressure = press_raw * 0.1f + 430.0f;
            msg->weather.has_pressure = true;
            idx += 2;
        }

        // State of Charge (bit 1)
        if ((header & 0x02) && idx < plen) {
            msg->weather.state_of_charge = (p[idx] & 0x0F) * (100.0f / 15.0f);
            msg->weather.has_soc = true;
            idx++;
        }
        break;
    }

    case FANET_TYPE_LANDMARK: {
        if (plen < 1) break;
        uint8_t ttl_raw = (p[0] >> 4) & 0x0F;
        bool ttl_scale = (ttl_raw >> 3) & 1;
        uint16_t ttl_val = (ttl_raw & 0x07);
        msg->landmark.ttl_minutes = (ttl_val + 1) * 10;
        if (ttl_scale) msg->landmark.ttl_minutes *= 6;

        msg->landmark.subtype = (fanet_landmark_subtype_t)(p[0] & 0x0F);

        if (plen < 2) break;
        msg->landmark.wind_dependent = (p[1] >> 4) & 1;
        msg->landmark.layer = (fanet_landmark_layer_t)(p[1] & 0x0F);

        int32_t idx = 2;

        msg->landmark.wind_sectors = 0;
        if (msg->landmark.wind_dependent && idx < plen) {
            msg->landmark.wind_sectors = p[idx];
            idx++;
        }

        // First position (absolute)
        if (idx + 6 <= plen) {
            msg->landmark.latitude = fanet_decode_lat(p + idx);
            msg->landmark.longitude = fanet_decode_lon(p + idx + 3);
            idx += 6;
        }

        // For text landmarks: string follows
        if (msg->landmark.subtype == FANET_LANDMARK_TEXT && idx < plen) {
            int32_t tlen = plen - idx;
            if (tlen > (int32_t)sizeof(msg->landmark.text) - 1)
                tlen = (int32_t)sizeof(msg->landmark.text) - 1;
            memcpy(msg->landmark.text, p + idx, tlen);
            msg->landmark.text[tlen] = '\0';
        } else {
            msg->landmark.text[0] = '\0';
        }

        // Additional points (compressed coordinates) for non-text types
        msg->landmark.num_points = 0;
        if (msg->landmark.subtype != FANET_LANDMARK_TEXT) {
            double ref_lat = msg->landmark.latitude;
            double ref_lon = msg->landmark.longitude;
            int32_t pt_idx = 0;

            while (idx + 4 <= plen && pt_idx < FANET_LANDMARK_MAX_POINTS) {
                double lat = fanet_decode_compressed(p + idx, ref_lat);
                double lon = fanet_decode_compressed(p + idx + 2, ref_lon);
                msg->landmark.points[pt_idx].latitude = lat;
                msg->landmark.points[pt_idx].longitude = lon;
                msg->landmark.points[pt_idx].radius = 0;
                msg->landmark.points[pt_idx].altitude_bottom = 0;
                msg->landmark.points[pt_idx].altitude_top = 0;
                idx += 4;

                // Circle types have radius byte
                if ((msg->landmark.subtype == FANET_LANDMARK_CIRCLE ||
                     msg->landmark.subtype == FANET_LANDMARK_CIRCLE_FILLED) && idx < plen) {
                    uint8_t r = p[idx++];
                    float radius = (r & 0x7F) * 50.0f;
                    if (r & 0x80) radius *= 8.0f;
                    msg->landmark.points[pt_idx].radius = radius;
                }

                // 3D types have altitude bytes
                if (msg->landmark.subtype == FANET_LANDMARK_3D_LINE && idx < plen) {
                    msg->landmark.points[pt_idx].altitude_bottom =
                        ((int8_t)p[idx] + 109) * 25;
                    idx++;
                }

                ref_lat = lat;
                ref_lon = lon;
                pt_idx++;
            }

            // 3D Area and 3D Cylinder: altitude bottom + top at start
            // (already consumed above, just set count)
            msg->landmark.num_points = (uint8_t)pt_idx;
        }
        break;
    }

    case FANET_TYPE_REMOTE:
        // Remote configuration: parse subtypes per FANET protocol spec
        if (plen >= 1) {
            uint8_t subtype = p[0];
            msg->remote.subtype = (fanet_remote_subtype_t)subtype;
            msg->remote.valid = true;
            msg->remote.has_position = false;
            msg->remote.has_altitude = false;
            msg->remote.has_heading = false;
            msg->remote.geofence_num_points = 0;

            if (subtype == 0 && plen >= 2) {
                // Ack: byte[1] = subtype being acknowledged
                msg->remote.ack_subtype = p[1];
            } else if (subtype == 1 && plen >= 2) {
                // Request: byte[1] = subtype being requested
                msg->remote.request_subtype = p[1];
            } else if (subtype == 2) {
                // Position: bytes[1-6] lat/lon, byte[7] alt, byte[8] heading
                if (plen >= 7) {
                    msg->remote.latitude = fanet_decode_lat(p + 1);
                    msg->remote.longitude = fanet_decode_lon(p + 4);
                    msg->remote.has_position = true;
                }
                if (plen >= 8) {
                    msg->remote.altitude = ((int32_t)(int8_t)p[7] + 109) * 25;
                    msg->remote.has_altitude = true;
                }
                if (plen >= 9) {
                    msg->remote.heading = p[8] * (360.0f / 256.0f);
                    msg->remote.has_heading = true;
                }
            } else if (subtype >= 4 && subtype <= 8) {
                // Geofence: altitude bottom + top, then n positions
                if (plen >= 3) {
                    msg->remote.geofence_alt_bottom = ((int32_t)(int8_t)p[1] + 109) * 25;
                    msg->remote.geofence_alt_top    = ((int32_t)(int8_t)p[2] + 109) * 25;
                    // Remaining bytes: compressed positions (4 bytes each)
                    int32_t idx = 3;
                    int32_t pt = 0;
                    double ref_lat = 0, ref_lon = 0;
                    while (idx + 3 < plen && pt < FANET_LANDMARK_MAX_POINTS) {
                        if (pt == 0) {
                            // First position is absolute (6 bytes)
                            if (idx + 6 > plen) break;
                            ref_lat = fanet_decode_lat(p + idx);
                            ref_lon = fanet_decode_lon(p + idx + 3);
                            msg->remote.geofence_points[pt].latitude = ref_lat;
                            msg->remote.geofence_points[pt].longitude = ref_lon;
                            idx += 6;
                        } else {
                            // Subsequent positions are compressed (4 bytes)
                            if (idx + 4 > plen) break;
                            msg->remote.geofence_points[pt].latitude = fanet_decode_compressed(p + idx, ref_lat);
                            msg->remote.geofence_points[pt].longitude = fanet_decode_compressed(p + idx + 2, ref_lon);
                            ref_lat = msg->remote.geofence_points[pt].latitude;
                            ref_lon = msg->remote.geofence_points[pt].longitude;
                            idx += 4;
                        }
                        pt++;
                    }
                    msg->remote.geofence_num_points = (uint8_t)pt;
                }
            } else if (subtype >= 9 && subtype <= 33) {
                // Broadcast reply feature
                if (plen >= 3) {
                    msg->remote.reply_wind_sectors = p[1];
                    msg->remote.reply_type = p[2] & 0x3F;
                    msg->remote.reply_forward = (p[2] >> 6) & 1;
                }
            }
        }
        break;

    case FANET_TYPE_GROUND:
        if (plen < 6) break;
        msg->ground.latitude = fanet_decode_lat(p);
        msg->ground.longitude = fanet_decode_lon(p + 3);
        msg->ground.position_valid = true;

        if (plen >= 7) {
            msg->ground.ground_type = (fanet_ground_type_t)((p[6] >> 4) & 0x0F);
            msg->ground.online_tracking = p[6] & 1;
        }
        break;

    case FANET_TYPE_HWINFO:
        if (plen >= 1) {
            msg->hwinfo.device_type = p[0];
            msg->hwinfo.has_build_date = false;
            msg->hwinfo.has_icao = false;
            msg->hwinfo.has_uptime = false;
            msg->hwinfo.has_rssi = false;
        }
        if (plen >= 3) {
            msg->hwinfo.build_date = (uint16_t)p[1] | ((uint16_t)p[2] << 8);
            msg->hwinfo.has_build_date = true;
        }
        break;

    case FANET_TYPE_THERMAL: {
        if (plen < 8) break;
        msg->thermal.latitude = fanet_decode_lat(p);
        msg->thermal.longitude = fanet_decode_lon(p + 3);

        uint16_t type_field = (uint16_t)p[6] | ((uint16_t)p[7] << 8);
        msg->thermal.confidence = (type_field >> 12) & 0x07;
        bool alt_scale = (type_field >> 11) & 1;
        msg->thermal.altitude = (int32_t)(type_field & 0x07FF);
        if (alt_scale) msg->thermal.altitude *= 4;

        if (plen >= 9)
            msg->thermal.climb = decode_scaled_signed(p[8], 0.1f, 5.0f);
        if (plen >= 10)
            msg->thermal.wind_speed = decode_scaled_unsigned(p[9], 0.5f);
        if (plen >= 11)
            msg->thermal.wind_heading = p[10] * (360.0f / 256.0f);

        msg->thermal.valid = true;
        break;
    }

    case FANET_TYPE_HWINFO2: {
        if (plen < 1) break;
        uint8_t subhdr = p[0];
        int32_t idx = 1;

        msg->hwinfo.has_build_date = false;
        msg->hwinfo.has_icao = false;
        msg->hwinfo.has_uptime = false;
        msg->hwinfo.has_rssi = false;
        msg->hwinfo.icao_address = 0;
        msg->hwinfo.uptime_minutes = 0;
        msg->hwinfo.rssi = 0;

        if (subhdr & 0x01) {
            if (idx < plen) idx++;
        }

        if ((subhdr & 0x40) && idx + 3 <= plen) {
            msg->hwinfo.device_type = p[idx];
            msg->hwinfo.build_date = (uint16_t)p[idx+1] | ((uint16_t)p[idx+2] << 8);
            msg->hwinfo.has_build_date = true;
            idx += 3;
        }

        if ((subhdr & 0x20) && idx + 3 <= plen) {
            msg->hwinfo.icao_address = (uint32_t)p[idx] | ((uint32_t)p[idx+1] << 8) | ((uint32_t)p[idx+2] << 16);
            msg->hwinfo.has_icao = true;
            idx += 3;
        }

        if ((subhdr & 0x10) && idx + 2 <= plen) {
            msg->hwinfo.uptime_minutes = (uint16_t)p[idx] | ((uint16_t)p[idx+1] << 8);
            msg->hwinfo.has_uptime = true;
            idx += 2;
        }

        if ((subhdr & 0x08) && idx + 1 <= plen) {
            msg->hwinfo.rssi = (int8_t)(p[idx]) - 50;
            msg->hwinfo.has_rssi = true;
            idx += 1;
        }
        break;
    }

    default:
        break;
    }

    msg->valid = true;
    return true;
}

// ======================== Packet assembly & decode ========================

static void decode_packet(fanet_state_t *s)
{
    if (s->raw_sym_count < 10) return;

    // LoRa PHY parameters
    int32_t sf = FANET_SF;             // 7
    int32_t sf_hdr = sf - 2;           // 5 (reduced rate for header)
    int32_t header_cr = 4;             // header always CR 4/8
    int32_t header_cw_len = header_cr + 4;  // 8

    // Step 1: Gray demap all symbols at full rate (for diagnostic + payload use)
    uint8_t symbols[256];
    for (int32_t i = 0; i < s->raw_sym_count && i < 256; i++) {
        symbols[i] = gray_demap(s->raw_symbols[i], sf, 0);
    }

    // Step 2: Decode header block
    // Header uses SF-2 bits per symbol and CR=4/8
    // Number of header symbols: cw_len = 8 (for CR=4/8)
    int32_t payload_cr = 1;
    int32_t expected_len = 0;
    bool has_crc = true;

    if (s->raw_sym_count < header_cw_len) {
        s->stats.header_errors++;
        return;
    }

    // Header symbols: reduced rate (divide by 4 before gray demap)
    uint8_t hdr_symbols[8];
    for (int32_t i = 0; i < header_cw_len; i++) {
        hdr_symbols[i] = gray_demap(s->raw_symbols[i], sf, 1);
    }

#ifdef FANET_DEBUG
    fprintf(stderr, "DBG: hdr_symbols (masked to %d bits): ", sf_hdr);
    for (int32_t i = 0; i < header_cw_len; i++) fprintf(stderr, "%d ", hdr_symbols[i]);
    fprintf(stderr, "\n");
#endif

    uint8_t hdr_codewords[128];
    int32_t hdr_num_cw = 0;
    deinterleave(hdr_symbols, header_cw_len, sf_hdr, header_cw_len,
                 hdr_codewords, &hdr_num_cw);

#ifdef FANET_DEBUG
    fprintf(stderr, "DBG: hdr_codewords (%d): ", hdr_num_cw);
    for (int32_t i = 0; i < hdr_num_cw; i++) fprintf(stderr, "0x%02X ", hdr_codewords[i]);
    fprintf(stderr, "\n");
#endif

    uint8_t hdr_nibbles[64];
    int32_t hdr_num = 0;
    for (int32_t i = 0; i < hdr_num_cw && hdr_num < 64; i++) {
        hdr_nibbles[hdr_num++] = hamming_decode_nibble(hdr_codewords[i], header_cr);
    }

#ifdef FANET_DEBUG
    fprintf(stderr, "DBG: hdr_nibbles (%d): ", hdr_num);
    for (int32_t i = 0; i < hdr_num; i++) fprintf(stderr, "0x%X ", hdr_nibbles[i]);
    fprintf(stderr, "\n");
#endif

    if (hdr_num >= 3) {
        expected_len = (hdr_nibbles[0] << 4) | (hdr_nibbles[1] & 0x0F);
        payload_cr = ((hdr_nibbles[2] >> 1) & 0x07);
        if (payload_cr < 1) payload_cr = 1;
        if (payload_cr > 4) payload_cr = 4;
        has_crc = 1; // FANET always uses CRC
    }

#ifdef FANET_DEBUG
    fprintf(stderr, "DBG: expected_len=%d, payload_cr=%d, has_crc=%d\n", expected_len, payload_cr, has_crc);
#endif

    // Step 3: Decode payload symbols
    int32_t payload_sym_start = header_cw_len;
    int32_t payload_sym_count = s->raw_sym_count - payload_sym_start;
    if (payload_sym_count < 0) payload_sym_count = 0;

    int32_t payload_cw_len = payload_cr + 4;
    int32_t sf_pay = sf; // payload uses full SF

    // Payload symbols: full rate gray demap (already computed in symbols[])
    uint8_t pay_symbols[256];
    for (int32_t i = 0; i < payload_sym_count && i < 256; i++) {
        pay_symbols[i] = symbols[payload_sym_start + i];
    }

    uint8_t codewords[256];
    int32_t num_codewords = 0;
    if (payload_sym_count > 0) {
        deinterleave(pay_symbols, payload_sym_count, sf_pay, payload_cw_len,
                     codewords, &num_codewords);
    }

    uint8_t nibbles[128];
    int32_t num_nibbles = 0;
    for (int32_t i = 0; i < num_codewords && num_nibbles < 128; i++) {
        nibbles[num_nibbles++] = hamming_decode_nibble(codewords[i], payload_cr);
    }

    uint8_t payload[64];
    int32_t payload_len = num_nibbles / 2;
    if (payload_len > 64) payload_len = 64;
    for (int32_t i = 0; i < payload_len; i++) {
        payload[i] = (nibbles[i*2 + 1] << 4) | (nibbles[i*2] & 0x0F);
    }

    if (expected_len > 0 && expected_len + (has_crc ? 2 : 0) < payload_len) {
        payload_len = expected_len + (has_crc ? 2 : 0);
    }

    // Step 4: De-whiten (only payload bytes, not CRC)
    int32_t data_only_len = has_crc ? (payload_len - 2) : payload_len;
    if (data_only_len < 0) data_only_len = 0;
    lora_dewhiten(payload, data_only_len);

    // Step 5: CRC check
    int32_t crc_ok = 0;
    if (has_crc) {
        if (payload_len < 4) {
            s->stats.crc_errors++;
            return;
        }
        int32_t data_len = payload_len - 2;
        uint16_t crc_recv = (uint16_t)payload[data_len] | ((uint16_t)payload[data_len + 1] << 8);
        // Method A: LoRa CRC (init=0, XOR last 2 data bytes)
        uint16_t crc_a = crc16_lora(payload, data_len - 2);
        crc_a ^= ((uint16_t)payload[data_len - 2] << 8) | (uint16_t)payload[data_len - 1];
        // Method B: simple CRC on all data (init=0)
        uint16_t crc_b = crc16_lora(payload, data_len);

        if (crc_recv == crc_a || crc_recv == crc_b) {
            crc_ok = 1;
        } else {
            s->stats.crc_errors++;
            fprintf(stderr, "fanet: CRC fail sym=%d len=%d cr=%d recv=%04X calcA=%04X calcB=%04X type=%u src=%02X:%02X%02X\n",
                    s->raw_sym_count, expected_len, payload_cr, crc_recv, crc_a, crc_b,
                    (uint32_t)(payload[0] & 0x3F),
                    (uint32_t)payload[1],
                    (uint32_t)payload[3], (uint32_t)payload[2]);
            // Continue to parse for diagnostics (won't be dispatched)
        }
        payload_len -= 2;
    }

    // Step 6: Parse FANET
    fanet_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.timestamp_ms = 0;
    msg.signal_level = sqrtf(s->packet_power / (float)(s->raw_sym_count + 1));
    msg.tracking.turn_rate = NAN;
    msg.tracking.qne_offset = INT_MIN;
    msg.weather.state_of_charge = -1;
    msg.weather.latitude = NAN;
    msg.weather.longitude = NAN;

    if (parse_fanet_packet(payload, payload_len, &msg)) {
        fprintf(stderr, "FANET: %s type=%u src=%02X:%04X len=%d preamble_off=%d\n",
                crc_ok ? "decoded" : "TENTATIVE",
                (uint32_t)msg.type, msg.src_manufacturer, msg.src_id, payload_len,
                s->preamble_bin_offset);

        // Update name cache
        if (crc_ok && msg.type == FANET_TYPE_NAME && msg.name[0]) {
            pthread_mutex_lock(&s->name_mutex);
            int32_t found = -1, oldest = 0;
            uint64_t oldest_time = UINT64_MAX;
            for (int32_t i = 0; i < s->name_cache_count; i++) {
                if (s->name_cache[i].manufacturer == msg.src_manufacturer &&
                    s->name_cache[i].id == msg.src_id) {
                    found = i;
                    break;
                }
                if (s->name_cache[i].last_seen < oldest_time) {
                    oldest_time = s->name_cache[i].last_seen;
                    oldest = i;
                }
            }
            if (found >= 0) {
                snprintf(s->name_cache[found].name, sizeof(s->name_cache[found].name), "%s", msg.name);
                s->name_cache[found].last_seen = msg.timestamp_ms;
            } else {
                int32_t idx = s->name_cache_count < FANET_NAME_CACHE_SIZE
                          ? s->name_cache_count++ : oldest;
                s->name_cache[idx].manufacturer = msg.src_manufacturer;
                s->name_cache[idx].id = msg.src_id;
                snprintf(s->name_cache[idx].name, sizeof(s->name_cache[idx].name), "%s", msg.name);
                s->name_cache[idx].last_seen = msg.timestamp_ms;
            }
            pthread_mutex_unlock(&s->name_mutex);
        }

        if (crc_ok && msg.type < 16) s->stats.type_counts[msg.type]++;

        if (crc_ok && msg_queue_push(s->out_queue, &msg)) {
            s->stats.packets_decoded++;
        }
    }
}

// ======================== Symbol processing state machine ========================

static void process_symbol(fanet_state_t *s, float *buf_i, float *buf_q)
{
    float peak_pwr = 0;
    int32_t bin = demodulate_symbol(s, buf_i, buf_q, &peak_pwr);

    if (!s->in_packet) {
        // --- PREAMBLE DETECTION ---
        int32_t diff = abs(bin - s->last_peak_bin);
        if (diff > LORA_N / 2) diff = LORA_N - diff;

        if (s->preamble_count == 0 || diff <= 1) {
            s->preamble_count++;
            s->last_peak_bin = bin;
        } else {
            if (s->preamble_count >= 6) {
                s->stats.preambles_detected++;
                // Estimate frequency offset from preamble bin
                s->preamble_bin_offset = s->last_peak_bin;
                s->freq_offset = (float)s->preamble_bin_offset;
                s->sync_symbols[0] = (bin - s->preamble_bin_offset + LORA_N) % LORA_N;
                s->sfd_count = 0;
                s->in_packet = true;
                s->packet_sym_idx = 0;
                s->raw_sym_count = 0;
                s->packet_power = 0;
            }
            s->preamble_count = 1;
            s->last_peak_bin = bin;
        }
    } else {
        // --- IN PACKET ---
        int32_t corrected_bin = (bin - s->preamble_bin_offset + LORA_N) % LORA_N;
        s->packet_sym_idx++;

        // Sync word verification (2 sync symbols after preamble)
        if (s->packet_sym_idx == 1) {
            s->sync_symbols[1] = corrected_bin;

            // FANET sync word 0xF1:
            // High nibble (0xF=15) → sym0 = 15 * (128/16) = 15*8 = 120
            // Low nibble  (0x1=1)  → sym1 = 1 * (128/16) = 1*8 = 8
            int32_t expected_s0 = ((FANET_SYNC_WORD >> 4) & 0x0F) * (LORA_N / 16);
            int32_t expected_s1 = ((FANET_SYNC_WORD >> 0) & 0x0F) * (LORA_N / 16);
            int32_t tol = 3;

            int32_t d0 = abs(s->sync_symbols[0] - expected_s0);
            if (d0 > LORA_N / 2) d0 = LORA_N - d0;
            int32_t d1 = abs(s->sync_symbols[1] - expected_s1);
            if (d1 > LORA_N / 2) d1 = LORA_N - d1;

            if (d0 > tol || d1 > tol) {
                s->in_packet = false;
                s->stats.header_errors++;
                return;
            }
            s->stats.sync_word_ok++;
            return;
        }

        // SFD detection: 2.25 downchirps (check 3 symbols for downchirp)
        if (s->packet_sym_idx <= 4) {
            int32_t dc_peak = -1;
            bool dc = is_downchirp(s, buf_i, buf_q, &dc_peak);
            if (dc) s->sfd_count++;
            // After checking all SFD slots: need at least 2 downchirps
            if (s->packet_sym_idx == 4) {
                if (s->sfd_count < 2) {
                    s->in_packet = false;
                    return;
                }
                // Real LoRa uses 2.25 downchirps. The 3rd SFD slot always
                // contains 0.25 downchirp + 0.75 of the first data symbol.
                // Always apply timing correction to realign to data boundary.
                s->sfd_adjust = true;
            }
            return;
        }

        // Data symbols (apply CFO correction: subtract fractional offset)
        if (s->raw_sym_count < 256) {
            s->raw_symbols[s->raw_sym_count++] = (uint8_t)(corrected_bin & 0x7F);
            s->packet_power += peak_pwr;
        }

        // End of packet detection
        bool end_packet = false;
        if (s->raw_sym_count >= 200 || s->packet_sym_idx > 210) {
            end_packet = true;
        } else if (s->raw_sym_count >= 4) {
            float avg_pwr = s->packet_power / s->raw_sym_count;
            if (peak_pwr < avg_pwr * 0.1f) {
                end_packet = true;
            }
        }

        if (end_packet) {
            decode_packet(s);
            s->in_packet = false;
        }
    }
}

// ======================== Public API ========================

fanet_state_t *fanet_create(uint32_t sample_rate)
{
    fanet_state_t *s = calloc(1, sizeof(fanet_state_t));
    if (!s) return NULL;

    s->sample_rate = sample_rate;
    s->samples_per_symbol = (uint32_t)((double)sample_rate * LORA_N / FANET_BW);

    s->dechirp_i = calloc(s->samples_per_symbol, sizeof(float));
    s->dechirp_q = calloc(s->samples_per_symbol, sizeof(float));
    s->downchirp_i = calloc(s->samples_per_symbol, sizeof(float));
    s->downchirp_q = calloc(s->samples_per_symbol, sizeof(float));
    s->sym_buf_i = calloc(s->samples_per_symbol, sizeof(float));
    s->sym_buf_q = calloc(s->samples_per_symbol, sizeof(float));
    s->fft_mag = calloc(LORA_N, sizeof(float));

    if (!s->dechirp_i || !s->dechirp_q || !s->downchirp_i || !s->downchirp_q ||
        !s->sym_buf_i || !s->sym_buf_q || !s->fft_mag) {
        fanet_destroy(s);
        return NULL;
    }

    s->out_queue = msg_queue_create(sizeof(fanet_message_t), MAX_MSG_QUEUE);
    if (!s->out_queue) {
        fanet_destroy(s);
        return NULL;
    }

    pthread_mutex_init(&s->name_mutex, NULL);
    generate_dechirp(s);

    fprintf(stderr, "fanet: decoder created, sr=%u, sps=%u, sym_rate=%.1f Hz, sync=0x%02X\n",
            sample_rate, s->samples_per_symbol,
            (double)FANET_BW / LORA_N, (uint32_t)FANET_SYNC_WORD);

    return s;
}

void fanet_destroy(fanet_state_t *s)
{
    if (!s) return;
    pthread_mutex_destroy(&s->name_mutex);
    if (s->out_queue) msg_queue_destroy(s->out_queue);
    free(s->dechirp_i);
    free(s->dechirp_q);
    free(s->downchirp_i);
    free(s->downchirp_q);
    free(s->sym_buf_i);
    free(s->sym_buf_q);
    free(s->fft_mag);
    free(s);
}

void fanet_process(fanet_state_t *s, const uint8_t *iq, uint32_t len)
{
    uint32_t num_samples = len / 2;
    for (uint32_t i = 0; i < num_samples; i++) {
        float I = (float)((int32_t)iq[i*2]     - 128) / 128.0f;
        float Q = (float)((int32_t)iq[i*2 + 1] - 128) / 128.0f;

        s->sym_buf_i[s->sym_buf_pos] = I;
        s->sym_buf_q[s->sym_buf_pos] = Q;
        s->sym_buf_pos++;

        if ((uint32_t)s->sym_buf_pos >= s->samples_per_symbol) {
            process_symbol(s, s->sym_buf_i, s->sym_buf_q);

            if (s->sfd_adjust) {
                // SFD timing correction for 2.25 downchirps:
                // The last SFD slot contained 0.25 downchirp + 0.75 of the
                // first data symbol. Shift the data portion to the buffer
                // start so the next fill completes data symbol 0 correctly.
                int32_t quarter = (int32_t)(s->samples_per_symbol / 4);
                int32_t data_len = (int32_t)s->samples_per_symbol - quarter;
                memmove(s->sym_buf_i, s->sym_buf_i + quarter, data_len * sizeof(float));
                memmove(s->sym_buf_q, s->sym_buf_q + quarter, data_len * sizeof(float));
                s->sym_buf_pos = data_len;
                s->sfd_adjust = false;
            } else {
                s->sym_buf_pos = 0;
            }

            s->stats.samples_processed += s->samples_per_symbol;
        }
    }
}

bool fanet_dequeue(fanet_state_t *s, fanet_message_t *msg)
{
    return msg_queue_pop(s->out_queue, msg) != 0;
}

void fanet_get_stats(const fanet_state_t *s, fanet_stats_t *stats)
{
    *stats = s->stats;
}

bool fanet_get_cached_name(const fanet_state_t *s,
                           uint8_t manufacturer, uint16_t id,
                           char *buf, int32_t buf_len)
{
    if (!s || !buf || buf_len < 1) return false;
    pthread_mutex_lock((pthread_mutex_t *)&s->name_mutex);
    for (int32_t i = 0; i < s->name_cache_count; i++) {
        if (s->name_cache[i].manufacturer == manufacturer &&
            s->name_cache[i].id == id) {
            strncpy(buf, s->name_cache[i].name, buf_len - 1);
            buf[buf_len - 1] = '\0';
            pthread_mutex_unlock((pthread_mutex_t *)&s->name_mutex);
            return true;
        }
    }
    pthread_mutex_unlock((pthread_mutex_t *)&s->name_mutex);
    buf[0] = '\0';
    return false;
}

const char *fanet_manufacturer_name(uint8_t mfr_id)
{
    switch (mfr_id) {
        case 0x01: return "Skytraxx";
        case 0x03: return "BitBroker";
        case 0x04: return "AirWhere";
        case 0x05: return "Windline";
        case 0x06: return "Burnair";
        case 0x07: return "SoftRF";
        case 0x08: return "GXAircom";
        case 0x09: return "Airtribune";
        case 0x0A: return "FLARM";
        case 0x0B: return "FlyBeeper";
        case 0x0C: return "Leaf";
        case 0x10: return "alfapilot";
        case 0x11: return "FANET+";
        case 0x20: return "XCTracer";
        case 0xBA: return "Burnair";
        case 0xCB: return "Cloudbuddy";
        case 0xE0: return "OGN";
        case 0xE4: return "4aviation";
        case 0xFA: return "Various";
        case 0xFB: return "ESP-base";
        case 0xFC: case 0xFD: return "Unregistered";
        default: return "Unknown";
    }
}
