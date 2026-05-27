// Part of dump1090-gg-light
//
// lte_sib.c: LTE SIB decoding — PDCCH, PDSCH, and ASN.1 UPER parsing.
//
// Implements passive decoding of System Information Blocks from LTE cells:
// - PDCCH blind search for SI-RNTI (DCI format 1A/1C)
// - PDSCH transport block decode (turbo code, rate 1/3)
// - ASN.1 UPER parsers for SIB1, SIB4-7, SIB10-12, SIB14
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#include "lte_sib.h"
#include <stdint.h>
#include "lte_decode.h"
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ======================== Internal Types ========================

typedef struct { float re, im; } cf_t;
static inline cf_t cf_mul(cf_t a, cf_t b) { return (cf_t){a.re*b.re-a.im*b.im, a.re*b.im+a.im*b.re}; }
static inline cf_t cf_conj(cf_t a) { return (cf_t){a.re, -a.im}; }
static inline float cf_abs2(cf_t a) { return a.re*a.re + a.im*a.im; }

// ======================== ASN.1 UPER Bit Reader ========================

typedef struct {
    const uint8_t *bits;    // Bit array (1 bit per byte, values 0 or 1)
    int32_t            len;     // Total bits available
    int32_t            pos;     // Current read position
} bitreader_t;

static inline void br_init(bitreader_t *br, const uint8_t *bits, int32_t len) {
    br->bits = bits; br->len = len; br->pos = 0;
}

static inline int32_t br_remaining(const bitreader_t *br) {
    return br->len - br->pos;
}

static inline uint32_t br_read(bitreader_t *br, int n) {
    if (n <= 0 || br->pos + n > br->len) { br->pos = br->len; return 0; }
    uint32_t val = 0;
    for (int32_t i = 0; i < n; i++)
        val = (val << 1) | (br->bits[br->pos++] & 1);
    return val;
}

static inline bool br_read_bool(bitreader_t *br) {
    return br_read(br, 1) != 0;
}

static inline void br_skip(bitreader_t *br, int32_t n) {
    br->pos += n;
    if (br->pos > br->len) br->pos = br->len;
}

// Read constrained whole number (UPER)
static inline uint32_t br_read_constrained(bitreader_t *br, uint32_t lb, uint32_t ub) {
    uint32_t range = ub - lb + 1;
    int32_t bits_needed = 0;
    uint32_t r = range - 1;
    while (r > 0) { bits_needed++; r >>= 1; }
    return lb + br_read(br, bits_needed);
}

// Read length determinant (unconstrained)
static inline int32_t br_read_length(bitreader_t *br) {
    if (!br_read_bool(br)) return (int)br_read(br, 7);  // < 128
    return (int)br_read(br, 14);  // < 16384
}

static uint8_t sib2_ra_preambles_to_value(uint32_t idx)
{
    static const uint8_t values[] = {
        4, 8, 12, 16, 20, 24, 28, 32,
        36, 40, 44, 48, 52, 56, 60, 64
    };
    return (idx < (sizeof(values) / sizeof(values[0]))) ? values[idx] : 64;
}

static uint8_t sib2_power_ramping_to_db(uint32_t idx)
{
    static const uint8_t values[] = { 0, 2, 4, 6 };
    return (idx < 4) ? values[idx] : 6;
}

static uint8_t sib2_preamble_trans_max_to_value(uint32_t idx)
{
    static const uint8_t values[] = { 3, 4, 5, 6, 7, 8, 10, 20, 50, 100, 200 };
    return (idx < (sizeof(values) / sizeof(values[0]))) ? values[idx] : 200;
}

static uint8_t sib2_ra_window_to_sf(uint32_t idx)
{
    static const uint8_t values[] = { 2, 3, 4, 5, 6, 7, 8, 10 };
    return (idx < 8) ? values[idx] : 10;
}

static uint8_t sib2_ul_bw_to_rb(uint32_t idx)
{
    static const uint8_t values[] = { 6, 15, 25, 50, 75, 100 };
    return (idx < 6) ? values[idx] : 100;
}

static uint16_t sib2_time_align_to_sf(uint32_t idx)
{
    static const uint16_t values[] = { 500, 750, 1280, 1920, 2560, 5120, 10240, 0xFFFF };
    return (idx < 8) ? values[idx] : 0xFFFF;
}

static uint8_t sib3_q_hyst_to_db(uint32_t idx)
{
    static const uint8_t values[] = { 0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24 };
    return (idx < (sizeof(values) / sizeof(values[0]))) ? values[idx] : 24;
}

// ======================== CRC-24A (for PDSCH transport blocks) ========================

static uint32_t crc24a(const uint8_t *bits, int32_t nbits)
{
    uint32_t crc = 0;
    for (int32_t i = 0; i < nbits; i++) {
        uint32_t bit = (crc >> 23) ^ bits[i];
        crc = (crc << 1) & 0xFFFFFF;
        if (bit) crc ^= 0x864CFB;
    }
    return crc;
}

// ======================== CRC-16 (for PDCCH DCI) ========================

static uint16_t crc16_dci(const uint8_t *bits, int32_t nbits, uint16_t rnti)
{
    uint16_t crc = 0xFFFF;
    for (int32_t i = 0; i < nbits; i++) {
        uint16_t bit = (crc >> 15) ^ bits[i];
        crc = (crc << 1) & 0xFFFF;
        if (bit) crc ^= 0x1021;
    }
    // XOR with RNTI
    crc ^= rnti;
    return crc;
}

// ======================== Convolutional Decoder (Viterbi, rate 1/3, K=7) ========================
// Reused for PDCCH decode (same as PBCH)

#define VITERBI_STATES 64
#define VITERBI_K      7
static const uint8_t viterbi_poly[3] = { 0133, 0171, 0165 };

static inline int32_t vit_output(int32_t state, int32_t input, int32_t poly) {
    int32_t sr = (state << 1) | input;
    int32_t out = 0;
    for (int32_t i = 0; i < VITERBI_K; i++)
        out ^= ((sr >> i) & 1) & ((poly >> i) & 1);
    return out;
}

static int32_t viterbi_decode_sib(const int8_t *soft_bits, int32_t coded_len, uint8_t *out_bits)
{
    int32_t n_out = coded_len / 3;
    if (n_out <= 0 || n_out > 512) return 0;

    int32_t *pm = static_cast<int32_t*>(calloc(VITERBI_STATES, sizeof(int32_t)));
    int32_t *pm_new = static_cast<int32_t*>(calloc(VITERBI_STATES, sizeof(int32_t)));
    uint8_t *decisions = static_cast<uint8_t*>(calloc((size_t)n_out * VITERBI_STATES, 1));
    if (!pm || !pm_new || !decisions) { free(pm); free(pm_new); free(decisions); return 0; }

    // Initialize: state 0 = 0, others = -inf (zero tail assumption)
    for (int32_t i = 1; i < VITERBI_STATES; i++) pm[i] = -999999;

    for (int32_t t = 0; t < n_out; t++) {
        for (int32_t i = 0; i < VITERBI_STATES; i++) pm_new[i] = -999999;
        for (int32_t state = 0; state < VITERBI_STATES; state++) {
            if (pm[state] == -999999) continue;
            for (int32_t input = 0; input < 2; input++) {
                int32_t next_state = ((state << 1) | input) & (VITERBI_STATES - 1);
                int32_t bm = 0;
                for (int32_t g = 0; g < 3; g++) {
                    int32_t expected = vit_output(state, input, viterbi_poly[g]);
                    bm += soft_bits[t*3 + g] * (expected ? -1 : 1);
                }
                int32_t metric = pm[state] + bm;
                if (metric > pm_new[next_state]) {
                    pm_new[next_state] = metric;
                    decisions[t * VITERBI_STATES + next_state] = (uint8_t)input;
                }
            }
        }
        memcpy(pm, pm_new, VITERBI_STATES * sizeof(int32_t));
    }

    // Traceback from state 0 (zero tail)
    int32_t state = 0;
    for (int32_t t = n_out - 1; t >= 0; t--) {
        out_bits[t] = decisions[t * VITERBI_STATES + state];
        // Recover previous state
        state = (state >> 1) | (out_bits[t] << (VITERBI_K - 2));
    }

    free(pm); free(pm_new); free(decisions);
    return n_out;
}

// ======================== Turbo Decoder (Max-Log-MAP, rate 1/3) ========================
// LTE turbo code: two RSC encoders (K=4, feedback poly 13, feedforward poly 15 octal)
// with QPP interleaver

#define TC_K         4         // Constraint length
#define TC_STATES    8         // 2^(K-1)
#define TC_POLY_FB   013       // Feedback polynomial (octal) = 1011
#define TC_POLY_FF   015       // Feedforward polynomial (octal) = 1101
#define TC_MAX_ITER  6         // Max turbo iterations

// QPP interleaver: f(i) = (f1*i + f2*i^2) mod K
// Table from 3GPP 36.212 Table 5.1.3-3 (selected sizes)
typedef struct { int32_t K; int32_t f1; int32_t f2; } qpp_entry_t;
static const qpp_entry_t qpp_table[] = {
    {40,   3,  10}, {48,   7,  12}, {56,  19,  42}, {64,   7,  16},
    {72,   7,  18}, {80,  11,  20}, {88,   5,  22}, {96,  11,  24},
    {104, 7,   26}, {112, 41,  84}, {120, 103, 90}, {128, 15,  32},
    {136, 9,   34}, {144, 17, 108}, {152, 9,   38}, {160, 21, 120},
    {168, 101, 84}, {176, 21,  44}, {184, 57,  46}, {192, 23,  48},
    {200, 13,  50}, {208, 27,  52}, {216, 11,  36}, {224, 27,  56},
    {232, 85, 58},  {240, 29,  60}, {248, 33,  62}, {256, 15,  32},
    {264, 17, 198}, {272, 33,  68}, {280, 103, 210},{288, 19,  36},
    {296, 19,  74}, {304, 37,  76}, {312, 19,  78}, {320, 21, 120},
    {328, 21,  82}, {336, 115, 84},{344, 193, 86}, {352, 21,  44},
    {360, 133, 90}, {368, 81,  46}, {376, 45,  94}, {384, 23,  48},
    {392, 243, 98}, {400, 151, 40},{408, 155, 102},{416, 25, 104},
    {424, 51, 106}, {432, 47,  72}, {440, 91, 110},{448, 29, 168},
    {456, 29, 114}, {464, 247, 58},{472, 29, 118},{480, 89, 180},
    {488, 91, 122}, {496, 157, 62},{504, 55,  84},{512, 31,  64},
    {528, 17,  66}, {544, 35,  68},{560, 227, 420},{576, 65,  96},
    {592, 19,  74}, {608, 37,  76},{624, 41, 234},{640, 39,  80},
    {656, 185, 82}, {672, 43, 252},{688, 21,  86},{704, 155, 44},
    {720, 79, 120}, {736, 139, 92},{752, 23,  94},{768, 217, 48},
    {784, 25,  98}, {800, 17,  80},{816, 127, 102},{832, 25, 52},
    {848, 239, 106},{864, 17, 48}, {880, 137, 110},{896, 215, 112},
    {912, 29, 114}, {928, 15, 58}, {944, 147, 118},{960, 29, 60},
    {976, 59, 122}, {992, 65, 124},{1008, 55, 84},{1024, 31, 64},
    {1056, 17, 66}, {1088, 171, 204},{1120, 67, 140},{1152, 35, 72},
    {1184, 19, 74}, {1216, 39, 76}, {1248, 19, 78}, {1280, 199, 240},
    {1312, 21, 82}, {1344, 211, 252},{1376, 21, 86},{1408, 43, 88},
    {1440, 149, 60},{1472, 45, 92},{1504, 49, 846},{1536, 71, 48},
    {1568, 13, 28}, {1600, 17, 80},{1632, 25, 102},{1664, 183, 104},
    {1696, 55, 954},{1728, 127, 96},{1760, 27, 110},{1792, 29, 112},
    {1824, 29, 114},{1856, 57, 116},{1888, 45, 354},{1920, 31, 120},
    {1952, 59, 610},{1984, 185, 124},{2016, 113, 420},{2048, 31, 64},
    {2112, 17, 66}, {2176, 171, 136},{2240, 209, 420},{2304, 253, 216},
    {2368, 367, 444},{2432, 265, 456},{2496, 181, 468},{2560, 39, 80},
    {2624, 27, 164},{2688, 127, 504},{2752, 143, 172},{2816, 43, 88},
    {2880, 29, 300},{2944, 45, 92}, {3008, 157, 188},{3072, 47, 96},
    {3136, 13, 28}, {3200, 111, 240},{3264, 443, 204},{3328, 51, 104},
    {3392, 51, 212},{3456, 451, 192},{3520, 257, 220},{3584, 57, 336},
    {3648, 313, 228},{3712, 271, 232},{3776, 179, 236},{3840, 331, 120},
    {3904, 363, 244},{3968, 375, 248},{4032, 127, 168},{4096, 31, 64},
    {4160, 33, 130},{4224, 43, 264},{4288, 33, 134},{4352, 477, 408},
    {4416, 35, 138},{4480, 233, 280},{4544, 357, 142},{4608, 337, 480},
    {4672, 37, 146},{4736, 71, 444},{4800, 71, 120},{4864, 37, 152},
    {4928, 39, 462},{4992, 127, 234},{5056, 39, 158},{5120, 39, 80},
    {5184, 31, 96}, {5248, 113, 902},{5312, 41, 166},{5376, 251, 336},
    {5440, 43, 170},{5504, 21, 86}, {5568, 43, 174},{5632, 45, 176},
    {5696, 45, 178},{5760, 161, 120},{5824, 89, 182},{5888, 323, 184},
    {5952, 47, 186},{6016, 23, 94}, {6080, 47, 190},{6144, 263, 480},
    {0, 0, 0} // sentinel
};

static bool qpp_interleave(int32_t K, int32_t *perm)
{
    int32_t f1 = 0, f2 = 0;
    for (int32_t i = 0; qpp_table[i].K; i++) {
        if (qpp_table[i].K == K) { f1 = qpp_table[i].f1; f2 = qpp_table[i].f2; break; }
    }
    if (f1 == 0) return false;
    for (int32_t i = 0; i < K; i++)
        perm[i] = ((int32_t)((int64_t)f1 * i + (int64_t)f2 * i * i)) % K;
    return true;
}

// Simplified turbo decoder (max-log-MAP, hard decision output)
static bool turbo_decode(const int8_t *systematic, const int8_t *parity0,
                         const int8_t *parity1, int32_t K, uint8_t *out_bits)
{
    // For a simplified decoder: use extrinsic information exchange
    // between two SISO decoders (MAP for RSC with K=4)
    // This is a hard-decision approximation for moderate SNR

    int32_t *perm = static_cast<int32_t*>(malloc(K * sizeof(int32_t)));
    int32_t *inv_perm = static_cast<int32_t*>(malloc(K * sizeof(int32_t)));
    float *ext1 = static_cast<float*>(calloc(K, sizeof(float)));
    float *ext2 = static_cast<float*>(calloc(K, sizeof(float)));
    float *llr_out = static_cast<float*>(calloc(K, sizeof(float)));
    if (!perm || !inv_perm || !ext1 || !ext2 || !llr_out) {
        free(perm); free(inv_perm); free(ext1); free(ext2); free(llr_out);
        return false;
    }

    if (!qpp_interleave(K, perm)) {
        // Fall back: try nearest smaller size
        free(perm); free(inv_perm); free(ext1); free(ext2); free(llr_out);
        return false;
    }
    for (int32_t i = 0; i < K; i++) inv_perm[perm[i]] = i;

    // Iterative decoding (simplified: forward-backward on trellis)
    for (int32_t iter = 0; iter < TC_MAX_ITER; iter++) {
        // ---- Decoder 1 (systematic + parity0 + extrinsic from dec2) ----
        // Forward metrics (alpha)
        float alpha[TC_STATES];
        float alpha_next[TC_STATES];
        for (int32_t s = 1; s < TC_STATES; s++) alpha[s] = -1e9f;
        alpha[0] = 0;

        float *gamma1 = static_cast<float*>(malloc(K * TC_STATES * 2 * sizeof(float)));
        float *alpha_store = static_cast<float*>(malloc((K + 1) * TC_STATES * sizeof(float)));
        if (!gamma1 || !alpha_store) { free(gamma1); free(alpha_store); goto cleanup; }
        memcpy(alpha_store, alpha, TC_STATES * sizeof(float));

        for (int32_t t = 0; t < K; t++) {
            float sys = (float)systematic[t] + ext2[t]; // a-priori + extrinsic
            float par = (float)parity0[t];

            for (int32_t s = 0; s < TC_STATES; s++) alpha_next[s] = -1e9f;
            for (int32_t s = 0; s < TC_STATES; s++) {
                if (alpha[s] < -1e8f) continue;
                for (int32_t b = 0; b < 2; b++) {
                    // RSC encoder: fb = s[2]^s[0]^input, next = {input, s[0], s[1]}
                    int32_t fb = ((s >> 2) ^ s ^ b) & 1;
                    int32_t next_s = (fb << 2) | (s >> 1);
                    // Parity output: ff = s[2]^s[1]^input
                    int32_t par_bit = ((s >> 2) ^ (s >> 1) ^ b) & 1;
                    float g = (b ? sys : -sys) + (par_bit ? par : -par);
                    gamma1[(t * TC_STATES + s) * 2 + b] = g;
                    float m = alpha[s] + g;
                    if (m > alpha_next[next_s]) alpha_next[next_s] = m;
                }
            }
            memcpy(alpha, alpha_next, TC_STATES * sizeof(float));
            memcpy(alpha_store + (t + 1) * TC_STATES, alpha, TC_STATES * sizeof(float));
        }

        // Backward + LLR computation
        float beta[TC_STATES];
        for (int32_t s = 1; s < TC_STATES; s++) beta[s] = -1e9f;
        beta[0] = 0;

        for (int32_t t = K - 1; t >= 0; t--) {
            float llr0 = -1e9f, llr1 = -1e9f;
            float beta_new[TC_STATES];
            for (int32_t s = 0; s < TC_STATES; s++) beta_new[s] = -1e9f;

            for (int32_t s = 0; s < TC_STATES; s++) {
                for (int32_t b = 0; b < 2; b++) {
                    int32_t fb = ((s >> 2) ^ s ^ b) & 1;
                    int32_t next_s = (fb << 2) | (s >> 1);
                    float m = alpha_store[t * TC_STATES + s]
                            + gamma1[(t * TC_STATES + s) * 2 + b]
                            + beta[next_s];
                    if (b == 0) { if (m > llr0) llr0 = m; }
                    else        { if (m > llr1) llr1 = m; }
                    // backward
                    float bm = gamma1[(t * TC_STATES + s) * 2 + b] + beta[next_s];
                    if (bm > beta_new[s]) beta_new[s] = bm;
                }
            }
            memcpy(beta, beta_new, TC_STATES * sizeof(float));
            ext1[t] = (llr1 - llr0) - (float)systematic[t] - ext2[t];
        }
        free(gamma1); free(alpha_store);

        // ---- Decoder 2 (interleaved systematic + parity1 + extrinsic from dec1) ----
        float *alpha2 = static_cast<float*>(malloc(TC_STATES * sizeof(float)));
        float *alpha2_store = static_cast<float*>(malloc((K + 1) * TC_STATES * sizeof(float)));
        float *gamma2 = static_cast<float*>(malloc(K * TC_STATES * 2 * sizeof(float)));
        if (!alpha2 || !alpha2_store || !gamma2) {
            free(alpha2); free(alpha2_store); free(gamma2); goto cleanup;
        }

        for (int32_t s = 1; s < TC_STATES; s++) alpha2[s] = -1e9f;
        alpha2[0] = 0;
        memcpy(alpha2_store, alpha2, TC_STATES * sizeof(float));

        for (int32_t t = 0; t < K; t++) {
            float sys = (float)systematic[perm[t]] + ext1[perm[t]];
            float par = (float)parity1[t];

            float alpha2_next[TC_STATES];
            for (int32_t s = 0; s < TC_STATES; s++) alpha2_next[s] = -1e9f;
            for (int32_t s = 0; s < TC_STATES; s++) {
                if (alpha2[s] < -1e8f) continue;
                for (int32_t b = 0; b < 2; b++) {
                    int32_t fb = ((s >> 2) ^ s ^ b) & 1;
                    int32_t next_s = (fb << 2) | (s >> 1);
                    int32_t par_bit = ((s >> 2) ^ (s >> 1) ^ b) & 1;
                    float g = (b ? sys : -sys) + (par_bit ? par : -par);
                    gamma2[(t * TC_STATES + s) * 2 + b] = g;
                    float m = alpha2[s] + g;
                    if (m > alpha2_next[next_s]) alpha2_next[next_s] = m;
                }
            }
            memcpy(alpha2, alpha2_next, TC_STATES * sizeof(float));
            memcpy(alpha2_store + (t + 1) * TC_STATES, alpha2, TC_STATES * sizeof(float));
        }

        // Backward for decoder 2
        float beta2[TC_STATES];
        for (int32_t s = 1; s < TC_STATES; s++) beta2[s] = -1e9f;
        beta2[0] = 0;

        for (int32_t t = K - 1; t >= 0; t--) {
            float llr0 = -1e9f, llr1 = -1e9f;
            float beta2_new[TC_STATES];
            for (int32_t s = 0; s < TC_STATES; s++) beta2_new[s] = -1e9f;

            for (int32_t s = 0; s < TC_STATES; s++) {
                for (int32_t b = 0; b < 2; b++) {
                    int32_t fb = ((s >> 2) ^ s ^ b) & 1;
                    int32_t next_s = (fb << 2) | (s >> 1);
                    float m = alpha2_store[t * TC_STATES + s]
                            + gamma2[(t * TC_STATES + s) * 2 + b]
                            + beta2[next_s];
                    if (b == 0) { if (m > llr0) llr0 = m; }
                    else        { if (m > llr1) llr1 = m; }
                    float bm = gamma2[(t * TC_STATES + s) * 2 + b] + beta2[next_s];
                    if (bm > beta2_new[s]) beta2_new[s] = bm;
                }
            }
            memcpy(beta2, beta2_new, TC_STATES * sizeof(float));
            // De-interleave extrinsic for decoder 1
            ext2[perm[t]] = (llr1 - llr0) - (float)systematic[perm[t]] - ext1[perm[t]];
        }
        free(alpha2); free(alpha2_store); free(gamma2);

        // Compute final LLR
        for (int32_t t = 0; t < K; t++)
            llr_out[t] = (float)systematic[t] + ext1[t] + ext2[t];
    }

    // Hard decision
    for (int32_t t = 0; t < K; t++)
        out_bits[t] = (llr_out[t] > 0) ? 1 : 0;

    free(perm); free(inv_perm); free(ext1); free(ext2); free(llr_out);
    return true;

cleanup:
    free(perm); free(inv_perm); free(ext1); free(ext2); free(llr_out);
    return false;
}

// ======================== Gold Sequence (for PDCCH/PDSCH scrambling) ========================

static void gold_seq(uint32_t c_init, int32_t offset, int32_t len, uint8_t *seq)
{
    uint32_t x1 = 1u;
    uint32_t x2 = c_init;
    for (int32_t n = 0; n < 1600 + offset; n++) {
        uint32_t new1 = ((x1 >> 3) ^ x1) & 1;
        x1 = (x1 >> 1) | (new1 << 30);
        uint32_t new2 = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1;
        x2 = (x2 >> 1) | (new2 << 30);
    }
    for (int32_t n = 0; n < len; n++) {
        seq[n] = (uint8_t)(((x1 >> 0) ^ (x2 >> 0)) & 1);
        uint32_t new1 = ((x1 >> 3) ^ x1) & 1;
        x1 = (x1 >> 1) | (new1 << 30);
        uint32_t new2 = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1;
        x2 = (x2 >> 1) | (new2 << 30);
    }
}

// ======================== FFT (shared with lte_decode.c) ========================

static void fft_dit_sib(cf_t *x, const cf_t *twiddle, int32_t n)
{
    for (int32_t i = 1, j = 0; i < n; i++) {
        int32_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { cf_t tmp = x[i]; x[i] = x[j]; x[j] = tmp; }
    }
    for (int32_t len = 2; len <= n; len <<= 1) {
        int32_t half = len >> 1;
        int32_t step = n / len;
        for (int32_t i = 0; i < n; i += len) {
            for (int32_t j = 0; j < half; j++) {
                cf_t w = twiddle[j * step];
                cf_t u = x[i + j];
                cf_t v = cf_mul(x[i + j + half], w);
                x[i + j]        = (cf_t){u.re + v.re, u.im + v.im};
                x[i + j + half] = (cf_t){u.re - v.re, u.im - v.im};
            }
        }
    }
}

// ======================== PDCCH Decode ========================

// DCI format 1A size for N_RB_DL PRBs
static int32_t dci_1a_size(int32_t n_rb_dl)
{
    // Format 1A: 1(flag) + ceil(log2(N_RB*(N_RB+1)/2))(RIV) + 5(MCS) + 3(HARQ) +
    //            1(new_data) + 2(RV) + 2(TPC) = depends on BW
    int32_t riv_bits = 0;
    int32_t n = n_rb_dl * (n_rb_dl + 1) / 2;
    while (n > 0) { riv_bits++; n >>= 1; }
    return 1 + riv_bits + 5 + 3 + 1 + 2 + 2; // + padding to format 0 size if needed
}

// DCI format 1C size for N_RB_DL PRBs
static int32_t dci_1c_size(int32_t n_rb_dl)
{
    // Format 1C: gap_indicator(if N_RB>50) + ceil(log2(N'_RB*(N'_RB+1)/2))(RBA) + 5(MCS)
    int32_t n_step = (n_rb_dl >= 50) ? 4 : ((n_rb_dl >= 25) ? 2 : 1);
    int32_t n_vrb = n_rb_dl / n_step;
    int32_t rba_bits = 0;
    int32_t n = n_vrb * (n_vrb + 1) / 2;
    while (n > 0) { rba_bits++; n >>= 1; }
    int32_t gap = (n_rb_dl > 50) ? 1 : 0;
    return gap + rba_bits + 5;
}

// TBS table for DCI format 1C (3GPP 36.213 Table 7.1.7.2.1-1)
static const int32_t tbs_table_1c[] = {
    40, 56, 72, 120, 136, 144, 176, 208, 224, 256, 280, 296, 328, 336, 392,
    408, 456, 480, 504, 528, 552, 584, 616, 648, 680, 712, 744, 776, 808, 840,
    872, 936
};

// Attempt PDCCH blind decode for SI-RNTI
bool lte_decode_pdcch_si(const float *iq, int32_t iq_len, int32_t subframe_start,
                         int32_t n_rb_dl, int32_t pci, float freq_offset_hz,
                         const void *fft_twiddle, lte_dci_t *dci)
{
    if (!iq || !dci || !fft_twiddle) return false;
    memset(dci, 0, sizeof(*dci));

    const cf_t *twiddle = (const cf_t *)fft_twiddle;

    // For 6-RB cell: PDCCH occupies first 1-3 OFDM symbols (from PCFICH)
    // We try all: 1, 2, 3 symbols
    // Control region = 72 subcarriers per symbol (center 6 RBs)
    // REGs: 4 REs per REG, excluding CRS positions

    int32_t fft_size = 128;
    float dp = 2.0f * (float)M_PI * freq_offset_hz / (float)LTE_SAMPLE_RATE;
    float step_re = cosf(dp), step_im = sinf(dp);

    // Extract OFDM symbols of the control region (up to 3 symbols)
    cf_t ctrl_syms[3][128];
    int32_t cp_lens[] = {10, 9, 9}; // CP for symbols 0, 1, 2 at 1.92 MS/s

    int32_t pos = subframe_start;
    for (int32_t sym = 0; sym < 3; sym++) {
        pos += cp_lens[sym]; // skip CP
        if (pos + fft_size > iq_len) return false;
        float osc_re = cosf(dp * pos);
        float osc_im = sinf(dp * pos);
        for (int32_t i = 0; i < fft_size; i++) {
            float I = iq[(pos + i) * 2];
            float Q = iq[(pos + i) * 2 + 1];
            ctrl_syms[sym][i].re = I * osc_re + Q * osc_im;
            ctrl_syms[sym][i].im = Q * osc_re - I * osc_im;
            float nr = osc_re * step_re - osc_im * step_im;
            osc_im = osc_re * step_im + osc_im * step_re;
            osc_re = nr;
        }
        fft_dit_sib(ctrl_syms[sym], twiddle, fft_size);
        pos += fft_size;
    }

    // Extract PDCCH candidates (common search space: aggregation level 4 and 8)
    // For 6-RB cell: max 2 CCEs available in 1 symbol, up to 6 CCEs in 3 symbols
    // CCE = 9 REGs = 36 REs = 72 QPSK bits
    // Aggregation level 4: 4 CCEs = 288 bits
    // Aggregation level 8: 8 CCEs = 576 bits
    // For SI-RNTI: try aggregation level 4 (common search space candidates 0-3)

    // Simplified: extract all usable REs from center 72 subcarriers
    // CRS pattern: symbol 0 → k%6==v_shift, k%6==(v_shift+3)%6
    int32_t v_shift = pci % 6;
    int32_t v_shift3 = (v_shift + 3) % 6;
    int8_t soft_ctrl[576]; // max for agg 8
    int32_t soft_idx = 0;

    for (int32_t sym = 0; sym < 3; sym++) {
        for (int32_t k = 0; k < 72; k++) {
            // CRS exclusion on symbols 0 and 1
            if (sym < 2) {
                int32_t kmod6 = k % 6;
                if (kmod6 == v_shift || kmod6 == v_shift3) continue;
            }
            int32_t bin = (k < 36) ? (fft_size - 36 + k) : (k - 35);
            float re = ctrl_syms[sym][bin].re;
            float im = ctrl_syms[sym][bin].im;
            // QPSK demod (soft)
            int32_t vr = (int32_t)(re * 8.0f);
            int32_t vi = (int32_t)(im * 8.0f);
            if (vr > 127) vr = 127;
            if (vr < -127) vr = -127;
            if (vi > 127) vi = 127;
            if (vi < -127) vi = -127;
            if (soft_idx < 576) soft_ctrl[soft_idx++] = (int8_t)vr;
            if (soft_idx < 576) soft_ctrl[soft_idx++] = (int8_t)vi;
        }
    }

    // Try DCI format 1C first (used for SIB1 on PDSCH), then 1A
    int32_t dci_sizes[2] = { dci_1c_size(n_rb_dl), dci_1a_size(n_rb_dl) };
    uint8_t dci_formats[2] = { 0x1C, 0x1A };

    for (int32_t fmt = 0; fmt < 2; fmt++) {
        int32_t dci_len = dci_sizes[fmt];
        int32_t coded_len = (dci_len + 16) * 3; // +16 CRC bits, rate 1/3

        if (coded_len > soft_idx) continue;

        // Try each aggregation level candidate
        // For common search space with agg level 4: start at CCE 0, 4
        for (int32_t cand_start = 0; cand_start + coded_len <= soft_idx; cand_start += coded_len) {
            // Descramble: c_init = rnti * 2^16 + n_s/2 * 2^9 + N_cell_id
            // For PDCCH on slot n_s, in subframe sf_idx
            uint32_t c_init = (uint32_t)SI_RNTI * 65536 + 0 * 512 + (uint32_t)pci;
            uint8_t scr_seq[576];
            gold_seq(c_init, cand_start, coded_len, scr_seq);

            int8_t descr[576];
            for (int32_t i = 0; i < coded_len; i++)
                descr[i] = scr_seq[i] ? -soft_ctrl[cand_start + i] : soft_ctrl[cand_start + i];

            // Viterbi decode
            uint8_t decoded[512];
            int32_t n_dec = viterbi_decode_sib(descr, coded_len, decoded);
            if (n_dec < dci_len + 16) continue;

            // Check CRC-16 with RNTI masking
            uint16_t crc = crc16_dci(decoded, dci_len, SI_RNTI);
            uint16_t rx_crc = 0;
            for (int32_t i = 0; i < 16; i++)
                rx_crc |= (uint16_t)(decoded[dci_len + i] & 1) << (15 - i);

            if (crc != rx_crc) continue;

            // Parse DCI
            bitreader_t br;
            br_init(&br, decoded, dci_len);

            if (dci_formats[fmt] == 0x1C) {
                // DCI format 1C
                dci->format = 0x1C;
                int32_t n_step = (n_rb_dl >= 50) ? 4 : ((n_rb_dl >= 25) ? 2 : 1);
                if (n_rb_dl > 50) br_skip(&br, 1); // gap indicator
                int32_t n_vrb = n_rb_dl / n_step;
                int32_t rba_bits = 0;
                { int32_t nn = n_vrb * (n_vrb + 1) / 2; while (nn > 0) { rba_bits++; nn >>= 1; } }
                uint32_t riv = br_read(&br, rba_bits);
                uint32_t mcs_idx = br_read(&br, 5);
                // Decode RIV → L_CRBs, RB_start
                int32_t L = 0, rb_start = 0;
                for (L = 1; L <= n_vrb; L++) {
                    if (riv < (uint32_t)(n_vrb - L + 1)) { rb_start = (int32_t)riv; break; }
                    riv -= (uint32_t)(n_vrb - L + 1);
                }
                dci->n_prb = (uint8_t)(L * n_step);
                dci->prb_start = (uint8_t)(rb_start * n_step);
                dci->mcs = (uint8_t)mcs_idx;
                dci->rv = 0; // Fixed for 1C in SIB1
                if (mcs_idx < 32) dci->tbs = (uint16_t)tbs_table_1c[mcs_idx];
            } else {
                // DCI format 1A
                dci->format = 0x1A;
                br_skip(&br, 1); // format flag
                int32_t riv_bits = 0;
                { int32_t nn = n_rb_dl * (n_rb_dl + 1) / 2; while (nn > 0) { riv_bits++; nn >>= 1; } }
                uint32_t riv = br_read(&br, riv_bits);
                uint32_t mcs_idx = br_read(&br, 5);
                br_skip(&br, 3); // HARQ
                br_skip(&br, 1); // new data
                uint32_t rv = br_read(&br, 2);
                // Decode RIV
                int32_t L = 0, rb_start = 0;
                for (L = 1; L <= n_rb_dl; L++) {
                    if (riv < (uint32_t)(n_rb_dl - L + 1)) { rb_start = (int32_t)riv; break; }
                    riv -= (uint32_t)(n_rb_dl - L + 1);
                }
                dci->n_prb = (uint8_t)L;
                dci->prb_start = (uint8_t)rb_start;
                dci->mcs = (uint8_t)mcs_idx;
                dci->rv = (uint8_t)rv;
                // TBS from 36.213 table (simplified — use mcs as index)
                // For SI: typically small TBS
                dci->tbs = (uint16_t)(dci->n_prb * 12 * 2); // approx
            }
            dci->rnti = SI_RNTI;
            dci->valid = true;
            return true;
        }
    }
    return false;
}

// ======================== PDSCH Decode ========================

bool lte_decode_pdsch_sib(const float *iq, int32_t iq_len, int32_t subframe_start,
                          int32_t n_rb_dl, int32_t pci, float freq_offset_hz,
                          const void *fft_twiddle, const lte_dci_t *dci,
                          uint8_t *out_bits, int32_t *out_len)
{
    if (!iq || !dci || !dci->valid || !out_bits || !out_len) return false;
    *out_len = 0;

    const cf_t *twiddle = (const cf_t *)fft_twiddle;
    int32_t fft_size = 128;
    float dp = 2.0f * (float)M_PI * freq_offset_hz / (float)LTE_SAMPLE_RATE;
    float step_re = cosf(dp), step_im = sinf(dp);

    // PDSCH occupies symbols 3-13 of the subframe (after control region of ~3 symbols)
    // For 6-RB cell: symbols 3-6 in slot 0, symbols 0-6 in slot 1
    // Actually for n_ctrl_sym=3: data starts at symbol 3

    int32_t n_ctrl = 3; // For 6 RBs, typically 3 OFDM symbols for control
    int32_t n_data_sym = 14 - n_ctrl; // 11 data symbols per subframe

    // Extract data OFDM symbols
    int32_t max_re = n_data_sym * dci->n_prb * 12;
    int8_t *soft_data = static_cast<int8_t*>(calloc(max_re * 2, sizeof(int8_t)));
    if (!soft_data) return false;

    int32_t v_shift = pci % 6;
    int32_t v_shift3 = (v_shift + 3) % 6;
    int32_t soft_idx = 0;

    // Navigate to the start of data symbols
    int32_t pos = subframe_start;
    // Skip control symbols (symbol 0 has CP0=10, rest CP=9)
    pos += 10 + fft_size; // symbol 0
    for (int32_t s = 1; s < n_ctrl; s++)
        pos += 9 + fft_size; // symbols 1..(n_ctrl-1)

    // Process remaining symbols
    for (int32_t sym_in_sf = n_ctrl; sym_in_sf < 14 && soft_idx < max_re * 2; sym_in_sf++) {
        int32_t cp_len = 9; // Normal CP (not first symbol of slot, which we already passed)
        // First symbol of slot 1 (sym_in_sf == 7) has CP0=10
        if (sym_in_sf == 7) cp_len = 10;
        pos += cp_len;
        if (pos + fft_size > iq_len) break;

        // Frequency-correct and FFT
        cf_t sym_fft[128];
        float osc_re = cosf(dp * pos);
        float osc_im = sinf(dp * pos);
        for (int32_t i = 0; i < fft_size; i++) {
            float I = iq[(pos + i) * 2];
            float Q = iq[(pos + i) * 2 + 1];
            sym_fft[i].re = I * osc_re + Q * osc_im;
            sym_fft[i].im = Q * osc_re - I * osc_im;
            float nr = osc_re * step_re - osc_im * step_im;
            osc_im = osc_re * step_im + osc_im * step_re;
            osc_re = nr;
        }
        fft_dit_sib(sym_fft, twiddle, fft_size);
        pos += fft_size;

        // Extract REs for allocated PRBs
        for (int32_t prb = dci->prb_start; prb < dci->prb_start + dci->n_prb && prb < n_rb_dl; prb++) {
            for (int32_t sc = 0; sc < 12; sc++) {
                int32_t k = prb * 12 + sc; // subcarrier index within BW
                // CRS exclusion (symbols 0,4,7,11 relative to subframe)
                int32_t sym_in_slot = sym_in_sf % 7;
                if (sym_in_slot == 0 || sym_in_slot == 4) {
                    int32_t kmod6 = k % 6;
                    if (kmod6 == v_shift || kmod6 == v_shift3) continue;
                }
                // Map to FFT bin (center-aligned)
                int32_t half_bw = n_rb_dl * 6; // Half BW in subcarriers
                int32_t sc_centered = k - half_bw; // Relative to DC
                if (sc_centered == 0) continue; // skip DC
                int32_t bin = (sc_centered > 0) ? sc_centered : (fft_size + sc_centered);
                if (bin < 0 || bin >= fft_size) continue;

                float re = sym_fft[bin].re;
                float im = sym_fft[bin].im;
                int32_t vr = (int32_t)(re * 8.0f);
                int32_t vi = (int32_t)(im * 8.0f);
                if (vr > 127) vr = 127;
                if (vr < -127) vr = -127;
                if (vi > 127) vi = 127;
                if (vi < -127) vi = -127;
                if (soft_idx < max_re * 2) soft_data[soft_idx++] = (int8_t)vr;
                if (soft_idx < max_re * 2) soft_data[soft_idx++] = (int8_t)vi;
            }
        }
    }

    // Rate de-matching: coded bits → turbo decoder input
    int32_t tbs = dci->tbs;
    int32_t K = tbs + 24; // +CRC24A
    // Find valid turbo interleaver size (>= K)
    int32_t K_turbo = 0;
    for (int32_t i = 0; qpp_table[i].K; i++) {
        if (qpp_table[i].K >= K) { K_turbo = qpp_table[i].K; break; }
    }
    if (K_turbo == 0 || K_turbo > 6144) { free(soft_data); return false; }

    // Separate into systematic + 2 parity streams
    // Rate matching: circular buffer [sys(K_turbo) | p0(K_turbo) | p1(K_turbo)]
    // Total coded = 3 * K_turbo; we have soft_idx bits available
    int8_t *sys_soft = static_cast<int8_t*>(calloc(K_turbo, sizeof(int8_t)));
    int8_t *p0_soft = static_cast<int8_t*>(calloc(K_turbo, sizeof(int8_t)));
    int8_t *p1_soft = static_cast<int8_t*>(calloc(K_turbo, sizeof(int8_t)));
    if (!sys_soft || !p0_soft || !p1_soft) {
        free(soft_data); free(sys_soft); free(p0_soft); free(p1_soft);
        return false;
    }

    // Simple rate de-matching: distribute soft bits into 3 streams (circular buffer read)
    int32_t cb_size = 3 * K_turbo;
    int32_t rv_offset = 0; // rv=0 start
    for (int32_t i = 0; i < soft_idx && i < cb_size; i++) {
        int32_t cb_idx = (rv_offset + i) % cb_size;
        if (cb_idx < K_turbo)
            sys_soft[cb_idx] += soft_data[i]; // accumulate for combining
        else if (cb_idx < 2 * K_turbo)
            p0_soft[cb_idx - K_turbo] += soft_data[i];
        else
            p1_soft[cb_idx - 2 * K_turbo] += soft_data[i];
    }

    // Turbo decode
    uint8_t *decoded = static_cast<uint8_t*>(calloc(K_turbo, 1));
    if (!decoded) { free(soft_data); free(sys_soft); free(p0_soft); free(p1_soft); return false; }

    bool ok = turbo_decode(sys_soft, p0_soft, p1_soft, K_turbo, decoded);
    free(soft_data); free(sys_soft); free(p0_soft); free(p1_soft);

    if (!ok) { free(decoded); return false; }

    // CRC-24A check
    uint32_t crc = crc24a(decoded, tbs);
    uint32_t rx_crc = 0;
    for (int32_t i = 0; i < 24; i++)
        rx_crc |= (uint32_t)(decoded[tbs + i] & 1) << (23 - i);

    if (crc != rx_crc) {
        free(decoded);
        return false;
    }

    // Output transport block bits
    memcpy(out_bits, decoded, tbs);
    *out_len = tbs;
    free(decoded);
    return true;
}

// ======================== ASN.1 UPER Parsers ========================

// Parse PLMN-Identity (MCC + MNC)
static bool parse_plmn(bitreader_t *br, uint16_t *mcc, uint16_t *mnc)
{
    // MCC: SEQUENCE (SIZE 3) OF INTEGER (0..9)
    *mcc = 0;
    for (int32_t i = 0; i < 3; i++)
        *mcc = *mcc * 10 + (uint16_t)br_read(br, 4);

    // MNC: SEQUENCE (SIZE 2..3) OF INTEGER (0..9)
    int32_t mnc_len = br_read_bool(br) ? 3 : 2; // length determinant
    *mnc = 0;
    for (int32_t i = 0; i < mnc_len; i++)
        *mnc = *mnc * 10 + (uint16_t)br_read(br, 4);

    return br_remaining(br) >= 0;
}

// Parse SystemInformationBlockType1
bool lte_parse_sib1(const uint8_t *bits, int32_t nbits, lte_sib1_info_t *sib1)
{
    if (!bits || nbits < 40 || !sib1) return false;
    memset(sib1, 0, sizeof(*sib1));

    bitreader_t br;
    br_init(&br, bits, nbits);

    // BCCH-DL-SCH-Message ::= SEQUENCE { message BCCH-DL-SCH-MessageType }
    // BCCH-DL-SCH-MessageType ::= CHOICE { c1 CHOICE { systemInformationBlockType1 ... } }
    // Skip message type choice (1 bit for c1 vs messageClassExtension)
    br_skip(&br, 1); // c1
    // c1: CHOICE index (0 = systemInformationBlockType1, 1 = systemInformation)
    uint32_t c1_choice = br_read(&br, 1);
    if (c1_choice != 0) return false; // Not SIB1

    // SystemInformationBlockType1 ::= SEQUENCE {
    //   extension marker (1 bit)
    bool ext = br_read_bool(&br);
    (void)ext;

    // cellAccessRelatedInfo SEQUENCE {
    //   plmn-IdentityList SEQUENCE (SIZE 1..6) OF PLMN-IdentityInfo
    int32_t plmn_count = (int32_t)br_read_constrained(&br, 1, 6);
    sib1->plmn_count = plmn_count;
    for (int32_t i = 0; i < plmn_count && i < 6; i++) {
        parse_plmn(&br, &sib1->plmn[i].mcc, &sib1->plmn[i].mnc);
        // cellReservedForOperatorUse ENUMERATED {reserved, notReserved}
        sib1->plmn[i].cell_reserved = br_read_bool(&br);
    }

    //   trackingAreaCode BIT STRING (SIZE 16)
    sib1->tac = (uint16_t)br_read(&br, 16);

    //   cellIdentity BIT STRING (SIZE 28)
    sib1->cell_id = br_read(&br, 28);

    //   cellBarred ENUMERATED {barred, notBarred}
    sib1->cell_barred = br_read_bool(&br);

    //   intraFreqReselection ENUMERATED {allowed, notAllowed}
    sib1->intra_freq_resel = br_read_bool(&br);

    //   csg-Indication BOOLEAN
    br_skip(&br, 1);

    // } -- end cellAccessRelatedInfo

    // cellSelectionInfo SEQUENCE {
    //   q-RxLevMin INTEGER (-70..-22)
    sib1->q_rxlevmin = (int8_t)br_read_constrained(&br, 0, 48) - 70;

    //   q-RxLevMinOffset INTEGER (1..8) OPTIONAL
    if (br_read_bool(&br)) br_skip(&br, 3); // optional present
    // }

    // p-Max INTEGER (-30..33) OPTIONAL
    if (br_read_bool(&br)) br_skip(&br, 6);

    // freqBandIndicator INTEGER (1..64)
    sib1->freq_band_indicator = (uint8_t)br_read_constrained(&br, 1, 64);

    // schedulingInfoList SEQUENCE (SIZE 1..maxSI-Message) OF SchedulingInfo
    int32_t si_count = (int32_t)br_read_constrained(&br, 1, LTE_MAX_SI_MSG);
    sib1->si_sched_count = si_count;
    for (int32_t i = 0; i < si_count && i < LTE_MAX_SI_MSG; i++) {
        // si-Periodicity ENUMERATED {rf8,rf16,rf32,rf64,rf128,rf256,rf512}
        sib1->si_sched[i].si_periodicity = (uint8_t)br_read(&br, 3);
        // sib-MappingInfo SEQUENCE (SIZE 0..maxSIB-1) OF SIB-Type
        int32_t sib_cnt = (int32_t)br_read_constrained(&br, 0, 31);
        sib1->si_sched[i].sib_count = sib_cnt;
        for (int32_t j = 0; j < sib_cnt && j < 8; j++) {
            // SIB-Type ENUMERATED {sibType3, sibType4, ..., sibType14, ...}
            sib1->si_sched[i].sib_mapping[j] = (uint8_t)br_read(&br, 5) + 3;
        }
    }

    // si-WindowLength ENUMERATED {ms1,ms2,ms5,ms10,ms15,ms20,ms40}
    static const uint8_t si_win_vals[] = {1, 2, 5, 10, 15, 20, 40};
    uint32_t win_idx = br_read(&br, 3);
    sib1->si_window_length = (win_idx < 7) ? si_win_vals[win_idx] : 40;

    sib1->valid = (br_remaining(&br) >= 0);
    return sib1->valid;
}

// Parse SystemInformationBlockType2 (common RACH/UL configuration)
static bool parse_sib2(bitreader_t *br, lte_sib2_t *sib2)
{
    memset(sib2, 0, sizeof(*sib2));

    bool ext = br_read_bool(br);
    (void)ext;

    // ac-BarringInfo OPTIONAL
    if (br_read_bool(br)) {
        bool mo_sig_present = br_read_bool(br);
        bool mo_data_present = br_read_bool(br);
        br_skip(br, 1); // ac-BarringForEmergency
        if (mo_sig_present) br_skip(br, 4 + 3 + 5);
        if (mo_data_present) br_skip(br, 4 + 3 + 5);
    }

    // radioResourceConfigCommon
    br_read_bool(br); // extension marker

    // rach-ConfigCommon
    uint32_t ra_preambles_idx = br_read(br, 4);
    sib2->ra_preambles = sib2_ra_preambles_to_value(ra_preambles_idx);

    // preamblesGroupAConfig OPTIONAL
    if (br_read_bool(br))
        br_skip(br, 4 + 4 + 3 + 4);

    sib2->power_ramping_step_db = sib2_power_ramping_to_db(br_read(br, 2));
    sib2->preamble_target_dbm = (int8_t)(-120 + 2 * (int)br_read(br, 4));
    sib2->preamble_trans_max = sib2_preamble_trans_max_to_value(br_read(br, 4));
    sib2->ra_response_window_sf = sib2_ra_window_to_sf(br_read(br, 3));
    br_skip(br, 3); // mac-ContentionResolutionTimer
    sib2->max_harq_msg3_tx = (uint8_t)br_read_constrained(br, 1, 8);

    // Skip the rest of RadioResourceConfigCommonSIB conservatively.
    br_skip(br, 2);          // BCCH-Config.modificationPeriodCoeff
    br_skip(br, 2 + 3);      // PCCH-Config
    br_skip(br, 10 + 6 + 1 + 4 + 7); // PRACH-ConfigSIB
    br_skip(br, 7 + 2);      // PDSCH-ConfigCommon
    br_skip(br, 2 + 1 + 7 + 1 + 1 + 5 + 1 + 3); // PUSCH-ConfigCommon
    br_skip(br, 2 + 7 + 3 + 11); // PUCCH-ConfigCommon
    if (br_read_bool(br)) {  // SoundingRS-UL-ConfigCommon = release/setup
        br_read_bool(br);    // extension marker in setup
        br_skip(br, 3 + 4 + 1 + 1);
    }
    br_skip(br, 7 + 3 + 5 + 2 + 2 + 1 + 2 + 2); // UplinkPowerControlCommon
    br_skip(br, 1);          // ul-CyclicPrefixLength
    if (br_read_bool(br)) br_skip(br, 2); // antennaInfoCommon OPTIONAL
    if (br_read_bool(br)) br_skip(br, 6); // p-Max OPTIONAL
    if (br_read_bool(br)) br_skip(br, 3 + 4); // tdd-Config OPTIONAL

    // ue-TimersAndConstants
    br_skip(br, 3 + 3 + 3 + 3 + 3 + 3);

    // freqInfo
    sib2->ul_carrier_freq_present = br_read_bool(br);
    sib2->ul_bandwidth_present = br_read_bool(br);
    if (sib2->ul_carrier_freq_present)
        sib2->ul_carrier_freq = (uint16_t)br_read(br, 16);
    if (sib2->ul_bandwidth_present)
        sib2->ul_bandwidth_rb = sib2_ul_bw_to_rb(br_read(br, 3));
    br_skip(br, 5); // additionalSpectrumEmission

    sib2->time_alignment_timer_sf = sib2_time_align_to_sf(br_read(br, 3));
    sib2->valid = (br_remaining(br) >= 0);
    return sib2->valid;
}

// Parse SystemInformationBlockType3 (cell reselection parameters)
static bool parse_sib3(bitreader_t *br, lte_sib3_t *sib3)
{
    memset(sib3, 0, sizeof(*sib3));

    bool ext = br_read_bool(br);
    (void)ext;

    sib3->q_hyst_db = sib3_q_hyst_to_db(br_read(br, 4));

    sib3->s_non_intra_search_present = br_read_bool(br);
    if (sib3->s_non_intra_search_present)
        sib3->s_non_intra_search = (uint8_t)br_read(br, 5);
    sib3->thresh_serving_low = (uint8_t)br_read(br, 5);
    sib3->cell_reselection_priority = (uint8_t)br_read(br, 3);

    bool p_max_present = br_read_bool(br);
    sib3->s_intra_search_present = br_read_bool(br);
    bool allowed_meas_bw_present = br_read_bool(br);
    bool t_reselection_sf_present = br_read_bool(br);

    sib3->q_rxlevmin = (int8_t)br_read_constrained(br, 0, 48) - 70;
    if (p_max_present) br_skip(br, 6);
    if (sib3->s_intra_search_present)
        sib3->s_intra_search = (uint8_t)br_read(br, 5);
    if (allowed_meas_bw_present) br_skip(br, 3);
    br_skip(br, 1 + 2); // presenceAntennaPort1 + neighCellConfig
    sib3->t_reselection_eutra = (uint8_t)br_read(br, 3);
    if (t_reselection_sf_present) br_skip(br, 6);

    sib3->valid = (br_remaining(br) >= 0);
    return sib3->valid;
}

// Parse SystemInformationBlockType4 (intra-freq neighbors)
static bool parse_sib4(bitreader_t *br, lte_sib4_t *sib4)
{
    memset(sib4, 0, sizeof(*sib4));

    // Extension marker
    bool ext = br_read_bool(br);
    (void)ext;

    // intraFreqNeighCellList SEQUENCE (SIZE 1..maxCellIntra) OPTIONAL
    if (br_read_bool(br)) {
        int32_t count = (int32_t)br_read_constrained(br, 1, LTE_MAX_NEIGH_CELLS);
        sib4->count = (count > LTE_MAX_NEIGH_CELLS) ? LTE_MAX_NEIGH_CELLS : count;
        for (int32_t i = 0; i < count; i++) {
            // physCellId INTEGER (0..503)
            uint16_t pci = (uint16_t)br_read_constrained(br, 0, 503);
            // q-OffsetCell ENUMERATED (-24..24 in steps, mapped to index 0..30)
            int8_t q_off = (int8_t)br_read(br, 5) - 15; // centered
            if (i < LTE_MAX_NEIGH_CELLS) {
                sib4->cells[i].pci = pci;
                sib4->cells[i].q_offset = q_off;
            }
        }
    }

    // intraFreqBlackCellList OPTIONAL — skip
    if (br_read_bool(br)) {
        int32_t count = (int32_t)br_read_constrained(br, 1, 16);
        for (int32_t i = 0; i < count; i++) {
            br_skip(br, 9 + 3); // PhysCellIdRange: start(9) + range(3)
        }
    }

    // csg-PhysCellIdRange OPTIONAL — skip
    if (br_read_bool(br)) br_skip(br, 9 + 3);

    sib4->valid = (br_remaining(br) >= 0);
    return sib4->valid;
}

// Parse SystemInformationBlockType5 (inter-freq carriers)
static bool parse_sib5(bitreader_t *br, lte_sib5_t *sib5)
{
    memset(sib5, 0, sizeof(*sib5));

    bool ext = br_read_bool(br);
    (void)ext;

    // interFreqCarrierFreqList SEQUENCE (SIZE 1..maxFreq)
    int32_t count = (int32_t)br_read_constrained(br, 1, LTE_MAX_EARFCN_LIST);
    sib5->count = (count > LTE_MAX_EARFCN_LIST) ? LTE_MAX_EARFCN_LIST : count;

    for (int32_t i = 0; i < count; i++) {
        // Extension marker for InterFreqCarrierFreqInfo
        br_read_bool(br);

        // dl-CarrierFreq ARFCN-ValueEUTRA INTEGER (0..65535)
        uint32_t earfcn = br_read(br, 16);
        // q-RxLevMin INTEGER (-70..-22)
        int8_t q_rxlevmin = (int8_t)br_read_constrained(br, 0, 48) - 70;
        // p-Max INTEGER (-30..33) OPTIONAL
        if (br_read_bool(br)) br_skip(br, 6);
        // t-ReselectionEUTRA INTEGER (0..7)
        br_skip(br, 3);
        // t-ReselectionEUTRA-SF OPTIONAL
        if (br_read_bool(br)) br_skip(br, 6); // SpeedStateScaleFactors
        // threshX-High INTEGER (0..31)
        uint8_t thresh_high = (uint8_t)br_read(br, 5);
        // threshX-Low INTEGER (0..31)
        uint8_t thresh_low = (uint8_t)br_read(br, 5);
        // allowedMeasBandwidth ENUMERATED {mbw6,mbw15,mbw25,mbw50,mbw75,mbw100}
        uint8_t bw = (uint8_t)br_read(br, 3);
        // presenceAntennaPort1 BOOLEAN
        br_skip(br, 1);
        // cellReselectionPriority INTEGER (0..7) OPTIONAL
        uint8_t priority = 0;
        if (br_read_bool(br)) priority = (uint8_t)br_read(br, 3);
        // neighCellConfig BIT STRING (SIZE 2)
        br_skip(br, 2);
        // q-OffsetFreq ENUMERATED (default 0) OPTIONAL — skip if present

        if (i < LTE_MAX_EARFCN_LIST) {
            sib5->carriers[i].earfcn = earfcn;
            sib5->carriers[i].q_rxlevmin = q_rxlevmin;
            sib5->carriers[i].priority = priority;
            sib5->carriers[i].thresh_x_high = thresh_high;
            sib5->carriers[i].thresh_x_low = thresh_low;
            sib5->carriers[i].bandwidth = bw;
        }
    }

    sib5->valid = (br_remaining(br) >= 0);
    return sib5->valid;
}

// Parse SystemInformationBlockType6 (UTRA neighbors)
static bool parse_sib6(bitreader_t *br, lte_sib6_t *sib6)
{
    memset(sib6, 0, sizeof(*sib6));

    bool ext = br_read_bool(br);
    (void)ext;

    // carrierFreqListUTRA-FDD SEQUENCE (SIZE 1..maxUTRA-FDD-Carrier) OPTIONAL
    if (br_read_bool(br)) {
        int32_t count = (int32_t)br_read_constrained(br, 1, LTE_MAX_UTRA_FREQ);
        sib6->count = (count > LTE_MAX_UTRA_FREQ) ? LTE_MAX_UTRA_FREQ : count;
        for (int32_t i = 0; i < count; i++) {
            // Extension marker
            br_read_bool(br);
            // carrierFreq ARFCN-ValueUTRA INTEGER (0..16383)
            uint16_t arfcn = (uint16_t)br_read(br, 14);
            // cellReselectionPriority INTEGER (0..7) OPTIONAL
            uint8_t priority = 0;
            if (br_read_bool(br)) priority = (uint8_t)br_read(br, 3);
            // threshX-High INTEGER (0..31)
            uint8_t thresh_high = (uint8_t)br_read(br, 5);
            // threshX-Low INTEGER (0..31)
            uint8_t thresh_low = (uint8_t)br_read(br, 5);
            // q-RxLevMin INTEGER (-60..-13)
            int8_t q_rxlevmin = (int8_t)br_read_constrained(br, 0, 47) - 60;
            // p-MaxUTRA INTEGER (-50..33)
            br_skip(br, 7);
            // q-QualMin INTEGER (-24..0)
            br_skip(br, 5);

            if (i < LTE_MAX_UTRA_FREQ) {
                sib6->carriers[i].arfcn = arfcn;
                sib6->carriers[i].q_rxlevmin = q_rxlevmin;
                sib6->carriers[i].priority = priority;
                sib6->carriers[i].thresh_x_high = thresh_high;
                sib6->carriers[i].thresh_x_low = thresh_low;
            }
        }
    }

    // carrierFreqListUTRA-TDD OPTIONAL — skip
    if (br_read_bool(br)) {
        int32_t count = (int32_t)br_read_constrained(br, 1, 8);
        for (int32_t i = 0; i < count; i++) br_skip(br, 14 + 5 + 5 + 7 + 5 + 1);
    }

    sib6->valid = (br_remaining(br) >= 0);
    return sib6->valid;
}

// Parse SystemInformationBlockType7 (GERAN neighbors)
static bool parse_sib7(bitreader_t *br, lte_sib7_t *sib7)
{
    memset(sib7, 0, sizeof(*sib7));

    bool ext = br_read_bool(br);
    (void)ext;

    // t-ReselectionGERAN INTEGER (0..7)
    br_skip(br, 3);
    // t-ReselectionGERAN-SF OPTIONAL
    if (br_read_bool(br)) br_skip(br, 6);

    // carrierFreqsInfoList SEQUENCE (SIZE 1..maxGNFG) OPTIONAL
    if (br_read_bool(br)) {
        int32_t count = (int32_t)br_read_constrained(br, 1, LTE_MAX_GERAN_FREQ);
        sib7->count = (count > LTE_MAX_GERAN_FREQ) ? LTE_MAX_GERAN_FREQ : count;
        for (int32_t i = 0; i < count; i++) {
            // Extension marker
            br_read_bool(br);
            // carrierFreqs SEQUENCE {
            //   startingARFCN ARFCN-ValueGERAN INTEGER (0..1023)
            uint16_t arfcn_start = (uint16_t)br_read(br, 10);
            //   bandIndicator ENUMERATED {dcs1800, pcs1900}
            uint8_t band_ind = (uint8_t)br_read(br, 1);
            //   followingARFCNs CHOICE { ... } — simplified, read length
            // For simplicity: assume explicitListOfARFCNs
            uint32_t following_choice = br_read(br, 2); // 0=explicit,1=equalSpaced,2=variableBitMap
            uint8_t num_arfcns = 1;
            if (following_choice == 0) {
                num_arfcns = (uint8_t)br_read_constrained(br, 0, 31);
                br_skip(br, num_arfcns * 10); // skip ARFCN list
            } else if (following_choice == 1) {
                br_skip(br, 3 + 10); // spacing + n_arfcns
            } else {
                br_skip(br, 16); // variable bitmap
            }
            // }
            // commonInfo SEQUENCE {
            //   cellReselectionPriority INTEGER (0..7) OPTIONAL
            uint8_t priority = 0;
            if (br_read_bool(br)) priority = (uint8_t)br_read(br, 3);
            //   ncc-Permitted BIT STRING (SIZE 8)
            br_skip(br, 8);
            //   q-RxLevMin INTEGER (0..45)
            br_skip(br, 6);
            //   p-MaxGERAN INTEGER (0..39) OPTIONAL
            if (br_read_bool(br)) br_skip(br, 6);
            //   threshX-High INTEGER (0..31)
            uint8_t thresh_high = (uint8_t)br_read(br, 5);
            //   threshX-Low INTEGER (0..31)
            uint8_t thresh_low = (uint8_t)br_read(br, 5);
            // }

            if (i < LTE_MAX_GERAN_FREQ) {
                sib7->freq_groups[i].arfcn_start = arfcn_start;
                sib7->freq_groups[i].band_indicator = band_ind;
                sib7->freq_groups[i].num_arfcns = num_arfcns;
                sib7->freq_groups[i].priority = priority;
                sib7->freq_groups[i].thresh_x_high = thresh_high;
                sib7->freq_groups[i].thresh_x_low = thresh_low;
            }
        }
    }

    sib7->valid = (br_remaining(br) >= 0);
    return sib7->valid;
}

// Parse SystemInformationBlockType10 (ETWS Primary)
static bool parse_sib10(bitreader_t *br, lte_sib10_t *sib10)
{
    memset(sib10, 0, sizeof(*sib10));

    // messageIdentifier BIT STRING (SIZE 16)
    sib10->message_id = (uint16_t)br_read(br, 16);
    // serialNumber BIT STRING (SIZE 16)
    sib10->serial_number = (uint16_t)br_read(br, 16);
    // warningType OCTET STRING (SIZE 2)
    sib10->warning_type[0] = (uint8_t)br_read(br, 8);
    sib10->warning_type[1] = (uint8_t)br_read(br, 8);
    // warningSecurityInfo OCTET STRING (SIZE 50) OPTIONAL
    sib10->warning_security = br_read_bool(br);
    if (sib10->warning_security) br_skip(br, 50 * 8);

    sib10->valid = (br_remaining(br) >= 0);
    return sib10->valid;
}

// Parse SystemInformationBlockType11 (ETWS Secondary)
static bool parse_sib11(bitreader_t *br, lte_sib11_t *sib11)
{
    memset(sib11, 0, sizeof(*sib11));

    // messageIdentifier BIT STRING (SIZE 16)
    sib11->message_id = (uint16_t)br_read(br, 16);
    // serialNumber BIT STRING (SIZE 16)
    sib11->serial_number = (uint16_t)br_read(br, 16);
    // warningMessageSegmentType ENUMERATED {notLastSegment, lastSegment}
    sib11->warning_msg_segment_type = (uint8_t)br_read(br, 1);
    // warningMessageSegmentNumber INTEGER (0..63)
    sib11->warning_msg_segment_num = (uint8_t)br_read(br, 6);
    // warningMessageSegment OCTET STRING
    int32_t seg_len = br_read_length(br);
    if (seg_len > LTE_ETWS_MSG_SIZE) seg_len = LTE_ETWS_MSG_SIZE;
    sib11->warning_msg_len = seg_len;
    for (int32_t i = 0; i < seg_len; i++)
        sib11->warning_msg[i] = (uint8_t)br_read(br, 8);
    // dataCodingScheme OCTET STRING (SIZE 1) OPTIONAL
    if (br_read_bool(br))
        sib11->data_coding_scheme = (uint8_t)br_read(br, 8);

    sib11->valid = (br_remaining(br) >= 0);
    return sib11->valid;
}

// Parse SystemInformationBlockType12 (CMAS / EU-Alert)
static bool parse_sib12(bitreader_t *br, lte_sib12_t *sib12)
{
    memset(sib12, 0, sizeof(*sib12));

    // messageIdentifier BIT STRING (SIZE 16)
    sib12->message_id = (uint16_t)br_read(br, 16);
    // serialNumber BIT STRING (SIZE 16)
    sib12->serial_number = (uint16_t)br_read(br, 16);
    // warningMessageSegmentType ENUMERATED {notLastSegment, lastSegment}
    sib12->warning_msg_segment_type = (uint8_t)br_read(br, 1);
    // warningMessageSegmentNumber INTEGER (0..63)
    sib12->warning_msg_segment_num = (uint8_t)br_read(br, 6);
    // warningMessageSegment OCTET STRING
    int32_t seg_len = br_read_length(br);
    if (seg_len > LTE_CMAS_MSG_SIZE) seg_len = LTE_CMAS_MSG_SIZE;
    sib12->warning_msg_len = seg_len;
    for (int32_t i = 0; i < seg_len; i++)
        sib12->warning_msg[i] = (uint8_t)br_read(br, 8);
    // dataCodingScheme OCTET STRING (SIZE 1) OPTIONAL
    if (br_read_bool(br))
        sib12->data_coding_scheme = (uint8_t)br_read(br, 8);

    sib12->valid = (br_remaining(br) >= 0);
    return sib12->valid;
}

// Parse SystemInformationBlockType14 (EAB)
static bool parse_sib14(bitreader_t *br, lte_sib14_t *sib14)
{
    memset(sib14, 0, sizeof(*sib14));

    bool ext = br_read_bool(br);
    (void)ext;

    // eab-Param CHOICE { eab-Common, eab-PerPLMN-List } OPTIONAL
    if (br_read_bool(br)) {
        uint32_t choice = br_read(br, 1); // 0=common, 1=per-PLMN
        if (choice == 0) {
            // EAB-Config ::= SEQUENCE {
            //   eab-Category ENUMERATED {a, b, c}
            uint8_t cat = (uint8_t)br_read(br, 2);
            //   eab-BarringBitmap BIT STRING (SIZE 10)
            for (int32_t ac = 0; ac < 10; ac++) {
                sib14->ac_barring[ac].barred = br_read_bool(br);
                sib14->ac_barring[ac].category = cat;
            }
        }
    }

    sib14->valid = (br_remaining(br) >= 0);
    return sib14->valid;
}

// Parse SI message (contains SystemInformation → list of SIBs)
bool lte_parse_si_msg(const uint8_t *bits, int32_t nbits, lte_sib_results_t *results)
{
    if (!bits || nbits < 10 || !results) return false;

    bitreader_t br;
    br_init(&br, bits, nbits);

    // BCCH-DL-SCH-Message: message CHOICE { c1 CHOICE {
    //   systemInformationBlockType1(0), systemInformation(1) } }
    br_skip(&br, 1); // c1
    uint32_t c1_choice = br_read(&br, 1);
    if (c1_choice != 1) return false; // Not SystemInformation (should be 1)

    // SystemInformation ::= SEQUENCE { criticalExtensions CHOICE { ... } }
    // criticalExtensions: systemInformation-r8
    br_skip(&br, 1); // choice bit

    // SystemInformation-r8-IEs ::= SEQUENCE {
    //   sib-TypeAndInfo SEQUENCE (SIZE 1..maxSIB) OF CHOICE { ... }
    int32_t sib_count = (int32_t)br_read_constrained(&br, 1, 32);

    for (int32_t i = 0; i < sib_count; i++) {
        // CHOICE index: sib2(0), sib3(1), sib4(2), sib5(3), sib6(4), sib7(5),
        //              sib8(6), sib9(7), sib10(8), sib11(9), sib12-v920(10), ...
        //              sib13-v920(11), sib14-v1130(12), ...
        uint32_t sib_type_idx = br_read(&br, 5); // up to 32 choices

        switch (sib_type_idx) {
            case 0: // SIB2
                parse_sib2(&br, &results->sib2);
                break;
            case 1: // SIB3
                parse_sib3(&br, &results->sib3);
                break;
            case 2: // SIB4
                parse_sib4(&br, &results->sib4);
                break;
            case 3: // SIB5
                parse_sib5(&br, &results->sib5);
                break;
            case 4: // SIB6
                parse_sib6(&br, &results->sib6);
                break;
            case 5: // SIB7
                parse_sib7(&br, &results->sib7);
                break;
            case 8: // SIB10
                parse_sib10(&br, &results->sib10);
                break;
            case 9: // SIB11
                parse_sib11(&br, &results->sib11);
                break;
            case 10: // SIB12
                parse_sib12(&br, &results->sib12);
                break;
            case 12: // SIB14
                parse_sib14(&br, &results->sib14);
                break;
            default:
                // Unknown SIB — can't skip without knowing size
                // Return what we have so far
                return true;
        }
    }

    return true;
}

// ======================== Utility Functions ========================

const char *lte_cmas_category(uint16_t message_id)
{
    if (message_id == 4370) return "Presidential Alert";
    if (message_id >= 4371 && message_id <= 4372) return "Extreme Alert (life threat)";
    if (message_id >= 4373 && message_id <= 4378) return "Severe Alert";
    if (message_id == 4379) return "AMBER Alert (child abduction)";
    if (message_id == 4380) return "Monthly Test";
    if (message_id == 4381) return "Exercise/Drill";
    if (message_id == 4382) return "Operator-defined";
    if (message_id == 4383) return "Presidential Alert (Spanish)";
    if (message_id == 4384) return "Extreme Alert (Spanish)";
    if (message_id >= 4396 && message_id <= 4396) return "EU-Alert Level 1 (extreme)";
    if (message_id == 4397) return "EU-Alert Level 2 (severe)";
    if (message_id == 4398) return "EU-Alert Level 3 (amber)";
    if (message_id == 4399) return "EU-Alert Level 4 (public safety)";
    if (message_id >= 4400 && message_id <= 4402) return "IT-Alert (Italy)";
    if (message_id >= 4352 && message_id <= 4359) return "ETWS (earthquake/tsunami)";
    return "Unknown Alert";
}

const char *lte_etws_warning_type(uint8_t type_byte)
{
    switch (type_byte & 0x7F) {
        case 0: return "Earthquake";
        case 1: return "Tsunami";
        case 2: return "Earthquake+Tsunami";
        case 3: return "Test";
        case 4: return "Other emergency";
        default: return "Unknown";
    }
}

// Decode CBS text from GSM 7-bit default alphabet or UCS-2
int32_t lte_cbs_decode_text(const uint8_t *data, int32_t data_len, uint8_t dcs,
                        char *utf8_out, int32_t utf8_max)
{
    if (!data || !utf8_out || utf8_max < 1) return 0;
    utf8_out[0] = '\0';

    int32_t coding_group = (dcs >> 4) & 0x0F;
    int32_t charset = 0; // 0=GSM7, 1=8bit, 2=UCS2

    if (coding_group <= 3) {
        charset = 0; // GSM 7-bit default alphabet
    } else if (coding_group == 4 || coding_group == 5) {
        charset = (dcs >> 2) & 3; // bits 3-2: 00=GSM7, 01=8bit, 10=UCS2
    } else if (coding_group == 15) {
        charset = (dcs & 4) ? 1 : 0;
    }

    int32_t out_pos = 0;

    if (charset == 0) {
        // GSM 7-bit unpacking
        // Basic GSM 7-bit → ASCII mapping (simplified — handles most common chars)
        static const char gsm7_basic[] =
            "@\xa3$\xa5\xe8\xe9\xf9\xec\xf2\xc7\n\xd8\xf8\r\xc5\xe5"
            "\x00\x5f""0000000000000000"
            " !\"#\xa4%&'()*+,-./"
            "0123456789:;<=>?"
            "\xa1""ABCDEFGHIJKLMNO"
            "PQRSTUVWXYZ\xc4\xd6\xd1\xdc\xa7"
            "\xbf""abcdefghijklmno"
            "pqrstuvwxyz\xe4\xf6\xf1\xfc\xe0";

        int32_t bit_pos = 0;
        while (bit_pos + 7 <= data_len * 8 && out_pos < utf8_max - 1) {
            int32_t byte_idx = bit_pos / 8;
            int32_t bit_off = bit_pos % 8;
            int32_t ch = (data[byte_idx] >> bit_off) & 0x7F;
            if (bit_off > 1 && byte_idx + 1 < data_len)
                ch |= (data[byte_idx + 1] << (8 - bit_off)) & 0x7F;
            if (ch < 128 && gsm7_basic[ch] >= 0x20)
                utf8_out[out_pos++] = gsm7_basic[ch];
            else if (ch == 0x0A || ch == 0x0D)
                utf8_out[out_pos++] = '\n';
            else
                utf8_out[out_pos++] = '?';
            bit_pos += 7;
        }
    } else if (charset == 2) {
        // UCS-2: simple 2-byte characters → UTF-8
        for (int32_t i = 0; i + 1 < data_len && out_pos < utf8_max - 3; i += 2) {
            uint16_t ucs2 = ((uint16_t)data[i] << 8) | data[i + 1];
            if (ucs2 == 0) break;
            if (ucs2 < 0x80) {
                utf8_out[out_pos++] = (char)ucs2;
            } else if (ucs2 < 0x800) {
                utf8_out[out_pos++] = (char)(0xC0 | (ucs2 >> 6));
                utf8_out[out_pos++] = (char)(0x80 | (ucs2 & 0x3F));
            } else {
                utf8_out[out_pos++] = (char)(0xE0 | (ucs2 >> 12));
                utf8_out[out_pos++] = (char)(0x80 | ((ucs2 >> 6) & 0x3F));
                utf8_out[out_pos++] = (char)(0x80 | (ucs2 & 0x3F));
            }
        }
    } else {
        // 8-bit: copy as-is (Latin-1)
        for (int32_t i = 0; i < data_len && out_pos < utf8_max - 1; i++) {
            if (data[i] >= 0x20 && data[i] < 0x7F)
                utf8_out[out_pos++] = (char)data[i];
            else if (data[i] == 0) break;
            else utf8_out[out_pos++] = '?';
        }
    }

    utf8_out[out_pos] = '\0';
    return out_pos;
}
