// Part of dump1090-gg-light
//
// lte_decode.c: LTE cell scanner — PSS/SSS detection, MIB/SIB1 decoding.
//
// Implements a passive LTE cell scanner using RTL-SDR at 1.92 MS/s.
// Detects cells by correlating against PSS Zadoff-Chu sequences,
// identifies PCI via SSS M-sequences, and decodes broadcast information
// from PBCH (MIB) and PDSCH (SIB1) without any uplink transmission.
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#include "lte_decode.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ======================== Internal Constants ========================

// Zadoff-Chu root indices for PSS (N_ID_2 = 0, 1, 2)
static const int pss_roots[3] = { 25, 29, 34 };

// PSS correlation threshold (normalized power)
#define PSS_THRESHOLD       0.10f
#define PSS_CORR_LEN        LTE_PSS_LEN

// SSS detection threshold
#define SSS_THRESHOLD       0.5f

// PBCH CRC polynomial (CRC-16)
#define PBCH_CRC_POLY       0x11021
#define PBCH_CRC_INIT       0x0000
#define PBCH_CRC_MASK_0     0x0000
#define PBCH_CRC_MASK_1     0xFFFF
#define PBCH_CRC_MASK_2     0x5555  // Alternating for antenna port detection

// ======================== Internal Types ========================

typedef struct {
    float re;
    float im;
} cf_t;

// Per-cell tracking state
typedef struct {
    lte_cell_info_t info;
    int             frame_offset;    // Sample offset within buffer to frame start
    uint64_t        last_pss_sample; // Last PSS detection sample count
    uint64_t        last_update;     // Monotonic sample count of last update

    // PBCH accumulation (spans 4 frames for MIB)
    cf_t            pbch_symbols[240]; // 4 frames * 60 PBCH symbols each
    int             pbch_frame_idx;   // 0-3, which quarter of MIB

    bool            active;
} lte_cell_state_t;

// Band 20 hop frequency table
static const double lte_hop_freqs[LTE_HOP_FREQS] = {
    LTE_BAND20_FREQ_1,  // 796 MHz (O2)
    LTE_BAND20_FREQ_2,  // 806 MHz (Vodafone)
    LTE_BAND20_FREQ_3   // 816 MHz (Telekom)
};

// Main decoder state
struct lte_state {
    lte_config_t    cfg;
    lte_stats_t     stats;

    // PSS reference signals (frequency domain, 62 subcarriers)
    cf_t            pss_freq[3][LTE_PSS_LEN];
    // PSS reference signals (time domain, for correlation)
    cf_t            pss_time[3][LTE_FFT_SIZE];

    // SSS lookup tables (m0/m1 sequences)
    int8_t          sss_d[LTE_N_ID_1_COUNT][2][LTE_SSS_LEN]; // [n_id_1][subframe0/5][62]

    // IQ buffer (accumulate samples until we have a full frame)
    float          *iq_buf;         // interleaved I/Q float
    int             iq_buf_size;    // capacity in IQ pairs
    int             iq_buf_len;     // current fill in IQ pairs
    uint64_t        sample_counter; // total samples seen

    // Detected cells
    lte_cell_state_t cells[LTE_MAX_CELLS];
    int             cell_count;

    // Frequency hopping state
    int             hop_idx;        // Current index into lte_hop_freqs[]
    uint64_t        hop_next_sample; // Sample count when next hop should occur
    double          hop_request;    // Non-zero = caller should retune to this freq

    // FFT twiddle factors (DIT radix-2)
    cf_t            fft_twiddle[LTE_FFT_SIZE / 2];

    // Working buffers
    cf_t            work_fft[LTE_FFT_SIZE];
    cf_t            work_sym[LTE_FFT_SIZE];
    float           corr_buf[LTE_SAMPLES_PER_FRAME];
};

// ======================== DSP Helpers ========================

static inline cf_t cf_mul(cf_t a, cf_t b) {
    return (cf_t){ a.re*b.re - a.im*b.im, a.re*b.im + a.im*b.re };
}

static inline cf_t cf_conj(cf_t a) {
    return (cf_t){ a.re, -a.im };
}

static inline float cf_abs2(cf_t a) {
    return a.re*a.re + a.im*a.im;
}

static inline cf_t cf_add(cf_t a, cf_t b) {
    return (cf_t){ a.re + b.re, a.im + b.im };
}

// Simple radix-2 DIT FFT (in-place, size must be power of 2)
static void fft_dit(cf_t *x, const cf_t *twiddle, int n)
{
    // Bit-reverse permutation
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { cf_t tmp = x[i]; x[i] = x[j]; x[j] = tmp; }
    }
    // Butterfly stages
    for (int len = 2; len <= n; len <<= 1) {
        int half = len >> 1;
        int step = n / len;
        for (int i = 0; i < n; i += len) {
            for (int j = 0; j < half; j++) {
                cf_t w = twiddle[j * step];
                cf_t u = x[i + j];
                cf_t v = cf_mul(x[i + j + half], w);
                x[i + j]        = cf_add(u, v);
                x[i + j + half] = (cf_t){ u.re - v.re, u.im - v.im };
            }
        }
    }
}

// IFFT via conjugate trick: IFFT(x) = conj(FFT(conj(x))) / N
static void ifft(cf_t *x, const cf_t *twiddle, int n)
{
    for (int i = 0; i < n; i++) x[i] = cf_conj(x[i]);
    fft_dit(x, twiddle, n);
    float inv = 1.0f / (float)n;
    for (int i = 0; i < n; i++) {
        x[i].re *= inv;
        x[i].im *= -inv;
    }
}

// ======================== PSS Generation ========================

// Generate Zadoff-Chu sequence for PSS (3GPP TS 36.211 §6.11.1.1)
// d_u(n) = exp(-j*pi*u*n*(n+1)/63), n=0..30
// d_u(n) = exp(-j*pi*u*(n+1)*(n+2)/63), n=31..61 (DC gap adjustment)
static void generate_pss(cf_t *out, int root_idx)
{
    int u = pss_roots[root_idx];
    for (int n = 0; n < 62; n++) {
        int nn = (n < 31) ? n : (n + 1); // accounts for DC subcarrier gap
        double phase = -M_PI * (double)u * (double)nn * (double)(nn + 1) / 63.0;
        out[n].re = (float)cos(phase);
        out[n].im = (float)sin(phase);
    }
}

// Map PSS to time domain for correlation
static void pss_to_time(const cf_t *pss_freq, cf_t *pss_time, const cf_t *twiddle)
{
    cf_t sym[LTE_FFT_SIZE];
    memset(sym, 0, sizeof(sym));

    // Map 62 PSS subcarriers to FFT bins
    // PSS occupies subcarriers -31...-1, 1...31 (DC excluded)
    // In FFT bin terms: bins N-31...N-1 and 1...31 (where N=128)
    for (int k = 0; k < 31; k++) {
        sym[LTE_FFT_SIZE - 31 + k] = pss_freq[k];       // negative frequencies
        sym[k + 1]                 = pss_freq[k + 31];   // positive frequencies
    }

    // IFFT to get time domain
    ifft(sym, twiddle, LTE_FFT_SIZE);
    memcpy(pss_time, sym, LTE_FFT_SIZE * sizeof(cf_t));
}

// ======================== SSS Generation ========================

// SSS is based on interleaved M-sequences
// Generate SSS for all N_ID_1 values (simplified — real SSS uses scrambled M-sequences)
static void generate_sss_tables(int8_t sss_d[LTE_N_ID_1_COUNT][2][LTE_SSS_LEN], int n_id_2)
{
    // M-sequence generation using x^5 + x^2 + 1
    int8_t x_s[31], x_c[31], x_z[31];

    // Initialize x_s: [0,0,0,0,1,...]  polynomial x^5 + x^2 + 1
    memset(x_s, 0, sizeof(x_s));
    x_s[4] = 1;
    for (int i = 5; i < 31; i++)
        x_s[i] = (x_s[i-3] + x_s[i-5]) & 1;  // s(i) = s(i-3) + s(i-5)

    // Initialize x_c: [0,0,0,0,1,...]  polynomial x^5 + x^3 + 1
    memset(x_c, 0, sizeof(x_c));
    x_c[4] = 1;
    for (int i = 5; i < 31; i++)
        x_c[i] = (x_c[i-2] + x_c[i-5]) & 1;  // c(i) = c(i-2) + c(i-5)

    // Initialize x_z: [0,0,0,0,1,...]  polynomial x^5 + x^4 + x^2 + x + 1
    memset(x_z, 0, sizeof(x_z));
    x_z[4] = 1;
    for (int i = 5; i < 31; i++)
        x_z[i] = (x_z[i-1] + x_z[i-3] + x_z[i-4] + x_z[i-5]) & 1;

    // Generate SSS for each N_ID_1 using 3GPP TS 36.211 §6.11.2.1 mapping
    for (int q = 0; q < LTE_N_ID_1_COUNT; q++) {
        int qp = q / 30;
        int qq = (q + qp * (qp + 1) / 2) / 30;
        int mp = q + qq * (qq + 1) / 2;
        int m0 = mp % 31;
        int m1 = (m0 + mp / 31 + 1) % 31;

        for (int n = 0; n < 31; n++) {
            int s0 = 1 - 2 * x_s[(n + m0) % 31];
            int s1 = 1 - 2 * x_s[(n + m1) % 31];
            int c0 = 1 - 2 * x_c[(n + n_id_2) % 31];
            int c1 = 1 - 2 * x_c[(n + n_id_2 + 3) % 31];
            int z0 = 1 - 2 * x_z[(n + (m0 % 8)) % 31];
            int z1 = 1 - 2 * x_z[(n + (m1 % 8)) % 31];

            // Subframe 0: d(2n) = s0*c0, d(2n+1) = s1*c1*z0
            sss_d[q][0][2*n]     = (int8_t)(s0 * c0);
            sss_d[q][0][2*n + 1] = (int8_t)(s1 * c1 * z0);
            // Subframe 5: d(2n) = s1*c0, d(2n+1) = s0*c1*z1
            sss_d[q][1][2*n]     = (int8_t)(s1 * c0);
            sss_d[q][1][2*n + 1] = (int8_t)(s0 * c1 * z1);
        }
    }
}

// ======================== CRC-16 ========================

static uint16_t crc16_lte(const uint8_t *bits, int nbits)
{
    uint16_t crc = PBCH_CRC_INIT;
    for (int i = 0; i < nbits; i++) {
        uint16_t bit = (crc >> 15) ^ bits[i];
        crc = (crc << 1) & 0xFFFF;
        if (bit) crc ^= 0x1021;
    }
    return crc;
}

// ======================== Convolutional Decoder (Viterbi, rate 1/3, K=7) ========================

// Simplified Viterbi for PBCH (rate 1/3, K=7, generators [133,171,165] octal)
#define VITERBI_STATES 64
#define VITERBI_K      7

static const uint8_t viterbi_poly[3] = { 0155, 0117, 0127 }; // G0=133, G1=171, G2=165 (bit-reversed for LSB=delay0 convention)

// Output bits for state transition
static inline int viterbi_output(int state, int input, int poly) {
    int sr = (state << 1) | input;
    int out = 0;
    for (int i = 0; i < VITERBI_K; i++)
        out ^= ((sr >> i) & 1) & ((poly >> i) & 1);
    return out;
}

// Decode rate 1/3 convolutional code (hard decision, simplified)
static int viterbi_decode(const int8_t *soft_bits, int coded_len, uint8_t *out_bits)
{
    int n_out = coded_len / 3;
    if (n_out <= 0) return 0;

    // Path metrics
    int pm[VITERBI_STATES];
    int pm_new[VITERBI_STATES];
    uint64_t *path = calloc((size_t)n_out, sizeof(uint64_t) * VITERBI_STATES);
    if (!path) return 0;

    for (int i = 0; i < VITERBI_STATES; i++) pm[i] = 0; // tail-biting: all states equally likely

    for (int t = 0; t < n_out; t++) {
        for (int i = 0; i < VITERBI_STATES; i++) pm_new[i] = -999999;

        for (int state = 0; state < VITERBI_STATES; state++) {
            if (pm[state] == -999999) continue;
            for (int input = 0; input < 2; input++) {
                int next_state = ((state << 1) | input) & (VITERBI_STATES - 1);
                // Calculate branch metric
                // QPSK convention: bit 0 → +1, bit 1 → -1
                int bm = 0;
                for (int g = 0; g < 3; g++) {
                    int expected = viterbi_output(state, input, viterbi_poly[g]);
                    bm += soft_bits[t*3 + g] * (expected ? -1 : 1);
                }
                int metric = pm[state] + bm;
                if (metric > pm_new[next_state]) {
                    pm_new[next_state] = metric;
                    path[t * VITERBI_STATES + next_state] = (uint64_t)state | ((uint64_t)input << 32);
                }
            }
        }
        memcpy(pm, pm_new, sizeof(pm));
    }

    // Traceback from best final state
    int best_state = 0;
    for (int i = 1; i < VITERBI_STATES; i++)
        if (pm[i] > pm[best_state]) best_state = i;

    // Trace back
    int state = best_state;
    for (int t = n_out - 1; t >= 0; t--) {
        uint64_t entry = path[t * VITERBI_STATES + state];
        out_bits[t] = (uint8_t)((entry >> 32) & 1);
        state = (int)(entry & 0xFFFFFFFF);
    }

    free(path);
    return n_out;
}

// ======================== Creator / Destructor ========================

struct lte_state *lte_create(const lte_config_t *cfg)
{
    struct lte_state *st = calloc(1, sizeof(struct lte_state));
    if (!st) return NULL;

    st->cfg = *cfg;

    // Generate PSS sequences
    for (int i = 0; i < 3; i++) {
        generate_pss(st->pss_freq[i], i);
    }

    // Generate FFT twiddle factors
    for (int i = 0; i < LTE_FFT_SIZE / 2; i++) {
        double angle = -2.0 * M_PI * (double)i / (double)LTE_FFT_SIZE;
        st->fft_twiddle[i].re = (float)cos(angle);
        st->fft_twiddle[i].im = (float)sin(angle);
    }

    // Convert PSS to time domain
    for (int i = 0; i < 3; i++) {
        pss_to_time(st->pss_freq[i], st->pss_time[i], st->fft_twiddle);
    }

    // Generate SSS tables for each N_ID_2
    // We generate for N_ID_2=0 initially; re-generate when PSS detected
    generate_sss_tables(st->sss_d, 0);

    // Allocate IQ buffer (3 frames worth)
    st->iq_buf_size = LTE_SAMPLES_PER_FRAME * 3;
    st->iq_buf = calloc((size_t)st->iq_buf_size * 2, sizeof(float));
    if (!st->iq_buf) { free(st); return NULL; }

    // Initialize frequency hopping
    st->hop_idx = 0;
    st->hop_request = 0;
    if (cfg->hop_enabled) {
        // Find which hop index matches the initial frequency
        for (int i = 0; i < LTE_HOP_FREQS; i++) {
            if (fabs(cfg->center_freq - lte_hop_freqs[i]) < 1e6) {
                st->hop_idx = i;
                break;
            }
        }
        uint64_t dwell_samples = (uint64_t)(cfg->sample_rate * LTE_HOP_DWELL_MS / 1000.0);
        st->hop_next_sample = dwell_samples;
    } else {
        st->hop_next_sample = UINT64_MAX; // Never hop
    }

    fprintf(stderr, "LTE: decoder created, freq=%.3f MHz, sr=%.0f Hz, FFT=%d, hop=%s\n",
            cfg->center_freq / 1e6, cfg->sample_rate, LTE_FFT_SIZE,
            cfg->hop_enabled ? "796/806/816" : "off");
    return st;
}

void lte_destroy(struct lte_state *st)
{
    if (!st) return;
    free(st->iq_buf);
    free(st);
}

// ======================== PSS Detection ========================

// Correlate against PSS time-domain reference with frequency offset compensation.
// Tries multiple carrier frequency offset hypotheses to handle RTL-SDR crystal error.
// Returns normalized correlation power (0..~0.5), peak position, and residual phase.
#define PSS_FREQ_HYPOS    7
#define PSS_FREQ_STEP     20000.0f  // 20 kHz steps
#define PSS_FREQ_BASE    -60000.0f  // start at -60 kHz (covers ±60 kHz)

static float pss_correlate(const float *iq, int n_samples, const cf_t *pss_ref,
                           int fft_size, int *peak_offset, float *peak_phase,
                           float *freq_offset_out)
{
    float best_power = 0;
    int best_pos = 0;
    float best_ph = 0;
    int best_hyp = PSS_FREQ_HYPOS / 2; // default to center (0 Hz)

    int max_pos = n_samples - fft_size;
    if (max_pos <= 0) return 0;

    // Pre-compute oscillator step (cos/sin of phase increment) for each hypothesis
    cf_t osc_step[PSS_FREQ_HYPOS];
    for (int h = 0; h < PSS_FREQ_HYPOS; h++) {
        float f_offset = PSS_FREQ_BASE + h * PSS_FREQ_STEP;
        float dp = 2.0f * (float)M_PI * f_offset / (float)LTE_SAMPLE_RATE;
        osc_step[h].re = cosf(dp);
        osc_step[h].im = sinf(dp);
    }

    // Coarse scan: every 8th sample, all frequency hypotheses
    for (int pos = 0; pos < max_pos; pos += 8) {
        float sig_power = 0;
        for (int k = 0; k < fft_size; k++) {
            float I = iq[(pos + k) * 2];
            float Q = iq[(pos + k) * 2 + 1];
            sig_power += I*I + Q*Q;
        }
        if (sig_power <= 0) continue;

        for (int h = 0; h < PSS_FREQ_HYPOS; h++) {
            float corr_re = 0, corr_im = 0;
            float osc_re = 1.0f, osc_im = 0.0f;
            float step_re = osc_step[h].re, step_im = osc_step[h].im;
            for (int k = 0; k < fft_size; k++) {
                float I = iq[(pos + k) * 2];
                float Q = iq[(pos + k) * 2 + 1];
                // Mix: (I+jQ) * (osc_re - j*osc_im) = derotate
                float Ir = I * osc_re + Q * osc_im;
                float Qr = Q * osc_re - I * osc_im;
                corr_re += Ir * pss_ref[k].re + Qr * pss_ref[k].im;
                corr_im += Qr * pss_ref[k].re - Ir * pss_ref[k].im;
                // Advance oscillator
                float new_re = osc_re * step_re - osc_im * step_im;
                osc_im = osc_re * step_im + osc_im * step_re;
                osc_re = new_re;
            }
            float corr_power = (corr_re*corr_re + corr_im*corr_im) / sig_power;
            if (corr_power > best_power) {
                best_power = corr_power;
                best_pos = pos;
                best_ph = atan2f(corr_im, corr_re);
                best_hyp = h;
            }
        }
    }

    // Refine around best position (sample-by-sample, all hypotheses)
    int refine_start = (best_pos > 7) ? best_pos - 7 : 0;
    int refine_end = (best_pos + 15 < max_pos) ? best_pos + 15 : max_pos;
    for (int pos = refine_start; pos < refine_end; pos++) {
        float sig_power = 0;
        for (int k = 0; k < fft_size; k++) {
            float I = iq[(pos + k) * 2];
            float Q = iq[(pos + k) * 2 + 1];
            sig_power += I*I + Q*Q;
        }
        if (sig_power <= 0) continue;

        for (int h = 0; h < PSS_FREQ_HYPOS; h++) {
            float corr_re = 0, corr_im = 0;
            float osc_re = 1.0f, osc_im = 0.0f;
            float step_re = osc_step[h].re, step_im = osc_step[h].im;
            for (int k = 0; k < fft_size; k++) {
                float I = iq[(pos + k) * 2];
                float Q = iq[(pos + k) * 2 + 1];
                float Ir = I * osc_re + Q * osc_im;
                float Qr = Q * osc_re - I * osc_im;
                corr_re += Ir * pss_ref[k].re + Qr * pss_ref[k].im;
                corr_im += Qr * pss_ref[k].re - Ir * pss_ref[k].im;
                float new_re = osc_re * step_re - osc_im * step_im;
                osc_im = osc_re * step_im + osc_im * step_re;
                osc_re = new_re;
            }
            float corr_power = (corr_re*corr_re + corr_im*corr_im) / sig_power;
            if (corr_power > best_power) {
                best_power = corr_power;
                best_pos = pos;
                best_ph = atan2f(corr_im, corr_re);
                best_hyp = h;
            }
        }
    }

    *peak_offset = best_pos;
    *peak_phase = best_ph;
    // Total frequency offset = coarse hypothesis + fine residual from phase
    float coarse_hz = PSS_FREQ_BASE + best_hyp * PSS_FREQ_STEP;
    float fine_hz = best_ph * (float)LTE_SAMPLE_RATE / (2.0f * (float)M_PI * (float)fft_size);
    *freq_offset_out = coarse_hz + fine_hz;
    return best_power;
}

// ======================== SSS Detection ========================

// Extract SSS from frequency domain, equalize using PSS channel estimate, correlate
static int detect_sss(struct lte_state *st, const float *iq, int offset, int n_id_2,
                      float freq_offset_hz, float *best_corr)
{
    // SSS is symbol 5, PSS is symbol 6 (both in slot 0 for subframe 0)
    // offset = start of PSS DATA (128 samples)
    // Layout: ...[SSS_CP(9)][SSS_DATA(128)][PSS_CP(9)][PSS_DATA(128)]...
    // SSS data starts at: offset - 9(PSS_CP) - 128(SSS_DATA) = offset - 137
    int sss_data_start = offset - LTE_CP_NORMAL - LTE_FFT_SIZE;
    if (sss_data_start < 0) return -1;

    // Frequency correction parameters
    // To remove offset: multiply by e^{-j*2π*f_offset*n/fs}
    // Our mixing does signal * conj(osc) where osc = e^{j*dp*n}
    // So we need dp = 2π*freq_offset_hz/fs
    float dp = 2.0f * (float)M_PI * freq_offset_hz / (float)LTE_SAMPLE_RATE;
    float step_re = cosf(dp);
    float step_im = sinf(dp);

    // Extract and frequency-correct SSS OFDM symbol (skip CP)
    cf_t sss_sym[LTE_FFT_SIZE];
    {
        int idx = sss_data_start; // directly at SSS data (CP already excluded)
        if (idx + LTE_FFT_SIZE > st->iq_buf_len) return -1;
        float osc_re = cosf(dp * idx);
        float osc_im = sinf(dp * idx);
        for (int i = 0; i < LTE_FFT_SIZE; i++) {
            float I = iq[(idx + i) * 2];
            float Q = iq[(idx + i) * 2 + 1];
            sss_sym[i].re = I * osc_re + Q * osc_im;
            sss_sym[i].im = Q * osc_re - I * osc_im;
            float new_re = osc_re * step_re - osc_im * step_im;
            osc_im = osc_re * step_im + osc_im * step_re;
            osc_re = new_re;
        }
    }
    fft_dit(sss_sym, st->fft_twiddle, LTE_FFT_SIZE);

    // Extract 62 SSS subcarriers (central subcarriers, skipping DC)
    cf_t sss_rx[LTE_SSS_LEN];
    for (int k = 0; k < 31; k++) {
        sss_rx[k]      = sss_sym[LTE_FFT_SIZE - 31 + k];
        sss_rx[k + 31] = sss_sym[k + 1];
    }

    // Regenerate SSS tables for this N_ID_2
    generate_sss_tables(st->sss_d, n_id_2);

    // Complex correlation: for a flat channel, SSS_rx[k] ≈ H * d[k]
    // |sum(SSS_rx[k] * d_ref[k])| = |H| * 62 for correct hypothesis
    // This works without explicit channel estimation when channel is flat
    int best_n_id_1 = -1;
    float max_corr = 0;

    for (int q = 0; q < LTE_N_ID_1_COUNT; q++) {
        for (int sf = 0; sf < 2; sf++) {
            float corr_re = 0, corr_im = 0;
            for (int n = 0; n < LTE_SSS_LEN; n++) {
                float d = (float)st->sss_d[q][sf][n];
                corr_re += sss_rx[n].re * d;
                corr_im += sss_rx[n].im * d;
            }
            float abs_corr = corr_re * corr_re + corr_im * corr_im;
            if (abs_corr > max_corr) {
                max_corr = abs_corr;
                best_n_id_1 = q;
            }
        }
    }

    // Normalize: max_corr is |corr|², sig_power = sum(|sss_rx|²)
    // Metric = |corr| / sqrt(sig_power * 62) → 1.0 for perfect match
    float sig_power = 0;
    for (int n = 0; n < LTE_SSS_LEN; n++)
        sig_power += cf_abs2(sss_rx[n]);
    float norm_corr = 0;
    if (sig_power > 0)
        norm_corr = sqrtf(max_corr) / sqrtf(sig_power * LTE_SSS_LEN);

    {
        static int sss_dbg = 0;
        if (++sss_dbg % 20 == 1)
            fprintf(stderr, "LTE SSS: n_id_2=%d best_n1=%d corr=%.4f sig_pwr=%.1f fo=%.0f\n",
                    n_id_2, best_n_id_1, norm_corr, sig_power, freq_offset_hz);
    }

    *best_corr = norm_corr;
    return (norm_corr > SSS_THRESHOLD) ? best_n_id_1 : -1;
}

// ======================== MIB Decoding ========================

// PBCH sub-block interleaver inverse permutation (D=40, R=2, C=32)
// Maps encoder output index t → position in interleaved stream
// Computed from 3GPP 36.212 Table 5.1.4-1 column permutation P={1,17,9,25,5,21,13,29,3,19,11,27,7,23,15,31,0,16,8,24,4,20,12,28,2,18,10,26,6,22,14,30}
// with R=2, C=32, D=40 (24 NULL bits in row 0, cols 0-23)
static const uint8_t pbch_deintl[40] = {
    23, 3, 33, 13, 28, 8, 38, 18,
    20, 0, 30, 10, 25, 5, 35, 15,
    22, 2, 32, 12, 27, 7, 37, 17,
    21, 1, 31, 11, 26, 6, 36, 16,
    24, 4, 34, 14, 29, 9, 39, 19
};

// Generate Gold sequence c(n) for n = offset..offset+len-1
// c_init = PCI, Nc = 1600
static void gold_sequence(uint32_t c_init, int offset, int len, uint8_t *seq)
{
    uint32_t x1 = 1u; // x1(0)=1
    uint32_t x2 = c_init;
    // Advance by Nc + offset
    for (int n = 0; n < 1600 + offset; n++) {
        uint32_t new1 = ((x1 >> 3) ^ x1) & 1;
        x1 = (x1 >> 1) | (new1 << 30);
        uint32_t new2 = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1;
        x2 = (x2 >> 1) | (new2 << 30);
    }
    for (int n = 0; n < len; n++) {
        seq[n] = (uint8_t)(((x1 >> 0) ^ (x2 >> 0)) & 1);
        uint32_t new1 = ((x1 >> 3) ^ x1) & 1;
        x1 = (x1 >> 1) | (new1 << 30);
        uint32_t new2 = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1;
        x2 = (x2 >> 1) | (new2 << 30);
    }
}

// Attempt to decode MIB from PBCH
// PBCH occupies symbols 0-3 of slot 1 in subframe 0 (72 subcarriers, 6 RBs center)
static bool decode_mib(struct lte_state *st, const float *iq, int frame_start,
                       int pci, float freq_offset_hz, lte_mib_t *mib)
{
    // PBCH is in subframe 0, slot 1, symbols 0-3
    int slot1_start = frame_start + LTE_SAMPLES_PER_SLOT;
    int iq_pairs = st->iq_buf_len;

    // Check bounds
    int pbch_end = slot1_start + 4 * (LTE_CP_NORMAL + LTE_FFT_SIZE);
    if (slot1_start < 0 || pbch_end > iq_pairs) return false;

    // Pre-compute frequency correction oscillator parameters
    float dp = 2.0f * (float)M_PI * freq_offset_hz / (float)LTE_SAMPLE_RATE;
    float step_re = cosf(dp);
    float step_im = sinf(dp);

    // Extract 4 OFDM symbols of PBCH with frequency correction + FFT
    cf_t pbch_syms[4][LTE_FFT_SIZE];
    int pos = slot1_start;
    for (int sym_idx = 0; sym_idx < 4; sym_idx++) {
        int cp_len = (sym_idx == 0) ? LTE_CP_NORMAL_0 : LTE_CP_NORMAL;
        pos += cp_len; // skip CP
        if (pos + LTE_FFT_SIZE > iq_pairs) return false;
        float osc_re = cosf(dp * pos);
        float osc_im = sinf(dp * pos);
        for (int i = 0; i < LTE_FFT_SIZE; i++) {
            float I = iq[(pos + i) * 2];
            float Q = iq[(pos + i) * 2 + 1];
            pbch_syms[sym_idx][i].re = I * osc_re + Q * osc_im;
            pbch_syms[sym_idx][i].im = Q * osc_re - I * osc_im;
            float new_re = osc_re * step_re - osc_im * step_im;
            osc_im = osc_re * step_im + osc_im * step_re;
            osc_re = new_re;
        }
        fft_dit(pbch_syms[sym_idx], st->fft_twiddle, LTE_FFT_SIZE);
        pos += LTE_FFT_SIZE;
    }

    // PSS-based channel estimation
    // PSS data is at frame_start + 832 (last symbol of slot 0)
    int pss_pos = frame_start + LTE_CP_NORMAL_0 + LTE_FFT_SIZE
                + 5 * (LTE_CP_NORMAL + LTE_FFT_SIZE) + LTE_CP_NORMAL;
    int n_id_2 = pci % 3;

    // FFT the received PSS with frequency correction
    cf_t pss_rx[LTE_FFT_SIZE];
    {
        float osc_re_p = cosf(dp * pss_pos);
        float osc_im_p = sinf(dp * pss_pos);
        for (int i = 0; i < LTE_FFT_SIZE; i++) {
            float I = iq[(pss_pos + i) * 2];
            float Q = iq[(pss_pos + i) * 2 + 1];
            pss_rx[i].re = I * osc_re_p + Q * osc_im_p;
            pss_rx[i].im = Q * osc_re_p - I * osc_im_p;
            float new_re = osc_re_p * step_re - osc_im_p * step_im;
            osc_im_p = osc_re_p * step_im + osc_im_p * step_re;
            osc_re_p = new_re;
        }
        fft_dit(pss_rx, st->fft_twiddle, LTE_FFT_SIZE);
    }

    // Estimate H at 62 PSS subcarriers: H = rx / pss_ref (complex division)
    // h_est[0..30] = H at subcarriers -31..-1 (bins 97..127)
    // h_est[31..61] = H at subcarriers +1..+31 (bins 1..31)
    cf_t h_est[62];
    for (int k = 0; k < 31; k++) {
        // Negative freq subcarrier -(31-k): bin (FFT_SIZE-31+k), pss_freq[k]
        cf_t rx = pss_rx[LTE_FFT_SIZE - 31 + k];
        cf_t ref = st->pss_freq[n_id_2][k];
        float denom = ref.re * ref.re + ref.im * ref.im;
        if (denom < 1e-10f) denom = 1e-10f;
        h_est[k].re = (rx.re * ref.re + rx.im * ref.im) / denom;
        h_est[k].im = (rx.im * ref.re - rx.re * ref.im) / denom;
        // Positive freq subcarrier +(k+1): bin (k+1), pss_freq[k+31]
        rx = pss_rx[k + 1];
        ref = st->pss_freq[n_id_2][k + 31];
        denom = ref.re * ref.re + ref.im * ref.im;
        if (denom < 1e-10f) denom = 1e-10f;
        h_est[31 + k].re = (rx.re * ref.re + rx.im * ref.im) / denom;
        h_est[31 + k].im = (rx.im * ref.re - rx.re * ref.im) / denom;
    }

    // Build channel estimate for all 72 PBCH subcarriers
    // PBCH k=0..35 → subcarriers -36..-1; k=36..71 → subcarriers +1..+36
    // PSS covers subcarriers -31..-1 and +1..+31; extrapolate edges
    cf_t h_pbch[72];
    for (int k = 0; k < 72; k++) {
        int sc = (k < 36) ? (k - 36) : (k - 35); // subcarrier index
        if (sc >= -31 && sc <= -1) {
            h_pbch[k] = h_est[sc + 31]; // index 0..30
        } else if (sc >= 1 && sc <= 31) {
            h_pbch[k] = h_est[30 + sc]; // index 31..61
        } else if (sc < -31) {
            h_pbch[k] = h_est[0]; // extrapolate left
        } else { // sc > 31
            h_pbch[k] = h_est[61]; // extrapolate right
        }
    }

    // Apply zero-forcing equalization to all PBCH symbols
    for (int sym_idx = 0; sym_idx < 4; sym_idx++) {
        for (int k = 0; k < 72; k++) {
            int bin = (k < 36) ? (LTE_FFT_SIZE - 36 + k) : (k - 35);
            float denom = h_pbch[k].re * h_pbch[k].re + h_pbch[k].im * h_pbch[k].im;
            if (denom < 1e-10f) continue;
            float re = pbch_syms[sym_idx][bin].re;
            float im = pbch_syms[sym_idx][bin].im;
            pbch_syms[sym_idx][bin].re = (re * h_pbch[k].re + im * h_pbch[k].im) / denom;
            pbch_syms[sym_idx][bin].im = (im * h_pbch[k].re - re * h_pbch[k].im) / denom;
        }
    }

    // Debug: print channel estimate quality
    static int mib_dbg_cnt = 0;
    if (++mib_dbg_cnt % 20 == 1) {
        float ph0 = atan2f(h_est[15].im, h_est[15].re);
        float ph1 = atan2f(h_est[45].im, h_est[45].re);
        float mag = sqrtf(h_est[15].re * h_est[15].re + h_est[15].im * h_est[15].im);
        fprintf(stderr, "LTE MIB: H_phase[mid]=%.2f,%.2f |H|=%.1f\n", ph0, ph1, mag);
    }

    // Extract PBCH subcarriers (center 72, excluding CRS for all antenna ports)
    // CRS exclusion for PBCH (3GPP 36.211 §6.6.4):
    //   Symbol 0 (slot1): CRS ports 0,1 → k%6 == v_shift OR k%6 == (v_shift+3)%6
    //   Symbol 1 (slot1): CRS ports 2,3 → k%6 == v_shift OR k%6 == (v_shift+3)%6
    //   Symbols 2,3: no CRS exclusion
    int v_shift = pci % 6;
    int v_shift3 = (v_shift + 3) % 6;
    int8_t soft_bits[480];
    int soft_idx = 0;

    for (int sym_idx = 0; sym_idx < 4; sym_idx++) {
        for (int k = 0; k < 72; k++) {
            // Map to FFT bin: center 72 subcarriers exclude DC
            // k=0..35 → subcarriers -36..-1 → FFT bins (N-36)..(N-1)
            // k=36..71 → subcarriers +1..+36 → FFT bins 1..36
            int bin;
            if (k < 36) bin = LTE_FFT_SIZE - 36 + k; // bins 92..127
            else bin = k - 35;                         // bins 1..36

            // CRS exclusion on symbols 0,1 (for ports 0-3 ambiguity)
            if (sym_idx < 2) {
                int kmod6 = k % 6;
                if (kmod6 == v_shift || kmod6 == v_shift3)
                    continue;
            }

            // QPSK demodulation (hard decision ±1)
            cf_t s = pbch_syms[sym_idx][bin];
            soft_bits[soft_idx++] = (s.re > 0) ? 1 : -1;
            soft_bits[soft_idx++] = (s.im > 0) ? 1 : -1;
        }
    }

    if (soft_idx != 480) return false; // sanity check: expect exactly 240 QPSK symbols

    // Try all 4 possible frame positions within the 40ms TTI
    // Each position uses a different scrambling sequence offset (0, 480, 960, 1440)
    for (int frame_quarter = 0; frame_quarter < 4; frame_quarter++) {
        int scr_offset = frame_quarter * 480;

        // Generate scrambling sequence for this quarter
        uint8_t scr_seq[480];
        gold_sequence((uint32_t)pci, scr_offset, 480, scr_seq);

        // Descramble soft bits
        int8_t descrambled[480];
        for (int n = 0; n < 480; n++)
            descrambled[n] = scr_seq[n] ? -soft_bits[n] : soft_bits[n];

        // Rate de-matching: soft-combine 4 repetitions of 120 coded bits
        int combined[120];
        for (int i = 0; i < 120; i++)
            combined[i] = descrambled[i] + descrambled[120+i] + descrambled[240+i] + descrambled[360+i];

        // De-interleave: circular buffer = [stream0_intl[40], stream1_intl[40], stream2_intl[40]]
        // Viterbi expects: viterbi_in[t*3+g] = stream_g[t] (encoder output order)
        // De-interleave each stream using pbch_deintl: stream_g[t] = combined[g*40 + pbch_deintl[t]]
        int8_t viterbi_in[120];
        for (int t = 0; t < 40; t++) {
            for (int g = 0; g < 3; g++) {
                int val = combined[g * 40 + pbch_deintl[t]];
                if (val > 127) val = 127;
                if (val < -127) val = -127;
                viterbi_in[t * 3 + g] = (int8_t)val;
            }
        }

        // Viterbi decode (tail-biting: init all states equally)
        uint8_t decoded[120];
        int n_decoded = viterbi_decode(viterbi_in, 120, decoded);
        if (n_decoded < 40) continue;

        // Check CRC-16
        uint16_t crc = crc16_lte(decoded, 24);
        uint16_t rx_crc = 0;
        for (int i = 0; i < 16; i++)
            rx_crc |= (uint16_t)(decoded[24 + i] & 1) << (15 - i);

        // CRC mask indicates antenna ports
        uint16_t crc_m0 = rx_crc ^ (uint16_t)PBCH_CRC_MASK_0;
        uint16_t crc_m1 = rx_crc ^ (uint16_t)PBCH_CRC_MASK_1;
        uint16_t crc_m2 = rx_crc ^ (uint16_t)PBCH_CRC_MASK_2;
        bool crc_ok = (crc == crc_m0) || (crc == crc_m1) || (crc == crc_m2);

        if (!crc_ok) continue;

        // Parse MIB: 24 bits
        mib->dl_bandwidth = (decoded[0] << 2) | (decoded[1] << 1) | decoded[2];
        mib->phich_duration = decoded[3];
        mib->phich_resources = (decoded[4] << 1) | decoded[5];
        mib->sfn = 0;
        for (int i = 0; i < 8; i++)
            mib->sfn |= (uint16_t)(decoded[6 + i] & 1) << (9 - i);
        // Lower 2 bits of SFN from frame quarter position
        mib->sfn |= (uint16_t)(frame_quarter & 3);
        mib->valid = true;
        return true;
    }

    return false;
}

// ======================== Main Processing ========================

void lte_process(struct lte_state *st, const uint8_t *iq_data, unsigned len)
{
    unsigned n_samples = len / 2;
    st->stats.samples_processed += n_samples;

    // Convert uint8 IQ to float and append to buffer
    unsigned offset = 0;
    while (offset < n_samples) {
        unsigned avail = (unsigned)(st->iq_buf_size - st->iq_buf_len);
        unsigned chunk = n_samples - offset;
        if (chunk > avail) chunk = avail;

        if (chunk == 0) {
            // Buffer full — process and shift
            // Try PSS detection on the buffered data
            float max_corr_dbg = 0;
            for (int nid2 = 0; nid2 < 3; nid2++) {
                int peak_pos = 0;
                float peak_phase = 0;
                float freq_offset_hz = 0;
                float corr = pss_correlate(st->iq_buf, st->iq_buf_len,
                                           st->pss_time[nid2], LTE_FFT_SIZE,
                                           &peak_pos, &peak_phase, &freq_offset_hz);
                if (corr > max_corr_dbg) max_corr_dbg = corr;

                if (corr > PSS_THRESHOLD) {
                    st->stats.pss_detected++;

                    // Try SSS detection using PSS-based channel estimation
                    float sss_corr = 0;
                    int n_id_1 = detect_sss(st, st->iq_buf, peak_pos, nid2,
                                            freq_offset_hz, &sss_corr);

                    int pci = -1;
                    if (n_id_1 >= 0) {
                        pci = 3 * n_id_1 + nid2;
                        st->stats.sss_decoded++;
                    }

                    // Find or create cell entry
                    lte_cell_state_t *cell = NULL;
                    if (pci >= 0) {
                        for (int c = 0; c < st->cell_count; c++) {
                            if (st->cells[c].active && st->cells[c].info.pci == (uint16_t)pci) {
                                cell = &st->cells[c];
                                break;
                            }
                        }
                    }
                    if (!cell && pci >= 0) {
                        // Allocate new cell
                        if (st->cell_count < LTE_MAX_CELLS) {
                            cell = &st->cells[st->cell_count++];
                            memset(cell, 0, sizeof(*cell));
                            cell->active = true;
                            cell->info.pci = (uint16_t)pci;
                            cell->info.n_id_2 = (uint8_t)nid2;
                            cell->info.n_id_1 = (uint8_t)n_id_1;
                            cell->info.freq_hz = st->cfg.center_freq;
                            cell->info.earfcn = lte_freq_to_earfcn(st->cfg.center_freq);
                            cell->info.sync_state = LTE_SYNC_SSS;
                        }
                    } else if (!cell && pci < 0) {
                        // PSS only — check if we already have a PSS-only cell for this N_ID_2
                        for (int c = 0; c < st->cell_count; c++) {
                            if (st->cells[c].active && st->cells[c].info.n_id_2 == (uint8_t)nid2 &&
                                st->cells[c].info.sync_state == LTE_SYNC_PSS) {
                                cell = &st->cells[c];
                                break;
                            }
                        }
                        if (!cell && st->cell_count < LTE_MAX_CELLS) {
                            cell = &st->cells[st->cell_count++];
                            memset(cell, 0, sizeof(*cell));
                            cell->active = true;
                            cell->info.n_id_2 = (uint8_t)nid2;
                            cell->info.freq_hz = st->cfg.center_freq;
                            cell->info.earfcn = lte_freq_to_earfcn(st->cfg.center_freq);
                            cell->info.sync_state = LTE_SYNC_PSS;
                        }
                    }

                    if (cell) {
                        cell->info.pss_count++;
                        cell->last_pss_sample = st->sample_counter;
                        cell->last_update = st->sample_counter;
                        cell->info.rsrp_dbfs = 10.0f * log10f(corr + 1e-10f);
                        cell->info.snr_db = 10.0f * log10f(corr / (1.0f - corr + 1e-10f));
                        cell->info.freq_offset_hz = freq_offset_hz;

                        if (pci >= 0 && cell->info.sync_state >= LTE_SYNC_SSS) {
                            cell->info.sss_count++;

                            // Try MIB decode
                            // PSS is symbol 6 of slot 0, peak_pos = PSS data start
                            // Frame start = peak_pos - PSS_CP - 5*(CP+FFT) - (CP0+FFT)
                            int frame_start = peak_pos - LTE_CP_NORMAL
                                            - 5 * (LTE_CP_NORMAL + LTE_FFT_SIZE)
                                            - (LTE_CP_NORMAL_0 + LTE_FFT_SIZE);
                            lte_mib_t mib;
                            memset(&mib, 0, sizeof(mib));
                            if (decode_mib(st, st->iq_buf, frame_start, pci,
                                           freq_offset_hz, &mib)) {
                                cell->info.mib = mib;
                                cell->info.sync_state = LTE_SYNC_MIB;
                                cell->info.mib_count++;
                                st->stats.mib_decoded++;
                            } else {
                                cell->info.crc_errors++;
                                st->stats.crc_errors++;
                            }
                        }

                        // Notify callback
                        if (st->cfg.callback) {
                            st->cfg.callback(&cell->info, st->cfg.callback_ctx);
                        }
                    }
                }
            }
            {
                static int dbg_n = 0;
                if (++dbg_n % 50 == 1)
                    fprintf(stderr, "LTE: buf#%d maxCorr=%.4f freq=%.1f MHz\n",
                            dbg_n, max_corr_dbg, st->cfg.center_freq / 1e6);
            }

            // Shift buffer left by half
            int shift = st->iq_buf_size / 2;
            memmove(st->iq_buf, st->iq_buf + shift * 2,
                    (size_t)(st->iq_buf_len - shift) * 2 * sizeof(float));
            st->iq_buf_len -= shift;
            continue;
        }

        // Convert u8 IQ to float (centered at 0)
        for (unsigned i = 0; i < chunk; i++) {
            st->iq_buf[(st->iq_buf_len + (int)i) * 2]     = ((float)iq_data[(offset + i) * 2] - 127.5f) / 127.5f;
            st->iq_buf[(st->iq_buf_len + (int)i) * 2 + 1] = ((float)iq_data[(offset + i) * 2 + 1] - 127.5f) / 127.5f;
        }
        st->iq_buf_len += (int)chunk;
        offset += chunk;
        st->sample_counter += chunk;
    }

    // Check if it's time to hop frequency
    if (st->cfg.hop_enabled && st->sample_counter >= st->hop_next_sample && st->hop_request == 0) {
        st->hop_idx = (st->hop_idx + 1) % LTE_HOP_FREQS;
        st->hop_request = lte_hop_freqs[st->hop_idx];
    }
}

// ======================== Query Functions ========================

void lte_get_stats(const struct lte_state *st, lte_stats_t *out)
{
    if (!st || !out) return;
    *out = st->stats;
}

int lte_get_cells(const struct lte_state *st, lte_cell_info_t *out, int max_cells)
{
    if (!st || !out) return 0;
    int count = 0;
    for (int i = 0; i < st->cell_count && count < max_cells; i++) {
        if (st->cells[i].active) {
            out[count++] = st->cells[i].info;
        }
    }
    return count;
}

lte_sync_state_t lte_get_best_sync(const struct lte_state *st)
{
    if (!st) return LTE_SYNC_NONE;
    lte_sync_state_t best = LTE_SYNC_NONE;
    for (int i = 0; i < st->cell_count; i++) {
        if (st->cells[i].active && st->cells[i].info.sync_state > best)
            best = st->cells[i].info.sync_state;
    }
    return best;
}

// Frequency hopping: returns non-zero if decoder wants a retune
double lte_get_hop_freq(struct lte_state *st)
{
    if (!st) return 0;
    return st->hop_request;
}

// Called after retune is complete — reset buffers and update dwell timer
void lte_set_freq(struct lte_state *st, double freq_hz)
{
    if (!st) return;
    st->cfg.center_freq = freq_hz;
    st->hop_request = 0;
    // Flush IQ buffer (samples at old freq are useless)
    st->iq_buf_len = 0;
    // Set next hop time
    uint64_t dwell_samples = (uint64_t)(st->cfg.sample_rate * LTE_HOP_DWELL_MS / 1000.0);
    st->hop_next_sample = st->sample_counter + dwell_samples;
}

// ======================== Utility ========================

double lte_earfcn_to_freq(uint32_t earfcn)
{
    // Band 20 (800 MHz): EARFCN 6150-6449, F_DL = 791 + 0.1*(EARFCN - 6150)
    if (earfcn >= 6150 && earfcn <= 6449)
        return 791000000.0 + 100000.0 * (earfcn - 6150);
    // Band 3 (1800 MHz): EARFCN 1200-1949, F_DL = 1805 + 0.1*(EARFCN - 1200)
    if (earfcn >= 1200 && earfcn <= 1949)
        return 1805000000.0 + 100000.0 * (earfcn - 1200);
    // Band 7 (2600 MHz): EARFCN 2750-3449, F_DL = 2620 + 0.1*(EARFCN - 2750)
    if (earfcn >= 2750 && earfcn <= 3449)
        return 2620000000.0 + 100000.0 * (earfcn - 2750);
    // Band 1 (2100 MHz): EARFCN 0-599, F_DL = 2110 + 0.1*(EARFCN - 0)
    if (earfcn <= 599)
        return 2110000000.0 + 100000.0 * earfcn;
    // Band 8 (900 MHz): EARFCN 3450-3799, F_DL = 925 + 0.1*(EARFCN - 3450)
    if (earfcn >= 3450 && earfcn <= 3799)
        return 925000000.0 + 100000.0 * (earfcn - 3450);
    // Band 28 (700 MHz): EARFCN 9210-9659, F_DL = 758 + 0.1*(EARFCN - 9210)
    if (earfcn >= 9210 && earfcn <= 9659)
        return 758000000.0 + 100000.0 * (earfcn - 9210);
    return 0;
}

uint32_t lte_freq_to_earfcn(double freq_hz)
{
    // Band 20 (800 MHz)
    if (freq_hz >= 791000000 && freq_hz <= 821000000)
        return 6150 + (uint32_t)((freq_hz - 791000000.0) / 100000.0 + 0.5);
    // Band 3 (1800 MHz)
    if (freq_hz >= 1805000000 && freq_hz <= 1880000000)
        return 1200 + (uint32_t)((freq_hz - 1805000000.0) / 100000.0 + 0.5);
    // Band 7 (2600 MHz)
    if (freq_hz >= 2620000000 && freq_hz <= 2690000000)
        return 2750 + (uint32_t)((freq_hz - 2620000000.0) / 100000.0 + 0.5);
    // Band 1 (2100 MHz)
    if (freq_hz >= 2110000000 && freq_hz <= 2170000000)
        return (uint32_t)((freq_hz - 2110000000.0) / 100000.0 + 0.5);
    // Band 8 (900 MHz)
    if (freq_hz >= 925000000 && freq_hz <= 960000000)
        return 3450 + (uint32_t)((freq_hz - 925000000.0) / 100000.0 + 0.5);
    // Band 28 (700 MHz)
    if (freq_hz >= 758000000 && freq_hz <= 803000000)
        return 9210 + (uint32_t)((freq_hz - 758000000.0) / 100000.0 + 0.5);
    return 0;
}

const char *lte_band_name(double freq_hz)
{
    if (freq_hz >= 791000000 && freq_hz <= 821000000) return "B20 (800 MHz)";
    if (freq_hz >= 1805000000 && freq_hz <= 1880000000) return "B3 (1800 MHz)";
    if (freq_hz >= 2620000000 && freq_hz <= 2690000000) return "B7 (2600 MHz)";
    if (freq_hz >= 2110000000 && freq_hz <= 2170000000) return "B1 (2100 MHz)";
    if (freq_hz >= 925000000 && freq_hz <= 960000000) return "B8 (900 MHz)";
    if (freq_hz >= 758000000 && freq_hz <= 803000000) return "B28 (700 MHz)";
    return "Unknown";
}

const char *lte_bw_string(uint8_t dl_bw)
{
    switch (dl_bw) {
        case 0: return "1.4 MHz";
        case 1: return "3 MHz";
        case 2: return "5 MHz";
        case 3: return "10 MHz";
        case 4: return "15 MHz";
        case 5: return "20 MHz";
        default: return "?";
    }
}

int lte_bw_to_nrb(uint8_t dl_bw)
{
    switch (dl_bw) {
        case 0: return 6;
        case 1: return 15;
        case 2: return 25;
        case 3: return 50;
        case 4: return 75;
        case 5: return 100;
        default: return 0;
    }
}
