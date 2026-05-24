// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// gsm_decode.c: GSM broadcast channel decoder implementation.
//
// Implements:
//   - GMSK differential demodulation
//   - FCCH detection (frequency correction burst — tone at +67.7 kHz)
//   - SCH decoding (synchronization burst — BSIC + frame number)
//   - Viterbi decoder (rate 1/2, K=5 convolutional code)
//   - Fire code CRC (40-bit, for BCCH/CCCH)
//   - Block diagonal deinterleaving
//   - LAPDm L2 frame parsing
//   - System Information Type 1/2/3/4 parsing
//   - Cell Broadcast (CBCH) parsing
//   - Paging channel (PCH) parsing
//
// References:
//   3GPP TS 05.02 v8.10.0 — Multiplexing and multiple access
//   3GPP TS 05.03 v8.6.0  — Channel coding
//   3GPP TS 05.04 v8.4.0  — Modulation
//   3GPP TS 04.06 v8.3.0  — LAPDm
//   3GPP TS 04.08 v7.21.0 — RR messages
//   3GPP TS 23.041 v11.4.0 — Cell Broadcast Service

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "gsm_decode.h"

// ======================== Training Sequences (TS 05.02) ========================

// Normal Burst TSC 0-7 (Table 5.2.3)
// Values: 0/1 bit values as transmitted
const int8_t gsm_nb_training[8][26] = {
    {0,0,1,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1,1,1}, // TSC 0
    {0,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1}, // TSC 1
    {0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0}, // TSC 2
    {0,1,0,0,0,1,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,0}, // TSC 3
    {0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,1}, // TSC 4
    {0,1,0,0,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,0,1,0}, // TSC 5
    {1,0,1,0,0,1,1,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,1}, // TSC 6
    {1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0}, // TSC 7
};

// Synchronization Burst training sequence (section 5.2.5), 64 bits
const int8_t gsm_sb_training[64] = {
    1,0,1,1,1,0,0,1,0,1,1,0,0,0,0,1,
    0,0,0,0,1,1,0,0,1,0,0,1,0,1,0,1,
    0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,
    1,1,0,1,0,0,0,1,0,1,1,1,0,0,0,0
};

// Dummy Burst fixed bits (section 5.2.6) — 142 fixed bits of a dummy burst.
// Only first 114 data bits used for comparison (57 + 57 from the NB structure).
// Used in normal burst processing to detect and skip dummy bursts.
static const int8_t dummy_burst_data[GSM_NB_DATA_BITS] __attribute__((unused)) = {
    1,1,1,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,1,0,0,1,0,0,
    1,1,1,0,0,0,0,0,1,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,1,1,1,1,
    0,0,0,1,1,1,0,0,0,0,0,0,1,0,1,1,1,0,0,0,1,0,1,1,1,0,0,0,
    1,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,0,1,0,1,1,0,0,0,0,0,1,0
};

// ======================== Internal State ========================

// Circular buffer for accumulating IQ samples
#define GSM_IQ_BUF_SIZE  (GSM_SAMPLES_PER_FRAME * 52 * 2) // ~52 frames worth of IQ

// Burst accumulator for 4-burst interleaving
typedef struct {
    float    soft_bits[4][GSM_NB_DATA_BITS]; // 4 bursts × 114 soft bits
    int32_t      n_bursts;                        // bursts collected (0-3)
    uint32_t fn_start;                        // frame number of first burst
} gsm_burst_acc_t;

struct gsm_state {
    gsm_config_t    cfg;
    gsm_sync_state_t sync_state;
    gsm_cell_info_t cell;
    gsm_stats_t     stats;

    // IQ sample buffer
    float          *phase_buf;          // instantaneous phase history
    int32_t             phase_buf_size;
    int32_t             phase_buf_len;      // valid samples in buffer

    // FCCH detection
    double          freq_offset;        // estimated frequency offset (Hz)
    int32_t             fcch_sample_pos;    // sample position of last FCCH

    // Frame timing
    int32_t             frame_sample_pos;   // sample position of current frame start
    int32_t             samples_per_frame;  // computed from sample rate
    double          symbol_period;      // samples per symbol (fractional)
    double carrier_offset_hz;   // ARFCN freq - tuned freq (Hz)
    uint32_t        fn;                 // current TDMA frame number
    int32_t             fn_mod51;           // position within 51-multiframe

    // SCH
    uint8_t         bsic;
    bool            bsic_valid;

    // TSC detection
    int32_t             tsc;                // detected or configured TSC

    // Burst accumulator for BCCH (4-burst interleaving)
    gsm_burst_acc_t bcch_acc;
    gsm_burst_acc_t ccch_acc[4];        // up to 4 CCCH blocks

    // Internal working buffers
    float          *work_soft;          // temp soft bit buffer
    uint8_t        *work_hard;          // temp hard bit buffer

    // Channel filter state (mix to baseband + LPF)
    float          *filt_I;             // filter delay line, I component
    float          *filt_Q;             // filter delay line, Q component
    float          *filt_coeff;         // FIR filter coefficients (symmetric)
    int32_t             filt_taps;          // number of FIR taps (odd)
    double          filt_phase;         // NCO phase for carrier mixing (radians)
    double          filt_phase_inc;     // NCO phase increment per sample
};

// ======================== Utility ========================

static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// Normalize angle to [-π, π]
static inline float wrap_phase(float p) {
    while (p > (float)M_PI) p -= 2.0f * (float)M_PI;
    while (p < -(float)M_PI) p += 2.0f * (float)M_PI;
    return p;
}

// ======================== GMSK Demodulation ========================

// Convert IQ uint8 pairs to phase (radians).
// iq_data: interleaved I,Q uint8 pairs. len: total bytes (n_samples = len/2)
// phase_out: output phase array, must hold len/2 floats.
static void iq_to_phase(const uint8_t *iq_data, uint32_t len, float *phase_out)
    __attribute__((unused));
static void iq_to_phase(const uint8_t *iq_data, uint32_t len, float *phase_out)
{
    uint32_t n = len / 2;
    for (uint32_t i = 0; i < n; i++) {
        float I = (float)iq_data[2*i]   - 127.5f;
        float Q = (float)iq_data[2*i+1] - 127.5f;
        phase_out[i] = atan2f(Q, I);
    }
}

// Mix IQ data to baseband, apply FIR low-pass filter, then compute phase.
// This removes the IF offset carrier and rejects out-of-band noise.
// Writes filtered phase samples to phase_out (n_samples floats).
static void iq_to_phase_filtered(struct gsm_state *st,
                                  const uint8_t *iq_data, uint32_t n_samples,
                                  float *phase_out)
{
    int32_t ntaps = st->filt_taps;
    int32_t half = ntaps / 2;

    for (uint32_t i = 0; i < n_samples; i++) {
        float raw_I = (float)iq_data[2*i]   - 127.5f;
        float raw_Q = (float)iq_data[2*i+1] - 127.5f;

        /* Mix to baseband: multiply by exp(-j * carrier_phase) */
        float cos_p = cosf((float)st->filt_phase);
        float sin_p = sinf((float)st->filt_phase);
        float bb_I = raw_I * cos_p - raw_Q * sin_p;
        float bb_Q = raw_I * sin_p + raw_Q * cos_p;
        st->filt_phase += st->filt_phase_inc;
        if (st->filt_phase > M_PI)  st->filt_phase -= 2.0 * M_PI;
        if (st->filt_phase < -M_PI) st->filt_phase += 2.0 * M_PI;

        /* Shift delay line and insert new sample */
        memmove(&st->filt_I[0], &st->filt_I[1], (size_t)(ntaps - 1) * sizeof(float));
        memmove(&st->filt_Q[0], &st->filt_Q[1], (size_t)(ntaps - 1) * sizeof(float));
        st->filt_I[ntaps - 1] = bb_I;
        st->filt_Q[ntaps - 1] = bb_Q;

        /* FIR convolution */
        float out_I = 0, out_Q = 0;
        for (int32_t k = 0; k < ntaps; k++) {
            out_I += st->filt_I[k] * st->filt_coeff[k];
            out_Q += st->filt_Q[k] * st->filt_coeff[k];
        }

        phase_out[i] = atan2f(out_Q, out_I);
    }
    (void)half;
}

// Differential phase detection: instantaneous frequency from phase buffer.
// freq_out[i] = phase[i+1] - phase[i], normalized.
static void phase_to_freq(const float *phase, int32_t n, float *freq_out)
{
    for (int32_t i = 0; i < n - 1; i++) {
        freq_out[i] = wrap_phase(phase[i+1] - phase[i]);
    }
}

// ======================== FCCH Detection ========================

// FCCH burst: 142 consecutive zero bits → constant phase increment of +π/2 per symbol.
// At our sample rate, the expected phase increment per sample is:
//   Δφ = (π/2) / samples_per_symbol = π/(2 * 3.6923) ≈ 0.4254 rad/sample
//
// We detect FCCH by looking for a sustained segment where the instantaneous
// frequency is close to this expected value.

#define FCCH_MIN_SYMBOLS    50   // minimum consecutive symbols matching
#define FCCH_FREQ_TOL       0.30f // tolerance in radians per sample

// Returns sample offset of FCCH center within freq_buf, or -1 if not found.
// Also estimates frequency offset from ideal.
int32_t detect_fcch(const float *freq_buf, int32_t n_freq,
                       float samples_per_symbol, float carrier_rps,
                       double *freq_offset_out)
{
    // Expected phase increment per sample for FCCH tone
    float expected_dphi = carrier_rps + (float)M_PI / (2.0f * samples_per_symbol);
    int32_t min_samples = (int32_t)(FCCH_MIN_SYMBOLS * samples_per_symbol);

    int32_t best_start = -1;
    int32_t best_len = 0;
    double best_sum = 0;

    int32_t run_start = 0;
    int32_t run_len = 0;
    double run_sum = 0;

    for (int32_t i = 0; i < n_freq; i++) {
        float diff = fabsf(freq_buf[i] - expected_dphi);
        if (diff < FCCH_FREQ_TOL) {
            if (run_len == 0) run_start = i;
            run_len++;
            run_sum += freq_buf[i];
        } else {
            if (run_len > best_len) {
                best_start = run_start;
                best_len = run_len;
                best_sum = run_sum;
            }
            run_len = 0;
            run_sum = 0;
        }
    }
    if (run_len > best_len) {
        best_start = run_start;
        best_len = run_len;
        best_sum = run_sum;
    }

    if (best_len < min_samples) return -1;

    // Estimate actual frequency offset from the mean phase increment
    double mean_dphi = best_sum / best_len;
    double actual_freq_per_sample = mean_dphi / (2.0 * M_PI) * GSM_SAMPLE_RATE;
    double ideal_freq = (double)carrier_rps / (2.0 * M_PI) * GSM_SAMPLE_RATE + (double)GSM_SYMBOL_RATE / 4.0;
    if (freq_offset_out) {
        *freq_offset_out = actual_freq_per_sample - ideal_freq;
    }

    return best_start + best_len; // return END of FCCH run (near SCH start)
}

// ======================== Burst Extraction ========================

// Extract soft bits from a burst at a given sample position.
// Uses the known training sequence to refine timing and estimate channel.
// phase_buf: instantaneous phase of the signal
// sample_start: sample position of burst start (first tail bit)
// samples_per_sym: fractional samples per symbol
// soft_out: GSM_BURST_BITS soft decisions
// freq_offset: current frequency offset correction (rad/sample)
static void extract_burst_soft(const float *phase_buf, int32_t phase_len,
                               int32_t sample_start, float samples_per_sym,
                               float freq_offset_rps, float *soft_out)
{
    // Sample at symbol centers, single-sample differential phase
    float expected_mag = (float)M_PI / (2.0f * samples_per_sym);

    for (int32_t b = 0; b < GSM_BURST_BITS; b++) {
        int32_t sample_idx = sample_start + (int32_t)(b * samples_per_sym + 0.5f);
        if (sample_idx < 1 || sample_idx >= phase_len) {
            soft_out[b] = 0.0f;
            continue;
        }

        float dphi = wrap_phase(phase_buf[sample_idx] - phase_buf[sample_idx - 1]
                                - freq_offset_rps);

        soft_out[b] = dphi / expected_mag;
        soft_out[b] = clampf(soft_out[b], -4.0f, 4.0f);
    }
}

// Correlate with training sequence to find exact timing.
// soft_bits: estimated soft bits for a burst (GSM_BURST_BITS)
// tsc: training sequence index (0-7)
// Returns timing offset in samples (fractional), and correlation quality.
static float correlate_training(const float *soft_bits, const int8_t *training,
                                int32_t train_len, int32_t train_start_bit, float *quality_out)
{
    // The training sequence starts at bit position train_start_bit within the burst
    // Search around the expected position for best correlation
    float best_corr = 0;
    int32_t best_offset = 0;

    for (int32_t offset = -20; offset <= 20; offset++) {
        float corr = 0;
        int32_t pos = train_start_bit + offset;
        if (pos < 0 || pos + train_len > GSM_BURST_BITS) continue;

        for (int32_t i = 0; i < train_len; i++) {
            float expected = training[i] ? -1.0f : 1.0f;
            corr += soft_bits[pos + i] * expected;
        }
        corr /= train_len;

        if (corr > best_corr) {
            best_corr = corr;
            best_offset = offset;
        }
    }

    if (quality_out) *quality_out = best_corr;
    return (float)best_offset;
}

// ======================== Convolutional Coding (TS 05.03) ========================

// Generator polynomials:
//   G0(D) = 1 + D^3 + D^4  →  taps at positions 0, 3, 4
//   G1(D) = 1 + D + D^3 + D^4  →  taps at positions 0, 1, 3, 4
//
// Encoding: for input bit u and shift register state s[3..0]:
//   c0 = u ^ s[2] ^ s[3]      (G0 taps: current input, delay 3, delay 4)
//   c1 = u ^ s[0] ^ s[2] ^ s[3]  (G1 taps: current input, delay 1, delay 3, delay 4)
//   New state: s[3..0] = {s[2], s[1], s[0], u}  (shift right, u enters at MSB)
//
// Wait, let me reconsider the register convention.
// Shift register: s[0] is the most recently entered bit.
// After clocking with input u:
//   s[3] = s[2] (oldest), s[2] = s[1], s[1] = s[0], s[0] = u (newest)
// G0: u ^ s[2] ^ s[3]  → positions 0,3,4 relative to input
// G1: u ^ s[0] ^ s[2] ^ s[3] → but s[0] is the PREVIOUS input, so this is delay 1
//
// Actually, with the register shifting convention:
// At time n, after shifting u into s[0]:
//   s[0] = u[n], s[1] = u[n-1], s[2] = u[n-2], s[3] = u[n-3]
// G0 = D^0 + D^3 + D^4 → u[n] + u[n-3] + u[n-4]
//   But u[n-4] is the bit that was just shifted out!
//
// Better approach: use state = (u[n-1], u[n-2], u[n-3], u[n-4]), 4 bits = 16 states.
// For input u[n]:
//   c0 = u[n] ^ u[n-3] ^ u[n-4]  = u[n] ^ state_bit1 ^ state_bit0
//   c1 = u[n] ^ u[n-1] ^ u[n-3] ^ u[n-4] = u[n] ^ state_bit3 ^ state_bit1 ^ state_bit0
//
// State encoding: state = (u[n-1]<<3 | u[n-2]<<2 | u[n-3]<<1 | u[n-4]<<0)
//   state_bit3 = u[n-1], state_bit2 = u[n-2], state_bit1 = u[n-3], state_bit0 = u[n-4]
// New state after input u[n]:
//   new_state = (u[n]<<3) | (state >> 1)

// Precomputed output table: gsm_conv_output[state][input] = (c0 << 1) | c1
static uint8_t gsm_conv_output[GSM_CONV_STATES][2];
static uint8_t gsm_conv_next_state[GSM_CONV_STATES][2];
static bool    gsm_conv_tables_init = false;

static void init_conv_tables(void)
{
    if (gsm_conv_tables_init) return;

    for (int32_t state = 0; state < GSM_CONV_STATES; state++) {
        for (int32_t input = 0; input < 2; input++) {
            // State bits: s3=u[n-1], s2=u[n-2], s1=u[n-3], s0=u[n-4]
            int32_t s3 = (state >> 3) & 1;
            int32_t s1 = (state >> 1) & 1;
            int32_t s0 = (state >> 0) & 1;

            // G0 = u[n] + u[n-3] + u[n-4] = input ^ s1 ^ s0
            int32_t c0 = input ^ s1 ^ s0;
            // G1 = u[n] + u[n-1] + u[n-3] + u[n-4] = input ^ s3 ^ s1 ^ s0
            int32_t c1 = input ^ s3 ^ s1 ^ s0;

            gsm_conv_output[state][input] = (uint8_t)((c0 << 1) | c1);

            // New state: (input << 3) | (state >> 1)
            gsm_conv_next_state[state][input] = (uint8_t)((input << 3) | (state >> 1));
        }
    }
    gsm_conv_tables_init = true;
}

// Convolutional encoder
void gsm_conv_encode(const uint8_t *input, int32_t n, uint8_t *output)
{
    init_conv_tables();
    int32_t state = 0;
    for (int32_t i = 0; i < n; i++) {
        int32_t u = input[i] & 1;
        uint8_t out = gsm_conv_output[state][u];
        output[2*i]     = (out >> 1) & 1; // c0
        output[2*i + 1] = out & 1;        // c1
        state = gsm_conv_next_state[state][u];
    }
}

// Viterbi decoder (rate 1/2, K=5)
// soft_input: pairs of soft values (c0, c1), total 2*n_output_bits values.
//   Positive = more likely 1, negative = more likely 0.
//   (Convention: transmitted bit 0 → soft > 0, bit 1 → soft < 0)
//   Wait — we need a consistent convention. Let's use:
//   soft > 0 means the transmitted coded bit was 0
//   soft < 0 means the transmitted coded bit was 1
// output: decoded bits, n_output_bits long.
// Returns accumulated path metric of best path.
int32_t gsm_viterbi_decode(const float *soft_input, int32_t n_output_bits, uint8_t *output)
{
    init_conv_tables();

    // Path metrics (use int for speed, quantize soft values)
    #define VITERBI_SCALE 16
    int32_t path_metric[GSM_CONV_STATES];
    int32_t new_metric[GSM_CONV_STATES];

    // Traceback memory
    int32_t n = n_output_bits;
    uint8_t *traceback = (uint8_t *)calloc((size_t)n * GSM_CONV_STATES, sizeof(uint8_t));
    if (!traceback) return -1;

    // Initialize: start in state 0 (all-zero register)
    for (int32_t s = 0; s < GSM_CONV_STATES; s++)
        path_metric[s] = (s == 0) ? 0 : 1000000;

    for (int32_t i = 0; i < n; i++) {
        // Received soft values for this step
        int32_t r0 = (int32_t)(soft_input[2*i]     * VITERBI_SCALE);
        int32_t r1 = (int32_t)(soft_input[2*i + 1] * VITERBI_SCALE);

        for (int32_t s = 0; s < GSM_CONV_STATES; s++)
            new_metric[s] = 1000000;

        for (int32_t s = 0; s < GSM_CONV_STATES; s++) {
            if (path_metric[s] >= 999999) continue;

            for (int32_t u = 0; u < 2; u++) {
                uint8_t coded_out = gsm_conv_output[s][u];
                int32_t c0 = (coded_out >> 1) & 1;
                int32_t c1 = coded_out & 1;

                // Branch metric: distance between received and expected
                // Expected soft for coded bit 0 → +VITERBI_SCALE, for bit 1 → -VITERBI_SCALE
                int32_t e0 = c0 ? -VITERBI_SCALE : VITERBI_SCALE;
                int32_t e1 = c1 ? -VITERBI_SCALE : VITERBI_SCALE;

                int32_t bm = abs(r0 - e0) + abs(r1 - e1);
                int32_t candidate = path_metric[s] + bm;

                int32_t ns = gsm_conv_next_state[s][u];
                if (candidate < new_metric[ns]) {
                    new_metric[ns] = candidate;
                    traceback[i * GSM_CONV_STATES + ns] = (uint8_t)s;
                }
            }
        }

        memcpy(path_metric, new_metric, sizeof(path_metric));
    }

    // Force final state to 0 (tail bits drive encoder to state 0)
    int32_t best_state = 0;
    int32_t best_metric = path_metric[0];

    // Traceback
    int32_t state = best_state;
    for (int32_t i = n - 1; i >= 0; i--) {
        int32_t prev_state = traceback[i * GSM_CONV_STATES + state];
        // The input bit that caused transition from prev_state to state
        // is the MSB of state (since new_state = (input<<3) | (prev>>1))
        output[i] = (uint8_t)((state >> 3) & 1);
        state = prev_state;
    }

    free(traceback);
    return best_metric;
    #undef VITERBI_SCALE
}

// ======================== Fire Code (TS 05.03 section 4.1) ========================

// Fire code generator polynomial:
// g(x) = (x^23 + 1)(x^17 + x^3 + 1)
// = x^40 + x^26 + x^23 + x^17 + x^3 + 1
//
// This is used for BCCH, CCCH, SACCH, SDCCH.
// Input: 184 data bits. Output: 40 parity bits.

// Generator polynomial as a 41-bit value (MSB = x^40):
// Bit positions: 40, 26, 23, 17, 3, 0
static const uint64_t FIRE_POLY = (1ULL << 40) | (1ULL << 26) | (1ULL << 23) |
                                   (1ULL << 17) | (1ULL << 3)  | (1ULL << 0);

void gsm_fire_encode(const uint8_t *data, int32_t n_bits, uint8_t *parity)
{
    // Compute remainder of data * x^40 / g(x)
    // Using bit-serial division
    uint64_t reg = 0;

    for (int32_t i = 0; i < n_bits; i++) {
        uint64_t feedback = ((reg >> 39) ^ data[i]) & 1;
        reg = (reg << 1) & ((1ULL << 40) - 1);
        if (feedback) {
            reg ^= FIRE_POLY & ((1ULL << 40) - 1); // XOR with lower 40 bits of poly
        }
    }

    // Output parity bits (MSB first)
    for (int32_t i = 0; i < 40; i++) {
        parity[i] = (uint8_t)((reg >> (39 - i)) & 1);
    }
}

bool gsm_fire_check(const uint8_t *data_with_parity, int32_t total_bits)
{
    // Check: remainder of entire codeword / g(x) should be 0
    uint64_t reg = 0;

    for (int32_t i = 0; i < total_bits; i++) {
        uint64_t feedback = ((reg >> 39) ^ data_with_parity[i]) & 1;
        reg = (reg << 1) & ((1ULL << 40) - 1);
        if (feedback) {
            reg ^= FIRE_POLY & ((1ULL << 40) - 1);
        }
    }

    return (reg == 0);
}

// ======================== SCH CRC (TS 05.03 section 4.7) ========================

// SCH uses a shortened cyclic code with generator polynomial:
// g(x) = x^10 + x^8 + x^6 + x^5 + x^4 + x^2 + x + 1
// Bit pattern: 10101110111 = 0x577

#define SCH_CRC_POLY 0x0175  // g(x)=x^10+x^8+x^6+x^5+x^4+x^2+1, matching osmocom gsm0503_sch_crc10
#define SCH_CRC_REMAINDER 0x3FF  // final XOR mask per osmocom convention

static uint16_t sch_crc_compute(const uint8_t *data, int32_t n_bits)
{
    uint16_t reg = 0;
    for (int32_t i = 0; i < n_bits; i++) {
        uint16_t feedback = ((reg >> 9) ^ data[i]) & 1;
        reg = (reg << 1) & 0x3FF;
        if (feedback) {
            reg ^= SCH_CRC_POLY;
        }
    }
    return reg;
}

static void sch_crc_encode(const uint8_t *data, int32_t n_bits, uint8_t *parity)
{
    uint16_t crc = sch_crc_compute(data, n_bits) ^ SCH_CRC_REMAINDER;
    for (int32_t i = 0; i < 10; i++) {
        parity[i] = (uint8_t)((crc >> (9 - i)) & 1);
    }
}

static bool sch_crc_check(const uint8_t *data_with_parity, int32_t total_bits)
{
    /* Recompute CRC from data, apply remainder XOR, compare with parity */
    int32_t data_bits = total_bits - 10;
    uint16_t expected = sch_crc_compute(data_with_parity, data_bits) ^ SCH_CRC_REMAINDER;
    uint16_t received = 0;
    for (int32_t i = 0; i < 10; i++)
        received = (received << 1) | (data_with_parity[data_bits + i] & 1);
    return (expected == received);
}

// ======================== SCH Encode/Decode (TS 05.03 section 4.7) ========================

void gsm_sch_encode(const uint8_t *info, uint8_t *coded)
{
    uint8_t pre_conv[GSM_SCH_PRE_CONV]; // 39 bits

    // Copy 25 info bits
    memcpy(pre_conv, info, 25);

    // Add 10 CRC bits (with remainder XOR per GSM convention)
    sch_crc_encode(info, 25, &pre_conv[25]);

    // Add 4 tail bits (zeros)
    pre_conv[35] = 0; pre_conv[36] = 0; pre_conv[37] = 0; pre_conv[38] = 0;

    // Convolutional encode (rate 1/2) → 78 coded bits
    gsm_conv_encode(pre_conv, GSM_SCH_PRE_CONV, coded);
}

bool gsm_sch_decode(const float *soft_input, uint8_t *info_out)
{
    /* Self-test on first call */
    static bool selftest_done = false;
    if (!selftest_done) {
        selftest_done = true;
        uint8_t test_info[25] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1};
        uint8_t test_coded[78];
        gsm_sch_encode(test_info, test_coded);
        float test_soft[78];
        for (int32_t i = 0; i < 78; i++) test_soft[i] = test_coded[i] ? -1.0f : 1.0f;
        uint8_t test_dec[25];
        bool ok = gsm_sch_decode(test_soft, test_dec);
        int32_t match = 1;
        if (ok) { for (int32_t i = 0; i < 25; i++) if (test_dec[i] != test_info[i]) match = 0; }
        if (!ok || !match)
            fprintf(stderr, "GSM SCH self-test: encode/decode %s, match=%d\n", ok ? "PASS" : "FAIL", match);
    }

    // Viterbi decode: 78 soft bits -> 39 pre-conv bits
    uint8_t pre_conv[GSM_SCH_PRE_CONV];
    gsm_viterbi_decode(soft_input, GSM_SCH_PRE_CONV, pre_conv);

    // Check CRC (25 data + 10 parity = 35 bits)
    if (!sch_crc_check(pre_conv, 35)) {
        uint16_t exp = sch_crc_compute(pre_conv, 25) ^ SCH_CRC_REMAINDER;
        uint16_t got = 0;
        for (int32_t i = 0; i < 10; i++) got = (got << 1) | (pre_conv[25+i] & 1);
        static int32_t crc_dbg = 0;
        if (++crc_dbg <= 10) {
            fprintf(stderr, "GSM SCH CRC fail: exp=0x%03X got=0x%03X bits=", exp, got);
            for (int32_t i = 0; i < 39; i++) fprintf(stderr, "%d", pre_conv[i]);
            fprintf(stderr, "\n");
        }
        return false;
    }

    // Output 25 info bits
    memcpy(info_out, pre_conv, 25);
    return true;
}

// ======================== SCH Info Bits (TS 05.02, section 3.3.2.2.1) ========================

// SCH info = 25 bits encoding BSIC (6 bits) and reduced frame number:
//   t1 (11 bits) = FN div (26*51)     → superframe number
//   t2 (5 bits)  = FN mod 26          → frame in traffic multiframe
//   t3' (3 bits) = (FN mod 51 - 1) / 10  → block in control multiframe
//
// Bit ordering: t1[0..10], t2[0..4], t3'[0..2], BSIC[0..5]
// Wait — per the spec, it's: BSIC is encoded as NCC(3 bits) || BCC(3 bits).
//
// Actually from TS 05.02:
// The 25-bit SCH information is: t1(11) || t2(5) || t3'(3) || BSIC(6)
// But the exact bit ordering... the standard says d(0)..d(24).

void gsm_sch_parse(const uint8_t *info, uint8_t *bsic, uint32_t *fn)
{
    // d(0)..d(24):
    // t1: d(0)..d(10)  → 11 bits MSB first
    // t2: d(11)..d(15) → 5 bits
    // t3': d(16)..d(18) → 3 bits
    // BSIC: d(19)..d(24) → 6 bits
    uint32_t t1 = 0;
    for (int32_t i = 0; i < 11; i++) t1 = (t1 << 1) | (info[i] & 1);

    uint32_t t2 = 0;
    for (int32_t i = 11; i < 16; i++) t2 = (t2 << 1) | (info[i] & 1);

    uint32_t t3p = 0;
    for (int32_t i = 16; i < 19; i++) t3p = (t3p << 1) | (info[i] & 1);

    uint32_t b = 0;
    for (int32_t i = 19; i < 25; i++) b = (b << 1) | (info[i] & 1);

    uint32_t t3 = 10 * t3p + 1; // recover t3 from t3'
    // TS 05.02: FN = 51 * ((t3 - t2) mod 26) + t3 + 51*26*t1
    *fn = 51 * (((t3 - t2) % 26 + 26) % 26) + t3 + 1326 * t1;
    *bsic = (uint8_t)b;
}

void gsm_sch_build(uint8_t bsic, uint32_t fn, uint8_t *info)
{
    uint32_t t1 = fn / (26 * 51);
    uint32_t t3 = fn % 51;
    uint32_t t2 = fn % 26;
    uint32_t t3p = (t3 - 1) / 10; // valid only when t3 = 1,11,21,31,41

    // t1: 11 bits MSB first
    for (int32_t i = 10; i >= 0; i--) { info[10-i] = (uint8_t)((t1 >> i) & 1); }
    // t2: 5 bits
    for (int32_t i = 4; i >= 0; i--) { info[15-i] = (uint8_t)((t2 >> i) & 1); }
    // t3': 3 bits
    for (int32_t i = 2; i >= 0; i--) { info[18-i] = (uint8_t)((t3p >> i) & 1); }
    // BSIC: 6 bits
    for (int32_t i = 5; i >= 0; i--) { info[24-i] = (uint8_t)((bsic >> i) & 1); }
}

// ======================== BCCH Interleaving (TS 05.03, section 4.1) ========================

// Block diagonal interleaving over 4 bursts.
// 456 coded bits → 4 bursts × 114 bits.
// Mapping: bit j of coded output → burst (j mod 4), position (j/4) within burst.
// Wait — the exact mapping from TS 05.03:
// For TCH/FS and control channels with diagonal interleaving:
//   The 456 coded bits c(0)..c(455) are mapped to 4 bursts B(0)..B(3):
//   Burst B(k) carries bits: e(k, j) for j=0..113
//   where the interleaving rule is:
//   e(B, j) = c(k) where k = j + B*114... no wait.
//
// Actually for BCCH/CCCH (TS 05.03 section 4.1, table 2):
// The 456 coded bits are reordered and distributed as:
// i(B, j) = c(n, k) where B = block number 0-3, j = bit position 0-113
// The interleaving is rectangular (not diagonal) for BCCH:
// c(k) → burst k mod 4, position k/4 (integer division)
//
// More precisely, for BCCH/CCCH/SACCH:
// c(0)..c(455) → burst 0: c(0),c(4),c(8),...,c(452)
//                 burst 1: c(1),c(5),c(9),...,c(453)
//                 burst 2: c(2),c(6),c(10),...,c(454)
//                 burst 3: c(3),c(7),c(11),...,c(455)

void gsm_bcch_interleave(const uint8_t *coded_bits, uint8_t burst_bits[4][GSM_NB_DATA_BITS])
{
    for (int32_t k = 0; k < GSM_BCCH_CODED_BITS; k++) {
        int32_t burst = k % 4;
        int32_t pos   = k / 4;
        burst_bits[burst][pos] = coded_bits[k];
    }
}

void gsm_bcch_deinterleave(const float burst_soft[4][GSM_NB_DATA_BITS], float *coded_soft)
{
    for (int32_t k = 0; k < GSM_BCCH_CODED_BITS; k++) {
        int32_t burst = k % 4;
        int32_t pos   = k / 4;
        coded_soft[k] = burst_soft[burst][pos];
    }
}

// ======================== BCCH Decode ========================

// Full BCCH decode: 4 bursts of soft bits → 23-octet L2 frame.
// Returns true if Fire code CRC passes.
static bool decode_bcch_block(const float burst_soft[4][GSM_NB_DATA_BITS],
                              uint8_t *l2_frame)
{
    // Step 1: Deinterleave — 4 × 114 soft bits → 456 coded soft bits
    float coded_soft[GSM_BCCH_CODED_BITS];
    gsm_bcch_deinterleave(burst_soft, coded_soft);

    // Step 2: Viterbi decode — 456 soft bits (228 pairs) → 228 bits
    uint8_t pre_conv[GSM_BCCH_PRE_CONV]; // 228 bits
    gsm_viterbi_decode(coded_soft, GSM_BCCH_PRE_CONV, pre_conv);

    // Step 3: Check Fire code CRC — first 224 bits (184 data + 40 parity)
    if (!gsm_fire_check(pre_conv, GSM_BCCH_L2_BITS + GSM_BCCH_PARITY_BITS)) {
        return false;
    }

    // Step 4: Extract 184 data bits → 23 octets
    for (int32_t i = 0; i < GSM_L2_FRAME_LEN; i++) {
        uint8_t byte = 0;
        for (int32_t b = 0; b < 8; b++) {
            byte = (uint8_t)((byte << 1) | (pre_conv[i * 8 + b] & 1));
        }
        l2_frame[i] = byte;
    }

    return true;
}

// ======================== Normal Burst Data Extraction ========================

// Extract 114 data bits from a Normal Burst (skip tail, training, stealing).
// Normal Burst structure (TS 05.02, section 5.2.3):
//   3 tail | 57 data | 1 steal | 26 training | 1 steal | 57 data | 3 tail | 8.25 guard
// The 114 data bits: bits[3..59] (57 bits) + bits[88..144] (57 bits)
static void extract_nb_data(const float *burst_soft, float *data_soft)
{
    // First 57 data bits: positions 3..59
    memcpy(data_soft, &burst_soft[3], 57 * sizeof(float));
    // Second 57 data bits: positions 88..144
    memcpy(&data_soft[57], &burst_soft[88], 57 * sizeof(float));
}

// ======================== L2 LAPDm Parsing (TS 04.06) ========================

// LAPDm frame format (23 octets):
//   Octet 1: Address field — SAPI(3) EA(1) C/R(1) spare(3)
//            Actually: [b7..b0] = LPD(2) SAPI(3) C/R(1) EA(1) spare(1)
//            Simplified for Bm channels: SAPI in bits 5-3, C/R in bit 1
//   Octet 2: Control field — frame type
//   Octet 3: Length indicator — length(6) M(1) EL(1)
//            Actually: [b7..b0] = L(6) M(1) EL(1)
//   Octets 4..4+L-1: Information field (L3 data)
//   Remaining: Fill (0x2B)

bool gsm_l2_parse(const uint8_t *data, int32_t len, gsm_l2_frame_t *frame)
{
    if (len < 3) return false;

    memset(frame, 0, sizeof(*frame));
    memcpy(frame->data, data, len > GSM_L2_FRAME_LEN ? GSM_L2_FRAME_LEN : len);

    // Address field (octet 1)
    frame->sapi = (data[0] >> 2) & 0x07; // SAPI bits 4-2
    frame->cr   = (data[0] >> 1) & 0x01; // C/R bit 1

    // Control field (octet 2)
    frame->frame_type = data[1];

    // Length indicator (octet 3)
    frame->length = (data[2] >> 2) & 0x3F; // L bits 7-2
    frame->more   = (data[2] >> 1) & 0x01; // M bit 1

    // Sanity check
    if (frame->length > 20) frame->length = 20; // max info field is 20 octets
    if (3 + frame->length > len) return false;

    return true;
}

// ======================== L3 System Information Parsing (TS 04.08) ========================

// L3 message header:
//   Octet 1: PD(4) | skip_indicator(4)  — PD=0x06 for RR
//   Octet 2: Message Type
// For SI on BCCH, there's a pseudo-header:
//   Octet 1: L2 pseudo length
//   Octet 2: Protocol Discriminator (0x06 for RR)
//   Octet 3: Message Type

// Decode BCD digit
static inline uint8_t bcd_lo(uint8_t b) { return b & 0x0F; }
static inline uint8_t bcd_hi(uint8_t b) { return (b >> 4) & 0x0F; }

// Decode LAI (Location Area Identification) — 5 octets (TS 04.08, 10.5.1.3)
static void decode_lai(const uint8_t *data, uint16_t *mcc, uint16_t *mnc, uint16_t *lac)
{
    // Octet 1: MCC digit 2 (hi) | MCC digit 1 (lo)
    // Octet 2: MNC digit 3 (hi) | MCC digit 3 (lo)
    // Octet 3: MNC digit 2 (hi) | MNC digit 1 (lo)
    // Octets 4-5: LAC (big-endian)
    uint8_t mcc1 = bcd_lo(data[0]);
    uint8_t mcc2 = bcd_hi(data[0]);
    uint8_t mcc3 = bcd_lo(data[1]);
    *mcc = (uint16_t)(mcc1 * 100 + mcc2 * 10 + mcc3);

    uint8_t mnc3 = bcd_hi(data[1]);
    uint8_t mnc1 = bcd_lo(data[2]);
    uint8_t mnc2 = bcd_hi(data[2]);

    if (mnc3 == 0x0F) {
        // 2-digit MNC
        *mnc = (uint16_t)(mnc1 * 10 + mnc2);
    } else {
        *mnc = (uint16_t)(mnc1 * 100 + mnc2 * 10 + mnc3);
    }

    *lac = (uint16_t)((data[3] << 8) | data[4]);
}

static int32_t decode_bitmap0_arfcn_list(const uint8_t *desc, uint16_t *list, int32_t max_list)
{
    int32_t count = 0;
    int32_t arfcn = 1;

    for (int32_t byte_idx = 0; byte_idx < 16 && arfcn <= 124; byte_idx++) {
        int32_t start_bit = (byte_idx == 0) ? 5 : 7;
        for (int32_t bit = start_bit; bit >= 0 && arfcn <= 124; bit--, arfcn++) {
            if ((desc[byte_idx] & (1u << bit)) && count < max_list) {
                list[count++] = (uint16_t)arfcn;
            }
        }
    }

    return count;
}

static void decode_rach_control(const uint8_t *data, uint8_t *max_retrans,
                                uint8_t *tx_integer, bool *cell_barred,
                                bool *re_not_allowed, uint16_t *ac_class)
{
    *max_retrans = (data[0] >> 6) & 0x03;
    *tx_integer = (data[0] >> 4) & 0x03;
    *cell_barred = ((data[0] >> 3) & 0x01) != 0;
    *re_not_allowed = ((data[0] >> 2) & 0x01) != 0;
    *ac_class = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
}

// SI3 Message Type = 0x1B
bool gsm_parse_si3(const uint8_t *l3, int32_t len, gsm_si3_t *out)
{
    memset(out, 0, sizeof(*out));

    // SI3 on BCCH has a pseudo-header:
    // Octet 0: L2 Pseudo Length (includes: skip_ind + PD + msg_type + IEs)
    // Octet 1: Skip Indicator (hi nibble) | Protocol Discriminator (lo nibble)
    // Octet 2: Message Type

    if (len < 18) return false;

    // Check PD = 0x06 (RR)
    if ((l3[0] & 0x0F) != 0x06) return false;
    // Check message type = 0x1B (SI3)
    if (l3[1] != 0x1B) return false;

    // Cell Identity: octets 2-3 (2 bytes, big-endian)
    out->cell_id = (uint16_t)((l3[2] << 8) | l3[3]);

    // LAI: octets 4-8 (5 bytes)
    decode_lai(&l3[4], &out->mcc, &out->mnc, &out->lac);

    // Control Channel Description: octets 9-11 (3 bytes, TS 44.018 10.5.2.11)
    out->ccch_conf       = l3[9] & 0x07;
    out->bs_ag_blks_res  = (l3[9] >> 3) & 0x07;
    out->bs_pa_mfrms     = (uint8_t)(((l3[10] >> 4) & 0x07) + 2);
    out->t3212           = l3[11];

    // Cell Options (BCCH): octet 12 (TS 44.018 10.5.2.3)
    out->radio_link_timeout = l3[12] & 0x0F;
    out->dtx                = (l3[12] >> 4) & 0x03;
    out->pwrc               = ((l3[12] >> 6) & 0x01) != 0;

    // Cell Selection Parameters: octets 13-14 (TS 44.018 10.5.2.4)
    out->ms_txpwr_max_cch         = l3[13] & 0x1F;
    out->cell_reselect_hysteresis = (l3[13] >> 5) & 0x07;
    out->rxlev_access_min         = l3[14] & 0x3F;

    // RACH Control Parameters: octets 15-17 (TS 44.018 10.5.2.29)
    decode_rach_control(&l3[15], &out->max_retrans, &out->tx_integer,
                        &out->cell_barred, &out->re_not_allowed, &out->ac_class);

    out->valid = true;
    return true;
}

// SI1 Message Type = 0x19 — Cell Channel Description (Cell Allocation)
bool gsm_parse_si1(const uint8_t *l3, int32_t len, gsm_si1_t *out)
{
    memset(out, 0, sizeof(*out));

    if (len < 19) return false;
    if ((l3[0] & 0x0F) != 0x06) return false;
    if (l3[1] != 0x19) return false;

    // Cell Channel Description: octets 2-17 (16 bytes)
    // Format: bitmap or variable-length frequency list
    // Bit 0 of octet 2 is the format ID
    const uint8_t *ca = &l3[2];

    // Bitmap 0 format: format identifier bits 8..7 of octet 3 are 00.
    if ((ca[0] & 0xC0) == 0x00) {
        out->n_arfcn = decode_bitmap0_arfcn_list(ca, out->arfcn_list, 64);
    }
    // Other formats (range 1024, range 512, etc.) — TODO for extended ARFCNs

    // RACH Control Parameters: octets 18-20
    // (same as in SI3, but we skip here for brevity)

    out->valid = true;
    return true;
}

// SI2 Message Type = 0x1A — Neighbour Cell Description
bool gsm_parse_si2(const uint8_t *l3, int32_t len, gsm_si2_t *out)
{
    memset(out, 0, sizeof(*out));

    if (len < 19) return false;
    if ((l3[0] & 0x0F) != 0x06) return false;
    if (l3[1] != 0x1A) return false;

    // Neighbour Cell Description: octets 2-17 (16 bytes)
    // Same bitmap format as SI1 Cell Channel Description
    const uint8_t *ba = &l3[2];

    if ((ba[0] & 0xC0) == 0x00) {
        out->n_neighbours = decode_bitmap0_arfcn_list(ba, out->neighbour_arfcn,
                                                      GSM_MAX_NEIGHBOURS);
    }

    // NCC Permitted: octet 18
    if (len > 18) {
        out->ncc_permitted = l3[18];
    }

    out->valid = true;
    return true;
}

// SI4 Message Type = 0x1C — LAI + Cell Selection + CBCH
bool gsm_parse_si4(const uint8_t *l3, int32_t len, gsm_si4_t *out)
{
    memset(out, 0, sizeof(*out));

    if (len < 12) return false;
    if ((l3[0] & 0x0F) != 0x06) return false;
    if (l3[1] != 0x1C) return false;

    // LAI: octets 2-6
    decode_lai(&l3[2], &out->mcc, &out->mnc, &out->lac);

    // Cell Selection Parameters: octets 7-8
    // (same format as SI3)

    // RACH Control Parameters: octets 9-11
    // (same format as SI3)

    // Optional: CBCH Channel Description (TLV/TV, IEI=0x64)
    int32_t pos = 12;
    while (pos < len - 1) {
        uint8_t iei = l3[pos];
        if (iei == 0x64 && pos + 4 <= len) {
            // CBCH Channel Description (4 octets value)
            out->cbch_present = true;
            out->cbch_ts = l3[pos + 1] & 0x07;
            // ARFCN: bits from octets pos+2..pos+3 if hopping not used
            if (!(l3[pos + 1] & 0x10)) { // H bit = 0 → no hopping
                out->cbch_arfcn = (uint16_t)(((l3[pos + 2] & 0x03) << 8) | l3[pos + 3]);
            }
            break;
        }
        pos++; // skip unknown IEIs
    }

    out->valid = true;
    return true;
}

// ======================== Cell Broadcast (TS 23.041) ========================

// CB message structure on CBCH:
// 4 SDCCH/4 blocks → 4 × 23 octets = 92 octets per page
// Header:
//   Octets 0-1: Serial Number
//   Octets 2-3: Message Identifier
//   Octet 4: Data Coding Scheme
//   Octet 5: Page Parameter (total_pages in hi, page_nr in lo)
//   Octets 6-87: Content (82 octets)

bool gsm_parse_cb(const uint8_t *data, int32_t len, gsm_cb_msg_t *out)
{
    memset(out, 0, sizeof(*out));

    if (len < 6) return false;

    out->serial_nr   = (uint16_t)((data[0] << 8) | data[1]);
    out->msg_id      = (uint16_t)((data[2] << 8) | data[3]);
    out->dcs         = data[4];
    out->page_param  = data[5];
    out->total_pages = (data[5] >> 4) & 0x0F;
    out->page_nr     = data[5] & 0x0F;

    // Decode text content
    int32_t content_len = len - 6;
    if (content_len > GSM_CB_PAGE_LEN) content_len = GSM_CB_PAGE_LEN;

    // Data Coding Scheme determines encoding
    uint8_t coding_group = (out->dcs >> 4) & 0x0F;

    if (coding_group == 0x00 || coding_group == 0x01) {
        // Default 7-bit GSM alphabet
        // GSM 7-bit packing: characters are packed LSB-first into octets
        int32_t out_pos = 0;
        int32_t bit_pos = 0;
        const uint8_t *src = &data[6];
        int32_t max_chars = (content_len * 8) / 7;

        for (int32_t n = 0; n < max_chars && out_pos < GSM_CB_PAGE_LEN; n++) {
            int32_t byte_off = bit_pos / 8;
            int32_t bit_off = bit_pos % 8;
            if (byte_off >= content_len) break;

            uint8_t ch = (uint8_t)(src[byte_off] >> bit_off);
            if (bit_off > 1 && byte_off + 1 < content_len)
                ch |= (uint8_t)(src[byte_off + 1] << (8 - bit_off));
            ch &= 0x7F;

            // Simple GSM default alphabet → ASCII subset mapping
            if (ch == 0) ch = '@';
            else if (ch < 0x20) ch = '?';
            else if (ch == 0x7F) ch = '?';

            out->text[out_pos++] = (char)ch;
            bit_pos += 7;
        }
        out->text_len = out_pos;
    } else {
        // 8-bit or UCS-2: just copy as-is for now (truncate to ASCII)
        for (int32_t i = 0; i < content_len && i < GSM_CB_PAGE_LEN; i++) {
            uint8_t ch = data[6 + i];
            out->text[i] = (ch >= 0x20 && ch < 0x7F) ? (char)ch : '.';
        }
        out->text_len = content_len;
    }
    out->text[out->text_len] = '\0';

    out->valid = true;
    return true;
}

// ======================== Paging (TS 04.08, section 9.1.22) ========================

// Paging Request Type 1 (msg type 0x21)
// Paging Request Type 2 (msg type 0x22)
// Paging Request Type 3 (msg type 0x24)

bool gsm_parse_paging(const uint8_t *l3, int32_t len, gsm_paging_t *out)
{
    memset(out, 0, sizeof(*out));

    if (len < 4) return false;
    if ((l3[0] & 0x0F) != 0x06) return false;

    uint8_t msg_type = l3[1];

    if (msg_type == 0x21) {
        // Paging Request Type 1
        out->paging_type = 1;

        // Channel Needed: l3[2] bits 3-0 (2 bits per mobile)
        out->channel_needed[0] = (l3[2] >> 0) & 0x03;
        out->channel_needed[1] = (l3[2] >> 2) & 0x03;

        // Mobile Identity 1: LV starting at octet 3
        if (len > 4) {
            uint8_t mi_len = l3[3];
            uint8_t mi_type = l3[4] & 0x07;
            if (mi_type == 4 && mi_len == 5 && len >= 3 + 1 + 5) {
                // TMSI
                out->tmsi[0] = (uint32_t)((l3[5] << 24) | (l3[6] << 16) | (l3[7] << 8) | l3[8]);
                out->n_identities = 1;
            }

            // Mobile Identity 2 (optional, TLV with IEI=0x17)
            int32_t pos = 3 + 1 + mi_len;
            if (pos + 2 < len && l3[pos] == 0x17) {
                uint8_t mi2_len = l3[pos + 1];
                uint8_t mi2_type = l3[pos + 2] & 0x07;
                if (mi2_type == 4 && mi2_len == 5 && pos + 2 + 5 <= len) {
                    out->tmsi[1] = (uint32_t)((l3[pos+3] << 24) | (l3[pos+4] << 16) |
                                               (l3[pos+5] << 8) | l3[pos+6]);
                    out->n_identities = 2;
                }
            }
        }
    } else if (msg_type == 0x22) {
        // Paging Request Type 2 — up to 3 TMSIs
        out->paging_type = 2;
        if (len >= 11) {
            out->tmsi[0] = (uint32_t)((l3[3] << 24) | (l3[4] << 16) | (l3[5] << 8) | l3[6]);
            out->tmsi[1] = (uint32_t)((l3[7] << 24) | (l3[8] << 16) | (l3[9] << 8) | l3[10]);
            out->n_identities = 2;
            // Third identity is optional (Mobile Identity LV at octet 11)
        }
    } else if (msg_type == 0x24) {
        // Paging Request Type 3 — 4 TMSIs
        out->paging_type = 3;
        if (len >= 19) {
            for (int32_t i = 0; i < 4 && 3 + i*4 + 3 < len; i++) {
                int32_t off = 3 + i * 4;
                out->tmsi[i] = (uint32_t)((l3[off] << 24) | (l3[off+1] << 16) |
                                           (l3[off+2] << 8) | l3[off+3]);
            }
            out->n_identities = 4;
        }
    } else {
        return false;
    }

    out->valid = true;
    return true;
}

// ======================== ARFCN ↔ Frequency Conversion ========================

double gsm_arfcn_to_freq(uint16_t arfcn, gsm_band_t band)
{
    switch (band) {
    case GSM_BAND_900:
    case GSM_BAND_E900:
        // P-GSM 900: ARFCN 1-124  → DL = 935 + 0.2*(ARFCN-0)
        // E-GSM 900: ARFCN 975-1023 → DL = 935 + 0.2*(ARFCN-1024)
        //            ARFCN 0 → DL = 935.0
        if (arfcn == 0) return 935.0;
        if (arfcn >= 1 && arfcn <= 124)
            return 935.0 + 0.2 * arfcn;
        if (arfcn >= 975 && arfcn <= 1023)
            return 935.0 + 0.2 * ((int32_t)arfcn - 1024);
        return 0;

    case GSM_BAND_1800:
        // DCS 1800: ARFCN 512-885 → DL = 1805.2 + 0.2*(ARFCN-512)
        if (arfcn >= 512 && arfcn <= 885)
            return 1805.2 + 0.2 * ((int32_t)arfcn - 512);
        return 0;

    case GSM_BAND_850:
        // GSM 850: ARFCN 128-251 → DL = 869.2 + 0.2*(ARFCN-128)
        if (arfcn >= 128 && arfcn <= 251)
            return 869.2 + 0.2 * ((int32_t)arfcn - 128);
        return 0;

    case GSM_BAND_1900:
        // PCS 1900: ARFCN 512-810 → DL = 1930.2 + 0.2*(ARFCN-512)
        if (arfcn >= 512 && arfcn <= 810)
            return 1930.2 + 0.2 * ((int32_t)arfcn - 512);
        return 0;

    default:
        return 0;
    }
}

uint16_t gsm_freq_to_arfcn(double freq_mhz, gsm_band_t *band)
{
    // Try each band
    // P-GSM 900: DL 935.2 - 959.8
    if (freq_mhz >= 935.0 && freq_mhz <= 960.0) {
        int32_t arfcn = (int32_t)((freq_mhz - 935.0) / 0.2 + 0.5);
        if (arfcn >= 0 && arfcn <= 124) {
            if (band) *band = GSM_BAND_900;
            return (uint16_t)arfcn;
        }
    }
    // E-GSM 900: DL 925.2 - 935.0 → ARFCN 975-1023
    if (freq_mhz >= 925.0 && freq_mhz < 935.0) {
        int32_t arfcn = (int32_t)((freq_mhz - 935.0) / 0.2 + 1024.5);
        if (arfcn >= 975 && arfcn <= 1023) {
            if (band) *band = GSM_BAND_E900;
            return (uint16_t)arfcn;
        }
    }
    // DCS 1800: DL 1805.2 - 1879.8
    if (freq_mhz >= 1805.0 && freq_mhz <= 1880.0) {
        int32_t arfcn = (int32_t)((freq_mhz - 1805.2) / 0.2 + 512.5);
        if (arfcn >= 512 && arfcn <= 885) {
            if (band) *band = GSM_BAND_1800;
            return (uint16_t)arfcn;
        }
    }
    // GSM 850: DL 869.2 - 893.8
    if (freq_mhz >= 869.0 && freq_mhz <= 894.0) {
        int32_t arfcn = (int32_t)((freq_mhz - 869.2) / 0.2 + 128.5);
        if (arfcn >= 128 && arfcn <= 251) {
            if (band) *band = GSM_BAND_850;
            return (uint16_t)arfcn;
        }
    }

    if (band) *band = GSM_BAND_UNKNOWN;
    return 0xFFFF;
}

// ======================== 51-Multiframe Channel Mapping ========================

// TS 05.02, Table 5 of section 6.3.1.3 — Channel combination v
// FCCH+SCH+BCCH+CCCH (for TS0 of the BCCH carrier with no SDCCH)
//
// Frame mapping for the 51-multiframe:
//   0:  FCCH
//   1:  SCH
//   2:  BCCH
//   3:  BCCH
//   4:  BCCH
//   5:  BCCH
//   6:  CCCH
//   7:  CCCH
//   8:  CCCH
//   9:  CCCH
//  10:  FCCH
//  11:  SCH
//  12-19: CCCH
//  20:  FCCH
//  21:  SCH
//  22-29: CCCH
//  30:  FCCH
//  31:  SCH
//  32-39: CCCH
//  40:  FCCH
//  41:  SCH
//  42-49: CCCH
//  50:  IDLE

gsm_chan_type_t gsm_get_channel_type(int32_t fn_mod51)
{
    // FCCH frames: 0, 10, 20, 30, 40
    if (fn_mod51 == 0  || fn_mod51 == 10 || fn_mod51 == 20 ||
        fn_mod51 == 30 || fn_mod51 == 40)
        return GSM_CHAN_FCCH;

    // SCH frames: 1, 11, 21, 31, 41
    if (fn_mod51 == 1  || fn_mod51 == 11 || fn_mod51 == 21 ||
        fn_mod51 == 31 || fn_mod51 == 41)
        return GSM_CHAN_SCH;

    // BCCH frames: 2, 3, 4, 5
    if (fn_mod51 >= 2 && fn_mod51 <= 5)
        return GSM_CHAN_BCCH;

    // IDLE frame: 50
    if (fn_mod51 == 50)
        return GSM_CHAN_IDLE;

    // Everything else is CCCH (PCH/AGCH)
    return GSM_CHAN_CCCH;
}

// ======================== Main Decoder State Machine ========================

struct gsm_state *gsm_create(const gsm_config_t *cfg)
{
    if (!cfg) return NULL;

    init_conv_tables();

    struct gsm_state *st = calloc(1, sizeof(struct gsm_state));
    if (!st) return NULL;

    st->cfg = *cfg;
    st->sync_state = GSM_SYNC_NONE;
    st->tsc = cfg->tsc;

    // Initialize cell identity from config frequency
    st->cell.freq_mhz = cfg->arfcn_freq / 1e6;
    st->cell.arfcn = gsm_freq_to_arfcn(st->cell.freq_mhz, &st->cell.band);

    st->symbol_period = (double)cfg->sample_rate / GSM_SYMBOL_RATE;
    st->samples_per_frame = (int32_t)(GSM_TIMESLOTS * GSM_BURST_BITS * st->symbol_period + 0.5);
    st->carrier_offset_hz = cfg->arfcn_freq - cfg->center_freq;

    // Allocate phase buffer — enough for ~4 frames of processing
    st->phase_buf_size = st->samples_per_frame * 4;
    st->phase_buf = calloc((size_t)st->phase_buf_size, sizeof(float));
    if (!st->phase_buf) { free(st); return NULL; }

    // Work buffers — work_soft must be >= phase_buf_size (reused as freq_buf in FCCH detect)
    size_t soft_size = (size_t)st->phase_buf_size;
    if (soft_size < GSM_BCCH_CODED_BITS + 128)
        soft_size = GSM_BCCH_CODED_BITS + 128;
    st->work_soft = calloc(soft_size, sizeof(float));
    st->work_hard = calloc(GSM_BCCH_PRE_CONV + 64, sizeof(uint8_t));
    if (!st->work_soft || !st->work_hard) {
        free(st->phase_buf);
        free(st->work_soft);
        free(st->work_hard);
        free(st);
        return NULL;
    }

    // Channel filter: mix to baseband + Hamming-windowed sinc LPF at 150 kHz
    {
        int32_t ntaps = 51; // odd number of taps
        double fc = 150000.0 / (double)cfg->sample_rate; // normalized cutoff
        st->filt_taps = ntaps;
        st->filt_coeff = calloc((size_t)ntaps, sizeof(float));
        st->filt_I = calloc((size_t)ntaps, sizeof(float));
        st->filt_Q = calloc((size_t)ntaps, sizeof(float));
        if (!st->filt_coeff || !st->filt_I || !st->filt_Q) {
            free(st->phase_buf); free(st->work_soft); free(st->work_hard);
            free(st->filt_coeff); free(st->filt_I); free(st->filt_Q);
            free(st);
            return NULL;
        }
        int32_t half = ntaps / 2;
        double sum = 0;
        for (int32_t i = 0; i < ntaps; i++) {
            double t = (double)(i - half);
            /* normalized sinc: sin(π·x)/(π·x) where x = 2·fc·t */
            double x = 2.0 * fc * t;
            double sinc_val = (t == 0.0) ? 1.0 : sin(M_PI * x) / (M_PI * x);
            double hamming = 0.54 - 0.46 * cos(2.0 * M_PI * i / (ntaps - 1));
            st->filt_coeff[i] = (float)(sinc_val * hamming);
            sum += st->filt_coeff[i];
        }
        for (int32_t i = 0; i < ntaps; i++)
            st->filt_coeff[i] /= (float)sum;

        st->filt_phase = 0.0;
        st->filt_phase_inc = -2.0 * M_PI * st->carrier_offset_hz / (double)cfg->sample_rate;

        fprintf(stderr, "GSM: channel filter %d taps, cutoff %.0f Hz, carrier %.0f Hz\n",
                ntaps, fc * cfg->sample_rate, st->carrier_offset_hz);

        /* After filter init, carrier is removed in the filter.
         * Set carrier_offset_hz to 0 so all downstream freq calcs
         * don't add the carrier again. */
        st->carrier_offset_hz = 0;
    }

    return st;
}

void gsm_destroy(struct gsm_state *st)
{
    if (!st) return;
    free(st->phase_buf);
    free(st->work_soft);
    free(st->work_hard);
    free(st->filt_coeff);
    free(st->filt_I);
    free(st->filt_Q);
    free(st);
}

// Process SCH burst at a known sample position.
// Returns true if successfully decoded.
static bool process_sch(struct gsm_state *st, int32_t sample_pos)
{
    float burst_soft[GSM_BURST_BITS];
    float sps = (float)st->symbol_period;
    float freq_rps = (float)((st->freq_offset + st->carrier_offset_hz) * 2.0 * M_PI / st->cfg.sample_rate);

    extract_burst_soft(st->phase_buf, st->phase_buf_len, sample_pos, sps,
                       freq_rps, burst_soft);

    // SCH burst: 3 tail | 39 data | 64 training | 39 data | 3 tail | guard
    // Training starts at bit 3+39 = 42
    float quality;
    float timing_adj = correlate_training(burst_soft, gsm_sb_training, 64, 42, &quality);

    if (quality < 0.15f) {
        st->stats.sch_failed++;
        static int32_t sch_dbg_count = 0;
        if (++sch_dbg_count <= 3) {
            fprintf(stderr, "GSM SCH: quality=%.3f < 0.15 at pos=%d freq_off=%.1f Hz\n",
                    quality, sample_pos, st->freq_offset);
        }
        return false;
    }

    // Re-extract with timing correction if needed
    if (fabsf(timing_adj) > 0.5f) {
        int32_t adj_samples = (int32_t)(timing_adj * sps + 0.5f);
        extract_burst_soft(st->phase_buf, st->phase_buf_len,
                           sample_pos + adj_samples, sps, freq_rps, burst_soft);
    }

    /* Debug: check post-correction training quality */
    {
        float q2;
        float tadj2 = correlate_training(burst_soft, gsm_sb_training, 64, 42, &q2);
        static int32_t pcdbg = 0;
        if (++pcdbg <= 5) {
            fprintf(stderr, "GSM SCH post-corr: q=%.3f tadj=%.1f (was q=%.3f tadj=%.1f)\n",
                    q2, tadj2, quality, timing_adj);
            fprintf(stderr, "  training soft[42..55]=");
            for (int32_t i = 42; i < 56; i++) fprintf(stderr, " %.2f", burst_soft[i]);
            fprintf(stderr, "\n  expect[0..13]       =");
            for (int32_t i = 0; i < 14; i++) fprintf(stderr, " %+.0f.00", gsm_sb_training[i] ? -1.0f : 1.0f);
            fprintf(stderr, "\n");
        }
    }

    // Extract SCH data bits: positions 3..41 (39 bits) + 109..147 (39 bits)
    float sch_soft[GSM_SCH_CODED_BITS]; // 78 soft bits
    for (int32_t i = 0; i < 39; i++) {
        sch_soft[i]      = burst_soft[3 + i];
        sch_soft[39 + i] = burst_soft[3 + 39 + 64 + i]; // after training
    }

    uint8_t info[GSM_SCH_INFO_BITS];
    if (!gsm_sch_decode(sch_soft, info)) {
        st->stats.sch_failed++;
        static int32_t sch_dec_dbg = 0;
        if (++sch_dec_dbg <= 5) {
            fprintf(stderr, "GSM SCH: decode failed (quality=%.3f ok) at pos=%d, timing_adj=%.1f\n", quality, sample_pos, timing_adj);
            fprintf(stderr, "GSM SCH soft[0..9]: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
                    sch_soft[0], sch_soft[1], sch_soft[2], sch_soft[3], sch_soft[4],
                    sch_soft[5], sch_soft[6], sch_soft[7], sch_soft[8], sch_soft[9]);
        }
        return false;
    }

    uint8_t bsic;
    uint32_t fn;
    gsm_sch_parse(info, &bsic, &fn);

    st->bsic = bsic;
    st->bsic_valid = true;
    st->fn = fn;
    st->fn_mod51 = (int32_t)(fn % 51);
    st->cell.bsic = bsic;
    st->cell.si3.ncc = bsic >> 3;
    st->cell.si3.bcc = bsic & 7;
    st->cell.fn = fn;
    st->cell.sch_count++;
    st->stats.sch_decoded++;

    // TSC = BCC (BSIC low 3 bits) for BCCH carrier
    if (st->tsc < 0) {
        st->tsc = bsic & 7;
    }

    if (st->sync_state < GSM_SYNC_SCH) {
        st->sync_state = GSM_SYNC_SCH;
    }

    return true;
}

// Process a Normal Burst and accumulate for BCCH/CCCH
static bool process_normal_burst(struct gsm_state *st, int32_t sample_pos,
                                 gsm_chan_type_t chan_type, int32_t fn_mod51)
{
    float burst_soft[GSM_BURST_BITS];
    float sps = (float)st->symbol_period;
    float freq_rps = (float)((st->freq_offset + st->carrier_offset_hz) * 2.0 * M_PI / st->cfg.sample_rate);

    extract_burst_soft(st->phase_buf, st->phase_buf_len, sample_pos, sps,
                       freq_rps, burst_soft);

    // Training sequence at bit 61 (3 tail + 57 data + 1 steal = 61)
    if (st->tsc < 0 || st->tsc > 7) return false;

    float quality;
    float timing_adj = correlate_training(burst_soft, gsm_nb_training[st->tsc],
                                          GSM_NB_TRAIN_BITS, 61, &quality);
    if (quality < 0.3f) return false;

    // Re-extract with timing correction
    if (fabsf(timing_adj) > 0.5f) {
        int32_t adj_samples = (int32_t)(timing_adj * sps + 0.5f);
        extract_burst_soft(st->phase_buf, st->phase_buf_len,
                           sample_pos + adj_samples, sps, freq_rps, burst_soft);
    }

    // Extract 114 data bits
    float data_soft[GSM_NB_DATA_BITS];
    extract_nb_data(burst_soft, data_soft);

    // Accumulate into the right block
    gsm_burst_acc_t *acc = NULL;

    if (chan_type == GSM_CHAN_BCCH) {
        acc = &st->bcch_acc;
        int32_t burst_idx = fn_mod51 - 2; // frames 2,3,4,5 → indices 0,1,2,3
        if (burst_idx < 0 || burst_idx > 3) return false;
        memcpy(acc->soft_bits[burst_idx], data_soft, sizeof(data_soft));
        if (burst_idx == 0) acc->fn_start = st->fn;
        acc->n_bursts = burst_idx + 1;
    } else if (chan_type == GSM_CHAN_CCCH) {
        // CCCH blocks: frames 6-9, 12-15, 16-19, 22-25, 26-29, 32-35, 36-39, 42-45, 46-49
        // Each block is 4 consecutive frames
        int32_t ccch_frame;
        if (fn_mod51 >= 6 && fn_mod51 <= 9) ccch_frame = fn_mod51 - 6;
        else if (fn_mod51 >= 12 && fn_mod51 <= 15) ccch_frame = fn_mod51 - 12;
        else if (fn_mod51 >= 16 && fn_mod51 <= 19) ccch_frame = fn_mod51 - 16;
        else if (fn_mod51 >= 22 && fn_mod51 <= 25) ccch_frame = fn_mod51 - 22;
        else if (fn_mod51 >= 26 && fn_mod51 <= 29) ccch_frame = fn_mod51 - 26;
        else if (fn_mod51 >= 32 && fn_mod51 <= 35) ccch_frame = fn_mod51 - 32;
        else if (fn_mod51 >= 36 && fn_mod51 <= 39) ccch_frame = fn_mod51 - 36;
        else if (fn_mod51 >= 42 && fn_mod51 <= 45) ccch_frame = fn_mod51 - 42;
        else if (fn_mod51 >= 46 && fn_mod51 <= 49) ccch_frame = fn_mod51 - 46;
        else return false;

        // Use a rotating CCCH accumulator
        int32_t ccch_block_idx = 0;
        if (fn_mod51 >= 6 && fn_mod51 <= 9)   ccch_block_idx = 0;
        else if (fn_mod51 >= 12 && fn_mod51 <= 19) ccch_block_idx = 1;
        else if (fn_mod51 >= 22 && fn_mod51 <= 29) ccch_block_idx = 2;
        else ccch_block_idx = 3;

        acc = &st->ccch_acc[ccch_block_idx % 4];
        memcpy(acc->soft_bits[ccch_frame], data_soft, sizeof(data_soft));
        if (ccch_frame == 0) acc->fn_start = st->fn;
        acc->n_bursts = ccch_frame + 1;
    } else {
        return false;
    }

    // Check if we have a complete block (4 bursts)
    if (acc && acc->n_bursts >= 4) {
        uint8_t l2_frame[GSM_L2_FRAME_LEN];
        if (decode_bcch_block(acc->soft_bits, l2_frame)) {
            // Parse L2
            gsm_l2_frame_t l2;
            if (gsm_l2_parse(l2_frame, GSM_L2_FRAME_LEN, &l2)) {
                // L3 data starts at octet 3 of L2 frame
                const uint8_t *l3_data = &l2_frame[3];
                int32_t l3_len = l2.length;

                if (l3_len > 0) {
                    // Parse the L3 message
                    if (chan_type == GSM_CHAN_BCCH) {
                        uint8_t msg_type = (l3_len >= 2) ? l3_data[1] : 0;
                        const char *si_name = "UNKNOWN";

                        switch (msg_type) {
                        case 0x19:
                            gsm_parse_si1(l3_data, l3_len, &st->cell.si1);
                            si_name = "SI1";
                            break;
                        case 0x1A:
                            gsm_parse_si2(l3_data, l3_len, &st->cell.si2);
                            si_name = "SI2";
                            break;
                        case 0x1B:
                            gsm_parse_si3(l3_data, l3_len, &st->cell.si3);
                            st->cell.si3.bsic = st->bsic;
                            st->cell.si3.ncc = st->bsic >> 3;
                            st->cell.si3.bcc = st->bsic & 7;
                            si_name = "SI3";
                            if (st->sync_state < GSM_SYNC_LOCKED)
                                st->sync_state = GSM_SYNC_LOCKED;
                            break;
                        case 0x1C:
                            gsm_parse_si4(l3_data, l3_len, &st->cell.si4);
                            si_name = "SI4";
                            break;
                        default:
                            break;
                        }

                        st->cell.bcch_count++;
                        st->stats.bcch_decoded++;

                        if (st->cfg.msg_cb) {
                            st->cfg.msg_cb(&st->cell, si_name, l3_data, l3_len,
                                          st->cfg.callback_ctx);
                        }
                    } else {
                        // CCCH — check for Paging messages
                        st->cell.ccch_count++;
                        st->stats.ccch_decoded++;

                        if (st->cfg.msg_cb) {
                            st->cfg.msg_cb(&st->cell, "CCCH", l3_data, l3_len,
                                          st->cfg.callback_ctx);
                        }
                    }
                }
            }
        } else {
            if (chan_type == GSM_CHAN_BCCH)
                st->stats.bcch_failed++;
        }

        acc->n_bursts = 0;
    }

    return true;
}

// Main sample processing — called from the SDR reader thread
void gsm_process(struct gsm_state *st, const uint8_t *iq_data, uint32_t len)
{
    uint32_t n_samples = len / 2;
    st->stats.samples_processed += n_samples;

    // Debug: log every ~30 seconds (30M samples at 1MHz)
    static uint64_t dbg_total = 0;
    static uint64_t dbg_last = 0;
    dbg_total += n_samples;
    if (dbg_total - dbg_last >= 30000000) {
        fprintf(stderr, "GSM: process called, %u samples, total=%.1fM, state=%u, fcch=%u, sch=%u, bcch=%u, phase_buf=%d/%d\n",
                n_samples, (double)dbg_total/1e6, st->sync_state,
                (uint32_t)st->stats.fcch_detected, (uint32_t)st->stats.sch_decoded, (uint32_t)st->stats.bcch_decoded,
                st->phase_buf_len, st->phase_buf_size);
        dbg_last = dbg_total;
    }

    // Convert IQ to phase
    // We work in chunks that fit in our phase buffer
    uint32_t offset = 0;
    while (offset < n_samples) {
        uint32_t chunk = n_samples - offset;
        uint32_t avail = (uint32_t)(st->phase_buf_size - st->phase_buf_len);
        if (chunk > avail) chunk = avail;
        if (chunk == 0) {
            // Buffer full: shift left by half
            int32_t shift = st->phase_buf_size / 2;
            memmove(st->phase_buf, &st->phase_buf[shift],
                    (size_t)(st->phase_buf_len - shift) * sizeof(float));
            st->phase_buf_len -= shift;
            st->fcch_sample_pos -= shift;
            st->frame_sample_pos -= shift;
            continue;
        }

        iq_to_phase_filtered(st, &iq_data[offset * 2], chunk,
                             &st->phase_buf[st->phase_buf_len]);
        st->phase_buf_len += (int32_t)chunk;
        offset += chunk;
    }

    // State machine
    switch (st->sync_state) {
    case GSM_SYNC_NONE:
    case GSM_SYNC_FCCH: {
        // Try to detect FCCH
        if (st->phase_buf_len < (int32_t)(150 * st->symbol_period)) break;

        float *freq_buf = st->work_soft; // reuse work buffer
        phase_to_freq(st->phase_buf, st->phase_buf_len, freq_buf);

        // Debug: dump freq_buf stats once per second
        {
            static uint64_t fcch_dbg_total = 0;
            static uint64_t fcch_dbg_last = 0;
            fcch_dbg_total += n_samples;
            if (fcch_dbg_total - fcch_dbg_last >= 10000000) {
                int32_t n_freq = st->phase_buf_len - 1;
                float carrier_rps_dbg = (float)(st->carrier_offset_hz * 2.0 * M_PI / st->cfg.sample_rate);
                float expected = carrier_rps_dbg + (float)M_PI / (2.0f * (float)st->symbol_period);
                float fmin = 1e9f, fmax = -1e9f;
                double fsum = 0;
                int32_t near = 0;
                for (int32_t i = 0; i < n_freq; i++) {
                    if (freq_buf[i] < fmin) fmin = freq_buf[i];
                    if (freq_buf[i] > fmax) fmax = freq_buf[i];
                    fsum += freq_buf[i];
                    if (fabsf(freq_buf[i] - expected) < 0.15f) near++;
                }
                fprintf(stderr, "GSM FCCH: n=%d expected=%.4f mean=%.4f min=%.4f max=%.4f near=%d/%d (%.1f%%)\n",
                        n_freq, expected, (float)(fsum/n_freq), fmin, fmax, near, n_freq, 100.0*near/n_freq);
                fcch_dbg_last = fcch_dbg_total;
            }
        }

        double freq_off;
        float carrier_rps = (float)(st->carrier_offset_hz * 2.0 * M_PI / st->cfg.sample_rate);
        int32_t fcch_pos = detect_fcch(freq_buf, st->phase_buf_len - 1,
                                   (float)st->symbol_period, carrier_rps, &freq_off);

        // Debug: show best run regardless of detection
        {
            static uint64_t run_dbg = 0;
            static uint64_t run_dbg_last = 0;
            run_dbg += n_samples;
            if (run_dbg - run_dbg_last >= 10000000) {
                float exp2 = (float)(st->carrier_offset_hz * 2.0 * M_PI / st->cfg.sample_rate)
                           + (float)M_PI / (2.0f * (float)st->symbol_period);
                int32_t n_freq2 = st->phase_buf_len - 1;
                int32_t best_run = 0, cur_run = 0;
                float best_mean = 0;
                double cur_sum = 0;
                for (int32_t i = 0; i < n_freq2; i++) {
                    if (fabsf(freq_buf[i] - exp2) < 0.15f) {
                        cur_run++;
                        cur_sum += freq_buf[i];
                    } else {
                        if (cur_run > best_run) {
                            best_run = cur_run;
                            best_mean = (float)(cur_sum / cur_run);
                        }
                        cur_run = 0;
                        cur_sum = 0;
                    }
                }
                if (cur_run > best_run) { best_run = cur_run; best_mean = (float)(cur_sum / cur_run); }
                int32_t min_needed = (int32_t)(100 * st->symbol_period);
                fprintf(stderr, "GSM FCCH: best_run=%d min_needed=%d (%.1f sym) best_mean=%.4f\n",
                        best_run, min_needed, best_run / st->symbol_period, best_mean);
                run_dbg_last = run_dbg;
            }
        }

        if (fcch_pos >= 0) {
            st->freq_offset = freq_off;
            st->fcch_sample_pos = fcch_pos;
            st->stats.fcch_detected++;
            st->cell.fcch_count++;
            st->stats.freq_offset_hz = freq_off;

            if (st->sync_state == GSM_SYNC_NONE) {
                st->sync_state = GSM_SYNC_FCCH;
            }

            /* SCH burst is in TS0 of frame N+1, one frame after FCCH.
             * fcch_end = end of FCCH run in phase_buf.
             * FCCH run ~ 142 bits = ~524 samples. Frame period = 4615 samples.
             * So SCH start ~ fcch_end + (4615 - 524) = fcch_end + 4091.
             * Search fcch_end+3400..fcch_end+5100 to cover timing uncertainty. */
            int32_t fcch_end = fcch_pos;
            float burst_try[GSM_BURST_BITS];
            float best_q = 0;
            int32_t best_pos = 0;
            float best_tadj = 0;
            float sps_f = (float)st->symbol_period;
            float frps = (float)((st->freq_offset + st->carrier_offset_hz) * 2.0 * M_PI / st->cfg.sample_rate);
            int32_t burst_len = (int32_t)(156 * st->symbol_period);
            int32_t frame_samples = (int32_t)(1250.0 * st->symbol_period + 0.5);
            int32_t search_start = fcch_end + frame_samples - 1200;
            int32_t search_end   = fcch_end + frame_samples + 500;
            if (search_start < 1) search_start = 1;
            if (search_end + burst_len > st->phase_buf_len)
                search_end = st->phase_buf_len - burst_len;

            for (int32_t pos = search_start; pos <= search_end; pos += (int32_t)(sps_f)) {
                extract_burst_soft(st->phase_buf, st->phase_buf_len, pos, sps_f, frps, burst_try);
                float q;
                float tadj = correlate_training(burst_try, gsm_sb_training, 64, 42, &q);
                if (q > best_q) {
                    best_q = q;
                    best_pos = pos;
                    best_tadj = tadj;
                }
            }

            {
                static int32_t search_dbg = 0;
                if (++search_dbg <= 5) {
                    int32_t expected = fcch_end + frame_samples - 524;
                    fprintf(stderr, "GSM SCH search: fcch=%d exp=%d best=%d (delta=%d) q=%.3f tadj=%.1f range=[%d..%d]\n",
                            fcch_end, expected, best_pos, best_pos - fcch_end, best_q, best_tadj, search_start, search_end);
                    if (best_pos > 0 && best_pos + burst_len < st->phase_buf_len) {
                        extract_burst_soft(st->phase_buf, st->phase_buf_len, best_pos, sps_f, frps, burst_try);
                        fprintf(stderr, "  soft[39..52]=");
                        for (int32_t i = 39; i < 53; i++) fprintf(stderr, " %.2f", burst_try[i]);
                        fprintf(stderr, "\n");
                    }
                }
            }

            if (best_q >= 0.25f && best_pos > 0 && best_pos + burst_len < st->phase_buf_len) {
                process_sch(st, best_pos);
                /* Initialize frame tracking from SCH burst position (with timing
                 * correction) so the tracking loop advances correctly. */
                if (st->sync_state >= GSM_SYNC_SCH) {
                    int32_t adj = (int32_t)(best_tadj * sps_f + 0.5f);
                    st->frame_sample_pos = best_pos + adj;
                }
            }
        }
        break;
    }

    case GSM_SYNC_SCH:
    case GSM_SYNC_LOCKED: {
        // We know frame timing and frame number.
        // Advance frame by frame and decode each burst on TS0.

        // Calculate where next frame starts
        int32_t frame_start = st->frame_sample_pos;

        // Skip frames whose data is no longer in the buffer
        while (frame_start < 0 && frame_start + st->samples_per_frame < st->phase_buf_len) {
            frame_start += st->samples_per_frame;
            st->fn++;
        }

        while (frame_start + st->samples_per_frame < st->phase_buf_len) {
            int32_t fn_mod51 = (int32_t)(st->fn % 51);
            gsm_chan_type_t chan = gsm_get_channel_type(fn_mod51);

            switch (chan) {
            case GSM_CHAN_FCCH:
                // Use FCCH to refine frequency offset
                // (already done during initial sync, could update here)
                break;

            case GSM_CHAN_SCH:
                process_sch(st, frame_start);
                break;

            case GSM_CHAN_BCCH:
            case GSM_CHAN_CCCH:
                process_normal_burst(st, frame_start, chan, fn_mod51);
                break;

            case GSM_CHAN_IDLE:
                // Nothing to do
                break;

            default:
                break;
            }

            // Advance to next frame
            frame_start += st->samples_per_frame;
            st->fn++;
        }

        st->frame_sample_pos = frame_start;
        break;
    }
    }
}

void gsm_get_cell_info(const struct gsm_state *st, gsm_cell_info_t *out)
{
    if (st && out) *out = st->cell;
}

void gsm_get_stats(const struct gsm_state *st, gsm_stats_t *out)
{
    if (st && out) *out = st->stats;
}

gsm_sync_state_t gsm_get_sync_state(const struct gsm_state *st)
{
    return st ? st->sync_state : GSM_SYNC_NONE;
}
