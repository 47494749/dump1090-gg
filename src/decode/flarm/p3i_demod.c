// p3i_demod.c — PilotAware P3I FSK demodulator
//
// Standalone demodulator for P3I protocol at 869.525 MHz.
// Architecture: NCO downconvert → FIR LPF → FM discriminator → NCC sync → bit extract → decode
//
// P3I parameters (from SoftRF GPL-3.0):
//   Modulation: 2-FSK, 38.4 kbps, ±10 kHz deviation
//   Preamble: 10 bytes 0xAA (80 bit transitions)
//   Syncword: 0xB4, 0x2B (16 bits)
//   Payload: Net ID (4) + Len (1) + CRC seed (1) + Data (24) + CRC (1) = 31 bytes
//
// This file is free software: GPL-3.0-or-later

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "p3i_demod.h"
#include "p3i_decode.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ======================== Tunable constants ========================

// FIR low-pass filter: wider cutoff for P3I to tolerate RTL-SDR crystal offset (±60 ppm = ±52 kHz @ 868 MHz)
// P3I signal BW = ~58 kHz (38.4 kbps + 2×10 kHz dev), plus ±60 kHz crystal error → need ~120 kHz
#define P3I_FIR_TAPS        65
#define P3I_LPF_CUTOFF      80000   // 80 kHz cutoff (was 30 kHz, widened for crystal offset tolerance)

// FM ring buffer: power of 2, must hold sync + payload + margin
// At 2.4 MSPS and P3I 38.4 kbps: 1 bit = 62.5 samples
// Syncword: 16 bits = 1000 samples
// Payload: 31 bytes × 8 = 248 bits = 15500 samples
// Total needed: ~17000 + margin → 32768
#define P3I_FM_RING_BITS    15
#define P3I_FM_RING_SIZE    (1 << P3I_FM_RING_BITS)   // 32768
#define P3I_FM_RING_MASK    (P3I_FM_RING_SIZE - 1)

// NCC sync threshold (lower than FLARM because P3I sync is shorter = 16 bits)
#define P3I_SYNC_NCC_THRESHOLD  0.55f

// Check correlation every N samples
#define P3I_CORR_CHECK_INTERVAL 8

// Refine sync position +/- this many samples
#define P3I_SYNC_REFINE_RANGE   32

// Lockout after sync detection
#define P3I_SYNC_LOCKOUT_SAMPLES 2000

// P3I frame: 31 bytes after syncword (NetID 4 + Len 1 + CRC seed 1 + Payload 24 + CRC 1)
#define P3I_FRAME_TOTAL_BITS    (P3I_FRAME_BYTES * 8)  // 248 bits

// ======================== FIR filter ========================

static float p3i_fir_coeffs[P3I_FIR_TAPS];
static int32_t   p3i_fir_initialized = 0;

typedef struct {
    float delay_i[P3I_FIR_TAPS];
    float delay_q[P3I_FIR_TAPS];
    int32_t pos;
} p3i_fir_state_t;

static void p3i_design_fir_lpf(float *h, int32_t N, float cutoff, float fs)
{
    float fc = cutoff / fs;
    int32_t M = (N - 1) / 2;
    for (int32_t n = 0; n < N; n++) {
        double w = 0.54 - 0.46 * cos(2.0 * M_PI * n / (N - 1));
        double sinc_val;
        if (n == M) {
            sinc_val = 2.0 * fc;
        } else {
            double x = 2.0 * M_PI * fc * (n - M);
            sinc_val = sin(x) / (M_PI * (n - M));
        }
        h[n] = (float)(sinc_val * w);
    }
}

static inline void p3i_fir_process(p3i_fir_state_t *fir, float in_i, float in_q,
                                    float *out_i, float *out_q)
{
    fir->delay_i[fir->pos] = in_i;
    fir->delay_q[fir->pos] = in_q;

    float sum_i = 0, sum_q = 0;
    int32_t idx = fir->pos;
    for (int32_t k = 0; k < P3I_FIR_TAPS; k++) {
        sum_i += p3i_fir_coeffs[k] * fir->delay_i[idx];
        sum_q += p3i_fir_coeffs[k] * fir->delay_q[idx];
        idx--;
        if (idx < 0) idx = P3I_FIR_TAPS - 1;
    }

    *out_i = sum_i;
    *out_q = sum_q;

    fir->pos++;
    if (fir->pos >= P3I_FIR_TAPS) fir->pos = 0;
}

// ======================== Sync template ========================

// P3I syncword: 0xB4, 0x2B — 16 bits total
// In FSK: 1-bit → positive deviation, 0-bit → negative deviation
// Template: ±1 values at samples_per_bit resolution

static float *p3i_sync_template = NULL;
static int32_t    p3i_sync_template_len = 0;
static float  p3i_template_energy = 0;
static int32_t    p3i_samples_per_bit = 0;
static uint32_t p3i_actual_sample_rate = 0;
static int32_t    p3i_templates_initialized = 0;

static void p3i_init_templates(uint32_t sample_rate)
{
    if (p3i_templates_initialized) return;

    // Init FIR
    if (!p3i_fir_initialized) {
        p3i_design_fir_lpf(p3i_fir_coeffs, P3I_FIR_TAPS, P3I_LPF_CUTOFF, (float)sample_rate);
        p3i_fir_initialized = 1;
    }

    // Samples per bit (integer, truncated)
    p3i_samples_per_bit = sample_rate / P3I_BITRATE;  // 2400000/38400 = 62
    p3i_actual_sample_rate = sample_rate;

    // Generate sync template: 0xB4 = 10110100, 0x2B = 00101011
    uint16_t pattern = (P3I_SYNCWORD_0 << 8) | P3I_SYNCWORD_1;  // 0xB42B
    p3i_sync_template_len = 16 * p3i_samples_per_bit;

    p3i_sync_template = malloc(p3i_sync_template_len * sizeof(float));
    if (!p3i_sync_template) return;

    for (int32_t bit = 0; bit < 16; bit++) {
        float val = ((pattern >> (15 - bit)) & 1) ? 1.0f : -1.0f;
        for (int32_t s = 0; s < p3i_samples_per_bit; s++)
            p3i_sync_template[bit * p3i_samples_per_bit + s] = val;
    }

    p3i_template_energy = (float)p3i_sync_template_len;
    p3i_templates_initialized = 1;
}

// ======================== Demod state ========================

struct p3i_demod_state {
    p3i_demod_config_t config;

    // NCO for downconverting to baseband
    float nco_cos, nco_sin;
    float nco_cos_inc, nco_sin_inc;
    uint32_t nco_count;

    // FIR filter
    p3i_fir_state_t fir;

    // FM discriminator
    float prev_i, prev_q;

    // DC block
    float dc_avg;

    // FM ring buffer
    float fm_ring[P3I_FM_RING_SIZE];
    uint32_t ring_wr;

    // Correlation timing
    uint32_t corr_countdown;

    // Sync lockout
    uint32_t lockout_remaining;

    // Packet collection state
    int32_t      collecting;
    int32_t      polarity;
    uint32_t payload_start;
    uint32_t samples_needed;

    // Stats
    p3i_demod_stats_t stats;
};

// ======================== NCC computation ========================

static float p3i_compute_ncc_flat(const float *signal, const float *tmpl,
                                   float tmpl_energy, int32_t len)
{
    float corr = 0, energy = 0, sig_sum = 0;
    for (int32_t i = 0; i < len; i++) {
        float v = signal[i];
        corr += v * tmpl[i];
        energy += v * v;
        sig_sum += v;
    }
    float mean_sq = (sig_sum * sig_sum) / (float)len;
    float var_energy = energy - mean_sq;
    if (var_energy < 1e-10f) return 0;
    return corr / sqrtf(var_energy * tmpl_energy);
}

static float p3i_compute_ncc_ring(const float *ring, uint32_t start,
                                   const float *tmpl, float tmpl_energy, int32_t tmpl_len)
{
    start &= P3I_FM_RING_MASK;
    if (start + tmpl_len <= P3I_FM_RING_SIZE) {
        return p3i_compute_ncc_flat(&ring[start], tmpl, tmpl_energy, tmpl_len);
    }
    // Wrap around: copy to temporary flat buffer
    // Max template: 16 bits × 62 samp/bit = 992 floats
    float temp[1024];
    if (tmpl_len > 1024) return 0;  // safety
    uint32_t first_part = P3I_FM_RING_SIZE - start;
    memcpy(temp, &ring[start], first_part * sizeof(float));
    memcpy(temp + first_part, ring, (tmpl_len - first_part) * sizeof(float));
    return p3i_compute_ncc_flat(temp, tmpl, tmpl_energy, tmpl_len);
}

static uint32_t p3i_refine_sync(const float *ring, uint32_t initial_start,
                                 const float *tmpl, float tmpl_energy, int32_t tmpl_len,
                                 float *best_ncc_out)
{
    uint32_t best_pos = initial_start;
    float best_ncc = fabsf(p3i_compute_ncc_ring(ring, initial_start, tmpl, tmpl_energy, tmpl_len));

    for (int32_t offset = -P3I_SYNC_REFINE_RANGE; offset <= P3I_SYNC_REFINE_RANGE; offset++) {
        if (offset == 0) continue;
        uint32_t pos = (initial_start + offset) & P3I_FM_RING_MASK;
        float ncc = fabsf(p3i_compute_ncc_ring(ring, pos, tmpl, tmpl_energy, tmpl_len));
        if (ncc > best_ncc) {
            best_ncc = ncc;
            best_pos = pos;
        }
    }
    *best_ncc_out = best_ncc;
    return best_pos;
}

// ======================== Bit extraction (NRZ, no Manchester) ========================

// Use float for precise bit timing (38400 bps at 2.4 MSPS = 62.5 samples/bit)
static void p3i_extract_bits(const float *ring, uint32_t start, int32_t polarity,
                              int32_t samples_per_bit_int __attribute__((unused)), int32_t n_bits, uint8_t *bits)
{
    // Precise samples per bit from the actual sample rate / bitrate
    float spb = (float)p3i_actual_sample_rate / (float)P3I_BITRATE;

    // DC estimate over payload region
    int32_t total_samples = (int32_t)(n_bits * spb);
    float dc_sum = 0;
    for (int32_t s = 0; s < total_samples; s++) {
        dc_sum += ring[(start + s) & P3I_FM_RING_MASK];
    }
    float dc_offset = dc_sum / (float)total_samples;

    for (int32_t bit = 0; bit < n_bits; bit++) {
        // Use float position to avoid cumulative timing drift
        uint32_t bit_start = (start + (uint32_t)(bit * spb)) & P3I_FM_RING_MASK;
        int32_t isamp = (int32_t)spb;  // integrate over integer samples
        float acc = 0;
        for (int32_t s = 0; s < isamp; s++) {
            acc += ring[(bit_start + s) & P3I_FM_RING_MASK];
        }
        acc -= dc_offset * isamp;
        bits[bit] = ((acc * polarity) > 0) ? 1 : 0;
    }
}

// ======================== PN9 de-whitening (SX1262 IBM whitening) ========================
// Polynomial: x^9 + x^5 + 1, init = 0x1FF
// Right-shifting LFSR, output = bit 0, MSB-first byte assembly

static void p3i_pn9_dewhiten(uint8_t *data, int32_t n_bytes)
{
    uint16_t state = 0x1FF;
    for (int32_t i = 0; i < n_bytes; i++) {
        uint8_t mask = 0;
        for (int32_t bit = 0; bit < 8; bit++) {
            uint8_t out = state & 1;
            uint8_t fb = ((state) ^ (state >> 5)) & 1;
            state = (state >> 1) | (fb << 8);
            mask |= (out << (7 - bit));  // MSB-first
        }
        data[i] ^= mask;
    }
}

// Convert bit array to bytes (MSB first)
static void p3i_bits_to_bytes(const uint8_t *bits, int32_t n_bits, uint8_t *bytes)
{
    int32_t n_bytes = n_bits / 8;
    memset(bytes, 0, n_bytes);
    for (int32_t i = 0; i < n_bits; i++) {
        bytes[i / 8] |= (bits[i] << (7 - (i % 8)));
    }
}

// ======================== Try decode P3I frame ========================

static bool p3i_try_decode(struct p3i_demod_state *state, uint32_t payload_start, int32_t polarity __attribute__((unused)))
{
    int32_t spb = p3i_samples_per_bit;
    int32_t total_bits = P3I_FRAME_TOTAL_BITS;
    // Extract extra bits for possible sync byte + length header (2 extra bytes = 16 bits)
    int32_t extra_bits = 16;
    int32_t extract_bits = total_bits + extra_bits;
    uint8_t bits[280];
    uint8_t raw_bytes[35];
    uint8_t frame[P3I_FRAME_BYTES];

    // Try both polarities
    for (int32_t pol = -1; pol <= 1; pol += 2) {
        p3i_extract_bits(state->fm_ring, payload_start, pol,
                         spb, extract_bits, bits);
        p3i_bits_to_bytes(bits, extract_bits, raw_bytes);

        // Strategy 1: Standard P3I (no extra bytes, no SX1262 whitening)
        memcpy(frame, raw_bytes, P3I_FRAME_BYTES);
        if (frame[0] == 0x00 && frame[1] == 0x00 && frame[2] == 0x00 && frame[3] == 0x00) {
            polarity = pol;
            goto try_decode;
        }

        // Strategy 2: Skip 1 extra sync byte + PN9 de-whitening (SX1262 with whitening)
        // The SX1262 may transmit 3 sync bytes (B4 2B XX) and apply PN9 whitening
        // After skipping 1 byte, the next byte might be the whitened length header,
        // followed by the whitened P3I payload
        {
            uint8_t dewhitened[35];
            // Skip 1 byte (extra sync), take next 32 bytes
            memcpy(dewhitened, &raw_bytes[1], P3I_FRAME_BYTES + 1);
            p3i_pn9_dewhiten(dewhitened, P3I_FRAME_BYTES + 1);
            // dewhitened[0] might be length header (0x1F = 31)
            // dewhitened[1:32] should be the P3I frame
            if (dewhitened[0] == P3I_FRAME_BYTES &&
                dewhitened[1] == 0x00 && dewhitened[2] == 0x00 &&
                dewhitened[3] == 0x00 && dewhitened[4] == 0x00) {
                memcpy(frame, &dewhitened[1], P3I_FRAME_BYTES);
                polarity = pol;
                goto try_decode;
            }

            // Also try: skip 1 byte, PN9 de-whiten WITHOUT length header
            memcpy(dewhitened, &raw_bytes[1], P3I_FRAME_BYTES);
            p3i_pn9_dewhiten(dewhitened, P3I_FRAME_BYTES);
            if (dewhitened[0] == 0x00 && dewhitened[1] == 0x00 &&
                dewhitened[2] == 0x00 && dewhitened[3] == 0x00) {
                memcpy(frame, dewhitened, P3I_FRAME_BYTES);
                polarity = pol;
                goto try_decode;
            }

            // Try: no skip, PN9 de-whiten directly
            memcpy(dewhitened, raw_bytes, P3I_FRAME_BYTES);
            p3i_pn9_dewhiten(dewhitened, P3I_FRAME_BYTES);
            if (dewhitened[0] == 0x00 && dewhitened[1] == 0x00 &&
                dewhitened[2] == 0x00 && dewhitened[3] == 0x00) {
                memcpy(frame, dewhitened, P3I_FRAME_BYTES);
                polarity = pol;
                goto try_decode;
            }
        }
    }

    // Nothing matched
    return false;

try_decode:

    // Verify length byte
    if (frame[4] != P3I_PAYLOAD_SIZE) {
        return false;
    }

    // Verify CRC seed (standard P3I uses 0x71, but test transmitters may use 0x00)
    if (frame[5] != P3I_CRC_SEED && frame[5] != 0x00) {
        return false;
    }

    // Extract the 24-byte payload (bytes 6..29)
    uint8_t payload[P3I_PAYLOAD_SIZE];
    memcpy(payload, &frame[6], P3I_PAYLOAD_SIZE);

    // Verify CRC-8 over the whitened payload, using the seed from the packet
    uint8_t received_crc = frame[30];
    uint8_t computed_crc = p3i_crc8(payload, P3I_PAYLOAD_SIZE);
    if (computed_crc != received_crc) {
        // Don't return false — try decoding anyway for diagnostics
    }

    // Decode (de-whitens internally)
    p3i_message_t msg;
    if (!p3i_decode_packet(payload, &msg))
        return false;

    state->stats.packets_decoded++;

    if (state->config.callback)
        state->config.callback(&msg, state->config.callback_ctx);

    return true;
}

// ======================== Create / Destroy ========================

struct p3i_demod_state *p3i_demod_create(const p3i_demod_config_t *config)
{
    p3i_init_templates(config->sample_rate);

    struct p3i_demod_state *state = calloc(1, sizeof(*state));
    if (!state) return NULL;

    state->config = *config;

    // NCO: mix 869.525 MHz channel down to baseband
    double freq_offset = (double)P3I_FREQ - (double)config->center_freq;
    double phase_inc = -2.0 * M_PI * freq_offset / (double)config->sample_rate;
    state->nco_cos = 1.0f;
    state->nco_sin = 0.0f;
    state->nco_cos_inc = (float)cos(phase_inc);
    state->nco_sin_inc = (float)sin(phase_inc);

    state->corr_countdown = P3I_CORR_CHECK_INTERVAL;

    return state;
}

void p3i_demod_destroy(struct p3i_demod_state *state)
{
    free(state);
}

// ======================== Process IQ samples ========================

void p3i_demod_process(struct p3i_demod_state *state, const uint8_t *iq_data, uint32_t len)
{
    uint32_t n_samples = len / 2;

    for (uint32_t i = 0; i < n_samples; i++) {
        // Convert uint8 IQ to float [-1, +1]
        float raw_i = (iq_data[i * 2]     - 127.5f) / 127.5f;
        float raw_q = (iq_data[i * 2 + 1] - 127.5f) / 127.5f;

        // NCO complex multiply: downconvert P3I channel to baseband
        float mix_i = raw_i * state->nco_cos - raw_q * state->nco_sin;
        float mix_q = raw_i * state->nco_sin + raw_q * state->nco_cos;

        // Advance NCO phase
        float new_cos = state->nco_cos * state->nco_cos_inc - state->nco_sin * state->nco_sin_inc;
        float new_sin = state->nco_sin * state->nco_cos_inc + state->nco_cos * state->nco_sin_inc;
        state->nco_cos = new_cos;
        state->nco_sin = new_sin;

        // Periodic renormalization (every 1024 samples)
        if (++state->nco_count >= 1024) {
            state->nco_count = 0;
            float mag = sqrtf(state->nco_cos * state->nco_cos + state->nco_sin * state->nco_sin);
            if (mag > 0) {
                state->nco_cos /= mag;
                state->nco_sin /= mag;
            }
        }

        // FIR low-pass filter (30 kHz cutoff for P3I's ±10 kHz deviation)
        float filt_i, filt_q;
        p3i_fir_process(&state->fir, mix_i, mix_q, &filt_i, &filt_q);

        // FM discriminator: atan2(cross, dot) where cross = I·dQ - Q·dI
        float cross = state->prev_i * filt_q - state->prev_q * filt_i;
        float dot   = state->prev_i * filt_i + state->prev_q * filt_q;
        float fm = atan2f(cross, dot);
        state->prev_i = filt_i;
        state->prev_q = filt_q;

        // DC removal (IIR high-pass, alpha ~0.001)
        state->dc_avg += 0.001f * (fm - state->dc_avg);
        fm -= state->dc_avg;

        // Store in ring buffer
        state->fm_ring[state->ring_wr] = fm;
        state->ring_wr = (state->ring_wr + 1) & P3I_FM_RING_MASK;

        state->stats.samples_processed++;

        // Lockout countdown
        if (state->lockout_remaining > 0) {
            state->lockout_remaining--;
            continue;
        }

        // If currently collecting payload, check if done
        if (state->collecting) {
            state->samples_needed--;
            if (state->samples_needed == 0) {
                state->collecting = 0;
                if (!p3i_try_decode(state, state->payload_start, state->polarity)) {
                    state->stats.packets_failed++;
                }
                state->lockout_remaining = P3I_SYNC_LOCKOUT_SAMPLES;
            }
            continue;
        }

        // Periodic sync correlation check
        if (--state->corr_countdown > 0) continue;
        state->corr_countdown = P3I_CORR_CHECK_INTERVAL;

        // NCC correlation against P3I sync template
        // Check on the raw FM ring — position the template window
        uint32_t tmpl_start = (state->ring_wr - p3i_sync_template_len) & P3I_FM_RING_MASK;
        float ncc = p3i_compute_ncc_ring(state->fm_ring, tmpl_start,
                                          p3i_sync_template, p3i_template_energy,
                                          p3i_sync_template_len);

        float abs_ncc = fabsf(ncc);
        if (abs_ncc < P3I_SYNC_NCC_THRESHOLD) continue;

        // Sync detected — refine position
        float refined_ncc;
        uint32_t refined_start = p3i_refine_sync(state->fm_ring, tmpl_start,
                                                  p3i_sync_template, p3i_template_energy,
                                                  p3i_sync_template_len, &refined_ncc);

        state->stats.sync_detected++;

        // Polarity: positive NCC → normal, negative → inverted
        int32_t polarity = (ncc > 0) ? 1 : -1;

        // Payload starts right after syncword
        uint32_t payload_start = (refined_start + p3i_sync_template_len) & P3I_FM_RING_MASK;
        // Extra 16 bits for possible SX1262 extra sync byte + length header
        uint32_t payload_samples = (P3I_FRAME_TOTAL_BITS + 16) * p3i_samples_per_bit;

        state->collecting = 1;
        state->polarity = polarity;
        state->payload_start = payload_start;
        state->samples_needed = payload_samples;
    }
}

// ======================== Stats ========================

void p3i_demod_get_stats(struct p3i_demod_state *state, p3i_demod_stats_t *stats)
{
    *stats = state->stats;
}
