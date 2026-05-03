// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// flarm_demod.c: GFSK demodulator for FLARM 868 MHz packets
//
// FLARM uses 2-FSK (GFSK) modulation at 100 kbps with +/-50 kHz deviation.
// The radio link uses Manchester encoding, an 8-byte syncword, and
// CRC-16 CCITT. After demodulation and CRC check, packets are passed
// to the FLARM protocol decoder for XXTEA decryption.
//
// Demodulation pipeline:
//   IQ samples (1.6 MSPS) -> NCO channelization -> FIR LPF (65-tap)
//   -> FM discriminator -> DC block -> sample-level matched filter (NCC)
//   -> payload extraction -> Manchester decode -> CRC -> decrypt/decode
//
// Key design: uses sample-level normalized cross-correlation (NCC) for sync
// detection. With 1024-sample sync template (64 chips x 16 samples/chip),
// noise floor std = 1/sqrt(1024) = 0.031, giving robust detection even at
// low SNR (real signals correlate at 0.35-0.60).
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "flarm_demod.h"
#include "flarm_decode.h"
#include "ogntp_decode.h"

// ======================== Constants ========================

// At 1.6 MSPS with 100 kbps on-air chip rate, each chip is 16 samples
#define SAMPLES_PER_BIT     16

// Sync template length in samples (64 chips x 16 samp/chip)
#define SYNC_TEMPLATE_LEN   (FLARM_SYNCWORD_SIZE * 8 * SAMPLES_PER_BIT)  // 1024

// After sync: 26 bytes x 8 bits x 2 (Manchester) = 416 chips
#define PAYLOAD_CHIPS       (FLARM_PACKET_TOTAL * 8 * 2)

// Payload length in samples
#define PAYLOAD_SAMPLES     (PAYLOAD_CHIPS * SAMPLES_PER_BIT)  // 6656

// FM sample ring buffer: power of 2, must hold sync + payload + margin
#define FM_RING_BITS        14
#define FM_RING_SIZE        (1 << FM_RING_BITS)   // 16384
#define FM_RING_MASK        (FM_RING_SIZE - 1)

// Sync NCC threshold.  Noise floor: std = 1/sqrt(1024) = 0.031.
// Peak noise over 100K trials (5s at 400K checks/s): ~4.5 sigma = 0.14.
// Weakest real signals: ~0.30 (from Python v7 decoder).
// Threshold at 0.20 catches everything; CRC rejects noise.
#define SYNC_NCC_THRESHOLD  0.30f

// Check correlation every N samples (timing resolution: +/-N/2 samples)
// Larger interval reduces CPU but may miss some peaks; refine compensates.
#define CORR_CHECK_INTERVAL 8

// After sync detection, refine position +/- this many samples
#define SYNC_REFINE_RANGE   8

// Minimum samples between sync detections to prevent re-triggering
#define SYNC_LOCKOUT_SAMPLES (SYNC_TEMPLATE_LEN + PAYLOAD_SAMPLES + 128)

// DC-blocking filter coefficient (time constant ~6ms at 1.6 Msps)
#define DC_BLOCK_ALPHA      0.9999f

// ======================== Channelizer ========================

#define FLARM_NUM_CHANNELS   2

static const uint32_t FLARM_CHANNEL_FREQS[FLARM_NUM_CHANNELS] = {
    868200000,  // 868.2 MHz (EU freq 1)
    868400000,  // 868.4 MHz (EU freq 2)
};

// Channel filter: 65-tap FIR low-pass, Hamming window, 120 kHz cutoff.
// Linear phase: no waveform distortion.  Matches Python v7 decoder exactly.
#define FIR_TAPS             65
#define CHANNEL_LPF_CUTOFF   120000.0

static float fir_coeffs[FIR_TAPS];  // shared by all channels (same filter)
static int fir_initialized = 0;

static void design_fir_lpf(float *h, int N, double cutoff_hz, double sample_rate)
{
    int M = (N - 1) / 2;
    double fc = cutoff_hz / sample_rate;  // normalized cutoff (0..0.5)
    for (int n = 0; n < N; n++) {
        // Hamming window
        double w = 0.54 - 0.46 * cos(2.0 * M_PI * n / (N - 1));
        // sinc
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

typedef struct {
    float delay_i[FIR_TAPS];
    float delay_q[FIR_TAPS];
    int pos;  // circular write position
} fir_state_t;

// ======================== Per-channel state ========================

typedef struct {
    // NCO (complex rotation)
    float nco_cos;           // current cos(phase)
    float nco_sin;           // current sin(phase)
    float nco_cos_inc;       // cos(phase_increment)
    float nco_sin_inc;       // sin(phase_increment)
    unsigned nco_count;      // counter for periodic renormalization

    // LPF (FIR)
    fir_state_t fir;

    // FM discriminator
    float prev_i, prev_q;

    // DC block
    float dc_avg;

    // FM sample ring buffer
    float fm_ring[FM_RING_SIZE];
    unsigned ring_wr;

    // Correlation timing
    unsigned corr_countdown;

    // Sync lockout
    unsigned lockout_remaining;

    // FLARM packet state
    int      flarm_collecting;
    int      flarm_polarity;
    float    flarm_det_ncc;     // NCC at detection for diagnostics
    unsigned flarm_payload_start;
    unsigned flarm_samples_needed;

    // OGNTP packet state
    int      ogntp_collecting;
    int      ogntp_polarity;
    unsigned ogntp_payload_start;
    unsigned ogntp_samples_needed;

    // Diagnostics
    float    diag_best_flarm_ncc;
    float    diag_best_ogntp_ncc;
    float    diag_fm_sum;
    unsigned diag_fm_count;
} flarm_channel_t;

// ======================== Overall state ========================

struct flarm_state {
    flarm_demod_config_t config;
    flarm_channel_t channels[FLARM_NUM_CHANNELS];
    uint64_t diag_report_interval;
    uint64_t diag_last_report;
    flarm_demod_stats_t stats;
    uint32_t time_override;  // If != 0, use this instead of time(NULL) for XXTEA
};

// ======================== FIR filter processing ========================

static inline void fir_process(fir_state_t *fir, float in_i, float in_q,
                               float *out_i, float *out_q)
{
    // Write new sample to delay line
    fir->delay_i[fir->pos] = in_i;
    fir->delay_q[fir->pos] = in_q;

    // Convolve: sum over taps
    float sum_i = 0, sum_q = 0;
    int idx = fir->pos;
    for (int k = 0; k < FIR_TAPS; k++) {
        sum_i += fir_coeffs[k] * fir->delay_i[idx];
        sum_q += fir_coeffs[k] * fir->delay_q[idx];
        idx--;
        if (idx < 0) idx = FIR_TAPS - 1;
    }

    *out_i = sum_i;
    *out_q = sum_q;

    fir->pos++;
    if (fir->pos >= FIR_TAPS) fir->pos = 0;
}

// ======================== Sync templates ========================

static float flarm_sync_template[SYNC_TEMPLATE_LEN];
static float ogntp_sync_template[SYNC_TEMPLATE_LEN];
static float flarm_template_energy;   // sum(template²)
static float ogntp_template_energy;
static int templates_initialized = 0;

// (No pre-filtering needed: FIR has linear phase, rectangular ±1 template works directly)

static void init_templates(void)
{
    if (templates_initialized) return;

    // Initialize FIR coefficients (once, shared by all channels)
    if (!fir_initialized) {
        design_fir_lpf(fir_coeffs, FIR_TAPS, CHANNEL_LPF_CUTOFF, FLARM_SAMPLE_RATE);
        fir_initialized = 1;
    }

    // Generate ideal ±1 FLARM sync template
    uint64_t pattern = 0;
    for (int i = 0; i < FLARM_SYNCWORD_SIZE; i++)
        pattern = (pattern << 8) | FLARM_SYNCWORD[i];
    for (int chip = 0; chip < 64; chip++) {
        float val = ((pattern >> (63 - chip)) & 1) ? 1.0f : -1.0f;
        for (int s = 0; s < SAMPLES_PER_BIT; s++)
            flarm_sync_template[chip * SAMPLES_PER_BIT + s] = val;
    }

    // Generate ideal ±1 OGNTP sync template
    uint64_t ogntp_pattern = 0;
    for (int i = 0; i < OGNTP_SYNCWORD_SIZE; i++)
        ogntp_pattern = (ogntp_pattern << 8) | OGNTP_SYNCWORD[i];
    for (int chip = 0; chip < 64; chip++) {
        float val = ((ogntp_pattern >> (63 - chip)) & 1) ? 1.0f : -1.0f;
        for (int s = 0; s < SAMPLES_PER_BIT; s++)
            ogntp_sync_template[chip * SAMPLES_PER_BIT + s] = val;
    }

    // Template energy = SYNC_TEMPLATE_LEN since all values are ±1
    flarm_template_energy = (float)SYNC_TEMPLATE_LEN;
    ogntp_template_energy = (float)SYNC_TEMPLATE_LEN;

    templates_initialized = 1;
}

// ======================== NCC computation ========================
// Uses DC-robust formula: subtracts windowed mean from energy denominator.
// Since template is zero-mean (sum=0), numerator is unaffected by signal DC.
// NCC = Σ(s·t) / sqrt((Σs² - (Σs)²/N) · N)

static float compute_ncc_flat(const float *signal, const float *tmpl, float tmpl_energy)
{
    float corr = 0, energy = 0, sig_sum = 0;
    for (int i = 0; i < SYNC_TEMPLATE_LEN; i++) {
        float v = signal[i];
        corr += v * tmpl[i];
        energy += v * v;
        sig_sum += v;
    }
    // Remove DC component from energy: var = E[x²] - E[x]²
    float mean_sq = (sig_sum * sig_sum) / (float)SYNC_TEMPLATE_LEN;
    float var_energy = energy - mean_sq;
    if (var_energy < 1e-10f) return 0;
    return corr / sqrtf(var_energy * tmpl_energy);
}

static float compute_ncc_ring(const float *ring, unsigned start,
                              const float *tmpl, float tmpl_energy)
{
    start &= FM_RING_MASK;
    if (start + SYNC_TEMPLATE_LEN <= FM_RING_SIZE) {
        return compute_ncc_flat(&ring[start], tmpl, tmpl_energy);
    }
    float temp[SYNC_TEMPLATE_LEN];
    unsigned first_part = FM_RING_SIZE - start;
    memcpy(temp, &ring[start], first_part * sizeof(float));
    memcpy(temp + first_part, ring, (SYNC_TEMPLATE_LEN - first_part) * sizeof(float));
    return compute_ncc_flat(temp, tmpl, tmpl_energy);
}

static unsigned refine_sync(const float *ring, unsigned initial_start,
                            const float *tmpl, float tmpl_energy,
                            float *best_ncc_out)
{
    unsigned best_pos = initial_start;
    float best_ncc = fabsf(compute_ncc_ring(ring, initial_start, tmpl, tmpl_energy));

    for (int offset = -SYNC_REFINE_RANGE; offset <= SYNC_REFINE_RANGE; offset++) {
        if (offset == 0) continue;
        unsigned pos = (initial_start + offset) & FM_RING_MASK;
        float ncc = fabsf(compute_ncc_ring(ring, pos, tmpl, tmpl_energy));
        if (ncc > best_ncc) {
            best_ncc = ncc;
            best_pos = pos;
        }
    }
    *best_ncc_out = best_ncc;
    return best_pos;
}

// ======================== Payload extraction ========================

static void extract_payload_bits(const float *ring, unsigned start, int polarity,
                                 uint8_t *manchester_bits)
{
    // Compute local DC estimate over the full payload region.
    // Manchester encoding is balanced (equal +1 and -1 chips), so the mean ≈ DC offset.
    float dc_sum = 0;
    for (int s = 0; s < PAYLOAD_CHIPS * SAMPLES_PER_BIT; s++) {
        dc_sum += ring[(start + s) & FM_RING_MASK];
    }
    float dc_offset = dc_sum / (float)(PAYLOAD_CHIPS * SAMPLES_PER_BIT);

    for (int chip = 0; chip < PAYLOAD_CHIPS; chip++) {
        unsigned chip_start = (start + chip * SAMPLES_PER_BIT) & FM_RING_MASK;
        float acc = 0;
        for (int s = 0; s < SAMPLES_PER_BIT; s++) {
            acc += ring[(chip_start + s) & FM_RING_MASK];
        }
        acc -= dc_offset * SAMPLES_PER_BIT;  // remove DC bias
        manchester_bits[chip] = ((acc * polarity) > 0) ? 1 : 0;
    }
}

// ======================== Manchester decode ========================

static bool manchester_decode_payload(const uint8_t *manchester_bits, unsigned n_bits,
                                      uint8_t *out_bytes, unsigned *out_len,
                                      unsigned *violations)
{
    unsigned n_data_bits = n_bits / 2;
    unsigned n_bytes = n_data_bits / 8;
    if (n_data_bits % 8 != 0) return false;

    *out_len = n_bytes;
    *violations = 0;
    memset(out_bytes, 0, n_bytes);

    for (unsigned i = 0; i < n_data_bits; i++) {
        uint8_t first  = manchester_bits[i * 2];
        uint8_t second = manchester_bits[i * 2 + 1];

        uint8_t data_bit;
        if (first == 1 && second == 0)
            data_bit = 0;
        else if (first == 0 && second == 1)
            data_bit = 1;
        else {
            (*violations)++;
            data_bit = second;
        }

        out_bytes[i / 8] |= (data_bit << (7 - (i % 8)));
    }
    return true;
}

// ======================== Bit error correction ========================

// Try CRC with up to max_corrections bit flips on the payload (excluding CRC bytes).
// Returns the number of corrections applied (0 = no error, -1 = uncorrectable).
static int try_bit_correction(uint8_t *payload, unsigned len, unsigned max_corrections)
{
    // 0 corrections: direct check
    if (flarm_check_crc(payload, len))
        return 0;

    unsigned data_bits = (len - 2) * 8;

    // 1-bit correction
    if (max_corrections >= 1) {
        for (unsigned b1 = 0; b1 < data_bits; b1++) {
            payload[b1 / 8] ^= (0x80 >> (b1 % 8));
            if (flarm_check_crc(payload, len))
                return 1;
            payload[b1 / 8] ^= (0x80 >> (b1 % 8));  // undo
        }
    }

    // 2-bit correction
    if (max_corrections >= 2) {
        for (unsigned b1 = 0; b1 < data_bits - 1; b1++) {
            payload[b1 / 8] ^= (0x80 >> (b1 % 8));
            for (unsigned b2 = b1 + 1; b2 < data_bits; b2++) {
                payload[b2 / 8] ^= (0x80 >> (b2 % 8));
                if (flarm_check_crc(payload, len))
                    return 2;
                payload[b2 / 8] ^= (0x80 >> (b2 % 8));  // undo b2
            }
            payload[b1 / 8] ^= (0x80 >> (b1 % 8));  // undo b1
        }
    }

    return -1;  // uncorrectable
}

// ======================== Try decode FLARM ========================

static void try_decode_flarm(struct flarm_state *state, flarm_channel_t *ch)
{
    uint8_t manchester_bits[PAYLOAD_CHIPS];
    extract_payload_bits(ch->fm_ring, ch->flarm_payload_start,
                         ch->flarm_polarity, manchester_bits);

    uint8_t payload[FLARM_PACKET_TOTAL];
    unsigned payload_len = 0;
    unsigned violations = 0;

    if (!manchester_decode_payload(manchester_bits, PAYLOAD_CHIPS, payload, &payload_len, &violations)) {
        state->stats.packets_failed++;
        return;
    }
    if (payload_len < FLARM_PACKET_TOTAL) {
        state->stats.packets_failed++;
        return;
    }

    // Try up to 2-bit correction for signals with few violations (likely real)
    // With >5 violations, it's likely noise — skip expensive correction
    unsigned max_corr = (violations <= 5) ? 2 : 0;
    int corrections = try_bit_correction(payload, FLARM_PACKET_TOTAL, max_corr);

    if (corrections < 0) {
        // Debug: dump failed payload for analysis
        fprintf(stderr, "flarm-crc-fail ncc=%.3f pol=%d viol=%u start=%u bytes=%02x%02x%02x%02x%02x%02x%02x%02x"
                "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
                ch->flarm_det_ncc, ch->flarm_polarity, violations,
                ch->flarm_payload_start,
                payload[0],payload[1],payload[2],payload[3],payload[4],payload[5],
                payload[6],payload[7],payload[8],payload[9],payload[10],payload[11],
                payload[12],payload[13],payload[14],payload[15],payload[16],payload[17],
                payload[18],payload[19],payload[20],payload[21],payload[22],payload[23],
                payload[24],payload[25]);
        state->stats.packets_failed++;
        return;
    }

    state->stats.packets_crc_ok++;
    fprintf(stderr, "FLARM CRC OK! corrections=%d ncc=%.3f payload=%02x%02x%02x%02x%02x%02x "
            "(detected=%llu crc_ok=%llu)\n",
            corrections, ch->flarm_det_ncc,
            payload[0], payload[1], payload[2], payload[3], payload[4], payload[5],
            (unsigned long long)state->stats.packets_detected,
            (unsigned long long)state->stats.packets_crc_ok);

    uint32_t now = state->time_override ? state->time_override : (uint32_t)time(NULL);
    flarm_message_t msg;

    // When replaying a file, scan a wider time window (file mtime = end of capture)
    int scan_range = state->time_override ? 35 : 1;

    for (int dt = 0; dt <= scan_range; dt++) {
        // Try dt=0 first, then -1,+1,-2,+2,...
        int offsets[2] = { -dt, dt };
        int n_offsets = (dt == 0) ? 1 : 2;
        for (int k = 0; k < n_offsets; k++) {
            if (flarm_decode_packet(payload, state->config.ref_lat, state->config.ref_lon,
                                    state->config.ref_alt_geoid, now + offsets[k], &msg)) {
                msg.signal_level = 0;

                // Validation: wrong timestamp produces garbage — try next
                // Skip stealth check for file replay (time_override != 0)
                if (msg.stealth && !state->time_override) continue;
                if (msg.addr == 0x000000 || msg.addr == 0xFFFFFF) continue;
                if ((msg.addr & 0xFF) == ((msg.addr >> 8) & 0xFF) &&
                    (msg.addr & 0xFF) == ((msg.addr >> 16) & 0xFF)) continue;
                if (msg.altitude < -500 || msg.altitude > 15000) continue;
                if (msg.speed < 0 || msg.speed > 500) continue;
                // Skip distance filter for file replay (time_override != 0)
                if (!state->time_override) {
                    double dlat = msg.latitude - state->config.ref_lat;
                    double dlon = msg.longitude - state->config.ref_lon;
                    if (sqrt(dlat*dlat + dlon*dlon) > 3.0) continue;
                }

                state->stats.packets_decoded++;
                fprintf(stderr, "FLARM DECODED! addr=%06X lat=%.4f lon=%.4f alt=%d vs=%.0f spd=%.0f dt=%d\n",
                        msg.addr, msg.latitude, msg.longitude,
                        msg.altitude, (double)msg.vs, msg.speed, offsets[k]);

                if (state->config.callback)
                    state->config.callback(&msg, state->config.callback_ctx);
                return;
            }
        }
    }
    state->stats.packets_failed++;
}

// ======================== Try decode OGNTP ========================

static void try_decode_ogntp(struct flarm_state *state, flarm_channel_t *ch)
{
    uint8_t manchester_bits[PAYLOAD_CHIPS];
    extract_payload_bits(ch->fm_ring, ch->ogntp_payload_start,
                         ch->ogntp_polarity, manchester_bits);

    uint8_t payload[OGNTP_PACKET_TOTAL];
    unsigned payload_len = 0;

    unsigned ogntp_violations = 0;
    if (!manchester_decode_payload(manchester_bits, PAYLOAD_CHIPS, payload, &payload_len, &ogntp_violations)) {
        state->stats.ogntp_packets_failed++;
        return;
    }
    if (payload_len < OGNTP_PACKET_TOTAL) {
        state->stats.ogntp_packets_failed++;
        return;
    }

    if (ogntp_ldpc_check(payload) != 0) {
        state->stats.ogntp_packets_failed++;
        return;
    }
    state->stats.ogntp_packets_ldpc_ok++;

    ogntp_message_t msg;
    if (ogntp_decode_packet(payload, state->config.ref_lat, state->config.ref_lon, &msg)) {
        state->stats.ogntp_packets_decoded++;
        msg.signal_level = 0.0f;
        if (state->config.ogntp_callback)
            state->config.ogntp_callback(&msg, state->config.ogntp_callback_ctx);
    } else {
        state->stats.ogntp_packets_failed++;
    }
}

// ======================== Create / Destroy ========================

struct flarm_state *flarm_demod_create(const flarm_demod_config_t *config)
{
    struct flarm_state *s = calloc(1, sizeof(struct flarm_state));
    if (!s) return NULL;

    init_templates();
    s->config = *config;

    uint32_t center = config->center_freq;
    if (center == 0) center = FLARM_CENTER_FREQ;

    for (int ch = 0; ch < FLARM_NUM_CHANNELS; ch++) {
        flarm_channel_t *c = &s->channels[ch];

        double offset_hz = (double)FLARM_CHANNEL_FREQS[ch] - (double)center;
        double phase_inc = -2.0 * M_PI * offset_hz / FLARM_SAMPLE_RATE;
        c->nco_cos = 1.0f;
        c->nco_sin = 0.0f;
        c->nco_cos_inc = (float)cos(phase_inc);
        c->nco_sin_inc = (float)sin(phase_inc);
        c->nco_count = 0;

        memset(&c->fir, 0, sizeof(c->fir));
        c->fir.pos = 0;

        c->prev_i = 0;
        c->prev_q = 0;
        c->dc_avg = 0;

        memset(c->fm_ring, 0, sizeof(c->fm_ring));
        c->ring_wr = 0;
        c->corr_countdown = CORR_CHECK_INTERVAL;
        c->lockout_remaining = 0;

        c->flarm_collecting = 0;
        c->ogntp_collecting = 0;

        c->diag_best_flarm_ncc = 0;
        c->diag_best_ogntp_ncc = 0;
        c->diag_fm_sum = 0;
        c->diag_fm_count = 0;

        fprintf(stderr, "flarm-ch%d: %.3f MHz, NCO offset %+.1f kHz, LPF %.0f kHz, "
                "NCC threshold %.2f, check every %d samp\n",
                ch, FLARM_CHANNEL_FREQS[ch] / 1e6, offset_hz / 1e3,
                CHANNEL_LPF_CUTOFF / 1e3, SYNC_NCC_THRESHOLD, CORR_CHECK_INTERVAL);
    }

    s->diag_report_interval = 5 * FLARM_SAMPLE_RATE;
    s->diag_last_report = 0;

    return s;
}

void flarm_demod_destroy(struct flarm_state *state)
{
    free(state);
}

void flarm_demod_set_position(struct flarm_state *state, double lat, double lon, float alt_geoid)
{
    state->config.ref_lat = lat;
    state->config.ref_lon = lon;
    state->config.ref_alt_geoid = alt_geoid;
}

void flarm_demod_set_time_override(struct flarm_state *state, uint32_t base_time)
{
    state->time_override = base_time;
}

void flarm_demod_get_stats(struct flarm_state *state, flarm_demod_stats_t *stats)
{
    *stats = state->stats;
}

// ======================== Main IQ processing ========================

void flarm_demod_process(struct flarm_state *state, const uint8_t *iq_data, unsigned len)
{
    unsigned n_samples = len / 2;
    state->stats.samples_processed += n_samples;

    for (unsigned i = 0; i < n_samples; i++) {
        float raw_i = (float)((int)iq_data[i * 2]     - 128);
        float raw_q = (float)((int)iq_data[i * 2 + 1] - 128);

        for (int ch_idx = 0; ch_idx < FLARM_NUM_CHANNELS; ch_idx++) {
            flarm_channel_t *ch = &state->channels[ch_idx];

            // ---- NCO mixing (complex rotation) ----
            float mix_i = raw_i * ch->nco_cos - raw_q * ch->nco_sin;
            float mix_q = raw_i * ch->nco_sin + raw_q * ch->nco_cos;

            // Advance NCO phase via complex multiplication
            float new_cos = ch->nco_cos * ch->nco_cos_inc - ch->nco_sin * ch->nco_sin_inc;
            float new_sin = ch->nco_sin * ch->nco_cos_inc + ch->nco_cos * ch->nco_sin_inc;
            ch->nco_cos = new_cos;
            ch->nco_sin = new_sin;

            // Periodic renormalization to prevent amplitude drift
            if (++ch->nco_count >= 1024) {
                float mag = ch->nco_cos * ch->nco_cos + ch->nco_sin * ch->nco_sin;
                float scale = 1.5f - 0.5f * mag;  // fast inverse sqrt approximation
                ch->nco_cos *= scale;
                ch->nco_sin *= scale;
                ch->nco_count = 0;
            }

            // ---- FIR LPF (linear phase) ----
            float filt_i, filt_q;
            fir_process(&ch->fir, mix_i, mix_q, &filt_i, &filt_q);

            // ---- FM discriminator ----
            float cross = filt_q * ch->prev_i - filt_i * ch->prev_q;
            float dot   = filt_i * ch->prev_i + filt_q * ch->prev_q;
            ch->prev_i = filt_i;
            ch->prev_q = filt_q;
            float fm_out = atan2f(cross, dot);

            // ---- Store raw FM in ring buffer (no DC block) ----
            // DC is handled per-window in NCC (robust formula) and in payload extraction
            float fm = fm_out;
            ch->dc_avg = DC_BLOCK_ALPHA * ch->dc_avg + (1.0f - DC_BLOCK_ALPHA) * fm_out;  // track for diagnostics only

            // ---- Store in ring buffer ----
            ch->fm_ring[ch->ring_wr] = fm;
            ch->ring_wr = (ch->ring_wr + 1) & FM_RING_MASK;

            // ---- Diagnostics ----
            ch->diag_fm_sum += fabsf(fm);
            ch->diag_fm_count++;

            // ---- Payload collection countdown ----
            if (ch->flarm_collecting) {
                ch->flarm_samples_needed--;
                if (ch->flarm_samples_needed == 0) {
                    try_decode_flarm(state, ch);
                    ch->flarm_collecting = 0;
                }
            }
            if (ch->ogntp_collecting) {
                ch->ogntp_samples_needed--;
                if (ch->ogntp_samples_needed == 0) {
                    try_decode_ogntp(state, ch);
                    ch->ogntp_collecting = 0;
                }
            }

            // ---- Lockout ----
            if (ch->lockout_remaining > 0) {
                ch->lockout_remaining--;
                continue;
            }

            // ---- Periodic NCC check ----
            ch->corr_countdown--;
            if (ch->corr_countdown > 0)
                continue;
            ch->corr_countdown = CORR_CHECK_INTERVAL;

            // Sync would end at current ring_wr, starts SYNC_TEMPLATE_LEN before
            unsigned sync_start = (ch->ring_wr - SYNC_TEMPLATE_LEN) & FM_RING_MASK;

            // ---- FLARM sync detection ----
            if (!ch->flarm_collecting) {
                float ncc = compute_ncc_ring(ch->fm_ring, sync_start,
                                            flarm_sync_template, flarm_template_energy);
                float abs_ncc = fabsf(ncc);

                if (abs_ncc > ch->diag_best_flarm_ncc)
                    ch->diag_best_flarm_ncc = abs_ncc;

                if (abs_ncc >= SYNC_NCC_THRESHOLD) {
                    // Refine to single-sample accuracy
                    float refined_ncc;
                    unsigned best_start = refine_sync(ch->fm_ring, sync_start,
                                                     flarm_sync_template,
                                                     flarm_template_energy, &refined_ncc);
                    float signed_ncc = compute_ncc_ring(ch->fm_ring, best_start,
                                                       flarm_sync_template,
                                                       flarm_template_energy);

                    state->stats.packets_detected++;
                    ch->flarm_collecting = 1;
                    ch->flarm_polarity = (signed_ncc > 0) ? 1 : -1;
                    ch->flarm_det_ncc = refined_ncc;
                    ch->flarm_payload_start = (best_start + SYNC_TEMPLATE_LEN) & FM_RING_MASK;
                    ch->flarm_samples_needed = PAYLOAD_SAMPLES;
                    ch->lockout_remaining = SYNC_LOCKOUT_SAMPLES;
                }
            }

            // ---- OGNTP sync detection ----
            if (!ch->ogntp_collecting) {
                float oncc = compute_ncc_ring(ch->fm_ring, sync_start,
                                             ogntp_sync_template, ogntp_template_energy);
                float abs_oncc = fabsf(oncc);

                if (abs_oncc > ch->diag_best_ogntp_ncc)
                    ch->diag_best_ogntp_ncc = abs_oncc;

                if (abs_oncc >= SYNC_NCC_THRESHOLD) {
                    float refined_ncc;
                    unsigned best_start = refine_sync(ch->fm_ring, sync_start,
                                                     ogntp_sync_template,
                                                     ogntp_template_energy, &refined_ncc);
                    float signed_ncc = compute_ncc_ring(ch->fm_ring, best_start,
                                                       ogntp_sync_template,
                                                       ogntp_template_energy);

                    state->stats.ogntp_packets_detected++;
                    ch->ogntp_collecting = 1;
                    ch->ogntp_polarity = (signed_ncc > 0) ? 1 : -1;
                    ch->ogntp_payload_start = (best_start + SYNC_TEMPLATE_LEN) & FM_RING_MASK;
                    ch->ogntp_samples_needed = PAYLOAD_SAMPLES;
                }
            }
        }
    }

    // ---- Periodic diagnostic report ----
    if (state->stats.samples_processed - state->diag_last_report >= state->diag_report_interval) {
        for (int ch = 0; ch < FLARM_NUM_CHANNELS; ch++) {
            flarm_channel_t *c = &state->channels[ch];
            float avg_fm = (c->diag_fm_count > 0) ?
                            c->diag_fm_sum / c->diag_fm_count : 0;
            fprintf(stderr, "flarm-diag ch%d (%.1fMHz): best_ncc=%.3f ogntp_ncc=%.3f "
                    "avg_fm=%.4f dc=%.4f det=%llu crc=%llu fail=%llu\n",
                    ch, FLARM_CHANNEL_FREQS[ch] / 1e6,
                    c->diag_best_flarm_ncc, c->diag_best_ogntp_ncc,
                    avg_fm, c->dc_avg,
                    (unsigned long long)state->stats.packets_detected,
                    (unsigned long long)state->stats.packets_crc_ok,
                    (unsigned long long)state->stats.packets_failed);
            c->diag_best_flarm_ncc = 0;
            c->diag_best_ogntp_ncc = 0;
            c->diag_fm_sum = 0;
            c->diag_fm_count = 0;
        }
        state->diag_last_report = state->stats.samples_processed;
    }
}
