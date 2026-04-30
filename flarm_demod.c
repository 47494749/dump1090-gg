// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// flarm_demod.c: GFSK demodulator for FLARM 868 MHz packets
//
// FLARM uses 2-FSK (GFSK) modulation at 100 kbps with ±50 kHz deviation.
// The radio link uses Manchester encoding, an 8-byte syncword, and
// CRC-16 CCITT. After demodulation and CRC check, packets are passed
// to the FLARM protocol decoder for XXTEA decryption.
//
// Demodulation pipeline:
//   IQ samples (1.6 MSPS) → FM discriminator → low-pass → bit slicer
//   → Manchester decode → syncword correlator → payload extraction
//   → CRC check → FLARM decrypt/decode
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

// At 1.6 MSPS and 100 kbps, we have 16 samples per bit
#define SAMPLES_PER_BIT     16

// Manchester encoding: each bit is 2 half-bits, so 8 samples per Manchester half-bit
#define SAMPLES_PER_HALF    (SAMPLES_PER_BIT / 2)

// Total Manchester-encoded bit length for syncword + payload + CRC
// After sync detection: 24 + 2 = 26 bytes = 208 bits = 416 Manchester half-bits
#define PAYLOAD_MANCHESTER_BITS  (FLARM_PACKET_TOTAL * 8 * 2)

// Syncword in Manchester-encoded bit pattern (64 bits = 8 bytes)
#define SYNCWORD_BITS       (FLARM_SYNCWORD_SIZE * 8)

// Syncword correlation threshold (out of 64 bits, allow up to 4 errors)
#define SYNC_THRESHOLD      60

// DC-blocking filter coefficient
#define DC_BLOCK_ALPHA      0.9999f

// ======================== Channelizer ========================

// FLARM uses two frequencies in EU; we process both simultaneously
#define FLARM_NUM_CHANNELS   2

static const uint32_t FLARM_CHANNEL_FREQS[FLARM_NUM_CHANNELS] = {
    868200000,  // 868.2 MHz (EU freq 1)
    868400000,  // 868.4 MHz (EU freq 2)
};

// LPF cutoff for channel isolation.
// GFSK at 100 kbps with ±50 kHz deviation → BW ≈ 200 kHz.
// Single-pole IIR at 100 kHz gives adequate selectivity.
#define CHANNEL_LPF_CUTOFF   100000.0

// ======================== Per-channel state ========================

typedef struct {
    // NCO (frequency shifter to move channel to baseband)
    float nco_phase;
    float nco_phase_inc;    // radians per sample

    // Low-pass filter (single-pole IIR, separate I and Q)
    float lpf_i, lpf_q;
    float lpf_alpha;

    // FM discriminator (float: after mixing/LPF the signal is float)
    float prev_i, prev_q;

    // DC-blocking
    float dc_avg;

    // Bit clock recovery
    float bit_accumulator;
    int bit_sample_count;

    // Syncword detection
    uint64_t shift_reg;

    // FLARM packet assembly
    int      in_packet;
    uint8_t  packet_manchester[PAYLOAD_MANCHESTER_BITS];
    unsigned packet_mbit_count;

    // OGNTP packet assembly
    int      in_ogntp_packet;
    uint8_t  ogntp_manchester[PAYLOAD_MANCHESTER_BITS];
    unsigned ogntp_mbit_count;

    // Per-channel diagnostics
    int diag_best_flarm_match;
    int diag_best_ogntp_match;
    float diag_fm_sum;
    unsigned diag_fm_count;
} flarm_channel_t;

// ======================== Overall state ========================

struct flarm_state {
    flarm_demod_config_t config;

    // Channelized receivers
    flarm_channel_t channels[FLARM_NUM_CHANNELS];

    // Diagnostics timing
    uint64_t diag_report_interval;
    uint64_t diag_last_report;

    // Stats
    flarm_demod_stats_t stats;
};

// ======================== Helper: popcount64 ========================

static int popcount64(uint64_t x)
{
    // Portable bit count
    x = x - ((x >> 1) & 0x5555555555555555ULL);
    x = (x & 0x3333333333333333ULL) + ((x >> 2) & 0x3333333333333333ULL);
    x = (x + (x >> 4)) & 0x0F0F0F0F0F0F0F0FULL;
    return (int)((x * 0x0101010101010101ULL) >> 56);
}

// ======================== Create / Destroy ========================

struct flarm_state *flarm_demod_create(const flarm_demod_config_t *config)
{
    struct flarm_state *s = calloc(1, sizeof(struct flarm_state));
    if (!s) return NULL;

    s->config = *config;

    // Default center frequency if not specified
    uint32_t center = config->center_freq;
    if (center == 0) center = FLARM_CENTER_FREQ;

    // Initialize channelized receivers
    for (int ch = 0; ch < FLARM_NUM_CHANNELS; ch++) {
        flarm_channel_t *c = &s->channels[ch];

        // NCO: shift channel frequency down to baseband
        double offset_hz = (double)FLARM_CHANNEL_FREQS[ch] - (double)center;
        c->nco_phase = 0;
        c->nco_phase_inc = (float)(-2.0 * M_PI * offset_hz / FLARM_SAMPLE_RATE);

        // LPF coefficient: single-pole IIR
        c->lpf_alpha = 1.0f - expf((float)(-2.0 * M_PI * CHANNEL_LPF_CUTOFF / FLARM_SAMPLE_RATE));
        c->lpf_i = 0;
        c->lpf_q = 0;

        // FM discriminator
        c->prev_i = 0;
        c->prev_q = 0;
        c->dc_avg = 0;

        // Bit clock
        c->bit_accumulator = 0;
        c->bit_sample_count = 0;

        // Sync + packet
        c->shift_reg = 0;
        c->in_packet = 0;
        c->packet_mbit_count = 0;
        c->in_ogntp_packet = 0;
        c->ogntp_mbit_count = 0;

        // Diagnostics
        c->diag_best_flarm_match = 0;
        c->diag_best_ogntp_match = 0;
        c->diag_fm_sum = 0;
        c->diag_fm_count = 0;

        fprintf(stderr, "flarm-ch%d: %.3f MHz, NCO offset %+.1f kHz, LPF %.0f kHz\n",
                ch, FLARM_CHANNEL_FREQS[ch] / 1e6, offset_hz / 1e3, CHANNEL_LPF_CUTOFF / 1e3);
    }

    // Diagnostics: report every 5 seconds (5 * 1.6M = 8M samples)
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

void flarm_demod_get_stats(struct flarm_state *state, flarm_demod_stats_t *stats)
{
    *stats = state->stats;
}

// ======================== Build syncword match pattern ========================

static uint64_t build_syncword_pattern(void)
{
    uint64_t pattern = 0;
    for (int i = 0; i < FLARM_SYNCWORD_SIZE; i++) {
        pattern = (pattern << 8) | FLARM_SYNCWORD[i];
    }
    return pattern;
}

// ======================== Manchester decode ========================
// Manchester encoding (IEEE): 0 = low→high (01), 1 = high→low (10)
// FLARM uses inverted payload: 0 = 10, 1 = 01
// Each pair of Manchester half-bits → one data bit

static bool manchester_decode_payload(const uint8_t *manchester_bits, unsigned n_manchester_bits,
                                      uint8_t *out_bytes, unsigned *out_len)
{
    unsigned n_data_bits = n_manchester_bits / 2;
    unsigned n_bytes = n_data_bits / 8;

    if (n_data_bits % 8 != 0) return false;

    *out_len = n_bytes;
    memset(out_bytes, 0, n_bytes);

    for (unsigned i = 0; i < n_data_bits; i++) {
        uint8_t first  = manchester_bits[i * 2];
        uint8_t second = manchester_bits[i * 2 + 1];

        // FLARM uses inverted Manchester: 10 = 0, 01 = 1
        uint8_t data_bit;
        if (first == 1 && second == 0) {
            data_bit = 0;  // inverted: high-low = 0
        } else if (first == 0 && second == 1) {
            data_bit = 1;  // inverted: low-high = 1
        } else {
            // Manchester violation — could be an error
            // Use majority logic: treat as whichever transition is closer
            data_bit = second;  // best guess
        }

        out_bytes[i / 8] |= (data_bit << (7 - (i % 8)));
    }

    return true;
}

// ======================== Try to decode a packet ========================

static void try_decode_packet(struct flarm_state *state, flarm_channel_t *ch)
{
    uint8_t payload[FLARM_PACKET_TOTAL];
    unsigned payload_len = 0;

    // Manchester-decode the collected half-bits into bytes
    if (!manchester_decode_payload(ch->packet_manchester,
                                   ch->packet_mbit_count,
                                   payload, &payload_len)) {
        state->stats.packets_failed++;
        return;
    }

    if (payload_len < FLARM_PACKET_TOTAL) {
        state->stats.packets_failed++;
        return;
    }

    // CRC check (on all 26 bytes: 24 payload + 2 CRC)
    if (!flarm_check_crc(payload, FLARM_PACKET_TOTAL)) {
        state->stats.packets_failed++;
        return;
    }

    state->stats.packets_crc_ok++;

    // FLARM protocol decode (XXTEA decrypt + coordinate decode)
    uint32_t now = (uint32_t)time(NULL);
    flarm_message_t msg;

    if (flarm_decode_packet(payload, state->config.ref_lat, state->config.ref_lon,
                            state->config.ref_alt_geoid, now, &msg)) {
        state->stats.packets_decoded++;
        msg.signal_level = 0;  // TODO: compute from FM discriminator amplitude

        // Skip stealth aircraft
        if (msg.stealth) return;

        // Sanity checks to filter noise packets that pass CRC + weak parity
        if (msg.addr == 0x000000 || msg.addr == 0xFFFFFF) return;
        // Reject repeating-byte addresses (likely noise pattern)
        if ((msg.addr & 0xFF) == ((msg.addr >> 8) & 0xFF) &&
            (msg.addr & 0xFF) == ((msg.addr >> 16) & 0xFF)) return;
        // Position must be within 300km of receiver
        double dlat = msg.latitude - state->config.ref_lat;
        double dlon = msg.longitude - state->config.ref_lon;
        double dist_deg = sqrt(dlat * dlat + dlon * dlon);
        if (dist_deg > 3.0) return;  // ~300km
        // Altitude sanity: -500m to 15000m
        if (msg.altitude < -500 || msg.altitude > 15000) return;
        // Speed sanity: 0 to 500 m/s (~1800 km/h)
        if (msg.speed < 0 || msg.speed > 500) return;

        // Invoke callback
        if (state->config.callback) {
            state->config.callback(&msg, state->config.callback_ctx);
        }
    } else {
        // V7 packets use timestamp >> 4 in the key, so try adjacent seconds
        for (int dt = -1; dt <= 1; dt++) {
            if (dt == 0) continue;
            if (flarm_decode_packet(payload, state->config.ref_lat, state->config.ref_lon,
                                    state->config.ref_alt_geoid, now + dt, &msg)) {
                state->stats.packets_decoded++;
                msg.signal_level = 0;
                if (msg.stealth) return;
                if (state->config.callback) {
                    state->config.callback(&msg, state->config.callback_ctx);
                }
                return;
            }
        }
        state->stats.packets_failed++;
    }
}

// ======================== OGNTP try-decode ========================

static void try_decode_ogntp_packet(struct flarm_state *state, flarm_channel_t *ch)
{
    uint8_t payload[OGNTP_PACKET_TOTAL];
    unsigned payload_len = 0;

    // Manchester-decode using same inverted convention as FLARM
    if (!manchester_decode_payload(ch->ogntp_manchester,
                                   ch->ogntp_mbit_count,
                                   payload, &payload_len)) {
        state->stats.ogntp_packets_failed++;
        return;
    }

    if (payload_len < OGNTP_PACKET_TOTAL) {
        state->stats.ogntp_packets_failed++;
        return;
    }

    // LDPC parity check (replaces CRC for OGN1)
    if (ogntp_ldpc_check(payload) != 0) {
        state->stats.ogntp_packets_failed++;
        return;
    }
    state->stats.ogntp_packets_ldpc_ok++;

    // Protocol decode
    ogntp_message_t msg;
    if (ogntp_decode_packet(payload, state->config.ref_lat, state->config.ref_lon, &msg)) {
        state->stats.ogntp_packets_decoded++;
        msg.signal_level = 0.0f; // TODO: propagate FM amplitude

        if (state->config.ogntp_callback) {
            state->config.ogntp_callback(&msg, state->config.ogntp_callback_ctx);
        }
    } else {
        state->stats.ogntp_packets_failed++;
    }
}

// ======================== Process one bit (per channel) ========================

static void process_bit(struct flarm_state *state, flarm_channel_t *ch, uint8_t bit)
{
    // Push into 64-bit shift register for syncword detection
    ch->shift_reg = (ch->shift_reg << 1) | (bit & 1);

    // ---- FLARM: collect bits or look for FLARM syncword ----
    if (ch->in_packet) {
        // Collecting FLARM payload Manchester bits
        ch->packet_manchester[ch->packet_mbit_count++] = bit;

        if (ch->packet_mbit_count >= PAYLOAD_MANCHESTER_BITS) {
            try_decode_packet(state, ch);
            ch->in_packet = 0;
            ch->packet_mbit_count = 0;
        }
    } else {
        static uint64_t sync_pattern = 0;
        if (sync_pattern == 0) {
            sync_pattern = build_syncword_pattern();
        }

        uint64_t diff = ch->shift_reg ^ sync_pattern;
        int errors = popcount64(diff);
        int match = 64 - errors;

        // Track best correlation for diagnostics
        if (match > ch->diag_best_flarm_match)
            ch->diag_best_flarm_match = match;

        if (errors <= (64 - SYNC_THRESHOLD)) {
            state->stats.packets_detected++;
            ch->in_packet = 1;
            ch->packet_mbit_count = 0;
        }
    }

    // ---- OGNTP: collect bits or look for OGNTP syncword (independent) ----
    if (ch->in_ogntp_packet) {
        // Collecting OGNTP payload Manchester bits
        ch->ogntp_manchester[ch->ogntp_mbit_count++] = bit;

        if (ch->ogntp_mbit_count >= PAYLOAD_MANCHESTER_BITS) {
            try_decode_ogntp_packet(state, ch);
            ch->in_ogntp_packet = 0;
            ch->ogntp_mbit_count = 0;
        }
    } else {
        static uint64_t ogntp_sync_pattern = 0;
        if (ogntp_sync_pattern == 0) {
            for (int i = 0; i < OGNTP_SYNCWORD_SIZE; i++)
                ogntp_sync_pattern = (ogntp_sync_pattern << 8) | OGNTP_SYNCWORD[i];
        }

        uint64_t ogntp_diff = ch->shift_reg ^ ogntp_sync_pattern;
        int ogntp_errors = popcount64(ogntp_diff);
        int ogntp_match = 64 - ogntp_errors;

        // Track best correlation for diagnostics
        if (ogntp_match > ch->diag_best_ogntp_match)
            ch->diag_best_ogntp_match = ogntp_match;

        if (ogntp_errors <= (64 - SYNC_THRESHOLD)) {
            state->stats.ogntp_packets_detected++;
            ch->in_ogntp_packet = 1;
            ch->ogntp_mbit_count = 0;
        }
    }
}

// ======================== Main IQ processing (channelized) ========================

void flarm_demod_process(struct flarm_state *state, const uint8_t *iq_data, unsigned len)
{
    unsigned n_samples = len / 2;
    state->stats.samples_processed += n_samples;

    for (unsigned i = 0; i < n_samples; i++) {
        // Raw IQ centered around 0 (RTL-SDR gives unsigned 0-255)
        float raw_i = (float)((int)iq_data[i * 2]     - 128);
        float raw_q = (float)((int)iq_data[i * 2 + 1] - 128);

        // Process each channel independently
        for (int ch_idx = 0; ch_idx < FLARM_NUM_CHANNELS; ch_idx++) {
            flarm_channel_t *ch = &state->channels[ch_idx];

            // ---- NCO mixing: shift channel to baseband ----
            float cos_p = cosf(ch->nco_phase);
            float sin_p = sinf(ch->nco_phase);
            float mix_i = raw_i * cos_p - raw_q * sin_p;
            float mix_q = raw_i * sin_p + raw_q * cos_p;

            ch->nco_phase += ch->nco_phase_inc;
            // Keep phase in [-π, π] to avoid float precision loss
            if (ch->nco_phase > (float)M_PI)  ch->nco_phase -= 2.0f * (float)M_PI;
            if (ch->nco_phase < -(float)M_PI) ch->nco_phase += 2.0f * (float)M_PI;

            // ---- Low-pass filter (single-pole IIR) ----
            ch->lpf_i += ch->lpf_alpha * (mix_i - ch->lpf_i);
            ch->lpf_q += ch->lpf_alpha * (mix_q - ch->lpf_q);

            // ---- FM discriminator on filtered signal ----
            float cross = ch->lpf_i * ch->prev_q - ch->lpf_q * ch->prev_i;
            float dot   = ch->lpf_i * ch->prev_i + ch->lpf_q * ch->prev_q;

            ch->prev_i = ch->lpf_i;
            ch->prev_q = ch->lpf_q;

            float fm_out;
            float denom = fabsf(dot) + fabsf(cross);
            if (denom < 1e-10f) {
                fm_out = 0.0f;
            } else {
                fm_out = cross / denom;
            }

            // ---- DC-blocking filter ----
            ch->dc_avg = DC_BLOCK_ALPHA * ch->dc_avg + (1.0f - DC_BLOCK_ALPHA) * fm_out;
            float fm_corrected = fm_out - ch->dc_avg;

            // ---- Diagnostics: FM amplitude ----
            ch->diag_fm_sum += fabsf(fm_corrected);
            ch->diag_fm_count++;

            // ---- Bit clock: every SAMPLES_PER_HALF, output one Manchester half-bit ----
            ch->bit_accumulator += fm_corrected;
            ch->bit_sample_count++;

            if (ch->bit_sample_count >= SAMPLES_PER_HALF) {
                uint8_t half_bit = (ch->bit_accumulator > 0) ? 1 : 0;
                process_bit(state, ch, half_bit);

                ch->bit_accumulator = 0;
                ch->bit_sample_count = 0;
            }
        }
    }

    // Periodic diagnostic report (per-channel)
    if (state->stats.samples_processed - state->diag_last_report >= state->diag_report_interval) {
        for (int ch = 0; ch < FLARM_NUM_CHANNELS; ch++) {
            flarm_channel_t *c = &state->channels[ch];
            float avg_fm = (c->diag_fm_count > 0) ?
                            c->diag_fm_sum / c->diag_fm_count : 0;
            fprintf(stderr, "flarm-diag ch%d (%.1fMHz): best_sync FLARM=%d/64 OGNTP=%d/64 "
                    "avg_fm=%.4f dc=%.4f detected=%llu\n",
                    ch, FLARM_CHANNEL_FREQS[ch] / 1e6,
                    c->diag_best_flarm_match,
                    c->diag_best_ogntp_match,
                    avg_fm,
                    c->dc_avg,
                    (unsigned long long)state->stats.packets_detected);
            c->diag_best_flarm_match = 0;
            c->diag_best_ogntp_match = 0;
            c->diag_fm_sum = 0;
            c->diag_fm_count = 0;
        }
        state->diag_last_report = state->stats.samples_processed;
    }
}
