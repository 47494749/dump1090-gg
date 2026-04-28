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
// Syncword: 8 bytes = 64 bits (already Manchester encoded in the syncword pattern)
// Payload: 24 bytes = 192 bits → 384 Manchester bits
// CRC: 2 bytes = 16 bits → 32 Manchester bits
// But FLARM syncword is transmitted in Manchester. After sync detection,
// we need: 24 + 2 = 26 bytes = 208 bits = 416 Manchester half-bits
#define PAYLOAD_MANCHESTER_BITS  (FLARM_PACKET_TOTAL * 8 * 2)

// Syncword in Manchester-encoded bit pattern (64 bits = 8 bytes)
// Each byte of FLARM_SYNCWORD is already the Manchester representation
#define SYNCWORD_BITS       (FLARM_SYNCWORD_SIZE * 8)

// FM discriminator output buffer size (ring buffer)
#define FM_BUFFER_SIZE      65536

// Minimum signal level to attempt decode (arbitrary, tune empirically)
#define MIN_SIGNAL_LEVEL    0.02f

// Syncword correlation threshold (out of 64 bits, allow up to 4 errors)
#define SYNC_THRESHOLD      60

// DC-blocking filter coefficient
// At 1.6 MSPS: time constant = 1/(1-alpha)/Fs = 1/0.0001/1.6e6 = 6.25 ms
// This tracks slow DC drift from frequency offset without tracking data
#define DC_BLOCK_ALPHA      0.9999f

// ======================== State structure ========================

struct flarm_state {
    // Config
    flarm_demod_config_t config;

    // FM discriminator state
    int prev_i;     // Previous I sample (for FM discriminator)
    int prev_q;     // Previous Q sample
    float dc_avg;   // DC-blocking: running average of FM output

    // Bit-level demodulation
    float fm_buffer[FM_BUFFER_SIZE];  // Ring buffer of FM discriminator output
    unsigned fm_write_pos;            // Write position in ring buffer

    // Bit clock recovery
    float bit_accumulator;  // Accumulated FM output for current bit period
    int bit_sample_count;   // Samples accumulated for current bit period
    float clock_phase;      // Phase tracking for bit clock (0..SAMPLES_PER_BIT)

    // Manchester bit stream buffer
    uint8_t manchester_bits[1024]; // Circular buffer of raw (pre-Manchester) bits
    unsigned mbit_write_pos;

    // Syncword detection
    uint64_t shift_reg;     // 64-bit shift register for syncword matching

    // FLARM packet assembly
    int      in_packet;     // Currently receiving a FLARM packet
    unsigned packet_bits;   // Bits received since sync
    uint8_t  packet_manchester[PAYLOAD_MANCHESTER_BITS]; // FLARM Manchester half-bits
    unsigned packet_mbit_count;

    // OGNTP packet assembly
    int      in_ogntp_packet;
    uint8_t  ogntp_manchester[PAYLOAD_MANCHESTER_BITS]; // OGNTP Manchester half-bits
    unsigned ogntp_mbit_count;

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
    s->prev_i = 0;
    s->prev_q = 0;
    s->dc_avg = 0;
    s->fm_write_pos = 0;
    s->bit_sample_count = 0;
    s->bit_accumulator = 0;
    s->clock_phase = 0;
    s->mbit_write_pos = 0;
    s->shift_reg = 0;
    s->in_packet = 0;
    s->packet_mbit_count = 0;
    s->in_ogntp_packet = 0;
    s->ogntp_mbit_count = 0;

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

static void try_decode_packet(struct flarm_state *state)
{
    uint8_t payload[FLARM_PACKET_TOTAL];
    unsigned payload_len = 0;

    // Manchester-decode the collected half-bits into bytes
    if (!manchester_decode_payload(state->packet_manchester,
                                   state->packet_mbit_count,
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

static void try_decode_ogntp_packet(struct flarm_state *state)
{
    uint8_t payload[OGNTP_PACKET_TOTAL];
    unsigned payload_len = 0;

    // Manchester-decode using same inverted convention as FLARM
    if (!manchester_decode_payload(state->ogntp_manchester,
                                   state->ogntp_mbit_count,
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

// ======================== Process one bit ========================

static void process_bit(struct flarm_state *state, uint8_t bit)
{
    // Push into 64-bit shift register for syncword detection
    state->shift_reg = (state->shift_reg << 1) | (bit & 1);

    // ---- FLARM: collect bits or look for FLARM syncword ----
    if (state->in_packet) {
        // Collecting FLARM payload Manchester bits
        state->packet_manchester[state->packet_mbit_count++] = bit;

        if (state->packet_mbit_count >= PAYLOAD_MANCHESTER_BITS) {
            try_decode_packet(state);
            state->in_packet = 0;
            state->packet_mbit_count = 0;
        }
    } else {
        static uint64_t sync_pattern = 0;
        if (sync_pattern == 0) {
            sync_pattern = build_syncword_pattern();
        }

        uint64_t diff = state->shift_reg ^ sync_pattern;
        int errors = popcount64(diff);

        if (errors <= (64 - SYNC_THRESHOLD)) {
            state->stats.packets_detected++;
            state->in_packet = 1;
            state->packet_mbit_count = 0;
        }
    }

    // ---- OGNTP: collect bits or look for OGNTP syncword (independent) ----
    if (state->in_ogntp_packet) {
        // Collecting OGNTP payload Manchester bits
        state->ogntp_manchester[state->ogntp_mbit_count++] = bit;

        if (state->ogntp_mbit_count >= PAYLOAD_MANCHESTER_BITS) {
            try_decode_ogntp_packet(state);
            state->in_ogntp_packet = 0;
            state->ogntp_mbit_count = 0;
        }
    } else {
        static uint64_t ogntp_sync_pattern = 0;
        if (ogntp_sync_pattern == 0) {
            for (int i = 0; i < OGNTP_SYNCWORD_SIZE; i++)
                ogntp_sync_pattern = (ogntp_sync_pattern << 8) | OGNTP_SYNCWORD[i];
        }

        uint64_t ogntp_diff = state->shift_reg ^ ogntp_sync_pattern;
        int ogntp_errors = popcount64(ogntp_diff);

        if (ogntp_errors <= (64 - SYNC_THRESHOLD)) {
            state->stats.ogntp_packets_detected++;
            state->in_ogntp_packet = 1;
            state->ogntp_mbit_count = 0;
        }
    }
}

// ======================== Main IQ processing ========================

void flarm_demod_process(struct flarm_state *state, const uint8_t *iq_data, unsigned len)
{
    unsigned n_samples = len / 2;
    state->stats.samples_processed += n_samples;

    for (unsigned i = 0; i < n_samples; i++) {
        // Get I/Q centered around 0 (RTL-SDR gives unsigned 0-255)
        int cur_i = (int)iq_data[i * 2]     - 128;
        int cur_q = (int)iq_data[i * 2 + 1] - 128;

        // FM discriminator: phase difference between consecutive samples
        // atan2(cross, dot) where cross = I[n]*Q[n-1] - Q[n]*I[n-1]
        //                         dot   = I[n]*I[n-1] + Q[n]*Q[n-1]
        int cross = cur_i * state->prev_q - cur_q * state->prev_i;
        int dot   = cur_i * state->prev_i + cur_q * state->prev_q;

        state->prev_i = cur_i;
        state->prev_q = cur_q;

        // Fast atan2 approximation: just use cross/dot ratio
        // For FSK, we only need the sign and rough magnitude
        float fm_out;
        if (dot == 0) {
            fm_out = (cross > 0) ? 1.0f : ((cross < 0) ? -1.0f : 0.0f);
        } else {
            fm_out = (float)cross / (float)(abs(dot) + abs(cross));
        }

        // DC-blocking filter: remove frequency offset bias
        // Without this, RTL-SDR PPM error shifts the FM output baseline,
        // causing the bit slicer to fail when offset exceeds FSK deviation
        state->dc_avg = DC_BLOCK_ALPHA * state->dc_avg + (1.0f - DC_BLOCK_ALPHA) * fm_out;
        float fm_corrected = fm_out - state->dc_avg;

        // Accumulate for bit decision
        state->bit_accumulator += fm_corrected;
        state->bit_sample_count++;

        // Bit clock: every SAMPLES_PER_HALF samples, output one Manchester half-bit
        if (state->bit_sample_count >= SAMPLES_PER_HALF) {
            uint8_t half_bit = (state->bit_accumulator > 0) ? 1 : 0;
            process_bit(state, half_bit);

            state->bit_accumulator = 0;
            state->bit_sample_count = 0;
        }
    }
}
