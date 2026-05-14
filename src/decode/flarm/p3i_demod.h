// p3i_demod.h — PilotAware P3I FSK demodulator
//
// Standalone FSK demodulator for the P3I (PilotAware) protocol.
// Operates on the same IQ stream as FLARM but with its own processing chain:
//   NCO → FIR LPF (narrow) → FM discriminator → NCC sync → bit extraction
//
// P3I uses 2-FSK, 38.4 kbps, ±10 kHz deviation on 869.525 MHz.
// No Manchester encoding — plain NRZ with MSB-first byte order.
//
// This file is free software: GPL-3.0-or-later

#ifndef P3I_DEMOD_H
#define P3I_DEMOD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "p3i_decode.h"

// Forward declaration
struct p3i_demod_state;

// Callback invoked when a complete, CRC-valid P3I packet is demodulated
typedef void (*p3i_demod_packet_cb)(const p3i_message_t *msg, void *ctx);

// Demodulator configuration
typedef struct {
    uint32_t sample_rate;       // SDR sample rate (e.g. 2400000)
    uint32_t center_freq;       // SDR center frequency (e.g. 868850000)
    p3i_demod_packet_cb callback;
    void    *callback_ctx;
} p3i_demod_config_t;

// Stats
typedef struct {
    uint64_t samples_processed;
    uint64_t sync_detected;     // Syncword correlation matches
    uint64_t packets_decoded;   // Successfully decoded P3I packets
    uint64_t packets_failed;    // Decode/CRC failures
} p3i_demod_stats_t;

// Create and initialize P3I demodulator
struct p3i_demod_state *p3i_demod_create(const p3i_demod_config_t *config);

// Destroy P3I demodulator
void p3i_demod_destroy(struct p3i_demod_state *state);

// Feed raw IQ samples (uint8_t interleaved I,Q pairs)
// len = number of bytes (= 2 * number of IQ samples)
void p3i_demod_process(struct p3i_demod_state *state, const uint8_t *iq_data, uint32_t len);

// Get stats
void p3i_demod_get_stats(struct p3i_demod_state *state, p3i_demod_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif // P3I_DEMOD_H
