// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// flarm_demod.h: GFSK demodulator for FLARM 868 MHz packets
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#ifndef FLARM_DEMOD_H
#define FLARM_DEMOD_H

#include <stdint.h>
#include <stdbool.h>
#include "flarm_decode.h"
#include "ogntp_decode.h"

// Forward declaration
struct flarm_state;

// Callback invoked when a complete, CRC-valid FLARM packet is demodulated
typedef void (*flarm_packet_cb)(const flarm_message_t *msg, void *ctx);

// Callback invoked when a complete, LDPC-valid OGNTP packet is demodulated
typedef void (*ogntp_packet_cb)(const ogntp_message_t *msg, void *ctx);

// Demodulator configuration
typedef struct {
    double   ref_lat;           // Receiver latitude
    double   ref_lon;           // Receiver longitude
    float    ref_alt_geoid;     // Receiver geoid separation (meters)
    flarm_packet_cb callback;   // FLARM packet callback
    void    *callback_ctx;      // User context for FLARM callback
    ogntp_packet_cb ogntp_callback;     // OGNTP packet callback
    void           *ogntp_callback_ctx; // User context for OGNTP callback
} flarm_demod_config_t;

// Create and initialize a FLARM demodulator instance
struct flarm_state *flarm_demod_create(const flarm_demod_config_t *config);

// Destroy a FLARM demodulator instance
void flarm_demod_destroy(struct flarm_state *state);

// Feed raw IQ samples (uint8_t interleaved I,Q pairs) to the demodulator
// len = number of bytes (= 2 * number of IQ samples)
void flarm_demod_process(struct flarm_state *state, const uint8_t *iq_data, unsigned len);

// Update receiver position (can be called at any time)
void flarm_demod_set_position(struct flarm_state *state, double lat, double lon, float alt_geoid);

// Get stats
typedef struct {
    uint64_t samples_processed;
    uint64_t packets_detected;          // FLARM syncword matches
    uint64_t packets_crc_ok;            // FLARM CRC valid
    uint64_t packets_decoded;           // FLARM decode successful
    uint64_t packets_failed;            // FLARM decode failed
    uint64_t ogntp_packets_detected;    // OGNTP syncword matches
    uint64_t ogntp_packets_ldpc_ok;     // OGNTP LDPC passed
    uint64_t ogntp_packets_decoded;     // OGNTP decode successful
    uint64_t ogntp_packets_failed;      // OGNTP decode failed
} flarm_demod_stats_t;

void flarm_demod_get_stats(struct flarm_state *state, flarm_demod_stats_t *stats);

#endif // FLARM_DEMOD_H
