// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// pocsag_demod.h: POCSAG pager decoder — FSK demodulation, sync detection,
//                 BCH error correction, alpha/numeric message extraction.
//
// Supports 512, 1200, and 2400 baud POCSAG.
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#ifndef POCSAG_DEMOD_H
#define POCSAG_DEMOD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// ======================== Constants ========================

#define POCSAG_SYNC_WORD        0x7CD215D8u
#define POCSAG_IDLE_WORD        0x7A89C197u
#define POCSAG_BATCH_WORDS      17       // 1 sync + 16 data codewords
#define POCSAG_MSG_MAX_LEN      256      // max decoded message text length
#define POCSAG_PREAMBLE_BITS    576      // minimum preamble bits (ITU-R M.584)
#define POCSAG_PREAMBLE_MIN     16       // practical minimum to gate sync acceptance

#define POCSAG_MAX_CHANNELS     8        // max simultaneous POCSAG channels
#define POCSAG_SAMPLE_RATE      2400000  // wideband sample rate for multi-channel

// Function codes
#define POCSAG_FUNC_NUMERIC     0
#define POCSAG_FUNC_TONE        3
// 1,2 = alpha

// ======================== Types ========================

// Decoded POCSAG message
typedef struct {
    uint32_t address;                       // 21-bit address (3-bit function excluded)
    int      function;                      // 0-3 function bits
    char     alpha_msg[POCSAG_MSG_MAX_LEN]; // decoded alpha text (NUL-terminated)
    char     numeric_msg[POCSAG_MSG_MAX_LEN]; // decoded numeric string
    int      alpha_len;
    int      numeric_len;
    bool     is_alpha;                      // true if function indicates alpha
    bool     is_numeric;                    // true if function indicates numeric
    bool     is_tone_only;                  // true if tone-only alert
    int      baud_rate;                     // detected baud rate
    int      errors_corrected;             // BCH error corrections applied
    float    signal_level;                  // estimated signal level
    double   channel_freq;                  // channel frequency in Hz
} pocsag_msg_t;

// Message callback
typedef void (*pocsag_callback_t)(const pocsag_msg_t *msg, void *ctx);

// Decoder configuration
typedef struct {
    double           center_freq;       // SDR center frequency in Hz
    double           sample_rate;       // SDR sample rate in Hz
    double           channel_freqs[POCSAG_MAX_CHANNELS]; // channel frequencies in Hz (0 = unused)
    int              num_channels;      // number of active channels (0 = single-channel legacy)
    pocsag_callback_t callback;         // message callback
    void            *callback_ctx;      // opaque context for callback
} pocsag_config_t;

// Decoder statistics
typedef struct {
    uint64_t samples_processed;
    uint64_t preambles_detected;
    uint64_t syncs_detected;
    uint64_t messages_decoded;
    uint64_t bch_corrections;
    uint64_t bch_failures;
} pocsag_stats_t;

// Opaque decoder state
struct pocsag_state;

// ======================== API ========================

// Create a POCSAG decoder with the given configuration.
// Returns NULL on failure.
struct pocsag_state *pocsag_create(const pocsag_config_t *cfg);

// Destroy a POCSAG decoder and free resources.
void pocsag_destroy(struct pocsag_state *st);

// Process a block of IQ samples (interleaved uint8_t I,Q pairs).
void pocsag_process(struct pocsag_state *st, const uint8_t *iq_data, uint32_t len);

// Get decoder statistics.
void pocsag_get_stats(const struct pocsag_state *st, pocsag_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif // POCSAG_DEMOD_H
