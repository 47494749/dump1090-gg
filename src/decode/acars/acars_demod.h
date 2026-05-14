// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// acars_demod.h: ACARS AM-MSK demodulator for VHF ACARS reception
//
// Based on algorithms from acarsdec by Thierry Leconte (GPLv2)
// Integrated into dump1090-gg as an internal decoder module.

#ifndef ACARS_DEMOD_H
#define ACARS_DEMOD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define ACARS_INTRATE     12500     // Internal sample rate per channel (Hz)
#define ACARS_MAX_CHANNELS 8       // Max simultaneous ACARS channels
#define ACARS_MAX_MSGLEN   250     // Max ACARS message text length
#define ACARS_BAUD_RATE    2400    // ACARS MSK baud rate
#define ACARS_DECIM_BUFSZ  1024    // Decimated buffer size per block

// Forward declaration
struct acars_state;

// ACARS message categories (mirrored from acars_label.h for convenience)
typedef enum {
    ACARS_MSG_CAT_UNKNOWN   = 0,
    ACARS_MSG_CAT_ATC       = 1,
    ACARS_MSG_CAT_AOC       = 2,
    ACARS_MSG_CAT_AAC       = 3,
    ACARS_MSG_CAT_SERVICE   = 4,
    ACARS_MSG_CAT_EMERGENCY = 5,
    ACARS_MSG_CAT_WEATHER   = 6,
    ACARS_MSG_CAT_PRINTER   = 7,
} acars_msg_category_t;

// Decoded ACARS message
typedef struct {
    int      channel;              // Channel index
    double   freq;                 // Frequency (Hz)
    float    level;                // Signal level (dB)
    int      errors;               // Parity errors corrected
    char     mode;                 // Mode character
    char     reg[8];               // Aircraft registration
    char     ack;                  // ACK character
    char     label[3];             // Label (2 chars)
    char     block_id;             // Block identifier
    char     dsp_header[33];       // Optional DSP route header (without surrounding '/')
    char     dsp_destination[17];  // DSP destination before the '.'
    char     dsp_route[17];        // DSP route/header after the '.'
    char     msgno[5];             // Message number (4 chars)
    char     flight[7];            // Flight ID (6 chars)
    char     sublabel[3];          // Optional 2-char sublabel after msgno/flight
    char     mfi[3];               // Optional 2-char message function identifier
    char     text[ACARS_MAX_MSGLEN + 1]; // Message text
    int      text_len;             // Text length
    // Label semantic fields (populated by label lookup)
    const char         *label_description;  // Human-readable label description (NULL if unknown)
    acars_msg_category_t label_category;    // Message category classification
} acars_msg_t;

// Callback invoked when a complete ACARS message is decoded
typedef void (*acars_msg_cb)(const acars_msg_t *msg, void *ctx);

// Configuration for ACARS demodulator
typedef struct {
    double   center_freq;          // RTL-SDR center frequency (Hz)
    double   sample_rate;          // RTL-SDR sample rate (Hz)
    double   channel_freqs[ACARS_MAX_CHANNELS]; // ACARS channel frequencies (Hz)
    int      num_channels;         // Number of channels to monitor
    acars_msg_cb callback;         // Message callback
    void    *callback_ctx;         // User context for callback
} acars_config_t;

// Default European ACARS frequencies
#define ACARS_FREQ_PRIMARY   131550000.0
#define ACARS_FREQ_SECONDARY 130025000.0
#define ACARS_FREQ_TERTIARY  131725000.0
#define ACARS_FREQ_4         130450000.0
#define ACARS_FREQ_5         129125000.0

// Create and initialize an ACARS demodulator instance
struct acars_state *acars_create(const acars_config_t *config);

// Destroy an ACARS demodulator instance
void acars_destroy(struct acars_state *state);

// Feed raw IQ samples (uint8_t interleaved I,Q pairs) to the demodulator
void acars_process(struct acars_state *state, const uint8_t *iq_data, uint32_t len);

// Statistics
typedef struct {
    uint64_t samples_processed;
    uint64_t messages_decoded;
    uint64_t crc_errors;
} acars_stats_t;

void acars_get_stats(struct acars_state *state, acars_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif // ACARS_DEMOD_H
