// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// acars_demod.h: ACARS AM-MSK demodulator for VHF ACARS reception
//
// Based on algorithms from acarsdec by Thierry Leconte (GPLv2)
// Integrated into dump1090-gg as an internal decoder module.

#ifndef ACARS_DEMOD_H
#define ACARS_DEMOD_H

#include <stdint.h>
#include <stdbool.h>

#define ACARS_INTRATE     12500     // Internal sample rate per channel (Hz)
#define ACARS_MAX_CHANNELS 8       // Max simultaneous ACARS channels
#define ACARS_MAX_MSGLEN   250     // Max ACARS message text length
#define ACARS_BAUD_RATE    2400    // ACARS MSK baud rate
#define ACARS_DECIM_BUFSZ  1024    // Decimated buffer size per block

// Forward declaration
struct acars_state;

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
    char     msgno[5];             // Message number (4 chars)
    char     flight[7];            // Flight ID (6 chars)
    char     text[ACARS_MAX_MSGLEN + 1]; // Message text
    int      text_len;             // Text length
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
void acars_process(struct acars_state *state, const uint8_t *iq_data, unsigned len);

// Statistics
typedef struct {
    uint64_t samples_processed;
    uint64_t messages_decoded;
    uint64_t crc_errors;
} acars_stats_t;

void acars_get_stats(struct acars_state *state, acars_stats_t *stats);

#endif // ACARS_DEMOD_H
