// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sonde_demod.h: Radiosonde (RS41) GFSK demodulator
//
// RS41 protocol (Vaisala):
//   - GFSK modulation at 4800 baud, ±2.4 kHz deviation
//   - Operating frequency: 400-406 MHz
//   - Frame: 320 bytes after syncword
//   - Reed-Solomon RS(255,231) ECC, two interleaved codewords
//   - CRC-16 CCITT on each sub-block
//   - XOR whitening (64-byte repeating mask, offset 8)
//
// Based on RS41 protocol analysis by rs1729 and radiosonde_auto_rx project.

#ifndef SONDE_DEMOD_H
#define SONDE_DEMOD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define SONDE_BAUD_RATE     4800     // RS41 baud rate
#define SONDE_FRAME_LEN     320      // RS41 frame length in bytes
#define SONDE_ID_LEN        16       // Sonde serial number length

// Reed-Solomon parameters: RS(255,231), GF(2^8), t=12
#define RS_NROOTS           24       // 2t check symbols
#define RS_T                12       // Error correction capability

// Forward declaration
struct sonde_state;

// Decoded radiosonde telemetry
typedef struct {
    char     serial[SONDE_ID_LEN];   // Sonde serial number (e.g., "S1234567")
    char     type[8];                // Sonde type ("RS41")
    double   lat;                    // Latitude (degrees)
    double   lon;                    // Longitude (degrees)
    double   alt;                    // Altitude MSL (meters)
    double   vel_h;                  // Horizontal velocity (m/s)
    double   heading;               // Heading (degrees)
    double   vel_v;                  // Vertical velocity (m/s)
    double   temp;                   // Temperature (°C)
    double   humidity;               // Relative humidity (%)
    int      frame_num;              // Frame counter
    float    freq;                   // Receive frequency (MHz)
    float    snr;                    // Signal-to-noise ratio (dB)
    bool     valid_pos;              // Position data valid
    int      rs_errors;              // RS errors corrected (-1 if uncorrectable)
    int      satellites;             // GPS satellites used
} sonde_msg_t;

// Callback for decoded messages
typedef void (*sonde_msg_cb)(const sonde_msg_t *msg, void *ctx);

// Configuration
typedef struct {
    double   center_freq;
    double   sample_rate;
    sonde_msg_cb callback;
    void    *callback_ctx;
} sonde_config_t;

struct sonde_state *sonde_create(const sonde_config_t *config);
void sonde_destroy(struct sonde_state *state);
void sonde_process(struct sonde_state *state, const uint8_t *iq_data, uint32_t len);

typedef struct {
    uint64_t samples_processed;
    uint64_t frames_detected;        // Syncword matches
    uint64_t frames_decoded;         // Passed RS + CRC validation
    uint64_t rs_corrected;           // Total RS symbol corrections
    uint64_t rs_uncorrectable;       // Frames with uncorrectable RS errors
    uint64_t crc_errors;             // Sub-block CRC failures
} sonde_stats_t;

void sonde_get_stats(struct sonde_state *state, sonde_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif // SONDE_DEMOD_H
