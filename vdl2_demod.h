// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// vdl2_demod.h: VDL Mode 2 D8PSK demodulator
//
// VDL Mode 2 uses D8PSK at 31500 symbols/sec on 136.975 MHz (Europe).
// Integrated into dump1090-gg as an internal decoder module.

#ifndef VDL2_DEMOD_H
#define VDL2_DEMOD_H

#include <stdint.h>
#include <stdbool.h>

#define VDL2_SYMBOL_RATE    31500     // symbols/sec
#define VDL2_BITS_PER_SYM   3         // D8PSK: 3 bits per symbol
#define VDL2_MAX_FRAME_LEN  2048      // max AVLC frame bytes
#define VDL2_MAX_MSG_LEN    512       // max extracted ACARS text
#define VDL2_MAX_CHANNELS   8         // max simultaneous VDL2 channels

// Forward declaration
struct vdl2_state;

// AVLC address
typedef struct {
    uint32_t addr;          // 24-bit ICAO address
} vdl2_addr_t;

// Decoded VDL2 message (ACARS over AVLC)
typedef struct {
    double   freq;
    float    level;
    float    snr;
    bool     has_acars;
    char     frame_type[8]; // "I", "S", "U" etc.
    vdl2_addr_t src;
    vdl2_addr_t dst;
    int      info_len;
    char     reg[8];
    char     flight[7];
    char     label[3];
    char     text[VDL2_MAX_MSG_LEN + 1];
    int      text_len;
} vdl2_msg_t;

// Callback for decoded messages
typedef void (*vdl2_msg_cb)(const vdl2_msg_t *msg, void *ctx);

// Configuration
typedef struct {
    double   center_freq;
    double   sample_rate;
    double   channel_freqs[VDL2_MAX_CHANNELS]; // VDL2 channel frequencies (Hz)
    int      num_channels;     // Number of channels to monitor
    float    squelch_level;    // Squelch level in dBFS
    vdl2_msg_cb callback;
    void    *callback_ctx;
} vdl2_config_t;

// Default VDL2 frequencies (Europe)
#define VDL2_FREQ_EU_PRIMARY   136975000.0
#define VDL2_FREQ_EU_1         VDL2_FREQ_EU_PRIMARY
#define VDL2_FREQ_EU_2         136875000.0
#define VDL2_FREQ_EU_3         136775000.0

struct vdl2_state *vdl2_create(const vdl2_config_t *config);
void vdl2_destroy(struct vdl2_state *state);
void vdl2_process(struct vdl2_state *state, const uint8_t *iq_data, unsigned len);

typedef struct {
    uint64_t samples_processed;
    uint64_t frames_detected;
    uint64_t messages_decoded;
    uint64_t fcs_errors;
} vdl2_stats_t;

void vdl2_get_stats(struct vdl2_state *state, vdl2_stats_t *stats);

#endif // VDL2_DEMOD_H
