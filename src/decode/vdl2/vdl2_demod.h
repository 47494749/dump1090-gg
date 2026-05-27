// SPDX-License-Identifier: GPL-3.0-or-later
//
// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// vdl2_demod.h: VDL Mode 2 D8PSK demodulator
//
// VDL Mode 2 uses D8PSK at 10500 symbols/sec (31500 bps) on 136.975 MHz (Europe).
// Integrated into dump1090-gg as an internal decoder module.

#ifndef VDL2_DEMOD_H
#define VDL2_DEMOD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define VDL2_SYMBOL_RATE    10500     // symbols/sec (D8PSK: 3 bits/sym → 31500 bps)
#define VDL2_BITS_PER_SYM   3         // D8PSK: 3 bits per symbol
#define VDL2_MAX_FRAME_LEN  2048      // max AVLC frame bytes
#define VDL2_MAX_MSG_LEN    512       // max extracted ACARS text
#define VDL2_MAX_CHANNELS   8         // max simultaneous VDL2 channels

// Upper-layer protocol detected in AVLC information field
typedef enum {
    VDL2_PROTO_UNKNOWN    = 0,   // Unrecognized payload
    VDL2_PROTO_ACARS      = 1,   // ACARS message (SOH marker found)
    VDL2_PROTO_ISO8208    = 2,   // ISO 8208 / X.25 packet layer (ATN routing)
    VDL2_PROTO_CLNP       = 3,   // CLNP (Connectionless Network Protocol, ATN/OSI)
    VDL2_PROTO_IDRP       = 4,   // IDRP (Inter-Domain Routing Protocol)
    VDL2_PROTO_XID        = 5,   // XID exchange (link management)
    VDL2_PROTO_SNDCF      = 6,   // Subnetwork Dependent Convergence Function
} vdl2_proto_t;

// AVLC frame subtypes
typedef enum {
    VDL2_FRAME_I     = 0,   // Information frame
    VDL2_FRAME_S_RR  = 1,   // Supervisory: Receive Ready
    VDL2_FRAME_S_RNR = 2,   // Supervisory: Receive Not Ready
    VDL2_FRAME_S_REJ = 3,   // Supervisory: Reject
    VDL2_FRAME_S_SREJ= 4,   // Supervisory: Selective Reject
    VDL2_FRAME_U_SABM= 5,   // Unnumbered: Set Async Balanced Mode
    VDL2_FRAME_U_DISC= 6,   // Unnumbered: Disconnect
    VDL2_FRAME_U_DM  = 7,   // Unnumbered: Disconnect Mode
    VDL2_FRAME_U_UA  = 8,   // Unnumbered: Unnumbered Acknowledge
    VDL2_FRAME_U_FRMR=9,    // Unnumbered: Frame Reject
    VDL2_FRAME_U_XID = 10,  // Unnumbered: Exchange ID
    VDL2_FRAME_U_UI  = 11,  // Unnumbered: Unnumbered Information
    VDL2_FRAME_U_TEST= 12,  // Unnumbered: Test
    VDL2_FRAME_UNKNOWN=13,  // Unknown
} vdl2_frame_type_t;

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
    char     frame_type[8]; // "I", "S-RR", "U-SABM" etc.
    vdl2_frame_type_t frame_subtype; // Detailed frame type
    vdl2_proto_t proto;     // Detected upper-layer protocol
    const char  *proto_name; // Human-readable protocol name
    vdl2_addr_t src;
    vdl2_addr_t dst;
    int32_t      info_len;
    char     reg[8];
    char     flight[7];
    char     label[3];
    const char  *label_description; // Human-readable label description (NULL if unknown)
    int32_t      label_category;    // ACARS category (acars_category_t values)
    char     text[VDL2_MAX_MSG_LEN + 1];
    int32_t      text_len;
} vdl2_msg_t;

// Callback for decoded messages
typedef void (*vdl2_msg_cb)(const vdl2_msg_t *msg, void *ctx);

// Configuration
typedef struct {
    double   center_freq;
    double   sample_rate;
    double   channel_freqs[VDL2_MAX_CHANNELS]; // VDL2 channel frequencies (Hz)
    int32_t      num_channels;     // Number of channels to monitor
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
void vdl2_process(struct vdl2_state *state, const uint8_t *iq_data, uint32_t len);

typedef struct {
    uint64_t samples_processed;
    uint64_t frames_detected;
    uint64_t messages_decoded;
    uint64_t fcs_errors;
} vdl2_stats_t;

void vdl2_get_stats(struct vdl2_state *state, vdl2_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif // VDL2_DEMOD_H
