// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sarsat_decode.h: Cospas-Sarsat 406 MHz beacon decoder
//
// Decodes 406 MHz emergency distress beacons (ELT, EPIRB, PLB, SSAS):
//   - 1st generation: BPSK modulation via FM, 400 baud, Biphase-L encoding
//   - Frame: 15-bit preamble + 9-bit frame sync + 120 data bits
//   - Short (112-bit) and Long (144-bit) message formats
//   - BCH(82,61) t=3 on PDF-1, BCH(38,26) t=2 on PDF-2
//   - Protocol parsing per C/S T.001 (country, ID, MMSI, position)
//
// Channels (all within 406.0-406.1 MHz, 100 kHz total):
//   406.025, 406.028, 406.037, 406.040 MHz
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#ifndef SARSAT_DECODE_H
#define SARSAT_DECODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// ======================== Constants ========================

#define SARSAT_SAMPLE_RATE      2400000   // 2.4 MSPS default
#define SARSAT_CENTER_FREQ      406040000 // 406.040 MHz center (covers all channels)
#define SARSAT_BAUD_RATE        400       // Symbol rate
#define SARSAT_FRAME_BITS       144       // Max frame bits (long format)
#define SARSAT_PREAMBLE_LEN     15        // Bit sync preamble length
#define SARSAT_FRAMESYNC_LEN    9         // Frame sync word length
#define SARSAT_PDF1_LEN         61        // PDF-1 data bits
#define SARSAT_BCH1_LEN         21        // BCH-1 parity bits
#define SARSAT_PDF2_LEN         26        // PDF-2 data bits
#define SARSAT_BCH2_LEN         12        // BCH-2 parity bits

// Frame sync patterns (9 bits after preamble)
#define SARSAT_SYNC_NORMAL      0x0D0     // 011010000 = normal/distress
#define SARSAT_SYNC_TEST        0x02F     // 000101111 = self-test

// ======================== Beacon types ========================

typedef enum {
    SARSAT_BEACON_UNKNOWN = 0,
    SARSAT_BEACON_ELT,        // Emergency Locator Transmitter (aircraft)
    SARSAT_BEACON_EPIRB,      // Emergency Position Indicating Radio Beacon (maritime)
    SARSAT_BEACON_PLB,        // Personal Locator Beacon
    SARSAT_BEACON_SSAS,       // Ship Security Alert System
    SARSAT_BEACON_ELT_DT      // ELT with Data Transmission
} sarsat_beacon_type_t;

// ======================== Protocol codes ========================

typedef enum {
    SARSAT_PROTO_UNKNOWN      = -1,
    SARSAT_PROTO_ORBITOGRAPHY  = 0x02,
    SARSAT_PROTO_ELT_SERIAL   = 0x03,
    SARSAT_PROTO_ELT_OPERATOR = 0x04,
    SARSAT_PROTO_ELT_AIRCRAFT = 0x05,
    SARSAT_PROTO_EPIRB_MMSI   = 0x06,
    SARSAT_PROTO_EPIRB_RADIO  = 0x08,
    SARSAT_PROTO_SHIP_MMSI    = 0x09,
    SARSAT_PROTO_PLB_SERIAL   = 0x0B,
    SARSAT_PROTO_NAT_LOC      = 0x0C,
    SARSAT_PROTO_STD_TEST     = 0x0E,
    SARSAT_PROTO_NAT_TEST     = 0x0F,
    SARSAT_PROTO_ELT_DT       = 0x01
} sarsat_protocol_t;

// ======================== Decoded message ========================

typedef struct {
    bool     valid;
    bool     is_test;             // Self-test message (vs distress)
    bool     long_message;        // true=144-bit, false=112-bit

    // Identification
    int32_t      country_code;        // MID / country code (10-bit)
    char     country_name[48];    // Resolved country name
    sarsat_protocol_t protocol;
    sarsat_beacon_type_t beacon_type;

    // Beacon ID
    char     hex_id[16];          // 15-char hex ID (bits 26-85) + NUL
    uint32_t serial_number;       // Serial or beacon number
    uint32_t cert_number;         // Type approval certificate number

    // Maritime ID
    char     mmsi[16];            // MMSI string (for EPIRB/SSAS)
    char     call_sign[16];       // Radio call sign

    // Aircraft ID
    uint32_t icao_address;        // 24-bit ICAO address (for ELT)
    char     aircraft_operator[8];// Operator designator

    // Position
    bool     position_valid;
    double   latitude;            // Decimal degrees (negative = South)
    double   longitude;           // Decimal degrees (negative = West)
    bool     position_from_gps;   // true = external GPS, false = internal nav
    bool     homing_121_5;        // 121.5 MHz homing device present

    // BCH error correction status
    bool     bch1_valid;
    bool     bch2_valid;
    int32_t      bch1_errors;         // Errors corrected in BCH-1
    int32_t      bch2_errors;         // Errors corrected in BCH-2

    // Signal info
    float    snr;                 // Signal-to-noise ratio estimate (dB)

    // Raw frame
    uint8_t  raw_bits[SARSAT_FRAME_BITS];
} sarsat_msg_t;

// ======================== Statistics ========================

typedef struct {
    uint64_t samples_processed;
    uint64_t bursts_detected;     // Preamble + frame sync matches
    uint64_t frames_decoded;      // Passed BCH-1 validation
    uint64_t bch1_corrected;      // BCH-1 errors corrected
    uint64_t bch2_corrected;      // BCH-2 errors corrected
    uint64_t bch1_failed;         // BCH-1 uncorrectable
    uint64_t bch2_failed;         // BCH-2 uncorrectable
} sarsat_stats_t;

// ======================== Callback & Config ========================

typedef void (*sarsat_msg_cb)(const sarsat_msg_t *msg, void *ctx);

typedef struct {
    double     center_freq;
    double     sample_rate;
    sarsat_msg_cb callback;
    void      *callback_ctx;
} sarsat_config_t;

// ======================== Opaque state ========================

struct sarsat_state;

// ======================== API ========================

struct sarsat_state *sarsat_create(const sarsat_config_t *config);
void sarsat_destroy(struct sarsat_state *state);
void sarsat_process(struct sarsat_state *state, const uint8_t *iq_data, uint32_t len);
void sarsat_flush(struct sarsat_state *state);
void sarsat_get_stats(struct sarsat_state *state, sarsat_stats_t *stats);

// Utility: get human-readable names
const char *sarsat_beacon_type_name(sarsat_beacon_type_t type);
const char *sarsat_protocol_name(sarsat_protocol_t proto);
const char *sarsat_country_name(int32_t mid);

#ifdef __cplusplus
}
#endif

#endif // SARSAT_DECODE_H
