// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// flarm_decode.h: FLARM protocol decoder (V6, V7, and type-1 header)
//
// Based on protocol reverse-engineering by Stanislaw Pusep and
// SoftRF project by Linar Yusupov (GPL-3.0).
// Keys from public open-source implementations (SoftRF, OGN Tracker).
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#ifndef FLARM_DECODE_H
#define FLARM_DECODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// FLARM radio parameters
#define FLARM_FREQ_EU_1         868200000   // 868.2 MHz
#define FLARM_FREQ_EU_2         868400000   // 868.4 MHz
#define FLARM_FREQ_P3I          869525000   // 869.525 MHz (PilotAware)
#define FLARM_SAMPLE_RATE       1600000     // 1.6 MSPS (FLARM-only, no P3I)
#define FLARM_SAMPLE_RATE_P3I   2400000     // 2.4 MSPS (with P3I enabled)
#define FLARM_CENTER_FREQ       868300000   // Center between 868.2 and 868.4
#define FLARM_CENTER_FREQ_P3I   868850000   // Center covering 868.2-869.525 MHz

#define FLARM_BITRATE           100000      // 100 kbps
#define FLARM_DEVIATION         50000       // ±50 kHz FSK deviation

// Packet structure
#define FLARM_SYNCWORD_SIZE     8
#define FLARM_PAYLOAD_SIZE      24          // bytes
#define FLARM_CRC_SIZE          2           // CCITT-FFFF CRC16
#define FLARM_PACKET_TOTAL      (FLARM_PAYLOAD_SIZE + FLARM_CRC_SIZE)

// Manchester-encoded syncword (same as OGN: pre-Manchester = 0x0AF3656C)
// Modern FLARM devices use this same syncword on 868.2/868.4 MHz
static const uint8_t FLARM_SYNCWORD[FLARM_SYNCWORD_SIZE] = {
    0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A
};

// Aircraft type codes (same as OGN)
enum flarm_aircraft_type {
    FLARM_ACFT_UNKNOWN     = 0,
    FLARM_ACFT_GLIDER      = 1,
    FLARM_ACFT_TOWPLANE    = 2,
    FLARM_ACFT_HELICOPTER  = 3,
    FLARM_ACFT_PARACHUTE   = 4,
    FLARM_ACFT_DROPPLANE   = 5,
    FLARM_ACFT_HANGGLIDER  = 6,
    FLARM_ACFT_PARAGLIDER  = 7,
    FLARM_ACFT_POWERED     = 8,
    FLARM_ACFT_JET         = 9,
    FLARM_ACFT_UFO         = 10,
    FLARM_ACFT_BALLOON     = 11,
    FLARM_ACFT_ZEPPELIN    = 12,
    FLARM_ACFT_UAV         = 13,
    FLARM_ACFT_RESERVED    = 14,
    FLARM_ACFT_STATIC      = 15
};

// Address type
enum flarm_addr_type {
    FLARM_ADDR_RANDOM    = 0,
    FLARM_ADDR_ICAO      = 1,
    FLARM_ADDR_FLARM     = 2,
    FLARM_ADDR_ANONYMOUS = 3
};

// Decoded FLARM position message
typedef struct {
    uint32_t addr;              // 24-bit device address
    uint8_t  addr_type;         // address type (enum flarm_addr_type)
    uint8_t  aircraft_type;     // aircraft type (enum flarm_aircraft_type)
    uint8_t  stealth;           // stealth mode flag
    uint8_t  no_track;          // no-tracking flag

    double   latitude;          // degrees
    double   longitude;         // degrees
    int32_t      altitude;          // meters above MSL (geoid corrected)
    float    speed;             // m/s ground speed
    float    course;            // degrees (0-360) ground track
    float    vs;                // m/s vertical speed
    float    turnrate;          // degrees/sec (V7 only)
    uint8_t  airborne;          // 0=unknown, 1=ground, 2=airborne, 3=circling (V7)

    uint32_t timestamp;         // unix epoch used for decoding
    uint8_t  version;           // 6, 7, or 1 (V6 pos, V7 pos, or other)
    float    signal_level;      // signal strength (0..1)
    bool     valid;             // decode successful
    bool     header_only;       // true if only header was decoded (type 1)
} flarm_message_t;

// Initialize FLARM decoder (call once at startup)
void flarm_decode_init(void);

// Load FLARM decryption keys from a text file.
// Must be called before flarm_decode_init() for decryption to work.
// Returns true if all keys loaded successfully.
bool flarm_load_keys(const char *path);

// Load built-in decryption keys (from public SoftRF/OGN sources).
// Use when no key file is provided on the command line.
void flarm_load_keys_builtin(void);

// Decode a raw FLARM packet (after Manchester decoding + CRC check)
// raw_payload: 24 bytes of payload data
// ref_lat, ref_lon: receiver reference position (needed for coordinate decoding)
// ref_alt_geoid: receiver geoid separation in meters
// timestamp: current unix epoch seconds
// out: decoded message output
// Returns true if decoded successfully
bool flarm_decode_packet(const uint8_t *raw_payload,
                         double ref_lat, double ref_lon,
                         float ref_alt_geoid,
                         uint32_t timestamp,
                         flarm_message_t *out);

// CRC-16 CCITT (initial value 0xFFFF) check
// Returns true if CRC matches
bool flarm_check_crc(const uint8_t *data, uint32_t len);

// Compute CRC-16 CCITT
uint16_t flarm_crc16(const uint8_t *data, uint32_t len);

// Key accessors for panel display
int32_t flarm_keys_are_loaded(void);
void flarm_get_key_table(uint32_t out[12]);
uint32_t flarm_get_key2(void);
uint32_t flarm_get_key3(void);
uint32_t flarm_get_key4(void);
void flarm_get_key5(uint32_t out[4]);

#ifdef __cplusplus
}
#endif

#endif // FLARM_DECODE_H
