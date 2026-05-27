// p3i_decode.h — PilotAware P3I protocol decoder
//
// Based on public GPL-3.0 source: SoftRF P3I.h/P3I.cpp by Linar Yusupov
// RF params from SoftRF: 869.525 MHz, 2-FSK, 38.4 kbps, ±10 kHz dev
//
// Copyright (C) 2026 — GPL-3.0-or-later

#ifndef P3I_DECODE_H
#define P3I_DECODE_H

#include <stdint.h>


// ======================== RF parameters ========================

#define P3I_FREQ            869525000   // 869.525 MHz
#define P3I_BITRATE         38400       // 38.4 kbps
#define P3I_DEVIATION       10000       // ±10 kHz FSK deviation

// ======================== Packet structure ========================
// From SoftRF P3I.h (GPL-3.0):
//   Preamble: 10 bytes 0xAA
//   Syncword: 0xB4, 0x2B
//   Net ID:   4 bytes 0x00
//   Length:    1 byte (0x18 = 24)
//   CRC seed: 1 byte (0x71)
//   Payload:  24 bytes (XOR-whitened)
//   CRC-8:    1 byte (polynomial 0x107)

#define P3I_SYNCWORD_0      0xB4
#define P3I_SYNCWORD_1      0x2B
#define P3I_PAYLOAD_SIZE    24
#define P3I_NET_ID_SIZE     4
#define P3I_CRC_SEED        0x71

// Total over-the-air bytes after syncword: NetID(4) + Len(1) + CRCseed(1) + Payload(24) + CRC(1) = 31
#define P3I_FRAME_BYTES     31

// ======================== Decoded message ========================

typedef struct {
    uint32_t addr;          // 24-bit PilotAware address
    double   latitude;      // degrees
    double   longitude;     // degrees
    int32_t      altitude;      // metres GNSS
    float    speed;         // knots ground speed
    float    course;        // degrees true
    int32_t      aircraft_type; // PAW aircraft type byte
    float    signal_level;  // 0-1 relative
    int32_t      valid;         // non-zero = successfully decoded
} p3i_message_t;

// ======================== Whitening pattern ========================
// From SoftRF P3I.cpp (GPL-3.0) — NiceRF SV650 whitening table

extern const uint8_t p3i_whitening[P3I_PAYLOAD_SIZE];

// ======================== Functions ========================

// Decode a raw 24-byte (de-whitened) P3I payload into a message
// Returns true if CRC valid and message parsed
bool p3i_decode_packet(const uint8_t *payload, p3i_message_t *msg);

// Apply/remove whitening from payload (XOR with fixed pattern)
void p3i_dewhiten(uint8_t *data, int32_t len);

// CRC-8 with polynomial 0x107
uint8_t p3i_crc8(const uint8_t *data, int32_t len);

#endif // P3I_DECODE_H
