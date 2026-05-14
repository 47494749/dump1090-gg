// p3i_decode.c — PilotAware P3I protocol decoder
//
// Based on public GPL-3.0 source: SoftRF P3I.h/P3I.cpp by Linar Yusupov
// Protocol spec from: pilotaware.com Protocol.pdf (referenced in SoftRF)
//
// Copyright (C) 2026 — GPL-3.0-or-later

#include <string.h>
#include <math.h>
#include "p3i_decode.h"

// ======================== Whitening pattern ========================
// From SoftRF P3I.cpp (GPL-3.0): NiceRF SV650 XOR whitening table
// Applied to the 24-byte payload after Net ID + Length + CRC seed

const uint8_t p3i_whitening[P3I_PAYLOAD_SIZE] = {
    0x05, 0xb4, 0x05, 0xae, 0x14, 0xda, 0xbf, 0x83,
    0xc4, 0x04, 0xb2, 0x04, 0xd6, 0x4d, 0x87, 0xe2,
    0x01, 0xa3, 0x26, 0xac, 0xbb, 0x63, 0xf1, 0x01
};

// ======================== CRC-8 polynomial 0x107 ========================
// Generator polynomial x^8 + x^2 + x^1 + x^0 = 0x107 (reflected: 0xE0)
// Used by NiceRF SV650 module

uint8_t p3i_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;  // polynomial 0x107, MSB-first
            else
                crc <<= 1;
        }
    }
    return crc;
}

// ======================== De-whitening ========================

void p3i_dewhiten(uint8_t *data, int len)
{
    int wlen = (len < P3I_PAYLOAD_SIZE) ? len : P3I_PAYLOAD_SIZE;
    for (int i = 0; i < wlen; i++)
        data[i] ^= p3i_whitening[i];
}

// ======================== Packet decode ========================
// P3I packet layout (24 bytes, after de-whitening):
//   [0]      sync   = '$' (0x24)
//   [1..3]   icao   = 24-bit address (little-endian)
//   [4..7]   lon    = IEEE-754 float (little-endian)
//   [8..11]  lat    = IEEE-754 float (little-endian)
//   [12..13] alt    = uint16_t metres (little-endian)
//   [14..15] track  = uint16_t degrees (little-endian)
//   [16..19] msd    = sequencer (4 bytes, ignored)
//   [20..21] knots  = uint16_t ground speed (little-endian)
//   [22]     aircraft = aircraft type byte
//   [23]     crc    = XOR of bytes 0..22

bool p3i_decode_packet(const uint8_t *payload, p3i_message_t *msg)
{
    memset(msg, 0, sizeof(*msg));

    // De-whiten a local copy
    uint8_t pkt[P3I_PAYLOAD_SIZE];
    memcpy(pkt, payload, P3I_PAYLOAD_SIZE);
    p3i_dewhiten(pkt, P3I_PAYLOAD_SIZE);

    // Verify sync byte
    if (pkt[0] != 0x24)  // '$'
        return false;

    // XOR checksum: XOR of all bytes should be 0
    uint8_t cs = 0;
    for (int i = 0; i < P3I_PAYLOAD_SIZE; i++)
        cs ^= pkt[i];
    if (cs != 0)
        return false;

    // Extract fields (all little-endian)
    msg->addr = pkt[1] | ((uint32_t)pkt[2] << 8) | ((uint32_t)pkt[3] << 16);

    // IEEE-754 float for lon/lat
    float lon_f, lat_f;
    memcpy(&lon_f, &pkt[4], 4);
    memcpy(&lat_f, &pkt[8], 4);
    msg->longitude = (double)lon_f;
    msg->latitude  = (double)lat_f;

    // Sanity checks
    if (msg->latitude < -90.0 || msg->latitude > 90.0) return false;
    if (msg->longitude < -180.0 || msg->longitude > 180.0) return false;
    if (msg->addr == 0 || msg->addr == 0xFFFFFF) return false;

    msg->altitude = pkt[12] | ((uint16_t)pkt[13] << 8);
    msg->course   = (float)(pkt[14] | ((uint16_t)pkt[15] << 8));
    msg->speed    = (float)(pkt[20] | ((uint16_t)pkt[21] << 8));
    msg->aircraft_type = pkt[22];

    // Altitude sanity (metres, GNSS)
    if (msg->altitude > 20000) return false;

    msg->valid = 1;
    return true;
}
