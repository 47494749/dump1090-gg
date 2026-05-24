// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// adsl_decode.h: ADS-L (SRD-860) protocol decoder
//
// ADS-L is the EASA Electronic Conspicuity standard for SRD-860 band.
// It uses 2-FSK at 100 kbps with Manchester encoding on 868.2/868.4 MHz,
// sharing the same RF parameters and frequency channels as FLARM/OGN.
// The packet format uses OGN-style position encoding with XXTEA whitening
// and Mode-S CRC-24 integrity check.
//
// Protocol references:
//   https://www.easa.europa.eu/sites/default/files/dfu/ads-l_4_srd860_issue_1.pdf
//   https://github.com/lyusupov/SoftRF (GPL-3.0)
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#ifndef ADSL_DECODE_H
#define ADSL_DECODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// ======================== Protocol constants ========================

// ADS-L syncword (Manchester-encoded). Pre-Manchester = 0xF5724B18.
#define ADSL_SYNCWORD_SIZE   8
extern const uint8_t ADSL_SYNCWORD[ADSL_SYNCWORD_SIZE];

// Packet structure: 1 byte version + 20 bytes data (5 x 32-bit words) + 3 bytes CRC-24
#define ADSL_VERSION_SIZE    1
#define ADSL_DATA_SIZE       20
#define ADSL_PAYLOAD_SIZE    21      // version + data
#define ADSL_CRC_SIZE        3
#define ADSL_PACKET_TOTAL    24      // payload + CRC

// ======================== Decoded message ========================

typedef struct {
    uint32_t addr;              // 24-bit device address
    uint8_t  addr_type;         // 0=random, 1=ICAO, 2=FLARM, 3=OGN
    uint8_t  aircraft_type;     // OGN aircraft type (0-15)

    double   latitude;          // degrees
    double   longitude;         // degrees
    int32_t      altitude;          // meters above MSL (geoid corrected)
    float    speed;             // m/s ground speed
    float    course;            // degrees (0-360) ground track
    float    vs;                // m/s vertical speed
    float    signal_level;      // signal strength (0..1)
    bool     valid;             // decode successful
} adsl_message_t;

// ======================== Functions ========================

// Check ADS-L CRC-24 (Mode-S polynomial) of a 24-byte packet.
// Returns true if CRC is valid (remainder = 0).
bool adsl_check_crc(const uint8_t *data24);

// Decode a 24-byte ADS-L packet (after Manchester decoding).
// data24: 24 bytes (1 version + 20 data + 3 CRC), CRC already verified.
// ref_lat, ref_lon: receiver position for sanity checks.
// ref_alt_geoid: receiver geoid separation in meters.
// Returns true on success; fills *msg with decoded fields.
bool adsl_decode_packet(const uint8_t *data24,
                        double ref_lat, double ref_lon,
                        float ref_alt_geoid,
                        adsl_message_t *msg);

#ifdef __cplusplus
}
#endif

#endif // ADSL_DECODE_H
