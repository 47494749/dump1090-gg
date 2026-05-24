// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// adsl_decode.c: ADS-L (SRD-860) protocol decoder
//
// Implements XXTEA zero-key descrambling, Mode-S CRC-24 check, and
// FANET cordic position decoding for ADS-L packets.
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "adsl_decode.h"

// ======================== Syncword ========================

// IEEE Manchester(F5724B18) = 55 99 95 A6 9A 65 A9 6A
const uint8_t ADSL_SYNCWORD[ADSL_SYNCWORD_SIZE] = {
    0x55, 0x99, 0x95, 0xA6, 0x9A, 0x65, 0xA9, 0x6A
};

// ======================== CRC-24 (Mode-S polynomial) ========================

// Polynomial: 0x1FFF409 (x^24 + x^23 + ... + x^10 + x^3 + 1)
// Implemented as MSB-first with poly in top 25 bits of 32-bit word.
static uint32_t adsl_crc_pass(uint32_t crc, uint8_t byte)
{
    const uint32_t poly = 0xFFFA0480;
    crc |= byte;
    for (int bit = 0; bit < 8; bit++) {
        if (crc & 0x80000000)
            crc ^= poly;
        crc <<= 1;
    }
    return crc;
}

bool adsl_check_crc(const uint8_t *data24)
{
    uint32_t crc = 0;
    for (int i = 0; i < ADSL_PACKET_TOTAL; i++) {
        crc = adsl_crc_pass(crc, data24[i]);
    }
    return (crc >> 8) == 0;
}

// ======================== XXTEA zero-key descramble ========================

// XXTEA decrypt with all-zero key, n=5 words, 6 rounds.
// Standard btea decrypt: y carries from previous iteration (initial = v[0]).
static void xxtea_decrypt_key0(uint32_t *data, int n, int rounds)
{
    const uint32_t delta = 0x9E3779B9;
    uint32_t sum = (uint32_t)rounds * delta;
    uint32_t y = data[0];

    for (int cycle = 0; cycle < rounds; cycle++) {
        uint32_t e = (sum >> 2) & 3;
        uint32_t z;

        for (int p = n - 1; p > 0; p--) {
            z = data[p - 1];
            uint32_t mx = ((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4))
                        ^ ((sum ^ y) + z);
            data[p] -= mx;
            y = data[p];
        }
        z = data[n - 1];
        uint32_t mx = ((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4))
                    ^ ((sum ^ y) + z);
        data[0] -= mx;
        y = data[0];

        sum -= delta;
        (void)e;  // key index (unused with zero key)
    }
}

// ======================== Byte helpers ========================

static inline uint32_t get3bytes(const uint8_t *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16);
}

static inline uint32_t get4bytes(const uint8_t *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8)
         | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

// ======================== Variable-resolution decode ========================

// UnsVRdecode<uint16_t, 6>: threshold=64, 6-bit mantissa
static uint16_t ur2v6_adsl(uint8_t code)
{
    if (code < 64) return code;
    int exp = (code >> 6) - 1;
    uint16_t mant = code & 0x3F;
    return (uint16_t)((mant + 64) << exp);
}

// UnsVRdecode<int32_t, 12>: threshold=4096, 12-bit mantissa
// Used for ADS-L altitude (14-bit field)
static int32_t ur2v12_adsl(uint16_t code)
{
    if (code < 4096) return (int32_t)code;
    int exp = (code >> 12) - 1;
    int32_t mant = code & 0x0FFF;
    return (mant + 4096) << exp;
}

// ======================== Position decode ========================

// FANET cordic to float degrees
static inline double fnt_to_float(int32_t coord)
{
    const double conv = 90.0007295677 / (double)0x40000000;
    return conv * coord;
}

// ======================== Packet decode ========================

bool adsl_decode_packet(const uint8_t *data24,
                        double ref_lat, double ref_lon,
                        float ref_alt_geoid,
                        adsl_message_t *msg)
{
    memset(msg, 0, sizeof(*msg));

    // Byte 0: Version
    // Bytes 1-20: 5 x 32-bit words (to be descrambled)
    // Bytes 21-23: CRC (already verified by caller)

    // Extract 5 words from bytes 1-20 (little-endian)
    uint32_t words[5];
    for (int i = 0; i < 5; i++) {
        words[i] = get4bytes(&data24[1 + i * 4]);
    }

    // Descramble (XXTEA decrypt with zero key, 6 rounds)
    xxtea_decrypt_key0(words, 5, 6);

    // Re-serialize descrambled words to byte array for field extraction
    uint8_t pkt[20];
    for (int i = 0; i < 5; i++) {
        pkt[i * 4 + 0] = (uint8_t)(words[i]);
        pkt[i * 4 + 1] = (uint8_t)(words[i] >> 8);
        pkt[i * 4 + 2] = (uint8_t)(words[i] >> 16);
        pkt[i * 4 + 3] = (uint8_t)(words[i] >> 24);
    }

    // Packet layout (20 bytes after descramble):
    //   Byte 0: Type (0x02 = iConspicuity position)
    //   Bytes 1-4: Address[4] — Address[30]/Reserved[1]/RelayForward[1]
    //   Bytes 5-6: Meta — TimeStamp[6]/FlightState[2]/AcftCat[5]/Emergency[3]
    //   Bytes 7-17: Position[11] — Lat[24]/Lon[24]/Speed[8]/Alt[14]/Climb[9]/Track[9]
    //   Bytes 18-19: Integrity[2]

    uint8_t type = pkt[0];
    if (type != 0x02) {
        fprintf(stderr, "ADSL-DBG decode: type=0x%02X (expected 0x02)\n", type);
        return false;
    }

    // Address: bits [29:6] of the 4-byte Address field
    uint32_t addr_word = get4bytes(&pkt[1]);
    uint32_t addr = (addr_word >> 6) & 0x00FFFFFF;
    uint8_t addr_table = pkt[1] & 0x3F;

    // Address sanity
    if (addr == 0x000000 || addr == 0xFFFFFF) {
        fprintf(stderr, "ADSL-DBG decode: invalid addr=0x%06X\n", addr);
        return false;
    }
    // Reject repeating-byte addresses
    if ((addr & 0xFF) == ((addr >> 8) & 0xFF) &&
        (addr & 0xFF) == ((addr >> 16) & 0xFF))
        return false;

    // Map address table to OGN address type
    uint8_t addr_type;
    switch (addr_table) {
        case 0x05: addr_type = 1; break;  // ICAO
        case 0x06: addr_type = 2; break;  // FLARM
        case 0x07: addr_type = 3; break;  // OGN
        case 0x08: addr_type = 2; break;  // FANET → FLARM
        default:   addr_type = 0; break;  // random/unknown
    }

    // Aircraft category → OGN aircraft type
    uint8_t acft_cat = (pkt[6] >> 2) & 0x1F;
    static const uint8_t cat_to_ogn[32] = {
         0,  8,  9,  3,  1, 11,  2,  7,
         4, 13,  3, 13, 13, 13,  0,  0,
         0,  0,  0,  0,  0,  0,  0,  0,
         0,  0,  0,  0,  0,  0,  0,  0
    };
    uint8_t aircraft_type = cat_to_ogn[acft_cat & 0x1F];

    // Position fields from Position[11] starting at pkt[7]
    const uint8_t *pos = &pkt[7];

    // Latitude: 3 bytes (FANET cordic)
    int32_t lat_raw = (int32_t)get3bytes(pos);
    lat_raw <<= 8;
    lat_raw >>= 1;  // sign-extend and shift: 24-bit → FANET cordic
    double latitude = fnt_to_float(lat_raw);

    // Longitude: 3 bytes (FANET cordic)
    int32_t lon_raw = (int32_t)get3bytes(pos + 3);
    lon_raw <<= 8;  // 24-bit → FANET cordic (full range)
    double longitude = fnt_to_float(lon_raw);

    // Speed: 1 byte, UnsVR6, units = 0.25 m/s
    uint16_t speed_raw = ur2v6_adsl(pos[6]);
    float speed = speed_raw * 0.25f;

    // Altitude: 14 bits (pos[7] + pos[8] bits 0-5), UnsVR12, offset -320
    uint16_t alt_code = (uint16_t)pos[7] | (((uint16_t)(pos[8] & 0x3F)) << 8);
    int altitude = (int)ur2v12_adsl(alt_code) - 320;

    // Apply geoid correction
    altitude -= (int)roundf(ref_alt_geoid);

    // Climb rate: 9 bits (pos[8] bits 6-7 + pos[9] bits 0-6)
    // Sign-magnitude with UnsVR6 decode, units = 0.125 m/s
    uint16_t climb_raw = ((uint16_t)(pos[8] >> 6)) | (((uint16_t)(pos[9] & 0x7F)) << 2);
    float vs = 0.0f;
    if (climb_raw != 0x100) {  // 0x100 = "not available"
        if (climb_raw < 256) {
            vs = ur2v6_adsl((uint8_t)climb_raw) * 0.125f;
        } else {
            uint16_t magnitude = 512 - climb_raw;
            vs = -(float)ur2v6_adsl((uint8_t)magnitude) * 0.125f;
        }
    }

    // Track: 9 bits (pos[9] bit 7 + pos[10])
    // Units: cordic, degrees = track * 45.0/64
    uint16_t track_raw = ((uint16_t)(pos[9] >> 7)) | ((uint16_t)pos[10] << 1);
    float course = track_raw * (45.0f / 64.0f);
    if (course >= 360.0f) course -= 360.0f;

    // ---- Sanity checks ----
    if (latitude < -90.0 || latitude > 90.0) {
        fprintf(stderr, "ADSL-DBG decode: bad lat=%.4f addr=0x%06X\n", latitude, addr);
        return false;
    }
    if (longitude < -180.0 || longitude > 180.0) {
        fprintf(stderr, "ADSL-DBG decode: bad lon=%.4f addr=0x%06X\n", longitude, addr);
        return false;
    }
    if (altitude < -500 || altitude > 20000) {
        fprintf(stderr, "ADSL-DBG decode: bad alt=%d addr=0x%06X\n", altitude, addr);
        return false;
    }
    if (speed < 0.0f || speed > 200.0f) {
        fprintf(stderr, "ADSL-DBG decode: bad speed=%.1f addr=0x%06X\n", speed, addr);
        return false;
    }

    // Distance check: max ~3° from receiver
    {
        double dlat = latitude - ref_lat;
        double dlon = longitude - ref_lon;
        if (sqrt(dlat * dlat + dlon * dlon) > 3.0) {
            fprintf(stderr, "ADSL-DBG decode: too far addr=0x%06X lat=%.4f lon=%.4f (ref=%.4f,%.4f)\n",
                    addr, latitude, longitude, ref_lat, ref_lon);
            return false;
        }
    }

    // ---- Fill output ----
    msg->addr = addr;
    msg->addr_type = addr_type;
    msg->aircraft_type = aircraft_type;
    msg->latitude = latitude;
    msg->longitude = longitude;
    msg->altitude = altitude;
    msg->speed = speed;
    msg->course = course;
    msg->vs = vs;
    msg->signal_level = 0.0f;
    msg->valid = true;

    return true;
}
