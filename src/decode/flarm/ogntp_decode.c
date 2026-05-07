// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// ogntp_decode.c: OGN Tracker Protocol (OGN1) LDPC check and packet decode
//
// LDPC table and decode formulas derived from the esp32-ogn-tracker and
// SoftRF open-source projects by Pawel Jalocha and Linar Yusupov.
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

#include "ogntp_decode.h"

// ======================== Syncword ========================

const uint8_t OGNTP_SYNCWORD[OGNTP_SYNCWORD_SIZE] = {
    0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A
};

// ======================== LDPC parity check table ========================
// LDPC (208, 160) code: 160 data bits + 48 parity bits = 208 code bits.
// 48 rows × 7 uint32_t = 48 × 28 bytes; only the first 26 bytes per row
// are used (corresponding to the 26-byte OGN1 packet).
// Source: pjalocha/esp32-ogn-tracker main/ldpc.cpp

static const uint32_t LDPC_ParityCheck[48][7] = {
    { 0x00000805, 0x00000020, 0x04000000, 0x20000000, 0x00000040, 0x00044020, 0x00000000 },
    { 0x00000001, 0x00800800, 0x00000000, 0x00000000, 0x00000000, 0x10010000, 0x00008C98 },
    { 0x00004001, 0x01000080, 0x80000400, 0x00000000, 0x08000200, 0x00200000, 0x00000005 },
    { 0x00000101, 0x20000200, 0x00000022, 0x00000000, 0x00000000, 0xCC008000, 0x00005002 },
    { 0x00000401, 0x00000000, 0x00004900, 0x00000020, 0x00000000, 0x20C00349, 0x00000020 },
    { 0x03140001, 0x00000002, 0x00000000, 0x40000001, 0x41534100, 0x00102C00, 0x00002000 },
    { 0x04008800, 0x82000642, 0x00000000, 0x00000020, 0x88040020, 0x03000010, 0x00000400 },
    { 0x00000802, 0x20000000, 0x02000014, 0x01200000, 0x04000403, 0x00800004, 0x0000A004 },
    { 0x02020820, 0x00000000, 0x80020820, 0x10190040, 0x30000000, 0x00000002, 0x00000900 },
    { 0x40804950, 0x00090000, 0x00000000, 0x00021204, 0x40001000, 0x10001100, 0x00000000 },
    { 0x08000A00, 0x00020008, 0x00040000, 0x02400010, 0x01002000, 0x40280280, 0x00000010 },
    { 0x00000000, 0x00008010, 0x118000A0, 0x00040080, 0x01000084, 0x00040100, 0x00000444 },
    { 0x20040108, 0x18000000, 0x08608800, 0x0000000A, 0x08000010, 0x00040080, 0x00008000 },
    { 0x00004080, 0x00422201, 0x00010000, 0x0000A400, 0x00400800, 0x00840000, 0x00000800 },
    { 0x00000000, 0x60200000, 0x80100240, 0x08000021, 0x02800000, 0x100C0000, 0x00000000 },
    { 0x00001000, 0x01010002, 0x00082001, 0x04000000, 0x00000001, 0x00040002, 0x00004030 },
    { 0x00002300, 0x04000000, 0xA0080000, 0x20004000, 0x00028000, 0x00800000, 0x00000400 },
    { 0x00004000, 0x00104100, 0x40041028, 0x24000020, 0x00200000, 0x00100000, 0x00008000 },
    { 0x08011000, 0x20040000, 0x00000000, 0xA0800000, 0x08090000, 0x00000100, 0x00000A00 },
    { 0x10180000, 0x00000204, 0x00002800, 0x20400800, 0x00000000, 0x10000000, 0x00000004 },
    { 0x00000000, 0xC0000000, 0x10200000, 0x20028000, 0x20000000, 0x80000008, 0x00002011 },
    { 0x82004000, 0x20000000, 0x04202000, 0x00000000, 0x00000000, 0x00020200, 0x00000400 },
    { 0x08600000, 0x00001200, 0x94000000, 0x00000000, 0x40000008, 0x00000000, 0x00008020 },
    { 0x04040000, 0x04010000, 0x04100000, 0x00000100, 0x00200000, 0x40000008, 0x00000804 },
    { 0x00000200, 0x00000110, 0x04000100, 0x00000000, 0x28400400, 0x10000000, 0x00004000 },
    { 0x00080000, 0x00000080, 0x04001000, 0x01882007, 0x00008024, 0x04000001, 0x00000010 },
    { 0x20200000, 0x00000020, 0x00010040, 0x81000800, 0x10001000, 0x00300008, 0x00004400 },
    { 0x90000010, 0x89841021, 0x00000118, 0x08080000, 0x00020000, 0x40000000, 0x00000040 },
    { 0x04C20000, 0x10404034, 0x00000000, 0x00004000, 0x00810001, 0x04000200, 0x00000009 },
    { 0x40102000, 0x020020A0, 0x40100000, 0x00100080, 0x00080400, 0x80030080, 0x00000020 },
    { 0x00010000, 0x04020920, 0x00000200, 0x00060000, 0x00000218, 0x01002007, 0x00001000 },
    { 0x00020008, 0x00A08040, 0x00080000, 0x40001400, 0x04200040, 0x80200001, 0x00000200 },
    { 0x40000402, 0x01100000, 0x20808000, 0x00008000, 0x10100060, 0x00080000, 0x00001008 },
    { 0x200010A0, 0x00000000, 0x01040100, 0x00000104, 0x02040042, 0x08012000, 0x00000001 },
    { 0x01000000, 0x50000880, 0x00000092, 0x14400000, 0x00001840, 0x02400000, 0x00000000 },
    { 0x00000010, 0x02000000, 0x00014000, 0x00200018, 0x00000240, 0x04000800, 0x00000180 },
    { 0x00008000, 0x00880008, 0x08000044, 0x00100000, 0x00000004, 0x00400820, 0x00001001 },
    { 0x01000000, 0x00002000, 0x02004001, 0x00000042, 0x00000000, 0x09201020, 0x00000048 },
    { 0x00800000, 0x01000400, 0x00400002, 0xC0002000, 0x00002080, 0x00010064, 0x00000100 },
    { 0x00000400, 0x08400840, 0x00000400, 0x00000890, 0x00008102, 0x00000020, 0x00000002 },
    { 0x00200040, 0x00000081, 0x00000000, 0x02050000, 0x04940000, 0x20008020, 0x00000080 },
    { 0x00000404, 0x00800000, 0x00001000, 0x00014000, 0x00082200, 0x0A000400, 0x00000000 },
    { 0x0000A024, 0x00000000, 0x00000402, 0x08A01000, 0x00004010, 0x20000000, 0x00000008 },
    { 0x00480046, 0x00008000, 0x00000208, 0x00000048, 0x00000000, 0x00410010, 0x00000002 },
    { 0x0000008C, 0x00044C00, 0x00824004, 0x00000200, 0x00000000, 0x00028000, 0x00000000 },
    { 0x10010004, 0x00080000, 0x43008000, 0x10000400, 0x80000100, 0x00000040, 0x00000080 },
    { 0x80000000, 0x0020000C, 0x20420480, 0x00000100, 0x00000008, 0x00005410, 0x00000080 },
    { 0x00000000, 0x00101000, 0x08000001, 0x02000200, 0x82004A80, 0x00004000, 0x00000202 },
};

// ======================== LDPC check ========================

static int popcount_byte(uint8_t v)
{
    // Brian Kernighan's bit count
    int count = 0;
    while (v) { count += v & 1; v >>= 1; }
    return count;
}

uint8_t ogntp_ldpc_check(const uint8_t *data)
{
    uint8_t errors = 0;
    for (int row = 0; row < 48; row++) {
        const uint8_t *check = (const uint8_t *)LDPC_ParityCheck[row];
        uint8_t count = 0;
        for (int i = 0; i < 26; i++) {
            count += (uint8_t)popcount_byte(data[i] & check[i]);
        }
        if (count & 1) errors++;
    }
    return errors;
}

// ======================== Variable-resolution decode helpers ========================

// Unsigned variable-resolution decode, N-bit (HalfBits = N/2)
// UnsVRdecode<uint16_t, 12>: HalfBits=6, ThresVal=64
static uint16_t unsvr_decode_12(uint16_t code)
{
    if (code <= 64) return code;
    int exp = (code >> 6) - 1;
    uint16_t mant = code & 0x3F;
    return (uint16_t)((mant + 64) << exp);
}

// UnsVRdecode<uint16_t, 8>: HalfBits=4, ThresVal=16
static uint16_t unsvr_decode_8(uint16_t code)
{
    if (code <= 16) return code;
    int exp = (code >> 4) - 1;
    uint16_t mant = code & 0x0F;
    return (uint16_t)((mant + 16) << exp);
}

// DecodeUR2V6: unsigned ratio-2, 6-bit mantissa
static uint16_t ur2v6_decode(uint8_t code)
{
    if (code <= 63) return code;
    int exp = (code >> 6) - 1;
    uint16_t mant = code & 0x3F;
    return (uint16_t)((mant + 64) << exp);
}

// DecodeUR2V5: unsigned ratio-2, 5-bit mantissa
static uint16_t ur2v5_decode(uint8_t code)
{
    if (code <= 15) return code;
    int exp = (code >> 5) - 1;
    uint16_t mant = code & 0x1F;
    return (uint16_t)((mant + 32) << exp);
}

// DecodeSR2V6: signed ratio-2, 6-bit mantissa (stored as int8_t)
static int16_t sr2v6_decode(int8_t code)
{
    if (code >= 0)
        return (int16_t)ur2v6_decode((uint8_t)code);
    else
        return -(int16_t)ur2v6_decode((uint8_t)(-code));
}

// DecodeSR2V5: signed ratio-2, 5-bit mantissa (stored as int8_t)
static int16_t sr2v5_decode(int8_t code)
{
    if (code >= 0)
        return (int16_t)ur2v5_decode((uint8_t)code);
    else
        return -(int16_t)ur2v5_decode((uint8_t)(-code));
}

// ======================== Packet decode ========================

bool ogntp_decode_packet(const uint8_t *data, double ref_lat, double ref_lon,
                         ogntp_message_t *msg)
{
    memset(msg, 0, sizeof(*msg));

    // ---- Header word (bytes 0-3, little-endian) ----
    uint32_t hdr = (uint32_t)data[0]
                 | ((uint32_t)data[1] << 8)
                 | ((uint32_t)data[2] << 16)
                 | ((uint32_t)data[3] << 24);

    uint32_t addr       = hdr & 0x00FFFFFF;
    uint8_t  addr_type  = (hdr >> 24) & 0x03;
    uint8_t  nonpos     = (hdr >> 26) & 0x01;
    uint8_t  encrypted  = (hdr >> 30) & 0x01;

    // Skip non-position packets and encrypted packets
    if (nonpos || encrypted)
        return false;

    // Skip all-zero or all-ones addresses (likely noise)
    if (addr == 0x000000 || addr == 0xFFFFFF)
        return false;

    // ---- Data word 0 (bytes 4-7): Latitude:24, Time:6, FixQuality:2 ----
    uint32_t d0 = (uint32_t)data[4]
                | ((uint32_t)data[5] << 8)
                | ((uint32_t)data[6] << 16)
                | ((uint32_t)data[7] << 24);

    // Latitude: 24-bit signed, units = 0.0001/60 deg before shift
    int32_t lat24 = (int32_t)(d0 & 0x00FFFFFF);
    if (lat24 & 0x00800000) lat24 |= (int32_t)0xFF000000; // sign extend
    // DecodeLatitude(): (lat24 << 3) + 4 -> 0.0001/60 deg units
    int32_t lat_units = (lat24 << 3) + 4;
    double  lat_deg   = lat_units * (0.0001 / 60.0);

    uint8_t fix_quality = (d0 >> 30) & 0x03;

    // ---- Data word 1 (bytes 8-11): Longitude:24, DOP:6, BaroMSB:1, FixMode:1 ----
    uint32_t d1 = (uint32_t)data[8]
                | ((uint32_t)data[9] << 8)
                | ((uint32_t)data[10] << 16)
                | ((uint32_t)data[11] << 24);

    int32_t lon24 = (int32_t)(d1 & 0x00FFFFFF);
    if (lon24 & 0x00800000) lon24 |= (int32_t)0xFF000000; // sign extend
    // DecodeLongitude(): (lon24 << 4) + 8 -> 0.0001/60 deg units
    int32_t lon_units = (lon24 << 4) + 8;
    double  lon_deg   = lon_units * (0.0001 / 60.0);

    // ---- Data word 2 (bytes 12-15): Altitude:14, Speed:10, TurnRate:8 ----
    uint32_t d2 = (uint32_t)data[12]
                | ((uint32_t)data[13] << 8)
                | ((uint32_t)data[14] << 16)
                | ((uint32_t)data[15] << 24);

    uint16_t alt_code   = (uint16_t)(d2 & 0x00003FFF);          // bits 0-13
    uint16_t speed_code = (uint16_t)((d2 >> 14) & 0x000003FF);  // bits 14-23
    int8_t   turn_code  = (int8_t)((d2 >> 24) & 0xFF);          // bits 24-31, signed

    // Altitude: UnsVRdecode_12(alt_code) -> meters
    int      alt_m    = (int)unsvr_decode_12(alt_code);
    // Speed: UnsVRdecode_8(speed_code) -> 0.1 m/s -> m/s
    float    speed_ms = unsvr_decode_8(speed_code) * 0.1f;
    // TurnRate: SR2V5(turn_code) -> 0.1 deg/s -> deg/s
    float    turn_dps = sr2v5_decode(turn_code) * 0.1f;

    // ---- Data word 3 (bytes 16-19): Heading:10, ClimbRate:9, Stealth:1, AcftType:4, BaroAltDiff:8 ----
    uint32_t d3 = (uint32_t)data[16]
                | ((uint32_t)data[17] << 8)
                | ((uint32_t)data[18] << 16)
                | ((uint32_t)data[19] << 24);

    uint16_t heading_code = (uint16_t)(d3 & 0x000003FF);         // bits 0-9
    uint16_t climb_code9  = (uint16_t)((d3 >> 10) & 0x000001FF); // bits 10-18, 9 bits
    uint8_t  stealth      = (uint8_t)((d3 >> 19) & 0x01);        // bit 19
    uint8_t  acft_type    = (uint8_t)((d3 >> 20) & 0x0F);        // bits 20-23

    // Heading: (heading_code * 3600 + 512) >> 10 -> tenths of degrees
    float heading_deg = ((heading_code * 3600u + 512u) >> 10) * 0.1f;

    // ClimbRate: 9-bit field, bits 7-0 are SR2V6 encoded magnitude, bit 8 is
    // the sign extension of bit 7 (redundant). Take bits 7-0 as int8_t.
    int8_t   climb_code8 = (int8_t)(climb_code9 & 0xFF);
    float    vs_ms       = sr2v6_decode(climb_code8) * 0.1f;     // m/s

    // ======================== Sanity checks ========================

    // Reject stealth aircraft
    if (stealth)
        return false;

    // Require at least a valid fix
    if (fix_quality == 0)
        return false;

    // Latitude must be within ±90 degrees
    if (lat_deg < -90.0 || lat_deg > 90.0)
        return false;

    // Longitude must be within ±180 degrees
    if (lon_deg < -180.0 || lon_deg > 180.0)
        return false;

    // Position must be within ~300 km of receiver
    {
        double dlat = lat_deg - ref_lat;
        double dlon = lon_deg - ref_lon;
        double dist2 = dlat * dlat + dlon * dlon;
        if (dist2 > 9.0) // 3 degrees ~ 330 km
            return false;
    }

    // Altitude: -500 m to 15000 m
    if (alt_m < -500 || alt_m > 15000)
        return false;

    // Speed: 0 to 400 m/s
    if (speed_ms < 0.0f || speed_ms > 400.0f)
        return false;

    // ======================== Fill output ========================

    msg->addr          = addr;
    msg->addr_type     = (int)addr_type;
    msg->aircraft_type = (int)acft_type;
    msg->stealth       = 0;
    msg->latitude      = lat_deg;
    msg->longitude     = lon_deg;
    msg->altitude      = alt_m;
    msg->speed         = speed_ms;
    msg->course        = heading_deg;
    msg->vs            = vs_ms;
    msg->turnrate      = turn_dps;
    msg->fix_quality   = (int)fix_quality;
    msg->signal_level  = 0.0f; // set by caller
    msg->valid         = 1;

    return true;
}
