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

#include <cstdio>
#include <cstring>
#include <cmath>
#include <stdint.h>


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

static int32_t popcount_byte(uint8_t v)
{
    // Brian Kernighan's bit count
    int32_t count = 0;
    while (v) { count += v & 1; v >>= 1; }
    return count;
}

static int32_t popcount32(uint32_t v)
{
    int32_t count = 0;
    while (v) {
        v &= v - 1;
        count++;
    }
    return count;
}

uint8_t ogntp_ldpc_check(const uint8_t *data)
{
    uint8_t errors = 0;
    for (int32_t row = 0; row < 48; row++) {
        const uint8_t *check = (const uint8_t *)LDPC_ParityCheck[row];
        uint8_t count = 0;
        for (int32_t i = 0; i < 26; i++) {
            count += (uint8_t)popcount_byte(data[i] & check[i]);
        }
        if (count & 1) errors++;
    }
    return errors;
}

// ======================== Variable-resolution decode helpers ========================

static bool ogntp_header_parity_ok(uint32_t hdr)
{
    return (popcount32(hdr & 0x0FFFFFFFu) & 1) == 0;
}

static uint32_t gray_decode_u32(uint32_t value)
{
    for (uint32_t shift = value >> 1; shift != 0; shift >>= 1)
        value ^= shift;
    return value;
}

static int32_t sign_extend_u32(uint32_t value, uint32_t bits)
{
    uint32_t sign = 1u << (bits - 1);
    if (value & sign)
        value |= ~((1u << bits) - 1u);
    return (int32_t)value;
}

static uint8_t unsvr_decode_4(uint8_t code)
{
    if (code <= 4) return code;
    int32_t exp = (code >> 2) - 1;
    uint8_t mant = code & 0x03;
    return (uint8_t)((mant + 4) << exp);
}

// Unsigned variable-resolution decode, N-bit (HalfBits = N/2)
// UnsVRdecode<uint16_t, 12>: HalfBits=6, ThresVal=64
static uint16_t unsvr_decode_12(uint16_t code)
{
    if (code <= 64) return code;
    int32_t exp = (code >> 6) - 1;
    uint16_t mant = code & 0x3F;
    return (uint16_t)((mant + 64) << exp);
}

// UnsVRdecode<uint16_t, 8>: HalfBits=4, ThresVal=16
static uint16_t unsvr_decode_8(uint16_t code)
{
    if (code <= 16) return code;
    int32_t exp = (code >> 4) - 1;
    uint16_t mant = code & 0x0F;
    return (uint16_t)((mant + 16) << exp);
}

// DecodeUR2V6: uint32_t ratio-2, 6-bit mantissa
static uint16_t ur2v6_decode(uint8_t code)
{
    if (code <= 63) return code;
    int32_t exp = (code >> 6) - 1;
    uint16_t mant = code & 0x3F;
    return (uint16_t)((mant + 64) << exp);
}

// DecodeUR2V5: uint32_t ratio-2, 5-bit mantissa
static uint16_t ur2v5_decode(uint8_t code)
{
    if (code <= 15) return code;
    int32_t exp = (code >> 5) - 1;
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

static double ogntp_deg_from_units(int32_t units)
{
    return units * (0.0001 / 60.0);
}

static int32_t ogntp_rx_rate_per_min(uint8_t code)
{
    if (code >= 31)
        return 0;
    return (1 << code) - 1;
}

static void ogntp_init_message(ogntp_message_t *msg)
{
    memset(msg, 0, sizeof(*msg));
    msg->report_type = -1;
    msg->status.temperature_c = NAN;
    msg->status.humidity_percent = NAN;
}

static bool ogntp_position_sane(double lat_deg, double lon_deg,
                                double ref_lat, double ref_lon,
                                int32_t alt_m, float speed_ms, int32_t fix_quality)
{
    if (fix_quality == 0)
        return false;
    if (lat_deg < -90.0 || lat_deg > 90.0)
        return false;
    if (lon_deg < -180.0 || lon_deg > 180.0)
        return false;
    if (alt_m < -1500 || alt_m > 20000)
        return false;
    if (speed_ms < 0.0f || speed_ms > 400.0f)
        return false;

    {
        double dlat = lat_deg - ref_lat;
        double dlon = lon_deg - ref_lon;
        double dist2 = dlat * dlat + dlon * dlon;
        if (dist2 > 9.0)
            return false;
    }

    return true;
}

static bool ogntp_decode_ogn1_status(const uint8_t *data, ogntp_message_t *msg)
{
    uint32_t d0 = (uint32_t)data[4]
                | ((uint32_t)data[5] << 8)
                | ((uint32_t)data[6] << 16)
                | ((uint32_t)data[7] << 24);
    uint32_t d1 = (uint32_t)data[8]
                | ((uint32_t)data[9] << 8)
                | ((uint32_t)data[10] << 16)
                | ((uint32_t)data[11] << 24);
    uint32_t d2 = (uint32_t)data[12]
                | ((uint32_t)data[13] << 8)
                | ((uint32_t)data[14] << 16)
                | ((uint32_t)data[15] << 24);
    uint32_t d3 = (uint32_t)data[16]
                | ((uint32_t)data[17] << 8)
                | ((uint32_t)data[18] << 16)
                | ((uint32_t)data[19] << 24);

    uint8_t report_type = (uint8_t)((d3 >> 20) & 0x0F);
    if (report_type != 0)
        return false;

    msg->report_type = 0;
    msg->status_valid = 1;
    msg->valid = 1;
    msg->altitude = (int32_t)unsvr_decode_12((uint16_t)(d2 & 0x3FFF)) - 1024;
    msg->fix_quality = (int32_t)((d0 >> 30) & 0x03);

    msg->status.time_seconds = (int32_t)((d0 >> 24) & 0x3F);
    msg->status.fix_quality = msg->fix_quality;
    msg->status.pulse_bpm = (int32_t)(d0 & 0xFF);
    msg->status.oxygen_percent = (int32_t)((d0 >> 8) & 0x7F);
    msg->status.sat_snr_db = (int32_t)(((d0 >> 15) & 0x1F) + 8);
    msg->status.rx_rate_per_min = ogntp_rx_rate_per_min((uint8_t)((d0 >> 20) & 0x0F));

    msg->status.audio_noise_db = (int32_t)(d1 & 0xFF);
    msg->status.radio_noise_dbm = -0.5f * (float)((d1 >> 8) & 0xFF);

    {
        int8_t temp_code = (int8_t)((d1 >> 16) & 0xFF);
        if ((uint8_t)temp_code != 0x80) {
            msg->status.has_temperature = 1;
            msg->status.temperature_c = (200 + sr2v5_decode(temp_code)) * 0.1f;
        }
    }

    {
        int8_t hum_code = (int8_t)((d1 >> 24) & 0xFF);
        if ((uint8_t)hum_code != 0x80) {
            msg->status.has_humidity = 1;
            msg->status.humidity_percent = (520 + sr2v5_decode(hum_code)) * 0.1f;
        }
    }

    msg->status.has_pressure = ((d2 >> 14) & 0x3FFF) != 0;
    msg->status.pressure_hpa = ((float)((d2 >> 14) & 0x3FFF)) * 0.08f;
    msg->status.satellites = (int32_t)((d2 >> 28) & 0x0F);

    msg->status.firmware = (int32_t)(d3 & 0xFF);
    msg->status.hardware = (int32_t)((d3 >> 8) & 0xFF);
    msg->status.tx_power_dbm = (int32_t)(((d3 >> 16) & 0x0F) + 4);
    msg->status.voltage_v = ur2v6_decode((uint8_t)((d3 >> 24) & 0xFF)) / 64.0f;

    return true;
}

static bool ogntp_decode_ogn2_status(const uint8_t *data, ogntp_message_t *msg)
{
    uint32_t d0 = (uint32_t)data[4]
                | ((uint32_t)data[5] << 8)
                | ((uint32_t)data[6] << 16)
                | ((uint32_t)data[7] << 24);
    uint32_t d1 = (uint32_t)data[8]
                | ((uint32_t)data[9] << 8)
                | ((uint32_t)data[10] << 16)
                | ((uint32_t)data[11] << 24);
    uint32_t d2 = (uint32_t)data[12]
                | ((uint32_t)data[13] << 8)
                | ((uint32_t)data[14] << 16)
                | ((uint32_t)data[15] << 24);
    uint32_t d3 = (uint32_t)data[16]
                | ((uint32_t)data[17] << 8)
                | ((uint32_t)data[18] << 16)
                | ((uint32_t)data[19] << 24);

    uint8_t report_type = (uint8_t)(d0 & 0x0F);
    if (report_type != 0)
        return false;

    msg->report_type = 0;
    msg->status_valid = 1;
    msg->valid = 1;
    msg->altitude = (int32_t)unsvr_decode_12((uint16_t)gray_decode_u32(d1 & 0x3FFF)) - 1024;
    msg->fix_quality = (int32_t)((d2 >> 30) & 0x03);

    msg->status.time_seconds = (int32_t)((d2 >> 24) & 0x3F);
    msg->status.fix_quality = msg->fix_quality;
    msg->status.pulse_bpm = (int32_t)(d2 & 0xFF);
    msg->status.oxygen_percent = (int32_t)((d2 >> 8) & 0x7F);
    msg->status.sat_snr_db = (int32_t)(((d2 >> 15) & 0x1F) + 8);
    msg->status.rx_rate_per_min = ogntp_rx_rate_per_min((uint8_t)((d2 >> 20) & 0x0F));

    msg->status.audio_noise_db = (int32_t)(d3 & 0xFF);
    msg->status.radio_noise_dbm = -0.5f * (float)((d3 >> 8) & 0xFF);

    {
        uint8_t temp_gray = (uint8_t)((d3 >> 16) & 0xFF);
        uint8_t temp_code = (uint8_t)gray_decode_u32(temp_gray);
        if (temp_code != 0x80) {
            msg->status.has_temperature = 1;
            msg->status.temperature_c = (200 + sr2v5_decode((int8_t)temp_code)) * 0.1f;
        }
    }

    {
        uint8_t hum_gray = (uint8_t)((d3 >> 24) & 0xFF);
        uint8_t hum_code = (uint8_t)gray_decode_u32(hum_gray);
        if (hum_code != 0x80) {
            msg->status.has_humidity = 1;
            msg->status.humidity_percent = (525 + sr2v5_decode((int8_t)hum_code)) * 0.1f;
        }
    }

    msg->status.has_pressure = ((d1 >> 14) & 0x3FFF) != 0;
    msg->status.pressure_hpa = ((float)((d1 >> 14) & 0x3FFF)) * 0.08f;
    msg->status.satellites = (int32_t)((d1 >> 28) & 0x0F);

    msg->status.tx_power_dbm = (int32_t)(((d0 >> 4) & 0x0F) + 4);
    msg->status.firmware = (int32_t)((d0 >> 8) & 0xFF);
    msg->status.hardware = (int32_t)((d0 >> 16) & 0xFF);
    msg->status.voltage_v = (80 + ur2v6_decode((uint8_t)gray_decode_u32((uint8_t)((d0 >> 24) & 0xFF)))) / 64.0f;

    return true;
}

static bool ogntp_decode_ogn1_packet(const uint8_t *data, double ref_lat, double ref_lon,
                                     ogntp_message_t *msg)
{
    ogntp_init_message(msg);

    uint32_t hdr = (uint32_t)data[0]
                 | ((uint32_t)data[1] << 8)
                 | ((uint32_t)data[2] << 16)
                 | ((uint32_t)data[3] << 24);

    msg->addr = hdr & 0x00FFFFFF;
    msg->addr_type = (int32_t)((hdr >> 24) & 0x03);
    msg->version = 1;
    msg->nonpos = (int32_t)((hdr >> 26) & 0x01);
    msg->relay = (int32_t)((hdr >> 28) & 0x03);
    msg->encrypted = (int32_t)((hdr >> 30) & 0x01);
    msg->emergency = (int32_t)((hdr >> 31) & 0x01);
    msg->other_system = 0;

    if (!ogntp_header_parity_ok(hdr))
        return false;
    if (msg->addr == 0x000000 || msg->addr == 0xFFFFFF)
        return false;
    if (msg->encrypted)
        return false;

    if (msg->nonpos)
        return ogntp_decode_ogn1_status(data, msg);

    uint32_t d0 = (uint32_t)data[4]
                | ((uint32_t)data[5] << 8)
                | ((uint32_t)data[6] << 16)
                | ((uint32_t)data[7] << 24);
    uint32_t d1 = (uint32_t)data[8]
                | ((uint32_t)data[9] << 8)
                | ((uint32_t)data[10] << 16)
                | ((uint32_t)data[11] << 24);
    uint32_t d2 = (uint32_t)data[12]
                | ((uint32_t)data[13] << 8)
                | ((uint32_t)data[14] << 16)
                | ((uint32_t)data[15] << 24);
    uint32_t d3 = (uint32_t)data[16]
                | ((uint32_t)data[17] << 8)
                | ((uint32_t)data[18] << 16)
                | ((uint32_t)data[19] << 24);

    int32_t lat24 = sign_extend_u32(d0 & 0x00FFFFFF, 24);
    int32_t lon24 = sign_extend_u32(d1 & 0x00FFFFFF, 24);
    int32_t lat_units = (lat24 << 3) + 4;
    int32_t lon_units = (lon24 << 4) + 8;
    double lat_deg = ogntp_deg_from_units(lat_units);
    double lon_deg = ogntp_deg_from_units(lon_units);
    int32_t fix_quality = (int32_t)((d0 >> 30) & 0x03);
    int32_t alt_m = (int32_t)unsvr_decode_12((uint16_t)(d2 & 0x3FFF)) - 1024;
    float speed_ms = unsvr_decode_8((uint16_t)((d2 >> 14) & 0x03FF)) * 0.1f;
    float turn_dps = sr2v5_decode((int8_t)((d2 >> 24) & 0xFF)) * 0.1f;
    float heading_deg = ((float)(((d3 & 0x03FFu) * 3600u + 512u) >> 10)) * 0.1f;
    float vs_ms = sr2v6_decode((int8_t)((d3 >> 10) & 0xFF)) * 0.1f;
    int32_t stealth = (int32_t)((d3 >> 19) & 0x01);

    if (stealth)
        return false;
    if (!ogntp_position_sane(lat_deg, lon_deg, ref_lat, ref_lon, alt_m, speed_ms, fix_quality))
        return false;

    msg->position_valid = 1;
    msg->valid = 1;
    msg->aircraft_type = (int32_t)((d3 >> 20) & 0x0F);
    msg->stealth = 0;
    msg->latitude = lat_deg;
    msg->longitude = lon_deg;
    msg->altitude = alt_m;
    msg->speed = speed_ms;
    msg->course = heading_deg;
    msg->vs = vs_ms;
    msg->turnrate = turn_dps;
    msg->fix_quality = fix_quality;

    return true;
}

static bool ogntp_decode_ogn2_packet(const uint8_t *data, double ref_lat, double ref_lon,
                                     ogntp_message_t *msg)
{
    ogntp_init_message(msg);

    uint32_t hdr = (uint32_t)data[0]
                 | ((uint32_t)data[1] << 8)
                 | ((uint32_t)data[2] << 16)
                 | ((uint32_t)data[3] << 24);

    msg->addr = hdr & 0x00FFFFFF;
    msg->addr_type = (int32_t)((hdr >> 24) & 0x03);
    msg->version = 2;
    msg->relay = (int32_t)((hdr >> 26) & 0x01);
    msg->nonpos = (int32_t)((hdr >> 28) & 0x01);
    msg->other_system = (int32_t)((hdr >> 29) & 0x01);
    msg->encrypted = (int32_t)((hdr >> 30) & 0x01);
    msg->emergency = (int32_t)((hdr >> 31) & 0x01);

    if (!ogntp_header_parity_ok(hdr))
        return false;
    if (msg->addr == 0x000000 || msg->addr == 0xFFFFFF)
        return false;
    if (msg->encrypted || msg->other_system)
        return false;

    if (msg->nonpos)
        return ogntp_decode_ogn2_status(data, msg);

    uint32_t d0 = (uint32_t)data[4]
                | ((uint32_t)data[5] << 8)
                | ((uint32_t)data[6] << 16)
                | ((uint32_t)data[7] << 24);
    uint32_t d1 = (uint32_t)data[8]
                | ((uint32_t)data[9] << 8)
                | ((uint32_t)data[10] << 16)
                | ((uint32_t)data[11] << 24);
    uint32_t d2 = (uint32_t)data[12]
                | ((uint32_t)data[13] << 8)
                | ((uint32_t)data[14] << 16)
                | ((uint32_t)data[15] << 24);
    uint32_t d3 = (uint32_t)data[16]
                | ((uint32_t)data[17] << 8)
                | ((uint32_t)data[18] << 16)
                | ((uint32_t)data[19] << 24);

    int32_t lat_units = (int32_t)(((int64_t)sign_extend_u32(gray_decode_u32(d2 & 0x00FFFFFF), 24) * 108000000 + (1 << 23)) >> 24);
    int32_t lon_units = (int32_t)(((int64_t)sign_extend_u32(gray_decode_u32(d3 & 0x01FFFFFF), 25) * 108000000 + (1 << 23)) >> 24);
    double lat_deg = ogntp_deg_from_units(lat_units);
    double lon_deg = ogntp_deg_from_units(lon_units);
    int32_t fix_quality = (int32_t)((d2 >> 30) & 0x03);
    int32_t alt_m = (int32_t)unsvr_decode_12((uint16_t)gray_decode_u32(d1 & 0x3FFF)) - 1024;
    float speed_ms = unsvr_decode_8((uint16_t)gray_decode_u32((d1 >> 14) & 0x03FF)) * 0.1f;
    uint16_t heading_code = (uint16_t)gray_decode_u32((d0 >> 4) & 0x03FF);
    float heading_deg = ((float)((heading_code * 3600u + 512u) >> 10)) * 0.1f;
    uint16_t climb_code = (uint16_t)((d0 >> 14) & 0x01FF);
    uint8_t turn_code = (uint8_t)((d1 >> 24) & 0xFF);
    float vs_ms = (climb_code == 0x0100) ? 0.0f : (sr2v6_decode((int8_t)(climb_code & 0xFF)) * 0.1f);
    float turn_dps = (turn_code == 0x80) ? 0.0f : (sr2v5_decode((int8_t)turn_code) * 0.1f);
    uint8_t dop = unsvr_decode_4((uint8_t)gray_decode_u32((d3 >> 25) & 0x3F));

    if (dop > 80)
        return false;
    if (!ogntp_position_sane(lat_deg, lon_deg, ref_lat, ref_lon, alt_m, speed_ms, fix_quality))
        return false;

    msg->position_valid = 1;
    msg->valid = 1;
    msg->aircraft_type = (int32_t)(d0 & 0x0F);
    msg->latitude = lat_deg;
    msg->longitude = lon_deg;
    msg->altitude = alt_m;
    msg->speed = speed_ms;
    msg->course = heading_deg;
    msg->vs = vs_ms;
    msg->turnrate = turn_dps;
    msg->fix_quality = fix_quality;

    return true;
}

static int32_t ogntp_candidate_score(const ogntp_message_t *msg, double ref_lat, double ref_lon)
{
    int32_t score = 0;

    if (msg->position_valid) {
        score += 20;
        score += msg->fix_quality > 0 ? 4 : 0;
        if (msg->speed <= 120.0f)
            score += 2;
        if (fabsf(msg->vs) <= 20.0f)
            score += 2;
        if (msg->course >= 0.0f && msg->course < 360.0f)
            score += 1;
        {
            double d = fabs(msg->latitude - ref_lat) + fabs(msg->longitude - ref_lon);
            if (d < 0.5)
                score += 3;
            else if (d < 1.0)
                score += 2;
            else if (d < 2.0)
                score += 1;
        }
    }

    if (msg->status_valid) {
        score += 18;
        if (msg->status.satellites > 0 && msg->status.satellites <= 16)
            score += 2;
        if (msg->status.voltage_v > 0.5f && msg->status.voltage_v < 20.0f)
            score += 2;
        if (msg->status.has_temperature && msg->status.temperature_c > -60.0f && msg->status.temperature_c < 100.0f)
            score += 1;
        if (msg->status.has_humidity && msg->status.humidity_percent >= 0.0f && msg->status.humidity_percent <= 100.0f)
            score += 1;
    }

    return score;
}

// ======================== Packet decode ========================

bool ogntp_decode_packet(const uint8_t *data, double ref_lat, double ref_lon,
                         ogntp_message_t *msg)
{
    ogntp_message_t v1;
    ogntp_message_t v2;
    bool ok1 = ogntp_decode_ogn1_packet(data, ref_lat, ref_lon, &v1);
    bool ok2 = ogntp_decode_ogn2_packet(data, ref_lat, ref_lon, &v2);

    if (ok1 && !ok2) {
        *msg = v1;
        return true;
    }
    if (ok2 && !ok1) {
        *msg = v2;
        return true;
    }
    if (!ok1 && !ok2) {
        ogntp_init_message(msg);
        return false;
    }

    if (ogntp_candidate_score(&v2, ref_lat, ref_lon) > ogntp_candidate_score(&v1, ref_lat, ref_lon))
        *msg = v2;
    else
        *msg = v1;

    return true;
}
