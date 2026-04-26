// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// flarm_decode.c: FLARM Legacy protocol decoder
//
// Based on protocol reverse-engineering by Stanislaw Pusep and
// SoftRF project by Linar Yusupov (GPL-3.0).
//
// Encryption keys and protocol details derived from open-source
// implementations (SoftRF Legacy.cpp).
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "flarm_decode.h"

// ======================== XXTEA (Block TEA) ========================
// http://en.wikipedia.org/wiki/XXTEA

#define DELTA 0x9e3779b9
#define ROUNDS 6
#define MX (((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4)) ^ ((sum ^ y) + (key[(p & 3) ^ e] ^ z)))

static void btea(uint32_t *v, int8_t n, const uint32_t key[4])
{
    uint32_t y, z, sum;
    uint32_t p, rounds, e;

    if (n > 1) {
        // Encoding
        rounds = ROUNDS;
        sum = 0;
        z = v[n - 1];
        do {
            sum += DELTA;
            e = (sum >> 2) & 3;
            for (p = 0; p < (uint32_t)(n - 1); p++) {
                y = v[p + 1];
                z = v[p] += MX;
            }
            y = v[0];
            z = v[n - 1] += MX;
        } while (--rounds);
    } else if (n < -1) {
        // Decoding
        n = -n;
        rounds = ROUNDS;
        sum = rounds * DELTA;
        y = v[0];
        do {
            e = (sum >> 2) & 3;
            for (p = (uint32_t)(n - 1); p > 0; p--) {
                z = v[p - 1];
                y = v[p] -= MX;
            }
            z = v[n - 1];
            y = v[0] -= MX;
            sum -= DELTA;
        } while (--rounds);
    }
}

// ======================== Key generation ========================
// Keys are NOT hardcoded. They must be loaded at runtime via --flarm-keys <file>.
// Without keys, FLARM packets are received but NOT decrypted.

static uint32_t key_table[12] = {0};
static uint32_t flarm_KEY2 = 0;
static uint32_t flarm_KEY3 = 0;
static uint32_t flarm_KEY4 = 0;
static uint32_t key5[4] = {0};
static int flarm_keys_loaded = 0;

int flarm_keys_are_loaded(void) { return flarm_keys_loaded; }
void flarm_get_key_table(uint32_t out[12]) { memcpy(out, key_table, sizeof(key_table)); }
uint32_t flarm_get_key2(void) { return flarm_KEY2; }
uint32_t flarm_get_key3(void) { return flarm_KEY3; }
uint32_t flarm_get_key4(void) { return flarm_KEY4; }
void flarm_get_key5(uint32_t out[4]) { memcpy(out, key5, sizeof(key5)); }

static long obscure(uint32_t key, uint32_t seed)
{
    uint32_t m1 = seed * (key ^ (key >> 16));
    uint32_t m2 = (seed * (m1 ^ (m1 >> 16)));
    return m2 ^ (m2 >> 16);
}

static void make_v6_key(uint32_t out_key[4], uint32_t timestamp, uint32_t address)
{
    for (int i = 0; i < 4; i++) {
        int ndx = ((timestamp >> 23) & 1) ? i + 4 : i;
        out_key[i] = obscure(key_table[ndx] ^ ((timestamp >> 6) ^ address), flarm_KEY2) ^ flarm_KEY3;
    }
}

static void make_v7_key(uint32_t key[4])
{
    uint8_t *bkeys = (uint8_t *)&key[0];
    int p, q, x, y, z, sum;

    x = bkeys[15];
    sum = 0;
    q = 2;

    do {
        sum += DELTA;
        for (p = 0; p < 16; p++) {
            z = x & 0xFF;
            y = bkeys[(p + 1) % 16];
            x = bkeys[p];
            x += ((((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4))) ^ (sum ^ y));
            bkeys[p] = (uint8_t)x;
        }
    } while (--q > 0);
}

// ======================== Parity check ========================

static unsigned count_bits(unsigned char byte)
{
    unsigned count = 0;
    while (byte) {
        count += byte & 1;
        byte >>= 1;
    }
    return count;
}

// ======================== V7 helper: descale ========================

static int descale(unsigned int value, unsigned int mbits, unsigned int ebits)
{
    unsigned int offset   = (1 << mbits);
    unsigned int signbit  = (offset << ebits);
    unsigned int negative = (value & signbit);

    value &= (signbit - 1);

    if (value >= offset) {
        unsigned int exp = value >> mbits;
        value &= (offset - 1);
        value += offset;
        value <<= exp;
        value -= offset;
    }

    return negative ? -(int)value : (int)value;
}

// ======================== Longitude division table (V7) ========================

static const uint16_t lon_div_table[] = {
    53,  53,  54,  54,  55,  55,  56,  56,  57,  57,  58,  58,  59,  59,  60,  60,
    61,  61,  62,  62,  63,  63,  64,  64,  65,  65,  67,  68,  70,  71,  73,  74,
    76,  77,  79,  80,  82,  83,  85,  86,  88,  89,  91,  94,  98, 101, 105, 108,
   112, 115, 119, 122, 126, 129, 137, 144, 152, 159, 167, 174, 190, 205, 221, 236,
   252, 267, 299, 330, 362, 425, 489, 552, 616, 679, 743, 806, 806
};

// ======================== CRC-16 CCITT ========================

uint16_t flarm_crc16(const uint8_t *data, unsigned len)
{
    uint16_t crc = 0xFFFF;
    for (unsigned i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

bool flarm_check_crc(const uint8_t *data, unsigned len)
{
    if (len < 2) return false;
    uint16_t computed = flarm_crc16(data, len - 2);
    uint16_t received = (uint16_t)data[len - 2] | ((uint16_t)data[len - 1] << 8);
    return computed == received;
}

// ======================== V6 packet structure (packed) ========================

typedef struct {
    unsigned int addr:24;
    unsigned int type:4;
    unsigned int addr_type:3;
    unsigned int _unk1:1;

    int vs:10;
    unsigned int _unk2:2;
    unsigned int airborne:1;
    unsigned int stealth:1;
    unsigned int no_track:1;
    unsigned int parity:1;
    unsigned int gps:12;
    unsigned int aircraft_type:4;

    unsigned int lat:19;
    unsigned int alt:13;

    unsigned int lon:20;
    unsigned int _unk3:10;
    unsigned int smult:2;

    int8_t ns[4];
    int8_t ew[4];
} __attribute__((packed)) legacy_v6_pkt_t;

// ======================== V7 packet structure (packed) ========================

typedef struct {
    unsigned int addr:24;
    unsigned int type:4;
    unsigned int addr_type:3;
    unsigned int _unk1:1;

    unsigned int _unk2:22;
    unsigned int stealth:1;
    unsigned int no_track:1;
    unsigned int _unk3:2;
    unsigned int _unk4:2;
    unsigned int _unk5:2;
    unsigned int _unk6:2;

    unsigned int _unk7:2;
    unsigned int tstamp:4;
    unsigned int aircraft_type:4;
    unsigned int _unk8:1;
    unsigned int alt:13;

    unsigned int lat:20;
    unsigned int lon:20;
    int          turn:9;
    unsigned int hs:10;
    int          vs:9;
    unsigned int course:10;
    unsigned int airborne:2;

    unsigned int hp:6;
    unsigned int vp:5;
    unsigned int _unk9:5;
    unsigned int _unk10:8;
} __attribute__((packed)) legacy_v7_pkt_t;

// ======================== Init ========================

// Load FLARM decryption keys from a text file.
// File format (one per line):
//   key_table=hex,hex,hex,hex,hex,hex,hex,hex,hex,hex,hex,hex
//   key2=hex
//   key3=hex
//   key4=hex
//   key5=hex,hex,hex,hex
// Lines starting with '#' are comments.
// Returns true if all keys were loaded successfully.
bool flarm_load_keys(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "FLARM: cannot open keys file: %s\n", path);
        return false;
    }

    char line[512];
    int got_table = 0, got_k2 = 0, got_k3 = 0, got_k4 = 0, got_k5 = 0;

    while (fgets(line, sizeof(line), f)) {
        // Strip trailing whitespace
        char *end = line + strlen(line) - 1;
        while (end >= line && (*end == '\n' || *end == '\r' || *end == ' ')) *end-- = '\0';

        if (line[0] == '#' || line[0] == '\0') continue;

        if (strncmp(line, "key_table=", 10) == 0) {
            char *p = line + 10;
            for (int i = 0; i < 12 && *p; i++) {
                key_table[i] = (uint32_t)strtoul(p, &p, 16);
                if (*p == ',') p++;
            }
            got_table = 1;
        } else if (strncmp(line, "key2=", 5) == 0) {
            flarm_KEY2 = (uint32_t)strtoul(line + 5, NULL, 16);
            got_k2 = 1;
        } else if (strncmp(line, "key3=", 5) == 0) {
            flarm_KEY3 = (uint32_t)strtoul(line + 5, NULL, 16);
            got_k3 = 1;
        } else if (strncmp(line, "key4=", 5) == 0) {
            flarm_KEY4 = (uint32_t)strtoul(line + 5, NULL, 16);
            got_k4 = 1;
        } else if (strncmp(line, "key5=", 5) == 0) {
            char *p = line + 5;
            for (int i = 0; i < 4 && *p; i++) {
                key5[i] = (uint32_t)strtoul(p, &p, 16);
                if (*p == ',') p++;
            }
            got_k5 = 1;
        }
    }
    fclose(f);

    if (got_table && got_k2 && got_k3 && got_k4 && got_k5) {
        flarm_keys_loaded = 1;
        fprintf(stderr, "FLARM: decryption keys loaded from %s\n", path);
        return true;
    }

    fprintf(stderr, "FLARM: incomplete keys file %s (need key_table, key2, key3, key4, key5)\n", path);
    return false;
}

void flarm_decode_init(void)
{
    // Keys must be loaded via flarm_load_keys() before decryption works.
    // Without keys, FLARM packets are received and CRC-checked but not decrypted.
    if (!flarm_keys_loaded) {
        fprintf(stderr, "FLARM: no decryption keys loaded — packets will be received but not decoded\n");
    }
}

// ======================== V6 Decode ========================

static bool decode_v6(const uint8_t *payload, double ref_lat, double ref_lon,
                      float ref_alt_geoid, uint32_t timestamp, flarm_message_t *out)
{
    legacy_v6_pkt_t pkt;
    memcpy(&pkt, payload, sizeof(pkt));

    if (pkt.type != 0) return false;  // Not V6 position packet
    if (!flarm_keys_loaded) return false;  // No keys — cannot decrypt

    // Decrypt: BTEA on words 1..5 (skip word 0 = header)
    uint32_t key[4];
    make_v6_key(key, timestamp, (pkt.addr << 8) & 0xffffff);
    uint32_t wpkt6[sizeof(pkt)/sizeof(uint32_t)];
    memcpy(wpkt6, &pkt, sizeof(pkt));
    btea(&wpkt6[1], -5, key);
    memcpy(&pkt, wpkt6, sizeof(pkt));

    // Parity check
    uint8_t pkt_parity = 0;
    for (unsigned i = 0; i < sizeof(legacy_v6_pkt_t); i++) {
        pkt_parity += count_bits(((unsigned char *)&pkt)[i]);
    }
    if (pkt_parity % 2) return false;

    // Decode latitude (relative to receiver)
    int32_t round_lat = (int32_t)(ref_lat * 1e7) >> 7;
    int32_t lat = ((int32_t)pkt.lat - round_lat) % (int32_t)0x080000;
    if (lat >= 0x040000) lat -= 0x080000;
    lat = ((lat + round_lat) << 7);

    // Decode longitude (relative to receiver)
    int32_t round_lon = (int32_t)(ref_lon * 1e7) >> 7;
    int32_t lon = ((int32_t)pkt.lon - round_lon) % (int32_t)0x100000;
    if (lon >= 0x080000) lon -= 0x100000;
    lon = ((lon + round_lon) << 7);

    // Speed and direction
    int32_t ns = (pkt.ns[0] + pkt.ns[1] + pkt.ns[2] + pkt.ns[3]) / 4;
    int32_t ew = (pkt.ew[0] + pkt.ew[1] + pkt.ew[2] + pkt.ew[3]) / 4;
    float speed4 = sqrtf((float)(ew * ew + ns * ns)) * (1 << pkt.smult);

    float direction = 0;
    if (speed4 > 0) {
        direction = atan2f((float)ns, (float)ew) * 180.0f / (float)M_PI;
        // Convert math angle → compass bearing
        direction = (direction <= 90.0f ? 90.0f - direction : 450.0f - direction);
    }

    // Vertical speed
    uint16_t vs_u16 = pkt.vs;
    int16_t vs_i16 = (int16_t)(vs_u16 | (vs_u16 & (1 << 9) ? 0xFC00U : 0));
    int16_t vs10 = vs_i16 << pkt.smult;

    // Altitude (relative to WGS84 ellipsoid)
    int16_t alt = (int16_t)pkt.alt;

    // Fill output
    out->addr          = pkt.addr;
    out->addr_type     = pkt.addr_type;
    out->aircraft_type = pkt.aircraft_type;
    out->stealth       = pkt.stealth;
    out->no_track      = pkt.no_track;
    out->latitude      = (double)lat / 1e7;
    out->longitude     = (double)lon / 1e7;
    out->altitude      = (int)((float)alt - ref_alt_geoid);
    out->speed         = speed4 / 4.0f;          // m/s
    out->course        = direction;
    out->vs            = (float)vs10 / 10.0f;    // m/s
    out->turnrate      = 0;
    out->timestamp     = timestamp;
    out->version       = 6;
    out->valid         = true;

    return true;
}

// ======================== V7 Decode ========================

static bool decode_v7(const uint8_t *payload, double ref_lat, double ref_lon,
                      float ref_alt_geoid, uint32_t timestamp, flarm_message_t *out)
{
    legacy_v7_pkt_t pkt;
    memcpy(&pkt, payload, sizeof(pkt));

    if (pkt.type != 2) return false;  // Not V7 position packet
    if (!flarm_keys_loaded) return false;  // No keys — cannot decrypt

    uint32_t wpkt_buf[sizeof(pkt)/sizeof(uint32_t)];
    memcpy(wpkt_buf, &pkt, sizeof(pkt));
    uint32_t *wpkt = wpkt_buf;

    // Step 1: XXTEA decrypt words 2..5
    btea(&wpkt[2], -4, key5);

    // Step 2: Generate V7 XOR key from header + timestamp
    uint32_t key_v7[4];
    key_v7[0] = wpkt[0];
    key_v7[1] = wpkt[1];
    key_v7[2] = timestamp >> 4;
    key_v7[3] = flarm_KEY4;
    make_v7_key(key_v7);

    // Step 3: XOR decrypt words 2..5
    wpkt[2] ^= key_v7[0];
    wpkt[3] ^= key_v7[1];
    wpkt[4] ^= key_v7[2];
    wpkt[5] ^= key_v7[3];

    // Copy decrypted data back into packed struct for field access
    memcpy(&pkt, wpkt_buf, sizeof(pkt));

    // Altitude (enscaled)
    int16_t alt = (int16_t)(descale(pkt.alt, 12, 1) - 1000);

    // Latitude
    int32_t round_lat = (int32_t)(ref_lat * 1e7) / 52;
    int32_t lat = ((int32_t)pkt.lat - round_lat) % (int32_t)0x100000;
    if (lat >= 0x080000) lat -= 0x100000;
    lat = (lat + round_lat) * 52;

    // Longitude
    int ilat = (int)fabs(ref_lat);
    if (ilat > 89) ilat = 89;
    int32_t lon_div = (ilat < 14) ? 52 : lon_div_table[ilat - 14];

    int32_t round_lon = (int32_t)(ref_lon * 1e7) / lon_div;
    int32_t lon = ((int32_t)pkt.lon - round_lon) % (int32_t)0x100000;
    if (lon >= 0x080000) lon -= 0x100000;
    lon = (lon + round_lon) * lon_div;

    // Speed (enscaled, in 0.1 m/s)
    uint16_t speed10 = (uint16_t)descale(pkt.hs, 8, 2);

    // Vertical speed (enscaled, in 0.1 m/s)
    int16_t vs10 = (int16_t)descale(pkt.vs, 6, 2);

    // Course
    float course = (float)pkt.course / 2.0f;

    // Turn rate (enscaled, in 0.05 deg/s)
    float turnrate = (float)descale(pkt.turn, 6, 2) / 20.0f;

    // Fill output
    out->addr          = pkt.addr;
    out->addr_type     = pkt.addr_type;
    out->aircraft_type = pkt.aircraft_type;
    out->stealth       = pkt.stealth;
    out->no_track      = pkt.no_track;
    out->latitude      = (double)lat / 1e7;
    out->longitude     = (double)lon / 1e7;
    out->altitude      = (int)((float)alt - ref_alt_geoid);
    out->speed         = (float)speed10 / 10.0f;   // m/s
    out->course        = course;
    out->vs            = (float)vs10 / 10.0f;       // m/s
    out->turnrate      = turnrate;
    out->timestamp     = timestamp;
    out->version       = 7;
    out->valid         = true;

    return true;
}

// ======================== Public API ========================

bool flarm_decode_packet(const uint8_t *raw_payload,
                         double ref_lat, double ref_lon,
                         float ref_alt_geoid,
                         uint32_t timestamp,
                         flarm_message_t *out)
{
    memset(out, 0, sizeof(*out));
    out->valid = false;

    // Try V7 first (type field = bits 24..27 of first word)
    uint8_t type_field = (raw_payload[3] & 0x0F);

    if (type_field == 2) {
        return decode_v7(raw_payload, ref_lat, ref_lon, ref_alt_geoid, timestamp, out);
    } else if (type_field == 0) {
        return decode_v6(raw_payload, ref_lat, ref_lon, ref_alt_geoid, timestamp, out);
    }

    return false;
}
