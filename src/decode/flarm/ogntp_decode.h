// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// ogntp_decode.h: OGN Tracker Protocol (OGNTP / OGN1) packet decoder
//
// OGN1 uses the same 868 MHz band and GFSK modulation as FLARM Legacy,
// with an 8-byte syncword, inverted Manchester encoding, and LDPC (208,160)
// forward error correction.  There is no encryption.
//
// Protocol references:
//   https://github.com/pjalocha/esp32-ogn-tracker
//   https://github.com/lyusupov/SoftRF
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#ifndef OGNTP_DECODE_H
#define OGNTP_DECODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// ======================== Protocol constants ========================

// OGN1 syncword (Manchester-encoded bytes as they appear in the bit-stream).
// Raw (pre-Manchester) sync = 0x0AF3656C.
#define OGNTP_SYNCWORD_SIZE  8
extern const uint8_t OGNTP_SYNCWORD[OGNTP_SYNCWORD_SIZE];

// Total bytes per OGN1 packet: 20 data bytes + 6 LDPC parity bytes
#define OGNTP_PACKET_TOTAL   26
#define OGNTP_DATA_BYTES     20

// ======================== Decoded message ========================

typedef struct {
    uint32_t addr;          // 24-bit address
    int      addr_type;     // 0=random, 1=ICAO, 2=FLARM, 3=OGN
    int      version;       // 1=OGN1, 2=OGN2
    int      relay;         // relay count (OGN1) or flag (OGN2)
    int      nonpos;        // non-zero = non-position packet
    int      encrypted;     // non-zero = encrypted/custom payload
    int      emergency;     // non-zero = emergency flag set
    int      other_system;  // OGN2 NonOGN bit, zero for OGN1
    int      report_type;   // non-position report type, -1 for position packets
    int      position_valid;
    int      status_valid;
    int      aircraft_type; // 0-15 (same numbering as FLARM aircraft types)
    int      stealth;       // non-zero = stealth flag set
    double   latitude;      // decimal degrees
    double   longitude;     // decimal degrees
    int      altitude;      // meters MSL
    float    speed;         // m/s ground speed
    float    course;        // degrees (0-359.9)
    float    vs;            // m/s vertical speed (positive = climbing)
    float    turnrate;      // deg/s (0 if unknown)
    int      fix_quality;   // 0=none, 1=GPS, 2=DGPS
    struct {
        int   time_seconds;
        int   fix_quality;
        int   hardware;
        int   firmware;
        int   satellites;
        int   tx_power_dbm;
        int   pulse_bpm;
        int   oxygen_percent;
        int   sat_snr_db;
        int   rx_rate_per_min;
        int   audio_noise_db;
        int   has_temperature;
        int   has_humidity;
        int   has_pressure;
        float radio_noise_dbm;
        float pressure_hpa;
        float voltage_v;
        float temperature_c;
        float humidity_percent;
    } status;
    float    signal_level;  // 0-1 relative signal level
    int      valid;         // non-zero = message is valid and fully decoded
} ogntp_message_t;

// ======================== Functions ========================

// Check LDPC parity of a 26-byte OGN1 packet.
// Returns 0 if all 48 parity checks pass (packet is valid).
// Returns the number of failed checks otherwise.
uint8_t ogntp_ldpc_check(const uint8_t *data26);

// Decode a 26-byte OGN1 packet.
// ref_lat, ref_lon: receiver position in decimal degrees (for sanity checks).
// Returns true on success; fills *msg with decoded fields.
bool ogntp_decode_packet(const uint8_t *data26,
                         double ref_lat, double ref_lon,
                         ogntp_message_t *msg);

#ifdef __cplusplus
}
#endif

#endif // OGNTP_DECODE_H
