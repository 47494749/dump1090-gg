// Part of dump1090-gg-light
//
// fanet_decode.h: FANET+ LoRa decoder for 868.2 MHz
//
// Decodes FANET+ (Flying Ad-hoc NETwork) packets transmitted using
// LoRa modulation (SF7, BW250kHz) on 868.2 MHz.
// FANET is used by paragliders, hang-gliders, and other aircraft for
// position reporting and messaging.
//
// Reference: https://github.com/3s1d/fanet-stm32/blob/master/Src/fanet/radio/protocol.txt
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#ifndef FANET_DECODE_H
#define FANET_DECODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// ======================== Constants ========================

#define FANET_SAMPLE_RATE     1000000   // 1 MSPS keeps 512 samples/symbol and is accepted by RTL-SDR
#define FANET_CENTER_FREQ     868200000 // 868.2 MHz
#define FANET_BW              250000    // 250 kHz LoRa bandwidth
#define FANET_SF              7         // Spreading Factor 7
#define FANET_PREAMBLE_LEN    8         // 8 preamble symbols
#define FANET_SYNC_WORD       0xF1      // FANET LoRa sync word (register value)
#define FANET_MAX_PAYLOAD     64        // max payload bytes
#define FANET_LANDMARK_MAX_POINTS 32    // max polygon/polyline points

// Derived constants
#define FANET_CHIPS_PER_SYM   (1 << FANET_SF)   // 128 chips per symbol
#define FANET_SYM_RATE        (FANET_BW / FANET_CHIPS_PER_SYM)  // ~1953 sym/s
#define FANET_SAMPLES_PER_SYM (FANET_SAMPLE_RATE / FANET_SYM_RATE) // ~256

// ======================== FANET message types ========================

typedef enum {
    FANET_TYPE_ACK       = 0,   // ACK / NACK
    FANET_TYPE_TRACKING  = 1,   // Air tracking (position/speed/altitude)
    FANET_TYPE_NAME      = 2,   // Name (pilot/aircraft name)
    FANET_TYPE_MESSAGE   = 3,   // Text message
    FANET_TYPE_SERVICE   = 4,   // Weather/service data
    FANET_TYPE_LANDMARK  = 5,   // Landmarks (airspace, waypoints)
    FANET_TYPE_REMOTE    = 6,   // Remote configuration
    FANET_TYPE_GROUND    = 7,   // Ground tracking
    FANET_TYPE_HWINFO    = 8,   // Hardware info (DEPRECATED, replaced by 0xA)
    FANET_TYPE_THERMAL   = 9,   // Thermal information
    FANET_TYPE_HWINFO2   = 0xA, // Hardware info v2 (replaces type 8)
} fanet_msg_type_t;

// ======================== Aircraft type (Type 1 tracking) ========================

typedef enum {
    FANET_AIRCRAFT_OTHER      = 0,
    FANET_AIRCRAFT_PARAGLIDER = 1,
    FANET_AIRCRAFT_HANGGLIDER = 2,
    FANET_AIRCRAFT_BALLOON    = 3,
    FANET_AIRCRAFT_GLIDER     = 4,
    FANET_AIRCRAFT_POWERED    = 5,
    FANET_AIRCRAFT_HELI       = 6,
    FANET_AIRCRAFT_UAV        = 7,
} fanet_aircraft_type_t;

// ======================== Ground type (Type 7 ground tracking) ========================

typedef enum {
    FANET_GROUND_OTHER          = 0,
    FANET_GROUND_WALKING        = 1,
    FANET_GROUND_VEHICLE        = 2,
    FANET_GROUND_BIKE           = 3,
    FANET_GROUND_BOOT           = 4,
    FANET_GROUND_NEED_RIDE      = 8,
    FANET_GROUND_LANDED_OK      = 9,
    FANET_GROUND_NEED_TECH      = 12,
    FANET_GROUND_NEED_MEDICAL   = 13,
    FANET_GROUND_DISTRESS       = 14,
    FANET_GROUND_DISTRESS_AUTO  = 15,
} fanet_ground_type_t;

// ======================== Landmark subtype (Type 5) ========================

typedef enum {
    FANET_LANDMARK_TEXT          = 0,
    FANET_LANDMARK_LINE          = 1,
    FANET_LANDMARK_ARROW         = 2,
    FANET_LANDMARK_AREA          = 3,
    FANET_LANDMARK_AREA_FILLED   = 4,
    FANET_LANDMARK_CIRCLE        = 5,
    FANET_LANDMARK_CIRCLE_FILLED = 6,
    FANET_LANDMARK_3D_LINE       = 7,
    FANET_LANDMARK_3D_AREA       = 8,
    FANET_LANDMARK_3D_CYLINDER   = 9,
} fanet_landmark_subtype_t;

// ======================== Landmark layer ========================

typedef enum {
    FANET_LAYER_INFO       = 0,
    FANET_LAYER_WARNING    = 1,
    FANET_LAYER_KEEPOUT    = 2,
    FANET_LAYER_TOUCHDOWN  = 3,
    FANET_LAYER_NO_WARN    = 4,
    FANET_LAYER_DONTCARE   = 15,
} fanet_landmark_layer_t;

// ======================== Remote configuration subtypes (Type 6) ========================

typedef enum {
    FANET_REMOTE_ACK          = 0,    // Acknowledge configuration
    FANET_REMOTE_REQUEST      = 1,    // Request subtype
    FANET_REMOTE_POSITION     = 2,    // Position + altitude + heading
    FANET_REMOTE_RESERVED     = 3,    // Reserved
    // 4..8: Geofence for geo-forwarding
    FANET_REMOTE_GEOFENCE_4   = 4,
    FANET_REMOTE_GEOFENCE_5   = 5,
    FANET_REMOTE_GEOFENCE_6   = 6,
    FANET_REMOTE_GEOFENCE_7   = 7,
    FANET_REMOTE_GEOFENCE_8   = 8,
    // 9..33: Broadcast reply features
} fanet_remote_subtype_t;

// ======================== Landmark point ========================

typedef struct {
    double   latitude;
    double   longitude;
    float    radius;            // for circle types (meters)
    int      altitude_bottom;   // for 3D types (meters MSL)
    int      altitude_top;      // for 3D types (meters MSL)
} fanet_landmark_point_t;

// ======================== Decoded message ========================

typedef struct {
    bool     valid;
    uint64_t timestamp_ms;      // reception timestamp
    float    signal_level;      // 0.0 .. 1.0

    // MAC header
    uint8_t  src_manufacturer;  // source manufacturer ID
    uint16_t src_id;            // source unique ID (16-bit, LE)
    uint8_t  dst_manufacturer;  // destination manufacturer (0=broadcast)
    uint16_t dst_id;            // destination ID (0=broadcast)
    bool     forward;           // forward bit
    bool     extended_header;   // extended header present
    bool     unicast;           // unicast (vs broadcast)
    bool     signature_present; // 4-byte signature present
    bool     signature_checked; // verification attempted with a configured PSK
    bool     signature_valid;   // signature matched configured PSK
    bool     geo_forwarded;     // geo-based forwarded flag
    uint8_t  ack_type;          // 0=none, 1=requested, 2=requested via fwd
    uint32_t signature;         // raw 32-bit signature (little-endian)

    // Application layer
    fanet_msg_type_t type;
    uint8_t  payload[FANET_MAX_PAYLOAD];
    uint8_t  payload_len;

    // ---- Type 1: Air tracking ----
    struct {
        double   latitude;          // degrees
        double   longitude;         // degrees
        int      altitude;          // meters (MSL)
        float    speed;             // km/h
        float    climb;             // m/s
        float    heading;           // degrees 0-360
        float    turn_rate;         // deg/s (optional, NaN if absent)
        int      qne_offset;       // meters (optional, INT_MIN if absent)
        fanet_aircraft_type_t aircraft_type;
        bool     online_tracking;
        bool     position_valid;
    } tracking;

    // ---- Type 2: Name ----
    char name[32];

    // ---- Type 3: Message ----
    struct {
        uint8_t  subtype;           // 0 = normal message
        char     text[FANET_MAX_PAYLOAD];
    } message;

    // ---- Type 4: Service / Weather ----
    struct {
        bool     is_gateway;        // internet gateway flag
        bool     remote_cfg;        // supports remote configuration
        double   latitude;          // degrees (NaN if absent)
        double   longitude;         // degrees (NaN if absent)
        float    temperature;       // degrees C
        float    wind_speed;        // km/h
        float    wind_gust;         // km/h
        float    wind_heading;      // degrees 0-360
        float    humidity;          // %rh
        float    pressure;          // hPa
        float    state_of_charge;   // 0-100% (-1 if absent)
        bool     has_position;
        bool     has_temp;
        bool     has_wind;
        bool     has_humidity;
        bool     has_pressure;
        bool     has_soc;
    } weather;

    // ---- Type 5: Landmark ----
    struct {
        fanet_landmark_subtype_t subtype;
        fanet_landmark_layer_t   layer;
        uint16_t ttl_minutes;           // time to live in minutes
        uint8_t  wind_sectors;          // bitmask of wind sectors (0=no wind dep)
        bool     wind_dependent;
        double   latitude;              // first coordinate
        double   longitude;
        char     text[FANET_MAX_PAYLOAD]; // for text landmarks
        uint8_t  num_points;            // number of additional coordinates
        fanet_landmark_point_t points[FANET_LANDMARK_MAX_POINTS];
    } landmark;

    // ---- Type 6: Remote configuration ----
    struct {
        fanet_remote_subtype_t subtype;
        bool     valid;
        // Subtype 0: Ack
        uint8_t  ack_subtype;        // which subtype is being acknowledged
        // Subtype 1: Request
        uint8_t  request_subtype;    // which subtype is being requested
        // Subtype 2: Position
        double   latitude;
        double   longitude;
        int      altitude;           // meters MSL
        float    heading;            // degrees 0-360
        bool     has_position;
        bool     has_altitude;
        bool     has_heading;
        // Subtype 4-8: Geofence
        int      geofence_alt_bottom;  // meters MSL
        int      geofence_alt_top;     // meters MSL
        uint8_t  geofence_num_points;
        fanet_landmark_point_t geofence_points[FANET_LANDMARK_MAX_POINTS];
        // Subtype 9-33: Broadcast reply
        uint8_t  reply_wind_sectors;
        uint8_t  reply_type;         // embedded message type
        bool     reply_forward;      // forward bit in reply
    } remote;

    // ---- Type 7: Ground tracking ----
    struct {
        double   latitude;
        double   longitude;
        fanet_ground_type_t ground_type;
        bool     online_tracking;
        bool     position_valid;
    } ground;

    // ---- Type 9: Thermal ----
    struct {
        double   latitude;          // thermal position
        double   longitude;
        int      altitude;          // thermal altitude (meters)
        float    climb;             // avg climb m/s (air, not glider)
        float    wind_speed;        // avg wind speed km/h
        float    wind_heading;      // avg wind heading degrees
        uint8_t  confidence;        // 0-7 (0=0%, 7=100%)
        bool     valid;
    } thermal;

    // ---- Type 8/A: HW Info ----
    struct {
        uint8_t  device_type;       // manufacturer-specific
        uint16_t build_date;        // encoded: bit15=dev, bit9-14=year+2019, bit5-8=month, bit0-4=day
        uint32_t icao_address;      // ICAO address (if present, 0 otherwise)
        uint16_t uptime_minutes;    // uptime (if present, 0 otherwise)
        int8_t   rssi;              // last rx RSSI in dBm (if present)
        bool     has_build_date;
        bool     has_icao;
        bool     has_uptime;
        bool     has_rssi;
    } hwinfo;

} fanet_message_t;

// ======================== Decoder state (opaque) ========================

typedef struct fanet_state fanet_state_t;

// ======================== API ========================

fanet_state_t *fanet_create(uint32_t sample_rate);
void fanet_destroy(fanet_state_t *state);
void fanet_process(fanet_state_t *state, const uint8_t *iq, uint32_t len);
bool fanet_dequeue(fanet_state_t *state, fanet_message_t *msg);

// Statistics
typedef struct {
    uint64_t samples_processed;
    uint64_t preambles_detected;
    uint64_t sync_word_ok;
    uint64_t packets_decoded;
    uint64_t crc_errors;
    uint64_t header_errors;
    // Per-type counters
    uint64_t type_counts[16];
} fanet_stats_t;

void fanet_get_stats(const fanet_state_t *state, fanet_stats_t *stats);

// ======================== Name cache ========================
// Thread-safe lookup: given a FANET address, return the cached name (if any).
// Returns true if found, copies name into buf (up to buf_len).
bool fanet_get_cached_name(const fanet_state_t *state,
                           uint8_t manufacturer, uint16_t id,
                           char *buf, int buf_len);

// Get FANET address as a 24-bit value for use as synthetic ICAO
// Format: manufacturer(8) | id(16) → 24-bit
static inline uint32_t fanet_addr24(uint8_t mfr, uint16_t id)
{
    return ((uint32_t)mfr << 16) | id;
}

// Manufacturer name lookup
const char *fanet_manufacturer_name(uint8_t mfr_id);

#ifdef __cplusplus
}
#endif

#endif // FANET_DECODE_H
