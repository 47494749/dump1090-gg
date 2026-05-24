// decoder_types.h: Common data structures shared by all decoders.
// These structs flow through DecoderQueue and are consumed by the Dispatcher.
// C/C++ compatible via extern "C" for the struct definitions.

#ifndef DECODER_TYPES_H
#define DECODER_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ======================== Decoder source ID ========================

typedef enum {
    DECODE_SOURCE_ADSB = 0,
    DECODE_SOURCE_FLARM,
    DECODE_SOURCE_FANET,
    DECODE_SOURCE_OGNTP,
    DECODE_SOURCE_P3I,
    DECODE_SOURCE_ADSL,
    DECODE_SOURCE_MLAT,
    DECODE_SOURCE_TISB,
    DECODE_SOURCE_POCSAG,
    DECODE_SOURCE_ACARS,
    DECODE_SOURCE_VDL2,
    DECODE_SOURCE_RADIOSONDE,
    DECODE_SOURCE_GSM,
    DECODE_SOURCE_LTE,
    DECODE_SOURCE_IOT868,
    DECODE_SOURCE_SARSAT,
    DECODE_SOURCE_COUNT
} decode_source_t;

// ======================== Air/ground state ========================

typedef enum {
    DECODE_AG_UNKNOWN = 0,
    DECODE_AG_GROUND,
    DECODE_AG_AIRBORNE
} decode_airground_t;

// ======================== AircraftUpdate ========================
// Common struct for all decoders → aircraft list + feeders.
// Each decoder fills only the fields it knows; others stay invalid.

typedef struct {
    uint32_t          addr;              // ICAO 24-bit (or pseudo-ICAO)
    uint64_t          timestamp_ms;      // epoch milliseconds
    double            signal_level;      // 0.0 .. 1.0
    decode_source_t   source;

    // Identity
    char              callsign[9];       // NUL-terminated, 8 chars max
    bool              callsign_valid;
    uint8_t           category;          // ADS-B emitter category
    bool              category_valid;

    // Position
    double            lat, lon;
    bool              position_valid;
    int32_t               altitude_ft;
    bool              altitude_valid;
    bool              altitude_is_baro;  // true=barometric, false=GNSS

    // Velocity
    int32_t               ground_speed_kt;
    int32_t               heading_deg;
    int32_t               vert_rate_fpm;
    bool              velocity_valid;

    // Squawk (ADS-B only)
    uint16_t          squawk;
    bool              squawk_valid;

    // Air/ground
    decode_airground_t air_ground;

    // FLARM/OGN/ADS-L metadata (optional)
    uint8_t           flarm_acft_type;     // FLARM aircraft type enum (1-15), 0=unset
    uint8_t           flarm_addr_type;     // FLARM address type (0=random,1=ICAO,2=FLARM,3=anon)
    uint8_t           flarm_proto_version; // FLARM protocol version (6,7), 0=unset

    // Radiosonde metadata (optional)
    struct {
        char          serial[16];      // sonde serial (e.g. "T1234567")
        char          sonde_type[8];   // "RS41", "DFM", etc.
        int32_t           frame_num;       // frame counter
        int32_t           rs_errors;       // RS corrections (-1=uncorrectable)
        int32_t           satellites;      // GPS satellites
        double        vel_v;           // vertical velocity m/s
        float         freq_mhz;        // receive frequency
        bool          valid;
    } sonde;

    // FANET/FLARM extended info (optional)
    struct {
        uint8_t       device_type;
        uint16_t      uptime_min;
        int8_t        rssi;
        bool          valid;
    } hw_info;

    struct {
        double        lat, lon;
        int32_t           altitude_m;
        double        climb_ms;
        double        wind_speed_kmh;
        int32_t           wind_heading_deg;
        uint8_t       confidence;
        bool          valid;
    } thermal;
} aircraft_update_t;

// ======================== TextMessage ========================
// FANET type 4, POCSAG pages, future FLARM messaging, etc.

typedef struct {
    uint64_t          timestamp_ms;
    decode_source_t   source;
    uint32_t          src_addr;
    char              text[256];
} text_message_t;

// ======================== GroundTrack ========================
// FANET type 7 ground tracking (walkers, vehicles, bikes...)

typedef struct {
    uint32_t          addr;
    uint64_t          timestamp_ms;
    double            lat, lon;
    int32_t               altitude_m;
    double            speed_kmh;
    int32_t               heading_deg;
    uint8_t           ground_type;       // walking, vehicle, bike...
    char              name[32];
    double            signal_level;
} ground_track_t;

// ======================== RawModeS ========================
// Raw Mode-S message for Beast/SBS output (ADS-B only).

typedef struct {
    uint8_t           msg[14];           // max 112 bits = 14 bytes
    uint8_t           msgbits;           // 56 or 112
    uint64_t          timestamp_12mhz;   // original 12 MHz clock
    double            signal_level;
    uint8_t           score;
} raw_modes_t;

#ifdef __cplusplus
}
#endif

#endif // DECODER_TYPES_H
