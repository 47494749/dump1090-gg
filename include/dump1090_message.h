// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// dump1090_message.h: decoded message structure
//

#ifndef DUMP1090_MESSAGE_H
#define DUMP1090_MESSAGE_H

#ifdef __cplusplus
extern "C" {
#endif

// The struct we use to store information about a decoded message.
struct modesMessage {
    // Generic fields
    uint8_t       msg[MODES_LONG_MSG_BYTES];      // Binary message.
    uint8_t       verbatim[MODES_LONG_MSG_BYTES]; // Binary message, as originally received before correction
    int           msgbits;                        // Number of bits in message
    int           msgtype;                        // Downlink format #
    uint32_t      crc;                            // Message CRC
    int           correctedbits;                  // No. of bits corrected
    uint32_t      addr;                           // Address Announced
    addrtype_t    addrtype;                       // address format / source
    uint64_t      timestampMsg;                   // Timestamp of the message (12MHz clock)
    uint64_t      sysTimestampMsg;                // Timestamp of the message (system time)
    int           remote;                         // If set this message is from a remote station
    double        signalLevel;                    // RSSI, in the range [0..1], as a fraction of full-scale power
    int           score;                          // Scoring from scoreModesMessage, if used
    int           reliable;                       // is this a "reliable" message (uncorrected DF11/DF17/DF18)?

    datasource_t  source;                         // Characterizes the overall message source

    // Raw data, just extracted directly from the message
    // The names reflect the field names in Annex 4
    uint32_t IID; // extracted from CRC of DF11s
    uint32_t AA;
    uint32_t AC;
    uint32_t CA;
    uint32_t CC;
    uint32_t CF;
    uint32_t DR;
    uint32_t FS;
    uint32_t ID;
    uint32_t KE;
    uint32_t ND;
    uint32_t RI;
    uint32_t SL;
    uint32_t UM;
    uint32_t VS;
    uint8_t MB[7];
    uint8_t MD[10];
    uint8_t ME[7];
    uint8_t MV[7];

    // Decoded data
    uint32_t altitude_baro_valid : 1;
    uint32_t altitude_geom_valid : 1;
    uint32_t track_valid : 1;
    uint32_t track_rate_valid : 1;
    uint32_t heading_valid : 1;
    uint32_t roll_valid : 1;
    uint32_t gs_valid : 1;
    uint32_t ias_valid : 1;
    uint32_t tas_valid : 1;
    uint32_t mach_valid : 1;
    uint32_t baro_rate_valid : 1;
    uint32_t geom_rate_valid : 1;
    uint32_t squawk_valid : 1;
    uint32_t callsign_valid : 1;
    uint32_t cpr_valid : 1;
    uint32_t cpr_odd : 1;
    uint32_t cpr_decoded : 1;
    uint32_t cpr_relative : 1;
    uint32_t category_valid : 1;
    uint32_t geom_delta_valid : 1;
    uint32_t from_mlat : 1;
    uint32_t from_tisb : 1;
    uint32_t spi_valid : 1;
    uint32_t spi : 1;
    uint32_t alert_valid : 1;
    uint32_t alert : 1;
    uint32_t emergency_valid : 1;

    uint32_t metype; // DF17/18 ME type
    uint32_t mesub;  // DF17/18 ME subtype

    commb_format_t commb_format; // Inferred format of a comm-b message

    // valid if altitude_baro_valid:
    int               altitude_baro;       // Altitude in either feet or meters
    altitude_unit_t   altitude_baro_unit;  // the unit used for altitude

    // valid if altitude_geom_valid:
    int               altitude_geom;       // Altitude in either feet or meters
    altitude_unit_t   altitude_geom_unit;  // the unit used for altitude

    // following fields are valid if the corresponding _valid field is set:
    int      geom_delta;        // Difference between geometric and baro alt
    float    heading;           // ground track or heading, degrees (0-359). Reported directly or computed from from EW and NS velocity
    heading_type_t heading_type;// how to interpret 'track_or_heading'
    float    track_rate;        // Rate of change of track, degrees/second
    float    roll;              // Roll, degrees, negative is left roll
    struct {
        // Groundspeed, kts, reported directly or computed from from EW and NS velocity
        // For surface movement, this has different interpretations for v0 and v2; both
        // fields are populated. The tracking layer will update "gs.selected".
        float v0;
        float v2;
        float selected;
    } gs;
    uint32_t ias;               // Indicated airspeed, kts
    uint32_t tas;               // True airspeed, kts
    double   mach;              // Mach number
    int      baro_rate;         // Rate of change of barometric altitude, feet/minute
    int      geom_rate;         // Rate of change of geometric (GNSS / INS) altitude, feet/minute
    uint32_t squawk;            // 13 bits identity (Squawk), encoded as 4 hex digits
    char     callsign[9];       // 8 chars flight number, NUL-terminated
    uint32_t category;          // A0 - D7 encoded as a single hex byte
    emergency_t emergency;      // emergency/priority status

    // valid if cpr_valid
    cpr_type_t cpr_type;       // The encoding type used (surface, airborne, coarse TIS-B)
    uint32_t   cpr_lat;        // Non decoded latitude.
    uint32_t   cpr_lon;        // Non decoded longitude.
    uint32_t   cpr_nucp;       // NUCp/NIC value implied by message type

    airground_t airground;     // air/ground state

    // valid if cpr_decoded:
    double decoded_lat;
    double decoded_lon;
    uint32_t decoded_nic;
    uint32_t decoded_rc;

    // various integrity/accuracy things
    struct {
        uint32_t nic_a_valid : 1;
        uint32_t nic_b_valid : 1;
        uint32_t nic_c_valid : 1;
        uint32_t nic_baro_valid : 1;
        uint32_t nac_p_valid : 1;
        uint32_t nac_v_valid : 1;
        uint32_t gva_valid : 1;
        uint32_t sda_valid : 1;

        uint32_t nic_a : 1;        // if nic_a_valid
        uint32_t nic_b : 1;        // if nic_b_valid
        uint32_t nic_c : 1;        // if nic_c_valid
        uint32_t nic_baro : 1;     // if nic_baro_valid

        uint32_t nac_p;        // if nac_p_valid
        uint32_t nac_v;        // if nac_v_valid

        uint32_t sil;          // if sil_type != SIL_INVALID
        sil_type_t sil_type;

        uint32_t gva;          // if gva_valid
        uint32_t sda;          // if sda_valid
    } accuracy;

    // Operational Status
    struct {
        uint32_t valid : 1;
        uint32_t version;

        uint32_t om_acas_ra : 1;
        uint32_t om_ident : 1;
        uint32_t om_atc : 1;
        uint32_t om_saf : 1;

        uint32_t cc_acas : 1;
        uint32_t cc_cdti : 1;
        uint32_t cc_1090_in : 1;
        uint32_t cc_arv : 1;
        uint32_t cc_ts : 1;
        uint32_t cc_tc;
        uint32_t cc_uat_in : 1;
        uint32_t cc_poa : 1;
        uint32_t cc_b2_low : 1;
        uint32_t cc_lw_valid : 1;

        heading_type_t tah;
        heading_type_t hrd;

        uint32_t cc_lw;
        uint32_t cc_antenna_offset;
    } opstatus;

    // combined:
    //   Target State & Status (ADS-B V2 only)
    //   Comm-B BDS4,0 Vertical Intent
    struct {
        uint32_t heading_valid : 1;
        uint32_t fms_altitude_valid : 1;
        uint32_t mcp_altitude_valid : 1;
        uint32_t qnh_valid : 1;
        uint32_t modes_valid : 1;

        float    heading;       // heading, degrees (0-359) (could be magnetic or true heading; magnetic recommended)
        heading_type_t heading_type;
        int      fms_altitude;  // FMS selected altitude
        int      mcp_altitude;  // MCP/FCU selected altitude
        float    qnh;           // altimeter setting (QFE or QNH/QNE), millibars

        nav_altitude_source_t altitude_source;

        nav_modes_t modes;
    } nav;

    // BDS 4,4 MRAR
    uint32_t mrar_source_valid : 1;
    uint32_t wind_valid : 1;
    uint32_t temperature_valid : 1;
    uint32_t pressure_valid : 1;
    uint32_t turbulence_valid : 1;
    uint32_t humidity_valid : 1;

    mrar_source_t mrar_source;
    float wind_speed;    // kts
    float wind_dir;      // degrees
    float temperature;   // degrees C
    float pressure;      // hPa
    hazard_t turbulence; // NIL/LIGHT/MODERATE/SEVERE
    float humidity;      // 0-100 %

    // BDS 4,5 MHAR (Meteorological Hazard Report)
    uint32_t mhar_turbulence_valid : 1;
    uint32_t mhar_windshear_valid : 1;
    uint32_t mhar_microburst_valid : 1;
    uint32_t mhar_icing_valid : 1;
    uint32_t mhar_wake_valid : 1;
    uint32_t mhar_sat_valid : 1;
    uint32_t mhar_asp_valid : 1;
    uint32_t mhar_rh_valid : 1;

    hazard_t mhar_turbulence;
    hazard_t mhar_windshear;
    hazard_t mhar_microburst;
    hazard_t mhar_icing;
    hazard_t mhar_wake;
    float mhar_sat;    // degrees C
    float mhar_asp;    // hPa
    float mhar_rh;     // radio height, ft (LSB 16 ft)

    // BDS 3,0 ACAS RA (decoded fields)
    uint32_t acas_ra_valid : 1;
    uint32_t acas_ara;       // 14 bits: Active Resolution Advisories
    uint32_t acas_rac;       // 4 bits: Resolution Advisory Complement
    uint32_t acas_rat : 1;   // RA Terminated
    uint32_t acas_mte : 1;   // Multiple Threat Encounter
    uint32_t acas_tti;       // 2 bits: Threat Type Indicator
    uint32_t acas_threat_id; // 26 bits: Threat Identity Data

    // BDS 4,1 - Next Waypoint Identifier
    uint32_t waypoint_valid : 1;
    char waypoint_id[9]; // 8 chars + NUL

    // BDS 4,2 - Next Waypoint Position
    uint32_t waypoint_lat_valid : 1;
    uint32_t waypoint_lon_valid : 1;
    uint32_t waypoint_alt_valid : 1;
    double waypoint_lat;
    double waypoint_lon;
    int waypoint_alt; // feet

    // BDS 4,3 - Next Waypoint Crossing Info
    uint32_t waypoint_crossing_alt_valid : 1;
    uint32_t waypoint_crossing_speed_valid : 1;
    int waypoint_crossing_alt; // feet
    uint32_t waypoint_crossing_speed; // kts
};

#ifdef __cplusplus
}
#endif

#endif // DUMP1090_MESSAGE_H
