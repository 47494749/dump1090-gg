// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// dump1090_message.h: decoded message structure
//

#ifndef DUMP1090_MESSAGE_H
#define DUMP1090_MESSAGE_H

// The struct we use to store information about a decoded message.
struct modesMessage {
    // Generic fields
    unsigned char msg[MODES_LONG_MSG_BYTES];      // Binary message.
    unsigned char verbatim[MODES_LONG_MSG_BYTES]; // Binary message, as originally received before correction
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
    unsigned IID; // extracted from CRC of DF11s
    unsigned AA;
    unsigned AC;
    unsigned CA;
    unsigned CC;
    unsigned CF;
    unsigned DR;
    unsigned FS;
    unsigned ID;
    unsigned KE;
    unsigned ND;
    unsigned RI;
    unsigned SL;
    unsigned UM;
    unsigned VS;
    unsigned char MB[7];
    unsigned char MD[10];
    unsigned char ME[7];
    unsigned char MV[7];

    // Decoded data
    unsigned altitude_baro_valid : 1;
    unsigned altitude_geom_valid : 1;
    unsigned track_valid : 1;
    unsigned track_rate_valid : 1;
    unsigned heading_valid : 1;
    unsigned roll_valid : 1;
    unsigned gs_valid : 1;
    unsigned ias_valid : 1;
    unsigned tas_valid : 1;
    unsigned mach_valid : 1;
    unsigned baro_rate_valid : 1;
    unsigned geom_rate_valid : 1;
    unsigned squawk_valid : 1;
    unsigned callsign_valid : 1;
    unsigned cpr_valid : 1;
    unsigned cpr_odd : 1;
    unsigned cpr_decoded : 1;
    unsigned cpr_relative : 1;
    unsigned category_valid : 1;
    unsigned geom_delta_valid : 1;
    unsigned from_mlat : 1;
    unsigned from_tisb : 1;
    unsigned spi_valid : 1;
    unsigned spi : 1;
    unsigned alert_valid : 1;
    unsigned alert : 1;
    unsigned emergency_valid : 1;

    unsigned metype; // DF17/18 ME type
    unsigned mesub;  // DF17/18 ME subtype

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
    unsigned ias;               // Indicated airspeed, kts
    unsigned tas;               // True airspeed, kts
    double   mach;              // Mach number
    int      baro_rate;         // Rate of change of barometric altitude, feet/minute
    int      geom_rate;         // Rate of change of geometric (GNSS / INS) altitude, feet/minute
    unsigned squawk;            // 13 bits identity (Squawk), encoded as 4 hex digits
    char     callsign[9];       // 8 chars flight number, NUL-terminated
    unsigned category;          // A0 - D7 encoded as a single hex byte
    emergency_t emergency;      // emergency/priority status

    // valid if cpr_valid
    cpr_type_t cpr_type;       // The encoding type used (surface, airborne, coarse TIS-B)
    unsigned   cpr_lat;        // Non decoded latitude.
    unsigned   cpr_lon;        // Non decoded longitude.
    unsigned   cpr_nucp;       // NUCp/NIC value implied by message type

    airground_t airground;     // air/ground state

    // valid if cpr_decoded:
    double decoded_lat;
    double decoded_lon;
    unsigned decoded_nic;
    unsigned decoded_rc;

    // various integrity/accuracy things
    struct {
        unsigned nic_a_valid : 1;
        unsigned nic_b_valid : 1;
        unsigned nic_c_valid : 1;
        unsigned nic_baro_valid : 1;
        unsigned nac_p_valid : 1;
        unsigned nac_v_valid : 1;
        unsigned gva_valid : 1;
        unsigned sda_valid : 1;

        unsigned nic_a : 1;        // if nic_a_valid
        unsigned nic_b : 1;        // if nic_b_valid
        unsigned nic_c : 1;        // if nic_c_valid
        unsigned nic_baro : 1;     // if nic_baro_valid

        unsigned nac_p;        // if nac_p_valid
        unsigned nac_v;        // if nac_v_valid

        unsigned sil;          // if sil_type != SIL_INVALID
        sil_type_t sil_type;

        unsigned gva;          // if gva_valid
        unsigned sda;          // if sda_valid
    } accuracy;

    // Operational Status
    struct {
        unsigned valid : 1;
        unsigned version;

        unsigned om_acas_ra : 1;
        unsigned om_ident : 1;
        unsigned om_atc : 1;
        unsigned om_saf : 1;

        unsigned cc_acas : 1;
        unsigned cc_cdti : 1;
        unsigned cc_1090_in : 1;
        unsigned cc_arv : 1;
        unsigned cc_ts : 1;
        unsigned cc_tc;
        unsigned cc_uat_in : 1;
        unsigned cc_poa : 1;
        unsigned cc_b2_low : 1;
        unsigned cc_lw_valid : 1;

        heading_type_t tah;
        heading_type_t hrd;

        unsigned cc_lw;
        unsigned cc_antenna_offset;
    } opstatus;

    // combined:
    //   Target State & Status (ADS-B V2 only)
    //   Comm-B BDS4,0 Vertical Intent
    struct {
        unsigned heading_valid : 1;
        unsigned fms_altitude_valid : 1;
        unsigned mcp_altitude_valid : 1;
        unsigned qnh_valid : 1;
        unsigned modes_valid : 1;

        float    heading;       // heading, degrees (0-359) (could be magnetic or true heading; magnetic recommended)
        heading_type_t heading_type;
        int      fms_altitude;  // FMS selected altitude
        int      mcp_altitude;  // MCP/FCU selected altitude
        float    qnh;           // altimeter setting (QFE or QNH/QNE), millibars

        nav_altitude_source_t altitude_source;

        nav_modes_t modes;
    } nav;

    // BDS 4,4 MRAR
    unsigned mrar_source_valid : 1;
    unsigned wind_valid : 1;
    unsigned temperature_valid : 1;
    unsigned pressure_valid : 1;
    unsigned turbulence_valid : 1;
    unsigned humidity_valid : 1;

    mrar_source_t mrar_source;
    float wind_speed;    // kts
    float wind_dir;      // degrees
    float temperature;   // degrees C
    float pressure;      // hPa
    hazard_t turbulence; // NIL/LIGHT/MODERATE/SEVERE
    float humidity;      // 0-100 %

    // BDS 4,5 MHAR (Meteorological Hazard Report)
    unsigned mhar_turbulence_valid : 1;
    unsigned mhar_windshear_valid : 1;
    unsigned mhar_microburst_valid : 1;
    unsigned mhar_icing_valid : 1;
    unsigned mhar_wake_valid : 1;
    unsigned mhar_sat_valid : 1;
    unsigned mhar_asp_valid : 1;
    unsigned mhar_rh_valid : 1;

    hazard_t mhar_turbulence;
    hazard_t mhar_windshear;
    hazard_t mhar_microburst;
    hazard_t mhar_icing;
    hazard_t mhar_wake;
    float mhar_sat;    // degrees C
    float mhar_asp;    // hPa
    float mhar_rh;     // 0-100 %

    // BDS 3,0 ACAS RA (decoded fields)
    unsigned acas_ra_valid : 1;
    unsigned acas_ara;       // 14 bits: Active Resolution Advisories
    unsigned acas_rac;       // 4 bits: Resolution Advisory Complement
    unsigned acas_rat : 1;   // RA Terminated
    unsigned acas_mte : 1;   // Multiple Threat Encounter
    unsigned acas_tti;       // 2 bits: Threat Type Indicator
    unsigned acas_threat_id; // 26 bits: Threat Identity Data

    // BDS 4,1 - Next Waypoint Identifier
    unsigned waypoint_valid : 1;
    char waypoint_id[9]; // 8 chars + NUL

    // BDS 4,2 - Next Waypoint Position
    unsigned waypoint_lat_valid : 1;
    unsigned waypoint_lon_valid : 1;
    unsigned waypoint_alt_valid : 1;
    double waypoint_lat;
    double waypoint_lon;
    int waypoint_alt; // feet

    // BDS 4,3 - Next Waypoint Crossing Info
    unsigned waypoint_crossing_alt_valid : 1;
    unsigned waypoint_crossing_speed_valid : 1;
    int waypoint_crossing_alt; // feet
    unsigned waypoint_crossing_speed; // kts
};

#endif // DUMP1090_MESSAGE_H
