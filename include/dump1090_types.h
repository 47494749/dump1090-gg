// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// dump1090_types.h: shared type definitions (enums)
//

#ifndef DUMP1090_TYPES_H
#define DUMP1090_TYPES_H

/* Where did a bit of data arrive from? In order of increasing priority */
typedef enum {
    SOURCE_INVALID,        /* data is not valid */
    SOURCE_MODE_AC,        /* A/C message */
    SOURCE_MLAT,           /* derived from mlat */
    SOURCE_MODE_S,         /* data from a Mode S message, no full CRC */
    SOURCE_MODE_S_CHECKED, /* data from a Mode S message with full CRC */
    SOURCE_TISB,           /* data from a TIS-B extended squitter message */
    SOURCE_ADSR,           /* data from a ADS-R extended squitter message */
    SOURCE_ADSB,           /* data from a ADS-B extended squitter message */
} datasource_t;

/* What sort of address is this and who sent it?
 * (Earlier values are higher priority)
 */
typedef enum {
    ADDR_ADSB_ICAO,       /* Mode S or ADS-B, ICAO address, transponder sourced */
    ADDR_ADSB_ICAO_NT,    /* ADS-B, ICAO address, non-transponder */
    ADDR_ADSR_ICAO,       /* ADS-R, ICAO address */
    ADDR_TISB_ICAO,       /* TIS-B, ICAO address */

    ADDR_ADSB_OTHER,      /* ADS-B, other address format */
    ADDR_ADSR_OTHER,      /* ADS-R, other address format */
    ADDR_TISB_TRACKFILE,  /* TIS-B, Mode A code + track file number */
    ADDR_TISB_OTHER,      /* TIS-B, other address format */

    ADDR_MODE_A,          /* Mode A */

    ADDR_UNKNOWN          /* unknown address format */
} addrtype_t;

typedef enum {
    UNIT_FEET,
    UNIT_METERS
} altitude_unit_t;

typedef enum {
    UNIT_NAUTICAL_MILES,
    UNIT_STATUTE_MILES,
    UNIT_KILOMETERS,
} interactive_distance_unit_t;

typedef enum {
    ALTITUDE_BARO,
    ALTITUDE_GEOM
} altitude_source_t;

typedef enum {
    AG_INVALID,
    AG_GROUND,
    AG_AIRBORNE,
    AG_UNCERTAIN
} airground_t;

typedef enum {
    SIL_INVALID, SIL_UNKNOWN, SIL_PER_SAMPLE, SIL_PER_HOUR
} sil_type_t;

typedef enum {
    CPR_SURFACE, CPR_AIRBORNE, CPR_COARSE
} cpr_type_t;

typedef enum {
    HEADING_INVALID,          // Not set
    HEADING_GROUND_TRACK,     // Direction of track over ground, degrees clockwise from true north
    HEADING_TRUE,             // Heading, degrees clockwise from true north
    HEADING_MAGNETIC,         // Heading, degrees clockwise from magnetic north
    HEADING_MAGNETIC_OR_TRUE, // HEADING_MAGNETIC or HEADING_TRUE depending on the HRD bit in opstatus
    HEADING_TRACK_OR_HEADING  // GROUND_TRACK / MAGNETIC / TRUE depending on the TAH bit in opstatus
} heading_type_t;

typedef enum {
    COMMB_UNKNOWN,
    COMMB_AMBIGUOUS,
    COMMB_NOT_DECODED,
    COMMB_EMPTY_RESPONSE,
    COMMB_DATALINK_CAPS,
    COMMB_GICB_CAPS,
    COMMB_AIRCRAFT_IDENT,
    COMMB_ACAS_RA,
    COMMB_VERTICAL_INTENT,
    COMMB_TRACK_TURN,
    COMMB_HEADING_SPEED,
    COMMB_MRAR,
    COMMB_MHAR,
    COMMB_WAYPOINT_ID,
    COMMB_WAYPOINT_POS,
    COMMB_WAYPOINT_INFO,
    COMMB_AIRBORNE_POSITION
} commb_format_t;

typedef enum {
    NAV_MODE_AUTOPILOT = 1,
    NAV_MODE_VNAV = 2,
    NAV_MODE_ALT_HOLD = 4,
    NAV_MODE_APPROACH = 8,
    NAV_MODE_LNAV = 16,
    NAV_MODE_TCAS = 32
} nav_modes_t;

// Matches encoding of the ES type 28/1 emergency/priority status subfield
typedef enum {
    EMERGENCY_NONE = 0,
    EMERGENCY_GENERAL = 1,
    EMERGENCY_LIFEGUARD = 2,
    EMERGENCY_MINFUEL = 3,
    EMERGENCY_NORDO = 4,
    EMERGENCY_UNLAWFUL = 5,
    EMERGENCY_DOWNED = 6,
    EMERGENCY_RESERVED = 7
} emergency_t;

typedef enum {
    NAV_ALT_INVALID, NAV_ALT_UNKNOWN, NAV_ALT_AIRCRAFT, NAV_ALT_MCP, NAV_ALT_FMS
} nav_altitude_source_t;

// BDS4,4 MRAR - FOM/Source values
typedef enum {
   MRAR_SOURCE_INVALID = 0,
   MRAR_SOURCE_INS = 1,
   MRAR_SOURCE_GNSS = 2,
   MRAR_SOURCE_DMEDME = 3,
   MRAR_SOURCE_VORDME = 4,
   MRAR_SOURCE_RESERVED = 5
} mrar_source_t;

// BDS4,4 and BDS4,5 hazard reports
typedef enum {
   HAZARD_NIL = 0,
   HAZARD_LIGHT = 1,
   HAZARD_MODERATE = 2,
   HAZARD_SEVERE = 3
} hazard_t;

typedef enum {
    SDR_NONE, SDR_IFILE, SDR_RTLSDR, SDR_BLADERF, SDR_HACKRF, SDR_LIMESDR, SDR_SOAPYSDR
} sdr_type_t;

#endif // DUMP1090_TYPES_H
