// Part of dump1090-gg, a Mode S message decoder.
//
// display_config.h: Display and interactive mode configuration

#ifndef DISPLAY_CONFIG_H
#define DISPLAY_CONFIG_H

#include <stdint.h>

typedef enum {
    UNIT_NAUTICAL = 0,
    UNIT_STATUTE,
    UNIT_METRIC
} distance_unit_t;

typedef struct {
    int         quiet;                  // suppress stdout
    int         interactive;            // interactive mode enabled
    uint64_t    interactive_ttl_ms;     // aircraft display TTL (ms)
    int         interactive_rows;       // max rows in interactive display
    int         show_distance;          // show distance/bearing instead of lat/lon
    distance_unit_t distance_units;     // units for distance display
    char        callsign_filter[64];    // filter interactive display by callsign
    uint32_t    show_only;              // only show this ICAO address (0 = all)
    int         onlyaddr;               // print only ICAO addresses
    int         tisb_verbose;           // log TIS-B/ADS-R messages to stderr
    int         metric;                 // use metric units
    int         raw;                    // raw output format
} display_config_t;

extern display_config_t DisplayConfig;

#endif // DISPLAY_CONFIG_H
