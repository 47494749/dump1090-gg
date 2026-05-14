// Part of dump1090-gg, a Mode S message decoder.
//
// station_config.h: Station identity and position (single source of truth)

#ifndef STATION_CONFIG_H
#define STATION_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char    name[64];           // station name (used for OGN, MLAT, SondeHub)
    double  latitude;           // receiver latitude (degrees)
    double  longitude;          // receiver longitude (degrees)
    float   altitude_m;         // receiver altitude above sea level (metres)
    double  max_range_m;        // maximum decode range (metres)
} station_config_t;

extern station_config_t StationConfig;

#ifdef __cplusplus
}
#endif

#endif // STATION_CONFIG_H
