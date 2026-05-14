// Part of dump1090-gg, a Mode S message decoder.
//
// json_config.h: JSON output configuration

#ifndef JSON_CONFIG_H
#define JSON_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    char        dir[256];               // path to JSON output directory (empty = disabled)
    uint64_t    interval_ms;            // aircraft.json rewrite interval (ms)
    uint64_t    stats_interval_ms;      // stats.json rewrite interval (ms)
    int         location_accuracy;      // 0=none, 1=approx, 2=exact
} json_config_t;

extern json_config_t JsonConfig;

#ifdef __cplusplus
}
#endif

#endif // JSON_CONFIG_H
