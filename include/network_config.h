// Part of dump1090-gg, a Mode S message decoder.
//
// network_config.h: Network configuration (ports, bindings, feeds)

#ifndef NETWORK_CONFIG_H
#define NETWORK_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define MAX_BEAST_FEEDS  16

typedef struct {
    char  name[32];
    char  host[256];
    int   port;
    int   format;       // 0=beast, 1=raw, 2=sbs
    int   enabled;
} beast_feed_config_t;

typedef struct {
    // Port configuration
    char    raw_out_ports[128];
    char    raw_in_ports[128];
    char    sbs_out_ports[128];
    char    stratux_out_ports[128];
    char    beast_out_ports[128];
    char    beast_in_ports[128];
    char    bind_address[128];

    // Buffer/flush
    int     sndbuf_size;
    int     output_flush_size;
    uint64_t output_flush_interval_ms;
    uint64_t heartbeat_interval_ms;

    // Options
    int     verbatim;
    int     forward_mlat;
    int     net_enabled;
    int     net_only;

    // Beast feed hubs
    beast_feed_config_t feeds[MAX_BEAST_FEEDS];
    int                 feed_count;

    // ADSBHub
    char    adsbhub_ckey[128];
} network_config_t;

extern network_config_t NetworkConfig;

#ifdef __cplusplus
}
#endif

#endif // NETWORK_CONFIG_H
