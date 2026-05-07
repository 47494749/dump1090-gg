// fr24_client.h: Stub header for dump1090-gg-light
// FlightRadar24 feeder removed — this is the open-source light version.

#ifndef FR24_CLIENT_H
#define FR24_CLIENT_H

#include <stdint.h>

typedef struct {
    int      enabled;
    char     fr24key[20];
    char     config_path[256];
    char     server_host[256];
    int      server_port;
    uint64_t radar_id;
    uint32_t feed_id;
    char     feed_alias[32];
    uint32_t session_key;
    int      udp_mode;
    uint32_t server_value_1;
    uint32_t server_value_2;
    uint64_t last_send;
    uint64_t last_keepalive;
    uint64_t packets_sent;
    uint64_t bytes_sent;
    uint64_t aircraft_sent;
    uint64_t connects;
    uint64_t connect_failures;
    uint64_t start_time;
} fr24_client_t;

extern fr24_client_t FR24Client;

int fr24_load_config(const char *path);
void *fr24_thread_entry(void *arg);

#endif // FR24_CLIENT_H
