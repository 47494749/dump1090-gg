// fr24_client.h: Stub header for dump1090-gg-light
// FlightRadar24 feeder removed — this is the open-source light version.

#ifndef FR24_CLIENT_H
#define FR24_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    int32_t      enabled;
    char     fr24key[20];
    char     config_path[256];
    char     server_host[256];
    int32_t      server_port;
    uint64_t radar_id;
    uint32_t feed_id;
    char     feed_alias[32];
    uint32_t session_key;
    int32_t      udp_mode;
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

int32_t fr24_load_config(const char *path);
void *fr24_thread_entry(void *arg);

#ifdef __cplusplus
}
#endif

#endif // FR24_CLIENT_H
