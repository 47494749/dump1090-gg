// radarbox_client.h: Stub header for dump1090-gg-light
// RadarBox feeder removed — this is the open-source light version.

#ifndef RADARBOX_CLIENT_H
#define RADARBOX_CLIENT_H

#include <stdint.h>


typedef enum {
    RB_STATE_DISCONNECTED = 0,
    RB_STATE_CONNECTING,
    RB_STATE_AUTH_SENT,
    RB_STATE_RUNNING,
    RB_STATE_FAILED
} rb_state_t;

typedef struct {
    int32_t      enabled;
    char     sharing_key[64];
    char     serial_number[32];
    char     config_path[256];
    char     keys_file[256];
    rb_state_t state;
    int32_t      fd;
    uint64_t next_reconnect;
    uint64_t last_stats;
    uint64_t last_data;
    uint64_t last_recv;
    uint64_t last_keepalive;
    uint64_t connect_time;
    uint64_t frames_sent;
    uint64_t frames_recv;
    uint64_t aircraft_sent;
    uint64_t commands_received;
    uint8_t  recv_buf[4098];
    int32_t      recv_buf_len;
} radarbox_client_t;

extern radarbox_client_t RadarBoxClient;

void radarboxClientInit(void);
void radarboxClientStart(void);
void radarboxClientPeriodicWork(void);
void radarboxClientStop(void);
int32_t radarboxLoadConfig(const char *path);
bool radarboxLoadKeys(const char *path);
int32_t rb_keys_are_loaded(void);
void rb_get_key(uint8_t out[16]);
void rb_get_nonce(uint8_t out[8]);
uint32_t rb_get_c2(void);

#endif // RADARBOX_CLIENT_H
