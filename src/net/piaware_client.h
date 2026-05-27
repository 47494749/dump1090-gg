// SPDX-License-Identifier: GPL-3.0-or-later
//
// piaware_client.h: Built-in FlightAware ADEPT client for dump1090-gg
//
// Connects directly to FlightAware servers over TLS using the ADEPT protocol,
// replacing the need for the external piaware daemon + faup1090.

#ifndef PIAWARE_CLIENT_H
#define PIAWARE_CLIENT_H

#include <stdint.h>
#include <sys/types.h>
#include <string>

// Connection states
#define PA_DISCONNECTED    0
#define PA_CONNECTING      1
#define PA_TLS_HANDSHAKE   2
#define PA_AWAITING_LOGIN  3
#define PA_LOGGED_IN       4

// Defaults
#define PA_DEFAULT_HOST         "piaware.flightaware.com"
#define PA_DEFAULT_PORT         1200
#define PA_DEFAULT_CA_DIR       "/usr/lib/piaware_packages/ca"
#define PA_DEFAULT_FEEDER_FILE  "/var/cache/piaware/feeder_id"
#define PA_RECONNECT_MS         60000
#define PA_FAST_RECONNECT_MS    5000
#define PA_LOGIN_TIMEOUT_MS     15000
#define PA_ALIVE_TIMEOUT_MS     90000
#define PA_FATSV_INTERVAL_MS    1000
#define PA_HEALTH_INTERVAL_MS   300000

typedef struct {
    int32_t enabled;
    int32_t state;

    // Socket and TLS (void* to avoid OpenSSL includes in header)
    int32_t fd;
    void *ssl_ctx;
    void *ssl;

    // Config
    std::string host;
    int32_t  port;
    std::string feeder_id;
    std::string feeder_id_source;
    std::string mac;
    std::string ca_dir;
    std::string feeder_id_file;

    // Timers
    uint64_t next_reconnect;
    uint64_t login_deadline;
    uint64_t alive_deadline;
    uint64_t next_fatsv;
    uint64_t next_health;
    uint64_t reconnect_interval;

    // Input buffer
    char inbuf[8192];
    int32_t  inbuf_len;

    // Stats
    uint64_t msgs_sent;
} piaware_client_t;

extern piaware_client_t PiawareClient;

void piawareClientInit(void);
void piawareClientPeriodicWork(void);
void piawareClientCleanup(void);

#endif
