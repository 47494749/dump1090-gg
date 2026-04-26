// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sondehub_client.h: SondeHub telemetry upload client
//
// Uploads decoded radiosonde (RS41) telemetry to the SondeHub v2 API
// (https://api.v2.sondehub.org/sondes/telemetry) when a radiosonde
// SDR receiver is active. Uses HTTPS PUT with JSON payload.

#ifndef SONDEHUB_CLIENT_H
#define SONDEHUB_CLIENT_H

#include <stdbool.h>
#include "sonde_demod.h"

typedef struct {
    bool  enabled;
    char  callsign[64];       // uploader_callsign (required)
} sondehub_config_t;

extern sondehub_config_t SondehubConfig;

void sondehubClientInit(void);
void sondehubClientSubmit(const sonde_msg_t *msg);
void sondehubClientPeriodicWork(void);
void sondehubClientCleanup(void);

#endif // SONDEHUB_CLIENT_H
