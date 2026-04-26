// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// opensky_client.h: OpenSky Network feeder client (native protocol)

#ifndef OPENSKY_CLIENT_H
#define OPENSKY_CLIENT_H

#include <stdint.h>
#include "feeder_thread.h"

struct opensky_config {
    int      enabled;
    char     host[128];
    int      port;
    char     username[64];
    int32_t  serial;           // persistent serial from server, 0 = request new
    char     serial_file[256]; // path to persist serial number
    double   lat;
    double   lon;
    double   alt;              // meters
};

extern struct opensky_config OpenSkyConfig;

extern struct feeder_msg_queue opensky_queue;

void openskyClientInit(void);
void *opensky_thread_entry(void *arg);

#endif // OPENSKY_CLIENT_H
