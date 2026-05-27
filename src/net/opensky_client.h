// SPDX-License-Identifier: GPL-3.0-or-later
//
// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// opensky_client.h: OpenSky Network feeder client (native protocol)

#ifndef OPENSKY_CLIENT_H
#define OPENSKY_CLIENT_H

#include <stdint.h>
#include <string>
#include "msg_queue.h"

struct opensky_config {
    int32_t      enabled;
    std::string  host;
    int32_t      port;
    std::string  username;
    int32_t  serial;           // persistent serial from server, 0 = request new
    std::string  serial_file;  // path to persist serial number
    double   lat;
    double   lon;
    double   alt;              // meters
};

extern struct opensky_config OpenSkyConfig;

extern msg_queue_t opensky_queue;

void openskyClientInit(void);
void *opensky_thread_entry(void *arg);

#endif // OPENSKY_CLIENT_H
