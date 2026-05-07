// planefinder_client.h: Stub header for dump1090-gg-light
// PlaneFinder feeder removed — this is the open-source light version.

#ifndef PLANEFINDER_CLIENT_H
#define PLANEFINDER_CLIENT_H

#include <stdint.h>

typedef struct {
    int     enabled;
    char    sharecode[64];
    int     upload_interval;
    int     aircraft_timeout;
    uint64_t last_upload;
    uint64_t uploads_ok;
    uint64_t uploads_failed;
    uint64_t uploads_failed_consecutive;
    uint64_t aircraft_sent;
    uint64_t start_time;
} planefinder_client_t;

extern planefinder_client_t PlaneFinderClient;

void *planefinder_thread_entry(void *arg);

#endif // PLANEFINDER_CLIENT_H
