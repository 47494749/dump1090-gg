// fr24_client.c: Stub — FlightRadar24 feeder removed in dump1090-gg-light
#include "fr24_client.h"
#include <stdint.h>
#include <string.h>

fr24_client_t FR24Client;

int32_t fr24_load_config(const char *path) {
    (void)path;
    return -1;
}

void *fr24_thread_entry(void *arg) {
    (void)arg;
    return NULL;
}
