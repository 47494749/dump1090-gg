// radarbox_client.c: Stub — RadarBox feeder removed in dump1090-gg-light
#include "radarbox_client.h"
#include <stdint.h>
#include <cstring>

radarbox_client_t RadarBoxClient;

void radarboxClientInit(void) { }
void radarboxClientStart(void) { }
void radarboxClientPeriodicWork(void) { }
void radarboxClientStop(void) { }

int32_t radarboxLoadConfig(const char *path) {
    (void)path;
    return -1;
}

bool radarboxLoadKeys(const char *path) {
    (void)path;
    return false;
}

int32_t rb_keys_are_loaded(void) { return 0; }

void rb_get_key(uint8_t out[16]) {
    memset(out, 0, 16);
}

void rb_get_nonce(uint8_t out[8]) {
    memset(out, 0, 8);
}

uint32_t rb_get_c2(void) { return 0; }
