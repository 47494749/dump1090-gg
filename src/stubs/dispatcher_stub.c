// dispatcher_stub.c: stub for view1090/faup1090 which link net_io.o
// but don't use the dispatcher infrastructure.

#include "dispatcher.h"
#include <stddef.h>

int dispatcher_push_adsb(adsb_queue_handle_t q, const struct modesMessage *mm) {
    (void)q; (void)mm;
    return 0;
}

void dispatcher_poll(void) { }
void dispatcher_init(void) { }
aircraft_queue_handle_t dispatcher_register_aircraft_queue(const char *name) { (void)name; return NULL; }
adsb_queue_handle_t dispatcher_register_adsb_queue(const char *name) { (void)name; return NULL; }
