// feeder_thread_stub.c: stub implementations for view1090/faup1090
// These programs don't need feeder threads but link against mode_s.o
// and mlat_client.o which reference feeder symbols.

#include "dump1090.h"

pthread_rwlock_t aircraft_lock = PTHREAD_RWLOCK_INITIALIZER;
struct feeder_msg_queue mlat_inject_queue;
struct feeder_msg_queue fa_mlat_queue;
atomic_int feeders_running;

void feederDispatchMessage(struct modesMessage *mm) {
    (void)mm;
}

void feederProcessInjectedMessages(void) {
}

void radarboxClientInit(void) {
}
