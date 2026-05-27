// SPDX-License-Identifier: GPL-3.0-or-later
//
// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// feeder_thread.h: threaded feeder architecture
//
// Each feeder (MLAT, PiAware, OGN, ADSBx) runs in its own pthread.
// The main decode loop dispatches decoded messages to per-feeder
// thread-safe queues. This prevents slow network I/O on any single
// feeder from blocking the decode pipeline.

#ifndef FEEDER_THREAD_H
#define FEEDER_THREAD_H

#include <stdint.h>

#include <pthread.h>
#include <atomic>
using std::atomic_int;
using std::atomic_uint;

#include "msg_queue.h"

// ===================== Aircraft list mutex =====================
extern pthread_rwlock_t aircraft_lock;

// ===================== MLAT result injection queue =====================
extern msg_queue_t mlat_inject_queue;

// ===================== FA MLAT queue (main → FA MLAT thread) =====================
extern msg_queue_t fa_mlat_queue;

// Process any MLAT-injected messages (call from main thread / backgroundTasks)
void feederProcessInjectedMessages(void);

// ===================== Feeder stop flag =====================
extern atomic_int feeders_running;

// ===================== Internet availability flag =====================
// Set by beast_feed_thread's internet watchdog. Other feeder threads
// check this to stop buffering real-time data while offline.
extern atomic_int net_available;

// ===================== Feeder thread API =====================

void feederThreadsStart(void);
void feederDispatchMessage(struct modesMessage *mm);
void feederThreadsStop(void);

#endif // FEEDER_THREAD_H
