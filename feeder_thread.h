// SPDX-License-Identifier: GPL-3.0-or-later
//
// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// feeder_thread.h: threaded feeder architecture
//
// Each feeder (MLAT, PiAware, OGN, ADSBx) runs in its own pthread.
// The main decode loop dispatches decoded messages to per-feeder
// lock-free SPSC ring buffers. This prevents slow network I/O
// on any single feeder from blocking the decode pipeline.

#ifndef FEEDER_THREAD_H
#define FEEDER_THREAD_H

#include <pthread.h>
#include <stdatomic.h>

// ===================== Message Queue (SPSC ring buffer) =====================
// Single-Producer Single-Consumer lock-free queue for modesMessage.
// Producer: main thread (after decode). Consumer: feeder thread.

#define FEEDER_QUEUE_SIZE 4096  // must be power of 2

struct feeder_msg_queue {
    struct modesMessage msgs[FEEDER_QUEUE_SIZE];
    atomic_uint head;  // written by producer
    atomic_uint tail;  // written by consumer
};

static inline void feeder_queue_init(struct feeder_msg_queue *q) {
    atomic_store(&q->head, 0);
    atomic_store(&q->tail, 0);
}

// Returns 1 on success, 0 if full
static inline int feeder_queue_push(struct feeder_msg_queue *q, const struct modesMessage *mm) {
    unsigned h = atomic_load_explicit(&q->head, memory_order_relaxed);
    unsigned next = (h + 1) & (FEEDER_QUEUE_SIZE - 1);
    if (next == atomic_load_explicit(&q->tail, memory_order_acquire))
        return 0;  // full
    q->msgs[h] = *mm;
    atomic_store_explicit(&q->head, next, memory_order_release);
    return 1;
}

// Returns 1 on success (msg filled), 0 if empty
static inline int feeder_queue_pop(struct feeder_msg_queue *q, struct modesMessage *mm) {
    unsigned t = atomic_load_explicit(&q->tail, memory_order_relaxed);
    if (t == atomic_load_explicit(&q->head, memory_order_acquire))
        return 0;  // empty
    *mm = q->msgs[t];
    atomic_store_explicit(&q->tail, (t + 1) & (FEEDER_QUEUE_SIZE - 1), memory_order_release);
    return 1;
}

// ===================== Aircraft list mutex =====================
extern pthread_rwlock_t aircraft_lock;

// ===================== MLAT result injection queue =====================
extern struct feeder_msg_queue mlat_inject_queue;

// ===================== FA MLAT queue (main → FA MLAT thread) =====================
extern struct feeder_msg_queue fa_mlat_queue;

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
