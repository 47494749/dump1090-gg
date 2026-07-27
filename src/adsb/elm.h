// SPDX-License-Identifier: GPL-3.0-or-later
//
// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// elm.h: Comm-D Extended Length Message (ELM) reassembly and decode
//

#ifndef ELM_H
#define ELM_H

#include <stdint.h>
#include <pthread.h>

// ELM constants
#define ELM_MAX_SEGMENTS   16
#define ELM_SEGMENT_SIZE   10
#define ELM_MAX_PAYLOAD    (ELM_MAX_SEGMENTS * ELM_SEGMENT_SIZE)  // 160 bytes
#define ELM_TTL_MS         60000   // 60 seconds TTL for incomplete messages
#define ELM_TABLE_SIZE     256     // hash table size
#define ELM_DECODE_QUEUE   64      // max queued complete messages
#define ELM_MIN_SEGMENTS   2       // Require at least 2 segments for reliable decode
#define ELM_SEG_GAP_MS     8000    // max ms between consecutive segments (real ELM < 5s)

// Reassembly entry (one per aircraft with active ELM)
struct elm_entry {
    uint32_t addr;                 // ICAO address
    uint64_t first_seen;           // sysTimestamp of first segment
    uint64_t last_seen;            // sysTimestamp of last segment
    uint16_t segments_mask;        // bitmask of received segments (bit N = ND N)
    uint8_t data[ELM_MAX_SEGMENTS][ELM_SEGMENT_SIZE];
    uint64_t seg_time[ELM_MAX_SEGMENTS]; // timestamp per segment
    struct elm_entry *next;        // hash chain
};

// Complete reassembled message, queued for decode thread
struct elm_complete {
    uint32_t addr;
    int32_t payload_len;               // actual bytes (segments * 10)
    int32_t segments_received;         // number of consecutive segments from 0
    int32_t complete_ke;               // 1 if all 16 segments received (= 100% complete)
    uint8_t payload[ELM_MAX_PAYLOAD];
    uint64_t timestamp;
    struct elm_complete *next;
};

// Global ELM state
struct elm_state {
    // Reassembly hash table (accessed only from main thread, no lock needed)
    struct elm_entry *table[ELM_TABLE_SIZE];

    // Decode queue (producer: main thread, consumer: decode thread)
    struct elm_complete *queue_head;
    struct elm_complete *queue_tail;
    int32_t queue_count;
    pthread_mutex_t queue_mutex;
    pthread_cond_t queue_cond;

    // Decode thread
    pthread_t decode_thread;
    int32_t decode_thread_running;

    // Stats
    uint64_t segments_received;    // total DF24 segments received
    uint64_t messages_completed;   // complete ELMs queued for decode
    uint64_t messages_expired;     // partial ELMs flushed by TTL
    uint64_t messages_rejected;    // rejected by content validation
    int32_t active_entries;            // current reassembly entries in table
    uint64_t last_stats_time;      // timestamp of last stats print
    uint64_t last_partial_time;    // timestamp of last partial print
};

// Public API
void elmInit(struct elm_state *state);
void elmCleanup(struct elm_state *state);
void elmAddSegment(struct elm_state *state, uint32_t addr, uint32_t nd,
                   uint32_t ke, const uint8_t *md, uint64_t timestamp);
void elmCleanupStale(struct elm_state *state, uint64_t now);
void elmPrintPartial(struct elm_state *state, uint64_t now);

#endif
