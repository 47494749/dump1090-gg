// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// elm.c: Comm-D Extended Length Message (ELM) reassembly and decode
//
// Implements:
//   - Per-aircraft segment reassembly with hash table
//   - TTL-based cleanup of stale incomplete messages
//   - Separate decode thread for ACARS/CPDLC payload parsing
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>
#include "elm.h"
#include "cpdlc_decode.h"

// ========== Hash table helpers ==========

static inline unsigned elm_hash(uint32_t addr) {
    return (addr ^ (addr >> 8) ^ (addr >> 16)) & (ELM_TABLE_SIZE - 1);
}

static struct elm_entry *elm_find(struct elm_state *state, uint32_t addr) {
    unsigned h = elm_hash(addr);
    struct elm_entry *e = state->table[h];
    while (e) {
        if (e->addr == addr)
            return e;
        e = e->next;
    }
    return NULL;
}

static struct elm_entry *elm_create(struct elm_state *state, uint32_t addr, uint64_t timestamp) {
    struct elm_entry *e = calloc(1, sizeof(struct elm_entry));
    if (!e)
        return NULL;
    e->addr = addr;
    e->first_seen = timestamp;
    e->last_seen = timestamp;
    e->segments_mask = 0;

    unsigned h = elm_hash(addr);
    e->next = state->table[h];
    state->table[h] = e;
    return e;
}

static void elm_remove(struct elm_state *state, uint32_t addr) {
    unsigned h = elm_hash(addr);
    struct elm_entry **pp = &state->table[h];
    while (*pp) {
        if ((*pp)->addr == addr) {
            struct elm_entry *e = *pp;
            *pp = e->next;
            free(e);
            return;
        }
        pp = &(*pp)->next;
    }
}

// ========== ACARS decode helpers ==========

// ACARS-over-Mode-S framing:
// The payload may contain ACARS messages with standard framing:
//   SOH (0x01) or prekey, then label (2 chars), then text ending with ETX (0x03) or ETB (0x17)
// We also look for plain ASCII text.

static void elm_decode_acars(uint32_t addr, const unsigned char *payload, int len) {
    // Look for ACARS framing
    int i;
    int found_acars = 0;

    for (i = 0; i < len; i++) {
        if (payload[i] == 0x01) { // SOH - start of ACARS header
            found_acars = 1;

            // Find end of message (ETX=0x03 or ETB=0x17)
            int end = i + 1;
            while (end < len && payload[end] != 0x03 && payload[end] != 0x17)
                end++;

            // Extract the ACARS content between SOH and ETX/ETB
            int msg_start = i + 1;
            int msg_len = end - msg_start;

            if (msg_len > 2) {
                // ACARS header: mode(1) addr(7) ack(1) label(2) blockid(1) ...
                // Print what we can extract
                printf("ELM ACARS %06X: ", addr);

                // Print mode character if printable
                if (msg_len > 0 && isprint(payload[msg_start]))
                    printf("mode=%c ", payload[msg_start]);

                // Print the address field (up to 7 chars)
                if (msg_len > 8) {
                    printf("reg=");
                    for (int j = 1; j < 8 && (msg_start + j) < len; j++) {
                        if (isprint(payload[msg_start + j]))
                            putchar(payload[msg_start + j]);
                    }
                    printf(" ");
                }

                // Print label (2 chars after address+ack)
                if (msg_len > 10) {
                    printf("label=");
                    if (isprint(payload[msg_start + 9]))
                        putchar(payload[msg_start + 9]);
                    if (isprint(payload[msg_start + 10]))
                        putchar(payload[msg_start + 10]);
                    printf(" ");
                }

                // Print STX onwards as text (the actual message body)
                int stx_pos = -1;
                for (int j = msg_start; j < end; j++) {
                    if (payload[j] == 0x02) { // STX
                        stx_pos = j + 1;
                        break;
                    }
                }

                if (stx_pos >= 0 && stx_pos < end) {
                    printf("text=\"");
                    for (int j = stx_pos; j < end; j++) {
                        unsigned char c = payload[j];
                        if (c >= 0x20 && c < 0x7f)
                            putchar(c);
                        else if (c == 0x0a || c == 0x0d)
                            putchar(' ');
                        else
                            printf("\\x%02X", c);
                    }
                    printf("\"");
                }

                printf("\n");
            }

            i = end; // skip past this message
        }
    }

    if (!found_acars) {
        // Try CPDLC decode first
        if (cpdlc_try_decode(addr, payload, len)) {
            fflush(stdout);
            return;
        }

        // No ACARS or CPDLC. Output raw payload as hex + printable ASCII.

        int has_printable = 0;
        for (i = 0; i < len; i++) {
            if (isprint(payload[i]) && payload[i] != ' ')
                has_printable++;
        }

        printf("ELM %06X [%d bytes]: ", addr, len);

        // Hex dump
        for (i = 0; i < len && i < 40; i++)
            printf("%02X", payload[i]);
        if (len > 40)
            printf("...");

        // ASCII if there's meaningful text
        if (has_printable > len / 4) {
            printf(" |");
            for (i = 0; i < len; i++) {
                unsigned char c = payload[i];
                putchar((c >= 0x20 && c < 0x7f) ? c : '.');
            }
            printf("|");
        }

        printf("\n");
    }

    fflush(stdout);
}

// ========== Decode thread ==========

static void *elm_decode_worker(void *arg) {
    struct elm_state *state = (struct elm_state *)arg;

    while (state->decode_thread_running) {
        struct elm_complete *msg = NULL;

        // Wait for a message in the queue
        pthread_mutex_lock(&state->queue_mutex);
        while (!state->queue_head && state->decode_thread_running) {
            pthread_cond_wait(&state->queue_cond, &state->queue_mutex);
        }

        if (state->queue_head) {
            msg = state->queue_head;
            state->queue_head = msg->next;
            if (!state->queue_head)
                state->queue_tail = NULL;
            state->queue_count--;
        }
        pthread_mutex_unlock(&state->queue_mutex);

        if (msg) {
            elm_decode_acars(msg->addr, msg->payload, msg->payload_len);
            free(msg);
        }
    }

    // Drain remaining queue on shutdown
    pthread_mutex_lock(&state->queue_mutex);
    while (state->queue_head) {
        struct elm_complete *msg = state->queue_head;
        state->queue_head = msg->next;
        free(msg);
    }
    pthread_mutex_unlock(&state->queue_mutex);

    return NULL;
}

// ========== Queue a complete message for decode ==========

static void elm_queue_complete(struct elm_state *state, struct elm_entry *entry) {
    // Find the highest consecutive segment from 0
    int max_seg = -1;
    for (int i = 0; i < ELM_MAX_SEGMENTS; i++) {
        if (entry->segments_mask & (1 << i))
            max_seg = i;
        else
            break;
    }

    if (max_seg < 0)
        return; // nothing usable

    int payload_len = (max_seg + 1) * ELM_SEGMENT_SIZE;

    struct elm_complete *msg = malloc(sizeof(struct elm_complete));
    if (!msg)
        return;

    msg->addr = entry->addr;
    msg->payload_len = payload_len;
    msg->timestamp = entry->last_seen;
    msg->next = NULL;

    // Assemble payload from segments in order
    for (int i = 0; i <= max_seg; i++) {
        memcpy(msg->payload + i * ELM_SEGMENT_SIZE, entry->data[i], ELM_SEGMENT_SIZE);
    }

    // Push to decode queue
    pthread_mutex_lock(&state->queue_mutex);
    if (state->queue_count >= ELM_DECODE_QUEUE) {
        // Queue full, drop oldest
        struct elm_complete *old = state->queue_head;
        if (old) {
            state->queue_head = old->next;
            if (!state->queue_head)
                state->queue_tail = NULL;
            state->queue_count--;
            free(old);
        }
    }

    if (state->queue_tail)
        state->queue_tail->next = msg;
    else
        state->queue_head = msg;
    state->queue_tail = msg;
    state->queue_count++;
    pthread_cond_signal(&state->queue_cond);
    pthread_mutex_unlock(&state->queue_mutex);
}

// ========== Public API ==========

void elmInit(struct elm_state *state) {
    memset(state, 0, sizeof(*state));
    pthread_mutex_init(&state->queue_mutex, NULL);
    pthread_cond_init(&state->queue_cond, NULL);

    state->decode_thread_running = 1;
    pthread_create(&state->decode_thread, NULL, elm_decode_worker, state);
}

void elmCleanup(struct elm_state *state) {
    // Signal decode thread to stop
    state->decode_thread_running = 0;
    pthread_cond_signal(&state->queue_cond);
    pthread_join(state->decode_thread, NULL);

    pthread_mutex_destroy(&state->queue_mutex);
    pthread_cond_destroy(&state->queue_cond);

    // Free all reassembly entries
    for (int i = 0; i < ELM_TABLE_SIZE; i++) {
        struct elm_entry *e = state->table[i];
        while (e) {
            struct elm_entry *next = e->next;
            free(e);
            e = next;
        }
        state->table[i] = NULL;
    }
}

void elmAddSegment(struct elm_state *state, uint32_t addr, unsigned nd,
                   unsigned ke, const unsigned char *md, uint64_t timestamp) {
    if (nd >= ELM_MAX_SEGMENTS)
        return;

    struct elm_entry *entry = elm_find(state, addr);

    if (!entry) {
        entry = elm_create(state, addr, timestamp);
        if (!entry)
            return;
        state->active_entries++;
    }

    // Store the segment
    int is_new = !(entry->segments_mask & (1 << nd));
    memcpy(entry->data[nd], md, ELM_SEGMENT_SIZE);
    entry->segments_mask |= (1 << nd);
    entry->last_seen = timestamp;
    state->segments_received++;

    // Log new segment arrival immediately
    if (is_new) {
        int segs_have = 0;
        for (int i = 0; i < ELM_MAX_SEGMENTS; i++)
            if (entry->segments_mask & (1 << i))
                segs_have++;
        fprintf(stderr, "ELM seg %06X: ND=%u KE=%u [%d/16] ", addr, nd, ke, segs_have);
        for (int b = 0; b < ELM_SEGMENT_SIZE; b++)
            fprintf(stderr, "%02X", md[b]);
        fprintf(stderr, "\n");
        fflush(stderr);
    }

    // Check for completion:
    // 1. We have segment 0 (required)
    // 2. We have consecutive segments from 0..N
    // 3. Either KE=1 (close-out) or we have a gap after the last received
    if (!(entry->segments_mask & 1))
        return; // no segment 0 yet

    int consecutive = 0;
    for (int i = 0; i < ELM_MAX_SEGMENTS; i++) {
        if (entry->segments_mask & (1 << i))
            consecutive = i + 1;
        else
            break;
    }

    // Complete if: KE=1 or we have at least 2 consecutive segments and
    // a gap (meaning the aircraft has moved on) or we've received segment 15
    int complete = 0;
    if (ke == 1 && consecutive >= 1) {
        complete = 1;
    } else if (consecutive >= 2 && consecutive == ELM_MAX_SEGMENTS) {
        complete = 1; // all 16 segments
    } else if (consecutive >= 2) {
        // Check if there's a gap after our consecutive run — this means
        // we likely have everything before the gap
        int has_later = 0;
        for (int i = consecutive; i < ELM_MAX_SEGMENTS; i++) {
            if (entry->segments_mask & (1 << i)) {
                has_later = 1;
                break;
            }
        }
        // If we received a segment beyond our consecutive run, that's probably
        // a new message, so flush the old one
        if (has_later)
            complete = 1;
    }

    if (complete) {
        elm_queue_complete(state, entry);
        elm_remove(state, addr);
        state->messages_completed++;
        state->active_entries--;
    }
}

void elmCleanupStale(struct elm_state *state, uint64_t now) {
    for (int i = 0; i < ELM_TABLE_SIZE; i++) {
        struct elm_entry **pp = &state->table[i];
        while (*pp) {
            struct elm_entry *e = *pp;
            if (now - e->last_seen > ELM_TTL_MS) {
                // TTL expired. If we have any data, try to decode what we have.
                if (e->segments_mask & 1) {
                    elm_queue_complete(state, e);
                    state->messages_completed++;
                }
                state->messages_expired++;
                state->active_entries--;
                *pp = e->next;
                free(e);
            } else {
                pp = &(*pp)->next;
            }
        }
    }

    // Print stats every 5 minutes
    #define ELM_STATS_INTERVAL_MS  300000
    if (state->segments_received > 0 &&
        (state->last_stats_time == 0 || now - state->last_stats_time >= ELM_STATS_INTERVAL_MS)) {
        fprintf(stderr, "ELM stats: %" PRIu64 " segments, %" PRIu64 " complete, %" PRIu64 " expired, %d active\n",
                state->segments_received, state->messages_completed,
                state->messages_expired, state->active_entries);
        state->last_stats_time = now;
    }

    // Print partial reassembly map every 60 seconds
    elmPrintPartial(state, now);
}

void elmPrintPartial(struct elm_state *state, uint64_t now) {
    #define ELM_PARTIAL_INTERVAL_MS  60000

    if (state->active_entries == 0)
        return;
    if (state->last_partial_time != 0 && now - state->last_partial_time < ELM_PARTIAL_INTERVAL_MS)
        return;

    state->last_partial_time = now;

    for (int i = 0; i < ELM_TABLE_SIZE; i++) {
        struct elm_entry *e = state->table[i];
        while (e) {
            int segs_have = 0;
            for (int s = 0; s < ELM_MAX_SEGMENTS; s++)
                if (e->segments_mask & (1 << s))
                    segs_have++;

            double age_sec = (now - e->first_seen) / 1000.0;

            fprintf(stderr, "ELM partial %06X [%d/16 seg, %.1fs]: ",
                    e->addr, segs_have, age_sec);

            // Print segment map: hex for received, ".." for missing
            for (int s = 0; s < ELM_MAX_SEGMENTS; s++) {
                if (e->segments_mask & (1 << s)) {
                    for (int b = 0; b < ELM_SEGMENT_SIZE; b++)
                        fprintf(stderr, "%02X", e->data[s][b]);
                } else {
                    // 10 bytes missing = 20 dots
                    fprintf(stderr, "....................");
                }
                if (s < ELM_MAX_SEGMENTS - 1)
                    fprintf(stderr, "|");
            }

            // Also print ASCII view of received segments
            fprintf(stderr, " |");
            for (int s = 0; s < ELM_MAX_SEGMENTS; s++) {
                if (e->segments_mask & (1 << s)) {
                    for (int b = 0; b < ELM_SEGMENT_SIZE; b++) {
                        unsigned char c = e->data[s][b];
                        fputc((c >= 0x20 && c < 0x7f) ? c : '.', stderr);
                    }
                } else {
                    fprintf(stderr, "          ");
                }
            }
            fprintf(stderr, "|\n");

            e = e->next;
        }
    }
    fflush(stderr);
}
