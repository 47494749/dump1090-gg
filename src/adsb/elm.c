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
#include "config_panel.h"
#include "sdr_receiver.h"

// ========== Hash table helpers ==========

static inline uint32_t elm_hash(uint32_t addr) {
    return (addr ^ (addr >> 8) ^ (addr >> 16)) & (ELM_TABLE_SIZE - 1);
}

static struct elm_entry *elm_find(struct elm_state *state, uint32_t addr) {
    uint32_t h = elm_hash(addr);
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

    uint32_t h = elm_hash(addr);
    e->next = state->table[h];
    state->table[h] = e;
    return e;
}

static void elm_remove(struct elm_state *state, uint32_t addr) {
    uint32_t h = elm_hash(addr);
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

static void elm_decode_acars(uint32_t addr, const uint8_t *payload, int len,
                             char *outbuf, int outbuf_size) {
    // Look for ACARS framing
    int i;
    int found_acars = 0;
    int n = 0;  // output buffer position

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
                n += snprintf(outbuf + n, outbuf_size - n, "ELM ACARS %06X: ", addr);

                // Print mode character if printable
                if (msg_len > 0 && isprint(payload[msg_start]))
                    n += snprintf(outbuf + n, outbuf_size - n, "mode=%c ", payload[msg_start]);

                // Print the address field (up to 7 chars)
                if (msg_len > 8) {
                    n += snprintf(outbuf + n, outbuf_size - n, "reg=");
                    for (int j = 1; j < 8 && (msg_start + j) < len; j++) {
                        if (isprint(payload[msg_start + j]) && n < outbuf_size - 1)
                            outbuf[n++] = payload[msg_start + j];
                    }
                    if (n < outbuf_size - 1) outbuf[n++] = ' ';
                    outbuf[n] = '\0';
                }

                // Print label (2 chars after address+ack)
                if (msg_len > 10) {
                    n += snprintf(outbuf + n, outbuf_size - n, "label=");
                    if (isprint(payload[msg_start + 9]) && n < outbuf_size - 1)
                        outbuf[n++] = payload[msg_start + 9];
                    if (isprint(payload[msg_start + 10]) && n < outbuf_size - 1)
                        outbuf[n++] = payload[msg_start + 10];
                    if (n < outbuf_size - 1) outbuf[n++] = ' ';
                    outbuf[n] = '\0';
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
                    n += snprintf(outbuf + n, outbuf_size - n, "text=\"");
                    for (int j = stx_pos; j < end && n < outbuf_size - 4; j++) {
                        uint8_t c = payload[j];
                        if (c >= 0x20 && c < 0x7f)
                            outbuf[n++] = c;
                        else if (c == 0x0a || c == 0x0d)
                            outbuf[n++] = ' ';
                        else
                            n += snprintf(outbuf + n, outbuf_size - n, "\\x%02X", c);
                    }
                    if (n < outbuf_size - 1) outbuf[n++] = '"';
                    outbuf[n] = '\0';
                }
            }

            i = end; // skip past this message
        }
    }

    if (!found_acars) {
        // Try CPDLC decode â€” capture output via open_memstream
        char *cpdlc_buf = NULL;
        size_t cpdlc_len = 0;
        FILE *old_stdout = stdout;
        FILE *mem = open_memstream(&cpdlc_buf, &cpdlc_len);
        if (mem) {
            stdout = mem;
            int decoded = cpdlc_try_decode(addr, payload, len);
            fflush(mem);
            stdout = old_stdout;
            fclose(mem);
            if (decoded && cpdlc_buf && cpdlc_len > 0) {
                // Remove trailing newline
                if (cpdlc_len > 0 && cpdlc_buf[cpdlc_len - 1] == '\n')
                    cpdlc_buf[cpdlc_len - 1] = '\0';
                n += snprintf(outbuf + n, outbuf_size - n, "%s", cpdlc_buf);
                free(cpdlc_buf);
                return;
            }
            free(cpdlc_buf);
        } else {
            // Fallback: just call directly (output goes to stdout)
            if (cpdlc_try_decode(addr, payload, len)) {
                fflush(stdout);
                snprintf(outbuf, outbuf_size, "CPDLC %06X (see stdout)", addr);
                return;
            }
        }

        // No ACARS or CPDLC. Output raw payload as hex + printable ASCII.
        n += snprintf(outbuf + n, outbuf_size - n, "ELM %06X [%d bytes]: ", addr, len);

        // Hex dump
        for (i = 0; i < len && i < 40 && n < outbuf_size - 3; i++)
            n += snprintf(outbuf + n, outbuf_size - n, "%02X", payload[i]);
        if (len > 40)
            n += snprintf(outbuf + n, outbuf_size - n, "...");

        // ASCII if there's meaningful text
        int has_printable = 0;
        for (i = 0; i < len; i++) {
            if (isprint(payload[i]) && payload[i] != ' ')
                has_printable++;
        }
        if (has_printable > len / 4) {
            n += snprintf(outbuf + n, outbuf_size - n, " |");
            for (i = 0; i < len && n < outbuf_size - 2; i++) {
                uint8_t c = payload[i];
                outbuf[n++] = (c >= 0x20 && c < 0x7f) ? c : '.';
            }
            outbuf[n] = '\0';
            n += snprintf(outbuf + n, outbuf_size - n, "|");
        }
    }

    outbuf[outbuf_size - 1] = '\0';
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
            char decoded[512];
            decoded[0] = '\0';
            elm_decode_acars(msg->addr, msg->payload, msg->payload_len,
                             decoded, sizeof(decoded));

            // Print to stdout (legacy)
            if (decoded[0]) {
                printf("%s\n", decoded);
                fflush(stdout);
            }

            // Log decoded content to panel
            if (decoded[0]) {
                int rxid = -1;
                for (int ri = 0; ri < SdrManager.count; ri++)
                    if (SdrManager.receivers[ri].config.role == SDR_ROLE_ADSB) { rxid = SdrManager.receivers[ri].dev_index; break; }
                panelLogMessage("[ELM rx%d] %s", rxid, decoded);
            }

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

// ========== Content validation ==========

// Check if payload looks like valid ACARS or CPDLC content.
// Returns 1 if content passes validation, 0 if it looks like garbage.
static int elm_validate_content(const uint8_t *payload, int len) {
    if (len < 10)
        return 0;

    // Check 1: ACARS framing â€” SOH (0x01) present
    for (int i = 0; i < len; i++) {
        if (payload[i] == 0x01) {
            // Found SOH â€” likely real ACARS
            return 1;
        }
    }

    // Check 2: CPDLC/ASN.1 â€” first byte often has recognizable tag patterns
    // FANS-1/A CPDLC uses UPER encoding; first bits are typically small tag values
    // Accept if first byte has high bit clear (tag < 128)
    if ((payload[0] & 0x80) == 0) {
        // Could be ASN.1 UPER, accept provisionally if enough printable content
        int printable = 0;
        for (int i = 0; i < len; i++) {
            if ((payload[i] >= 0x20 && payload[i] < 0x7f) ||
                payload[i] == 0x0a || payload[i] == 0x0d)
                printable++;
        }
        if (printable * 3 >= len)  // >= 33% printable
            return 1;
    }

    // Check 3: Plain text content â€” at least 40% printable ASCII
    int printable = 0;
    for (int i = 0; i < len; i++) {
        if ((payload[i] >= 0x20 && payload[i] < 0x7f) ||
            payload[i] == 0x0a || payload[i] == 0x0d)
            printable++;
    }
    if (printable * 5 >= len * 2)  // >= 40% printable
        return 1;

    return 0;
}

// ========== Segment timing validation ==========

// Check if consecutive segments arrived within reasonable time gaps.
// Real ELM segments arrive in rapid succession (< 5 seconds apart).
static int elm_validate_timing(struct elm_entry *entry, int num_segments) {
    for (int i = 1; i < num_segments; i++) {
        if (entry->seg_time[i] == 0 || entry->seg_time[i - 1] == 0)
            return 0;
        uint64_t gap = entry->seg_time[i] - entry->seg_time[i - 1];
        if (gap > ELM_SEG_GAP_MS)
            return 0;  // gap too large â€” probably unrelated false DF24 frames
    }
    return 1;
}

// ========== Queue a complete message for decode ==========

static void elm_queue_complete(struct elm_state *state, struct elm_entry *entry, int complete_ke) {
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

    int num_segments = max_seg + 1;

    // Require minimum number of consecutive segments
    if (num_segments < ELM_MIN_SEGMENTS) {
        state->messages_rejected++;
        fprintf(stderr, "ELM reject %06X: only %d consecutive segments (need %d)\n",
                entry->addr, num_segments, ELM_MIN_SEGMENTS);
        return;
    }

    // Validate timing between segments
    if (!elm_validate_timing(entry, num_segments)) {
        state->messages_rejected++;
        fprintf(stderr, "ELM reject %06X: segment timing too slow (>%ds gap)\n",
                entry->addr, (int)(ELM_SEG_GAP_MS / 1000));
        return;
    }

    int payload_len = num_segments * ELM_SEGMENT_SIZE;

    // Assemble payload temporarily for content validation
    uint8_t payload[ELM_MAX_PAYLOAD];
    for (int i = 0; i < num_segments; i++) {
        memcpy(payload + i * ELM_SEGMENT_SIZE, entry->data[i], ELM_SEGMENT_SIZE);
    }

    // Validate content â€” must look like ACARS, CPDLC, or structured text
    if (!elm_validate_content(payload, payload_len)) {
        state->messages_rejected++;
        fprintf(stderr, "ELM reject %06X: content validation failed (%d bytes, no ACARS/CPDLC/text)\n",
                entry->addr, payload_len);
        return;
    }

    struct elm_complete *msg = malloc(sizeof(struct elm_complete));
    if (!msg)
        return;

    msg->addr = entry->addr;
    msg->payload_len = payload_len;
    msg->segments_received = num_segments;
    msg->complete_ke = complete_ke;
    msg->timestamp = entry->last_seen;
    msg->next = NULL;
    memcpy(msg->payload, payload, payload_len);

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

void elmAddSegment(struct elm_state *state, uint32_t addr, uint32_t nd,
                   uint32_t ke, const uint8_t *md, uint64_t timestamp) {
    if (nd >= ELM_MAX_SEGMENTS)
        return;

    struct elm_entry *entry = elm_find(state, addr);

    if (!entry) {
        // First segment for this aircraft: only accept ND=0 to start a sequence.
        // Real ELM always starts from segment 0. Random ND values are false positives.
        if (nd != 0)
            return;
        entry = elm_create(state, addr, timestamp);
        if (!entry)
            return;
        state->active_entries++;
    } else {
        // Existing entry: only accept segments that extend the sequence.
        // Find the highest consecutive segment we have so far.
        int next_expected = 0;
        for (int i = 0; i < ELM_MAX_SEGMENTS; i++) {
            if (entry->segments_mask & (1 << i))
                next_expected = i + 1;
            else
                break;
        }
        // Accept only the next expected segment or a re-delivery of one we have.
        // This filters random ND values from misclassified frames.
        if (nd > (uint32_t)next_expected) {
            // Segment too far ahead â€” not sequential, likely garbage.
            // If KE=1 on a non-sequential segment, just ignore it.
            return;
        }
    }

    // Store the segment
    int is_new = !(entry->segments_mask & (1 << nd));
    memcpy(entry->data[nd], md, ELM_SEGMENT_SIZE);
    entry->segments_mask |= (1 << nd);
    entry->seg_time[nd] = timestamp;
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

    // Complete if: KE=1 or we have enough consecutive segments.
    // All validation (min segments, timing, content) happens in elm_queue_complete.
    int complete = 0;
    if (ke == 1 && consecutive >= ELM_MIN_SEGMENTS) {
        complete = 1;
    } else if (consecutive >= ELM_MIN_SEGMENTS && consecutive == ELM_MAX_SEGMENTS) {
        complete = 1; // all 16 segments
    } else if (consecutive >= ELM_MIN_SEGMENTS) {
        // Check if there's a gap after our consecutive run â€” this means
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
        elm_queue_complete(state, entry, (ke == 1) ? 1 : 0);
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
                // TTL expired. If we have enough consecutive segments from 0,
                // try to decode what we have.
                int consec = 0;
                for (int s = 0; s < ELM_MAX_SEGMENTS; s++) {
                    if (e->segments_mask & (1 << s))
                        consec = s + 1;
                    else
                        break;
                }
                if (consec >= ELM_MIN_SEGMENTS) {
                    elm_queue_complete(state, e, 0);
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
        fprintf(stderr, "ELM stats: %" PRIu64 " segments, %" PRIu64 " complete, %" PRIu64 " expired, %" PRIu64 " rejected, %d active\n",
                state->segments_received, state->messages_completed,
                state->messages_expired, state->messages_rejected, state->active_entries);
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
                        uint8_t c = e->data[s][b];
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
