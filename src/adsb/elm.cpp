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

#include <cstdio>
#include <stdint.h>
#include <cstdlib>
#include <cstring>
#include <ctype.h>
#include <inttypes.h>
#include "elm.h"
#include "cpdlc_decode.h"
#include "config_panel.h"
#include "sdr_receiver.h"
#include "gg_format.h"
#include <string>

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
    struct elm_entry *e = static_cast<struct elm_entry*>(calloc(1, sizeof(struct elm_entry)));
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

static void elm_decode_acars(uint32_t addr, const uint8_t *payload, int32_t len,
                             char *outbuf, int32_t outbuf_size) {
    // Look for ACARS framing
    int32_t i;
    int32_t found_acars = 0;
    int32_t n = 0;  // output buffer position

    for (i = 0; i < len; i++) {
        if (payload[i] == 0x01) { // SOH - start of ACARS header

            // Find end of message (ETX=0x03 or ETB=0x17)
            int32_t end = i + 1;
            while (end < len && payload[end] != 0x03 && payload[end] != 0x17)
                end++;

            // Extract the ACARS content between SOH and ETX/ETB
            int32_t msg_start = i + 1;
            int32_t msg_len = end - msg_start;

            // Structural validation (ARINC 622 framing): an explicit ETX/ETB
            // terminator must be present and the header (mode char, 7-char
            // address, TAK, 2-char label) must be complete and printable.
            // A lone SOH byte inside random data is not an ACARS message.
            if (end >= len || msg_len < 12 ||
                !isprint(payload[msg_start]) ||
                !isprint(payload[msg_start + 9]) ||
                !isprint(payload[msg_start + 10])) {
                i = end;
                continue;
            }
            int32_t bad_addr = 0;
            for (int32_t j = 1; j <= 7; j++)
                if (!isprint(payload[msg_start + j]))
                    bad_addr = 1;
            if (bad_addr) {
                i = end;
                continue;
            }

            found_acars = 1;

            if (msg_len > 2) {
                n += snprintf(outbuf + n, outbuf_size - n, "ELM ACARS %06X: ", addr);

                // Print mode character if printable
                if (msg_len > 0 && isprint(payload[msg_start]))
                    n += snprintf(outbuf + n, outbuf_size - n, "mode=%c ", payload[msg_start]);

                // Print the address field (up to 7 chars)
                if (msg_len > 8) {
                    n += snprintf(outbuf + n, outbuf_size - n, "reg=");
                    for (int32_t j = 1; j < 8 && (msg_start + j) < len; j++) {
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
                int32_t stx_pos = -1;
                for (int32_t j = msg_start; j < end; j++) {
                    if (payload[j] == 0x02) { // STX
                        stx_pos = j + 1;
                        break;
                    }
                }

                if (stx_pos >= 0 && stx_pos < end) {
                    n += snprintf(outbuf + n, outbuf_size - n, "text=\"");
                    for (int32_t j = stx_pos; j < end && n < outbuf_size - 4; j++) {
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
        // Try CPDLC decode -- capture output via open_memstream
        char *cpdlc_buf = NULL;
        size_t cpdlc_len = 0;
        FILE *old_stdout = stdout;
        FILE *mem = open_memstream(&cpdlc_buf, &cpdlc_len);
        if (mem) {
            stdout = mem;
            int32_t decoded = cpdlc_try_decode_dir(addr, payload, len, 0);
            fflush(mem);
            stdout = old_stdout;
            fclose(mem);
            if (decoded && cpdlc_buf && cpdlc_len > 0) {
                if (cpdlc_len > 0 && cpdlc_buf[cpdlc_len - 1] == '\n')
                    cpdlc_buf[cpdlc_len - 1] = '\0';
                n += snprintf(outbuf + n, outbuf_size - n, "%s", cpdlc_buf);
                free(cpdlc_buf);
                return;
            }
            free(cpdlc_buf);
        } else {
            if (cpdlc_try_decode_dir(addr, payload, len, 0)) {
                fflush(stdout);
                snprintf(outbuf, outbuf_size, "CPDLC %06X (see stdout)", addr);
                return;
            }
        }

        // CPDLC decode failed. Show as raw candidate so the user can see
        // that ELM data was received, but mark it clearly as unvalidated.
        n += snprintf(outbuf + n, outbuf_size - n, "ELM RAW %06X [%d bytes]: ", addr, len);
        for (int32_t j = 0; j < len && n < outbuf_size - 4; j++) {
            n += snprintf(outbuf + n, outbuf_size - n, "%02X", payload[j]);
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
                gg::print("%s\n", decoded);
                fflush(stdout);
            }

            // Log decoded content to panel
            if (decoded[0]) {
                int32_t rxid = -1;
                for (int32_t ri = 0; ri < SdrManager.count; ri++)
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
static int32_t elm_validate_content(const uint8_t *payload, int32_t len) {
    if (len < 20)
        return 0;

    // Check 1: ACARS framing -- SOH (0x01) present with proper terminator
    for (int32_t i = 0; i < len; i++) {
        if (payload[i] == 0x01) {
            for (int32_t j = i + 12; j < len; j++) {
                if (payload[j] == 0x03 || payload[j] == 0x17)
                    return 1;
            }
        }
    }

    // Check 2: CPDLC/ASN.1 UPER -- basic noise filter.
    // The full CPDLC decoder performs strict structural validation (PER range
    // checks, reserved element rejection, padding <= 7 bits, all-zero tail).
    // Here we only reject payloads that are obviously not structured data.

    // All-identical bytes are never a real message
    int32_t all_same = 1;
    for (int32_t i = 1; i < len; i++) {
        if (payload[i] != payload[0]) { all_same = 0; break; }
    }
    if (all_same) return 0;

    // Majority high-bit bytes indicates random noise, not UPER data.
    // CPDLC UPER encodes small constrained integers; high-bit dominance
    // means most values exceed their valid ranges.
    int32_t highbit_count = 0;
    for (int32_t i = 0; i < len; i++) {
        if (payload[i] & 0x80) highbit_count++;
    }
    if (highbit_count * 2 > len)
        return 0;

    // Reject payloads that are all-zero after the first byte (padding artifact)
    int32_t nonzero_after_first = 0;
    for (int32_t i = 1; i < len; i++) {
        if (payload[i] != 0) { nonzero_after_first = 1; break; }
    }
    if (!nonzero_after_first)
        return 0;

    return 1;
}

// ========== Segment timing validation ==========

// Check if consecutive segments arrived within reasonable time gaps.
// Real ELM segments arrive in rapid succession (< 5 seconds apart).
static int32_t elm_validate_timing(struct elm_entry *entry, int32_t num_segments) {
    for (int32_t i = 1; i < num_segments; i++) {
        if (entry->seg_time[i] == 0 || entry->seg_time[i - 1] == 0)
            return 0;
        uint64_t gap = entry->seg_time[i] - entry->seg_time[i - 1];
        if (gap > ELM_SEG_GAP_MS)
            return 0;  // gap too large -- probably unrelated false DF24 frames
    }
    return 1;
}

// ========== Queue a complete message for decode ==========

static void elm_queue_complete(struct elm_state *state, struct elm_entry *entry, int32_t complete_ke) {
    // Find the highest consecutive segment from 0
    int32_t max_seg = -1;
    for (int32_t i = 0; i < ELM_MAX_SEGMENTS; i++) {
        if (entry->segments_mask & (1 << i))
            max_seg = i;
        else
            break;
    }

    if (max_seg < 0)
        return; // nothing usable

    int32_t num_segments = max_seg + 1;

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
                entry->addr, (int32_t)(ELM_SEG_GAP_MS / 1000));
        return;
    }

    int32_t payload_len = num_segments * ELM_SEGMENT_SIZE;

    // Assemble payload temporarily for content validation
    uint8_t payload[ELM_MAX_PAYLOAD];
    for (int32_t i = 0; i < num_segments; i++) {
        memcpy(payload + i * ELM_SEGMENT_SIZE, entry->data[i], ELM_SEGMENT_SIZE);
    }

    // Validate content -- must look like ACARS, CPDLC, or structured text
    if (!elm_validate_content(payload, payload_len)) {
        state->messages_rejected++;
        fprintf(stderr, "ELM reject %06X: content validation failed (%d bytes, no ACARS/CPDLC/text)\n",
                entry->addr, payload_len);
        return;
    }

    struct elm_complete *msg = static_cast<struct elm_complete*>(malloc(sizeof(struct elm_complete)));
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
    for (int32_t i = 0; i < ELM_TABLE_SIZE; i++) {
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
    // KE=1 marks an uplink ELM acknowledgement (Annex 10 Vol IV, 3.1.2.7.3):
    // the MD field then carries a TAS (Transmission Acknowledgement Summary),
    // not downlink data, and ND is not defined. Such frames must never enter
    // downlink reassembly.
    if (ke == 1)
        return;

    if (nd >= ELM_MAX_SEGMENTS)
        return;

    struct elm_entry *entry = elm_find(state, addr);

    if (!entry) {
        // First segment for this aircraft: only accept ND=0 to start a sequence.
        if (nd != 0)
            return;
        entry = elm_create(state, addr, timestamp);
        if (!entry)
            return;
        state->active_entries++;
    } else if (nd == 0 && (timestamp - entry->last_seen > ELM_SEG_GAP_MS)) {
        // New segment 0 arrived after a long gap -- start fresh sequence.
        // The old entry was likely an incomplete/stale message.
        elm_remove(state, addr);
        state->active_entries--;
        entry = elm_create(state, addr, timestamp);
        if (!entry)
            return;
        state->active_entries++;
    } else {
        // Existing entry: only accept segments that extend the sequence.
        // Find the highest consecutive segment we have so far.
        int32_t next_expected = 0;
        for (int32_t i = 0; i < ELM_MAX_SEGMENTS; i++) {
            if (entry->segments_mask & (1 << i))
                next_expected = i + 1;
            else
                break;
        }
        // Accept only the next expected segment or a re-delivery of one we have.
        // This filters random ND values from misclassified frames.
        if (nd > (uint32_t)next_expected) {
            // Segment too far ahead -- not sequential, likely garbage.
            // If KE=1 on a non-sequential segment, just ignore it.
            return;
        }
    }

    // Store the segment
    int32_t is_new = !(entry->segments_mask & (1 << nd));
    memcpy(entry->data[nd], md, ELM_SEGMENT_SIZE);
    entry->segments_mask |= (1 << nd);
    entry->seg_time[nd] = timestamp;
    entry->last_seen = timestamp;
    state->segments_received++;

    // Log new segment arrival immediately
    if (is_new) {
        int32_t segs_have = 0;
        for (int32_t i = 0; i < ELM_MAX_SEGMENTS; i++)
            if (entry->segments_mask & (1 << i))
                segs_have++;
        gg::eprint("ELM seg %06X: ND=%u KE=%u [%d/16] ", addr, nd, ke, segs_have);
        for (int32_t b = 0; b < ELM_SEGMENT_SIZE; b++)
            gg::eprint("%02X", md[b]);
        gg::eprint("\n");
        fflush(stderr);
    }

    // Check for completion:
    // 1. We have segment 0 (required)
    // 2. We have consecutive segments from 0..N
    // 3. Either KE=1 (close-out) or we have a gap after the last received
    if (!(entry->segments_mask & 1))
        return; // no segment 0 yet

    int32_t consecutive = 0;
    for (int32_t i = 0; i < ELM_MAX_SEGMENTS; i++) {
        if (entry->segments_mask & (1 << i))
            consecutive = i + 1;
        else
            break;
    }

    // DF24 carries no in-band close-out flag: the downlink ELM segment count
    // is announced via the DR field (values 16-31) of the surveillance
    // protocol, which a passive receiver cannot rely on. Complete immediately
    // only when all 16 segments are present (the message cannot grow further);
    // shorter messages are flushed by the TTL in elmCleanupStale().
    int32_t complete = 0;
    if (consecutive == ELM_MAX_SEGMENTS)
        complete = 1;

    if (complete) {
        elm_queue_complete(state, entry, 1);
        elm_remove(state, addr);
        state->messages_completed++;
        state->active_entries--;
    }
}

void elmCleanupStale(struct elm_state *state, uint64_t now) {
    for (int32_t i = 0; i < ELM_TABLE_SIZE; i++) {
        struct elm_entry **pp = &state->table[i];
        while (*pp) {
            struct elm_entry *e = *pp;
            if (now - e->last_seen > ELM_TTL_MS) {
                // TTL expired. If we have enough consecutive segments from 0,
                // try to decode what we have.
                int32_t consec = 0;
                for (int32_t s = 0; s < ELM_MAX_SEGMENTS; s++) {
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

    for (int32_t i = 0; i < ELM_TABLE_SIZE; i++) {
        struct elm_entry *e = state->table[i];
        while (e) {
            int32_t segs_have = 0;
            for (int32_t s = 0; s < ELM_MAX_SEGMENTS; s++)
                if (e->segments_mask & (1 << s))
                    segs_have++;

            double age_sec = (now - e->first_seen) / 1000.0;

            fprintf(stderr, "ELM partial %06X [%d/16 seg, %.1fs]: ",
                    e->addr, segs_have, age_sec);

            // Print segment map: hex for received, ".." for missing
            for (int32_t s = 0; s < ELM_MAX_SEGMENTS; s++) {
                if (e->segments_mask & (1 << s)) {
                    for (int32_t b = 0; b < ELM_SEGMENT_SIZE; b++)
                        gg::eprint("%02X", e->data[s][b]);
                } else {
                    // 10 bytes missing = 20 dots
                    gg::eprint("....................");
                }
                if (s < ELM_MAX_SEGMENTS - 1)
                    gg::eprint("|");
            }

            // Also print ASCII view of received segments
            gg::eprint(" |");
            for (int32_t s = 0; s < ELM_MAX_SEGMENTS; s++) {
                if (e->segments_mask & (1 << s)) {
                    for (int32_t b = 0; b < ELM_SEGMENT_SIZE; b++) {
                        uint8_t c = e->data[s][b];
                        fputc((c >= 0x20 && c < 0x7f) ? c : '.', stderr);
                    }
                } else {
                    gg::eprint("          ");
                }
            }
            gg::eprint("|\n");

            e = e->next;
        }
    }
    fflush(stderr);
}
