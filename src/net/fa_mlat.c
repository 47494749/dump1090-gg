// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// fa_mlat.c: Built-in FlightAware MLAT client for dump1090-gg
//
// Replaces the external fa-mlat-client Python program.
// Implements the FlightAware ADEPT UDP binary protocol for sending
// multilateration data to FlightAware's MLAT server.
//
// Protocol reference: fa-mlat-client by Oliver Jowett, GPL-3+
// https://github.com/mutability/mlat-client
// (flightaware/client/adeptclient.py, mlat/client/coordinator.py)
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include "dump1090.h"
#include "fa_mlat.h"
#include "feeder_thread.h"

#include <math.h>
#include <stdarg.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>

struct fa_mlat_state FaMlat;

// ============================= UDP Protocol ==============================
//
// All multi-byte values are big-endian (network byte order).
// Datagram format: HEADER + submessages
// Flushed when buffer exceeds MTU (~1400 bytes) or on heartbeat (500ms).

struct fa_mlat_udp {
    int      fd;
    uint32_t key;
    uint16_t seq;
    uint64_t base_timestamp;
    int      has_base;
    uint8_t  buf[1500];
    int      used;
    int      count;             // total datagrams sent
    int      mtu;
};

// Pack uint64 big-endian into buffer
static inline void pack_u64(uint8_t *p, uint64_t v) {
    p[0] = (v >> 56) & 0xFF;
    p[1] = (v >> 48) & 0xFF;
    p[2] = (v >> 40) & 0xFF;
    p[3] = (v >> 32) & 0xFF;
    p[4] = (v >> 24) & 0xFF;
    p[5] = (v >> 16) & 0xFF;
    p[6] = (v >> 8)  & 0xFF;
    p[7] = v & 0xFF;
}

// Pack uint32 big-endian into buffer
static inline void pack_u32(uint8_t *p, uint32_t v) {
    p[0] = (v >> 24) & 0xFF;
    p[1] = (v >> 16) & 0xFF;
    p[2] = (v >> 8)  & 0xFF;
    p[3] = v & 0xFF;
}

// Pack int32 big-endian into buffer
static inline void pack_i32(uint8_t *p, int32_t v) {
    pack_u32(p, (uint32_t)v);
}

// Pack uint16 big-endian into buffer
static inline void pack_u16(uint8_t *p, uint16_t v) {
    p[0] = (v >> 8) & 0xFF;
    p[1] = v & 0xFF;
}

static void udp_init(struct fa_mlat_udp *u) {
    memset(u, 0, sizeof(*u));
    u->fd = -1;
    u->mtu = 1400;
}

static int udp_start(struct fa_mlat_udp *u, const char *host, int port, uint32_t key) {
    struct addrinfo hints, *res;
    char port_str[16];

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_NUMERICHOST;

    snprintf(port_str, sizeof(port_str), "%d", port);

    int err = getaddrinfo(host, port_str, &hints, &res);
    if (err != 0) {
        // Try without AI_NUMERICHOST for hostname resolution
        hints.ai_flags = 0;
        err = getaddrinfo(host, port_str, &hints, &res);
        if (err != 0) {
            fprintf(stderr, "FA-MLAT: UDP resolve failed for %s:%d: %s\n",
                    host, port, gai_strerror(err));
            return -1;
        }
    }

    u->fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (u->fd < 0) {
        fprintf(stderr, "FA-MLAT: UDP socket failed: %s\n", strerror(errno));
        freeaddrinfo(res);
        return -1;
    }

    // "Connect" UDP socket to set default destination
    if (connect(u->fd, res->ai_addr, res->ai_addrlen) < 0) {
        // Not fatal for UDP, but log it
        fprintf(stderr, "FA-MLAT: UDP connect warning: %s\n", strerror(errno));
    }

    freeaddrinfo(res);

    u->key = key;
    u->seq = 0;
    u->used = 0;
    u->has_base = 0;
    u->count = 0;

    fprintf(stderr, "FA-MLAT: UDP transport started to %s:%d (key=%u)\n",
            host, port, key);
    return 0;
}

static void udp_close(struct fa_mlat_udp *u) {
    if (u->fd >= 0) {
        close(u->fd);
        u->fd = -1;
    }
    u->used = 0;
    u->has_base = 0;
}

// Prepare datagram header: key(4) + seq(2) + base_timestamp(8) = 14 bytes
static void udp_prepare_header(struct fa_mlat_udp *u, uint64_t timestamp) {
    u->base_timestamp = timestamp;
    u->has_base = 1;
    pack_u32(u->buf, u->key);
    pack_u16(u->buf + 4, u->seq);
    pack_u64(u->buf + 6, u->base_timestamp);
    u->used = FA_MLAT_HDR_SIZE;
}

// Append a REBASE submessage: type(1) + timestamp(8) = 9 bytes
static void udp_rebase(struct fa_mlat_udp *u, uint64_t timestamp) {
    u->base_timestamp = timestamp;
    u->buf[u->used] = FA_MLAT_TYPE_REBASE;
    pack_u64(u->buf + u->used + 1, u->base_timestamp);
    u->used += FA_MLAT_REBASE_SIZE;
}

// Flush the current datagram
static void udp_flush(struct fa_mlat_udp *u) {
    if (!u->used || u->fd < 0)
        return;

    ssize_t sent = send(u->fd, u->buf, u->used, 0);
    (void)sent;  // UDP send is best-effort

    u->used = 0;
    u->has_base = 0;
    u->seq = (u->seq + 1) & 0xFFFF;
    u->count++;
}

// Pack ICAO address as 3 bytes big-endian
static inline void pack_icao(uint8_t *p, uint32_t addr) {
    p[0] = (addr >> 16) & 0xFF;
    p[1] = (addr >> 8) & 0xFF;
    p[2] = addr & 0xFF;
}

// Send an individual MLAT message via UDP
// msg is raw Mode S bytes, msglen is 2 (ModeAC), 7 (short), or 14 (long)
static void udp_send_mlat(struct fa_mlat_udp *u, uint32_t addr,
                           uint64_t timestamp, const uint8_t *msg, int msglen) {
    if (u->fd < 0) return;

    if (!u->has_base)
        udp_prepare_header(u, timestamp);

    int32_t delta = (int32_t)(timestamp - u->base_timestamp);
    if (abs(delta) > 0x7FFFFFF0) {
        udp_rebase(u, timestamp);
        delta = 0;
    }

    uint8_t *p = u->buf + u->used;

    if (msglen == 2) {
        // Mode A/C: type(1) + delta(4) + msg(2)
        p[0] = FA_MLAT_TYPE_MLAT_MODEAC;
        pack_i32(p + 1, delta);
        memcpy(p + 5, msg, 2);
        u->used += FA_MLAT_MODEAC_SIZE;
    } else if (msglen == 7) {
        // Short: type(1) + icao(3) + delta(4) + msg(7)
        p[0] = FA_MLAT_TYPE_MLAT_SHORT;
        pack_icao(p + 1, addr);
        pack_i32(p + 4, delta);
        memcpy(p + 8, msg, 7);
        u->used += FA_MLAT_SHORT_SIZE;
    } else if (msglen == 14) {
        // Long: type(1) + icao(3) + delta(4) + msg(14)
        p[0] = FA_MLAT_TYPE_MLAT_LONG;
        pack_icao(p + 1, addr);
        pack_i32(p + 4, delta);
        memcpy(p + 8, msg, 14);
        u->used += FA_MLAT_LONG_SIZE;
    } else {
        return;
    }

    if (u->used > u->mtu)
        udp_flush(u);
}

// Send a sync pair (even + odd DF17 messages) via UDP
static void udp_send_sync(struct fa_mlat_udp *u, uint32_t addr,
                           uint64_t even_ts, uint64_t odd_ts,
                           const uint8_t *even_msg, const uint8_t *odd_msg) {
    if (u->fd < 0) return;

    if (!u->has_base)
        udp_prepare_header(u, (uint64_t)((even_ts + odd_ts) / 2));

    // Check if timestamps are too far apart for relative encoding
    if ((uint64_t)llabs((int64_t)(even_ts - odd_ts)) > 0xFFFFFFF0ULL) {
        // Use absolute sync
        uint8_t *p = u->buf + u->used;
        p[0] = FA_MLAT_TYPE_ABS_SYNC;
        pack_icao(p + 1, addr);
        pack_u64(p + 4, even_ts);
        pack_u64(p + 12, odd_ts);
        memcpy(p + 20, even_msg, 14);
        memcpy(p + 34, odd_msg, 14);
        u->used += FA_MLAT_ABS_SYNC_SIZE;
    } else {
        int32_t edelta = (int32_t)(even_ts - u->base_timestamp);
        int32_t odelta = (int32_t)(odd_ts - u->base_timestamp);

        if (abs(edelta) > 0x7FFFFFF0 || abs(odelta) > 0x7FFFFFF0) {
            udp_rebase(u, (uint64_t)((even_ts + odd_ts) / 2));
            edelta = (int32_t)(even_ts - u->base_timestamp);
            odelta = (int32_t)(odd_ts - u->base_timestamp);
        }

        uint8_t *p = u->buf + u->used;
        p[0] = FA_MLAT_TYPE_SYNC;
        pack_icao(p + 1, addr);
        pack_i32(p + 4, edelta);
        pack_i32(p + 8, odelta);
        memcpy(p + 12, even_msg, 14);
        memcpy(p + 26, odd_msg, 14);
        u->used += FA_MLAT_SYNC_SIZE;
    }

    if (u->used > u->mtu)
        udp_flush(u);
}

// ============================= Status Output =============================

// Enqueue a status line for PiAware to pick up and forward to FA server.
// Thread-safe (called from FA MLAT thread).
static void fa_mlat_send_status(const char *fmt, ...) {
    va_list ap;
    char line[FA_MLAT_STATUS_LINE_LEN];

    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    pthread_mutex_lock(&FaMlat.status_mutex);

    int next = (FaMlat.status_head + 1) % FA_MLAT_STATUS_LINES;
    if (next != FaMlat.status_tail) {
        snprintf(FaMlat.status_lines[FaMlat.status_head], FA_MLAT_STATUS_LINE_LEN, "%s", line);
        FaMlat.status_head = next;
    }

    pthread_mutex_unlock(&FaMlat.status_mutex);
}

// ============================= Aircraft Hash Table =======================

static struct fa_mlat_aircraft *fa_mlat_find_aircraft(uint32_t addr) {
    uint32_t hash = addr & FA_MLAT_HASH_MASK;
    for (int i = 0; i < 4; i++) {
        uint32_t idx = (hash + i) & FA_MLAT_HASH_MASK;
        if (FaMlat.aircraft[idx].addr == addr)
            return &FaMlat.aircraft[idx];
        if (FaMlat.aircraft[idx].addr == 0)
            return NULL;
    }
    return NULL;
}

static struct fa_mlat_aircraft *fa_mlat_get_or_create_aircraft(uint32_t addr, uint64_t now) {
    uint32_t hash = addr & FA_MLAT_HASH_MASK;
    struct fa_mlat_aircraft *empty = NULL;

    for (int i = 0; i < 8; i++) {
        uint32_t idx = (hash + i) & FA_MLAT_HASH_MASK;
        if (FaMlat.aircraft[idx].addr == addr)
            return &FaMlat.aircraft[idx];
        if (FaMlat.aircraft[idx].addr == 0 && !empty)
            empty = &FaMlat.aircraft[idx];
    }

    if (!empty) return NULL;  // hash table full in this region

    memset(empty, 0, sizeof(*empty));
    empty->addr = addr;
    empty->rate_measurement_start = now;
    return empty;
}

static void fa_mlat_expire_aircraft(uint64_t now) {
    for (int i = 0; i < FA_MLAT_HASH_SIZE; i++) {
        if (FaMlat.aircraft[i].addr != 0) {
            if ((now - FaMlat.aircraft[i].last_message_time) > FA_MLAT_EXPIRY_AGE) {
                FaMlat.aircraft[i].addr = 0;  // mark as empty
            }
        }
    }
}


static int fa_mlat_is_wanted_modeac(uint32_t code) {
    for (int i = 0; i < FaMlat.wanted_modeac_count; i++) {
        if (FaMlat.wanted_modeac[i] == code)
            return 1;
    }
    return 0;
}

// ============================= Coordinator Logic =========================
//
// Replicates the logic from coordinator.py:
// - Track aircraft by ICAO
// - For DF17 position: track even/odd CPR, send sync pairs
// - For DF0/4/5/11/16/20/21: send individual MLAT if requested, no recent position
// - For Mode A/C: send if requested
// - Report seen/lost aircraft every 30s
// - Report rates every 30s

static void fa_mlat_handle_df17(struct modesMessage *mm, struct fa_mlat_aircraft *ac,
                                 struct fa_mlat_udp *udp, uint64_t now) {
    ac->messages++;
    ac->last_message_time = now;

    if (ac->messages < FA_MLAT_MIN_MESSAGES)
        return;

    // Only process position messages (even or odd CPR)
    if (!mm->cpr_valid)
        return;

    ac->last_position_time = now;

    // Need altitude
    if (!mm->altitude_baro_valid)
        return;

    // Need NUCp >= 6
    if (mm->cpr_nucp < 6)
        return;

    ac->recent_adsb_positions++;

    // Store even/odd message for sync pair
    int msgbytes = mm->msgbits / 8;
    if (msgbytes != 14) return;

    if (!mm->cpr_odd) {
        // Even message
        ac->has_even = 1;
        ac->even_timestamp = mm->timestampMsg;
        memcpy(ac->even_msg, mm->verbatim, 14);
        ac->even_msgbits = mm->msgbits;
        ac->even_nucp = mm->cpr_nucp;
    } else {
        // Odd message
        ac->has_odd = 1;
        ac->odd_timestamp = mm->timestampMsg;
        memcpy(ac->odd_msg, mm->verbatim, 14);
        ac->odd_msgbits = mm->msgbits;
        ac->odd_nucp = mm->cpr_nucp;
    }

    if (!ac->requested)
        return;

    // Need both even and odd
    if (!ac->has_even || !ac->has_odd)
        return;

    // Timestamps must be within 5 seconds (60M ticks at 12MHz)
    uint64_t ts_diff;
    if (ac->even_timestamp > ac->odd_timestamp)
        ts_diff = ac->even_timestamp - ac->odd_timestamp;
    else
        ts_diff = ac->odd_timestamp - ac->even_timestamp;

    if (ts_diff > FA_MLAT_SYNC_MAX_DELTA)
        return;

    // Send sync pair
    udp_send_sync(udp, ac->addr,
                  ac->even_timestamp, ac->odd_timestamp,
                  ac->even_msg, ac->odd_msg);
}

static void fa_mlat_handle_df_misc(struct modesMessage *mm, struct fa_mlat_aircraft *ac,
                                    struct fa_mlat_udp *udp, uint64_t now) {
    ac->messages++;
    ac->last_message_time = now;

    if (ac->messages < FA_MLAT_MIN_MESSAGES)
        return;

    if (!ac->requested)
        return;

    // Don't send if aircraft has a recent ADS-B position
    if ((now - ac->last_position_time) < FA_MLAT_POSITION_EXPIRY)
        return;

    int msgbytes = mm->msgbits / 8;
    udp_send_mlat(udp, ac->addr, mm->timestampMsg, mm->verbatim, msgbytes);
}

static void fa_mlat_handle_df11(struct modesMessage *mm, struct fa_mlat_aircraft *ac,
                                 struct fa_mlat_udp *udp, uint64_t now) {
    // DF11: same as df_misc but also creates new aircraft entries
    fa_mlat_handle_df_misc(mm, ac, udp, now);
}

static void fa_mlat_handle_modeac(struct modesMessage *mm,
                                   struct fa_mlat_udp *udp) {
    // Mode A/C: check if the code is in the requested modeac set
    uint32_t code = (mm->msg[0] << 8) | mm->msg[1];
    if (!fa_mlat_is_wanted_modeac(code))
        return;

    udp_send_mlat(udp, 0, mm->timestampMsg, mm->verbatim, 2);
}

// ============================= Reports ===================================

static void fa_mlat_send_reports(uint64_t now) {
    // Build seen/lost/rate reports

    // Collect all aircraft with >1 message
    uint32_t all_icao[FA_MLAT_HASH_SIZE];
    int all_count = 0;
    for (int i = 0; i < FA_MLAT_HASH_SIZE && all_count < FA_MLAT_HASH_SIZE; i++) {
        if (FaMlat.aircraft[i].addr != 0 && FaMlat.aircraft[i].messages > 1)
            all_icao[all_count++] = FaMlat.aircraft[i].addr;
    }

    // Build "seen" list (aircraft we know about but haven't reported)
    char seen_buf[FA_MLAT_STATUS_LINE_LEN];
    int seen_pos = 0;
    seen_pos = snprintf(seen_buf, sizeof(seen_buf), "type\tmlat_seen\thexids\t");
    int seen_count = 0;

    for (int i = 0; i < all_count; i++) {
        struct fa_mlat_aircraft *ac = fa_mlat_find_aircraft(all_icao[i]);
        if (ac && !ac->reported) {
            if (seen_count > 0 && seen_pos < (int)sizeof(seen_buf) - 8)
                seen_buf[seen_pos++] = ' ';
            seen_pos += snprintf(seen_buf + seen_pos, sizeof(seen_buf) - seen_pos,
                                 "%06X", ac->addr);
            ac->reported = 1;
            seen_count++;
        }
    }

    if (seen_count > 0)
        fa_mlat_send_status("%s", seen_buf);

    // Build "lost" list (aircraft we reported but no longer see)
    char lost_buf[FA_MLAT_STATUS_LINE_LEN];
    int lost_pos = 0;
    lost_pos = snprintf(lost_buf, sizeof(lost_buf), "type\tmlat_lost\thexids\t");
    int lost_count = 0;

    for (int i = 0; i < FA_MLAT_HASH_SIZE; i++) {
        struct fa_mlat_aircraft *ac = &FaMlat.aircraft[i];
        if (ac->addr != 0 && ac->reported && ac->messages <= 1) {
            // Was reported but no longer qualifies
            if (lost_count > 0 && lost_pos < (int)sizeof(lost_buf) - 8)
                lost_buf[lost_pos++] = ' ';
            lost_pos += snprintf(lost_buf + lost_pos, sizeof(lost_buf) - lost_pos,
                                 "%06X", ac->addr);
            ac->reported = 0;
            lost_count++;
        }
    }

    // Also report as lost: aircraft we reported but have since expired (addr==0)
    // These are already cleaned up by expire, so we track them via the reported set.
    // Since we use a hash table and expired entries have addr=0, we handle this
    // by checking for reported aircraft no longer in all_icao. For simplicity,
    // we clear reported on expiry.

    if (lost_count > 0)
        fa_mlat_send_status("%s", lost_buf);

    // Build rate report
    char rate_buf[FA_MLAT_STATUS_LINE_LEN];
    int rate_pos = 0;
    rate_pos = snprintf(rate_buf, sizeof(rate_buf), "type\tmlat_rates\trates\t");
    int rate_count = 0;

    for (int i = 0; i < FA_MLAT_HASH_SIZE; i++) {
        struct fa_mlat_aircraft *ac = &FaMlat.aircraft[i];
        if (ac->addr != 0 && ac->recent_adsb_positions > 0) {
            double interval = (now - ac->rate_measurement_start) / 1000.0;
            if (interval > 0) {
                double rate = ac->recent_adsb_positions / interval;
                if (rate_count > 0 && rate_pos < (int)sizeof(rate_buf) - 16)
                    rate_buf[rate_pos++] = ' ';
                rate_pos += snprintf(rate_buf + rate_pos, sizeof(rate_buf) - rate_pos,
                                     "%06X %.2f", ac->addr, rate);
                rate_count++;
            }
            ac->rate_measurement_start = now;
            ac->recent_adsb_positions = 0;
        }
    }

    if (rate_count > 0)
        fa_mlat_send_status("%s", rate_buf);
}

// ============================= Thread Entry ==============================

static void *fa_mlat_thread_entry(void *arg) {
    (void)arg;

    struct fa_mlat_udp udp;
    udp_init(&udp);

    // Read config
    pthread_mutex_lock(&FaMlat.ctl_mutex);
    char host[256];
    strncpy(host, FaMlat.udp_host, sizeof(host) - 1);
    host[sizeof(host) - 1] = 0;
    int port = FaMlat.udp_port;
    uint32_t key = FaMlat.udp_key;
    pthread_mutex_unlock(&FaMlat.ctl_mutex);

    // Start UDP transport
    if (udp_start(&udp, host, port, key) < 0) {
        fprintf(stderr, "FA-MLAT: failed to start UDP transport, thread exiting\n");
        FaMlat.thread_running = 0;
        return NULL;
    }

    // Send ready event
    fa_mlat_send_status("type\tmlat_event\tevent\tready\tmlat_client_version\tdump1090-gg-builtin");

    // Send connected event (we're always "connected" since we read from the internal queue)
    fa_mlat_send_status("type\tmlat_event\tevent\tconnected");

    // Send clock_reset with Beast parameters
    fa_mlat_send_status("type\tmlat_event\tevent\tclock_reset\treason\tInitial clock setup\tfrequency\t12000000\tepoch\tnone\tmode\tBeast");

    // Send position update
    if (Modes.fUserLat != 0 || Modes.fUserLon != 0) {
        fa_mlat_send_status("type\tmlat_location_update\tlat\t%.5f\tlon\t%.5f\talt\t%.0f\taltref\twgs84_meters",
                            Modes.fUserLat, Modes.fUserLon, MlatConfig.alt);
    }

    uint64_t last_report = mstime();
    uint64_t last_heartbeat = mstime();
    uint64_t last_udp_report = mstime();
    uint64_t last_wanted_refresh = 0;

    // Local copy of wanted set for fast lookup (refreshed periodically)
    uint32_t local_wanted[FA_MLAT_MAX_WANTED];
    int local_wanted_count = 0;
    uint32_t local_modeac[FA_MLAT_MAX_MODEAC];
    int local_modeac_count = 0;

    // Clear aircraft hash table
    memset(FaMlat.aircraft, 0, sizeof(FaMlat.aircraft));

    struct modesMessage mm;
    struct timespec sleep_ts = {0, 5 * 1000 * 1000};  // 5ms

    while (1) {
        // Check stop signal
        pthread_mutex_lock(&FaMlat.ctl_mutex);
        int should_stop = FaMlat.stop_requested;
        pthread_mutex_unlock(&FaMlat.ctl_mutex);
        if (should_stop) break;

        // When internet is offline, drain queue without processing
        if (!atomic_load(&net_available)) {
            while (msg_queue_pop(fa_mlat_queue, &mm)) { /* discard */ }
            usleep(500000);
            continue;
        }

        uint64_t now = mstime();

        // Refresh wanted set from shared state (every 1s)
        if ((now - last_wanted_refresh) >= 1000) {
            pthread_mutex_lock(&FaMlat.ctl_mutex);
            local_wanted_count = FaMlat.wanted_count;
            if (local_wanted_count > FA_MLAT_MAX_WANTED)
                local_wanted_count = FA_MLAT_MAX_WANTED;
            memcpy(local_wanted, FaMlat.wanted_icao, local_wanted_count * sizeof(uint32_t));
            local_modeac_count = FaMlat.wanted_modeac_count;
            if (local_modeac_count > FA_MLAT_MAX_MODEAC)
                local_modeac_count = FA_MLAT_MAX_MODEAC;
            memcpy(local_modeac, FaMlat.wanted_modeac, local_modeac_count * sizeof(uint32_t));
            pthread_mutex_unlock(&FaMlat.ctl_mutex);

            // Update per-aircraft requested flags
            for (int i = 0; i < FA_MLAT_HASH_SIZE; i++) {
                if (FaMlat.aircraft[i].addr != 0) {
                    int wanted = 0;
                    for (int j = 0; j < local_wanted_count; j++) {
                        if (local_wanted[j] == FaMlat.aircraft[i].addr) {
                            wanted = 1;
                            break;
                        }
                    }
                    FaMlat.aircraft[i].requested = wanted;
                }
            }

            last_wanted_refresh = now;
        }

        // Process messages from queue
        int got_msg = 0;

        while (msg_queue_pop(fa_mlat_queue, &mm)) {
            got_msg = 1;

            // Skip MLAT results (don't feed back)
            if (mm.source == SOURCE_MLAT) continue;

            int df = mm.msgtype;

            // Mode A/C
            if (mm.msgbits == MODEAC_MSG_BYTES * 8) {
                fa_mlat_handle_modeac(&mm, &udp);
                continue;
            }

            // Need an ICAO address for everything else
            if (mm.addr == 0) continue;

            struct fa_mlat_aircraft *ac = fa_mlat_get_or_create_aircraft(mm.addr, now);
            if (!ac) continue;

            switch (df) {
            case 17:
                fa_mlat_handle_df17(&mm, ac, &udp, now);
                break;
            case 0:
            case 4:
            case 5:
            case 16:
            case 20:
            case 21:
                fa_mlat_handle_df_misc(&mm, ac, &udp, now);
                break;
            case 11:
                fa_mlat_handle_df11(&mm, ac, &udp, now);
                break;
            }
        }

        // Heartbeat: flush UDP buffer every 500ms
        if ((now - last_heartbeat) >= FA_MLAT_HEARTBEAT_MS) {
            udp_flush(&udp);
            last_heartbeat = now;
        }

        // Periodic reports: seen/lost/rates every 30s
        if ((now - last_report) >= FA_MLAT_REPORT_INTERVAL) {
            fa_mlat_expire_aircraft(now);
            fa_mlat_send_reports(now);
            last_report = now;
        }

        // UDP count report every 60s
        if ((now - last_udp_report) >= FA_MLAT_UDP_REPORT_INTERVAL) {
            fa_mlat_send_status("type\tmlat_udp_report\tmessages_sent\t%d", udp.count);
            last_udp_report = now;
        }

        if (!got_msg)
            nanosleep(&sleep_ts, NULL);
    }

    udp_flush(&udp);
    udp_close(&udp);

    fprintf(stderr, "FA-MLAT: thread stopped\n");
    FaMlat.thread_running = 0;
    return NULL;
}

// ============================= Result Injection ==========================
//
// When PiAware receives mlat_result from FA server, it calls faMlatInjectResult().
// We generate synthetic DF18 position frames and inject via mlat_inject_queue.
// Reuses the same approach as mlat_client.c.

// NL table for CPR encoding
static const struct { double lat; int nl; } fa_cpr_nl_table[] = {
    {10.47047130, 59}, {14.82817437, 58}, {18.18626357, 57}, {21.02939493, 56},
    {23.54504487, 55}, {25.82924707, 54}, {27.93898710, 53}, {29.91135686, 52},
    {31.77209708, 51}, {33.53993436, 50}, {35.22899598, 49}, {36.85025108, 48},
    {38.41241892, 47}, {39.92256684, 46}, {41.38651832, 45}, {42.80914012, 44},
    {44.19454951, 43}, {45.54626723, 42}, {46.86733252, 41}, {48.16039128, 40},
    {49.42776439, 39}, {50.67150166, 38}, {51.89342469, 37}, {53.09516153, 36},
    {54.27817472, 35}, {55.44378444, 34}, {56.59318756, 33}, {57.72747354, 32},
    {58.84763776, 31}, {59.95459277, 30}, {61.04917774, 29}, {62.13216659, 28},
    {63.20427479, 27}, {64.26616523, 26}, {65.31845310, 25}, {66.36171008, 24},
    {67.39646774, 23}, {68.42322022, 22}, {69.44242631, 21}, {70.45451075, 20},
    {71.45986473, 19}, {72.45884545, 18}, {73.45177442, 17}, {74.43893416, 16},
    {75.42056257, 15}, {76.39684391, 14}, {77.36789461, 13}, {78.33374083, 12},
    {79.29428225, 11}, {80.24923213, 10}, {81.19801349,  9}, {82.13956981,  8},
    {83.07199445,  7}, {83.99173563,  6}, {84.89166191,  5}, {85.75541621,  4},
    {86.53536998,  3}, {87.00000000,  2}, {90.00000000,  1}
};

static int fa_cpr_NL(double lat) {
    if (lat < 0) lat = -lat;
    for (int i = 0; i < 59; i++) {
        if (lat < fa_cpr_nl_table[i].lat)
            return fa_cpr_nl_table[i].nl;
    }
    return 1;
}

static void fa_cpr_encode(double lat, double lon, int odd, int *rlat, int *rlon) {
    double NbPow = 131072.0;
    double Dlat = 360.0 / (odd ? 59 : 60);

    double lat_mod = fmod(lat, Dlat);
    if (lat_mod < 0) lat_mod += Dlat;

    double yz = floor(NbPow * lat_mod / Dlat + 0.5);
    int YZ = ((int)yz) & 0x1FFFF;

    double Rlat = Dlat * (yz / NbPow + floor(lat / Dlat));
    int nl = fa_cpr_NL(Rlat) - (odd ? 1 : 0);
    if (nl < 1) nl = 1;
    double Dlon = 360.0 / nl;

    double lon_mod = fmod(lon, Dlon);
    if (lon_mod < 0) lon_mod += Dlon;

    double xz = floor(NbPow * lon_mod / Dlon + 0.5);
    int XZ = ((int)xz) & 0x1FFFF;

    *rlat = YZ;
    *rlon = XZ;
}

static int fa_encode_altitude(double ft) {
    int i = (int)((ft + 1012.5) / 25.0);
    if (i < 0) i = 0;
    if (i > 0x7FF) i = 0x7FF;
    return ((i & 0x7F0) << 1) | 0x010 | (i & 0x00F);
}

static void fa_build_position_frame(uint8_t *frame, uint32_t addr,
                                     int elat, int elon, int ealt, int oddflag) {
    memset(frame, 0, 14);

    // DF=18, CF=2 (TIS-B, ICAO address)
    frame[0] = (18 << 3) | 2;
    frame[1] = (addr >> 16) & 0xFF;
    frame[2] = (addr >> 8) & 0xFF;
    frame[3] = addr & 0xFF;

    // ME type 18: airborne position, baro alt
    frame[4] = (18 << 3);
    frame[5] = (ealt >> 4) & 0xFF;
    frame[6] = ((ealt & 0x0F) << 4);
    if (oddflag) frame[6] |= 0x04;
    frame[6] |= (elat >> 15) & 0x03;
    frame[7] = (elat >> 7) & 0xFF;
    frame[8] = ((elat & 0x7F) << 1) | ((elon >> 16) & 0x01);
    frame[9] = (elon >> 8) & 0xFF;
    frame[10] = elon & 0xFF;

    uint32_t crc = modesChecksum(frame, 112);
    frame[11] = (crc >> 16) & 0xFF;
    frame[12] = (crc >> 8) & 0xFF;
    frame[13] = crc & 0xFF;
}

static void fa_build_velocity_frame(uint8_t *frame, uint32_t addr,
                                     double nsvel, double ewvel, double vrate) {
    memset(frame, 0, 14);

    int supersonic = (fabs(nsvel) > 1000 || fabs(ewvel) > 1000);
    int e_ew = 0, e_ns = 0, e_vr = 0;

    if (ewvel < 0) { e_ew = 0x400; ewvel = -ewvel; }
    if (supersonic) ewvel /= 4;
    e_ew |= ((int)(ewvel + 1.5)) & 0x3FF;

    if (nsvel < 0) { e_ns = 0x400; nsvel = -nsvel; }
    if (supersonic) nsvel /= 4;
    e_ns |= ((int)(nsvel + 1.5)) & 0x3FF;

    if (vrate < 0) { e_vr = 0x200; vrate = -vrate; }
    e_vr |= ((int)(vrate / 64 + 1.5)) & 0x1FF;

    frame[0] = (18 << 3) | 2;
    frame[1] = (addr >> 16) & 0xFF;
    frame[2] = (addr >> 8) & 0xFF;
    frame[3] = addr & 0xFF;

    frame[4] = (19 << 3) | (supersonic ? 2 : 1);
    frame[5] = (e_ew >> 8) & 0x07;
    frame[6] = e_ew & 0xFF;
    frame[7] = (e_ns >> 3) & 0xFF;
    frame[8] = ((e_ns & 0x07) << 5) | 0x10 | ((e_vr >> 6) & 0x0F);
    frame[9] = ((e_vr & 0x3F) << 2);
    frame[10] = 0;

    uint32_t crc = modesChecksum(frame, 112);
    frame[11] = (crc >> 16) & 0xFF;
    frame[12] = (crc >> 8) & 0xFF;
    frame[13] = crc & 0xFF;
}

static void fa_inject_beast_message(const uint8_t *frame, int len) {
    struct modesMessage mm;
    memset(&mm, 0, sizeof(mm));

    mm.timestampMsg = 0xFF004D4C4154ULL;  // MAGIC_MLAT_TIMESTAMP
    mm.sysTimestampMsg = mstime();
    mm.remote = 1;
    mm.signalLevel = 0;
    mm.msgbits = len * 8;

    memcpy(mm.msg, frame, len);
    memcpy(mm.verbatim, frame, len);

    int result = decodeModesMessage(&mm, frame);
    if (result < 0)
        return;

    msg_queue_push(mlat_inject_queue, &mm);
}

// ============================= Public API ================================

void faMlatInit(void) {
    memset(&FaMlat, 0, sizeof(FaMlat));
    pthread_mutex_init(&FaMlat.ctl_mutex, NULL);
    pthread_mutex_init(&FaMlat.status_mutex, NULL);
}

void faMlatEnable(const char *host, int port, uint32_t key) {
    pthread_mutex_lock(&FaMlat.ctl_mutex);

    // If already running with same config, skip
    if (FaMlat.thread_running &&
        strcmp(FaMlat.udp_host, host) == 0 &&
        FaMlat.udp_port == port &&
        FaMlat.udp_key == key) {
        pthread_mutex_unlock(&FaMlat.ctl_mutex);
        return;
    }

    // Stop existing thread if running
    if (FaMlat.thread_running) {
        FaMlat.stop_requested = 1;
        pthread_mutex_unlock(&FaMlat.ctl_mutex);
        join_thread(FaMlat.thread, NULL, 5000);
        pthread_mutex_lock(&FaMlat.ctl_mutex);
    }

    strncpy(FaMlat.udp_host, host, sizeof(FaMlat.udp_host) - 1);
    FaMlat.udp_host[sizeof(FaMlat.udp_host) - 1] = 0;
    FaMlat.udp_port = port;
    FaMlat.udp_key = key;
    FaMlat.enabled = 1;
    FaMlat.stop_requested = 0;
    FaMlat.wanted_count = 0;
    FaMlat.wanted_modeac_count = 0;
    FaMlat.status_head = 0;
    FaMlat.status_tail = 0;
    FaMlat.thread_running = 1;

    pthread_mutex_unlock(&FaMlat.ctl_mutex);

    // Initialize the fa_mlat_queue
    msg_queue_clear(fa_mlat_queue);

    if (pthread_create(&FaMlat.thread, NULL, fa_mlat_thread_entry, NULL) != 0) {
        fprintf(stderr, "FA-MLAT: failed to create thread: %s\n", strerror(errno));
        FaMlat.thread_running = 0;
        return;
    }

    fprintf(stderr, "FA-MLAT: thread started (udp %s:%d key=%u)\n", host, port, key);
}

void faMlatDisable(void) {
    pthread_mutex_lock(&FaMlat.ctl_mutex);
    if (!FaMlat.thread_running) {
        FaMlat.enabled = 0;
        pthread_mutex_unlock(&FaMlat.ctl_mutex);
        return;
    }

    FaMlat.stop_requested = 1;
    FaMlat.enabled = 0;
    pthread_mutex_unlock(&FaMlat.ctl_mutex);

    join_thread(FaMlat.thread, NULL, 5000);
    fprintf(stderr, "FA-MLAT: disabled\n");
}

void faMlatStartSending(const uint32_t *icao, int count,
                         const uint32_t *modeac, int modeac_count) {
    pthread_mutex_lock(&FaMlat.ctl_mutex);

    // Add ICAOs to wanted set (avoid duplicates)
    for (int i = 0; i < count; i++) {
        int found = 0;
        for (int j = 0; j < FaMlat.wanted_count; j++) {
            if (FaMlat.wanted_icao[j] == icao[i]) { found = 1; break; }
        }
        if (!found && FaMlat.wanted_count < FA_MLAT_MAX_WANTED)
            FaMlat.wanted_icao[FaMlat.wanted_count++] = icao[i];
    }

    // Add Mode A/C codes
    for (int i = 0; i < modeac_count; i++) {
        int found = 0;
        for (int j = 0; j < FaMlat.wanted_modeac_count; j++) {
            if (FaMlat.wanted_modeac[j] == modeac[i]) { found = 1; break; }
        }
        if (!found && FaMlat.wanted_modeac_count < FA_MLAT_MAX_MODEAC)
            FaMlat.wanted_modeac[FaMlat.wanted_modeac_count++] = modeac[i];
    }

    pthread_mutex_unlock(&FaMlat.ctl_mutex);
}

void faMlatStopSending(const uint32_t *icao, int count,
                        const uint32_t *modeac, int modeac_count) {
    pthread_mutex_lock(&FaMlat.ctl_mutex);

    // Remove ICAOs from wanted set
    for (int i = 0; i < count; i++) {
        for (int j = 0; j < FaMlat.wanted_count; j++) {
            if (FaMlat.wanted_icao[j] == icao[i]) {
                FaMlat.wanted_icao[j] = FaMlat.wanted_icao[--FaMlat.wanted_count];
                break;
            }
        }
    }

    // Remove Mode A/C codes
    for (int i = 0; i < modeac_count; i++) {
        for (int j = 0; j < FaMlat.wanted_modeac_count; j++) {
            if (FaMlat.wanted_modeac[j] == modeac[i]) {
                FaMlat.wanted_modeac[j] = FaMlat.wanted_modeac[--FaMlat.wanted_modeac_count];
                break;
            }
        }
    }

    pthread_mutex_unlock(&FaMlat.ctl_mutex);
}

int faMlatPollStatus(char *buf, int bufsize) {
    pthread_mutex_lock(&FaMlat.status_mutex);

    if (FaMlat.status_head == FaMlat.status_tail) {
        pthread_mutex_unlock(&FaMlat.status_mutex);
        return 0;
    }

    strncpy(buf, FaMlat.status_lines[FaMlat.status_tail], bufsize - 1);
    buf[bufsize - 1] = 0;
    FaMlat.status_tail = (FaMlat.status_tail + 1) % FA_MLAT_STATUS_LINES;

    pthread_mutex_unlock(&FaMlat.status_mutex);
    return 1;
}

void faMlatInjectResult(uint32_t addr, double lat, double lon, double alt,
                         double nsvel, double ewvel, double vrate,
                         int anon, int modeac_flag) {
    (void)anon;
    (void)modeac_flag;

    if (addr == 0) return;

    double alt_ft = alt;  // FA sends altitude in feet

    // Generate synthetic DF18 position frames (even + odd)
    int ealt = fa_encode_altitude(alt_ft);
    int elat_even, elon_even, elat_odd, elon_odd;
    fa_cpr_encode(lat, lon, 0, &elat_even, &elon_even);
    fa_cpr_encode(lat, lon, 1, &elat_odd, &elon_odd);

    uint8_t even_frame[14], odd_frame[14];
    fa_build_position_frame(even_frame, addr, elat_even, elon_even, ealt, 0);
    fa_build_position_frame(odd_frame, addr, elat_odd, elon_odd, ealt, 1);

    fa_inject_beast_message(even_frame, 14);
    fa_inject_beast_message(odd_frame, 14);

    // Inject velocity if available
    if (nsvel != 0 || ewvel != 0 || vrate != 0) {
        uint8_t vel_frame[14];
        fa_build_velocity_frame(vel_frame, addr, nsvel, ewvel, vrate);
        fa_inject_beast_message(vel_frame, 14);
    }
}

void faMlatCleanup(void) {
    faMlatDisable();
    pthread_mutex_destroy(&FaMlat.ctl_mutex);
    pthread_mutex_destroy(&FaMlat.status_mutex);
}
