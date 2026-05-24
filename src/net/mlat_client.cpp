// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// mlat_client.c: built-in MLAT (multilateration) client
//
// Implements the JSON-over-TCP protocol used by mlat-server.
// Protocol: newline-delimited JSON over TCP (no compression).
// Supports up to MAX_MLAT_SERVERS independent server connections.
//
// Reference: mlat-client by Oliver Jowett (GPL-3+)
// Server: https://github.com/adsbexchange/mlat-server
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "dump1090.h"
#include "mlat_client.h"
#include "feeder_thread.h"
#include "crc.h"

#include <sys/socket.h>
#include <netdb.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string_view>
#include <stdarg.h>
#include <math.h>
#include <time.h>

// ============================= Globals ===================================

struct mlat_config MlatConfig;

// ============================= Forward Declarations ======================

static void mlat_server_connect(struct mlat_server *s);
static void mlat_server_disconnect(struct mlat_server *s, const char *reason);
static void mlat_server_send_handshake(struct mlat_server *s);
static void mlat_server_try_read(struct mlat_server *s);
static void mlat_server_try_write(struct mlat_server *s);
static void mlat_server_process_line(struct mlat_server *s, const char *line, int len);
static void mlat_server_handle_handshake(struct mlat_server *s, const char *line, int len);
static void mlat_server_handle_request(struct mlat_server *s, const char *line, int len);
static void mlat_server_heartbeat(struct mlat_server *s);
static void mlat_server_update_aircraft(struct mlat_server *s);

static struct mlat_aircraft *mlat_find_aircraft(uint32_t addr);
static struct mlat_aircraft *mlat_get_aircraft(uint32_t addr);
static void mlat_expire_aircraft(uint64_t now);

static void mlat_send_mlat_message(struct mlat_server *s, struct modesMessage *mm);
static void mlat_send_sync(struct mlat_server *s, struct mlat_aircraft *ac);
static void mlat_send_split_sync(struct mlat_server *s, struct modesMessage *mm);
static void mlat_send_seen(struct mlat_server *s);
static void mlat_send_rate_report(struct mlat_server *s, uint64_t now);

static void mlat_inject_result(const char *line, int len);
static void mlat_build_position_frame(uint8_t *frame, uint32_t addr, int elat, int elon, int ealt, int oddflag);
static void mlat_build_velocity_frame(uint8_t *frame, uint32_t addr, double nsvel, double ewvel, double vrate);
static void mlat_inject_beast_message(const uint8_t *frame, int len);

// CPR encoding for synthetic frames
static int cpr_NL(double lat);
static void cpr_encode(double lat, double lon, int odd, int *rlat, int *rlon);
static int encode_altitude(double ft);

// Simple JSON helpers (no external library needed)
static int json_find_key(const char *json, int len, const char *key, const char **val_start, int *val_len);
static int json_find_string(const char *json, int len, const char *key, char *out, int outsize);
static int json_find_double_array(const char *json, int len, const char *key, double *out, int maxcount);
static double json_find_number(const char *json, int len, const char *key, double defval);
static int json_find_bool(const char *json, int len, const char *key, int defval);
static int json_find_string_array(const char *json, int len, const char *key, uint32_t *icao_out, int maxcount);

// Buffer helpers
static int mlat_buf_append(struct mlat_server *s, const char *data, int len);
static int mlat_buf_printf(struct mlat_server *s, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

// ECEF to LLH conversion
static void ecef_to_llh(double x, double y, double z, double *lat, double *lon, double *alt);

// ============================= Backoff helpers ============================

static uint64_t mlat_reconnect_delay_ms(int reconnect_count) {
    uint64_t delay = MLAT_RECONNECT_INITIAL_MS;
    if (reconnect_count <= 1) return delay;
    for (int i = 1; i < reconnect_count; i++) {
        if (delay >= MLAT_RECONNECT_MAX_MS / 2) { delay = MLAT_RECONNECT_MAX_MS; break; }
        delay *= 2;
    }
    return delay;
}

static void mlat_format_delay(uint64_t delay_ms, char *buf, size_t buf_len) {
    if (delay_ms >= 60ULL * 60ULL * 1000ULL && delay_ms % (60ULL * 60ULL * 1000ULL) == 0)
        snprintf(buf, buf_len, "%lluh", (unsigned long long)(delay_ms / (60ULL * 60ULL * 1000ULL)));
    else if (delay_ms >= 60ULL * 1000ULL && delay_ms % (60ULL * 1000ULL) == 0)
        snprintf(buf, buf_len, "%llum", (unsigned long long)(delay_ms / (60ULL * 1000ULL)));
    else
        snprintf(buf, buf_len, "%llus", (unsigned long long)(delay_ms / 1000ULL));
}

static void mlat_reset_backoff(struct mlat_server *s) {
    s->reconnect_count = 0;
    s->max_backoff_count = 0;
}

static uint64_t mlat_schedule_reconnect(struct mlat_server *s) {
    s->reconnect_count++;
    uint64_t delay = mlat_reconnect_delay_ms(s->reconnect_count);

    if (delay >= MLAT_RECONNECT_MAX_MS) {
        s->max_backoff_count++;
        if (s->max_backoff_count >= MLAT_MAX_48H_RETRIES) {
            s->disabled_by_backoff = true;
            s->next_reconnect = 0;
            fprintf(stderr, "MLAT[%s:%d]: disabled after %d retries at 48h backoff\n",
                    s->host, s->port, s->max_backoff_count);
            if (PanelState.enabled)
                panelLog("MLAT[%s:%d]: disabled after %d retries at 48h backoff",
                         s->host, s->port, s->max_backoff_count);
            return 0;
        }
    } else {
        s->max_backoff_count = 0;
    }

    s->next_reconnect = mstime() + delay;
    return delay;
}

// ============================= Initialization ============================

void mlatClientInit(void)
{
    // NOTE: Do NOT memset MlatConfig here — it was already zero-initialized
    // as a global, and CLI argument parsing has already populated
    // server_count, servers[], user, uuid_file, lat/lon/alt, etc.

    // Read UUID from file if configured
    if (MlatConfig.uuid_file) {
        FILE *f = fopen(MlatConfig.uuid_file, "r");
        if (f) {
            char raw[128];
            if (fgets(raw, sizeof(raw), f)) {
                std::string buf(raw);
                while (!buf.empty() && (buf.back() == '\n' || buf.back() == '\r'))
                    buf.pop_back();
                if (!buf.empty()) {
                    MlatConfig.uuid = strdup(buf.c_str());
                }
            }
            fclose(f);
        }
    }

    // Use receiver position if MLAT position not explicitly set
    if (!MlatConfig.position_set && (Modes.bUserFlags & MODES_USER_LATLON_VALID)) {
        MlatConfig.lat = Modes.fUserLat;
        MlatConfig.lon = Modes.fUserLon;
        MlatConfig.position_set = true;
    }

    // Initialize server connections
    for (int i = 0; i < MlatConfig.server_count; i++) {
        struct mlat_server *s = &MlatConfig.servers[i];
        s->fd = -1;
        s->state = MLAT_DISCONNECTED;
        s->index = i;
        s->peer_index = -1;
        s->readbuf_len = 0;
        s->writebuf_len = 0;
        s->next_reconnect = mstime();  // connect immediately
        s->report_counter = 0;
        s->reconnect_count = 0;
        s->max_backoff_count = 0;
        s->disabled_by_backoff = false;
    }

    // Detect mutual-exclusive MLAT server pairs (shared backend)
    // adsb.one and airplanes.live share the same wiedehopf/mlat-server;
    // connecting both causes the server to kick one when the other connects.
    static const struct { const char *a; const char *b; } mlat_peers[] = {
        { "feed.adsb.one",      "feed.airplanes.live" },
        { NULL, NULL }
    };

    for (int p = 0; mlat_peers[p].a; p++) {
        int idx_a = -1, idx_b = -1;
        for (int i = 0; i < MlatConfig.server_count; i++) {
            if (MlatConfig.servers[i].host) {
                if (strstr(MlatConfig.servers[i].host, mlat_peers[p].a)) idx_a = i;
                if (strstr(MlatConfig.servers[i].host, mlat_peers[p].b)) idx_b = i;
            }
        }
        if (idx_a >= 0 && idx_b >= 0) {
            MlatConfig.servers[idx_a].peer_index = idx_b;
            MlatConfig.servers[idx_b].peer_index = idx_a;
            fprintf(stderr,
                "MLAT: WARNING — %s and %s share the same backend server.\n"
                "MLAT: They will take turns disconnecting each other. Only one will stay connected.\n",
                mlat_peers[p].a, mlat_peers[p].b);
            if (PanelState.enabled)
                panelLog("MLAT: %s e %s condividono lo stesso backend, solo uno resterà connesso",
                         mlat_peers[p].a, mlat_peers[p].b);
        }
    }

    if (MlatConfig.server_count > 0) {
        fprintf(stderr, "MLAT client: %d server(s) configured, user=%s\n",
                MlatConfig.server_count,
                MlatConfig.user ? MlatConfig.user : "(not set)");
    }
}

int mlatClientAddServer(const char *hostport)
{
    if (MlatConfig.server_count >= MAX_MLAT_SERVERS)
        return -1;

    // First server added: set defaults
    if (MlatConfig.server_count == 0) {
        MlatConfig.return_results = true;
    }

    struct mlat_server *s = &MlatConfig.servers[MlatConfig.server_count];
    *s = {};
    s->fd = -1;

    // Parse host:port
    std::string copy(hostport);
    size_t colon_pos = copy.rfind(':');
    if (colon_pos != std::string::npos) {
        s->host = strdup(copy.substr(0, colon_pos).c_str());
        s->port = atoi(copy.c_str() + colon_pos + 1);
    } else {
        s->host = strdup(copy.c_str());
        s->port = 31090;  // default MLAT port
    }

    if (s->port <= 0 || s->port > 65535) {
        fprintf(stderr, "MLAT: invalid port in '%s'\n", hostport);
        free(s->host);
        return -1;
    }

    MlatConfig.server_count++;
    return 0;
}

void mlatClientCleanup(void)
{
    for (int i = 0; i < MlatConfig.server_count; i++) {
        mlat_server_disconnect(&MlatConfig.servers[i], "shutdown");
        free(MlatConfig.servers[i].host);
    }
    free(MlatConfig.user);
    free(MlatConfig.uuid);
    free(MlatConfig.uuid_file);
}

void mlatClientDisconnectAll(const char *reason)
{
    for (int i = 0; i < MlatConfig.server_count; i++) {
        if (MlatConfig.servers[i].state != MLAT_DISCONNECTED)
            mlat_server_disconnect(&MlatConfig.servers[i], reason);
    }
}

// ============================= Periodic Work =============================

void mlatClientPeriodicWork(void)
{
    uint64_t now = mstime();

    for (int i = 0; i < MlatConfig.server_count; i++) {
        struct mlat_server *s = &MlatConfig.servers[i];

        if (s->disabled_by_backoff) continue;

        switch (s->state) {
        case MLAT_DISCONNECTED:
            if (now >= s->next_reconnect) {
                // If this server has a mutual-exclusive peer that is connected,
                // don't reconnect — the backend will just kick us again.
                if (s->peer_index >= 0) {
                    struct mlat_server *peer = &MlatConfig.servers[s->peer_index];
                    if (peer->state == MLAT_READY || peer->state == MLAT_HANDSHAKING) {
                        // Peer is connected, defer reconnect
                        s->next_reconnect = now + 60000; // retry check in 60s
                        break;
                    }
                }
                mlat_server_connect(s);
            }
            break;

        case MLAT_CONNECTING: {
            // Check if non-blocking connect completed
            int err = 0;
            socklen_t errlen = sizeof(err);
            if (getsockopt(s->fd, SOL_SOCKET, SO_ERROR, &err, &errlen) < 0 || err != 0) {
                if (err == EINPROGRESS || err == EALREADY) {
                    break;  // still connecting
                }
                mlat_server_disconnect(s, "connect failed");
                break;
            }
            // Connected!
            s->state = MLAT_HANDSHAKING;
            s->last_data_received = now;
            mlat_server_send_handshake(s);
            break;
        }

        case MLAT_HANDSHAKING:
        case MLAT_READY:
            // Check inactivity
            if ((now - s->last_data_received) > MLAT_INACTIVITY_TIMEOUT) {
                mlat_server_disconnect(s, "inactivity timeout");
                break;
            }

            // Read incoming data
            mlat_server_try_read(s);
            if (s->state == MLAT_DISCONNECTED) break;

            // Heartbeat
            if (s->state == MLAT_READY && now >= s->next_heartbeat) {
                mlat_server_heartbeat(s);
                s->next_heartbeat = now + MLAT_HEARTBEAT_INTERVAL;
            }

            // Aircraft update + reporting (only when READY)
            if (s->state == MLAT_READY && now >= s->next_aircraft_update) {
                mlat_server_update_aircraft(s);
                s->next_aircraft_update = now + MLAT_UPDATE_INTERVAL;
            }

            // Flush write buffer
            mlat_server_try_write(s);
            break;
        }
    }

    // Expire old aircraft from MLAT tracker
    static uint64_t next_expire = 0;
    if (now >= next_expire) {
        mlat_expire_aircraft(now);
        next_expire = now + 30000;
    }
}

// ============================= Connection Management =====================

static void mlat_server_connect(struct mlat_server *s)
{
    struct addrinfo hints = {}, *res, *rp;

    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    std::string portstr = std::to_string(s->port);

    int gai = getaddrinfo(s->host, portstr.c_str(), &hints, &res);
    if (gai != 0) {
        uint64_t delay = mlat_schedule_reconnect(s);
        if (delay > 0) {
            char dbuf[32];
            mlat_format_delay(delay, dbuf, sizeof(dbuf));
            fprintf(stderr, "MLAT[%s:%d]: DNS resolve failed: %s, retry in %s\n", s->host, s->port, gai_strerror(gai), dbuf);
        }
        return;
    }

    int fd = -1;
    for (rp = res; rp; rp = rp->ai_next) {
        fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (fd < 0) continue;

        // Set non-blocking
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0) {
            break;  // connected immediately
        }
        if (errno == EINPROGRESS) {
            break;  // connection in progress
        }
        close(fd);
        fd = -1;
    }
    freeaddrinfo(res);

    if (fd < 0) {
        uint64_t delay = mlat_schedule_reconnect(s);
        if (delay > 0) {
            char dbuf[32];
            mlat_format_delay(delay, dbuf, sizeof(dbuf));
            fprintf(stderr, "MLAT[%s:%d]: connect failed, retry in %s\n", s->host, s->port, dbuf);
        }
        return;
    }

    s->fd = fd;
    s->state = MLAT_CONNECTING;
    s->readbuf_len = 0;
    s->writebuf_len = 0;
    s->report_counter = 0;
    s->split_sync = false;

    // Check if connect already completed (local connections)
    int err = 0;
    socklen_t errlen = sizeof(err);
    getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &errlen);
    if (err == 0) {
        s->state = MLAT_HANDSHAKING;
        s->last_data_received = mstime();
        fprintf(stderr, "MLAT[%s:%d]: connected\n", s->host, s->port);
        mlat_server_send_handshake(s);
    }
}

static void mlat_server_disconnect(struct mlat_server *s, const char *reason)
{
    if (s->fd >= 0) {
        close(s->fd);
        s->fd = -1;
    }

    if (s->state != MLAT_DISCONNECTED) {
        fprintf(stderr, "MLAT[%s:%d]: disconnected (%s)\n", s->host, s->port, reason);
        if (PanelState.enabled)
            panelLog("MLAT[%s:%d]: disconnected (%s)", s->host, s->port, reason);
    }

    s->state = MLAT_DISCONNECTED;
    s->readbuf_len = 0;
    s->writebuf_len = 0;

    {
        uint64_t delay = mlat_schedule_reconnect(s);
        if (delay > 0) {
            char dbuf[32];
            mlat_format_delay(delay, dbuf, sizeof(dbuf));
            fprintf(stderr, "MLAT[%s:%d]: retry in %s\n", s->host, s->port, dbuf);
        }
    }

    // If we have a mutual-exclusive peer that is disconnected, wake it up
    // so it can try to reconnect now that we're gone.
    if (s->peer_index >= 0) {
        struct mlat_server *peer = &MlatConfig.servers[s->peer_index];
        if (peer->state == MLAT_DISCONNECTED && peer->next_reconnect > mstime()) {
            peer->next_reconnect = mstime();  // reconnect immediately
        }
    }

    // Clear requested flags for this server
    uint32_t mask = ~(1U << s->index);
    for (int i = 0; i < MLAT_HASH_SIZE; i++) {
        MlatConfig.aircraft[i].requested &= mask;
        MlatConfig.aircraft[i].reported &= mask;
    }
}

// ============================= Handshake =================================

static void mlat_server_send_handshake(struct mlat_server *s)
{
    // Build handshake JSON
    std::string hs = "{\"version\":3,"
        "\"client_version\":\"dump1090-mlat 1.0\","
        "\"compress\":[\"none\"],"
        "\"selective_traffic\":true,"
        "\"heartbeat\":true,"
        "\"return_results\":" + std::string(MlatConfig.return_results ? "true" : "false") + ","
        "\"return_result_format\":\"ecef\","
        "\"return_stats\":true";

    if (MlatConfig.user) {
        hs += ",\"user\":\"" + std::string(MlatConfig.user) + "\"";
    }
    if (MlatConfig.uuid) {
        hs += ",\"uuid\":\"" + std::string(MlatConfig.uuid) + "\"";
    }
    if (MlatConfig.position_set) {
        char pos_tmp[128];
        snprintf(pos_tmp, sizeof(pos_tmp),
            ",\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,\"altref\":\"egm96_meters\"",
            MlatConfig.lat, MlatConfig.lon, MlatConfig.alt);
        hs += pos_tmp;
    }

    hs += ",\"clock_type\":\"dump1090\""
         ",\"frequency\":12000000"
         ",\"epoch\":\"none\""
         "}";

    // Pad with spaces (protocol requires padding for initial handshake)
    hs.append(128, ' ');
    hs += '\n';

    mlat_buf_append(s, hs.data(), (int)hs.size());
    mlat_server_try_write(s);
}

// ============================= I/O =======================================

static void mlat_server_try_read(struct mlat_server *s)
{
    if (s->fd < 0) return;

    int space = MLAT_READ_BUF_SIZE - s->readbuf_len - 1;
    if (space <= 0) {
        mlat_server_disconnect(s, "read buffer overflow");
        return;
    }

    int nread = read(s->fd, s->readbuf + s->readbuf_len, space);
    if (nread < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return;
        std::string errmsg = std::string("read error: ") + strerror(errno) +
                             " (state=" + std::to_string(s->state) + ")";
        mlat_server_disconnect(s, errmsg.c_str());
        return;
    }
    if (nread == 0) {
        mlat_server_disconnect(s, "connection closed by server");
        return;
    }

    s->last_data_received = mstime();
    s->readbuf_len += nread;
    s->readbuf[s->readbuf_len] = 0;

    // Process complete lines
    char *start = s->readbuf;
    char *nl;
    while ((nl = (char*)memchr(start, '\n', s->readbuf_len - (start - s->readbuf))) != NULL) {
        int linelen = nl - start;
        if (linelen > 0) {
            mlat_server_process_line(s, start, linelen);
            if (s->state == MLAT_DISCONNECTED) return;
        }
        start = nl + 1;
    }

    // Move remaining data to front of buffer
    int remaining = s->readbuf_len - (start - s->readbuf);
    if (remaining > 0 && start != s->readbuf) {
        memmove(s->readbuf, start, remaining);
    }
    s->readbuf_len = remaining;
}

static void mlat_server_try_write(struct mlat_server *s)
{
    if (s->fd < 0 || s->writebuf_len == 0) return;

    int nwritten = write(s->fd, s->writebuf, s->writebuf_len);
    if (nwritten < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return;
        mlat_server_disconnect(s, "write error");
        return;
    }

    if (nwritten > 0) {
        if (nwritten < s->writebuf_len) {
            memmove(s->writebuf, s->writebuf + nwritten, s->writebuf_len - nwritten);
        }
        s->writebuf_len -= nwritten;
    }
}

static void mlat_server_process_line(struct mlat_server *s, const char *line, int len)
{
    if (s->state == MLAT_HANDSHAKING) {
        mlat_server_handle_handshake(s, line, len);
    } else if (s->state == MLAT_READY) {
        mlat_server_handle_request(s, line, len);
    }
}

// ============================= Server Messages ===========================

static void mlat_server_handle_handshake(struct mlat_server *s, const char *line, int len)
{
    // Check for denial
    const char *val;
    int vlen;
    if (json_find_key(line, len, "deny", &val, &vlen)) {
        fprintf(stderr, "MLAT[%s:%d]: server rejected connection\n", s->host, s->port);
        mlat_server_disconnect(s, "server denied connection");
        return;
    }

    // Check compression
    char compress[32];
    json_find_string(line, len, "compress", compress, sizeof(compress));
    if (compress[0] && std::string_view(compress) != "none") {
        fprintf(stderr, "MLAT[%s:%d]: server requested unsupported compression '%s'\n",
                s->host, s->port, compress);
        mlat_server_disconnect(s, "unsupported compression");
        return;
    }

    // Check split_sync preference
    s->split_sync = json_find_bool(line, len, "split_sync", 0);

    // Check for motd — clean up newlines and excess whitespace for display
    char motd_raw[256];
    if (json_find_string(line, len, "motd", motd_raw, sizeof(motd_raw)) && motd_raw[0]) {
        // Replace \n and collapse whitespace for readable single-line display
        std::string clean;
        clean.reserve(256);
        bool had_space = true;
        for (int mi = 0; motd_raw[mi]; mi++) {
            char ch = motd_raw[mi];
            if (ch == '\n' || ch == '\r' || ch == '\t') ch = ' ';
            if (ch == ' ') {
                if (!had_space) { clean += ' '; had_space = true; }
            } else {
                clean += ch;
                had_space = false;
            }
        }
        while (!clean.empty() && clean.back() == ' ') clean.pop_back();
        fprintf(stderr, "MLAT[%s:%d]: server says: %s\n", s->host, s->port, clean.c_str());
        if (PanelState.enabled)
            panelLog("MLAT[%s]: %s", s->host, clean.c_str());
    }

    // Handshake complete
    s->state = MLAT_READY;
    mlat_reset_backoff(s);
    s->next_heartbeat = mstime() + MLAT_HEARTBEAT_INTERVAL;
    s->next_aircraft_update = mstime() + MLAT_UPDATE_INTERVAL;

    fprintf(stderr, "MLAT[%s:%d]: handshake complete (split_sync=%s)\n",
            s->host, s->port, s->split_sync ? "yes" : "no");
    if (PanelState.enabled)
        panelLog("MLAT[%s:%d]: connected", s->host, s->port);

    // Tell server we're connected to input
    mlat_buf_printf(s, "{\"input_connected\":\"connected\"}\n");

    // Send initial clock info
    mlat_buf_printf(s, "{\"clock_reset\":{\"reason\":\"New connection\",\"frequency\":12000000,\"epoch\":\"none\",\"mode\":\"Beast\"}}\n");

    // Send empty rate report
    mlat_buf_printf(s, "{\"rate_report\":{}}\n");
}

static void mlat_server_handle_request(struct mlat_server *s, const char *line, int len)
{
    const char *val;
    int vlen;
    uint32_t mask = 1U << s->index;

    if (json_find_key(line, len, "start_sending", &val, &vlen)) {
        // Parse array of hex ICAO addresses
        uint32_t icaos[512];
        int count = json_find_string_array(line, len, "start_sending", icaos, 512);
        for (int i = 0; i < count; i++) {
            struct mlat_aircraft *ac = mlat_get_aircraft(icaos[i]);
            if (ac) {
                ac->requested |= mask;
            }
        }
    }
    else if (json_find_key(line, len, "stop_sending", &val, &vlen)) {
        uint32_t icaos[512];
        int count = json_find_string_array(line, len, "stop_sending", icaos, 512);
        uint32_t imask = ~mask;
        for (int i = 0; i < count; i++) {
            struct mlat_aircraft *ac = mlat_find_aircraft(icaos[i]);
            if (ac) {
                ac->requested &= imask;
            }
        }
    }
    else if (json_find_key(line, len, "result", &val, &vlen)) {
        mlat_inject_result(line, len);
    }
    else if (json_find_key(line, len, "heartbeat", &val, &vlen)) {
        // Server heartbeat, just update last_data_received (already done)
    }
    else if (json_find_key(line, len, "stats", &val, &vlen)) {
        // Server stats, ignore
    }
}

// ============================= Heartbeat =================================

static void mlat_server_heartbeat(struct mlat_server *s)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    double client_time = ts.tv_sec + ts.tv_nsec / 1e9;
    mlat_buf_printf(s, "{\"heartbeat\":{\"client_time\":%.3f}}\n", client_time);
}

// ============================= Aircraft Tracking =========================

static struct mlat_aircraft *mlat_find_aircraft(uint32_t addr)
{
    if (addr == 0) return NULL;
    uint32_t hash = addr & MLAT_HASH_MASK;
    for (int i = 0; i < 16; i++) {  // linear probing, max 16 steps
        uint32_t idx = (hash + i) & MLAT_HASH_MASK;
        if (MlatConfig.aircraft[idx].addr == addr)
            return &MlatConfig.aircraft[idx];
        if (MlatConfig.aircraft[idx].addr == 0)
            return NULL;
    }
    return NULL;
}

static struct mlat_aircraft *mlat_get_aircraft(uint32_t addr)
{
    if (addr == 0) return NULL;
    uint32_t hash = addr & MLAT_HASH_MASK;
    struct mlat_aircraft *empty = NULL;

    for (int i = 0; i < 16; i++) {
        uint32_t idx = (hash + i) & MLAT_HASH_MASK;
        if (MlatConfig.aircraft[idx].addr == addr)
            return &MlatConfig.aircraft[idx];
        if (MlatConfig.aircraft[idx].addr == 0) {
            if (!empty) empty = &MlatConfig.aircraft[idx];
            break;
        }
    }

    if (empty) {
        *empty = {};
        empty->addr = addr;
        empty->rate_measurement_start = mstime();
        return empty;
    }
    return NULL;  // hash table full in this neighborhood
}

static void mlat_expire_aircraft(uint64_t now)
{
    for (int i = 0; i < MLAT_HASH_SIZE; i++) {
        struct mlat_aircraft *ac = &MlatConfig.aircraft[i];
        if (ac->addr == 0) continue;
        if ((now - ac->last_message) > MLAT_AIRCRAFT_EXPIRY) {
            ac->addr = 0;  // mark slot as empty
        }
    }
}

// Aircraft list update: report seen/lost to server, send rate reports
static void mlat_server_update_aircraft(struct mlat_server *s)
{
    uint64_t now = mstime();
    (void)s;  // used indirectly via mask in mlat_send_seen

    // Update ADS-B good status and message counts from dump1090's tracker
    pthread_rwlock_rdlock(&aircraft_lock);
    for (struct aircraft *a = Modes.aircrafts; a; a = a->next) {
        if (!a->reliable) continue;

        struct mlat_aircraft *mac = mlat_get_aircraft(a->addr);
        if (!mac) continue;

        // Update message count and timestamp
        if (a->seen > mac->last_message) {
            mac->last_message = a->seen;
            mac->messages = a->messages;
        }

        // Check ADS-B position freshness
        if (a->cpr_even_valid.updated && (now - a->cpr_even_valid.updated) < MLAT_POSITION_EXPIRY) {
            mac->last_even_time = a->cpr_even_valid.updated;
        }
        if (a->cpr_odd_valid.updated && (now - a->cpr_odd_valid.updated) < MLAT_POSITION_EXPIRY) {
            mac->last_odd_time = a->cpr_odd_valid.updated;
        }

        mac->adsb_good = (mac->last_even_time > 0 &&
                          mac->last_odd_time > 0 &&
                          (now - mac->last_even_time) < MLAT_POSITION_EXPIRY &&
                          (now - mac->last_odd_time) < MLAT_POSITION_EXPIRY);
    }
    pthread_rwlock_unlock(&aircraft_lock);

    // Increment report counter
    s->report_counter++;
    if (s->report_counter < MLAT_REPORT_INTERVAL) return;
    s->report_counter = 0;

    // Send seen/lost reports
    mlat_send_seen(s);

    // Send rate report
    mlat_send_rate_report(s, now);

    MlatConfig.last_aircraft_update = now;
}

static void mlat_send_seen(struct mlat_server *s)
{
    uint32_t mask = 1U << s->index;

    // Build seen list (aircraft we see that server doesn't know about)
    std::string seen_list;
    int seen_count = 0;

    for (int i = 0; i < MLAT_HASH_SIZE; i++) {
        struct mlat_aircraft *ac = &MlatConfig.aircraft[i];
        if (ac->addr == 0) continue;
        if (ac->messages < 2) continue;

        bool is_reported = (ac->reported & mask) != 0;
        bool should_report = true;

        if (should_report && !is_reported) {
            char tmp[16];
            if (seen_count > 0) seen_list += ',';
            snprintf(tmp, sizeof(tmp), "\"%06x\"", ac->addr);
            seen_list += tmp;
            seen_count++;
            ac->reported |= mask;
        }
    }

    if (seen_count > 0) {
        std::string msg = "{\"seen\":[" + seen_list + "]}\n";
        mlat_buf_append(s, msg.data(), (int)msg.size());
    }
}

static void mlat_send_rate_report(struct mlat_server *s, uint64_t now)
{
    std::string rate_list;
    int count = 0;

    for (int i = 0; i < MLAT_HASH_SIZE; i++) {
        struct mlat_aircraft *ac = &MlatConfig.aircraft[i];
        if (ac->addr == 0) continue;
        if (ac->recent_adsb_positions == 0) continue;

        double interval = (now - ac->rate_measurement_start) / 1000.0;
        if (interval <= 0) continue;

        double rate = ac->recent_adsb_positions / interval;
        ac->rate_measurement_start = now;
        ac->recent_adsb_positions = 0;

        char tmp[32];
        if (count > 0) rate_list += ',';
        snprintf(tmp, sizeof(tmp), "\"%06X\":%.2f", ac->addr, rate);
        rate_list += tmp;
        count++;
    }

    if (count > 0) {
        std::string msg = "{\"rate_report\":{" + rate_list + "}}\n";
        mlat_buf_append(s, msg.data(), (int)msg.size());
    }
}

// ============================= Message Processing ========================

void mlatClientProcessMessage(struct modesMessage *mm)
{
    if (MlatConfig.server_count == 0) return;
    if (mm->source == SOURCE_MLAT) return;     // don't feed MLAT results back
    if (mm->remote) return;                     // only process local SDR messages
    if (mm->msgbits != 56 && mm->msgbits != 112) return;

    uint32_t addr = mm->addr;
    if (addr == 0) return;

    struct mlat_aircraft *ac = mlat_get_aircraft(addr);
    if (!ac) return;

    ac->last_message = mstime();
    ac->messages++;

    // Handle DF17 (ADS-B with position) - update sync data
    if (mm->msgtype == 17 && mm->reliable) {
        // Check for CPR even/odd position
        if (mm->cpr_valid) {
            if (mm->cpr_odd) {
                ac->odd_timestamp = mm->timestampMsg;
                memcpy(ac->odd_msg, mm->verbatim, (mm->msgbits + 7) / 8);
                ac->odd_msgbits = mm->msgbits;
                ac->last_odd_time = mstime();
            } else {
                ac->even_timestamp = mm->timestampMsg;
                memcpy(ac->even_msg, mm->verbatim, (mm->msgbits + 7) / 8);
                ac->even_msgbits = mm->msgbits;
                ac->last_even_time = mstime();
            }

            // Update ADS-B good status
            uint64_t now = mstime();
            ac->adsb_good = (ac->last_even_time > 0 &&
                             ac->last_odd_time > 0 &&
                             (now - ac->last_even_time) < MLAT_POSITION_EXPIRY &&
                             (now - ac->last_odd_time) < MLAT_POSITION_EXPIRY);

            if (mm->altitude_baro_valid) {
                ac->recent_adsb_positions++;
            }

            // Send sync message to servers that are ready
            if (ac->adsb_good) {
                // Check timestamp proximity: even and odd must be within 5 seconds of 12MHz clock
                uint64_t ts_diff;
                if (ac->even_timestamp > ac->odd_timestamp)
                    ts_diff = ac->even_timestamp - ac->odd_timestamp;
                else
                    ts_diff = ac->odd_timestamp - ac->even_timestamp;

                if (ts_diff < (uint64_t)5 * 12000000) {
                    for (int i = 0; i < MlatConfig.server_count; i++) {
                        struct mlat_server *s = &MlatConfig.servers[i];
                        if (s->state != MLAT_READY) continue;

                        if (s->split_sync) {
                            mlat_send_split_sync(s, mm);
                        } else {
                            mlat_send_sync(s, ac);
                        }
                    }
                }
            }
        }
        return;  // DF17 with position is sync-only, not MLAT candidate
    }

    // For DF0, 4, 5, 11, 16, 20, 21: potential MLAT candidate
    if (mm->msgtype != 0 && mm->msgtype != 4 && mm->msgtype != 5 &&
        mm->msgtype != 11 && mm->msgtype != 16 && mm->msgtype != 20 &&
        mm->msgtype != 21) {
        return;
    }

    if (ac->messages < MLAT_MIN_MESSAGES) return;

    // Only send MLAT data for aircraft that DON'T have good ADS-B position
    // (aircraft with ADS-B position don't need multilateration)
    if (ac->adsb_good) return;

    // Send to each server that has requested this aircraft
    for (int i = 0; i < MlatConfig.server_count; i++) {
        struct mlat_server *s = &MlatConfig.servers[i];
        if (s->state != MLAT_READY) continue;
        if (!(ac->requested & (1U << i))) continue;

        mlat_send_mlat_message(s, mm);
    }
}

// ============================= MLAT/Sync Message Sending =================

static void mlat_send_mlat_message(struct mlat_server *s, struct modesMessage *mm)
{
    // Format hex message
    static const char hexchars[] = "0123456789abcdef";
    int msgbytes = (mm->msgbits + 7) / 8;
    std::string hexmsg;
    hexmsg.reserve(msgbytes * 2);
    for (int i = 0; i < msgbytes; i++) {
        hexmsg += hexchars[mm->verbatim[i] >> 4];
        hexmsg += hexchars[mm->verbatim[i] & 0x0f];
    }

    mlat_buf_printf(s, "{\"mlat\":{\"t\":%" PRIu64 ",\"m\":\"%s\"}}\n",
                    (uint64_t)mm->timestampMsg, hexmsg.c_str());
}

static void mlat_send_sync(struct mlat_server *s, struct mlat_aircraft *ac)
{
    static const char hexchars[] = "0123456789abcdef";
    int even_bytes = (ac->even_msgbits + 7) / 8;
    int odd_bytes = (ac->odd_msgbits + 7) / 8;

    std::string even_hex, odd_hex;
    even_hex.reserve(even_bytes * 2);
    odd_hex.reserve(odd_bytes * 2);
    for (int i = 0; i < even_bytes; i++) {
        even_hex += hexchars[ac->even_msg[i] >> 4];
        even_hex += hexchars[ac->even_msg[i] & 0x0f];
    }
    for (int i = 0; i < odd_bytes; i++) {
        odd_hex += hexchars[ac->odd_msg[i] >> 4];
        odd_hex += hexchars[ac->odd_msg[i] & 0x0f];
    }

    mlat_buf_printf(s, "{\"sync\":{\"et\":%" PRIu64 ",\"em\":\"%s\",\"ot\":%" PRIu64 ",\"om\":\"%s\"}}\n",
                    (uint64_t)ac->even_timestamp, even_hex.c_str(),
                    (uint64_t)ac->odd_timestamp, odd_hex.c_str());
}

static void mlat_send_split_sync(struct mlat_server *s, struct modesMessage *mm)
{
    static const char hexchars[] = "0123456789abcdef";
    int msgbytes = (mm->msgbits + 7) / 8;
    std::string hexmsg;
    hexmsg.reserve(msgbytes * 2);
    for (int i = 0; i < msgbytes; i++) {
        hexmsg += hexchars[mm->verbatim[i] >> 4];
        hexmsg += hexchars[mm->verbatim[i] & 0x0f];
    }

    mlat_buf_printf(s, "{\"ssync\":{\"t\":%" PRIu64 ",\"m\":\"%s\"}}\n",
                    (uint64_t)mm->timestampMsg, hexmsg.c_str());
}

// ============================= Result Injection ==========================

// ECEF to geodetic (WGS84) conversion
// Uses Bowring's iterative method
static void ecef_to_llh(double x, double y, double z, double *lat, double *lon, double *alt)
{
    const double a = 6378137.0;          // WGS84 semi-major axis
    const double f = 1.0 / 298.257223563;
    const double b = a * (1.0 - f);
    const double e2 = 2 * f - f * f;     // first eccentricity squared
    const double ep2 = (a * a - b * b) / (b * b);  // second eccentricity squared

    double p = sqrt(x * x + y * y);
    *lon = atan2(y, x) * 180.0 / M_PI;

    // Bowring's method (converges in 2-3 iterations)
    double theta = atan2(z * a, p * b);
    double lat_rad = atan2(z + ep2 * b * pow(sin(theta), 3),
                           p - e2 * a * pow(cos(theta), 3));

    double sin_lat = sin(lat_rad);
    double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);

    *lat = lat_rad * 180.0 / M_PI;
    if (fabs(sin_lat) > 1e-10)
        *alt = z / sin_lat - N * (1.0 - e2);
    else
        *alt = p / cos(lat_rad) - N;
}

static void mlat_inject_result(const char *line, int len)
{
    const char *result_start;
    int result_len;

    if (!json_find_key(line, len, "result", &result_start, &result_len))
        return;

    // Parse address
    char addr_str[16];
    if (!json_find_string(result_start, result_len, "addr", addr_str, sizeof(addr_str)))
        return;
    uint32_t addr = (uint32_t)strtol(addr_str, NULL, 16);
    if (addr == 0) return;

    // Try ECEF format first (preferred)
    double ecef[3] = {0};
    double lat, lon, alt_m;
    int got_position = 0;

    if (json_find_double_array(result_start, result_len, "ecef", ecef, 3) == 3) {
        ecef_to_llh(ecef[0], ecef[1], ecef[2], &lat, &lon, &alt_m);
        got_position = 1;
    } else {
        // Fallback: direct lat/lon/alt
        lat = json_find_number(result_start, result_len, "lat", 999);
        lon = json_find_number(result_start, result_len, "lon", 999);
        double alt_ft = json_find_number(result_start, result_len, "alt", -9999);
        if (lat != 999 && lon != 999 && alt_ft != -9999) {
            alt_m = alt_ft * 0.3048;
            got_position = 1;
        }
    }

    if (!got_position) return;

    double alt_ft = alt_m / 0.3048;  // convert to feet for ADS-B frames

    // Parse velocity (optional)
    double nsvel = json_find_number(result_start, result_len, "nsvel", 0);
    double ewvel = json_find_number(result_start, result_len, "ewvel", 0);
    double vrate = json_find_number(result_start, result_len, "vrate", 0);

    // Generate synthetic DF18 position frames (even + odd)
    int ealt = encode_altitude(alt_ft);
    int elat_even, elon_even, elat_odd, elon_odd;
    cpr_encode(lat, lon, 0, &elat_even, &elon_even);
    cpr_encode(lat, lon, 1, &elat_odd, &elon_odd);

    uint8_t even_frame[14], odd_frame[14];
    mlat_build_position_frame(even_frame, addr, elat_even, elon_even, ealt, 0);
    mlat_build_position_frame(odd_frame, addr, elat_odd, elon_odd, ealt, 1);

    // Inject both position frames as Beast messages with MAGIC_MLAT_TIMESTAMP
    mlat_inject_beast_message(even_frame, 14);
    mlat_inject_beast_message(odd_frame, 14);

    // Inject velocity frame if we have velocity data
    if (nsvel != 0 || ewvel != 0 || vrate != 0) {
        uint8_t vel_frame[14];
        mlat_build_velocity_frame(vel_frame, addr, nsvel, ewvel, vrate);
        mlat_inject_beast_message(vel_frame, 14);
    }
}

// Build a synthetic DF18 airborne position frame
// metype=18: airborne position, baro alt, NUCp=0
static void mlat_build_position_frame(uint8_t *frame, uint32_t addr,
                                       int elat, int elon, int ealt, int oddflag)
{
    memset(frame, 0, 14);

    // DF=18, CF=2 (ES/NT, fine TIS-B message, ICAO address, IMF=0)
    frame[0] = (18 << 3) | 2;
    frame[1] = (addr >> 16) & 0xFF;
    frame[2] = (addr >> 8) & 0xFF;
    frame[3] = addr & 0xFF;

    // ME type 18: airborne position, baro alt, NUCp=0
    frame[4] = (18 << 3);           // TC=18, SAF/IMF=0
    frame[5] = (ealt >> 4) & 0xFF;  // Altitude MSB
    frame[6] = ((ealt & 0x0F) << 4);// Altitude LSB
    if (oddflag) frame[6] |= 0x04;  // CPR format (odd)
    frame[6] |= (elat >> 15) & 0x03;
    frame[7] = (elat >> 7) & 0xFF;
    frame[8] = ((elat & 0x7F) << 1) | ((elon >> 16) & 0x01);
    frame[9] = (elon >> 8) & 0xFF;
    frame[10] = elon & 0xFF;

    // Compute and append CRC
    uint32_t crc = modesChecksum(frame, 112);
    frame[11] = (crc >> 16) & 0xFF;
    frame[12] = (crc >> 8) & 0xFF;
    frame[13] = crc & 0xFF;
}

// Build a synthetic DF18 airborne velocity frame
static void mlat_build_velocity_frame(uint8_t *frame, uint32_t addr,
                                       double nsvel, double ewvel, double vrate)
{
    memset(frame, 0, 14);

    int supersonic = (fabs(nsvel) > 1000 || fabs(ewvel) > 1000);

    // Encode velocities
    int e_ew = 0, e_ns = 0, e_vr = 0;

    // E/W velocity
    if (ewvel < 0) { e_ew = 0x400; ewvel = -ewvel; }
    if (supersonic) ewvel /= 4;
    e_ew |= ((int)(ewvel + 1.5)) & 0x3FF;

    // N/S velocity
    if (nsvel < 0) { e_ns = 0x400; nsvel = -nsvel; }
    if (supersonic) nsvel /= 4;
    e_ns |= ((int)(nsvel + 1.5)) & 0x3FF;

    // Vertical rate
    if (vrate < 0) { e_vr = 0x200; vrate = -vrate; }
    e_vr |= ((int)(vrate / 64 + 1.5)) & 0x1FF;

    // DF=18, CF=2
    frame[0] = (18 << 3) | 2;
    frame[1] = (addr >> 16) & 0xFF;
    frame[2] = (addr >> 8) & 0xFF;
    frame[3] = addr & 0xFF;

    // ME type 19: airborne velocity
    frame[4] = (19 << 3) | (supersonic ? 2 : 1);  // subtype 1=subsonic, 2=supersonic
    frame[5] = (e_ew >> 8) & 0x07;                 // IMF=0, NACp=0, E/W sign+MSB
    frame[6] = e_ew & 0xFF;
    frame[7] = (e_ns >> 3) & 0xFF;
    frame[8] = ((e_ns & 0x07) << 5) | 0x10 | ((e_vr >> 6) & 0x0F);  // VR source=baro
    frame[9] = ((e_vr & 0x3F) << 2);
    frame[10] = 0;  // GNSS/baro offset: no data

    // CRC
    uint32_t crc = modesChecksum(frame, 112);
    frame[11] = (crc >> 16) & 0xFF;
    frame[12] = (crc >> 8) & 0xFF;
    frame[13] = crc & 0xFF;
}

// Inject a raw 14-byte Mode S frame as if it came from Beast input
// with the magic MLAT timestamp.
// When running in a feeder thread, we queue the message for the main thread
// to process (useModesMessage is not thread-safe for network output).
static void mlat_inject_beast_message(const uint8_t *frame, int len)
{
    // Create a modesMessage and decode it
    struct modesMessage mm = {};

    mm.timestampMsg = 0xFF004D4C4154ULL;  // MAGIC_MLAT_TIMESTAMP
    mm.sysTimestampMsg = mstime();
    mm.remote = 1;
    mm.signalLevel = 0;
    mm.msgbits = len * 8;

    memcpy(mm.msg, frame, len);
    memcpy(mm.verbatim, frame, len);

    // Decode the message (needs mm + separate msg pointer)
    int result = decodeModesMessage(&mm, frame);
    if (result < 0)
        return;

    // Queue for main thread processing instead of calling useModesMessage directly
    msg_queue_push(mlat_inject_queue, &mm);
}

// ============================= CPR Encoding ==============================

// NL table for CPR encoding (latitude → number of longitude zones)
static const struct { double lat; int nl; } cpr_nl_table[] = {
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

static int cpr_NL(double lat)
{
    if (lat < 0) lat = -lat;
    for (int i = 0; i < 59; i++) {
        if (lat < cpr_nl_table[i].lat)
            return cpr_nl_table[i].nl;
    }
    return 1;
}

static void cpr_encode(double lat, double lon, int odd, int *rlat, int *rlon)
{
    double NbPow = 131072.0;  // 2^17
    double Dlat = 360.0 / (odd ? 59 : 60);

    // Python's % is always positive for positive divisor; C's fmod can be negative
    double lat_mod = fmod(lat, Dlat);
    if (lat_mod < 0) lat_mod += Dlat;

    double yz = floor(NbPow * lat_mod / Dlat + 0.5);
    int YZ = ((int)yz) & 0x1FFFF;

    double Rlat = Dlat * (yz / NbPow + floor(lat / Dlat));
    int nl = cpr_NL(Rlat) - (odd ? 1 : 0);
    if (nl < 1) nl = 1;
    double Dlon = 360.0 / nl;

    double lon_mod = fmod(lon, Dlon);
    if (lon_mod < 0) lon_mod += Dlon;

    double xz = floor(NbPow * lon_mod / Dlon + 0.5);
    int XZ = ((int)xz) & 0x1FFFF;

    *rlat = YZ;
    *rlon = XZ;
}

static int encode_altitude(double ft)
{
    int i = (int)((ft + 1012.5) / 25.0);
    if (i < 0) i = 0;
    if (i > 0x7FF) i = 0x7FF;
    // Insert Q=1 in bit 4
    return ((i & 0x7F0) << 1) | 0x010 | (i & 0x00F);
}

// ============================= Simple JSON Parser ========================
//
// Minimal JSON parser for the MLAT server protocol.
// Only handles the specific JSON patterns used by mlat-server.
// NOT a general-purpose JSON parser.

// Find a key in a JSON object. Returns 1 if found, 0 if not.
// Sets val_start/val_len to point to the value (after the colon).
static int json_find_key(const char *json, int len, const char *key, const char **val_start, int *val_len)
{
    int keylen = strlen(key);
    const char *end = json + len;

    for (const char *p = json; p < end - keylen - 3; p++) {
        if (*p != '"') continue;
        if (memcmp(p + 1, key, keylen) == 0 && p[keylen + 1] == '"') {
            // Found key, skip to colon
            const char *q = p + keylen + 2;
            while (q < end && (*q == ' ' || *q == ':')) q++;
            if (q >= end) return 0;

            *val_start = q;

            // Determine value length
            if (*q == '"') {
                // String value
                const char *r = q + 1;
                while (r < end && *r != '"') {
                    if (*r == '\\') r++;  // skip escaped char
                    r++;
                }
                *val_len = (r < end) ? (r + 1 - q) : (end - q);
            } else if (*q == '[' || *q == '{') {
                // Array or object: find matching bracket
                char open = *q, close = (*q == '[') ? ']' : '}';
                int depth = 1;
                const char *r = q + 1;
                while (r < end && depth > 0) {
                    if (*r == open) depth++;
                    else if (*r == close) depth--;
                    else if (*r == '"') {
                        r++;
                        while (r < end && *r != '"') {
                            if (*r == '\\') r++;
                            r++;
                        }
                    }
                    r++;
                }
                *val_len = r - q;
            } else {
                // Number, bool, null
                const char *r = q;
                while (r < end && *r != ',' && *r != '}' && *r != ']' && *r != '\n')
                    r++;
                *val_len = r - q;
            }
            return 1;
        }
    }
    return 0;
}

// Extract a string value for a key. Returns 1 if found, 0 if not.
static int json_find_string(const char *json, int len, const char *key, char *out, int outsize)
{
    const char *val;
    int vlen;
    out[0] = 0;

    if (!json_find_key(json, len, key, &val, &vlen)) return 0;
    if (*val != '"') return 0;

    // Copy string content (between quotes), with JSON unescape
    int i = 0;
    const char *p = val + 1;
    const char *end = val + vlen;
    while (p < end && *p != '"' && i < outsize - 1) {
        if (*p == '\\' && p + 1 < end) {
            p++;
            switch (*p) {
                case 'n': out[i++] = '\n'; break;
                case 'r': out[i++] = '\r'; break;
                case 't': out[i++] = '\t'; break;
                case '"': out[i++] = '"'; break;
                case '\\': out[i++] = '\\'; break;
                default: out[i++] = *p; break;
            }
            p++;
        } else {
            out[i++] = *p++;
        }
    }
    out[i] = 0;
    return 1;
}

// Extract a number value for a key.
static double json_find_number(const char *json, int len, const char *key, double defval)
{
    const char *val;
    int vlen;

    if (!json_find_key(json, len, key, &val, &vlen)) return defval;
    return atof(val);
}

// Extract a boolean value for a key.
static int json_find_bool(const char *json, int len, const char *key, int defval)
{
    const char *val;
    int vlen;

    if (!json_find_key(json, len, key, &val, &vlen)) return defval;
    if (vlen >= 4 && memcmp(val, "true", 4) == 0) return 1;
    if (vlen >= 5 && memcmp(val, "false", 5) == 0) return 0;
    return defval;
}

// Extract an array of doubles.
static int json_find_double_array(const char *json, int len, const char *key, double *out, int maxcount)
{
    const char *val;
    int vlen;

    if (!json_find_key(json, len, key, &val, &vlen)) return 0;
    if (*val != '[') return 0;

    int count = 0;
    const char *p = val + 1;
    const char *end = val + vlen;

    while (p < end && count < maxcount) {
        while (p < end && (*p == ' ' || *p == ',')) p++;
        if (p >= end || *p == ']') break;
        out[count++] = atof(p);
        while (p < end && *p != ',' && *p != ']') p++;
    }
    return count;
}

// Parse an array of hex string ICAO addresses: ["aabbcc","ddeeff",...]
static int json_find_string_array(const char *json, int len, const char *key, uint32_t *icao_out, int maxcount)
{
    const char *val;
    int vlen;

    if (!json_find_key(json, len, key, &val, &vlen)) return 0;
    if (*val != '[') return 0;

    int count = 0;
    const char *p = val + 1;
    const char *end = val + vlen;

    while (p < end && count < maxcount) {
        while (p < end && *p != '"') {
            if (*p == ']') return count;
            p++;
        }
        if (p >= end) break;
        p++;  // skip opening quote

        // Parse hex
        uint32_t addr = 0;
        while (p < end && *p != '"') {
            char c = *p++;
            addr <<= 4;
            if (c >= '0' && c <= '9') addr |= (c - '0');
            else if (c >= 'a' && c <= 'f') addr |= (c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') addr |= (c - 'A' + 10);
        }
        if (p < end) p++;  // skip closing quote
        if (addr) icao_out[count++] = addr;
    }
    return count;
}

// ============================= Buffer Helpers ============================

static int mlat_buf_append(struct mlat_server *s, const char *data, int len)
{
    if (s->writebuf_len + len > MLAT_WRITE_BUF_SIZE) {
        fprintf(stderr, "MLAT[%s:%d]: write buffer overflow, dropping data\n", s->host, s->port);
        return -1;
    }
    memcpy(s->writebuf + s->writebuf_len, data, len);
    s->writebuf_len += len;
    return 0;
}

static int mlat_buf_printf(struct mlat_server *s, const char *fmt, ...)
{
    char buf[4096];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n < 0 || n >= (int)sizeof(buf)) return -1;
    return mlat_buf_append(s, buf, n);
}
