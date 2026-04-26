// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sondehub_client.c: SondeHub telemetry upload client
//
// Uploads decoded radiosonde telemetry to SondeHub v2 API:
//   PUT https://api.v2.sondehub.org/sondes/telemetry
//   PUT https://api.v2.sondehub.org/listeners
//
// Uses OpenSSL for TLS (already linked). Batches telemetry every 30 seconds.
// Only active when SondehubConfig.enabled && radiosonde receiver present.
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <sys/socket.h>
#include <netdb.h>
#include <fcntl.h>

#include <openssl/ssl.h>
#include <openssl/err.h>

#include "dump1090.h"

// ======================== Constants ========================

#define SONDEHUB_HOST       "api.v2.sondehub.org"
#define SONDEHUB_PORT       "443"
#define SONDEHUB_SW_NAME    MODES_DUMP1090_VARIANT
#define SONDEHUB_QUEUE_SIZE 64
#define SONDEHUB_UPLOAD_INTERVAL_MS  30000   // 30 seconds
#define SONDEHUB_LISTENER_INTERVAL_MS 600000 // 10 minutes
#define SONDEHUB_CONNECT_TIMEOUT     5       // seconds

// ======================== State ========================

sondehub_config_t SondehubConfig;

static struct {
    sonde_msg_t  queue[SONDEHUB_QUEUE_SIZE];
    unsigned     queue_head;
    unsigned     queue_tail;

    uint64_t     last_upload_ms;
    uint64_t     last_listener_ms;
    bool         listener_sent;
    unsigned     listener_fails;

    uint64_t     uploads_ok;
    uint64_t     uploads_fail;
    uint64_t     telemetry_sent;
} SH;

// ======================== Init / Cleanup ========================

void sondehubClientInit(void)
{
    memset(&SH, 0, sizeof(SH));
}

void sondehubClientCleanup(void)
{
    if (SH.telemetry_sent > 0 || SH.uploads_ok > 0) {
        fprintf(stderr, "sondehub: sent %lu telemetry in %lu uploads (%lu failed)\n",
                (unsigned long)SH.telemetry_sent,
                (unsigned long)SH.uploads_ok,
                (unsigned long)SH.uploads_fail);
    }
}

// ======================== Queue ========================

void sondehubClientSubmit(const sonde_msg_t *msg)
{
    if (!SondehubConfig.enabled || !msg->valid_pos) return;

    unsigned next = (SH.queue_head + 1) % SONDEHUB_QUEUE_SIZE;
    if (next == SH.queue_tail) return;  // full — drop oldest

    SH.queue[SH.queue_head] = *msg;
    SH.queue_head = next;
}

static unsigned queue_count(void)
{
    return (SH.queue_head - SH.queue_tail + SONDEHUB_QUEUE_SIZE) % SONDEHUB_QUEUE_SIZE;
}

// ======================== TLS HTTP PUT ========================

// Perform a single HTTPS PUT to SondeHub.
// Returns true on HTTP 2xx response.
static bool sondehub_put(const char *path, const char *json_body, size_t body_len)
{
    struct addrinfo hints, *res;
    int fd = -1;
    SSL_CTX *ctx = NULL;
    SSL *ssl = NULL;
    bool ok = false;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    int err = getaddrinfo(SONDEHUB_HOST, SONDEHUB_PORT, &hints, &res);
    if (err != 0) {
        fprintf(stderr, "sondehub: DNS failed: %s\n", gai_strerror(err));
        return false;
    }

    // Connect with timeout
    for (struct addrinfo *rp = res; rp; rp = rp->ai_next) {
        fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (fd < 0) continue;

        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0 || errno == EINPROGRESS) {
            fd_set wfds;
            struct timeval tv = {SONDEHUB_CONNECT_TIMEOUT, 0};
            FD_ZERO(&wfds);
            FD_SET(fd, &wfds);

            if (select(fd + 1, NULL, &wfds, NULL, &tv) > 0) {
                int so_error;
                socklen_t len = sizeof(so_error);
                getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &len);
                if (so_error == 0) {
                    fcntl(fd, F_SETFL, flags);  // restore blocking
                    goto connected;
                }
            }
        }
        close(fd);
        fd = -1;
    }
    freeaddrinfo(res);
    if (fd < 0) {
        fprintf(stderr, "sondehub: connect failed\n");
        return false;
    }

connected:
    freeaddrinfo(res);

    // Set socket read/write timeout
    {
        struct timeval tv = {10, 0};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    }

    // TLS setup
    ctx = SSL_CTX_new(TLS_client_method());
    if (!ctx) goto cleanup;

    ssl = SSL_new(ctx);
    if (!ssl) goto cleanup;

    SSL_set_fd(ssl, fd);
    SSL_set_tlsext_host_name(ssl, SONDEHUB_HOST);

    if (SSL_connect(ssl) <= 0) {
        fprintf(stderr, "sondehub: TLS handshake failed\n");
        goto cleanup;
    }

    // Build HTTP PUT request
    {
        // RFC 7231 Date header
        time_t now = time(NULL);
        struct tm *gmt = gmtime(&now);
        char date_str[64];
        strftime(date_str, sizeof(date_str), "%a, %d %b %Y %H:%M:%S GMT", gmt);

        char header[512];
        int hlen = snprintf(header, sizeof(header),
            "PUT %s HTTP/1.1\r\n"
            "Host: %s\r\n"
            "User-Agent: %s/%s\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %zu\r\n"
            "Date: %s\r\n"
            "Connection: close\r\n"
            "\r\n",
            path, SONDEHUB_HOST,
            SONDEHUB_SW_NAME, MODES_DUMP1090_VERSION,
            body_len, date_str);

        if (hlen <= 0 || hlen >= (int)sizeof(header)) goto cleanup;

        // Send header
        if (SSL_write(ssl, header, hlen) != hlen) goto cleanup;

        // Send body
        int total = 0;
        while (total < (int)body_len) {
            int w = SSL_write(ssl, json_body + total, (int)body_len - total);
            if (w <= 0) goto cleanup;
            total += w;
        }
    }

    // Read response — just need status line
    {
        char resp[256];
        int n = SSL_read(ssl, resp, sizeof(resp) - 1);
        if (n > 0) {
            resp[n] = '\0';
            // Parse "HTTP/1.1 2xx"
            int status = 0;
            if (sscanf(resp, "HTTP/%*d.%*d %d", &status) == 1 && status >= 200 && status < 300) {
                ok = true;
            } else {
                // Truncate response for logging
                char *nl = strchr(resp, '\r');
                if (nl) *nl = '\0';
                fprintf(stderr, "sondehub: PUT %s → %s\n", path, resp);
            }
        }
    }

cleanup:
    if (ssl) {
        SSL_shutdown(ssl);
        SSL_free(ssl);
    }
    if (ctx) SSL_CTX_free(ctx);
    if (fd >= 0) close(fd);
    return ok;
}

// ======================== JSON formatting ========================

// Format ISO 8601 UTC timestamp for current time
static void format_iso8601(char *buf, size_t len)
{
    time_t now = time(NULL);
    struct tm *gmt = gmtime(&now);
    strftime(buf, len, "%Y-%m-%dT%H:%M:%SZ", gmt);
}

// Build JSON array of telemetry messages from queue.
// Returns malloc'd string (caller frees), or NULL on empty.
static char *build_telemetry_json(unsigned count)
{
    if (count == 0) return NULL;

    // Estimate ~512 bytes per entry
    size_t bufsize = 128 + count * 600;
    char *buf = malloc(bufsize);
    if (!buf) return NULL;

    size_t pos = 0;
    buf[pos++] = '[';

    char time_recv[32];
    format_iso8601(time_recv, sizeof(time_recv));

    for (unsigned i = 0; i < count; i++) {
        if (SH.queue_tail == SH.queue_head) break;

        const sonde_msg_t *msg = &SH.queue[SH.queue_tail];
        SH.queue_tail = (SH.queue_tail + 1) % SONDEHUB_QUEUE_SIZE;

        if (i > 0) buf[pos++] = ',';

        int n = snprintf(buf + pos, bufsize - pos,
            "{"
            "\"software_name\":\"%s\","
            "\"software_version\":\"%s\","
            "\"uploader_callsign\":\"%s\","
            "\"time_received\":\"%s\","
            "\"manufacturer\":\"Vaisala\","
            "\"type\":\"%s\","
            "\"serial\":\"%s\","
            "\"frame\":%d,"
            "\"datetime\":\"%s\","
            "\"lat\":%.6f,"
            "\"lon\":%.6f,"
            "\"alt\":%.1f,"
            "\"vel_h\":%.1f,"
            "\"vel_v\":%.1f,"
            "\"heading\":%.1f,"
            "\"sats\":%d,"
            "\"frequency\":%.3f",
            SONDEHUB_SW_NAME, MODES_DUMP1090_VERSION,
            SondehubConfig.callsign,
            time_recv,
            msg->type,
            msg->serial,
            msg->frame_num,
            time_recv,     // datetime = time_received (we have no sonde GPS time)
            msg->lat, msg->lon, msg->alt,
            msg->vel_h, msg->vel_v, msg->heading,
            msg->satellites,
            (double)msg->freq);

        if (n <= 0 || n >= (int)(bufsize - pos)) { free(buf); return NULL; }
        pos += n;

        // Optional: uploader_position if we have station coords
        if (Modes.bUserFlags & MODES_USER_LATLON_VALID) {
            n = snprintf(buf + pos, bufsize - pos,
                ",\"uploader_position\":[%.6f,%.6f,0]",
                Modes.fUserLat, Modes.fUserLon);
            if (n > 0 && n < (int)(bufsize - pos)) pos += n;
        }

        // Optional: temp/humidity if available
        if (msg->temp != 0.0) {
            n = snprintf(buf + pos, bufsize - pos, ",\"temp\":%.1f", msg->temp);
            if (n > 0 && n < (int)(bufsize - pos)) pos += n;
        }
        if (msg->humidity != 0.0) {
            n = snprintf(buf + pos, bufsize - pos, ",\"humidity\":%.1f", msg->humidity);
            if (n > 0 && n < (int)(bufsize - pos)) pos += n;
        }
        if (msg->snr != 0.0f) {
            n = snprintf(buf + pos, bufsize - pos, ",\"snr\":%.1f", (double)msg->snr);
            if (n > 0 && n < (int)(bufsize - pos)) pos += n;
        }

        buf[pos++] = '}';
    }

    buf[pos++] = ']';
    buf[pos] = '\0';
    return buf;
}

// Build listener station JSON (single object, NOT array)
static char *build_listener_json(void)
{
    if (!(Modes.bUserFlags & MODES_USER_LATLON_VALID)) return NULL;

    char *buf = malloc(512);
    if (!buf) return NULL;

    int n = snprintf(buf, 512,
        "{"
        "\"software_name\":\"%s\","
        "\"software_version\":\"%s\","
        "\"uploader_callsign\":\"%s\","
        "\"uploader_position\":[%.6f,%.6f,0],"
        "\"uploader_antenna\":\"\","
        "\"mobile\":false"
        "}",
        SONDEHUB_SW_NAME, MODES_DUMP1090_VERSION,
        SondehubConfig.callsign,
        Modes.fUserLat, Modes.fUserLon);

    if (n <= 0 || n >= 512) { free(buf); return NULL; }
    return buf;
}

// ======================== Periodic work ========================

void sondehubClientPeriodicWork(void)
{
    if (!SondehubConfig.enabled) return;

    uint64_t now = mstime();

    // Upload listener station position: immediately on start, then every 10 min.
    // On failure, back off: 30s, 60s, 120s, ... up to 10 min.
    {
        uint64_t retry_interval = SH.listener_sent
            ? SONDEHUB_LISTENER_INTERVAL_MS
            : (SH.listener_fails == 0 ? 0 :
               (30000ULL << (SH.listener_fails < 5 ? SH.listener_fails - 1 : 4)));
        if (retry_interval > SONDEHUB_LISTENER_INTERVAL_MS)
            retry_interval = SONDEHUB_LISTENER_INTERVAL_MS;

        if (now - SH.last_listener_ms >= retry_interval) {
            char *json = build_listener_json();
            if (json) {
                if (sondehub_put("/listeners", json, strlen(json))) {
                    if (!SH.listener_sent) {
                        fprintf(stderr, "sondehub: listener station registered (%s)\n",
                                SondehubConfig.callsign);
                    }
                    SH.listener_sent = true;
                    SH.listener_fails = 0;
                } else {
                    SH.listener_fails++;
                }
                free(json);
            }
            SH.last_listener_ms = now;
        }
    }

    // Upload telemetry batch every 30 seconds (or when queue has data)
    if (now - SH.last_upload_ms >= SONDEHUB_UPLOAD_INTERVAL_MS) {
        unsigned count = queue_count();
        if (count > 0) {
            char *json = build_telemetry_json(count);
            if (json) {
                if (sondehub_put("/sondes/telemetry", json, strlen(json))) {
                    SH.uploads_ok++;
                    SH.telemetry_sent += count;
                } else {
                    SH.uploads_fail++;
                }
                free(json);
            }
        }
        SH.last_upload_ms = now;
    }
}
