// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sondehub_client.cpp: SondeHub telemetry upload client
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

#include <string>
#include <cstdarg>

#include "dump1090.h"
#include "decoder_queue.h"

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
    DecoderQueue<sonde_msg_t> queue{SONDEHUB_QUEUE_SIZE};

    uint64_t     last_upload_ms;
    uint64_t     last_listener_ms;
    bool         listener_sent;
    uint32_t     listener_fails;

    uint64_t     uploads_ok;
    uint64_t     uploads_fail;
    uint64_t     telemetry_sent;
} SH;

// ======================== Init / Cleanup ========================

void sondehubClientInit(void)
{
    SH.queue.clear();
    SH.last_upload_ms = 0;
    SH.last_listener_ms = 0;
    SH.listener_sent = false;
    SH.listener_fails = 0;
    SH.uploads_ok = 0;
    SH.uploads_fail = 0;
    SH.telemetry_sent = 0;
}

void sondehubClientCleanup(void)
{
    if (SH.telemetry_sent > 0 || SH.uploads_ok > 0) {
        fprintf(stderr, "sondehub: sent %" PRIu64 " telemetry in %" PRIu64 " uploads (%" PRIu64 " failed)\n",
                SH.telemetry_sent,
                SH.uploads_ok,
                SH.uploads_fail);
    }
}

// ======================== Queue ========================

void sondehubClientSubmit(const sonde_msg_t *msg)
{
    if (!SondehubConfig.enabled || !msg->valid_pos) return;
    SH.queue.push(*msg);
}

static uint32_t queue_count(void)
{
    return (uint32_t)SH.queue.size();
}

// ======================== TLS HTTP PUT ========================

// Perform a single HTTPS PUT to SondeHub.
// Returns true on HTTP 2xx response.
static bool sondehub_put(const char *path, const char *json_body, size_t body_len)
{
    struct addrinfo hints = {}, *res;
    int fd = -1;
    SSL_CTX *ctx = NULL;
    SSL *ssl = NULL;
    bool ok = false;

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
    // Load system CA trust store and enable peer verification
    SSL_CTX_set_default_verify_paths(ctx);
    SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER, NULL);
    // Force HTTP/1.1 via ALPN to prevent h2 binary framing
    static const uint8_t alpn[] = { 8, 'h','t','t','p','/','1','.','1' };
    SSL_CTX_set_alpn_protos(ctx, alpn, sizeof(alpn));

    ssl = SSL_new(ctx);
    if (!ssl) goto cleanup;

    SSL_set_fd(ssl, fd);
    SSL_set_tlsext_host_name(ssl, SONDEHUB_HOST);
    // Enable hostname verification
    SSL_set1_host(ssl, SONDEHUB_HOST);

    if (SSL_connect(ssl) <= 0) {
        fprintf(stderr, "sondehub: TLS handshake failed\n");
        goto cleanup;
    }

    // Build HTTP PUT request
    {
        // RFC 7231 Date header
        time_t now = time(NULL);
        struct tm *gmt = gmtime(&now);
        char date_buf[64];
        strftime(date_buf, sizeof(date_buf), "%a, %d %b %Y %H:%M:%S GMT", gmt);

        std::string header = std::string("PUT ") + path + " HTTP/1.1\r\n"
            "Host: " SONDEHUB_HOST "\r\n"
            "User-Agent: " SONDEHUB_SW_NAME "/" MODES_DUMP1090_VERSION "\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: " + std::to_string(body_len) + "\r\n"
            "Date: " + date_buf + "\r\n"
            "Connection: close\r\n"
            "\r\n";

        // Send header
        if (SSL_write(ssl, header.data(), (int)header.size()) != (int)header.size()) goto cleanup;

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
        char rbuf[256];
        int n = SSL_read(ssl, rbuf, sizeof(rbuf) - 1);
        if (n > 0) {
            std::string resp(rbuf, n);
            int status = 0;
            if (sscanf(resp.c_str(), "HTTP/%*d.%*d %d", &status) == 1 && status >= 200 && status < 300) {
                ok = true;
            } else {
                auto nl = resp.find('\r');
                if (nl != std::string::npos) resp.resize(nl);
                fprintf(stderr, "sondehub: PUT %s → %s\n", path, resp.c_str());
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
static std::string format_iso8601(void)
{
    time_t now = time(NULL);
    struct tm *gmt = gmtime(&now);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", gmt);
    return buf;
}

// Helper: format into std::string
static std::string sfmt(const char *fmt, ...) __attribute__((format(printf,1,2)));
static std::string sfmt(const char *fmt, ...) {
    char tmp[1024];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n < 0) return {};
    if ((size_t)n < sizeof(tmp)) return std::string(tmp, n);
    std::string s(n, '\0');
    va_start(ap, fmt);
    vsnprintf(&s[0], n + 1, fmt, ap);
    va_end(ap);
    return s;
}

// Build JSON array of telemetry messages from queue.
// Returns empty string on empty queue.
static std::string build_telemetry_json(uint32_t count)
{
    if (count == 0) return {};

    std::string time_recv = format_iso8601();

    std::string s = "[";

    for (uint32_t i = 0; i < count; i++) {
        sonde_msg_t item;
        if (!SH.queue.pop(item)) break;

        const sonde_msg_t *msg = &item;

        if (i > 0) s += ',';

        s += sfmt(
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
            time_recv.c_str(),
            msg->type,
            msg->serial,
            msg->frame_num,
            time_recv.c_str(),
            msg->lat, msg->lon, msg->alt,
            msg->vel_h, msg->vel_v, msg->heading,
            msg->satellites,
            (double)msg->freq);

        if (Modes.bUserFlags & MODES_USER_LATLON_VALID)
            s += sfmt(",\"uploader_position\":[%.6f,%.6f,0]", Modes.fUserLat, Modes.fUserLon);

        if (msg->temp != 0.0)
            s += sfmt(",\"temp\":%.1f", msg->temp);
        if (msg->humidity != 0.0)
            s += sfmt(",\"humidity\":%.1f", msg->humidity);
        if (msg->snr != 0.0f)
            s += sfmt(",\"snr\":%.1f", (double)msg->snr);

        s += '}';
    }

    s += ']';
    return s;
}

// Build listener station JSON (single object, NOT array)
static std::string build_listener_json(void)
{
    if (!(Modes.bUserFlags & MODES_USER_LATLON_VALID)) return {};

    return sfmt(
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
            std::string json = build_listener_json();
            if (!json.empty()) {
                if (sondehub_put("/listeners", json.c_str(), json.size())) {
                    if (!SH.listener_sent) {
                        fprintf(stderr, "sondehub: listener station registered (%s)\n",
                                SondehubConfig.callsign);
                    }
                    SH.listener_sent = true;
                    SH.listener_fails = 0;
                } else {
                    SH.listener_fails++;
                }
            }
            SH.last_listener_ms = now;
        }
    }

    // Upload telemetry batch every 30 seconds (or when queue has data)
    if (now - SH.last_upload_ms >= SONDEHUB_UPLOAD_INTERVAL_MS) {
        uint32_t count = queue_count();
        if (count > 0) {
            std::string json = build_telemetry_json(count);
            if (!json.empty()) {
                if (sondehub_put("/sondes/telemetry", json.c_str(), json.size())) {
                    SH.uploads_ok++;
                    SH.telemetry_sent += count;
                } else {
                    SH.uploads_fail++;
                }
            }
        }
        SH.last_upload_ms = now;
    }
}
