// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// feeder_thread.c: threaded feeder architecture
//
// Each network feeder runs in its own pthread with its own event loop.
// Threads: MLAT, PiAware, OGN, ADSBx.
// No fork/exec — everything is threads.
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include "dump1090.h"
#include "feeder_thread.h"
#include "fa_mlat.h"
#include "opensky_client.h"
#include "sondehub_client.h"

#include <math.h>
#include <time.h>
#include <sys/socket.h>
#include <netdb.h>
#include <fcntl.h>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <openssl/md5.h>

// ===================== Globals =====================

pthread_rwlock_t aircraft_lock = PTHREAD_RWLOCK_INITIALIZER;

static pthread_t mlat_thread;
static pthread_t piaware_thread;
static pthread_t ogn_thread;
static pthread_t beast_feed_thread;
static pthread_t opensky_thread;
static pthread_t sondehub_thread;

static struct feeder_msg_queue mlat_queue;
static struct feeder_msg_queue beast_feed_queue;
struct feeder_msg_queue mlat_inject_queue;
struct feeder_msg_queue fa_mlat_queue;

atomic_int feeders_running;
atomic_int net_available = 1;

// ===================== Beast binary encoding =====================

// Encode a modesMessage as Beast binary (verbatim mode).
// buf must be at least 46 bytes.
static int encode_beast_binary(const struct modesMessage *mm, unsigned char *buf, int bufsize) {
    unsigned char *p = buf;
    unsigned char *end = buf + bufsize;
    int msgLen = mm->msgbits / 8;

    if (msgLen != MODES_SHORT_MSG_BYTES && msgLen != MODES_LONG_MSG_BYTES && msgLen != MODEAC_MSG_BYTES)
        return 0;

    if (mm->source == SOURCE_MLAT && !Modes.forward_mlat)
        return 0;

#define BEAST_PUSH(b) do { \
    unsigned char _b = (unsigned char)(b); \
    if (p >= end) return 0; \
    *p++ = _b; \
    if (_b == 0x1a) { if (p >= end) return 0; *p++ = 0x1a; } \
} while(0)

    if (p >= end) return 0;
    *p++ = 0x1a;
    if (p >= end) return 0;
    if (msgLen == MODEAC_MSG_BYTES) *p++ = '1';
    else if (msgLen == MODES_SHORT_MSG_BYTES) *p++ = '2';
    else *p++ = '3';

    BEAST_PUSH((mm->timestampMsg >> 40) & 0xFF);
    BEAST_PUSH((mm->timestampMsg >> 32) & 0xFF);
    BEAST_PUSH((mm->timestampMsg >> 24) & 0xFF);
    BEAST_PUSH((mm->timestampMsg >> 16) & 0xFF);
    BEAST_PUSH((mm->timestampMsg >> 8) & 0xFF);
    BEAST_PUSH(mm->timestampMsg & 0xFF);

    int sig = (int)(sqrt(mm->signalLevel) * 255 + 0.5);
    if (mm->signalLevel > 0 && sig < 1) sig = 1;
    if (sig > 255) sig = 255;
    BEAST_PUSH(sig);

    for (int i = 0; i < msgLen; i++) {
        BEAST_PUSH(mm->verbatim[i]);
    }

#undef BEAST_PUSH

    return (int)(p - buf);
}

// ===================== TCP connect helper =====================

static int feeder_tcp_connect(const char *host, int port) {
    struct addrinfo hints, *res, *rp;
    char port_str[16];
    int fd = -1;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    snprintf(port_str, sizeof(port_str), "%d", port);
    if (getaddrinfo(host, port_str, &hints, &res) != 0)
        return -1;

    for (rp = res; rp; rp = rp->ai_next) {
        fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (fd < 0) continue;

        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0) {
            break;
        }
        if (errno == EINPROGRESS) {
            fd_set wfds;
            struct timeval tv = {5, 0};
            FD_ZERO(&wfds);
            FD_SET(fd, &wfds);
            if (select(fd + 1, NULL, &wfds, NULL, &tv) > 0) {
                int so_error;
                socklen_t len = sizeof(so_error);
                getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &len);
                if (so_error == 0)
                    break;
            }
        }
        close(fd);
        fd = -1;
    }

    freeaddrinfo(res);

    if (fd >= 0) {
        // Keep non-blocking, enable TCP keepalive
        int val = 1;
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &val, sizeof(val));
    }

    return fd;
}

// ===================== Beast feed thread (multi-destination) =====================

static const unsigned char beast_heartbeat[] = { 0x1a, '1', 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// Encode a modesMessage as Raw ASCII hex: *<hex>;\n
// Returns length, or 0 if not encodable.
static int encode_raw_ascii(const struct modesMessage *mm, unsigned char *buf, int bufsize) {
    int msgLen = mm->msgbits / 8;
    if (msgLen != MODES_SHORT_MSG_BYTES && msgLen != MODES_LONG_MSG_BYTES)
        return 0;
    if (mm->source == SOURCE_MLAT && !Modes.forward_mlat)
        return 0;

    // Need: '*' + 2*msgLen hex chars + ';' + '\n' + NUL
    int needed = 1 + 2 * msgLen + 2;
    if (needed >= bufsize) return 0;

    static const char hex[] = "0123456789ABCDEF";
    unsigned char *p = buf;
    *p++ = '*';
    for (int i = 0; i < msgLen; i++) {
        *p++ = hex[(mm->verbatim[i] >> 4) & 0x0F];
        *p++ = hex[mm->verbatim[i] & 0x0F];
    }
    *p++ = ';';
    *p++ = '\n';
    return (int)(p - buf);
}

// Encode a modesMessage as SBS/BaseStation CSV line
// Format: MSG,<type>,1,1,<ICAO>,1,<date>,<time>,<date>,<time>,<cs>,<alt>,<gs>,<trk>,<lat>,<lon>,<vr>,<sq>,,,,,\r\n
// Returns length, or 0 if not encodable.
static int encode_sbs_line(const struct modesMessage *mm, char *buf, int bufsize) {
    if (mm->source == SOURCE_MLAT && !Modes.forward_mlat)
        return 0;
    if (mm->correctedbits >= 2)
        return 0;
    if (mm->addr & MODES_NON_ICAO_ADDRESS)
        return 0;

    // Determine SBS message type from Mode S message type
    int msgType;
    switch (mm->msgtype) {
    case 4: case 20:   msgType = 5; break;
    case 5: case 21:   msgType = 6; break;
    case 0: case 16:   msgType = 7; break;
    case 11:           msgType = 8; break;
    case 17: case 18: case 19:
        if (mm->metype >= 1 && mm->metype <= 4)        msgType = 1;
        else if (mm->metype >= 5 && mm->metype <= 8)   msgType = 2;
        else if (mm->metype == 19)                      msgType = 4;
        else if ((mm->metype >= 9 && mm->metype <= 18) || (mm->metype >= 20 && mm->metype <= 22)) msgType = 3;
        else return 0;
        break;
    default: return 0;
    }

    char *p = buf;
    char *end = buf + bufsize - 2; // reserve for \r\n

    // Fields 1-6: MSG,type,1,1,ICAO,1,
    int n = snprintf(p, (size_t)(end - p), "MSG,%d,1,1,%06X,1,", msgType, mm->addr);
    if (n < 0 || p + n >= end) return 0;
    p += n;

    // Fields 7-10: dates and times
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    struct tm stNow, stRx;
    localtime_r(&now.tv_sec, &stNow);
    time_t rx = (time_t)(mm->sysTimestampMsg / 1000);
    localtime_r(&rx, &stRx);

    n = snprintf(p, (size_t)(end - p),
        "%04d/%02d/%02d,%02d:%02d:%02d.%03u,%04d/%02d/%02d,%02d:%02d:%02d.%03u",
        stRx.tm_year+1900, stRx.tm_mon+1, stRx.tm_mday,
        stRx.tm_hour, stRx.tm_min, stRx.tm_sec, (unsigned)(mm->sysTimestampMsg % 1000),
        stNow.tm_year+1900, stNow.tm_mon+1, stNow.tm_mday,
        stNow.tm_hour, stNow.tm_min, stNow.tm_sec, (unsigned)(now.tv_nsec / 1000000U));
    if (n < 0 || p + n >= end) return 0;
    p += n;

    // Field 11: callsign
    if (mm->callsign_valid) { n = snprintf(p, (size_t)(end - p), ",%s", mm->callsign); }
    else                    { n = snprintf(p, (size_t)(end - p), ","); }
    if (n < 0 || p + n >= end) return 0;
    p += n;

    // Field 12: altitude
    if (mm->altitude_baro_valid)      { n = snprintf(p, (size_t)(end - p), ",%d", mm->altitude_baro); }
    else if (mm->altitude_geom_valid) { n = snprintf(p, (size_t)(end - p), ",%d", mm->altitude_geom); }
    else                              { n = snprintf(p, (size_t)(end - p), ","); }
    if (n < 0 || p + n >= end) return 0;
    p += n;

    // Field 13: ground speed
    if (mm->gs_valid) { n = snprintf(p, (size_t)(end - p), ",%.0f", mm->gs.selected); }
    else              { n = snprintf(p, (size_t)(end - p), ","); }
    if (n < 0 || p + n >= end) return 0;
    p += n;

    // Field 14: track
    if (mm->heading_valid && mm->heading_type == HEADING_GROUND_TRACK) {
        n = snprintf(p, (size_t)(end - p), ",%.0f", mm->heading);
    } else { n = snprintf(p, (size_t)(end - p), ","); }
    if (n < 0 || p + n >= end) return 0;
    p += n;

    // Fields 15-16: lat/lon
    if (mm->cpr_decoded) { n = snprintf(p, (size_t)(end - p), ",%1.5f,%1.5f", mm->decoded_lat, mm->decoded_lon); }
    else                 { n = snprintf(p, (size_t)(end - p), ",,"); }
    if (n < 0 || p + n >= end) return 0;
    p += n;

    // Field 17: vertical rate
    if (mm->baro_rate_valid)      { n = snprintf(p, (size_t)(end - p), ",%d", mm->baro_rate); }
    else if (mm->geom_rate_valid) { n = snprintf(p, (size_t)(end - p), ",%d", mm->geom_rate); }
    else                          { n = snprintf(p, (size_t)(end - p), ","); }
    if (n < 0 || p + n >= end) return 0;
    p += n;

    // Field 18: squawk
    if (mm->squawk_valid) { n = snprintf(p, (size_t)(end - p), ",%04x", mm->squawk); }
    else                  { n = snprintf(p, (size_t)(end - p), ","); }
    if (n < 0 || p + n >= end) return 0;
    p += n;

    // Fields 19-20: alert, emergency
    if (mm->alert_valid) { n = snprintf(p, (size_t)(end - p), ",%d", mm->alert ? -1 : 0); }
    else                 { n = snprintf(p, (size_t)(end - p), ","); }
    if (n < 0 || p + n >= end) return 0;
    p += n;

    if (mm->squawk_valid && (mm->squawk == 0x7500 || mm->squawk == 0x7600 || mm->squawk == 0x7700)) {
        n = snprintf(p, (size_t)(end - p), ",-1");
    } else if (mm->squawk_valid) {
        n = snprintf(p, (size_t)(end - p), ",0");
    } else {
        n = snprintf(p, (size_t)(end - p), ",");
    }
    if (n < 0 || p + n >= end) return 0;
    p += n;

    // Field 21: SPI
    if (mm->spi_valid) { n = snprintf(p, (size_t)(end - p), ",%d", mm->spi ? -1 : 0); }
    else               { n = snprintf(p, (size_t)(end - p), ","); }
    if (n < 0 || p + n >= end) return 0;
    p += n;

    // Field 22: on ground
    if (mm->airground == AG_GROUND)        { n = snprintf(p, (size_t)(end - p), ",-1"); }
    else if (mm->airground == AG_AIRBORNE) { n = snprintf(p, (size_t)(end - p), ",0"); }
    else                                   { n = snprintf(p, (size_t)(end - p), ","); }
    if (n < 0 || p + n >= end) return 0;
    p += n;

    *p++ = '\r'; *p++ = '\n';
    return (int)(p - buf);
}

struct beast_feed_conn {
    int fd;
    uint64_t next_reconnect;
    uint64_t last_heartbeat;
    int reconnect_count;  // suppress repeated log spam
};

// ===================== ADSBHub ckey IP update =====================
// Protocol (from official adsbhub.sh):
//  1. GET https://ip4.adsbhub.org/getmyip.php  -> my IPv4
//  2. GET https://www.adsbhub.org/key.php       -> skey (server challenge)
//  3. sessid = md5(ckey + skey[:-1]) + skey[-1]
//  4. GET https://www.adsbhub.org/updateip.php?sessid=<sessid>&myip=<ipv4>&myip6=::

// HTTPS GET helper: fetches response body into buf. Returns body length or -1.
static int https_get(const char *host, const char *path, char *buf, int bufsz) {
    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET; // Force IPv4 — ADSBHub server doesn't handle IPv6 ckey updates correctly
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host, "443", &hints, &res) != 0) return -1;

    int fd = socket(res->ai_family, SOCK_STREAM, 0);
    if (fd < 0) { freeaddrinfo(res); return -1; }

    struct timeval tv = {10, 0};
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (connect(fd, res->ai_addr, res->ai_addrlen) < 0) {
        freeaddrinfo(res); close(fd); return -1;
    }
    freeaddrinfo(res);

    SSL_CTX *ctx = SSL_CTX_new(TLS_client_method());
    if (!ctx) { close(fd); return -1; }
    SSL *ssl = SSL_new(ctx);
    if (!ssl) { SSL_CTX_free(ctx); close(fd); return -1; }
    SSL_set_fd(ssl, fd);
    SSL_set_tlsext_host_name(ssl, host);

    if (SSL_connect(ssl) <= 0) {
        SSL_free(ssl); SSL_CTX_free(ctx); close(fd); return -1;
    }

    char req[1024];
    int reqlen = snprintf(req, sizeof(req),
        "GET %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: Wget/1.21\r\nConnection: close\r\n\r\n", path, host);

    int result = -1;
    if (SSL_write(ssl, req, reqlen) > 0) {
        char resp[4096];
        int total = 0, r;
        while ((r = SSL_read(ssl, resp + total, (int)sizeof(resp) - total - 1)) > 0) {
            total += r;
            if (total >= (int)sizeof(resp) - 1) break;
        }
        resp[total] = '\0';
        // Find body after \r\n\r\n
        char *body = strstr(resp, "\r\n\r\n");
        if (body) {
            body += 4;
            int bodylen = total - (int)(body - resp);
            if (bodylen > 0 && bodylen < bufsz) {
                memcpy(buf, body, bodylen);
                buf[bodylen] = '\0';
                result = bodylen;
            }
        }
    }

    SSL_shutdown(ssl);
    SSL_free(ssl);
    SSL_CTX_free(ctx);
    close(fd);
    return result;
}

static void url_encode(const char *src, char *dst, size_t dst_size) {
    static const char hex[] = "0123456789ABCDEF";
    char *end = dst + dst_size - 1;
    while (*src && dst < end) {
        unsigned char c = (unsigned char)*src;
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
            *dst++ = c;
        } else if (dst + 3 <= end) {
            *dst++ = '%';
            *dst++ = hex[c >> 4];
            *dst++ = hex[c & 0x0f];
        } else break;
        src++;
    }
    *dst = '\0';
}

// ===================== Internet Connectivity Check =====================
// Quick DNS probe: resolve a well-known host to detect internet.

#define NET_CHECK_HOST         "feed.adsbexchange.com"
#define NET_CHECK_INTERVAL_MS  30000   // check every 30s when online
#define NET_CHECK_OFFLINE_MS   5000    // check every 5s when offline

static int check_internet(void) {
    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    int rc = getaddrinfo(NET_CHECK_HOST, "443", &hints, &res);
    if (rc == 0) {
        freeaddrinfo(res);
        return 1;
    }
    return 0;
}

// Returns 0 on success, -1 on failure
static int adsbhub_update_ckey(const char *ckey) {
    // Step 1: get my IPv4
    char myip[64];
    if (https_get("ip4.adsbhub.org", "/getmyip.php", myip, sizeof(myip)) < 7) {
        fprintf(stderr, "ADSBHub ckey: failed to get public IP\n");
        return -1;
    }
    // Strip trailing whitespace
    int len = (int)strlen(myip);
    while (len > 0 && (myip[len-1] == '\n' || myip[len-1] == '\r' || myip[len-1] == ' '))
        myip[--len] = '\0';
    fprintf(stderr, "ADSBHub ckey: my IP = %s\n", myip);

    // Step 2: get server key
    char skey[128];
    int skeylen = https_get("www.adsbhub.org", "/key.php", skey, sizeof(skey));
    fprintf(stderr, "ADSBHub ckey: skey len=%d\n", skeylen);
    if (skeylen < 33) {
        fprintf(stderr, "ADSBHub ckey: failed to get server key (len=%d)\n", skeylen);
        return -1;
    }
    // Strip trailing whitespace
    while (skeylen > 0 && (skey[skeylen-1] == '\n' || skey[skeylen-1] == '\r' || skey[skeylen-1] == ' '))
        skey[--skeylen] = '\0';
    fprintf(stderr, "ADSBHub ckey: skey=%s (len=%d)\n", skey, skeylen);

    // Step 3: compute sessid = md5(ckey + skey[:-1]) + skey[-1]
    char ss = skey[skeylen - 1]; // last char of skey
    skey[skeylen - 1] = '\0';   // skey without last char

    // Concatenate ckey + skey_trimmed
    char concat[512];
    int clen = snprintf(concat, sizeof(concat), "%s%s", ckey, skey);
    if (clen < 0 || clen >= (int)sizeof(concat)) return -1;

    // MD5
    unsigned char md5_raw[MD5_DIGEST_LENGTH];
    MD5((unsigned char *)concat, clen, md5_raw);

    char sessid[64];
    for (int i = 0; i < MD5_DIGEST_LENGTH; i++)
        sprintf(sessid + i*2, "%02x", md5_raw[i]);
    // Append the last char of original skey
    int slen = (int)strlen(sessid);
    sessid[slen] = ss;
    sessid[slen+1] = '\0';
    fprintf(stderr, "ADSBHub ckey: sessid=%s\n", sessid);

    // Step 3b: get my IPv6 (best-effort, fall back to ::)
    char myip6[128];
    strcpy(myip6, "::");
    int ip6len = https_get("ip6.adsbhub.org", "/getmyip.php", myip6, sizeof(myip6));
    if (ip6len > 1) {
        while (ip6len > 0 && (myip6[ip6len-1] == '\n' || myip6[ip6len-1] == '\r' || myip6[ip6len-1] == ' '))
            myip6[--ip6len] = '\0';
    } else {
        strcpy(myip6, "::");
    }

    // Step 4: call updateip.php
    char myip_enc[128], myip6_enc[256];
    url_encode(myip, myip_enc, sizeof(myip_enc));
    url_encode(myip6, myip6_enc, sizeof(myip6_enc));

    char path[512];
    snprintf(path, sizeof(path), "/updateip.php?sessid=%s&myip=%s&myip6=%s", sessid, myip_enc, myip6_enc);

    char result[256];
    int rlen = https_get("www.adsbhub.org", path, result, sizeof(result));
    fprintf(stderr, "ADSBHub ckey: updateip response len=%d\n", rlen);
    if (rlen > 0) {
        // Strip trailing whitespace
        while (rlen > 0 && (result[rlen-1] == '\n' || result[rlen-1] == '\r' || result[rlen-1] == ' '))
            result[--rlen] = '\0';
        if (strcmp(result, sessid) == 0) {
            fprintf(stderr, "ADSBHub ckey: IP update OK (ip=%s, ip6=%s)\n", myip, myip6);
            return 0;
        }
        fprintf(stderr, "ADSBHub ckey: server returned '%s', expected '%s'\n", result, sessid);
    } else {
        fprintf(stderr, "ADSBHub ckey: updateip request failed\n");
    }
    return -1;
}

static void *beast_feed_thread_entry(void *arg) {
    MODES_NOTUSED(arg);

    int n = Modes.beast_feed_count;
    struct beast_feed_conn conns[MAX_BEAST_FEEDS];
    struct modesMessage mm;

    for (int i = 0; i < n; i++) {
        conns[i].fd = -1;
        conns[i].next_reconnect = 0;
        conns[i].last_heartbeat = 0;
        conns[i].reconnect_count = 0;
    }

    // ADSBHub: update dynamic IP via ckey before first connection
    uint64_t ckey_next_update = 0;
    #define CKEY_UPDATE_INTERVAL_MS (300000ULL) // 5 minutes
    if (Modes.adsbhub_ckey) {
        fprintf(stderr, "ADSBHub: updating dynamic IP via ckey...\n");
        int rc = adsbhub_update_ckey(Modes.adsbhub_ckey);
        if (rc == 0) {
            fprintf(stderr, "ADSBHub: ckey IP update OK\n");
            if (PanelState.enabled) panelLog("ADSBHub: dynamic IP updated OK");
        } else {
            fprintf(stderr, "ADSBHub: ckey IP update FAILED\n");
            if (PanelState.enabled) panelLog("ADSBHub: dynamic IP update FAILED");
        }
        ckey_next_update = mstime() + CKEY_UPDATE_INTERVAL_MS;
    }

    while (atomic_load(&feeders_running)) {
        uint64_t now = mstime();

        // ADSBHub: periodic ckey update (every 5 min)
        if (Modes.adsbhub_ckey && now >= ckey_next_update) {
            int rc = adsbhub_update_ckey(Modes.adsbhub_ckey);
            if (rc == 0) {
                fprintf(stderr, "ADSBHub: ckey IP update OK\n");
            } else {
                fprintf(stderr, "ADSBHub: ckey IP update FAILED\n");
                if (PanelState.enabled) panelLog("ADSBHub: periodic IP update failed");
            }
            ckey_next_update = now + CKEY_UPDATE_INTERVAL_MS;
        }

        // ---- Internet watchdog ----
        static uint64_t next_net_check = 0;
        if (now >= next_net_check) {
            int was_online = atomic_load(&net_available);
            int is_online = check_internet();
            atomic_store(&net_available, is_online);
            if (was_online && !is_online) {
                fprintf(stderr, "NET: internet offline — pausing all feeders\n");
                if (PanelState.enabled) panelLog("NET: internet OFFLINE — feeders paused");
            } else if (!was_online && is_online) {
                fprintf(stderr, "NET: internet back online — resuming feeders\n");
                if (PanelState.enabled) panelLog("NET: internet ONLINE — feeders resumed");
                ckey_next_update = now; // force ckey update on reconnect
            }
            next_net_check = now + (is_online ? NET_CHECK_INTERVAL_MS : NET_CHECK_OFFLINE_MS);
        }

        if (!atomic_load(&net_available)) {
            usleep(1000000); // 1s sleep while offline
            continue;
        }

        // Manage connections
        for (int i = 0; i < n; i++) {
            if (!Modes.beast_feeds[i].enabled) continue;
            if (conns[i].fd >= 0) continue;
            if (now < conns[i].next_reconnect) continue;

            conns[i].fd = feeder_tcp_connect(Modes.beast_feeds[i].host, Modes.beast_feeds[i].port);
            if (conns[i].fd < 0) {
                conns[i].reconnect_count++;
                fprintf(stderr, "%s: failed to connect to %s:%d, retry in 30s\n",
                        Modes.beast_feeds[i].name, Modes.beast_feeds[i].host, Modes.beast_feeds[i].port);
                if (PanelState.enabled && conns[i].reconnect_count <= 1)
                    panelLog("%s: connect FAILED %s:%d", Modes.beast_feeds[i].name, Modes.beast_feeds[i].host, Modes.beast_feeds[i].port);
                conns[i].next_reconnect = now + 30000;
            } else {
                fprintf(stderr, "%s: connected to %s:%d\n",
                        Modes.beast_feeds[i].name, Modes.beast_feeds[i].host, Modes.beast_feeds[i].port);
                if (PanelState.enabled) {
                    if (conns[i].reconnect_count == 0)
                        panelLog("%s: connected %s:%d", Modes.beast_feeds[i].name, Modes.beast_feeds[i].host, Modes.beast_feeds[i].port);
                    else
                        panelLog("%s: reconnected %s:%d (after %d attempts)", Modes.beast_feeds[i].name, Modes.beast_feeds[i].host, Modes.beast_feeds[i].port, conns[i].reconnect_count);
                }
                conns[i].reconnect_count = 0;
                conns[i].last_heartbeat = now;
            }
        }

        // Check if any feed is connected
        int any_connected = 0;
        for (int i = 0; i < n; i++) {
            if (conns[i].fd >= 0) { any_connected = 1; break; }
        }

        // Drain queue, send to all connected feeds
        int sent = 0;
        while (feeder_queue_pop(&beast_feed_queue, &mm)) {
            if (!any_connected) continue; // drain but don't send

            // Encode once per format type (lazy)
            unsigned char beast_buf_enc[256];
            int beast_len = -1; // -1 = not yet encoded
            unsigned char raw_buf[64];
            int raw_len = -1;
            char sbs_buf[512];
            int sbs_len = -1;

            for (int i = 0; i < n; i++) {
                if (conns[i].fd < 0) continue;
                int len;
                void *buf;

                if (Modes.beast_feeds[i].format == FEED_FORMAT_RAW) {
                    if (raw_len < 0) raw_len = encode_raw_ascii(&mm, raw_buf, sizeof(raw_buf));
                    len = raw_len; buf = raw_buf;
                } else if (Modes.beast_feeds[i].format == FEED_FORMAT_SBS) {
                    if (sbs_len < 0) sbs_len = encode_sbs_line(&mm, sbs_buf, sizeof(sbs_buf));
                    len = sbs_len; buf = sbs_buf;
                } else {
                    if (beast_len < 0) beast_len = encode_beast_binary(&mm, beast_buf_enc, sizeof(beast_buf_enc));
                    len = beast_len; buf = beast_buf_enc;
                }
                if (len <= 0) continue;

                int w = write(conns[i].fd, buf, len);
                if (w < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    fprintf(stderr, "%s: write error: %s\n", Modes.beast_feeds[i].name, strerror(errno));
                    if (PanelState.enabled && conns[i].reconnect_count == 0)
                        panelLog("%s: disconnected (write: %s)", Modes.beast_feeds[i].name, strerror(errno));
                    close(conns[i].fd); conns[i].fd = -1;
                    conns[i].reconnect_count++;
                    conns[i].next_reconnect = mstime() + 30000;
                }
            }
            sent++;
        }

        // Heartbeats and server drain
        now = mstime();
        for (int i = 0; i < n; i++) {
            if (conns[i].fd < 0) continue;

            // Heartbeat every 30s (only for Beast format feeds)
            if (Modes.beast_feeds[i].format == FEED_FORMAT_BEAST && now - conns[i].last_heartbeat >= 30000) {
                if (write(conns[i].fd, beast_heartbeat, sizeof(beast_heartbeat)) < 0) {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        fprintf(stderr, "%s: heartbeat error: %s\n", Modes.beast_feeds[i].name, strerror(errno));
                        if (PanelState.enabled && conns[i].reconnect_count == 0)
                            panelLog("%s: disconnected (heartbeat: %s)", Modes.beast_feeds[i].name, strerror(errno));
                        close(conns[i].fd); conns[i].fd = -1;
                        conns[i].reconnect_count++;
                        conns[i].next_reconnect = now + 30000;
                        continue;
                    }
                }
                conns[i].last_heartbeat = now;
            }

            // Discard any server responses
            char discard[1024];
            int r = read(conns[i].fd, discard, sizeof(discard));
            if (r == 0) {
                fprintf(stderr, "%s: connection closed by server\n", Modes.beast_feeds[i].name);
                if (PanelState.enabled && conns[i].reconnect_count == 0)
                    panelLog("%s: disconnected (server closed)", Modes.beast_feeds[i].name);
                close(conns[i].fd); conns[i].fd = -1;
                conns[i].reconnect_count++;
                conns[i].next_reconnect = now + 30000;
            } else if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                fprintf(stderr, "%s: read error: %s\n", Modes.beast_feeds[i].name, strerror(errno));
                if (PanelState.enabled && conns[i].reconnect_count == 0)
                    panelLog("%s: disconnected (%s)", Modes.beast_feeds[i].name, strerror(errno));
                close(conns[i].fd); conns[i].fd = -1;
                conns[i].reconnect_count++;
                conns[i].next_reconnect = now + 30000;
            }
        }

        if (!sent) {
            struct timespec ts = {0, 5 * 1000 * 1000};
            nanosleep(&ts, NULL);
        }
    }

    for (int i = 0; i < n; i++) {
        if (conns[i].fd >= 0) close(conns[i].fd);
    }
    return NULL;
}

// ===================== MLAT feeder thread =====================

static void *mlat_thread_entry(void *arg) {
    MODES_NOTUSED(arg);

    struct modesMessage mm;
    struct timespec sleep_ts = {0, 5 * 1000 * 1000};

    while (atomic_load(&feeders_running)) {
        // When internet is offline, drain queue without processing
        // (real-time data is useless when buffered for later)
        if (!atomic_load(&net_available)) {
            while (feeder_queue_pop(&mlat_queue, &mm)) { /* discard */ }
            mlatClientDisconnectAll("internet offline");
            usleep(500000); // 500ms sleep while offline
            continue;
        }

        int got_msg = 0;

        while (feeder_queue_pop(&mlat_queue, &mm)) {
            mlatClientProcessMessage(&mm);
            got_msg = 1;
        }

        mlatClientPeriodicWork();

        if (!got_msg) {
            nanosleep(&sleep_ts, NULL);
        }
    }

    return NULL;
}

// ===================== PiAware feeder thread =====================

static void *piaware_thread_entry(void *arg) {
    MODES_NOTUSED(arg);

    struct timespec sleep_ts = {0, 50 * 1000 * 1000};

    while (atomic_load(&feeders_running)) {
        if (!atomic_load(&net_available)) {
            usleep(500000);
            continue;
        }

        pthread_rwlock_rdlock(&aircraft_lock);
        piawareClientPeriodicWork();
        pthread_rwlock_unlock(&aircraft_lock);

        nanosleep(&sleep_ts, NULL);
    }

    return NULL;
}

// ===================== OGN feeder thread =====================

static void *ogn_thread_entry(void *arg) {
    MODES_NOTUSED(arg);

    struct timespec sleep_ts = {0, 100 * 1000 * 1000};

    while (atomic_load(&feeders_running)) {
        if (!atomic_load(&net_available)) {
            usleep(500000);
            continue;
        }

        ognClientPeriodicWork();
        nanosleep(&sleep_ts, NULL);
    }

    return NULL;
}

// ===================== SondeHub feeder thread =====================

static void *sondehub_thread_entry(void *arg) {
    MODES_NOTUSED(arg);

    struct timespec sleep_ts = {1, 0};   // 1 second interval

    while (atomic_load(&feeders_running)) {
        if (!atomic_load(&net_available)) {
            usleep(500000);
            continue;
        }

        sondehubClientPeriodicWork();
        nanosleep(&sleep_ts, NULL);
    }

    return NULL;
}

// ===================== Public API =====================

void feederProcessInjectedMessages(void) {
    struct modesMessage mm;
    while (feeder_queue_pop(&mlat_inject_queue, &mm)) {
        useModesMessage(&mm);
    }
}

void feederThreadsStart(void) {
    feeder_queue_init(&mlat_queue);
    feeder_queue_init(&beast_feed_queue);
    feeder_queue_init(&mlat_inject_queue);
    feeder_queue_init(&fa_mlat_queue);
    atomic_store(&feeders_running, 1);

    // MLAT thread: start if we have servers or PiAware (FA may add server dynamically)
    if (MlatConfig.server_count > 0 || PiawareClient.enabled) {
        if (pthread_create(&mlat_thread, NULL, mlat_thread_entry, NULL) != 0) {
            fprintf(stderr, "feeder: failed to create MLAT thread: %s\n", strerror(errno));
        } else {
            fprintf(stderr, "feeder: MLAT thread started\n");
        }
    }

    if (PiawareClient.enabled) {
        if (pthread_create(&piaware_thread, NULL, piaware_thread_entry, NULL) != 0) {
            fprintf(stderr, "feeder: failed to create PiAware thread: %s\n", strerror(errno));
        } else {
            fprintf(stderr, "feeder: PiAware thread started\n");
        }
    }

    if (FlarmConfig.enabled && FlarmConfig.ogn_station[0] != '\0') {
        if (pthread_create(&ogn_thread, NULL, ogn_thread_entry, NULL) != 0) {
            fprintf(stderr, "feeder: failed to create OGN thread: %s\n", strerror(errno));
        } else {
            fprintf(stderr, "feeder: OGN thread started\n");
        }
    }

    if (Modes.beast_feed_count > 0) {
        if (pthread_create(&beast_feed_thread, NULL, beast_feed_thread_entry, NULL) != 0) {
            fprintf(stderr, "feeder: failed to create beast feed thread: %s\n", strerror(errno));
        } else {
            for (int i = 0; i < Modes.beast_feed_count; i++)
                fprintf(stderr, "feeder: beast feed enabled: %s -> %s:%d\n",
                        Modes.beast_feeds[i].name, Modes.beast_feeds[i].host, Modes.beast_feeds[i].port);
            fprintf(stderr, "feeder: beast feed thread started (%d feeds)\n", Modes.beast_feed_count);
        }
    }

    // PlaneFinder, FR24, RadarBox feeders removed in light version

    if (OpenSkyConfig.enabled) {
        feeder_queue_init(&opensky_queue);
        if (pthread_create(&opensky_thread, NULL, opensky_thread_entry, NULL) != 0) {
            fprintf(stderr, "feeder: failed to create OpenSky thread: %s\n", strerror(errno));
        } else {
            fprintf(stderr, "feeder: OpenSky thread started (user=%s, host=%s:%d)\n",
                    OpenSkyConfig.username, OpenSkyConfig.host, OpenSkyConfig.port);
        }
    }

    if (SondehubConfig.enabled) {
        if (pthread_create(&sondehub_thread, NULL, sondehub_thread_entry, NULL) != 0) {
            fprintf(stderr, "feeder: failed to create SondeHub thread: %s\n", strerror(errno));
        } else {
            fprintf(stderr, "feeder: SondeHub thread started (callsign=%s)\n",
                    SondehubConfig.callsign);
        }
    }
}

void feederDispatchMessage(struct modesMessage *mm) {
    if (MlatConfig.server_count > 0 || PiawareClient.enabled) {
        feeder_queue_push(&mlat_queue, mm);
    }
    if (Modes.beast_feed_count > 0) {
        feeder_queue_push(&beast_feed_queue, mm);
    }
    if (OpenSkyConfig.enabled) {
        feeder_queue_push(&opensky_queue, mm);
    }
    // FA MLAT thread reads from its own queue (enabled dynamically by PiAware)
    if (FaMlat.enabled) {
        feeder_queue_push(&fa_mlat_queue, mm);
    }
}

void feederThreadsStop(void) {
    atomic_store(&feeders_running, 0);

    fprintf(stderr, "feeder: waiting for feeder threads to stop...\n");

    if (MlatConfig.server_count > 0 || PiawareClient.enabled) {
        join_thread(mlat_thread, NULL, 5000);
        fprintf(stderr, "feeder: MLAT thread stopped\n");
    }

    if (PiawareClient.enabled) {
        join_thread(piaware_thread, NULL, 5000);
        fprintf(stderr, "feeder: PiAware thread stopped\n");
    }

    if (FlarmConfig.enabled && FlarmConfig.ogn_station[0] != '\0') {
        join_thread(ogn_thread, NULL, 5000);
        fprintf(stderr, "feeder: OGN thread stopped\n");
    }

    if (Modes.beast_feed_count > 0) {
        join_thread(beast_feed_thread, NULL, 5000);
        fprintf(stderr, "feeder: beast feed thread stopped\n");
    }

    // PlaneFinder, FR24, RadarBox feeders removed in light version

    if (OpenSkyConfig.enabled) {
        join_thread(opensky_thread, NULL, 15000);
        fprintf(stderr, "feeder: OpenSky thread stopped\n");
    }

    if (SondehubConfig.enabled) {
        join_thread(sondehub_thread, NULL, 15000);
        fprintf(stderr, "feeder: SondeHub thread stopped\n");
    }
}
