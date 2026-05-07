// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// piaware_client.c: Built-in FlightAware ADEPT client for dump1090-gg
//
// Implements the ADEPT (Aviation Data Exchange Protocol) to connect directly
// to FlightAware servers over TLS, send ADS-B data in FATSV format,
// and handle alive/health messages.
//
// Based on the open-source piaware client (BSD license):
// https://github.com/flightaware/piaware
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include "dump1090.h"
#include "piaware_client.h"
#include "fa_mlat.h"

#include <stdarg.h>
#include <sys/utsname.h>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <openssl/x509v3.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <dirent.h>

piaware_client_t PiawareClient;

// Forward declarations
static void paConnect(void);
static void paDisconnect(const char *reason);
static void paTLSHandshake(void);
static void paSendLogin(void);
static void paHandleInput(void);
static void paHandleLine(const char *line);
static int  paSend(const char *data, int len);
static void paSendFATSV(void);
static void paSendHealth(void);
static void paForwardFaMlatStatus(void);
static void paHandleMlatWanted(const char *line);
static void paHandleMlatUnwanted(const char *line);
static void paHandleMlatResult(const char *line);

// ============================================================================
// Helpers
// ============================================================================

static inline float heading_difference(float h1, float h2) {
    float d = fabs(h1 - h2);
    return d > 180.0 ? 360.0 - d : d;
}

// FATSV enum string helpers (match net_io.c format)
static const char *pa_addrtype_str(addrtype_t t) {
    switch (t) {
    case ADDR_ADSB_ICAO:     return "adsb_icao";
    case ADDR_ADSB_ICAO_NT:  return "adsb_icao_nt";
    case ADDR_ADSR_ICAO:     return "adsr_icao";
    case ADDR_TISB_ICAO:     return "tisb_icao";
    case ADDR_ADSB_OTHER:    return "adsb_other";
    case ADDR_ADSR_OTHER:    return "adsr_other";
    case ADDR_TISB_OTHER:    return "tisb_other";
    case ADDR_TISB_TRACKFILE:return "tisb_trackfile";
    default:                 return "unknown";
    }
}

static const char *pa_emergency_str(emergency_t e) {
    switch (e) {
    case EMERGENCY_NONE:      return "none";
    case EMERGENCY_GENERAL:   return "general";
    case EMERGENCY_LIFEGUARD: return "lifeguard";
    case EMERGENCY_MINFUEL:   return "minfuel";
    case EMERGENCY_NORDO:     return "nordo";
    case EMERGENCY_UNLAWFUL:  return "unlawful";
    case EMERGENCY_DOWNED:    return "downed";
    default:                  return "reserved";
    }
}

static const char *pa_sil_type_str(sil_type_t t) {
    switch (t) {
    case SIL_UNKNOWN:    return "unknown";
    case SIL_PER_HOUR:   return "perhour";
    case SIL_PER_SAMPLE: return "persample";
    default:             return "invalid";
    }
}

static const char *pa_nav_alt_src_str(nav_altitude_source_t s) {
    switch (s) {
    case NAV_ALT_UNKNOWN:  return "unknown";
    case NAV_ALT_AIRCRAFT: return "aircraft";
    case NAV_ALT_MCP:      return "mcp";
    case NAV_ALT_FMS:      return "fms";
    default:               return "invalid";
    }
}

static const char *pa_airground_str(airground_t ag) {
    switch (ag) {
    case AG_AIRBORNE: return "A+";
    case AG_GROUND:   return "G+";
    default:          return "?";
    }
}

static const char *pa_source_char(datasource_t s) {
    switch (s) {
    case SOURCE_MODE_S:         return "U";
    case SOURCE_MODE_S_CHECKED: return "S";
    case SOURCE_TISB:           return "T";
    case SOURCE_ADSR:           return "R";
    case SOURCE_ADSB:           return "A";
    default:                    return NULL;
    }
}

// Read MAC address from first non-loopback network interface
static void paDetectMAC(void) {
    DIR *d = opendir("/sys/class/net");
    if (!d) {
        strcpy(PiawareClient.mac, "00:00:00:00:00:00");
        return;
    }

    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        if (strcmp(ent->d_name, "lo") == 0 || ent->d_name[0] == '.')
            continue;

        char path[512];
        snprintf(path, sizeof(path), "/sys/class/net/%s/address", ent->d_name);
        FILE *f = fopen(path, "r");
        if (f) {
            if (fgets(PiawareClient.mac, sizeof(PiawareClient.mac), f)) {
                // strip newline
                char *nl = strchr(PiawareClient.mac, '\n');
                if (nl) *nl = 0;
            }
            fclose(f);
            if (strlen(PiawareClient.mac) >= 17) {
                closedir(d);
                return;
            }
        }
    }
    closedir(d);
    if (strlen(PiawareClient.mac) < 17)
        strcpy(PiawareClient.mac, "00:00:00:00:00:00");
}

// Read feeder ID from file
static void paReadFeederID(void) {
    PiawareClient.feeder_id[0] = 0;
    strcpy(PiawareClient.feeder_id_source, "none");

    FILE *f = fopen(PiawareClient.feeder_id_file, "r");
    if (!f) return;

    char buf[128];
    if (fgets(buf, sizeof(buf), f)) {
        char *nl = strchr(buf, '\n');
        if (nl) *nl = 0;
        char *cr = strchr(buf, '\r');
        if (cr) *cr = 0;
        if (strlen(buf) > 0) {
            strncpy(PiawareClient.feeder_id, buf, sizeof(PiawareClient.feeder_id) - 1);
            PiawareClient.feeder_id[sizeof(PiawareClient.feeder_id) - 1] = '\0';
            strcpy(PiawareClient.feeder_id_source, "cache");
        }
    }
    fclose(f);
}

// TSV builder helpers
typedef struct {
    char buf[4096];
    int  pos;
} tsv_buf_t;

static void tsv_init(tsv_buf_t *t) {
    t->pos = 0;
    t->buf[0] = 0;
}

static void tsv_field(tsv_buf_t *t, const char *key, const char *fmt, ...) {
    int space = sizeof(t->buf) - t->pos;
    if (space < 10) return;

    int n = snprintf(t->buf + t->pos, space, "%s\t", key);
    t->pos += n;
    space -= n;
    if (space < 2) return;

    va_list ap;
    va_start(ap, fmt);
    n = vsnprintf(t->buf + t->pos, space, fmt, ap);
    va_end(ap);
    t->pos += n;
    space -= n;
    if (space < 2) return;

    t->buf[t->pos++] = '\t';
    t->buf[t->pos] = 0;
}

// Append a field with metadata (value age source)
static int tsv_field_meta(tsv_buf_t *t, const char *key, struct aircraft *a,
                          const data_validity *v, const char *fmt, ...) {
    const char *src = pa_source_char(v->source);
    if (!src) return 0;
    if (!trackDataValid(v)) return 0;
    if (v->updated > messageNow()) return 0;
    if (v->updated < a->fatsv_last_emitted) return 0;

    uint64_t age = (messageNow() - v->updated) / 1000;
    if (age > 255) return 0;

    int space = sizeof(t->buf) - t->pos;
    if (space < 10) return 0;

    int n = snprintf(t->buf + t->pos, space, "%s\t", key);
    t->pos += n;
    space -= n;

    va_list ap;
    va_start(ap, fmt);
    n = vsnprintf(t->buf + t->pos, space, fmt, ap);
    va_end(ap);
    t->pos += n;
    space -= n;

    n = snprintf(t->buf + t->pos, space, " %" PRIu64 " %s\t", age, src);
    t->pos += n;

    return 1;
}

// Finish TSV line: replace trailing tab with newline
static void tsv_finish(tsv_buf_t *t) {
    if (t->pos > 0 && t->buf[t->pos - 1] == '\t') {
        t->buf[t->pos - 1] = '\n';
    } else {
        if (t->pos < (int)sizeof(t->buf) - 1)
            t->buf[t->pos++] = '\n';
    }
    t->buf[t->pos] = 0;
}

// ============================================================================
// TLS Connection
// ============================================================================

static int paInitSSL(void) {
    SSL_library_init();
    SSL_load_error_strings();
    OpenSSL_add_all_algorithms();

    SSL_CTX *ctx = SSL_CTX_new(TLS_client_method());
    if (!ctx) {
        fprintf(stderr, "PiAware: SSL_CTX_new failed\n");
        return -1;
    }

    // Require TLS 1.0+, no SSLv2/SSLv3
    SSL_CTX_set_options(ctx, SSL_OP_NO_SSLv2 | SSL_OP_NO_SSLv3);

    // Load FlightAware CA certificates
    if (SSL_CTX_load_verify_locations(ctx, NULL, PiawareClient.ca_dir) != 1) {
        fprintf(stderr, "PiAware: failed to load CA certificates from %s\n", PiawareClient.ca_dir);
        SSL_CTX_free(ctx);
        return -1;
    }

    SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER, NULL);

    PiawareClient.ssl_ctx = ctx;
    return 0;
}

static void paConnect(void) {
    if (PiawareClient.state != PA_DISCONNECTED)
        return;

    struct addrinfo hints, *res, *rp;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    char portstr[16];
    snprintf(portstr, sizeof(portstr), "%d", PiawareClient.port);

    fprintf(stderr, "PiAware: connecting to %s:%d\n", PiawareClient.host, PiawareClient.port);

    int err = getaddrinfo(PiawareClient.host, portstr, &hints, &res);
    if (err != 0) {
        fprintf(stderr, "PiAware: DNS resolution failed for %s: %s\n",
                PiawareClient.host, gai_strerror(err));
        PiawareClient.next_reconnect = mstime() + PiawareClient.reconnect_interval;
        return;
    }

    int fd = -1;
    for (rp = res; rp != NULL; rp = rp->ai_next) {
        fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (fd < 0) continue;

        // Set non-blocking
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        // Enable TCP keepalive
        int yes = 1;
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));

        err = connect(fd, rp->ai_addr, rp->ai_addrlen);
        if (err == 0 || errno == EINPROGRESS) {
            break;  // connection initiated
        }
        close(fd);
        fd = -1;
    }
    freeaddrinfo(res);

    if (fd < 0) {
        fprintf(stderr, "PiAware: connection to %s:%d failed\n",
                PiawareClient.host, PiawareClient.port);
        PiawareClient.next_reconnect = mstime() + PiawareClient.reconnect_interval;
        return;
    }

    PiawareClient.fd = fd;
    PiawareClient.state = PA_CONNECTING;
    PiawareClient.login_deadline = mstime() + PA_LOGIN_TIMEOUT_MS;
}

static void paCheckConnect(void) {
    // Check if async connect completed
    int err = 0;
    socklen_t len = sizeof(err);
    if (getsockopt(PiawareClient.fd, SOL_SOCKET, SO_ERROR, &err, &len) < 0 || err != 0) {
        // Check if still in progress
        if (err == EINPROGRESS || err == EALREADY)
            return;
        paDisconnect(err ? strerror(err) : "getsockopt failed");
        return;
    }

    // Use select to check writability (connection complete)
    fd_set wset;
    struct timeval tv = {0, 0};
    FD_ZERO(&wset);
    FD_SET(PiawareClient.fd, &wset);
    int ret = select(PiawareClient.fd + 1, NULL, &wset, NULL, &tv);
    if (ret <= 0)
        return;  // not yet

    fprintf(stderr, "PiAware: TCP connected to %s:%d\n", PiawareClient.host, PiawareClient.port);

    // Start TLS handshake
    SSL *ssl = SSL_new((SSL_CTX *)PiawareClient.ssl_ctx);
    if (!ssl) {
        paDisconnect("SSL_new failed");
        return;
    }

    SSL_set_fd(ssl, PiawareClient.fd);
    SSL_set_tlsext_host_name(ssl, PiawareClient.host);
    PiawareClient.ssl = ssl;
    PiawareClient.state = PA_TLS_HANDSHAKE;
}

static void paTLSHandshake(void) {
    int ret = SSL_connect((SSL *)PiawareClient.ssl);
    if (ret == 1) {
        // Handshake complete
        fprintf(stderr, "PiAware: TLS handshake complete\n");

        // Verify certificate
        X509 *cert = SSL_get_peer_certificate((SSL *)PiawareClient.ssl);
        if (!cert) {
            paDisconnect("no server certificate");
            return;
        }
        X509_free(cert);

        long verify = SSL_get_verify_result((SSL *)PiawareClient.ssl);
        if (verify != X509_V_OK) {
            fprintf(stderr, "PiAware: certificate verification failed: %s\n",
                    X509_verify_cert_error_string(verify));
            paDisconnect("certificate verification failed");
            return;
        }

        fprintf(stderr, "PiAware: server certificate validated\n");

        // Configure socket for line-buffered I/O
        PiawareClient.state = PA_AWAITING_LOGIN;
        PiawareClient.inbuf_len = 0;

        paSendLogin();
        return;
    }

    int err = SSL_get_error((SSL *)PiawareClient.ssl, ret);
    if (err == SSL_ERROR_WANT_READ || err == SSL_ERROR_WANT_WRITE) {
        // Handshake in progress, try again later
        return;
    }

    // Handshake failed
    unsigned long e = ERR_get_error();
    char errbuf[256];
    ERR_error_string_n(e, errbuf, sizeof(errbuf));
    fprintf(stderr, "PiAware: TLS handshake failed: %s\n", errbuf);
    paDisconnect("TLS handshake failed");
}

static void paDisconnect(const char *reason) {
    if (PiawareClient.state == PA_DISCONNECTED)
        return;

    fprintf(stderr, "PiAware: disconnected (%s)\n", reason ? reason : "unknown");

    // Stop FA MLAT thread
    faMlatDisable();

    if (PiawareClient.ssl) {
        SSL_shutdown((SSL *)PiawareClient.ssl);
        SSL_free((SSL *)PiawareClient.ssl);
        PiawareClient.ssl = NULL;
    }

    if (PiawareClient.fd >= 0) {
        close(PiawareClient.fd);
        PiawareClient.fd = -1;
    }

    PiawareClient.state = PA_DISCONNECTED;
    PiawareClient.inbuf_len = 0;
    PiawareClient.next_reconnect = mstime() + PiawareClient.reconnect_interval;
}

// ============================================================================
// ADEPT Protocol
// ============================================================================

// Extract a field value from a TSV line (key\tval\tkey\tval\t...) without
// modifying the input string. Returns 1 if found, 0 if not.
static int pa_tsv_get(const char *line, const char *key, char *buf, int buflen) {
    int keylen = strlen(key);
    const char *scan = line;
    while (scan && *scan) {
        const char *k = scan;
        const char *tab1 = strchr(k, '\t');
        if (!tab1) break;
        int klen = (int)(tab1 - k);

        const char *v = tab1 + 1;
        const char *tab2 = strchr(v, '\t');
        int vlen = tab2 ? (int)(tab2 - v) : (int)strlen(v);

        if (klen == keylen && memcmp(k, key, keylen) == 0) {
            if (vlen >= buflen) vlen = buflen - 1;
            memcpy(buf, v, vlen);
            buf[vlen] = 0;
            return 1;
        }

        scan = tab2 ? tab2 + 1 : NULL;
    }
    buf[0] = 0;
    return 0;
}

static int paSend(const char *data, int len) {
    if (PiawareClient.state < PA_AWAITING_LOGIN || !PiawareClient.ssl)
        return -1;

    int written = SSL_write((SSL *)PiawareClient.ssl, data, len);
    if (written <= 0) {
        int err = SSL_get_error((SSL *)PiawareClient.ssl, written);
        if (err == SSL_ERROR_WANT_WRITE)
            return 0;  // retry later
        paDisconnect("write error");
        return -1;
    }
    return written;
}

// Send a type=log message to FlightAware (like real piaware does)
static void paSendLog(const char *fmt, ...) {
    if (PiawareClient.state != PA_LOGGED_IN) return;

    char message[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(message, sizeof(message), fmt, ap);
    va_end(ap);

    tsv_buf_t tsv;
    tsv_init(&tsv);
    tsv_field(&tsv, "type", "log");
    tsv_field(&tsv, "message", "%s", message);
    tsv_field(&tsv, "mac", "%s", PiawareClient.mac);
    tsv_field(&tsv, "clock", "%" PRIu64, (uint64_t)time(NULL));
    tsv_finish(&tsv);
    paSend(tsv.buf, tsv.pos);

    // Also log locally
    fprintf(stderr, "PiAware: LOG_SENT: %s\n", message);
}

static void paSendLogin(void) {
    tsv_buf_t tsv;
    tsv_init(&tsv);

    tsv_field(&tsv, "type", "login");
    tsv_field(&tsv, "mac", "%s", PiawareClient.mac);

    if (PiawareClient.feeder_id[0]) {
        tsv_field(&tsv, "feeder_id", "%s %s",
                  PiawareClient.feeder_id_source, PiawareClient.feeder_id);
    }

    tsv_field(&tsv, "piaware_version", "8.2");
    tsv_field(&tsv, "piaware_version_full", "8.2~bpo10+1");
    tsv_field(&tsv, "image_type", "piaware_package");
    tsv_field(&tsv, "connected_host", "%s", PiawareClient.host);

    if (Modes.fUserLat != 0 || Modes.fUserLon != 0) {
        tsv_field(&tsv, "receiverlat", "%.5f", Modes.fUserLat);
        tsv_field(&tsv, "receiverlon", "%.5f", Modes.fUserLon);
    }

    tsv_field(&tsv, "adsbprogram", "dump1090-gg");
    tsv_field(&tsv, "transprogram", "dump1090-gg-builtin");
    tsv_field(&tsv, "receiver_type", "rtlsdr");
    tsv_field(&tsv, "local_auto_update_enable", "1");
    tsv_field(&tsv, "local_manual_update_enable", "1");
    tsv_field(&tsv, "local_mlat_enable", "1");

    struct utsname uts;
    if (uname(&uts) == 0) {
        char uname_str[512];
        snprintf(uname_str, sizeof(uname_str), "%s %s %s %s %s",
                 uts.sysname, uts.nodename, uts.release, uts.version, uts.machine);
        tsv_field(&tsv, "uname", "%s", uname_str);
    }

    tsv_field(&tsv, "clock", "%" PRIu64, (uint64_t)time(NULL));

    tsv_finish(&tsv);

    fprintf(stderr, "PiAware: sending login (mac=%s, feeder_id=%s)\n",
            PiawareClient.mac, PiawareClient.feeder_id);

    paSend(tsv.buf, tsv.pos);
}

static void paHandleLoginResponse(const char *line) {
    char status[64], reason[256], user[128], feeder_id[128], site_url[256];

    if (!pa_tsv_get(line, "status", status, sizeof(status))) {
        paDisconnect("missing status in login_response");
        return;
    }

    if (strcmp(status, "ok") != 0) {
        pa_tsv_get(line, "reason", reason, sizeof(reason));
        fprintf(stderr, "PiAware: login FAILED: %s: %s\n",
                status, reason[0] ? reason : "unknown");
        paDisconnect("login rejected");
        return;
    }

    PiawareClient.state = PA_LOGGED_IN;
    PiawareClient.reconnect_interval = PA_FAST_RECONNECT_MS;
    PiawareClient.alive_deadline = mstime() + PA_ALIVE_TIMEOUT_MS;
    PiawareClient.next_fatsv = mstime() + 1000;
    PiawareClient.next_health = mstime() + PA_HEALTH_INTERVAL_MS;
    PiawareClient.msgs_sent = 0;

    pa_tsv_get(line, "user", user, sizeof(user));
    fprintf(stderr, "PiAware: logged in as %s\n", user[0] ? user : "unknown");

    if (pa_tsv_get(line, "feeder_id", feeder_id, sizeof(feeder_id)))
        fprintf(stderr, "PiAware: feeder ID %s\n", feeder_id);
    if (pa_tsv_get(line, "site_url", site_url, sizeof(site_url)))
        fprintf(stderr, "PiAware: site URL %s\n", site_url);

    // Send log messages like real piaware does after login
    paSendLog("logged in to FlightAware as user %s", user[0] ? user : "unknown");
    if (feeder_id[0])
        paSendLog("my feeder ID is %s", feeder_id);
    if (site_url[0])
        paSendLog("site statistics URL: %s", site_url);
    paSendLog("ADS-B data program 'dump1090-gg' is listening on port 30005");
}

static void paHandleAlive(const char *line) {
    char interval_str[32];
    int interval = 300;

    if (pa_tsv_get(line, "interval", interval_str, sizeof(interval_str)))
        interval = atoi(interval_str);

    // Reset alive timeout (1.2x interval)
    int timeout_ms = (int)(interval * 1200);
    if (timeout_ms < PA_ALIVE_TIMEOUT_MS)
        timeout_ms = PA_ALIVE_TIMEOUT_MS;
    PiawareClient.alive_deadline = mstime() + timeout_ms;

    // Respond with our clock
    tsv_buf_t tsv;
    tsv_init(&tsv);
    tsv_field(&tsv, "type", "alive");
    tsv_field(&tsv, "clock", "%" PRIu64, (uint64_t)time(NULL));
    tsv_finish(&tsv);
    paSend(tsv.buf, tsv.pos);
}

static void paHandleMlatEnable(const char *line) {
    char transport[256];

    if (!pa_tsv_get(line, "udp_transport", transport, sizeof(transport))) {
        fprintf(stderr, "PiAware: mlat_enable without udp_transport, ignoring\n");
        return;
    }

    // Parse space-separated "host port key"
    char host[128], portstr[16], keystr[64];
    keystr[0] = 0;
    int fields = sscanf(transport, "%127s %15s %63s", host, portstr, keystr);
    if (fields < 2) {
        fprintf(stderr, "PiAware: mlat_enable: invalid udp_transport format: %s\n", transport);
        return;
    }

    int port = atoi(portstr);
    uint32_t key = keystr[0] ? (uint32_t)strtoul(keystr, NULL, 10) : 0;

    fprintf(stderr, "PiAware: MLAT enabled (udp %s:%d key=%u)\n", host, port, key);

    // Start the built-in FA MLAT thread
    faMlatEnable(host, port, key);
}

static void paHandleLine(const char *line) {
    // Extract type field without modifying the line
    char type[64];
    if (!pa_tsv_get(line, "type", type, sizeof(type)))
        return;

    // Log ALL incoming server messages (truncate long lines)
    if (strcmp(type, "alive") != 0) {
        // Log everything except alive (too frequent)
        fprintf(stderr, "PiAware: SERVER_CMD type=%s | %.512s\n", type, line);
        if (PanelState.enabled)
            panelLog("PiAware: %s", type);
    }

    if (strcmp(type, "login_response") == 0) {
        paHandleLoginResponse(line);
    } else if (strcmp(type, "alive") == 0) {
        paHandleAlive(line);
    } else if (strcmp(type, "notice") == 0) {
        char msg[512];
        if (pa_tsv_get(line, "message", msg, sizeof(msg))) {
            fprintf(stderr, "PiAware: NOTICE: %s\n", msg);
            if (PanelState.enabled)
                panelLog("PiAware NOTICE: %s", msg);
        }
    } else if (strcmp(type, "shutdown") == 0) {
        fprintf(stderr, "PiAware: server shutting down\n");
        if (PanelState.enabled)
            panelLog("PiAware: server shutting down!");
        paDisconnect("server shutdown");
    } else if (strcmp(type, "mlat_enable") == 0) {
        paHandleMlatEnable(line);
    } else if (strcmp(type, "mlat_disable") == 0) {
        fprintf(stderr, "PiAware: MLAT disabled by server\n");
        faMlatDisable();
    } else if (strcmp(type, "mlat_wanted") == 0) {
        paHandleMlatWanted(line);
    } else if (strcmp(type, "mlat_unwanted") == 0) {
        paHandleMlatUnwanted(line);
    } else if (strcmp(type, "mlat_result") == 0) {
        paHandleMlatResult(line);
    } else if (strcmp(type, "request_manual_update") == 0) {
        char action[256];
        if (pa_tsv_get(line, "action", action, sizeof(action))) {
            fprintf(stderr, "PiAware: REQUEST_MANUAL_UPDATE action=%s\n", action);
            if (PanelState.enabled)
                panelLog("PiAware: REMOTE CMD action=%s", action);
            paSendLog("manual update (user-initiated via their flightaware control page) requested by adept server");
            paSendLog("performing manual update, action: %s", action);
            // Execute supported actions
            if (strstr(action, "restart_piaware") || strstr(action, "restart_dump1090") || strstr(action, "restart_receiver")) {
                paSendLog("restart requested, but dump1090-gg manages itself - ignoring restart action");
            } else if (strstr(action, "reboot")) {
                paSendLog("reboot requested via manual update");
                fprintf(stderr, "PiAware: REBOOT requested by FlightAware server!\n");
            } else if (strstr(action, "halt")) {
                paSendLog("halt requested via manual update");
                fprintf(stderr, "PiAware: HALT requested by FlightAware server!\n");
            } else if (strstr(action, "piaware") || strstr(action, "dump1090") || strstr(action, "packages") || strstr(action, "full")) {
                paSendLog("upgrade action '%s' received but dump1090-gg does not support remote upgrades", action);
            } else {
                paSendLog("unknown manual update action: %s", action);
            }
        } else {
            fprintf(stderr, "PiAware: REQUEST_MANUAL_UPDATE (no action field)\n");
        }
    } else if (strcmp(type, "request_auto_update") == 0) {
        char action[256];
        if (pa_tsv_get(line, "action", action, sizeof(action))) {
            fprintf(stderr, "PiAware: REQUEST_AUTO_UPDATE action=%s\n", action);
            paSendLog("auto update requested by adept server, action: %s", action);
            paSendLog("auto update action '%s' received but dump1090-gg does not support remote upgrades", action);
        } else {
            fprintf(stderr, "PiAware: REQUEST_AUTO_UPDATE (no action field)\n");
        }
    } else {
        fprintf(stderr, "PiAware: UNKNOWN_CMD type=%s | %.512s\n", type, line);
    }
}

static void paHandleInput(void) {
    if (!PiawareClient.ssl) return;

    // Read available data
    int space = sizeof(PiawareClient.inbuf) - PiawareClient.inbuf_len - 1;
    if (space <= 0) {
        // Buffer overflow, discard
        PiawareClient.inbuf_len = 0;
        return;
    }

    int n = SSL_read((SSL *)PiawareClient.ssl,
                     PiawareClient.inbuf + PiawareClient.inbuf_len, space);
    if (n > 0) {
        PiawareClient.inbuf_len += n;
        PiawareClient.inbuf[PiawareClient.inbuf_len] = 0;

        // Process complete lines
        char *start = PiawareClient.inbuf;
        char *nl;
        while ((nl = strchr(start, '\n')) != NULL) {
            *nl = 0;
            // Make a copy for parsing (paHandleLine modifies the string)
            char linebuf[8192];
            strncpy(linebuf, start, sizeof(linebuf) - 1);
            linebuf[sizeof(linebuf) - 1] = 0;
            paHandleLine(linebuf);
            start = nl + 1;
        }

        // Move remaining data to start of buffer
        int remaining = PiawareClient.inbuf_len - (start - PiawareClient.inbuf);
        if (remaining > 0 && start != PiawareClient.inbuf) {
            memmove(PiawareClient.inbuf, start, remaining);
        }
        PiawareClient.inbuf_len = remaining;
    } else {
        int err = SSL_get_error((SSL *)PiawareClient.ssl, n);
        if (err == SSL_ERROR_WANT_READ)
            return;  // no data yet
        if (err == SSL_ERROR_ZERO_RETURN) {
            paDisconnect("server closed connection");
            return;
        }
        paDisconnect("read error");
    }
}

// ============================================================================
// MLAT Message Handlers (route FA server messages to built-in FA MLAT thread)
// ============================================================================

// Parse hex ID list "AABBCC DDEEFF ..." into uint32_t array.
// IDs starting with '@' are Mode A/C codes.
static void pa_parse_hexid_list(const char *hexids,
                                 uint32_t *icao, int *icao_count, int icao_max,
                                 uint32_t *modeac, int *modeac_count, int modeac_max) {
    *icao_count = 0;
    *modeac_count = 0;
    if (!hexids || hexids[0] == 0) return;

    const char *p = hexids;
    while (*p) {
        while (*p == ' ') p++;
        if (!*p) break;

        if (*p == '@') {
            // Mode A/C code
            p++;
            uint32_t code = (uint32_t)strtoul(p, (char **)&p, 16);
            if (*modeac_count < modeac_max)
                modeac[(*modeac_count)++] = code;
        } else {
            uint32_t addr = (uint32_t)strtoul(p, (char **)&p, 16);
            if (addr != 0 && *icao_count < icao_max)
                icao[(*icao_count)++] = addr;
        }
    }
}

static void paHandleMlatWanted(const char *line) {
    char hexids[4096];
    if (!pa_tsv_get(line, "hexids", hexids, sizeof(hexids)))
        return;

    uint32_t icao[FA_MLAT_MAX_WANTED];
    int icao_count = 0;
    uint32_t modeac[FA_MLAT_MAX_MODEAC];
    int modeac_count = 0;

    pa_parse_hexid_list(hexids, icao, &icao_count, FA_MLAT_MAX_WANTED,
                        modeac, &modeac_count, FA_MLAT_MAX_MODEAC);

    faMlatStartSending(icao, icao_count, modeac, modeac_count);
}

static void paHandleMlatUnwanted(const char *line) {
    char hexids[4096];
    if (!pa_tsv_get(line, "hexids", hexids, sizeof(hexids)))
        return;

    uint32_t icao[FA_MLAT_MAX_WANTED];
    int icao_count = 0;
    uint32_t modeac[FA_MLAT_MAX_MODEAC];
    int modeac_count = 0;

    pa_parse_hexid_list(hexids, icao, &icao_count, FA_MLAT_MAX_WANTED,
                        modeac, &modeac_count, FA_MLAT_MAX_MODEAC);

    faMlatStopSending(icao, icao_count, modeac, modeac_count);
}

static void paHandleMlatResult(const char *line) {
    char hexid[16], lat_str[32], lon_str[32], alt_str[32];
    char nsvel_str[32], ewvel_str[32], fpm_str[32];
    char anon_str[8], modeac_str[8];

    if (!pa_tsv_get(line, "hexid", hexid, sizeof(hexid)))
        return;

    uint32_t addr = (uint32_t)strtoul(hexid, NULL, 16);
    if (addr == 0) return;

    double lat = 0, lon = 0, alt = 0;
    double nsvel = 0, ewvel = 0, vrate = 0;
    int anon = 0, modeac_flag = 0;

    if (pa_tsv_get(line, "lat", lat_str, sizeof(lat_str)))
        lat = atof(lat_str);
    if (pa_tsv_get(line, "lon", lon_str, sizeof(lon_str)))
        lon = atof(lon_str);
    if (pa_tsv_get(line, "alt", alt_str, sizeof(alt_str)))
        alt = atof(alt_str);
    if (pa_tsv_get(line, "nsvel", nsvel_str, sizeof(nsvel_str)))
        nsvel = atof(nsvel_str);
    if (pa_tsv_get(line, "ewvel", ewvel_str, sizeof(ewvel_str)))
        ewvel = atof(ewvel_str);
    if (pa_tsv_get(line, "fpm", fpm_str, sizeof(fpm_str)))
        vrate = atof(fpm_str);
    if (pa_tsv_get(line, "anon", anon_str, sizeof(anon_str)))
        anon = atoi(anon_str);
    if (pa_tsv_get(line, "modeac", modeac_str, sizeof(modeac_str)))
        modeac_flag = atoi(modeac_str);

    faMlatInjectResult(addr, lat, lon, alt, nsvel, ewvel, vrate, anon, modeac_flag);
}

// Forward status messages from FA MLAT thread to FA server
static void paForwardFaMlatStatus(void) {
    char line[FA_MLAT_STATUS_LINE_LEN];

    while (faMlatPollStatus(line, sizeof(line))) {
        if (PiawareClient.state == PA_LOGGED_IN && line[0] != 0) {
            char sendbuf[FA_MLAT_STATUS_LINE_LEN + 2];
            int len = snprintf(sendbuf, sizeof(sendbuf), "%s\n", line);
            if (len > 0 && len < (int)sizeof(sendbuf)) {
                paSend(sendbuf, len);
            }
        }
    }
}

// ============================================================================
// FATSV Data Generation
// ============================================================================

#define PA_TSV_VERSION "9E"

static void paSendFATSV(void) {
    uint64_t now = mstime();
    struct aircraft *a;

    if (PiawareClient.state != PA_LOGGED_IN)
        return;

    if (now < PiawareClient.next_fatsv)
        return;

    PiawareClient.next_fatsv = now + PA_FATSV_INTERVAL_MS;

    for (a = Modes.aircrafts; a; a = a->next) {
        if (!a->reliable)
            continue;
        if (a->seen < a->fatsv_last_emitted)
            continue;

        _messageNow = a->seen;

        int altValid = trackDataValid(&a->altitude_baro_valid);
        int airgroundValid = trackDataValid(&a->airground_valid) && a->airground_valid.source >= SOURCE_MODE_S_CHECKED;
        int gsValid = trackDataValid(&a->gs_valid);
        int squawkValid = trackDataValid(&a->squawk_valid);
        int callsignValid = trackDataValid(&a->callsign_valid) && strcmp(a->callsign, "        ") != 0;
        int positionValid = trackDataValid(&a->position_valid);

        if (airgroundValid && a->airground == AG_GROUND && a->altitude_baro_valid.source < SOURCE_MODE_S_CHECKED)
            altValid = 0;

        // Change detection (same logic as writeFATSV in net_io.c)
        int changed =
            (altValid && abs(a->altitude_baro - a->fatsv_emitted_altitude_baro) >= 50) ||
            (trackDataValid(&a->altitude_geom_valid) && abs(a->altitude_geom - a->fatsv_emitted_altitude_geom) >= 50) ||
            (trackDataValid(&a->baro_rate_valid) && abs(a->baro_rate - a->fatsv_emitted_baro_rate) > 500) ||
            (trackDataValid(&a->geom_rate_valid) && abs(a->geom_rate - a->fatsv_emitted_geom_rate) > 500) ||
            (trackDataValid(&a->track_valid) && heading_difference(a->track, a->fatsv_emitted_track) >= 2) ||
            (trackDataValid(&a->track_rate_valid) && fabs(a->track_rate - a->fatsv_emitted_track_rate) >= 0.5) ||
            (trackDataValid(&a->roll_valid) && fabs(a->roll - a->fatsv_emitted_roll) >= 5.0) ||
            (trackDataValid(&a->mag_heading_valid) && heading_difference(a->mag_heading, a->fatsv_emitted_mag_heading) >= 2) ||
            (trackDataValid(&a->true_heading_valid) && heading_difference(a->true_heading, a->fatsv_emitted_true_heading) >= 2) ||
            (gsValid && fabs(a->gs - a->fatsv_emitted_gs) >= 25) ||
            (trackDataValid(&a->ias_valid) && abs((int)a->ias - (int)a->fatsv_emitted_ias) >= 25) ||
            (trackDataValid(&a->tas_valid) && abs((int)a->tas - (int)a->fatsv_emitted_tas) >= 25) ||
            (trackDataValid(&a->mach_valid) && fabs(a->mach - a->fatsv_emitted_mach) >= 0.02);

        int immediate =
            (trackDataValid(&a->nav_altitude_mcp_valid) && abs(a->nav_altitude_mcp - a->fatsv_emitted_nav_altitude_mcp) > 50) ||
            (trackDataValid(&a->nav_altitude_fms_valid) && abs(a->nav_altitude_fms - a->fatsv_emitted_nav_altitude_fms) > 50) ||
            (trackDataValid(&a->nav_altitude_src_valid) && a->nav_altitude_src != a->fatsv_emitted_nav_altitude_src) ||
            (trackDataValid(&a->nav_heading_valid) && heading_difference(a->nav_heading, a->fatsv_emitted_nav_heading) > 2) ||
            (trackDataValid(&a->nav_modes_valid) && a->nav_modes != a->fatsv_emitted_nav_modes) ||
            (trackDataValid(&a->nav_qnh_valid) && fabs(a->nav_qnh - a->fatsv_emitted_nav_qnh) > 0.8) ||
            (callsignValid && strcmp(a->callsign, a->fatsv_emitted_callsign) != 0) ||
            (airgroundValid && a->airground == AG_AIRBORNE && a->fatsv_emitted_airground == AG_GROUND) ||
            (airgroundValid && a->airground == AG_GROUND && a->fatsv_emitted_airground == AG_AIRBORNE) ||
            (squawkValid && a->squawk != a->fatsv_emitted_squawk) ||
            (trackDataValid(&a->emergency_valid) && a->emergency != a->fatsv_emitted_emergency);

        uint64_t minAge;
        if (immediate) {
            minAge = 0;
        } else if (!positionValid) {
            minAge = 30000;
        } else if ((airgroundValid && a->airground == AG_GROUND) ||
                   (altValid && a->altitude_baro < 500 && (!gsValid || a->gs < 200)) ||
                   (gsValid && a->gs < 100 && (!altValid || a->altitude_baro < 1000))) {
            minAge = 1000;
        } else if (!altValid || a->altitude_baro < 10000) {
            minAge = changed ? 5000 : 10000;
        } else {
            minAge = changed ? 10000 : 30000;
        }

        if ((now - a->fatsv_last_emitted) < minAge)
            continue;

        // Build TSV message
        tsv_buf_t tsv;
        tsv_init(&tsv);

        tsv_field(&tsv, "_v", PA_TSV_VERSION);
        tsv_field(&tsv, "clock", "%" PRIu64, messageNow() / 1000);
        tsv_field(&tsv, (a->addr & MODES_NON_ICAO_ADDRESS) ? "otherid" : "hexid",
                  "%06X", a->addr & 0xFFFFFF);

        int forceEmit = (now - a->fatsv_last_force_emit) > 600000;

        if (forceEmit || a->addrtype != a->fatsv_emitted_addrtype)
            tsv_field(&tsv, "addrtype", "%s", pa_addrtype_str(a->addrtype));
        if (forceEmit || a->adsb_version != a->fatsv_emitted_adsb_version)
            tsv_field(&tsv, "adsb_version", "%d", a->adsb_version);
        if (forceEmit || a->category != a->fatsv_emitted_category)
            tsv_field(&tsv, "category", "%02X", a->category);

        if (trackDataValid(&a->nac_p_valid) && (forceEmit || a->nac_p != a->fatsv_emitted_nac_p))
            tsv_field_meta(&tsv, "nac_p", a, &a->nac_p_valid, "%u", a->nac_p);
        if (trackDataValid(&a->nac_v_valid) && (forceEmit || a->nac_v != a->fatsv_emitted_nac_v))
            tsv_field_meta(&tsv, "nac_v", a, &a->nac_v_valid, "%u", a->nac_v);
        if (trackDataValid(&a->sil_valid) && (forceEmit || a->sil != a->fatsv_emitted_sil))
            tsv_field_meta(&tsv, "sil", a, &a->sil_valid, "%u", a->sil);
        if (trackDataValid(&a->sil_valid) && (forceEmit || a->sil_type != a->fatsv_emitted_sil_type))
            tsv_field_meta(&tsv, "sil_type", a, &a->sil_valid, "%s", pa_sil_type_str(a->sil_type));
        if (trackDataValid(&a->nic_baro_valid) && (forceEmit || a->nic_baro != a->fatsv_emitted_nic_baro))
            tsv_field_meta(&tsv, "nic_baro", a, &a->nic_baro_valid, "%u", (unsigned)a->nic_baro);

        // Data fields
        int hadData = tsv.pos;

        if (airgroundValid)
            tsv_field_meta(&tsv, "airGround", a, &a->airground_valid, "%s", pa_airground_str(a->airground));
        if (squawkValid)
            tsv_field_meta(&tsv, "squawk", a, &a->squawk_valid, "%04x", a->squawk);
        if (callsignValid)
            tsv_field_meta(&tsv, "ident", a, &a->callsign_valid, "{%s}", a->callsign);
        if (altValid)
            tsv_field_meta(&tsv, "alt", a, &a->altitude_baro_valid, "%d", a->altitude_baro);
        if (positionValid)
            tsv_field_meta(&tsv, "position", a, &a->position_valid, "{%.5f %.5f %u %u}",
                          a->lat, a->lon, a->pos_nic, a->pos_rc);

        tsv_field_meta(&tsv, "alt_gnss", a, &a->altitude_geom_valid, "%d", a->altitude_geom);
        tsv_field_meta(&tsv, "vrate", a, &a->baro_rate_valid, "%d", a->baro_rate);
        tsv_field_meta(&tsv, "vrate_geom", a, &a->geom_rate_valid, "%d", a->geom_rate);
        tsv_field_meta(&tsv, "speed", a, &a->gs_valid, "%.1f", a->gs);
        tsv_field_meta(&tsv, "speed_ias", a, &a->ias_valid, "%u", a->ias);
        tsv_field_meta(&tsv, "speed_tas", a, &a->tas_valid, "%u", a->tas);
        tsv_field_meta(&tsv, "mach", a, &a->mach_valid, "%.3f", a->mach);
        tsv_field_meta(&tsv, "track", a, &a->track_valid, "%.1f", a->track);
        tsv_field_meta(&tsv, "track_rate", a, &a->track_rate_valid, "%.2f", a->track_rate);
        tsv_field_meta(&tsv, "roll", a, &a->roll_valid, "%.1f", a->roll);
        tsv_field_meta(&tsv, "heading_magnetic", a, &a->mag_heading_valid, "%.1f", a->mag_heading);
        tsv_field_meta(&tsv, "heading_true", a, &a->true_heading_valid, "%.1f", a->true_heading);
        tsv_field_meta(&tsv, "nav_alt_mcp", a, &a->nav_altitude_mcp_valid, "%d", a->nav_altitude_mcp);
        tsv_field_meta(&tsv, "nav_alt_fms", a, &a->nav_altitude_fms_valid, "%d", a->nav_altitude_fms);
        tsv_field_meta(&tsv, "nav_alt_src", a, &a->nav_altitude_src_valid, "%s", pa_nav_alt_src_str(a->nav_altitude_src));
        tsv_field_meta(&tsv, "nav_heading", a, &a->nav_heading_valid, "%.1f", a->nav_heading);
        tsv_field_meta(&tsv, "nav_qnh", a, &a->nav_qnh_valid, "%.1f", a->nav_qnh);
        tsv_field_meta(&tsv, "emergency", a, &a->emergency_valid, "%s", pa_emergency_str(a->emergency));

        // Skip if no data fields were added
        if (tsv.pos == hadData)
            continue;

        tsv_finish(&tsv);

        if (paSend(tsv.buf, tsv.pos) > 0) {
            PiawareClient.msgs_sent++;

            // Update tracking state
            a->fatsv_emitted_altitude_baro = a->altitude_baro;
            a->fatsv_emitted_altitude_geom = a->altitude_geom;
            a->fatsv_emitted_baro_rate = a->baro_rate;
            a->fatsv_emitted_geom_rate = a->geom_rate;
            a->fatsv_emitted_gs = a->gs;
            a->fatsv_emitted_ias = a->ias;
            a->fatsv_emitted_tas = a->tas;
            a->fatsv_emitted_mach = a->mach;
            a->fatsv_emitted_track = a->track;
            a->fatsv_emitted_track_rate = a->track_rate;
            a->fatsv_emitted_roll = a->roll;
            a->fatsv_emitted_mag_heading = a->mag_heading;
            a->fatsv_emitted_true_heading = a->true_heading;
            a->fatsv_emitted_airground = a->airground;
            a->fatsv_emitted_nav_altitude_mcp = a->nav_altitude_mcp;
            a->fatsv_emitted_nav_altitude_fms = a->nav_altitude_fms;
            a->fatsv_emitted_nav_altitude_src = a->nav_altitude_src;
            a->fatsv_emitted_nav_heading = a->nav_heading;
            a->fatsv_emitted_nav_modes = a->nav_modes;
            a->fatsv_emitted_nav_qnh = a->nav_qnh;
            memcpy(a->fatsv_emitted_callsign, a->callsign, sizeof(a->fatsv_emitted_callsign));
            a->fatsv_emitted_addrtype = a->addrtype;
            a->fatsv_emitted_adsb_version = a->adsb_version;
            a->fatsv_emitted_category = a->category;
            a->fatsv_emitted_squawk = a->squawk;
            a->fatsv_emitted_nac_p = a->nac_p;
            a->fatsv_emitted_nac_v = a->nac_v;
            a->fatsv_emitted_sil = a->sil;
            a->fatsv_emitted_sil_type = a->sil_type;
            a->fatsv_emitted_nic_baro = a->nic_baro;
            a->fatsv_emitted_emergency = a->emergency;
            a->fatsv_last_emitted = now;
            if (forceEmit)
                a->fatsv_last_force_emit = now;
        }
    }
}

// ============================================================================
// Health
// ============================================================================

static void paSendHealth(void) {
    uint64_t now = mstime();
    if (PiawareClient.state != PA_LOGGED_IN)
        return;
    if (now < PiawareClient.next_health)
        return;

    PiawareClient.next_health = now + PA_HEALTH_INTERVAL_MS;

    tsv_buf_t tsv;
    tsv_init(&tsv);

    tsv_field(&tsv, "type", "health");
    tsv_field(&tsv, "clock", "%" PRIu64, (uint64_t)time(NULL));

    // CPU temperature
    FILE *f = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (f) {
        int temp;
        if (fscanf(f, "%d", &temp) == 1)
            tsv_field(&tsv, "cputemp", "%.1f", temp / 1000.0);
        fclose(f);
    }

    // Uptime
    f = fopen("/proc/uptime", "r");
    if (f) {
        double uptime;
        if (fscanf(f, "%lf", &uptime) == 1)
            tsv_field(&tsv, "uptime", "%.0f", uptime);
        fclose(f);
    }

    // Load average
    f = fopen("/proc/loadavg", "r");
    if (f) {
        double load1;
        if (fscanf(f, "%lf", &load1) == 1)
            tsv_field(&tsv, "cpuload", "%.2f", load1);
        fclose(f);
    }

    if (Modes.fUserLat != 0 || Modes.fUserLon != 0) {
        tsv_field(&tsv, "receiverlat", "%.5f", Modes.fUserLat);
        tsv_field(&tsv, "receiverlon", "%.5f", Modes.fUserLon);
    }

    tsv_finish(&tsv);
    paSend(tsv.buf, tsv.pos);
}

// ============================================================================
// Public API
// ============================================================================

void piawareClientInit(void) {
    if (!PiawareClient.enabled)
        return;

    // Set defaults
    if (!PiawareClient.host[0])
        strncpy(PiawareClient.host, PA_DEFAULT_HOST, sizeof(PiawareClient.host) - 1);
    if (!PiawareClient.port)
        PiawareClient.port = PA_DEFAULT_PORT;
    if (!PiawareClient.ca_dir[0])
        strncpy(PiawareClient.ca_dir, PA_DEFAULT_CA_DIR, sizeof(PiawareClient.ca_dir) - 1);
    if (!PiawareClient.feeder_id_file[0])
        strncpy(PiawareClient.feeder_id_file, PA_DEFAULT_FEEDER_FILE, sizeof(PiawareClient.feeder_id_file) - 1);

    PiawareClient.fd = -1;
    PiawareClient.state = PA_DISCONNECTED;
    PiawareClient.reconnect_interval = PA_FAST_RECONNECT_MS;

    // Auto-detect MAC address
    paDetectMAC();
    fprintf(stderr, "PiAware: MAC address: %s\n", PiawareClient.mac);

    // Read feeder ID (skip if already set via --piaware-feeder-id)
    if (PiawareClient.feeder_id[0]) {
        fprintf(stderr, "PiAware: feeder ID (from config): %s\n", PiawareClient.feeder_id);
    } else {
        paReadFeederID();
        if (PiawareClient.feeder_id[0])
            fprintf(stderr, "PiAware: feeder ID (from file): %s\n", PiawareClient.feeder_id);
        else
            fprintf(stderr, "PiAware: WARNING: no feeder ID found at %s\n", PiawareClient.feeder_id_file);
    }

    // Initialize OpenSSL
    if (paInitSSL() < 0) {
        fprintf(stderr, "PiAware: SSL initialization failed, disabling\n");
        PiawareClient.enabled = 0;
        return;
    }

    fprintf(stderr, "PiAware: client initialized, connecting to %s:%d\n",
            PiawareClient.host, PiawareClient.port);

    // Initialize FA MLAT subsystem
    faMlatInit();

    // Start first connection attempt
    PiawareClient.next_reconnect = mstime();
}

void piawareClientPeriodicWork(void) {
    if (!PiawareClient.enabled)
        return;

    uint64_t now = mstime();

    switch (PiawareClient.state) {
    case PA_DISCONNECTED:
        if (now >= PiawareClient.next_reconnect) {
            paConnect();
        }
        break;

    case PA_CONNECTING:
        if (now > PiawareClient.login_deadline) {
            paDisconnect("connection timeout");
            break;
        }
        paCheckConnect();
        break;

    case PA_TLS_HANDSHAKE:
        if (now > PiawareClient.login_deadline) {
            paDisconnect("TLS handshake timeout");
            break;
        }
        paTLSHandshake();
        break;

    case PA_AWAITING_LOGIN:
        if (now > PiawareClient.login_deadline) {
            paDisconnect("login timeout");
            break;
        }
        paHandleInput();
        break;

    case PA_LOGGED_IN:
        // Check alive timeout
        if (now > PiawareClient.alive_deadline) {
            paDisconnect("alive timeout");
            break;
        }

        // Handle incoming messages
        paHandleInput();

        // Send FATSV data
        paSendFATSV();

        // Send health info
        paSendHealth();
        break;
    }

    // Forward FA MLAT status messages to FA server
    paForwardFaMlatStatus();
}

void piawareClientCleanup(void) {
    faMlatCleanup();
    if (PiawareClient.ssl) {
        SSL_shutdown((SSL *)PiawareClient.ssl);
        SSL_free((SSL *)PiawareClient.ssl);
        PiawareClient.ssl = NULL;
    }
    if (PiawareClient.fd >= 0) {
        close(PiawareClient.fd);
        PiawareClient.fd = -1;
    }
    if (PiawareClient.ssl_ctx) {
        SSL_CTX_free((SSL_CTX *)PiawareClient.ssl_ctx);
        PiawareClient.ssl_ctx = NULL;
    }
    PiawareClient.state = PA_DISCONNECTED;
}
