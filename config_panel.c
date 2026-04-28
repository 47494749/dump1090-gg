// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// config_panel.c: Built-in configuration and monitoring web panel
//
// Provides a minimal HTTP server on a configurable port with:
//   - Configuration page for all dump1090-gg settings
//   - Feeder connection status with links
//   - Real-time log viewer
//   - Decoded message viewer
//   - Live aircraft table
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include "dump1090.h"
#include "config_panel.h"
#include "sdr_rtlsdr.h"
#include "sdr.h"
#include "fifo.h"
#include "opensky_client.h"
#include "sondehub_client.h"

#ifdef ENABLE_RTLSDR
#include <rtl-sdr.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdarg.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <limits.h>

panel_state_t PanelState;

// Sonde feed config (radiosondy.info and wettersonde.net — no C client yet, UI-only)
static bool RadiosondyEnabled = false;
static bool WettersondeEnabled = false;

// ============================= Initialization ============================

void panelInitConfig(void)
{
    memset(&PanelState, 0, sizeof(PanelState));
    PanelState.port = PANEL_DEFAULT_PORT;
    PanelState.listen_fd = -1;
    snprintf(PanelState.html_dir, sizeof(PanelState.html_dir), "%s", PANEL_HTML_DIR);
    pthread_mutex_init(&PanelState.log_mutex, NULL);
    pthread_mutex_init(&PanelState.msg_mutex, NULL);
}

// ============================= CLI Options ================================

bool panelHandleOption(int argc, char **argv, int *jptr)
{
    int j = *jptr;
    bool more = (j + 1 < argc);

    if (!strcmp(argv[j], "--panel")) {
        PanelState.enabled = 1;
    } else if (!strcmp(argv[j], "--panel-port") && more) {
        PanelState.enabled = 1;
        PanelState.port = atoi(argv[++j]);
    } else if (!strcmp(argv[j], "--panel-password") && more) {
        PanelState.enabled = 1;
        snprintf(PanelState.password, sizeof(PanelState.password), "%s", argv[++j]);
    } else if (!strcmp(argv[j], "--panel-html-dir") && more) {
        snprintf(PanelState.html_dir, sizeof(PanelState.html_dir), "%s", argv[++j]);
    } else {
        return false;
    }

    *jptr = j;
    return true;
}

// ============================= Ring Buffers ==============================

void panelLog(const char *fmt, ...)
{
    char line[PANEL_LOG_LINE_LEN];
    va_list ap;

    // Format timestamp + message
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    gmtime_r(&ts.tv_sec, &tm);
    int off = snprintf(line, sizeof(line), "%02d:%02d:%02d.%03d ",
                       tm.tm_hour, tm.tm_min, tm.tm_sec, (int)(ts.tv_nsec / 1000000));

    va_start(ap, fmt);
    vsnprintf(line + off, sizeof(line) - (size_t)off, fmt, ap);
    va_end(ap);

    // Store in ring buffer (no stderr — caller handles that)
    pthread_mutex_lock(&PanelState.log_mutex);
    int idx = (PanelState.log_head + PanelState.log_count) % PANEL_LOG_LINES;
    if (PanelState.log_count >= PANEL_LOG_LINES) {
        PanelState.log_head = (PanelState.log_head + 1) % PANEL_LOG_LINES;
    } else {
        PanelState.log_count++;
    }
    snprintf(PanelState.log_buf[idx], PANEL_LOG_LINE_LEN, "%s", line);
    PanelState.log_seq++;
    pthread_mutex_unlock(&PanelState.log_mutex);
}

void panelLogMessage(const char *fmt, ...)
{
    char line[PANEL_MSG_LINE_LEN];
    va_list ap;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    gmtime_r(&ts.tv_sec, &tm);
    int off = snprintf(line, sizeof(line), "%02d:%02d:%02d.%03d ",
                       tm.tm_hour, tm.tm_min, tm.tm_sec, (int)(ts.tv_nsec / 1000000));

    va_start(ap, fmt);
    vsnprintf(line + off, sizeof(line) - (size_t)off, fmt, ap);
    va_end(ap);

    pthread_mutex_lock(&PanelState.msg_mutex);
    int idx = (PanelState.msg_head + PanelState.msg_count) % PANEL_MSG_LINES;
    if (PanelState.msg_count >= PANEL_MSG_LINES) {
        PanelState.msg_head = (PanelState.msg_head + 1) % PANEL_MSG_LINES;
    } else {
        PanelState.msg_count++;
    }
    snprintf(PanelState.msg_buf[idx], PANEL_MSG_LINE_LEN, "%s", line);
    PanelState.msg_seq++;
    pthread_mutex_unlock(&PanelState.msg_mutex);
}

// ============================= Base64 decode =============================

static const unsigned char b64_table[256] = {
    ['A']=0,['B']=1,['C']=2,['D']=3,['E']=4,['F']=5,['G']=6,['H']=7,
    ['I']=8,['J']=9,['K']=10,['L']=11,['M']=12,['N']=13,['O']=14,['P']=15,
    ['Q']=16,['R']=17,['S']=18,['T']=19,['U']=20,['V']=21,['W']=22,['X']=23,
    ['Y']=24,['Z']=25,['a']=26,['b']=27,['c']=28,['d']=29,['e']=30,['f']=31,
    ['g']=32,['h']=33,['i']=34,['j']=35,['k']=36,['l']=37,['m']=38,['n']=39,
    ['o']=40,['p']=41,['q']=42,['r']=43,['s']=44,['t']=45,['u']=46,['v']=47,
    ['w']=48,['x']=49,['y']=50,['z']=51,['0']=52,['1']=53,['2']=54,['3']=55,
    ['4']=56,['5']=57,['6']=58,['7']=59,['+']=60,['/']=61
};

static int base64_decode(const char *in, char *out, int outlen)
{
    int i = 0, o = 0;
    int len = (int)strlen(in);
    while (i < len && o < outlen - 1) {
        if (i + 3 >= len) break;
        unsigned a = b64_table[(unsigned char)in[i++]];
        unsigned b = b64_table[(unsigned char)in[i++]];
        unsigned c = b64_table[(unsigned char)in[i++]];
        unsigned d = b64_table[(unsigned char)in[i++]];
        unsigned triple = (a << 18) | (b << 12) | (c << 6) | d;
        if (o < outlen - 1) out[o++] = (char)((triple >> 16) & 0xFF);
        if (o < outlen - 1 && in[i-2] != '=') out[o++] = (char)((triple >> 8) & 0xFF);
        if (o < outlen - 1 && in[i-1] != '=') out[o++] = (char)(triple & 0xFF);
    }
    out[o] = '\0';
    return o;
}

// ============================= HTTP Helpers ===============================

static void http_send(int fd, int code, const char *content_type, const char *body, int body_len)
{
    char header[512];
    const char *status_text = (code == 200) ? "OK" :
                              (code == 401) ? "Unauthorized" :
                              (code == 404) ? "Not Found" :
                              (code == 500) ? "Internal Server Error" : "OK";

    int hlen = snprintf(header, sizeof(header),
        "HTTP/1.0 %d %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %d\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "%s"
        "Connection: close\r\n"
        "\r\n",
        code, status_text, content_type, body_len,
        (code == 401) ? "WWW-Authenticate: Basic realm=\"dump1090-gg\"\r\n" : "");

    ssize_t r1 __attribute__((unused)) = write(fd, header, (size_t)hlen);
    if (body_len > 0) {
        ssize_t r2 __attribute__((unused)) = write(fd, body, (size_t)body_len);
    }
}

static void http_send_json(int fd, const char *json, int len)
{
    http_send(fd, 200, "application/json; charset=utf-8", json, len);
}

// ============================= Auth Check ================================

// Constant-time comparison to prevent timing attacks
static bool timing_safe_equal(const char *a, const char *b, size_t len)
{
    volatile unsigned char result = 0;
    for (size_t i = 0; i < len; i++)
        result |= (unsigned char)a[i] ^ (unsigned char)b[i];
    return result == 0;
}

static bool check_auth(const char *request)
{
    if (PanelState.password[0] == '\0')
        return true;  // no password set = no auth required

    const char *auth = strstr(request, "Authorization: Basic ");
    if (!auth) return false;

    auth += 21;
    char encoded[256] = {0};
    int i = 0;
    while (auth[i] && auth[i] != '\r' && auth[i] != '\n' && i < 255)
        encoded[i] = auth[i], i++;
    encoded[i] = '\0';

    char decoded[256];
    int dec_len = base64_decode(encoded, decoded, sizeof(decoded));

    // Expected: "admin:<password>"
    char expected[320];
    int exp_len = snprintf(expected, sizeof(expected), "admin:%s", PanelState.password);
    if (dec_len != exp_len) return false;
    return timing_safe_equal(decoded, expected, (size_t)exp_len);
}

// ============================= JSON Escape ================================

static char *json_escape(char *out, int outlen, const char *in)
{
    int o = 0;
    for (int i = 0; in[i] && o < outlen - 8; i++) {
        unsigned char c = (unsigned char)in[i];
        switch (c) {
            case '"':  out[o++] = '\\'; out[o++] = '"'; break;
            case '\\': out[o++] = '\\'; out[o++] = '\\'; break;
            case '\n': out[o++] = '\\'; out[o++] = 'n'; break;
            case '\r': out[o++] = '\\'; out[o++] = 'r'; break;
            case '\t': out[o++] = '\\'; out[o++] = 't'; break;
            default:
                if (c < 0x20) {
                    // Escape all control characters as \u00XX
                    o += snprintf(out + o, (size_t)(outlen - o), "\\u%04x", c);
                } else {
                    out[o++] = (char)c;
                }
                break;
        }
    }
    out[o] = '\0';
    return out;
}

// ============================= Mini JSON value helpers ====================

// Find a key in JSON and return pointer to its value (after the colon)
static const char *json_find_key(const char *json, const char *key)
{
    char needle[128];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = json;
    while ((p = strstr(p, needle)) != NULL) {
        p += strlen(needle);
        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
        if (*p == ':') return p + 1;
        // not a key:value pair, keep searching
    }
    return NULL;
}

static double json_get_double(const char *json, const char *key, double def)
{
    const char *v = json_find_key(json, key);
    if (!v) return def;
    while (*v == ' ') v++;
    return atof(v);
}

static int json_get_int(const char *json, const char *key, int def)
{
    const char *v = json_find_key(json, key);
    if (!v) return def;
    while (*v == ' ') v++;
    return atoi(v);
}

static bool json_get_bool(const char *json, const char *key, bool def)
{
    const char *v = json_find_key(json, key);
    if (!v) return def;
    while (*v == ' ') v++;
    if (strncmp(v, "true", 4) == 0) return true;
    if (strncmp(v, "false", 5) == 0) return false;
    return def;
}

// Find a nested JSON object value: e.g. json_find_obj(json, "station") returns the "{...}" block
static const char *json_find_obj(const char *json, const char *key)
{
    const char *v = json_find_key(json, key);
    if (!v) return NULL;
    while (*v == ' ' || *v == '\t' || *v == '\n' || *v == '\r') v++;
    if (*v == '{') return v;
    return NULL;
}

// ============================= Apply config at runtime ====================

static void panelApplyConfig(const char *body)
{
    // Station
    const char *station = json_find_obj(body, "station");
    if (station) {
        double lat = json_get_double(station, "lat", Modes.fUserLat);
        double lon = json_get_double(station, "lon", Modes.fUserLon);
        double mr = json_get_double(station, "max_range", Modes.maxRange / 1852.0);
        Modes.fUserLat = lat;
        Modes.fUserLon = lon;
        Modes.maxRange = mr * 1852.0;

        // Station name -> MLAT user
        const char *v = json_find_key(station, "name");
        if (v) {
            while (*v == ' ') v++;
            if (*v == '"') {
                v++;
                const char *e = strchr(v, '"');
                if (e && (e - v) > 0 && (e - v) < 64) {
                    free(MlatConfig.user);
                    int nlen = (int)(e - v);
                    MlatConfig.user = malloc(nlen + 1);
                    if (MlatConfig.user) { memcpy(MlatConfig.user, v, nlen); MlatConfig.user[nlen] = '\0'; }
                    panelLog("Panel: station name set to '%s'", MlatConfig.user);
                    // Sync OGN station name from station.name
                    strncpy(FlarmConfig.ogn_station, MlatConfig.user, sizeof(FlarmConfig.ogn_station) - 1);
                    FlarmConfig.ogn_station[sizeof(FlarmConfig.ogn_station) - 1] = '\0';
                }
            }
        }
    }

    // SDR ADS-B (runtime-safe fields only)
    const char *adsb = json_find_obj(body, "sdr_adsb");
    if (adsb) {
        Modes.adaptive_range_control = json_get_bool(adsb, "adaptive_range", Modes.adaptive_range_control);
        Modes.adaptive_burst_control = json_get_bool(adsb, "adaptive_burst", Modes.adaptive_burst_control);
        Modes.adaptive_min_gain_db = (float)json_get_double(adsb, "adaptive_min_gain", Modes.adaptive_min_gain_db);
        Modes.adaptive_max_gain_db = (float)json_get_double(adsb, "adaptive_max_gain", Modes.adaptive_max_gain_db);
        Modes.crc_rescue = json_get_bool(adsb, "crc_rescue", Modes.crc_rescue != 0) ? 1 : 0;
        Modes.nfix_crc = json_get_int(adsb, "fix_crc", Modes.nfix_crc);
        if (Modes.nfix_crc < 0) Modes.nfix_crc = 0;
        if (Modes.nfix_crc > 2) Modes.nfix_crc = 2;
        Modes.mode_ac = json_get_bool(adsb, "mode_ac", Modes.mode_ac);
        Modes.mode_ac_auto = json_get_bool(adsb, "mode_ac_auto", Modes.mode_ac_auto);
    }

    // SDR FLARM settings (device assignment now managed via SdrManager/Devices page)
    const char *flarm = json_find_obj(body, "sdr_flarm");
    if (flarm) {
        FlarmConfig.flarm_ogn_only = json_get_bool(flarm, "ogn_only", FlarmConfig.flarm_ogn_only != 0) ? 1 : 0;
    }

    // OGN settings
    const char *ogn = json_find_obj(body, "ogn");
    if (ogn) {
        // Station name is always taken from station.name (synced above)
        // Extract server
        const char *v = json_find_key(ogn, "server");
        if (v) {
            while (*v == ' ') v++;
            if (*v == '"') {
                v++;
                const char *e = strchr(v, '"');
                if (e && (e - v) < (int)sizeof(FlarmConfig.ogn_server) - 1) {
                    memcpy(FlarmConfig.ogn_server, v, e - v);
                    FlarmConfig.ogn_server[e - v] = '\0';
                }
            }
        }
        FlarmConfig.ogn_port = json_get_int(ogn, "port", FlarmConfig.ogn_port);
    }

    // Beast feeds enabled/disabled
    for (int i = 0; i < Modes.beast_feed_count; i++) {
        char needle[64];
        snprintf(needle, sizeof(needle), "\"name\":\"%s\"", Modes.beast_feeds[i].name);
        const char *pos = strstr(body, needle);
        if (!pos) continue;
        const char *en_false = strstr(pos, "\"enabled\":false");
        const char *en_true = strstr(pos, "\"enabled\":true");
        const char *next_obj = strchr(pos + 1, '}');
        if (en_false && (!next_obj || en_false < next_obj)) {
            Modes.beast_feeds[i].enabled = 0;
        } else if (en_true && (!next_obj || en_true < next_obj)) {
            Modes.beast_feeds[i].enabled = 1;
        }
    }

    // OpenSky enabled
    const char *osky = json_find_obj(body, "opensky");
    if (osky) {
        OpenSkyConfig.enabled = json_get_bool(osky, "enabled", OpenSkyConfig.enabled != 0) ? 1 : 0;
    }

    // PiAware enabled
    const char *piaw = json_find_obj(body, "piaware");
    if (piaw) {
        PiawareClient.enabled = json_get_bool(piaw, "enabled", PiawareClient.enabled != 0) ? 1 : 0;
    }

    // ADSBHub ckey
    const char *ahub = json_find_obj(body, "adsbhub");
    if (ahub) {
        const char *v = json_find_key(ahub, "ckey");
        if (v) {
            while (*v == ' ') v++;
            if (*v == '"') {
                v++;
                const char *e = strchr(v, '"');
                if (e && (e - v) > 0 && (e - v) < 256) {
                    free(Modes.adsbhub_ckey);
                    int clen = (int)(e - v);
                    Modes.adsbhub_ckey = malloc(clen + 1);
                    if (Modes.adsbhub_ckey) { memcpy(Modes.adsbhub_ckey, v, clen); Modes.adsbhub_ckey[clen] = '\0'; }
                    panelLog("Panel: ADSBHub ckey updated");
                } else if (e && e == v) {
                    free(Modes.adsbhub_ckey);
                    Modes.adsbhub_ckey = NULL;
                }
            }
        }
    }

    // SondeHub
    const char *shub = json_find_obj(body, "sondehub");
    if (shub) {
        SondehubConfig.enabled = json_get_bool(shub, "enabled", SondehubConfig.enabled);
    }
    // Callsign always synced from station.name
    if (MlatConfig.user && MlatConfig.user[0]) {
        strncpy(SondehubConfig.callsign, MlatConfig.user, sizeof(SondehubConfig.callsign) - 1);
        SondehubConfig.callsign[sizeof(SondehubConfig.callsign) - 1] = '\0';
    }
    if (shub) {
        panelLog("Panel: SondeHub %s (callsign: %s)",
                 SondehubConfig.enabled ? "enabled" : "disabled",
                 SondehubConfig.callsign);
    }

    // radiosondy.info
    const char *rsondy = json_find_obj(body, "radiosondy");
    if (rsondy) {
        RadiosondyEnabled = json_get_bool(rsondy, "enabled", RadiosondyEnabled);
    }

    // wettersonde.net
    const char *wetter = json_find_obj(body, "wettersonde");
    if (wetter) {
        WettersondeEnabled = json_get_bool(wetter, "enabled", WettersondeEnabled);
    }

    panelLog("Panel: configuration applied at runtime (no restart)");
}

// ============================= Beast feed helpers =========================

// Add or find a beast feed by name (mirrors addBeastFeed in dump1090.c)
static int panelAddBeastFeed(const char *name, const char *host, int port, int format)
{
    // Check if already exists
    for (int i = 0; i < Modes.beast_feed_count; i++) {
        if (!strcmp(Modes.beast_feeds[i].name, name))
            return i;
    }
    if (Modes.beast_feed_count >= MAX_BEAST_FEEDS) return -1;
    int idx = Modes.beast_feed_count++;
    snprintf(Modes.beast_feeds[idx].name, sizeof(Modes.beast_feeds[idx].name), "%s", name);
    Modes.beast_feeds[idx].host = strdup(host);
    Modes.beast_feeds[idx].port = port;
    Modes.beast_feeds[idx].format = format;
    Modes.beast_feeds[idx].enabled = 0;  // disabled by default until user enables
    return idx;
}

// Pre-configured beast feeds for well-known aggregators
void panelEnsureDefaultBeastFeeds(void)
{
    static const struct { const char *name; const char *host; int port; int format; } defaults[] = {
        { "ADSBx",          "feed.adsbexchange.com",   30005, FEED_FORMAT_BEAST },
        { "adsb.fi",        "feed.adsb.fi",            30004, FEED_FORMAT_BEAST },
        { "FlyItaly",       "dati.flyitalyadsb.com",   4905,  FEED_FORMAT_BEAST },
        { "PlaneWatch",     "atc.plane.watch",         30004, FEED_FORMAT_BEAST },
        { "adsb.one",       "feed.adsb.one",           64004, FEED_FORMAT_BEAST },
        { "adsb.lol",       "feed.adsb.lol",           30004, FEED_FORMAT_BEAST },
        { "airplanes.live", "feed.airplanes.live",     30004, FEED_FORMAT_BEAST },
        { "Planespotters",  "feed.planespotters.net",  30004, FEED_FORMAT_BEAST },
        { "TheAirTraffic",  "feed.theairtraffic.com",  30004, FEED_FORMAT_BEAST },
        { "AVDelphi",       "data.avdelphi.com",       24999, FEED_FORMAT_BEAST },
        { "ADSBHub",        "data.adsbhub.org",        5001,  FEED_FORMAT_RAW   },
    };
    for (int i = 0; i < (int)(sizeof(defaults)/sizeof(defaults[0])); i++) {
        panelAddBeastFeed(defaults[i].name, defaults[i].host, defaults[i].port, defaults[i].format);
    }
}

// ============================= Load saved beast feed state ================

void panelLoadBeastFeedState(void)
{
    FILE *f = fopen(PANEL_CONF_PATH, "r");
    if (!f) return;

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 131072) { fclose(f); return; }

    char *data = malloc((size_t)sz + 1);
    if (!data) { fclose(f); return; }
    size_t rd = fread(data, 1, (size_t)sz, f);
    fclose(f);
    data[rd] = '\0';

    // For each beast feed, look for its name in saved config and restore enabled state
    for (int i = 0; i < Modes.beast_feed_count; i++) {
        // Search for "name":"<feedname>" in the JSON
        char needle[64];
        snprintf(needle, sizeof(needle), "\"name\":\"%s\"", Modes.beast_feeds[i].name);
        const char *pos = strstr(data, needle);
        if (!pos) continue;

        // Use the closing brace of this JSON object as the boundary
        const char *next_obj = strchr(pos + 1, '}');
        const char *en_true = strstr(pos, "\"enabled\":true");
        const char *en_false = strstr(pos, "\"enabled\":false");
        if (en_false && (!next_obj || en_false < next_obj)) {
            Modes.beast_feeds[i].enabled = 0;
            fprintf(stderr, "Panel: %s disabled by saved config\n", Modes.beast_feeds[i].name);
        } else if (en_true && (!next_obj || en_true < next_obj)) {
            Modes.beast_feeds[i].enabled = 1;
            fprintf(stderr, "Panel: %s enabled by saved config\n", Modes.beast_feeds[i].name);
        }
    }

    // Restore FLARM settings from saved config (device assignment handled by receivers.json)
    const char *flarm = json_find_obj(data, "sdr_flarm");
    if (flarm) {
        FlarmConfig.flarm_ogn_only = json_get_bool(flarm, "ogn_only", FlarmConfig.flarm_ogn_only != 0) ? 1 : 0;
    }

    // Restore station name and sync to OGN station
    const char *stationobj = json_find_obj(data, "station");
    if (stationobj) {
        const char *v = json_find_key(stationobj, "name");
        if (v) {
            while (*v == ' ') v++;
            if (*v == '"') {
                v++;
                const char *e = strchr(v, '"');
                if (e && (e - v) > 0 && (e - v) < 64) {
                    free(MlatConfig.user);
                    int nlen = (int)(e - v);
                    MlatConfig.user = malloc(nlen + 1);
                    if (MlatConfig.user) { memcpy(MlatConfig.user, v, nlen); MlatConfig.user[nlen] = '\0'; }
                    strncpy(FlarmConfig.ogn_station, MlatConfig.user, sizeof(FlarmConfig.ogn_station) - 1);
                    FlarmConfig.ogn_station[sizeof(FlarmConfig.ogn_station) - 1] = '\0';
                }
            }
        }
        double lat = json_get_double(stationobj, "lat", Modes.fUserLat);
        double lon = json_get_double(stationobj, "lon", Modes.fUserLon);
        double mr  = json_get_double(stationobj, "max_range", Modes.maxRange / 1852.0);
        if (lat != 0.0 || lon != 0.0) { Modes.fUserLat = lat; Modes.fUserLon = lon; }
        if (mr > 0) Modes.maxRange = mr * 1852.0;
    }

    // Restore OGN settings from saved config
    const char *ogn = json_find_obj(data, "ogn");
    if (ogn) {
        // ogn.station is always derived from station.name (no separate restore)
        const char *v = json_find_key(ogn, "server");
        if (v) {
            while (*v == ' ') v++;
            if (*v == '"') {
                v++;
                const char *e = strchr(v, '"');
                if (e && (e - v) > 0 && (e - v) < (int)sizeof(FlarmConfig.ogn_server) - 1) {
                    memcpy(FlarmConfig.ogn_server, v, e - v);
                    FlarmConfig.ogn_server[e - v] = '\0';
                }
            }
        }
        int port = json_get_int(ogn, "port", 0);
        if (port > 0 && port < 65536) FlarmConfig.ogn_port = port;
    }

    // Restore SondeHub settings
    const char *shub = json_find_obj(data, "sondehub");
    if (shub) {
        SondehubConfig.enabled = json_get_bool(shub, "enabled", SondehubConfig.enabled);
    }
    // Callsign always synced from station.name
    if (MlatConfig.user && MlatConfig.user[0]) {
        strncpy(SondehubConfig.callsign, MlatConfig.user, sizeof(SondehubConfig.callsign) - 1);
        SondehubConfig.callsign[sizeof(SondehubConfig.callsign) - 1] = '\0';
    }
    if (shub) {
        fprintf(stderr, "Panel: SondeHub %s (callsign: %s)\n",
                SondehubConfig.enabled ? "enabled" : "disabled",
                SondehubConfig.callsign);
    }

    // Restore radiosondy / wettersonde enabled state
    const char *rsondy = json_find_obj(data, "radiosondy");
    if (rsondy) RadiosondyEnabled = json_get_bool(rsondy, "enabled", false);
    const char *wetter = json_find_obj(data, "wettersonde");
    if (wetter) WettersondeEnabled = json_get_bool(wetter, "enabled", false);

    free(data);
}

// ============================= API: GET /api/config ======================

static void api_get_config(int fd)
{
    // 64KB buffer for config JSON
    char *buf = malloc(65536);
    if (!buf) { http_send(fd, 500, "text/plain", "OOM", 3); return; }

    char *p = buf;
    char *end = buf + 65536;
    char esc[256];

    p += snprintf(p, (size_t)(end - p), "{\n");

    // Station
    p += snprintf(p, (size_t)(end - p),
        "\"station\":{\"lat\":%.6f,\"lon\":%.6f,\"max_range\":%.0f,\"name\":\"%s\"},\n",
        Modes.fUserLat, Modes.fUserLon, Modes.maxRange / 1852.0,
        MlatConfig.user ? json_escape(esc, sizeof(esc), MlatConfig.user) : "");

    // SDR ADS-B
    p += snprintf(p, (size_t)(end - p),
        "\"sdr_adsb\":{\"device\":\"%s\",\"gain\":%.1f,"
        "\"adaptive_range\":%s,\"adaptive_burst\":%s,"
        "\"adaptive_min_gain\":%.1f,\"adaptive_max_gain\":%.1f,"
        "\"crc_rescue\":%s,\"fix_crc\":%d,\"mode_ac\":%s,\"mode_ac_auto\":%s},\n",
        Modes.dev_name ? json_escape(esc, sizeof(esc), Modes.dev_name) : "",
        Modes.gain == 999999 ? -10.0f : Modes.gain,  // -10 = auto
        Modes.adaptive_range_control ? "true" : "false",
        Modes.adaptive_burst_control ? "true" : "false",
        Modes.adaptive_min_gain_db, Modes.adaptive_max_gain_db,
        Modes.crc_rescue ? "true" : "false",
        Modes.nfix_crc,
        Modes.mode_ac ? "true" : "false",
        Modes.mode_ac_auto ? "true" : "false");

    // SDR FLARM — derive enabled from SdrManager
    {
        char esc2[256];
        int flarm_active = 0;
        char flarm_serial[64] = {0};
        float flarm_gain = FlarmConfig.gain / 10.0f;
        int flarm_ppm = FlarmConfig.ppm_error;
        for (int i = 0; i < SdrManager.count; i++) {
            if (SdrManager.receivers[i].config.role == SDR_ROLE_FLARM) {
                flarm_active = 1;
                snprintf(flarm_serial, sizeof(flarm_serial), "%.63s", SdrManager.receivers[i].config.serial);
                flarm_gain = SdrManager.receivers[i].config.gain;
                flarm_ppm = SdrManager.receivers[i].config.ppm_error;
                break;
            }
        }
        if (!flarm_serial[0])
            snprintf(flarm_serial, sizeof(flarm_serial), "%.63s", FlarmConfig.device_serial);
        p += snprintf(p, (size_t)(end - p),
            "\"sdr_flarm\":{\"enabled\":%s,\"device\":\"%s\","
            "\"gain\":%.1f,\"ppm\":%d,\"keys_file\":\"%s\","
            "\"ogn_only\":%s},\n",
            flarm_active ? "true" : "false",
            json_escape(esc, sizeof(esc), flarm_serial),
            flarm_gain, flarm_ppm,
            json_escape(esc2, sizeof(esc2), FlarmConfig.keys_file),
            FlarmConfig.flarm_ogn_only ? "true" : "false");
    }

    // Beast feeds
    p += snprintf(p, (size_t)(end - p), "\"beast_feeds\":[\n");
    for (int i = 0; i < Modes.beast_feed_count; i++) {
        p += snprintf(p, (size_t)(end - p),
            "%s{\"name\":\"%s\",\"host\":\"%s\",\"port\":%d,\"format\":\"%s\",\"enabled\":%s}",
            i ? "," : "",
            json_escape(esc, sizeof(esc), Modes.beast_feeds[i].name),
            Modes.beast_feeds[i].host ? Modes.beast_feeds[i].host : "",
            Modes.beast_feeds[i].port,
            Modes.beast_feeds[i].format == FEED_FORMAT_RAW ? "raw" : Modes.beast_feeds[i].format == FEED_FORMAT_SBS ? "sbs" : "beast",
            Modes.beast_feeds[i].enabled ? "true" : "false");
    }
    p += snprintf(p, (size_t)(end - p), "],\n");

    // FR24, PlaneFinder, RadarBox feeders removed in light version

    // OpenSky
    p += snprintf(p, (size_t)(end - p),
        "\"opensky\":{\"enabled\":%s,\"username\":\"%s\","
        "\"serial\":%d,\"host\":\"%s\",\"port\":%d},\n",
        OpenSkyConfig.enabled ? "true" : "false",
        json_escape(esc, sizeof(esc), OpenSkyConfig.username),
        OpenSkyConfig.serial,
        json_escape(esc, sizeof(esc), OpenSkyConfig.host),
        OpenSkyConfig.port);

    // PiAware
    p += snprintf(p, (size_t)(end - p),
        "\"piaware\":{\"enabled\":%s,\"feeder_id\":\"%s\","
        "\"state\":%d,\"msgs_sent\":%llu},\n",
        PiawareClient.enabled ? "true" : "false",
        json_escape(esc, sizeof(esc), PiawareClient.feeder_id),
        PiawareClient.state,
        (unsigned long long)PiawareClient.msgs_sent);

    // Radiosonde SDR status (derived from SdrManager)
    {
        int sonde_active = 0;
        for (int i = 0; i < SdrManager.count; i++) {
            if (SdrManager.receivers[i].config.role == SDR_ROLE_RADIOSONDE) {
                sonde_active = 1;
                break;
            }
        }
        p += snprintf(p, (size_t)(end - p),
            "\"sdr_radiosonde\":{\"enabled\":%s},\n",
            sonde_active ? "true" : "false");
    }

    // Sonde feeds
    p += snprintf(p, (size_t)(end - p),
        "\"sondehub\":{\"enabled\":%s,\"callsign\":\"%s\"},\n",
        SondehubConfig.enabled ? "true" : "false",
        json_escape(esc, sizeof(esc), SondehubConfig.callsign));
    p += snprintf(p, (size_t)(end - p),
        "\"radiosondy\":{\"enabled\":%s},\n",
        RadiosondyEnabled ? "true" : "false");
    p += snprintf(p, (size_t)(end - p),
        "\"wettersonde\":{\"enabled\":%s},\n",
        WettersondeEnabled ? "true" : "false");

    // MLAT
    p += snprintf(p, (size_t)(end - p), "\"mlat\":{\"servers\":[\n");
    for (int i = 0; i < MlatConfig.server_count; i++) {
        p += snprintf(p, (size_t)(end - p),
            "%s{\"host\":\"%s\",\"port\":%d,\"state\":%d}",
            i ? "," : "",
            MlatConfig.servers[i].host ? json_escape(esc, sizeof(esc), MlatConfig.servers[i].host) : "",
            MlatConfig.servers[i].port,
            (int)MlatConfig.servers[i].state);
    }
    p += snprintf(p, (size_t)(end - p),
        "],\"alt\":%.0f,\"return_results\":%s},\n",
        MlatConfig.alt,
        MlatConfig.return_results ? "true" : "false");

    // OGN
    p += snprintf(p, (size_t)(end - p),
        "\"ogn\":{\"server\":\"%s\",\"port\":%d},\n",
        json_escape(esc, sizeof(esc), FlarmConfig.ogn_server),
        FlarmConfig.ogn_port);

    // ADSBHub
    p += snprintf(p, (size_t)(end - p),
        "\"adsbhub\":{\"ckey\":\"%s\"},\n",
        Modes.adsbhub_ckey ? json_escape(esc, sizeof(esc), Modes.adsbhub_ckey) : "");

    // Keys — actual values from loaded key files
    {
        char kt_str[256] = "";
        char k5_str[64] = "";
        char rb_key_str[48] = "";
        char rb_nonce_str[24] = "";
        uint32_t rb_c2_val = 0;

        if (flarm_keys_are_loaded()) {
            uint32_t kt[12], k5v[4];
            flarm_get_key_table(kt);
            flarm_get_key5(k5v);
            snprintf(kt_str, sizeof(kt_str),
                "%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x",
                kt[0],kt[1],kt[2],kt[3],kt[4],kt[5],kt[6],kt[7],kt[8],kt[9],kt[10],kt[11]);
            snprintf(k5_str, sizeof(k5_str), "%08x,%08x,%08x,%08x", k5v[0],k5v[1],k5v[2],k5v[3]);
        }

        // RadarBox keys removed in light version

        p += snprintf(p, (size_t)(end - p),
            "\"keys\":{\"flarm_key_table\":\"%s\","
            "\"flarm_key2\":\"%08x\","
            "\"flarm_key3\":\"%08x\","
            "\"flarm_key4\":\"%08x\","
            "\"flarm_key5\":\"%s\","
            "\"rb_key\":\"%s\","
            "\"rb_nonce\":\"%s\","
            "\"rb_c2\":\"%08x\"},\n",
            kt_str,
            flarm_keys_are_loaded() ? flarm_get_key2() : 0,
            flarm_keys_are_loaded() ? flarm_get_key3() : 0,
            flarm_keys_are_loaded() ? flarm_get_key4() : 0,
            k5_str,
            rb_key_str,
            rb_nonce_str,
            rb_c2_val);
    }

    // Network ports
    p += snprintf(p, (size_t)(end - p),
        "\"network\":{\"raw_out\":\"%s\",\"raw_in\":\"%s\","
        "\"sbs_out\":\"%s\",\"beast_in\":\"%s\",\"beast_out\":\"%s\","
        "\"json_dir\":\"%s\",\"json_interval\":%llu},\n",
        Modes.net_output_raw_ports ? Modes.net_output_raw_ports : "",
        Modes.net_input_raw_ports ? Modes.net_input_raw_ports : "",
        Modes.net_output_sbs_ports ? Modes.net_output_sbs_ports : "",
        Modes.net_input_beast_ports ? Modes.net_input_beast_ports : "",
        Modes.net_output_beast_ports ? Modes.net_output_beast_ports : "",
        Modes.json_dir ? json_escape(esc, sizeof(esc), Modes.json_dir) : "",
        (unsigned long long)Modes.json_interval);

    // Panel
    p += snprintf(p, (size_t)(end - p),
        "\"panel\":{\"port\":%d,\"has_password\":%s}\n",
        PanelState.port,
        PanelState.password[0] ? "true" : "false");

    p += snprintf(p, (size_t)(end - p), "}\n");

    http_send_json(fd, buf, (int)(p - buf));
    free(buf);
}

// ============================= API: GET /api/status ======================

static void api_get_status(int fd)
{
    char *buf = malloc(16384);
    if (!buf) { http_send(fd, 500, "text/plain", "OOM", 3); return; }

    char *p = buf;
    char *end = buf + 16384;

    p += snprintf(p, (size_t)(end - p), "{\"feeders\":[\n");

    // Beast feeds
    for (int i = 0; i < Modes.beast_feed_count; i++) {
        p += snprintf(p, (size_t)(end - p),
            "%s{\"name\":\"%s\",\"type\":\"beast\",\"enabled\":%s,"
            "\"host\":\"%s\",\"port\":%d}",
            (i > 0 || 0) ? "," : "",
            Modes.beast_feeds[i].name,
            Modes.beast_feeds[i].enabled ? "true" : "false",
            Modes.beast_feeds[i].host ? Modes.beast_feeds[i].host : "",
            Modes.beast_feeds[i].port);
    }

    int need_comma = (Modes.beast_feed_count > 0);

    // FR24, PlaneFinder, RadarBox feeders removed in light version

    // OpenSky — always show with enabled/disabled state
    {
        p += snprintf(p, (size_t)(end - p),
            "%s{\"name\":\"OpenSky Network\",\"type\":\"native\",\"enabled\":%s,"
            "\"host\":\"%s\",\"port\":%d,\"user\":\"%s\","
            "\"link\":\"https://opensky-network.org/my-opensky/sensors/view-sensors\"}",
            need_comma ? "," : "",
            OpenSkyConfig.enabled ? "true" : "false",
            OpenSkyConfig.host, OpenSkyConfig.port, OpenSkyConfig.username);
        need_comma = 1;
    }

    // PiAware — always show with enabled/disabled state
    {
        const char *pa_states[] = {"disconnected","connecting","tls_handshake","awaiting_login","logged_in"};
        int si = PiawareClient.state;
        if (si < 0 || si > 4) si = 0;
        p += snprintf(p, (size_t)(end - p),
            "%s{\"name\":\"PiAware / FlightAware\",\"type\":\"native\",\"enabled\":%s,"
            "\"state\":\"%s\",\"feeder_id\":\"%s\",\"msgs\":%llu,"
            "\"link\":\"https://flightaware.com/adsb/stats\"}",
            need_comma ? "," : "",
            PiawareClient.enabled ? "true" : "false",
            pa_states[si], PiawareClient.feeder_id,
            (unsigned long long)PiawareClient.msgs_sent);
        need_comma = 1;
    }

    // MLAT servers
    for (int i = 0; i < MlatConfig.server_count; i++) {
        const char *ml_states[] = {"disconnected","connecting","handshaking","ready"};
        int mi = (int)MlatConfig.servers[i].state;
        if (mi < 0 || mi > 3) mi = 0;
        p += snprintf(p, (size_t)(end - p),
            "%s{\"name\":\"MLAT:%s\",\"type\":\"mlat\",\"enabled\":true,"
            "\"host\":\"%s\",\"port\":%d,\"state\":\"%s\"}",
            need_comma ? "," : "",
            MlatConfig.servers[i].host ? MlatConfig.servers[i].host : "?",
            MlatConfig.servers[i].host ? MlatConfig.servers[i].host : "",
            MlatConfig.servers[i].port,
            ml_states[mi]);
        need_comma = 1;
    }

    // FLARM/OGN — derive state from SdrManager
    {
        int flarm_running = 0;
        for (int i = 0; i < SdrManager.count; i++) {
            if (SdrManager.receivers[i].config.role == SDR_ROLE_FLARM &&
                SdrManager.receivers[i].state == RX_STATE_RUNNING) {
                flarm_running = 1;
                break;
            }
        }
        p += snprintf(p, (size_t)(end - p),
            "%s{\"name\":\"FLARM / OGNTP\",\"type\":\"flarm\",\"enabled\":%s,"
            "\"station\":\"%s\",\"server\":\"%s\","
            "\"link\":\"http://live.glidernet.org\"}",
            need_comma ? "," : "",
            flarm_running ? "true" : "false",
            FlarmConfig.ogn_station, FlarmConfig.ogn_server);
    }

    p += snprintf(p, (size_t)(end - p),
        "],\"version\":\"" MODES_DUMP1090_VERSION "\",\"variant\":\"" MODES_DUMP1090_VARIANT "\"}\n");

    http_send_json(fd, buf, (int)(p - buf));
    free(buf);
}

// ============================= API: GET /api/aircraft =====================

static void api_get_aircraft(int fd)
{
    // Read aircraft.json from disk (already generated by dump1090)
    if (!Modes.json_dir) {
        http_send(fd, 404, "text/plain", "No JSON dir", 11);
        return;
    }

    char path[512];
    snprintf(path, sizeof(path), "%s/aircraft.json", Modes.json_dir);

    FILE *f = fopen(path, "r");
    if (!f) {
        http_send(fd, 404, "text/plain", "No aircraft data", 16);
        return;
    }

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (fsize <= 0 || fsize > 1048576) {
        fclose(f);
        http_send(fd, 500, "text/plain", "Bad file", 8);
        return;
    }

    char *data = malloc((size_t)fsize + 1);
    if (!data) { fclose(f); http_send(fd, 500, "text/plain", "OOM", 3); return; }

    size_t nread = fread(data, 1, (size_t)fsize, f);
    fclose(f);
    data[nread] = '\0';

    http_send_json(fd, data, (int)nread);
    free(data);
}

// ============================= API: GET /api/connections ==================

static void api_get_connections(int fd)
{
    char *buf = malloc(32768);
    if (!buf) { http_send(fd, 500, "text/plain", "OOM", 3); return; }

    char *p = buf;
    char *end = buf + 32768;

    p += snprintf(p, (size_t)(end - p), "{\"services\":[\n");

    int first_svc = 1;
    struct net_service *svc;
    for (svc = Modes.services; svc; svc = svc->next) {
        if (!first_svc) p += snprintf(p, (size_t)(end - p), ",");
        first_svc = 0;

        char esc[256];
        p += snprintf(p, (size_t)(end - p),
            "{\"descr\":\"%s\",\"connections\":%d,\"listeners\":[",
            json_escape(esc, sizeof(esc), svc->descr ? svc->descr : "?"),
            svc->connections);

        // Emit listener ports
        for (int i = 0; i < svc->listener_count; i++) {
            struct sockaddr_in sa;
            socklen_t slen = sizeof(sa);
            if (getsockname(svc->listener_fds[i], (struct sockaddr *)&sa, &slen) == 0) {
                p += snprintf(p, (size_t)(end - p), "%s%d",
                    i ? "," : "", ntohs(sa.sin_port));
            }
        }
        p += snprintf(p, (size_t)(end - p), "],\"clients\":[");

        // Walk client list and find clients belonging to this service
        int first_cli = 1;
        struct client *c;
        for (c = Modes.clients; c; c = c->next) {
            if (c->service != svc) continue;
            if (p >= end - 256) break;  // safety margin

            struct sockaddr_in peer;
            socklen_t plen = sizeof(peer);
            char addr[64] = "?";
            if (getpeername(c->fd, (struct sockaddr *)&peer, &plen) == 0) {
                snprintf(addr, sizeof(addr), "%s:%d",
                    inet_ntoa(peer.sin_addr), ntohs(peer.sin_port));
            }

            // Build flags string
            char flags[64] = "";
            if (c->modeac_requested) strcat(flags, "AC ");
            if (c->verbatim_requested) strcat(flags, "V ");
            if (c->local_requested) strcat(flags, "L ");

            if (!first_cli) p += snprintf(p, (size_t)(end - p), ",");
            first_cli = 0;
            p += snprintf(p, (size_t)(end - p),
                "{\"addr\":\"%s\",\"fd\":%d,\"flags\":\"%s\"}",
                addr, c->fd, flags);
        }

        p += snprintf(p, (size_t)(end - p), "]}");
    }

    p += snprintf(p, (size_t)(end - p), "]}\n");

    http_send_json(fd, buf, (int)(p - buf));
    free(buf);
}

// ============================= API: GET /api/stats =======================

static void api_get_stats(int fd)
{
    if (!Modes.json_dir) {
        http_send(fd, 404, "text/plain", "No JSON dir", 11);
        return;
    }

    char path[512];
    snprintf(path, sizeof(path), "%s/stats.json", Modes.json_dir);

    FILE *f = fopen(path, "r");
    if (!f) {
        http_send(fd, 404, "text/plain", "No stats data", 13);
        return;
    }

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (fsize <= 0 || fsize > 1048576) {
        fclose(f);
        http_send(fd, 500, "text/plain", "Bad file", 8);
        return;
    }

    char *data = malloc((size_t)fsize + 1);
    if (!data) { fclose(f); http_send(fd, 500, "text/plain", "OOM", 3); return; }

    size_t nread = fread(data, 1, (size_t)fsize, f);
    fclose(f);

    http_send_json(fd, data, (int)nread);
    free(data);
}

// ============================= API: GET /api/logs ========================

static void api_get_logs(int fd)
{
    char *buf = malloc(65536);
    if (!buf) { http_send(fd, 500, "text/plain", "OOM", 3); return; }

    char *p = buf;
    char *end = buf + 65536;
    char esc[512];

    pthread_mutex_lock(&PanelState.log_mutex);

    p += snprintf(p, (size_t)(end - p), "{\"seq\":%d,\"lines\":[\n", PanelState.log_seq);

    // Return last N lines (max 200 per request)
    int count = PanelState.log_count;
    int start_offset = 0;
    if (count > 200) {
        start_offset = count - 200;
        count = 200;
    }

    for (int i = 0; i < count && p < end - 300; i++) {
        int idx = (PanelState.log_head + start_offset + i) % PANEL_LOG_LINES;
        p += snprintf(p, (size_t)(end - p), "%s\"%s\"",
                      i ? "," : "",
                      json_escape(esc, sizeof(esc), PanelState.log_buf[idx]));
    }

    pthread_mutex_unlock(&PanelState.log_mutex);

    p += snprintf(p, (size_t)(end - p), "]}\n");

    http_send_json(fd, buf, (int)(p - buf));
    free(buf);
}

// ============================= API: GET /api/messages =====================

static void api_get_messages(int fd)
{
    char *buf = malloc(65536);
    if (!buf) { http_send(fd, 500, "text/plain", "OOM", 3); return; }

    char *p = buf;
    char *end = buf + 65536;
    char esc[512];

    pthread_mutex_lock(&PanelState.msg_mutex);

    p += snprintf(p, (size_t)(end - p), "{\"seq\":%d,\"messages\":[\n", PanelState.msg_seq);

    int count = PanelState.msg_count;
    int start_offset = 0;
    if (count > 200) {
        start_offset = count - 200;
        count = 200;
    }

    for (int i = 0; i < count && p < end - 300; i++) {
        int idx = (PanelState.msg_head + start_offset + i) % PANEL_MSG_LINES;
        p += snprintf(p, (size_t)(end - p), "%s\"%s\"",
                      i ? "," : "",
                      json_escape(esc, sizeof(esc), PanelState.msg_buf[idx]));
    }

    pthread_mutex_unlock(&PanelState.msg_mutex);

    p += snprintf(p, (size_t)(end - p), "]}\n");

    http_send_json(fd, buf, (int)(p - buf));
    free(buf);
}

// ============================= Input Validation ==========================

// Validate that a string is plausible JSON (basic structural check)
// Rejects obviously malformed or dangerous payloads
static bool is_valid_json_object(const char *body, int maxlen)
{
    if (!body || maxlen <= 0) return false;
    int len = (int)strnlen(body, (size_t)maxlen);
    if (len < 2 || len >= maxlen) return false;

    // Must start with { and end with }
    // Skip leading whitespace
    int start = 0;
    while (start < len && (body[start] == ' ' || body[start] == '\t'
           || body[start] == '\r' || body[start] == '\n')) start++;
    int end = len - 1;
    while (end > start && (body[end] == ' ' || body[end] == '\t'
           || body[end] == '\r' || body[end] == '\n' || body[end] == '\0')) end--;

    if (body[start] != '{' || body[end] != '}') return false;

    // Check balanced braces and brackets
    int braces = 0, brackets = 0;
    bool in_string = false;
    for (int i = start; i <= end; i++) {
        if (body[i] == '\\' && in_string) { i++; continue; }
        if (body[i] == '"') { in_string = !in_string; continue; }
        if (in_string) continue;
        if (body[i] == '{') braces++;
        else if (body[i] == '}') braces--;
        else if (body[i] == '[') brackets++;
        else if (body[i] == ']') brackets--;
        if (braces < 0 || brackets < 0) return false;
    }
    return (braces == 0 && brackets == 0 && !in_string);
}

// Sanitize a JSON config body: reject dangerous characters in string values
// Exception: "ckey" values are allowed to contain special characters
static bool sanitize_json_config(const char *body)
{
    bool in_string = false;
    bool in_key = false;       // true when inside a JSON key (before ':')
    bool skip_value = false;   // true when current string value is whitelisted
    char last_key[32] = {0};
    int key_pos = 0;

    for (int i = 0; body[i]; i++) {
        if (body[i] == '\\' && in_string) { i++; continue; }  // skip escaped
        if (body[i] == '"') {
            if (!in_string) {
                // Starting a string — determine if it's a key or value
                in_string = true;
                // Look back: if preceded by '{' or ',' (skipping whitespace), it's a key
                int j = i - 1;
                while (j >= 0 && (body[j] == ' ' || body[j] == '\t' || body[j] == '\n' || body[j] == '\r')) j--;
                if (j < 0 || body[j] == '{' || body[j] == ',' || body[j] == '[') {
                    in_key = true;
                    key_pos = 0;
                    memset(last_key, 0, sizeof(last_key));
                } else {
                    in_key = false;
                    // Check if previous key was "ckey" — skip sanitization for its value
                    skip_value = (strcmp(last_key, "ckey") == 0);
                }
            } else {
                // Ending a string
                in_string = false;
                in_key = false;
                skip_value = false;
            }
            continue;
        }
        if (in_string) {
            if (in_key) {
                // Accumulate key name
                if (key_pos < (int)sizeof(last_key) - 1)
                    last_key[key_pos++] = body[i];
            } else if (!skip_value) {
                unsigned char c = (unsigned char)body[i];
                // Block dangerous characters inside JSON string values
                if (c == '`' || c == '$' || c == '!'
                    || c == '|' || c == ';' || c == '&'
                    || c == '<' || c == '>') {
                    return false;
                }
                // Block non-printable control characters (except tab, newline, CR)
                if (c < 0x20 && c != '\t' && c != '\n' && c != '\r') {
                    return false;
                }
            }
        }
    }
    return true;
}

// ============================= API: POST /api/config =====================

static void api_post_config(int fd, const char *body)
{
    // Validate: must be valid JSON object
    if (!is_valid_json_object(body, 65536)) {
        const char *err = "{\"error\":\"Invalid JSON format\"}";
        http_send(fd, 400, "application/json", err, (int)strlen(err));
        panelLog("Panel: rejected config POST — invalid JSON structure");
        return;
    }

    // Sanitize: reject shell metacharacters and HTML injection in string values
    if (!sanitize_json_config(body)) {
        const char *err = "{\"error\":\"Config contains forbidden characters\"}";
        http_send(fd, 400, "application/json", err, (int)strlen(err));
        panelLog("Panel: rejected config POST — dangerous characters in values");
        return;
    }

    // Size limit
    size_t body_len = strlen(body);
    if (body_len > 32768) {
        const char *err = "{\"error\":\"Config too large\"}";
        http_send(fd, 400, "application/json", err, (int)strlen(err));
        return;
    }

    // Write the validated JSON config to panel.conf
    FILE *f = fopen(PANEL_CONF_PATH, "w");
    if (!f) {
        const char *err = "{\"error\":\"Cannot write config file\"}";
        http_send_json(fd, err, (int)strlen(err));
        return;
    }
    fprintf(f, "%s", body);
    fclose(f);

    // Apply config at runtime without restart
    panelApplyConfig(body);

    const char *ok = "{\"status\":\"saved\",\"applied\":true}";
    http_send_json(fd, ok, (int)strlen(ok));

    panelLog("Panel: configuration saved to %s and applied live", PANEL_CONF_PATH);
}

// ============================= File Serving ===============================

static void serve_file(int fd, const char *filename)
{
    // Reject obviously dangerous filenames
    if (!filename || filename[0] == '\0' || filename[0] == '/') {
        http_send(fd, 404, "text/plain", "Not found", 9);
        return;
    }

    // Prevent path traversal: reject .., encoded dots, backslash
    if (strstr(filename, "..") || strstr(filename, "%2e") || strstr(filename, "%2E")
        || strchr(filename, '\\') || strstr(filename, "//")) {
        http_send(fd, 404, "text/plain", "Not found", 9);
        return;
    }

    // Only allow safe filename characters
    for (int i = 0; filename[i]; i++) {
        unsigned char c = (unsigned char)filename[i];
        if (!(c >= 'a' && c <= 'z') && !(c >= 'A' && c <= 'Z')
            && !(c >= '0' && c <= '9') && c != '.' && c != '-'
            && c != '_' && c != '/') {
            http_send(fd, 404, "text/plain", "Not found", 9);
            return;
        }
    }

    char path[512];
    snprintf(path, sizeof(path), "%s/%s", PanelState.html_dir, filename);

    // Resolve real path and verify it's under html_dir
    char resolved[PATH_MAX];
    char resolved_base[PATH_MAX];
    if (realpath(path, resolved) && realpath(PanelState.html_dir, resolved_base)) {
        if (strncmp(resolved, resolved_base, strlen(resolved_base)) != 0) {
            http_send(fd, 404, "text/plain", "Not found", 9);
            panelLog("Panel: blocked path traversal attempt: %s", filename);
            return;
        }
    }
    // If realpath fails (dir missing), fall through to fopen which will trigger fallback

    FILE *f = fopen(path, "r");
    if (!f) {
        // Serve embedded fallback
        const char *fallback =
            "<!DOCTYPE html><html><head><title>dump1090-gg Panel</title></head>"
            "<body style='background:#0a0a1a;color:#e0e0e0;font-family:sans-serif;text-align:center;padding:40px'>"
            "<h1>dump1090-gg Control Panel</h1>"
            "<p>Panel HTML not found at: <code>%s</code></p>"
            "<p>Install panel files to <code>%s/</code></p>"
            "<p>API endpoints: <a href='/api/config'>/api/config</a> | <a href='/api/status'>/api/status</a> | "
            "<a href='/api/connections'>/api/connections</a> | "
            "<a href='/api/aircraft'>/api/aircraft</a> | <a href='/api/stats'>/api/stats</a> | "
            "<a href='/api/logs'>/api/logs</a> | <a href='/api/messages'>/api/messages</a> | "
            "<a href='/api/devices'>/api/devices</a></p>"
            "<p><a href='/devices.html' style='color:#4fc3f7;font-size:1.2em'>&#x1f4e1; SDR Devices Page</a></p>"
            "</body></html>";
        char buf[1024];
        int len = snprintf(buf, sizeof(buf), fallback, path, PanelState.html_dir);
        http_send(fd, 200, "text/html; charset=utf-8", buf, len);
        return;
    }

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (fsize <= 0 || fsize > 2097152) { // 2MB max
        fclose(f);
        http_send(fd, 500, "text/plain", "File too large", 14);
        return;
    }

    char *data = malloc((size_t)fsize);
    if (!data) { fclose(f); http_send(fd, 500, "text/plain", "OOM", 3); return; }

    size_t nread = fread(data, 1, (size_t)fsize, f);
    fclose(f);

    // Determine content type
    const char *ct = "text/plain";
    if (strstr(filename, ".html")) ct = "text/html; charset=utf-8";
    else if (strstr(filename, ".js")) ct = "application/javascript";
    else if (strstr(filename, ".css")) ct = "text/css";
    else if (strstr(filename, ".json")) ct = "application/json";
    else if (strstr(filename, ".png")) ct = "image/png";
    else if (strstr(filename, ".svg")) ct = "image/svg+xml";

    http_send(fd, 200, ct, data, (int)nread);
    free(data);
}

// ============================= SDR Devices API ===========================

// Tuner type cache: maps serial -> tuner_type to survive across API calls
// (probing fails when devices are already open by the legacy decoder)
#define MAX_TUNER_CACHE 16
static struct { char serial[64]; int tuner_type; } tuner_cache[MAX_TUNER_CACHE];
static int tuner_cache_count = 0;

static int tuner_cache_lookup(const char *serial) {
    for (int i = 0; i < tuner_cache_count; i++)
        if (!strcmp(tuner_cache[i].serial, serial))
            return tuner_cache[i].tuner_type;
    return -1;
}

static void tuner_cache_store(const char *serial, int tuner_type) {
    if (tuner_type < 0) return;
    // Update existing entry
    for (int i = 0; i < tuner_cache_count; i++) {
        if (!strcmp(tuner_cache[i].serial, serial)) {
            tuner_cache[i].tuner_type = tuner_type;
            return;
        }
    }
    // Add new entry
    if (tuner_cache_count < MAX_TUNER_CACHE) {
        snprintf(tuner_cache[tuner_cache_count].serial,
                 sizeof(tuner_cache[tuner_cache_count].serial), "%.63s", serial);
        tuner_cache[tuner_cache_count].tuner_type = tuner_type;
        tuner_cache_count++;
    }
}

static const char *tuner_name(int type);

// Probe all RTL-SDR devices and cache their tuner types.
// Must be called before any device is opened (so rtlsdr_open succeeds for all).
void panelProbeAllTuners(void)
{
#ifdef ENABLE_RTLSDR
    int count = rtlsdr_get_device_count();
    if (count <= 0) return;

    // Open all devices at once to avoid kernel driver reattach race
    rtlsdr_dev_t **devs = calloc(count, sizeof(rtlsdr_dev_t *));
    if (!devs) return;

    for (int i = 0; i < count; i++)
        rtlsdr_open(&devs[i], i);

    for (int i = 0; i < count; i++) {
        if (!devs[i]) continue;
        char serial[256] = {0};
        rtlsdr_get_device_usb_strings(i, NULL, NULL, serial);
        int t = rtlsdr_get_tuner_type(devs[i]);
        tuner_cache_store(serial, t);
        fprintf(stderr, "Panel: device #%d SN=%s tuner=%s (%d)\n",
                i, serial, tuner_name(t), t);
    }

    for (int i = 0; i < count; i++)
        if (devs[i]) rtlsdr_close(devs[i]);

    free(devs);
#endif
}

// Tuner type to name + frequency range
static const char *tuner_name(int type) {
    switch(type) {
        case RTLSDR_TUNER_E4000:  return "E4000";
        case RTLSDR_TUNER_FC0012: return "FC0012";
        case RTLSDR_TUNER_FC0013: return "FC0013";
        case RTLSDR_TUNER_FC2580: return "FC2580";
        case RTLSDR_TUNER_R820T:  return "R820T";
        case RTLSDR_TUNER_R828D:  return "R828D";
        default:                  return "unknown";
    }
}
static const char *tuner_freq_range(int type) {
    switch(type) {
        case RTLSDR_TUNER_E4000:  return "52-2200 MHz (with gap 1100-1250)";
        case RTLSDR_TUNER_FC0012: return "22-948 MHz";
        case RTLSDR_TUNER_FC0013: return "22-1100 MHz";
        case RTLSDR_TUNER_FC2580: return "146-308, 438-924 MHz";
        case RTLSDR_TUNER_R820T:  return "24-1766 MHz";
        case RTLSDR_TUNER_R828D:  return "24-1766 MHz";
        default:                  return "unknown";
    }
}

static void api_get_receivers(int fd)
{
    char *buf = malloc(16384);
    if (!buf) { http_send(fd, 500, "text/plain", "OOM", 3); return; }
    int pos = 0;

    pos += snprintf(buf + pos, 16384 - pos, "{\"receivers\":[");

    for (int i = 0; i < SdrManager.count; i++) {
        sdr_receiver_t *rx = &SdrManager.receivers[i];
        if (i > 0) pos += snprintf(buf + pos, 16384 - pos, ",");
        pos += snprintf(buf + pos, 16384 - pos,
            "{\"id\":%d,\"serial\":\"%.63s\",\"serial_actual\":\"%.63s\","
            "\"role\":\"%s\",\"state\":\"%s\","
            "\"freq\":%d,\"gain\":%.1f,\"ppm\":%d,"
            "\"manufacturer\":\"%.63s\",\"product\":\"%.63s\","
            "\"tuner\":\"%s\",\"freq_range\":\"%s\","
            "\"dev_index\":%d}",
            rx->id, rx->config.serial, rx->serial_actual,
            sdrRoleName(rx->config.role), rxStateName(rx->state),
            rx->config.freq, rx->config.gain, rx->config.ppm_error,
            rx->manufacturer, rx->product,
            rx->rtl.tuner_type >= 0 ? tuner_name(rx->rtl.tuner_type) : "unknown",
            rx->rtl.tuner_type >= 0 ? tuner_freq_range(rx->rtl.tuner_type) : "unknown",
            rx->dev_index);
    }

    pos += snprintf(buf + pos, 16384 - pos, "],\"count\":%d,\"max\":%d}",
                    SdrManager.count, MAX_SDR_RECEIVERS);

    http_send_json(fd, buf, pos);
    free(buf);
}

static void rx_set_freq_for_role(rx_config_t *cfg)
{
    switch (cfg->role) {
        case SDR_ROLE_ADSB:       cfg->freq = 1090000000; cfg->sample_rate = 2400000; break;
        case SDR_ROLE_FLARM:      cfg->freq = 868400000;  cfg->sample_rate = 1600000; break;
        case SDR_ROLE_ACARS:      cfg->freq = 131550000;  cfg->sample_rate = 2400000; break;
        case SDR_ROLE_VDL2:       cfg->freq = 136975000;  cfg->sample_rate = 2400000; break;
        case SDR_ROLE_RADIOSONDE: cfg->freq = 403000000;  cfg->sample_rate = 2400000; break;
        default:                  cfg->freq = 0;          cfg->sample_rate = 0;       break;
    }
}

static void api_post_receiver_assign(int fd, const char *body)
{
    // Body format: {"serial":"00000101","role":"adsb","gain":40.0}
    // or           {"serial":"00000101","role":"none"} to unassign

    char serial[64] = {0};
    char role_str[16] = {0};
    float gain = MODES_DEFAULT_GAIN;
    int ppm = 0;

    // Simple JSON parsing (no library)
    const char *p;
    if ((p = strstr(body, "\"serial\"")) != NULL) {
        p = strchr(p + 8, '"'); if (p) { p++; const char *e = strchr(p, '"');
        if (e && (e - p) < 63) { memcpy(serial, p, e - p); serial[e - p] = '\0'; }
        }
    }
    if ((p = strstr(body, "\"role\"")) != NULL) {
        p = strchr(p + 6, '"'); if (p) { p++; const char *e = strchr(p, '"');
        if (e && (e - p) < 15) { memcpy(role_str, p, e - p); role_str[e - p] = '\0'; }
        }
    }
    if ((p = strstr(body, "\"gain\"")) != NULL) {
        p = strchr(p + 6, ':'); if (p) gain = strtof(p + 1, NULL);
    }
    if ((p = strstr(body, "\"ppm\"")) != NULL) {
        p = strchr(p + 5, ':'); if (p) ppm = atoi(p + 1);
    }

    if (!serial[0]) {
        http_send(fd, 400, "application/json", "{\"error\":\"missing serial\"}", 25);
        return;
    }

    sdr_role_t role = SDR_ROLE_NONE;
    if (!strcasecmp(role_str, "adsb")) role = SDR_ROLE_ADSB;
    else if (!strcasecmp(role_str, "flarm")) role = SDR_ROLE_FLARM;
    else if (!strcasecmp(role_str, "acars")) role = SDR_ROLE_ACARS;
    else if (!strcasecmp(role_str, "vdl2")) role = SDR_ROLE_VDL2;
    else if (!strcasecmp(role_str, "radiosonde")) role = SDR_ROLE_RADIOSONDE;

    // Check if this serial is already managed
    int idx = sdrManagerFindBySerial(serial);

    if (role == SDR_ROLE_NONE) {
        // Unassign: remove SdrManager receiver if it exists
        if (idx >= 0) {
            // Sync FlarmConfig.enabled if removing FLARM
            if (SdrManager.receivers[idx].config.role == SDR_ROLE_FLARM)
                FlarmConfig.enabled = 0;
            sdrManagerRemoveReceiver(idx);
        }

        sdrManagerSave();
        char resp[256];
        int rlen = snprintf(resp, sizeof(resp),
            "{\"ok\":true,\"message\":\"Receiver %s removed\"}", serial);
        http_send(fd, 200, "application/json", resp, rlen);
        return;
    }

    // Assign or reassign
    if (idx >= 0) {
        // Already exists — stop, close, update config, reopen
        sdr_receiver_t *rx = &SdrManager.receivers[idx];
        // Sync FlarmConfig.enabled on role change
        if (rx->config.role == SDR_ROLE_FLARM && role != SDR_ROLE_FLARM)
            FlarmConfig.enabled = 0;
        if (rx->state == RX_STATE_RUNNING) rxStop(rx);
        if (rx->state != RX_STATE_IDLE) rxClose(rx);
        rx->config.role = role;
        rx->config.gain = gain;
        rx->config.ppm_error = ppm;
        rx_set_freq_for_role(&rx->config);

        bool ok = rxOpen(rx);
        if (ok) ok = rxStart(rx);
        if (ok && role == SDR_ROLE_FLARM) {
            FlarmConfig.enabled = 1;
            ognClientInit();
        }

        sdrManagerSave();
        char resp[256];
        int rlen = snprintf(resp, sizeof(resp),
            "{\"ok\":%s,\"message\":\"Receiver %s reassigned to %s\",\"state\":\"%s\"}",
            ok ? "true" : "false", serial, sdrRoleName(role), rxStateName(rx->state));
        http_send(fd, 200, "application/json", resp, rlen);
    } else {
        // New assignment
        rx_config_t cfg = {0};
        snprintf(cfg.serial, sizeof(cfg.serial), "%.63s", serial);
        cfg.role = role;
        cfg.gain = gain;
        cfg.ppm_error = ppm;
        rx_set_freq_for_role(&cfg);

        idx = sdrManagerAddReceiver(&cfg);
        if (idx < 0) {
            http_send(fd, 500, "application/json",
                "{\"ok\":false,\"message\":\"Max receivers reached\"}", 46);
            return;
        }

        sdr_receiver_t *rx = &SdrManager.receivers[idx];
        bool ok = rxOpen(rx);
        if (ok) ok = rxStart(rx);
        if (ok && role == SDR_ROLE_FLARM) {
            FlarmConfig.enabled = 1;
            ognClientInit();
        }

        sdrManagerSave();
        char resp[256];
        int rlen = snprintf(resp, sizeof(resp),
            "{\"ok\":%s,\"message\":\"Receiver %s assigned to %s\",\"state\":\"%s\"}",
            ok ? "true" : "false", serial, sdrRoleName(role), rxStateName(rx->state));
        http_send(fd, ok ? 200 : 500, "application/json", resp, rlen);
    }
}

static void api_get_devices(int fd)
{
    char *buf = malloc(16384);
    if (!buf) { http_send(fd, 500, "text/plain", "OOM", 3); return; }
    int pos = 0;

    pos += snprintf(buf + pos, 16384 - pos, "{\"sdr_devices\":[");

#ifdef ENABLE_RTLSDR
    int count = rtlsdr_get_device_count();
    for (int i = 0; i < count; i++) {
        char vendor[256] = {0}, product[256] = {0}, serial[256] = {0};
        rtlsdr_get_device_usb_strings(i, vendor, product, serial);
        const char *name = rtlsdr_get_device_name(i);

        // Check SdrManager for this device
        const char *role = "none";
        const char *state = "idle";
        int tuner_type = -1;

        for (int r = 0; r < SdrManager.count; r++) {
            sdr_receiver_t *rx = &SdrManager.receivers[r];
            if (rx->state >= RX_STATE_OPEN &&
                (!strcmp(rx->serial_actual, serial) || !strcmp(rx->config.serial, serial))) {
                role = sdrRoleName(rx->config.role);
                state = rxStateName(rx->state);
                tuner_type = rx->rtl.tuner_type;
                break;
            }
        }

        // Use cache for any device we don't already know
        if (tuner_type < 0) {
            tuner_type = tuner_cache_lookup(serial);
        }

        if (i > 0) pos += snprintf(buf + pos, 16384 - pos, ",");
        pos += snprintf(buf + pos, 16384 - pos,
            "{\"index\":%d,\"name\":\"%s\",\"vendor\":\"%s\",\"product\":\"%s\","
            "\"serial\":\"%s\",\"role\":\"%s\",\"state\":\"%s\","
            "\"tuner\":\"%s\",\"freq_range\":\"%s\"}",
            i, name ? name : "", vendor, product, serial,
            role, state,
            tuner_type >= 0 ? tuner_name(tuner_type) : "unknown",
            tuner_type >= 0 ? tuner_freq_range(tuner_type) : "unknown");
    }
#endif

    pos += snprintf(buf + pos, 16384 - pos, "],\"usb_devices\":[");

    // Enumerate all USB devices via /sys/bus/usb
    FILE *fp = popen("lsusb 2>/dev/null", "r");
    if (fp) {
        char line[512];
        int first = 1;
        while (fgets(line, sizeof(line), fp)) {
            // Strip newline
            int llen = (int)strlen(line);
            while (llen > 0 && (line[llen-1] == '\n' || line[llen-1] == '\r'))
                line[--llen] = '\0';
            // Skip root hubs
            if (strstr(line, "1d6b:000")) continue;
            if (!first) pos += snprintf(buf + pos, 16384 - pos, ",");
            first = 0;
            // Escape for JSON
            pos += snprintf(buf + pos, 16384 - pos, "\"%s\"", line);
        }
        pclose(fp);
    }

    pos += snprintf(buf + pos, 16384 - pos,
        "],\"rx_count\":%d,\"rx_max\":%d}",
        SdrManager.count, MAX_SDR_RECEIVERS);

    http_send_json(fd, buf, pos);
    free(buf);
}

static void serve_devices_page(int fd)
{
    const char *html =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<title>SDR Devices - dump1090-gg</title>"
        "<style>"
        ":root{--bg:#0a0a1a;--head:#0d0d1f;--border:#1a1a3a;--accent:#4fc3f7;--text:#e0e0e0;--dim:#888;--hover:rgba(79,195,247,0.1)}"
        "body{background:var(--bg);color:var(--text);font-family:'Segoe UI',sans-serif;margin:0;padding:0}"
        "nav{background:var(--head);border-bottom:1px solid var(--border);padding:8px 16px;display:flex;align-items:center;gap:16px;position:sticky;top:0;z-index:100}"
        "nav h1{font-size:18px;color:var(--accent);white-space:nowrap;margin:0}"
        ".tabs{display:flex;gap:4px;flex-wrap:wrap}"
        ".tabs a{padding:6px 14px;border-radius:6px;cursor:pointer;color:var(--dim);transition:.2s;font-size:13px;user-select:none;text-decoration:none}"
        ".tabs a:hover{background:var(--hover);color:var(--text)}"
        ".tabs a.active{background:var(--accent);color:#000;font-weight:600}"
        ".main{padding:20px}"
        "h2{color:var(--accent);margin-top:20px}"
        "table{border-collapse:collapse;width:100%;margin-bottom:20px}"
        "th,td{border:1px solid #333;padding:8px 12px;text-align:left}"
        "th{background:var(--head);color:var(--accent)}"
        "tr:nth-child(even){background:#111122}"
        "tr:hover{background:#1a1a3e}"
        ".present{color:var(--accent);font-weight:bold}"
        ".absent{color:#f44336;font-weight:bold}"
        ".role-none{color:#666}"
        ".chip{display:inline-block;padding:2px 8px;border-radius:10px;font-size:0.85em}"
        ".chip-adsb{background:#1a3a5c;color:var(--accent)}"
        ".chip-flarm{background:#3a2a0c;color:#ff9800}"
        ".chip-none{background:#222;color:#666}"
        ".freq{color:#aaa;font-size:0.9em}"
        ".usb-list{color:#aaa;font-size:0.9em;padding-left:20px}"
        "a{color:var(--accent);text-decoration:none}"
        "a:hover{text-decoration:underline}"
        ".btn{border:1px solid var(--accent);padding:6px 14px;cursor:pointer;border-radius:4px;font-size:0.95em;margin:2px}"
        ".btn-refresh{background:#1a1a2e;color:#4fc3f7}"
        ".btn-refresh:hover{background:#4fc3f7;color:#0a0a1a}"
        ".btn-apply{background:#1a3a1a;color:#66bb6a;border-color:#66bb6a}"
        ".btn-apply:hover{background:#66bb6a;color:#0a0a1a}"
        ".btn-apply:disabled{opacity:0.4;cursor:not-allowed}"
        "select{background:#1a1a2e;color:#e0e0e0;border:1px solid #444;padding:5px 8px;border-radius:4px;font-size:0.95em;min-width:100px}"
        "select:focus{border-color:#4fc3f7;outline:none}"
        "input.gain{background:#1a1a2e;color:#e0e0e0;border:1px solid #444;padding:5px 8px;border-radius:4px;width:60px;text-align:center}"
        ".status-msg{margin:10px 0;padding:10px;border-radius:4px;display:none}"
        ".status-ok{background:#1a3a1a;border:1px solid #66bb6a;color:#66bb6a;display:block}"
        ".status-err{background:#3a1a1a;border:1px solid #f44336;color:#f44336;display:block}"
        ".rx-section{margin-top:0;padding:12px;background:#111122;border:1px solid #333;border-radius:6px}"
        ".rx-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(280px,1fr));gap:12px;margin-top:15px}"
        ".rx-title{color:#4fc3f7;font-size:1.1em;margin-bottom:8px}"
        ".state-running{color:#66bb6a}"
        ".state-open{color:#4fc3f7}"
        ".state-idle{color:#888}"
        ".state-error{color:#f44336}"
        ".state-stopping{color:#ff9800}"
        "</style></head><body>"
        "<nav><h1>&#x2708; dump1090-gg</h1><div class='tabs'>"
        "<a href='/'>&#x2699;&#xfe0f; Config</a>"
        "<a href='/status.html'>&#x1f4e1; Status</a>"
        "<a href='/connections.html'>&#x1f50c; Connections</a>"
        "<a href='/logs.html'>&#x1f4cb; Logs</a>"
        "<a href='/messages.html'>&#x1f4e8; Messages</a>"
        "<a href='/aircraft.html'>&#x2708;&#xfe0f; Aircraft</a>"
        "<a class='active' href='/devices.html'>&#x1f4fb; Devices</a>"
        "</div></nav>"
        "<div class='main'>"
        "<h2>&#x1f50c; SDR Devices</h2>"
        "<p style='color:#888;margin-bottom:15px'>Assign roles to RTL-SDR dongles &mdash; choose ADS-B, FLARM, or None</p>"
        "<button class='btn btn-refresh' onclick='load()'>&#x21bb; Refresh</button>"
        "<div id='status' class='status-msg'></div>"
        "<div id='content'><p>Loading...</p></div>"
        "<script>"
        "var devData=null,rxData=null;"
        ""
        "function showStatus(msg,ok){"
        "var el=document.getElementById('status');"
        "el.textContent=msg;el.className='status-msg '+(ok?'status-ok':'status-err');"
        "setTimeout(function(){el.style.display='none';},5000);"
        "}"
        ""
        "function assign(serial,role,gainEl){"
        "var gain=gainEl?parseFloat(gainEl.value):999999;"
        "if(isNaN(gain))gain=999999;"
        "var body=JSON.stringify({serial:serial,role:role,gain:gain,ppm:0});"
        "fetch('/api/receivers/assign',{method:'POST',headers:{'Content-Type':'application/json'},body:body})"
        ".then(r=>r.json()).then(d=>{"
        "showStatus(d.message||'Done',d.ok);"
        "setTimeout(load,500);"
        "}).catch(e=>showStatus('Error: '+e,false));"
        "}"
        ""
        "function load(){"
        "Promise.all(["
        "fetch('/api/devices').then(r=>r.json()),"
        "fetch('/api/receivers').then(r=>r.json())"
        "]).then(function(results){"
        "devData=results[0];rxData=results[1];"
        "render();"
        "}).catch(e=>{document.getElementById('content').innerHTML='<p style=color:red>Error: '+e+'</p>';});"
        "}"
        ""
        "function getRxForSerial(serial){"
        "if(!rxData)return null;"
        "for(var i=0;i<rxData.receivers.length;i++){"
        "var r=rxData.receivers[i];"
        "if(r.serial==serial||r.serial_actual==serial)return r;}"
        "return null;}"
        ""
        "function render(){"
        "var d=devData;"
        "var h='<h2>SDR Dongles</h2>';"
        "if(d.sdr_devices.length==0){"
        "h+='<p style=\"color:#f44336\">No RTL-SDR devices detected. Connect a dongle and click Refresh.</p>';"
        "}else{"
        "h+='<table><tr><th>#</th><th>Device</th><th>Serial</th><th>Tuner</th>"
        "<th>Assign Role</th><th>Gain (dB)</th><th>Action</th><th>Status</th></tr>';"
        "d.sdr_devices.forEach(function(s){"
        "var rx=getRxForSerial(s.serial);"
        "var curRole=rx?rx.role:(s.role!='none'?s.role:'none');"
        "var curGain=rx?rx.gain:(curRole!='none'?40.0:40.0);"
        "var stateHtml='';"
        "if(s.state=='running'){"
        "stateHtml='<span class=state-running>RUNNING</span>';"
        "}else if(s.state=='open'){"
        "stateHtml='<span class=present>OPEN</span>';"
        "}else{"
        "stateHtml='<span class=role-none>IDLE</span>';"
        "}"
        ""
        "var selId='sel_'+s.serial;"
        "var gainId='gain_'+s.serial;"
        "h+='<tr>';"
        "h+='<td>'+s.index+'</td>';"
        "h+='<td>'+s.name+'<br><small style=color:#888>'+s.vendor+'</small></td>';"
        "h+='<td><code>'+s.serial+'</code></td>';"
        "h+='<td>'+s.tuner+'<br><small class=freq>'+s.freq_range+'</small></td>';"
        // Dropdown
        "h+='<td><select id=\"'+selId+'\">';"
        "h+='<option value=none'+(curRole=='none'?' selected':'')+'>&#x274c; None</option>';"
        "h+='<option value=adsb'+(curRole=='adsb'?' selected':'')+'>&#x2708; ADS-B (1090 MHz)</option>';"
        "h+='<option value=flarm'+(curRole=='flarm'?' selected':'')+'>&#x1f6a9; FLARM / OGNTP (868 MHz)</option>';"
        "h+='<option value=acars'+(curRole=='acars'?' selected':'')+'>&#x1f4e1; ACARS (131 MHz)</option>';"
        "h+='<option value=vdl2'+(curRole=='vdl2'?' selected':'')+'>&#x1f4e1; VDL2 (136 MHz)</option>';"
        "h+='<option value=radiosonde'+(curRole=='radiosonde'?' selected':'')+'>&#x1f388; Radiosonde (403 MHz)</option>';"
        "h+='</select></td>';"
        // Gain input
        "h+='<td><input type=number class=gain id=\"'+gainId+'\" value=\"'+curGain.toFixed(1)+'\" step=0.1 min=-10 max=50></td>';"
        // Apply button
        "h+='<td><button class=\"btn btn-apply\" onclick=\"assign(\\''+s.serial+'\\',document.getElementById(\\''+selId+'\\').value,document.getElementById(\\''+gainId+'\\'))\">&#x2714; Apply</button></td>';"
        "h+='<td>'+stateHtml+'</td>';"
        "h+='</tr>';"
        "});"
        "h+='</table>';"
        "}"
        ""
        // Active receivers section
        "if(rxData&&rxData.count>0){"
        "h+='<h2>Active Multi-SDR Receivers ('+rxData.count+'/'+rxData.max+')</h2>';"
        "h+='<div class=rx-grid>';"
        "rxData.receivers.forEach(function(r){"
        "var sc='state-'+r.state;"
        "h+='<div class=rx-section>';"
        "h+='<div class=rx-title>Receiver #'+r.id+' &mdash; <span class=\"'+sc+'\">'+r.state.toUpperCase()+'</span></div>';"
        "h+='<table style=\"width:auto\"><tr><td>Serial</td><td><code>'+r.serial_actual+'</code></td></tr>';"
        "h+='<tr><td>Role</td><td><span class=\"chip chip-'+r.role+'\">'+r.role.toUpperCase()+'</span></td></tr>';"
        "h+='<tr><td>Frequency</td><td>'+(r.freq/1e6).toFixed(1)+' MHz</td></tr>';"
        "h+='<tr><td>Gain</td><td>'+r.gain.toFixed(1)+' dB</td></tr>';"
        "if(r.manufacturer)h+='<tr><td>Hardware</td><td>'+r.manufacturer+' '+r.product+'</td></tr>';"
        "if(r.tuner)h+='<tr><td>Tuner</td><td>'+r.tuner+' ('+r.freq_range+')</td></tr>';"
        "h+='</table></div>';"
        "});"
        "h+='</div>';"
        "}else{"
        "h+='<h2>Multi-SDR Receivers</h2><p style=color:#888>No receivers configured yet. Use the dropdowns above to assign roles.</p>';"
        "}"
        ""
        "h+='<h2>All USB Devices</h2><ul class=usb-list>';"
        "devData.usb_devices.forEach(function(u){h+='<li>'+u+'</li>';});"
        "if(devData.usb_devices.length==0)h+='<li>No USB devices found</li>';"
        "h+='</ul>';"
        "document.getElementById('content').innerHTML=h;"
        "}"
        "load();"
        "</script></div></body></html>";

    http_send(fd, 200, "text/html; charset=utf-8", html, (int)strlen(html));
}

// ============================= Request Router ============================

static void handle_request(int fd, const char *request, int reqlen)
{
    (void)reqlen;

    // Check auth
    if (!check_auth(request)) {
        http_send(fd, 401, "text/plain", "Unauthorized", 12);
        return;
    }

    // Parse method and path
    char method[8] = {0};
    char path[256] = {0};
    sscanf(request, "%7s %255s", method, path);

    // Route
    if (strcmp(method, "GET") == 0) {
        if (strcmp(path, "/") == 0 || strcmp(path, "/index.html") == 0) {
            serve_file(fd, "index.html");
        } else if (strcmp(path, "/api/config") == 0) {
            api_get_config(fd);
        } else if (strcmp(path, "/api/status") == 0) {
            api_get_status(fd);
        } else if (strcmp(path, "/api/aircraft") == 0) {
            api_get_aircraft(fd);
        } else if (strcmp(path, "/api/stats") == 0) {
            api_get_stats(fd);
        } else if (strcmp(path, "/api/connections") == 0) {
            api_get_connections(fd);
        } else if (strcmp(path, "/api/logs") == 0) {
            api_get_logs(fd);
        } else if (strcmp(path, "/api/messages") == 0) {
            api_get_messages(fd);
        } else if (strcmp(path, "/api/devices") == 0) {
            api_get_devices(fd);
        } else if (strcmp(path, "/api/receivers") == 0) {
            api_get_receivers(fd);
        } else if (strcmp(path, "/devices.html") == 0 || strcmp(path, "/devices") == 0) {
            serve_devices_page(fd);
        } else if (path[0] == '/') {
            serve_file(fd, path + 1);
        } else {
            http_send(fd, 404, "text/plain", "Not found", 9);
        }
    } else if (strcmp(method, "POST") == 0) {
        if (strcmp(path, "/api/config") == 0) {
            // Find body (after \r\n\r\n)
            const char *body = strstr(request, "\r\n\r\n");
            if (body) {
                body += 4;
                api_post_config(fd, body);
            } else {
                http_send(fd, 400, "text/plain", "No body", 7);
            }
        } else if (strcmp(path, "/api/receivers/assign") == 0) {
            const char *body = strstr(request, "\r\n\r\n");
            if (body) {
                body += 4;
                api_post_receiver_assign(fd, body);
            } else {
                http_send(fd, 400, "text/plain", "No body", 7);
            }
        } else {
            http_send(fd, 404, "text/plain", "Not found", 9);
        }
    } else if (strcmp(method, "OPTIONS") == 0) {
        // CORS preflight
        http_send(fd, 200, "text/plain", "", 0);
    } else {
        http_send(fd, 405, "text/plain", "Method not allowed", 18);
    }
}

// ============================= Server Thread =============================

static void *panel_thread_entry(void *arg)
{
    (void)arg;

    panelLog("Panel: HTTP server started on port %d", PanelState.port);

    while (PanelState.running) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);

        int client_fd = accept(PanelState.listen_fd,
                               (struct sockaddr *)&client_addr, &client_len);
        if (client_fd < 0) {
            if (errno == EINTR) continue;
            if (!PanelState.running) break;
            usleep(100000);
            continue;
        }

        // Set receive timeout
        struct timeval tv = {5, 0};
        setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // Read request (up to 64KB for POST bodies)
        char *reqbuf = malloc(65536);
        if (reqbuf) {
            int total = 0;
            while (total < 65535) {
                int n = (int)read(client_fd, reqbuf + total, (size_t)(65535 - total));
                if (n <= 0) break;
                total += n;
                reqbuf[total] = '\0';
                // Check if we have the full headers
                if (strstr(reqbuf, "\r\n\r\n")) {
                    // For GET requests, we're done
                    if (strncmp(reqbuf, "GET", 3) == 0) break;
                    // For POST, check Content-Length
                    const char *cl = strstr(reqbuf, "Content-Length:");
                    if (cl) {
                        long content_len = strtol(cl + 15, NULL, 10);
                        if (content_len < 0 || content_len > 65000) break;  // reject absurd sizes
                        const char *body_start = strstr(reqbuf, "\r\n\r\n") + 4;
                        int header_len = (int)(body_start - reqbuf);
                        if (total >= header_len + (int)content_len) break;
                    } else {
                        break;
                    }
                }
            }

            if (total > 0)
                handle_request(client_fd, reqbuf, total);

            free(reqbuf);
        }

        close(client_fd);
    }

    return NULL;
}

// ============================= Public API ================================

void panelStart(void)
{
    if (!PanelState.enabled) return;

    // Create listening socket
    PanelState.listen_fd = socket(AF_INET6, SOCK_STREAM, 0);
    if (PanelState.listen_fd < 0) {
        PanelState.listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    }
    if (PanelState.listen_fd < 0) {
        fprintf(stderr, "Panel: cannot create socket: %s\n", strerror(errno));
        return;
    }

    int opt = 1;
    setsockopt(PanelState.listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Dual-stack: allow IPv4 connections on IPv6 socket
    int v6only = 0;
    setsockopt(PanelState.listen_fd, IPPROTO_IPV6, IPV6_V6ONLY, &v6only, sizeof(v6only));

    struct sockaddr_in6 addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin6_family = AF_INET6;
    addr.sin6_port = htons((uint16_t)PanelState.port);
    addr.sin6_addr = in6addr_any;

    if (bind(PanelState.listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        // Fallback to IPv4
        close(PanelState.listen_fd);
        PanelState.listen_fd = socket(AF_INET, SOCK_STREAM, 0);
        setsockopt(PanelState.listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in addr4;
        memset(&addr4, 0, sizeof(addr4));
        addr4.sin_family = AF_INET;
        addr4.sin_port = htons((uint16_t)PanelState.port);
        addr4.sin_addr.s_addr = INADDR_ANY;

        if (bind(PanelState.listen_fd, (struct sockaddr *)&addr4, sizeof(addr4)) < 0) {
            fprintf(stderr, "Panel: cannot bind to port %d: %s\n",
                    PanelState.port, strerror(errno));
            close(PanelState.listen_fd);
            PanelState.listen_fd = -1;
            return;
        }
    }

    if (listen(PanelState.listen_fd, 5) < 0) {
        fprintf(stderr, "Panel: listen failed: %s\n", strerror(errno));
        close(PanelState.listen_fd);
        PanelState.listen_fd = -1;
        return;
    }

    PanelState.running = 1;
    if (pthread_create(&PanelState.thread, NULL, panel_thread_entry, NULL) != 0) {
        fprintf(stderr, "Panel: cannot create thread: %s\n", strerror(errno));
        PanelState.running = 0;
        close(PanelState.listen_fd);
        PanelState.listen_fd = -1;
        return;
    }

    pthread_detach(PanelState.thread);
}

void panelStop(void)
{
    if (!PanelState.running) return;

    PanelState.running = 0;
    if (PanelState.listen_fd >= 0) {
        close(PanelState.listen_fd);
        PanelState.listen_fd = -1;
    }
}
