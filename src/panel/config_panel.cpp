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
#include "gsm_calibrate.h"
#include "gsm_tracker.h"
#include "lte_tracker.h"
#include "iot_tracker.h"
#include "fanet_decode.h"
#include "sarsat_decode.h"
#include "pocsag_demod.h"
#include "decoder_config.h"
#include "airframes_feed.h"

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
#include <poll.h>
#include <limits.h>

#include <string>
#include <string_view>
#include <cstdio>

// Helper: format into std::string (like sprintf but returns std::string)
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

panel_state_t PanelState;

// Sonde feed config (radiosondy.info and wettersonde.net — no C client yet, UI-only)
static bool RadiosondyEnabled = false;
static bool WettersondeEnabled = false;

// ============================= SDR Diagnostics ============================

#define DIAG_MAX_DEVICES    8
#define DIAG_MAX_FREQ_STEPS 20
#define DIAG_MAX_SR_STEPS   12

typedef struct {
    int     freq_hz;
    int     sample_rate;
    bool    pll_locked;
    float   noise_floor_db;     // average magnitude in dB
    float   dc_offset;          // DC bias (deviation from 127.5)
    float   iq_spread;          // std deviation of IQ samples
} diag_measurement_t;

typedef struct {
    char    serial[64];
    char    tuner[32];
    char    freq_range[64];
    int     tuner_type;
    int     max_gain_steps;
    float   max_gain_db;
    int     gain_list[64];      // supported gain values in 0.1 dB
    int     num_measurements;
    diag_measurement_t measurements[DIAG_MAX_FREQ_STEPS * DIAG_MAX_SR_STEPS];
    bool    complete;
    char    error[128];
} diag_device_result_t;

typedef struct {
    bool    running;
    bool    complete;
    int     device_count;
    char    librtlsdr_version[64];
    diag_device_result_t devices[DIAG_MAX_DEVICES];
    pthread_mutex_t mutex;
    pthread_t thread;
} diag_state_t;

static diag_state_t DiagState = {0};

// Forward declarations
static const char *tuner_name_sdr(sdr_tuner_type_t type);
static const char *tuner_freq_range_sdr(sdr_tuner_type_t type);
static void http_send(int fd, int code, const char *content_type, const char *body, int body_len);
static void http_send_json(int fd, const char *json, int len);

// Run a single diagnostic measurement on an opened SDR device (via backend layer)
static void diag_measure(sdr_device_t *dev, int freq, int sample_rate,
                         diag_measurement_t *out)
{
    out->freq_hz = freq;
    out->sample_rate = sample_rate;
    out->pll_locked = true;
    out->noise_floor_db = -99.0f;
    out->dc_offset = 0.0f;
    out->iq_spread = 0.0f;

    // Set sample rate
    if (sdr_set_sample_rate(dev, (uint32_t)sample_rate) < 0) {
        out->pll_locked = false;
        return;
    }

    // Set frequency — PLL lock check happens here
    if (sdr_set_frequency(dev, (uint32_t)freq) < 0) {
        out->pll_locked = false;
        return;
    }

    usleep(50000);  // 50ms settle time

    // Read a small buffer of IQ samples
    uint8_t buf[16384];
    int n_read = 0;
    sdr_reset_buffer(dev);
    if (sdr_read_sync(dev, buf, sizeof(buf), &n_read) < 0 || n_read < 1024) {
        out->pll_locked = false;
        return;
    }

    // Analyze IQ data
    double sum_i = 0, sum_q = 0;
    double sum_i2 = 0, sum_q2 = 0;
    double sum_mag = 0;
    int samples = n_read / 2;

    for (int i = 0; i < n_read; i += 2) {
        double vi = (double)buf[i] - 127.5;
        double vq = (double)buf[i+1] - 127.5;
        sum_i += buf[i];
        sum_q += buf[i+1];
        sum_i2 += vi * vi;
        sum_q2 += vq * vq;
        sum_mag += sqrt(vi * vi + vq * vq);
    }

    double mean_i = sum_i / samples;
    double mean_q = sum_q / samples;
    out->dc_offset = (float)(fabs(mean_i - 127.5) + fabs(mean_q - 127.5));

    double variance = (sum_i2 + sum_q2) / samples;
    out->iq_spread = (float)sqrt(variance);

    // If IQ spread is very narrow (<3), the tuner is deaf (PLL not truly locked)
    if (out->iq_spread < 3.0f) {
        out->pll_locked = false;
    }

    // Noise floor in dB (relative to full scale 128)
    double avg_mag = sum_mag / samples;
    if (avg_mag > 0)
        out->noise_floor_db = (float)(20.0 * log10(avg_mag / 128.0));
    else
        out->noise_floor_db = -99.0f;
}

static void *diag_thread_entry(void *arg)
{
    (void)arg;

    // Test frequencies (Hz)
    static const int test_freqs[] = {
        24000000, 50000000, 100000000, 200000000, 300000000,
        400000000, 404500000, 500000000, 600000000, 700000000,
        800000000, 868800000, 935000000, 1000000000, 1090000000,
        1200000000, 1400000000, 1600000000, 1766000000
    };
    static const int num_freqs = 19;

    // Test sample rates (Hz)
    static const int test_srs[] = {
        250000, 1024000, 1600000, 2000000, 2400000, 3200000
    };
    static const int num_srs = 6;

    // Get librtlsdr version info
    pthread_mutex_lock(&DiagState.mutex);
    snprintf(DiagState.librtlsdr_version, sizeof(DiagState.librtlsdr_version),
             "backend layer");
    pthread_mutex_unlock(&DiagState.mutex);

    sdr_dev_info_t devinfo[DIAG_MAX_DEVICES];
    int dev_count = sdrBackendEnumerateAll(devinfo, DIAG_MAX_DEVICES);
    if (dev_count <= 0) {
        pthread_mutex_lock(&DiagState.mutex);
        DiagState.running = false;
        DiagState.complete = true;
        DiagState.device_count = 0;
        pthread_mutex_unlock(&DiagState.mutex);
        return NULL;
    }
    if (dev_count > DIAG_MAX_DEVICES) dev_count = DIAG_MAX_DEVICES;

    pthread_mutex_lock(&DiagState.mutex);
    DiagState.device_count = dev_count;
    pthread_mutex_unlock(&DiagState.mutex);

    for (int d = 0; d < dev_count; d++) {
        diag_device_result_t *dr = &DiagState.devices[d];

        pthread_mutex_lock(&DiagState.mutex);
        snprintf(dr->serial, sizeof(dr->serial), "%.63s", devinfo[d].serial);
        dr->complete = false;
        dr->num_measurements = 0;
        dr->error[0] = '\0';
        pthread_mutex_unlock(&DiagState.mutex);

        // Check if this device is currently in use by SdrManager
        int rx_idx = sdrManagerFindBySerial(devinfo[d].serial);
        sdr_receiver_t *managed_rx = NULL;
        rx_state_t prev_state = RX_STATE_IDLE;
        rx_config_t saved_config = {0};

        if (rx_idx >= 0) {
            managed_rx = &SdrManager.receivers[rx_idx];
            prev_state = managed_rx->state;
            saved_config = managed_rx->config;

            // Stop the receiver to free the device
            if (managed_rx->state == RX_STATE_RUNNING) rxStop(managed_rx);
            if (managed_rx->state != RX_STATE_IDLE) rxClose(managed_rx);
            usleep(500000);  // let OS release USB
        }

        // Open device via backend layer
        const sdr_backend_ops_t *ops;
        sdr_device_t *dev;
        int ttype;
        int gains[64];
        int n_gains;
        int meas_idx;

        ops = sdrBackendResolve(devinfo[d].backend);
        dev = ops ? ops->open_by_serial(devinfo[d].serial) : NULL;
        if (!dev) {
            pthread_mutex_lock(&DiagState.mutex);
            snprintf(dr->error, sizeof(dr->error), "Failed to open device %s", devinfo[d].serial);
            dr->complete = true;
            pthread_mutex_unlock(&DiagState.mutex);
            goto restore;
        }
        dev->ops = ops;

        // Get tuner info from backend
        ttype = sdr_get_tuner_type(dev);
        pthread_mutex_lock(&DiagState.mutex);
        dr->tuner_type = ttype;
        snprintf(dr->tuner, sizeof(dr->tuner), "%s", tuner_name_sdr((sdr_tuner_type_t)ttype));
        snprintf(dr->freq_range, sizeof(dr->freq_range), "%s", tuner_freq_range_sdr((sdr_tuner_type_t)ttype));
        pthread_mutex_unlock(&DiagState.mutex);

        // Get gain info
        n_gains = sdr_get_tuner_gains(dev, gains, 64);
        if (n_gains > 0) {
            dr->max_gain_steps = n_gains;
            dr->max_gain_db = gains[n_gains - 1] / 10.0f;
            int copy = n_gains > 64 ? 64 : n_gains;
            memcpy(dr->gain_list, gains, copy * sizeof(int));
        }

        // Set max gain for testing
        if (n_gains > 0) {
            sdr_set_gain_mode(dev, 1);
            sdr_set_gain(dev, gains[n_gains - 1]);
        }

        // Run frequency sweep at default sample rate (2.4 MSPS)
        meas_idx = 0;
        for (int f = 0; f < num_freqs && meas_idx < DIAG_MAX_FREQ_STEPS * DIAG_MAX_SR_STEPS; f++) {
            diag_measurement_t m = {0};
            diag_measure(dev, test_freqs[f], 2400000, &m);

            pthread_mutex_lock(&DiagState.mutex);
            dr->measurements[meas_idx++] = m;
            dr->num_measurements = meas_idx;
            pthread_mutex_unlock(&DiagState.mutex);
        }

        // Run sample rate sweep at 404.5 MHz
        for (int s = 0; s < num_srs && meas_idx < DIAG_MAX_FREQ_STEPS * DIAG_MAX_SR_STEPS; s++) {
            diag_measurement_t m = {0};
            diag_measure(dev, 404500000, test_srs[s], &m);

            pthread_mutex_lock(&DiagState.mutex);
            dr->measurements[meas_idx++] = m;
            dr->num_measurements = meas_idx;
            pthread_mutex_unlock(&DiagState.mutex);
        }

        // Run sample rate sweep at 1090 MHz
        for (int s = 0; s < num_srs && meas_idx < DIAG_MAX_FREQ_STEPS * DIAG_MAX_SR_STEPS; s++) {
            diag_measurement_t m = {0};
            diag_measure(dev, 1090000000, test_srs[s], &m);

            pthread_mutex_lock(&DiagState.mutex);
            dr->measurements[meas_idx++] = m;
            dr->num_measurements = meas_idx;
            pthread_mutex_unlock(&DiagState.mutex);
        }

        ops->close(dev);

        pthread_mutex_lock(&DiagState.mutex);
        dr->complete = true;
        pthread_mutex_unlock(&DiagState.mutex);

restore:
        // Restore the receiver if it was managed
        if (managed_rx && prev_state >= RX_STATE_OPEN) {
            usleep(200000);
            bool ok = rxOpen(managed_rx);
            if (ok && prev_state == RX_STATE_RUNNING) {
                ok = rxStart(managed_rx);
                if (ok && saved_config.role == SDR_ROLE_FLARM) {
                    FlarmConfig.enabled = 1;
                    ognClientInit();
                }
            }
        }
    }

    pthread_mutex_lock(&DiagState.mutex);
    DiagState.running = false;
    DiagState.complete = true;
    pthread_mutex_unlock(&DiagState.mutex);

    return NULL;
}

static void api_get_diagnostics(int fd)
{
    std::string buf;
    buf.reserve(4096);

    pthread_mutex_lock(&DiagState.mutex);

    buf += sfmt(
        "{\"running\":%s,\"complete\":%s,\"device_count\":%d,"
        "\"librtlsdr_version\":\"%s\",\"devices\":[",
        DiagState.running ? "true" : "false",
        DiagState.complete ? "true" : "false",
        DiagState.device_count,
        DiagState.librtlsdr_version);

    for (int d = 0; d < DiagState.device_count && d < DIAG_MAX_DEVICES; d++) {
        diag_device_result_t *dr = &DiagState.devices[d];
        if (d > 0) buf += ',';
        buf += sfmt(
            "{\"serial\":\"%s\",\"tuner\":\"%s\",\"freq_range\":\"%s\","
            "\"tuner_type\":%d,\"max_gain_db\":%.1f,\"max_gain_steps\":%d,"
            "\"gain_list\":[",
            dr->serial, dr->tuner, dr->freq_range,
            dr->tuner_type, dr->max_gain_db, dr->max_gain_steps);

        for (int g = 0; g < dr->max_gain_steps && g < 64; g++) {
            if (g > 0) buf += ',';
            buf += sfmt("%.1f", dr->gain_list[g] / 10.0f);
        }

        buf += sfmt(
            "],\"complete\":%s,\"error\":\"%s\",\"measurements\":[",
            dr->complete ? "true" : "false", dr->error);

        for (int m = 0; m < dr->num_measurements; m++) {
            diag_measurement_t *meas = &dr->measurements[m];
            if (m > 0) buf += ',';
            buf += sfmt(
                "{\"freq\":%d,\"sr\":%d,\"pll\":%s,"
                "\"noise\":%.1f,\"dc_offset\":%.2f,\"iq_spread\":%.2f}",
                meas->freq_hz, meas->sample_rate,
                meas->pll_locked ? "true" : "false",
                meas->noise_floor_db, meas->dc_offset, meas->iq_spread);
        }

        buf += "]}";
    }

    buf += "]}";

    pthread_mutex_unlock(&DiagState.mutex);

    http_send_json(fd, buf.c_str(), (int)buf.size());
}

static void api_post_diagnostics_start(int fd)
{
    pthread_mutex_lock(&DiagState.mutex);
    if (DiagState.running) {
        pthread_mutex_unlock(&DiagState.mutex);
        http_send(fd, 409, "application/json",
            "{\"ok\":false,\"error\":\"Diagnostics already running\"}", 51);
        return;
    }

    // Reset state
    memset(&DiagState.devices, 0, sizeof(DiagState.devices));
    DiagState.running = true;
    DiagState.complete = false;
    DiagState.device_count = 0;
    DiagState.librtlsdr_version[0] = '\0';
    pthread_mutex_unlock(&DiagState.mutex);

    pthread_create(&DiagState.thread, NULL, diag_thread_entry, NULL);
    pthread_detach(DiagState.thread);
    http_send(fd, 200, "application/json",
        "{\"ok\":true,\"message\":\"Diagnostics started\"}", 43);
}

// ============================= Initialization ============================

void panelInitConfig(void)
{
    memset(&PanelState, 0, sizeof(PanelState));
    PanelState.port = PANEL_DEFAULT_PORT;
    PanelState.listen_fd = -1;
    snprintf(PanelState.html_dir, sizeof(PanelState.html_dir), "%s", PANEL_HTML_DIR);
    pthread_mutex_init(&PanelState.log_mutex, NULL);
    pthread_mutex_init(&PanelState.msg_mutex, NULL);
    pthread_mutex_init(&DiagState.mutex, NULL);
}

// ============================= CLI Options ================================

bool panelHandleOption(int argc, char **argv, int *jptr)
{
    int j = *jptr;
    bool more = (j + 1 < argc);
    std::string_view arg(argv[j]);

    if (arg == "--panel") {
        PanelState.enabled = 1;
    } else if (arg == "--panel-port" && more) {
        PanelState.enabled = 1;
        PanelState.port = atoi(argv[++j]);
    } else if (arg == "--panel-password" && more) {
        PanelState.enabled = 1;
        snprintf(PanelState.password, sizeof(PanelState.password), "%s", argv[++j]);
    } else if (arg == "--panel-html-dir" && more) {
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
    int off = snprintf(line, sizeof(line), "%02d/%02d %02d:%02d:%02d.%03d ",
                       tm.tm_mday, tm.tm_mon + 1,
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

static uint8_t b64_table[256];
static bool b64_table_init = false;

static void init_b64_table(void) {
    if (b64_table_init) return;
    memset(b64_table, 0, sizeof(b64_table));
    const char *chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (int i = 0; chars[i]; i++)
        b64_table[(uint8_t)chars[i]] = (uint8_t)i;
    b64_table_init = true;
}

static int base64_decode(const char *in, char *out, int outlen)
{
    init_b64_table();
    int i = 0, o = 0;
    int len = (int)strlen(in);
    while (i < len && o < outlen - 1) {
        if (i + 3 >= len) break;
        uint32_t a = b64_table[(uint8_t)in[i++]];
        uint32_t b = b64_table[(uint8_t)in[i++]];
        uint32_t c = b64_table[(uint8_t)in[i++]];
        uint32_t d = b64_table[(uint8_t)in[i++]];
        uint32_t triple = (a << 18) | (b << 12) | (c << 6) | d;
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
    volatile uint8_t result = 0;
    for (size_t i = 0; i < len; i++)
        result |= (uint8_t)a[i] ^ (uint8_t)b[i];
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
        uint8_t c = (uint8_t)in[i];
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
    std::string_view sv(v);
    if (sv.substr(0, 4) == "true") return true;
    if (sv.substr(0, 5) == "false") return false;
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
                    MlatConfig.user = (char*)malloc(nlen + 1);
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
        std::string needle = std::string("\"name\":\"") + Modes.beast_feeds[i].name + "\"";
        const char *pos = strstr(body, needle.c_str());
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
                    Modes.adsbhub_ckey = (char*)malloc(clen + 1);
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

    // POCSAG decoder output toggle
    const char *pocsag = json_find_obj(body, "sdr_pocsag");
    if (pocsag) {
        PocsagOutputEnabled = json_get_bool(pocsag, "enabled", PocsagOutputEnabled) ? 1 : 0;
        panelLog("Panel: POCSAG output %s", PocsagOutputEnabled ? "enabled" : "disabled");
    }

    // GSM decoder output toggle
    const char *gsm = json_find_obj(body, "sdr_gsm");
    if (gsm) {
        GsmOutputEnabled = json_get_bool(gsm, "enabled", GsmOutputEnabled) ? 1 : 0;
        panelLog("Panel: GSM output %s", GsmOutputEnabled ? "enabled" : "disabled");
    }

    // IoT 868 decoder output toggle
    const char *iot = json_find_obj(body, "sdr_iot868");
    if (iot) {
        IotOutputEnabled = json_get_bool(iot, "enabled", IotOutputEnabled) ? 1 : 0;
        panelLog("Panel: IoT 868 output %s", IotOutputEnabled ? "enabled" : "disabled");
    }

    // FANET decoder output toggle
    const char *fanet = json_find_obj(body, "sdr_fanet");
    if (fanet) {
        FanetOutputEnabled = json_get_bool(fanet, "enabled", FanetOutputEnabled) ? 1 : 0;
        panelLog("Panel: FANET output %s", FanetOutputEnabled ? "enabled" : "disabled");
    }

    // Sarsat decoder output toggle
    const char *sarsat = json_find_obj(body, "sdr_sarsat");
    if (sarsat) {
        SarsatOutputEnabled = json_get_bool(sarsat, "enabled", SarsatOutputEnabled) ? 1 : 0;
        panelLog("Panel: Sarsat output %s", SarsatOutputEnabled ? "enabled" : "disabled");
    }

    // Airframes.io ACARS/VDL2 feeds
    const char *airframes = json_find_obj(body, "airframes");
    if (airframes) {
        // Station ID
        const char *sid = json_find_key(airframes, "station_id");
        if (sid) {
            while (*sid == ' ') sid++;
            if (*sid == '"') {
                sid++;
                const char *e = strchr(sid, '"');
                if (e && (e - sid) < (int)sizeof(Modes.airframes_station_id) - 1) {
                    memcpy(Modes.airframes_station_id, sid, e - sid);
                    Modes.airframes_station_id[e - sid] = '\0';
                }
            }
        }

        // ACARS feed
        const char *af_acars = json_find_obj(airframes, "acars");
        if (af_acars) {
            int was_enabled = Modes.airframes_acars_feed.enabled;
            Modes.airframes_acars_feed.enabled = json_get_bool(af_acars, "enabled", Modes.airframes_acars_feed.enabled) ? 1 : 0;
            if (was_enabled != Modes.airframes_acars_feed.enabled) {
                panelLog("Panel: Airframes ACARS feed %s",
                         Modes.airframes_acars_feed.enabled ? "enabled" : "disabled");
                if (Modes.airframes_acars_feed.enabled)
                    airframesFeedInit();
            }
        }

        // VDL2 feed
        const char *af_vdl2 = json_find_obj(airframes, "vdl2");
        if (af_vdl2) {
            int was_enabled = Modes.airframes_vdl2_feed.enabled;
            Modes.airframes_vdl2_feed.enabled = json_get_bool(af_vdl2, "enabled", Modes.airframes_vdl2_feed.enabled) ? 1 : 0;
            if (was_enabled != Modes.airframes_vdl2_feed.enabled) {
                panelLog("Panel: Airframes VDL2 feed %s",
                         Modes.airframes_vdl2_feed.enabled ? "enabled" : "disabled");
                if (Modes.airframes_vdl2_feed.enabled)
                    airframesFeedInit();
            }
        }
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

    char *data = (char*)malloc((size_t)sz + 1);
    if (!data) { fclose(f); return; }
    size_t rd = fread(data, 1, (size_t)sz, f);
    fclose(f);
    data[rd] = '\0';

    // For each beast feed, look for its name in saved config and restore enabled state
    for (int i = 0; i < Modes.beast_feed_count; i++) {
        // Search for "name":"<feedname>" in the JSON
        std::string needle = std::string("\"name\":\"") + Modes.beast_feeds[i].name + "\"";
        const char *pos = strstr(data, needle.c_str());
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
                    MlatConfig.user = (char*)malloc(nlen + 1);
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

    // Restore airframes.io feed settings
    const char *airframes = json_find_obj(data, "airframes");
    if (airframes) {
        const char *sid = json_find_key(airframes, "station_id");
        if (sid) {
            while (*sid == ' ') sid++;
            if (*sid == '"') {
                sid++;
                const char *e = strchr(sid, '"');
                if (e && (e - sid) < (int)sizeof(Modes.airframes_station_id) - 1) {
                    memcpy(Modes.airframes_station_id, sid, e - sid);
                    Modes.airframes_station_id[e - sid] = '\0';
                }
            }
        }
        const char *af_acars = json_find_obj(airframes, "acars");
        if (af_acars) {
            Modes.airframes_acars_feed.enabled = json_get_bool(af_acars, "enabled", false) ? 1 : 0;
            fprintf(stderr, "Panel: Airframes ACARS %s by saved config\n",
                    Modes.airframes_acars_feed.enabled ? "enabled" : "disabled");
        }
        const char *af_vdl2 = json_find_obj(airframes, "vdl2");
        if (af_vdl2) {
            Modes.airframes_vdl2_feed.enabled = json_get_bool(af_vdl2, "enabled", false) ? 1 : 0;
            fprintf(stderr, "Panel: Airframes VDL2 %s by saved config\n",
                    Modes.airframes_vdl2_feed.enabled ? "enabled" : "disabled");
        }
    }

    free(data);
}

// ============================= API: GET /api/config ======================

static void api_get_config(int fd)
{
    std::string buf;
    buf.reserve(8192);
    char esc[256];

    buf += "{\n";

    // Station
    buf += sfmt(
        "\"station\":{\"lat\":%.6f,\"lon\":%.6f,\"max_range\":%.0f,\"name\":\"%s\"},\n",
        Modes.fUserLat, Modes.fUserLon, Modes.maxRange / 1852.0,
        MlatConfig.user ? json_escape(esc, sizeof(esc), MlatConfig.user) : "");

    // SDR ADS-B and FLARM hardware details moved to /api/receivers and /api/decoders

    // Beast feeds
    buf += "\"beast_feeds\":[\n";
    for (int i = 0; i < Modes.beast_feed_count; i++) {
        buf += sfmt(
            "%s{\"name\":\"%s\",\"host\":\"%s\",\"port\":%d,\"format\":\"%s\",\"enabled\":%s}",
            i ? "," : "",
            json_escape(esc, sizeof(esc), Modes.beast_feeds[i].name),
            Modes.beast_feeds[i].host ? Modes.beast_feeds[i].host : "",
            Modes.beast_feeds[i].port,
            Modes.beast_feeds[i].format == FEED_FORMAT_RAW ? "raw" : Modes.beast_feeds[i].format == FEED_FORMAT_SBS ? "sbs" : "beast",
            Modes.beast_feeds[i].enabled ? "true" : "false");
    }
    buf += "],\n";

    // FR24, PlaneFinder, RadarBox feeders removed in light version

    // OpenSky
    buf += sfmt(
        "\"opensky\":{\"enabled\":%s,\"username\":\"%s\","
        "\"serial\":%d,\"host\":\"%s\",\"port\":%d},\n",
        OpenSkyConfig.enabled ? "true" : "false",
        json_escape(esc, sizeof(esc), OpenSkyConfig.username),
        OpenSkyConfig.serial,
        json_escape(esc, sizeof(esc), OpenSkyConfig.host),
        OpenSkyConfig.port);

    // PiAware
    buf += sfmt(
        "\"piaware\":{\"enabled\":%s,\"feeder_id\":\"%s\","
        "\"state\":%d,\"msgs_sent\":%" PRIu64 "},\n",
        PiawareClient.enabled ? "true" : "false",
        json_escape(esc, sizeof(esc), PiawareClient.feeder_id),
        PiawareClient.state,
        (uint64_t)PiawareClient.msgs_sent);

    // Radiosonde SDR status (derived from SdrManager)
    {
        int sonde_active = 0;
        for (int i = 0; i < SdrManager.count; i++) {
            if (SdrManager.receivers[i].config.role == SDR_ROLE_RADIOSONDE) {
                sonde_active = 1;
                break;
            }
        }
        buf += sfmt(
            "\"sdr_radiosonde\":{\"enabled\":%s},\n",
            sonde_active ? "true" : "false");
    }

    // POCSAG SDR status (derived from SdrManager)
    {
        int pocsag_active = 0;
        double pocsag_freq = 466.150;
        for (int i = 0; i < SdrManager.count; i++) {
            if (SdrManager.receivers[i].config.role == SDR_ROLE_POCSAG) {
                pocsag_active = 1;
                pocsag_freq = SdrManager.receivers[i].config.freq / 1e6;
                break;
            }
        }
        buf += sfmt(
            "\"sdr_pocsag\":{\"active\":%s,\"enabled\":%s,\"freq_mhz\":\"%.3f\"},\n",
            pocsag_active ? "true" : "false",
            PocsagOutputEnabled ? "true" : "false",
            pocsag_freq);
    }

    // GSM SDR status (derived from SdrManager)
    {
        int gsm_active = 0;
        double gsm_freq = 935.200;
        for (int i = 0; i < SdrManager.count; i++) {
            if (SdrManager.receivers[i].config.role == SDR_ROLE_GSM) {
                gsm_active = 1;
                gsm_freq = SdrManager.receivers[i].config.freq / 1e6;
                break;
            }
        }
        buf += sfmt(
            "\"sdr_gsm\":{\"active\":%s,\"enabled\":%s,\"freq_mhz\":\"%.3f\",\"cells\":%d},\n",
            gsm_active ? "true" : "false",
            GsmOutputEnabled ? "true" : "false",
            gsm_freq,
            gsmTrackerActiveCount());
    }

    // LTE SDR status
    {
        int lte_active = 0;
        double lte_freq = 806.0;
        for (int i = 0; i < SdrManager.count; i++) {
            if (SdrManager.receivers[i].config.role == SDR_ROLE_LTE) {
                lte_active = 1;
                lte_freq = SdrManager.receivers[i].config.freq / 1e6;
                break;
            }
        }
        buf += sfmt(
            "\"sdr_lte\":{\"active\":%s,\"enabled\":%s,\"freq_mhz\":\"%.3f\",\"hop\":true,\"cells\":%d},\n",
            lte_active ? "true" : "false",
            LteOutputEnabled ? "true" : "false",
            lte_freq,
            lteTrackerCount());
    }

    // IoT 868 MHz SDR status
    {
        int iot_active = 0;
        double iot_freq = 868.300;
        for (int i = 0; i < SdrManager.count; i++) {
            if (SdrManager.receivers[i].config.role == SDR_ROLE_IOT868) {
                iot_active = 1;
                iot_freq = SdrManager.receivers[i].config.freq / 1e6;
                break;
            }
        }
        buf += sfmt(
            "\"sdr_iot868\":{\"active\":%s,\"enabled\":%s,\"freq_mhz\":\"%.3f\",\"devices\":%d},\n",
            iot_active ? "true" : "false",
            IotOutputEnabled ? "true" : "false",
            iot_freq,
            iotTrackerActiveCount());
    }

    // FANET SDR status
    {
        int fanet_active = 0;
        double fanet_freq = 868.200;
        for (int i = 0; i < SdrManager.count; i++) {
            if (SdrManager.receivers[i].config.role == SDR_ROLE_FANET) {
                fanet_active = 1;
                fanet_freq = SdrManager.receivers[i].config.freq / 1e6;
                break;
            }
        }
        buf += sfmt(
            "\"sdr_fanet\":{\"active\":%s,\"enabled\":%s,\"freq_mhz\":\"%.3f\"},\n",
            fanet_active ? "true" : "false",
            FanetOutputEnabled ? "true" : "false",
            fanet_freq);
    }

    // Sarsat SDR status
    {
        int sarsat_active = 0;
        double sarsat_freq = 406.040;
        for (int i = 0; i < SdrManager.count; i++) {
            if (SdrManager.receivers[i].config.role == SDR_ROLE_SARSAT) {
                sarsat_active = 1;
                sarsat_freq = SdrManager.receivers[i].config.freq / 1e6;
                break;
            }
        }
        buf += sfmt(
            "\"sdr_sarsat\":{\"active\":%s,\"enabled\":%s,\"freq_mhz\":\"%.3f\"},\n",
            sarsat_active ? "true" : "false",
            SarsatOutputEnabled ? "true" : "false",
            sarsat_freq);
    }

    // Sonde feeds
    buf += sfmt(
        "\"sondehub\":{\"enabled\":%s,\"callsign\":\"%s\"},\n",
        SondehubConfig.enabled ? "true" : "false",
        json_escape(esc, sizeof(esc), SondehubConfig.callsign));
    buf += sfmt(
        "\"radiosondy\":{\"enabled\":%s},\n",
        RadiosondyEnabled ? "true" : "false");
    buf += sfmt(
        "\"wettersonde\":{\"enabled\":%s},\n",
        WettersondeEnabled ? "true" : "false");

    // MLAT
    buf += "\"mlat\":{\"servers\":[\n";
    for (int i = 0; i < MlatConfig.server_count; i++) {
        buf += sfmt(
            "%s{\"host\":\"%s\",\"port\":%d,\"state\":%d}",
            i ? "," : "",
            MlatConfig.servers[i].host ? json_escape(esc, sizeof(esc), MlatConfig.servers[i].host) : "",
            MlatConfig.servers[i].port,
            (int)MlatConfig.servers[i].state);
    }
    buf += sfmt(
        "],\"alt\":%.0f,\"return_results\":%s},\n",
        MlatConfig.alt,
        MlatConfig.return_results ? "true" : "false");

    // OGN
    buf += sfmt(
        "\"ogn\":{\"server\":\"%s\",\"port\":%d},\n",
        json_escape(esc, sizeof(esc), FlarmConfig.ogn_server),
        FlarmConfig.ogn_port);

    // ADSBHub
    buf += sfmt(
        "\"adsbhub\":{\"ckey\":\"%s\"},\n",
        Modes.adsbhub_ckey ? json_escape(esc, sizeof(esc), Modes.adsbhub_ckey) : "");

    // Keys moved to /api/decoders

    // Network ports
    buf += sfmt(
        "\"network\":{\"raw_out\":\"%s\",\"raw_in\":\"%s\","
        "\"sbs_out\":\"%s\",\"beast_in\":\"%s\",\"beast_out\":\"%s\","
        "\"json_dir\":\"%s\",\"json_interval\":%" PRIu64 "},\n",
        Modes.net_output_raw_ports ? Modes.net_output_raw_ports : "",
        Modes.net_input_raw_ports ? Modes.net_input_raw_ports : "",
        Modes.net_output_sbs_ports ? Modes.net_output_sbs_ports : "",
        Modes.net_input_beast_ports ? Modes.net_input_beast_ports : "",
        Modes.net_output_beast_ports ? Modes.net_output_beast_ports : "",
        Modes.json_dir ? json_escape(esc, sizeof(esc), Modes.json_dir) : "",
        (uint64_t)Modes.json_interval);

    // Airframes.io ACARS/VDL2 feeds
    {
        int acars_active = 0, vdl2_active = 0;
        for (int i = 0; i < SdrManager.count; i++) {
            if (SdrManager.receivers[i].config.role == SDR_ROLE_ACARS)
                acars_active = 1;
            if (SdrManager.receivers[i].config.role == SDR_ROLE_VDL2)
                vdl2_active = 1;
        }
        buf += sfmt(
            "\"airframes\":{\"station_id\":\"%s\","
            "\"acars\":{\"enabled\":%s,\"active\":%s,\"host\":\"%s\",\"port\":%d},"
            "\"vdl2\":{\"enabled\":%s,\"active\":%s,\"host\":\"%s\",\"port\":%d}},\n",
            json_escape(esc, sizeof(esc), Modes.airframes_station_id),
            Modes.airframes_acars_feed.enabled ? "true" : "false",
            acars_active ? "true" : "false",
            Modes.airframes_acars_feed.host ? json_escape(esc, sizeof(esc), Modes.airframes_acars_feed.host) : "feed.acars.io",
            Modes.airframes_acars_feed.port,
            Modes.airframes_vdl2_feed.enabled ? "true" : "false",
            vdl2_active ? "true" : "false",
            Modes.airframes_vdl2_feed.host ? json_escape(esc, sizeof(esc), Modes.airframes_vdl2_feed.host) : "feed.acars.io",
            Modes.airframes_vdl2_feed.port);
    }

    // Panel
    buf += sfmt(
        "\"panel\":{\"port\":%d,\"has_password\":%s}\n",
        PanelState.port,
        PanelState.password[0] ? "true" : "false");

    buf += "}\n";

    http_send_json(fd, buf.c_str(), (int)buf.size());
}

// ============================= API: GET /api/status ======================

static void api_get_status(int fd)
{
    std::string buf;
    buf.reserve(4096);
    char esc[256], esc2[256];

    buf += "{\"feeders\":[\n";

    // Beast feeds
    for (int i = 0; i < Modes.beast_feed_count; i++) {
        buf += sfmt(
            "%s{\"name\":\"%s\",\"type\":\"beast\",\"enabled\":%s,"
            "\"host\":\"%s\",\"port\":%d}",
            (i > 0 || 0) ? "," : "",
            json_escape(esc, sizeof(esc), Modes.beast_feeds[i].name),
            Modes.beast_feeds[i].enabled ? "true" : "false",
            Modes.beast_feeds[i].host ? json_escape(esc2, sizeof(esc2), Modes.beast_feeds[i].host) : "",
            Modes.beast_feeds[i].port);
    }

    int need_comma = (Modes.beast_feed_count > 0);

    // FR24, PlaneFinder, RadarBox feeders removed in light version

    // OpenSky — always show with enabled/disabled state
    {
        buf += sfmt(
            "%s{\"name\":\"OpenSky Network\",\"type\":\"native\",\"enabled\":%s,"
            "\"host\":\"%s\",\"port\":%d,\"user\":\"%s\","
            "\"link\":\"https://opensky-network.org/my-opensky/sensors/view-sensors\"}",
            need_comma ? "," : "",
            OpenSkyConfig.enabled ? "true" : "false",
            json_escape(esc, sizeof(esc), OpenSkyConfig.host), OpenSkyConfig.port,
            json_escape(esc2, sizeof(esc2), OpenSkyConfig.username));
        need_comma = 1;
    }

    // PiAware — always show with enabled/disabled state
    {
        const char *pa_states[] = {"disconnected","connecting","tls_handshake","awaiting_login","logged_in"};
        int si = PiawareClient.state;
        if (si < 0 || si > 4) si = 0;
        buf += sfmt(
            "%s{\"name\":\"PiAware / FlightAware\",\"type\":\"native\",\"enabled\":%s,"
            "\"state\":\"%s\",\"feeder_id\":\"%s\",\"msgs\":%" PRIu64 ","
            "\"link\":\"https://flightaware.com/adsb/stats\"}",
            need_comma ? "," : "",
            PiawareClient.enabled ? "true" : "false",
            pa_states[si], json_escape(esc, sizeof(esc), PiawareClient.feeder_id),
            (uint64_t)PiawareClient.msgs_sent);
        need_comma = 1;
    }

    // MLAT servers
    for (int i = 0; i < MlatConfig.server_count; i++) {
        const char *ml_states[] = {"disconnected","connecting","handshaking","ready"};
        int mi = (int)MlatConfig.servers[i].state;
        if (mi < 0 || mi > 3) mi = 0;
        buf += sfmt(
            "%s{\"name\":\"MLAT:%s\",\"type\":\"mlat\",\"enabled\":true,"
            "\"host\":\"%s\",\"port\":%d,\"state\":\"%s\"}",
            need_comma ? "," : "",
            MlatConfig.servers[i].host ? json_escape(esc, sizeof(esc), MlatConfig.servers[i].host) : "?",
            MlatConfig.servers[i].host ? json_escape(esc2, sizeof(esc2), MlatConfig.servers[i].host) : "",
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
        buf += sfmt(
            "%s{\"name\":\"FLARM / OGNTP / ADS-L / P3I\",\"type\":\"flarm\",\"enabled\":%s,"
            "\"station\":\"%s\",\"server\":\"%s\","
            "\"link\":\"http://live.glidernet.org\"}",
            need_comma ? "," : "",
            flarm_running ? "true" : "false",
            json_escape(esc, sizeof(esc), FlarmConfig.ogn_station),
            json_escape(esc2, sizeof(esc2), FlarmConfig.ogn_server));
        need_comma = 1;
    }

    // Airframes.io ACARS feed
    {
        int acars_active = 0;
        for (int i = 0; i < SdrManager.count; i++) {
            if (SdrManager.receivers[i].config.role == SDR_ROLE_ACARS) { acars_active = 1; break; }
        }
        buf += sfmt(
            "%s{\"name\":\"Airframes ACARS\",\"type\":\"airframes\",\"enabled\":%s,"
            "\"host\":\"%s\",\"port\":%d,\"decoder_active\":%s,"
            "\"link\":\"https://app.airframes.io\"}",
            need_comma ? "," : "",
            Modes.airframes_acars_feed.enabled ? "true" : "false",
            Modes.airframes_acars_feed.host ? json_escape(esc, sizeof(esc), Modes.airframes_acars_feed.host) : "",
            Modes.airframes_acars_feed.port,
            acars_active ? "true" : "false");
        need_comma = 1;
    }

    // Airframes.io VDL2 feed
    {
        int vdl2_active = 0;
        for (int i = 0; i < SdrManager.count; i++) {
            if (SdrManager.receivers[i].config.role == SDR_ROLE_VDL2) { vdl2_active = 1; break; }
        }
        buf += sfmt(
            "%s{\"name\":\"Airframes VDL2\",\"type\":\"airframes\",\"enabled\":%s,"
            "\"host\":\"%s\",\"port\":%d,\"decoder_active\":%s,"
            "\"link\":\"https://app.airframes.io\"}",
            need_comma ? "," : "",
            Modes.airframes_vdl2_feed.enabled ? "true" : "false",
            Modes.airframes_vdl2_feed.host ? json_escape(esc, sizeof(esc), Modes.airframes_vdl2_feed.host) : "",
            Modes.airframes_vdl2_feed.port,
            vdl2_active ? "true" : "false");
        need_comma = 1;
    }

    buf += sfmt(
        "],\"version\":\"" MODES_DUMP1090_VERSION "\",\"variant\":\"" MODES_DUMP1090_VARIANT "\"}\n");

    http_send_json(fd, buf.c_str(), (int)buf.size());
}

// ============================= API: GET /api/aircraft =====================

static void api_get_aircraft(int fd)
{
    // Read aircraft.json from disk (already generated by dump1090)
    if (!Modes.json_dir) {
        http_send(fd, 404, "text/plain", "No JSON dir", 11);
        return;
    }

    std::string path = std::string(Modes.json_dir) + "/aircraft.json";

    FILE *f = fopen(path.c_str(), "r");
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

    std::string data((size_t)fsize, '\0');
    size_t nread = fread(data.data(), 1, (size_t)fsize, f);
    fclose(f);
    data.resize(nread);

    http_send_json(fd, data.c_str(), (int)data.size());
}

// ============================= API: GET /api/gsm =========================

static void api_get_gsm(int fd)
{
    std::string json = gsmTrackerToJSON();
    http_send_json(fd, json.c_str(), (int)json.size());
}

// ============================= API: GET /api/lte =========================

static void api_get_lte(int fd)
{
    std::string json = lteTrackerToJSON();
    http_send_json(fd, json.c_str(), (int)json.size());
}

// ============================= API: GET /api/iot868 =======================

static void api_get_iot868(int fd)
{
    std::string json = iotTrackerToJSON();
    http_send_json(fd, json.c_str(), (int)json.size());
}

// ============================= API: GET /api/fanet ========================

// Callback for ground track serialization
struct fanet_ground_json_ctx {
    std::string *buf;
    int count;
};

static void fanet_ground_json_cb(const fanet_ground_entry_t *e, void *ctx)
{
    struct fanet_ground_json_ctx *c = (struct fanet_ground_json_ctx *)ctx;
    uint64_t now = mstime();
    int age_sec = (int)((now - e->last_seen) / 1000);
    char esc_name[64];
    *c->buf += sfmt(
        "%s{\"addr\":\"%06X\",\"lat\":%.5f,\"lon\":%.5f,\"type\":%u,\"name\":\"%s\",\"age\":%d}",
        c->count ? "," : "", e->addr, e->latitude, e->longitude,
        (uint32_t)e->ground_type, json_escape(esc_name, sizeof(esc_name), e->name), age_sec);
    c->count++;
}

static void api_get_fanet(int fd)
{
    std::string buf;
    buf.reserve(4096);
    fanet_stats_t stats = {0};

    // Find the FANET receiver and get stats
    for (int i = 0; i < SdrManager.count; i++) {
        if (SdrManager.receivers[i].config.role == SDR_ROLE_FANET &&
            SdrManager.receivers[i].decoder_state) {
            fanet_get_stats((const fanet_state_t *)SdrManager.receivers[i].decoder_state, &stats);
            break;
        }
    }

    buf += sfmt(
        "{\"samples_processed\":%" PRIu64 ","
        "\"preambles_detected\":%" PRIu64 ","
        "\"sync_word_ok\":%" PRIu64 ","
        "\"header_errors\":%" PRIu64 ","
        "\"packets_decoded\":%" PRIu64 ","
        "\"crc_errors\":%" PRIu64 ","
        "\"type_counts\":{"
        "\"ack\":%" PRIu64 ",\"tracking\":%" PRIu64 ",\"name\":%" PRIu64 ",\"message\":%" PRIu64 ","
        "\"service\":%" PRIu64 ",\"landmark\":%" PRIu64 ",\"remote\":%" PRIu64 ",\"ground\":%" PRIu64 ","
        "\"hwinfo\":%" PRIu64 ",\"thermal\":%" PRIu64 ",\"hwinfo2\":%" PRIu64 "}",
        (uint64_t)stats.samples_processed,
        (uint64_t)stats.preambles_detected,
        (uint64_t)stats.sync_word_ok,
        (uint64_t)stats.header_errors,
        (uint64_t)stats.packets_decoded,
        (uint64_t)stats.crc_errors,
        (uint64_t)stats.type_counts[0],
        (uint64_t)stats.type_counts[1],
        (uint64_t)stats.type_counts[2],
        (uint64_t)stats.type_counts[3],
        (uint64_t)stats.type_counts[4],
        (uint64_t)stats.type_counts[5],
        (uint64_t)stats.type_counts[6],
        (uint64_t)stats.type_counts[7],
        (uint64_t)stats.type_counts[8],
        (uint64_t)stats.type_counts[9],
        (uint64_t)stats.type_counts[10]);

    // Append ground tracking entries
    buf += ",\"ground_tracks\":[";
    struct fanet_ground_json_ctx ctx = { &buf, 0 };
    fanetGetGroundTracks(fanet_ground_json_cb, &ctx);
    buf += "]}";

    http_send_json(fd, buf.c_str(), (int)buf.size());
}

// ============================= API: GET /api/connections ==================

static void api_get_connections(int fd)
{
    std::string buf;
    buf.reserve(4096);

    buf += "{\"services\":[\n";

    int first_svc = 1;
    struct net_service *svc;
    for (svc = Modes.services; svc; svc = svc->next) {
        if (!first_svc) buf += ',';
        first_svc = 0;

        char esc[256];
        buf += sfmt(
            "{\"descr\":\"%s\",\"connections\":%d,\"listeners\":[",
            json_escape(esc, sizeof(esc), svc->descr ? svc->descr : "?"),
            svc->connections);

        // Emit listener ports
        for (int i = 0; i < svc->listener_count; i++) {
            struct sockaddr_in sa;
            socklen_t slen = sizeof(sa);
            if (getsockname(svc->listener_fds[i], (struct sockaddr *)&sa, &slen) == 0) {
                buf += sfmt("%s%d", i ? "," : "", ntohs(sa.sin_port));
            }
        }
        buf += "],\"clients\":[";

        // Walk client list and find clients belonging to this service
        int first_cli = 1;
        struct client *c;
        for (c = Modes.clients; c; c = c->next) {
            if (c->service != svc) continue;

            struct sockaddr_in peer;
            socklen_t plen = sizeof(peer);
            char addr[64] = "?";
            if (getpeername(c->fd, (struct sockaddr *)&peer, &plen) == 0) {
                snprintf(addr, sizeof(addr), "%s:%d",
                    inet_ntoa(peer.sin_addr), ntohs(peer.sin_port));
            }

            // Build flags string
            std::string flags;
            if (c->modeac_requested) flags += "AC ";
            if (c->verbatim_requested) flags += "V ";
            if (c->local_requested) flags += "L ";

            if (!first_cli) buf += ',';
            first_cli = 0;
            buf += sfmt(
                "{\"addr\":\"%s\",\"fd\":%d,\"flags\":\"%s\"}",
                addr, c->fd, flags.c_str());
        }

        buf += "]}";
    }

    buf += "]}\n";

    http_send_json(fd, buf.c_str(), (int)buf.size());
}

// ============================= API: GET /api/stats =======================

static void api_get_stats(int fd)
{
    if (!Modes.json_dir) {
        http_send(fd, 404, "text/plain", "No JSON dir", 11);
        return;
    }

    std::string path = std::string(Modes.json_dir) + "/stats.json";

    FILE *f = fopen(path.c_str(), "r");
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

    std::string data((size_t)fsize, '\0');
    size_t nread = fread(data.data(), 1, (size_t)fsize, f);
    fclose(f);
    data.resize(nread);

    http_send_json(fd, data.c_str(), (int)data.size());
}

// ============================= API: GET /api/logs ========================

static void api_get_logs(int fd)
{
    std::string buf;
    buf.reserve(4096);
    char esc[512];

    pthread_mutex_lock(&PanelState.log_mutex);

    buf += sfmt("{\"seq\":%d,\"lines\":[\n", PanelState.log_seq);

    // Return last N lines (max 200 per request)
    int count = PanelState.log_count;
    int start_offset = 0;
    if (count > 200) {
        start_offset = count - 200;
        count = 200;
    }

    for (int i = 0; i < count; i++) {
        int idx = (PanelState.log_head + start_offset + i) % PANEL_LOG_LINES;
        buf += sfmt("%s\"%s\"",
                    i ? "," : "",
                    json_escape(esc, sizeof(esc), PanelState.log_buf[idx]));
    }

    pthread_mutex_unlock(&PanelState.log_mutex);

    buf += "]}\n";

    http_send_json(fd, buf.c_str(), (int)buf.size());
}

// ============================= API: GET /api/messages =====================

static void api_get_messages(int fd)
{
    std::string buf;
    buf.reserve(8192);
    char esc[512];

    pthread_mutex_lock(&PanelState.msg_mutex);

    buf += sfmt("{\"seq\":%d,\"messages\":[\n", PanelState.msg_seq);

    int count = PanelState.msg_count;
    int start_offset = 0;
    if (count > 2000) {
        start_offset = count - 2000;
        count = 2000;
    }

    for (int i = 0; i < count; i++) {
        int idx = (PanelState.msg_head + start_offset + i) % PANEL_MSG_LINES;
        buf += sfmt("%s\"%s\"",
                    i ? "," : "",
                    json_escape(esc, sizeof(esc), PanelState.msg_buf[idx]));
    }

    pthread_mutex_unlock(&PanelState.msg_mutex);

    buf += "]}\n";

    http_send_json(fd, buf.c_str(), (int)buf.size());
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
                uint8_t c = (uint8_t)body[i];
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
        uint8_t c = (uint8_t)filename[i];
        if (!(c >= 'a' && c <= 'z') && !(c >= 'A' && c <= 'Z')
            && !(c >= '0' && c <= '9') && c != '.' && c != '-'
            && c != '_' && c != '/') {
            http_send(fd, 404, "text/plain", "Not found", 9);
            return;
        }
    }

    std::string path = std::string(PanelState.html_dir) + "/" + filename;

    // Resolve real path and verify it's under html_dir
    char resolved[PATH_MAX];
    char resolved_base[PATH_MAX];
    if (realpath(path.c_str(), resolved) && realpath(PanelState.html_dir, resolved_base)) {
        if (strncmp(resolved, resolved_base, strlen(resolved_base)) != 0) {
            http_send(fd, 404, "text/plain", "Not found", 9);
            panelLog("Panel: blocked path traversal attempt: %s", filename);
            return;
        }
    }
    // If realpath fails (dir missing), fall through to fopen which will trigger fallback

    FILE *f = fopen(path.c_str(), "r");
    if (!f) {
        // Serve embedded fallback
        std::string buf = sfmt(
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
            "</body></html>", path.c_str(), PanelState.html_dir);
        http_send(fd, 200, "text/html; charset=utf-8", buf.c_str(), (int)buf.size());
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

    char *data = (char*)malloc((size_t)fsize);
    if (!data) { fclose(f); http_send(fd, 500, "text/plain", "OOM", 3); return; }

    size_t nread = fread(data, 1, (size_t)fsize, f);
    fclose(f);

    // Determine content type
    const char *ct = "text/plain";
    std::string_view fn(filename);
    if (fn.find(".html") != std::string_view::npos) ct = "text/html; charset=utf-8";
    else if (fn.find(".js") != std::string_view::npos) ct = "application/javascript";
    else if (fn.find(".css") != std::string_view::npos) ct = "text/css";
    else if (fn.find(".json") != std::string_view::npos) ct = "application/json";
    else if (fn.find(".png") != std::string_view::npos) ct = "image/png";
    else if (fn.find(".svg") != std::string_view::npos) ct = "image/svg+xml";

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

// Probe all SDR devices and cache their tuner types.
// Must be called before any device is opened (so open succeeds for all).
void panelProbeAllTuners(void)
{
    sdr_dev_info_t devs[MAX_SDR_RECEIVERS];
    int count = sdrBackendEnumerateAll(devs, MAX_SDR_RECEIVERS);
    if (count <= 0) return;

    // Open all devices via backend to read tuner type, then close
    for (int i = 0; i < count; i++) {
        const sdr_backend_ops_t *ops = sdrBackendResolve(devs[i].backend);
        if (!ops) continue;
        sdr_device_t *dev = ops->open_by_serial(devs[i].serial);
        if (!dev) continue;
        dev->ops = ops;

        int t = sdr_get_tuner_type(dev);
        tuner_cache_store(devs[i].serial, t);
        fprintf(stderr, "Panel: device #%d SN=%s tuner=%s (%d) via %s\n",
                i, devs[i].serial, tuner_name_sdr((sdr_tuner_type_t)t), t, ops->name);

        ops->close(dev);
    }
}

// Tuner type to name + frequency range (SDR_TUNER enum)
static const char *tuner_name_sdr(sdr_tuner_type_t type) {
    switch(type) {
        case SDR_TUNER_E4000:  return "E4000";
        case SDR_TUNER_FC0012: return "FC0012";
        case SDR_TUNER_FC0013: return "FC0013";
        case SDR_TUNER_FC2580: return "FC2580";
        case SDR_TUNER_R820T:  return "R820T";
        case SDR_TUNER_R820T2: return "R828D";
        default:               return "unknown";
    }
}
static const char *tuner_freq_range_sdr(sdr_tuner_type_t type) {
    switch(type) {
        case SDR_TUNER_E4000:  return "52-2200 MHz (with gap 1100-1250)";
        case SDR_TUNER_FC0012: return "22-948 MHz";
        case SDR_TUNER_FC0013: return "22-1100 MHz";
        case SDR_TUNER_FC2580: return "146-308, 438-924 MHz";
        case SDR_TUNER_R820T:  return "24-1766 MHz";
        case SDR_TUNER_R820T2: return "24-1766 MHz";
        default:               return "unknown";
    }
}

static void api_get_receivers(int fd)
{
    std::string buf;
    buf.reserve(4096);

    buf += "{\"receivers\":[";

    for (int i = 0; i < SdrManager.count; i++) {
        sdr_receiver_t *rx = &SdrManager.receivers[i];
        if (i > 0) buf += ',';

        // Resolve tuner name: prefer tuner_cache (SDR_TUNER enum from probe), fallback to backend_dev
        const char *tname = "unknown", *trange = "unknown";
        int cached = tuner_cache_lookup(rx->serial_actual[0] ? rx->serial_actual : rx->config.serial);
        if (cached >= 0) {
            tname = tuner_name_sdr((sdr_tuner_type_t)cached);
            trange = tuner_freq_range_sdr((sdr_tuner_type_t)cached);
        } else if (rx->backend_dev && rx->backend_dev->tuner_type != SDR_TUNER_UNKNOWN) {
            tname = tuner_name_sdr(rx->backend_dev->tuner_type);
            trange = tuner_freq_range_sdr(rx->backend_dev->tuner_type);
        }

        buf += sfmt(
            "{\"id\":%d,\"serial\":\"%.63s\",\"serial_actual\":\"%.63s\","
            "\"role\":\"%s\",\"state\":\"%s\","
            "\"freq\":%d,\"gain\":%.1f,\"ppm\":%d,"
            "\"manufacturer\":\"%.63s\",\"product\":\"%.63s\","
            "\"tuner\":\"%s\",\"freq_range\":\"%s\","
            "\"dev_index\":%d,\"gain_list\":[",
            rx->id, rx->config.serial, rx->serial_actual,
            sdrRoleName(rx->config.role), rxStateName(rx->state),
            rx->config.freq, rx->config.gain, rx->config.ppm_error,
            rx->manufacturer, rx->product,
            tname, trange,
            rx->dev_index);

        // Emit supported gain values in dB
        if (rx->rtl.gains && rx->rtl.gain_steps > 0) {
            for (int g = 0; g < rx->rtl.gain_steps; g++) {
                if (g > 0) buf += ',';
                buf += sfmt("%.1f", rx->rtl.gains[g] / 10.0);
            }
        }
        const char *be_name = (rx->backend_ops && rx->backend_ops->name)
                              ? rx->backend_ops->name : sdrBackendName(rx->config.backend);
        buf += sfmt("],\"backend\":\"%s\"}", be_name);
    }

    buf += sfmt("],\"count\":%d,\"max\":%d,\"backends_available\":%d}",
                SdrManager.count, MAX_SDR_RECEIVERS, sdrBackendAvailable());

    http_send_json(fd, buf.c_str(), (int)buf.size());
}

static void rx_set_freq_for_role(rx_config_t *cfg)
{
    switch (cfg->role) {
        case SDR_ROLE_ADSB:       cfg->freq = 1090000000; cfg->sample_rate = 2400000; break;
        case SDR_ROLE_FLARM:      cfg->freq = 868300000;  cfg->sample_rate = 1600000; break;
        case SDR_ROLE_ACARS:      cfg->freq = 131550000;  cfg->sample_rate = 2400000; break;
        case SDR_ROLE_VDL2:       cfg->freq = 136975000;  cfg->sample_rate = 2400000; break;
        case SDR_ROLE_RADIOSONDE: cfg->freq = 403000000;  cfg->sample_rate = 2400000; break;
        case SDR_ROLE_POCSAG:     cfg->freq = 466150000;  cfg->sample_rate = POCSAG_SAMPLE_RATE; break;
        case SDR_ROLE_GSM:        cfg->freq = 947000000;  cfg->sample_rate = 1000000;  break;
        case SDR_ROLE_LTE:        cfg->freq = LTE_DEFAULT_FREQ; cfg->sample_rate = LTE_SAMPLE_RATE; break;
        case SDR_ROLE_IOT868:     cfg->freq = IOT_CENTER_FREQ;  cfg->sample_rate = IOT_SAMPLE_RATE; break;
        case SDR_ROLE_FANET:      cfg->freq = FANET_CENTER_FREQ; cfg->sample_rate = FANET_SAMPLE_RATE; break;
        case SDR_ROLE_SARSAT:     cfg->freq = SARSAT_CENTER_FREQ; cfg->sample_rate = SARSAT_SAMPLE_RATE; break;
        default:                  cfg->freq = 0;          cfg->sample_rate = 0;       break;
    }
}

// ============================= API: GET /api/stats/quick =================
// Returns a quick snapshot of demod counters + per-receiver IQ noise for auto-gain sweep.
static void api_get_stats_quick(int fd)
{
    // Sum alltime + current for monotonically-increasing totals
    uint32_t demod_total = 0;
    for (int i = 0; i <= MODES_MAX_BITERRORS; i++) {
        demod_total += Modes.stats_alltime.demod_accepted[i];
        demod_total += Modes.stats_current.demod_accepted[i];
    }
    uint32_t strong = Modes.stats_alltime.strong_signal_count
                    + Modes.stats_current.strong_signal_count;

    char buf[2048];
    char *p = buf;
    char *end = buf + sizeof(buf);
    p += snprintf(p, (size_t)(end - p),
        "{\"demod_total\":%u,\"strong_signals\":%u,\"rx_noise\":[", demod_total, strong);

    for (int i = 0; i < SdrManager.count && p < end - 128; i++) {
        sdr_receiver_t *rx = &SdrManager.receivers[i];
        uint64_t sum = __atomic_load_n(&rx->ag_iq_sum, __ATOMIC_RELAXED);
        uint64_t cnt = __atomic_load_n(&rx->ag_iq_count, __ATOMIC_RELAXED);
        p += snprintf(p, (size_t)(end - p),
            "%s{\"serial\":\"%s\",\"iq_sum\":%" PRIu64 ",\"iq_count\":%" PRIu64 "}",
            i ? "," : "", rx->serial_actual,
            (uint64_t)sum, (uint64_t)cnt);
    }

    p += snprintf(p, (size_t)(end - p), "]}");
    http_send_json(fd, buf, (int)(p - buf));
}

// ============================= API: POST /api/receivers/setgain ==========
// Fast gain change on a running receiver (no stop/restart).
// Body: {"serial":"00000101","step":15}
static void api_post_setgain(int fd, const char *body)
{
    char serial[64] = {0};
    int step = -1;

    const char *p;
    if ((p = strstr(body, "\"serial\"")) != NULL) {
        p = strchr(p + 8, '"'); if (p) { p++; const char *e = strchr(p, '"');
        if (e && (e - p) < 63) { memcpy(serial, p, e - p); serial[e - p] = '\0'; }
        }
    }
    if ((p = strstr(body, "\"step\"")) != NULL) {
        p = strchr(p + 6, ':'); if (p) step = atoi(p + 1);
    }

    if (!serial[0] || step < 0) {
        http_send(fd, 400, "application/json",
            "{\"ok\":false,\"error\":\"missing serial or step\"}", 45);
        return;
    }

    int idx = sdrManagerFindBySerial(serial);
    if (idx < 0) {
        http_send(fd, 404, "application/json",
            "{\"ok\":false,\"error\":\"receiver not found\"}", 40);
        return;
    }

    sdr_receiver_t *rx = &SdrManager.receivers[idx];
    if (rx->state != RX_STATE_RUNNING) {
        http_send(fd, 400, "application/json",
            "{\"ok\":false,\"error\":\"receiver not running\"}", 43);
        return;
    }
    if (!rx->rtl.gains || rx->rtl.gain_steps < 2) {
        http_send(fd, 400, "application/json",
            "{\"ok\":false,\"error\":\"no gain table\"}", 35);
        return;
    }

    int result = rxSetGain(rx, step);
    if (result < 0) {
        http_send(fd, 500, "application/json",
            "{\"ok\":false,\"error\":\"gain change failed\"}", 41);
        return;
    }

    float gain_db = rx->rtl.gains[result] / 10.0f;
    rx->config.gain = gain_db;

    char buf[256];
    int len = snprintf(buf, sizeof(buf),
        "{\"ok\":true,\"gain_db\":%.1f,\"step\":%d}", gain_db, result);
    http_send_json(fd, buf, len);
}

// ============================= Toggle (start/stop) a receiver ==============

static void api_post_receiver_toggle(int fd, const char *body)
{
    // Body: {"serial":"00000101","action":"start"} or {"action":"stop"}
    char serial[64] = {0};
    char action[8] = {0};
    const char *p;

    if ((p = strstr(body, "\"serial\"")) != NULL) {
        p = strchr(p + 8, '"'); if (p) { p++; const char *e = strchr(p, '"');
        if (e && (e - p) < 63) { memcpy(serial, p, e - p); serial[e - p] = '\0'; }
        }
    }
    if ((p = strstr(body, "\"action\"")) != NULL) {
        p = strchr(p + 8, '"'); if (p) { p++; const char *e = strchr(p, '"');
        if (e && (e - p) < 7) { memcpy(action, p, e - p); action[e - p] = '\0'; }
        }
    }

    if (!serial[0] || !action[0]) {
        http_send(fd, 400, "application/json",
            "{\"ok\":false,\"message\":\"missing serial or action\"}", 49);
        return;
    }

    int idx = sdrManagerFindBySerial(serial);
    if (idx < 0) {
        char resp[128];
        int rlen = snprintf(resp, sizeof(resp),
            "{\"ok\":false,\"message\":\"Receiver %s not found\"}", serial);
        http_send(fd, 404, "application/json", resp, rlen);
        return;
    }

    sdr_receiver_t *rx = &SdrManager.receivers[idx];
    char resp[256];
    int rlen;

    std::string_view action_sv(action);
    if (action_sv == "stop") {
        if (rx->state == RX_STATE_RUNNING) {
            rxStop(rx);
            rxClose(rx);
            rlen = snprintf(resp, sizeof(resp),
                "{\"ok\":true,\"message\":\"Receiver %s stopped\",\"state\":\"idle\"}", serial);
        } else {
            rlen = snprintf(resp, sizeof(resp),
                "{\"ok\":true,\"message\":\"Receiver %s already idle\",\"state\":\"%s\"}",
                serial, rxStateName(rx->state));
        }
        http_send(fd, 200, "application/json", resp, rlen);
    } else if (action_sv == "start") {
        if (rx->state == RX_STATE_RUNNING) {
            rlen = snprintf(resp, sizeof(resp),
                "{\"ok\":true,\"message\":\"Receiver %s already running\",\"state\":\"running\"}", serial);
            http_send(fd, 200, "application/json", resp, rlen);
            return;
        }
        // Need to open if idle
        bool ok = true;
        if (rx->state == RX_STATE_IDLE) {
            ok = rxOpen(rx);
        }
        if (ok && rx->state == RX_STATE_OPEN) {
            ok = rxStart(rx);
        }
        if (ok && rx->config.role == SDR_ROLE_FLARM) {
            FlarmConfig.enabled = 1;
            ognClientInit();
        }
        rlen = snprintf(resp, sizeof(resp),
            "{\"ok\":%s,\"message\":\"Receiver %s %s\",\"state\":\"%s\"}",
            ok ? "true" : "false", serial,
            ok ? "started" : "failed to start",
            rxStateName(rx->state));
        http_send(fd, 200, "application/json", resp, rlen);
    } else {
        http_send(fd, 400, "application/json",
            "{\"ok\":false,\"message\":\"action must be start or stop\"}", 53);
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
    sdr_backend_type_t backend = sdrBackendParse("");  // default: sdrgg if available
    if ((p = strstr(body, "\"backend\"")) != NULL) {
        p = strchr(p + 9, '"'); if (p) { p++; const char *e = strchr(p, '"');
        if (e) { char bstr[16] = {0}; int blen = (int)(e - p); if (blen > 15) blen = 15;
        memcpy(bstr, p, blen); backend = sdrBackendParse(bstr); }
        }
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
    else if (!strcasecmp(role_str, "pocsag")) role = SDR_ROLE_POCSAG;
    else if (!strcasecmp(role_str, "gsm")) role = SDR_ROLE_GSM;
    else if (!strcasecmp(role_str, "lte")) role = SDR_ROLE_LTE;
    else if (!strcasecmp(role_str, "iot868")) role = SDR_ROLE_IOT868;
    else if (!strcasecmp(role_str, "fanet")) role = SDR_ROLE_FANET;
    else if (!strcasecmp(role_str, "sarsat")) role = SDR_ROLE_SARSAT;

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
        // Already exists — reconfigure in-place (avoids device close/reopen
        // which crashes in libsdrgg when other devices are still streaming).
        sdr_receiver_t *rx = &SdrManager.receivers[idx];
        fprintf(stderr, "panel: reassigning rx[%d] %s -> %s\n",
                idx, sdrRoleName(rx->config.role), sdrRoleName(role));

        // Sync FlarmConfig before reconfigure
        if (rx->config.role == SDR_ROLE_FLARM && role != SDR_ROLE_FLARM)
            FlarmConfig.enabled = 0;

        // NULL decoder_ops under lock to prevent main thread drain race
        pthread_mutex_lock(&SdrManager.lock);
        rx->decoder_ops = NULL;
        pthread_mutex_unlock(&SdrManager.lock);

        // Compute target freq/sample_rate for new role
        rx_config_t tmp_cfg = rx->config;
        tmp_cfg.role = role;
        tmp_cfg.gain = gain;
        tmp_cfg.ppm_error = ppm;
        tmp_cfg.backend = backend;
        rx_set_freq_for_role(&tmp_cfg);

        bool ok;
        // If backend changed, must do full close+reopen (can't switch library in-place)
        if (backend != rx->config.backend) {
            fprintf(stderr, "panel: backend change %s -> %s, doing full close+reopen\n",
                    sdrBackendName(rx->config.backend), sdrBackendName(backend));
            rxStop(rx);
            rxClose(rx);
            rx->config = tmp_cfg;
            usleep(300000);  // let USB settle
            ok = rxOpen(rx);
            if (ok) ok = rxStart(rx);
        } else {
            // Same backend — reconfigure in-place (avoids USB close)
            ok = rxReconfigure(rx, role, gain, ppm,
                               tmp_cfg.freq, tmp_cfg.sample_rate);
            if (ok) ok = rxStart(rx);
        }
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
        cfg.backend = backend;
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

// ============================= API: GET /api/decoders =====================

static void api_get_decoders(int fd)
{
    std::string buf;
    buf.reserve(8192);
    char esc[256], esc2[256], esc3[256];

    buf += "{\n";

    // ADS-B decoder config — full adsb_decoder_config_t
    buf += sfmt(
        "\"adsb\":{"
        "\"fix_crc\":%d,\"check_crc\":%s,\"fix_df\":%s,"
        "\"enable_df24\":%s,\"mode_ac\":%s,\"mode_ac_auto\":%s,"
        "\"crc_rescue\":%s,\"use_gnss\":%s,\"mlat\":%s,"
        "\"adaptive_range\":%s,\"adaptive_burst\":%s,"
        "\"adaptive_min_gain\":%.1f,\"adaptive_max_gain\":%.1f,"
        "\"adaptive_duty_cycle\":%.2f,"
        "\"adaptive_burst_alpha\":%.4f,\"adaptive_burst_change_delay\":%u,"
        "\"adaptive_burst_loud_rate\":%.4f,\"adaptive_burst_loud_runlength\":%u,"
        "\"adaptive_burst_quiet_rate\":%.4f,\"adaptive_burst_quiet_runlength\":%u,"
        "\"adaptive_range_alpha\":%.4f,\"adaptive_range_percentile\":%u,"
        "\"adaptive_range_target\":%.2f,\"adaptive_range_change_delay\":%u,"
        "\"adaptive_range_scan_delay\":%u,\"adaptive_range_rescan_delay\":%u},\n",
        DecoderConfigs.adsb.fix_crc,
        DecoderConfigs.adsb.check_crc ? "true" : "false",
        DecoderConfigs.adsb.fix_df ? "true" : "false",
        DecoderConfigs.adsb.enable_df24 ? "true" : "false",
        DecoderConfigs.adsb.mode_ac ? "true" : "false",
        DecoderConfigs.adsb.mode_ac_auto ? "true" : "false",
        DecoderConfigs.adsb.crc_rescue ? "true" : "false",
        DecoderConfigs.adsb.use_gnss ? "true" : "false",
        DecoderConfigs.adsb.mlat ? "true" : "false",
        DecoderConfigs.adsb.adaptive_range ? "true" : "false",
        DecoderConfigs.adsb.adaptive_burst ? "true" : "false",
        DecoderConfigs.adsb.adaptive_min_gain,
        DecoderConfigs.adsb.adaptive_max_gain,
        DecoderConfigs.adsb.adaptive_duty_cycle,
        DecoderConfigs.adsb.adaptive_burst_alpha,
        DecoderConfigs.adsb.adaptive_burst_change_delay,
        DecoderConfigs.adsb.adaptive_burst_loud_rate,
        DecoderConfigs.adsb.adaptive_burst_loud_runlength,
        DecoderConfigs.adsb.adaptive_burst_quiet_rate,
        DecoderConfigs.adsb.adaptive_burst_quiet_runlength,
        DecoderConfigs.adsb.adaptive_range_alpha,
        DecoderConfigs.adsb.adaptive_range_percentile,
        DecoderConfigs.adsb.adaptive_range_target,
        DecoderConfigs.adsb.adaptive_range_change_delay,
        DecoderConfigs.adsb.adaptive_range_scan_delay,
        DecoderConfigs.adsb.adaptive_range_rescan_delay);

    // FLARM decoder config (includes keys)
    {
        char kt[256] = "";
        char k5s[64] = "";
        if (DecoderConfigs.flarm.keys_loaded) {
            snprintf(kt, sizeof(kt),
                "%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x",
                DecoderConfigs.flarm.key_table[0], DecoderConfigs.flarm.key_table[1],
                DecoderConfigs.flarm.key_table[2], DecoderConfigs.flarm.key_table[3],
                DecoderConfigs.flarm.key_table[4], DecoderConfigs.flarm.key_table[5],
                DecoderConfigs.flarm.key_table[6], DecoderConfigs.flarm.key_table[7],
                DecoderConfigs.flarm.key_table[8], DecoderConfigs.flarm.key_table[9],
                DecoderConfigs.flarm.key_table[10], DecoderConfigs.flarm.key_table[11]);
            snprintf(k5s, sizeof(k5s), "%08x,%08x,%08x,%08x",
                DecoderConfigs.flarm.key5[0], DecoderConfigs.flarm.key5[1],
                DecoderConfigs.flarm.key5[2], DecoderConfigs.flarm.key5[3]);
        }
        buf += sfmt(
            "\"flarm\":{\"enabled\":%s,\"ogn_only\":%s,"
            "\"ogn_server\":\"%s\",\"ogn_port\":%d,\"ogn_station\":\"%s\","
            "\"keys_file\":\"%s\",\"keys_loaded\":%s,\"key_table\":\"%s\","
            "\"key2\":\"%08x\",\"key3\":\"%08x\",\"key4\":\"%08x\","
            "\"key5\":\"%s\"},\n",
            DecoderConfigs.flarm.enabled ? "true" : "false",
            DecoderConfigs.flarm.ogn_only ? "true" : "false",
            json_escape(esc, sizeof(esc), DecoderConfigs.flarm.ogn_server),
            DecoderConfigs.flarm.ogn_port,
            json_escape(esc2, sizeof(esc2), DecoderConfigs.flarm.ogn_station),
            json_escape(esc3, sizeof(esc3), DecoderConfigs.flarm.keys_file),
            DecoderConfigs.flarm.keys_loaded ? "true" : "false",
            kt,
            DecoderConfigs.flarm.key2, DecoderConfigs.flarm.key3, DecoderConfigs.flarm.key4,
            k5s);
    }

    // ACARS decoder config
    buf += sfmt("\"acars\":{\"enabled\":%s,\"center_freq\":%.0f,\"channels\":[",
        DecoderConfigs.acars.enabled ? "true" : "false", DecoderConfigs.acars.center_freq);
    for (int i = 0; i < DecoderConfigs.acars.num_channels; i++)
        buf += sfmt("%s%.0f", i ? "," : "", DecoderConfigs.acars.channel_freqs[i]);
    buf += "]},\n";

    // VDL2 decoder config
    buf += sfmt("\"vdl2\":{\"enabled\":%s,\"center_freq\":%.0f,\"squelch_level\":%.1f,\"channels\":[",
        DecoderConfigs.vdl2.enabled ? "true" : "false", DecoderConfigs.vdl2.center_freq, DecoderConfigs.vdl2.squelch_level);
    for (int i = 0; i < DecoderConfigs.vdl2.num_channels; i++)
        buf += sfmt("%s%.0f", i ? "," : "", DecoderConfigs.vdl2.channel_freqs[i]);
    buf += "]},\n";

    // Radiosonde decoder config
    buf += sfmt(
        "\"radiosonde\":{\"enabled\":%s,\"sondehub_upload\":%s,"
        "\"radiosondy_upload\":%s,\"wettersonde_upload\":%s,"
        "\"callsign\":\"%s\",\"center_freq\":%.0f},\n",
        DecoderConfigs.radiosonde.enabled ? "true" : "false",
        DecoderConfigs.radiosonde.sondehub_upload ? "true" : "false",
        DecoderConfigs.radiosonde.radiosondy_upload ? "true" : "false",
        DecoderConfigs.radiosonde.wettersonde_upload ? "true" : "false",
        DecoderConfigs.radiosonde.callsign,
        DecoderConfigs.radiosonde.center_freq);

    // POCSAG decoder config
    buf += sfmt("\"pocsag\":{\"enabled\":%s,\"output_enabled\":%s,\"center_freq\":%.0f,\"channels\":[",
        DecoderConfigs.pocsag.enabled ? "true" : "false",
        DecoderConfigs.pocsag.output_enabled ? "true" : "false",
        DecoderConfigs.pocsag.center_freq);
    for (int i = 0; i < DecoderConfigs.pocsag.num_channels; i++)
        buf += sfmt("%s%.0f", i ? "," : "", DecoderConfigs.pocsag.channel_freqs[i]);
    buf += "]},\n";

    // GSM decoder config
    buf += sfmt(
        "\"gsm\":{\"enabled\":%s,\"output_enabled\":%s,\"arfcn_freq\":%.0f,\"tsc\":%d},\n",
        DecoderConfigs.gsm.enabled ? "true" : "false",
        DecoderConfigs.gsm.output_enabled ? "true" : "false",
        DecoderConfigs.gsm.arfcn_freq, DecoderConfigs.gsm.tsc);

    // LTE decoder config
    buf += sfmt(
        "\"lte\":{\"enabled\":%s,\"output_enabled\":%s,\"hop_enabled\":%s,\"center_freq\":%.0f},\n",
        DecoderConfigs.lte.enabled ? "true" : "false",
        DecoderConfigs.lte.output_enabled ? "true" : "false",
        DecoderConfigs.lte.hop_enabled ? "true" : "false",
        DecoderConfigs.lte.center_freq);

    // IoT 868 decoder config
    buf += sfmt(
        "\"iot868\":{\"enabled\":%s,\"output_enabled\":%s,\"center_freq\":%.0f},\n",
        DecoderConfigs.iot868.enabled ? "true" : "false",
        DecoderConfigs.iot868.output_enabled ? "true" : "false",
        DecoderConfigs.iot868.center_freq);

    // FANET decoder config
    buf += sfmt(
        "\"fanet\":{\"enabled\":%s,\"output_enabled\":%s,\"center_freq\":%.0f},\n",
        DecoderConfigs.fanet.enabled ? "true" : "false",
        DecoderConfigs.fanet.output_enabled ? "true" : "false",
        DecoderConfigs.fanet.center_freq);

    // Sarsat decoder config
    buf += sfmt(
        "\"sarsat\":{\"enabled\":%s,\"output_enabled\":%s,\"center_freq\":%.0f},\n",
        DecoderConfigs.sarsat.enabled ? "true" : "false",
        DecoderConfigs.sarsat.output_enabled ? "true" : "false",
        DecoderConfigs.sarsat.center_freq);

    // Dongles (from SDR Manager)
    buf += "\"dongles\":[\n";
    pthread_mutex_lock(&SdrManager.lock);
    for (int i = 0; i < SdrManager.count; i++) {
        sdr_receiver_t *rx = &SdrManager.receivers[i];
        const char *role_str = "none";
        switch (rx->config.role) {
            case SDR_ROLE_ADSB: role_str = "adsb"; break;
            case SDR_ROLE_FLARM: role_str = "flarm"; break;
            case SDR_ROLE_ACARS: role_str = "acars"; break;
            case SDR_ROLE_VDL2: role_str = "vdl2"; break;
            case SDR_ROLE_RADIOSONDE: role_str = "radiosonde"; break;
            case SDR_ROLE_POCSAG: role_str = "pocsag"; break;
            case SDR_ROLE_GSM: role_str = "gsm"; break;
            case SDR_ROLE_LTE: role_str = "lte"; break;
            case SDR_ROLE_IOT868: role_str = "iot868"; break;
            case SDR_ROLE_FANET: role_str = "fanet"; break;
            case SDR_ROLE_SARSAT: role_str = "sarsat"; break;
            default: role_str = "none"; break;
        }
        buf += sfmt(
            "%s{\"id\":%d,\"serial\":\"%s\",\"gain\":%.1f,\"ppm\":%d,\"decoder\":\"%s\"}",
            i ? ",\n" : "",
            rx->id, rx->config.serial, rx->config.gain, rx->config.ppm_error, role_str);
    }
    pthread_mutex_unlock(&SdrManager.lock);
    buf += "\n]\n}\n";

    http_send_json(fd, buf.c_str(), (int)buf.size());
}

// ============================= API: POST /api/decoders ====================

static void api_post_decoders(int fd, const char *body)
{
    if (!body || !is_valid_json_object(body, 32768)) {
        http_send(fd, 400, "application/json", "{\"error\":\"invalid JSON\"}", 23);
        return;
    }

    if (!decoderConfigParseJson(body)) {
        http_send(fd, 400, "application/json", "{\"error\":\"parse error\"}", 22);
        return;
    }

    // Apply runtime changes
    PocsagOutputEnabled = DecoderConfigs.pocsag.output_enabled ? 1 : 0;
    GsmOutputEnabled = DecoderConfigs.gsm.output_enabled ? 1 : 0;
    LteOutputEnabled = DecoderConfigs.lte.output_enabled ? 1 : 0;
    IotOutputEnabled = DecoderConfigs.iot868.output_enabled ? 1 : 0;
    FanetOutputEnabled = DecoderConfigs.fanet.output_enabled ? 1 : 0;
    SarsatOutputEnabled = DecoderConfigs.sarsat.output_enabled ? 1 : 0;

    // Save
    decoderConfigSave();

    // Also save FLARM keys if they were updated
    if (DecoderConfigs.flarm.keys_loaded && DecoderConfigs.flarm.keys_file[0]) {
        decoderConfigSaveFlarmKeys(DecoderConfigs.flarm.keys_file);
    }

    http_send(fd, 200, "application/json", "{\"ok\":true}", 11);
}

static void api_get_devices(int fd)
{
    std::string buf;
    buf.reserve(4096);

    buf += "{\"sdr_devices\":[";

    sdr_dev_info_t all_devs[MAX_SDR_RECEIVERS];
    int count = sdrBackendEnumerateAll(all_devs, MAX_SDR_RECEIVERS);
    for (int i = 0; i < count; i++) {
        const char *serial = all_devs[i].serial;
        const char *vendor = all_devs[i].manufacturer;
        const char *product = all_devs[i].product;
        const char *name = product;

        // Check SdrManager for this device
        const char *role = "none";
        const char *state = "idle";
        int tuner_type = -1;
        sdr_tuner_type_t sdr_tuner = SDR_TUNER_UNKNOWN;

        for (int r = 0; r < SdrManager.count; r++) {
            sdr_receiver_t *rx = &SdrManager.receivers[r];
            if (rx->state >= RX_STATE_OPEN &&
                (!strcmp(rx->serial_actual, serial) || !strcmp(rx->config.serial, serial))) {
                role = sdrRoleName(rx->config.role);
                state = rxStateName(rx->state);
                if (rx->backend_dev)
                    sdr_tuner = rx->backend_dev->tuner_type;
                break;
            }
        }

        // Prefer tuner_cache (uses SDR_TUNER enum from backend probe at startup)
        tuner_type = tuner_cache_lookup(serial);

        // Determine tuner name and range
        const char *tname, *trange;
        if (tuner_type >= 0) {
            tname = tuner_name_sdr((sdr_tuner_type_t)tuner_type);
            trange = tuner_freq_range_sdr((sdr_tuner_type_t)tuner_type);
        } else if (sdr_tuner != SDR_TUNER_UNKNOWN) {
            tname = tuner_name_sdr(sdr_tuner);
            trange = tuner_freq_range_sdr(sdr_tuner);
        } else {
            tname = "unknown";
            trange = "unknown";
        }

        if (i > 0) buf += ',';
        buf += sfmt(
            "{\"index\":%d,\"name\":\"%s\",\"vendor\":\"%s\",\"product\":\"%s\","
            "\"serial\":\"%s\",\"role\":\"%s\",\"state\":\"%s\","
            "\"tuner\":\"%s\",\"freq_range\":\"%s\"}",
            i, name ? name : "", vendor, product, serial,
            role, state, tname, trange);
    }

    // Append virtual (file) devices from SdrManager
    {
        int need_comma = (count > 0);
        for (int r = 0; r < SdrManager.count; r++) {
            sdr_receiver_t *rx = &SdrManager.receivers[r];
            if (rx->config.ifile_path[0] == '\0') continue;  // skip real SDR
            if (need_comma) buf += ',';
            need_comma = 1;
            buf += sfmt(
                "{\"index\":%d,\"name\":\"Virtual IQ Replay\",\"vendor\":\"Virtual\","
                "\"product\":\"IQ File\",\"serial\":\"%s\","
                "\"role\":\"%s\",\"state\":\"%s\","
                "\"tuner\":\"file\",\"freq_range\":\"any\","
                "\"file\":\"%s\"}",
                rx->id, rx->serial_actual,
                sdrRoleName(rx->config.role), rxStateName(rx->state),
                rx->config.ifile_path);
        }
    }

    buf += "],\"usb_devices\":[";

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
            if (!first) buf += ',';
            first = 0;
            // Escape for JSON
            buf += sfmt("\"%s\"", line);
        }
        pclose(fp);
    }

    buf += sfmt(
        "],\"rx_count\":%d,\"rx_max\":%d}",
        SdrManager.count, MAX_SDR_RECEIVERS);

    http_send_json(fd, buf.c_str(), (int)buf.size());
}

// ============================= GSM Cells Page ============================

static void serve_gsm_page(int fd)
{
    const char *html =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<title>GSM Cells - dump1090-gg</title>"
        "<style>"
        ":root{--bg:#0a0a1a;--card:#141428;--head:#1a1a2e;--border:#2a2a4a;--accent:#4fc3f7;--text:#d0d0d0;--dim:#888;--hover:#1e1e3a;--danger:#ff4444;--warn:#ffaa00;--link:#44aaff;--ok:#00cc44;--input-bg:#0e0e22}"
        "body{background:var(--bg);color:var(--text);font-family:'Segoe UI',sans-serif;margin:0;padding:0}"
        "nav{background:var(--head);border-bottom:1px solid var(--border);padding:8px 16px;display:flex;align-items:center;gap:16px;position:sticky;top:0;z-index:100}"
        "nav h1{font-size:18px;color:var(--accent);white-space:nowrap;margin:0}"
        ".tabs{display:flex;gap:4px;flex-wrap:wrap;flex:1}"
        ".tabs a{padding:6px 14px;border-radius:6px;cursor:pointer;color:var(--dim);transition:.2s;font-size:13px;user-select:none;text-decoration:none}"
        ".tabs a:hover{background:var(--hover);color:var(--text)}"
        ".tabs a.active{background:var(--accent);color:#000;font-weight:600}"
        ".main{padding:20px;max-width:1600px;margin:0 auto}"
        "h2{color:var(--accent);margin-top:0}"
        "table{border-collapse:collapse;width:100%;font-size:13px}"
        "th{background:var(--head);color:var(--accent);padding:8px;text-align:left;position:sticky;top:42px;z-index:10;cursor:pointer;user-select:none}"
        "td{padding:6px 8px;border-bottom:1px solid var(--border)}"
        "tr:hover td{background:var(--hover)}"
        ".badge{display:inline-block;padding:2px 8px;border-radius:10px;font-size:11px;font-weight:600}"
        ".badge-ok{background:#003322;color:var(--ok)}"
        ".badge-warn{background:#332200;color:var(--warn)}"
        ".badge-err{background:#330000;color:var(--danger)}"
        ".badge-info{background:#002244;color:var(--link)}"
        ".card{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:16px;margin-bottom:12px}"
        ".stats{display:grid;grid-template-columns:repeat(auto-fill,minmax(180px,1fr));gap:12px;margin-bottom:16px}"
        ".stat{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:12px;text-align:center}"
        ".stat .val{font-size:24px;font-weight:700;color:var(--accent)}"
        ".stat .lbl{font-size:11px;color:var(--dim);margin-top:4px}"
        ".no-gsm{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:40px;text-align:center;color:var(--dim)}"
        ".no-gsm h3{color:var(--accent);margin-bottom:8px}"
        ".toolbar{display:flex;gap:8px;margin-bottom:12px;align-items:center}"
        ".btn{padding:6px 14px;border:1px solid var(--accent);border-radius:4px;cursor:pointer;font-size:13px;background:var(--head);color:var(--accent)}"
        ".btn:hover{background:var(--accent);color:#000}"
        "code{background:var(--input-bg);padding:1px 5px;border-radius:3px;font-size:12px}"
        ".cb-text{max-width:250px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;color:var(--dim);font-size:12px}"
        "@media(max-width:768px){table{font-size:12px}th,td{padding:4px 6px}}"
        "</style></head><body>"
        "<nav><h1>&#x2708; dump1090-gg-light</h1><div class='tabs'>"
        "<a href='/'>&#x2699;&#xfe0f; Config</a>"
        "<a href='/status.html'>&#x1f4e1; Status</a>"
        "<a href='/connections.html'>&#x1f50c; Connections</a>"
        "<a href='/logs.html'>&#x1f4cb; Logs</a>"
        "<a href='/messages.html'>&#x1f4e8; Messages</a>"
        "<a href='/aircraft.html'>&#x2708;&#xfe0f; Aircraft</a>"
        "<a href='/devices.html'>&#x1f4fb; Devices</a>"
        "<a class='active' href='/gsm.html'>&#x1f4f6; GSM</a>"
        "<a href='/lte.html'>&#x1f4f6; LTE</a>"
        "<a href='/iot868.html'>&#x1f321;&#xfe0f; IoT 868</a>"
        "<a href='/fanet.html'>&#x1f6a9; FANET</a>"
        "<a style='margin-left:auto' href='/diagnostics.html'>&#x1f527; Diagnostics</a>"
        "</div>"
        "<span id='ver-badge' style='font-size:11px;color:var(--dim);white-space:nowrap;padding:2px 8px;border:1px solid var(--border);border-radius:10px' title='dump1090-gg version'>v&hellip;</span>"
        "</nav>"
        "<div class='main'>"
        "<h2>&#x1f4f6; GSM Cell Monitor</h2>"
        "<div class='toolbar'>"
        "<span id='cell-count' style='font-size:16px;font-weight:600;color:var(--accent)'></span>"
        "<span id='update-time' style='color:var(--dim);margin-left:16px;font-size:12px'></span>"
        "<button class='btn' onclick='load()' style='margin-left:auto'>&#x21bb; Refresh</button>"
        "</div>"
        "<div id='content'><p style='color:var(--dim)'>Loading...</p></div>"

        "<script>"
        "var sortKey='mcc',sortDir=1,cellData=[];"
        ""
        "function load(){"
        "  fetch('/api/config').then(r=>r.json()).then(cfg=>{"
        "    var gsm=cfg.sdr_gsm||{};"
        "    if(!gsm.active){"
        "      document.getElementById('content').innerHTML="
        "        '<div class=\"no-gsm\"><h3>&#x1f4f6; GSM Scanner Not Active</h3>'"
        "        +'<p>No SDR device is configured for GSM reception.</p>'"
        "        +'<p style=\"margin-top:12px\">Go to <a href=\"/devices.html\">Devices</a> and assign an RTL-SDR dongle to the <strong>GSM (935 MHz)</strong> role.</p></div>';"
        "      document.getElementById('cell-count').textContent='';"
        "      return;"
        "    }"
        "    if(!gsm.enabled){"
        "      document.getElementById('content').innerHTML="
        "        '<div class=\"no-gsm\"><h3>&#x1f4f6; GSM Scanner Disabled</h3>'"
        "        +'<p>The GSM decoder is currently disabled.</p>'"
        "        +'<p style=\"margin-top:12px\">Enable it from the <a href=\"/\">Config</a> page.</p></div>';"
        "      document.getElementById('cell-count').textContent='';"
        "      return;"
        "    }"
        "    fetch('/api/gsm').then(r=>r.json()).then(data=>render(data));"
        "  }).catch(()=>{});"
        "}"
        ""
        "function render(data){"
        "  cellData=data.cells||[];"
        "  document.getElementById('cell-count').textContent=cellData.length+' cell'+(cellData.length!==1?'s':'');"
        "  document.getElementById('update-time').textContent='Updated: '+new Date().toLocaleTimeString();"
        "  if(!cellData.length){"
        "    document.getElementById('content').innerHTML='<div class=\"no-gsm\"><h3>Scanning...</h3><p>GSM decoder is active but no cells have been detected yet.</p><p style=\"color:var(--dim);font-size:12px;margin-top:8px\">The decoder searches for an FCCH tone. Cells appear once FCCH is found.</p></div>';"
        "    return;"
        "  }"
        ""
        "  /* Stats summary */"
        "  var totalFcch=0,totalCcch=0,totalCb=0,nIdentified=0,nFcchOnly=0;"
        "  cellData.forEach(c=>{"
        "    totalFcch+=c.bcch;totalCcch+=c.ccch;totalCb+=c.cb;"
        "    if(c.mcc>0) nIdentified++; else nFcchOnly++;"
        "  });"
        "  var html='<div class=\"stats\">';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+cellData.length+'</div><div class=\"lbl\">Cells Found</div></div>';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+nIdentified+' / '+nFcchOnly+'</div><div class=\"lbl\">Identified / FCCH-only</div></div>';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+totalFcch+'</div><div class=\"lbl\">FCCH Detections</div></div>';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+totalCb+'</div><div class=\"lbl\">Cell Broadcasts</div></div>';"
        "  html+='</div>';"
        ""
        "  /* Sort */"
        "  cellData.sort((a,b)=>{"
        "    var va=a[sortKey],vb=b[sortKey];"
        "    if(typeof va==='string') return sortDir*va.localeCompare(vb);"
        "    return sortDir*((va||0)-(vb||0));"
        "  });"
        ""
        "  /* Table */"
        "  html+='<table><thead><tr>';"
        "  var cols=[{k:'mcc',l:'MCC'},{k:'mnc',l:'MNC'},{k:'lac',l:'LAC'},{k:'cid',l:'Cell ID'},"
        "    {k:'arfcn',l:'ARFCN'},{k:'freq_mhz',l:'Freq (MHz)'},{k:'bsic',l:'BSIC'},"
        "    {k:'sync',l:'Sync'},{k:'t3212',l:'T3212'},{k:'cell_barred',l:'Barred'},"
        "    {k:'bcch',l:'FCCH'},{k:'ccch',l:'CCCH'},{k:'cb',l:'CB'},"
        "    {k:'freq_offset',l:'F.Offset'},{k:'last_cb_text',l:'Last CB Text'},"
        "    {k:'age',l:'Age'}];"
        "  cols.forEach(c=>{"
        "    var arrow=sortKey===c.k?(sortDir>0?' &#x25b2;':' &#x25bc;'):'';"
        "    html+='<th onclick=\"sortBy(\\''+c.k+'\\')\">' +c.l+arrow+'</th>';"
        "  });"
        "  html+='</tr></thead><tbody>';"
        ""
        "  cellData.forEach(c=>{"
        "    var syncCls=c.sync==='locked'?'badge-ok':c.sync==='sch'?'badge-warn':'badge-info';"
        "    var barCls=c.cell_barred?'badge-err':'badge-ok';"
        "    var ageCls=c.age>60?'badge-warn':c.stale?'badge-err':'badge-ok';"
        "    var fcchOnly=c.mcc===0;"
        "    html+='<tr'+(c.stale?' style=\"opacity:0.5\"':'')+'>';"
        "    html+='<td><strong>'+(fcchOnly?'&mdash;':c.mcc)+'</strong></td>';"
        "    html+='<td>'+(fcchOnly?'&mdash;':(''+c.mnc).padStart(2,'0'))+'</td>';"
        "    html+='<td>'+(fcchOnly?'&mdash;':c.lac)+'</td>';"
        "    html+='<td>'+(fcchOnly?'&mdash;':'<code>'+c.cid+'</code>')+'</td>';"
        "    html+='<td>'+c.arfcn+'</td>';"
        "    html+='<td>'+c.freq_mhz.toFixed(3)+'</td>';"
        "    html+='<td>'+(fcchOnly?'&mdash;':c.bsic)+'</td>';"
        "    html+='<td><span class=\"badge '+syncCls+'\">'+c.sync+'</span></td>';"
        "    html+='<td>'+(fcchOnly?'&mdash;':c.t3212)+'</td>';"
        "    html+='<td>'+(fcchOnly?'&mdash;':'<span class=\"badge '+(c.cell_barred?'badge-err':'badge-ok')+'\">'+(c.cell_barred?'YES':'no')+'</span>')+'</td>';"
        "    html+='<td>'+c.bcch+'</td>';"
        "    html+='<td>'+c.ccch+'</td>';"
        "    html+='<td>'+c.cb+'</td>';"
        "    var ppm=c.freq_mhz>0?(c.freq_offset/(c.freq_mhz*1e6)*1e6).toFixed(2):'';"
        "    html+='<td>'+(c.freq_offset>=0?'+':'')+c.freq_offset.toFixed(1)+' Hz'+(ppm?' ('+ppm+' ppm)':'')+'</td>';"
        "    html+='<td class=\"cb-text\" title=\"'+(c.last_cb_text||'')+'\">'+(c.last_cb_text||'&mdash;')+'</td>';"
        "    html+='<td><span class=\"badge '+ageCls+'\">'+c.age.toFixed(0)+'s</span></td>';"
        "    html+='</tr>';"
        "  });"
        "  html+='</tbody></table>';"
        "  document.getElementById('content').innerHTML=html;"
        "}"
        ""
        "function sortBy(key){"
        "  if(sortKey===key) sortDir=-sortDir;"
        "  else{sortKey=key;sortDir=1;}"
        "  if(cellData.length) render({cells:cellData});"
        "}"
        ""
        "load();"
        "setInterval(load,5000);"  // auto-refresh every 5s
        ""
        // Fetch version
        "fetch('/api/status').then(r=>r.json()).then(s=>{"
        "  var v=document.getElementById('ver-badge');"
        "  if(v&&s.version) v.textContent='v'+s.version;"
        "}).catch(()=>{});"
        "</script></div></body></html>";

    http_send(fd, 200, "text/html", html, (int)strlen(html));
}

static void serve_lte_page(int fd)
{
    const char *html =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<title>LTE Cells - dump1090-gg</title>"
        "<style>"
        ":root{--bg:#0a0a1a;--card:#141428;--head:#1a1a2e;--border:#2a2a4a;--accent:#4fc3f7;--text:#d0d0d0;--dim:#888;--hover:#1e1e3a;--danger:#ff4444;--warn:#ffaa00;--link:#44aaff;--ok:#00cc44;--input-bg:#0e0e22}"
        "body{background:var(--bg);color:var(--text);font-family:'Segoe UI',sans-serif;margin:0;padding:0}"
        "nav{background:var(--head);border-bottom:1px solid var(--border);padding:8px 16px;display:flex;align-items:center;gap:16px;position:sticky;top:0;z-index:100}"
        "nav h1{font-size:18px;color:var(--accent);white-space:nowrap;margin:0}"
        ".tabs{display:flex;gap:4px;flex-wrap:wrap;flex:1}"
        ".tabs a{padding:6px 14px;border-radius:6px;cursor:pointer;color:var(--dim);transition:.2s;font-size:13px;user-select:none;text-decoration:none}"
        ".tabs a:hover{background:var(--hover);color:var(--text)}"
        ".tabs a.active{background:var(--accent);color:#000;font-weight:600}"
        ".main{padding:20px;max-width:1600px;margin:0 auto}"
        "h2{color:var(--accent);margin-top:0}"
        "table{border-collapse:collapse;width:100%;font-size:13px}"
        "th{background:var(--head);color:var(--accent);padding:8px;text-align:left;position:sticky;top:42px;z-index:10;cursor:pointer;user-select:none}"
        "td{padding:6px 8px;border-bottom:1px solid var(--border)}"
        "tr:hover td{background:var(--hover)}"
        ".badge{display:inline-block;padding:2px 8px;border-radius:10px;font-size:11px;font-weight:600}"
        ".badge-ok{background:#003322;color:var(--ok)}"
        ".badge-warn{background:#332200;color:var(--warn)}"
        ".badge-err{background:#330000;color:var(--danger)}"
        ".badge-info{background:#002244;color:var(--link)}"
        ".card{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:16px;margin-bottom:12px}"
        ".stats{display:grid;grid-template-columns:repeat(auto-fill,minmax(180px,1fr));gap:12px;margin-bottom:16px}"
        ".stat{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:12px;text-align:center}"
        ".stat .val{font-size:24px;font-weight:700;color:var(--accent)}"
        ".stat .lbl{font-size:11px;color:var(--dim);margin-top:4px}"
        ".no-lte{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:40px;text-align:center;color:var(--dim)}"
        ".no-lte h3{color:var(--accent);margin-bottom:8px}"
        ".toolbar{display:flex;gap:8px;margin-bottom:12px;align-items:center}"
        ".btn{padding:6px 14px;border:1px solid var(--accent);border-radius:4px;cursor:pointer;font-size:13px;background:var(--head);color:var(--accent)}"
        ".btn:hover{background:var(--accent);color:#000}"
        "code{background:var(--input-bg);padding:1px 5px;border-radius:3px;font-size:12px}"
        "@media(max-width:768px){table{font-size:12px}th,td{padding:4px 6px}}"
        "</style></head><body>"
        "<nav><h1>&#x2708; dump1090-gg-light</h1><div class='tabs'>"
        "<a href='/'>&#x2699;&#xfe0f; Config</a>"
        "<a href='/status.html'>&#x1f4e1; Status</a>"
        "<a href='/connections.html'>&#x1f50c; Connections</a>"
        "<a href='/logs.html'>&#x1f4cb; Logs</a>"
        "<a href='/messages.html'>&#x1f4e8; Messages</a>"
        "<a href='/aircraft.html'>&#x2708;&#xfe0f; Aircraft</a>"
        "<a href='/devices.html'>&#x1f4fb; Devices</a>"
        "<a href='/gsm.html'>&#x1f4f6; GSM</a>"
        "<a class='active' href='/lte.html'>&#x1f4f6; LTE</a>"
        "<a href='/iot868.html'>&#x1f321;&#xfe0f; IoT 868</a>"
        "<a href='/fanet.html'>&#x1f6a9; FANET</a>"
        "<a style='margin-left:auto' href='/diagnostics.html'>&#x1f527; Diagnostics</a>"
        "</div>"
        "<span id='ver-badge' style='font-size:11px;color:var(--dim);white-space:nowrap;padding:2px 8px;border:1px solid var(--border);border-radius:10px' title='dump1090-gg version'>v&hellip;</span>"
        "</nav>"
        "<div class='main'>"
        "<h2>&#x1f4f6; LTE Cell Scanner</h2>"
        "<div class='toolbar'>"
        "<span id='cell-count' style='font-size:16px;font-weight:600;color:var(--accent)'></span>"
        "<span id='update-time' style='color:var(--dim);margin-left:16px;font-size:12px'></span>"
        "<button class='btn' onclick='load()' style='margin-left:auto'>&#x21bb; Refresh</button>"
        "</div>"
        "<div id='content'><p style='color:var(--dim)'>Loading...</p></div>"

        "<script>"
        "var sortKey='pci',sortDir=1,cellData=[];"
        ""
        "function load(){"
        "  fetch('/api/config').then(r=>r.json()).then(cfg=>{"
        "    var lte=cfg.sdr_lte||{};"
        "    if(!lte.active){"
        "      document.getElementById('content').innerHTML="
        "        '<div class=\"no-lte\"><h3>&#x1f4f6; LTE Scanner Not Active</h3>'"
        "        +'<p>No SDR device is configured for LTE reception.</p>'"
        "        +'<p style=\"margin-top:12px\">Go to <a href=\"/devices.html\">Devices</a> and assign an RTL-SDR dongle to the <strong>LTE (800 MHz)</strong> role.</p></div>';"
        "      document.getElementById('cell-count').textContent='';"
        "      return;"
        "    }"
        "    if(!lte.enabled){"
        "      document.getElementById('content').innerHTML="
        "        '<div class=\"no-lte\"><h3>&#x1f4f6; LTE Scanner Disabled</h3>'"
        "        +'<p>The LTE decoder is currently disabled.</p>'"
        "        +'<p style=\"margin-top:12px\">Enable it from the <a href=\"/\">Config</a> page.</p></div>';"
        "      document.getElementById('cell-count').textContent='';"
        "      return;"
        "    }"
        "    fetch('/api/lte').then(r=>r.json()).then(data=>render(data));"
        "  }).catch(()=>{});"
        "}"
        ""
        "function render(data){"
        "  cellData=data.cells||[];"
        "  document.getElementById('cell-count').textContent=cellData.length+' cell'+(cellData.length!==1?'s':'');"
        "  document.getElementById('update-time').textContent='Updated: '+new Date().toLocaleTimeString();"
        "  if(!cellData.length){"
        "    document.getElementById('content').innerHTML='<div class=\"no-lte\"><h3>Scanning...</h3><p>LTE decoder is active but no cells have been detected yet.</p><p style=\"color:var(--dim);font-size:12px;margin-top:8px\">The decoder correlates PSS (Zadoff-Chu) sequences. Cells appear once PSS/SSS are found.</p></div>';"
        "    return;"
        "  }"
        ""
        "  /* Stats summary */"
        "  var nPss=0,nSss=0,nMib=0,nSib=0;"
        "  cellData.forEach(c=>{"
        "    nPss+=c.pss_count||0;nSss+=c.sss_count||0;nMib+=c.mib_count||0;nSib+=c.sib1_count||0;"
        "  });"
        "  var html='<div class=\"stats\">';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+cellData.length+'</div><div class=\"lbl\">Cells Found</div></div>';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+nPss+'</div><div class=\"lbl\">PSS Detections</div></div>';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+nMib+'</div><div class=\"lbl\">MIB Decoded</div></div>';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+nSib+'</div><div class=\"lbl\">SIB1 Decoded</div></div>';"
        "  html+='</div>';"
        ""
        "  /* Sort */"
        "  cellData.sort((a,b)=>{"
        "    var va=a[sortKey],vb=b[sortKey];"
        "    if(typeof va==='string') return sortDir*va.localeCompare(vb);"
        "    return sortDir*((va||0)-(vb||0));"
        "  });"
        ""
        "  /* Table */"
        "  html+='<table><thead><tr>';"
        "  var cols=[{k:'pci',l:'PCI'},{k:'operator',l:'Operator'},{k:'band',l:'Band'},"
        "    {k:'freq',l:'Freq (MHz)'},{k:'earfcn',l:'EARFCN'},"
        "    {k:'rsrp',l:'RSRP (dBFS)'},{k:'snr',l:'SNR (dB)'},{k:'freq_offset',l:'F.Offset (Hz)'},"
        "    {k:'sync',l:'Sync Level'},{k:'bw',l:'Bandwidth'},{k:'sfn',l:'SFN'},"
        "    {k:'mcc',l:'MCC'},{k:'mnc',l:'MNC'},{k:'tac',l:'TAC'},{k:'cell_id',l:'Cell ID'},"
        "    {k:'location',l:'Location'},{k:'age',l:'Age'}];"
        "  cols.forEach(c=>{"
        "    var arrow=sortKey===c.k?(sortDir>0?' &#x25b2;':' &#x25bc;'):'';"
        "    html+='<th onclick=\"sortBy(\\''+c.k+'\\')\">' +c.l+arrow+'</th>';"
        "  });"
        "  html+='</tr></thead><tbody>';"
        ""
        "  cellData.forEach(c=>{"
        "    var syncCls=c.sync==='SIB1'?'badge-ok':c.sync==='MIB'?'badge-ok':c.sync==='SSS'?'badge-warn':'badge-info';"
        "    var mib=c.mib||{};"
        "    var sib=c.sib1||{};"
        "    var db=c.celldb||{};"
        "    var mcc=sib.mcc||db.mcc||'';"
        "    var mnc=sib.mnc||db.mnc||'';"
        "    var tac=sib.tac||db.tac||'';"
        "    var cid=sib.cell_id||db.cell_id||'';"
        "    var eid=db.enodeb_id||'';"
        "    var loc=(db.lat&&db.lon)?db.lat.toFixed(4)+', '+db.lon.toFixed(4):'';"
        "    html+='<tr>';"
        "    html+='<td><strong>'+c.pci+'</strong></td>';"
        "    html+='<td>'+(c.operator||'&mdash;')+'</td>';"
        "    html+='<td>'+c.band+'</td>';"
        "    html+='<td>'+(c.freq/1e6).toFixed(3)+'</td>';"
        "    html+='<td>'+c.earfcn+'</td>';"
        "    html+='<td>'+c.rsrp.toFixed(1)+'</td>';"
        "    html+='<td>'+c.snr.toFixed(1)+'</td>';"
        "    html+='<td>'+(c.freq_offset>=0?'+':'')+c.freq_offset.toFixed(1)+'</td>';"
        "    html+='<td><span class=\"badge '+syncCls+'\">'+c.sync+'</span></td>';"
        "    html+='<td>'+(mib.bw||'&mdash;')+'</td>';"
        "    html+='<td>'+(mib.sfn!==undefined?mib.sfn:'&mdash;')+'</td>';"
        "    html+='<td>'+(mcc||'&mdash;')+'</td>';"
        "    html+='<td>'+(mnc||'&mdash;')+'</td>';"
        "    html+='<td>'+(tac?'0x'+tac.toString(16):'&mdash;')+'</td>';"
        "    html+='<td>'+(cid?'<code>'+cid.toString(16).toUpperCase()+'</code> (eNB '+eid+')':'&mdash;')+'</td>';"
        "    html+='<td>'+(loc?'<a href=\"https://www.google.com/maps?q='+db.lat+','+db.lon+'\" target=\"_blank\">'+loc+'</a>':'&mdash;')+'</td>';"
        "    html+='<td><span class=\"badge '+(c.age>60?'badge-warn':'badge-ok')+'\">'+c.age+'s</span></td>';"
        "    html+='</tr>';"
        "  });"
        "  html+='</tbody></table>';"
        "  document.getElementById('content').innerHTML=html;"
        "}"
        ""
        "function sortBy(key){"
        "  if(sortKey===key) sortDir=-sortDir;"
        "  else{sortKey=key;sortDir=1;}"
        "  if(cellData.length) render({cells:cellData});"
        "}"
        ""
        "load();"
        "setInterval(load,5000);"
        ""
        "fetch('/api/status').then(r=>r.json()).then(s=>{"
        "  var v=document.getElementById('ver-badge');"
        "  if(v&&s.version) v.textContent='v'+s.version;"
        "}).catch(()=>{});"
        "</script></div></body></html>";

    http_send(fd, 200, "text/html", html, (int)strlen(html));
}

static void serve_iot868_page(int fd)
{
    const char *html =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<title>IoT 868 MHz - dump1090-gg</title>"
        "<style>"
        ":root{--bg:#0a0a1a;--card:#141428;--head:#1a1a2e;--border:#2a2a4a;--accent:#4fc3f7;--text:#d0d0d0;--dim:#888;--hover:#1e1e3a;--danger:#ff4444;--warn:#ffaa00;--link:#44aaff;--ok:#00cc44;--input-bg:#0e0e22}"
        "body{background:var(--bg);color:var(--text);font-family:'Segoe UI',sans-serif;margin:0;padding:0}"
        "nav{background:var(--head);border-bottom:1px solid var(--border);padding:8px 16px;display:flex;align-items:center;gap:16px;position:sticky;top:0;z-index:100}"
        "nav h1{font-size:18px;color:var(--accent);white-space:nowrap;margin:0}"
        ".tabs{display:flex;gap:4px;flex-wrap:wrap;flex:1}"
        ".tabs a{padding:6px 14px;border-radius:6px;cursor:pointer;color:var(--dim);transition:.2s;font-size:13px;user-select:none;text-decoration:none}"
        ".tabs a:hover{background:var(--hover);color:var(--text)}"
        ".tabs a.active{background:var(--accent);color:#000;font-weight:600}"
        ".main{padding:20px;max-width:1600px;margin:0 auto}"
        "h2{color:var(--accent);margin-top:0}"
        "table{border-collapse:collapse;width:100%;font-size:13px}"
        "th{background:var(--head);color:var(--accent);padding:8px;text-align:left;position:sticky;top:42px;z-index:10;cursor:pointer;user-select:none}"
        "td{padding:6px 8px;border-bottom:1px solid var(--border)}"
        "tr:hover td{background:var(--hover)}"
        ".badge{display:inline-block;padding:2px 8px;border-radius:10px;font-size:11px;font-weight:600}"
        ".badge-ok{background:#003322;color:var(--ok)}"
        ".badge-warn{background:#332200;color:var(--warn)}"
        ".badge-err{background:#330000;color:var(--danger)}"
        ".badge-info{background:#002244;color:var(--link)}"
        ".card{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:16px;margin-bottom:12px}"
        ".stats{display:grid;grid-template-columns:repeat(auto-fill,minmax(180px,1fr));gap:12px;margin-bottom:16px}"
        ".stat{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:12px;text-align:center}"
        ".stat .val{font-size:24px;font-weight:700;color:var(--accent)}"
        ".stat .lbl{font-size:11px;color:var(--dim);margin-top:4px}"
        ".no-data{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:40px;text-align:center;color:var(--dim)}"
        ".no-data h3{color:var(--accent);margin-bottom:8px}"
        ".toolbar{display:flex;gap:8px;margin-bottom:12px;align-items:center}"
        ".btn{padding:6px 14px;border:1px solid var(--accent);border-radius:4px;cursor:pointer;font-size:13px;background:var(--head);color:var(--accent)}"
        ".btn:hover{background:var(--accent);color:#000}"
        "code{background:var(--input-bg);padding:1px 5px;border-radius:3px;font-size:12px}"
        "@media(max-width:768px){table{font-size:12px}th,td{padding:4px 6px}}"
        "</style></head><body>"
        "<nav><h1>&#x2708; dump1090-gg-light</h1><div class='tabs'>"
        "<a href='/'>&#x2699;&#xfe0f; Config</a>"
        "<a href='/status.html'>&#x1f4e1; Status</a>"
        "<a href='/connections.html'>&#x1f50c; Connections</a>"
        "<a href='/logs.html'>&#x1f4cb; Logs</a>"
        "<a href='/messages.html'>&#x1f4e8; Messages</a>"
        "<a href='/aircraft.html'>&#x2708;&#xfe0f; Aircraft</a>"
        "<a href='/devices.html'>&#x1f4fb; Devices</a>"
        "<a href='/gsm.html'>&#x1f4f6; GSM</a>"
        "<a href='/lte.html'>&#x1f4f6; LTE</a>"
        "<a class='active' href='/iot868.html'>&#x1f321;&#xfe0f; IoT 868</a>"
        "<a href='/fanet.html'>&#x1f6a9; FANET</a>"
        "<a style='margin-left:auto' href='/diagnostics.html'>&#x1f527; Diagnostics</a>"
        "</div>"
        "<span id='ver-badge' style='font-size:11px;color:var(--dim);white-space:nowrap;padding:2px 8px;border:1px solid var(--border);border-radius:10px' title='dump1090-gg version'>v&hellip;</span>"
        "</nav>"
        "<div class='main'>"
        "<h2>&#x1f321;&#xfe0f; IoT 868 MHz Monitor</h2>"
        "<div class='toolbar'>"
        "<span id='dev-count' style='font-size:16px;font-weight:600;color:var(--accent)'></span>"
        "<span id='update-time' style='color:var(--dim);margin-left:16px;font-size:12px'></span>"
        "<button class='btn' onclick='load()' style='margin-left:auto'>&#x21bb; Refresh</button>"
        "</div>"
        "<div id='content'><p style='color:var(--dim)'>Loading...</p></div>"

        "<script>"
        "var sortKey='protocol',sortDir=1,devData=[];"
        ""
        "function load(){"
        "  fetch('/api/config').then(r=>r.json()).then(cfg=>{"
        "    var iot=cfg.sdr_iot868||{};"
        "    if(!iot.active){"
        "      document.getElementById('content').innerHTML="
        "        '<div class=\"no-data\"><h3>&#x1f321;&#xfe0f; IoT 868 MHz Scanner Not Active</h3>'"
        "        +'<p>No SDR device is configured for IoT 868 MHz reception.</p>'"
        "        +'<p style=\"margin-top:12px\">Go to <a href=\"/devices.html\">Devices</a> and assign an RTL-SDR dongle to the <strong>IoT 868 MHz</strong> role.</p></div>';"
        "      document.getElementById('dev-count').textContent='';"
        "      return;"
        "    }"
        "    if(!iot.enabled){"
        "      document.getElementById('content').innerHTML="
        "        '<div class=\"no-data\"><h3>&#x1f321;&#xfe0f; IoT 868 MHz Scanner Disabled</h3>'"
        "        +'<p>The IoT 868 MHz decoder is currently disabled.</p>'"
        "        +'<p style=\"margin-top:12px\">Enable it from the <a href=\"/\">Config</a> page.</p></div>';"
        "      document.getElementById('dev-count').textContent='';"
        "      return;"
        "    }"
        "    fetch('/api/iot868').then(r=>r.json()).then(data=>render(data)).catch(()=>{"
        "      document.getElementById('content').innerHTML='<div class=\"no-data\"><h3>Error</h3><p>Failed to fetch IoT data.</p></div>';"
        "    });"
        "  }).catch(()=>{});"
        "}"
        ""
        "function render(data){"
        "  devData=data.devices||[];"
        "  document.getElementById('dev-count').textContent=devData.length+' device'+(devData.length!==1?'s':'');"
        "  document.getElementById('update-time').textContent='Updated: '+new Date().toLocaleTimeString();"
        "  if(!devData.length){"
        "    document.getElementById('content').innerHTML='<div class=\"no-data\"><h3>Scanning...</h3><p>IoT decoder is active but no devices detected yet.</p><p style=\"color:var(--dim);font-size:12px;margin-top:8px\">Listening on 868.3 MHz (2 MHz BW) for OOK/FSK signals from weather stations, smart meters, thermostats, etc.</p></div>';"
        "    return;"
        "  }"
        ""
        "  /* Stats summary */"
        "  var protos={};var totalMsg=0;"
        "  devData.forEach(d=>{"
        "    protos[d.protocol]=(protos[d.protocol]||0)+1;"
        "    totalMsg+=d.msg_count;"
        "  });"
        "  var html='<div class=\"stats\">';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+devData.length+'</div><div class=\"lbl\">Devices</div></div>';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+Object.keys(protos).length+'</div><div class=\"lbl\">Protocols</div></div>';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+totalMsg+'</div><div class=\"lbl\">Messages</div></div>';"
        "  html+='</div>';"
        ""
        "  /* Sort */"
        "  devData.sort((a,b)=>{"
        "    var va=a[sortKey],vb=b[sortKey];"
        "    if(typeof va==='string') return sortDir*va.localeCompare(vb);"
        "    return sortDir*((va||0)-(vb||0));"
        "  });"
        ""
        "  /* Table */"
        "  html+='<table><thead><tr>';"
        "  var cols=[{k:'protocol',l:'Protocol'},{k:'modulation',l:'Mod'},{k:'device_id',l:'Device ID'},"
        "    {k:'channel',l:'Ch'},{k:'temperature',l:'Temp \\u00b0C'},{k:'humidity',l:'Hum %'},"
        "    {k:'pressure',l:'hPa'},{k:'wind_speed',l:'Wind m/s'},{k:'wind_dir',l:'Dir \\u00b0'},"
        "    {k:'rain',l:'Rain mm'},{k:'power',l:'Power W'},{k:'energy',l:'kWh'},"
        "    {k:'battery_ok',l:'Bat'},{k:'freq_mhz',l:'Freq MHz'},"
        "    {k:'rssi',l:'RSSI'},{k:'msg_count',l:'Msgs'},{k:'age',l:'Age'}];"
        "  cols.forEach(c=>{"
        "    var arrow=sortKey===c.k?(sortDir>0?' \\u25b2':' \\u25bc'):'';"
        "    html+='<th onclick=\"sortBy(\\''+c.k+'\\')\">' +c.l+arrow+'</th>';"
        "  });"
        "  html+='</tr></thead><tbody>';"
        ""
        "  devData.forEach(d=>{"
        "    var ageCls=d.age>120?'badge-err':d.age>60?'badge-warn':'badge-ok';"
        "    var batCls=d.battery_ok===0?'badge-err':d.battery_ok===1?'badge-ok':'badge-info';"
        "    var batTxt=d.battery_ok===0?'LOW':d.battery_ok===1?'OK':'?';"
        "    html+='<tr'+(d.stale?' style=\"opacity:0.5\"':'')+'>';"
        "    html+='<td><strong>'+d.protocol+'</strong></td>';"
        "    html+='<td><span class=\"badge badge-info\">'+d.modulation+'</span></td>';"
        "    html+='<td><code>'+d.device_id+'</code></td>';"
        "    html+='<td>'+(d.channel||'&mdash;')+'</td>';"
        "    html+='<td>'+(d.temperature>-900?d.temperature.toFixed(1):'&mdash;')+'</td>';"
        "    html+='<td>'+(d.humidity>=0?d.humidity.toFixed(0):'&mdash;')+'</td>';"
        "    html+='<td>'+(d.pressure>=0?d.pressure.toFixed(1):'&mdash;')+'</td>';"
        "    html+='<td>'+(d.wind_speed>=0?d.wind_speed.toFixed(1):'&mdash;')+'</td>';"
        "    html+='<td>'+(d.wind_dir>=0?d.wind_dir.toFixed(0):'&mdash;')+'</td>';"
        "    html+='<td>'+(d.rain>=0?d.rain.toFixed(1):'&mdash;')+'</td>';"
        "    html+='<td>'+(d.power>=0?d.power.toFixed(1):'&mdash;')+'</td>';"
        "    html+='<td>'+(d.energy>=0?d.energy.toFixed(2):'&mdash;')+'</td>';"
        "    html+='<td><span class=\"badge '+batCls+'\">'+batTxt+'</span></td>';"
        "    html+='<td>'+d.freq_mhz.toFixed(3)+'</td>';"
        "    html+='<td>'+d.rssi.toFixed(1)+'</td>';"
        "    html+='<td>'+d.msg_count+'</td>';"
        "    html+='<td><span class=\"badge '+ageCls+'\">'+d.age.toFixed(0)+'s</span></td>';"
        "    html+='</tr>';"
        "  });"
        "  html+='</tbody></table>';"
        "  document.getElementById('content').innerHTML=html;"
        "}"
        ""
        "function sortBy(key){"
        "  if(sortKey===key) sortDir=-sortDir;"
        "  else{sortKey=key;sortDir=1;}"
        "  if(devData.length) render({devices:devData});"
        "}"
        ""
        "load();"
        "setInterval(load,5000);"
        ""
        "fetch('/api/status').then(r=>r.json()).then(s=>{"
        "  var v=document.getElementById('ver-badge');"
        "  if(v&&s.version) v.textContent='v'+s.version;"
        "}).catch(()=>{});"
        "</script></div></body></html>";

    http_send(fd, 200, "text/html", html, (int)strlen(html));
}

static void serve_fanet_page(int fd)
{
    const char *html =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<title>FANET - dump1090-gg</title>"
        "<style>"
        ":root{--bg:#0a0a1a;--card:#141428;--head:#1a1a2e;--border:#2a2a4a;--accent:#4fc3f7;--text:#d0d0d0;--dim:#888;--hover:#1e1e3a;--danger:#ff4444;--warn:#ffaa00;--link:#44aaff;--ok:#00cc44;--input-bg:#0e0e22}"
        "body{background:var(--bg);color:var(--text);font-family:'Segoe UI',sans-serif;margin:0;padding:0}"
        "nav{background:var(--head);border-bottom:1px solid var(--border);padding:8px 16px;display:flex;align-items:center;gap:16px;position:sticky;top:0;z-index:100}"
        "nav h1{font-size:18px;color:var(--accent);white-space:nowrap;margin:0}"
        ".tabs{display:flex;gap:4px;flex-wrap:wrap;flex:1}"
        ".tabs a{padding:6px 14px;border-radius:6px;cursor:pointer;color:var(--dim);transition:.2s;font-size:13px;user-select:none;text-decoration:none}"
        ".tabs a:hover{background:var(--hover);color:var(--text)}"
        ".tabs a.active{background:var(--accent);color:#000;font-weight:600}"
        ".main{padding:20px;max-width:1600px;margin:0 auto}"
        "h2{color:var(--accent);margin-top:0}"
        ".card{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:16px;margin-bottom:12px}"
        ".stats{display:grid;grid-template-columns:repeat(auto-fill,minmax(180px,1fr));gap:12px;margin-bottom:16px}"
        ".stat{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:12px;text-align:center}"
        ".stat .val{font-size:24px;font-weight:700;color:var(--accent)}"
        ".stat .lbl{font-size:11px;color:var(--dim);margin-top:4px}"
        ".no-data{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:40px;text-align:center;color:var(--dim)}"
        ".no-data h3{color:var(--accent);margin-bottom:8px}"
        ".toolbar{display:flex;gap:8px;margin-bottom:12px;align-items:center}"
        ".btn{padding:6px 14px;border:1px solid var(--accent);border-radius:4px;cursor:pointer;font-size:13px;background:var(--head);color:var(--accent)}"
        ".btn:hover{background:var(--accent);color:#000}"
        "table{width:100%;border-collapse:collapse;font-size:13px;margin-top:8px}"
        "th{background:var(--head);color:var(--accent);padding:8px;text-align:left}"
        "td{padding:6px 8px;border-bottom:1px solid var(--border)}"
        "tr:hover td{background:var(--hover)}"
        "@media(max-width:768px){.stats{grid-template-columns:1fr 1fr}}"
        "</style></head><body>"
        "<nav><h1>&#x2708; dump1090-gg-light</h1><div class='tabs'>"
        "<a href='/'>&#x2699;&#xfe0f; Config</a>"
        "<a href='/status.html'>&#x1f4e1; Status</a>"
        "<a href='/connections.html'>&#x1f50c; Connections</a>"
        "<a href='/logs.html'>&#x1f4cb; Logs</a>"
        "<a href='/messages.html'>&#x1f4e8; Messages</a>"
        "<a href='/aircraft.html'>&#x2708;&#xfe0f; Aircraft</a>"
        "<a href='/devices.html'>&#x1f4fb; Devices</a>"
        "<a href='/gsm.html'>&#x1f4f6; GSM</a>"
        "<a href='/lte.html'>&#x1f4f6; LTE</a>"
        "<a href='/iot868.html'>&#x1f321;&#xfe0f; IoT 868</a>"
        "<a class='active' href='/fanet.html'>&#x1f6a9; FANET</a>"
        "<a style='margin-left:auto' href='/diagnostics.html'>&#x1f527; Diagnostics</a>"
        "</div>"
        "<span id='ver-badge' style='font-size:11px;color:var(--dim);white-space:nowrap;padding:2px 8px;border:1px solid var(--border);border-radius:10px' title='dump1090-gg version'>v&hellip;</span>"
        "</nav>"
        "<div class='main'>"
        "<h2>&#x1f6a9; FANET Monitor (868.2 MHz LoRa)</h2>"
        "<div class='toolbar'>"
        "<span id='status-badge' style='font-size:13px;font-weight:600'></span>"
        "<span id='update-time' style='color:var(--dim);margin-left:16px;font-size:12px'></span>"
        "<button class='btn' onclick='load()' style='margin-left:auto'>&#x21bb; Refresh</button>"
        "</div>"
        "<div id='content'><p style='color:var(--dim)'>Loading...</p></div>"

        "<script>"
        "function load(){"
        "  fetch('/api/config').then(r=>r.json()).then(cfg=>{"
        "    var fn=cfg.sdr_fanet||{};"
        "    if(!fn.active){"
        "      document.getElementById('content').innerHTML="
        "        '<div class=\"no-data\"><h3>&#x1f6a9; FANET Decoder Not Active</h3>'"
        "        +'<p>No SDR device is configured for FANET reception.</p>'"
        "        +'<p style=\"margin-top:12px\">Go to <a href=\"/devices.html\">Devices</a> and assign an RTL-SDR dongle to the <strong>FANET</strong> role.</p></div>';"
        "      document.getElementById('status-badge').innerHTML='<span style=\"color:var(--dim)\">&#x26aa; Not Active</span>';"
        "      return;"
        "    }"
        "    if(!fn.enabled){"
        "      document.getElementById('content').innerHTML="
        "        '<div class=\"no-data\"><h3>&#x1f6a9; FANET Decoder Disabled</h3>'"
        "        +'<p>The FANET decoder is currently disabled (output muted).</p>'"
        "        +'<p style=\"margin-top:12px\">Enable it from the <a href=\"/\">Config</a> page.</p></div>';"
        "      document.getElementById('status-badge').innerHTML='<span style=\"color:var(--warn)\">&#x1f7e1; Disabled</span>';"
        "      return;"
        "    }"
        "    document.getElementById('status-badge').innerHTML='<span style=\"color:var(--ok)\">&#x1f7e2; Active</span>';"
        "    fetch('/api/fanet').then(r=>r.json()).then(data=>render(data)).catch(()=>{"
        "      document.getElementById('content').innerHTML='<div class=\"no-data\"><h3>Error</h3><p>Failed to fetch FANET stats.</p></div>';"
        "    });"
        "  }).catch(()=>{});"
        "}"
        ""
        "function render(data){"
        "  document.getElementById('update-time').textContent='Updated: '+new Date().toLocaleTimeString();"
        "  var html='<div class=\"stats\">';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+data.preambles_detected+'</div><div class=\"lbl\">Preambles</div></div>';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+data.sync_word_ok+'</div><div class=\"lbl\">Sync OK</div></div>';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+data.packets_decoded+'</div><div class=\"lbl\">Decoded</div></div>';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+data.crc_errors+'</div><div class=\"lbl\">CRC Errors</div></div>';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+data.header_errors+'</div><div class=\"lbl\">Header Errors</div></div>';"
        "  html+='<div class=\"stat\"><div class=\"val\">'+(data.samples_processed/1e6).toFixed(1)+'M</div><div class=\"lbl\">Samples</div></div>';"
        "  html+='</div>';"
        "  if(data.packets_decoded>0 && data.type_counts){"
        "    var tc=data.type_counts;"
        "    html+='<div class=\"card\"><h3 style=\"color:var(--accent);margin-top:0\">Message Types</h3>';"
        "    html+='<div class=\"stats\">';"
        "    var types=[['Tracking',tc.tracking],['Name',tc.name],['Message',tc.message],"
        "      ['Weather',tc.service],['Ground',tc.ground],['Thermal',tc.thermal],"
        "      ['Landmark',tc.landmark],['HW Info',tc.hwinfo+tc.hwinfo2],"
        "      ['ACK',tc.ack],['Remote',tc.remote]];"
        "    for(var i=0;i<types.length;i++){"
        "      if(types[i][1]>0) html+='<div class=\"stat\"><div class=\"val\">'+types[i][1]+'</div><div class=\"lbl\">'+types[i][0]+'</div></div>';"
        "    }"
        "    html+='</div></div>';"
        "  }"
        "  if(data.packets_decoded===0){"
        "    html+='<div class=\"no-data\"><h3>Scanning...</h3><p>FANET decoder is active but no packets decoded yet.</p>'"
        "      +'<p style=\"color:var(--dim);font-size:12px;margin-top:8px\">Listening on 868.2 MHz for LoRa FANET+ signals from paragliders, drones, and weather stations.</p></div>';"
        "  }"
        "  if(data.ground_tracks && data.ground_tracks.length>0){"
        "    var gtypes=['Other','Walking','Vehicle','Bike','Boot','Need ride','Landed OK','Need tech','Need medical','DISTRESS','DISTRESS AUTO'];"
        "    html+='<div class=\"card\"><h3 style=\"color:var(--accent);margin-top:0\">&#x1f6b6; Ground Tracking</h3>';"
        "    html+='<table><thead><tr><th>Address</th><th>Name</th><th>Type</th><th>Lat</th><th>Lon</th><th>Age</th></tr></thead><tbody>';"
        "    for(var i=0;i<data.ground_tracks.length;i++){"
        "      var g=data.ground_tracks[i];"
        "      var tname=gtypes[g.type]||('Type '+g.type);"
        "      var age=g.age<60?g.age+'s':Math.floor(g.age/60)+'m';"
        "      html+='<tr><td style=\"font-family:monospace;color:var(--accent)\">'+g.addr+'</td>';"
        "      html+='<td><strong>'+(g.name||'—')+'</strong></td>';"
        "      html+='<td>'+tname+'</td>';"
        "      html+='<td>'+g.lat.toFixed(5)+'</td>';"
        "      html+='<td>'+g.lon.toFixed(5)+'</td>';"
        "      html+='<td>'+age+'</td></tr>';"
        "    }"
        "    html+='</tbody></table></div>';"
        "  }"
        "  document.getElementById('content').innerHTML=html;"
        "}"
        ""
        "load();"
        "setInterval(load,5000);"
        ""
        "fetch('/api/status').then(r=>r.json()).then(s=>{"
        "  var v=document.getElementById('ver-badge');"
        "  if(v&&s.version) v.textContent='v'+s.version;"
        "}).catch(()=>{});"
        "</script></div></body></html>";

    http_send(fd, 200, "text/html", html, (int)strlen(html));
}

static void serve_devices_page(int fd)
{
    const char *html =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<title>SDR Devices - dump1090-gg</title>"
        "<style>"
        ":root{--bg:#0a0a1a;--card:#141428;--head:#1a1a2e;--border:#2a2a4a;--accent:#4fc3f7;--text:#d0d0d0;--dim:#888;--hover:#1e1e3a;--danger:#ff4444;--warn:#ffaa00;--link:#44aaff;--ok:#00cc44;--input-bg:#0e0e22}"
        "body{background:var(--bg);color:var(--text);font-family:'Segoe UI',sans-serif;margin:0;padding:0}"
        "nav{background:var(--head);border-bottom:1px solid var(--border);padding:8px 16px;display:flex;align-items:center;gap:16px;position:sticky;top:0;z-index:100}"
        "nav h1{font-size:18px;color:var(--accent);white-space:nowrap;margin:0}"
        ".tabs{display:flex;gap:4px;flex-wrap:wrap;flex:1}"
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
        ".btn-run{background:#1a3a1a;color:#66bb6a;border-color:#66bb6a;padding:5px 8px}"
        ".btn-run:hover{background:#66bb6a;color:#0a0a1a}"
        ".btn-stop{background:#3a1a1a;color:#f44336;border-color:#f44336;padding:5px 8px}"
        ".btn-stop:hover{background:#f44336;color:#0a0a1a}"
        ".btn-auto{background:#2a1a0a;color:#ff9800;border-color:#ff9800}"
        ".btn-auto:hover{background:#ff9800;color:#0a0a1a}"
        "select{background:#1a1a2e;color:#e0e0e0;border:1px solid #444;padding:5px 8px;border-radius:4px;font-size:0.95em;min-width:100px}"
        "select:focus{border-color:#4fc3f7;outline:none}"
        "select.gain{background:#1a1a2e;color:#e0e0e0;border:1px solid #444;padding:5px 8px;border-radius:4px;min-width:70px;text-align:center}"
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
        "<nav><h1>&#x2708; dump1090-gg-light</h1><div class='tabs'>"
        "<a href='/'>&#x2699;&#xfe0f; Config</a>"
        "<a href='/status.html'>&#x1f4e1; Status</a>"
        "<a href='/connections.html'>&#x1f50c; Connections</a>"
        "<a href='/logs.html'>&#x1f4cb; Logs</a>"
        "<a href='/messages.html'>&#x1f4e8; Messages</a>"
        "<a href='/aircraft.html'>&#x2708;&#xfe0f; Aircraft</a>"
        "<a class='active' href='/devices.html'>&#x1f4fb; Devices</a>"
        "<a href='/gsm.html'>&#x1f4f6; GSM</a>"
        "<a href='/lte.html'>&#x1f4f6; LTE</a>"
        "<a href='/iot868.html'>&#x1f321;&#xfe0f; IoT 868</a>"
        "<a href='/fanet.html'>&#x1f6a9; FANET</a>"
        "<a style='margin-left:auto' href='/diagnostics.html'>&#x1f527; Diagnostics</a>"
        "</div>"
        "<span id='ver-badge' style='font-size:11px;color:var(--dim);white-space:nowrap;padding:2px 8px;border:1px solid var(--border);border-radius:10px' title='dump1090-gg version'>v&hellip;</span>"
        "</nav>"
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
        "el.style.display='block';"
        "setTimeout(function(){el.style.display='';el.className='status-msg';},5000);"
        "}"
        ""
        "function assign(serial,role,gainEl,ppmEl){"
        "var gain=gainEl?parseFloat(gainEl.value):999999;"
        "if(isNaN(gain))gain=999999;"
        "var ppm=ppmEl?parseInt(ppmEl.value)||0:0;"
        "var bSel=document.getElementById('be_'+serial);"
        "var be=bSel?bSel.value:'auto';"
        "var body=JSON.stringify({serial:serial,role:role,gain:gain,ppm:ppm,backend:be});"
        "fetch('/api/receivers/assign',{method:'POST',headers:{'Content-Type':'application/json'},body:body})"
        ".then(r=>r.json()).then(d=>{"
        "showStatus(d.message||'Done',d.ok);"
        "setTimeout(load,500);"
        "}).catch(e=>showStatus('Error: '+e,false));"
        "}"
        ""
        "function toggleRx(serial,running){"
        "var action=running?'stop':'start';"
        "fetch('/api/receivers/toggle',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({serial:serial,action:action})})"
        ".then(r=>r.json()).then(d=>{"
        "showStatus(d.message||'Done',d.ok);"
        "setTimeout(load,500);"
        "}).catch(e=>showStatus('Error: '+e,false));"
        "}"
        ""
        "function calibratePpm(serial){"
        "var btn=document.getElementById('cal_'+serial);"
        "var st=document.getElementById('calst_'+serial);"
        "if(btn)btn.disabled=true;"
        "if(st)st.textContent='';"
        // Show blocking overlay dialog
        "var ov=document.createElement('div');"
        "ov.id='cal-overlay';"
        "ov.style.cssText='position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.75);display:flex;align-items:center;justify-content:center;z-index:9999';"
        "var box=document.createElement('div');"
        "box.style.cssText='background:#1a1a2e;border:2px solid #4fc3f7;border-radius:12px;padding:30px 40px;text-align:center;max-width:420px;box-shadow:0 0 30px rgba(79,195,247,0.3)';"
        "box.innerHTML='<div style=\"font-size:2em;margin-bottom:12px\">&#x1f4e1;</div>'"
        "+'<div style=\"color:#4fc3f7;font-size:1.3em;font-weight:600;margin-bottom:10px\">GSM PPM Calibration</div>'"
        "+'<div style=\"color:#ccc;margin-bottom:6px\">Device: <code>'+serial+'</code></div>'"
        "+'<div id=\"cal-msg\" style=\"color:#ff9800;margin-top:12px;font-size:1.1em\">&#x23f3; Scanning GSM band (920-960 MHz)...</div>'"
        "+'<div style=\"color:#888;margin-top:8px;font-size:0.85em\">Receiver is stopped during calibration.<br>This takes ~20 seconds.</div>';"
        "ov.appendChild(box);"
        "document.body.appendChild(ov);"
        // Do the calibration request
        "fetch('/api/calibrate-ppm',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({serial:serial})})"
        ".then(r=>r.json()).then(d=>{"
        "var msg=document.getElementById('cal-msg');"
        "if(d.ok){"
        "var inp=document.getElementById('ppm_'+serial);"
        "if(inp)inp.value=d.ppm;"
        "var selEl=document.getElementById('sel_'+serial);"
        "var role=selEl?selEl.value:'adsb';"
        "var gainEl=document.getElementById('gain_'+serial);"
        "assign(serial,role,gainEl,inp);"
        "if(msg)msg.innerHTML='&#x2705; PPM calibrated: <b>'+d.ppm+'</b> (offset='+d.offset.toFixed(1)+', rms='+d.rms.toFixed(2)+', '+d.samples+' samples)';"
        "if(msg)msg.style.color='#66bb6a';"
        "if(st)st.textContent='PPM: '+d.ppm;"
        "}else{"
        "if(msg)msg.innerHTML='&#x274c; '+( d.error||'Calibration failed');"
        "if(msg)msg.style.color='#ff4444';"
        "if(st)st.textContent=d.error||'Failed';"
        "}"
        "if(btn)btn.disabled=false;"
        "setTimeout(function(){var o=document.getElementById('cal-overlay');if(o)o.remove();},4000);"
        "}).catch(e=>{"
        "var msg=document.getElementById('cal-msg');"
        "if(msg){msg.innerHTML='&#x274c; Error: '+e;msg.style.color='#ff4444';}"
        "if(btn)btn.disabled=false;"
        "if(st)st.textContent='Error';"
        "setTimeout(function(){var o=document.getElementById('cal-overlay');if(o)o.remove();},3000);"
        "});"
        "}"
        ""
        "var _agRunning=false;"
        "function _rxNoise(stats,serial){"
        "if(!stats.rx_noise)return null;"
        "for(var i=0;i<stats.rx_noise.length;i++)"
        "if(stats.rx_noise[i].serial==serial)return stats.rx_noise[i];"
        "return null;}"
        ""
        "async function autoGain(serial){"
        "if(_agRunning){showStatus('Auto-gain already running',false);return;}"
        "var rx=getRxForSerial(serial);"
        "if(!rx||!rx.gain_list||rx.gain_list.length<2){showStatus('No gain table for '+serial,false);return;}"
        "_agRunning=true;"
        "var isAdsb=(rx.role=='adsb');"
        "var btn=document.getElementById('ag_'+serial);"
        "if(btn)btn.disabled=true;"
        "var steps=rx.gain_list.length;"
        "var results=[];"
        "var bestStep=0,bestScore=-999;"
        "showStatus('Auto-gain: testing '+steps+' gain steps for '+serial+'... ('+(isAdsb?'msg count':'noise floor')+')',true);"
        ""
        "for(var i=0;i<steps;i++){"
        "if(btn)btn.textContent='\\u23f3 '+(i+1)+'/'+steps;"
        "try{"
        "await fetch('/api/receivers/setgain',{method:'POST',headers:{'Content-Type':'application/json'},"
        "body:JSON.stringify({serial:serial,step:i})});"
        "var s1=await fetch('/api/stats/quick').then(r=>r.json());"
        "await new Promise(r=>setTimeout(r,2000));"
        "var s2=await fetch('/api/stats/quick').then(r=>r.json());"
        ""
        "if(isAdsb){"
        "var msgs=s2.demod_total-s1.demod_total;"
        "results.push({step:i,gain:rx.gain_list[i],msgs:msgs,noise:0});"
        "if(msgs>bestScore){bestScore=msgs;bestStep=i;}"
        "}else{"
        "var n1=_rxNoise(s1,rx.serial_actual||serial);"
        "var n2=_rxNoise(s2,rx.serial_actual||serial);"
        "var noiseDb=-99;"
        "if(n1&&n2&&(n2.iq_count-n1.iq_count)>0){"
        "var dSum=n2.iq_sum-n1.iq_sum;"
        "var dCnt=n2.iq_count-n1.iq_count;"
        "var pwr=dSum/dCnt;"
        "noiseDb=10*Math.log10(pwr/32768);"
        "}"
        "results.push({step:i,gain:rx.gain_list[i],msgs:0,noise:noiseDb});"
        "if(noiseDb<-8&&i>=bestStep){bestScore=noiseDb;bestStep=i;}"
        "}"
        "}catch(e){results.push({step:i,gain:rx.gain_list[i],msgs:-1,noise:-99});}"
        "}"
        ""
        "await fetch('/api/receivers/setgain',{method:'POST',headers:{'Content-Type':'application/json'},"
        "body:JSON.stringify({serial:serial,step:bestStep})});"
        ""
        "var gainSel=document.getElementById('gain_'+serial);"
        "if(gainSel){"
        "for(var j=0;j<gainSel.options.length;j++){"
        "if(Math.abs(parseFloat(gainSel.options[j].value)-rx.gain_list[bestStep])<0.05){"
        "gainSel.selectedIndex=j;break;}}"
        "}"
        ""
        "var summary='Best: '+rx.gain_list[bestStep].toFixed(1)+' dB. ';"
        "results.forEach(function(r){"
        "summary+=r.gain.toFixed(1)+'=';"
        "if(isAdsb)summary+=r.msgs+'msg ';"
        "else summary+=r.noise.toFixed(1)+'dB ';"
        "});"
        "var el=document.getElementById('status');"
        "el.textContent=summary;el.className='status-msg status-ok';"
        "el.style.display='block';"
        ""
        "var selEl=document.getElementById('sel_'+serial);"
        "var role=selEl?selEl.value:'adsb';"
        "var ppmEl=document.getElementById('ppm_'+serial);"
        "assign(serial,role,gainSel,ppmEl);"
        "if(btn){btn.textContent='\\ud83d\\udd0d';btn.disabled=false;}"
        "_agRunning=false;"
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
        "h+='<table><tr><th>#</th><th>Device</th><th>Serial</th><th>Tuner</th>';"
        "h+='<th>Library</th>';"
        "var multiBackend=(rxData&&rxData.backends_available&&(rxData.backends_available&(rxData.backends_available-1))!=0);"
        "h+='<th>Assign Role</th><th>Gain (dB)</th><th>PPM</th><th>Action</th><th>Status</th></tr>';"
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
        // Backend dropdown (always shown; selectable only if multiple backends available)
        "var curBe=rx?rx.backend:'sdrgg';"
        "if(multiBackend){"
        "h+='<td><select id=\"be_'+s.serial+'\">';"
        "h+='<option value=rtlsdr'+(curBe=='rtlsdr'?' selected':'')+'>librtlsdr</option>';"
        "h+='<option value=sdrgg'+(curBe=='sdrgg'?' selected':'')+'>libsdrgg</option>';"
        "h+='</select></td>';"
        "}else{"
        "h+='<td><span style=color:var(--dim)>librtlsdr</span></td>';"
        "}"
        // Role Dropdown
        "h+='<td><select id=\"'+selId+'\">';"
        "h+='<option value=none'+(curRole=='none'?' selected':'')+'>&#x274c; None</option>';"
        "h+='<option value=adsb'+(curRole=='adsb'?' selected':'')+'>&#x2708; ADS-B (1090 MHz)</option>';"
        "h+='<option value=flarm'+(curRole=='flarm'?' selected':'')+'>&#x1f6a9; FLARM / OGNTP / ADS-L / P3I (868 MHz)</option>';"
        "h+='<option value=acars'+(curRole=='acars'?' selected':'')+'>&#x1f4e1; ACARS (131 MHz)</option>';"
        "h+='<option value=vdl2'+(curRole=='vdl2'?' selected':'')+'>&#x1f4e1; VDL2 (136 MHz)</option>';"
        "h+='<option value=radiosonde'+(curRole=='radiosonde'?' selected':'')+'>&#x1f388; Radiosonde (403 MHz)</option>';"
        "h+='<option value=pocsag'+(curRole=='pocsag'?' selected':'')+'>&#x1f4df; POCSAG (466 MHz)</option>';"
        "h+='<option value=gsm'+(curRole=='gsm'?' selected':'')+'>&#x1f4f6; GSM (935 MHz)</option>';"
        "h+='<option value=lte'+(curRole=='lte'?' selected':'')+'>&#x1f4f6; LTE (800 MHz)</option>';"
        "h+='<option value=iot868'+(curRole=='iot868'?' selected':'')+'>&#x1f321;&#xfe0f; IoT 868 MHz</option>';"
        "h+='<option value=fanet'+(curRole=='fanet'?' selected':'')+'>&#x1f6a9; FANET (868.2 MHz)</option>';"
        "h+='<option value=sarsat'+(curRole=='sarsat'?' selected':'')+'>&#x1f6a8; Sarsat ELT (406 MHz)</option>';"
        "h+='</select></td>';"
        // Gain dropdown - populated from receiver's gain_list
        "var gainOpts='';"
        "var gList=(rx&&rx.gain_list&&rx.gain_list.length>0)?rx.gain_list:null;"
        "if(gList){"
        "gList.forEach(function(g){"
        "var sel=(Math.abs(g-curGain)<0.05)?' selected':'';"
        "gainOpts+='<option value=\"'+g.toFixed(1)+'\"'+sel+'>'+g.toFixed(1)+'</option>';"
        "});"
        "}else{"
        "gainOpts='<option value=\"'+curGain.toFixed(1)+'\">'+curGain.toFixed(1)+'</option>';"
        "}"
        "h+='<td style=\"white-space:nowrap\"><select class=gain id=\"'+gainId+'\">'+gainOpts+'</select>';"
        "if(rx&&s.state=='running'){"
        "h+=' <button id=\"ag_'+s.serial+'\" class=\"btn btn-apply\" style=\"border-color:#ff9800;color:#ff9800;padding:5px 8px\" onclick=\"autoGain(\\''+s.serial+'\\')\" title=\"Sweep all gain steps and pick the best\">&#x1f50d;</button>';"
        "}"
        "h+='</td>';"
        // PPM input + Calibrate button
        "var curPpm=rx?rx.ppm:0;"
        "var ppmId='ppm_'+s.serial;"
        "h+='<td style=\"white-space:nowrap\"><input type=number step=1 value=\"'+curPpm+'\" id=\"'+ppmId+'\" style=\"width:60px\">';"
        "h+=' <button id=\"cal_'+s.serial+'\" class=\"btn btn-apply\" style=\"border-color:var(--accent);color:var(--accent);padding:5px 8px\" onclick=\"calibratePpm(\\''+s.serial+'\\')\" title=\"GSM calibrate PPM\">&#x1f4e1;</button>';"
        "h+='<div id=\"calst_'+s.serial+'\" style=\"font-size:10px;color:#888;margin-top:2px\"></div></td>';"
        // Apply + Run/Stop buttons
        "h+='<td style=\"white-space:nowrap\">';"
        "h+='<button class=\"btn btn-apply\" onclick=\"assign(\\''+s.serial+'\\',document.getElementById(\\''+selId+'\\').value,document.getElementById(\\''+gainId+'\\'),document.getElementById(\\''+ppmId+'\\'))\">&#x2714; Apply</button> ';"
        "if(s.state=='running'){"
        "h+='<button class=\"btn btn-stop\" onclick=\"toggleRx(\\''+s.serial+'\\',true)\" title=\"Stop receiver\">&#x23f9; Stop</button>';"
        "}else if(rx){"
        "h+='<button class=\"btn btn-run\" onclick=\"toggleRx(\\''+s.serial+'\\',false)\" title=\"Start receiver\">&#x25b6; Run</button>';"
        "}"
        "h+='</td>';"
        "h+='<td>'+stateHtml+'</td>';"
        "h+='</tr>';"
        "});"
        "h+='</table>';"
        "}"
        ""
        // Role descriptions info box
        "h+='<div style=\"margin:16px 0;padding:14px;background:#111122;border:1px solid #333;border-radius:6px;font-size:0.9em;line-height:1.7\">';"
        "h+='<strong style=\"color:#4fc3f7\">&#x1f4e1; Decoder Roles:</strong><br>';"
        "h+='<b>&#x2708; ADS-B</b> &mdash; Mode S / ADS-B aircraft surveillance (1090 MHz)<br>';"
        "h+='<b>&#x1f6a9; FLARM / OGNTP / ADS-L / P3I</b> &mdash; 868 MHz EC band multi-protocol:<br>';"
        "h+='&nbsp;&nbsp;&bull; <b>FLARM</b> V6/V7: collision-avoidance for gliders/GA<br>';"
        "h+='&nbsp;&nbsp;&bull; <b>OGN-TP</b>: Open Glider Network tracking protocol<br>';"
        "h+='&nbsp;&nbsp;&bull; <b>ADS-L</b> (EASA): lightweight ADS-B for drones/UAS (868.2/868.4 MHz, Manchester encoded)<br>';"
        "h+='&nbsp;&nbsp;&bull; <b>P3I</b> (PilotAware): GA anti-collision (869.525 MHz, FSK 38.4 kbps)<br>';"
        "h+='<b>&#x1f4e1; ACARS</b> &mdash; Aircraft VHF datalink (131 MHz)<br>';"
        "h+='<b>&#x1f4e1; VDL2</b> &mdash; VHF Data Link Mode 2, D8PSK (136 MHz)<br>';"
        "h+='<b>&#x1f388; Radiosonde</b> &mdash; Weather sonde decoding: RS41, DFM, M10 (403 MHz)<br>';"
        "h+='<b>&#x1f4df; POCSAG</b> &mdash; Pager decoding, multi-channel 512/1200/2400 baud (466 MHz)<br>';"
        "h+='<b>&#x1f4f6; GSM</b> &mdash; GSM downlink SCH/BCCH for PPM calibration (935 MHz)<br>';"
        "h+='<b>&#x1f4f6; LTE</b> &mdash; LTE Band 20 MIB/SIB decode (800 MHz)<br>';"
        "h+='<b>&#x1f321;&#xfe0f; IoT 868</b> &mdash; ISM OOK/FSK: Bresser, LaCrosse, Honeywell (868 MHz)<br>';"
        "h+='<b>&#x1f6a9; FANET</b> &mdash; LoRa CSS paraglider network, SF7 BW250k (868.2 MHz)<br>';"
        "h+='<b>&#x1f6f0;&#xfe0f; SARSAT</b> &mdash; COSPAS-SARSAT 406 MHz emergency beacon decoder<br>';"
        "h+='</div>';"
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
        "h+='<tr><td>PPM</td><td>'+r.ppm+'</td></tr>';"
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
        "fetch('/api/status').then(r=>r.json()).then(d=>{var vb=document.getElementById('ver-badge');if(vb&&d.version)vb.textContent='v'+d.version;}).catch(function(){});"
        "load();"
        "</script></div></body></html>";

    http_send(fd, 200, "text/html; charset=utf-8", html, (int)strlen(html));
}

// ============================= SDR Diagnostics Page ============================

static void serve_diagnostics_page(int fd)
{
    const char *html =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<title>SDR Diagnostics - dump1090-gg</title>"
        "<style>"
        ":root{--bg:#0a0a1a;--card:#141428;--head:#1a1a2e;--border:#2a2a4a;--accent:#4fc3f7;--text:#d0d0d0;--dim:#888;--hover:#1e1e3a;--danger:#ff4444;--warn:#ffaa00;--link:#44aaff;--ok:#00cc44;--input-bg:#0e0e22}"
        "body{background:var(--bg);color:var(--text);font-family:'Segoe UI',sans-serif;margin:0;padding:0}"
        "nav{background:var(--head);border-bottom:1px solid var(--border);padding:8px 16px;display:flex;align-items:center;gap:16px;position:sticky;top:0;z-index:100}"
        "nav h1{font-size:18px;color:var(--accent);white-space:nowrap;margin:0}"
        ".tabs{display:flex;gap:4px;flex-wrap:wrap;flex:1}"
        ".tabs a{padding:6px 14px;border-radius:6px;cursor:pointer;color:var(--dim);transition:.2s;font-size:13px;user-select:none;text-decoration:none}"
        ".tabs a:hover{background:var(--hover);color:var(--text)}"
        ".tabs a.active{background:var(--accent);color:#000;font-weight:600}"
        ".main{padding:20px;max-width:1400px;margin:0 auto}"
        "h2{color:var(--accent);margin-top:20px}"
        ".info-box{background:#111122;border:1px solid #333;border-radius:6px;padding:12px;margin:10px 0}"
        ".btn{border:1px solid var(--accent);padding:8px 20px;cursor:pointer;border-radius:4px;font-size:1em;margin:4px}"
        ".btn-start{background:#1a3a1a;color:#66bb6a;border-color:#66bb6a;font-size:1.1em;padding:10px 24px}"
        ".btn-start:hover{background:#66bb6a;color:#0a0a1a}"
        ".btn-start:disabled{opacity:0.4;cursor:not-allowed}"
        ".status-run{color:#ff9800;font-weight:bold}"
        ".status-done{color:#66bb6a;font-weight:bold}"
        ".status-idle{color:#888}"
        ".dev-card{background:#111122;border:1px solid #333;border-radius:8px;padding:16px;margin:16px 0}"
        ".dev-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px}"
        ".dev-title{color:var(--accent);font-size:1.2em;font-weight:600}"
        ".dev-info{color:#aaa;font-size:0.9em}"
        ".chart-container{position:relative;width:100%;height:340px;background:#0a0a1a;border:1px solid #333;border-radius:4px;margin:10px 0;overflow:hidden}"
        "canvas{width:100%;height:100%}"
        ".legend{display:flex;gap:16px;flex-wrap:wrap;font-size:0.85em;color:#aaa;margin:8px 0}"
        ".legend-item{display:flex;align-items:center;gap:4px}"
        ".legend-dot{width:10px;height:10px;border-radius:50%}"
        ".metric-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(180px,1fr));gap:8px;margin:10px 0}"
        ".metric{background:#0e0e22;border:1px solid #2a2a4a;border-radius:4px;padding:8px;text-align:center}"
        ".metric-val{font-size:1.4em;font-weight:bold;color:var(--accent)}"
        ".metric-label{font-size:0.8em;color:#888}"
        ".pll-ok{color:#66bb6a}"
        ".pll-fail{color:#ff4444}"
        "</style></head><body>"
        "<nav><h1>&#x2708; dump1090-gg-light</h1><div class='tabs'>"
        "<a href='/'>&#x2699;&#xfe0f; Config</a>"
        "<a href='/status.html'>&#x1f4e1; Status</a>"
        "<a href='/connections.html'>&#x1f50c; Connections</a>"
        "<a href='/logs.html'>&#x1f4cb; Logs</a>"
        "<a href='/messages.html'>&#x1f4e8; Messages</a>"
        "<a href='/aircraft.html'>&#x2708;&#xfe0f; Aircraft</a>"
        "<a href='/devices.html'>&#x1f4fb; Devices</a>"
        "<a href='/gsm.html'>&#x1f4f6; GSM</a>"
        "<a href='/lte.html'>&#x1f4f6; LTE</a>"
        "<a href='/iot868.html'>&#x1f321;&#xfe0f; IoT 868</a>"
        "<a href='/fanet.html'>&#x1f6a9; FANET</a>"
        "<a class='active' style='margin-left:auto' href='/diagnostics.html'>&#x1f527; Diagnostics</a>"
        "</div>"
        "<span id='ver-badge' style='font-size:11px;color:var(--dim);white-space:nowrap;padding:2px 8px;border:1px solid var(--border);border-radius:10px' title='dump1090-gg version'>v&hellip;</span>"
        "</nav>"
        "<div class='main'>"
        "<h2>&#x1f527; SDR Dongle Diagnostics</h2>"
        "<p style='color:#888'>Test PLL lock, noise floor, and IQ quality across frequencies and sample rates for each connected RTL-SDR dongle.</p>"
        "<div class='info-box'>"
        "<strong>&#x26a0;&#xfe0f; Note:</strong> Running diagnostics will <em>temporarily stop</em> active receivers. "
        "They will be automatically restarted after the test completes."
        "</div>"
        "<div style='margin:16px 0'>"
        "<button id='btn-start' class='btn btn-start' onclick='startDiag()'>&#x25b6; Start Diagnostics</button>"
        "<span id='status-text' class='status-idle' style='margin-left:16px'>Idle</span>"
        "</div>"
        "<div id='lib-info'></div>"
        "<div id='results'></div>"
        "<script>"
        "var pollTimer=null;"
        ""
        "function startDiag(){"
        "document.getElementById('btn-start').disabled=true;"
        "document.getElementById('status-text').textContent='Starting...';"
        "document.getElementById('status-text').className='status-run';"
        "document.getElementById('results').innerHTML='';"
        "fetch('/api/diagnostics/start',{method:'POST'})"
        ".then(r=>r.json()).then(d=>{"
        "if(d.ok){pollTimer=setInterval(pollResults,1500);}"
        "else{document.getElementById('status-text').textContent='Error: '+d.error;"
        "document.getElementById('btn-start').disabled=false;}"
        "}).catch(e=>{document.getElementById('status-text').textContent='Error: '+e;"
        "document.getElementById('btn-start').disabled=false;});"
        "}"
        ""
        "function pollResults(){"
        "fetch('/api/diagnostics').then(r=>r.json()).then(d=>{"
        "renderResults(d);"
        "if(d.complete){"
        "clearInterval(pollTimer);pollTimer=null;"
        "document.getElementById('btn-start').disabled=false;"
        "document.getElementById('status-text').textContent='Complete';"
        "document.getElementById('status-text').className='status-done';"
        "}else{"
        "document.getElementById('status-text').textContent='Running... ('+d.device_count+' devices)';"
        "document.getElementById('status-text').className='status-run';"
        "}"
        "}).catch(e=>{});"
        "}"
        ""
        "function renderResults(d){"
        "var libHtml='<div class=info-box><strong>Library:</strong> '+d.librtlsdr_version+'</div>';"
        "document.getElementById('lib-info').innerHTML=libHtml;"
        ""
        "var h='';"
        "d.devices.forEach(function(dev,idx){"
        "h+='<div class=dev-card>';"
        "h+='<div class=dev-header>';"
        "h+='<div><span class=dev-title>'+dev.serial+'</span> &mdash; '+dev.tuner+'</div>';"
        "h+='<div class=dev-info>Range: '+dev.freq_range+' | Gain: '+(dev.gain_list&&dev.gain_list.length?dev.gain_list[0].toFixed(1)+'~'+dev.gain_list[dev.gain_list.length-1].toFixed(1)+' dB ('+dev.gain_list.length+' steps)':dev.max_gain_db.toFixed(1)+' dB')+'</div>';"
        "h+='</div>';"
        ""
        "if(dev.error){h+='<p style=color:var(--danger)>Error: '+dev.error+'</p>';}"
        "else if(dev.measurements.length==0){h+='<p style=color:var(--dim)>Waiting...</p>';}"
        "else{"
        // Metrics summary
        "var pllOk=0,pllFail=0;"
        "dev.measurements.forEach(function(m){if(m.pll)pllOk++;else pllFail++;});"
        "h+='<div class=metric-grid>';"
        "h+='<div class=metric><div class=\"metric-val pll-ok\">'+pllOk+'</div><div class=metric-label>PLL Locked</div></div>';"
        "h+='<div class=metric><div class=\"metric-val pll-fail\">'+pllFail+'</div><div class=metric-label>PLL Failed</div></div>';"
        "h+='<div class=metric><div class=metric-val>'+dev.max_gain_db.toFixed(1)+'</div><div class=metric-label>Max Gain (dB)</div></div>';"
        "h+='</div>';"
        // Gain list
        "if(dev.gain_list&&dev.gain_list.length>0){"
        "h+='<div style=\"margin:8px 0;padding:8px;background:#0e0e22;border:1px solid #2a2a4a;border-radius:4px\">';"
        "h+='<span style=\"color:var(--accent);font-size:0.85em;font-weight:600\">Supported gains ('+dev.gain_list.length+' steps): </span>';"
        "h+='<span style=\"color:#aaa;font-size:0.82em\">';"
        "h+=dev.gain_list.map(function(g){return g.toFixed(1)+' dB'}).join(', ');"
        "h+='</span></div>';}"
        // Chart
        "h+='<div class=chart-container><canvas id=chart'+idx+'></canvas></div>';"
        "h+='<div class=legend>';"
        "h+='<div class=legend-item><div class=legend-dot style=background:#66bb6a></div>PLL Locked</div>';"
        "h+='<div class=legend-item><div class=legend-dot style=background:#ff4444></div>PLL Failed</div>';"
        "h+='<div class=legend-item><div class=legend-dot style=background:#4fc3f7></div>IQ Spread</div>';"
        "h+='<div class=legend-item><div class=legend-dot style=background:#ff9800></div>Noise Floor (dB)</div>';"
        "h+='</div>';"
        // Table
        "h+='<details><summary style=cursor:pointer;color:var(--accent)>Detailed measurements ('+dev.measurements.length+')</summary>';"
        "h+='<table style=font-size:0.85em;margin-top:8px><tr><th>Freq (MHz)</th><th>SR (kHz)</th><th>PLL</th><th>Noise (dB)</th><th>DC Offset</th><th>IQ Spread</th></tr>';"
        "dev.measurements.forEach(function(m){"
        "var pllC=m.pll?'pll-ok':'pll-fail';"
        "h+='<tr><td>'+(m.freq/1e6).toFixed(1)+'</td><td>'+(m.sr/1000)+'</td>';"
        "h+='<td class='+pllC+'>'+(m.pll?'OK':'FAIL')+'</td>';"
        "h+='<td>'+m.noise.toFixed(1)+'</td><td>'+m.dc_offset.toFixed(2)+'</td><td>'+m.iq_spread.toFixed(2)+'</td></tr>';"
        "});"
        "h+='</table></details>';"
        "}"
        "h+='</div>';"
        "});"
        "document.getElementById('results').innerHTML=h;"
        ""
        // Draw charts
        "setTimeout(function(){d.devices.forEach(function(dev,idx){drawChart(idx,dev);});},50);"
        "}"
        ""
        "function drawChart(idx,dev){"
        "var canvas=document.getElementById('chart'+idx);"
        "if(!canvas||dev.measurements.length==0)return;"
        "var ctx=canvas.getContext('2d');"
        "var W=canvas.parentElement.clientWidth;"
        "var H=canvas.parentElement.clientHeight;"
        "canvas.width=W;canvas.height=H;"
        "var pad={t:36,r:70,b:52,l:70};"
        "var cw=W-pad.l-pad.r;"
        "var ch=H-pad.t-pad.b;"
        "var n=dev.measurements.length;"
        ""
        // Background
        "ctx.fillStyle='#0a0a1a';ctx.fillRect(0,0,W,H);"
        ""
        // Find section boundaries (19 freq sweep, then SR sweeps)
        "var secFreq=Math.min(19,n);"
        "var secSR1=Math.min(secFreq+6,n);"
        ""
        // Find ranges
        "var maxSpread=0,minNoise=0,maxNoise=-99;"
        "dev.measurements.forEach(function(m){"
        "if(m.iq_spread>maxSpread)maxSpread=m.iq_spread;"
        "if(m.noise>maxNoise)maxNoise=m.noise;"
        "if(m.noise<minNoise)minNoise=m.noise;"
        "});"
        "if(maxSpread<10)maxSpread=10;"
        "if(maxNoise-minNoise<1){minNoise=-50;maxNoise=0;}"
        ""
        // Grid lines + Y-axis scale
        "ctx.strokeStyle='#1a1a2e';ctx.lineWidth=1;"
        "ctx.font='10px sans-serif';"
        "for(var i=0;i<=4;i++){"
        "var y=pad.t+ch*i/4;"
        "ctx.beginPath();ctx.moveTo(pad.l,y);ctx.lineTo(W-pad.r,y);ctx.stroke();"
        // Left Y axis: IQ Spread
        "ctx.fillStyle='#4fc3f7';ctx.textAlign='right';"
        "var sv=maxSpread*(1-i/4);"
        "ctx.fillText(sv.toFixed(0),pad.l-6,y+4);"
        // Right Y axis: Noise dB
        "var nRange=maxNoise-minNoise;"
        "var nv=maxNoise-nRange*i/4;"
        "ctx.fillStyle='#ff9800';ctx.textAlign='left';"
        "ctx.fillText(nv.toFixed(0)+' dB',W-pad.r+6,y+4);"
        "}"
        ""
        // Section separators (vertical dashed lines)
        "ctx.setLineDash([4,3]);ctx.strokeStyle='#444';ctx.lineWidth=1;"
        "if(secFreq<n){"
        "var sx=pad.l+secFreq*(cw/n);"
        "ctx.beginPath();ctx.moveTo(sx,pad.t);ctx.lineTo(sx,pad.t+ch);ctx.stroke();"
        "}"
        "if(secSR1<n){"
        "var sx2=pad.l+secSR1*(cw/n);"
        "ctx.beginPath();ctx.moveTo(sx2,pad.t);ctx.lineTo(sx2,pad.t+ch);ctx.stroke();"
        "}"
        "ctx.setLineDash([]);"
        ""
        // Section titles at top
        "ctx.font='bold 11px sans-serif';ctx.textAlign='center';"
        "ctx.fillStyle='#aaa';"
        "if(secFreq>0)ctx.fillText('Frequency Sweep (2.4 MSPS)',pad.l+secFreq*(cw/n)/2,pad.t-8);"
        "if(secSR1>secFreq)ctx.fillText('SR @404.5',pad.l+(secFreq+secSR1)/2*(cw/n),pad.t-8);"
        "if(n>secSR1)ctx.fillText('SR @1090',pad.l+(secSR1+n)/2*(cw/n),pad.t-8);"
        ""
        // PLL status bars (background color per measurement)
        "var barW=Math.max(2,cw/n-2);"
        "dev.measurements.forEach(function(m,i){"
        "var x=pad.l+i*(cw/n)+1;"
        "ctx.fillStyle=m.pll?'rgba(102,187,106,0.15)':'rgba(255,68,68,0.25)';"
        "ctx.fillRect(x,pad.t,barW,ch);"
        "});"
        ""
        // PLL status dots at bottom
        "dev.measurements.forEach(function(m,i){"
        "var x=pad.l+i*(cw/n)+barW/2;"
        "ctx.beginPath();ctx.arc(x,pad.t+ch+8,3,0,6.28);"
        "ctx.fillStyle=m.pll?'#66bb6a':'#ff4444';ctx.fill();"
        "});"
        ""
        // IQ Spread line (cyan) — higher = tuner is receiving signal
        "ctx.strokeStyle='#4fc3f7';ctx.lineWidth=2.5;ctx.beginPath();"
        "dev.measurements.forEach(function(m,i){"
        "var x=pad.l+i*(cw/n)+barW/2;"
        "var y=pad.t+ch-(m.iq_spread/maxSpread)*ch;"
        "if(i==0)ctx.moveTo(x,y);else ctx.lineTo(x,y);"
        "});ctx.stroke();"
        ""
        // IQ Spread threshold line (spread < 3 = deaf)
        "var threshY=pad.t+ch-(3.0/maxSpread)*ch;"
        "if(threshY<pad.t+ch&&threshY>pad.t){"
        "ctx.strokeStyle='rgba(255,68,68,0.6)';ctx.lineWidth=1;ctx.setLineDash([6,4]);"
        "ctx.beginPath();ctx.moveTo(pad.l,threshY);ctx.lineTo(W-pad.r,threshY);ctx.stroke();"
        "ctx.setLineDash([]);"
        "ctx.fillStyle='#ff6666';ctx.font='9px sans-serif';ctx.textAlign='left';"
        "ctx.fillText('DEAF threshold (IQ<3)',pad.l+4,threshY-4);"
        "}"
        ""
        // Noise floor line (orange) — higher = more signal/noise received
        "ctx.strokeStyle='#ff9800';ctx.lineWidth=2;ctx.setLineDash([5,3]);ctx.beginPath();"
        "dev.measurements.forEach(function(m,i){"
        "var x=pad.l+i*(cw/n)+barW/2;"
        "var nRange=maxNoise-minNoise;"
        "var y=pad.t+ch-((m.noise-minNoise)/nRange)*ch;"
        "if(i==0)ctx.moveTo(x,y);else ctx.lineTo(x,y);"
        "});ctx.stroke();ctx.setLineDash([]);"
        ""
        // X-axis labels
        "ctx.fillStyle='#888';ctx.font='9px sans-serif';ctx.textAlign='center';"
        "for(var i=0;i<n;i++){"
        "var m=dev.measurements[i];"
        "var x=pad.l+i*(cw/n)+barW/2;"
        "var label;"
        "if(i<secFreq){label=(m.freq/1e6).toFixed(0);}"
        "else{label=(m.sr/1e6).toFixed(1)+'M';}"
        // Show every label for SR, skip some for freq
        "if(i<secFreq&&n>15){if(i%2!=0&&i!=secFreq-1)continue;}"
        "ctx.save();ctx.translate(x,pad.t+ch+20);"
        "if(i<secFreq)ctx.rotate(-0.5);"
        "ctx.fillText(label,0,0);ctx.restore();"
        "}"
        ""
        // X-axis unit labels
        "ctx.font='10px sans-serif';ctx.fillStyle='#888';ctx.textAlign='center';"
        "if(secFreq>0)ctx.fillText('MHz',pad.l+secFreq*(cw/n)/2,H-4);"
        "if(secSR1>secFreq)ctx.fillText('MSPS',pad.l+(secFreq+secSR1)/2*(cw/n),H-4);"
        "if(n>secSR1)ctx.fillText('MSPS',pad.l+(secSR1+n)/2*(cw/n),H-4);"
        ""
        // Y-axis titles (rotated)
        "ctx.save();ctx.translate(12,pad.t+ch/2);ctx.rotate(-Math.PI/2);"
        "ctx.fillStyle='#4fc3f7';ctx.font='bold 10px sans-serif';ctx.textAlign='center';"
        "ctx.fillText('IQ Spread (signal strength)',0,0);ctx.restore();"
        "ctx.save();ctx.translate(W-8,pad.t+ch/2);ctx.rotate(Math.PI/2);"
        "ctx.fillStyle='#ff9800';ctx.font='bold 10px sans-serif';ctx.textAlign='center';"
        "ctx.fillText('Noise Floor (dB)',0,0);ctx.restore();"
        ""
        // Inline legend box
        "ctx.fillStyle='rgba(10,10,26,0.85)';ctx.fillRect(pad.l+8,pad.t+4,220,52);ctx.strokeStyle='#333';ctx.strokeRect(pad.l+8,pad.t+4,220,52);"
        "ctx.font='10px sans-serif';var lx=pad.l+16,ly=pad.t+18;"
        "ctx.fillStyle='#4fc3f7';ctx.fillRect(lx,ly-4,14,3);ctx.fillText('IQ Spread (solid) \u2014 tuner sensitivity',lx+20,ly);"
        "ctx.fillStyle='#ff9800';ctx.setLineDash([4,2]);ctx.strokeStyle='#ff9800';ctx.beginPath();ctx.moveTo(lx,ly+12);ctx.lineTo(lx+14,ly+12);ctx.stroke();ctx.setLineDash([]);"
        "ctx.fillText('Noise Floor (dashed) \u2014 signal received dB',lx+20,ly+14);"
        "ctx.beginPath();ctx.arc(lx+4,ly+27,3,0,6.28);ctx.fillStyle='#66bb6a';ctx.fill();"
        "ctx.fillStyle='#aaa';ctx.fillText('= PLL locked (tuner working)',lx+20,ly+30);"
        "ctx.beginPath();ctx.arc(lx+4,ly+41,3,0,6.28);ctx.fillStyle='#ff4444';ctx.fill();"
        "ctx.fillStyle='#aaa';ctx.fillText('= PLL FAIL (tuner deaf, no signal)',lx+20,ly+44);"
        "}"
        ""
        // Auto-load previous results if available
        "fetch('/api/diagnostics').then(r=>r.json()).then(d=>{"
        "if(d.complete&&d.device_count>0)renderResults(d);"
        "if(d.running){pollTimer=setInterval(pollResults,1500);"
        "document.getElementById('btn-start').disabled=true;"
        "document.getElementById('status-text').textContent='Running...';"
        "document.getElementById('status-text').className='status-run';}"
        "}).catch(function(){});"
        "</script></div></body></html>";

    http_send(fd, 200, "text/html; charset=utf-8", html, (int)strlen(html));
}

// ============================= GSM PPM Calibration API ============================

static void api_post_calibrate_ppm(int fd, const char *body)
{
    char serial[64] = {0};
    char backend_str[32] = {0};
    const char *p;

    if ((p = strstr(body, "\"serial\"")) != NULL) {
        p = strchr(p + 8, '"'); if (p) { p++; const char *e = strchr(p, '"');
        if (e && (e - p) < 63) { memcpy(serial, p, e - p); serial[e - p] = '\0'; }
        }
    }

    // Optional backend override for testing
    if ((p = strstr(body, "\"backend\"")) != NULL) {
        p = strchr(p + 9, '"'); if (p) { p++; const char *e = strchr(p, '"');
        if (e && (e - p) < 31) { memcpy(backend_str, p, e - p); backend_str[e - p] = '\0'; }
        }
    }

    if (!serial[0]) {
        char resp[256];
        int rlen = snprintf(resp, sizeof(resp),
            "{\"error\":\"missing serial\"}");
        http_send(fd, 400, "application/json", resp, rlen);
        return;
    }

    // Find the receiver in SdrManager
    int idx = sdrManagerFindBySerial(serial);
    if (idx < 0) {
        char resp[256];
        int rlen = snprintf(resp, sizeof(resp),
            "{\"error\":\"No receiver with serial %s\"}", serial);
        http_send(fd, 404, "application/json", resp, rlen);
        return;
    }

    sdr_receiver_t *rx = &SdrManager.receivers[idx];
    int current_ppm = rx->config.ppm_error;
    float gain = rx->config.gain;
    sdr_role_t role = rx->config.role;

    panelLog("PPM calibration: stopping receiver %s (ppm=%d, gain=%.1f)",
             serial, current_ppm, gain);

    // Stop and close the receiver to free the USB device
    if (rx->state == RX_STATE_RUNNING) rxStop(rx);
    if (rx->state != RX_STATE_IDLE) rxClose(rx);
    usleep(500000);  // let OS release USB (sdrgg needs more time)

    // Run GSM calibration — prefer rtlsdr for sync reads (sdrgg read_sync
    // is unreliable on some devices). Use explicit backend if specified.
    sdr_backend_type_t cal_backend = SDR_BACKEND_RTLSDR;
    {
        std::string_view bs(backend_str);
        if (bs == "rtlsdr") cal_backend = SDR_BACKEND_RTLSDR;
        else if (bs == "sdrgg") cal_backend = SDR_BACKEND_SDRGG;
    }
    gsm_cal_result_t cal = gsm_calibrate(serial, current_ppm, gain, cal_backend);

    if (cal.success) {
        int new_ppm = (int)round(cal.corrected_ppm);
        int apply = (cal.rms < 5.0f);  // only auto-apply if RMS < 5 ppm

        if (apply) {
            rx->config.ppm_error = new_ppm;
            panelLog("PPM calibration OK: %d -> %d ppm (offset %+.1f, RMS %.3f, %d samples)",
                     current_ppm, new_ppm, cal.measured_offset, cal.rms, cal.samples);
        } else {
            panelLog("PPM calibration noisy: %d ppm suggested but RMS=%.1f too high, keeping %d ppm",
                     new_ppm, cal.rms, current_ppm);
        }

        // Restart receiver
        bool ok = rxOpen(rx);
        if (ok) ok = rxStart(rx);
        if (ok && role == SDR_ROLE_FLARM) {
            FlarmConfig.enabled = 1;
            ognClientInit();
        }
        if (apply) sdrManagerSave();

        char resp[512];
        int rlen = snprintf(resp, sizeof(resp),
            "{\"ok\":true,\"ppm\":%d,\"old_ppm\":%d,"
            "\"applied\":%s,\"backend\":\"%s\","
            "\"offset\":%.3f,\"rms\":%.3f,\"samples\":%d%s%s%s}",
            new_ppm, current_ppm,
            apply ? "true" : "false",
            backend_str[0] ? backend_str : "rtlsdr",
            cal.measured_offset, cal.rms, cal.samples,
            cal.error[0] ? ",\"warning\":\"" : "",
            cal.error[0] ? cal.error : "",
            cal.error[0] ? "\"" : "");
        http_send(fd, 200, "application/json", resp, rlen);
    } else {
        panelLog("PPM calibration FAILED: %s", cal.error);

        // Restart receiver anyway (unchanged PPM)
        bool ok = rxOpen(rx);
        if (ok) ok = rxStart(rx);
        if (ok && role == SDR_ROLE_FLARM) {
            FlarmConfig.enabled = 1;
            ognClientInit();
        }

        char resp[512];
        int rlen = snprintf(resp, sizeof(resp),
            "{\"ok\":false,\"backend\":\"%s\",\"error\":\"%s\"}",
            backend_str[0] ? backend_str : "rtlsdr", cal.error);
        http_send(fd, 200, "application/json", resp, rlen);
    }
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

    std::string_view method_sv(method);
    std::string_view path_sv(path);

    // Route
    if (method_sv == "GET") {
        if (path_sv == "/" || path_sv == "/index.html") {
            serve_file(fd, "index.html");
        } else if (path_sv == "/api/config") {
            api_get_config(fd);
        } else if (path_sv == "/api/status") {
            api_get_status(fd);
        } else if (path_sv == "/api/aircraft") {
            api_get_aircraft(fd);
        } else if (path_sv == "/api/gsm") {
            api_get_gsm(fd);
        } else if (path_sv == "/api/lte") {
            api_get_lte(fd);
        } else if (path_sv == "/api/iot868") {
            api_get_iot868(fd);
        } else if (path_sv == "/api/fanet") {
            api_get_fanet(fd);
        } else if (path_sv == "/api/stats") {
            api_get_stats(fd);
        } else if (path_sv == "/api/connections") {
            api_get_connections(fd);
        } else if (path_sv == "/api/logs") {
            api_get_logs(fd);
        } else if (path_sv == "/api/messages") {
            api_get_messages(fd);
        } else if (path_sv == "/api/devices") {
            api_get_devices(fd);
        } else if (path_sv == "/api/receivers") {
            api_get_receivers(fd);
        } else if (path_sv == "/api/decoders") {
            api_get_decoders(fd);
        } else if (path_sv == "/devices.html" || path_sv == "/devices") {
            serve_devices_page(fd);
        } else if (path_sv == "/gsm.html" || path_sv == "/gsm") {
            serve_gsm_page(fd);
        } else if (path_sv == "/lte.html" || path_sv == "/lte") {
            serve_lte_page(fd);
        } else if (path_sv == "/iot868.html" || path_sv == "/iot868") {
            serve_iot868_page(fd);
        } else if (path_sv == "/fanet.html" || path_sv == "/fanet") {
            serve_fanet_page(fd);
        } else if (path_sv == "/diagnostics.html" || path_sv == "/diagnostics") {
            serve_diagnostics_page(fd);
        } else if (path_sv == "/api/diagnostics") {
            api_get_diagnostics(fd);
        } else if (path_sv == "/api/stats/quick") {
            api_get_stats_quick(fd);
        } else if (path[0] == '/') {
            serve_file(fd, path + 1);
        } else {
            http_send(fd, 404, "text/plain", "Not found", 9);
        }
    } else if (method_sv == "POST") {
        if (path_sv == "/api/config") {
            // Find body (after \r\n\r\n)
            const char *body = strstr(request, "\r\n\r\n");
            if (body) {
                body += 4;
                api_post_config(fd, body);
            } else {
                http_send(fd, 400, "text/plain", "No body", 7);
            }
        } else if (path_sv == "/api/receivers/assign") {
            const char *body = strstr(request, "\r\n\r\n");
            if (body) {
                body += 4;
                api_post_receiver_assign(fd, body);
            } else {
                http_send(fd, 400, "text/plain", "No body", 7);
            }
        } else if (path_sv == "/api/receivers/toggle") {
            const char *body = strstr(request, "\r\n\r\n");
            if (body) {
                body += 4;
                api_post_receiver_toggle(fd, body);
            } else {
                http_send(fd, 400, "text/plain", "No body", 7);
            }
        } else if (path_sv == "/api/decoders") {
            const char *body = strstr(request, "\r\n\r\n");
            if (body) {
                body += 4;
                api_post_decoders(fd, body);
            } else {
                http_send(fd, 400, "text/plain", "No body", 7);
            }
        } else if (path_sv == "/api/calibrate-ppm") {
            const char *body = strstr(request, "\r\n\r\n");
            if (body) {
                body += 4;
                api_post_calibrate_ppm(fd, body);
            } else {
                http_send(fd, 400, "text/plain", "No body", 7);
            }
        } else if (path_sv == "/api/diagnostics/start") {
            api_post_diagnostics_start(fd);
        } else if (path_sv == "/api/receivers/setgain") {
            const char *body = strstr(request, "\r\n\r\n");
            if (body) {
                body += 4;
                api_post_setgain(fd, body);
            } else {
                http_send(fd, 400, "text/plain", "No body", 7);
            }
        } else {
            http_send(fd, 404, "text/plain", "Not found", 9);
        }
    } else if (method_sv == "OPTIONS") {
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
        // Use poll() to avoid blocking indefinitely on accept
        struct pollfd pfd = { .fd = PanelState.listen_fd, .events = POLLIN };
        int pret = poll(&pfd, 1, 1000);  // 1 second timeout
        if (pret <= 0) continue;  // timeout or error, re-check running flag

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
        struct timeval tv = {3, 0};
        setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        struct timeval stv = {3, 0};
        setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));

        // Wait for data readiness before reading (avoid blocking on idle connections)
        struct pollfd cpfd = { .fd = client_fd, .events = POLLIN };
        if (poll(&cpfd, 1, 3000) <= 0) {
            close(client_fd);
            continue;
        }

        // Read request (up to 64KB for POST bodies)
        std::string reqbuf(65536, '\0');
        {
            int total = 0;
            while (total < 65535) {
                int n = (int)read(client_fd, reqbuf.data() + total, (size_t)(65535 - total));
                if (n <= 0) break;
                total += n;
                reqbuf[total] = '\0';
                // Check if we have the full headers
                if (strstr(reqbuf.c_str(), "\r\n\r\n")) {
                    // For GET requests, we're done
                    if (reqbuf.compare(0, 3, "GET") == 0) break;
                    // For POST, check Content-Length
                    const char *cl = strstr(reqbuf.c_str(), "Content-Length:");
                    if (cl) {
                        long content_len = strtol(cl + 15, NULL, 10);
                        if (content_len < 0 || content_len > 65000) break;  // reject absurd sizes
                        const char *body_start = strstr(reqbuf.c_str(), "\r\n\r\n") + 4;
                        int header_len = (int)(body_start - reqbuf.c_str());
                        if (total >= header_len + (int)content_len) break;
                    } else {
                        break;
                    }
                }
            }

            if (total > 0)
                handle_request(client_fd, reqbuf.data(), total);
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

    if (listen(PanelState.listen_fd, 128) < 0) {
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
