// Part of dump1090-gg-light
//
// iot_tracker.cpp: ISM 868 MHz IoT device tracking for web panel.
//
// Thread-safe device list updated from IoT decoder callbacks.

#include "iot_tracker.h"
#include <cstdint>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <time.h>

#include <string>
#include <cstdarg>

static iot_tracked_device_t devices[IOT_TRACKER_MAX_DEVICES];
static pthread_mutex_t tracker_mutex = PTHREAD_MUTEX_INITIALIZER;

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

void iotTrackerInit(void)
{
    pthread_mutex_lock(&tracker_mutex);
    memset(devices, 0, sizeof(devices));
    pthread_mutex_unlock(&tracker_mutex);
}

void iotTrackerDestroy(void)
{
    pthread_mutex_lock(&tracker_mutex);
    memset(devices, 0, sizeof(devices));
    pthread_mutex_unlock(&tracker_mutex);
}

// Find existing device or allocate a new slot
static iot_tracked_device_t *find_or_alloc(iot_protocol_t proto, uint32_t device_id, uint8_t channel)
{
    int32_t free_slot = -1;
    uint64_t oldest_time = UINT64_MAX;
    int32_t oldest_slot = 0;

    for (int32_t i = 0; i < IOT_TRACKER_MAX_DEVICES; i++) {
        if (devices[i].active &&
            devices[i].protocol == proto &&
            devices[i].device_id == device_id &&
            devices[i].channel == channel) {
            return &devices[i];
        }
        if (!devices[i].active && free_slot < 0) {
            free_slot = i;
        }
        if (devices[i].active && devices[i].last_seen_ms < oldest_time) {
            oldest_time = devices[i].last_seen_ms;
            oldest_slot = i;
        }
    }

    // Use free slot or evict oldest
    int32_t slot = (free_slot >= 0) ? free_slot : oldest_slot;
    memset(&devices[slot], 0, sizeof(devices[slot]));
    devices[slot].active = true;
    devices[slot].first_seen_ms = now_ms();
    return &devices[slot];
}

void iotTrackerUpdate(const iot_device_msg_t *msg)
{
    if (!msg) return;

    pthread_mutex_lock(&tracker_mutex);

    iot_tracked_device_t *dev = find_or_alloc(msg->protocol, msg->device_id, msg->channel);

    dev->protocol = msg->protocol;
    dev->modulation = msg->modulation;
    dev->device_id = msg->device_id;
    dev->channel = msg->channel;

    // Update measurements only if valid (not NAN)
    if (!isnan(msg->temperature_c)) dev->temperature_c = msg->temperature_c;
    if (!isnan(msg->humidity_pct))  dev->humidity_pct = msg->humidity_pct;
    if (!isnan(msg->pressure_hpa))  dev->pressure_hpa = msg->pressure_hpa;
    if (!isnan(msg->wind_speed_ms)) dev->wind_speed_ms = msg->wind_speed_ms;
    if (!isnan(msg->wind_dir_deg))  dev->wind_dir_deg = msg->wind_dir_deg;
    if (!isnan(msg->rain_mm))       dev->rain_mm = msg->rain_mm;
    if (!isnan(msg->power_w))       dev->power_w = msg->power_w;
    if (!isnan(msg->energy_kwh))    dev->energy_kwh = msg->energy_kwh;
    if (!isnan(msg->battery_v))     dev->battery_v = msg->battery_v;
    if (msg->battery_ok != 255)     dev->battery_ok = msg->battery_ok;

    dev->rssi_db = msg->rssi_db;
    dev->freq_offset_hz = msg->freq_offset_hz;
    dev->freq_hz = msg->freq_hz;

    dev->msg_count++;
    dev->last_seen_ms = msg->timestamp_ms ? msg->timestamp_ms : now_ms();

    if (msg->payload_len > 0) {
        int32_t copy = msg->payload_len < 64 ? msg->payload_len : 64;
        memcpy(dev->payload, msg->payload, copy);
        dev->payload_len = copy;
    }

    pthread_mutex_unlock(&tracker_mutex);
}

int32_t iotTrackerActiveCount(void)
{
    uint64_t ts = now_ms();
    uint64_t cutoff = ts - (uint64_t)IOT_DEVICE_TIMEOUT * 1000;
    int32_t count = 0;

    pthread_mutex_lock(&tracker_mutex);
    for (int32_t i = 0; i < IOT_TRACKER_MAX_DEVICES; i++) {
        if (devices[i].active && devices[i].last_seen_ms >= cutoff)
            count++;
    }
    pthread_mutex_unlock(&tracker_mutex);
    return count;
}

// Helper: format into std::string
static std::string sfmt(const char *fmt, ...) __attribute__((format(printf,1,2)));
static std::string sfmt(const char *fmt, ...) {
    char tmp[1024];
    va_list ap;
    va_start(ap, fmt);
    int32_t n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n < 0) return {};
    if ((size_t)n < sizeof(tmp)) return std::string(tmp, n);
    std::string s(n, '\0');
    va_start(ap, fmt);
    vsnprintf(&s[0], n + 1, fmt, ap);
    va_end(ap);
    return s;
}

// Convert payload to hex string
static std::string hex_encode(const uint8_t *data, int32_t len)
{
    std::string out;
    out.reserve(len * 2);
    for (int32_t i = 0; i < len; i++)
        out += sfmt("%02x", data[i]);
    return out;
}

std::string iotTrackerToJSON(void)
{
    uint64_t ts = now_ms();
    uint64_t cutoff = ts - (uint64_t)IOT_DEVICE_TIMEOUT * 1000;

    std::string s = sfmt("{\"now\":%.1f,\"devices\":[\n", ts / 1000.0);

    pthread_mutex_lock(&tracker_mutex);

    int32_t first = 1;
    for (int32_t i = 0; i < IOT_TRACKER_MAX_DEVICES; i++) {
        iot_tracked_device_t *d = &devices[i];
        if (!d->active) continue;

        bool stale = (d->last_seen_ms < cutoff);

        if (!first) s += ",\n";
        first = 0;

        std::string hex_payload = hex_encode(d->payload, d->payload_len);

        s += sfmt(
            "{\"protocol\":\"%s\",\"modulation\":\"%s\","
            "\"device_id\":\"%06X\",\"channel\":%d,"
            "\"temperature\":%.1f,\"humidity\":%.1f,"
            "\"pressure\":%.1f,\"wind_speed\":%.1f,\"wind_dir\":%.0f,"
            "\"rain\":%.1f,\"power\":%.1f,\"energy\":%.2f,"
            "\"battery_v\":%.2f,\"battery_ok\":%d,"
            "\"rssi\":%.1f,\"freq_offset\":%.0f,\"freq_mhz\":%.3f,"
            "\"msg_count\":%" PRIu64 ","
            "\"first_seen\":%.1f,\"last_seen\":%.1f,\"age\":%.1f,"
            "\"stale\":%s,\"payload\":\"%s\"}",
            iotProtocolName(d->protocol),
            iotModulationName(d->modulation),
            d->device_id, d->channel,
            isnan(d->temperature_c) ? -999.0 : (double)d->temperature_c,
            isnan(d->humidity_pct) ? -1.0 : (double)d->humidity_pct,
            isnan(d->pressure_hpa) ? -1.0 : (double)d->pressure_hpa,
            isnan(d->wind_speed_ms) ? -1.0 : (double)d->wind_speed_ms,
            isnan(d->wind_dir_deg) ? -1.0 : (double)d->wind_dir_deg,
            isnan(d->rain_mm) ? -1.0 : (double)d->rain_mm,
            isnan(d->power_w) ? -1.0 : (double)d->power_w,
            isnan(d->energy_kwh) ? -1.0 : (double)d->energy_kwh,
            isnan(d->battery_v) ? -1.0 : (double)d->battery_v,
            (int32_t)d->battery_ok,
            (double)d->rssi_db, (double)d->freq_offset_hz, d->freq_hz / 1e6,
            (uint64_t)d->msg_count,
            d->first_seen_ms / 1000.0,
            d->last_seen_ms / 1000.0,
            (ts - d->last_seen_ms) / 1000.0,
            stale ? "true" : "false",
            hex_payload.c_str());
    }

    pthread_mutex_unlock(&tracker_mutex);

    s += "\n]}";
    return s;
}
