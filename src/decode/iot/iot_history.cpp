#include "iot_history.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <cinttypes>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>

static struct {
    iot_history_record *records;
    int32_t  head;
    int32_t  count;
    int32_t  capacity;
    int32_t  unsaved;
    pthread_mutex_t lock;
} H;

static void iot_history_save_locked(void)
{
    char tmp_path[] = IOT_HISTORY_FILE ".tmp";
    FILE *fp = fopen(tmp_path, "wb");
    if (!fp) return;

    iot_history_header hdr = {};
    hdr.magic    = IOT_HISTORY_MAGIC;
    hdr.version  = IOT_HISTORY_VERSION;
    hdr.count    = H.count;
    hdr.head     = H.head;
    hdr.capacity = H.capacity;

    fwrite(&hdr, sizeof(hdr), 1, fp);

    int32_t n = (H.count < H.capacity) ? H.count : H.capacity;
    if (n > 0) {
        if (H.count <= H.capacity) {
            fwrite(H.records, sizeof(iot_history_record), n, fp);
        } else {
            int32_t start = H.head;
            int32_t first_chunk = H.capacity - start;
            if (first_chunk > 0)
                fwrite(H.records + start, sizeof(iot_history_record), first_chunk, fp);
            if (start > 0)
                fwrite(H.records, sizeof(iot_history_record), start, fp);
        }
    }

    fclose(fp);
    rename(tmp_path, IOT_HISTORY_FILE);
    H.unsaved = 0;
}

static void iot_history_load(void)
{
    FILE *fp = fopen(IOT_HISTORY_FILE, "rb");
    if (!fp) {
        fprintf(stderr, "iot_history: no history file, starting fresh\n");
        return;
    }

    iot_history_header hdr = {};
    if (fread(&hdr, sizeof(hdr), 1, fp) != 1 ||
        hdr.magic != IOT_HISTORY_MAGIC ||
        hdr.version != IOT_HISTORY_VERSION) {
        fprintf(stderr, "iot_history: invalid header, starting fresh\n");
        fclose(fp);
        return;
    }

    int32_t n = hdr.count;
    if (n > H.capacity) n = H.capacity;
    if (n < 0) n = 0;

    if (hdr.count <= hdr.capacity) {
        size_t rd = fread(H.records, sizeof(iot_history_record), n, fp);
        H.count = (int32_t)rd;
        H.head = H.count % H.capacity;
    } else {
        size_t rd = fread(H.records, sizeof(iot_history_record), n, fp);
        H.count = hdr.count;
        H.head = hdr.head % H.capacity;
        if ((int32_t)rd < n) {
            H.count = (int32_t)rd;
            H.head = H.count % H.capacity;
        }
    }

    fclose(fp);
    fprintf(stderr, "iot_history: loaded %d records\n", H.count < H.capacity ? H.count : H.capacity);
}

void iotHistoryInit(void)
{
    pthread_mutex_init(&H.lock, NULL);
    H.capacity = IOT_HISTORY_MAX_RECORDS;
    H.records = (iot_history_record *)calloc(H.capacity, sizeof(iot_history_record));
    if (!H.records) {
        fprintf(stderr, "iot_history: failed to allocate %d records\n", H.capacity);
        H.capacity = 0;
        return;
    }
    H.head = 0;
    H.count = 0;
    H.unsaved = 0;

    iot_history_load();
}

void iotHistoryShutdown(void)
{
    pthread_mutex_lock(&H.lock);
    if (H.unsaved > 0)
        iot_history_save_locked();
    pthread_mutex_unlock(&H.lock);

    free(H.records);
    H.records = NULL;
    H.capacity = 0;
    pthread_mutex_destroy(&H.lock);
}

void iotHistoryRecord(uint16_t sensor_id, float temperature_c,
                      float humidity_pct, float rssi_db,
                      uint8_t battery_ok, uint8_t source)
{
    if (!H.records || H.capacity <= 0) return;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t now = (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;

    iot_history_record rec = {};
    rec.ts          = now;
    rec.sensor_id   = sensor_id;
    rec.temperature = std::isnan(temperature_c) ? -9990 : (int16_t)(temperature_c * 10.0f);
    rec.humidity    = std::isnan(humidity_pct) ? -1 : (int16_t)humidity_pct;
    rec.rssi        = std::isnan(rssi_db) ? 0 : (int16_t)rssi_db;
    rec.battery_ok  = battery_ok;
    rec.source      = source;

    pthread_mutex_lock(&H.lock);

    H.records[H.head] = rec;
    H.head = (H.head + 1) % H.capacity;
    H.count++;
    H.unsaved++;

    if (H.unsaved >= IOT_HISTORY_SAVE_INTERVAL)
        iot_history_save_locked();

    pthread_mutex_unlock(&H.lock);
}

std::string iotHistoryToJSON(uint16_t sensor_id, int32_t max_points)
{
    if (!H.records || H.capacity <= 0)
        return "{\"sensor_id\":0,\"records\":[]}";

    if (max_points <= 0 || max_points > 100000) max_points = 5000;

    pthread_mutex_lock(&H.lock);

    int32_t total = (H.count < H.capacity) ? H.count : H.capacity;

    int32_t matched[100000];
    int32_t match_count = 0;

    int32_t start_idx;
    if (H.count <= H.capacity)
        start_idx = 0;
    else
        start_idx = H.head;

    for (int32_t i = 0; i < total && match_count < 100000; i++) {
        int32_t idx = (start_idx + i) % H.capacity;
        if (H.records[idx].sensor_id == sensor_id)
            matched[match_count++] = idx;
    }

    int32_t step = 1;
    if (match_count > max_points)
        step = (match_count + max_points - 1) / max_points;

    char buf[64];
    std::string s;
    s.reserve(match_count / step * 48 + 128);
    snprintf(buf, sizeof(buf), "{\"sensor_id\":%u,\"count\":%d,\"records\":[", sensor_id, match_count);
    s += buf;

    bool first = true;
    for (int32_t i = 0; i < match_count; i += step) {
        int32_t idx = matched[i];
        const iot_history_record &r = H.records[idx];
        if (!first) s += ',';
        first = false;

        snprintf(buf, sizeof(buf), "[%" PRIu64 ",%.1f,%d,%d,%d,%d]",
                 (uint64_t)r.ts,
                 r.temperature == -9990 ? -999.0 : r.temperature / 10.0,
                 (int)r.humidity,
                 (int)r.rssi,
                 (int)r.battery_ok,
                 (int)r.source);
        s += buf;
    }

    pthread_mutex_unlock(&H.lock);

    s += "]}";
    return s;
}

std::string iotHistorySensorsJSON(void)
{
    if (!H.records || H.capacity <= 0)
        return "{\"sensors\":[]}";

    pthread_mutex_lock(&H.lock);

    int32_t total = (H.count < H.capacity) ? H.count : H.capacity;
    int32_t start_idx = (H.count <= H.capacity) ? 0 : H.head;

    struct sensor_info {
        uint16_t id;
        int32_t  count;
        uint64_t last_ts;
        int16_t  last_temp;
        int16_t  last_hum;
        int16_t  last_rssi;
    };

    sensor_info sensors[IOT_HISTORY_MAX_SENSORS];
    int32_t n_sensors = 0;

    for (int32_t i = 0; i < total; i++) {
        int32_t idx = (start_idx + i) % H.capacity;
        const iot_history_record &r = H.records[idx];

        int32_t si = -1;
        for (int32_t j = 0; j < n_sensors; j++) {
            if (sensors[j].id == r.sensor_id) { si = j; break; }
        }
        if (si < 0 && n_sensors < IOT_HISTORY_MAX_SENSORS) {
            si = n_sensors++;
            sensors[si].id = r.sensor_id;
            sensors[si].count = 0;
            sensors[si].last_ts = 0;
        }
        if (si >= 0) {
            sensors[si].count++;
            if (r.ts > sensors[si].last_ts) {
                sensors[si].last_ts = r.ts;
                sensors[si].last_temp = r.temperature;
                sensors[si].last_hum = r.humidity;
                sensors[si].last_rssi = r.rssi;
            }
        }
    }

    pthread_mutex_unlock(&H.lock);

    char buf[128];
    std::string s = "{\"sensors\":[";
    for (int32_t i = 0; i < n_sensors; i++) {
        if (i > 0) s += ',';
        snprintf(buf, sizeof(buf),
                 "{\"id\":%u,\"count\":%d,\"last_ts\":%" PRIu64 ","
                 "\"temp\":%.1f,\"hum\":%d,\"rssi\":%d}",
                 sensors[i].id, sensors[i].count, (uint64_t)sensors[i].last_ts,
                 sensors[i].last_temp == -9990 ? -999.0 : sensors[i].last_temp / 10.0,
                 (int)sensors[i].last_hum, (int)sensors[i].last_rssi);
        s += buf;
    }
    s += "]}";
    return s;
}
