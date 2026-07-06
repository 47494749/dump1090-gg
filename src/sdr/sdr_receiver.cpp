// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sdr_receiver.c: Multi-SDR dynamic receiver management
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#include "dump1090.h"
#include <stdint.h>
#include "sdr_receiver.h"
#include "dispatcher.h"
#include "msg_queue.h"

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <cassert>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <ctype.h>
#include <dirent.h>
#include "config_panel.h"
#include "decoder_config.h"
#include "pocsag_demod.h"
#include "gsm_decode.h"
#include "gsm_tracker.h"
#include "lte_decode.h"
#include "lte_tracker.h"
#include "iot_decode.h"
#include "iot_tracker.h"
#include "airframes_feed.h"
#include "fanet_decode.h"
#include "sarsat_decode.h"
#include "gg_format.h"

// sdr_receiver.c no longer needs direct rtlsdr include — all access through sdr_backend

// ======================== Global state ========================

sdr_manager_t SdrManager;
int32_t PocsagOutputEnabled = 1;  // toggled from panel; 1 = decode & show messages
int32_t GsmOutputEnabled = 1;     // toggled from panel; 1 = decode & show GSM cells
int32_t LteOutputEnabled = 1;     // toggled from panel; 1 = decode & show LTE cells
int32_t IotOutputEnabled = 1;     // toggled from panel; 1 = decode & show IoT devices
int32_t FanetOutputEnabled = 1;   // toggled from panel; 1 = decode & show FANET traffic
int32_t SarsatOutputEnabled = 1;  // toggled from panel; 1 = decode & show Sarsat beacons
int32_t DvbDriverWarning = 0;     // set to 1 if dvb_usb_rtl28xxu kernel module is loaded

// Dispatcher aircraft queue for FANET (registered on first use)
static aircraft_queue_handle_t fanet_aircraft_queue = NULL;

// FANET ground tracking cache (type 7 targets — not injected into aircraft list)
#define FANET_GROUND_MAX 64

static fanet_ground_entry_t fanet_ground_cache[FANET_GROUND_MAX];
static int32_t fanet_ground_count = 0;
static pthread_mutex_t fanet_ground_mutex = PTHREAD_MUTEX_INITIALIZER;

// FANET recent messages caches (for panel display)
#define FANET_NAME_MAX 32
#define FANET_MSG_MAX 32
#define FANET_WX_MAX 16
#define FANET_THERMAL_MAX 16
#define FANET_ACK_MAX 32

static fanet_name_entry_t fanet_name_cache[FANET_NAME_MAX];
static int32_t fanet_name_count = 0;
static pthread_mutex_t fanet_name_mutex = PTHREAD_MUTEX_INITIALIZER;

static fanet_msg_entry_t fanet_msg_cache[FANET_MSG_MAX];
static int32_t fanet_msg_count = 0;
static pthread_mutex_t fanet_msg_mutex = PTHREAD_MUTEX_INITIALIZER;

static fanet_wx_entry_t fanet_wx_cache[FANET_WX_MAX];
static int32_t fanet_wx_count = 0;
static pthread_mutex_t fanet_wx_mutex = PTHREAD_MUTEX_INITIALIZER;

static fanet_thermal_entry_t fanet_thermal_cache[FANET_THERMAL_MAX];
static int32_t fanet_thermal_count = 0;
static pthread_mutex_t fanet_thermal_mutex = PTHREAD_MUTEX_INITIALIZER;

static fanet_ack_entry_t fanet_ack_cache[FANET_ACK_MAX];
static int32_t fanet_ack_count = 0;
static int32_t fanet_ack_write = 0;  // ring buffer index
static pthread_mutex_t fanet_ack_mutex = PTHREAD_MUTEX_INITIALIZER;

void fanetGetGroundTracks(void (*cb)(const fanet_ground_entry_t *e, void *ctx), void *ctx)
{
    uint64_t now = mstime();
    pthread_mutex_lock(&fanet_ground_mutex);
    for (int32_t i = 0; i < fanet_ground_count; i++) {
        if (fanet_ground_cache[i].valid && (now - fanet_ground_cache[i].last_seen) < 300000)
            cb(&fanet_ground_cache[i], ctx);
    }
    pthread_mutex_unlock(&fanet_ground_mutex);
}

static void fanet_ground_cache_update(uint32_t addr, double lat, double lon,
                                       uint8_t gtype, const char *name)
{
    uint64_t now = mstime();
    pthread_mutex_lock(&fanet_ground_mutex);

    // Find existing entry or oldest slot
    int32_t slot = -1, oldest = -1;
    uint64_t oldest_time = UINT64_MAX;
    for (int32_t i = 0; i < fanet_ground_count; i++) {
        if (fanet_ground_cache[i].addr == addr) { slot = i; break; }
        if (fanet_ground_cache[i].last_seen < oldest_time) {
            oldest_time = fanet_ground_cache[i].last_seen;
            oldest = i;
        }
    }
    if (slot < 0) {
        if (fanet_ground_count < FANET_GROUND_MAX) {
            slot = fanet_ground_count++;
        } else {
            slot = oldest;
        }
    }

    fanet_ground_cache[slot].addr = addr;
    fanet_ground_cache[slot].latitude = lat;
    fanet_ground_cache[slot].longitude = lon;
    fanet_ground_cache[slot].ground_type = gtype;
    fanet_ground_cache[slot].last_seen = now;
    fanet_ground_cache[slot].valid = 1;
    if (name && name[0])
        snprintf(fanet_ground_cache[slot].name, sizeof(fanet_ground_cache[slot].name), "%s", name);

    pthread_mutex_unlock(&fanet_ground_mutex);
}

// ---- Name cache (type 2) ----
void fanetGetNames(void (*cb)(const fanet_name_entry_t *e, void *ctx), void *ctx)
{
    uint64_t now = mstime();
    pthread_mutex_lock(&fanet_name_mutex);
    for (int32_t i = 0; i < fanet_name_count; i++) {
        if (fanet_name_cache[i].valid && (now - fanet_name_cache[i].last_seen) < 600000)
            cb(&fanet_name_cache[i], ctx);
    }
    pthread_mutex_unlock(&fanet_name_mutex);
}

static void fanet_name_cache_update(uint32_t addr, const char *name)
{
    uint64_t now = mstime();
    pthread_mutex_lock(&fanet_name_mutex);
    int32_t slot = -1, oldest = -1;
    uint64_t oldest_time = UINT64_MAX;
    for (int32_t i = 0; i < fanet_name_count; i++) {
        if (fanet_name_cache[i].addr == addr) { slot = i; break; }
        if (fanet_name_cache[i].last_seen < oldest_time) { oldest_time = fanet_name_cache[i].last_seen; oldest = i; }
    }
    if (slot < 0) {
        if (fanet_name_count < FANET_NAME_MAX) slot = fanet_name_count++;
        else slot = oldest;
    }
    fanet_name_cache[slot].addr = addr;
    fanet_name_cache[slot].last_seen = now;
    fanet_name_cache[slot].valid = 1;
    snprintf(fanet_name_cache[slot].name, sizeof(fanet_name_cache[slot].name), "%s", name ? name : "");
    pthread_mutex_unlock(&fanet_name_mutex);
}

// ---- Message cache (type 3) ----
void fanetGetMessages(void (*cb)(const fanet_msg_entry_t *e, void *ctx), void *ctx)
{
    uint64_t now = mstime();
    pthread_mutex_lock(&fanet_msg_mutex);
    for (int32_t i = 0; i < fanet_msg_count; i++) {
        if (fanet_msg_cache[i].valid && (now - fanet_msg_cache[i].last_seen) < 600000)
            cb(&fanet_msg_cache[i], ctx);
    }
    pthread_mutex_unlock(&fanet_msg_mutex);
}

static void fanet_msg_cache_update(uint32_t addr, uint8_t subtype, const char *text)
{
    uint64_t now = mstime();
    pthread_mutex_lock(&fanet_msg_mutex);
    // Messages are appended as a ring (most recent wins per slot)
    int32_t slot = -1, oldest = -1;
    uint64_t oldest_time = UINT64_MAX;
    for (int32_t i = 0; i < fanet_msg_count; i++) {
        if (fanet_msg_cache[i].addr == addr && fanet_msg_cache[i].subtype == subtype) { slot = i; break; }
        if (fanet_msg_cache[i].last_seen < oldest_time) { oldest_time = fanet_msg_cache[i].last_seen; oldest = i; }
    }
    if (slot < 0) {
        if (fanet_msg_count < FANET_MSG_MAX) slot = fanet_msg_count++;
        else slot = oldest;
    }
    fanet_msg_cache[slot].addr = addr;
    fanet_msg_cache[slot].subtype = subtype;
    fanet_msg_cache[slot].last_seen = now;
    fanet_msg_cache[slot].valid = 1;
    snprintf(fanet_msg_cache[slot].text, sizeof(fanet_msg_cache[slot].text), "%s", text ? text : "");
    pthread_mutex_unlock(&fanet_msg_mutex);
}

// ---- Weather cache (type 4) ----
void fanetGetWeather(void (*cb)(const fanet_wx_entry_t *e, void *ctx), void *ctx)
{
    uint64_t now = mstime();
    pthread_mutex_lock(&fanet_wx_mutex);
    for (int32_t i = 0; i < fanet_wx_count; i++) {
        if (fanet_wx_cache[i].valid && (now - fanet_wx_cache[i].last_seen) < 600000)
            cb(&fanet_wx_cache[i], ctx);
    }
    pthread_mutex_unlock(&fanet_wx_mutex);
}

static void fanet_wx_cache_update(uint32_t addr, const char *name, const fanet_message_t *msg)
{
    uint64_t now = mstime();
    pthread_mutex_lock(&fanet_wx_mutex);
    int32_t slot = -1, oldest = -1;
    uint64_t oldest_time = UINT64_MAX;
    for (int32_t i = 0; i < fanet_wx_count; i++) {
        if (fanet_wx_cache[i].addr == addr) { slot = i; break; }
        if (fanet_wx_cache[i].last_seen < oldest_time) { oldest_time = fanet_wx_cache[i].last_seen; oldest = i; }
    }
    if (slot < 0) {
        if (fanet_wx_count < FANET_WX_MAX) slot = fanet_wx_count++;
        else slot = oldest;
    }
    fanet_wx_cache[slot].addr = addr;
    fanet_wx_cache[slot].last_seen = now;
    fanet_wx_cache[slot].valid = 1;
    snprintf(fanet_wx_cache[slot].name, sizeof(fanet_wx_cache[slot].name), "%s", name ? name : "");
    fanet_wx_cache[slot].latitude = msg->weather.latitude;
    fanet_wx_cache[slot].longitude = msg->weather.longitude;
    fanet_wx_cache[slot].temperature = msg->weather.temperature;
    fanet_wx_cache[slot].wind_speed = msg->weather.wind_speed;
    fanet_wx_cache[slot].wind_gust = msg->weather.wind_gust;
    fanet_wx_cache[slot].wind_heading = msg->weather.wind_heading;
    fanet_wx_cache[slot].humidity = msg->weather.humidity;
    fanet_wx_cache[slot].pressure = msg->weather.pressure;
    fanet_wx_cache[slot].state_of_charge = msg->weather.state_of_charge;
    fanet_wx_cache[slot].has_pos = msg->weather.has_position ? 1 : 0;
    fanet_wx_cache[slot].has_temp = msg->weather.has_temp ? 1 : 0;
    fanet_wx_cache[slot].has_wind = msg->weather.has_wind ? 1 : 0;
    fanet_wx_cache[slot].has_humidity = msg->weather.has_humidity ? 1 : 0;
    fanet_wx_cache[slot].has_pressure = msg->weather.has_pressure ? 1 : 0;
    fanet_wx_cache[slot].has_soc = msg->weather.has_soc ? 1 : 0;
    pthread_mutex_unlock(&fanet_wx_mutex);
}

// ---- Thermal cache (type 9) ----
void fanetGetThermals(void (*cb)(const fanet_thermal_entry_t *e, void *ctx), void *ctx)
{
    uint64_t now = mstime();
    pthread_mutex_lock(&fanet_thermal_mutex);
    for (int32_t i = 0; i < fanet_thermal_count; i++) {
        if (fanet_thermal_cache[i].valid && (now - fanet_thermal_cache[i].last_seen) < 600000)
            cb(&fanet_thermal_cache[i], ctx);
    }
    pthread_mutex_unlock(&fanet_thermal_mutex);
}

static void fanet_thermal_cache_update(uint32_t addr, const fanet_message_t *msg)
{
    uint64_t now = mstime();
    pthread_mutex_lock(&fanet_thermal_mutex);
    int32_t slot = -1, oldest = -1;
    uint64_t oldest_time = UINT64_MAX;
    for (int32_t i = 0; i < fanet_thermal_count; i++) {
        if (fanet_thermal_cache[i].addr == addr) { slot = i; break; }
        if (fanet_thermal_cache[i].last_seen < oldest_time) { oldest_time = fanet_thermal_cache[i].last_seen; oldest = i; }
    }
    if (slot < 0) {
        if (fanet_thermal_count < FANET_THERMAL_MAX) slot = fanet_thermal_count++;
        else slot = oldest;
    }
    fanet_thermal_cache[slot].addr = addr;
    fanet_thermal_cache[slot].last_seen = now;
    fanet_thermal_cache[slot].valid = 1;
    fanet_thermal_cache[slot].latitude = msg->thermal.latitude;
    fanet_thermal_cache[slot].longitude = msg->thermal.longitude;
    fanet_thermal_cache[slot].altitude = msg->thermal.altitude;
    fanet_thermal_cache[slot].climb = msg->thermal.climb;
    fanet_thermal_cache[slot].wind_speed = msg->thermal.wind_speed;
    fanet_thermal_cache[slot].wind_heading = msg->thermal.wind_heading;
    fanet_thermal_cache[slot].confidence = msg->thermal.confidence;
    pthread_mutex_unlock(&fanet_thermal_mutex);
}

// ---- ACK cache (type 0) — ring buffer of recent ACKs ----
void fanetGetAcks(void (*cb)(const fanet_ack_entry_t *e, void *ctx), void *ctx)
{
    uint64_t now = mstime();
    pthread_mutex_lock(&fanet_ack_mutex);
    for (int32_t i = 0; i < fanet_ack_count; i++) {
        if (fanet_ack_cache[i].valid && (now - fanet_ack_cache[i].timestamp) < 300000)
            cb(&fanet_ack_cache[i], ctx);
    }
    pthread_mutex_unlock(&fanet_ack_mutex);
}

static void fanet_ack_cache_update(uint32_t src_addr, uint32_t dst_addr)
{
    uint64_t now = mstime();
    pthread_mutex_lock(&fanet_ack_mutex);
    int32_t slot = fanet_ack_write;
    fanet_ack_cache[slot].src_addr = src_addr;
    fanet_ack_cache[slot].dst_addr = dst_addr;
    fanet_ack_cache[slot].timestamp = now;
    fanet_ack_cache[slot].valid = 1;
    fanet_ack_write = (fanet_ack_write + 1) % FANET_ACK_MAX;
    if (fanet_ack_count < FANET_ACK_MAX) fanet_ack_count++;
    pthread_mutex_unlock(&fanet_ack_mutex);
}

// ======================== Utility ========================

const char *sdrRoleName(sdr_role_t role)
{
    switch (role) {
        case SDR_ROLE_ADSB:       return "adsb";
        case SDR_ROLE_FLARM:      return "flarm";
        case SDR_ROLE_ACARS:      return "acars";
        case SDR_ROLE_VDL2:       return "vdl2";
        case SDR_ROLE_RADIOSONDE: return "radiosonde";
        case SDR_ROLE_POCSAG:     return "pocsag";
        case SDR_ROLE_GSM:        return "gsm";
        case SDR_ROLE_LTE:        return "lte";
        case SDR_ROLE_IOT868:     return "iot868";
        case SDR_ROLE_FANET:      return "fanet";
        case SDR_ROLE_SARSAT:     return "sarsat";
        default:                  return "none";
    }
}

bool rxRoleIsDecoder(sdr_role_t role)
{
    return (role == SDR_ROLE_ACARS || role == SDR_ROLE_VDL2 || role == SDR_ROLE_RADIOSONDE || role == SDR_ROLE_POCSAG || role == SDR_ROLE_GSM || role == SDR_ROLE_LTE || role == SDR_ROLE_FANET || role == SDR_ROLE_SARSAT);
}

const char *rxStateName(rx_state_t state)
{
    switch (state) {
        case RX_STATE_IDLE:     return "idle";
        case RX_STATE_OPEN:     return "open";
        case RX_STATE_RUNNING:  return "running";
        case RX_STATE_STOPPING: return "stopping";
        case RX_STATE_ERROR:    return "error";
        default:                return "unknown";
    }
}

bool rxParseConfig(const char *arg, rx_config_t *config)
{
    // Format: "serial:role[:gain=X][:ppm=Y][:agc][:freq=Z][:rate=R]"
    // Example: "00000101:adsb:gain=40.0:ppm=0"
    // Example: "00000102:flarm:gain=30.0"

    memset(config, 0, sizeof(*config));
    config->gain = MODES_DEFAULT_GAIN;
    config->freq = MODES_DEFAULT_FREQ;
    config->sample_rate = 2400000;

    // Make a mutable copy
    char buf[512];
    strncpy(buf, arg, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *saveptr = NULL;
    char *token = strtok_r(buf, ":", &saveptr);

    // First token: serial
    if (!token) return false;
    strncpy(config->serial, token, sizeof(config->serial) - 1);

    // Second token: role
    token = strtok_r(NULL, ":", &saveptr);
    if (!token) return false;

    if (!strcasecmp(token, "adsb")) {
        config->role = SDR_ROLE_ADSB;
        config->freq = 1090000000;
        config->sample_rate = 2400000;
    } else if (!strcasecmp(token, "flarm")) {
        config->role = SDR_ROLE_FLARM;
        config->freq = 868300000;
        config->sample_rate = 1600000;
    } else if (!strcasecmp(token, "acars")) {
        config->role = SDR_ROLE_ACARS;
        config->freq = 131550000;
        config->sample_rate = 2400000;
    } else if (!strcasecmp(token, "vdl2")) {
        config->role = SDR_ROLE_VDL2;
        config->freq = 136975000;
        config->sample_rate = 2400000;
    } else if (!strcasecmp(token, "radiosonde")) {
        config->role = SDR_ROLE_RADIOSONDE;
        config->freq = 403000000;
        config->sample_rate = 2400000;
    } else if (!strcasecmp(token, "pocsag")) {
        config->role = SDR_ROLE_POCSAG;
        config->freq = 466075000;
        config->sample_rate = 1200000;
    } else if (!strcasecmp(token, "gsm")) {
        config->role = SDR_ROLE_GSM;
        config->freq = 947000000;
        config->sample_rate = GSM_SAMPLE_RATE;
    } else if (!strcasecmp(token, "lte")) {
        config->role = SDR_ROLE_LTE;
        config->freq = LTE_DEFAULT_FREQ;
        config->sample_rate = LTE_SAMPLE_RATE;
    } else if (!strcasecmp(token, "iot868")) {
        config->role = SDR_ROLE_IOT868;
        config->freq = IOT_CENTER_FREQ;
        config->sample_rate = IOT_SAMPLE_RATE;
    } else if (!strcasecmp(token, "fanet")) {
        config->role = SDR_ROLE_FANET;
        config->freq = FANET_CENTER_FREQ;
        config->sample_rate = FANET_SAMPLE_RATE;
    } else if (!strcasecmp(token, "sarsat")) {
        config->role = SDR_ROLE_SARSAT;
        config->freq = SARSAT_CENTER_FREQ;
        config->sample_rate = SARSAT_SAMPLE_RATE;
    } else {
        gg::eprint("sdr_receiver: unknown role '%s' (use adsb/flarm/acars/vdl2/radiosonde/pocsag/gsm/lte/iot868/fanet/sarsat)\n", token);
        return false;
    }

    // Remaining tokens: key=value pairs
    while ((token = strtok_r(NULL, ":", &saveptr)) != NULL) {
        if (!strncasecmp(token, "gain=", 5)) {
            config->gain = atof(token + 5);
        } else if (!strncasecmp(token, "ppm=", 4)) {
            config->ppm_error = atoi(token + 4);
        } else if (!strcasecmp(token, "agc")) {
            config->digital_agc = true;
        } else if (!strncasecmp(token, "freq=", 5)) {
            config->freq = (int32_t)strtol(token + 5, NULL, 10);
        } else if (!strncasecmp(token, "rate=", 5)) {
            config->sample_rate = strtod(token + 5, NULL);
        } else if (!strncasecmp(token, "path=", 5)) {
            strncpy(config->ifile_path, token + 5, sizeof(config->ifile_path) - 1);
        } else if (!strncasecmp(token, "backend=", 8)) {
            config->backend = sdrBackendParse(token + 8);
        } else {
            gg::eprint("sdr_receiver: unknown option '%s'\n", token);
            return false;
        }
    }

    return true;
}

// ======================== Per-receiver FIFO ========================

bool rxFifoCreate(rx_fifo_t *fifo, uint32_t buffer_count, uint32_t buffer_size, uint32_t overlap)
{
    pthread_mutex_init(&fifo->mutex, NULL);
    pthread_cond_init(&fifo->notempty_cond, NULL);
    pthread_cond_init(&fifo->empty_cond, NULL);
    pthread_cond_init(&fifo->free_cond, NULL);

    fifo->head = NULL;
    fifo->tail = NULL;
    fifo->freelist = NULL;
    fifo->halted = false;

    if (!(fifo->overlap_buffer = static_cast<decltype(fifo->overlap_buffer)>(calloc(overlap, sizeof(uint16_t)))))
        goto nomem;
    fifo->overlap_length = overlap;

    for (uint32_t i = 0; i < buffer_count; ++i) {
        struct mag_buf *newbuf;
        if (!(newbuf = static_cast<decltype(newbuf)>(calloc(1, sizeof(*newbuf)))))
            goto nomem;
        if (!(newbuf->data = static_cast<decltype(newbuf->data)>(calloc(buffer_size, sizeof(newbuf->data[0]))))) {
            free(newbuf);
            goto nomem;
        }
        newbuf->totalLength = buffer_size;
        newbuf->next = fifo->freelist;
        fifo->freelist = newbuf;
    }

    return true;

nomem:
    rxFifoDestroy(fifo);
    return false;
}

static void rx_free_buffer_list(struct mag_buf *head)
{
    while (head) {
        struct mag_buf *next = head->next;
        free(head->data);
        free(head);
        head = next;
    }
}

void rxFifoDestroy(rx_fifo_t *fifo)
{
    rx_free_buffer_list(fifo->head);
    fifo->head = fifo->tail = NULL;

    rx_free_buffer_list(fifo->freelist);
    fifo->freelist = NULL;

    free(fifo->overlap_buffer);
    fifo->overlap_buffer = NULL;

    pthread_mutex_destroy(&fifo->mutex);
    pthread_cond_destroy(&fifo->notempty_cond);
    pthread_cond_destroy(&fifo->empty_cond);
    pthread_cond_destroy(&fifo->free_cond);
}

void rxFifoDrain(rx_fifo_t *fifo)
{
    pthread_mutex_lock(&fifo->mutex);
    while (fifo->head && !fifo->halted) {
        pthread_cond_wait(&fifo->empty_cond, &fifo->mutex);
    }
    pthread_mutex_unlock(&fifo->mutex);
}

void rxFifoHalt(rx_fifo_t *fifo)
{
    pthread_mutex_lock(&fifo->mutex);

    while (fifo->head) {
        struct mag_buf *freebuf = fifo->head;
        fifo->head = freebuf->next;
        freebuf->next = fifo->freelist;
        fifo->freelist = freebuf;
    }
    fifo->tail = NULL;
    fifo->halted = true;

    pthread_cond_broadcast(&fifo->notempty_cond);
    pthread_cond_broadcast(&fifo->empty_cond);
    pthread_cond_broadcast(&fifo->free_cond);
    pthread_mutex_unlock(&fifo->mutex);
}

struct mag_buf *rxFifoAcquire(rx_fifo_t *fifo, uint32_t timeout_ms)
{
    struct timespec deadline;
    if (timeout_ms)
        get_deadline(timeout_ms, &deadline);

    pthread_mutex_lock(&fifo->mutex);

    struct mag_buf *result = NULL;
    while (!fifo->halted && !fifo->freelist) {
        if (!timeout_ms) {
            goto done;
        }
        int err = pthread_cond_timedwait(&fifo->free_cond, &fifo->mutex, &deadline);
        if (err) {
            if (err != ETIMEDOUT)
                gg::eprint("rxFifoAcquire: pthread_cond_timedwait: %s\n", strerror(err));
            goto done;
        }
    }

    if (!fifo->halted) {
        result = fifo->freelist;
        fifo->freelist = result->next;

        result->overlap = fifo->overlap_length;
        result->validLength = result->overlap;
        result->sampleTimestamp = 0;
        result->sysTimestamp = 0;
        result->flags = 0;
        result->mean_level = 0;
        result->mean_power = 0;
        result->dropped = 0;
        result->next = NULL;
    }

done:
    pthread_mutex_unlock(&fifo->mutex);
    return result;
}

void rxFifoEnqueue(rx_fifo_t *fifo, struct mag_buf *buf)
{
    assert(buf->validLength <= buf->totalLength);
    assert(buf->validLength >= fifo->overlap_length);

    pthread_mutex_lock(&fifo->mutex);

    if (fifo->halted) {
        buf->next = fifo->freelist;
        fifo->freelist = buf;
        goto done;
    }

    // Populate the overlap region
    if (buf->flags & MAGBUF_DISCONTINUOUS) {
        memset(buf->data, 0, fifo->overlap_length * sizeof(buf->data[0]));
    } else {
        memcpy(buf->data, fifo->overlap_buffer, fifo->overlap_length * sizeof(buf->data[0]));
    }

    // Save tail for next time
    memcpy(fifo->overlap_buffer,
           &buf->data[buf->validLength - fifo->overlap_length],
           fifo->overlap_length * sizeof(uint16_t));

    // Enqueue
    buf->next = NULL;
    if (!fifo->head) {
        fifo->head = fifo->tail = buf;
        pthread_cond_signal(&fifo->notempty_cond);
    } else {
        fifo->tail->next = buf;
        fifo->tail = buf;
    }

done:
    pthread_mutex_unlock(&fifo->mutex);
}

struct mag_buf *rxFifoDequeue(rx_fifo_t *fifo, uint32_t timeout_ms)
{
    struct timespec deadline;
    if (timeout_ms)
        get_deadline(timeout_ms, &deadline);

    pthread_mutex_lock(&fifo->mutex);

    struct mag_buf *result = NULL;
    while (!fifo->head && !fifo->halted) {
        if (!timeout_ms) {
            goto done;
        }
        int err = pthread_cond_timedwait(&fifo->notempty_cond, &fifo->mutex, &deadline);
        if (err) {
            if (err != ETIMEDOUT)
                gg::eprint("rxFifoDequeue: pthread_cond_timedwait: %s\n", strerror(err));
            goto done;
        }
    }

    if (!fifo->halted) {
        result = fifo->head;
        fifo->head = result->next;
        result->next = NULL;
        if (!fifo->head) {
            fifo->tail = NULL;
            pthread_cond_broadcast(&fifo->empty_cond);
        }
    }

done:
    pthread_mutex_unlock(&fifo->mutex);
    return result;
}

void rxFifoRelease(rx_fifo_t *fifo, struct mag_buf *buf)
{
    pthread_mutex_lock(&fifo->mutex);
    if (!fifo->freelist)
        pthread_cond_signal(&fifo->free_cond);
    buf->next = fifo->freelist;
    fifo->freelist = buf;
    pthread_mutex_unlock(&fifo->mutex);
}

// ======================== Per-receiver SDR ops ========================

#if defined(ENABLE_RTLSDR) || defined(ENABLE_SDRGG)

static int32_t rx_sort_gains(const void *left, const void *right)
{
    return *(const int32_t *)left - *(const int32_t *)right;
}

int32_t rxGetGain(sdr_receiver_t *rx)
{
    return rx->rtl.current_gain;
}

static bool rxSupportsTunerAgc(const sdr_receiver_t *rx)
{
    return rx && rx->backend_dev && rx->backend_dev->supports_tuner_agc;
}

static bool rxDebugInitTrace(const sdr_receiver_t *rx)
{
    return rx != NULL;
}

static void rxDebugLogState(const sdr_receiver_t *rx, const char *step, int32_t rc)
{
    if (!rxDebugInitTrace(rx) || !rx->backend_dev || !rx->backend_ops) {
        return;
    }

    int32_t actual_gain = 0;
    if (rx->backend_ops->get_gain) {
        actual_gain = rx->backend_ops->get_gain(rx->backend_dev);
    }

    fprintf(stderr,
            "rx[%d]: DEBUG %s rc=%d serial=%s role=%s freq=%u rate=%u dev_gain=%d api_gain=%d gain_step=%d direct=%d digital_agc=%d tuner_agc=%d\n",
            rx->id,
            step,
            rc,
            rx->serial_actual[0] ? rx->serial_actual : rx->config.serial,
            sdrRoleName(rx->config.role),
            rx->backend_dev->current_freq,
            rx->backend_dev->current_rate,
            rx->backend_dev->current_gain,
            actual_gain,
            rx->rtl.current_gain,
            rx->config.direct_sampling,
            rx->config.digital_agc,
            rx->backend_dev->supports_tuner_agc);
}

// ============== USB/RF HEALTH DIAGNOSTIC ==============
// Dumps all R820T registers (0x00-0x1F) and checks PLL lock.
// Called after init and when "no messages" condition is detected.

static const char *r820t_reg_names[32] = {
    "ChipID",   "r01",      "r02",      "r03",      "r04",      "LNA-power",
    "LNA-gain", "Mixer-gn", "ImgR-adj", "ImgI-adj", "IF-fltcal","IF-flt2",
    "Filt-BW",  "Filt-ext", "VGA-clk",  "VGA-gain", "PLL-syn",  "PLL-Xtal",
    "PLL-Div",  "PLL-SD3",  "PLL-SD2",  "PLL-SD1",  "PLL-SD0",  "PLL-test",
    "Cal-VCO",  "Mix-LNA",  "LO-div",   "LNA-dpw",  "RF-flt3",  "RF-flt2",
    "RF-flt1",  "RF-flt0"
};

static void rxDiagDumpR820T(const sdr_receiver_t *rx, const char *reason)
{
    if (!rx || !rx->backend_dev || !rx->backend_ops) return;
    if (!rx->backend_ops->read_tuner_reg) return;

    uint8_t regs[32];
    int32_t errors = 0;
    for (int32_t i = 0; i < 32; i++) {
        if (rx->backend_ops->read_tuner_reg(rx->backend_dev, (uint8_t)i, &regs[i]) != 0) {
            regs[i] = 0xFF;
            errors++;
        }
    }

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    localtime_r(&ts.tv_sec, &tm);

    fprintf(stderr,
            "\n===== R820T DIAG rx[%d] serial=%s role=%s reason=%s time=%02d:%02d:%02d =====\n",
            rx->id,
            rx->serial_actual[0] ? rx->serial_actual : rx->config.serial,
            sdrRoleName(rx->config.role),
            reason,
            tm.tm_hour, tm.tm_min, tm.tm_sec);

    if (errors == 32) {
        fprintf(stderr, "  *** ALL READS FAILED — I2C bus dead! ***\n");
        fprintf(stderr, "======================================================\n\n");
        return;
    }

    // Print register table
    fprintf(stderr, "  REG  NAME        VALUE\n");
    for (int32_t i = 0; i < 32; i++) {
        fprintf(stderr, "  0x%02X %-10s  0x%02X\n", i, r820t_reg_names[i], regs[i]);
    }

    // Key diagnostics
    bool pll_locked = (regs[0x12] & 0x40) != 0;  // PLL lock indicator in reg 0x12 bit 6
    uint8_t lna_gain = (regs[0x06] >> 4) & 0x0F;
    uint8_t mixer_gain = (regs[0x07] >> 4) & 0x0F;
    uint8_t vga_gain = regs[0x0F] & 0x0F;
    uint8_t lo_div = (regs[0x1A] >> 5) & 0x07;

    fprintf(stderr, "  --- KEY STATUS ---\n");
    fprintf(stderr, "  PLL locked: %s\n", pll_locked ? "YES" : "*** NO — NOT LOCKED! ***");
    fprintf(stderr, "  LNA gain index: %d  Mixer gain index: %d  VGA gain: %d\n",
            lna_gain, mixer_gain, vga_gain);
    fprintf(stderr, "  LO divider: %d (div=%d)\n", lo_div, 2 << lo_div);
    fprintf(stderr, "  Freq set: %u Hz  Rate: %u\n",
            rx->backend_dev->current_freq, rx->backend_dev->current_rate);

    if (!pll_locked) {
        fprintf(stderr, "  *** PLL NOT LOCKED — RF front-end is DEAD! ***\n");
    }

    fprintf(stderr, "======================================================\n\n");
}

// Per-receiver health state for the watchdog
struct rx_health_state {
    uint64_t last_check_time;       // mstime of last check
    uint64_t last_sample_counter;   // sample_counter at last check
    uint32_t last_messages_total;   // messages at last check (ADSB only)
    uint32_t no_msg_seconds;        // consecutive seconds with 0 new messages
    bool     dump_done;             // already dumped for this "episode"
    bool     baseline_done;         // init dump completed
};
static struct rx_health_state rx_health[MAX_SDR_RECEIVERS];

// Helper: dump all registers to /tmp/sdrgg_watchdog_dump.log and stderr
static void rxDiagFullDump(const sdr_receiver_t *rx, const char *reason)
{
    if (!rx || !rx->backend_dev || !rx->backend_ops) return;
    if (!rx->backend_ops->dump_registers) return;

    // Dump to persistent log file
    FILE *dumpf = fopen("/tmp/sdrgg_watchdog_dump.log", "a");
    if (dumpf) {
        fprintf(dumpf, "\n*** %s rx[%d] serial=%s ***\n",
                reason, rx->id,
                rx->serial_actual[0] ? rx->serial_actual : rx->config.serial);
        rx->backend_ops->dump_registers(rx->backend_dev, dumpf);
        fclose(dumpf);
    }
    // Also dump to stderr (journald)
    fprintf(stderr, "\n*** %s rx[%d] serial=%s — FULL REG DUMP ***\n",
            reason, rx->id,
            rx->serial_actual[0] ? rx->serial_actual : rx->config.serial);
    rx->backend_ops->dump_registers(rx->backend_dev, stderr);
}

// Called from backgroundTasks() every ~1 second to monitor receiver health
void rxDiagHealthCheck(void)
{
    uint64_t now = mstime();

    for (int32_t i = 0; i < SdrManager.count; i++) {
        sdr_receiver_t *rx = &SdrManager.receivers[i];
        struct rx_health_state *h = &rx_health[i];

        if (rx->state != RX_STATE_RUNNING || !rx->backend_dev) continue;

        // Only check R820T-based receivers (ADSB, FLARM)
        if (rx->config.role != SDR_ROLE_ADSB && rx->config.role != SDR_ROLE_FLARM)
            continue;

        // Throttle to once per second
        if (now - h->last_check_time < 1000) continue;
        h->last_check_time = now;

        // Check if samples are flowing (USB data arriving)
        bool samples_flowing = (rx->sample_counter > h->last_sample_counter);
        h->last_sample_counter = rx->sample_counter;

        if (!samples_flowing) continue;  // USB problem, not RF — handled elsewhere

        // For ADSB: check message count
        if (rx->config.role == SDR_ROLE_ADSB) {
            uint32_t current_msgs = Modes.stats_current.messages_total +
                                    Modes.stats_alltime.messages_total;
            if (current_msgs == h->last_messages_total) {
                h->no_msg_seconds++;
            } else {
                // Messages flowing — reset watchdog
                if (h->no_msg_seconds >= 30 && h->dump_done) {
                    // Recovered! Log recovery dump
                    fprintf(stderr, "rx[%d]: DIAG receiver RECOVERED after %u seconds of silence\n",
                            rx->id, h->no_msg_seconds);
                    rxDiagDumpR820T(rx, "RECOVERED");
                    rxDiagFullDump(rx, "RECOVERED");
                }
                h->no_msg_seconds = 0;
                h->dump_done = false;
            }
            h->last_messages_total = current_msgs;

            // Trigger dump after 30s of 0 messages (samples flowing but no RF decoded)
            if (h->no_msg_seconds == 30 && !h->dump_done) {
                fprintf(stderr, "rx[%d]: DIAG *** 30s with 0 ADS-B messages while USB data flows! ***\n",
                        rx->id);
                rxDiagDumpR820T(rx, "NO_MESSAGES_30s");
                rxDiagFullDump(rx, "NO_MESSAGES_30s");
                h->dump_done = true;
            }
            // Repeat dump every 5 minutes if still silent
            if (h->no_msg_seconds > 0 && (h->no_msg_seconds % 300) == 0) {
                rxDiagDumpR820T(rx, "STILL_SILENT_5min");
                rxDiagFullDump(rx, "STILL_SILENT_5min");
            }
        }

        // For FLARM: use sample_counter growth without decoded frames
        // (FLARM is sparse, so use longer timeout — handled by FLARM decoder stats)
    }
}

int32_t rxGetMaxGain(sdr_receiver_t *rx)
{
    return rx->rtl.gain_steps - 1;
}

double rxGetGainDb(sdr_receiver_t *rx, int32_t step)
{
    if (!rx->rtl.gains) return 0.0;
    if (step < 0) step = 0;
    if (step >= rx->rtl.gain_steps) step = rx->rtl.gain_steps - 1;
    return rx->rtl.gains[step] / 10.0;
}

int32_t rxSetGain(sdr_receiver_t *rx, int32_t step)
{
    if (!rx->rtl.gains || !rx->backend_dev) return -1;
    bool supports_tuner_agc = rxSupportsTunerAgc(rx);
    int32_t mode_rc = 0;
    int32_t gain_rc = 0;

    if (step < 0) step = 0;
    if (step >= rx->rtl.gain_steps) step = rx->rtl.gain_steps - 1;

    if (supports_tuner_agc && step == rx->rtl.gain_steps - 1) {
        mode_rc = rx->backend_ops->set_gain_mode(rx->backend_dev, 0);
        if (mode_rc < 0) {
            gg::eprint("rx[%d]: failed to enable tuner AGC\n", rx->id);
            return rx->rtl.current_gain;
        }
        fprintf(stderr, "rx[%d]: tuner gain set to about %.1f dB (step %d, AGC)\n",
                rx->id, rx->rtl.gains[step] / 10.0, step);
    } else {
        mode_rc = rx->backend_ops->set_gain_mode(rx->backend_dev, 1);
        if (mode_rc < 0) {
            gg::eprint("rx[%d]: failed to disable tuner AGC\n", rx->id);
            return rx->rtl.current_gain;
        }
        gain_rc = rx->backend_ops->set_gain(rx->backend_dev, rx->rtl.gains[step]);
        if (gain_rc < 0) {
            fprintf(stderr, "rx[%d]: failed to set tuner gain to %.1fdB\n",
                    rx->id, rx->rtl.gains[step] / 10.0);
            return rx->rtl.current_gain;
        }
        fprintf(stderr, "rx[%d]: tuner gain set to %.1f dB (step %d)\n",
                rx->id, rx->rtl.gains[step] / 10.0, step);
    }

    rx->rtl.current_gain = step;
    if (rxDebugInitTrace(rx)) {
        fprintf(stderr,
                "rx[%d]: DEBUG gain-select step=%d requested=%.1f mode_rc=%d gain_rc=%d gain_steps=%d\n",
                rx->id,
                step,
                rx->rtl.gains[step] / 10.0,
                mode_rc,
                gain_rc,
                rx->rtl.gain_steps);
        rxDebugLogState(rx, "after-gain", gain_rc ? gain_rc : mode_rc);
    }
    return step;
}

static int32_t rx_find_device_index(const sdr_backend_ops_t *ops, const char *serial)
{
    sdr_dev_info_t devs[MAX_SDR_RECEIVERS];
    int32_t count = ops->enumerate(devs, MAX_SDR_RECEIVERS);
    if (!count) return -1;

    // Exact match
    for (int32_t i = 0; i < count; i++) {
        if (!strcmp(serial, devs[i].serial))
            return devs[i].index;
    }

    // Prefix match
    size_t slen = strlen(serial);
    for (int32_t i = 0; i < count; i++) {
        if (!strncmp(serial, devs[i].serial, slen))
            return devs[i].index;
    }

    return -1;
}

// Async stream callback — context points to the sdr_receiver_t
static void rx_stream_callback(uint8_t *buf, uint32_t len, void *ctx)
{
    sdr_receiver_t *rx = (sdr_receiver_t *)ctx;

    if (Modes.exit || rx->state == RX_STATE_STOPPING) {
        if (rx->backend_dev && rx->backend_ops)
            rx->backend_ops->cancel_async(rx->backend_dev);
        return;
    }

    // Apply deferred retune (e.g. LTE frequency hopping)
    uint32_t new_freq = rx->pending_freq;
    if (new_freq) {
        rx->pending_freq = 0;
        int32_t ret = rx->backend_ops->set_frequency(rx->backend_dev, new_freq);
        if (ret < 0) {
            rx->usb_error_count++;
            rx->usb_error_total++;
            if (rx->usb_error_count == 1) {
                fprintf(stderr, "rx[%d]: set_frequency failed (%d), "
                        "USB error recovery started\n", rx->id, ret);
            }
            if (rx->usb_error_count >= 10) {
                fprintf(stderr, "rx[%d]: %u consecutive USB errors, "
                        "cancelling async for device reset\n",
                        rx->id, rx->usb_error_count);
                rx->backend_ops->cancel_async(rx->backend_dev);
            }
            return;
        }
        rx->usb_error_count = 0;
        rx->config.freq = new_freq;
        return;
    }

    uint32_t samples_read = len / 2;
    if (!samples_read) return;

    // Waterfall IQ tap: copy raw IQ into ring buffer for spectrum display
    if (__atomic_load_n(&rx->wf_tap_active, __ATOMIC_ACQUIRE)) {
        uint8_t *tap_buf = rx->wf_tap_buf;
        uint32_t sz = rx->wf_tap_size;
        if (tap_buf && sz > 0) {
            uint32_t wr = rx->wf_tap_wr;
            uint32_t to_copy = len;
            if (to_copy > sz) to_copy = sz;  // cap at buffer size
            uint32_t first = sz - wr;
            if (first >= to_copy) {
                memcpy(tap_buf + wr, buf, to_copy);
            } else {
                memcpy(tap_buf + wr, buf, first);
                memcpy(tap_buf, buf + first, to_copy - first);
            }
            __atomic_store_n(&rx->wf_tap_wr, (wr + to_copy) % sz, __ATOMIC_RELEASE);
        }
    }

    // Dispatch to decoder_ops (all roles use this uniform path)
    if (rx->decoder_ops && rx->decoder_ops->process) {
        rx->decoder_ops->process(rx, buf, len);
    }

    // Sample IQ noise power for auto-gain (first 256 pairs per callback)
    {
        uint32_t n = (samples_read < 256) ? samples_read : 256;
        uint64_t s = 0;
        for (uint32_t i = 0; i < n * 2; i += 2) {
            int32_t I = (int32_t)buf[i] - 128;
            int32_t Q = (int32_t)buf[i+1] - 128;
            s += (uint32_t)(I*I + Q*Q);
        }
        __atomic_add_fetch(&rx->ag_iq_sum, s, __ATOMIC_RELAXED);
        __atomic_add_fetch(&rx->ag_iq_count, n, __ATOMIC_RELAXED);
    }

    rx->sample_counter += samples_read;
}

bool rxOpen(sdr_receiver_t *rx)
{
    if (rx->state != RX_STATE_IDLE) {
        gg::eprint("rx[%d]: cannot open, state is %s\n", rx->id, rxStateName(rx->state));
        return false;
    }

    // ---- Virtual file device (--receiver FILE:role:path=/path/to/file.raw) ----
    if (rx->config.ifile_path[0] != '\0') {
        FILE *fp = fopen(rx->config.ifile_path, "rb");
        if (!fp) {
            fprintf(stderr, "rx[%d]: cannot open file '%s': %s\n",
                    rx->id, rx->config.ifile_path, strerror(errno));
            rx->state = RX_STATE_ERROR;
            return false;
        }
        fclose(fp);

        snprintf(rx->serial_actual, sizeof(rx->serial_actual), "FILE_%d", rx->id);
        snprintf(rx->manufacturer, sizeof(rx->manufacturer), "Virtual");
        snprintf(rx->product, sizeof(rx->product), "IQ File Replay");

        rx->decoder_ops = decoderOpsForRole(rx->config.role);
        if (rx->decoder_ops) {
            if (!rx->decoder_ops->init(rx)) {
                fprintf(stderr, "rx[%d]: can't init decoder for virtual device (role %s)\n",
                        rx->id, sdrRoleName(rx->config.role));
                rx->state = RX_STATE_ERROR;
                return false;
            }
        }

        rx->dropped = 0;
        rx->sample_counter = 0;
        rx->state = RX_STATE_OPEN;

        fprintf(stderr, "rx[%d]: virtual file device opened (role=%s, file=%s)\n",
                rx->id, sdrRoleName(rx->config.role), rx->config.ifile_path);
        return true;
    }

    // ---- Real SDR device via backend abstraction ----

    // Resolve backend for this receiver
    const sdr_backend_ops_t *ops = sdrBackendResolve(rx->config.backend);
    if (!ops) {
        gg::eprint("rx[%d]: no SDR backend available\n", rx->id);
        rx->state = RX_STATE_ERROR;
        return false;
    }
    rx->backend_ops = ops;

    // Enumerate devices via the resolved backend
    sdr_dev_info_t devs[MAX_SDR_RECEIVERS];
    int32_t dev_count = ops->enumerate(devs, MAX_SDR_RECEIVERS);
    if (!dev_count) {
        gg::eprint("rx[%d]: no SDR devices found (backend=%s)\n", rx->id, ops->name);
        rx->state = RX_STATE_ERROR;
        return false;
    }

    int32_t dev_index = -1;
    if (rx->config.serial[0]) {
        dev_index = rx_find_device_index(ops, rx->config.serial);
        if (dev_index < 0) {
            gg::eprint("rx[%d]: no device matching serial '%s'\n", rx->id, rx->config.serial);
            rx->state = RX_STATE_ERROR;
            return false;
        }
    } else {
        // Auto-assign: find first device not already in use by another receiver
        for (int32_t i = 0; i < dev_count; i++) {
            // Check if already used
            bool in_use = false;
            for (int32_t r = 0; r < SdrManager.count; r++) {
                if (r != rx->id && SdrManager.receivers[r].state >= RX_STATE_OPEN &&
                    !strcmp(SdrManager.receivers[r].serial_actual, devs[i].serial)) {
                    in_use = true;
                    break;
                }
            }
            if (!in_use) {
                dev_index = devs[i].index;
                break;
            }
        }
        if (dev_index < 0) {
            gg::eprint("rx[%d]: no free SDR device available\n", rx->id);
            rx->state = RX_STATE_ERROR;
            return false;
        }
    }

    char selected_serial[sizeof(rx->serial_actual)] = {0};

    for (int32_t i = 0; i < dev_count; i++) {
        if (devs[i].index == dev_index) {
            strncpy(selected_serial, devs[i].serial, sizeof(selected_serial) - 1);
            break;
        }
    }

    /* FC0012 MUST use the rtlsdr backend.
     * The sdrgg enumerate does not probe tuners (tuner_type = UNKNOWN),
     * so we use the rtlsdr backend to probe the tuner type for this serial.
     * If the device is FC0012, force rtlsdr which has proven RF reception. */
    if (ops->type == SDR_BACKEND_SDRGG) {
        const sdr_backend_ops_t *rtl_ops = sdrBackendGet(SDR_BACKEND_RTLSDR);
        if (rtl_ops) {
            sdr_dev_info_t rtl_devs[MAX_SDR_RECEIVERS];
            int32_t rtl_count = rtl_ops->enumerate(rtl_devs, MAX_SDR_RECEIVERS);
            for (int32_t i = 0; i < rtl_count; i++) {
                if (strcmp(rtl_devs[i].serial, selected_serial) == 0 &&
                    rtl_devs[i].tuner == SDR_TUNER_FC0012) {
                    gg::eprint("rx[%d]: forcing backend rtlsdr for FC0012 serial %s\n",
                               rx->id, selected_serial);
                    ops = rtl_ops;
                    rx->backend_ops = ops;
                    dev_count = rtl_count;
                    dev_index = rtl_devs[i].index;
                    break;
                }
            }
        }
    }

    // Fill identity from enumeration info
    for (int32_t i = 0; i < dev_count; i++) {
        if (devs[i].index == dev_index) {
            strncpy(rx->serial_actual, devs[i].serial, sizeof(rx->serial_actual) - 1);
            strncpy(rx->manufacturer, devs[i].manufacturer, sizeof(rx->manufacturer) - 1);
            strncpy(rx->product, devs[i].product, sizeof(rx->product) - 1);
            break;
        }
    }
    rx->dev_index = dev_index;

    fprintf(stderr, "rx[%d]: opening device #%d via %s (%s, %s, SN %s) role=%s\n",
            rx->id, dev_index, ops->name,
            rx->manufacturer, rx->product, rx->serial_actual,
            sdrRoleName(rx->config.role));

    // Open device via backend
    sdr_device_t *sdev = ops->open_by_index(dev_index);
    if (!sdev) {
        gg::eprint("rx[%d]: error opening device #%d via %s\n", rx->id, dev_index, ops->name);
        rx->state = RX_STATE_ERROR;
        return false;
    }
    rx->backend_dev = sdev;
    rx->rtl.dev = sdev->handle;  // legacy compat: keep raw handle in rtl.dev
    rx->rtl.tuner_type = (int32_t)sdev->tuner_type;

    /*
     * IMPORTANT RTL-SDR NOTE - DO NOT "RESET" direct sampling here.
     *
     * What MUST be done here:
     * - If digital AGC is not requested, disable only digital AGC.
     * - Call set_direct_sampling() ONLY when direct sampling was explicitly requested
     *   by config and we really want to enter that mode.
     *
     * What MUST NOT be done here:
     * - Do NOT call set_direct_sampling(dev, 0) as a generic baseline reset when
     *   direct_sampling == 0.
     *
     * Why this warning exists:
     * - On real R820T devices traced against stock rtl_adsb/rtl_sdr, that call is
     *   NOT a harmless no-op.
     * - Reintroducing it made librtlsdr flip through direct-sampling mode during open,
     *   produced PLL-not-locked behaviour, and diverged from the stock startup
     *   sequence that actually works.
     * - This regression broke dump1090-gg's rtlsdr path even though stock tools and
     *   the sdrgg backend were fine.
     */
    if (!rx->config.digital_agc) {
        int32_t agc_rc = ops->set_agc(sdev, 0);
        rxDebugLogState(rx, "set_agc(0)", agc_rc);
    }

    /* ================================================================
     * FC0012 CRITICAL FIX — DO NOT REMOVE
     * ================================================================
     * FC0012 Zero-IF tuners NEED rtlsdr_set_direct_sampling(dev, 0)
     * called after open. This is the ONLY place in librtlsdr that
     * programs RTL2832U demod register page0:0x08 = 0xCD, which
     * enables BOTH I and Q ADC channels for Zero-IF reception.
     *
     * WITHOUT this call:
     *   - Only one ADC channel is active
     *   - Gain changes return success but have NO effect on IQ data
     *   - Frequency changes return success but spectrum doesn't move
     *   - Waterfall shows identical noise regardless of settings
     *
     * This call is HARMFUL for R820T (triggers PLL-not-locked),
     * so the tuner_type == SDR_TUNER_FC0012 guard is essential.
     *
     * If this code is accidentally removed, the FC0012 dongle
     * appears to work (streaming OK, API calls return 0) but
     * the RF front-end is effectively disconnected from the ADC.
     * ================================================================ */
    if (!rx->config.direct_sampling && sdev->tuner_type == SDR_TUNER_FC0012) {
        int32_t ds_rc = ops->set_direct_sampling(sdev, 0);
        rxDebugLogState(rx, "set_direct_sampling(0) for FC0012", ds_rc);
    }

    // Gain setup
    if (rx->config.direct_sampling) {
        gg::eprint("rx[%d]: direct sampling from input %d\n", rx->id, rx->config.direct_sampling);
        int32_t ds_rc = ops->set_direct_sampling(sdev, rx->config.direct_sampling);
        rxDebugLogState(rx, "set_direct_sampling", ds_rc);
        rx->rtl.gain_steps = 0;
    } else {
        int32_t numgains = ops->get_tuner_gains(sdev, NULL, 0);
        if (numgains <= 0) {
            gg::eprint("rx[%d]: error getting tuner gains\n", rx->id);
            rxClose(rx);
            return false;
        }

        bool supports_tuner_agc = sdev->supports_tuner_agc;
        int32_t gain_slots = numgains + (supports_tuner_agc ? 1 : 0);
        int32_t *gains = static_cast<int32_t*>(malloc((size_t)gain_slots * sizeof(int32_t)));
        if (ops->get_tuner_gains(sdev, gains, numgains) != numgains) {
            gg::eprint("rx[%d]: error getting tuner gains\n", rx->id);
            free(gains);
            rxClose(rx);
            return false;
        }

        qsort(gains, numgains, sizeof(gains[0]), rx_sort_gains);

        if (supports_tuner_agc) {
            // Fake entry at slightly higher than max: "tuner AGC enabled"
            gains[numgains] = gains[numgains - 1] + 90;
        }
        rx->rtl.gain_steps = gain_slots;
        rx->rtl.gains = gains;

        // Select gain step
        int32_t selected = -1;
        if (rx->config.gain == MODES_LEGACY_AUTO_GAIN) {
            selected = supports_tuner_agc ? numgains : (numgains - 1);
        } else if (rx->config.gain == MODES_DEFAULT_GAIN) {
            selected = numgains - 1;  // max manual gain
        } else {
            for (int32_t i = 0; i < gain_slots; ++i) {
                if (selected == -1 || fabs(gains[i] / 10.0 - rx->config.gain) < fabs(gains[selected] / 10.0 - rx->config.gain))
                    selected = i;
            }
        }

        if (rxDebugInitTrace(rx)) {
            fprintf(stderr,
                    "rx[%d]: DEBUG gain-table role=%s serial=%s requested_gain=%.1f numgains=%d gain_slots=%d supports_tuner_agc=%d selected=%d selected_db=%.1f\n",
                    rx->id,
                    sdrRoleName(rx->config.role),
                    rx->serial_actual[0] ? rx->serial_actual : rx->config.serial,
                    rx->config.gain,
                    numgains,
                    gain_slots,
                    supports_tuner_agc,
                    selected,
                    gains[selected] / 10.0);
        }

        rxSetGain(rx, selected);
    }

    if (rx->config.digital_agc) {
        gg::eprint("rx[%d]: enabling digital AGC\n", rx->id);
        int32_t agc_rc = ops->set_agc(sdev, 1);
        rxDebugLogState(rx, "set_agc(1)", agc_rc);
    }

    int32_t ppm_rc = ops->set_freq_correction(sdev, rx->config.ppm_error);
    rxDebugLogState(rx, "set_freq_correction", ppm_rc);

    int32_t freq_rc = ops->set_frequency(sdev, rx->config.freq);
    rxDebugLogState(rx, "set_frequency(initial)", freq_rc);

    int32_t rate_rc = ops->set_sample_rate(sdev, (uint32_t)rx->config.sample_rate);
    rxDebugLogState(rx, "set_sample_rate(initial)", rate_rc);
    if (rate_rc != 0) {
        fprintf(stderr, "rx[%d]: failed to set sample rate %.0f for role %s\n",
                rx->id, rx->config.sample_rate, sdrRoleName(rx->config.role));
        rxClose(rx);
        return false;
    }

    int32_t reset_rc = ops->reset_buffer(sdev);
    rxDebugLogState(rx, "reset_buffer", reset_rc);

    // Set decoder_ops for this role (uniform plugin interface)
    rx->decoder_ops = decoderOpsForRole(rx->config.role);
    if (rx->decoder_ops) {
        if (!rx->decoder_ops->init(rx)) {
            fprintf(stderr, "rx[%d]: can't init decoder_ops '%s' for role %s\n",
                    rx->id, rx->decoder_ops->name, sdrRoleName(rx->config.role));
            rxClose(rx);
            return false;
        }
        /* GSM init may change rx->config.freq (IF offset); retune if needed */
        if (ops->get_frequency(sdev) != (uint32_t)rx->config.freq) {
            int32_t retune_rc = ops->set_frequency(sdev, rx->config.freq);
            rxDebugLogState(rx, "set_frequency(post-init)", retune_rc);
        }
    } else if (rx->config.role != SDR_ROLE_NONE) {
        fprintf(stderr, "rx[%d]: no decoder_ops for role %s\n",
                rx->id, sdrRoleName(rx->config.role));
        rxClose(rx);
        return false;
    }

    rx->dropped = 0;
    rx->sample_counter = 0;
    rx->state = RX_STATE_OPEN;

    fprintf(stderr, "rx[%d]: opened successfully (freq=%d, rate=%.0f, role=%s)\n",
            rx->id, rx->config.freq, rx->config.sample_rate, sdrRoleName(rx->config.role));

    // Diagnostic: dump R820T baseline registers after init
    if (rx->config.role == SDR_ROLE_ADSB || rx->config.role == SDR_ROLE_FLARM) {
        rxDiagDumpR820T(rx, "POST_INIT_BASELINE");
        rxDiagFullDump(rx, "POST_INIT_BASELINE");
        if (rx->id < MAX_SDR_RECEIVERS) {
            rx_health[rx->id].baseline_done = true;
            rx_health[rx->id].last_check_time = mstime();
            rx_health[rx->id].no_msg_seconds = 0;
            rx_health[rx->id].dump_done = false;
        }
    }

    return true;
}

// Reader thread entry point
static void *rx_reader_thread(void *arg)
{
    sdr_receiver_t *rx = (sdr_receiver_t *)arg;
    const sdr_backend_ops_t *ops = rx->backend_ops;
    sdr_device_t *sdev = rx->backend_dev;

    fprintf(stderr, "rx[%d]: reader thread started (role=%s, serial=%s, backend=%s)\n",
            rx->id, sdrRoleName(rx->config.role), rx->serial_actual, ops->name);

    ops->read_async(sdev, rx_stream_callback, rx, 4, MODES_RTL_BUF_SIZE);

    // If cancelled due to USB errors (not shutdown), try to recover
    if (!Modes.exit && rx->state != RX_STATE_STOPPING && rx->usb_error_count > 0) {
        gg::eprint("rx[%d]: USB error recovery — closing and reopening device\n", rx->id);
        ops->close(sdev);
        rx->backend_dev = NULL;
        rx->rtl.dev = NULL;

        // Wait 2 seconds for USB to settle
        struct timespec ts = {2, 0};
        nanosleep(&ts, NULL);

        if (Modes.exit) goto done;

        // Reopen device
        sdr_device_t *new_dev = ops->open_by_index(rx->dev_index);
        if (!new_dev) {
            gg::eprint("rx[%d]: failed to reopen device, giving up\n", rx->id);
            rx->state = RX_STATE_ERROR;
            goto done;
        }
        rx->backend_dev = new_dev;
        rx->rtl.dev = new_dev->handle;
        sdev = new_dev;

        // Reconfigure device
        /*
         * IMPORTANT: same rule as initial open.
         * Do NOT reintroduce set_direct_sampling(dev, 0) here as a reopen/reset step.
         * For R820T dongles that "reset" caused the same bad direct-sampling toggle
         * seen during initial open and does not match stock rtl_sdr behaviour.
         */
        if (!rx->config.digital_agc) {
            ops->set_agc(sdev, 0);
        }
        ops->set_gain_mode(sdev, 1);
        if (rx->rtl.gains && rx->rtl.current_gain < rx->rtl.gain_steps)
            ops->set_gain(sdev, rx->rtl.gains[rx->rtl.current_gain]);
        ops->set_freq_correction(sdev, rx->config.ppm_error);
        ops->set_frequency(sdev, rx->config.freq);
        if (ops->set_sample_rate(sdev, (uint32_t)rx->config.sample_rate) != 0) {
            fprintf(stderr, "rx[%d]: failed to restore sample rate %.0f during reopen\n",
                    rx->id, rx->config.sample_rate);
            ops->close(sdev);
            rx->backend_dev = NULL;
            rx->rtl.dev = NULL;
            rx->state = RX_STATE_ERROR;
            goto done;
        }
        ops->reset_buffer(sdev);
        rx->usb_error_count = 0;

        gg::eprint("rx[%d]: device reopened successfully, resuming\n", rx->id);

        // Resume async read
        ops->read_async(sdev, rx_stream_callback, rx, 4, MODES_RTL_BUF_SIZE);
    }

    if (!Modes.exit && rx->state != RX_STATE_STOPPING) {
        gg::eprint("rx[%d]: read_async returned unexpectedly, device may be lost\n", rx->id);
        rx->state = RX_STATE_ERROR;
    }

done:
    gg::eprint("rx[%d]: reader thread exiting\n", rx->id);
    return NULL;
}

// File replay reader thread — reads IQ file in loop at real-time pace
static void *rx_file_reader_thread(void *arg)
{
    sdr_receiver_t *rx = (sdr_receiver_t *)arg;
    const char *path = rx->config.ifile_path;
    const uint32_t buf_size = 262144;  // 256 KB per read
    uint8_t *buf = static_cast<uint8_t*>(malloc(buf_size));
    if (!buf) {
        gg::eprint("rx[%d]: file reader: failed to allocate buffer\n", rx->id);
        return NULL;
    }

    fprintf(stderr, "rx[%d]: file reader thread started (role=%s, file=%s)\n",
            rx->id, sdrRoleName(rx->config.role), path);

    uint32_t loop_count = 0;

    while (!Modes.exit && rx->state == RX_STATE_RUNNING) {
        FILE *fp = fopen(path, "rb");
        if (!fp) {
            fprintf(stderr, "rx[%d]: file reader: cannot open '%s': %s\n",
                    rx->id, path, strerror(errno));
            break;
        }

        if (loop_count > 0) {
            gg::eprint("rx[%d]: file reader: replaying (loop %u)\n", rx->id, loop_count);
        }
        loop_count++;

        while (!Modes.exit && rx->state == RX_STATE_RUNNING) {
            size_t nread = fread(buf, 1, buf_size, fp);
            if (nread == 0) break;  // EOF → loop
            nread &= ~(size_t)1;    // ensure even (IQ pairs)

            if (nread > 0 && rx->decoder_ops && rx->decoder_ops->process) {
                rx->decoder_ops->process(rx, buf, (uint32_t)nread);
            }
            rx->sample_counter += nread / 2;

            // Throttle to real-time pace
            double samples = nread / 2.0;
            double sleep_us = (samples / rx->config.sample_rate) * 1e6;
            if (sleep_us > 0) {
                struct timespec ts;
                ts.tv_sec = (time_t)(sleep_us / 1e6);
                ts.tv_nsec = (int64_t)(fmod(sleep_us, 1e6) * 1000);
                nanosleep(&ts, NULL);
            }
        }

        fclose(fp);
    }

    free(buf);
    gg::eprint("rx[%d]: file reader thread exiting (loops=%u)\n", rx->id, loop_count);
    return NULL;
}

bool rxStart(sdr_receiver_t *rx)
{
    if (rx->state != RX_STATE_OPEN) {
        gg::eprint("rx[%d]: cannot start, state is %s\n", rx->id, rxStateName(rx->state));
        return false;
    }

    rx->state = RX_STATE_RUNNING;

    // Virtual file device: use file reader thread
    void *(*thread_fn)(void *) = rx->config.ifile_path[0] ? rx_file_reader_thread : rx_reader_thread;

    if (pthread_create(&rx->thread, NULL, thread_fn, rx)) {
        gg::eprint("rx[%d]: failed to create reader thread: %s\n", rx->id, strerror(errno));
        rx->state = RX_STATE_ERROR;
        return false;
    }

    rx->thread_started = true;
    return true;
}

void rxStop(sdr_receiver_t *rx)
{
    if (rx->state != RX_STATE_RUNNING)
        return;

    rx->state = RX_STATE_STOPPING;

    if (rx->backend_dev && rx->backend_ops)
        rx->backend_ops->cancel_async(rx->backend_dev);

    rxFifoHalt(&rx->fifo);

    if (rx->thread_started) {
        pthread_join(rx->thread, NULL);
        rx->thread_started = false;
    }

    rx->state = RX_STATE_OPEN;
    gg::eprint("rx[%d]: stopped\n", rx->id);
}

void rxClose(sdr_receiver_t *rx)
{
    if (rx->state == RX_STATE_RUNNING)
        rxStop(rx);

    // Cleanup via decoder_ops (handles FIFO, converter, demod, etc.)
    if (rx->decoder_ops && rx->decoder_ops->stop) {
        rx->decoder_ops->stop(rx);
    }
    rx->decoder_ops = NULL;

    if (rx->backend_dev && rx->backend_ops) {
        rx->backend_ops->close(rx->backend_dev);
        rx->backend_dev = NULL;
        rx->rtl.dev = NULL;
    }

    free(rx->rtl.gains);
    rx->rtl.gains = NULL;

    rx->backend_ops = NULL;
    rx->state = RX_STATE_IDLE;
    gg::eprint("rx[%d]: closed\n", rx->id);
}

// Reconfigure a receiver in-place without closing/reopening the device.
// This avoids sdr::close() which can SEGV in libsdrgg when another device
// is still streaming on the same USB bus.
// Precondition: rx->state == RX_STATE_RUNNING (will be stopped internally).
bool rxReconfigure(sdr_receiver_t *rx, sdr_role_t new_role, double new_gain,
                   int32_t new_ppm, uint32_t new_freq, double new_sample_rate)
{
    if (rx->state == RX_STATE_RUNNING)
        rxStop(rx);

    // If reader thread died (e.g., start_stream failed), recover by joining it
    if (rx->state == RX_STATE_ERROR && rx->thread_started) {
        pthread_join(rx->thread, NULL);
        rx->thread_started = false;
        rx->state = RX_STATE_OPEN;
        gg::eprint("rx[%d]: rxReconfigure: recovered from error state\n", rx->id);
    }

    if (rx->state != RX_STATE_OPEN) {
        fprintf(stderr, "rx[%d]: rxReconfigure: unexpected state %s\n",
                rx->id, rxStateName(rx->state));
        return false;
    }

    // Stop old decoder
    if (rx->decoder_ops && rx->decoder_ops->stop) {
        rx->decoder_ops->stop(rx);
    }
    rx->decoder_ops = NULL;

    // Update config
    rx->config.role = new_role;
    rx->config.gain = new_gain;
    rx->config.ppm_error = new_ppm;
    rx->config.freq = new_freq;
    rx->config.sample_rate = new_sample_rate;

    // Reconfigure device (it's still open)
    const sdr_backend_ops_t *ops = rx->backend_ops;
    sdr_device_t *sdev = rx->backend_dev;

    // Gain: find closest step
    if (rx->rtl.gains && rx->rtl.gain_steps > 0) {
        int32_t numgains = rx->rtl.gain_steps;
        bool supports_tuner_agc = sdev->supports_tuner_agc;
        int32_t selected = -1;
        if (rx->config.gain == MODES_LEGACY_AUTO_GAIN) {
            selected = numgains - 1;
        } else if (rx->config.gain == MODES_DEFAULT_GAIN) {
            selected = supports_tuner_agc ? (numgains - 2) : (numgains - 1);
        } else {
            for (int32_t i = 0; i < numgains; ++i) {
                if (selected == -1 || fabs(rx->rtl.gains[i] / 10.0 - rx->config.gain) <
                                      fabs(rx->rtl.gains[selected] / 10.0 - rx->config.gain))
                    selected = i;
            }
        }
        rxSetGain(rx, selected);
    }

    ops->set_freq_correction(sdev, rx->config.ppm_error);
    ops->set_frequency(sdev, rx->config.freq);
    if (ops->set_sample_rate(sdev, (uint32_t)rx->config.sample_rate) != 0) {
        fprintf(stderr, "rx[%d]: rxReconfigure: failed to set sample rate %.0f for role %s\n",
                rx->id, rx->config.sample_rate, sdrRoleName(rx->config.role));
        rx->state = RX_STATE_ERROR;
        return false;
    }
    ops->reset_buffer(sdev);

    // Create new decoder
    rx->decoder_ops = decoderOpsForRole(rx->config.role);
    if (rx->decoder_ops) {
        if (!rx->decoder_ops->init(rx)) {
            fprintf(stderr, "rx[%d]: rxReconfigure: can't init decoder for role %s\n",
                    rx->id, sdrRoleName(rx->config.role));
            rx->state = RX_STATE_ERROR;
            return false;
        }
        // Some decoders modify freq (e.g., GSM IF offset, POCSAG center)
        if (ops->get_frequency(sdev) != (uint32_t)rx->config.freq) {
            ops->set_frequency(sdev, rx->config.freq);
        }
    }

    rx->dropped = 0;
    rx->sample_counter = 0;

    fprintf(stderr, "rx[%d]: reconfigured to role=%s freq=%d rate=%.0f\n",
            rx->id, sdrRoleName(rx->config.role), rx->config.freq, rx->config.sample_rate);
    return true;
}

int32_t sdrEnumerateDevices(char serials[][64], int32_t max_devices)
{
    sdr_dev_info_t devs[MAX_SDR_RECEIVERS];
    int32_t count = sdrBackendEnumerateAll(devs, max_devices < MAX_SDR_RECEIVERS ? max_devices : MAX_SDR_RECEIVERS);
    int32_t filled = 0;
    for (int32_t i = 0; i < count && filled < max_devices; i++) {
        snprintf(serials[filled], 64, "%.63s", devs[i].serial);
        filled++;
    }
    return filled;
}

#else // !ENABLE_RTLSDR && !ENABLE_SDRGG — stubs

int32_t rxGetGain(sdr_receiver_t *rx)     { MODES_NOTUSED(rx); return -1; }
int32_t rxGetMaxGain(sdr_receiver_t *rx)  { MODES_NOTUSED(rx); return -1; }
double rxGetGainDb(sdr_receiver_t *rx, int32_t step) { MODES_NOTUSED(rx); MODES_NOTUSED(step); return 0.0; }
int32_t rxSetGain(sdr_receiver_t *rx, int32_t step) { MODES_NOTUSED(rx); MODES_NOTUSED(step); return -1; }
bool rxOpen(sdr_receiver_t *rx)       { gg::eprint("rx[%d]: no SDR backend compiled\n", rx->id); rx->state = RX_STATE_ERROR; return false; }
bool rxStart(sdr_receiver_t *rx)      { MODES_NOTUSED(rx); return false; }
void rxStop(sdr_receiver_t *rx)       { MODES_NOTUSED(rx); }
void rxClose(sdr_receiver_t *rx)      { MODES_NOTUSED(rx); rx->state = RX_STATE_IDLE; }
bool rxReconfigure(sdr_receiver_t *rx, sdr_role_t r, double g, int32_t p, uint32_t f, double s) { MODES_NOTUSED(rx); MODES_NOTUSED(r); MODES_NOTUSED(g); MODES_NOTUSED(p); MODES_NOTUSED(f); MODES_NOTUSED(s); return false; }
int32_t sdrEnumerateDevices(char serials[][64], int32_t max_devices) { MODES_NOTUSED(serials); MODES_NOTUSED(max_devices); return 0; }

#endif // ENABLE_RTLSDR || ENABLE_SDRGG

// ======================== Internal Decoder Integration ========================

#include "acars_demod.h"
#include "vdl2_demod.h"
#include "sonde_demod.h"

// ---- Queue-based decoder wrapper contexts ----
// Each decoder's output goes through a msg_queue so that processing (reader thread)
// is decoupled from consumption (main thread drain).

typedef struct {
    struct acars_state *inner;
    msg_queue_t         queue;
    sdr_receiver_t     *rx;
} acars_ctx_t;

typedef struct {
    struct vdl2_state  *inner;
    msg_queue_t         queue;
    sdr_receiver_t     *rx;
} vdl2_ctx_t;

typedef struct {
    struct sonde_state *inner;
    msg_queue_t         queue;
    sdr_receiver_t     *rx;
    aircraft_queue_handle_t aircraft_queue;
} sonde_ctx_t;

typedef struct {
    struct pocsag_state *inner;
    msg_queue_t          queue;
    sdr_receiver_t      *rx;
} pocsag_ctx_t;

// GSM queue item types
typedef struct {
    gsm_cell_info_t cell;
    char            msg_type[32];
} gsm_msg_item_t;

typedef struct {
    gsm_cell_info_t cell;
    gsm_cb_msg_t    cb;
} gsm_cb_item_t;

typedef struct {
    struct gsm_state *inner;
    msg_queue_t       msg_queue;   // for gsm_msg_item_t
    msg_queue_t       cb_queue;    // for gsm_cb_item_t
    sdr_receiver_t   *rx;
} gsm_ctx_t;

typedef struct {
    struct lte_state *inner;
    msg_queue_t       queue;
    sdr_receiver_t   *rx;
} lte_ctx_t;

typedef struct {
    struct sarsat_state *inner;
    msg_queue_t          queue;
    sdr_receiver_t      *rx;
} sarsat_ctx_t;

static void acars_queue_cb(const acars_msg_t *msg, void *ctx) {
    acars_ctx_t *c = (acars_ctx_t *)ctx;
    msg_queue_push(c->queue, msg);
}

static void vdl2_queue_cb(const vdl2_msg_t *msg, void *ctx) {
    vdl2_ctx_t *c = (vdl2_ctx_t *)ctx;
    msg_queue_push(c->queue, msg);
}

static void sonde_queue_cb(const sonde_msg_t *msg, void *ctx) {
    sonde_ctx_t *c = (sonde_ctx_t *)ctx;
    msg_queue_push(c->queue, msg);
}

static void pocsag_queue_cb(const pocsag_msg_t *msg, void *ctx) {
    pocsag_ctx_t *c = (pocsag_ctx_t *)ctx;
    msg_queue_push(c->queue, msg);
}

static void gsm_msg_queue_cb(const gsm_cell_info_t *cell, const char *msg_type,
                              const uint8_t *l3_data, int32_t l3_len, void *ctx) {
    (void)l3_data; (void)l3_len;
    gsm_ctx_t *c = (gsm_ctx_t *)ctx;
    gsm_msg_item_t item;
    item.cell = *cell;
    snprintf(item.msg_type, sizeof(item.msg_type), "%s", msg_type);
    msg_queue_push(c->msg_queue, &item);
}

static void gsm_cb_queue_cb(const gsm_cell_info_t *cell, const gsm_cb_msg_t *cb, void *ctx) {
    gsm_ctx_t *c = (gsm_ctx_t *)ctx;
    gsm_cb_item_t item;
    item.cell = *cell;
    item.cb = *cb;
    msg_queue_push(c->cb_queue, &item);
}

static void lte_queue_cb(const lte_cell_info_t *cell, void *ctx) {
    lte_ctx_t *c = (lte_ctx_t *)ctx;
    msg_queue_push(c->queue, cell);
}

static void sarsat_queue_cb(const sarsat_msg_t *msg, void *ctx) {
    sarsat_ctx_t *c = (sarsat_ctx_t *)ctx;
    msg_queue_push(c->queue, msg);
}

bool rxDecoderCreate(sdr_receiver_t *rx)
{
    switch (rx->config.role) {
    case SDR_ROLE_ACARS: {
        acars_ctx_t *ctx = static_cast<acars_ctx_t*>(calloc(1, sizeof(*ctx)));
        if (!ctx) return false;
        ctx->rx = rx;
        ctx->queue = msg_queue_create(sizeof(acars_msg_t), 64);
        if (!ctx->queue) { free(ctx); return false; }

        acars_config_t cfg;
        cfg = {};
        cfg.center_freq = rx->config.freq;
        cfg.sample_rate = rx->config.sample_rate;
        // Default European ACARS channels (within ~1 MHz of center)
        cfg.channel_freqs[0] = ACARS_FREQ_PRIMARY;    // 131.550
        cfg.channel_freqs[1] = ACARS_FREQ_SECONDARY;  // 130.025
        cfg.channel_freqs[2] = ACARS_FREQ_TERTIARY;   // 131.725
        cfg.channel_freqs[3] = ACARS_FREQ_4;          // 130.450
        cfg.channel_freqs[4] = ACARS_FREQ_5;          // 129.125
        cfg.num_channels = 5;
        cfg.callback = acars_queue_cb;
        cfg.callback_ctx = ctx;

        // Adjust center freq to cover all channels
        // Center between min and max channel freq
        double fmin = cfg.channel_freqs[4]; // 129.125
        double fmax = cfg.channel_freqs[2]; // 131.725
        cfg.center_freq = (fmin + fmax) / 2.0;
        // Update the receiver's actual center freq
        rx->config.freq = (uint32_t)cfg.center_freq;

        ctx->inner = acars_create(&cfg);
        if (!ctx->inner) { msg_queue_destroy(ctx->queue); free(ctx); return false; }
        rx->decoder_state = ctx;
        fprintf(stderr, "rx[%d]: ACARS decoder created, center=%.3f MHz, %d channels\n",
                rx->id, cfg.center_freq / 1e6, cfg.num_channels);
        return true;
    }
    case SDR_ROLE_VDL2: {
        vdl2_ctx_t *ctx = static_cast<vdl2_ctx_t*>(calloc(1, sizeof(*ctx)));
        if (!ctx) return false;
        ctx->rx = rx;
        ctx->queue = msg_queue_create(sizeof(vdl2_msg_t), 64);
        if (!ctx->queue) { free(ctx); return false; }

        vdl2_config_t cfg;
        cfg = {};
        cfg.center_freq = rx->config.freq;
        cfg.sample_rate = rx->config.sample_rate;
        cfg.num_channels = 3;
        cfg.channel_freqs[0] = VDL2_FREQ_EU_1;   // 136.975 MHz
        cfg.channel_freqs[1] = VDL2_FREQ_EU_2;   // 136.875 MHz
        cfg.channel_freqs[2] = VDL2_FREQ_EU_3;   // 136.775 MHz
        cfg.squelch_level = -32.0f;               // -32 dBFS squelch
        cfg.callback = vdl2_queue_cb;
        cfg.callback_ctx = ctx;

        ctx->inner = vdl2_create(&cfg);
        if (!ctx->inner) { msg_queue_destroy(ctx->queue); free(ctx); return false; }
        rx->decoder_state = ctx;
        fprintf(stderr, "rx[%d]: VDL2 decoder created, %d channels\n",
                rx->id, cfg.num_channels);
        return true;
    }
    case SDR_ROLE_RADIOSONDE: {
        sonde_ctx_t *ctx = static_cast<sonde_ctx_t*>(calloc(1, sizeof(*ctx)));
        if (!ctx) return false;
        ctx->rx = rx;
        ctx->queue = msg_queue_create(sizeof(sonde_msg_t), 32);
        if (!ctx->queue) { free(ctx); return false; }

        sonde_config_t cfg;
        cfg = {};
        cfg.center_freq = rx->config.freq;
        cfg.sample_rate = rx->config.sample_rate;
        cfg.callback = sonde_queue_cb;
        cfg.callback_ctx = ctx;
        cfg.scan_enabled = (DecoderConfigs.radiosonde.freq_mode == SONDE_FREQ_MODE_SCAN);
        cfg.scan_dwell_sec = DecoderConfigs.radiosonde.scan_dwell_sec;

        ctx->inner = sonde_create(&cfg);
        if (!ctx->inner) { msg_queue_destroy(ctx->queue); free(ctx); return false; }
        rx->decoder_state = ctx;
        fprintf(stderr, "rx[%d]: Radiosonde decoder created, freq=%.3f MHz%s\n",
                rx->id, cfg.center_freq / 1e6,
                cfg.scan_enabled ? " (SCAN)" : "");
        return true;
    }
    case SDR_ROLE_POCSAG: {
        pocsag_ctx_t *ctx = static_cast<pocsag_ctx_t*>(calloc(1, sizeof(*ctx)));
        if (!ctx) return false;
        ctx->rx = rx;
        ctx->queue = msg_queue_create(sizeof(pocsag_msg_t), 64);
        if (!ctx->queue) { free(ctx); return false; }

        pocsag_config_t cfg;
        cfg = {};
        cfg.center_freq = rx->config.freq;
        cfg.sample_rate = rx->config.sample_rate;
        cfg.callback = pocsag_queue_cb;
        cfg.callback_ctx = ctx;

        // Multi-channel: German BOS POCSAG frequencies within the 2.4 MHz band
        // Center at 466.150 MHz covers 466.0–466.3 MHz
        // Default channels: 466.075, 466.175, 466.225 MHz
        if (cfg.sample_rate >= 2000000) {
            cfg.channel_freqs[0] = 466075000;  // BOS national (Feuerwehr, DRK, THW)
            cfg.channel_freqs[1] = 466175000;  // BOS regional
            cfg.channel_freqs[2] = 466225000;  // Baden-Württemberg
            cfg.num_channels = 3;
            // Adjust center frequency to cover all channels
            cfg.center_freq = 466150000;
            rx->config.freq = (uint32_t)cfg.center_freq;  // retune SDR
            fprintf(stderr, "rx[%d]: POCSAG multi-channel mode, center=%.3f MHz, %d channels\n",
                    rx->id, cfg.center_freq / 1e6, cfg.num_channels);
        }

        ctx->inner = pocsag_create(&cfg);
        if (!ctx->inner) { msg_queue_destroy(ctx->queue); free(ctx); return false; }
        rx->decoder_state = ctx;
        fprintf(stderr, "rx[%d]: POCSAG decoder created, freq=%.3f MHz, sr=%.0f\n",
                rx->id, cfg.center_freq / 1e6, cfg.sample_rate);
        return true;
    }
    case SDR_ROLE_GSM: {
        gsm_ctx_t *ctx = static_cast<gsm_ctx_t*>(calloc(1, sizeof(*ctx)));
        if (!ctx) return false;
        ctx->rx = rx;
        ctx->msg_queue = msg_queue_create(sizeof(gsm_msg_item_t), 32);
        ctx->cb_queue = msg_queue_create(sizeof(gsm_cb_item_t), 16);
        if (!ctx->msg_queue || !ctx->cb_queue) {
            msg_queue_destroy(ctx->msg_queue);
            msg_queue_destroy(ctx->cb_queue);
            free(ctx);
            return false;
        }

        gsm_config_t cfg;
        cfg = {};
        cfg.arfcn_freq = rx->config.freq;
        cfg.center_freq = rx->config.freq - GSM_IF_OFFSET;
        cfg.sample_rate = rx->config.sample_rate;
        cfg.tsc = -1; // auto-detect
        cfg.msg_cb = gsm_msg_queue_cb;
        cfg.cb_cb = gsm_cb_queue_cb;
        cfg.callback_ctx = ctx;

        ctx->inner = gsm_create(&cfg);
        if (!ctx->inner) {
            msg_queue_destroy(ctx->msg_queue);
            msg_queue_destroy(ctx->cb_queue);
            free(ctx);
            return false;
        }
        rx->decoder_state = ctx;
        /* Retune to IF-offset frequency to avoid DC spike */
        rx->config.freq = (uint32_t)cfg.center_freq;
        fprintf(stderr, "rx[%d]: GSM decoder created, arfcn=%.3f MHz, tuned=%.3f MHz (IF offset %d Hz), sr=%.0f\n",
                rx->id, cfg.arfcn_freq / 1e6, cfg.center_freq / 1e6, GSM_IF_OFFSET, cfg.sample_rate);
        return true;
    }
    case SDR_ROLE_LTE: {
        lte_ctx_t *ctx = static_cast<lte_ctx_t*>(calloc(1, sizeof(*ctx)));
        if (!ctx) return false;
        ctx->rx = rx;
        ctx->queue = msg_queue_create(sizeof(lte_cell_info_t), 16);
        if (!ctx->queue) { free(ctx); return false; }

        lte_config_t cfg;
        cfg = {};
        cfg.center_freq = rx->config.freq;
        cfg.sample_rate = rx->config.sample_rate;
        cfg.callback = lte_queue_cb;
        cfg.callback_ctx = ctx;
        cfg.hop_enabled = true;  // Enable Band 20 hopping (796/806/816 MHz)

        ctx->inner = lte_create(&cfg);
        if (!ctx->inner) { msg_queue_destroy(ctx->queue); free(ctx); return false; }
        rx->decoder_state = ctx;
        fprintf(stderr, "rx[%d]: LTE decoder created, freq=%.3f MHz, sr=%.0f, hop=on\n",
                rx->id, cfg.center_freq / 1e6, cfg.sample_rate);
        return true;
    }
    case SDR_ROLE_IOT868: {
        rx->decoder_state = iotDecoderCreate((uint32_t)rx->config.sample_rate);
        if (!rx->decoder_state) return false;
        fprintf(stderr, "rx[%d]: IoT 868 MHz decoder created, freq=%.3f MHz, sr=%.0f\n",
                rx->id, rx->config.freq / 1e6, rx->config.sample_rate);
        return true;
    }
    case SDR_ROLE_FANET: {
        rx->decoder_state = fanet_create((uint32_t)rx->config.sample_rate);
        if (!rx->decoder_state) return false;
        fprintf(stderr, "rx[%d]: FANET LoRa decoder created, freq=%.3f MHz, sr=%.0f\n",
                rx->id, rx->config.freq / 1e6, rx->config.sample_rate);
        return true;
    }
    case SDR_ROLE_SARSAT: {
        sarsat_ctx_t *ctx = static_cast<sarsat_ctx_t*>(calloc(1, sizeof(*ctx)));
        if (!ctx) return false;
        ctx->rx = rx;
        ctx->queue = msg_queue_create(sizeof(sarsat_msg_t), 16);
        if (!ctx->queue) { free(ctx); return false; }

        sarsat_config_t cfg;
        cfg = {};
        cfg.center_freq = rx->config.freq;
        cfg.sample_rate = rx->config.sample_rate;
        cfg.callback = sarsat_queue_cb;
        cfg.callback_ctx = ctx;

        ctx->inner = sarsat_create(&cfg);
        if (!ctx->inner) { msg_queue_destroy(ctx->queue); free(ctx); return false; }
        rx->decoder_state = ctx;
        fprintf(stderr, "rx[%d]: Sarsat 406 MHz decoder created, freq=%.3f MHz\n",
                rx->id, cfg.center_freq / 1e6);
        return true;
    }
    default:
        return false;
    }
}

void rxDecoderDestroy(sdr_receiver_t *rx)
{
    if (!rx->decoder_state) return;

    switch (rx->config.role) {
    case SDR_ROLE_ACARS: {
        acars_ctx_t *ctx = (acars_ctx_t *)rx->decoder_state;
        acars_destroy(ctx->inner);
        msg_queue_destroy(ctx->queue);
        free(ctx);
        break;
    }
    case SDR_ROLE_VDL2: {
        vdl2_ctx_t *ctx = (vdl2_ctx_t *)rx->decoder_state;
        vdl2_destroy(ctx->inner);
        msg_queue_destroy(ctx->queue);
        free(ctx);
        break;
    }
    case SDR_ROLE_RADIOSONDE: {
        sonde_ctx_t *ctx = (sonde_ctx_t *)rx->decoder_state;
        sonde_destroy(ctx->inner);
        msg_queue_destroy(ctx->queue);
        free(ctx);
        break;
    }
    case SDR_ROLE_POCSAG: {
        pocsag_ctx_t *ctx = (pocsag_ctx_t *)rx->decoder_state;
        pocsag_destroy(ctx->inner);
        msg_queue_destroy(ctx->queue);
        free(ctx);
        break;
    }
    case SDR_ROLE_GSM: {
        gsm_ctx_t *ctx = (gsm_ctx_t *)rx->decoder_state;
        gsm_destroy(ctx->inner);
        msg_queue_destroy(ctx->msg_queue);
        msg_queue_destroy(ctx->cb_queue);
        free(ctx);
        break;
    }
    case SDR_ROLE_LTE: {
        lte_ctx_t *ctx = (lte_ctx_t *)rx->decoder_state;
        lte_destroy(ctx->inner);
        msg_queue_destroy(ctx->queue);
        free(ctx);
        break;
    }
    case SDR_ROLE_IOT868:
        iotDecoderDestroy((iot_decoder_state_t *)rx->decoder_state);
        break;
    case SDR_ROLE_FANET:
        fanet_destroy((fanet_state_t *)rx->decoder_state);
        break;
    case SDR_ROLE_SARSAT: {
        sarsat_ctx_t *ctx = (sarsat_ctx_t *)rx->decoder_state;
        sarsat_destroy(ctx->inner);
        msg_queue_destroy(ctx->queue);
        free(ctx);
        break;
    }
    default:
        break;
    }
    rx->decoder_state = NULL;
}

void rxDecoderProcess(sdr_receiver_t *rx, const uint8_t *iq_data, uint32_t len)
{
    if (!rx->decoder_state) return;

    switch (rx->config.role) {
    case SDR_ROLE_ACARS:
        acars_process(((acars_ctx_t *)rx->decoder_state)->inner, iq_data, len);
        break;
    case SDR_ROLE_VDL2:
        vdl2_process(((vdl2_ctx_t *)rx->decoder_state)->inner, iq_data, len);
        break;
    case SDR_ROLE_RADIOSONDE: {
        sonde_ctx_t *ctx = (sonde_ctx_t *)rx->decoder_state;
        sonde_process(ctx->inner, iq_data, len);
        // Check if scanner requests a frequency hop
        uint32_t scan_freq = sonde_get_scan_freq(ctx->inner);
        if (scan_freq > 0) {
            rx->pending_freq = scan_freq;
        }
        break;
    }
    case SDR_ROLE_POCSAG:
        pocsag_process(((pocsag_ctx_t *)rx->decoder_state)->inner, iq_data, len);
        break;
    case SDR_ROLE_GSM:
        gsm_process(((gsm_ctx_t *)rx->decoder_state)->inner, iq_data, len);
        break;
    case SDR_ROLE_LTE: {
        lte_ctx_t *ctx = (lte_ctx_t *)rx->decoder_state;
        lte_process(ctx->inner, iq_data, len);
        // Check if decoder requests a frequency hop (deferred retune)
        double hop_freq = lte_get_hop_freq(ctx->inner);
        if (hop_freq > 0) {
            rx->pending_freq = (uint32_t)hop_freq;
            lte_set_freq(ctx->inner, hop_freq);
        }
        break;
    }
    case SDR_ROLE_IOT868:
        iotDecoderProcess((iot_decoder_state_t *)rx->decoder_state, iq_data, len);
        break;
    case SDR_ROLE_FANET:
        fanet_process((fanet_state_t *)rx->decoder_state, iq_data, len);
        break;
    case SDR_ROLE_SARSAT:
        sarsat_process(((sarsat_ctx_t *)rx->decoder_state)->inner, iq_data, len);
        break;
    default:
        break;
    }
}
//
// Thin wrappers that adapt the existing decoder/FIFO code to the decoder_ops_t
// plugin interface, so that all roles (ADSB, FLARM, ACARS, VDL2, Radiosonde, POCSAG)
// can be managed uniformly by SdrManager.

#include "flarm_reader.h"  // flarmDecoderInit/Process/Drain/Stop
#include "flarm_decode.h"  // FLARM_CENTER_FREQ*, FLARM_SAMPLE_RATE*
#include "demod_2400.h"     // demodulate2400
#include "gg_format.h"

// ---- ACARS decoder_ops ----

static bool acarsDecoderInit(sdr_receiver_t *rx)    { return rxDecoderCreate(rx); }
static void acarsDecoderProcess(sdr_receiver_t *rx, const uint8_t *iq, uint32_t len) { rxDecoderProcess(rx, iq, len); }
static bool acarsDecoderDrain(sdr_receiver_t *rx) {
    acars_ctx_t *ctx = (acars_ctx_t *)rx->decoder_state;
    if (!ctx) return false;
    acars_msg_t msg;
    bool had_data = false;
    while (msg_queue_pop(ctx->queue, &msg)) {
        had_data = true;
        panelLogMessage("[ACARS rx%d] %.3f MHz %s %s (reg:%s) [%s%s%s]%s%s%s%s%s%s %s",
                        ctx->rx->dev_index, msg.freq / 1e6,
                        msg.flight[0] ? msg.flight : "???",
                        msg.label,
                        msg.reg[0] ? msg.reg : "?",
                        msg.label,
                        msg.label_description ? "=" : "",
                        msg.label_description ? msg.label_description : "",
                        msg.dsp_header[0] ? " route=" : "",
                        msg.dsp_header[0] ? msg.dsp_header : "",
                        msg.sublabel[0] ? " sub=" : "",
                        msg.sublabel[0] ? msg.sublabel : "",
                        msg.mfi[0] ? " mfi=" : "",
                        msg.mfi[0] ? msg.mfi : "",
                        msg.text[0] ? msg.text : "(empty)");
        airframesFeedSendAcars(&msg);
    }
    return had_data;
}
static void acarsDecoderStop(sdr_receiver_t *rx)     { rxDecoderDestroy(rx); }

static const decoder_ops_t acars_decoder_ops = {
    .name    = "acars",
    .init    = acarsDecoderInit,
    .process = acarsDecoderProcess,
    .drain   = acarsDecoderDrain,
    .stop    = acarsDecoderStop,
};

// ---- VDL2 decoder_ops ----

static bool vdl2DecoderInit(sdr_receiver_t *rx)    { return rxDecoderCreate(rx); }
static void vdl2DecoderProcess(sdr_receiver_t *rx, const uint8_t *iq, uint32_t len) { rxDecoderProcess(rx, iq, len); }
static bool vdl2DecoderDrain(sdr_receiver_t *rx) {
    vdl2_ctx_t *ctx = (vdl2_ctx_t *)rx->decoder_state;
    if (!ctx) return false;
    vdl2_msg_t msg;
    bool had_data = false;
    while (msg_queue_pop(ctx->queue, &msg)) {
        had_data = true;
        if (msg.has_acars) {
            panelLogMessage("[VDL2 rx%d] %.3f MHz %s %s (reg:%s) [%s%s%s] SNR=%.0fdB %s",
                            ctx->rx->dev_index, msg.freq / 1e6,
                            msg.frame_type,
                            msg.flight[0] ? msg.flight : "???",
                            msg.reg[0] ? msg.reg : "?",
                            msg.label,
                            msg.label_description ? "=" : "",
                            msg.label_description ? msg.label_description : "",
                            msg.snr,
                            msg.text[0] ? msg.text : "(no text)");
        } else {
            panelLogMessage("[VDL2 rx%d] %.3f MHz %s%s%s src=%06X dst=%06X SNR=%.0fdB [%d bytes]%s%s",
                            ctx->rx->dev_index, msg.freq / 1e6,
                            msg.frame_type,
                            msg.proto_name ? " " : "",
                            msg.proto_name ? msg.proto_name : "",
                            msg.src.addr,
                            msg.dst.addr,
                            msg.snr,
                            msg.info_len,
                            (msg.proto != VDL2_PROTO_UNKNOWN && msg.text[0]) ? " " : "",
                            (msg.proto != VDL2_PROTO_UNKNOWN && msg.text[0]) ? msg.text : "");
        }
        airframesFeedSendVdl2(&msg);
    }
    return had_data;
}
static void vdl2DecoderStop(sdr_receiver_t *rx)     { rxDecoderDestroy(rx); }

static const decoder_ops_t vdl2_decoder_ops = {
    .name    = "vdl2",
    .init    = vdl2DecoderInit,
    .process = vdl2DecoderProcess,
    .drain   = vdl2DecoderDrain,
    .stop    = vdl2DecoderStop,
};

// ---- Radiosonde decoder_ops ----

static bool sondeDecoderInit(sdr_receiver_t *rx)    { return rxDecoderCreate(rx); }
static void sondeDecoderProcess(sdr_receiver_t *rx, const uint8_t *iq, uint32_t len) { rxDecoderProcess(rx, iq, len); }
static bool sondeDecoderDrain(sdr_receiver_t *rx) {
    sonde_ctx_t *ctx = (sonde_ctx_t *)rx->decoder_state;
    if (!ctx) return false;
    sonde_msg_t msg;
    bool had_data = false;
    while (msg_queue_pop(ctx->queue, &msg)) {
        had_data = true;
        if (msg.valid_pos) {
            panelLogMessage("[SONDE rx%d] %s %s pos=%.4f,%.4f alt=%.0fm vel=%.1fm/s frame=%d",
                            ctx->rx->dev_index, msg.type, msg.serial,
                            msg.lat, msg.lon, msg.alt,
                            msg.vel_h, msg.frame_num);
            sondehubClientSubmit(&msg);

            // Inject into aircraft tracking list
            if (!ctx->aircraft_queue)
                ctx->aircraft_queue = dispatcher_register_aircraft_queue("sonde");
            if (ctx->aircraft_queue) {
                aircraft_update_t upd;
                upd = {};

                // Generate pseudo-ICAO from serial hash (fd0000-fdFFFF range)
                uint32_t hash = 0;
                for (int32_t i = 0; msg.serial[i]; i++)
                    hash = hash * 31 + (uint8_t)msg.serial[i];
                upd.addr = 0xFD0000 | (hash & 0xFFFF);
                upd.timestamp_ms = mstime();
                upd.signal_level = 0.01;
                upd.source = DECODE_SOURCE_RADIOSONDE;

                // Callsign = serial
                snprintf(upd.callsign, sizeof(upd.callsign), "%.8s", msg.serial);
                upd.callsign_valid = 1;

                // Category B1 (meteorological balloon/radiosonde)
                upd.category = 0xB1;
                upd.category_valid = 1;

                // Position
                upd.lat = msg.lat;
                upd.lon = msg.lon;
                upd.position_valid = 1;

                // Altitude (meters → feet, geometric/GPS)
                upd.altitude_ft = (int32_t)(msg.alt * 3.28084);
                upd.altitude_valid = 1;
                upd.altitude_is_baro = 0;

                // Velocity
                if (msg.vel_h > 0.1 || fabs(msg.vel_v) > 0.1) {
                    upd.ground_speed_kt = (int32_t)(msg.vel_h * 1.94384);
                    upd.heading_deg = (int32_t)msg.heading;
                    upd.vert_rate_fpm = (int32_t)(msg.vel_v * 196.85);
                    upd.velocity_valid = 1;
                }

                upd.air_ground = DECODE_AG_AIRBORNE;

                // Sonde metadata
                upd.sonde.valid = true;
                snprintf(upd.sonde.serial, sizeof(upd.sonde.serial), "%.15s", msg.serial);
                snprintf(upd.sonde.sonde_type, sizeof(upd.sonde.sonde_type), "%.7s", msg.type);
                upd.sonde.frame_num = msg.frame_num;
                upd.sonde.rs_errors = msg.rs_errors;
                upd.sonde.satellites = msg.satellites;
                upd.sonde.vel_v = msg.vel_v;
                upd.sonde.freq_mhz = msg.freq;

                dispatcher_push_aircraft(ctx->aircraft_queue, &upd);
            }
        } else {
            panelLogMessage("[SONDE rx%d] %s %s frame=%d (no GPS fix)",
                            ctx->rx->dev_index, msg.type, msg.serial, msg.frame_num);
        }
    }
    return had_data;
}
static void sondeDecoderStop(sdr_receiver_t *rx)     { rxDecoderDestroy(rx); }

static const decoder_ops_t sonde_decoder_ops = {
    .name    = "radiosonde",
    .init    = sondeDecoderInit,
    .process = sondeDecoderProcess,
    .drain   = sondeDecoderDrain,
    .stop    = sondeDecoderStop,
};

// ---- POCSAG decoder_ops ----

static bool pocsagDecoderInit(sdr_receiver_t *rx)    { return rxDecoderCreate(rx); }
static void pocsagDecoderProcess(sdr_receiver_t *rx, const uint8_t *iq, uint32_t len) { rxDecoderProcess(rx, iq, len); }
static bool pocsagDecoderDrain(sdr_receiver_t *rx) {
    pocsag_ctx_t *ctx = (pocsag_ctx_t *)rx->decoder_state;
    if (!ctx) return false;

    // Keep watchdog alive: return true if the receiver has processed IQ data
    // even when no POCSAG messages are decoded (quiet channel is normal)
    static uint64_t last_sample_counter = 0;
    bool receiver_active = (rx->sample_counter > last_sample_counter);
    last_sample_counter = rx->sample_counter;

    if (!PocsagOutputEnabled) return receiver_active;
    pocsag_msg_t msg;
    bool had_data = false;
    while (msg_queue_pop(ctx->queue, &msg)) {
        had_data = true;
        const char *freq_str = "";
        char freq_buf[32];
        if (msg.channel_freq > 0) {
            snprintf(freq_buf, sizeof(freq_buf), " %.3fMHz", msg.channel_freq / 1e6);
            freq_str = freq_buf;
        }
        if (msg.is_tone_only) {
            panelLogMessage("[POCSAG rx%d]%s %d baud addr=%07u func=%d TONE-ONLY",
                            ctx->rx->dev_index, freq_str, msg.baud_rate, msg.address, msg.function);
        } else if (msg.is_alpha && msg.alpha_len > 0) {
            panelLogMessage("[POCSAG rx%d]%s %d baud addr=%07u func=%d \"%s\"",
                            ctx->rx->dev_index, freq_str, msg.baud_rate, msg.address, msg.function,
                            msg.alpha_msg);
        } else if (msg.is_numeric && msg.numeric_len > 0) {
            panelLogMessage("[POCSAG rx%d]%s %d baud addr=%07u func=%d num=%s",
                            ctx->rx->dev_index, freq_str, msg.baud_rate, msg.address, msg.function,
                            msg.numeric_msg);
        } else {
            panelLogMessage("[POCSAG rx%d]%s %d baud addr=%07u func=%d (empty)",
                            ctx->rx->dev_index, freq_str, msg.baud_rate, msg.address, msg.function);
        }
    }
    return had_data || receiver_active;
}
static void pocsagDecoderStop(sdr_receiver_t *rx)
{
    pocsag_ctx_t *ctx = (pocsag_ctx_t *)rx->decoder_state;
    if (ctx && ctx->inner) {
        pocsag_stats_t stats;
        pocsag_get_stats(ctx->inner, &stats);
        fprintf(stderr, "rx[%d]: POCSAG stats: samples=%" PRIu64 " preambles=%" PRIu64 " sync=%" PRIu64 " decoded=%" PRIu64 " bch_corr=%" PRIu64 " bch_fail=%" PRIu64 "\n",
                rx->id,
                (uint64_t)stats.samples_processed,
                (uint64_t)stats.preambles_detected,
                (uint64_t)stats.syncs_detected,
                (uint64_t)stats.messages_decoded,
                (uint64_t)stats.bch_corrections,
                (uint64_t)stats.bch_failures);
    }
    rxDecoderDestroy(rx);
}

static const decoder_ops_t pocsag_decoder_ops = {
    .name    = "pocsag",
    .init    = pocsagDecoderInit,
    .process = pocsagDecoderProcess,
    .drain   = pocsagDecoderDrain,
    .stop    = pocsagDecoderStop,
};

// ---- GSM decoder_ops ----

static bool gsmDecoderInit(sdr_receiver_t *rx)    { return rxDecoderCreate(rx); }
static void gsmDecoderProcess(sdr_receiver_t *rx, const uint8_t *iq, uint32_t len) { rxDecoderProcess(rx, iq, len); }
static bool gsmDecoderDrain(sdr_receiver_t *rx) {
    gsm_ctx_t *ctx = (gsm_ctx_t *)rx->decoder_state;
    if (!ctx) return false;

    // Periodic FCCH-only tracker update (moved from process to drain for thread safety)
    {
        static uint64_t gsm_tracker_last = 0;
        gsm_stats_t gstats;
        gsm_get_stats(ctx->inner, &gstats);
        if (gstats.fcch_detected > 0 && gstats.samples_processed - gsm_tracker_last >= 5000000) {
            gsm_tracker_last = gstats.samples_processed;
            gsm_cell_info_t gcell;
            gsm_get_cell_info(ctx->inner, &gcell);
            gsm_sync_state_t gsync = gsm_get_sync_state(ctx->inner);
            if (gcell.si3.mcc == 0) {
                gsmTrackerUpdateFCCH(gcell.arfcn, gcell.freq_mhz, &gstats, gsync);
            }
        }
    }

    if (!GsmOutputEnabled) return false;

    // Drain regular GSM messages
    bool had_data = false;
    gsm_msg_item_t item;
    while (msg_queue_pop(ctx->msg_queue, &item)) {
        had_data = true;
        gsm_stats_t stats;
        gsm_get_stats(ctx->inner, &stats);
        gsm_sync_state_t sync = gsm_get_sync_state(ctx->inner);
        gsmTrackerUpdate(&item.cell, &stats, sync);

        panelLogMessage("[GSM rx%d] MCC=%d MNC=%d LAC=%u CID=%u ARFCN=%d BSIC=%02X %s",
                        ctx->rx->dev_index, item.cell.si3.mcc, item.cell.si3.mnc,
                        item.cell.si3.lac, item.cell.si3.cell_id,
                        item.cell.arfcn, item.cell.bsic, item.msg_type);
    }

    // Drain Cell Broadcast messages
    gsm_cb_item_t cb_item;
    while (msg_queue_pop(ctx->cb_queue, &cb_item)) {
        had_data = true;
        gsmTrackerUpdateCB(&cb_item.cell, &cb_item.cb);

        panelLogMessage("[GSM-CB rx%d] MCC=%d MNC=%d LAC=%u CID=%u serial=%u id=%u \"%s\"",
                        ctx->rx->dev_index, cb_item.cell.si3.mcc, cb_item.cell.si3.mnc,
                        cb_item.cell.si3.lac, cb_item.cell.si3.cell_id,
                        cb_item.cb.serial_nr, cb_item.cb.msg_id, cb_item.cb.text);
    }
    return had_data;
}
static void gsmDecoderStop(sdr_receiver_t *rx)     { rxDecoderDestroy(rx); }

static const decoder_ops_t gsm_decoder_ops = {
    .name    = "gsm",
    .init    = gsmDecoderInit,
    .process = gsmDecoderProcess,
    .drain   = gsmDecoderDrain,
    .stop    = gsmDecoderStop,
};

// ---- LTE decoder_ops ----

static bool lteDecoderInit(sdr_receiver_t *rx)    { return rxDecoderCreate(rx); }
static void lteDecoderProcess(sdr_receiver_t *rx, const uint8_t *iq, uint32_t len) { rxDecoderProcess(rx, iq, len); }
static bool lteDecoderDrain(sdr_receiver_t *rx) {
    lte_ctx_t *ctx = (lte_ctx_t *)rx->decoder_state;
    if (!ctx) return false;
    if (!LteOutputEnabled) return false;
    lte_cell_info_t cell;
    bool had_data = false;
    while (msg_queue_pop(ctx->queue, &cell)) {
        had_data = true;
        lteTrackerUpdate(&cell);

        int32_t rxid = ctx->rx->dev_index;
        for (int32_t i = 0; i < cell.alert_count; i++) {
            const lte_alert_t *a = &cell.alerts[i];
            if (!a->active) continue;
            switch (a->type) {
            case LTE_ALERT_ETWS:
                if (a->text[0])
                    panelLogMessage("[LTE rx%d] \xf0\x9f\x9a\xa8 ETWS PCI=%u %s msgid=%u \"%s\"",
                        rxid, cell.pci, a->category, a->message_id, a->text);
                else
                    panelLogMessage("[LTE rx%d] \xf0\x9f\x9a\xa8 ETWS PCI=%u %s msgid=%u serial=%u",
                        rxid, cell.pci, a->category, a->message_id, a->serial_number);
                break;
            case LTE_ALERT_CMAS:
                panelLogMessage("[LTE rx%d] \xf0\x9f\x9a\xa8 CMAS PCI=%u %s msgid=%u \"%s\"",
                    rxid, cell.pci, a->category, a->message_id,
                    a->text[0] ? a->text : "(no text)");
                break;
            case LTE_ALERT_EAB:
                panelLogMessage("[LTE rx%d] \xe2\x9a\xa0 EAB PCI=%u %s %s",
                    rxid, cell.pci, a->category, a->text);
                break;
            default:
                break;
            }
        }
    }
    return had_data;
}
static void lteDecoderStop(sdr_receiver_t *rx)     { rxDecoderDestroy(rx); }

static const decoder_ops_t lte_decoder_ops = {
    .name    = "lte",
    .init    = lteDecoderInit,
    .process = lteDecoderProcess,
    .drain   = lteDecoderDrain,
    .stop    = lteDecoderStop,
};

// ---- IoT 868 MHz decoder_ops ----

static bool iot868_decoder_init(sdr_receiver_t *rx)    { return rxDecoderCreate(rx); }
static void iot868_decoder_process(sdr_receiver_t *rx, const uint8_t *iq, uint32_t len) { rxDecoderProcess(rx, iq, len); }
static bool iot868_decoder_drain(sdr_receiver_t *rx) {
    iot_decoder_state_t *state = (iot_decoder_state_t *)rx->decoder_state;
    if (!state) return false;
    if (!IotOutputEnabled) return false;
    bool had_data = false;
    iot_device_msg_t msg;
    while (iotDecoderDequeue(state, &msg)) {
        had_data = true;
        iotTrackerUpdate(&msg);
    }
    return had_data;
}
static void iot868_decoder_stop(sdr_receiver_t *rx)     { rxDecoderDestroy(rx); }

static const decoder_ops_t iot868_decoder_ops = {
    .name    = "iot868",
    .init    = iot868_decoder_init,
    .process = iot868_decoder_process,
    .drain   = iot868_decoder_drain,
    .stop    = iot868_decoder_stop,
};

// ---- FANET LoRa decoder_ops ----

static bool fanetDecoderInit(sdr_receiver_t *rx)    {
    // Register dispatcher queue for FANET aircraft updates
    if (!fanet_aircraft_queue) {
        fanet_aircraft_queue = dispatcher_register_aircraft_queue("fanet");
    }
    return rxDecoderCreate(rx);
}
static void fanetDecoderProcess(sdr_receiver_t *rx, const uint8_t *iq, uint32_t len) { rxDecoderProcess(rx, iq, len); }

static const char *fanet_ground_type_str(fanet_ground_type_t t)
{
    switch (t) {
        case FANET_GROUND_WALKING:       return "Walking";
        case FANET_GROUND_VEHICLE:       return "Vehicle";
        case FANET_GROUND_BIKE:          return "Bike";
        case FANET_GROUND_BOOT:          return "Boot";
        case FANET_GROUND_NEED_RIDE:     return "Need ride";
        case FANET_GROUND_LANDED_OK:     return "Landed OK";
        case FANET_GROUND_NEED_TECH:     return "Need tech help";
        case FANET_GROUND_NEED_MEDICAL:  return "Need medical";
        case FANET_GROUND_DISTRESS:      return "DISTRESS";
        case FANET_GROUND_DISTRESS_AUTO: return "DISTRESS AUTO";
        default: return "Other";
    }
}

static const char *fanet_aircraft_type_str(fanet_aircraft_type_t t)
{
    switch (t) {
        case FANET_AIRCRAFT_PARAGLIDER: return "Paraglider";
        case FANET_AIRCRAFT_HANGGLIDER: return "Hangglider";
        case FANET_AIRCRAFT_BALLOON:    return "Balloon";
        case FANET_AIRCRAFT_GLIDER:     return "Glider";
        case FANET_AIRCRAFT_POWERED:    return "Powered";
        case FANET_AIRCRAFT_HELI:       return "Helicopter";
        case FANET_AIRCRAFT_UAV:        return "UAV";
        default: return "Other";
    }
}

static bool fanetDecoderDrain(sdr_receiver_t *rx)
{
    if (!FanetOutputEnabled) { return false; }
    fanet_state_t *st = (fanet_state_t *)rx->decoder_state;
    if (!st) { return false; }

    bool had_data = false;
    fanet_message_t msg;
    while (fanet_dequeue(st, &msg)) {
        if (!msg.valid) continue;
        had_data = true;

        uint32_t addr = fanet_addr24(msg.src_manufacturer, msg.src_id);
        double signal = msg.signal_level;
        if (signal <= 0) signal = 0.001;
        if (signal > 1.0) signal = 1.0;

        // Try to get cached name for this address
        char cached_name[32] = "";
        fanet_get_cached_name(st, msg.src_manufacturer, msg.src_id,
                              cached_name, sizeof(cached_name));

        switch (msg.type) {
        case FANET_TYPE_TRACKING: {
            if (!msg.tracking.position_valid) break;

            // Map FANET aircraft type → ADS-B category
            uint32_t category;
            switch (msg.tracking.aircraft_type) {
                case FANET_AIRCRAFT_PARAGLIDER:
                case FANET_AIRCRAFT_HANGGLIDER:
                    category = 0xB1; break;
                case FANET_AIRCRAFT_GLIDER:
                    category = 0xB1; break;
                case FANET_AIRCRAFT_BALLOON:
                    category = 0xB2; break;
                case FANET_AIRCRAFT_HELI:
                    category = 0xA7; break;
                case FANET_AIRCRAFT_UAV:
                    category = 0xB6; break;
                case FANET_AIRCRAFT_POWERED:
                    category = 0xA1; break;
                default:
                    category = 0xC0; break;
            }

            // Map FANET type → FLARM type enum (for UI icons)
            static const uint8_t fanet_to_flarm[] = {
                0, 7, 6, 11, 1, 8, 3, 13
            };
            uint8_t ft = msg.tracking.aircraft_type;
            uint8_t flarm_type = (ft < sizeof(fanet_to_flarm)) ? fanet_to_flarm[ft] : 0;

            char callsign[9];
            if (cached_name[0]) {
                snprintf(callsign, sizeof(callsign), "%.8s", cached_name);
            } else {
                snprintf(callsign, sizeof(callsign), "FNT%05X", addr & 0xFFFFF);
            }

            // Push aircraft update via dispatcher queue
            if (fanet_aircraft_queue) {
                aircraft_update_t upd;
                upd = {};
                upd.addr = addr;
                upd.timestamp_ms = mstime();
                upd.signal_level = signal;
                upd.source = DECODE_SOURCE_FANET;

                snprintf(upd.callsign, sizeof(upd.callsign), "%s", callsign);
                upd.callsign_valid = 1;
                upd.category = category;
                upd.category_valid = 1;

                upd.lat = msg.tracking.latitude;
                upd.lon = msg.tracking.longitude;
                upd.position_valid = 1;

                upd.altitude_ft = (int32_t)(msg.tracking.altitude * 3.28084);
                upd.altitude_valid = 1;
                upd.altitude_is_baro = 0;

                if (msg.tracking.speed > 0.1f || fabsf(msg.tracking.climb) > 0.1f) {
                    upd.ground_speed_kt = (int32_t)(msg.tracking.speed * 0.539957f);
                    upd.heading_deg = (int32_t)msg.tracking.heading;
                    upd.vert_rate_fpm = (int32_t)(msg.tracking.climb * 196.85f);
                    upd.velocity_valid = 1;
                }

                upd.air_ground = DECODE_AG_AIRBORNE;
                upd.flarm_acft_type = flarm_type;

                dispatcher_push_aircraft(fanet_aircraft_queue, &upd);
            }

            panelLogMessage("[FANET rx%d] Track %02X:%04X %s %.5f,%.5f alt=%dm spd=%.0fkm/h hdg=%.0f",
                            rx->dev_index, msg.src_manufacturer, msg.src_id,
                            fanet_aircraft_type_str(msg.tracking.aircraft_type),
                            msg.tracking.latitude, msg.tracking.longitude,
                            msg.tracking.altitude, msg.tracking.speed, msg.tracking.heading);
            break;
        }

        case FANET_TYPE_GROUND: {
            if (!msg.ground.position_valid) break;

            // Ground targets go to the FANET ground cache (not aircraft list)
            fanet_ground_cache_update(addr, msg.ground.latitude, msg.ground.longitude,
                                       (uint8_t)msg.ground.ground_type, cached_name);

            panelLogMessage("[FANET rx%d] Ground %02X:%04X %s %.5f,%.5f",
                            rx->dev_index, msg.src_manufacturer, msg.src_id,
                            fanet_ground_type_str(msg.ground.ground_type),
                            msg.ground.latitude, msg.ground.longitude);
            break;
        }

        case FANET_TYPE_NAME:
            fanet_name_cache_update(addr, msg.name);
            panelLogMessage("[FANET rx%d] Name %02X:%04X \"%s\"",
                            rx->dev_index, msg.src_manufacturer, msg.src_id, msg.name);
            break;

        case FANET_TYPE_MESSAGE:
            fanet_msg_cache_update(addr, msg.message.subtype, msg.message.text);
            panelLogMessage("[FANET rx%d] Msg %02X:%04X [sub=%d] %s",
                            rx->dev_index, msg.src_manufacturer, msg.src_id,
                            msg.message.subtype, msg.message.text);
            break;

        case FANET_TYPE_SERVICE: {
            char buf[200];
            int32_t pos = 0;
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                            "[FANET rx%d] Weather %02X:%04X",
                            rx->dev_index, msg.src_manufacturer, msg.src_id);
            if (msg.weather.has_temp)
                pos += snprintf(buf + pos, sizeof(buf) - pos, " T=%.1fC", msg.weather.temperature);
            if (msg.weather.has_wind)
                pos += snprintf(buf + pos, sizeof(buf) - pos, " wind=%.0fkm/h@%.0f gust=%.0f",
                                msg.weather.wind_speed, msg.weather.wind_heading, msg.weather.wind_gust);
            if (msg.weather.has_humidity)
                pos += snprintf(buf + pos, sizeof(buf) - pos, " RH=%.0f%%", msg.weather.humidity);
            if (msg.weather.has_pressure)
                pos += snprintf(buf + pos, sizeof(buf) - pos, " P=%.1fhPa", msg.weather.pressure);
            if (msg.weather.has_soc)
                pos += snprintf(buf + pos, sizeof(buf) - pos, " SoC=%.0f%%", msg.weather.state_of_charge);
            fanet_wx_cache_update(addr, cached_name, &msg);
            panelLogMessage("%s", buf);
            break;
        }

        case FANET_TYPE_THERMAL:
            if (msg.thermal.valid) {
                fanet_thermal_cache_update(addr, &msg);
                panelLogMessage("[FANET rx%d] Thermal %02X:%04X %.5f,%.5f alt=%dm climb=%.1fm/s conf=%d/7",
                                rx->dev_index, msg.src_manufacturer, msg.src_id,
                                msg.thermal.latitude, msg.thermal.longitude,
                                msg.thermal.altitude, msg.thermal.climb, msg.thermal.confidence);
                // Push thermal data via dispatcher queue
                if (fanet_aircraft_queue) {
                    aircraft_update_t upd;
                    upd = {};
                    upd.addr = addr;
                    upd.timestamp_ms = mstime();
                    upd.source = DECODE_SOURCE_FANET;
                    upd.thermal.lat = msg.thermal.latitude;
                    upd.thermal.lon = msg.thermal.longitude;
                    upd.thermal.altitude_m = msg.thermal.altitude;
                    upd.thermal.climb_ms = msg.thermal.climb;
                    upd.thermal.wind_speed_kmh = msg.thermal.wind_speed;
                    upd.thermal.wind_heading_deg = (int32_t)msg.thermal.wind_heading;
                    upd.thermal.confidence = msg.thermal.confidence;
                    upd.thermal.valid = 1;
                    dispatcher_push_aircraft(fanet_aircraft_queue, &upd);
                }
            }
            break;

        case FANET_TYPE_LANDMARK:
            panelLogMessage("[FANET rx%d] Landmark %02X:%04X type=%u layer=%u TTL=%dmin %s",
                            rx->dev_index, msg.src_manufacturer, msg.src_id,
                            msg.landmark.subtype, msg.landmark.layer,
                            msg.landmark.ttl_minutes, msg.landmark.text);
            break;

        case FANET_TYPE_HWINFO:
        case FANET_TYPE_HWINFO2:
            panelLogMessage("[FANET rx%d] HWInfo %02X:%04X dev=0x%02X %s%s",
                            rx->dev_index, msg.src_manufacturer, msg.src_id,
                            msg.hwinfo.device_type,
                            msg.hwinfo.has_icao ? "ICAO=" : "",
                            msg.hwinfo.has_icao ? "" : "");
            // Push HW info via dispatcher queue
            if (fanet_aircraft_queue) {
                aircraft_update_t upd;
                upd = {};
                upd.addr = addr;
                upd.timestamp_ms = mstime();
                upd.source = DECODE_SOURCE_FANET;
                upd.hw_info.device_type = msg.hwinfo.device_type;
                upd.hw_info.uptime_min = msg.hwinfo.has_uptime ? msg.hwinfo.uptime_minutes : 0;
                upd.hw_info.rssi = msg.hwinfo.has_rssi ? msg.hwinfo.rssi : 0;
                upd.hw_info.valid = 1;
                dispatcher_push_aircraft(fanet_aircraft_queue, &upd);
            }
            break;

        case FANET_TYPE_ACK: {
            uint32_t dst = fanet_addr24(msg.dst_manufacturer, msg.dst_id);
            fanet_ack_cache_update(addr, dst);
            panelLogMessage("[FANET rx%d] ACK %02X:%04X -> %02X:%04X",
                            rx->dev_index, msg.src_manufacturer, msg.src_id,
                            msg.dst_manufacturer, msg.dst_id);
            break;
        }

        default:
            panelLogMessage("[FANET rx%d] Type%u %02X:%04X payload=%d bytes",
                            rx->dev_index, (uint32_t)msg.type, msg.src_manufacturer, msg.src_id,
                            msg.payload_len);
            break;
        }
    }
    return had_data;
}

static void fanetDecoderStop(sdr_receiver_t *rx)
{
    fanet_state_t *st = (fanet_state_t *)rx->decoder_state;
    if (st) {
        fanet_stats_t stats;
        fanet_get_stats(st, &stats);
        fprintf(stderr, "rx[%d]: FANET stats: samples=%" PRIu64 " preambles=%" PRIu64 " sync=%" PRIu64 " decoded=%" PRIu64 " crc_err=%" PRIu64 " hdr_err=%" PRIu64 "\n",
                rx->id,
                (uint64_t)stats.samples_processed,
                (uint64_t)stats.preambles_detected,
                (uint64_t)stats.sync_word_ok,
                (uint64_t)stats.packets_decoded,
                (uint64_t)stats.crc_errors,
                (uint64_t)stats.header_errors);
    }
    rxDecoderDestroy(rx);
}

static const decoder_ops_t fanet_decoder_ops = {
    .name    = "fanet",
    .init    = fanetDecoderInit,
    .process = fanetDecoderProcess,
    .drain   = fanetDecoderDrain,
    .stop    = fanetDecoderStop,
};

// ---- Sarsat decoder_ops ----

static bool sarsatDecoderInit(sdr_receiver_t *rx)    { return rxDecoderCreate(rx); }
static void sarsatDecoderProcess(sdr_receiver_t *rx, const uint8_t *iq, uint32_t len) { rxDecoderProcess(rx, iq, len); }
static bool sarsatDecoderDrain(sdr_receiver_t *rx) {
    sarsat_ctx_t *ctx = (sarsat_ctx_t *)rx->decoder_state;
    if (!ctx) { return false; }
    if (!SarsatOutputEnabled) { return false; }
    sarsat_msg_t msg;
    bool had_data = false;
    while (msg_queue_pop(ctx->queue, &msg)) {
        had_data = true;
        // Build identification string based on protocol
        std::string ident;
        if (msg.mmsi[0])
            ident = gg::format(" MMSI=%s", msg.mmsi);
        else if (msg.icao_address > 0)
            ident = gg::format(" ICAO=%06X", msg.icao_address);
        else if (msg.call_sign[0])
            ident = gg::format(" Call=%s", msg.call_sign);
        if (msg.cert_number > 0) {
            ident += gg::format(" Cert=%u S/N=%u", msg.cert_number, msg.serial_number);
        } else if (msg.serial_number > 0 && !msg.mmsi[0]) {
            ident += gg::format(" S/N=%u", msg.serial_number);
        }

        // Always print to stderr for debug visibility
        if (msg.position_valid) {
            fprintf(stderr, "[SARSAT rx%d] %s %s %s [%s] HexID=%s%s pos=%.4f,%.4f BCH1=%s BCH2=%s%s\n",
                            ctx->rx->dev_index,
                            msg.is_test ? "TEST" : "DISTRESS",
                            sarsat_beacon_type_name(msg.beacon_type),
                            sarsat_protocol_name(msg.protocol),
                            msg.country_name,
                            msg.hex_id,
                            ident.c_str(),
                            msg.latitude, msg.longitude,
                            msg.bch1_valid ? "OK" : "FAIL",
                            msg.bch2_valid ? "OK" : "FAIL",
                            msg.homing_121_5 ? " 121.5MHz" : "");
        } else {
            fprintf(stderr, "[SARSAT rx%d] %s %s %s [%s] HexID=%s%s BCH1=%s BCH2=%s%s\n",
                            ctx->rx->dev_index,
                            msg.is_test ? "TEST" : "DISTRESS",
                            sarsat_beacon_type_name(msg.beacon_type),
                            sarsat_protocol_name(msg.protocol),
                            msg.country_name,
                            msg.hex_id,
                            ident.c_str(),
                            msg.bch1_valid ? "OK" : "FAIL",
                            msg.bch2_valid ? "OK" : "FAIL",
                            msg.homing_121_5 ? " 121.5MHz" : "");
        }

        if (msg.position_valid) {
            panelLogMessage("[SARSAT rx%d] %s %s %s [%s] HexID=%s%s pos=%.4f,%.4f BCH1=%s BCH2=%s%s",
                            ctx->rx->dev_index,
                            msg.is_test ? "TEST" : "DISTRESS",
                            sarsat_beacon_type_name(msg.beacon_type),
                            sarsat_protocol_name(msg.protocol),
                            msg.country_name,
                            msg.hex_id,
                            ident.c_str(),
                            msg.latitude, msg.longitude,
                            msg.bch1_valid ? "OK" : "FAIL",
                            msg.bch2_valid ? "OK" : "FAIL",
                            msg.homing_121_5 ? " 121.5MHz" : "");
        } else {
            panelLogMessage("[SARSAT rx%d] %s %s %s [%s] HexID=%s%s BCH1=%s BCH2=%s%s",
                            ctx->rx->dev_index,
                            msg.is_test ? "TEST" : "DISTRESS",
                            sarsat_beacon_type_name(msg.beacon_type),
                            sarsat_protocol_name(msg.protocol),
                            msg.country_name,
                            msg.hex_id,
                            ident.c_str(),
                            msg.bch1_valid ? "OK" : "FAIL",
                            msg.bch2_valid ? "OK" : "FAIL",
                            msg.homing_121_5 ? " 121.5MHz" : "");
        }
    }
    return had_data;
}
static void sarsatDecoderStop(sdr_receiver_t *rx)     { rxDecoderDestroy(rx); }

static const decoder_ops_t sarsat_decoder_ops = {
    .name    = "sarsat",
    .init    = sarsatDecoderInit,
    .process = sarsatDecoderProcess,
    .drain   = sarsatDecoderDrain,
    .stop    = sarsatDecoderStop,
};

// ---- ADS-B decoder_ops ----
// init:    creates IQ→magnitude converter + rx FIFO
// process: converts IQ block to magnitude, enqueues to FIFO (from callback thread)
// drain:   dequeues mag_buf from FIFO, calls demodulate2400 (main thread)
// stop:    halts FIFO, destroys converter

static bool adsbDecoderInit(sdr_receiver_t *rx)
{
    rx->rtl.converter = init_converter(INPUT_UC8,
                                       rx->config.sample_rate,
                                       Modes.dc_filter,
                                       &rx->rtl.converter_state);
    if (!rx->rtl.converter) {
        gg::eprint("rx[%d]: can't initialize sample converter\n", rx->id);
        return false;
    }

#if defined(__arm__) || defined(__aarch64__)
    rx->rtl.bounce_buffer = static_cast<decltype(rx->rtl.bounce_buffer)>(malloc(MODES_RTL_BUF_SIZE));
    if (!rx->rtl.bounce_buffer) {
        gg::eprint("rx[%d]: can't allocate bounce buffer\n", rx->id);
        return false;
    }
#endif

    uint32_t overlap = Modes.trailing_samples;
    if (!rxFifoCreate(&rx->fifo, MODES_MAG_BUFFERS,
                      MODES_MAG_BUF_SAMPLES + overlap, overlap)) {
        gg::eprint("rx[%d]: can't create FIFO\n", rx->id);
        return false;
    }

    gg::eprint("rx[%d]: ADS-B decoder created (IQ→mag converter + FIFO)\n", rx->id);
    return true;
}

static void adsbDecoderProcess(sdr_receiver_t *rx, const uint8_t *buf, uint32_t len)
{
    uint32_t samples_read = len / 2;

    struct mag_buf *outbuf = rxFifoAcquire(&rx->fifo, 0);
    if (!outbuf) {
        rx->dropped += samples_read;
        return;
    }

    outbuf->flags = 0;
    if (rx->dropped) {
        outbuf->flags |= MAGBUF_DISCONTINUOUS;
        outbuf->dropped = rx->dropped;
    }
    rx->dropped = 0;

    outbuf->sampleTimestamp = rx->sample_counter * 12e6 / rx->config.sample_rate;

    uint64_t block_duration = 1e3 * samples_read / rx->config.sample_rate;
    outbuf->sysTimestamp = mstime() - block_duration;

    uint32_t to_convert = samples_read;
    if (to_convert + outbuf->overlap > outbuf->totalLength) {
        to_convert = outbuf->totalLength - outbuf->overlap;
        rx->dropped = samples_read - to_convert;
    }

#if defined(__arm__) || defined(__aarch64__)
    if (rx->rtl.bounce_buffer) {
        memcpy(rx->rtl.bounce_buffer, buf, to_convert * 2);
        buf = rx->rtl.bounce_buffer;
    }
#endif

    rx->rtl.converter((void *)buf, &outbuf->data[outbuf->overlap], to_convert,
                      rx->rtl.converter_state,
                      &outbuf->mean_level, &outbuf->mean_power);
    outbuf->validLength = outbuf->overlap + to_convert;

    rxFifoEnqueue(&rx->fifo, outbuf);
}

static bool adsbDecoderDrain(sdr_receiver_t *rx)
{
    struct mag_buf *buf = rxFifoDequeue(&rx->fifo, 0);  // non-blocking
    if (!buf) return false;

    demodulate2400(buf);
    if (Modes.mode_ac) demodulate2400AC(buf);

    Modes.stats_current.samples_processed += buf->validLength - buf->overlap;
    Modes.stats_current.samples_dropped += buf->dropped;

    rxFifoRelease(&rx->fifo, buf);
    return true;
}

static void adsbDecoderStop(sdr_receiver_t *rx)
{
    rxFifoHalt(&rx->fifo);

    if (rx->rtl.converter) {
        cleanup_converter(rx->rtl.converter_state);
        rx->rtl.converter = NULL;
        rx->rtl.converter_state = NULL;
    }

#if defined(__arm__) || defined(__aarch64__)
    free(rx->rtl.bounce_buffer);
    rx->rtl.bounce_buffer = NULL;
#endif

    rxFifoDestroy(&rx->fifo);
    gg::eprint("rx[%d]: ADS-B decoder stopped\n", rx->id);
}

static const decoder_ops_t adsb_decoder_ops = {
    .name    = "adsb",
    .init    = adsbDecoderInit,
    .process = adsbDecoderProcess,
    .drain   = adsbDecoderDrain,
    .stop    = adsbDecoderStop,
};

// ---- FLARM decoder_ops (implemented in flarm_reader.c) ----

static const decoder_ops_t flarm_decoder_ops = {
    .name    = "flarm",
    .init    = flarmDecoderInit,
    .process = flarmDecoderProcess,
    .drain   = flarmDecoderDrain,
    .stop    = flarmDecoderStop,
};

// ---- decoderOpsForRole() ----

const decoder_ops_t *decoderOpsForRole(sdr_role_t role)
{
    switch (role) {
    case SDR_ROLE_ADSB:       return &adsb_decoder_ops;
    case SDR_ROLE_FLARM:      return &flarm_decoder_ops;
    case SDR_ROLE_ACARS:      return &acars_decoder_ops;
    case SDR_ROLE_VDL2:       return &vdl2_decoder_ops;
    case SDR_ROLE_RADIOSONDE: return &sonde_decoder_ops;
    case SDR_ROLE_POCSAG:     return &pocsag_decoder_ops;
    case SDR_ROLE_GSM:        return &gsm_decoder_ops;
    case SDR_ROLE_LTE:        return &lte_decoder_ops;
    case SDR_ROLE_IOT868:     return &iot868_decoder_ops;
    case SDR_ROLE_FANET:      return &fanet_decoder_ops;
    case SDR_ROLE_SARSAT:     return &sarsat_decoder_ops;
    default:                  return NULL;
    }
}

// ======================== Manager ========================

// Check for conflicting kernel DVB driver that corrupts RTL2832U tuner state
static void checkDvbKernelDriver(void)
{
    // Check if the dvb_usb_rtl28xxu module is loaded by probing /sys/module
    if (access("/sys/module/dvb_usb_rtl28xxu", F_OK) == 0) {
        DvbDriverWarning = 1;
        gg::eprint("\n");
        gg::eprint("╔══════════════════════════════════════════════════════════════════╗\n");
        gg::eprint("║  WARNING: Kernel DVB driver 'dvb_usb_rtl28xxu' is loaded!       ║\n");
        gg::eprint("║                                                                  ║\n");
        gg::eprint("║  This driver conflicts with SDR reception and will corrupt       ║\n");
        gg::eprint("║  the R820T tuner state, causing NO signal reception.             ║\n");
        gg::eprint("║                                                                  ║\n");
        gg::eprint("║  FIX: Create /etc/modprobe.d/rtlsdr-blacklist.conf with:         ║\n");
        gg::eprint("║    blacklist dvb_usb_rtl28xxu                                    ║\n");
        gg::eprint("║    blacklist rtl2832_sdr                                         ║\n");
        gg::eprint("║    blacklist rtl2832                                             ║\n");
        gg::eprint("║    blacklist dvb_usb_v2                                          ║\n");
        gg::eprint("║  Then reboot, or run: sudo rmmod dvb_usb_rtl28xxu               ║\n");
        gg::eprint("╚══════════════════════════════════════════════════════════════════╝\n");
        gg::eprint("\n");
    }
}

void sdrManagerInit(void)
{
    SdrManager = {};
    pthread_mutex_init(&SdrManager.lock, NULL);

    // Check for conflicting kernel drivers before touching SDR hardware
    checkDvbKernelDriver();

    // Initialize backend subsystem (detects available libraries)
    sdrBackendInit();

    for (int32_t i = 0; i < MAX_SDR_RECEIVERS; i++) {
        SdrManager.receivers[i].id = i;
        SdrManager.receivers[i].state = RX_STATE_IDLE;
        pthread_mutex_init(&SdrManager.receivers[i].cpu_mutex, NULL);
    }
}

int32_t sdrManagerAddReceiver(const rx_config_t *config)
{
    pthread_mutex_lock(&SdrManager.lock);

    // Reject duplicate serials (skip check for virtual file devices)
    if (config->ifile_path[0] == '\0') {
        for (int32_t i = 0; i < SdrManager.count; i++) {
            if (!strcmp(SdrManager.receivers[i].config.serial, config->serial)) {
                gg::eprint("sdr_manager: serial %s already managed (index %d)\n", config->serial, i);
                pthread_mutex_unlock(&SdrManager.lock);
                return -1;
            }
        }
    }

    if (SdrManager.count >= MAX_SDR_RECEIVERS) {
        gg::eprint("sdr_manager: max receivers (%d) reached\n", MAX_SDR_RECEIVERS);
        pthread_mutex_unlock(&SdrManager.lock);
        return -1;
    }

    int32_t idx = SdrManager.count;
    sdr_receiver_t *rx = &SdrManager.receivers[idx];

    memset(&rx->config, 0, sizeof(rx->config));
    rx->config = *config;
    memset(&rx->rtl, 0, sizeof(rx->rtl));
    rx->rtl.tuner_type = -1;
    rx->state = RX_STATE_IDLE;
    rx->thread_started = false;
    rx->dropped = 0;
    rx->sample_counter = 0;

    SdrManager.count++;

    fprintf(stderr, "sdr_manager: added receiver[%d] serial=%s role=%s freq=%d gain=%.1f\n",
            idx, config->serial, sdrRoleName(config->role),
            config->freq, config->gain);

    pthread_mutex_unlock(&SdrManager.lock);
    return idx;
}

bool sdrManagerRemoveReceiver(int32_t index)
{
    if (index < 0 || index >= SdrManager.count) return false;

    pthread_mutex_lock(&SdrManager.lock);

    sdr_receiver_t *rx = &SdrManager.receivers[index];
    if (rx->state == RX_STATE_RUNNING)
        rxStop(rx);
    if (rx->state != RX_STATE_IDLE)
        rxClose(rx);

    // Shift remaining receivers down
    for (int32_t i = index; i < SdrManager.count - 1; i++) {
        SdrManager.receivers[i] = SdrManager.receivers[i + 1];
        SdrManager.receivers[i].id = i;
    }
    SdrManager.count--;

    // Clear the last slot
    memset(&SdrManager.receivers[SdrManager.count], 0, sizeof(sdr_receiver_t));
    SdrManager.receivers[SdrManager.count].id = SdrManager.count;

    pthread_mutex_unlock(&SdrManager.lock);
    return true;
}

// ======================== USB Device Reset ========================
// Resets all RTL2832U USB devices via sysfs before opening.
// Clears stale hardware state (e.g. direct sampling mode left by buggy librtlsdr)
// that persists across normal open/close cycles and even USB soft resets.
static void rxResetRtlUsbDevices(void)
{
    const char *sysfs = "/sys/bus/usb/devices";
    DIR *d = opendir(sysfs);
    if (!d) return;

    int32_t reset_count = 0;
    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        if (ent->d_name[0] == '.') continue;

        // Only look at top-level device entries (e.g. "1-1.1.4"), skip interfaces (contain ':')
        if (strchr(ent->d_name, ':')) continue;

        char path[512];

        // Check idVendor = 0bda (Realtek)
        snprintf(path, sizeof(path), "%s/%s/idVendor", sysfs, ent->d_name);
        FILE *f = fopen(path, "r");
        if (!f) continue;
        char vid[16] = {0};
        if (fgets(vid, sizeof(vid), f)) {
            vid[strcspn(vid, "\n")] = 0;
        }
        fclose(f);
        if (strcmp(vid, "0bda") != 0) continue;

        // Check idProduct = 2838 (RTL2832U)
        snprintf(path, sizeof(path), "%s/%s/idProduct", sysfs, ent->d_name);
        f = fopen(path, "r");
        if (!f) continue;
        char pid[16] = {0};
        if (fgets(pid, sizeof(pid), f)) {
            pid[strcspn(pid, "\n")] = 0;
        }
        fclose(f);
        if (strcmp(pid, "2838") != 0) continue;

        // Read serial for logging
        char serial[128] = {0};
        snprintf(path, sizeof(path), "%s/%s/serial", sysfs, ent->d_name);
        f = fopen(path, "r");
        if (f) {
            if (fgets(serial, sizeof(serial), f)) {
                serial[strcspn(serial, "\n")] = 0;
            }
            fclose(f);
        }

        // Deauthorize the device
        snprintf(path, sizeof(path), "%s/%s/authorized", sysfs, ent->d_name);
        f = fopen(path, "w");
        if (!f) {
            fprintf(stderr, "sdr_usb_reset: can't write %s: %s (need root?)\n", path, strerror(errno));
            continue;
        }
        fputs("0\n", f);
        fclose(f);

        // Wait for USB bus to process the disconnection
        usleep(200000); // 200ms

        // Reauthorize the device
        f = fopen(path, "w");
        if (f) {
            fputs("1\n", f);
            fclose(f);
            reset_count++;
            fprintf(stderr, "sdr_usb_reset: reset %s (SN %s)\n", ent->d_name, serial);
        }
    }
    closedir(d);

    if (reset_count > 0) {
        // Wait for USB re-enumeration to complete
        fprintf(stderr, "sdr_usb_reset: reset %d RTL2832U device(s), waiting for re-enumeration...\n", reset_count);
        usleep(2000000); // 2 seconds
        fprintf(stderr, "sdr_usb_reset: ready\n");
    }
}

int32_t sdrManagerOpenAll(void)
{
    // Reset all RTL2832U USB devices to clear stale hardware state
    rxResetRtlUsbDevices();

    int32_t opened = 0;
    for (int32_t i = 0; i < SdrManager.count; i++) {
        if (SdrManager.receivers[i].state == RX_STATE_IDLE) {
            if (rxOpen(&SdrManager.receivers[i]))
                opened++;
        }
    }
    return opened;
}

int32_t sdrManagerStartAll(void)
{
    int32_t started = 0;
    for (int32_t i = 0; i < SdrManager.count; i++) {
        if (SdrManager.receivers[i].state == RX_STATE_OPEN) {
            if (rxStart(&SdrManager.receivers[i]))
                started++;
        }
    }
    return started;
}

bool sdrManagerDrainAll(void)
{
    bool had_data = false;
    pthread_mutex_lock(&SdrManager.lock);
    for (int32_t i = 0; i < SdrManager.count; i++) {
        sdr_receiver_t *rx = &SdrManager.receivers[i];
        if (rx->state == RX_STATE_RUNNING && rx->decoder_ops && rx->decoder_ops->drain) {
            if (rx->decoder_ops->drain(rx))
                had_data = true;
        }
    }
    pthread_mutex_unlock(&SdrManager.lock);
    return had_data;
}

void sdrManagerStopAll(void)
{
    for (int32_t i = 0; i < SdrManager.count; i++) {
        if (SdrManager.receivers[i].state == RX_STATE_RUNNING)
            rxStop(&SdrManager.receivers[i]);
    }
}

void sdrManagerCloseAll(void)
{
    for (int32_t i = 0; i < SdrManager.count; i++) {
        if (SdrManager.receivers[i].state != RX_STATE_IDLE)
            rxClose(&SdrManager.receivers[i]);
    }
}

void sdrManagerShutdown(void)
{
    sdrManagerStopAll();
    sdrManagerCloseAll();
    pthread_mutex_destroy(&SdrManager.lock);
}

int32_t sdrManagerFindBySerial(const char *serial)
{
    for (int32_t i = 0; i < SdrManager.count; i++) {
        if (!strcmp(SdrManager.receivers[i].config.serial, serial) ||
            !strcmp(SdrManager.receivers[i].serial_actual, serial))
            return i;
    }
    return -1;
}

void sdrManagerUpdateConfig(int32_t index, const rx_config_t *config)
{
    if (index < 0 || index >= SdrManager.count) return;
    pthread_mutex_lock(&SdrManager.lock);
    sdr_receiver_t *rx = &SdrManager.receivers[index];
    rx->config.role        = config->role;
    rx->config.freq        = config->freq;
    rx->config.sample_rate = config->sample_rate;
    rx->config.gain        = config->gain;
    rx->config.ppm_error   = config->ppm_error;
    pthread_mutex_unlock(&SdrManager.lock);
}

sdr_receiver_t *sdrManagerGetReceiver(int32_t index)
{
    if (index < 0 || index >= SdrManager.count) return NULL;
    return &SdrManager.receivers[index];
}

// ======================== Persistence ========================

#define RECEIVERS_JSON_PATH "/etc/dump1090-gg/receivers.json"

bool sdrManagerSave(void)
{
    FILE *f = fopen(RECEIVERS_JSON_PATH, "w");
    if (!f) {
        gg::eprint("sdr_manager: can't write %s: %s\n", RECEIVERS_JSON_PATH, strerror(errno));
        return false;
    }

    gg::fprint(f, "{\"receivers\":[\n");
    int32_t written = 0;
    for (int32_t i = 0; i < SdrManager.count; i++) {
        sdr_receiver_t *rx = &SdrManager.receivers[i];
        // Don't persist virtual file devices
        if (rx->config.ifile_path[0] != '\0') continue;
        if (written > 0) gg::fprint(f, ",\n");
        fprintf(f, "  {\"serial\":\"%s\",\"role\":\"%s\",\"gain\":%.1f,\"ppm\":%d,\"backend\":\"%s\"}",
                rx->config.serial, sdrRoleName(rx->config.role),
                rx->config.gain, rx->config.ppm_error,
                sdrBackendName(rx->config.backend));
        written++;
    }
    gg::fprint(f, "\n]}\n");
    fclose(f);
    gg::eprint("sdr_manager: saved %d receivers to %s\n", written, RECEIVERS_JSON_PATH);
    return true;
}

int32_t sdrManagerLoad(void)
{
    FILE *f = fopen(RECEIVERS_JSON_PATH, "r");
    if (!f) return 0;  // no saved config — normal on first run

    fseek(f, 0, SEEK_END);
    int64_t sz = ftell(f);
    if (sz <= 0 || sz > 65536) { fclose(f); return 0; }
    rewind(f);

    char *data = static_cast<char*>(malloc((size_t)sz + 1));
    if (!data) { fclose(f); return 0; }
    size_t rd = fread(data, 1, (size_t)sz, f);
    fclose(f);
    data[rd] = '\0';

    int32_t loaded = 0;
    // Simple JSON parser: find each {"serial":"...","role":"...","gain":...,"ppm":...}
    const char *p = data;
    while ((p = strstr(p, "\"serial\"")) != NULL) {
        char serial[64] = {0};
        char role_str[16] = {0};
        float gain = 0;
        int32_t ppm = 0;

        // Parse serial
        const char *v = strchr(p + 8, '"');
        if (!v) break;
        v++;
        const char *e = strchr(v, '"');
        if (!e || (e - v) >= 64) break;
        memcpy(serial, v, e - v);

        // Parse role
        const char *rp = strstr(e, "\"role\"");
        if (!rp) break;
        v = strchr(rp + 6, '"');
        if (!v) break;
        v++;
        e = strchr(v, '"');
        if (!e || (e - v) >= 16) break;
        memcpy(role_str, v, e - v);

        // Parse gain
        const char *gp = strstr(e, "\"gain\"");
        if (gp) {
            gp += 6;
            while (*gp == ' ' || *gp == ':') gp++;
            gain = (float)atof(gp);
        }

        // Parse ppm
        const char *pp = strstr(e, "\"ppm\"");
        if (pp) {
            pp += 5;
            while (*pp == ' ' || *pp == ':') pp++;
            ppm = atoi(pp);
        }

        // Parse backend
        sdr_backend_type_t backend = sdrBackendParse("");  // default: sdrgg if available
        const char *bp = strstr(e, "\"backend\"");
        if (bp) {
            const char *bv = strchr(bp + 9, '"');
            if (bv) { bv++;
                const char *be = strchr(bv, '"');
                if (be && (be - bv) < 16) {
                    char bstr[16] = {0};
                    memcpy(bstr, bv, be - bv);
                    backend = sdrBackendParse(bstr);
                }
            }
        }

        // Map role string to enum
        sdr_role_t role = SDR_ROLE_NONE;
        if (!strcmp(role_str, "adsb")) role = SDR_ROLE_ADSB;
        else if (!strcmp(role_str, "flarm")) role = SDR_ROLE_FLARM;
        else if (!strcmp(role_str, "acars")) role = SDR_ROLE_ACARS;
        else if (!strcmp(role_str, "vdl2")) role = SDR_ROLE_VDL2;
        else if (!strcmp(role_str, "radiosonde")) role = SDR_ROLE_RADIOSONDE;
        else if (!strcmp(role_str, "pocsag")) role = SDR_ROLE_POCSAG;
        else if (!strcmp(role_str, "gsm")) role = SDR_ROLE_GSM;
        else if (!strcmp(role_str, "lte")) role = SDR_ROLE_LTE;
        else if (!strcmp(role_str, "iot868")) role = SDR_ROLE_IOT868;
        else if (!strcmp(role_str, "fanet")) role = SDR_ROLE_FANET;
        else if (!strcmp(role_str, "sarsat")) role = SDR_ROLE_SARSAT;

        if (role != SDR_ROLE_NONE && serial[0]) {
            // Skip FILE (virtual) entries — they are only valid via --receiver FILE:...
            if (!strcmp(serial, "FILE")) { p = e + 1; continue; }
            // Skip duplicates within JSON itself
            if (sdrManagerFindBySerial(serial) < 0) {
                rx_config_t cfg = {0};
                snprintf(cfg.serial, sizeof(cfg.serial), "%.63s", serial);
                cfg.role = role;
                cfg.gain = gain;
                cfg.ppm_error = ppm;
                cfg.backend = backend;
                // Set freq/sample_rate for role
                switch (role) {
                    case SDR_ROLE_ADSB:       cfg.freq = 1090000000; cfg.sample_rate = 2400000; break;
                    case SDR_ROLE_FLARM:
                        if (FlarmConfig.p3i_enabled) {
                            cfg.freq = FLARM_CENTER_FREQ_P3I;
                            cfg.sample_rate = FLARM_SAMPLE_RATE_P3I;
                        } else {
                            cfg.freq = FLARM_CENTER_FREQ;
                            cfg.sample_rate = FLARM_SAMPLE_RATE;
                        }
                        break;
                    case SDR_ROLE_ACARS:      cfg.freq = 131550000;  cfg.sample_rate = 2400000; break;
                    case SDR_ROLE_VDL2:       cfg.freq = 136975000;  cfg.sample_rate = 2400000; break;
                    case SDR_ROLE_RADIOSONDE:
                        if (DecoderConfigs.radiosonde.freq_mode == SONDE_FREQ_MODE_SCAN) {
                            cfg.freq = 401000000;  // scan starts at 401 MHz
                        } else {
                            cfg.freq = (uint32_t)DecoderConfigs.radiosonde.center_freq;
                            if (cfg.freq < 400000000 || cfg.freq > 406000000)
                                cfg.freq = 403000000;
                        }
                        cfg.sample_rate = 2400000;
                        break;
                    case SDR_ROLE_POCSAG:     cfg.freq = 466075000;  cfg.sample_rate = 1200000; break;
                    case SDR_ROLE_GSM:        cfg.freq = 947000000;  cfg.sample_rate = 1000000; break;
                    case SDR_ROLE_LTE:        cfg.freq = LTE_DEFAULT_FREQ; cfg.sample_rate = LTE_SAMPLE_RATE; break;
                    case SDR_ROLE_IOT868:     cfg.freq = IOT_CENTER_FREQ;  cfg.sample_rate = IOT_SAMPLE_RATE; break;
                    case SDR_ROLE_FANET:      cfg.freq = FANET_CENTER_FREQ; cfg.sample_rate = FANET_SAMPLE_RATE; break;
                    case SDR_ROLE_SARSAT:     cfg.freq = SARSAT_CENTER_FREQ; cfg.sample_rate = SARSAT_SAMPLE_RATE; break;
                    default: break;
                }
                if (sdrManagerAddReceiver(&cfg) >= 0) {
                    loaded++;
                    // Set FlarmConfig.enabled so OGN/feeder code knows FLARM is active
                    if (role == SDR_ROLE_FLARM) {
                        FlarmConfig.enabled = 1;
                    }
                }
            }
        }

        p = e + 1;
    }

    free(data);
    if (loaded > 0)
        gg::eprint("sdr_manager: loaded %d receivers from %s\n", loaded, RECEIVERS_JSON_PATH);
    return loaded;
}

// ======================== Decoder stats JSON ========================

char *rxGetDecoderStatsJSON(void)
{
    // Build JSON with stats from all active decoder receivers.
    // Returns a malloc'd string — caller must free().
    char *buf = (char *)malloc(8192);
    if (!buf) return NULL;
    int32_t pos = 0, cap = 8192;

    #define APPEND(...) do { \
        int32_t n = snprintf(buf + pos, cap - pos, __VA_ARGS__); \
        if (n > 0) pos += n; \
    } while(0)

    APPEND("{");
    int32_t first_top = 1;

    for (int32_t i = 0; i < SdrManager.count; i++) {
        sdr_receiver_t *rx = &SdrManager.receivers[i];
        if (rx->state < RX_STATE_RUNNING) continue;
        // ADS-B uses Modes.stats_* instead of decoder_state; others need decoder_state
        if (rx->config.role != SDR_ROLE_ADSB && !rx->decoder_state) continue;

        switch (rx->config.role) {
        case SDR_ROLE_ADSB: {
            // Combine alltime + current (current hasn't been flushed yet)
            struct stats s;
            add_stats(&Modes.stats_alltime, &Modes.stats_current, &s);
            uint32_t accepted = 0;
            for (int32_t b = 0; b <= MODES_MAX_BITERRORS; b++)
                accepted += s.demod_accepted[b];
            double noise_db = -999;
            if (s.noise_power_count > 0) {
                double p = s.noise_power_sum / s.noise_power_count;
                if (p > 0) noise_db = 10.0 * log10(p);
            }
            double signal_db = -999;
            if (s.signal_power_count > 0) {
                double p = s.signal_power_sum / s.signal_power_count;
                if (p > 0) signal_db = 10.0 * log10(p);
            }
            if (!first_top) APPEND(","); first_top = 0;
            APPEND("\"adsb\":{\"rx\":%d"
                ",\"samples\":%" PRIu64
                ",\"samples_dropped\":%" PRIu64
                ",\"messages_total\":%" PRIu32
                ",\"preambles\":%" PRIu32
                ",\"accepted\":%" PRIu32
                ",\"rejected_bad\":%" PRIu32
                ",\"rejected_unknown\":%" PRIu32
                ",\"crc_rescued\":%" PRIu32
                ",\"modeac\":%" PRIu32
                ",\"strong_signals\":%" PRIu32
                ",\"noise_dbfs\":%.1f"
                ",\"signal_dbfs\":%.1f"
                ",\"peak_signal_dbfs\":%.1f"
                ",\"tracks\":%" PRIu32
                ",\"single_msg\":%" PRIu32
                ",\"gain_db\":%d}",
                rx->id,
                (uint64_t)s.samples_processed,
                (uint64_t)s.samples_dropped,
                s.messages_total,
                s.demod_preambles,
                accepted,
                s.demod_rejected_bad,
                s.demod_rejected_unknown_icao,
                s.demod_crc_rescued,
                s.demod_modeac,
                s.strong_signal_count,
                noise_db,
                signal_db,
                s.peak_signal_power > 0 ? 10.0 * log10(s.peak_signal_power) : -999.0,
                s.unique_aircraft,
                s.single_message_aircraft,
                s.sdr_gain);
            break;
        }
        case SDR_ROLE_FLARM: {
            flarm_demod_stats_t s;
            if (flarmDecoderGetStats(rx, &s)) {
                if (!first_top) APPEND(","); first_top = 0;
                APPEND("\"flarm\":{\"rx\":%d,\"samples\":%" PRIu64
                    ",\"detected\":%" PRIu64 ",\"crc_ok\":%" PRIu64
                    ",\"decoded\":%" PRIu64 ",\"failed\":%" PRIu64
                    ",\"type1\":%" PRIu64 ",\"type3\":%" PRIu64 ",\"type4\":%" PRIu64
                    ",\"ogntp_detected\":%" PRIu64 ",\"ogntp_ldpc_ok\":%" PRIu64
                    ",\"ogntp_decoded\":%" PRIu64 ",\"ogntp_failed\":%" PRIu64
                    ",\"p3i_detected\":%" PRIu64 ",\"p3i_decoded\":%" PRIu64 ",\"p3i_failed\":%" PRIu64
                    ",\"adsl_detected\":%" PRIu64 ",\"adsl_crc_ok\":%" PRIu64
                    ",\"adsl_decoded\":%" PRIu64 ",\"adsl_failed\":%" PRIu64 "}",
                    rx->id, s.samples_processed,
                    s.packets_detected, s.packets_crc_ok,
                    s.packets_decoded, s.packets_failed,
                    s.packets_type1, s.packets_type3, s.packets_type4,
                    s.ogntp_packets_detected, s.ogntp_packets_ldpc_ok,
                    s.ogntp_packets_decoded, s.ogntp_packets_failed,
                    s.p3i_packets_detected, s.p3i_packets_decoded, s.p3i_packets_failed,
                    s.adsl_packets_detected, s.adsl_packets_crc_ok,
                    s.adsl_packets_decoded, s.adsl_packets_failed);
            }
            break;
        }
        case SDR_ROLE_ACARS: {
            acars_ctx_t *c = (acars_ctx_t *)rx->decoder_state;
            if (c && c->inner) {
                acars_stats_t s;
                acars_get_stats(c->inner, &s);
                if (!first_top) APPEND(","); first_top = 0;
                APPEND("\"acars\":{\"rx\":%d,\"samples\":%" PRIu64
                    ",\"decoded\":%" PRIu64 ",\"crc_errors\":%" PRIu64 "}",
                    rx->id, s.samples_processed, s.messages_decoded, s.crc_errors);
            }
            break;
        }
        case SDR_ROLE_VDL2: {
            vdl2_ctx_t *c = (vdl2_ctx_t *)rx->decoder_state;
            if (c && c->inner) {
                vdl2_stats_t s;
                vdl2_get_stats(c->inner, &s);
                if (!first_top) APPEND(","); first_top = 0;
                APPEND("\"vdl2\":{\"rx\":%d,\"samples\":%" PRIu64
                    ",\"frames\":%" PRIu64 ",\"decoded\":%" PRIu64
                    ",\"fcs_errors\":%" PRIu64 "}",
                    rx->id, s.samples_processed, s.frames_detected,
                    s.messages_decoded, s.fcs_errors);
            }
            break;
        }
        case SDR_ROLE_RADIOSONDE: {
            sonde_ctx_t *c = (sonde_ctx_t *)rx->decoder_state;
            if (c && c->inner) {
                sonde_stats_t s;
                sonde_get_stats(c->inner, &s);
                if (!first_top) APPEND(","); first_top = 0;
                APPEND("\"sonde\":{\"rx\":%d,\"samples\":%" PRIu64
                    ",\"frames_detected\":%" PRIu64 ",\"frames_decoded\":%" PRIu64
                    ",\"rs_corrected\":%" PRIu64 ",\"rs_uncorrectable\":%" PRIu64
                    ",\"crc_errors\":%" PRIu64 "}",
                    rx->id, s.samples_processed, s.frames_detected,
                    s.frames_decoded, s.rs_corrected, s.rs_uncorrectable, s.crc_errors);
            }
            break;
        }
        case SDR_ROLE_POCSAG: {
            pocsag_ctx_t *c = (pocsag_ctx_t *)rx->decoder_state;
            if (c && c->inner) {
                pocsag_stats_t s;
                pocsag_get_stats(c->inner, &s);
                if (!first_top) APPEND(","); first_top = 0;
                APPEND("\"pocsag\":{\"rx\":%d,\"samples\":%" PRIu64
                    ",\"preambles\":%" PRIu64 ",\"syncs\":%" PRIu64
                    ",\"decoded\":%" PRIu64 ",\"bch_corrections\":%" PRIu64
                    ",\"bch_failures\":%" PRIu64 "}",
                    rx->id, s.samples_processed, s.preambles_detected,
                    s.syncs_detected, s.messages_decoded,
                    s.bch_corrections, s.bch_failures);
            }
            break;
        }
        case SDR_ROLE_GSM: {
            gsm_ctx_t *c = (gsm_ctx_t *)rx->decoder_state;
            if (c && c->inner) {
                gsm_stats_t s;
                gsm_get_stats(c->inner, &s);
                if (!first_top) APPEND(","); first_top = 0;
                APPEND("\"gsm\":{\"rx\":%d,\"samples\":%" PRIu64
                    ",\"fcch\":%" PRIu64 ",\"sch_ok\":%" PRIu64 ",\"sch_fail\":%" PRIu64
                    ",\"bcch\":%" PRIu64 ",\"bcch_fail\":%" PRIu64
                    ",\"ccch\":%" PRIu64 ",\"cb\":%" PRIu64
                    ",\"freq_offset\":%.1f}",
                    rx->id, s.samples_processed, s.fcch_detected,
                    s.sch_decoded, s.sch_failed, s.bcch_decoded, s.bcch_failed,
                    s.ccch_decoded, s.cb_decoded, s.freq_offset_hz);
            }
            break;
        }
        case SDR_ROLE_LTE: {
            lte_ctx_t *c = (lte_ctx_t *)rx->decoder_state;
            if (c && c->inner) {
                lte_stats_t s;
                lte_get_stats(c->inner, &s);
                if (!first_top) APPEND(","); first_top = 0;
                APPEND("\"lte\":{\"rx\":%d,\"samples\":%" PRIu64
                    ",\"pss\":%" PRIu32 ",\"sss\":%" PRIu32
                    ",\"mib\":%" PRIu32 ",\"sib1\":%" PRIu32
                    ",\"crc_errors\":%" PRIu32 ",\"freq_offset\":%.1f}",
                    rx->id, s.samples_processed, s.pss_detected, s.sss_decoded,
                    s.mib_decoded, s.sib1_decoded, s.crc_errors, s.freq_offset_hz);
            }
            break;
        }
        case SDR_ROLE_FANET: {
            fanet_state_t *st = (fanet_state_t *)rx->decoder_state;
            if (st) {
                fanet_stats_t s;
                fanet_get_stats(st, &s);
                if (!first_top) APPEND(","); first_top = 0;
                APPEND("\"fanet\":{\"rx\":%d,\"samples\":%" PRIu64
                    ",\"preambles\":%" PRIu64 ",\"sync_ok\":%" PRIu64
                    ",\"decoded\":%" PRIu64 ",\"crc_errors\":%" PRIu64
                    ",\"header_errors\":%" PRIu64 "}",
                    rx->id, s.samples_processed, s.preambles_detected,
                    s.sync_word_ok, s.packets_decoded,
                    s.crc_errors, s.header_errors);
            }
            break;
        }
        case SDR_ROLE_SARSAT: {
            sarsat_ctx_t *c = (sarsat_ctx_t *)rx->decoder_state;
            if (c && c->inner) {
                sarsat_stats_t s;
                sarsat_get_stats(c->inner, &s);
                if (!first_top) APPEND(","); first_top = 0;
                APPEND("\"sarsat\":{\"rx\":%d,\"samples\":%" PRIu64
                    ",\"bursts\":%" PRIu64 ",\"frames\":%" PRIu64
                    ",\"bch1_corrected\":%" PRIu64 ",\"bch1_failed\":%" PRIu64
                    ",\"bch2_corrected\":%" PRIu64 ",\"bch2_failed\":%" PRIu64 "}",
                    rx->id, s.samples_processed, s.bursts_detected, s.frames_decoded,
                    s.bch1_corrected, s.bch1_failed, s.bch2_corrected, s.bch2_failed);
            }
            break;
        }
        default:
            break;
        }
    }

    APPEND("}");
    #undef APPEND
    return buf;
}

void rxGetStatsSnapshot(rx_stats_snapshot_t *out)
{
    memset(out, 0, sizeof(*out));

    for (int32_t i = 0; i < SdrManager.count; i++) {
        sdr_receiver_t *rx = &SdrManager.receivers[i];
        if (rx->state < RX_STATE_RUNNING) continue;
        if (rx->config.role != SDR_ROLE_ADSB && !rx->decoder_state) continue;

        switch (rx->config.role) {
        case SDR_ROLE_ADSB: {
            struct stats s;
            add_stats(&Modes.stats_alltime, &Modes.stats_current, &s);
            out->adsb_messages = s.messages_total;
            out->adsb_tracks = s.unique_aircraft;
            out->adsb_gain_db = (int16_t)s.sdr_gain;
            if (s.noise_power_count > 0) {
                double p = s.noise_power_sum / s.noise_power_count;
                out->adsb_noise_dbfs = (p > 0) ? (float)(10.0 * log10(p)) : -999.0f;
            } else {
                out->adsb_noise_dbfs = -999.0f;
            }
            if (s.signal_power_count > 0) {
                double p = s.signal_power_sum / s.signal_power_count;
                out->adsb_signal_dbfs = (p > 0) ? (float)(10.0 * log10(p)) : -999.0f;
            } else {
                out->adsb_signal_dbfs = -999.0f;
            }
            break;
        }
        case SDR_ROLE_FLARM: {
            flarm_demod_stats_t fs;
            if (flarmDecoderGetStats(rx, &fs)) {
                out->flarm_detected = (uint32_t)fs.packets_detected;
                out->flarm_decoded  = (uint32_t)fs.packets_decoded;
            }
            break;
        }
        case SDR_ROLE_ACARS: {
            acars_ctx_t *c = (acars_ctx_t *)rx->decoder_state;
            if (c && c->inner) {
                acars_stats_t as;
                acars_get_stats(c->inner, &as);
                out->acars_decoded = (uint32_t)as.messages_decoded;
            }
            break;
        }
        case SDR_ROLE_VDL2: {
            vdl2_ctx_t *c = (vdl2_ctx_t *)rx->decoder_state;
            if (c && c->inner) {
                vdl2_stats_t vs;
                vdl2_get_stats(c->inner, &vs);
                out->vdl2_decoded = (uint32_t)vs.messages_decoded;
            }
            break;
        }
        case SDR_ROLE_RADIOSONDE: {
            sonde_ctx_t *c = (sonde_ctx_t *)rx->decoder_state;
            if (c && c->inner) {
                sonde_stats_t ss;
                sonde_get_stats(c->inner, &ss);
                out->sonde_decoded = (uint32_t)ss.frames_decoded;
            }
            break;
        }
        case SDR_ROLE_POCSAG: {
            pocsag_ctx_t *c = (pocsag_ctx_t *)rx->decoder_state;
            if (c && c->inner) {
                pocsag_stats_t ps;
                pocsag_get_stats(c->inner, &ps);
                out->pocsag_decoded = (uint32_t)ps.messages_decoded;
            }
            break;
        }
        case SDR_ROLE_GSM: {
            gsm_ctx_t *c = (gsm_ctx_t *)rx->decoder_state;
            if (c && c->inner) {
                gsm_stats_t gs;
                gsm_get_stats(c->inner, &gs);
                out->gsm_bcch = (uint32_t)gs.bcch_decoded;
            }
            break;
        }
        case SDR_ROLE_LTE: {
            lte_ctx_t *c = (lte_ctx_t *)rx->decoder_state;
            if (c && c->inner) {
                lte_stats_t ls;
                lte_get_stats(c->inner, &ls);
                out->lte_mib = ls.mib_decoded;
            }
            break;
        }
        case SDR_ROLE_FANET: {
            fanet_state_t *st = (fanet_state_t *)rx->decoder_state;
            if (st) {
                fanet_stats_t fs;
                fanet_get_stats(st, &fs);
                out->fanet_decoded = (uint32_t)fs.packets_decoded;
            }
            break;
        }
        case SDR_ROLE_SARSAT: {
            sarsat_ctx_t *c = (sarsat_ctx_t *)rx->decoder_state;
            if (c && c->inner) {
                sarsat_stats_t ss;
                sarsat_get_stats(c->inner, &ss);
                out->sarsat_frames = (uint32_t)ss.frames_decoded;
            }
            break;
        }
        default: break;
        }
    }
}
