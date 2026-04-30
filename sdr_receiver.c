// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sdr_receiver.c: Multi-SDR dynamic receiver management
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#include "dump1090.h"
#include "sdr_receiver.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include "config_panel.h"
#include "pocsag_demod.h"
#include "gsm_decode.h"
#include "gsm_tracker.h"
#include "lte_decode.h"
#include "lte_tracker.h"

#ifdef ENABLE_RTLSDR
#include <rtl-sdr.h>
#endif

// ======================== Global state ========================

sdr_manager_t SdrManager;
int PocsagOutputEnabled = 1;  // toggled from panel; 1 = decode & show messages
int GsmOutputEnabled = 1;     // toggled from panel; 1 = decode & show GSM cells
int LteOutputEnabled = 1;     // toggled from panel; 1 = decode & show LTE cells

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
        default:                  return "none";
    }
}

bool rxRoleIsDecoder(sdr_role_t role)
{
    return (role == SDR_ROLE_ACARS || role == SDR_ROLE_VDL2 || role == SDR_ROLE_RADIOSONDE || role == SDR_ROLE_POCSAG || role == SDR_ROLE_GSM || role == SDR_ROLE_LTE);
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
    } else {
        fprintf(stderr, "sdr_receiver: unknown role '%s' (use adsb/flarm/acars/vdl2/radiosonde/pocsag/gsm/lte)\n", token);
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
            config->freq = (int)strtol(token + 5, NULL, 10);
        } else if (!strncasecmp(token, "rate=", 5)) {
            config->sample_rate = strtod(token + 5, NULL);
        } else {
            fprintf(stderr, "sdr_receiver: unknown option '%s'\n", token);
            return false;
        }
    }

    return true;
}

// ======================== Per-receiver FIFO ========================

bool rxFifoCreate(rx_fifo_t *fifo, unsigned buffer_count, unsigned buffer_size, unsigned overlap)
{
    pthread_mutex_init(&fifo->mutex, NULL);
    pthread_cond_init(&fifo->notempty_cond, NULL);
    pthread_cond_init(&fifo->empty_cond, NULL);
    pthread_cond_init(&fifo->free_cond, NULL);

    fifo->head = NULL;
    fifo->tail = NULL;
    fifo->freelist = NULL;
    fifo->halted = false;

    if (!(fifo->overlap_buffer = calloc(overlap, sizeof(uint16_t))))
        goto nomem;
    fifo->overlap_length = overlap;

    for (unsigned i = 0; i < buffer_count; ++i) {
        struct mag_buf *newbuf;
        if (!(newbuf = calloc(1, sizeof(*newbuf))))
            goto nomem;
        if (!(newbuf->data = calloc(buffer_size, sizeof(newbuf->data[0])))) {
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
                fprintf(stderr, "rxFifoAcquire: pthread_cond_timedwait: %s\n", strerror(err));
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
                fprintf(stderr, "rxFifoDequeue: pthread_cond_timedwait: %s\n", strerror(err));
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

// ======================== Per-receiver RTL-SDR ops ========================

#ifdef ENABLE_RTLSDR

static int rx_sort_gains(const void *left, const void *right)
{
    return *(const int *)left - *(const int *)right;
}

int rxGetGain(sdr_receiver_t *rx)
{
    return rx->rtl.current_gain;
}

int rxGetMaxGain(sdr_receiver_t *rx)
{
    return rx->rtl.gain_steps - 1;
}

double rxGetGainDb(sdr_receiver_t *rx, int step)
{
    if (!rx->rtl.gains) return 0.0;
    if (step < 0) step = 0;
    if (step >= rx->rtl.gain_steps) step = rx->rtl.gain_steps - 1;
    return rx->rtl.gains[step] / 10.0;
}

int rxSetGain(sdr_receiver_t *rx, int step)
{
    rtlsdr_dev_t *dev = (rtlsdr_dev_t *)rx->rtl.dev;
    if (!rx->rtl.gains || !dev) return -1;

    if (step < 0) step = 0;
    if (step >= rx->rtl.gain_steps) step = rx->rtl.gain_steps - 1;

    if (step == rx->rtl.gain_steps - 1) {
        if (rtlsdr_set_tuner_gain_mode(dev, 0) < 0) {
            fprintf(stderr, "rx[%d]: failed to enable tuner AGC\n", rx->id);
            return rx->rtl.current_gain;
        }
        fprintf(stderr, "rx[%d]: tuner gain set to about %.1f dB (step %d, AGC)\n",
                rx->id, rx->rtl.gains[step] / 10.0, step);
    } else {
        if (rtlsdr_set_tuner_gain_mode(dev, 1) < 0) {
            fprintf(stderr, "rx[%d]: failed to disable tuner AGC\n", rx->id);
            return rx->rtl.current_gain;
        }
        if (rtlsdr_set_tuner_gain(dev, rx->rtl.gains[step]) < 0) {
            fprintf(stderr, "rx[%d]: failed to set tuner gain to %.1fdB\n",
                    rx->id, rx->rtl.gains[step] / 10.0);
            return rx->rtl.current_gain;
        }
        fprintf(stderr, "rx[%d]: tuner gain set to %.1f dB (step %d)\n",
                rx->id, rx->rtl.gains[step] / 10.0, step);
    }

    rx->rtl.current_gain = step;
    return step;
}

static int rx_find_device_index(const char *serial)
{
    int device_count = rtlsdr_get_device_count();
    if (!device_count) return -1;

    for (int i = 0; i < device_count; i++) {
        char dev_serial[256];
        if (rtlsdr_get_device_usb_strings(i, NULL, NULL, dev_serial) == 0) {
            if (!strcmp(serial, dev_serial))
                return i;
        }
    }

    // Try prefix match
    for (int i = 0; i < device_count; i++) {
        char dev_serial[256];
        if (rtlsdr_get_device_usb_strings(i, NULL, NULL, dev_serial) == 0) {
            if (!strncmp(serial, dev_serial, strlen(serial)))
                return i;
        }
    }

    return -1;
}

// RTL-SDR async callback — context points to the sdr_receiver_t
static void rx_rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
    sdr_receiver_t *rx = (sdr_receiver_t *)ctx;
    rtlsdr_dev_t *dev = (rtlsdr_dev_t *)rx->rtl.dev;

    if (Modes.exit || rx->state == RX_STATE_STOPPING) {
        rtlsdr_cancel_async(dev);
        return;
    }

    // Apply deferred retune (e.g. LTE frequency hopping)
    uint32_t new_freq = rx->pending_freq;
    if (new_freq) {
        rx->pending_freq = 0;
        int ret = rtlsdr_set_center_freq(dev, new_freq);
        if (ret < 0) {
            rx->usb_error_count++;
            rx->usb_error_total++;
            if (rx->usb_error_count == 1) {
                fprintf(stderr, "rx[%d]: rtlsdr_set_center_freq failed (%d), "
                        "USB error recovery started\n", rx->id, ret);
            }
            // After 10 consecutive failures, cancel async to trigger device reset
            if (rx->usb_error_count >= 10) {
                fprintf(stderr, "rx[%d]: %u consecutive USB errors, "
                        "cancelling async for device reset\n",
                        rx->id, rx->usb_error_count);
                rtlsdr_cancel_async(dev);
            }
            return; // discard buffer, don't process stale data
        }
        rx->usb_error_count = 0;  // reset on success
        rx->config.freq = new_freq;
        return; // discard this buffer (samples at transitional freq)
    }

    unsigned samples_read = len / 2;
    if (!samples_read) return;

    // Dispatch to decoder_ops (all roles use this uniform path)
    if (rx->decoder_ops && rx->decoder_ops->process) {
        rx->decoder_ops->process(rx, buf, len);
    }

    // Sample IQ noise power for auto-gain (first 256 pairs per callback)
    {
        unsigned n = (samples_read < 256) ? samples_read : 256;
        uint64_t s = 0;
        for (unsigned i = 0; i < n * 2; i += 2) {
            int I = (int)buf[i] - 128;
            int Q = (int)buf[i+1] - 128;
            s += (unsigned)(I*I + Q*Q);
        }
        __atomic_add_fetch(&rx->ag_iq_sum, s, __ATOMIC_RELAXED);
        __atomic_add_fetch(&rx->ag_iq_count, n, __ATOMIC_RELAXED);
    }

    rx->sample_counter += samples_read;
}

bool rxOpen(sdr_receiver_t *rx)
{
    if (rx->state != RX_STATE_IDLE) {
        fprintf(stderr, "rx[%d]: cannot open, state is %s\n", rx->id, rxStateName(rx->state));
        return false;
    }

    // All roles (ADSB, FLARM, ACARS, VDL2, Radiosonde) open the RTL-SDR device
    // Decoder roles skip converter/FIFO setup (handled after RTL-SDR configuration below)

    int dev_count = rtlsdr_get_device_count();
    if (!dev_count) {
        fprintf(stderr, "rx[%d]: no RTL-SDR devices found\n", rx->id);
        rx->state = RX_STATE_ERROR;
        return false;
    }

    int dev_index = -1;
    if (rx->config.serial[0]) {
        dev_index = rx_find_device_index(rx->config.serial);
        if (dev_index < 0) {
            fprintf(stderr, "rx[%d]: no device matching serial '%s'\n", rx->id, rx->config.serial);
            rx->state = RX_STATE_ERROR;
            return false;
        }
    } else {
        // Auto-assign: find first device not already in use by another receiver
        for (int i = 0; i < dev_count; i++) {
            char serial[256];
            if (rtlsdr_get_device_usb_strings(i, NULL, NULL, serial) != 0)
                continue;
            // Check if already used
            bool in_use = false;
            for (int r = 0; r < SdrManager.count; r++) {
                if (r != rx->id && SdrManager.receivers[r].state >= RX_STATE_OPEN &&
                    !strcmp(SdrManager.receivers[r].serial_actual, serial)) {
                    in_use = true;
                    break;
                }
            }
            if (!in_use) {
                dev_index = i;
                break;
            }
        }
        if (dev_index < 0) {
            fprintf(stderr, "rx[%d]: no free RTL-SDR device available\n", rx->id);
            rx->state = RX_STATE_ERROR;
            return false;
        }
    }

    // Query device identity
    if (rtlsdr_get_device_usb_strings(dev_index,
                                       rx->manufacturer,
                                       rx->product,
                                       rx->serial_actual) < 0) {
        fprintf(stderr, "rx[%d]: error querying device #%d\n", rx->id, dev_index);
        rx->state = RX_STATE_ERROR;
        return false;
    }
    rx->dev_index = dev_index;

    fprintf(stderr, "rx[%d]: opening device #%d: %s (%s, %s, SN %s) role=%s\n",
            rx->id, dev_index,
            rtlsdr_get_device_name(dev_index),
            rx->manufacturer, rx->product, rx->serial_actual,
            sdrRoleName(rx->config.role));

    rtlsdr_dev_t *dev = NULL;
    if (rtlsdr_open(&dev, dev_index) < 0) {
        fprintf(stderr, "rx[%d]: error opening RTL-SDR: %s\n", rx->id, strerror(errno));
        rx->state = RX_STATE_ERROR;
        return false;
    }
    rx->rtl.dev = dev;
    rx->rtl.tuner_type = rtlsdr_get_tuner_type(dev);

    // Gain setup
    if (rx->config.direct_sampling) {
        fprintf(stderr, "rx[%d]: direct sampling from input %d\n", rx->id, rx->config.direct_sampling);
        rtlsdr_set_direct_sampling(dev, rx->config.direct_sampling);
        rx->rtl.gain_steps = 0;
    } else {
        int numgains = rtlsdr_get_tuner_gains(dev, NULL);
        if (numgains <= 0) {
            fprintf(stderr, "rx[%d]: error getting tuner gains\n", rx->id);
            rxClose(rx);
            return false;
        }

        int *gains = malloc((numgains + 1) * sizeof(int));
        if (rtlsdr_get_tuner_gains(dev, gains) != numgains) {
            fprintf(stderr, "rx[%d]: error getting tuner gains\n", rx->id);
            free(gains);
            rxClose(rx);
            return false;
        }

        qsort(gains, numgains, sizeof(gains[0]), rx_sort_gains);

        // Fake entry at slightly higher than max: "tuner AGC enabled"
        gains[numgains] = gains[numgains - 1] + 90;
        rx->rtl.gain_steps = numgains + 1;
        rx->rtl.gains = gains;

        // Select gain step
        int selected = -1;
        if (rx->config.gain == MODES_LEGACY_AUTO_GAIN) {
            selected = numgains;  // AGC
        } else if (rx->config.gain == MODES_DEFAULT_GAIN) {
            selected = numgains - 1;  // max manual gain
        } else {
            for (int i = 0; i <= numgains; ++i) {
                if (selected == -1 || fabs(gains[i] / 10.0 - rx->config.gain) < fabs(gains[selected] / 10.0 - rx->config.gain))
                    selected = i;
            }
        }

        rxSetGain(rx, selected);
    }

    if (rx->config.digital_agc) {
        fprintf(stderr, "rx[%d]: enabling digital AGC\n", rx->id);
        rtlsdr_set_agc_mode(dev, 1);
    }

    rtlsdr_set_freq_correction(dev, rx->config.ppm_error);
    rtlsdr_set_center_freq(dev, rx->config.freq);
    rtlsdr_set_sample_rate(dev, (unsigned)rx->config.sample_rate);
    rtlsdr_reset_buffer(dev);

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
        if (rtlsdr_get_center_freq(dev) != (uint32_t)rx->config.freq) {
            rtlsdr_set_center_freq(dev, rx->config.freq);
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
    return true;
}

// Reader thread entry point
static void *rx_reader_thread(void *arg)
{
    sdr_receiver_t *rx = (sdr_receiver_t *)arg;
    rtlsdr_dev_t *dev = (rtlsdr_dev_t *)rx->rtl.dev;

    fprintf(stderr, "rx[%d]: reader thread started (role=%s, serial=%s)\n",
            rx->id, sdrRoleName(rx->config.role), rx->serial_actual);

    rtlsdr_read_async(dev, rx_rtlsdr_callback, rx, 4, MODES_RTL_BUF_SIZE);

    // If cancelled due to USB errors (not shutdown), try to recover
    if (!Modes.exit && rx->state != RX_STATE_STOPPING && rx->usb_error_count > 0) {
        fprintf(stderr, "rx[%d]: USB error recovery — closing and reopening device\n", rx->id);
        rtlsdr_close(dev);
        rx->rtl.dev = NULL;

        // Wait 2 seconds for USB to settle
        struct timespec ts = {2, 0};
        nanosleep(&ts, NULL);

        if (Modes.exit) goto done;

        // Reopen device
        rtlsdr_dev_t *new_dev = NULL;
        int ret = rtlsdr_open(&new_dev, (uint32_t)rx->dev_index);
        if (ret < 0) {
            fprintf(stderr, "rx[%d]: failed to reopen device (err=%d), giving up\n", rx->id, ret);
            rx->state = RX_STATE_ERROR;
            goto done;
        }
        rx->rtl.dev = new_dev;
        dev = new_dev;

        // Reconfigure device
        rtlsdr_set_tuner_gain_mode(dev, 1);
        if (rx->rtl.gains && rx->rtl.current_gain < rx->rtl.gain_steps)
            rtlsdr_set_tuner_gain(dev, rx->rtl.gains[rx->rtl.current_gain]);
        rtlsdr_set_freq_correction(dev, rx->config.ppm_error);
        rtlsdr_set_center_freq(dev, rx->config.freq);
        rtlsdr_set_sample_rate(dev, (unsigned)rx->config.sample_rate);
        rtlsdr_reset_buffer(dev);
        rx->usb_error_count = 0;

        fprintf(stderr, "rx[%d]: device reopened successfully, resuming\n", rx->id);

        // Resume async read
        rtlsdr_read_async(dev, rx_rtlsdr_callback, rx, 4, MODES_RTL_BUF_SIZE);
    }

    if (!Modes.exit && rx->state != RX_STATE_STOPPING) {
        fprintf(stderr, "rx[%d]: rtlsdr_read_async returned unexpectedly, device may be lost\n", rx->id);
        rx->state = RX_STATE_ERROR;
    }

done:
    fprintf(stderr, "rx[%d]: reader thread exiting\n", rx->id);
    return NULL;
}

bool rxStart(sdr_receiver_t *rx)
{
    if (rx->state != RX_STATE_OPEN) {
        fprintf(stderr, "rx[%d]: cannot start, state is %s\n", rx->id, rxStateName(rx->state));
        return false;
    }

    // External decoder: spawn process
    // Decoder roles use the same reader thread, no special start needed

    rx->state = RX_STATE_RUNNING;

    if (pthread_create(&rx->thread, NULL, rx_reader_thread, rx)) {
        fprintf(stderr, "rx[%d]: failed to create reader thread: %s\n", rx->id, strerror(errno));
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

    // Decoder roles use the same stop path as ADSB/FLARM

    rx->state = RX_STATE_STOPPING;

    rtlsdr_dev_t *dev = (rtlsdr_dev_t *)rx->rtl.dev;
    if (dev)
        rtlsdr_cancel_async(dev);

    rxFifoHalt(&rx->fifo);

    if (rx->thread_started) {
        pthread_join(rx->thread, NULL);
        rx->thread_started = false;
    }

    rx->state = RX_STATE_OPEN;
    fprintf(stderr, "rx[%d]: stopped\n", rx->id);
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

    rtlsdr_dev_t *dev = (rtlsdr_dev_t *)rx->rtl.dev;
    if (dev) {
        rtlsdr_close(dev);
        rx->rtl.dev = NULL;
    }

    free(rx->rtl.gains);
    rx->rtl.gains = NULL;

    rx->state = RX_STATE_IDLE;
    fprintf(stderr, "rx[%d]: closed\n", rx->id);
}

int sdrEnumerateDevices(char serials[][64], int max_devices)
{
    int count = rtlsdr_get_device_count();
    int filled = 0;
    for (int i = 0; i < count && filled < max_devices; i++) {
        char serial[256];
        if (rtlsdr_get_device_usb_strings(i, NULL, NULL, serial) == 0) {
            snprintf(serials[filled], 64, "%.63s", serial);
            filled++;
        }
    }
    return filled;
}

#else // !ENABLE_RTLSDR — stubs

int rxGetGain(sdr_receiver_t *rx)     { MODES_NOTUSED(rx); return -1; }
int rxGetMaxGain(sdr_receiver_t *rx)  { MODES_NOTUSED(rx); return -1; }
double rxGetGainDb(sdr_receiver_t *rx, int step) { MODES_NOTUSED(rx); MODES_NOTUSED(step); return 0.0; }
int rxSetGain(sdr_receiver_t *rx, int step) { MODES_NOTUSED(rx); MODES_NOTUSED(step); return -1; }
bool rxOpen(sdr_receiver_t *rx)       { fprintf(stderr, "rx[%d]: RTLSDR support not compiled\n", rx->id); rx->state = RX_STATE_ERROR; return false; }
bool rxStart(sdr_receiver_t *rx)      { MODES_NOTUSED(rx); return false; }
void rxStop(sdr_receiver_t *rx)       { MODES_NOTUSED(rx); }
void rxClose(sdr_receiver_t *rx)      { MODES_NOTUSED(rx); rx->state = RX_STATE_IDLE; }
int sdrEnumerateDevices(char serials[][64], int max_devices) { MODES_NOTUSED(serials); MODES_NOTUSED(max_devices); return 0; }

#endif // ENABLE_RTLSDR

// ======================== Internal Decoder Integration ========================

#include "acars_demod.h"
#include "vdl2_demod.h"
#include "sonde_demod.h"

// ACARS message callback → panelLogMessage
static void acars_message_handler(const acars_msg_t *msg, void *ctx)
{
    (void)ctx;
    panelLogMessage("[ACARS] %.3f MHz %s %s (reg:%s) [%s] %s",
                    msg->freq / 1e6,
                    msg->flight[0] ? msg->flight : "???",
                    msg->label,
                    msg->reg[0] ? msg->reg : "?",
                    msg->label,
                    msg->text[0] ? msg->text : "(empty)");
}

// VDL2 message callback → panelLogMessage
static void vdl2_message_handler(const vdl2_msg_t *msg, void *ctx)
{
    (void)ctx;
    if (msg->has_acars) {
        panelLogMessage("[VDL2] %.3f MHz %s %s (reg:%s) [%s] SNR=%.0fdB %s",
                        msg->freq / 1e6,
                        msg->frame_type,
                        msg->flight[0] ? msg->flight : "???",
                        msg->reg[0] ? msg->reg : "?",
                        msg->label,
                        msg->snr,
                        msg->text[0] ? msg->text : "(no text)");
    } else {
        panelLogMessage("[VDL2] %.3f MHz %s src=%06X dst=%06X SNR=%.0fdB [%d bytes]",
                        msg->freq / 1e6,
                        msg->frame_type,
                        msg->src.addr,
                        msg->dst.addr,
                        msg->snr,
                        msg->info_len);
    }
}

// Sonde message callback → panelLogMessage + SondeHub upload
static void sonde_message_handler(const sonde_msg_t *msg, void *ctx)
{
    (void)ctx;
    if (msg->valid_pos) {
        panelLogMessage("[SONDE] %s %s pos=%.4f,%.4f alt=%.0fm vel=%.1fm/s frame=%d",
                        msg->type, msg->serial,
                        msg->lat, msg->lon, msg->alt,
                        msg->vel_h, msg->frame_num);
        sondehubClientSubmit(msg);
    } else {
        panelLogMessage("[SONDE] %s %s frame=%d (no GPS fix)",
                        msg->type, msg->serial, msg->frame_num);
    }
}

// POCSAG message callback → panelLogMessage
static void pocsag_message_handler(const pocsag_msg_t *msg, void *ctx)
{
    (void)ctx;
    if (!PocsagOutputEnabled) return;

    const char *freq_str = "";
    char freq_buf[32];
    if (msg->channel_freq > 0) {
        snprintf(freq_buf, sizeof(freq_buf), " %.3fMHz", msg->channel_freq / 1e6);
        freq_str = freq_buf;
    }

    if (msg->is_tone_only) {
        panelLogMessage("[POCSAG]%s %d baud addr=%07u func=%d TONE-ONLY",
                        freq_str, msg->baud_rate, msg->address, msg->function);
    } else if (msg->is_alpha && msg->alpha_len > 0) {
        panelLogMessage("[POCSAG]%s %d baud addr=%07u func=%d \"%s\"",
                        freq_str, msg->baud_rate, msg->address, msg->function,
                        msg->alpha_msg);
    } else if (msg->is_numeric && msg->numeric_len > 0) {
        panelLogMessage("[POCSAG]%s %d baud addr=%07u func=%d num=%s",
                        freq_str, msg->baud_rate, msg->address, msg->function,
                        msg->numeric_msg);
    } else {
        panelLogMessage("[POCSAG]%s %d baud addr=%07u func=%d (empty)",
                        freq_str, msg->baud_rate, msg->address, msg->function);
    }
}

// GSM message callback → panelLogMessage + cell tracking
static void gsm_message_handler(const gsm_cell_info_t *cell, const char *msg_type,
                                 const uint8_t *l3_data, int l3_len, void *ctx)
{
    (void)l3_data; (void)l3_len;
    if (!GsmOutputEnabled) return;

    // Update tracker from the decoder that produced this message
    sdr_receiver_t *rx = (sdr_receiver_t *)ctx;
    if (rx && rx->decoder_state) {
        gsm_stats_t stats;
        gsm_get_stats((struct gsm_state *)rx->decoder_state, &stats);
        gsm_sync_state_t sync = gsm_get_sync_state((struct gsm_state *)rx->decoder_state);
        gsmTrackerUpdate(cell, &stats, sync);
    }

    panelLogMessage("[GSM] MCC=%d MNC=%d LAC=%u CID=%u ARFCN=%d BSIC=%02X %s",
                    cell->si3.mcc, cell->si3.mnc, cell->si3.lac, cell->si3.cell_id,
                    cell->arfcn, cell->bsic, msg_type);
}

// GSM Cell Broadcast callback
static void gsm_cb_handler(const gsm_cell_info_t *cell, const gsm_cb_msg_t *cb, void *ctx)
{
    (void)ctx;
    if (!GsmOutputEnabled) return;

    gsmTrackerUpdateCB(cell, cb);

    panelLogMessage("[GSM-CB] MCC=%d MNC=%d LAC=%u CID=%u serial=%u id=%u \"%s\"",
                    cell->si3.mcc, cell->si3.mnc, cell->si3.lac, cell->si3.cell_id,
                    cb->serial_nr, cb->msg_id, cb->text);
}

// LTE cell callback → tracker update + log
static void lte_cell_handler(const lte_cell_info_t *cell, void *ctx)
{
    (void)ctx;
    if (!LteOutputEnabled) return;

    lteTrackerUpdate(cell);
}

bool rxDecoderCreate(sdr_receiver_t *rx)
{
    switch (rx->config.role) {
    case SDR_ROLE_ACARS: {
        acars_config_t cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.center_freq = rx->config.freq;
        cfg.sample_rate = rx->config.sample_rate;
        // Default European ACARS channels (within ~1 MHz of center)
        cfg.channel_freqs[0] = ACARS_FREQ_PRIMARY;    // 131.550
        cfg.channel_freqs[1] = ACARS_FREQ_SECONDARY;  // 130.025
        cfg.channel_freqs[2] = ACARS_FREQ_TERTIARY;   // 131.725
        cfg.channel_freqs[3] = ACARS_FREQ_4;          // 130.450
        cfg.channel_freqs[4] = ACARS_FREQ_5;          // 129.125
        cfg.num_channels = 5;
        cfg.callback = acars_message_handler;
        cfg.callback_ctx = rx;

        // Adjust center freq to cover all channels
        // Center between min and max channel freq
        double fmin = cfg.channel_freqs[4]; // 129.125
        double fmax = cfg.channel_freqs[2]; // 131.725
        cfg.center_freq = (fmin + fmax) / 2.0;
        // Update the receiver's actual center freq
        rx->config.freq = (unsigned)cfg.center_freq;

        rx->decoder_state = acars_create(&cfg);
        if (!rx->decoder_state) return false;
        fprintf(stderr, "rx[%d]: ACARS decoder created, center=%.3f MHz, %d channels\n",
                rx->id, cfg.center_freq / 1e6, cfg.num_channels);
        return true;
    }
    case SDR_ROLE_VDL2: {
        vdl2_config_t cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.center_freq = rx->config.freq;
        cfg.sample_rate = rx->config.sample_rate;
        cfg.num_channels = 3;
        cfg.channel_freqs[0] = VDL2_FREQ_EU_1;   // 136.975 MHz
        cfg.channel_freqs[1] = VDL2_FREQ_EU_2;   // 136.875 MHz
        cfg.channel_freqs[2] = VDL2_FREQ_EU_3;   // 136.775 MHz
        cfg.squelch_level = -32.0f;               // -32 dBFS squelch
        cfg.callback = vdl2_message_handler;
        cfg.callback_ctx = rx;

        rx->decoder_state = vdl2_create(&cfg);
        if (!rx->decoder_state) return false;
        fprintf(stderr, "rx[%d]: VDL2 decoder created, %d channels\n",
                rx->id, cfg.num_channels);
        return true;
    }
    case SDR_ROLE_RADIOSONDE: {
        sonde_config_t cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.center_freq = rx->config.freq;
        cfg.sample_rate = rx->config.sample_rate;
        cfg.callback = sonde_message_handler;
        cfg.callback_ctx = rx;

        rx->decoder_state = sonde_create(&cfg);
        if (!rx->decoder_state) return false;
        fprintf(stderr, "rx[%d]: Radiosonde decoder created, freq=%.3f MHz\n",
                rx->id, cfg.center_freq / 1e6);
        return true;
    }
    case SDR_ROLE_POCSAG: {
        pocsag_config_t cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.center_freq = rx->config.freq;
        cfg.sample_rate = rx->config.sample_rate;
        cfg.callback = pocsag_message_handler;
        cfg.callback_ctx = rx;

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
            rx->config.freq = (unsigned)cfg.center_freq;  // retune SDR
            fprintf(stderr, "rx[%d]: POCSAG multi-channel mode, center=%.3f MHz, %d channels\n",
                    rx->id, cfg.center_freq / 1e6, cfg.num_channels);
        }

        rx->decoder_state = pocsag_create(&cfg);
        if (!rx->decoder_state) return false;
        fprintf(stderr, "rx[%d]: POCSAG decoder created, freq=%.3f MHz, sr=%.0f\n",
                rx->id, cfg.center_freq / 1e6, cfg.sample_rate);
        return true;
    }
    case SDR_ROLE_GSM: {
        gsm_config_t cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.arfcn_freq = rx->config.freq;
        cfg.center_freq = rx->config.freq - GSM_IF_OFFSET;
        cfg.sample_rate = rx->config.sample_rate;
        cfg.tsc = -1; // auto-detect
        cfg.msg_cb = gsm_message_handler;
        cfg.cb_cb = gsm_cb_handler;
        cfg.callback_ctx = rx;

        rx->decoder_state = gsm_create(&cfg);
        if (!rx->decoder_state) return false;
        /* Retune to IF-offset frequency to avoid DC spike */
        rx->config.freq = (unsigned)cfg.center_freq;
        fprintf(stderr, "rx[%d]: GSM decoder created, arfcn=%.3f MHz, tuned=%.3f MHz (IF offset %d Hz), sr=%.0f\n",
                rx->id, cfg.arfcn_freq / 1e6, cfg.center_freq / 1e6, GSM_IF_OFFSET, cfg.sample_rate);
        return true;
    }
    case SDR_ROLE_LTE: {
        lte_config_t cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.center_freq = rx->config.freq;
        cfg.sample_rate = rx->config.sample_rate;
        cfg.callback = lte_cell_handler;
        cfg.callback_ctx = rx;
        cfg.hop_enabled = true;  // Enable Band 20 hopping (796/806/816 MHz)

        rx->decoder_state = lte_create(&cfg);
        if (!rx->decoder_state) return false;
        fprintf(stderr, "rx[%d]: LTE decoder created, freq=%.3f MHz, sr=%.0f, hop=on\n",
                rx->id, cfg.center_freq / 1e6, cfg.sample_rate);
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
    case SDR_ROLE_ACARS:
        acars_destroy((struct acars_state *)rx->decoder_state);
        break;
    case SDR_ROLE_VDL2:
        vdl2_destroy((struct vdl2_state *)rx->decoder_state);
        break;
    case SDR_ROLE_RADIOSONDE:
        sonde_destroy((struct sonde_state *)rx->decoder_state);
        break;
    case SDR_ROLE_POCSAG:
        pocsag_destroy((struct pocsag_state *)rx->decoder_state);
        break;
    case SDR_ROLE_GSM:
        gsm_destroy((struct gsm_state *)rx->decoder_state);
        break;
    case SDR_ROLE_LTE:
        lte_destroy((struct lte_state *)rx->decoder_state);
        break;
    default:
        break;
    }
    rx->decoder_state = NULL;
}

void rxDecoderProcess(sdr_receiver_t *rx, const uint8_t *iq_data, unsigned len)
{
    if (!rx->decoder_state) return;

    switch (rx->config.role) {
    case SDR_ROLE_ACARS:
        acars_process((struct acars_state *)rx->decoder_state, iq_data, len);
        break;
    case SDR_ROLE_VDL2:
        vdl2_process((struct vdl2_state *)rx->decoder_state, iq_data, len);
        break;
    case SDR_ROLE_RADIOSONDE:
        sonde_process((struct sonde_state *)rx->decoder_state, iq_data, len);
        break;
    case SDR_ROLE_POCSAG:
        pocsag_process((struct pocsag_state *)rx->decoder_state, iq_data, len);
        break;
    case SDR_ROLE_GSM:
        gsm_process((struct gsm_state *)rx->decoder_state, iq_data, len);
        // Periodic FCCH-only tracker update (even without decoded SI)
        {
            static uint64_t gsm_tracker_last = 0;
            gsm_stats_t gstats;
            gsm_get_stats((struct gsm_state *)rx->decoder_state, &gstats);
            if (gstats.fcch_detected > 0 && gstats.samples_processed - gsm_tracker_last >= 5000000) {
                gsm_tracker_last = gstats.samples_processed;
                gsm_cell_info_t gcell;
                gsm_get_cell_info((struct gsm_state *)rx->decoder_state, &gcell);
                gsm_sync_state_t gsync = gsm_get_sync_state((struct gsm_state *)rx->decoder_state);
                if (gcell.si3.mcc == 0) {
                    // No SI3 decoded yet — update with FCCH-only info
                    gsmTrackerUpdateFCCH(gcell.arfcn, gcell.freq_mhz, &gstats, gsync);
                }
            }
        }
        break;
    case SDR_ROLE_LTE:
        lte_process((struct lte_state *)rx->decoder_state, iq_data, len);
        // Check if decoder requests a frequency hop (deferred retune)
        {
            double hop_freq = lte_get_hop_freq((struct lte_state *)rx->decoder_state);
            if (hop_freq > 0) {
                rx->pending_freq = (uint32_t)hop_freq;
                lte_set_freq((struct lte_state *)rx->decoder_state, hop_freq);
            }
        }
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
#include "demod_2400.h"     // demodulate2400

// ---- ACARS decoder_ops ----

static bool acarsDecoderInit(sdr_receiver_t *rx)    { return rxDecoderCreate(rx); }
static void acarsDecoderProcess(sdr_receiver_t *rx, const uint8_t *iq, uint32_t len) { rxDecoderProcess(rx, iq, len); }
static void acarsDecoderDrain(sdr_receiver_t *rx)    { (void)rx; /* messages dispatched inline via callback */ }
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
static void vdl2DecoderDrain(sdr_receiver_t *rx)    { (void)rx; }
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
static void sondeDecoderDrain(sdr_receiver_t *rx)    { (void)rx; }
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
static void pocsagDecoderDrain(sdr_receiver_t *rx)    { (void)rx; /* messages dispatched inline via callback */ }
static void pocsagDecoderStop(sdr_receiver_t *rx)     { rxDecoderDestroy(rx); }

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
static void gsmDecoderDrain(sdr_receiver_t *rx)    { (void)rx; /* messages dispatched inline via callback */ }
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
static void lteDecoderDrain(sdr_receiver_t *rx)    { (void)rx; }
static void lteDecoderStop(sdr_receiver_t *rx)     { rxDecoderDestroy(rx); }

static const decoder_ops_t lte_decoder_ops = {
    .name    = "lte",
    .init    = lteDecoderInit,
    .process = lteDecoderProcess,
    .drain   = lteDecoderDrain,
    .stop    = lteDecoderStop,
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
        fprintf(stderr, "rx[%d]: can't initialize sample converter\n", rx->id);
        return false;
    }

#if defined(__arm__) || defined(__aarch64__)
    rx->rtl.bounce_buffer = malloc(MODES_RTL_BUF_SIZE);
    if (!rx->rtl.bounce_buffer) {
        fprintf(stderr, "rx[%d]: can't allocate bounce buffer\n", rx->id);
        return false;
    }
#endif

    unsigned overlap = Modes.trailing_samples;
    if (!rxFifoCreate(&rx->fifo, MODES_MAG_BUFFERS,
                      MODES_MAG_BUF_SAMPLES + overlap, overlap)) {
        fprintf(stderr, "rx[%d]: can't create FIFO\n", rx->id);
        return false;
    }

    fprintf(stderr, "rx[%d]: ADS-B decoder created (IQ→mag converter + FIFO)\n", rx->id);
    return true;
}

static void adsbDecoderProcess(sdr_receiver_t *rx, const uint8_t *buf, uint32_t len)
{
    unsigned samples_read = len / 2;

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

    unsigned to_convert = samples_read;
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

static void adsbDecoderDrain(sdr_receiver_t *rx)
{
    struct mag_buf *buf = rxFifoDequeue(&rx->fifo, 0);  // non-blocking
    if (!buf) return;

    demodulate2400(buf);
    if (Modes.mode_ac) demodulate2400AC(buf);

    Modes.stats_current.samples_processed += buf->validLength - buf->overlap;
    Modes.stats_current.samples_dropped += buf->dropped;

    rxFifoRelease(&rx->fifo, buf);
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
    fprintf(stderr, "rx[%d]: ADS-B decoder stopped\n", rx->id);
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
    default:                  return NULL;
    }
}

// ======================== Manager ========================

void sdrManagerInit(void)
{
    memset(&SdrManager, 0, sizeof(SdrManager));
    pthread_mutex_init(&SdrManager.lock, NULL);

    for (int i = 0; i < MAX_SDR_RECEIVERS; i++) {
        SdrManager.receivers[i].id = i;
        SdrManager.receivers[i].state = RX_STATE_IDLE;
        pthread_mutex_init(&SdrManager.receivers[i].cpu_mutex, NULL);
    }
}

int sdrManagerAddReceiver(const rx_config_t *config)
{
    pthread_mutex_lock(&SdrManager.lock);

    // Reject duplicate serials
    for (int i = 0; i < SdrManager.count; i++) {
        if (!strcmp(SdrManager.receivers[i].config.serial, config->serial)) {
            fprintf(stderr, "sdr_manager: serial %s already managed (index %d)\n", config->serial, i);
            pthread_mutex_unlock(&SdrManager.lock);
            return -1;
        }
    }

    if (SdrManager.count >= MAX_SDR_RECEIVERS) {
        fprintf(stderr, "sdr_manager: max receivers (%d) reached\n", MAX_SDR_RECEIVERS);
        pthread_mutex_unlock(&SdrManager.lock);
        return -1;
    }

    int idx = SdrManager.count;
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

bool sdrManagerRemoveReceiver(int index)
{
    if (index < 0 || index >= SdrManager.count) return false;

    pthread_mutex_lock(&SdrManager.lock);

    sdr_receiver_t *rx = &SdrManager.receivers[index];
    if (rx->state == RX_STATE_RUNNING)
        rxStop(rx);
    if (rx->state != RX_STATE_IDLE)
        rxClose(rx);

    // Shift remaining receivers down
    for (int i = index; i < SdrManager.count - 1; i++) {
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

int sdrManagerOpenAll(void)
{
    int opened = 0;
    for (int i = 0; i < SdrManager.count; i++) {
        if (SdrManager.receivers[i].state == RX_STATE_IDLE) {
            if (rxOpen(&SdrManager.receivers[i]))
                opened++;
        }
    }
    return opened;
}

int sdrManagerStartAll(void)
{
    int started = 0;
    for (int i = 0; i < SdrManager.count; i++) {
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
    for (int i = 0; i < SdrManager.count; i++) {
        sdr_receiver_t *rx = &SdrManager.receivers[i];
        if (rx->state == RX_STATE_RUNNING && rx->decoder_ops && rx->decoder_ops->drain) {
            rx->decoder_ops->drain(rx);
            had_data = true;  // We checked at least one active receiver
        }
    }
    return had_data;
}

void sdrManagerStopAll(void)
{
    for (int i = 0; i < SdrManager.count; i++) {
        if (SdrManager.receivers[i].state == RX_STATE_RUNNING)
            rxStop(&SdrManager.receivers[i]);
    }
}

void sdrManagerCloseAll(void)
{
    for (int i = 0; i < SdrManager.count; i++) {
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

int sdrManagerFindBySerial(const char *serial)
{
    for (int i = 0; i < SdrManager.count; i++) {
        if (!strcmp(SdrManager.receivers[i].config.serial, serial) ||
            !strcmp(SdrManager.receivers[i].serial_actual, serial))
            return i;
    }
    return -1;
}

void sdrManagerUpdateConfig(int index, const rx_config_t *config)
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

sdr_receiver_t *sdrManagerGetReceiver(int index)
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
        fprintf(stderr, "sdr_manager: can't write %s: %s\n", RECEIVERS_JSON_PATH, strerror(errno));
        return false;
    }

    fprintf(f, "{\"receivers\":[\n");
    for (int i = 0; i < SdrManager.count; i++) {
        sdr_receiver_t *rx = &SdrManager.receivers[i];
        if (i > 0) fprintf(f, ",\n");
        fprintf(f, "  {\"serial\":\"%s\",\"role\":\"%s\",\"gain\":%.1f,\"ppm\":%d}",
                rx->config.serial, sdrRoleName(rx->config.role),
                rx->config.gain, rx->config.ppm_error);
    }
    fprintf(f, "\n]}\n");
    fclose(f);
    fprintf(stderr, "sdr_manager: saved %d receivers to %s\n", SdrManager.count, RECEIVERS_JSON_PATH);
    return true;
}

int sdrManagerLoad(void)
{
    FILE *f = fopen(RECEIVERS_JSON_PATH, "r");
    if (!f) return 0;  // no saved config — normal on first run

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    if (sz <= 0 || sz > 65536) { fclose(f); return 0; }
    rewind(f);

    char *data = malloc((size_t)sz + 1);
    if (!data) { fclose(f); return 0; }
    size_t rd = fread(data, 1, (size_t)sz, f);
    fclose(f);
    data[rd] = '\0';

    int loaded = 0;
    // Simple JSON parser: find each {"serial":"...","role":"...","gain":...,"ppm":...}
    const char *p = data;
    while ((p = strstr(p, "\"serial\"")) != NULL) {
        char serial[64] = {0};
        char role_str[16] = {0};
        float gain = 0;
        int ppm = 0;

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

        // Map role string to enum
        sdr_role_t role = SDR_ROLE_NONE;
        if (!strcmp(role_str, "adsb")) role = SDR_ROLE_ADSB;
        else if (!strcmp(role_str, "flarm")) role = SDR_ROLE_FLARM;
        else if (!strcmp(role_str, "acars")) role = SDR_ROLE_ACARS;
        else if (!strcmp(role_str, "vdl2")) role = SDR_ROLE_VDL2;
        else if (!strcmp(role_str, "radiosonde")) role = SDR_ROLE_RADIOSONDE;
        else if (!strcmp(role_str, "pocsag")) role = SDR_ROLE_POCSAG;
        else if (!strcmp(role_str, "gsm")) role = SDR_ROLE_GSM;

        if (role != SDR_ROLE_NONE && serial[0]) {
            // Skip duplicates within JSON itself
            if (sdrManagerFindBySerial(serial) < 0) {
                rx_config_t cfg = {0};
                snprintf(cfg.serial, sizeof(cfg.serial), "%.63s", serial);
                cfg.role = role;
                cfg.gain = gain;
                cfg.ppm_error = ppm;
                // Set freq/sample_rate for role
                switch (role) {
                    case SDR_ROLE_ADSB:       cfg.freq = 1090000000; cfg.sample_rate = 2400000; break;
                    case SDR_ROLE_FLARM:      cfg.freq = 868300000;  cfg.sample_rate = 1600000; break;
                    case SDR_ROLE_ACARS:      cfg.freq = 131550000;  cfg.sample_rate = 2400000; break;
                    case SDR_ROLE_VDL2:       cfg.freq = 136975000;  cfg.sample_rate = 2400000; break;
                    case SDR_ROLE_RADIOSONDE: cfg.freq = 403000000;  cfg.sample_rate = 2400000; break;
                    case SDR_ROLE_POCSAG:     cfg.freq = 466075000;  cfg.sample_rate = 1200000; break;
                    case SDR_ROLE_GSM:        cfg.freq = 947000000;  cfg.sample_rate = 1000000; break;
                    case SDR_ROLE_LTE:        cfg.freq = LTE_DEFAULT_FREQ; cfg.sample_rate = LTE_SAMPLE_RATE; break;
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
        fprintf(stderr, "sdr_manager: loaded %d receivers from %s\n", loaded, RECEIVERS_JSON_PATH);
    return loaded;
}
