// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sdr_backend.c: Unified SDR backend — global state and rtlsdr implementation
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#include "sdr_backend.h"
#include <stdint.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>

#ifndef MAX_SDR_RECEIVERS
#define MAX_SDR_RECEIVERS 8
#endif

#ifdef ENABLE_RTLSDR
#include <rtl-sdr.h>
#include "gg_format.h"
#include <string>
#endif

// ======================== Forward declarations ========================

#ifdef ENABLE_SDRGG
extern const sdr_backend_ops_t sdrgg_backend_ops;
#endif

// ======================== rtlsdr backend implementation ========================

#ifdef ENABLE_RTLSDR

static int32_t rtl_enumerate(sdr_dev_info_t *devs, int32_t max_devs)
{
    int32_t count = rtlsdr_get_device_count();
    int32_t found = 0;
    for (int32_t i = 0; i < count && found < max_devs; i++) {
        char manuf[256] = {0}, prod[256] = {0}, serial[256] = {0};
        if (rtlsdr_get_device_usb_strings(i, manuf, prod, serial) == 0) {
            devs[found].index = i;
            snprintf(devs[found].serial, sizeof(devs[found].serial), "%.63s", serial);
            snprintf(devs[found].manufacturer, sizeof(devs[found].manufacturer), "%.255s", manuf);
            snprintf(devs[found].product, sizeof(devs[found].product), "%.255s", prod);
            devs[found].tuner = SDR_TUNER_UNKNOWN;  // filled after open
            devs[found].backend = SDR_BACKEND_RTLSDR;
            found++;
        }
    }
    return found;
}

static sdr_device_t *rtl_open_by_index(int32_t index)
{
    rtlsdr_dev_t *dev = NULL;
    if (rtlsdr_open(&dev, (uint32_t)index) < 0)
        return NULL;

    sdr_device_t *sdev = static_cast<sdr_device_t*>(calloc(1, sizeof(sdr_device_t)));
    if (!sdev) { rtlsdr_close(dev); return NULL; }

    sdev->handle = dev;
    sdev->ctx = NULL;

    // Map tuner type
    int32_t tt = rtlsdr_get_tuner_type(dev);
    switch (tt) {
        case RTLSDR_TUNER_R820T:  sdev->tuner_type = SDR_TUNER_R820T;  break;
        case RTLSDR_TUNER_R828D:  sdev->tuner_type = SDR_TUNER_R820T2; break;
        case RTLSDR_TUNER_FC0012: sdev->tuner_type = SDR_TUNER_FC0012; break;
        case RTLSDR_TUNER_FC0013: sdev->tuner_type = SDR_TUNER_FC0013; break;
        case RTLSDR_TUNER_FC2580: sdev->tuner_type = SDR_TUNER_FC2580; break;
        case RTLSDR_TUNER_E4000:  sdev->tuner_type = SDR_TUNER_E4000;  break;
        default:                  sdev->tuner_type = SDR_TUNER_UNKNOWN; break;
    }

    return sdev;
}

static sdr_device_t *rtl_open_by_serial(const char *serial)
{
    int32_t count = rtlsdr_get_device_count();
    for (int32_t i = 0; i < count; i++) {
        char dev_serial[256] = {0};
        if (rtlsdr_get_device_usb_strings(i, NULL, NULL, dev_serial) == 0) {
            if (strcmp(dev_serial, serial) == 0) {
                return rtl_open_by_index(i);
            }
        }
    }
    return NULL;
}

static void rtl_close(sdr_device_t *dev)
{
    if (dev && dev->handle) {
        rtlsdr_close((rtlsdr_dev_t *)dev->handle);
        dev->handle = NULL;
    }
    free(dev);
}

static int32_t rtl_set_frequency(sdr_device_t *dev, uint32_t freq_hz)
{
    int32_t rc = rtlsdr_set_center_freq((rtlsdr_dev_t *)dev->handle, freq_hz);
    if (rc == 0) dev->current_freq = freq_hz;
    return rc;
}

static uint32_t rtl_get_frequency(sdr_device_t *dev)
{
    return rtlsdr_get_center_freq((rtlsdr_dev_t *)dev->handle);
}

static int32_t rtl_set_sample_rate(sdr_device_t *dev, uint32_t rate_hz)
{
    int32_t rc = rtlsdr_set_sample_rate((rtlsdr_dev_t *)dev->handle, rate_hz);
    if (rc == 0) dev->current_rate = rate_hz;
    return rc;
}

static int32_t rtl_set_gain_mode(sdr_device_t *dev, int32_t manual)
{
    return rtlsdr_set_tuner_gain_mode((rtlsdr_dev_t *)dev->handle, manual);
}

static int32_t rtl_set_gain(sdr_device_t *dev, int32_t gain_tenth_db)
{
    int32_t rc = rtlsdr_set_tuner_gain((rtlsdr_dev_t *)dev->handle, gain_tenth_db);
    if (rc == 0) dev->current_gain = gain_tenth_db;
    return rc;
}

static int32_t rtl_get_gain(sdr_device_t *dev)
{
    return rtlsdr_get_tuner_gain((rtlsdr_dev_t *)dev->handle);
}

static int32_t rtl_set_freq_correction(sdr_device_t *dev, int32_t ppm)
{
    return rtlsdr_set_freq_correction((rtlsdr_dev_t *)dev->handle, ppm);
}

static int32_t rtl_set_agc(sdr_device_t *dev, int32_t enable)
{
    return rtlsdr_set_agc_mode((rtlsdr_dev_t *)dev->handle, enable);
}

static int32_t rtl_set_direct_sampling(sdr_device_t *dev, int32_t mode)
{
    return rtlsdr_set_direct_sampling((rtlsdr_dev_t *)dev->handle, mode);
}

static int32_t rtl_reset_buffer(sdr_device_t *dev)
{
    return rtlsdr_reset_buffer((rtlsdr_dev_t *)dev->handle);
}

static int32_t rtl_get_tuner_gains(sdr_device_t *dev, int32_t *gains, int32_t max_count)
{
    int32_t count = rtlsdr_get_tuner_gains((rtlsdr_dev_t *)dev->handle, NULL);
    if (count <= 0) return 0;
    if (!gains) return count;
    if (count > max_count) count = max_count;
    int32_t *tmp = static_cast<int32_t*>(malloc((size_t)count * sizeof(int32_t)));
    if (!tmp) return 0;
    rtlsdr_get_tuner_gains((rtlsdr_dev_t *)dev->handle, tmp);
    memcpy(gains, tmp, (size_t)count * sizeof(int32_t));
    free(tmp);
    return count;
}

static int32_t rtl_get_tuner_type(sdr_device_t *dev)
{
    return (int32_t)dev->tuner_type;
}

static int32_t rtl_read_async(sdr_device_t *dev, sdr_async_cb_t cb, void *ctx,
                          uint32_t buf_count, uint32_t buf_size)
{
    return rtlsdr_read_async((rtlsdr_dev_t *)dev->handle,
                             (rtlsdr_read_async_cb_t)cb, ctx, buf_count, buf_size);
}

static int32_t rtl_cancel_async(sdr_device_t *dev)
{
    return rtlsdr_cancel_async((rtlsdr_dev_t *)dev->handle);
}

static int32_t rtl_read_sync(sdr_device_t *dev, uint8_t *buf, uint32_t len, int32_t *n_read)
{
    return rtlsdr_read_sync((rtlsdr_dev_t *)dev->handle, buf, len, n_read);
}

static const sdr_backend_ops_t rtlsdr_backend_ops = {
    .name               = "rtlsdr",
    .type               = SDR_BACKEND_RTLSDR,
    .enumerate          = rtl_enumerate,
    .open_by_index      = rtl_open_by_index,
    .open_by_serial     = rtl_open_by_serial,
    .close              = rtl_close,
    .set_frequency      = rtl_set_frequency,
    .get_frequency      = rtl_get_frequency,
    .set_sample_rate    = rtl_set_sample_rate,
    .set_gain_mode      = rtl_set_gain_mode,
    .set_gain           = rtl_set_gain,
    .get_gain           = rtl_get_gain,
    .set_freq_correction = rtl_set_freq_correction,
    .set_agc            = rtl_set_agc,
    .set_direct_sampling = rtl_set_direct_sampling,
    .reset_buffer       = rtl_reset_buffer,
    .get_tuner_gains    = rtl_get_tuner_gains,
    .get_tuner_type     = rtl_get_tuner_type,
    .read_async         = rtl_read_async,
    .cancel_async       = rtl_cancel_async,
    .read_sync          = rtl_read_sync,
    // Extended ops not supported by librtlsdr
    .get_tuner_caps     = NULL,
    .set_bandwidth      = NULL,
    .set_lna_gain       = NULL,
    .set_mixer_gain     = NULL,
    .set_vga_gain       = NULL,
    .read_tuner_reg     = NULL,
    .write_tuner_reg    = NULL,
};

#endif // ENABLE_RTLSDR

// ======================== Global backend state ========================

static int32_t backends_available = 0;
static int32_t backends_initialized = 0;

void sdrBackendInit(void)
{
    if (backends_initialized) return;
    backends_initialized = 1;
    backends_available = 0;

#ifdef ENABLE_RTLSDR
    backends_available |= (1 << SDR_BACKEND_RTLSDR);
#endif
#ifdef ENABLE_SDRGG
    backends_available |= (1 << SDR_BACKEND_SDRGG);
#endif

    gg::eprint("sdr_backend: available backends:");
#ifdef ENABLE_RTLSDR
    gg::eprint(" rtlsdr");
#endif
#ifdef ENABLE_SDRGG
    gg::eprint(" sdrgg");
#endif
    gg::eprint("\n");
}

int32_t sdrBackendAvailable(void)
{
    return backends_available;
}

const sdr_backend_ops_t *sdrBackendGet(sdr_backend_type_t type)
{
    switch (type) {
#ifdef ENABLE_RTLSDR
        case SDR_BACKEND_RTLSDR: return &rtlsdr_backend_ops;
#endif
#ifdef ENABLE_SDRGG
        case SDR_BACKEND_SDRGG:  return &sdrgg_backend_ops;
#endif
        default: return NULL;
    }
}

const sdr_backend_ops_t *sdrBackendGetDefault(void)
{
    // Prefer sdrgg if available (full chip capabilities)
#ifdef ENABLE_SDRGG
    return &sdrgg_backend_ops;
#elif defined(ENABLE_RTLSDR)
    return &rtlsdr_backend_ops;
#else
    return NULL;
#endif
}

const sdr_backend_ops_t *sdrBackendResolve(sdr_backend_type_t type)
{
    if (type == SDR_BACKEND_AUTO)
        return sdrBackendGetDefault();

    const sdr_backend_ops_t *ops = sdrBackendGet(type);
    if (ops) return ops;

    // Requested backend not available, use default
    fprintf(stderr, "sdr_backend: requested backend '%s' not available, using default\n",
            sdrBackendName(type));
    return sdrBackendGetDefault();
}

int32_t sdrBackendEnumerateAll(sdr_dev_info_t *devs, int32_t max_devs)
{
    int32_t total = 0;

#ifdef ENABLE_SDRGG
    // sdrgg enumerate first (preferred)
    int32_t n = sdrgg_backend_ops.enumerate(devs + total, max_devs - total);
    total += n;
#endif

#ifdef ENABLE_RTLSDR
    // rtlsdr enumerate — skip devices already found by sdrgg (by serial)
    sdr_dev_info_t rtl_devs[MAX_SDR_RECEIVERS];
    int32_t rtl_count = rtlsdr_backend_ops.enumerate(rtl_devs, MAX_SDR_RECEIVERS);
    for (int32_t i = 0; i < rtl_count && total < max_devs; i++) {
        // Check if this serial is already in the list from sdrgg
        int32_t duplicate = 0;
        for (int32_t j = 0; j < total; j++) {
            if (strcmp(devs[j].serial, rtl_devs[i].serial) == 0) {
                duplicate = 1;
                break;
            }
        }
        if (!duplicate) {
            devs[total] = rtl_devs[i];
            total++;
        }
    }
#endif

    return total;
}

sdr_backend_type_t sdrBackendParse(const char *name)
{
    if (!strcasecmp(name, "rtlsdr") || !strcasecmp(name, "librtlsdr")) return SDR_BACKEND_RTLSDR;
    if (!strcasecmp(name, "sdrgg") || !strcasecmp(name, "libsdrgg")) return SDR_BACKEND_SDRGG;
    // "auto" or empty or unknown → resolve to concrete default
#ifdef ENABLE_SDRGG
    return SDR_BACKEND_SDRGG;
#else
    return SDR_BACKEND_RTLSDR;
#endif
}

const char *sdrBackendName(sdr_backend_type_t type)
{
    switch (type) {
        case SDR_BACKEND_RTLSDR: return "rtlsdr";
        case SDR_BACKEND_SDRGG:  return "sdrgg";
        default:
#ifdef ENABLE_SDRGG
            return "sdrgg";
#else
            return "rtlsdr";
#endif
    }
}

// Needed by sdrBackendEnumerateAll — fallback if sdr_receiver.h not included
// (intentionally removed — moved to top of file)
