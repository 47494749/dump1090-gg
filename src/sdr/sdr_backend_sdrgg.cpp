// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sdr_backend_sdrgg.cpp: libsdrgg backend implementation
//
// This is a C++ file that wraps the libsdrgg C++ namespace API
// and exports a C-linkage sdr_backend_ops_t vtable.
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>
#include <pthread.h>
#include <time.h>

#include "sdr_backend.h"

#include "sdrgg.h"
#include "gg_format.h"
#include <string>

// ======================== Global sdrgg context ========================

static sdrgg_ctx_t *g_sdrgg_ctx = nullptr;
static pthread_mutex_t g_ctx_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_stream_mutex = PTHREAD_MUTEX_INITIALIZER;

static sdrgg_ctx_t *get_ctx(void)
{
    if (!g_sdrgg_ctx) {
        pthread_mutex_lock(&g_ctx_mutex);
        if (!g_sdrgg_ctx) {
            g_sdrgg_ctx = sdr::create();
        }
        pthread_mutex_unlock(&g_ctx_mutex);
    }
    return g_sdrgg_ctx;
}

// ======================== Ring buffer for async data delivery ========================
// The sdrgg event_loop_thread calls our callback synchronously.
// We must return quickly to avoid blocking URB processing for other devices.
// Data is copied to a ring buffer and consumed by the reader thread.

#define SDRGG_RING_SLOTS 8
#define SDRGG_RING_BUFSIZE 262144

struct ring_slot {
    uint8_t data[SDRGG_RING_BUFSIZE];
    uint32_t len;
    volatile int32_t ready;  // 0=free, 1=filled
};

struct sdrgg_ring {
    struct ring_slot slots[SDRGG_RING_SLOTS];
    volatile int32_t write_idx;
    volatile int32_t read_idx;
};

// ======================== Streaming adapter ========================

struct stream_adapter {
    sdr_async_cb_t user_cb;
    void          *user_ctx;
    volatile int32_t   stopping;
    struct sdrgg_ring *ring;
    volatile uint32_t cb_count;      // debug: sdrgg callback invocations
    volatile uint32_t ring_full;     // debug: ring full drops
    volatile uint32_t deliver_count; // debug: deliveries to user callback
    int32_t               adapter_id;   // debug: adapter identifier
};

// Called from libsdrgg event_loop_thread — must return quickly!
static void sdrgg_stream_callback(sdrgg_dev_t * /*dev*/, const sdrgg_buffer_t *buf, void *user_ctx)
{
    auto *adapter = static_cast<stream_adapter *>(user_ctx);
    if (!adapter || adapter->stopping || !buf || !buf->data || buf->length == 0)
        return;

    adapter->cb_count++;

    struct sdrgg_ring *ring = adapter->ring;
    if (!ring) return;

    // Push to ring buffer (fast memcpy, no heavy processing here)
    int32_t idx = ring->write_idx;
    struct ring_slot *slot = &ring->slots[idx];
    if (slot->ready) { adapter->ring_full++; return; }  // ring full — drop this buffer
    uint32_t copy_len = buf->length < SDRGG_RING_BUFSIZE ? buf->length : SDRGG_RING_BUFSIZE;
    memcpy(slot->data, buf->data, copy_len);
    slot->len = copy_len;
    __atomic_store_n(&slot->ready, 1, __ATOMIC_RELEASE);
    ring->write_idx = (idx + 1) % SDRGG_RING_SLOTS;
}

// ======================== Backend operations ========================

static int32_t gg_enumerate(sdr_dev_info_t *devs, int32_t max_devs)
{
    sdrgg_ctx_t *ctx = get_ctx();
    if (!ctx) return 0;

    sdrgg_devinfo_t gg_devs[8];
    int32_t count = sdrgg_enumerate(ctx, gg_devs, max_devs < 8 ? max_devs : 8);

    for (int32_t i = 0; i < count && i < max_devs; i++) {
        devs[i].index = i;
        strncpy(devs[i].serial, gg_devs[i].serial, sizeof(devs[i].serial) - 1);
        devs[i].serial[sizeof(devs[i].serial) - 1] = '\0';
        snprintf(devs[i].manufacturer, sizeof(devs[i].manufacturer), "Realtek");
        snprintf(devs[i].product, sizeof(devs[i].product), "RTL2832U");
        devs[i].backend = SDR_BACKEND_SDRGG;

        // Map tuner type
        switch (gg_devs[i].tuner) {
            case SDRGG_TUNER_R820T:  devs[i].tuner = SDR_TUNER_R820T;  break;
            case SDRGG_TUNER_R820T2: devs[i].tuner = SDR_TUNER_R820T2; break;
            case SDRGG_TUNER_FC0012: devs[i].tuner = SDR_TUNER_FC0012; break;
            case SDRGG_TUNER_FC0013: devs[i].tuner = SDR_TUNER_FC0013; break;
            case SDRGG_TUNER_FC2580: devs[i].tuner = SDR_TUNER_FC2580; break;
            case SDRGG_TUNER_E4000:  devs[i].tuner = SDR_TUNER_E4000;  break;
            default:                 devs[i].tuner = SDR_TUNER_UNKNOWN; break;
        }
    }
    return count;
}

static sdr_device_t *gg_open_by_index(int32_t index)
{
    sdrgg_ctx_t *ctx = get_ctx();
    if (!ctx) return nullptr;

    sdrgg_dev_t *dev = sdr::open(ctx, (int32_t)index);
    if (!dev) return nullptr;

    auto *sdev = new (std::nothrow) sdr_device_t{};
    if (!sdev) { sdr::close(dev); return nullptr; }

    sdev->handle = dev;

    // Get tuner type
    sdrgg_tuner_type_t tt = sdr::get_tuner_type(dev);
    switch (tt) {
        case SDRGG_TUNER_R820T:  sdev->tuner_type = SDR_TUNER_R820T;  break;
        case SDRGG_TUNER_R820T2: sdev->tuner_type = SDR_TUNER_R820T2; break;
        case SDRGG_TUNER_FC0012: sdev->tuner_type = SDR_TUNER_FC0012; break;
        case SDRGG_TUNER_FC0013: sdev->tuner_type = SDR_TUNER_FC0013; break;
        case SDRGG_TUNER_FC2580: sdev->tuner_type = SDR_TUNER_FC2580; break;
        case SDRGG_TUNER_E4000:  sdev->tuner_type = SDR_TUNER_E4000;  break;
        default:                 sdev->tuner_type = SDR_TUNER_UNKNOWN; break;
    }

    sdev->supports_tuner_agc =
        (sdev->tuner_type == SDR_TUNER_R820T) ||
        (sdev->tuner_type == SDR_TUNER_R820T2);

    // Note: sdr::open() already calls rtl::init + tuner family startup
    // (r820t::init or fc0012::init) internally via the family contract.
    // No additional tuner init needed here.

    return sdev;
}

static sdr_device_t *gg_open_by_serial(const char *serial)
{
    sdrgg_ctx_t *ctx = get_ctx();
    if (!ctx) return nullptr;

    // Enumerate and find by serial
    sdrgg_devinfo_t devs[8];
    int32_t count = sdrgg_enumerate(ctx, devs, 8);
    for (int32_t i = 0; i < count; i++) {
        if (strcmp(devs[i].serial, serial) == 0) {
            return gg_open_by_index(i);
        }
    }
    return nullptr;
}

static void gg_close(sdr_device_t *dev)
{
    if (dev && dev->handle) {
        auto *adapter = static_cast<stream_adapter *>(dev->ctx);
        if (adapter) adapter->stopping = 1;
        dev->async_running = 0;

        // Let in-flight callbacks drain before closing
        struct timespec settle = { .tv_sec = 0, .tv_nsec = 200000000 }; // 200ms
        nanosleep(&settle, nullptr);

        pthread_mutex_lock(&g_stream_mutex);
        sdr::close(static_cast<sdrgg_dev_t *>(dev->handle));
        pthread_mutex_unlock(&g_stream_mutex);
        dev->handle = nullptr;
    }
    // Free adapter + ring after close ensures no more callbacks
    if (dev && dev->ctx) {
        auto *adapter = static_cast<stream_adapter *>(dev->ctx);
        delete adapter->ring;
        delete adapter;
        dev->ctx = nullptr;
    }
    delete dev;
}

static int32_t gg_set_frequency(sdr_device_t *dev, uint32_t freq_hz)
{
    uint32_t actual = 0;
    int32_t rc = sdr::set_frequency(static_cast<sdrgg_dev_t *>(dev->handle), freq_hz, &actual);
    gg::eprint("sdrgg: set_frequency(%u) rc=%d actual=%u\n", freq_hz, rc, actual);
    if (rc == SDRGG_OK) {
        dev->current_freq = actual;
        // Check PLL lock for R820T
        if (dev->tuner_type == SDR_TUNER_R820T || dev->tuner_type == SDR_TUNER_R820T2) {
            bool locked = false;
            r820t::pll_locked(static_cast<sdrgg_dev_t *>(dev->handle), &locked);
            gg::eprint("sdrgg: r820t pll_locked=%d\n", locked);
        }
    }
    return rc;
}

static uint32_t gg_get_frequency(sdr_device_t *dev)
{
    return dev->current_freq;
}

static int32_t gg_set_sample_rate(sdr_device_t *dev, uint32_t rate_hz)
{
    uint32_t actual = 0;
    int32_t rc = sdr::set_sample_rate(static_cast<sdrgg_dev_t *>(dev->handle), rate_hz, &actual);
    gg::eprint("sdrgg: set_sample_rate(%u) rc=%d actual=%u\n", rate_hz, rc, actual);
    fflush(stderr);
    if (rc == SDRGG_OK) {
        dev->current_rate = actual;
        // After DDC reset, retune to restore the IF signal path.
        // sdr::set_frequency internally calls r820t::set_freq with correct IF offset.
        if (dev->current_freq > 0) {
            uint32_t f_actual = 0;
            sdr::set_frequency(static_cast<sdrgg_dev_t *>(dev->handle),
                               dev->current_freq, &f_actual);
        }
    }
    return rc;
}

static int32_t gg_set_gain_mode(sdr_device_t *dev, int32_t manual)
{
    if (!manual) {
        // Auto gain: set SDRGG_GAIN_AUTO
        return sdr::set_gain(static_cast<sdrgg_dev_t *>(dev->handle), SDRGG_GAIN_AUTO);
    }
    return SDRGG_OK;  // manual mode is implicit when setting a specific gain
}

static int32_t gg_set_gain(sdr_device_t *dev, int32_t gain_tenth_db)
{
    int32_t rc = sdr::set_gain(static_cast<sdrgg_dev_t *>(dev->handle), (int32_t)gain_tenth_db);
    if (rc == SDRGG_OK) dev->current_gain = gain_tenth_db;
    return rc;
}

static int32_t gg_get_gain(sdr_device_t *dev)
{
    int32_t gain = 0;
    sdr::get_gain(static_cast<sdrgg_dev_t *>(dev->handle), &gain);
    return (int32_t)gain;
}

static int32_t gg_set_freq_correction(sdr_device_t *dev, int32_t ppm)
{
    return sdr::set_freq_correction(static_cast<sdrgg_dev_t *>(dev->handle), (int32_t)ppm);
}

static int32_t gg_set_agc(sdr_device_t *dev, int32_t enable)
{
    return sdr::set_digital_agc(static_cast<sdrgg_dev_t *>(dev->handle), enable != 0);
}

static int32_t gg_set_direct_sampling(sdr_device_t * /*dev*/, int32_t /*mode*/)
{
    // libsdrgg does not support direct sampling (RTL2832U limitation)
    return 0;
}

static int32_t gg_reset_buffer(sdr_device_t * /*dev*/)
{
    // libsdrgg uses zero-copy URBs — no explicit buffer reset needed
    return 0;
}

static int32_t gg_get_tuner_gains(sdr_device_t *dev, int32_t *gains, int32_t max_count)
{
    // For R820T, always use the well-known 29-step gain table (librtlsdr compatible)
    if (dev->tuner_type == SDR_TUNER_R820T || dev->tuner_type == SDR_TUNER_R820T2) {
        static const int32_t r820t_gains[] = {
            0, 9, 14, 27, 37, 77, 87, 125, 144, 157,
            166, 197, 207, 229, 254, 280, 297, 328, 338, 364,
            372, 386, 402, 421, 434, 439, 445, 480, 496
        };
        int32_t count = 29;
        if (!gains) return count;
        if (count > max_count) count = max_count;
        memcpy(gains, r820t_gains, (size_t)count * sizeof(int32_t));
        return count;
    }

    // FC0012: use the fc0012 gain table from libsdrgg
    if (dev->tuner_type == SDR_TUNER_FC0012) {
        const int16_t *fc_gains = nullptr;
        int32_t fc_count = 0;
        fc0012::get_gains(&fc_gains, &fc_count);
        if (!gains) return (int32_t)fc_count;
        if (fc_gains && fc_count > 0) {
            int32_t count = fc_count > max_count ? max_count : (int32_t)fc_count;
            for (int32_t i = 0; i < count; i++)
                gains[i] = (int32_t)fc_gains[i];
            return count;
        }
    }

    // Unknown tuner: try caps introspection
    const tuner_caps *caps = nullptr;
    int32_t rc = sdr::get_tuner_caps(static_cast<sdrgg_dev_t *>(dev->handle), &caps);
    if (rc == SDRGG_OK && caps) {
        int32_t min_g = caps->total_gain_min_tenth_db;
        int32_t max_g = caps->total_gain_max_tenth_db;
        if (!gains) return (int32_t)((max_g - min_g) / 10 + 1);
        int32_t count = 0;
        for (int32_t g = min_g; g <= max_g && count < max_count; g += 10)
            gains[count++] = (int32_t)g;
        return count;
    }

    return 0;
}

static int32_t gg_get_tuner_type(sdr_device_t *dev)
{
    return (int32_t)dev->tuner_type;
}

static int32_t gg_read_async(sdr_device_t *dev, sdr_async_cb_t cb, void *ctx,
                         uint32_t buf_count, uint32_t buf_size)
{
    // Allocate ring buffer (large — ~2MB per device)
    auto *ring = new (std::nothrow) sdrgg_ring{};
    if (!ring) return -1;

    // Allocate stream adapter
    auto *adapter = new (std::nothrow) stream_adapter;
    if (!adapter) { delete ring; return -1; }
    adapter->user_cb = cb;
    adapter->user_ctx = ctx;
    adapter->stopping = 0;
    static int32_t next_adapter_id = 0;
    adapter->cb_count = 0;
    adapter->ring_full = 0;
    adapter->deliver_count = 0;
    adapter->adapter_id = next_adapter_id++;
    adapter->ring = ring;
    dev->ctx = adapter;

    gg::eprint("sdrgg-diag: adapter[%d] created ring=%p\n", adapter->adapter_id, (void*)ring);

    sdrgg_stream_cfg_t cfg = {};
    cfg.buf_count = buf_count ? buf_count : 4;
    cfg.buf_size = buf_size ? buf_size : SDRGG_RING_BUFSIZE;

    pthread_mutex_lock(&g_stream_mutex);
    int32_t rc = sdr::start_stream(static_cast<sdrgg_dev_t *>(dev->handle),
                                   &cfg, sdrgg_stream_callback, adapter);
    pthread_mutex_unlock(&g_stream_mutex);
    if (rc != SDRGG_OK) {
        gg::eprint("sdrgg-diag: adapter[%d] start_stream FAILED rc=%d\n", adapter->adapter_id, rc);
        dev->ctx = nullptr;
        delete ring;
        delete adapter;
        return rc;
    }
    dev->async_running = 1;
    gg::eprint("sdrgg-diag: adapter[%d] streaming started, entering consumer loop\n", adapter->adapter_id);

    // Consume ring buffer data (reader thread context)
    // This replaces the old spin-wait: instead of sleeping, we poll the ring.
    struct timespec ts_poll = { .tv_sec = 0, .tv_nsec = 1000000 }; // 1ms poll interval
    uint32_t poll_empty = 0;
    while (dev->handle && dev->async_running) {
        int32_t idx = ring->read_idx;
        struct ring_slot *slot = &ring->slots[idx];
        if (__atomic_load_n(&slot->ready, __ATOMIC_ACQUIRE)) {
            // Deliver data to user callback (heavy processing happens HERE, not in event_loop)
            if (adapter->user_cb && !adapter->stopping) {
                adapter->user_cb(slot->data, slot->len, adapter->user_ctx);
                adapter->deliver_count++;
            }
            slot->ready = 0;
            ring->read_idx = (idx + 1) % SDRGG_RING_SLOTS;
            poll_empty = 0;
        } else {
            nanosleep(&ts_poll, nullptr);
            poll_empty++;
            // Log every 10s of no data
            if (poll_empty == 10000) {
                fprintf(stderr, "sdrgg-diag: adapter[%d] NO DATA for 10s! cb_count=%u ring_full=%u deliver=%u\n",
                        adapter->adapter_id, adapter->cb_count, adapter->ring_full, adapter->deliver_count);
            }
        }
    }

    fprintf(stderr, "sdrgg-diag: adapter[%d] consumer loop exited — cb=%u ring_full=%u deliver=%u handle=%p async=%d\n",
            adapter->adapter_id, adapter->cb_count, adapter->ring_full, adapter->deliver_count,
            (void*)dev->handle, dev->async_running);
    adapter->stopping = 1;

    // Stop the stream so callback is unregistered (allows future start_stream)
    pthread_mutex_lock(&g_stream_mutex);
    sdr::stop_stream(static_cast<sdrgg_dev_t *>(dev->handle));
    pthread_mutex_unlock(&g_stream_mutex);

    // Free ring and adapter
    delete adapter->ring;
    adapter->ring = nullptr;
    delete adapter;
    dev->ctx = nullptr;

    dev->async_running = 0;
    return 0;
}

static int32_t gg_cancel_async(sdr_device_t *dev)
{
    if (dev) {
        // Just signal the spin-wait to exit; gg_read_async handles stop_stream
        dev->async_running = 0;
    }
    return 0;
}

static int32_t gg_read_sync(sdr_device_t *dev, uint8_t *buf, uint32_t len, int32_t *n_read)
{
    int32_t rc = sdr::read_sync(static_cast<sdrgg_dev_t *>(dev->handle),
                                buf, len, 10000 /* 10s timeout for large reads */);
    if (rc >= 0 && n_read) *n_read = rc;
    return (rc >= 0) ? 0 : rc;
}

// ======================== Extended operations ========================

static int32_t gg_get_tuner_caps(sdr_device_t *dev, sdr_tuner_caps_t *out)
{
    const tuner_caps *caps = nullptr;
    int32_t rc = sdr::get_tuner_caps(static_cast<sdrgg_dev_t *>(dev->handle), &caps);
    if (rc != SDRGG_OK || !caps) return -1;

    *out = {};
    out->tuner = dev->tuner_type;
    out->chip_name = caps->chip_name;
    out->freq_min_hz = caps->freq_min_hz;
    out->freq_max_hz = caps->freq_max_hz;
    out->total_gain_min_tenth_db = caps->total_gain_min_tenth_db;
    out->total_gain_max_tenth_db = caps->total_gain_max_tenth_db;
    out->has_per_stage_gain = caps->num_gain_stages > 1;
    out->has_bandwidth_control = caps->num_bw_options > 0;
    out->has_pll_lock_detect = true;
    out->default_bw_khz = caps->default_bw_khz;

    // Copy gain stages (up to 4)
    out->num_gain_stages = caps->num_gain_stages > 4 ? 4 : caps->num_gain_stages;
    for (int32_t i = 0; i < out->num_gain_stages; i++) {
        out->stages[i].name = caps->gain_stages[i].name;
        out->stages[i].min_tenth_db = caps->gain_stages[i].min_tenth_db;
        out->stages[i].max_tenth_db = caps->gain_stages[i].max_tenth_db;
        out->stages[i].num_steps = caps->gain_stages[i].num_steps;
    }

    return 0;
}

static int32_t gg_set_bandwidth(sdr_device_t *dev, uint32_t bw_khz)
{
    // Only R820T supports bandwidth control through libsdrgg
    if (dev->tuner_type == SDR_TUNER_R820T || dev->tuner_type == SDR_TUNER_R820T2) {
        return r820t::set_bandwidth(static_cast<sdrgg_dev_t *>(dev->handle), bw_khz);
    }
    return -1;
}

static int32_t gg_set_lna_gain(sdr_device_t *dev, int32_t index)
{
    if (dev->tuner_type == SDR_TUNER_R820T || dev->tuner_type == SDR_TUNER_R820T2) {
        return r820t::set_lna_gain(static_cast<sdrgg_dev_t *>(dev->handle), (int32_t)index);
    }
    return -1;
}

static int32_t gg_set_mixer_gain(sdr_device_t *dev, int32_t index)
{
    if (dev->tuner_type == SDR_TUNER_R820T || dev->tuner_type == SDR_TUNER_R820T2) {
        return r820t::set_mixer_gain(static_cast<sdrgg_dev_t *>(dev->handle), (int32_t)index);
    }
    return -1;
}

static int32_t gg_set_vga_gain(sdr_device_t *dev, int32_t index)
{
    if (dev->tuner_type == SDR_TUNER_R820T || dev->tuner_type == SDR_TUNER_R820T2) {
        return r820t::set_vga_gain(static_cast<sdrgg_dev_t *>(dev->handle), (int32_t)index);
    }
    return -1;
}

static int32_t gg_read_tuner_reg(sdr_device_t *dev, uint8_t reg, uint8_t *val)
{
    return tuner::read_reg(static_cast<sdrgg_dev_t *>(dev->handle), reg, val);
}

static int32_t gg_write_tuner_reg(sdr_device_t *dev, uint8_t reg, uint8_t val)
{
    return tuner::write_reg(static_cast<sdrgg_dev_t *>(dev->handle), reg, val);
}

// ======================== Diagnostic operations ========================

static int32_t gg_read_demod_reg(sdr_device_t *dev, uint8_t block, uint16_t reg, uint8_t *val)
{
    return demod::read(static_cast<sdrgg_dev_t *>(dev->handle), block, reg, val);
}

static void gg_dump_all_registers(sdr_device_t *dev, FILE *out)
{
    sdrgg_dev_t *h = static_cast<sdrgg_dev_t *>(dev->handle);
    if (!h || !out) return;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    localtime_r(&ts.tv_sec, &tm);

    fprintf(out, "\n======= FULL REGISTER DUMP %02d:%02d:%02d =======\n",
            tm.tm_hour, tm.tm_min, tm.tm_sec);

    // Demod page 0 (block 0, reg = (page<<8)|offset)
    fprintf(out, "  DEMOD PAGE 0:\n   ");
    for (int i = 0; i < 32; i++) {
        uint8_t val = 0xFF;
        demod::read(h, 0, (0 << 8) | i, &val);
        fprintf(out, " %02X:%02X", i, val);
        if ((i & 7) == 7) fprintf(out, "\n   ");
    }

    // Demod page 1
    fprintf(out, "  DEMOD PAGE 1:\n   ");
    for (int i = 0; i < 32; i++) {
        uint8_t val = 0xFF;
        demod::read(h, 0, (1 << 8) | i, &val);
        fprintf(out, " %02X:%02X", i, val);
        if ((i & 7) == 7) fprintf(out, "\n   ");
    }

    // Page 1 extended critical regs
    fprintf(out, "  DEMOD P1 EXT:");
    static const uint8_t p1_ext[] = {0x93, 0x94, 0x9F, 0xA0, 0xA1, 0xA2, 0xB1};
    for (int i = 0; i < 7; i++) {
        uint8_t val = 0xFF;
        demod::read(h, 0, (1 << 8) | p1_ext[i], &val);
        fprintf(out, " %02X:%02X", p1_ext[i], val);
    }
    fputc('\n', out);

    // SYS block (block 2) key registers
    fprintf(out, "  SYS:");
    static const uint16_t sys_regs[] = {0x0001, 0x0002, 0x0003, 0x0004, 0x000B, 0x3000};
    for (int i = 0; i < 6; i++) {
        uint8_t val = 0xFF;
        demod::read(h, 2, sys_regs[i], &val);
        fprintf(out, " %04X:%02X", sys_regs[i], val);
    }
    fputc('\n', out);

    // USB block (block 1) key registers
    fprintf(out, "  USB:");
    static const uint16_t usb_regs[] = {0x2000, 0x2040, 0x2048, 0x2100, 0x2104};
    for (int i = 0; i < 5; i++) {
        uint8_t val = 0xFF;
        demod::read(h, 1, usb_regs[i], &val);
        fprintf(out, " %04X:%02X", usb_regs[i], val);
    }
    fputc('\n', out);

    fprintf(out, "=============================================\n");
    fflush(out);
}

// ======================== Exported vtable ========================

extern "C" const sdr_backend_ops_t sdrgg_backend_ops = {
    .name               = "sdrgg",
    .type               = SDR_BACKEND_SDRGG,
    .enumerate          = gg_enumerate,
    .open_by_index      = gg_open_by_index,
    .open_by_serial     = gg_open_by_serial,
    .close              = gg_close,
    .set_frequency      = gg_set_frequency,
    .get_frequency      = gg_get_frequency,
    .set_sample_rate    = gg_set_sample_rate,
    .set_gain_mode      = gg_set_gain_mode,
    .set_gain           = gg_set_gain,
    .get_gain           = gg_get_gain,
    .set_freq_correction = gg_set_freq_correction,
    .set_agc            = gg_set_agc,
    .set_direct_sampling = gg_set_direct_sampling,
    .reset_buffer       = gg_reset_buffer,
    .get_tuner_gains    = gg_get_tuner_gains,
    .get_tuner_type     = gg_get_tuner_type,
    .read_async         = gg_read_async,
    .cancel_async       = gg_cancel_async,
    .read_sync          = gg_read_sync,
    // Extended ops
    .get_tuner_caps     = gg_get_tuner_caps,
    .set_bandwidth      = gg_set_bandwidth,
    .set_lna_gain       = gg_set_lna_gain,
    .set_mixer_gain     = gg_set_mixer_gain,
    .set_vga_gain       = gg_set_vga_gain,
    .read_tuner_reg     = gg_read_tuner_reg,
    .write_tuner_reg    = gg_write_tuner_reg,
    .read_demod_reg     = gg_read_demod_reg,
    .dump_registers     = gg_dump_all_registers,
};
