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
#include <libusb-1.0/libusb.h>

#include "sdr_backend.h"

#include "sdrgg.h"
#include "gg_format.h"

/* ================================================================
 * USB HUB PORT POWER CYCLE (for FC0012 recovery)
 * ================================================================
 *
 * When the FC0012 tuner enters a stuck state where its I2C bus is
 * dead (all register reads return 0xFF, writes silently ignored),
 * the ONLY recovery is a full 5V power cycle on the USB port.
 *
 * No software reset (GPIO pulse, demod reset, USB device reset,
 * authorized toggle, unbind/rebind) can restore it — the FC0012
 * silicon needs its power rail to go to 0V and back.
 *
 * This function sends USB hub class requests to cut and restore
 * power on a specific port, equivalent to physically unplugging
 * and replugging the dongle.
 *
 * Uses libusb directly (already linked via librtlsdr dependency).
 * ================================================================ */

static bool usb_hub_power_cycle(uint16_t vid, uint16_t pid, const char *serial)
{
    libusb_context *usb_ctx = NULL;
    if (libusb_init(&usb_ctx) != 0) return false;

    libusb_device **devlist = NULL;
    ssize_t cnt = libusb_get_device_list(usb_ctx, &devlist);
    if (cnt < 0) { libusb_exit(usb_ctx); return false; }

    // Find the target device by VID:PID and serial
    libusb_device *target = NULL;
    for (ssize_t i = 0; i < cnt; i++) {
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(devlist[i], &desc) != 0) continue;
        if (desc.idVendor != vid || desc.idProduct != pid) continue;

        libusb_device_handle *h = NULL;
        if (libusb_open(devlist[i], &h) != 0) continue;
        char sn[64] = {};
        if (desc.iSerialNumber)
            libusb_get_string_descriptor_ascii(h, desc.iSerialNumber, (uint8_t*)sn, sizeof(sn));
        libusb_close(h);

        if (strcmp(sn, serial) == 0) { target = devlist[i]; break; }
    }

    if (!target) {
        libusb_free_device_list(devlist, 1);
        libusb_exit(usb_ctx);
        return false;
    }

    // Get parent hub and port number
    libusb_device *hub = libusb_get_parent(target);
    uint8_t port = libusb_get_port_number(target);
    if (!hub || port == 0) {
        libusb_free_device_list(devlist, 1);
        libusb_exit(usb_ctx);
        return false;
    }

    // Open the hub
    libusb_device_handle *hub_h = NULL;
    if (libusb_open(hub, &hub_h) != 0) {
        libusb_free_device_list(devlist, 1);
        libusb_exit(usb_ctx);
        return false;
    }

    fprintf(stderr, "sdrgg: FC0012 USB power cycle: hub bus=%d dev=%d port=%d serial=%s\n",
            libusb_get_bus_number(hub), libusb_get_device_address(hub), port, serial);

    // USB Hub Class: CLEAR_FEATURE(PORT_POWER) — turns off 5V
    int rc = libusb_control_transfer(hub_h,
        0x23,   // bmRequestType: class | other | host-to-device
        0x01,   // bRequest: CLEAR_FEATURE
        0x0008, // wValue: PORT_POWER
        port,   // wIndex: port number
        NULL, 0, 1000);

    if (rc < 0) {
        fprintf(stderr, "sdrgg: USB power off failed: %s\n", libusb_error_name(rc));
        libusb_close(hub_h);
        libusb_free_device_list(devlist, 1);
        libusb_exit(usb_ctx);
        return false;
    }

    fprintf(stderr, "sdrgg: USB port %d powered OFF, waiting 3s...\n", port);
    { struct timespec ts = {3, 0}; nanosleep(&ts, nullptr); }

    // USB Hub Class: SET_FEATURE(PORT_POWER) — turns on 5V
    rc = libusb_control_transfer(hub_h,
        0x23,   // bmRequestType: class | other | host-to-device
        0x03,   // bRequest: SET_FEATURE
        0x0008, // wValue: PORT_POWER
        port,   // wIndex: port number
        NULL, 0, 1000);

    fprintf(stderr, "sdrgg: USB port %d powered ON (rc=%d), waiting 5s for re-enumeration...\n", port, rc);
    libusb_close(hub_h);
    libusb_free_device_list(devlist, 1);
    libusb_exit(usb_ctx);

    { struct timespec ts = {5, 0}; nanosleep(&ts, nullptr); }

    return (rc >= 0);
}
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

// ======================== Streaming adapter ========================
// With the new libsdrgg ring-buffer engine (v1.3.1+), data flows:
//   USB → libsdrgg reader thread → ring buffer → consumer thread → our callback
// The old double-ring (sdrgg → adapter ring → consumer loop) is eliminated.
// The consumer thread in libsdrgg calls our callback directly, so we just
// need a thin adapter to bridge the sdrgg callback signature to the dump1090
// sdr_async_cb_t signature.

struct stream_adapter {
    sdr_async_cb_t    user_cb;
    void             *user_ctx;
    sdr_device_t     *sdr_dev;
    volatile int32_t  stopping;
    int32_t           adapter_id;
    int32_t           sub_handle;    // libsdrgg subscriber handle (-1 if none)
    volatile uint32_t deliver_count;
};

// Called from libsdrgg consumer thread — runs in its own thread context,
// so heavy processing (demod, decode) is fine here.
static void sdrgg_stream_callback(sdrgg_dev_t * /*dev*/, const sdrgg_buffer_t *buf, void *user_ctx)
{
    auto *adapter = static_cast<stream_adapter *>(user_ctx);
    if (!adapter || adapter->stopping || !buf || !buf->data || buf->length == 0)
        return;

    adapter->deliver_count++;
    adapter->user_cb(buf->data, buf->length, adapter->user_ctx);
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

    /* ================================================================
     * FC0012 CRITICAL INIT SEQUENCE — DO NOT REMOVE OR REORDER
     * ================================================================
     *
     * The Fitipower FC0012 tuner has THREE known hardware issues that
     * must all be addressed at open time, or the dongle appears to work
     * (streaming OK, API calls return success) but is actually deaf:
     *
     * ISSUE 1 — GPIO7 RESET (dongle-specific, discovered by brute-force)
     *
     *   On this dongle (serial 00000103, RTL2838UHIDIR), the FC0012
     *   reset pin is connected to RTL2832U GPIO7 — NOT GPIO4 as in
     *   the standard documentation and librtlsdr code.
     *
     *   When the dongle enters a stuck state (I2C bus dead: all tuner
     *   register reads return 0xFF, chip ID reads as 0x00 instead of
     *   0xA1), pulsing GPIO7 low-then-high resets the FC0012 and
     *   restores I2C communication.
     *
     *   Without this pulse: no USB reset, no software reset, no register
     *   write, and no amount of close/reopen will recover the tuner.
     *   Only a physical USB disconnect/reconnect (power cycle) works.
     *
     *   The GPIO7 pulse is harmless on dongles where GPIO7 is not
     *   connected to anything — it just toggles an unused pin.
     *
     * ISSUE 2 — DUAL ADC CHANNEL ENABLE (demod page0:0x08 = 0xCD)
     *
     *   FC0012 is a Zero-IF tuner that needs both I and Q ADC channels
     *   active. The RTL2832U register page0:0x08 must be 0xCD to enable
     *   both channels. Without this write, only one ADC channel is active
     *   and gain/frequency changes have no effect on IQ data.
     *
     *   This register is normally written by rtlsdr_set_direct_sampling(0)
     *   but the sdrgg backend previously had that as a no-op.
     *
     * ISSUE 3 — GAIN LATCH BUG (tuner reg 0x13, librtlsdr PR#74)
     *
     *   The FC0012 silicon has a bug where the analog gain circuitry
     *   only latches a new value when transitioning from the minimum
     *   (Low Gain) state. Initializing reg 0x13 to 0x08 (Middle Gain)
     *   causes all subsequent gain writes to be silently ignored.
     *
     *   Fix: init reg 0x13 = 0x00 (Low Gain), and before every gain
     *   change, always set gain to minimum first, then desired value.
     *   (The min-first workaround is in rxSetGain in sdr_receiver.cpp)
     *
     *   Confirmed by: librtlsdr PR#74 (March 2024), rtl_433 PR#2417
     *   (March 2023), RTLSDR-Airband issue #145.
     *
     * ================================================================ */
    if (tt == SDRGG_TUNER_FC0012 || tt == SDRGG_TUNER_FC0013) {
        // Check if I2C to tuner is alive (chip ID should be 0xA1)
        uint8_t chip_id = 0;
        tuner::read_reg(dev, 0x00, &chip_id);

        if (chip_id != 0xA1) {
            // ISSUE 1: I2C bus stuck — tuner not responding.
            // Recovery: demod reset + I2C repeater enable + GPIO brute-force.
            // The FC0012 reset pin varies by dongle (GPIO3, GPIO4, or GPIO7).
            // We try all GPIOs with demod reset between rounds.
            // sdr::open() already ran fc0012::init() but it failed silently
            // because I2C was dead. After recovery we re-init manually.
            for (int round = 0; round < 3 && chip_id != 0xA1; round++) {
                // Demod soft reset
                demod::write(dev, 1, 0x01, 0x14);
                { struct timespec ts = {0, 100000000}; nanosleep(&ts, nullptr); }
                demod::write(dev, 1, 0x01, 0x10);
                { struct timespec ts = {0, 100000000}; nanosleep(&ts, nullptr); }
                // Enable I2C repeater
                uint8_t p1 = 0; demod::read(dev, 1, 0x01, &p1);
                demod::write(dev, 1, 0x01, p1 | 0x18);
                { struct timespec ts = {0, 50000000}; nanosleep(&ts, nullptr); }
                // Pulse each GPIO pin
                for (int pin = 0; pin < 8 && chip_id != 0xA1; pin++) {
                    uint8_t mask = 1 << pin;
                    demod::write(dev, 2, 0x0003, mask);
                    demod::write(dev, 2, 0x0002, mask);
                    demod::write(dev, 2, 0x0001, 0x00);
                    { struct timespec ts = {0, 100000000}; nanosleep(&ts, nullptr); }
                    demod::write(dev, 2, 0x0001, mask);
                    { struct timespec ts = {0, 100000000}; nanosleep(&ts, nullptr); }
                    tuner::read_reg(dev, 0x00, &chip_id);
                }
            }

            if (chip_id != 0xA1) {
                /* ============================================================
                 * LAST RESORT: USB hub port power cycle
                 * ============================================================
                 * GPIO brute-force failed to unblock the FC0012 I2C bus.
                 * The only remaining option is a full 5V power cycle on the
                 * USB port. This is equivalent to physically unplugging the
                 * dongle. We send USB hub class requests to cut and restore
                 * power on the port, then reopen the device.
                 *
                 * This works because the FC0012 silicon needs its power rail
                 * to go to 0V and back to fully reset internal state.
                 * ============================================================ */
                fprintf(stderr, "sdrgg: FC0012 GPIO recovery failed, attempting USB power cycle\n");

                // Get serial before closing
                sdrgg_devinfo_t di[8];
                int dc = sdrgg_enumerate(ctx, di, 8);
                char serial[64] = {};
                for (int i = 0; i < dc; i++) {
                    if (i == index) { strncpy(serial, di[i].serial, 63); break; }
                }

                // Close device before power cycle
                sdr::close(dev);
                dev = nullptr;

                if (serial[0] && usb_hub_power_cycle(0x0bda, 0x2838, serial)) {
                    fprintf(stderr, "sdrgg: USB power cycle done, reopening device\n");
                    dev = sdr::open(ctx, index);
                    if (dev) {
                        sdev->handle = dev;
                        tt = sdr::get_tuner_type(dev);
                        tuner::read_reg(dev, 0x00, &chip_id);
                        fprintf(stderr, "sdrgg: after power cycle: chip_id=0x%02X tuner=%d\n",
                                chip_id, tt);
                    }
                }

                if (!dev) {
                    fprintf(stderr, "sdrgg: FC0012 recovery failed completely\n");
                    delete sdev;
                    return nullptr;
                }
            }

            if (chip_id == 0xA1) {
                fprintf(stderr, "sdrgg: FC0012 I2C recovered, re-running init\n");
                // Re-run FC0012 init (the one during sdr::open failed on dead I2C)
                static const uint8_t init_regs[] = {
                    0x05,0x10,0x00,0x00,0x0F,0x00,0x20,0xFF,
                    0x6E,0xB8,0x82,0xFE,0x02,0x00,0x00,0x00,
                    0x00,0x1F,0x00,0x00,0x04
                };
                for (int i = 0; i < 21; i++)
                    tuner::write_reg(dev, 0x01 + i, init_regs[i]);
                // VCO calibration
                tuner::write_reg(dev, 0x0E, 0x80);
                { struct timespec ts = {0, 10000000}; nanosleep(&ts, nullptr); }
                tuner::write_reg(dev, 0x0E, 0x00);
                { struct timespec ts = {0, 10000000}; nanosleep(&ts, nullptr); }
            }
        }

        // ISSUE 2: Enable both I+Q ADC channels
        demod::write(dev, 0, 0x08, 0xCD);

        // ISSUE 3: Init gain register to Low Gain (unlocks the latch)
        tuner::write_reg(dev, 0x13, 0x00);
    }

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

        pthread_mutex_lock(&g_stream_mutex);
        sdr::close(static_cast<sdrgg_dev_t *>(dev->handle));
        pthread_mutex_unlock(&g_stream_mutex);
        dev->handle = nullptr;
    }
    if (dev && dev->ctx) {
        auto *adapter = static_cast<stream_adapter *>(dev->ctx);
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

static int32_t gg_set_direct_sampling(sdr_device_t *dev, int32_t mode)
{
    if (mode != 0) return 0;
    // FC0012 Zero-IF tuners need demod page0:0x08 = 0xCD.
    // ONLY write for FC0012/FC0013 — R820T has 0x4D and must NOT be changed.
    if (dev->tuner_type == SDR_TUNER_FC0012 || dev->tuner_type == SDR_TUNER_FC0013) {
        sdrgg_dev_t *h = static_cast<sdrgg_dev_t *>(dev->handle);
        if (!h) return -1;
        return demod::write(h, 0, 0x08, 0xCD);
    }
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
    auto *adapter = new (std::nothrow) stream_adapter{};
    if (!adapter) return -1;
    adapter->user_cb = cb;
    adapter->user_ctx = ctx;
    adapter->sdr_dev = dev;
    adapter->stopping = 0;
    static int32_t next_adapter_id = 0;
    adapter->adapter_id = next_adapter_id++;
    adapter->sub_handle = -1;
    adapter->deliver_count = 0;
    dev->ctx = adapter;

    sdrgg_stream_cfg_t cfg = {};
    cfg.buf_count = buf_count ? buf_count : 4;
    cfg.buf_size = buf_size ? buf_size : 262144;

    /* start_stream initializes the ring buffer, starts the reader thread,
     * and subscribes our callback as consumer[0]. The consumer thread in
     * libsdrgg calls sdrgg_stream_callback directly — no intermediate
     * ring buffer needed on our side. */
    pthread_mutex_lock(&g_stream_mutex);
    int32_t rc = sdr::start_stream(static_cast<sdrgg_dev_t *>(dev->handle),
                                   &cfg, sdrgg_stream_callback, adapter);
    pthread_mutex_unlock(&g_stream_mutex);
    if (rc != SDRGG_OK) {
        gg::eprint("sdrgg-diag: adapter[%d] start_stream FAILED rc=%d\n", adapter->adapter_id, rc);
        dev->ctx = nullptr;
        delete adapter;
        return rc;
    }
    dev->async_running = 1;
    gg::eprint("sdrgg-diag: adapter[%d] streaming started (ring-engine)\n", adapter->adapter_id);

    /* FC0012 demod fixup: in multi-device setups, opening R820T devices
     * overwrites the FC0012's demod registers with Low-IF/single-ADC config.
     * Re-assert Zero-IF dual-ADC mode now that streaming has started and
     * all devices are initialized. */
    if (dev->tuner_type == SDR_TUNER_FC0012 || dev->tuner_type == SDR_TUNER_FC0013) {
        auto *gg_dev = static_cast<sdrgg_dev_t *>(dev->handle);
        demod::write(gg_dev, 1, 0xB1, 0x1B);   /* Zero-IF + DC cancel */
        demod::write(gg_dev, 0, 0x08, 0xCD);   /* Dual I+Q ADC */
        demod::write(gg_dev, 1, 0x15, 0x00);   /* No spectrum inversion */
        gg::eprint("sdrgg-diag: adapter[%d] FC0012 demod fixup applied (0xCD)\n", adapter->adapter_id);
    }

    /* Block this thread until async_running is cleared (by gg_cancel_async
     * or gg_close). The actual data delivery happens in libsdrgg's consumer
     * thread which calls sdrgg_stream_callback → adapter->user_cb.
     * This matches the contract of read_async: it blocks the caller's
     * reader thread and returns only when streaming is cancelled. */
    struct timespec ts_wait = { .tv_sec = 0, .tv_nsec = 50000000 }; // 50ms
    while (dev->handle && dev->async_running && !adapter->stopping) {
        if (!sdr::is_alive(static_cast<sdrgg_dev_t *>(dev->handle))) {
            gg::eprint("sdrgg-diag: adapter[%d] device disconnected, exiting read_async\n",
                       adapter->adapter_id);
            break;
        }
        nanosleep(&ts_wait, nullptr);
    }

    gg::eprint("sdrgg-diag: adapter[%d] exiting — deliver=%u\n",
            adapter->adapter_id, adapter->deliver_count);
    adapter->stopping = 1;

    pthread_mutex_lock(&g_stream_mutex);
    sdr::stop_stream(static_cast<sdrgg_dev_t *>(dev->handle));
    pthread_mutex_unlock(&g_stream_mutex);

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
