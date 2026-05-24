// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sdr_backend.h: Unified SDR backend abstraction layer
//
// Provides a transparent API layer between the application and the
// underlying SDR library (librtlsdr or libsdrgg).  Each device can
// independently use either backend.  If only one library is present
// at compile time, it is used by default.
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#ifndef SDR_BACKEND_H
#define SDR_BACKEND_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// ======================== Backend selection ========================

typedef enum {
    SDR_BACKEND_AUTO = 0,    // legacy/internal: resolves to sdrgg (or rtlsdr if sdrgg unavailable)
    SDR_BACKEND_RTLSDR,     // librtlsdr
    SDR_BACKEND_SDRGG       // libsdrgg (preferred)
} sdr_backend_type_t;

// ======================== Tuner type (unified) ========================

typedef enum {
    SDR_TUNER_UNKNOWN = 0,
    SDR_TUNER_R820T,
    SDR_TUNER_R820T2,
    SDR_TUNER_FC0012,
    SDR_TUNER_FC0013,
    SDR_TUNER_FC2580,
    SDR_TUNER_E4000
} sdr_tuner_type_t;

// ======================== Device info ========================

typedef struct {
    int32_t     index;              // device index for open
    char    serial[64];
    char    manufacturer[256];
    char    product[256];
    sdr_tuner_type_t tuner;
    sdr_backend_type_t backend; // which backend found this device
} sdr_dev_info_t;

// ======================== Gain stage info (extended, sdrgg only) ========================

typedef struct {
    const char *name;           // "LNA", "Mixer", "VGA"
    int32_t min_tenth_db;
    int32_t max_tenth_db;
    uint8_t num_steps;
} sdr_gain_stage_t;

// ======================== Tuner capabilities (extended) ========================

typedef struct {
    sdr_tuner_type_t tuner;
    const char *chip_name;
    uint32_t freq_min_hz;
    uint32_t freq_max_hz;
    int32_t  total_gain_min_tenth_db;
    int32_t  total_gain_max_tenth_db;
    uint8_t  num_gain_stages;
    sdr_gain_stage_t stages[4]; // up to 4 gain stages
    bool     has_per_stage_gain;
    bool     has_bandwidth_control;
    bool     has_pll_lock_detect;
    uint32_t default_bw_khz;
} sdr_tuner_caps_t;

// ======================== Async callback type ========================

// Same signature as rtlsdr for drop-in compatibility
typedef void (*sdr_async_cb_t)(uint8_t *buf, uint32_t len, void *ctx);

// ======================== Device handle ========================

typedef struct sdr_device sdr_device_t;

// ======================== Backend operations vtable ========================

typedef struct sdr_backend_ops {
    const char *name;           // "rtlsdr" or "sdrgg"
    sdr_backend_type_t type;

    // Enumeration
    int32_t  (*enumerate)(sdr_dev_info_t *devs, int32_t max_devs);

    // Lifecycle
    sdr_device_t *(*open_by_index)(int32_t index);
    sdr_device_t *(*open_by_serial)(const char *serial);
    void (*close)(sdr_device_t *dev);

    // Configuration
    int32_t  (*set_frequency)(sdr_device_t *dev, uint32_t freq_hz);
    uint32_t (*get_frequency)(sdr_device_t *dev);
    int32_t  (*set_sample_rate)(sdr_device_t *dev, uint32_t rate_hz);
    int32_t  (*set_gain_mode)(sdr_device_t *dev, int32_t manual);   // 0=auto, 1=manual
    int32_t  (*set_gain)(sdr_device_t *dev, int32_t gain_tenth_db);
    int32_t  (*get_gain)(sdr_device_t *dev);                    // returns gain in tenth dB
    int32_t  (*set_freq_correction)(sdr_device_t *dev, int32_t ppm);
    int32_t  (*set_agc)(sdr_device_t *dev, int32_t enable);
    int32_t  (*set_direct_sampling)(sdr_device_t *dev, int32_t mode);
    int32_t  (*reset_buffer)(sdr_device_t *dev);

    // Gain table
    int32_t  (*get_tuner_gains)(sdr_device_t *dev, int32_t *gains, int32_t max_count);
    int32_t  (*get_tuner_type)(sdr_device_t *dev);

    // Streaming
    int32_t  (*read_async)(sdr_device_t *dev, sdr_async_cb_t cb, void *ctx,
                       uint32_t buf_count, uint32_t buf_size);
    int32_t  (*cancel_async)(sdr_device_t *dev);
    int32_t  (*read_sync)(sdr_device_t *dev, uint8_t *buf, uint32_t len, int32_t *n_read);

    // Extended capabilities (may be NULL if not supported)
    int32_t  (*get_tuner_caps)(sdr_device_t *dev, sdr_tuner_caps_t *caps);
    int32_t  (*set_bandwidth)(sdr_device_t *dev, uint32_t bw_khz);
    int32_t  (*set_lna_gain)(sdr_device_t *dev, int32_t index);
    int32_t  (*set_mixer_gain)(sdr_device_t *dev, int32_t index);
    int32_t  (*set_vga_gain)(sdr_device_t *dev, int32_t index);
    int32_t  (*read_tuner_reg)(sdr_device_t *dev, uint8_t reg, uint8_t *val);
    int32_t  (*write_tuner_reg)(sdr_device_t *dev, uint8_t reg, uint8_t val);
} sdr_backend_ops_t;

// ======================== Device handle structure ========================

struct sdr_device {
    const sdr_backend_ops_t *ops;   // backend vtable
    void *handle;                   // backend-specific device handle
    void *ctx;                      // backend context (sdrgg_ctx_t* or NULL)
    uint32_t current_freq;          // cached frequency
    uint32_t current_rate;          // cached sample rate
    int32_t      current_gain;          // cached gain (tenth dB)
    sdr_tuner_type_t tuner_type;    // cached tuner type
    volatile int32_t async_running;     // nonzero while read_async should stay blocked
};

// ======================== Global API ========================

// Initialize backend subsystem. Call once at startup.
// Detects which libraries are available.
void sdrBackendInit(void);

// Get the available backends (bitmask)
// Returns combination of (1 << SDR_BACKEND_RTLSDR) | (1 << SDR_BACKEND_SDRGG)
int32_t sdrBackendAvailable(void);

// Get backend ops by type. Returns NULL if not available.
const sdr_backend_ops_t *sdrBackendGet(sdr_backend_type_t type);

// Get the default backend (auto-detect: prefer sdrgg if available).
const sdr_backend_ops_t *sdrBackendGetDefault(void);

// Resolve backend for a device based on preference.
// If type is AUTO, resolves to the best available.
// If requested backend is not available, falls back to what's present.
const sdr_backend_ops_t *sdrBackendResolve(sdr_backend_type_t type);

// Enumerate devices across all available backends.
// Returns total number of devices found.
int32_t sdrBackendEnumerateAll(sdr_dev_info_t *devs, int32_t max_devs);

// Parse backend name string ("rtlsdr", "sdrgg", "auto")
sdr_backend_type_t sdrBackendParse(const char *name);

// Get backend name string
const char *sdrBackendName(sdr_backend_type_t type);

// ======================== Convenience dispatch API ========================
// These call through the device's ops vtable transparently.
// Use these instead of accessing ops-> directly from application code.

static inline int32_t sdr_set_frequency(sdr_device_t *dev, uint32_t freq_hz)
{ return dev->ops->set_frequency(dev, freq_hz); }

static inline uint32_t sdr_get_frequency(sdr_device_t *dev)
{ return dev->ops->get_frequency(dev); }

static inline int32_t sdr_set_sample_rate(sdr_device_t *dev, uint32_t rate_hz)
{ return dev->ops->set_sample_rate(dev, rate_hz); }

static inline int32_t sdr_set_gain_mode(sdr_device_t *dev, int32_t manual)
{ return dev->ops->set_gain_mode(dev, manual); }

static inline int32_t sdr_set_gain(sdr_device_t *dev, int32_t gain_tenth_db)
{ return dev->ops->set_gain(dev, gain_tenth_db); }

static inline int32_t sdr_get_gain(sdr_device_t *dev)
{ return dev->ops->get_gain(dev); }

static inline int32_t sdr_set_freq_correction(sdr_device_t *dev, int32_t ppm)
{ return dev->ops->set_freq_correction(dev, ppm); }

static inline int32_t sdr_set_agc(sdr_device_t *dev, int32_t enable)
{ return dev->ops->set_agc(dev, enable); }

static inline int32_t sdr_reset_buffer(sdr_device_t *dev)
{ return dev->ops->reset_buffer(dev); }

static inline int32_t sdr_get_tuner_gains(sdr_device_t *dev, int32_t *gains, int32_t max)
{ return dev->ops->get_tuner_gains(dev, gains, max); }

static inline int32_t sdr_get_tuner_type(sdr_device_t *dev)
{ return dev->ops->get_tuner_type(dev); }

static inline int32_t sdr_read_async(sdr_device_t *dev, sdr_async_cb_t cb, void *ctx,
                                 uint32_t buf_count, uint32_t buf_size)
{ return dev->ops->read_async(dev, cb, ctx, buf_count, buf_size); }

static inline int32_t sdr_cancel_async(sdr_device_t *dev)
{ return dev->ops->cancel_async(dev); }

static inline int32_t sdr_read_sync(sdr_device_t *dev, uint8_t *buf, uint32_t len, int32_t *n_read)
{ return dev->ops->read_sync(dev, buf, len, n_read); }

// Extended ops — return -1 if not supported by backend
static inline int32_t sdr_get_tuner_caps(sdr_device_t *dev, sdr_tuner_caps_t *caps)
{ return dev->ops->get_tuner_caps ? dev->ops->get_tuner_caps(dev, caps) : -1; }

static inline int32_t sdr_set_bandwidth(sdr_device_t *dev, uint32_t bw_khz)
{ return dev->ops->set_bandwidth ? dev->ops->set_bandwidth(dev, bw_khz) : -1; }

static inline int32_t sdr_set_lna_gain(sdr_device_t *dev, int32_t index)
{ return dev->ops->set_lna_gain ? dev->ops->set_lna_gain(dev, index) : -1; }

static inline int32_t sdr_set_mixer_gain(sdr_device_t *dev, int32_t index)
{ return dev->ops->set_mixer_gain ? dev->ops->set_mixer_gain(dev, index) : -1; }

static inline int32_t sdr_set_vga_gain(sdr_device_t *dev, int32_t index)
{ return dev->ops->set_vga_gain ? dev->ops->set_vga_gain(dev, index) : -1; }

static inline int32_t sdr_read_tuner_reg(sdr_device_t *dev, uint8_t reg, uint8_t *val)
{ return dev->ops->read_tuner_reg ? dev->ops->read_tuner_reg(dev, reg, val) : -1; }

static inline int32_t sdr_write_tuner_reg(sdr_device_t *dev, uint8_t reg, uint8_t val)
{ return dev->ops->write_tuner_reg ? dev->ops->write_tuner_reg(dev, reg, val) : -1; }

#ifdef __cplusplus
}
#endif

#endif // SDR_BACKEND_H
