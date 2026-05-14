// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sdr_receiver.h: Multi-SDR dynamic receiver management
//
// Supports a dynamic number of SDR receivers, each independently
// assigned to a decode or feed role with its own FIFO, converter,
// gain table, reader thread, and decoder state.
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#ifndef SDR_RECEIVER_H
#define SDR_RECEIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "sdr_backend.h"
#include <stdint.h>
#include <pthread.h>
#include "fifo.h"
#include "convert.h"

// ======================== Limits ========================

#define MAX_SDR_RECEIVERS   8

// ======================== Types ========================

// Forward declaration
struct sdr_receiver;

typedef enum {
    SDR_ROLE_NONE = 0,      // not assigned
    SDR_ROLE_ADSB,          // 1090 MHz ADS-B reception
    SDR_ROLE_FLARM,         // 868 MHz FLARM/OGN reception
    SDR_ROLE_ACARS,         // ~130 MHz ACARS decoding
    SDR_ROLE_VDL2,          // ~136.975 MHz VDL Mode 2 decoding
    SDR_ROLE_RADIOSONDE,    // ~403 MHz radiosonde decoding
    SDR_ROLE_POCSAG,        // ~466 MHz POCSAG pager decoding
    SDR_ROLE_GSM,           // ~935 MHz GSM broadcast channel decoder
    SDR_ROLE_LTE,           // ~800 MHz LTE cell scanner (PSS/SSS/MIB/SIB1)
    SDR_ROLE_IOT868,        // ~868 MHz ISM band IoT device monitor (OOK/FSK)
    SDR_ROLE_FANET,         // ~868.2 MHz FANET+ LoRa (SF7/BW250)
    SDR_ROLE_SARSAT         // ~406 MHz Cospas-Sarsat ELT/EPIRB/PLB beacon
} sdr_role_t;

// Decoder operations — plugin interface for each receiver role.
// init:    Create decoder state, called after RTL-SDR device is open.
// process: Called in the reader thread for each IQ buffer from rtlsdr_read_async.
// drain:   Called in the main thread to consume decoded output (dequeue from FIFO/queue).
// stop:    Cleanup decoder state, called before device close.
typedef struct {
    const char *name;
    bool  (*init)(struct sdr_receiver *rx);
    void  (*process)(struct sdr_receiver *rx, const uint8_t *iq, uint32_t len);
    void  (*drain)(struct sdr_receiver *rx);
    void  (*stop)(struct sdr_receiver *rx);
} decoder_ops_t;

// Get the decoder_ops for a given role (returns NULL for SDR_ROLE_NONE)
const decoder_ops_t *decoderOpsForRole(sdr_role_t role);

typedef enum {
    RX_STATE_IDLE = 0,      // created, not opened
    RX_STATE_OPEN,          // device opened, ready to start
    RX_STATE_RUNNING,       // reader thread active
    RX_STATE_STOPPING,      // stop requested
    RX_STATE_ERROR          // device error (e.g. USB disconnect)
} rx_state_t;

// Per-receiver FIFO context — mirrors the static globals of fifo.c
// so each receiver has its own independent magnitude buffer queue.
typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t  notempty_cond;
    pthread_cond_t  empty_cond;
    pthread_cond_t  free_cond;

    struct mag_buf *head;           // queued buffers awaiting demod
    struct mag_buf *tail;
    struct mag_buf *freelist;       // preallocated free buffers
    bool            halted;

    uint32_t        overlap_length;
    uint16_t       *overlap_buffer;
} rx_fifo_t;

// Per-receiver configuration (set before open, changeable at runtime for some fields)
typedef struct {
    char    serial[64];             // RTL-SDR serial number (empty = auto, "FILE" = virtual)
    sdr_role_t role;                // receiver role
    sdr_backend_type_t backend;     // backend library preference (auto/rtlsdr/sdrgg)
    int     freq;                   // center frequency in Hz
    float   gain;                   // gain in dB, or MODES_DEFAULT_GAIN / MODES_LEGACY_AUTO_GAIN
    int     ppm_error;              // oscillator correction in PPM
    bool    digital_agc;            // enable digital AGC
    int     direct_sampling;        // direct sampling mode (0=off)
    double  sample_rate;            // sample rate in Hz
    char    ifile_path[512];        // IQ file path for virtual device (empty = real SDR)
} rx_config_t;

// RTL-SDR device state (per-receiver, replaces the static RTLSDR struct)
typedef struct {
    void           *dev;            // rtlsdr_dev_t* (void* to avoid header dep outside ENABLE_RTLSDR)
    iq_convert_fn   converter;
    struct converter_state *converter_state;
    uint8_t        *bounce_buffer;
    int            *gains;          // sorted gain table
    int             gain_steps;
    int             current_gain;   // current gain step index
    int             tuner_type;     // RTLSDR_TUNER_* enum, -1 if unknown
} rx_rtlsdr_t;

// Main per-receiver structure
typedef struct sdr_receiver {
    int             id;             // receiver index 0..MAX_SDR_RECEIVERS-1
    rx_state_t      state;
    rx_config_t     config;
    const decoder_ops_t *decoder_ops; // decoder plugin for this role
    rx_rtlsdr_t     rtl;            // RTL-SDR device state (legacy, used when backend=rtlsdr)
    rx_fifo_t       fifo;           // per-receiver FIFO (ADSB/FLARM roles)

    // Backend abstraction layer
    const sdr_backend_ops_t *backend_ops; // resolved backend vtable
    sdr_device_t   *backend_dev;    // backend device handle (NULL = legacy rtl path)

    pthread_t       thread;         // reader thread
    bool            thread_started;

    // Callback state (replaces static locals in rtlsdrCallback)
    uint32_t        dropped;
    uint64_t        sample_counter;

    // CPU accounting
    pthread_mutex_t cpu_mutex;
    struct timespec cpu_accumulator;
    struct timespec cpu_start;

    // USB identity (filled after open)
    char            manufacturer[256];
    char            product[256];
    char            serial_actual[256];
    int             dev_index;      // rtlsdr device index used to open

    // Internal decoder state for the assigned role.
    void           *decoder_state;  // Opaque pointer to decoder

    // Auto-gain: IQ noise power sampling (updated atomically in reader thread)
    volatile uint64_t ag_iq_sum;    // sum of (I-128)^2 + (Q-128)^2
    volatile uint64_t ag_iq_count;  // number of IQ sample pairs accumulated

    // Pending retune request (set by decoder, applied in callback context)
    volatile uint32_t pending_freq; // 0 = no retune pending

    // USB error recovery
    uint32_t        usb_error_count;    // consecutive set_freq failures
    uint32_t        usb_error_total;    // total set_freq failures
} sdr_receiver_t;

// Global receiver manager state
typedef struct {
    sdr_receiver_t  receivers[MAX_SDR_RECEIVERS];
    int             count;          // number of configured receivers
    pthread_mutex_t lock;           // protects count and receiver state changes
} sdr_manager_t;

extern sdr_manager_t SdrManager;
extern int PocsagOutputEnabled;
extern int GsmOutputEnabled;
extern int LteOutputEnabled;
extern int IotOutputEnabled;
extern int FanetOutputEnabled;
extern int SarsatOutputEnabled;

// FANET ground tracking entry (exposed for API serialization)
typedef struct {
    uint32_t addr;
    double   latitude;
    double   longitude;
    uint8_t  ground_type;
    char     name[32];
    uint64_t last_seen;
    uint8_t  valid;
} fanet_ground_entry_t;

// Iterate over active ground tracking entries (thread-safe, <5min old)
void fanetGetGroundTracks(void (*cb)(const fanet_ground_entry_t *e, void *ctx), void *ctx);

// ======================== Manager API ========================

// Initialize the manager (call once at startup before any receiver ops)
void sdrManagerInit(void);

// Add a receiver with the given config. Returns receiver index (0..N-1) or -1 on error.
int sdrManagerAddReceiver(const rx_config_t *config);

// Update config of an existing receiver by index (role, freq, gain, ppm, sample_rate).
void sdrManagerUpdateConfig(int index, const rx_config_t *config);

// Remove a receiver by index. Stops and closes it first if needed. Returns true on success.
bool sdrManagerRemoveReceiver(int index);

// Open all configured receivers. Returns number successfully opened.
int sdrManagerOpenAll(void);

// Start reader threads on all open receivers. Returns number started.
int sdrManagerStartAll(void);

// Drain decoded data from all running receivers (call from main thread).
// Returns true if at least one receiver was drained.
bool sdrManagerDrainAll(void);

// Stop all running receivers.
void sdrManagerStopAll(void);

// Close all receivers and free resources.
void sdrManagerCloseAll(void);

// Shutdown the entire manager.
void sdrManagerShutdown(void);

// ======================== Per-receiver FIFO API ========================

// These mirror fifo.h but operate on a per-receiver FIFO context.
bool rxFifoCreate(rx_fifo_t *fifo, uint32_t buffer_count, uint32_t buffer_size, uint32_t overlap);
void rxFifoDestroy(rx_fifo_t *fifo);
void rxFifoDrain(rx_fifo_t *fifo);
void rxFifoHalt(rx_fifo_t *fifo);
struct mag_buf *rxFifoAcquire(rx_fifo_t *fifo, uint32_t timeout_ms);
void rxFifoEnqueue(rx_fifo_t *fifo, struct mag_buf *buf);
struct mag_buf *rxFifoDequeue(rx_fifo_t *fifo, uint32_t timeout_ms);
void rxFifoRelease(rx_fifo_t *fifo, struct mag_buf *buf);

// ======================== Per-receiver SDR ops ========================

// Open the RTL-SDR device for a specific receiver. Returns true on success.
bool rxOpen(sdr_receiver_t *rx);

// Start the reader thread for a receiver.
bool rxStart(sdr_receiver_t *rx);

// Stop the reader thread.
void rxStop(sdr_receiver_t *rx);

// Close the device and free RTL-SDR resources.
void rxClose(sdr_receiver_t *rx);

// Reconfigure a running receiver in-place (no device close/reopen).
bool rxReconfigure(sdr_receiver_t *rx, sdr_role_t new_role, double new_gain,
                   int new_ppm, uint32_t new_freq, double new_sample_rate);

// Gain control for a specific receiver
int rxGetGain(sdr_receiver_t *rx);
int rxGetMaxGain(sdr_receiver_t *rx);
double rxGetGainDb(sdr_receiver_t *rx, int step);
int rxSetGain(sdr_receiver_t *rx, int step);

// ======================== Utility ========================

// Enumerate connected RTL-SDR devices, return count.
// Fills serials[] array with serial numbers (caller provides array of char[64]).
int sdrEnumerateDevices(char serials[][64], int max_devices);

// Find receiver by serial number. Returns index or -1.
int sdrManagerFindBySerial(const char *serial);

// Get receiver pointer by index (NULL if invalid).
sdr_receiver_t *sdrManagerGetReceiver(int index);

// ======================== Persistence ========================

// Save current receiver config to /etc/dump1090-gg/receivers.json
bool sdrManagerSave(void);

// Load receiver config from /etc/dump1090-gg/receivers.json (skips serials already added via CLI)
// Returns number of receivers loaded.
int sdrManagerLoad(void);

// Return string name for role
const char *sdrRoleName(sdr_role_t role);

// Return string name for state
const char *rxStateName(rx_state_t state);

// Parse a --receiver CLI argument: "serial:role[:gain=X][:ppm=Y][:agc]"
// Fills config on success, returns true.
bool rxParseConfig(const char *arg, rx_config_t *config);

// ======================== External decoder process ========================

// Check if a role uses an internal decoder (ACARS/VDL2/Radiosonde) rather than FIFO-based demod
bool rxRoleIsDecoder(sdr_role_t role);

// Create internal decoder for a receiver (called during rxOpen for decoder roles)
bool rxDecoderCreate(sdr_receiver_t *rx);

// Destroy internal decoder (called during rxClose)
void rxDecoderDestroy(sdr_receiver_t *rx);

// Process IQ samples through the internal decoder (called from RTL-SDR callback)
void rxDecoderProcess(sdr_receiver_t *rx, const uint8_t *iq_data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif // SDR_RECEIVER_H
