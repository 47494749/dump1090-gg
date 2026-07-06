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
    SDR_ROLE_SARSAT,        // ~406 MHz Cospas-Sarsat ELT/EPIRB/PLB beacon

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
    bool  (*drain)(struct sdr_receiver *rx);
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
    int32_t     freq;                   // center frequency in Hz
    float   gain;                   // gain in dB, or MODES_DEFAULT_GAIN / MODES_LEGACY_AUTO_GAIN
    int32_t     ppm_error;              // oscillator correction in PPM
    bool    digital_agc;            // enable digital AGC
    int32_t     direct_sampling;        // direct sampling mode (0=off)
    double  sample_rate;            // sample rate in Hz
    char    ifile_path[512];        // IQ file path for virtual device (empty = real SDR)
} rx_config_t;

// RTL-SDR device state (per-receiver, replaces the static RTLSDR struct)
typedef struct {
    void           *dev;            // rtlsdr_dev_t* (void* to avoid header dep outside ENABLE_RTLSDR)
    iq_convert_fn   converter;
    struct converter_state *converter_state;
    uint8_t        *bounce_buffer;
    int32_t            *gains;          // sorted gain table
    int32_t             gain_steps;
    int32_t             current_gain;   // current gain step index
    int32_t             tuner_type;     // RTLSDR_TUNER_* enum, -1 if unknown
} rx_rtlsdr_t;

// Main per-receiver structure
typedef struct sdr_receiver {
    int32_t             id;             // receiver index 0..MAX_SDR_RECEIVERS-1
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
    int32_t             dev_index;      // rtlsdr device index used to open

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

    // Waterfall IQ tap (written by stream callback, read by panel thread)
    volatile int32_t     wf_tap_active;     // nonzero = tapping enabled
    uint8_t         *wf_tap_buf;        // IQ ring buffer (allocated by panel)
    volatile uint32_t wf_tap_wr;        // write offset (updated atomically by callback)
    uint32_t         wf_tap_size;       // buffer size in bytes
} sdr_receiver_t;

// Global receiver manager state
typedef struct {
    sdr_receiver_t  receivers[MAX_SDR_RECEIVERS];
    int32_t             count;          // number of configured receivers
    pthread_mutex_t lock;           // protects count and receiver state changes
} sdr_manager_t;

extern sdr_manager_t SdrManager;
extern int32_t PocsagOutputEnabled;
extern int32_t GsmOutputEnabled;
extern int32_t LteOutputEnabled;
extern int32_t IotOutputEnabled;
extern int32_t FanetOutputEnabled;
extern int32_t SarsatOutputEnabled;
extern int32_t DvbDriverWarning;  // 1 if dvb_usb_rtl28xxu kernel module detected at startup

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

// FANET name entry (type 2)
typedef struct {
    uint32_t addr;
    char     name[32];
    uint64_t last_seen;
    uint8_t  valid;
} fanet_name_entry_t;

// FANET message entry (type 3)
typedef struct {
    uint32_t addr;
    uint8_t  subtype;
    char     text[200];
    uint64_t last_seen;
    uint8_t  valid;
} fanet_msg_entry_t;

// FANET weather entry (type 4)
typedef struct {
    uint32_t addr;
    char     name[32];
    double   latitude;
    double   longitude;
    float    temperature;
    float    wind_speed;
    float    wind_gust;
    float    wind_heading;
    float    humidity;
    float    pressure;
    float    state_of_charge;
    uint8_t  has_pos;
    uint8_t  has_temp;
    uint8_t  has_wind;
    uint8_t  has_humidity;
    uint8_t  has_pressure;
    uint8_t  has_soc;
    uint64_t last_seen;
    uint8_t  valid;
} fanet_wx_entry_t;

// FANET thermal entry (type 9)
typedef struct {
    uint32_t addr;
    double   latitude;
    double   longitude;
    int32_t      altitude;
    float    climb;
    float    wind_speed;
    float    wind_heading;
    uint8_t  confidence;
    uint64_t last_seen;
    uint8_t  valid;
} fanet_thermal_entry_t;

// FANET ACK entry (type 0)
typedef struct {
    uint32_t src_addr;
    uint32_t dst_addr;
    uint64_t timestamp;
    uint8_t  valid;
} fanet_ack_entry_t;

// Iterate over active ground tracking entries (thread-safe, <5min old)
void fanetGetGroundTracks(void (*cb)(const fanet_ground_entry_t *e, void *ctx), void *ctx);
void fanetGetNames(void (*cb)(const fanet_name_entry_t *e, void *ctx), void *ctx);
void fanetGetMessages(void (*cb)(const fanet_msg_entry_t *e, void *ctx), void *ctx);
void fanetGetWeather(void (*cb)(const fanet_wx_entry_t *e, void *ctx), void *ctx);
void fanetGetThermals(void (*cb)(const fanet_thermal_entry_t *e, void *ctx), void *ctx);
void fanetGetAcks(void (*cb)(const fanet_ack_entry_t *e, void *ctx), void *ctx);

// ======================== Manager API ========================

// Initialize the manager (call once at startup before any receiver ops)
void sdrManagerInit(void);

// Add a receiver with the given config. Returns receiver index (0..N-1) or -1 on error.
int32_t sdrManagerAddReceiver(const rx_config_t *config);

// Update config of an existing receiver by index (role, freq, gain, ppm, sample_rate).
void sdrManagerUpdateConfig(int32_t index, const rx_config_t *config);

// Remove a receiver by index. Stops and closes it first if needed. Returns true on success.
bool sdrManagerRemoveReceiver(int32_t index);

// Open all configured receivers. Returns number successfully opened.
int32_t sdrManagerOpenAll(void);

// Start reader threads on all open receivers. Returns number started.
int32_t sdrManagerStartAll(void);

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
                   int32_t new_ppm, uint32_t new_freq, double new_sample_rate);

// Gain control for a specific receiver
int32_t rxGetGain(sdr_receiver_t *rx);
int32_t rxGetMaxGain(sdr_receiver_t *rx);
double rxGetGainDb(sdr_receiver_t *rx, int32_t step);
int32_t rxSetGain(sdr_receiver_t *rx, int32_t step);

// Diagnostic: periodic health check for receiver RF (call from backgroundTasks)
void rxDiagHealthCheck(void);

// ======================== Utility ========================

// Enumerate connected RTL-SDR devices, return count.
// Fills serials[] array with serial numbers (caller provides array of char[64]).
int32_t sdrEnumerateDevices(char serials[][64], int32_t max_devices);

// Find receiver by serial number. Returns index or -1.
int32_t sdrManagerFindBySerial(const char *serial);

// Get receiver pointer by index (NULL if invalid).
sdr_receiver_t *sdrManagerGetReceiver(int32_t index);

// ======================== Persistence ========================

// Save current receiver config to /etc/dump1090-gg/receivers.json
bool sdrManagerSave(void);

// Load receiver config from /etc/dump1090-gg/receivers.json (skips serials already added via CLI)
// Returns number of receivers loaded.
int32_t sdrManagerLoad(void);

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

// Collect decoder stats from all running receivers as a JSON string (malloc'd, caller frees)
char *rxGetDecoderStatsJSON(void);

// Snapshot of key decoder metrics for time-series history
typedef struct {
    uint32_t adsb_messages;
    uint32_t adsb_tracks;
    float    adsb_noise_dbfs;
    float    adsb_signal_dbfs;
    int16_t  adsb_gain_db;
    uint32_t flarm_detected;
    uint32_t flarm_decoded;
    uint32_t acars_decoded;
    uint32_t vdl2_decoded;
    uint32_t sonde_decoded;
    uint32_t pocsag_decoded;
    uint32_t gsm_bcch;
    uint32_t lte_mib;
    uint32_t fanet_decoded;
    uint32_t sarsat_frames;
} rx_stats_snapshot_t;

// Fill a stats snapshot from all running decoder receivers
void rxGetStatsSnapshot(rx_stats_snapshot_t *out);

#endif // SDR_RECEIVER_H
