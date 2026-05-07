// Part of dump1090-gg-light
//
// lte_decode.h: LTE cell scanner — PSS/SSS detection, MIB decoding, SIB1 extraction.
//
// Scans for LTE cells using the Primary and Secondary Synchronization Signals
// to identify Physical Cell Identity (PCI), then decodes MIB from PBCH and
// attempts SIB1 decoding for operator identity (MCC/MNC), TAC, and Cell ID.
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#ifndef LTE_DECODE_H
#define LTE_DECODE_H

#include <stdint.h>
#include <stdbool.h>

// ======================== Constants ========================

#define LTE_SAMPLE_RATE     1920000   // 1.92 MS/s (covers 1.4 MHz LTE BW)
#define LTE_FFT_SIZE        128       // FFT size for 1.92 MS/s (15 kHz subcarrier spacing)
#define LTE_CP_NORMAL_0     10        // CP length for symbol 0 (normal CP, 1.92 MS/s)
#define LTE_CP_NORMAL       9         // CP length for symbols 1-6 (normal CP, 1.92 MS/s)
#define LTE_SYMBOLS_PER_SLOT 7        // Normal CP
#define LTE_SLOTS_PER_FRAME 20        // 10 ms frame = 20 slots
#define LTE_SAMPLES_PER_SLOT 960      // 0.5 ms * 1.92 MS/s
#define LTE_SAMPLES_PER_FRAME 19200   // 10 ms * 1.92 MS/s

#define LTE_PSS_LEN         62        // PSS occupies 62 subcarriers
#define LTE_SSS_LEN         62        // SSS occupies 62 subcarriers
#define LTE_PBCH_RES        72        // PBCH occupies 72 subcarriers (6 RBs)

#define LTE_MAX_CELLS       32        // Max cells tracked simultaneously
#define LTE_N_ID_2_COUNT    3         // PSS sequences (Zadoff-Chu roots 25, 29, 34)
#define LTE_N_ID_1_COUNT    168       // SSS sequences

// LTE frequency bands common in Europe
#define LTE_BAND_20_DL_LOW  791000000   // Band 20 (800 MHz) DL start
#define LTE_BAND_20_DL_HIGH 821000000
#define LTE_BAND_3_DL_LOW   1805000000  // Band 3 (1800 MHz) DL start
#define LTE_BAND_3_DL_HIGH  1880000000
#define LTE_BAND_7_DL_LOW   2620000000  // Band 7 (2600 MHz) DL start
#define LTE_BAND_7_DL_HIGH  2690000000

// Default: Band 20 center (good for RTL-SDR, lower frequency = better sensitivity)
#define LTE_DEFAULT_FREQ    806000000   // 806 MHz (EARFCN 6300, Vodafone DE)

// Band 20 frequency hopping table (Germany / Europe)
#define LTE_BAND20_FREQ_1   796000000   // O2/Telefónica
#define LTE_BAND20_FREQ_2   806000000   // Vodafone
#define LTE_BAND20_FREQ_3   816000000   // Telekom
#define LTE_HOP_FREQS       3           // Number of hop frequencies
#define LTE_HOP_DWELL_MS    2000        // Dwell time per frequency (ms)

// ======================== Types ========================

// Sync state
typedef enum {
    LTE_SYNC_NONE = 0,    // No synchronization
    LTE_SYNC_PSS,         // PSS found, timing established
    LTE_SYNC_SSS,         // SSS found, PCI known
    LTE_SYNC_MIB,         // MIB decoded from PBCH
    LTE_SYNC_SIB1         // SIB1 decoded (full cell info)
} lte_sync_state_t;

// MIB (Master Information Block) — decoded from PBCH
typedef struct {
    uint8_t  dl_bandwidth;       // 0=1.4MHz,1=3MHz,2=5MHz,3=10MHz,4=15MHz,5=20MHz (n_rb_dl)
    uint8_t  phich_duration;     // 0=normal, 1=extended
    uint8_t  phich_resources;    // 0=1/6, 1=1/2, 2=1, 3=2
    uint16_t sfn;                // System Frame Number (0-1023)
    bool     valid;
} lte_mib_t;

// SIB1 info (partial — what we can extract)
typedef struct {
    uint16_t mcc;                // Mobile Country Code
    uint16_t mnc;                // Mobile Network Code
    uint16_t tac;                // Tracking Area Code
    uint32_t cell_id;            // 28-bit Cell Identity
    bool     cell_barred;        // Cell barred for access
    bool     intra_freq_resel;   // Intra-freq reselection allowed
    int8_t   q_rxlevmin;         // Minimum RX level (dBm * 2)
    uint8_t  si_window_length;   // SI scheduling window (ms)
    bool     valid;
} lte_sib1_t;

// LTE alert (ETWS/CMAS/EAB) — decoded from SIB10-14
typedef struct {
    enum { LTE_ALERT_NONE = 0, LTE_ALERT_ETWS, LTE_ALERT_CMAS, LTE_ALERT_EAB } type;
    uint16_t message_id;         // Warning message ID
    uint16_t serial_number;      // Serial number (scope + code)
    char     text[256];          // Decoded alert text (UTF-8)
    char     category[64];       // Human-readable category
    uint64_t timestamp;          // Unix timestamp when received
    bool     active;             // Currently active
} lte_alert_t;

#define LTE_MAX_ALERTS  4        // Max concurrent alerts per cell

// Detected cell information
typedef struct {
    uint16_t        pci;             // Physical Cell Identity (0-503)
    uint8_t         n_id_2;          // PSS index (0-2)
    uint8_t         n_id_1;          // SSS group (0-167)
    double          freq_hz;         // Center frequency
    uint32_t        earfcn;          // E-UTRA ARFCN
    float           rsrp_dbfs;       // Reference Signal Received Power (dBFS)
    float           rsrq_db;         // Reference Signal Received Quality
    float           snr_db;          // Signal-to-Noise Ratio
    float           freq_offset_hz;  // Measured frequency offset
    lte_mib_t       mib;             // Decoded MIB
    lte_sib1_t      sib1;            // Decoded SIB1 (partial)
    lte_alert_t     alerts[LTE_MAX_ALERTS]; // Active alerts (ETWS/CMAS/EAB)
    int             alert_count;     // Number of active alerts
    lte_sync_state_t sync_state;     // Current sync level

    // Statistics
    uint32_t        pss_count;       // PSS detections
    uint32_t        sss_count;       // SSS detections
    uint32_t        mib_count;       // Successful MIB decodes
    uint32_t        sib1_count;      // Successful SIB1 decodes
    uint32_t        crc_errors;      // CRC failures
} lte_cell_info_t;

// Decoder statistics
typedef struct {
    uint64_t samples_processed;
    uint32_t pss_detected;       // Total PSS correlations above threshold
    uint32_t sss_decoded;        // Successful SSS decodes
    uint32_t mib_decoded;        // Successful MIB decodes
    uint32_t sib1_decoded;       // Successful SIB1 decodes
    uint32_t crc_errors;         // Total CRC failures
    double   freq_offset_hz;     // Current frequency offset estimate
} lte_stats_t;

// Cell detection callback
typedef void (*lte_cell_callback_t)(const lte_cell_info_t *cell, void *ctx);

// Decoder configuration
typedef struct {
    double              center_freq;    // SDR center frequency in Hz
    double              sample_rate;    // SDR sample rate in Hz
    lte_cell_callback_t callback;       // Called when cell info updates
    void               *callback_ctx;
    bool                hop_enabled;    // Enable frequency hopping (Band 20)
} lte_config_t;

// Opaque decoder state
struct lte_state;

// ======================== API ========================

struct lte_state *lte_create(const lte_config_t *cfg);
void lte_destroy(struct lte_state *st);
void lte_process(struct lte_state *st, const uint8_t *iq_data, unsigned len);
void lte_get_stats(const struct lte_state *st, lte_stats_t *out);
int  lte_get_cells(const struct lte_state *st, lte_cell_info_t *out, int max_cells);
lte_sync_state_t lte_get_best_sync(const struct lte_state *st);

// Frequency hopping: returns non-zero if decoder wants to retune
// Caller should retune SDR to returned frequency and call lte_set_freq() after
double lte_get_hop_freq(struct lte_state *st);
void   lte_set_freq(struct lte_state *st, double freq_hz);

// ======================== Utility ========================

// Convert EARFCN to frequency (Hz)
double lte_earfcn_to_freq(uint32_t earfcn);

// Convert frequency to EARFCN (returns 0 if not in known band)
uint32_t lte_freq_to_earfcn(double freq_hz);

// Get band name string
const char *lte_band_name(double freq_hz);

// Get DL bandwidth string from MIB dl_bandwidth field
const char *lte_bw_string(uint8_t dl_bw);

// Get number of RBs from dl_bandwidth
int lte_bw_to_nrb(uint8_t dl_bw);

#endif // LTE_DECODE_H
