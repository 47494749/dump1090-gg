// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// gsm_decode.h: GSM broadcast channel decoder — GMSK demodulation, FCCH/SCH
//               synchronization, Viterbi channel decoding, LAPDm L2 parsing,
//               System Information L3 parsing, CBCH cell broadcast.
//
// Decodes broadcast-only channels: FCCH, SCH, BCCH, CCCH (PCH/AGCH), CBCH.
// No traffic channel decoding.
//
// References:
//   3GPP TS 05.01 — Physical layer on the radio path
//   3GPP TS 05.02 — Multiplexing and multiple access on the radio path
//   3GPP TS 05.03 — Channel coding
//   3GPP TS 05.04 — Modulation
//   3GPP TS 04.06 — LAPDm (L2)
//   3GPP TS 04.08 — RR/MM/CC messages (L3)
//   3GPP TS 23.041 — Cell Broadcast Service
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#ifndef GSM_DECODE_H
#define GSM_DECODE_H

#include <stdint.h>
#include <stdbool.h>

// ======================== Constants (TS 05.02, TS 05.03) ========================

// Physical layer
#define GSM_SYMBOL_RATE         270833      // 13 MHz / 48 (symbols/sec)
#define GSM_FRAME_DURATION_US   4615        // 60/13 ms ≈ 4.615 ms per TDMA frame
#define GSM_TIMESLOTS           8           // timeslots per TDMA frame
#define GSM_BURST_BITS          156         // bits per timeslot (156.25 with guard)
#define GSM_SYMBOLS_PER_FRAME   (GSM_BURST_BITS * GSM_TIMESLOTS) // 1248 + 2 guard
#define GSM_SAMPLE_RATE         1000000     // 1 MHz SDR sample rate
#define GSM_IF_OFFSET           100000      // 100 kHz IF offset to avoid DC spike
#define GSM_SAMPLES_PER_SYMBOL  3.6923f     // 1000000 / 270833
#define GSM_SAMPLES_PER_FRAME   (int)(GSM_FRAME_DURATION_US * GSM_SAMPLE_RATE / 1000000) // ~4615

// Multiframe (51-multiframe for control channels, TS 05.02 section 6.3.1.3)
#define GSM_CTRL_MULTIFRAME     51          // frames per control multiframe
#define GSM_TRAFFIC_MULTIFRAME  26          // frames per traffic multiframe
#define GSM_SUPERFRAME          (GSM_CTRL_MULTIFRAME * GSM_TRAFFIC_MULTIFRAME) // 1326

// Burst sizes (TS 05.02, section 5.2)
#define GSM_NB_DATA_BITS        114         // Normal Burst: 57 + 57 data bits
#define GSM_NB_TRAIN_BITS       26          // Normal Burst: training sequence bits
#define GSM_NB_TAIL_BITS        3           // Normal Burst: tail bits each side
#define GSM_NB_STEALING_BITS    2           // Normal Burst: stealing flags (hl, hu)

#define GSM_SB_DATA_BITS        78          // Sync Burst: 39 + 39 data bits
#define GSM_SB_TRAIN_BITS       64          // Sync Burst: training sequence bits

#define GSM_FB_FIXED_BITS       142         // Freq Correction Burst: 142 fixed zero bits

// Channel coding (TS 05.03)
#define GSM_CONV_K              5           // Constraint length
#define GSM_CONV_STATES         16          // 2^(K-1) = 16 states
#define GSM_CONV_RATE_INV       2           // rate 1/2

// BCCH/CCCH/PCH/AGCH coding (TS 05.03 section 4.1)
#define GSM_BCCH_L2_BITS        184         // input: 23 octets
#define GSM_BCCH_PARITY_BITS    40          // Fire code CRC
#define GSM_BCCH_TAIL_BITS      4           // tail bits
#define GSM_BCCH_PRE_CONV       228         // 184 + 40 + 4
#define GSM_BCCH_CODED_BITS     456         // 228 * 2 (after conv)
#define GSM_BCCH_INTERLEAVE     4           // interleaved over 4 bursts

// SCH coding (TS 05.03 section 4.7)
#define GSM_SCH_INFO_BITS       25          // input: reduced frame number + BSIC
#define GSM_SCH_PARITY_BITS     10          // CRC bits
#define GSM_SCH_TAIL_BITS       4           // tail bits
#define GSM_SCH_PRE_CONV        39          // 25 + 10 + 4
#define GSM_SCH_CODED_BITS      78          // 39 * 2 (after conv)

// L2 (TS 04.06)
#define GSM_L2_FRAME_LEN        23          // L2 frame: 23 octets

// Maximum messages
#define GSM_MAX_NEIGHBOURS      32          // max neighbour cells in BA list
#define GSM_MAX_ARFCN           1024        // ARFCN range 0-1023
#define GSM_CB_PAGE_LEN         82          // Cell Broadcast page: 82 octets

// ======================== Training Sequences (TS 05.02) ========================

// Normal Burst training sequences (TSC 0-7), 26 bits each
// TS 05.02, Table 5.2.3
extern const int8_t gsm_nb_training[8][26];

// Synchronization Burst training sequence, 64 bits
// TS 05.02, section 5.2.5
extern const int8_t gsm_sb_training[64];

// ======================== Types ========================

// ARFCN frequency band
typedef enum {
    GSM_BAND_900 = 0,    // P-GSM 900: ARFCN 1-124
    GSM_BAND_E900,       // E-GSM 900: ARFCN 975-1023, 0
    GSM_BAND_1800,       // DCS 1800: ARFCN 512-885
    GSM_BAND_850,        // GSM 850: ARFCN 128-251
    GSM_BAND_1900,       // PCS 1900: ARFCN 512-810
    GSM_BAND_UNKNOWN
} gsm_band_t;

// Channel type on TS0
typedef enum {
    GSM_CHAN_UNKNOWN = 0,
    GSM_CHAN_FCCH,          // Frequency Correction Channel
    GSM_CHAN_SCH,           // Synchronization Channel
    GSM_CHAN_BCCH,          // Broadcast Control Channel
    GSM_CHAN_CCCH,          // Common Control Channel (PCH/AGCH)
    GSM_CHAN_SDCCH,         // Stand-alone Dedicated Control Channel
    GSM_CHAN_CBCH,          // Cell Broadcast Channel
    GSM_CHAN_IDLE           // Idle frame
} gsm_chan_type_t;

// Synchronization state
typedef enum {
    GSM_SYNC_NONE = 0,     // no sync
    GSM_SYNC_FCCH,         // FCCH tone detected, coarse timing known
    GSM_SYNC_SCH,          // SCH decoded, frame number and BSIC known
    GSM_SYNC_LOCKED        // stable lock, decoding BCCH
} gsm_sync_state_t;

// System Information Type 3 — the most informative SI message
typedef struct {
    bool     valid;
    uint16_t cell_id;               // Cell Identity
    uint16_t mcc;                   // Mobile Country Code (decimal, e.g. 222)
    uint16_t mnc;                   // Mobile Network Code (decimal, e.g. 10)
    uint16_t lac;                   // Location Area Code
    uint8_t  bsic;                  // Base Station Identity Code (NCC:3 + BCC:3)
    uint8_t  ncc;                   // Network Colour Code (BSIC >> 3)
    uint8_t  bcc;                   // Base Station Colour Code (BSIC & 7)
    // Control Channel Description
    uint8_t  bs_ag_blks_res;        // blocks reserved for AGCH
    uint8_t  ccch_conf;             // CCCH configuration
    uint8_t  bs_pa_mfrms;           // paging multiframes
    uint8_t  t3212;                 // periodic location update timer (decihours)
    // Cell Options
    uint8_t  radio_link_timeout;
    uint8_t  dtx;                   // DTX indicator
    bool     pwrc;                  // power control indicator
    // Cell Selection Parameters
    uint8_t  ms_txpwr_max_cch;
    uint8_t  rxlev_access_min;
    int8_t   cell_reselect_offset;
    uint8_t  cell_reselect_hysteresis;
    // RACH Control Parameters
    uint8_t  max_retrans;
    uint8_t  tx_integer;
    bool     cell_barred;
    bool     re_not_allowed;        // RE (call re-establishment) not allowed
    uint16_t ac_class;              // access class barring (16 bits)
} gsm_si3_t;

// System Information Type 1 — Cell Allocation
typedef struct {
    bool     valid;
    int      n_arfcn;               // number of ARFCNs in cell allocation
    uint16_t arfcn[GSM_MAX_ARFCN / 16]; // bitmap or list of ARFCNs
    uint16_t arfcn_list[64];        // decoded ARFCN list
} gsm_si1_t;

// System Information Type 2 — Neighbour Cell Description
typedef struct {
    bool     valid;
    int      n_neighbours;
    uint16_t neighbour_arfcn[GSM_MAX_NEIGHBOURS];
    uint8_t  ncc_permitted;         // bitmask of permitted NCCs
} gsm_si2_t;

// System Information Type 4 — CBCH description
typedef struct {
    bool     valid;
    uint16_t mcc;
    uint16_t mnc;
    uint16_t lac;
    bool     cbch_present;          // CBCH Channel Description present
    uint16_t cbch_arfcn;            // CBCH ARFCN (if hopping) or 0
    uint8_t  cbch_ts;               // CBCH timeslot
} gsm_si4_t;

// Cell Broadcast message
typedef struct {
    bool     valid;
    uint16_t serial_nr;             // serial number
    uint16_t msg_id;                // message identifier
    uint8_t  dcs;                   // data coding scheme
    uint8_t  page_param;            // page parameter
    uint8_t  total_pages;           // total pages
    uint8_t  page_nr;               // current page number (1-based)
    char     text[GSM_CB_PAGE_LEN + 1]; // decoded text (NUL-terminated)
    int      text_len;
} gsm_cb_msg_t;

// Paging message from PCH
typedef struct {
    bool     valid;
    uint8_t  paging_type;           // 1, 2, or 3
    uint8_t  n_identities;          // number of paged identities
    uint32_t tmsi[4];               // TMSI values (paging type 1/2)
    uint8_t  channel_needed[4];     // channel needed indicator
} gsm_paging_t;

// Cell information — aggregation of all SI data
typedef struct {
    gsm_si1_t si1;
    gsm_si2_t si2;
    gsm_si3_t si3;
    gsm_si4_t si4;

    // Derived
    uint16_t arfcn;                 // tuned ARFCN
    gsm_band_t band;               // frequency band
    double   freq_mhz;             // actual downlink frequency in MHz
    uint8_t  bsic;                  // from SCH

    // Timing
    uint32_t t1;                    // superframe number (T1)
    uint8_t  t2;                    // T2 (frame within 26-multiframe)
    uint8_t  t3p;                   // T3' (frame within 51-multiframe / 10)
    uint32_t fn;                    // absolute frame number

    // Stats
    uint32_t fcch_count;            // FCCH detections
    uint32_t sch_count;             // SCH decodings
    uint32_t bcch_count;            // BCCH messages decoded
    uint32_t ccch_count;            // CCCH messages decoded
    uint32_t cb_count;              // Cell Broadcast messages
} gsm_cell_info_t;

// Decoded L2 frame
typedef struct {
    uint8_t  data[GSM_L2_FRAME_LEN];
    uint8_t  sapi;                  // Service Access Point Identifier
    bool     cr;                    // Command/Response
    uint8_t  frame_type;            // U/S/I frame type
    uint8_t  length;                // information field length
    bool     more;                  // M bit (more data)
} gsm_l2_frame_t;

// Burst buffer (decoded soft bits from one timeslot)
typedef struct {
    float    soft_bits[GSM_BURST_BITS]; // soft decisions (-1.0 to +1.0)
    int      hard_bits[GSM_BURST_BITS]; // hard decisions (0 or 1)
    gsm_chan_type_t chan_type;
    uint32_t fn;                        // TDMA frame number
    int      ts;                        // timeslot (0 for BCCH carrier)
    float    snr;                       // estimated SNR
    int      tsc;                       // detected training sequence code
} gsm_burst_t;

// Message callback
typedef void (*gsm_message_callback_t)(const gsm_cell_info_t *cell, const char *msg_type,
                                       const uint8_t *l3_data, int l3_len, void *ctx);

// Cell Broadcast callback
typedef void (*gsm_cb_callback_t)(const gsm_cell_info_t *cell, const gsm_cb_msg_t *cb, void *ctx);

// Decoder configuration
typedef struct {
    double   center_freq;           // tuned center frequency (ARFCN freq - IF offset)
    double   arfcn_freq;            // actual ARFCN frequency (before IF offset)
    double   sample_rate;           // sample rate (should be ~1 MHz)
    int      tsc;                   // expected TSC (0-7), or -1 for auto-detect
    gsm_message_callback_t msg_cb;  // L3 message callback
    gsm_cb_callback_t      cb_cb;   // Cell Broadcast callback
    void    *callback_ctx;          // opaque context for callbacks
} gsm_config_t;

// Decoder statistics
typedef struct {
    uint64_t samples_processed;
    uint64_t fcch_detected;
    uint64_t sch_decoded;
    uint64_t sch_failed;
    uint64_t bcch_decoded;
    uint64_t bcch_failed;
    uint64_t ccch_decoded;
    uint64_t cb_decoded;
    double   freq_offset_hz;        // estimated frequency offset
} gsm_stats_t;

// Opaque decoder state
struct gsm_state;

// ======================== API ========================

// Create a GSM broadcast decoder with the given configuration.
// Returns NULL on failure.
struct gsm_state *gsm_create(const gsm_config_t *cfg);

// Destroy a GSM decoder and free resources.
void gsm_destroy(struct gsm_state *st);

// Process a block of IQ samples (interleaved uint8_t I,Q pairs).
void gsm_process(struct gsm_state *st, const uint8_t *iq_data, unsigned len);

// Get current cell information (aggregated from decoded SI messages).
void gsm_get_cell_info(const struct gsm_state *st, gsm_cell_info_t *out);

// Get decoder statistics.
void gsm_get_stats(const struct gsm_state *st, gsm_stats_t *out);

// Get current synchronization state.
gsm_sync_state_t gsm_get_sync_state(const struct gsm_state *st);

// ======================== Test / Internal API ========================
// Exposed for unit testing only.

// Convolutional encoder (rate 1/2, K=5, TS 05.03)
// input: 'n' bits, output: 2*n bits (interleaved c0,c1)
void gsm_conv_encode(const uint8_t *input, int n, uint8_t *output);

// Viterbi decoder (rate 1/2, K=5)
// input: 2*n soft bits (float, positive=1, negative=0), output: n hard bits
// Returns number of bit errors (path metric).
int gsm_viterbi_decode(const float *soft_input, int n_input_bits, uint8_t *output);

// Fire code encoder for BCCH/CCCH (TS 05.03, section 4.1)
// input: 184 bits, output: 40 parity bits
void gsm_fire_encode(const uint8_t *data, int n_bits, uint8_t *parity);

// Fire code check: returns true if CRC is valid
bool gsm_fire_check(const uint8_t *data_with_parity, int total_bits);

// BCCH interleaver (TS 05.03, section 4.1)
// coded_bits: 456 bits in, burst_bits[4][114]: bits per burst out
void gsm_bcch_interleave(const uint8_t *coded_bits, uint8_t burst_bits[4][GSM_NB_DATA_BITS]);

// BCCH deinterleaver
// burst_soft[4][114]: soft bits per burst in, coded_soft[456]: soft bits out
void gsm_bcch_deinterleave(const float burst_soft[4][GSM_NB_DATA_BITS], float *coded_soft);

// SCH encoder (TS 05.03, section 4.7)
// input: 25 bits, output: 78 coded bits
void gsm_sch_encode(const uint8_t *info, uint8_t *coded);

// SCH decoder
// input: 78 soft bits, output: 25 info bits
// Returns true if CRC is valid.
bool gsm_sch_decode(const float *soft_input, uint8_t *info_out);

// Decode SCH info bits into BSIC and frame number (TS 05.02, section 3.3.2.2.1)
void gsm_sch_parse(const uint8_t *info, uint8_t *bsic, uint32_t *fn);

// Encode BSIC + frame number into SCH info bits
void gsm_sch_build(uint8_t bsic, uint32_t fn, uint8_t *info);

// Parse L2 LAPDm frame (TS 04.06)
bool gsm_l2_parse(const uint8_t *data, int len, gsm_l2_frame_t *frame);

// Parse L3 System Information messages (TS 04.08)
bool gsm_parse_si1(const uint8_t *l3, int len, gsm_si1_t *out);
bool gsm_parse_si2(const uint8_t *l3, int len, gsm_si2_t *out);
bool gsm_parse_si3(const uint8_t *l3, int len, gsm_si3_t *out);
bool gsm_parse_si4(const uint8_t *l3, int len, gsm_si4_t *out);

// Parse Cell Broadcast message (TS 23.041)
bool gsm_parse_cb(const uint8_t *data, int len, gsm_cb_msg_t *out);

// Parse Paging message (TS 04.08)
bool gsm_parse_paging(const uint8_t *l3, int len, gsm_paging_t *out);

// ARFCN to downlink frequency (MHz)
double gsm_arfcn_to_freq(uint16_t arfcn, gsm_band_t band);

// Frequency to ARFCN (returns 0xFFFF if not found)
uint16_t gsm_freq_to_arfcn(double freq_mhz, gsm_band_t *band);

// Get channel type for frame number within 51-multiframe (TS 05.02, Table 5)
gsm_chan_type_t gsm_get_channel_type(int fn_mod51);

// FCCH detection in frequency buffer
int detect_fcch(const float *freq_buf, int n_freq,
                float samples_per_symbol, float carrier_rps,
                double *freq_offset_out);

#endif // GSM_DECODE_H
