// Part of dump1090-gg-light
//
// lte_sib.h: LTE System Information Block decoding — SIB1 through SIB14.
//
// Implements PDCCH blind detection (SI-RNTI), PDSCH transport block decode
// (turbo code), and ASN.1 UPER parsing for SIB content.
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#ifndef LTE_SIB_H
#define LTE_SIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// ======================== Constants ========================

#define SI_RNTI             0xFFFF      // System Information RNTI
#define LTE_MAX_NEIGH_CELLS 16          // Max neighbor cells per SIB
#define LTE_MAX_EARFCN_LIST 8           // Max inter-freq carriers
#define LTE_MAX_UTRA_FREQ   8           // Max UMTS frequencies
#define LTE_MAX_GERAN_FREQ  16          // Max GSM frequencies
#define LTE_CMAS_MSG_SIZE   1024        // Max CMAS message bytes
#define LTE_ETWS_MSG_SIZE   512         // Max ETWS message bytes
#define LTE_MAX_SI_MSG      8           // Max SI messages (scheduling)

// SI periodicity values (radio frames)
static const int32_t si_periodicity_rf[] = { 8, 16, 32, 64, 128, 256, 512 };

// ======================== SIB Structures ========================

// SIB1: Cell identity and SI scheduling
typedef struct {
    // PLMN identity list (up to 6 PLMNs)
    struct {
        uint16_t mcc;
        uint16_t mnc;
        bool     cell_reserved;
    } plmn[6];
    int32_t      plmn_count;

    uint16_t tac;                   // Tracking Area Code
    uint32_t cell_id;               // 28-bit Cell Identity (eNodeB_ID << 8 | sector)
    bool     cell_barred;           // Cell barred for access
    bool     intra_freq_resel;      // Intra-freq reselection allowed
    int8_t   q_rxlevmin;            // Min RX level (dBm * 2)
    uint8_t  freq_band_indicator;   // E-UTRA band number

    // SI scheduling
    struct {
        uint8_t si_periodicity;     // Index into si_periodicity_rf[]
        uint8_t sib_mapping[8];     // SIB types in this SI message
        int32_t     sib_count;
    } si_sched[LTE_MAX_SI_MSG];
    int32_t      si_sched_count;
    uint8_t  si_window_length;      // ms (1,2,5,10,15,20,40)

    bool     valid;
} lte_sib1_info_t;

// SIB2: Common random-access and uplink configuration
typedef struct {
    uint8_t  ra_preambles;            // Number of RA preambles
    uint8_t  power_ramping_step_db;   // Power ramping step in dB
    int8_t   preamble_target_dbm;     // Initial preamble target power
    uint8_t  preamble_trans_max;      // Max RACH preamble transmissions
    uint8_t  ra_response_window_sf;   // RA response window in subframes
    uint8_t  max_harq_msg3_tx;        // Max HARQ retransmissions for Msg3
    bool     ul_carrier_freq_present;
    uint16_t ul_carrier_freq;         // UL EARFCN if present
    bool     ul_bandwidth_present;
    uint8_t  ul_bandwidth_rb;         // UL bandwidth in RBs if present
    uint16_t time_alignment_timer_sf; // Time alignment timer in subframes, 0xFFFF=infinity
    bool     valid;
} lte_sib2_t;

// SIB3: Intra-frequency cell reselection parameters
typedef struct {
    uint8_t  q_hyst_db;               // Hysteresis in dB
    bool     s_non_intra_search_present;
    uint8_t  s_non_intra_search;
    uint8_t  thresh_serving_low;
    uint8_t  cell_reselection_priority;
    int8_t   q_rxlevmin;              // Minimum RX level (dBm)
    bool     s_intra_search_present;
    uint8_t  s_intra_search;
    uint8_t  t_reselection_eutra;     // Raw enum value 0..7
    bool     valid;
} lte_sib3_t;

// SIB4: Intra-frequency neighbor cells
typedef struct {
    struct {
        uint16_t pci;               // Physical Cell ID
        int8_t   q_offset;          // Q-offset (dB * 2)
    } cells[LTE_MAX_NEIGH_CELLS];
    int32_t count;
    bool valid;
} lte_sib4_t;

// SIB5: Inter-frequency cell reselection
typedef struct {
    struct {
        uint32_t earfcn;            // DL EARFCN
        int8_t   q_rxlevmin;        // Min RX level (dBm * 2)
        uint8_t  priority;          // Cell reselection priority (0-7)
        uint8_t  thresh_x_high;     // Threshold for high priority
        uint8_t  thresh_x_low;      // Threshold for low priority
        uint8_t  bandwidth;         // Allowed measurement bandwidth
    } carriers[LTE_MAX_EARFCN_LIST];
    int32_t count;
    bool valid;
} lte_sib5_t;

// SIB6: UTRA (UMTS) neighbor frequencies
typedef struct {
    struct {
        uint16_t arfcn;             // UTRA-FDD ARFCN
        int8_t   q_rxlevmin;        // Min RX level
        uint8_t  priority;          // Cell reselection priority
        uint8_t  thresh_x_high;
        uint8_t  thresh_x_low;
    } carriers[LTE_MAX_UTRA_FREQ];
    int32_t count;
    bool valid;
} lte_sib6_t;

// SIB7: GERAN (GSM) neighbor frequencies
typedef struct {
    struct {
        uint16_t arfcn_start;       // Starting ARFCN
        uint8_t  band_indicator;    // 0=DCS1800, 1=PCS1900
        uint8_t  num_arfcns;        // Number of ARFCNs in group
        uint8_t  priority;          // Cell reselection priority
        uint8_t  thresh_x_high;
        uint8_t  thresh_x_low;
    } freq_groups[LTE_MAX_GERAN_FREQ];
    int32_t count;
    bool valid;
} lte_sib7_t;

// SIB10: ETWS Primary Notification
typedef struct {
    uint16_t message_id;            // Warning type identifier
    uint16_t serial_number;         // Geographic scope + message code
    uint8_t  warning_type[2];       // 2 bytes: type + emergency user alert + popup
    bool     warning_security;      // Security info present
    bool     valid;
} lte_sib10_t;

// SIB11: ETWS Secondary Notification (with text)
typedef struct {
    uint16_t message_id;
    uint16_t serial_number;
    uint8_t  warning_msg_segment_type;  // 0=last, 1=not last
    uint8_t  warning_msg_segment_num;   // Segment number
    uint8_t  data_coding_scheme;        // CBS DCS (GSM 03.38)
    uint8_t  warning_msg[LTE_ETWS_MSG_SIZE];
    int32_t      warning_msg_len;
    bool     valid;
} lte_sib11_t;

// SIB12: CMAS (Commercial Mobile Alert System / EU-Alert)
typedef struct {
    uint16_t message_id;            // Alert category (4370-4399)
    uint16_t serial_number;
    uint8_t  warning_msg_segment_type;
    uint8_t  warning_msg_segment_num;
    uint8_t  data_coding_scheme;
    uint8_t  warning_msg[LTE_CMAS_MSG_SIZE];
    int32_t      warning_msg_len;
    bool     valid;
} lte_sib12_t;

// SIB14: Extended Access Barring
typedef struct {
    // EAB categories (AC 0-9)
    struct {
        bool barred;
        uint8_t category;   // 0=a, 1=b, 2=c
    } ac_barring[10];
    bool valid;
} lte_sib14_t;

// Aggregate SIB results
typedef struct {
    lte_sib1_info_t sib1;
    lte_sib2_t      sib2;
    lte_sib3_t      sib3;
    lte_sib4_t      sib4;
    lte_sib5_t      sib5;
    lte_sib6_t      sib6;
    lte_sib7_t      sib7;
    lte_sib10_t     sib10;
    lte_sib11_t     sib11;
    lte_sib12_t     sib12;
    lte_sib14_t     sib14;
} lte_sib_results_t;

// ======================== DCI (Downlink Control Information) ========================

typedef struct {
    uint8_t  format;        // DCI format (1A or 1C)
    uint16_t rnti;          // RNTI used for scrambling
    uint8_t  n_prb;         // Number of PRBs allocated
    uint8_t  prb_start;     // Starting PRB (for localized VRB)
    uint8_t  mcs;           // Modulation and Coding Scheme
    uint8_t  rv;            // Redundancy version
    uint16_t tbs;           // Transport Block Size (bits)
    bool     valid;
} lte_dci_t;

// ======================== API ========================

// Attempt PDCCH blind decode for SI-RNTI in center 6 RBs
// iq = float IQ buffer, subframe_start = sample offset of subframe start
// n_rb_dl = total DL resource blocks, pci = physical cell identity
// Returns true if DCI found for system information
bool lte_decode_pdcch_si(const float *iq, int32_t iq_len, int32_t subframe_start,
                         int32_t n_rb_dl, int32_t pci, float freq_offset_hz,
                         const void *fft_twiddle, lte_dci_t *dci);

// Decode PDSCH transport block for SIB
// iq = float IQ buffer, subframe_start = sample offset
// dci = DCI from PDCCH, pci = physical cell identity
// out_bits = decoded transport block bits, out_len = bit count
// Returns true if CRC passes
bool lte_decode_pdsch_sib(const float *iq, int32_t iq_len, int32_t subframe_start,
                          int32_t n_rb_dl, int32_t pci, float freq_offset_hz,
                          const void *fft_twiddle, const lte_dci_t *dci,
                          uint8_t *out_bits, int32_t *out_len);

// Parse SIB1 from decoded BCCH-DL-SCH transport block
bool lte_parse_sib1(const uint8_t *bits, int32_t nbits, lte_sib1_info_t *sib1);

// Parse SI message (contains one or more SIBs according to scheduling)
bool lte_parse_si_msg(const uint8_t *bits, int32_t nbits, lte_sib_results_t *results);

// Get human-readable CMAS category from message_id
const char *lte_cmas_category(uint16_t message_id);

// Get human-readable ETWS warning type
const char *lte_etws_warning_type(uint8_t type_byte);

// Decode CBS data coding scheme (GSM 03.38) to UTF-8
int32_t lte_cbs_decode_text(const uint8_t *data, int32_t data_len, uint8_t dcs,
                        char *utf8_out, int32_t utf8_max);

#ifdef __cplusplus
}
#endif

#endif // LTE_SIB_H
