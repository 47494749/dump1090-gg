// gsm_tracker.h — Global GSM cell tracking for web panel
//
// Maintains a list of discovered GSM cells, updated from decoder callbacks,
// and provides JSON serialization for the /api/gsm endpoint.

#ifndef GSM_TRACKER_H
#define GSM_TRACKER_H

#include <stdint.h>

#include "gsm_decode.h"

#define GSM_MAX_CELLS  64    // max tracked cells
#define GSM_CELL_TIMEOUT 300 // seconds before cell is considered stale

// Tracked cell entry (enriched from SI1/SI2/SI3/SI4 + CB messages)
typedef struct {
    // Identity
    uint16_t mcc;
    uint16_t mnc;
    uint16_t lac;
    uint16_t cell_id;
    uint16_t arfcn;
    uint8_t  bsic;

    // Cell info (from SI3/SI4)
    uint8_t  ccch_conf;
    uint8_t  t3212;              // periodic location update timer
    bool     cell_barred;
    int8_t   rxlev_access_min;

    // Frequency info
    double   freq_mhz;          // downlink frequency

    // Statistics
    uint64_t bcch_count;         // BCCH blocks decoded
    uint64_t ccch_count;         // CCCH blocks decoded
    uint64_t cb_count;           // Cell Broadcast messages
    uint64_t paging_count;       // Paging requests seen
    double   freq_offset_hz;     // current frequency offset

    // Sync state
    int32_t      sync_state;         // gsm_sync_state_t

    // Last Cell Broadcast text
    char     last_cb_text[256];
    uint16_t last_cb_id;

    // Timestamps
    uint64_t first_seen_ms;      // when first discovered
    uint64_t last_seen_ms;       // last activity
    uint64_t last_si3_ms;        // last SI3 received

    bool     active;             // slot in use
} gsm_tracked_cell_t;

// Initialize the tracker (call once at startup)
void gsmTrackerInit(void);

// Update a cell from decoder callback data
void gsmTrackerUpdate(const gsm_cell_info_t *cell, const gsm_stats_t *stats,
                      gsm_sync_state_t sync_state);

// Update tracker with FCCH-only info (no decoded SI required)
void gsmTrackerUpdateFCCH(uint16_t arfcn, double freq_mhz,
                          const gsm_stats_t *stats, gsm_sync_state_t sync_state);

// Update cell broadcast info
void gsmTrackerUpdateCB(const gsm_cell_info_t *cell, const gsm_cb_msg_t *cb);

// Get number of active (non-stale) cells
int32_t gsmTrackerActiveCount(void);

// Returns std::string
#include <string>
std::string gsmTrackerToJSON(void);

#endif // GSM_TRACKER_H
