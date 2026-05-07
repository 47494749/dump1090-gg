// Part of dump1090-gg-light
//
// lte_tracker.c: Thread-safe LTE cell tracking and JSON export.
//
// Maintains a global table of detected LTE cells. Updated from the decoder
// callback (reader thread) and queried from the HTTP server thread (JSON API).
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#include "lte_tracker.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

#define TRACKER_MAX_CELLS  64
#define CELL_TIMEOUT_SEC   30  // Remove cells not seen for this long

// Infer operator from EARFCN for Band 20 Germany (before SIB1 decode)
static const char *lte_operator_guess(unsigned earfcn)
{
    switch (earfcn) {
        case 6200: return "O2";
        case 6300: return "Vodafone";
        case 6400: return "Telekom";
        default:   return NULL;
    }
}

// ====== Static cell database (from cellmapper.net / opencellid) ======
// Since SIB1 decode requires full BW (15.36 MS/s for 10 MHz cells, impossible
// with RTL-SDR at 1.92 MS/s), we use a static lookup table for known cells.
// Key: PCI + EARFCN → Cell ID, eNodeB, TAC, location
typedef struct {
    uint16_t pci;
    uint16_t earfcn;
    uint32_t cell_id;       // 28-bit (eNodeB_id << 8 | sector)
    uint16_t tac;
    uint16_t mcc;
    uint16_t mnc;
    float    lat;           // Latitude
    float    lon;           // Longitude
    const char *enodeb_name; // Human-readable name/address
} lte_celldb_entry_t;

// Cell database — populated from cellmapper.net for the local area
// To find entries: https://www.cellmapper.net/map?MCC=262&MNC=2&type=LTE&band=20
// Cell ID = eNodeB_ID * 256 + sector_id
static const lte_celldb_entry_t lte_celldb[] = {
    // PCI  EARFCN  Cell_ID      TAC   MCC MNC  Lat       Lon       Name
    // === Vodafone (262/02) Band 20 EARFCN 6300 ===
    {128,  6300,  0x1A2C801,   0x5A01, 262, 2, 48.1351f, 11.5820f, "Vodafone München-Ost"},

    // === Telekom (262/01) Band 20 EARFCN 6400 ===
    {328,  6400,  0x0,         0x0,    262, 1, 0.0f,     0.0f,     NULL},

    // === O2 (262/03) Band 20 EARFCN 6200 ===
    // Add entries as cells are discovered

    {0, 0, 0, 0, 0, 0, 0.0f, 0.0f, NULL}  // Sentinel
};

// Lookup a cell by PCI + EARFCN
static const lte_celldb_entry_t *lte_celldb_lookup(uint16_t pci, uint16_t earfcn)
{
    for (int i = 0; lte_celldb[i].pci || lte_celldb[i].earfcn; i++) {
        if (lte_celldb[i].pci == pci && lte_celldb[i].earfcn == earfcn)
            return &lte_celldb[i];
    }
    return NULL;
}

typedef struct {
    lte_cell_info_t info;
    time_t          first_seen;
    time_t          last_seen;
    bool            active;
} tracked_cell_t;

static pthread_mutex_t tracker_mutex = PTHREAD_MUTEX_INITIALIZER;
static tracked_cell_t  tracker_cells[TRACKER_MAX_CELLS];
static int             tracker_count = 0;
static bool            tracker_inited = false;

void lteTrackerInit(void)
{
    pthread_mutex_lock(&tracker_mutex);
    memset(tracker_cells, 0, sizeof(tracker_cells));
    tracker_count = 0;
    tracker_inited = true;
    pthread_mutex_unlock(&tracker_mutex);
}

void lteTrackerUpdate(const lte_cell_info_t *cell)
{
    if (!cell || !tracker_inited) return;

    pthread_mutex_lock(&tracker_mutex);
    time_t now = time(NULL);
    tracked_cell_t *slot = NULL;

    // Find existing entry by PCI + freq
    for (int i = 0; i < tracker_count; i++) {
        if (tracker_cells[i].active &&
            tracker_cells[i].info.pci == cell->pci &&
            tracker_cells[i].info.freq_hz == cell->freq_hz) {
            slot = &tracker_cells[i];
            break;
        }
    }

    if (!slot) {
        // Find a free slot
        for (int i = 0; i < TRACKER_MAX_CELLS; i++) {
            if (!tracker_cells[i].active) {
                slot = &tracker_cells[i];
                if (i >= tracker_count) tracker_count = i + 1;
                slot->first_seen = now;
                break;
            }
        }
    }

    if (slot) {
        slot->info = *cell;
        slot->last_seen = now;
        slot->active = true;
    }

    // Expire old cells
    for (int i = 0; i < tracker_count; i++) {
        if (tracker_cells[i].active &&
            (now - tracker_cells[i].last_seen) > CELL_TIMEOUT_SEC) {
            tracker_cells[i].active = false;
        }
    }

    pthread_mutex_unlock(&tracker_mutex);
}

int lteTrackerCount(void)
{
    pthread_mutex_lock(&tracker_mutex);
    int count = 0;
    for (int i = 0; i < tracker_count; i++)
        if (tracker_cells[i].active) count++;
    pthread_mutex_unlock(&tracker_mutex);
    return count;
}

char *lteTrackerToJSON(void)
{
    pthread_mutex_lock(&tracker_mutex);

    int buf_size = 256 + tracker_count * 512;
    char *buf = malloc((size_t)buf_size);
    if (!buf) { pthread_mutex_unlock(&tracker_mutex); return NULL; }

    int pos = 0;
    pos += snprintf(buf + pos, (size_t)(buf_size - pos), "{\"cells\":[");

    int first = 1;
    int active_count = 0;
    time_t now = time(NULL);

    for (int i = 0; i < tracker_count; i++) {
        if (!tracker_cells[i].active) continue;
        active_count++;
        const lte_cell_info_t *c = &tracker_cells[i].info;

        if (!first) buf[pos++] = ',';
        first = 0;

        pos += snprintf(buf + pos, (size_t)(buf_size - pos),
            "{\"pci\":%u,\"n_id_1\":%u,\"n_id_2\":%u,"
            "\"freq\":%.0f,\"earfcn\":%u,\"band\":\"%s\","
            "\"rsrp\":%.1f,\"snr\":%.1f,\"freq_offset\":%.1f,"
            "\"sync\":\"%s\"",
            c->pci, c->n_id_1, c->n_id_2,
            c->freq_hz, c->earfcn, lte_band_name(c->freq_hz),
            c->rsrp_dbfs, c->snr_db, c->freq_offset_hz,
            (c->sync_state == LTE_SYNC_SIB1) ? "SIB1" :
            (c->sync_state == LTE_SYNC_MIB) ? "MIB" :
            (c->sync_state == LTE_SYNC_SSS) ? "SSS" :
            (c->sync_state == LTE_SYNC_PSS) ? "PSS" : "none");

        // Operator: from SIB1 (MCC/MNC) or inferred from EARFCN
        const char *op_name = NULL;
        if (c->sib1.valid && c->sib1.mcc == 262) {
            switch (c->sib1.mnc) {
                case 1: case 6: op_name = "Telekom"; break;
                case 2: case 4: op_name = "Vodafone"; break;
                case 3: case 7: case 77: op_name = "O2"; break;
            }
        }
        if (!op_name) op_name = lte_operator_guess(c->earfcn);
        if (op_name) {
            pos += snprintf(buf + pos, (size_t)(buf_size - pos),
                ",\"operator\":\"%s\"", op_name);
        }

        if (c->mib.valid) {
            pos += snprintf(buf + pos, (size_t)(buf_size - pos),
                ",\"mib\":{\"bw\":\"%s\",\"nrb\":%d,\"sfn\":%u,\"phich_dur\":%u,\"phich_res\":%u}",
                lte_bw_string(c->mib.dl_bandwidth),
                lte_bw_to_nrb(c->mib.dl_bandwidth),
                c->mib.sfn, c->mib.phich_duration, c->mib.phich_resources);
        }

        if (c->sib1.valid) {
            pos += snprintf(buf + pos, (size_t)(buf_size - pos),
                ",\"sib1\":{\"mcc\":%u,\"mnc\":%u,\"tac\":%u,\"cell_id\":%u,"
                "\"barred\":%s,\"q_rxlevmin\":%d}",
                c->sib1.mcc, c->sib1.mnc, c->sib1.tac, c->sib1.cell_id,
                c->sib1.cell_barred ? "true" : "false", c->sib1.q_rxlevmin);
        }

        // Cell database lookup (enriches with known info when SIB1 unavailable)
        const lte_celldb_entry_t *db = lte_celldb_lookup(c->pci, c->earfcn);
        if (db && db->cell_id) {
            uint32_t enodeb_id = db->cell_id >> 8;
            uint8_t sector_id = db->cell_id & 0xFF;
            pos += snprintf(buf + pos, (size_t)(buf_size - pos),
                ",\"celldb\":{\"cell_id\":%u,\"enodeb_id\":%u,\"sector\":%u,"
                "\"tac\":%u,\"mcc\":%u,\"mnc\":%u",
                db->cell_id, enodeb_id, sector_id,
                db->tac, db->mcc, db->mnc);
            if (db->lat != 0.0f || db->lon != 0.0f) {
                pos += snprintf(buf + pos, (size_t)(buf_size - pos),
                    ",\"lat\":%.6f,\"lon\":%.6f", db->lat, db->lon);
            }
            if (db->enodeb_name) {
                pos += snprintf(buf + pos, (size_t)(buf_size - pos),
                    ",\"name\":\"%s\"", db->enodeb_name);
            }
            pos += snprintf(buf + pos, (size_t)(buf_size - pos), "}");
        }

        // Alerts (ETWS/CMAS/EAB)
        if (c->alert_count > 0) {
            pos += snprintf(buf + pos, (size_t)(buf_size - pos), ",\"alerts\":[");
            for (int ai = 0; ai < c->alert_count; ai++) {
                const lte_alert_t *a = &c->alerts[ai];
                if (ai > 0) buf[pos++] = ',';
                pos += snprintf(buf + pos, (size_t)(buf_size - pos),
                    "{\"type\":\"%s\",\"message_id\":%u,\"serial\":%u,"
                    "\"category\":\"%s\",\"text\":\"%s\",\"timestamp\":%llu,\"active\":%s}",
                    (a->type == LTE_ALERT_ETWS) ? "ETWS" :
                    (a->type == LTE_ALERT_CMAS) ? "CMAS" :
                    (a->type == LTE_ALERT_EAB)  ? "EAB" : "unknown",
                    a->message_id, a->serial_number,
                    a->category, a->text,
                    (unsigned long long)a->timestamp,
                    a->active ? "true" : "false");
            }
            pos += snprintf(buf + pos, (size_t)(buf_size - pos), "]");
        }

        pos += snprintf(buf + pos, (size_t)(buf_size - pos),
            ",\"pss_count\":%u,\"sss_count\":%u,\"mib_count\":%u,"
            "\"sib1_count\":%u,\"crc_errors\":%u,"
            "\"first_seen\":%ld,\"last_seen\":%ld,\"age\":%ld}",
            c->pss_count, c->sss_count, c->mib_count,
            c->sib1_count, c->crc_errors,
            (long)tracker_cells[i].first_seen,
            (long)tracker_cells[i].last_seen,
            (long)(now - tracker_cells[i].first_seen));
    }

    pos += snprintf(buf + pos, (size_t)(buf_size - pos), "],\"count\":%d}", active_count);

    pthread_mutex_unlock(&tracker_mutex);
    return buf;
}

void lteTrackerDestroy(void)
{
    pthread_mutex_lock(&tracker_mutex);
    memset(tracker_cells, 0, sizeof(tracker_cells));
    tracker_count = 0;
    tracker_inited = false;
    pthread_mutex_unlock(&tracker_mutex);
}
