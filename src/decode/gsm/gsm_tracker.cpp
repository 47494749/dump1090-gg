// gsm_tracker.cpp — Global GSM cell tracking for web panel
//
// Thread-safe cell list updated from GSM decoder callbacks.

#include "gsm_tracker.h"
#include <cstdint>
#include <inttypes.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <pthread.h>
#include <ctime>

#include <string>
#include <cstdarg>

static gsm_tracked_cell_t cells[GSM_MAX_CELLS];
static pthread_mutex_t tracker_mutex = PTHREAD_MUTEX_INITIALIZER;

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

void gsmTrackerInit(void)
{
    pthread_mutex_lock(&tracker_mutex);
    memset(cells, 0, sizeof(cells));
    pthread_mutex_unlock(&tracker_mutex);
}

// Find or allocate a cell slot by MCC/MNC/LAC/CID
static gsm_tracked_cell_t *find_or_alloc(uint16_t mcc, uint16_t mnc, uint16_t lac, uint16_t cell_id)
{
    int32_t free_slot = -1;
    uint64_t oldest_time = UINT64_MAX;
    int32_t oldest_slot = 0;

    for (int32_t i = 0; i < GSM_MAX_CELLS; i++) {
        if (cells[i].active &&
            cells[i].mcc == mcc && cells[i].mnc == mnc &&
            cells[i].lac == lac && cells[i].cell_id == cell_id) {
            return &cells[i];
        }
        if (!cells[i].active && free_slot < 0) {
            free_slot = i;
        }
        if (cells[i].active && cells[i].last_seen_ms < oldest_time) {
            oldest_time = cells[i].last_seen_ms;
            oldest_slot = i;
        }
    }

    // Use free slot or evict oldest
    int32_t slot = (free_slot >= 0) ? free_slot : oldest_slot;
    memset(&cells[slot], 0, sizeof(cells[slot]));
    cells[slot].active = true;
    cells[slot].mcc = mcc;
    cells[slot].mnc = mnc;
    cells[slot].lac = lac;
    cells[slot].cell_id = cell_id;
    cells[slot].first_seen_ms = now_ms();
    return &cells[slot];
}

void gsmTrackerUpdateFCCH(uint16_t arfcn, double freq_mhz,
                          const gsm_stats_t *stats, gsm_sync_state_t sync_state)
{
    if (!stats) return;

    pthread_mutex_lock(&tracker_mutex);

    // Find existing FCCH-only entry or allocate with MCC=0
    gsm_tracked_cell_t *t = NULL;
    int32_t free_slot = -1;
    for (int32_t i = 0; i < GSM_MAX_CELLS; i++) {
        if (cells[i].active && cells[i].mcc == 0 && cells[i].arfcn == arfcn) {
            t = &cells[i];
            break;
        }
        if (!cells[i].active && free_slot < 0)
            free_slot = i;
    }
    if (!t && free_slot >= 0) {
        t = &cells[free_slot];
        memset(t, 0, sizeof(*t));
        t->active = true;
        t->arfcn = arfcn;
        t->first_seen_ms = now_ms();
    }
    if (t) {
        t->freq_mhz = freq_mhz;
        t->sync_state = (int32_t)sync_state;
        t->last_seen_ms = now_ms();
        t->freq_offset_hz = stats->freq_offset_hz;
        t->bcch_count = stats->fcch_detected;  // repurpose: show FCCH count
    }

    pthread_mutex_unlock(&tracker_mutex);
}

void gsmTrackerUpdate(const gsm_cell_info_t *cell, const gsm_stats_t *stats,
                      gsm_sync_state_t sync_state)
{
    if (!cell || cell->si3.mcc == 0) return;

    pthread_mutex_lock(&tracker_mutex);

    gsm_tracked_cell_t *t = find_or_alloc(cell->si3.mcc, cell->si3.mnc,
                                           cell->si3.lac, cell->si3.cell_id);

    t->arfcn = cell->arfcn;
    t->bsic = cell->bsic;
    t->freq_mhz = cell->freq_mhz;
    t->sync_state = (int32_t)sync_state;
    t->last_seen_ms = now_ms();

    // Update from SI3 if available
    if (cell->si3.valid) {
        t->ccch_conf = cell->si3.ccch_conf;
        t->t3212 = cell->si3.t3212;
        t->cell_barred = cell->si3.cell_barred;
        t->rxlev_access_min = (int8_t)cell->si3.rxlev_access_min;
        t->last_si3_ms = now_ms();
    }

    if (stats) {
        t->bcch_count = stats->bcch_decoded;
        t->ccch_count = stats->ccch_decoded;
        t->cb_count = stats->cb_decoded;
        t->freq_offset_hz = stats->freq_offset_hz;
    }

    pthread_mutex_unlock(&tracker_mutex);
}

void gsmTrackerUpdateCB(const gsm_cell_info_t *cell, const gsm_cb_msg_t *cb)
{
    if (!cell || !cb || cell->si3.mcc == 0) return;

    pthread_mutex_lock(&tracker_mutex);

    gsm_tracked_cell_t *t = find_or_alloc(cell->si3.mcc, cell->si3.mnc,
                                           cell->si3.lac, cell->si3.cell_id);

    t->last_cb_id = cb->msg_id;
    if (cb->text_len > 0) {
        snprintf(t->last_cb_text, sizeof(t->last_cb_text), "%.*s",
                 cb->text_len < 255 ? cb->text_len : 255, cb->text);
    }
    t->cb_count++;
    t->last_seen_ms = now_ms();

    pthread_mutex_unlock(&tracker_mutex);
}

int32_t gsmTrackerActiveCount(void)
{
    uint64_t cutoff = now_ms() - (uint64_t)GSM_CELL_TIMEOUT * 1000;
    int32_t count = 0;
    pthread_mutex_lock(&tracker_mutex);
    for (int32_t i = 0; i < GSM_MAX_CELLS; i++) {
        if (cells[i].active && cells[i].last_seen_ms >= cutoff)
            count++;
    }
    pthread_mutex_unlock(&tracker_mutex);
    return count;
}

// Helper: format into std::string
static std::string sfmt(const char *fmt, ...) __attribute__((format(printf,1,2)));
static std::string sfmt(const char *fmt, ...) {
    char tmp[1024];
    va_list ap;
    va_start(ap, fmt);
    int32_t n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n < 0) return {};
    if ((size_t)n < sizeof(tmp)) return std::string(tmp, n);
    std::string s(n, '\0');
    va_start(ap, fmt);
    vsnprintf(&s[0], n + 1, fmt, ap);
    va_end(ap);
    return s;
}

// Escape a string for JSON (minimal: escape quotes, backslash, control chars)
static std::string json_esc(const char *src)
{
    std::string out;
    for (const char *s = src; *s; s++) {
        if (*s == '"' || *s == '\\') {
            out += '\\';
            out += *s;
        } else if ((uint8_t)*s < 0x20) {
            // Skip control chars
        } else {
            out += *s;
        }
    }
    return out;
}

std::string gsmTrackerToJSON(void)
{
    uint64_t ts = now_ms();
    uint64_t cutoff = ts - (uint64_t)GSM_CELL_TIMEOUT * 1000;

    std::string s = sfmt("{\"now\":%.1f,\"cells\":[\n", ts / 1000.0);

    pthread_mutex_lock(&tracker_mutex);

    int32_t first = 1;
    for (int32_t i = 0; i < GSM_MAX_CELLS; i++) {
        gsm_tracked_cell_t *c = &cells[i];
        if (!c->active) continue;
        bool stale = (c->last_seen_ms < cutoff);

        if (!first) s += ",\n";
        first = 0;

        std::string cb_esc = json_esc(c->last_cb_text);

        const char *sync_names[] = {"none", "fcch", "sch", "locked"};
        const char *sync_name = (c->sync_state >= 0 && c->sync_state <= 3)
                                ? sync_names[c->sync_state] : "?";

        s += sfmt(
            "{\"mcc\":%d,\"mnc\":%d,\"lac\":%u,\"cid\":%u,"
            "\"arfcn\":%d,\"bsic\":%d,\"freq_mhz\":%.3f,"
            "\"sync\":\"%s\",\"stale\":%s,"
            "\"ccch_conf\":%d,\"t3212\":%d,\"cell_barred\":%s,"
            "\"rxlev_min\":%d,"
            "\"bcch\":%" PRIu64 ",\"ccch\":%" PRIu64 ",\"cb\":%" PRIu64 ",\"paging\":%" PRIu64 ","
            "\"freq_offset\":%.1f,"
            "\"last_cb_id\":%d,\"last_cb_text\":\"%s\","
            "\"first_seen\":%.1f,\"last_seen\":%.1f,\"age\":%.1f}",
            c->mcc, c->mnc, c->lac, c->cell_id,
            c->arfcn, c->bsic, c->freq_mhz,
            sync_name, stale ? "true" : "false",
            c->ccch_conf, c->t3212, c->cell_barred ? "true" : "false",
            c->rxlev_access_min,
            (uint64_t)c->bcch_count,
            (uint64_t)c->ccch_count,
            (uint64_t)c->cb_count,
            (uint64_t)c->paging_count,
            c->freq_offset_hz,
            c->last_cb_id, cb_esc.c_str(),
            c->first_seen_ms / 1000.0,
            c->last_seen_ms / 1000.0,
            (ts - c->last_seen_ms) / 1000.0);
    }

    pthread_mutex_unlock(&tracker_mutex);

    s += "\n]}";
    return s;
}
