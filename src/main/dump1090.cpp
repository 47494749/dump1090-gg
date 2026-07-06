// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// dump1090.c: main program & miscellany
//
// Copyright (c) 2014-2016 Oliver Jowett <oliver@mutability.co.uk>
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// This file incorporates work covered by the following copyright and
// permission notice:
//
//   Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//    *  Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//    *  Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "dump1090.h"
#include <stdint.h>
#include "cpu.h"
#include "opensky_client.h"
#include "pocsag_demod.h"
#include "sonde_demod.h"
#include "acars_demod.h"
#include "gsm_tracker.h"
#include "lte_tracker.h"
#include "iot_tracker.h"
#include "dispatcher.h"
#include "airframes_feed.h"

#ifdef ENABLE_SDRGG
#include "sdrgg.h"
#endif

#include <cstdarg>
#include "gg_format.h"

struct _Modes Modes;

// POCSAG CLI config (used to build rx_config_t for SdrManager)
static struct {
    int32_t    enabled;
    std::string device_serial;
    int32_t    freq;           // center frequency in Hz
    float  gain;           // gain in dB
    int32_t    ppm_error;
    std::string ifile_path;    // Path to raw IQ file (uint8 I/Q pairs, 2.4 MSPS)
} PocsagConfig;

// POCSAG ifile reader state
static struct {
    struct pocsag_state *demod;
    pthread_t thread;
    int32_t       thread_running;
    volatile int32_t stop_flag;
} PocsagIfile;

// ======================== POCSAG ifile reader ========================

// Callback for POCSAG messages decoded from file input
static void pocsag_ifile_msg_handler(const pocsag_msg_t *msg, void *ctx)
{
    (void)ctx;

    const char *freq_str = "";
    char freq_buf[32];
    if (msg->channel_freq > 0) {
        snprintf(freq_buf, sizeof(freq_buf), " %.3fMHz", msg->channel_freq / 1e6);
        freq_str = freq_buf;
    }

    if (msg->is_tone_only) {
        fprintf(stderr, "[POCSAG-ifile]%s %d baud addr=%07u func=%d TONE-ONLY\n",
                freq_str, msg->baud_rate, msg->address, msg->function);
    } else if (msg->is_alpha && msg->alpha_len > 0) {
        fprintf(stderr, "[POCSAG-ifile]%s %d baud addr=%07u func=%d \"%s\"\n",
                freq_str, msg->baud_rate, msg->address, msg->function, msg->alpha_msg);
    } else if (msg->is_numeric && msg->numeric_len > 0) {
        fprintf(stderr, "[POCSAG-ifile]%s %d baud addr=%07u func=%d num=%s\n",
                freq_str, msg->baud_rate, msg->address, msg->function, msg->numeric_msg);
    } else {
        fprintf(stderr, "[POCSAG-ifile]%s %d baud addr=%07u func=%d (empty)\n",
                freq_str, msg->baud_rate, msg->address, msg->function);
    }
}

// Thread that reads raw IQ file and feeds to POCSAG decoder
static void *pocsag_ifile_reader_thread(void *arg)
{
    (void)arg;

    const char *path = PocsagConfig.ifile_path.c_str();
    const uint32_t buf_size = 262144; // 256 KB
    uint8_t *buf = static_cast<uint8_t*>(malloc(buf_size));
    if (!buf) {
        gg::eprint("pocsag-ifile: failed to allocate read buffer\n");
        return NULL;
    }

    gg::eprint("pocsag-ifile: reader thread started, file=%s\n", path);

    FILE *fp = fopen(path, "rb");
    if (!fp) {
        gg::eprint("pocsag-ifile: cannot open '%s': %s\n", path, strerror(errno));
        free(buf);
        return NULL;
    }

    while (!PocsagIfile.stop_flag && !Modes.exit) {
        size_t nread = fread(buf, 1, buf_size, fp);
        if (nread == 0) break; // EOF

        nread &= ~(size_t)1; // ensure even (IQ pairs)

        if (PocsagIfile.demod && nread > 0) {
            pocsag_process(PocsagIfile.demod, buf, (uint32_t)nread);
        }

        // Throttle to approximate real-time
        double samples = nread / 2.0;
        double sleep_us = (samples / POCSAG_SAMPLE_RATE) * 1e6;
        if (sleep_us > 0) {
            struct timespec ts;
            ts.tv_sec = (time_t)(sleep_us / 1e6);
            ts.tv_nsec = (int64_t)(fmod(sleep_us, 1e6) * 1000);
            nanosleep(&ts, NULL);
        }
    }

    fclose(fp);
    free(buf);

    // Print final stats
    pocsag_stats_t stats;
    pocsag_get_stats(PocsagIfile.demod, &stats);
    fprintf(stderr, "pocsag-ifile: done. samples=%" PRIu64 " preambles=%" PRIu64 " syncs=%" PRIu64 " messages=%" PRIu64 " bch_ok=%" PRIu64 " bch_fail=%" PRIu64 "\n",
            (uint64_t)stats.samples_processed,
            (uint64_t)stats.preambles_detected,
            (uint64_t)stats.syncs_detected,
            (uint64_t)stats.messages_decoded,
            (uint64_t)stats.bch_corrections,
            (uint64_t)stats.bch_failures);

    // Signal main loop to exit when file playback is done
    Modes.exit = 1;

    return NULL;
}

// ======================== Sonde (RS41) ifile reader ========================

static struct {
    std::string ifile_path;
    int32_t    freq;           // center frequency in Hz (default 403000000)
} SondeIfileConfig;

static struct {
    struct sonde_state *demod;
    pthread_t thread;
    int32_t       thread_running;
    volatile int32_t stop_flag;
} SondeIfile;

static void sonde_ifile_msg_handler(const sonde_msg_t *msg, void *ctx)
{
    (void)ctx;
    if (msg->valid_pos) {
        fprintf(stderr, "[SONDE-ifile] %s %s pos=%.5f,%.5f alt=%.1fm vel=%.1f/%.1fm/s hdg=%.0f frame=%d rs=%d sat=%d\n",
                msg->type, msg->serial,
                msg->lat, msg->lon, msg->alt,
                msg->vel_h, msg->vel_v, msg->heading,
                msg->frame_num, msg->rs_errors, msg->satellites);
    } else {
        fprintf(stderr, "[SONDE-ifile] %s %s frame=%d (no GPS fix) rs=%d\n",
                msg->type, msg->serial, msg->frame_num, msg->rs_errors);
    }
}

#define SONDE_IFILE_SAMPLE_RATE 2400000

static void *sonde_ifile_reader_thread(void *arg)
{
    (void)arg;

    const char *path = SondeIfileConfig.ifile_path.c_str();
    const uint32_t buf_size = 262144;
    uint8_t *buf = static_cast<uint8_t*>(malloc(buf_size));
    if (!buf) {
        gg::eprint("sonde-ifile: failed to allocate read buffer\n");
        return NULL;
    }

    gg::eprint("sonde-ifile: reader thread started, file=%s\n", path);

    FILE *fp = fopen(path, "rb");
    if (!fp) {
        gg::eprint("sonde-ifile: cannot open '%s': %s\n", path, strerror(errno));
        free(buf);
        return NULL;
    }

    while (!SondeIfile.stop_flag && !Modes.exit) {
        size_t nread = fread(buf, 1, buf_size, fp);
        if (nread == 0) break;

        nread &= ~(size_t)1; // ensure even (IQ pairs)

        if (SondeIfile.demod && nread > 0) {
            sonde_process(SondeIfile.demod, buf, (uint32_t)nread);
        }

        // Throttle to approximate real-time
        double samples = nread / 2.0;
        double sleep_us = (samples / SONDE_IFILE_SAMPLE_RATE) * 1e6;
        if (sleep_us > 0) {
            struct timespec ts;
            ts.tv_sec = (time_t)(sleep_us / 1e6);
            ts.tv_nsec = (int64_t)(fmod(sleep_us, 1e6) * 1000);
            nanosleep(&ts, NULL);
        }
    }

    fclose(fp);
    free(buf);

    // Print final stats
    sonde_stats_t stats;
    sonde_get_stats(SondeIfile.demod, &stats);
    fprintf(stderr, "sonde-ifile: done. samples=%" PRIu64 " frames_detected=%" PRIu64 " frames_decoded=%" PRIu64 " rs_corrected=%" PRIu64 " rs_uncorrectable=%" PRIu64 " crc_errors=%" PRIu64 "\n",
            (uint64_t)stats.samples_processed,
            (uint64_t)stats.frames_detected,
            (uint64_t)stats.frames_decoded,
            (uint64_t)stats.rs_corrected,
            (uint64_t)stats.rs_uncorrectable,
            (uint64_t)stats.crc_errors);

    Modes.exit = 1;
    return NULL;
}

// ======================== ACARS ifile reader ========================

static struct {
    std::string ifile_path;
    int32_t    freq;           // center frequency in Hz (default 131550000)
} AcarsIfileConfig;

static struct {
    struct acars_state *demod;
    pthread_t thread;
    int32_t       thread_running;
    volatile int32_t stop_flag;
} AcarsIfile;

#define ACARS_IFILE_SAMPLE_RATE 2400000

static void acars_ifile_msg_handler(const acars_msg_t *msg, void *ctx)
{
    (void)ctx;
    fprintf(stderr, "[ACARS-ifile] ch=%d %.3fMHz mode=%c reg=%s label=%s blk=%c%s%s%s%s%s msgno=%s flight=%s \"%.*s\"\n",
            msg->channel, msg->freq / 1e6, msg->mode,
            msg->reg, msg->label, msg->block_id,
            msg->dsp_header[0] ? " route=" : "",
            msg->dsp_header[0] ? msg->dsp_header : "",
            msg->sublabel[0] ? " sub=" : "",
            msg->sublabel[0] ? msg->sublabel : "",
            msg->mfi[0] ? msg->mfi : "",
            msg->msgno, msg->flight,
            msg->text_len, msg->text);
}

static void *acars_ifile_reader_thread(void *arg)
{
    (void)arg;

    const char *path = AcarsIfileConfig.ifile_path.c_str();
    const uint32_t buf_size = 262144;
    uint8_t *buf = static_cast<uint8_t*>(malloc(buf_size));
    if (!buf) {
        gg::eprint("acars-ifile: failed to allocate read buffer\n");
        return NULL;
    }

    gg::eprint("acars-ifile: reader thread started, file=%s\n", path);

    FILE *fp = fopen(path, "rb");
    if (!fp) {
        gg::eprint("acars-ifile: cannot open '%s': %s\n", path, strerror(errno));
        free(buf);
        return NULL;
    }

    while (!AcarsIfile.stop_flag && !Modes.exit) {
        size_t nread = fread(buf, 1, buf_size, fp);
        if (nread == 0) break;
        nread &= ~(size_t)1;
        if (AcarsIfile.demod && nread > 0)
            acars_process(AcarsIfile.demod, buf, (uint32_t)nread);

        double samples = nread / 2.0;
        double sleep_us = (samples / ACARS_IFILE_SAMPLE_RATE) * 1e6;
        if (sleep_us > 0) {
            struct timespec ts;
            ts.tv_sec = (time_t)(sleep_us / 1e6);
            ts.tv_nsec = (int64_t)(fmod(sleep_us, 1e6) * 1000);
            nanosleep(&ts, NULL);
        }
    }

    fclose(fp);
    free(buf);

    acars_stats_t stats;
    acars_get_stats(AcarsIfile.demod, &stats);
    fprintf(stderr, "acars-ifile: done. samples=%" PRIu64 " messages=%" PRIu64 " crc_errors=%" PRIu64 "\n",
            (uint64_t)stats.samples_processed,
            (uint64_t)stats.messages_decoded,
            (uint64_t)stats.crc_errors);

    Modes.exit = 1;
    return NULL;
}

// ======================== GSM ifile reader ========================

static struct {
    std::string ifile_path;
    int32_t    freq;           // ARFCN frequency in Hz (default 939200000 = ARFCN 16, Vodafone DE)
} GsmIfileConfig;

static struct {
    struct gsm_state *demod;
    pthread_t thread;
    int32_t       thread_running;
    volatile int32_t stop_flag;
} GsmIfile;

#define GSM_IFILE_SAMPLE_RATE 1000000

static void gsm_ifile_msg_handler(const gsm_cell_info_t *cell, const char *msg_type,
                                   const uint8_t *l3_data, int32_t l3_len, void *ctx)
{
    (void)l3_data; (void)l3_len; (void)ctx;
    fprintf(stderr, "[GSM-ifile] %s MCC=%03u MNC=%02u LAC=%u CellID=%u BSIC=%u ARFCN=%u\n",
            msg_type, cell->si3.mcc, cell->si3.mnc, cell->si3.lac,
            cell->si3.cell_id, cell->bsic, cell->arfcn);
}

static void gsm_ifile_cb_handler(const gsm_cell_info_t *cell, const gsm_cb_msg_t *cb, void *ctx)
{
    (void)cell; (void)ctx;
    fprintf(stderr, "[GSM-ifile] CB serial=%u msg_id=%u page=%u/%u \"%.*s\"\n",
            cb->serial_nr, cb->msg_id, cb->page_nr, cb->total_pages,
            cb->text_len, cb->text);
}

static void *gsm_ifile_reader_thread(void *arg)
{
    (void)arg;

    const char *path = GsmIfileConfig.ifile_path.c_str();
    const uint32_t buf_size = 262144;
    uint8_t *buf = static_cast<uint8_t*>(malloc(buf_size));
    if (!buf) {
        gg::eprint("gsm-ifile: failed to allocate read buffer\n");
        return NULL;
    }

    gg::eprint("gsm-ifile: reader thread started, file=%s\n", path);

    FILE *fp = fopen(path, "rb");
    if (!fp) {
        gg::eprint("gsm-ifile: cannot open '%s': %s\n", path, strerror(errno));
        free(buf);
        return NULL;
    }

    while (!GsmIfile.stop_flag && !Modes.exit) {
        size_t nread = fread(buf, 1, buf_size, fp);
        if (nread == 0) break;
        nread &= ~(size_t)1;
        if (GsmIfile.demod && nread > 0) {
            /* Feed in small chunks so the state machine runs frequently
             * enough to track frame positions across buffer shifts.
             * chunk_size = 9230 samples (18460 bytes) = half the phase buffer */
            const uint32_t chunk_bytes = 18460;
            uint32_t off = 0;
            while (off < (uint32_t)nread) {
                uint32_t n = (uint32_t)nread - off;
                if (n > chunk_bytes) n = chunk_bytes;
                gsm_process(GsmIfile.demod, buf + off, n);
                off += n;
            }
        }

        double samples = nread / 2.0;
        double sleep_us = (samples / GSM_IFILE_SAMPLE_RATE) * 1e6;
        if (sleep_us > 0) {
            struct timespec ts;
            ts.tv_sec = (time_t)(sleep_us / 1e6);
            ts.tv_nsec = (int64_t)(fmod(sleep_us, 1e6) * 1000);
            nanosleep(&ts, NULL);
        }
    }

    fclose(fp);
    free(buf);

    gsm_stats_t stats;
    gsm_get_stats(GsmIfile.demod, &stats);
    fprintf(stderr, "gsm-ifile: done. samples=%" PRIu64 " fcch=%" PRIu64 " sch=%" PRIu64 "/%" PRIu64 " bcch=%" PRIu64 "/%" PRIu64 " ccch=%" PRIu64 " cb=%" PRIu64 "\n",
            (uint64_t)stats.samples_processed,
            (uint64_t)stats.fcch_detected,
            (uint64_t)stats.sch_decoded,
            (uint64_t)stats.sch_failed,
            (uint64_t)stats.bcch_decoded,
            (uint64_t)stats.bcch_failed,
            (uint64_t)stats.ccch_decoded,
            (uint64_t)stats.cb_decoded);

    Modes.exit = 1;
    return NULL;
}

// ======================== LTE ifile reader ========================

static struct {
    std::string ifile_path;
    int32_t    freq;           // center frequency in Hz (default 806000000, Band 20)
} LteIfileConfig;

static struct {
    struct lte_state *demod;
    pthread_t thread;
    int32_t       thread_running;
    volatile int32_t stop_flag;
} LteIfile;

#define LTE_IFILE_SAMPLE_RATE 1920000

static void lte_ifile_cell_handler(const lte_cell_info_t *cell, void *ctx)
{
    (void)ctx;
    const char *state_str = "none";
    switch (cell->sync_state) {
        case LTE_SYNC_PSS:  state_str = "PSS"; break;
        case LTE_SYNC_SSS:  state_str = "SSS"; break;
        case LTE_SYNC_MIB:  state_str = "MIB"; break;
        case LTE_SYNC_SIB1: state_str = "SIB1"; break;
        default: break;
    }
    fprintf(stderr, "[LTE-ifile] PCI=%u sync=%s freq=%.1fMHz",
            cell->pci, state_str, cell->freq_hz / 1e6);
    if (cell->mib.valid)
        gg::eprint(" BW=%s SFN=%u", lte_bw_string(cell->mib.dl_bandwidth), cell->mib.sfn);
    if (cell->sib1.valid)
        fprintf(stderr, " MCC=%u MNC=%u TAC=%u CellID=%u",
                cell->sib1.mcc, cell->sib1.mnc, cell->sib1.tac, cell->sib1.cell_id);
    if (cell->sib2.valid)
        fprintf(stderr, " RACH=%upream maxHARQ=%u TA=%u",
                cell->sib2.ra_preambles,
                cell->sib2.max_harq_msg3_tx,
                cell->sib2.time_alignment_timer_sf);
    if (cell->sib3.valid)
        fprintf(stderr, " Qhyst=%udB QRxMin=%d Prio=%u",
                cell->sib3.q_hyst_db,
                cell->sib3.q_rxlevmin,
                cell->sib3.cell_reselection_priority);
    gg::eprint(" SNR=%.1fdB\n", cell->snr_db);
}

static void *lte_ifile_reader_thread(void *arg)
{
    (void)arg;

    const char *path = LteIfileConfig.ifile_path.c_str();
    const uint32_t buf_size = 262144;
    uint8_t *buf = static_cast<uint8_t*>(malloc(buf_size));
    if (!buf) {
        gg::eprint("lte-ifile: failed to allocate read buffer\n");
        return NULL;
    }

    gg::eprint("lte-ifile: reader thread started, file=%s\n", path);

    FILE *fp = fopen(path, "rb");
    if (!fp) {
        gg::eprint("lte-ifile: cannot open '%s': %s\n", path, strerror(errno));
        free(buf);
        return NULL;
    }

    while (!LteIfile.stop_flag && !Modes.exit) {
        size_t nread = fread(buf, 1, buf_size, fp);
        if (nread == 0) break;
        nread &= ~(size_t)1;
        if (LteIfile.demod && nread > 0)
            lte_process(LteIfile.demod, buf, (uint32_t)nread);

        double samples = nread / 2.0;
        double sleep_us = (samples / LTE_IFILE_SAMPLE_RATE) * 1e6;
        if (sleep_us > 0) {
            struct timespec ts;
            ts.tv_sec = (time_t)(sleep_us / 1e6);
            ts.tv_nsec = (int64_t)(fmod(sleep_us, 1e6) * 1000);
            nanosleep(&ts, NULL);
        }
    }

    fclose(fp);
    free(buf);

    lte_stats_t stats;
    lte_get_stats(LteIfile.demod, &stats);
    fprintf(stderr, "lte-ifile: done. samples=%" PRIu64 " pss=%u sss=%u mib=%u sib1=%u crc_err=%u\n",
            (uint64_t)stats.samples_processed,
            stats.pss_detected, stats.sss_decoded,
            stats.mib_decoded, stats.sib1_decoded, stats.crc_errors);

    Modes.exit = 1;
    return NULL;
}

//
// ============================= Utility functions ==========================
//

static void log_with_timestamp(const char *format, ...) __attribute__((format (printf, 1, 2) ));

static void log_with_timestamp(const char *format, ...)
{
    char timebuf[128];
    char msg[1024];
    time_t now;
    struct tm local;
    va_list ap;

    now = time(NULL);
    localtime_r(&now, &local);
    strftime(timebuf, 128, "%c %Z", &local);
    timebuf[127] = 0;

    va_start(ap, format);
    vsnprintf(msg, 1024, format, ap);
    va_end(ap);
    msg[1023] = 0;

    gg::eprint("%s  %s\n", timebuf, msg);

    // Push to panel log ring buffer
    if (PanelState.enabled) {
        panelLog("%s", msg);
    }
}

static void sigintHandler(int32_t dummy) {
    MODES_NOTUSED(dummy);
    signal(SIGINT, SIG_DFL);  // reset signal handler - bit extra safety
    Modes.exit = 1;           // Signal to threads that we are done
    const char msg[] = "Caught SIGINT, shutting down..\n";
    (void)!write(STDERR_FILENO, msg, sizeof(msg) - 1);
}

static void sigtermHandler(int32_t dummy) {
    MODES_NOTUSED(dummy);
    signal(SIGTERM, SIG_DFL); // reset signal handler - bit extra safety
    Modes.exit = 1;           // Signal to threads that we are done
    // NOTE: do NOT call fprintf/log_with_timestamp here — not async-signal-safe
    const char msg[] = "Caught SIGTERM, shutting down..\n";
    (void)!write(STDERR_FILENO, msg, sizeof(msg) - 1);
}

void receiverPositionChanged(float lat, float lon, float alt)
{
    log_with_timestamp("Autodetected receiver location: %.5f, %.5f at %.0fm AMSL", lat, lon, alt);
    writeJsonToFile("receiver.json", generateReceiverJson); // location changed
}


//
// =============================== Initialization ===========================
//
static void modesInitConfig(void) {
    // Default everything to zero/NULL
    memset(reinterpret_cast<void*>(&Modes), 0, sizeof(Modes));
    new (&Modes.exit) atomic_int{0};

    // Now initialise things that should not be 0/NULL to their defaults
    Modes.gain                    = MODES_DEFAULT_GAIN;
    Modes.freq                    = MODES_DEFAULT_FREQ;
    Modes.fix_df                  = 1;
    Modes.enable_df24             = 1;  // ELM/CPDLC reassembly on by default
    Modes.interactive_display_ttl = MODES_INTERACTIVE_DISPLAY_TTL;
    Modes.json_interval           = 1000;
    Modes.json_stats_interval     = 60000;
    Modes.json_location_accuracy  = 1;
    Modes.maxRange                = 1852 * 300; // 300NM default max range
    Modes.mode_ac_auto            = 1;

    Modes.net_heartbeat_interval = MODES_NET_HEARTBEAT_INTERVAL;
    Modes.net_output_flush_size = 1300;
    Modes.net_output_flush_interval = 500;

    // adaptive
    Modes.adaptive_min_gain_db = 0;
    Modes.adaptive_max_gain_db = 99999;

    Modes.adaptive_duty_cycle = 0.5;

    Modes.adaptive_burst_control = false;
    Modes.adaptive_burst_alpha = 2.0 / (5 + 1);
    Modes.adaptive_burst_change_delay = 5;
    Modes.adaptive_burst_loud_runlength = 10;
    Modes.adaptive_burst_loud_rate = 5.0;
    Modes.adaptive_burst_quiet_runlength = 10;
    Modes.adaptive_burst_quiet_rate = 5.0;

    Modes.adaptive_range_control = false;
    Modes.adaptive_range_alpha = 2.0 / (5 + 1);
    Modes.adaptive_range_percentile = 40;
    Modes.adaptive_range_change_delay = 10;
    Modes.adaptive_range_scan_delay = 300;
    Modes.adaptive_range_rescan_delay = 3600;

    Modes.beast_reduce_interval = 250;  // 250ms default BeastReduce interval

    sdrInitConfig();
    flarmReaderInitConfig();
    PocsagConfig = {};
    PocsagConfig.freq = 466150000;  // Center for multi-channel (covers 466.0-466.3 MHz)
    openskyClientInit();
    sondehubClientInit();
    panelInitConfig();

    // Airframes.io feed defaults (disabled until --airframes-acars/--airframes-vdl2 or panel)
    Modes.airframes_acars_feed.host = strdup("feed.acars.io");
    Modes.airframes_acars_feed.port = 5550;
    Modes.airframes_vdl2_feed.host  = strdup("feed.acars.io");
    Modes.airframes_vdl2_feed.port  = 5552;
    snprintf(Modes.airframes_station_id, sizeof(Modes.airframes_station_id), "dump1090-gg");
}
//
//=========================================================================
//
static void modesInit(void) {
    int32_t i;

    Modes.sample_rate = 2400000.0;

    // Allocate the various buffers used by Modes
    Modes.trailing_samples = (MODES_PREAMBLE_US + MODES_LONG_MSG_BITS + 16) * 1e-6 * Modes.sample_rate;

    if ( ((Modes.log10lut   = (uint16_t *) malloc(sizeof(uint16_t) * 256 * 256)                                 ) == NULL) )
    {
        gg::eprint("Out of memory allocating data buffer.\n");
        exit(1);
    }

    if (!fifo_create(MODES_MAG_BUFFERS, MODES_MAG_BUF_SAMPLES + Modes.trailing_samples, Modes.trailing_samples)) {
        gg::eprint("Out of memory allocating FIFO\n");
        exit(1);
    }

    // Validate the users Lat/Lon home location inputs
    if ( (Modes.fUserLat >   90.0)  // Latitude must be -90 to +90
      || (Modes.fUserLat <  -90.0)  // and
      || (Modes.fUserLon >  360.0)  // Longitude must be -180 to +360
      || (Modes.fUserLon < -180.0) ) {
        Modes.fUserLat = Modes.fUserLon = 0.0;
    } else if (Modes.fUserLon > 180.0) { // If Longitude is +180 to +360, make it -180 to 0
        Modes.fUserLon -= 360.0;
    }
    // If both Lat and Lon are 0.0 then the users location is either invalid/not-set, or (s)he's in the
    // Atlantic ocean off the west coast of Africa. This is unlikely to be correct.
    // Set the user LatLon valid flag only if either Lat or Lon are non zero. Note the Greenwich meridian
    // is at 0.0 Lon,so we must check for either fLat or fLon being non zero not both.
    // Testing the flag at runtime will be much quicker than ((fLon != 0.0) || (fLat != 0.0))
    Modes.bUserFlags &= ~MODES_USER_LATLON_VALID;
    if ((Modes.fUserLat != 0.0) || (Modes.fUserLon != 0.0)) {
        Modes.bUserFlags |= MODES_USER_LATLON_VALID;
    }

    // Limit the maximum requested raw output size to less than one Ethernet Block
    if (Modes.net_output_flush_size > (MODES_OUT_FLUSH_SIZE))
      {Modes.net_output_flush_size = MODES_OUT_FLUSH_SIZE;}
    if (Modes.net_output_flush_interval > (MODES_OUT_FLUSH_INTERVAL))
      {Modes.net_output_flush_interval = MODES_OUT_FLUSH_INTERVAL;}
    if (Modes.net_sndbuf_size > (MODES_NET_SNDBUF_MAX))
      {Modes.net_sndbuf_size = MODES_NET_SNDBUF_MAX;}

    // Prepare the log10 lookup table: 100log10(x)
    Modes.log10lut[0] = 0; // poorly defined..
    for (i = 1; i <= 65535; i++) {
        Modes.log10lut[i] = (uint16_t) round(100.0 * log10(i));
    }

    // Prepare error correction tables
    modesChecksumInit(Modes.nfix_crc);
    icaoFilterInit();
    modeACInit();

    if (Modes.show_only)
        icaoFilterAdd(Modes.show_only);

    // Initialize ELM (Comm-D) reassembly
    elmInit(&Modes.elm);
}

//
//=========================================================================
//
// We use a thread reading data in background, while the main thread
// handles decoding and visualization of data to the user.
//
// The reading thread calls the RTLSDR API to read data asynchronously, and
// uses a callback to populate the data buffer.
//
// A Mutex is used to avoid races with the decoding thread.
//

//
//=========================================================================
//
// We read data using a thread, so the main thread only handles decoding
// without caring about data acquisition
//

//
// ============================== Snip mode =================================
//
// Get raw IQ samples and filter everything is < than the specified level
// for more than 256 samples in order to reduce example file size
//
static void snipMode(int32_t level) {
    int32_t i, q;
    uint64_t c = 0;

    while ((i = getchar()) != EOF && (q = getchar()) != EOF) {
        if (abs(i-127) < level && abs(q-127) < level) {
            c++;
            if (c > MODES_PREAMBLE_SIZE) continue;
        } else {
            c = 0;
        }
        putchar(i);
        putchar(q);
    }
}
//
// ================================ Main ====================================
//
static void showVersion()
{
    gg::print("-----------------------------------------------------------------------------\n");
    gg::print("| dump1090 ModeS Receiver     %45s |\n", MODES_DUMP1090_VARIANT " " MODES_DUMP1090_VERSION);
    printf("| build options: %-58s |\n",
           ""
#ifdef ENABLE_RTLSDR
           "ENABLE_RTLSDR "
#endif
#ifdef ENABLE_BLADERF
           "ENABLE_BLADERF "
#endif
#ifdef ENABLE_HACKRF
           "ENABLE_HACKRF "
#endif
#ifdef ENABLE_LIMESDR
           "ENABLE_LIMESDR "
#endif
           );
    gg::print("-----------------------------------------------------------------------------\n");
}

static void showDSP()
{
    gg::print("  detected runtime CPU features: ");
    if (cpu_supports_avx())
        gg::print("AVX ");
    if (cpu_supports_avx2())
        gg::print("AVX2 ");
    if (cpu_supports_armv7_neon_vfpv4())
        gg::print("ARMv7+NEON+VFPv4 ");
    gg::print("\n");

    gg::print("  selected DSP implementations: \n");
#define SHOW(x) do {                                                    \
        gg::print("    %-40s %s\n", #x , starch_ ## x ## _select()->name);  \
        gg::print("    %-40s %s\n", #x "_aligned", starch_ ## x ## _aligned_select()->name); \
    } while(0)

    SHOW(magnitude_uc8);
    SHOW(magnitude_power_uc8);
    SHOW(magnitude_sc16);
    SHOW(magnitude_sc16q11);
    SHOW(mean_power_u16);
    SHOW(count_above_u16);

#undef SHOW

    gg::print("\n");
}

static void showHelp(void)
{
    showVersion();

    sdrShowHelp();

    printf(
"      Output modes\n"
"\n"
// ------ 80 char limit ----------------------------------------------------------|
"--raw                    Show only messages hex values\n"
"--modeac                 Enable decoding of SSR Modes 3/A & 3/C\n"
"--mlat                   display raw messages in Beast ascii mode\n"
"--onlyaddr               Show only ICAO addresses (testing purposes)\n"
"--metric                 Use metric units (meters, km/h, ...)\n"
"--gnss                   Show altitudes as HAE/GNSS when available\n"
"--tisb-verbose           Log TIS-B/ADS-R messages (DF18) to stderr\n"
"--crc-rescue             Enable CRC-based message rescue (recover corrupted preambles)\n"
"--quiet                  Disable output to stdout. Use for daemon applications\n"
"--no-saved-config        Skip decoders.json, receivers.json and saved panel/feed state\n"
"--show-only <addr>       Show only messages from the given ICAO on stdout\n"
"--snip <level>           Strip IQ file removing samples < level\n"
"\n"
"      Decoder settings\n"
"\n"
// ------ 80 char limit ----------------------------------------------------------|
"--gain <db>              Set gain in dB (default: varies by SDR type)\n"
"--freq <hz>              Set frequency (default: 1090 Mhz)\n"
"--fix                    Enable single-bit error correction using CRC\n"
"--fix-2bit               Enable two-bit error correction using CRC\n"
"                          (use with caution!)\n"
"--no-fix                 Disable error correction using CRC\n"
"--no-fix-df              Disable error correction of the DF message field\n"
"                          (reduces CPU requirements)\n"
"--enable-df24            Enable decoding of DF24 Comm-D ELM messages\n"
"--lat <latitude>         Reference/receiver latitude for surface positions\n"
"--lon <longitude>        Reference/receiver longitude for surface positions\n"
"--max-range <distance>   Absolute maximum range for position decoding (in NM)\n"
"\n"
// ------ 80 char limit ----------------------------------------------------------|
"      Adaptive gain\n"
"\n"
"--adaptive-burst                     Adjust gain for too-loud message bursts\n"
"--adaptive-burst-change-delay <s>     Set delay after changing gain before\n"
"                                       resuming burst control (seconds)\n"
"--adaptive-burst-alpha <a>            Set burst rate smoothing factor\n"
"                                       (0..1, smaller=more smoothing)\n"
"--adaptive-burst-loud-rate <r>        Set burst rate for gain decrease\n"
"--adaptive-burst-loud-runlength <l>   Set burst runlength for gain decrease\n"
"--adaptive-burst-quiet-rate <r>       Set burst rate for gain increase\n"
"--adaptive-burst-quiet-runlength <l>  Set burst runlength for gain increase\n"
"--adaptive-range                     Adjust gain for target dynamic range\n"
"--adaptive-range-target <db>          Set target dynamic range in dB\n"
"--adaptive-range-alpha <a>            Set dynamic range noise smoothing factor\n"
"                                       (0..1, smaller=more smoothing)\n"
"--adaptive-range-percentile <p>       Set dynamic range noise percentile\n"
"--adaptive-range-change-delay <s>     Set delay after changing gain before\n"
"                                       resuming dynamic range control (seconds)\n"
"--adaptive-range-scan-delay <s>       Set scan interval for dynamic range\n"
"                                       gain scanning following a gain decrease\n"
"                                       due to an increase in noise (seconds)\n"
"--adaptive-range-rescan-delay <s>     Set periodic rescan interval for dynamic\n"
"                                       range gain scanning (seconds)\n"
"--adaptive-min-gain <g>              Set gain adjustment range lower limit (dB)\n"
"--adaptive-max-gain <g>              Set gain adjustment range upper limit (dB)\n"
"--adaptive-duty-cycle <p>            Set adaptive gain duty cycle %% (1..100)\n"
"\n"
// ------ 80 char limit ----------------------------------------------------------|
"      Network connections\n"
"\n"
"--net                    Enable networking with default ports unless overridden\n"
"--no-modeac-auto         Don't enable Mode A/C if requested by a net connection\n"
"--net-only               Enable just networking, no RTL device or file used\n"
"--net-bind-address <ip>  IP address to bind to (use 127.0.0.1 for private)\n"
"--net-ri-port <ports>    TCP raw input listen ports  (default: 30001)\n"
"--net-ro-port <ports>    TCP raw output listen ports (default: 30002)\n"
"--net-sbs-port <ports>   TCP BaseStation output listen ports (default: 30003)\n"
"--net-bi-port <ports>    TCP Beast input listen ports  (default: 30004,30104)\n"
"--net-bo-port <ports>    TCP Beast output listen ports (default: 30005)\n"
"--net-stratux-port <ports>  TCP Stratux output listen ports (default: disabled)\n"
"--net-ro-size <size>     TCP output minimum size (default: 0)\n"
"--net-ro-interval <rate> TCP output memory flush rate in seconds (default: 0)\n"
"--net-heartbeat <rate>   TCP heartbeat rate in seconds\n"
"                          (default: 60 sec; 0 to disable)\n"
"--net-buffer <n>         TCP buffer size 64Kb * (2^n) (default: n=0, 64Kb)\n"
"--net-verbatim           Make output connections default to verbatim mode\n"
"                           (forward all messages without correction)\n"
"--forward-mlat           Allow forwarding of received mlat results\n"
"\n"
"      Beast feed outputs (ADS-B data sharing networks)\n"
"      BeastReduce: per-aircraft rate limiting for bandwidth-efficient feeding.\n"
"      All beast feeds use BeastReduce by default (250ms interval).\n"
"\n"
"--beast-reduce-interval <ms> BeastReduce min interval in ms (default: 250, 0=off)\n"
"\n"
"--adsbx                  Enable ADSBexchange feed (feed.adsbexchange.com:30005)\n"
"--adsbx-host <host>      ADSBexchange feed host override\n"
"--adsbx-port <port>      ADSBexchange feed port override\n"
"--opensky                Enable OpenSky Network native feed (collector.opensky-network.org:10004)\n"
"--opensky-user <user>    OpenSky Network username\n"
"--opensky-serial <n>     OpenSky serial number (auto-assigned if not set)\n"
"--opensky-alt <meters>   Receiver altitude for OpenSky (defaults to --mlat-alt)\n"
"--opensky-host <host>    OpenSky collector host override\n"
"--opensky-port <port>    OpenSky collector port override\n"
"--adsbfi                 Enable adsb.fi feed (feed.adsb.fi:30004)\n"
"--adsbfi-host <host>     adsb.fi feed host override\n"
"--adsbfi-port <port>     adsb.fi feed port override\n"
"--flyitalyadsb           Enable Fly Italy ADSB feed (dati.flyitalyadsb.com:4905)\n"
"--flyitalyadsb-host <h>  Fly Italy ADSB feed host override\n"
"--flyitalyadsb-port <p>  Fly Italy ADSB feed port override\n"
"--planewatch             Enable plane.watch feed (atc.plane.watch:30004)\n"
"--planewatch-host <host> plane.watch feed host override\n"
"--planewatch-port <port> plane.watch feed port override\n"
"--adsbone                Enable adsb.one feed (feed.adsb.one:64004)\n"
"--adsbone-host <host>    adsb.one feed host override\n"
"--adsbone-port <port>    adsb.one feed port override\n"
"--adsblol                Enable adsb.lol feed (feed.adsb.lol:30004)\n"
"--adsblol-host <host>    adsb.lol feed host override\n"
"--adsblol-port <port>    adsb.lol feed port override\n"
"--airplaneslive          Enable airplanes.live feed (feed.airplanes.live:30004)\n"
"--airplaneslive-host <h> airplanes.live feed host override\n"
"--airplaneslive-port <p> airplanes.live feed port override\n"
"--planespotters          Enable Planespotters feed (feed.planespotters.net:30004)\n"
"--planespotters-host <h> Planespotters feed host override\n"
"--planespotters-port <p> Planespotters feed port override\n"
"--theairtraffic          Enable TheAirTraffic feed (feed.theairtraffic.com:30004)\n"
"--theairtraffic-host <h> TheAirTraffic feed host override\n"
"--theairtraffic-port <p> TheAirTraffic feed port override\n"
"--avdelphi               Enable AVDelphi feed (data.avdelphi.com:24999)\n"
"--avdelphi-host <host>   AVDelphi feed host override\n"
"--avdelphi-port <port>   AVDelphi feed port override\n"
"--adsbhub                Enable ADSBHub feed (data.adsbhub.org:5001)\n"
"--adsbhub-host <host>    ADSBHub feed host override\n"
"--adsbhub-port <port>    ADSBHub feed port override\n"
"--adsbhub-ckey <key>     ADSBHub station ckey for dynamic IP registration\n"
"--adsbhub-ckey-file <f>  Read ADSBHub ckey from file (avoids shell escaping)\n"
"\n"
"      Airframes.io ACARS/VDL2 feeds (UDP JSON)\n"
"\n"
"--airframes-acars        Enable ACARS feed to airframes.io (feed.acars.io:5550)\n"
"--airframes-acars-host <h> ACARS feed host override\n"
"--airframes-acars-port <p> ACARS feed port override\n"
"--airframes-vdl2         Enable VDL2 feed to airframes.io (feed.acars.io:5552)\n"
"--airframes-vdl2-host <h>  VDL2 feed host override\n"
"--airframes-vdl2-port <p>  VDL2 feed port override\n"
"--airframes-station-id <s> Station identifier for airframes.io\n"
"\n"
"      Built-in MLAT client\n"
"\n"
"--mlat-server <host:port>  MLAT server address (can be repeated, max 4)\n"
"--mlat-user <name>         Feeder name for MLAT servers\n"
"--mlat-uuid-file <path>    Path to file containing feeder UUID\n"
"--mlat-lat <deg>           Receiver latitude for MLAT (default: --lat value)\n"
"--mlat-lon <deg>           Receiver longitude for MLAT (default: --lon value)\n"
"--mlat-alt <meters>        Receiver altitude in meters for MLAT\n"
"--mlat-no-results          Don't request MLAT results back from server\n"
"\n"
"      Built-in FlightAware/PiAware client\n"
"\n"
"--piaware                    Enable built-in PiAware ADEPT client\n"
"--piaware-feeder-id <uuid>   FlightAware feeder ID (UUID), avoids needing piaware installed\n"
"--piaware-feeder-id-file <p> Path to feeder ID file (default: /var/cache/piaware/feeder_id)\n"
"--piaware-ca-dir <path>      Path to FlightAware CA certs (default: /usr/lib/piaware_packages/ca)\n"
"\n"
"      FLARM 868 MHz decoder\n"
"\n"
"--flarm                      Enable FLARM 868 MHz decoder\n"
"--flarm-device <serial>      RTL-SDR serial number for 868 MHz dongle\n"
"--flarm-gain <dB>            Gain in dB (0 = auto, default: auto)\n"
"--flarm-ppm <correction>     Frequency correction in PPM\n"
"--flarm-keys <file>          Load FLARM decryption keys from file\n"
"--ogn-station <name>         OGN station name for APRS-IS feed\n"
"--ogn-server <host>          OGN APRS-IS server (default: aprs.glidernet.org)\n"
"--ogn-port <port>            OGN APRS-IS port (default: 14580)\n"
"\n"
"      POCSAG pager decoder\n"
"\n"
"--pocsag                     Enable POCSAG pager decoder (multi-channel 466.0-466.3 MHz)\n"
"--pocsag-device <serial>     RTL-SDR serial number for POCSAG dongle\n"
"--pocsag-ifile <path>        Replay raw IQ file (uint8 I/Q pairs, 2.4 MSPS)\n"
"--pocsag-freq <Hz>           POCSAG center frequency in Hz (default: 466150000)\n"
"--pocsag-gain <dB>           Gain in dB (0 = auto, default: auto)\n"
"--pocsag-ppm <correction>    Frequency correction in PPM\n"
"\n"
"--sondehub <callsign>        Enable SondeHub upload with this callsign\n"
"--sonde-ifile <path>         Replay raw IQ file through RS41 decoder (uint8 I/Q, 2.4 MSPS)\n"
"--sonde-freq <Hz>            Sonde center frequency in Hz (default: 403000000)\n"
"--acars-ifile <path>         Replay raw IQ file through ACARS decoder (uint8 I/Q, 2.4 MSPS)\n"
"--acars-freq <Hz>            ACARS center frequency in Hz (default: 131550000)\n"
"--gsm-ifile <path>           Replay raw IQ file through GSM decoder (uint8 I/Q, 1 MSPS)\n"
"--gsm-freq <Hz>              GSM ARFCN frequency in Hz (default: 939200000)\n"
"--lte-ifile <path>           Replay raw IQ file through LTE decoder (uint8 I/Q, 1.92 MSPS)\n"
"--lte-freq <Hz>              LTE center frequency in Hz (default: 806000000)\n"
"\n"
"      Multi-SDR dynamic receiver management\n"
"\n"
"--receiver <spec>            Add a receiver: serial:role[:gain=X][:ppm=Y][:agc]\n"
"                             role is 'adsb', 'flarm', 'acars', 'vdl2',\n"
"                             'radiosonde', 'pocsag', 'gsm', or 'lte'.\n"
"                             Example: --receiver 00000101:adsb:gain=40\n"
"                                      --receiver 00000102:flarm:gain=30\n"
"\n"
// ------ 80 char limit ----------------------------------------------------------|
"      Stats and json output\n"
"\n"
"--stats                  Show stats summary at exit.\n"
"--stats-every <seconds>  Show and reset stats every <seconds> seconds\n"
"--stats-range            Collect/show range histogram\n"
"--write-json <dir>       Periodically write json output to <dir>\n"
"                          (for serving by a separate webserver)\n"
"--write-json-every <t>   Write json aircraft output every t seconds (default 1)\n"
"--json-stats-every <t>   Write json stats output every t seconds (default 60)\n"
"--json-location-accuracy <n>  Accuracy of receiver location in json metadata\n"
"                          (0=no location, 1=approximate, 2=exact)\n"
"\n"
"      Interactive mode\n"
"\n"
"--interactive                       Interactive mode refreshing data on screen.\n"
"                                     Implies --throttle\n"
"--interactive-ttl <sec>             Remove from list if idle for <sec>\n"
"--interactive-show-distance         Show aircraft distance and bearing\n"
"                                     (requires --lat and --lon)\n"
"--interactive-distance-units <u>    Distance units ('km', 'sm', 'nm')\n"
"--interactive-callsign-filter <r>   Filter rows by callsign against regex\n"
"\n"
"      Misc\n"
"\n"
"      Control panel\n"
"\n"
"--panel                      Enable built-in web control panel\n"
"--panel-port <port>          Panel HTTP port (default: 8888)\n"
"--panel-password <password>  Panel password (user: admin, default: none)\n"
"--panel-html-dir <dir>       Panel HTML directory (default: /usr/share/dump1090-gg/panel)\n"
"\n"
"      Misc\n"
"\n"
"--wisdom <path>          Read DSP wisdom from given path\n"
"--version                Show version, build and DSP options\n"
"--help                   Show this help\n"
    );
}

// Accumulate stats data from stats_current to stats_periodic, stats_alltime and stats_latest;
// reset stats_current
void flush_stats(uint64_t now);
void flush_stats(uint64_t now)
{
    if (Modes.sdr_type != SDR_NONE) {
        Modes.stats_current.sdr_gain = sdrGetGain();
    }

    add_stats(&Modes.stats_current, &Modes.stats_periodic, &Modes.stats_periodic);
    add_stats(&Modes.stats_current, &Modes.stats_alltime, &Modes.stats_alltime);
    add_stats(&Modes.stats_current, &Modes.stats_latest, &Modes.stats_latest);

    reset_stats(&Modes.stats_current);
    Modes.stats_current.start = Modes.stats_current.end = now;
}

//
//=========================================================================
//
// This function is called a few times every second by main in order to
// perform tasks we need to do continuously, like accepting new clients
// from the net, refreshing the screen in interactive mode, and so forth
//
static void backgroundTasks(void) {
    static uint64_t next_stats_display;
    static uint64_t next_stats_update;
    static uint64_t next_json_stats_update;
    static uint64_t next_json, next_history;

    uint64_t now = mstime();

    if (Modes.sdr_type != SDR_IFILE) {
        // don't run these if processing data from a file
        icaoFilterExpire();
        // trackPeriodicUpdate modifies aircraft list — protect for feeder threads
        pthread_rwlock_wrlock(&aircraft_lock);
        trackPeriodicUpdate();
        pthread_rwlock_unlock(&aircraft_lock);

        // Diagnostic: check receiver RF health (detects 0-message condition)
        rxDiagHealthCheck();
        elmCleanupStale(&Modes.elm, now);
    }

    if (Modes.net) {
        modesNetPeriodicWork();
    }

    // OGN and PiAware and MLAT now run in their own feeder threads
    // (see feeder_thread.c)

    // Process MLAT results injected back from the MLAT feeder thread
    feederProcessInjectedMessages();

    // Process FLARM messages from standalone reader (--flarm-ifile)
    if (FlarmConfig.enabled && FlarmConfig.ifile_path[0]) {
        flarmReaderPeriodicWork();
    }

    // Poll the C++ dispatcher: drain all decoder queues → aircraft list + APIs
    // Must come after all producers (feeders, FLARM, SDR drains) have pushed.
    dispatcher_poll();


    // Refresh screen when in interactive mode
    if (Modes.interactive) {
        interactiveShowData();
    }

    // copy out reader CPU time and reset it
    sdrUpdateCPUTime(&Modes.stats_current.reader_cpu);

    // always update end time so it is current when requests arrive
    Modes.stats_current.end = mstime();

    // 1-minute stats update
    if (now >= next_stats_update) {
        int32_t i;

        if (next_stats_update == 0) {
            next_stats_update = now + 60000;
        } else {
            flush_stats(now); // Ensure stats_latest is up to date

            // move stats_latest into 1-min ring buffer
            Modes.stats_newest_1min = (Modes.stats_newest_1min + 1) % 15;
            Modes.stats_1min[Modes.stats_newest_1min] = Modes.stats_latest;
            reset_stats(&Modes.stats_latest);

            // recalculate 5-min window
            reset_stats(&Modes.stats_5min);
            for (i = 0; i < 5; ++i)
                add_stats(&Modes.stats_1min[(Modes.stats_newest_1min - i + 15) % 15], &Modes.stats_5min, &Modes.stats_5min);

            // recalculate 15-min window
            reset_stats(&Modes.stats_15min);
            for (i = 0; i < 15; ++i)
                add_stats(&Modes.stats_1min[i], &Modes.stats_15min, &Modes.stats_15min);

            next_stats_update += 60000;
        }
    }

    // --stats-every display
    if (Modes.stats && now >= next_stats_display) {
        if (next_stats_display == 0) {
            next_stats_display = now + Modes.stats;
        } else {
            flush_stats(now); // Ensure stats_periodic is up to date

            display_stats(&Modes.stats_periodic);
            reset_stats(&Modes.stats_periodic);

            next_stats_display += Modes.stats;
            if (next_stats_display <= now) {
                /* something has gone wrong, perhaps the system clock jumped */
                next_stats_display = now + Modes.stats;
            }
        }
    }

    // json stats update
    if (Modes.json_dir && now >= next_json_stats_update) {
        if (next_json_stats_update == 0) {
            next_json_stats_update = now + Modes.json_stats_interval;
        } else {
            flush_stats(now); // Ensure everything we'll write is up to date
            writeJsonToFile("stats.json", generateStatsJson);
            next_json_stats_update += Modes.json_stats_interval;
        }
    }

    if (Modes.json_dir && now >= next_json) {
        writeJsonToFile("aircraft.json", generateAircraftJson);
        next_json = now + Modes.json_interval;
    }

    if (now >= next_history) {
        int32_t rewrite_receiver_json = (Modes.json_dir && Modes.json_aircraft_history[HISTORY_SIZE-1].content == NULL);

        free(Modes.json_aircraft_history[Modes.json_aircraft_history_next].content); // might be NULL, that's OK.
        Modes.json_aircraft_history[Modes.json_aircraft_history_next].content =
            generateAircraftJson("/data/aircraft.json", &Modes.json_aircraft_history[Modes.json_aircraft_history_next].clen);

        if (Modes.json_dir) {
            char filebuf[PATH_MAX];
            snprintf(filebuf, PATH_MAX, "history_%d.json", Modes.json_aircraft_history_next);
            writeJsonToFile(filebuf, generateHistoryJson);
        }

        Modes.json_aircraft_history_next = (Modes.json_aircraft_history_next+1) % HISTORY_SIZE;

        if (rewrite_receiver_json)
            writeJsonToFile("receiver.json", generateReceiverJson); // number of history entries changed

        next_history = now + HISTORY_INTERVAL;
    }
}

//
//=========================================================================
//
//
//=========================================================================
//

static void applyNetDefaults()
{
    if (!Modes.net_input_raw_ports)
        Modes.net_input_raw_ports = strdup("30001");
    if (!Modes.net_output_raw_ports)
        Modes.net_output_raw_ports = strdup("30002");
    if (!Modes.net_output_sbs_ports)
        Modes.net_output_sbs_ports = strdup("30003");
    if (!Modes.net_input_beast_ports)
        Modes.net_input_beast_ports = strdup("30004,30104");
    if (!Modes.net_output_beast_ports)
        Modes.net_output_beast_ports = strdup("30005");
}

// Add or find a beast feed entry by name, returning its index.
// If the feed already exists (by name), returns its index.
// If new, initializes with given defaults.
static int32_t addBeastFeed(const char *name, const char *default_host, int32_t default_port) {
    // Check if already exists
    for (int32_t i = 0; i < Modes.beast_feed_count; i++) {
        if (!strcmp(Modes.beast_feeds[i].name, name))
            return i;
    }
    if (Modes.beast_feed_count >= MAX_BEAST_FEEDS) {
        gg::eprint("Too many beast feeds (max %d)\n", MAX_BEAST_FEEDS);
        exit(1);
    }
    int32_t idx = Modes.beast_feed_count++;
    snprintf(Modes.beast_feeds[idx].name, sizeof(Modes.beast_feeds[idx].name), "%s", name);
    Modes.beast_feeds[idx].host = strdup(default_host);
    Modes.beast_feeds[idx].port = default_port;
    Modes.beast_feeds[idx].format = FEED_FORMAT_BEAST_REDUCE;
    Modes.beast_feeds[idx].enabled = 1;
    return idx;
}

static void stopStandaloneIfileReaders(void)
{
    // Stop standalone FLARM reader (ifile mode)
    if (FlarmConfig.enabled && FlarmConfig.ifile_path[0]) {
        flarmReaderClose();
    }

    // Stop standalone POCSAG reader (ifile mode)
    if (PocsagIfile.thread_running) {
        PocsagIfile.stop_flag = 1;
        pthread_join(PocsagIfile.thread, NULL);
        PocsagIfile.thread_running = 0;
        if (PocsagIfile.demod) {
            pocsag_destroy(PocsagIfile.demod);
            PocsagIfile.demod = NULL;
        }
        gg::eprint("pocsag-ifile: reader stopped\n");
    }

    // Stop standalone Sonde reader (ifile mode)
    if (SondeIfile.thread_running) {
        SondeIfile.stop_flag = 1;
        pthread_join(SondeIfile.thread, NULL);
        SondeIfile.thread_running = 0;
        if (SondeIfile.demod) {
            sonde_destroy(SondeIfile.demod);
            SondeIfile.demod = NULL;
        }
        gg::eprint("sonde-ifile: reader stopped\n");
    }

    // Stop standalone ACARS reader (ifile mode)
    if (AcarsIfile.thread_running) {
        AcarsIfile.stop_flag = 1;
        pthread_join(AcarsIfile.thread, NULL);
        AcarsIfile.thread_running = 0;
        if (AcarsIfile.demod) {
            acars_destroy(AcarsIfile.demod);
            AcarsIfile.demod = NULL;
        }
        gg::eprint("acars-ifile: reader stopped\n");
    }

    // Stop standalone GSM reader (ifile mode)
    if (GsmIfile.thread_running) {
        GsmIfile.stop_flag = 1;
        pthread_join(GsmIfile.thread, NULL);
        GsmIfile.thread_running = 0;
        if (GsmIfile.demod) {
            gsm_destroy(GsmIfile.demod);
            GsmIfile.demod = NULL;
        }
        gg::eprint("gsm-ifile: reader stopped\n");
    }

    // Stop standalone LTE reader (ifile mode)
    if (LteIfile.thread_running) {
        LteIfile.stop_flag = 1;
        pthread_join(LteIfile.thread, NULL);
        LteIfile.thread_running = 0;
        if (LteIfile.demod) {
            lte_destroy(LteIfile.demod);
            LteIfile.demod = NULL;
        }
        gg::eprint("lte-ifile: reader stopped\n");
    }
}

int main(int argc, char **argv) {
    int32_t j;
    bool load_saved_config = true;

    for (j = 1; j < argc; j++) {
        if (!strcmp(argv[j], "--no-saved-config")) {
            load_saved_config = false;
            break;
        }
    }

    // Set sane defaults
    modesInitConfig();

    // Initialize multi-SDR receiver manager
    sdrManagerInit();

    // Initialize decoder configs with defaults, then load from file
    decoderConfigInit();
    if (load_saved_config) {
        if (!decoderConfigLoad()) {
            // JSON config doesn't exist yet - still try to load FLARM keys
            decoderConfigLoadFlarmKeys(DecoderConfigs.flarm.keys_file);
        }
    } else {
        gg::eprint("startup: skipping saved decoder config (--no-saved-config)\n");
    }

    // Initialize GSM cell tracker
    gsmTrackerInit();

    // Initialize LTE cell tracker
    lteTrackerInit();

    // Initialize IoT 868 MHz device tracker
    iotTrackerInit();

    // signal handlers:
    signal(SIGINT, sigintHandler);
    signal(SIGTERM, sigtermHandler);

    // Parse the command line options
    for (j = 1; j < argc; j++) {
        int32_t more = j+1 < argc; // There are more arguments

        if (!strcmp(argv[j],"--freq") && more) {
            Modes.freq = (int32_t) strtoll(argv[++j],NULL,10);
        } else if ( (!strcmp(argv[j], "--device") || !strcmp(argv[j], "--device-index")) && more) {
            Modes.dev_name = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--gain") && more) {
            Modes.gain = atof(argv[++j]);
        } else if (!strcmp(argv[j],"--dcfilter")) {
#if 0
            Modes.dc_filter = 1;
#else
            gg::eprint("--dcfilter option ignored (please raise an issue on github if you have a usecase that needs this)\n");
#endif
        } else if (!strcmp(argv[j],"--measure-noise")) {
            // Ignored
        } else if (!strcmp(argv[j],"--fix")) {
            if (Modes.nfix_crc < 1)
                Modes.nfix_crc = 1;
        } else if (!strcmp(argv[j],"--fix-2bit")) {
            Modes.nfix_crc = 2;
        } else if (!strcmp(argv[j],"--enable-df24")) {
            Modes.enable_df24 = 1;
        } else if (!strcmp(argv[j],"--no-fix")) {
            Modes.nfix_crc = 0;
        } else if (!strcmp(argv[j],"--no-fix-df")) {
            Modes.fix_df = 0;
        } else if (!strcmp(argv[j],"--no-crc-check")) {
            gg::eprint("warning: --no-crc-check no longer supported, option ignored (please raise an issue on github if you have a usecase that needs this)\n");
        } else if (!strcmp(argv[j],"--phase-enhance")) {
            // Ignored, always enabled
        } else if (!strcmp(argv[j],"--raw")) {
            Modes.raw = 1;
        } else if (!strcmp(argv[j],"--net")) {
            Modes.net = 1;
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--modeac")) {
            Modes.mode_ac = 1;
            Modes.mode_ac_auto = 0;
        } else if (!strcmp(argv[j],"--no-modeac-auto")) {
            Modes.mode_ac_auto = 0;
        } else if (!strcmp(argv[j],"--net-beast")) {
            gg::eprint("--net-beast ignored, use --net-bo-port to control where Beast output is generated\n");
        } else if (!strcmp(argv[j],"--net-only")) {
            Modes.net = 1;
            Modes.sdr_type = SDR_NONE;
            applyNetDefaults();
       } else if (!strcmp(argv[j],"--net-heartbeat") && more) {
            Modes.net_heartbeat_interval = (uint64_t)(1000 * atof(argv[++j]));
       } else if (!strcmp(argv[j],"--net-ro-size") && more) {
            Modes.net_output_flush_size = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-ro-rate") && more) {
            Modes.net_output_flush_interval = 1000 * atoi(argv[++j]) / 15; // backwards compatibility
        } else if (!strcmp(argv[j],"--net-ro-interval") && more) {
            Modes.net_output_flush_interval = (uint64_t)(1000 * atof(argv[++j]));
        } else if (!strcmp(argv[j],"--net-ro-port") && more) {
            Modes.net = 1;
            free(Modes.net_output_raw_ports);
            Modes.net_output_raw_ports = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-ri-port") && more) {
            Modes.net = 1;
            free(Modes.net_input_raw_ports);
            Modes.net_input_raw_ports = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bo-port") && more) {
            Modes.net = 1;
            free(Modes.net_output_beast_ports);
            Modes.net_output_beast_ports = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bi-port") && more) {
            Modes.net = 1;
            free(Modes.net_input_beast_ports);
            Modes.net_input_beast_ports = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bind-address") && more) {
            free(Modes.net_bind_address);
            Modes.net_bind_address = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-http-port") && more) {
            if (strcmp(argv[++j], "0")) {
                gg::eprint("warning: --net-http-port not supported in this build, option ignored.\n");
            }
        } else if (!strcmp(argv[j],"--net-sbs-port") && more) {
            Modes.net = 1;
            free(Modes.net_output_sbs_ports);
            Modes.net_output_sbs_ports = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-stratux-port") && more) {
            Modes.net = 1;
            free(Modes.net_output_stratux_ports);
            Modes.net_output_stratux_ports = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-buffer") && more) {
            Modes.net_sndbuf_size = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-verbatim")) {
            Modes.net_verbatim = 1;
        } else if (!strcmp(argv[j],"--forward-mlat")) {
            Modes.forward_mlat = 1;

        // BeastReduce interval
        } else if (!strcmp(argv[j],"--beast-reduce-interval") && more) {
            Modes.beast_reduce_interval = atoi(argv[++j]);

        // Beast feed networks
        } else if (!strcmp(argv[j],"--adsbx")) {
            Modes.net = 1;
            addBeastFeed("ADSBx", "feed.adsbexchange.com", 30005);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--adsbx-host") && more) {
            Modes.net = 1;
            int32_t idx = addBeastFeed("ADSBx", "feed.adsbexchange.com", 30005);
            free(Modes.beast_feeds[idx].host);
            Modes.beast_feeds[idx].host = strdup(argv[++j]);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--adsbx-port") && more) {
            int32_t idx = addBeastFeed("ADSBx", "feed.adsbexchange.com", 30005);
            Modes.beast_feeds[idx].port = atoi(argv[++j]);

        } else if (!strcmp(argv[j],"--opensky")) {
            Modes.net = 1;
            OpenSkyConfig.enabled = 1;
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--opensky-user") && more) {
            OpenSkyConfig.enabled = 1;
            OpenSkyConfig.username = argv[++j];
        } else if (!strcmp(argv[j],"--opensky-serial") && more) {
            OpenSkyConfig.enabled = 1;
            OpenSkyConfig.serial = (int32_t)atol(argv[++j]);
        } else if (!strcmp(argv[j],"--opensky-alt") && more) {
            OpenSkyConfig.enabled = 1;
            OpenSkyConfig.alt = atof(argv[++j]);
        } else if (!strcmp(argv[j],"--opensky-host") && more) {
            Modes.net = 1;
            OpenSkyConfig.enabled = 1;
            OpenSkyConfig.host = argv[++j];
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--opensky-port") && more) {
            OpenSkyConfig.enabled = 1;
            OpenSkyConfig.port = atoi(argv[++j]);

        } else if (!strcmp(argv[j],"--adsbfi")) {
            Modes.net = 1;
            addBeastFeed("adsb.fi", "feed.adsb.fi", 30004);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--adsbfi-host") && more) {
            Modes.net = 1;
            int32_t idx = addBeastFeed("adsb.fi", "feed.adsb.fi", 30004);
            free(Modes.beast_feeds[idx].host);
            Modes.beast_feeds[idx].host = strdup(argv[++j]);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--adsbfi-port") && more) {
            int32_t idx = addBeastFeed("adsb.fi", "feed.adsb.fi", 30004);
            Modes.beast_feeds[idx].port = atoi(argv[++j]);

        } else if (!strcmp(argv[j],"--flyitalyadsb")) {
            Modes.net = 1;
            addBeastFeed("FlyItaly", "dati.flyitalyadsb.com", 4905);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--flyitalyadsb-host") && more) {
            Modes.net = 1;
            int32_t idx = addBeastFeed("FlyItaly", "dati.flyitalyadsb.com", 4905);
            free(Modes.beast_feeds[idx].host);
            Modes.beast_feeds[idx].host = strdup(argv[++j]);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--flyitalyadsb-port") && more) {
            int32_t idx = addBeastFeed("FlyItaly", "dati.flyitalyadsb.com", 4905);
            Modes.beast_feeds[idx].port = atoi(argv[++j]);

        } else if (!strcmp(argv[j],"--planewatch")) {
            Modes.net = 1;
            addBeastFeed("PlaneWatch", "atc.plane.watch", 30004);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--planewatch-host") && more) {
            Modes.net = 1;
            int32_t idx = addBeastFeed("PlaneWatch", "atc.plane.watch", 30004);
            free(Modes.beast_feeds[idx].host);
            Modes.beast_feeds[idx].host = strdup(argv[++j]);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--planewatch-port") && more) {
            int32_t idx = addBeastFeed("PlaneWatch", "atc.plane.watch", 30004);
            Modes.beast_feeds[idx].port = atoi(argv[++j]);

        } else if (!strcmp(argv[j],"--adsbone")) {
            Modes.net = 1;
            addBeastFeed("adsb.one", "feed.adsb.one", 64004);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--adsbone-host") && more) {
            Modes.net = 1;
            int32_t idx = addBeastFeed("adsb.one", "feed.adsb.one", 64004);
            free(Modes.beast_feeds[idx].host);
            Modes.beast_feeds[idx].host = strdup(argv[++j]);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--adsbone-port") && more) {
            int32_t idx = addBeastFeed("adsb.one", "feed.adsb.one", 64004);
            Modes.beast_feeds[idx].port = atoi(argv[++j]);

        } else if (!strcmp(argv[j],"--adsblol")) {
            Modes.net = 1;
            addBeastFeed("adsb.lol", "feed.adsb.lol", 30004);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--adsblol-host") && more) {
            Modes.net = 1;
            int32_t idx = addBeastFeed("adsb.lol", "feed.adsb.lol", 30004);
            free(Modes.beast_feeds[idx].host);
            Modes.beast_feeds[idx].host = strdup(argv[++j]);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--adsblol-port") && more) {
            int32_t idx = addBeastFeed("adsb.lol", "feed.adsb.lol", 30004);
            Modes.beast_feeds[idx].port = atoi(argv[++j]);

        } else if (!strcmp(argv[j],"--airplaneslive")) {
            Modes.net = 1;
            addBeastFeed("airplanes.live", "feed.airplanes.live", 30004);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--airplaneslive-host") && more) {
            Modes.net = 1;
            int32_t idx = addBeastFeed("airplanes.live", "feed.airplanes.live", 30004);
            free(Modes.beast_feeds[idx].host);
            Modes.beast_feeds[idx].host = strdup(argv[++j]);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--airplaneslive-port") && more) {
            int32_t idx = addBeastFeed("airplanes.live", "feed.airplanes.live", 30004);
            Modes.beast_feeds[idx].port = atoi(argv[++j]);

        } else if (!strcmp(argv[j],"--planespotters")) {
            Modes.net = 1;
            addBeastFeed("Planespotters", "feed.planespotters.net", 30004);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--planespotters-host") && more) {
            Modes.net = 1;
            int32_t idx = addBeastFeed("Planespotters", "feed.planespotters.net", 30004);
            free(Modes.beast_feeds[idx].host);
            Modes.beast_feeds[idx].host = strdup(argv[++j]);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--planespotters-port") && more) {
            int32_t idx = addBeastFeed("Planespotters", "feed.planespotters.net", 30004);
            Modes.beast_feeds[idx].port = atoi(argv[++j]);

        } else if (!strcmp(argv[j],"--theairtraffic")) {
            Modes.net = 1;
            addBeastFeed("TheAirTraffic", "feed.theairtraffic.com", 30004);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--theairtraffic-host") && more) {
            Modes.net = 1;
            int32_t idx = addBeastFeed("TheAirTraffic", "feed.theairtraffic.com", 30004);
            free(Modes.beast_feeds[idx].host);
            Modes.beast_feeds[idx].host = strdup(argv[++j]);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--theairtraffic-port") && more) {
            int32_t idx = addBeastFeed("TheAirTraffic", "feed.theairtraffic.com", 30004);
            Modes.beast_feeds[idx].port = atoi(argv[++j]);

        } else if (!strcmp(argv[j],"--avdelphi")) {
            Modes.net = 1;
            addBeastFeed("AVDelphi", "data.avdelphi.com", 24999);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--avdelphi-host") && more) {
            Modes.net = 1;
            int32_t idx = addBeastFeed("AVDelphi", "data.avdelphi.com", 24999);
            free(Modes.beast_feeds[idx].host);
            Modes.beast_feeds[idx].host = strdup(argv[++j]);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--avdelphi-port") && more) {
            int32_t idx = addBeastFeed("AVDelphi", "data.avdelphi.com", 24999);
            Modes.beast_feeds[idx].port = atoi(argv[++j]);

        } else if (!strcmp(argv[j],"--adsbhub")) {
            Modes.net = 1;
            int32_t idx = addBeastFeed("ADSBHub", "data.adsbhub.org", 5001);
            Modes.beast_feeds[idx].format = FEED_FORMAT_RAW;
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--adsbhub-host") && more) {
            Modes.net = 1;
            int32_t idx = addBeastFeed("ADSBHub", "data.adsbhub.org", 5001);
            Modes.beast_feeds[idx].format = FEED_FORMAT_RAW;
            free(Modes.beast_feeds[idx].host);
            Modes.beast_feeds[idx].host = strdup(argv[++j]);
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--adsbhub-port") && more) {
            int32_t idx = addBeastFeed("ADSBHub", "data.adsbhub.org", 5001);
            Modes.beast_feeds[idx].format = FEED_FORMAT_RAW;
            Modes.beast_feeds[idx].port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--adsbhub-ckey") && more) {
            Modes.adsbhub_ckey = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--adsbhub-ckey-file") && more) {
            FILE *f = fopen(argv[++j], "r");
            if (!f) { gg::eprint("Cannot open ckey file: %s\n", argv[j]); exit(1); }
            char ckbuf[256];
            if (fgets(ckbuf, sizeof(ckbuf), f)) {
                int32_t len = strlen(ckbuf);
                while (len > 0 && (ckbuf[len-1] == '\n' || ckbuf[len-1] == '\r' || ckbuf[len-1] == ' '))
                    ckbuf[--len] = '\0';
                Modes.adsbhub_ckey = strdup(ckbuf);
            }
            fclose(f);
            if (!Modes.adsbhub_ckey) { gg::eprint("Empty ckey file: %s\n", argv[j]); exit(1); }

        // Airframes.io ACARS/VDL2 feeds
        } else if (!strcmp(argv[j],"--airframes-acars")) {
            Modes.airframes_acars_feed.enabled = 1;
            if (!Modes.airframes_acars_feed.host)
                Modes.airframes_acars_feed.host = strdup("feed.acars.io");
            if (!Modes.airframes_acars_feed.port)
                Modes.airframes_acars_feed.port = 5550;
        } else if (!strcmp(argv[j],"--airframes-acars-host") && more) {
            free(Modes.airframes_acars_feed.host);
            Modes.airframes_acars_feed.host = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--airframes-acars-port") && more) {
            Modes.airframes_acars_feed.port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--airframes-vdl2")) {
            Modes.airframes_vdl2_feed.enabled = 1;
            if (!Modes.airframes_vdl2_feed.host)
                Modes.airframes_vdl2_feed.host = strdup("feed.acars.io");
            if (!Modes.airframes_vdl2_feed.port)
                Modes.airframes_vdl2_feed.port = 5552;
        } else if (!strcmp(argv[j],"--airframes-vdl2-host") && more) {
            free(Modes.airframes_vdl2_feed.host);
            Modes.airframes_vdl2_feed.host = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--airframes-vdl2-port") && more) {
            Modes.airframes_vdl2_feed.port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--airframes-station-id") && more) {
            snprintf(Modes.airframes_station_id, sizeof(Modes.airframes_station_id), "%s", argv[++j]);

        } else if (!strcmp(argv[j],"--mlat-server") && more) {
            Modes.net = 1;
            if (mlatClientAddServer(argv[++j]) < 0) {
                gg::eprint("Too many MLAT servers (max %d)\n", MAX_MLAT_SERVERS);
                exit(1);
            }
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--mlat-user") && more) {
            free(MlatConfig.user);
            MlatConfig.user = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--mlat-uuid-file") && more) {
            MlatConfig.uuid_file = argv[++j];
        } else if (!strcmp(argv[j],"--mlat-lat") && more) {
            MlatConfig.lat = atof(argv[++j]);
            MlatConfig.position_set = true;
        } else if (!strcmp(argv[j],"--mlat-lon") && more) {
            MlatConfig.lon = atof(argv[++j]);
            MlatConfig.position_set = true;
        } else if (!strcmp(argv[j],"--mlat-alt") && more) {
            MlatConfig.alt = atof(argv[++j]);
        } else if (!strcmp(argv[j],"--mlat-no-results")) {
            MlatConfig.return_results = false;
        } else if (!strcmp(argv[j],"--piaware")) {
            Modes.net = 1;
            PiawareClient.enabled = 1;
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--piaware-feeder-id") && more) {
            Modes.net = 1;
            PiawareClient.enabled = 1;
            PiawareClient.feeder_id = argv[++j];
            PiawareClient.feeder_id_source = "config";
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--piaware-feeder-id-file") && more) {
            Modes.net = 1;
            PiawareClient.enabled = 1;
            PiawareClient.feeder_id_file = argv[++j];
            applyNetDefaults();
        } else if (!strcmp(argv[j],"--piaware-ca-dir") && more) {
            PiawareClient.ca_dir = argv[++j];
        } else if (!strcmp(argv[j],"--planefinder") && more) {
            // Removed in light version
            ++j;
        } else if (!strcmp(argv[j],"--fr24") && more) {
            // Removed in light version
            ++j;
        } else if (!strcmp(argv[j],"--fr24-config") && more) {
            // Removed in light version
            ++j;
        } else if (!strcmp(argv[j],"--radarbox")) {
            // Removed in light version
        } else if (!strcmp(argv[j],"--radarbox-config") && more) {
            // Removed in light version
            ++j;
        } else if (!strcmp(argv[j],"--radarbox-key") && more) {
            // Removed in light version
            ++j;
        } else if (!strcmp(argv[j],"--radarbox-binary") && more) {
            // Removed in light version
            ++j;
        } else if (!strcmp(argv[j],"--rb-keys") && more) {
            // Removed in light version
            ++j;
        } else if (flarmReaderHandleOption(argc, argv, &j)) {
            // handled by flarm reader
        } else if (!strcmp(argv[j],"--pocsag")) {
            PocsagConfig.enabled = 1;
        } else if (!strcmp(argv[j],"--pocsag-device") && more) {
            PocsagConfig.enabled = 1;
            PocsagConfig.device_serial = argv[++j];
        } else if (!strcmp(argv[j],"--pocsag-ifile") && more) {
            PocsagConfig.enabled = 1;
            PocsagConfig.ifile_path = argv[++j];
        } else if (!strcmp(argv[j],"--pocsag-freq") && more) {
            PocsagConfig.freq = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--pocsag-gain") && more) {
            PocsagConfig.gain = (float)atof(argv[++j]);
        } else if (!strcmp(argv[j],"--pocsag-ppm") && more) {
            PocsagConfig.ppm_error = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--sondehub") && more) {
            SondehubConfig.enabled = true;
            SondehubConfig.callsign = argv[++j];
        } else if (!strcmp(argv[j],"--sonde-ifile") && more) {
            SondeIfileConfig.ifile_path = argv[++j];
        } else if (!strcmp(argv[j],"--sonde-freq") && more) {
            SondeIfileConfig.freq = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--acars-ifile") && more) {
            AcarsIfileConfig.ifile_path = argv[++j];
        } else if (!strcmp(argv[j],"--acars-freq") && more) {
            AcarsIfileConfig.freq = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--gsm-ifile") && more) {
            GsmIfileConfig.ifile_path = argv[++j];
        } else if (!strcmp(argv[j],"--gsm-freq") && more) {
            GsmIfileConfig.freq = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--lte-ifile") && more) {
            LteIfileConfig.ifile_path = argv[++j];
        } else if (!strcmp(argv[j],"--lte-freq") && more) {
            LteIfileConfig.freq = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--receiver") && more) {
            rx_config_t rxcfg;
            if (!rxParseConfig(argv[++j], &rxcfg)) {
                gg::eprint("Invalid --receiver argument. Format: serial:role[:gain=X][:ppm=Y][:agc]\n");
                exit(1);
            }
            if (sdrManagerAddReceiver(&rxcfg) < 0) {
                gg::eprint("Too many receivers (max %d)\n", MAX_SDR_RECEIVERS);
                exit(1);
            }
        } else if (!strcmp(argv[j],"--onlyaddr")) {
            Modes.onlyaddr = 1;
        } else if (!strcmp(argv[j],"--tisb-verbose")) {
            Modes.tisb_verbose = 1;
        } else if (!strcmp(argv[j],"--crc-rescue")) {
            Modes.crc_rescue = 1;
        } else if (panelHandleOption(argc, argv, &j)) {
            // handled by panel
        } else if (!strcmp(argv[j],"--metric")) {
            Modes.metric = 1;
        } else if (!strcmp(argv[j],"--hae") || !strcmp(argv[j],"--gnss")) {
            Modes.use_gnss = 1;
        } else if (!strcmp(argv[j],"--aggressive")) {
            gg::eprint("warning: --aggressive not supported in this build, option ignored (consider '--fix-2bit' instead)\n");
        } else if (!strcmp(argv[j],"--interactive")) {
            Modes.interactive = 1;
        } else if (!strcmp(argv[j],"--interactive-ttl") && more) {
            Modes.interactive_display_ttl = (uint64_t)(1000 * atof(argv[++j]));
        } else if (!strcmp(argv[j],"--interactive-show-distance")) {
            Modes.interactive_show_distance = 1;
        } else if (!strcmp(argv[j], "--interactive-distance-units") && more) {
            char *units = argv[++j];
            if (!strcmp(units, "km")) {
                Modes.interactive_distance_units = UNIT_KILOMETERS;
            } else if (!strcmp(units, "sm")) {
                Modes.interactive_distance_units = UNIT_STATUTE_MILES;
            } else {
                Modes.interactive_distance_units = UNIT_NAUTICAL_MILES;
            }
        } else if (!strcmp(argv[j], "--interactive-callsign-filter") && more) {
            Modes.interactive_callsign_filter = strdup(argv[++j]);
        } else if (!strcmp(argv[j], "--lat") && more) {
            Modes.fUserLat = atof(argv[++j]);
        } else if (!strcmp(argv[j],"--lon") && more) {
            Modes.fUserLon = atof(argv[++j]);
        } else if (!strcmp(argv[j],"--max-range") && more) {
            Modes.maxRange = atof(argv[++j]) * 1852.0; // convert to metres
        } else if (!strcmp(argv[j],"--debug") && more) {
            gg::eprint("warning: --debug is obsolete and ignored\n");
            ++j;
        } else if (!strcmp(argv[j],"--stats")) {
            if (!Modes.stats)
                Modes.stats = (uint64_t)1 << 60; // "never"
        } else if (!strcmp(argv[j],"--stats-range")) {
            Modes.stats_range_histo = 1;
        } else if (!strcmp(argv[j],"--stats-every") && more) {
            Modes.stats = (uint64_t) (1000 * atof(argv[++j]));
        } else if (!strcmp(argv[j],"--json-stats-every") && more) {
            Modes.json_stats_interval = (uint64_t) (1000 * atof(argv[++j]));
        } else if (!strcmp(argv[j],"--snip") && more) {
            snipMode(atoi(argv[++j]));
            exit(0);
        } else if (!strcmp(argv[j],"--help")) {
            showHelp();
            exit(0);
        } else if (!strcmp(argv[j],"--version")) {
            showVersion();
            showDSP();
            exit(0);
        } else if (!strcmp(argv[j],"--no-saved-config")) {
            // handled by the pre-scan before config loading
        } else if (!strcmp(argv[j],"--quiet")) {
            Modes.quiet = 1;
        } else if (!strcmp(argv[j],"--show-only") && more) {
            Modes.show_only = (uint32_t) strtoul(argv[++j], NULL, 16);
        } else if (!strcmp(argv[j],"--mlat")) {
            Modes.mlat = 1;
        } else if (!strcmp(argv[j],"--oversample")) {
            // Ignored
        } else if (!strcmp(argv[j], "--write-json") && more) {
            Modes.json_dir = strdup(argv[++j]);
        } else if (!strcmp(argv[j], "--write-json-every") && more) {
            Modes.json_interval = (uint64_t)(1000 * atof(argv[++j]));
            if (Modes.json_interval < 100) // 0.1s
                Modes.json_interval = 100;
        } else if (!strcmp(argv[j], "--json-location-accuracy") && more) {
            Modes.json_location_accuracy = atoi(argv[++j]);
        } else if (!strcmp(argv[j], "--wisdom") && more) {
            if (starch_read_wisdom (argv[++j]) < 0) {
                fprintf(stderr,
                        "Failed to read wisdom file %s: %s\n", argv[j], strerror(errno));
                exit(1);
            }
        } else if (!strcmp(argv[j], "--adaptive-min-gain") && more) {
            Modes.adaptive_min_gain_db = atof(argv[++j]);
        } else if (!strcmp(argv[j], "--adaptive-max-gain") && more) {
            Modes.adaptive_max_gain_db = atof(argv[++j]);
        } else if (!strcmp(argv[j], "--adaptive-duty-cycle") && more) {
            Modes.adaptive_duty_cycle = atof(argv[++j]) / 100.0;
        } else if (!strcmp(argv[j], "--adaptive-burst")) {
            Modes.adaptive_burst_control = true;
        } else if (!strcmp(argv[j], "--adaptive-burst-alpha") && more) {
            Modes.adaptive_burst_alpha = atof(argv[++j]);
        } else if (!strcmp(argv[j], "--adaptive-burst-change-delay") && more) {
            Modes.adaptive_burst_change_delay = atoi(argv[++j]);
        } else if (!strcmp(argv[j], "--adaptive-burst-loud-rate") && more) {
            Modes.adaptive_burst_loud_rate = atof(argv[++j]);
        } else if (!strcmp(argv[j], "--adaptive-burst-loud-runlength") && more) {
            Modes.adaptive_burst_loud_runlength = atoi(argv[++j]);
        } else if (!strcmp(argv[j], "--adaptive-burst-quiet-rate") && more) {
            Modes.adaptive_burst_quiet_rate = atof(argv[++j]);
        } else if (!strcmp(argv[j], "--adaptive-burst-quiet-runlength") && more) {
            Modes.adaptive_burst_quiet_runlength = atoi(argv[++j]);
        } else if (!strcmp(argv[j], "--adaptive-range")) {
            Modes.adaptive_range_control = true;
        } else if (!strcmp(argv[j], "--adaptive-range-alpha") && more) {
            Modes.adaptive_range_alpha = atof(argv[++j]);
        } else if (!strcmp(argv[j], "--adaptive-range-percentile") && more) {
            Modes.adaptive_range_percentile = atoi(argv[++j]);
        } else if (!strcmp(argv[j], "--adaptive-range-target") && more) {
            Modes.adaptive_range_target = atof(argv[++j]);
        } else if (!strcmp(argv[j], "--adaptive-range-change-delay") && more) {
            Modes.adaptive_range_change_delay = atoi(argv[++j]);
        } else if (!strcmp(argv[j], "--adaptive-range-scan-delay") && more) {
            Modes.adaptive_range_scan_delay = atoi(argv[++j]);
        } else if (!strcmp(argv[j], "--adaptive-range-rescan-delay") && more) {
            Modes.adaptive_range_rescan_delay = atoi(argv[++j]);
        } else if (sdrHandleOption(argc, argv, &j)) {
            /* handled */
        } else {
            fprintf(stderr,
                    "Unknown or not enough arguments for option '%s'.\nTry %s --help for full option help.\n",
                    argv[j],
                    argv[0]);
            exit(1);
        }
    }

    bool has_standalone_ifile = FlarmConfig.enabled || PocsagConfig.enabled ||
                                !SondeIfileConfig.ifile_path.empty() ||
                                !AcarsIfileConfig.ifile_path.empty() ||
                                !GsmIfileConfig.ifile_path.empty() ||
                                !LteIfileConfig.ifile_path.empty();

    if (Modes.sdr_type == SDR_NONE && !Modes.net && !has_standalone_ifile && SdrManager.count == 0) {
        // Note: CLI ADS-B/FLARM args are converted to SdrManager receivers at startup,
        // so this check only fires if truly nothing was configured
        fprintf(stderr,
                "No SDR available and network mode not enabled; nothing to do!\n"
                "Select a SDR type (--device-type or --ifile), or enable network mode (--net)\n"
                "Try %s --help for full option help.\n",
                argv[0]);
        exit(1);
    }

    if (Modes.nfix_crc > MODES_MAX_BITERRORS)
        Modes.nfix_crc = MODES_MAX_BITERRORS;

    // Initialization
    log_with_timestamp("%s %s starting up.", MODES_DUMP1090_VARIANT, MODES_DUMP1090_VERSION);
#ifdef ENABLE_SDRGG
    log_with_timestamp("libsdrgg %d.%d.%d", SDRGG_VERSION_MAJOR, SDRGG_VERSION_MINOR, SDRGG_VERSION_PATCH);
#endif
    modesInit();

    // Sync new config structs from Modes (migration step)
    appConfigSyncFromModes();

    // Probe all RTL-SDR tuner types before any device is opened
    panelProbeAllTuners();

    // ========== Load saved receivers, then apply CLI overrides ==========

    // Step 1: Load all receivers from receivers.json (baseline config)
    int32_t loaded = 0;
    if (load_saved_config) {
        loaded = sdrManagerLoad();
        if (loaded > 0) {
            log_with_timestamp("Loaded %d receiver(s) from receivers.json", loaded);
        }
    } else {
        log_with_timestamp("Skipping receivers.json load (--no-saved-config)");
    }

    // Step 2: CLI args override or add receivers
    // Primary ADS-B receiver (--device-index / --device-type rtlsdr)
    if (Modes.sdr_type == SDR_RTLSDR) {
        rx_config_t adsb_cfg = {0};
        adsb_cfg.role = SDR_ROLE_ADSB;
        adsb_cfg.freq = 1090000000;
        adsb_cfg.sample_rate = 2400000;
        adsb_cfg.gain = Modes.gain;
        if (Modes.dev_name) {
            snprintf(adsb_cfg.serial, sizeof(adsb_cfg.serial), "%.63s", Modes.dev_name);
        }
        int32_t existing = sdrManagerFindBySerial(adsb_cfg.serial);
        if (existing >= 0) {
            sdrManagerUpdateConfig(existing, &adsb_cfg);
            log_with_timestamp("ADS-B receiver %s: CLI override (--device-type rtlsdr --gain %.1f)",
                               adsb_cfg.serial[0] ? adsb_cfg.serial : "auto", adsb_cfg.gain);
        } else {
            int32_t idx = sdrManagerAddReceiver(&adsb_cfg);
            if (idx >= 0) {
                log_with_timestamp("ADS-B receiver added (serial=%s)",
                                   adsb_cfg.serial[0] ? adsb_cfg.serial : "auto");
            }
        }
        Modes.sdr_type = SDR_NONE;
        Modes.net = 1;
    }

    // FLARM receiver (--flarm --flarm-device)
    if (FlarmConfig.enabled && FlarmConfig.device_serial[0]) {
        rx_config_t flarm_cfg = {0};
        flarm_cfg.role = SDR_ROLE_FLARM;
        flarm_cfg.freq = 868300000;
        flarm_cfg.sample_rate = 1600000;
        flarm_cfg.gain = FlarmConfig.gain / 10.0;  // FlarmConfig stores tenths of dB
        flarm_cfg.ppm_error = FlarmConfig.ppm_error;
        snprintf(flarm_cfg.serial, sizeof(flarm_cfg.serial), "%.63s", FlarmConfig.device_serial.c_str());
        int32_t existing = sdrManagerFindBySerial(flarm_cfg.serial);
        if (existing >= 0) {
            sdrManagerUpdateConfig(existing, &flarm_cfg);
            log_with_timestamp("FLARM receiver %s: CLI override (--flarm --flarm-gain %.1f --flarm-ppm %d)",
                               FlarmConfig.device_serial.c_str(), flarm_cfg.gain, flarm_cfg.ppm_error);
        } else {
            int32_t idx = sdrManagerAddReceiver(&flarm_cfg);
            if (idx >= 0) {
                log_with_timestamp("FLARM receiver added (serial=%s)", FlarmConfig.device_serial.c_str());
            }
        }
    }

    // FLARM IQ file input (--flarm-ifile) — standalone path
    if (FlarmConfig.enabled && !FlarmConfig.ifile_path.empty()) {
        if (!flarmReaderOpen()) {
            gg::eprint("flarm-ifile: failed to open, continuing without FLARM\n");
            FlarmConfig.enabled = 0;
        } else {
            flarmReaderStart();
            log_with_timestamp("FLARM IQ file reader started: %s", FlarmConfig.ifile_path.c_str());
        }
    }

    // POCSAG IQ file input (--pocsag-ifile) — standalone path
    if (PocsagConfig.enabled && !PocsagConfig.ifile_path.empty()) {
        FILE *fp = fopen(PocsagConfig.ifile_path.c_str(), "rb");
        if (!fp) {
            fprintf(stderr, "pocsag-ifile: cannot open '%s': %s\n",
                    PocsagConfig.ifile_path.c_str(), strerror(errno));
            PocsagConfig.enabled = 0;
        } else {
            fseek(fp, 0, SEEK_END);
            int64_t file_size = ftell(fp);
            fclose(fp);
            double duration = (file_size / 2.0) / POCSAG_SAMPLE_RATE;
            fprintf(stderr, "pocsag-ifile: file '%s' (%ld bytes, %.1f seconds at %.1f MSPS)\n",
                    PocsagConfig.ifile_path.c_str(), file_size, duration, POCSAG_SAMPLE_RATE / 1e6);

            pocsag_config_t cfg;
            cfg = {};
            cfg.center_freq = PocsagConfig.freq ? PocsagConfig.freq : 466150000;
            cfg.sample_rate = POCSAG_SAMPLE_RATE;
            cfg.channel_freqs[0] = 466075000;
            cfg.channel_freqs[1] = 466175000;
            cfg.channel_freqs[2] = 466225000;
            cfg.num_channels = 3;
            cfg.callback = pocsag_ifile_msg_handler;
            cfg.callback_ctx = NULL;

            PocsagIfile.demod = pocsag_create(&cfg);
            if (!PocsagIfile.demod) {
                gg::eprint("pocsag-ifile: failed to create decoder\n");
                PocsagConfig.enabled = 0;
            } else {
                PocsagIfile.stop_flag = 0;
                PocsagIfile.thread_running = 1;
                pthread_create(&PocsagIfile.thread, NULL, pocsag_ifile_reader_thread, NULL);
                log_with_timestamp("POCSAG IQ file reader started: %s", PocsagConfig.ifile_path.c_str());
            }
        }
    }

    // Sonde IQ file input (--sonde-ifile) — standalone path
    if (!SondeIfileConfig.ifile_path.empty()) {
        FILE *fp = fopen(SondeIfileConfig.ifile_path.c_str(), "rb");
        if (!fp) {
            fprintf(stderr, "sonde-ifile: cannot open '%s': %s\n",
                    SondeIfileConfig.ifile_path.c_str(), strerror(errno));
        } else {
            fseek(fp, 0, SEEK_END);
            int64_t file_size = ftell(fp);
            fclose(fp);
            double duration = (file_size / 2.0) / SONDE_IFILE_SAMPLE_RATE;
            fprintf(stderr, "sonde-ifile: file '%s' (%ld bytes, %.1f seconds at %.1f MSPS)\n",
                    SondeIfileConfig.ifile_path.c_str(), file_size, duration, SONDE_IFILE_SAMPLE_RATE / 1e6);

            sonde_config_t cfg;
            cfg = {};
            cfg.center_freq = SondeIfileConfig.freq ? SondeIfileConfig.freq : 403000000;
            cfg.sample_rate = SONDE_IFILE_SAMPLE_RATE;
            cfg.callback = sonde_ifile_msg_handler;
            cfg.callback_ctx = NULL;

            SondeIfile.demod = sonde_create(&cfg);
            if (!SondeIfile.demod) {
                gg::eprint("sonde-ifile: failed to create decoder\n");
            } else {
                SondeIfile.stop_flag = 0;
                SondeIfile.thread_running = 1;
                pthread_create(&SondeIfile.thread, NULL, sonde_ifile_reader_thread, NULL);
                log_with_timestamp("Sonde IQ file reader started: %s", SondeIfileConfig.ifile_path.c_str());
            }
        }
    }

    // ACARS IQ file input (--acars-ifile)
    if (!AcarsIfileConfig.ifile_path.empty()) {
        FILE *fp = fopen(AcarsIfileConfig.ifile_path.c_str(), "rb");
        if (!fp) {
            fprintf(stderr, "acars-ifile: cannot open '%s': %s\n",
                    AcarsIfileConfig.ifile_path.c_str(), strerror(errno));
        } else {
            fseek(fp, 0, SEEK_END);
            int64_t file_size = ftell(fp);
            fclose(fp);
            double duration = (file_size / 2.0) / ACARS_IFILE_SAMPLE_RATE;
            fprintf(stderr, "acars-ifile: file '%s' (%ld bytes, %.1f seconds at %.1f MSPS)\n",
                    AcarsIfileConfig.ifile_path.c_str(), file_size, duration, ACARS_IFILE_SAMPLE_RATE / 1e6);

            acars_config_t cfg;
            cfg = {};
            cfg.center_freq = AcarsIfileConfig.freq ? AcarsIfileConfig.freq : 131550000;
            cfg.sample_rate = ACARS_IFILE_SAMPLE_RATE;
            cfg.channel_freqs[0] = ACARS_FREQ_PRIMARY;
            cfg.channel_freqs[1] = ACARS_FREQ_SECONDARY;
            cfg.channel_freqs[2] = ACARS_FREQ_TERTIARY;
            cfg.channel_freqs[3] = ACARS_FREQ_4;
            cfg.channel_freqs[4] = ACARS_FREQ_5;
            cfg.num_channels = 5;
            cfg.callback = acars_ifile_msg_handler;
            cfg.callback_ctx = NULL;

            AcarsIfile.demod = acars_create(&cfg);
            if (!AcarsIfile.demod) {
                gg::eprint("acars-ifile: failed to create decoder\n");
            } else {
                AcarsIfile.stop_flag = 0;
                AcarsIfile.thread_running = 1;
                pthread_create(&AcarsIfile.thread, NULL, acars_ifile_reader_thread, NULL);
                log_with_timestamp("ACARS IQ file reader started: %s", AcarsIfileConfig.ifile_path.c_str());
            }
        }
    }

    // GSM IQ file input (--gsm-ifile)
    if (!GsmIfileConfig.ifile_path.empty()) {
        FILE *fp = fopen(GsmIfileConfig.ifile_path.c_str(), "rb");
        if (!fp) {
            fprintf(stderr, "gsm-ifile: cannot open '%s': %s\n",
                    GsmIfileConfig.ifile_path.c_str(), strerror(errno));
        } else {
            fseek(fp, 0, SEEK_END);
            int64_t file_size = ftell(fp);
            fclose(fp);
            double duration = (file_size / 2.0) / GSM_IFILE_SAMPLE_RATE;
            fprintf(stderr, "gsm-ifile: file '%s' (%ld bytes, %.1f seconds at %.1f MSPS)\n",
                    GsmIfileConfig.ifile_path.c_str(), file_size, duration, GSM_IFILE_SAMPLE_RATE / 1e6);

            gsm_config_t cfg;
            cfg = {};
            int32_t arfcn_freq = GsmIfileConfig.freq ? GsmIfileConfig.freq : 939200000;
            cfg.arfcn_freq = arfcn_freq;
            cfg.center_freq = arfcn_freq - GSM_IF_OFFSET;
            cfg.sample_rate = GSM_IFILE_SAMPLE_RATE;
            cfg.tsc = -1;
            cfg.msg_cb = gsm_ifile_msg_handler;
            cfg.cb_cb = gsm_ifile_cb_handler;
            cfg.callback_ctx = NULL;

            GsmIfile.demod = gsm_create(&cfg);
            if (!GsmIfile.demod) {
                gg::eprint("gsm-ifile: failed to create decoder\n");
            } else {
                GsmIfile.stop_flag = 0;
                GsmIfile.thread_running = 1;
                pthread_create(&GsmIfile.thread, NULL, gsm_ifile_reader_thread, NULL);
                log_with_timestamp("GSM IQ file reader started: %s", GsmIfileConfig.ifile_path.c_str());
            }
        }
    }

    // LTE IQ file input (--lte-ifile)
    if (!LteIfileConfig.ifile_path.empty()) {
        FILE *fp = fopen(LteIfileConfig.ifile_path.c_str(), "rb");
        if (!fp) {
            fprintf(stderr, "lte-ifile: cannot open '%s': %s\n",
                    LteIfileConfig.ifile_path.c_str(), strerror(errno));
        } else {
            fseek(fp, 0, SEEK_END);
            int64_t file_size = ftell(fp);
            fclose(fp);
            double duration = (file_size / 2.0) / LTE_IFILE_SAMPLE_RATE;
            fprintf(stderr, "lte-ifile: file '%s' (%ld bytes, %.1f seconds at %.1f MSPS)\n",
                    LteIfileConfig.ifile_path.c_str(), file_size, duration, LTE_IFILE_SAMPLE_RATE / 1e6);

            lte_config_t cfg;
            cfg = {};
            cfg.center_freq = LteIfileConfig.freq ? LteIfileConfig.freq : LTE_DEFAULT_FREQ;
            cfg.sample_rate = LTE_IFILE_SAMPLE_RATE;
            cfg.callback = lte_ifile_cell_handler;
            cfg.callback_ctx = NULL;
            cfg.hop_enabled = false;

            LteIfile.demod = lte_create(&cfg);
            if (!LteIfile.demod) {
                gg::eprint("lte-ifile: failed to create decoder\n");
            } else {
                LteIfile.stop_flag = 0;
                LteIfile.thread_running = 1;
                pthread_create(&LteIfile.thread, NULL, lte_ifile_reader_thread, NULL);
                log_with_timestamp("LTE IQ file reader started: %s", LteIfileConfig.ifile_path.c_str());
            }
        }
    }

    // POCSAG receiver (--pocsag --pocsag-device)
    if (PocsagConfig.enabled && PocsagConfig.ifile_path.empty()) {
        rx_config_t pocsag_cfg = {0};
        pocsag_cfg.role = SDR_ROLE_POCSAG;
        pocsag_cfg.freq = PocsagConfig.freq;
        pocsag_cfg.sample_rate = POCSAG_SAMPLE_RATE;
        pocsag_cfg.gain = PocsagConfig.gain;
        pocsag_cfg.ppm_error = PocsagConfig.ppm_error;
        snprintf(pocsag_cfg.serial, sizeof(pocsag_cfg.serial), "%.63s", PocsagConfig.device_serial.c_str());
        int32_t existing = sdrManagerFindBySerial(pocsag_cfg.serial);
        if (existing >= 0) {
            sdrManagerUpdateConfig(existing, &pocsag_cfg);
            log_with_timestamp("POCSAG receiver %s: CLI override (--pocsag --pocsag-freq %d)",
                               PocsagConfig.device_serial.c_str(), PocsagConfig.freq);
        } else {
            int32_t idx = sdrManagerAddReceiver(&pocsag_cfg);
            if (idx >= 0) {
                log_with_timestamp("POCSAG receiver added (serial=%s, freq=%d)",
                                   PocsagConfig.device_serial.c_str(), PocsagConfig.freq);
            }
        }
    }

    // Save current receiver state (merges CLI + loaded receivers)
    if (SdrManager.count > 0) {
        sdrManagerSave();
    }

    // Open all SdrManager receivers
    if (SdrManager.count > 0) {
        int32_t opened = sdrManagerOpenAll();
        log_with_timestamp("SdrManager: %d/%d receivers opened", opened, SdrManager.count);
        if (opened > 0) {
            Modes.net = 1;
        }
    }

    if (Modes.net) {
        modesInitNet();
    }

    // Initialize the C++ dispatcher (decoder queue → aircraft list + feeders)
    dispatcher_init();

    // Initialize FLARM decode tables and keys
    if (!FlarmConfig.keys_file.empty()) {
        flarm_load_keys(FlarmConfig.keys_file.c_str());
    }
    flarm_decode_init();

    // Copy receiver position to OpenSky config if not explicitly set
    if (OpenSkyConfig.enabled) {
        if (OpenSkyConfig.lat == 0.0 && OpenSkyConfig.lon == 0.0) {
            OpenSkyConfig.lat = Modes.fUserLat;
            OpenSkyConfig.lon = Modes.fUserLon;
        }
        if (OpenSkyConfig.alt == 0.0 && MlatConfig.alt > 0) {
            OpenSkyConfig.alt = MlatConfig.alt;
        }
    }

    // Start web control panel
    panelStart();

    if (load_saved_config) {
        // Ensure all well-known beast feeds exist (disabled by default)
        panelEnsureDefaultBeastFeeds();

        // Load saved beast feed enabled/disabled state from panel.conf
        panelLoadBeastFeedState();
    } else {
        log_with_timestamp("Skipping saved panel/feed state (--no-saved-config)");
    }

    // init stats:
    reset_stats(&Modes.stats_current);
    reset_stats(&Modes.stats_alltime);
    reset_stats(&Modes.stats_periodic);
    reset_stats(&Modes.stats_latest);
    reset_stats(&Modes.stats_5min);
    reset_stats(&Modes.stats_15min);

    Modes.stats_current.start = Modes.stats_current.end =
        Modes.stats_alltime.start = Modes.stats_alltime.end =
        Modes.stats_periodic.start = Modes.stats_periodic.end =
        Modes.stats_latest.start = Modes.stats_latest.end =
        Modes.stats_5min.start = Modes.stats_5min.end =
        Modes.stats_15min.start = Modes.stats_15min.end = mstime();

    for (j = 0; j < 15; ++j) {
        reset_stats(&Modes.stats_1min[j]);
        Modes.stats_1min[j].start = Modes.stats_1min[j].end = Modes.stats_current.start;
    }

    adaptive_init();

    // write initial json files so they're not missing
    writeJsonToFile("receiver.json", generateReceiverJson);
    writeJsonToFile("stats.json", generateStatsJson);
    writeJsonToFile("aircraft.json", generateAircraftJson);

    interactiveInit();

    // Start all SdrManager receiver threads
    if (SdrManager.count > 0) {
        int32_t started = sdrManagerStartAll();
        log_with_timestamp("SdrManager: %d receiver threads started", started);
    }

    // Start feeder threads (MLAT, PiAware, OGN) — each in its own pthread
    feederThreadsStart();

    // Start airframes.io ACARS/VDL2 UDP feeds
    airframesFeedInit();

    // ======================== Main loop ========================
    if (SdrManager.count == 0) {
        // Net-only mode: no SDR devices, just serve network data
        while (!Modes.exit) {
            struct timespec start_time;
            struct timespec slp = { 0, 100 * 1000 * 1000};

            start_cpu_timing(&start_time);
            backgroundTasks();
            end_cpu_timing(&start_time, &Modes.stats_current.background_cpu);

            nanosleep(&slp, NULL);
        }
    } else {
        // SdrManager main loop: drain all receivers
        int32_t watchdogCounter = 3000; // 3000 * 10ms = ~30 seconds

        while (!Modes.exit) {
            struct timespec start_time;

            // Drain decoded data from all running receivers
            start_cpu_timing(&start_time);
            bool got_data = sdrManagerDrainAll();
            end_cpu_timing(&start_time, &Modes.stats_current.demod_cpu);

            if (got_data) {
                watchdogCounter = 3000;
            } else {
                struct timespec slp = { 0, 10 * 1000 * 1000 }; // 10ms
                nanosleep(&slp, NULL);
                if (--watchdogCounter <= 0) {
                    log_with_timestamp("No samples received from any SDR for a int64_t time. Giving up.");
                    Modes.exit = 2;
                }
            }

            start_cpu_timing(&start_time);
            backgroundTasks();
            end_cpu_timing(&start_time, &Modes.stats_current.background_cpu);
        }

        // Shutdown all receivers
        log_with_timestamp("Stopping all SDR receivers...");
        sdrManagerStopAll();
    }

    stopStandaloneIfileReaders();

    interactiveCleanup();
    elmCleanup(&Modes.elm);

    // Stop feeder threads first (they use MLAT/PiAware/OGN clients)
    feederThreadsStop();

    mlatClientCleanup();
    piawareClientCleanup();
    ognClientCleanup();
    sondehubClientCleanup();
    airframesFeedCleanup();

    // Write final stats
    flush_stats(0);
    writeJsonToFile("stats.json", generateStatsJson);

    // Stop panel (saves stats history to disk)
    panelStop();
    if (Modes.stats) {
        display_stats(&Modes.stats_alltime);
    }

    sdrManagerShutdown();
    fifo_destroy();

    if (Modes.exit == 1) {
        log_with_timestamp("Normal exit.");
        return 0;
    } else {
        log_with_timestamp("Abnormal exit.");
        return 1;
    }
}
//
//=========================================================================
//
