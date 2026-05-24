// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// flarm_reader.c: Second RTL-SDR reader for FLARM 868 MHz reception
//
// Opens a second RTL-SDR dongle (identified by serial number), tunes it to
// 868.3 MHz, and feeds IQ samples into the FLARM GFSK demodulator.
// Decoded FLARM packets are queued and integrated into the aircraft list
// during backgroundTasks().
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <errno.h>
#include <sys/stat.h>

#include "dump1090.h"
#include "dispatcher.h"
#include "msg_queue.h"
#include "ogntp_decode.h"
#include "adsl_decode.h"
#include "p3i_demod.h"

#ifdef ENABLE_RTLSDR
#include <rtl-sdr.h>
#endif

// ======================== Globals ========================

flarm_reader_config_t FlarmConfig;

// Thread-safe message queues: decoded messages consumed by main thread
#define FLARM_MSG_QUEUE_SIZE 64

static struct {
    pthread_t        thread;
    int32_t              thread_running;

#ifdef ENABLE_RTLSDR
    rtlsdr_dev_t    *dev;
#endif

    struct flarm_state *demod;
    struct p3i_demod_state *p3i_demod;   // P3I demodulator (NULL if disabled)

    // Thread-safe queues (C++ mutex-based, C-opaque handles)
    msg_queue_t      msg_queue;
    msg_queue_t      ogntp_queue;
    msg_queue_t      p3i_queue;
    msg_queue_t      adsl_queue;

    volatile int32_t     stop_flag;
} Flarm;

// Dispatcher aircraft queue for FLARM/OGNTP/P3I/ADS-L (registered at init)
static aircraft_queue_handle_t flarm_aircraft_queue = NULL;

// ======================== Config defaults ========================

void flarmReaderInitConfig(void)
{
    memset(&FlarmConfig, 0, sizeof(FlarmConfig));
    FlarmConfig.enabled = 0;
    FlarmConfig.gain = 0;  // auto
    FlarmConfig.ppm_error = 0;
    FlarmConfig.ifile_once = 0;
    strncpy(FlarmConfig.ogn_server, "aprs.glidernet.org", sizeof(FlarmConfig.ogn_server) - 1);
    FlarmConfig.ogn_port = 14580;

    memset(&Flarm, 0, sizeof(Flarm));
}

// ======================== Message queue ========================

static void flarm_enqueue_message(const flarm_message_t *msg, void *ctx)
{
    (void)ctx;
    msg_queue_push(Flarm.msg_queue, msg);
}

static bool flarm_dequeue_message(flarm_message_t *msg)
{
    return msg_queue_pop(Flarm.msg_queue, msg) != 0;
}

static void ogntp_enqueue_message(const ogntp_message_t *msg, void *ctx)
{
    (void)ctx;
    msg_queue_push(Flarm.ogntp_queue, msg);
}

static bool ogntp_dequeue_message(ogntp_message_t *msg)
{
    return msg_queue_pop(Flarm.ogntp_queue, msg) != 0;
}

static void p3i_enqueue_message(const p3i_message_t *msg, void *ctx)
{
    (void)ctx;
    msg_queue_push(Flarm.p3i_queue, msg);
}

static bool p3i_dequeue_message(p3i_message_t *msg)
{
    return msg_queue_pop(Flarm.p3i_queue, msg) != 0;
}

static void adsl_enqueue_message(const adsl_message_t *msg, void *ctx)
{
    (void)ctx;
    msg_queue_push(Flarm.adsl_queue, msg);
}

static bool adsl_dequeue_message(adsl_message_t *msg)
{
    return msg_queue_pop(Flarm.adsl_queue, msg) != 0;
}

static void ogntp_log_status(const ogntp_message_t *msg)
{
    char line[256];
    int32_t len = snprintf(line, sizeof(line),
                       "OGNTP v%d %d:%06X status relay=%d time=%02ds hw=%02X fw=%02X sats=%d fix=%d V=%.2f Tx=%ddBm",
                       msg->version,
                       msg->addr_type,
                       (uint32_t)(msg->addr & 0xFFFFFF),
                       msg->relay,
                       msg->status.time_seconds,
                       (uint32_t)(msg->status.hardware & 0xFF),
                       (uint32_t)(msg->status.firmware & 0xFF),
                       msg->status.satellites,
                       msg->status.fix_quality,
                       msg->status.voltage_v,
                       msg->status.tx_power_dbm);

    if (msg->status.has_pressure && len > 0 && len < (int32_t)sizeof(line)) {
        len += snprintf(line + len, sizeof(line) - (size_t)len,
                        " P=%.1fhPa", msg->status.pressure_hpa);
    }
    if (msg->status.has_temperature && len > 0 && len < (int32_t)sizeof(line)) {
        len += snprintf(line + len, sizeof(line) - (size_t)len,
                        " T=%.1fC", msg->status.temperature_c);
    }
    if (msg->status.has_humidity && len > 0 && len < (int32_t)sizeof(line)) {
        len += snprintf(line + len, sizeof(line) - (size_t)len,
                        " H=%.1f%%", msg->status.humidity_percent);
    }

    fprintf(stderr, "%s\n", line);
    panelLog("%s", line);
}

// ======================== RTL-SDR callback ========================

#ifdef ENABLE_RTLSDR
static void flarm_rtlsdr_callback(uint8_t *buf, uint32_t len, void *ctx)
{
    (void)ctx;

    if (Flarm.stop_flag || Modes.exit) {
        rtlsdr_cancel_async(Flarm.dev);
        return;
    }

    if (Flarm.demod && len > 0) {
        flarm_demod_process(Flarm.demod, buf, len);
    }
    if (Flarm.p3i_demod && len > 0) {
        p3i_demod_process(Flarm.p3i_demod, buf, len);
    }
}
#endif

// ======================== IQ file reader thread ========================

// Read raw IQ file in a loop, feeding samples to the demodulator at real-time rate.
// File format: uint8 I/Q pairs at FLARM_SAMPLE_RATE (1.6 MSPS).
// The file is replayed continuously until stop_flag is set, unless single-pass mode is requested.

static void *flarm_ifile_reader_thread(void *arg)
{
    (void)arg;

    const char *path = FlarmConfig.ifile_path;
    const uint32_t buf_size = MODES_RTL_BUF_SIZE; // 256 KB
    uint8_t *buf = malloc(buf_size);
    if (!buf) {
        fprintf(stderr, "flarm-ifile: failed to allocate read buffer\n");
        return NULL;
    }

    fprintf(stderr, "flarm-ifile: reader thread started, file=%s\n", path);

    uint32_t loop_count = 0;

    while (!Flarm.stop_flag && !Modes.exit) {
        FILE *fp = fopen(path, "rb");
        if (!fp) {
            fprintf(stderr, "flarm-ifile: cannot open '%s': %s\n", path, strerror(errno));
            break;
        }

        if (loop_count > 0) {
            fprintf(stderr, "flarm-ifile: replaying file (loop %u)\n", loop_count);
        }
        loop_count++;

        while (!Flarm.stop_flag && !Modes.exit) {
            size_t nread = fread(buf, 1, buf_size, fp);
            if (nread == 0) {
                if (FlarmConfig.ifile_once) {
                    fprintf(stderr, "flarm-ifile: reached EOF after one pass, requesting shutdown\n");
                    Modes.exit = 1;
                }
                break;
            }

            // Ensure even number of bytes (IQ pairs)
            nread &= ~(size_t)1;

            if (Flarm.demod && nread > 0) {
                flarm_demod_process(Flarm.demod, buf, (uint32_t)nread);
            }
            if (Flarm.p3i_demod && nread > 0) {
                p3i_demod_process(Flarm.p3i_demod, buf, (uint32_t)nread);
            }

            // Throttle to real-time pace
            double actual_samples = nread / 2.0;
            double sleep_us = (actual_samples / FLARM_SAMPLE_RATE) * 1e6;
            if (sleep_us > 0) {
                struct timespec ts;
                ts.tv_sec = (time_t)(sleep_us / 1e6);
                ts.tv_nsec = (int64_t)(fmod(sleep_us, 1e6) * 1000);
                nanosleep(&ts, NULL);
            }
        }

        fclose(fp);
    }

    free(buf);
    fprintf(stderr, "flarm-ifile: reader thread exiting (loops=%u)\n", loop_count);
    return NULL;
}

// ======================== Reader thread ========================

static void *flarm_reader_thread(void *arg)
{
    (void)arg;

#ifdef ENABLE_RTLSDR
    if (!Flarm.dev) {
        return NULL;
    }

    fprintf(stderr, "flarm: reader thread started, reading async...\n");
    rtlsdr_read_async(Flarm.dev, flarm_rtlsdr_callback, NULL, 4, MODES_RTL_BUF_SIZE);

    if (!Modes.exit && !Flarm.stop_flag) {
        fprintf(stderr, "flarm: rtlsdr_read_async returned unexpectedly\n");
    }
#endif

    fprintf(stderr, "flarm: reader thread exiting\n");
    return NULL;
}

// ======================== Find RTL-SDR by serial ========================

#ifdef ENABLE_RTLSDR
static int32_t find_flarm_device(const char *serial)
{
    int32_t count = rtlsdr_get_device_count();
    if (count == 0) return -1;

    for (int32_t i = 0; i < count; i++) {
        char vendor[256], product[256], sn[256];
        if (rtlsdr_get_device_usb_strings(i, vendor, product, sn) == 0) {
            if (strcmp(sn, serial) == 0) {
                return i;
            }
        }
    }
    return -1;
}
#endif

// ======================== Open ========================

bool flarmReaderOpen(void)
{
    if (!FlarmConfig.enabled) return true;

    // ---- IQ file mode (--flarm-ifile) ----
    if (FlarmConfig.ifile_path[0] != '\0') {
        // Determine SDR parameters based on P3I enable
        uint32_t sample_rate = FlarmConfig.p3i_enabled ? FLARM_SAMPLE_RATE_P3I : FLARM_SAMPLE_RATE;
        uint32_t center_freq = FlarmConfig.p3i_enabled ? FLARM_CENTER_FREQ_P3I : FLARM_CENTER_FREQ;

        // Verify file exists and is readable
        FILE *fp = fopen(FlarmConfig.ifile_path, "rb");
        if (!fp) {
            fprintf(stderr, "flarm-ifile: cannot open '%s': %s\n",
                    FlarmConfig.ifile_path, strerror(errno));
            return false;
        }
        // Get file size for info
        fseek(fp, 0, SEEK_END);
        int64_t file_size = ftell(fp);
        fclose(fp);

        double duration = (file_size / 2.0) / sample_rate;
        fprintf(stderr, "flarm-ifile: file '%s' (%ld bytes, %.1f seconds at %.1f MSPS)\n",
                FlarmConfig.ifile_path, file_size, duration, sample_rate / 1e6);

        // Create demodulator
        flarm_demod_config_t demod_config = {
            .ref_lat      = Modes.fUserLat,
            .ref_lon      = Modes.fUserLon,
            .ref_alt_geoid = 0,
            .center_freq  = center_freq,
            .sample_rate  = sample_rate,
            .callback     = flarm_enqueue_message,
            .callback_ctx = NULL,
            .ogntp_callback     = ogntp_enqueue_message,
            .ogntp_callback_ctx = NULL,
            .p3i_callback       = NULL,
            .p3i_callback_ctx   = NULL,
            .p3i_enabled        = 0,
            .adsl_callback      = adsl_enqueue_message,
            .adsl_callback_ctx  = NULL
        };

        Flarm.demod = flarm_demod_create(&demod_config);
        if (!Flarm.demod) {
            fprintf(stderr, "flarm-ifile: failed to create demodulator\n");
            return false;
        }

        // Use file mtime as timestamp for XXTEA decryption
        struct stat st;
        if (stat(FlarmConfig.ifile_path, &st) == 0) {
            uint32_t file_time = (uint32_t)st.st_mtime;
            flarm_demod_set_time_override(Flarm.demod, file_time);
            fprintf(stderr, "flarm-ifile: using file mtime %u for decryption\n", file_time);
        }

        // Create P3I demodulator if enabled
        if (FlarmConfig.p3i_enabled) {
            p3i_demod_config_t p3i_cfg = {
                .sample_rate  = sample_rate,
                .center_freq  = center_freq,
                .callback     = p3i_enqueue_message,
                .callback_ctx = NULL
            };
            Flarm.p3i_demod = p3i_demod_create(&p3i_cfg);
            if (Flarm.p3i_demod) {
                fprintf(stderr, "flarm-ifile: P3I decoder enabled (869.525 MHz)\n");
            }
        }

        fprintf(stderr, "flarm-ifile: initialized (%s)\n",
            FlarmConfig.ifile_once ? "single-pass replay" : "will replay in loop");
        return true;
    }

#ifdef ENABLE_RTLSDR
    if (FlarmConfig.device_serial[0] == '\0') {
        fprintf(stderr, "flarm: no device serial specified (use --flarm-device or --flarm-ifile)\n");
        return false;
    }

    int32_t dev_index = find_flarm_device(FlarmConfig.device_serial);
    if (dev_index < 0) {
        fprintf(stderr, "flarm: no RTL-SDR device with serial '%s' found\n", FlarmConfig.device_serial);

        // List available devices
        int32_t count = rtlsdr_get_device_count();
        fprintf(stderr, "flarm: available devices:\n");
        for (int32_t i = 0; i < count; i++) {
            char vendor[256], product[256], sn[256];
            if (rtlsdr_get_device_usb_strings(i, vendor, product, sn) == 0) {
                fprintf(stderr, "  %d: %s %s SN:%s\n", i, vendor, product, sn);
            }
        }
        return false;
    }

    fprintf(stderr, "flarm: opening RTL-SDR device #%d (SN: %s) for 868 MHz\n",
            dev_index, FlarmConfig.device_serial);

    if (rtlsdr_open(&Flarm.dev, dev_index) < 0) {
        fprintf(stderr, "flarm: error opening RTL-SDR device: %s\n", strerror(errno));
        return false;
    }

    // Determine SDR parameters based on P3I enable
    uint32_t sample_rate = FlarmConfig.p3i_enabled ? FLARM_SAMPLE_RATE_P3I : FLARM_SAMPLE_RATE;
    uint32_t center_freq = FlarmConfig.p3i_enabled ? FLARM_CENTER_FREQ_P3I : FLARM_CENTER_FREQ;

    // Configure for FLARM reception
    rtlsdr_set_freq_correction(Flarm.dev, FlarmConfig.ppm_error);
    rtlsdr_set_center_freq(Flarm.dev, center_freq);
    rtlsdr_set_sample_rate(Flarm.dev, sample_rate);

    if (FlarmConfig.gain == 0) {
        // Auto gain
        rtlsdr_set_tuner_gain_mode(Flarm.dev, 0);
        fprintf(stderr, "flarm: using automatic gain\n");
    } else {
        rtlsdr_set_tuner_gain_mode(Flarm.dev, 1);
        rtlsdr_set_tuner_gain(Flarm.dev, FlarmConfig.gain);
        fprintf(stderr, "flarm: gain set to %.1f dB\n", FlarmConfig.gain / 10.0);
    }

    rtlsdr_reset_buffer(Flarm.dev);

    // Create demodulator
    flarm_demod_config_t demod_config = {
        .ref_lat      = Modes.fUserLat,
        .ref_lon      = Modes.fUserLon,
        .ref_alt_geoid = 0,
        .center_freq  = center_freq,
        .sample_rate  = sample_rate,
        .callback     = flarm_enqueue_message,
        .callback_ctx = NULL,
        .ogntp_callback     = ogntp_enqueue_message,
        .ogntp_callback_ctx = NULL,
        .p3i_callback       = NULL,
        .p3i_callback_ctx   = NULL,
        .p3i_enabled        = 0,
        .adsl_callback      = adsl_enqueue_message,
        .adsl_callback_ctx  = NULL
    };

    Flarm.demod = flarm_demod_create(&demod_config);
    if (!Flarm.demod) {
        fprintf(stderr, "flarm: failed to create demodulator\n");
        rtlsdr_close(Flarm.dev);
        Flarm.dev = NULL;
        return false;
    }

    // Create P3I demodulator if enabled
    if (FlarmConfig.p3i_enabled) {
        p3i_demod_config_t p3i_cfg = {
            .sample_rate  = sample_rate,
            .center_freq  = center_freq,
            .callback     = p3i_enqueue_message,
            .callback_ctx = NULL
        };
        Flarm.p3i_demod = p3i_demod_create(&p3i_cfg);
        if (Flarm.p3i_demod) {
            fprintf(stderr, "flarm: P3I decoder enabled (869.525 MHz)\n");
        }
    }

    fprintf(stderr, "flarm: initialized successfully (center freq: %.1f MHz, sample rate: %.1f MSPS%s)\n",
            center_freq / 1e6, sample_rate / 1e6,
            FlarmConfig.p3i_enabled ? ", P3I enabled" : "");
    return true;

#else
    fprintf(stderr, "flarm: RTL-SDR support not compiled in\n");
    return false;
#endif
}

// ======================== Start ========================

void flarmReaderStart(void)
{
    if (!FlarmConfig.enabled) return;

    // Create thread-safe message queues
    if (!Flarm.msg_queue) Flarm.msg_queue = msg_queue_create(sizeof(flarm_message_t), FLARM_MSG_QUEUE_SIZE);
    if (!Flarm.ogntp_queue) Flarm.ogntp_queue = msg_queue_create(sizeof(ogntp_message_t), FLARM_MSG_QUEUE_SIZE);
    if (!Flarm.p3i_queue) Flarm.p3i_queue = msg_queue_create(sizeof(p3i_message_t), FLARM_MSG_QUEUE_SIZE);
    if (!Flarm.adsl_queue) Flarm.adsl_queue = msg_queue_create(sizeof(adsl_message_t), FLARM_MSG_QUEUE_SIZE);

    // Register dispatcher queue for FLARM/OGNTP/P3I/ADS-L aircraft updates
    if (!flarm_aircraft_queue) {
        flarm_aircraft_queue = dispatcher_register_aircraft_queue("flarm");
    }

    // IQ file mode
    if (FlarmConfig.ifile_path[0] != '\0') {
        if (!Flarm.demod) return;
        Flarm.stop_flag = 0;
        Flarm.thread_running = 1;
        pthread_create(&Flarm.thread, NULL, flarm_ifile_reader_thread, NULL);
        fprintf(stderr, "flarm-ifile: reader thread started\n");
        return;
    }

#ifdef ENABLE_RTLSDR
    if (!Flarm.dev) return;

    Flarm.stop_flag = 0;
    Flarm.thread_running = 1;
    pthread_create(&Flarm.thread, NULL, flarm_reader_thread, NULL);
    fprintf(stderr, "flarm: reader thread started\n");
#endif
}

// ======================== Stop ========================

void flarmReaderStop(void)
{
    if (!FlarmConfig.enabled) return;

    if (!Flarm.thread_running) return;

    Flarm.stop_flag = 1;

#ifdef ENABLE_RTLSDR
    if (Flarm.dev) {
        rtlsdr_cancel_async(Flarm.dev);
    }
#endif

    pthread_join(Flarm.thread, NULL);
    Flarm.thread_running = 0;
    fprintf(stderr, "flarm: reader thread stopped\n");
}

// ======================== Close ========================

void flarmReaderClose(void)
{
    flarmReaderStop();

    if (Flarm.demod) {
        // Print final stats
        flarm_demod_stats_t stats;
        flarm_demod_get_stats(Flarm.demod, &stats);
        fprintf(stderr, "flarm: final stats: samples=%" PRIu64 " detected=%" PRIu64 " crc_ok=%" PRIu64 " decoded=%" PRIu64 " failed=%" PRIu64 " type1=%" PRIu64 " type3=%" PRIu64 " type4=%" PRIu64 "\n",
                (uint64_t)stats.samples_processed,
                (uint64_t)stats.packets_detected,
                (uint64_t)stats.packets_crc_ok,
                (uint64_t)stats.packets_decoded,
                (uint64_t)stats.packets_failed,
                (uint64_t)stats.packets_type1,
                (uint64_t)stats.packets_type3,
                (uint64_t)stats.packets_type4);
        fprintf(stderr, "flarm: OGNTP stats: detected=%" PRIu64 " ldpc_ok=%" PRIu64 " decoded=%" PRIu64 " failed=%" PRIu64 "\n",
                (uint64_t)stats.ogntp_packets_detected,
                (uint64_t)stats.ogntp_packets_ldpc_ok,
                (uint64_t)stats.ogntp_packets_decoded,
                (uint64_t)stats.ogntp_packets_failed);
        fprintf(stderr, "flarm: ADS-L stats: detected=%" PRIu64 " crc_ok=%" PRIu64 " decoded=%" PRIu64 " failed=%" PRIu64 "\n",
                (uint64_t)stats.adsl_packets_detected,
                (uint64_t)stats.adsl_packets_crc_ok,
                (uint64_t)stats.adsl_packets_decoded,
                (uint64_t)stats.adsl_packets_failed);

        flarm_demod_destroy(Flarm.demod);
        Flarm.demod = NULL;
    }

    if (Flarm.p3i_demod) {
        p3i_demod_stats_t p3i_stats;
        p3i_demod_get_stats(Flarm.p3i_demod, &p3i_stats);
        fprintf(stderr, "flarm: P3I stats: samples=%" PRIu64 " sync=%" PRIu64 " decoded=%" PRIu64 " failed=%" PRIu64 "\n",
                (uint64_t)p3i_stats.samples_processed,
                (uint64_t)p3i_stats.sync_detected,
                (uint64_t)p3i_stats.packets_decoded,
                (uint64_t)p3i_stats.packets_failed);
        p3i_demod_destroy(Flarm.p3i_demod);
        Flarm.p3i_demod = NULL;
    }

#ifdef ENABLE_RTLSDR
    if (Flarm.dev) {
        rtlsdr_close(Flarm.dev);
        Flarm.dev = NULL;
    }
#endif

    // Destroy message queues
    if (Flarm.msg_queue) { msg_queue_destroy(Flarm.msg_queue); Flarm.msg_queue = NULL; }
    if (Flarm.ogntp_queue) { msg_queue_destroy(Flarm.ogntp_queue); Flarm.ogntp_queue = NULL; }
    if (Flarm.p3i_queue) { msg_queue_destroy(Flarm.p3i_queue); Flarm.p3i_queue = NULL; }
    if (Flarm.adsl_queue) { msg_queue_destroy(Flarm.adsl_queue); Flarm.adsl_queue = NULL; }
}

// ======================== Synthetic Mode S message generation =================

// Map FLARM/OGN aircraft type to ADS-B emitter category (DO-260B)
static uint32_t flarm_to_adsb_category(uint8_t aircraft_type)
{
    switch (aircraft_type) {
        case FLARM_ACFT_GLIDER:
        case FLARM_ACFT_HANGGLIDER:
        case FLARM_ACFT_PARAGLIDER:
            return 0xB1;  // Glider/sailplane
        case FLARM_ACFT_HELICOPTER:
            return 0xA7;  // Rotorcraft
        case FLARM_ACFT_BALLOON:
        case FLARM_ACFT_ZEPPELIN:
            return 0xB2;  // Lighter-than-air
        case FLARM_ACFT_PARACHUTE:
            return 0xB3;  // Parachutist/skydiver
        case FLARM_ACFT_UAV:
            return 0xB6;  // UAV
        case FLARM_ACFT_POWERED:
        case FLARM_ACFT_TOWPLANE:
        case FLARM_ACFT_DROPPLANE:
            return 0xA1;  // Light aircraft (<15500 lbs)
        case FLARM_ACFT_JET:
            return 0xA3;  // Large aircraft (75000-300000 lbs)
        case FLARM_ACFT_STATIC:
            return 0xB4;  // Ground obstruction
        case FLARM_ACFT_RESERVED:
            return 0xC1;  // Emergency/surface vehicle
        default:
            return 0xC0;  // No info
    }
}

// ======================== Periodic work (main thread) ========================

// Push a decoded FLARM/OGNTP/P3I/ADS-L message as aircraft_update_t
// through the dispatcher queue (bypasses synthetic DF18 generation).
static void flarm_push_update(
    uint32_t addr,
    const char *callsign,
    uint32_t category,
    double lat, double lon,
    int32_t altitude_m,         // meters MSL (converted to feet internally)
    float speed_ms,         // m/s ground speed
    float course_deg,       // degrees track
    float vs_ms,            // m/s vertical speed
    double signal,
    uint8_t acft_type,
    uint8_t addr_type,
    uint8_t proto_version,
    decode_source_t source)
{
    if (!flarm_aircraft_queue) return;

    aircraft_update_t upd;
    memset(&upd, 0, sizeof(upd));

    upd.addr = addr;
    upd.timestamp_ms = mstime();
    upd.signal_level = signal;
    upd.source = source;

    // Callsign
    if (callsign && callsign[0]) {
        strncpy(upd.callsign, callsign, sizeof(upd.callsign) - 1);
        upd.callsign_valid = 1;
    }

    // Category
    if (category > 0) {
        upd.category = category;
        upd.category_valid = 1;
    }

    // Position
    if (lat != 0 && lon != 0) {
        upd.lat = lat;
        upd.lon = lon;
        upd.position_valid = 1;
    }

    // Altitude (convert m → ft)
    if (altitude_m != 0 || (lat != 0 && lon != 0)) {
        upd.altitude_ft = (int32_t)(altitude_m * 3.28084);
        upd.altitude_valid = 1;
        upd.altitude_is_baro = 0;  // FLARM uses geometric (GPS) altitude
    }

    // Velocity
    if (speed_ms > 0.1f || fabsf(vs_ms) > 0.1f) {
        upd.ground_speed_kt = speed_ms * 1.94384f;
        upd.heading_deg = course_deg;
        upd.vert_rate_fpm = (int32_t)(vs_ms * 196.85f);
        upd.velocity_valid = 1;
    }

    // Air/ground
    upd.air_ground = DECODE_AG_AIRBORNE;

    // FLARM metadata
    upd.flarm_acft_type = acft_type;
    upd.flarm_addr_type = addr_type;
    upd.flarm_proto_version = proto_version;

    dispatcher_push_aircraft(flarm_aircraft_queue, &upd);
}

// Convert decoded FLARM messages into synthetic DF18 TIS-B messages
// and feed them through the standard Mode S pipeline (tracking + Beast output).
// Called from backgroundTasks() in the main thread.

void flarmReaderPeriodicWork(void)
{
    if (!FlarmConfig.enabled) return;

    flarm_message_t msg;
    while (flarm_dequeue_message(&msg)) {
        if (!msg.valid) continue;

        // Submit to OGN APRS-IS feed
        ognClientSubmit(&msg);

        uint32_t addr = msg.addr & 0xFFFFFF;

        double signal = msg.signal_level;
        if (signal <= 0) signal = 0.001;
        if (signal > 1.0) signal = 1.0;

        uint32_t category = flarm_to_adsb_category(msg.aircraft_type);

        char callsign[9];
        snprintf(callsign, sizeof(callsign), "FLR%05X", addr & 0xFFFFF);

        flarm_push_update(addr, callsign, category,
                          msg.latitude, msg.longitude, msg.altitude,
                          msg.speed, msg.course, msg.vs,
                          signal, msg.aircraft_type, msg.addr_type,
                          msg.version, DECODE_SOURCE_FLARM);
    }

    // ---- Drain OGNTP queue ----
    ogntp_message_t omsg;
    while (ogntp_dequeue_message(&omsg)) {
        if (!omsg.valid) continue;
        if (omsg.status_valid) {
            ogntp_log_status(&omsg);
        }
        if (!omsg.position_valid) continue;

        uint32_t addr = omsg.addr & 0xFFFFFF;

        double signal = omsg.signal_level;
        if (signal <= 0) signal = 0.001;
        if (signal > 1.0) signal = 1.0;

        uint32_t category = flarm_to_adsb_category(omsg.aircraft_type);

        char callsign[9];
        snprintf(callsign, sizeof(callsign), "OGN%05X", addr & 0xFFFFF);

        flarm_push_update(addr, callsign, category,
                          omsg.latitude, omsg.longitude, omsg.altitude,
                          omsg.speed, omsg.course, omsg.vs,
                          signal, omsg.aircraft_type, omsg.addr_type,
                          omsg.version, DECODE_SOURCE_OGNTP);
    }

    // ---- Drain P3I queue ----
    p3i_message_t pmsg;
    while (p3i_dequeue_message(&pmsg)) {
        if (!pmsg.valid) continue;

        uint32_t addr = pmsg.addr & 0xFFFFFF;

        double signal = pmsg.signal_level;
        if (signal <= 0) signal = 0.001;
        if (signal > 1.0) signal = 1.0;

        uint32_t category = flarm_to_adsb_category(pmsg.aircraft_type);

        char callsign[9];
        snprintf(callsign, sizeof(callsign), "PAW%05X", addr & 0xFFFFF);

        flarm_push_update(addr, callsign, category,
                          pmsg.latitude, pmsg.longitude, pmsg.altitude,
                          pmsg.speed, pmsg.course, 0,
                          signal, pmsg.aircraft_type, 0,
                          0, DECODE_SOURCE_P3I);
    }

    // ---- Drain ADS-L queue ----
    adsl_message_t amsg;
    while (adsl_dequeue_message(&amsg)) {
        if (!amsg.valid) continue;

        uint32_t addr = amsg.addr & 0xFFFFFF;

        double signal = amsg.signal_level;
        if (signal <= 0) signal = 0.001;
        if (signal > 1.0) signal = 1.0;

        uint32_t category = flarm_to_adsb_category(amsg.aircraft_type);

        char callsign[9];
        snprintf(callsign, sizeof(callsign), "ADL%05X", addr & 0xFFFFF);

        flarm_push_update(addr, callsign, category,
                          amsg.latitude, amsg.longitude, amsg.altitude,
                          amsg.speed, amsg.course, amsg.vs,
                          signal, amsg.aircraft_type, amsg.addr_type,
                          0, DECODE_SOURCE_ADSL);
    }
}

// ======================== SdrManager decoder_ops for FLARM ========================
//
// These allow FLARM reception to be managed by SdrManager instead of the
// standalone Flarm.dev / Flarm.thread subsystem. The RTL-SDR device is
// opened and the reader thread is managed by SdrManager; these ops only
// handle the demodulator, queue, and message processing.

#include "sdr_receiver.h"
#include "ogn_client.h"

#define FLARM_DEC_QUEUE_SIZE 64

typedef struct {
    struct flarm_state *demod;
    msg_queue_t         queue;
    struct p3i_demod_state *p3i_demod;
    msg_queue_t         p3i_queue;
    volatile int32_t        active;   // set by process callback, cleared by drain
} flarm_decoder_state_t;

static void flarm_dec_enqueue(const flarm_message_t *msg, void *ctx)
{
    flarm_decoder_state_t *st = (flarm_decoder_state_t *)ctx;
    msg_queue_push(st->queue, msg);
}

static void flarm_dec_p3i_enqueue(const p3i_message_t *msg, void *ctx)
{
    flarm_decoder_state_t *st = (flarm_decoder_state_t *)ctx;
    if (st->p3i_queue) msg_queue_push(st->p3i_queue, msg);
}

static bool flarm_dec_dequeue(flarm_decoder_state_t *st, flarm_message_t *out)
{
    return msg_queue_pop(st->queue, out) != 0;
}

bool flarmDecoderInit(struct sdr_receiver *rx)
{
    flarm_decoder_state_t *st = calloc(1, sizeof(*st));
    if (!st) return false;

    st->queue = msg_queue_create(sizeof(flarm_message_t), FLARM_DEC_QUEUE_SIZE);
    if (!st->queue) { free(st); return false; }

    flarm_demod_config_t cfg = {
        .ref_lat      = Modes.fUserLat,
        .ref_lon      = Modes.fUserLon,
        .ref_alt_geoid = 0,
        .center_freq  = (uint32_t)rx->config.freq,
        .sample_rate  = (uint32_t)rx->config.sample_rate,
        .callback     = flarm_dec_enqueue,
        .callback_ctx = st,
        .ogntp_callback     = NULL,    // OGNTP not supported via SdrManager yet
        .ogntp_callback_ctx = NULL
    };

    st->demod = flarm_demod_create(&cfg);
    if (!st->demod) {
        free(st);
        return false;
    }

    // For virtual file devices, set time override from file mtime
    if (rx->config.ifile_path[0] != '\0') {
        struct stat file_st;
        if (stat(rx->config.ifile_path, &file_st) == 0) {
            flarm_demod_set_time_override(st->demod, (uint32_t)file_st.st_mtime);
            fprintf(stderr, "rx[%d]: FLARM using file mtime %u for decryption\n",
                    rx->id, (uint32_t)file_st.st_mtime);
        }
    }

    rx->decoder_state = st;

    // Create P3I demodulator if enabled
    if (FlarmConfig.p3i_enabled) {
        st->p3i_queue = msg_queue_create(sizeof(p3i_message_t), FLARM_DEC_QUEUE_SIZE);
        if (st->p3i_queue) {
            p3i_demod_config_t p3i_cfg = {
                .sample_rate  = (uint32_t)rx->config.sample_rate,
                .center_freq  = (uint32_t)rx->config.freq,
                .callback     = flarm_dec_p3i_enqueue,
                .callback_ctx = st
            };
            st->p3i_demod = p3i_demod_create(&p3i_cfg);
            if (st->p3i_demod) {
                fprintf(stderr, "rx[%d]: P3I decoder enabled (869.525 MHz)\n", rx->id);
            }
        }
    }

    // Initialize OGN client if station is configured
    if (FlarmConfig.ogn_station[0]) {
        ognClientInit();
    }

    // Register dispatcher queue for FLARM aircraft updates
    if (!flarm_aircraft_queue) {
        flarm_aircraft_queue = dispatcher_register_aircraft_queue("flarm");
    }

    fprintf(stderr, "rx[%d]: FLARM decoder created (868 MHz GFSK)\n", rx->id);
    return true;
}

void flarmDecoderProcess(struct sdr_receiver *rx, const uint8_t *iq, uint32_t len)
{
    flarm_decoder_state_t *st = (flarm_decoder_state_t *)rx->decoder_state;
    if (!st || !st->demod) return;
    st->active = 1;
    flarm_demod_process(st->demod, iq, len);
    if (st->p3i_demod) p3i_demod_process(st->p3i_demod, iq, len);
}

bool flarmDecoderDrain(struct sdr_receiver *rx)
{
    flarm_decoder_state_t *st = (flarm_decoder_state_t *)rx->decoder_state;
    if (!st) return false;

    // If process callback ran since last drain, receiver is alive
    bool had_data = false;
    if (st->active) {
        st->active = 0;
        had_data = true;
    }

    flarm_message_t msg;
    while (flarm_dec_dequeue(st, &msg)) {
        if (!msg.valid) continue;
        had_data = true;

        // Submit to OGN
        ognClientSubmit(&msg);

        uint32_t addr = msg.addr & 0xFFFFFF;
        double signal = msg.signal_level;
        if (signal <= 0) signal = 0.001;
        if (signal > 1.0) signal = 1.0;

        uint32_t category = flarm_to_adsb_category(msg.aircraft_type);

        char callsign[9];
        snprintf(callsign, sizeof(callsign), "FLR%05X", addr & 0xFFFFF);

        flarm_push_update(addr, callsign, category,
                          msg.latitude, msg.longitude, msg.altitude,
                          msg.speed, msg.course, msg.vs,
                          signal, msg.aircraft_type, msg.addr_type,
                          msg.version, DECODE_SOURCE_FLARM);
    }

    // Drain P3I queue
    if (st->p3i_queue) {
        p3i_message_t pmsg;
        while (msg_queue_pop(st->p3i_queue, &pmsg) != 0) {
            if (!pmsg.valid) continue;
            had_data = true;

            uint32_t addr = pmsg.addr & 0xFFFFFF;
            double signal = pmsg.signal_level;
            if (signal <= 0) signal = 0.001;
            if (signal > 1.0) signal = 1.0;

            uint32_t category = flarm_to_adsb_category(pmsg.aircraft_type);

            char callsign[9];
            snprintf(callsign, sizeof(callsign), "PAW%05X", addr & 0xFFFFF);

            flarm_push_update(addr, callsign, category,
                              pmsg.latitude, pmsg.longitude, pmsg.altitude,
                              pmsg.speed, pmsg.course, 0,
                              signal, pmsg.aircraft_type, 0,
                              0, DECODE_SOURCE_P3I);
        }
    }

    return had_data;
}

void flarmDecoderStop(struct sdr_receiver *rx)
{
    flarm_decoder_state_t *st = (flarm_decoder_state_t *)rx->decoder_state;
    if (!st) return;

    if (st->demod) {
        flarm_demod_stats_t stats;
        flarm_demod_get_stats(st->demod, &stats);
        fprintf(stderr, "rx[%d]: FLARM stats: samples=%" PRIu64 " detected=%" PRIu64 " crc_ok=%" PRIu64 " decoded=%" PRIu64 " failed=%" PRIu64 " type1=%" PRIu64 " type3=%" PRIu64 " type4=%" PRIu64 "\n",
                rx->id,
                (uint64_t)stats.samples_processed,
                (uint64_t)stats.packets_detected,
                (uint64_t)stats.packets_crc_ok,
                (uint64_t)stats.packets_decoded,
                (uint64_t)stats.packets_failed,
                (uint64_t)stats.packets_type1,
                (uint64_t)stats.packets_type3,
                (uint64_t)stats.packets_type4);
        flarm_demod_destroy(st->demod);
    }

    if (st->p3i_demod) {
        p3i_demod_stats_t p3i_stats;
        p3i_demod_get_stats(st->p3i_demod, &p3i_stats);
        fprintf(stderr, "rx[%d]: P3I stats: sync=%" PRIu64 " decoded=%" PRIu64 " failed=%" PRIu64 "\n",
                rx->id,
                (uint64_t)p3i_stats.sync_detected,
                (uint64_t)p3i_stats.packets_decoded,
                (uint64_t)p3i_stats.packets_failed);
        p3i_demod_destroy(st->p3i_demod);
    }
    if (st->p3i_queue) msg_queue_destroy(st->p3i_queue);

    if (st->queue) msg_queue_destroy(st->queue);
    free(st);
    rx->decoder_state = NULL;
    fprintf(stderr, "rx[%d]: FLARM decoder destroyed\n", rx->id);
}

bool flarmDecoderGetStats(struct sdr_receiver *rx, flarm_demod_stats_t *stats)
{
    if (!rx || !rx->decoder_state) return false;
    flarm_decoder_state_t *st = (flarm_decoder_state_t *)rx->decoder_state;
    if (!st->demod) return false;
    flarm_demod_get_stats(st->demod, stats);
    return true;
}

// ======================== CLI option handling ========================

void flarmReaderShowHelp(void)
{
    printf(
        "\n"
        "      FLARM 868 MHz decoder options\n"
        "\n"
        "--flarm                      Enable FLARM 868 MHz decoder\n"
        "--flarm-device <serial>      RTL-SDR serial number for 868 MHz dongle\n"
        "--flarm-ifile <path>         Read raw IQ from file instead of RTL-SDR (uint8 I/Q, 1.6 MSPS, loops)\n"
        "--flarm-ifile-once          Stop after one --flarm-ifile replay pass\n"
        "--flarm-gain <dB>            Gain in dB (0 = auto, default: auto)\n"
        "--flarm-ppm <correction>     Frequency correction in PPM\n"
        "--ogn-station <name>         OGN station name for APRS-IS feed\n"
        "--ogn-server <host>          OGN APRS-IS server (default: aprs.glidernet.org)\n"
        "--ogn-port <port>            OGN APRS-IS port (default: 14580)\n"
        "--p3i                        Enable P3I (PilotAware) decoder on 869.525 MHz\n"
        "\n"
    );
}

bool flarmReaderHandleOption(int argc, char **argv, int *jptr)
{
    int32_t j = *jptr;
    bool more = (j + 1 < argc);

    if (!strcmp(argv[j], "--flarm")) {
        FlarmConfig.enabled = 1;
    } else if (!strcmp(argv[j], "--flarm-device") && more) {
        FlarmConfig.enabled = 1;
        strncpy(FlarmConfig.device_serial, argv[++j], sizeof(FlarmConfig.device_serial) - 1);
    } else if (!strcmp(argv[j], "--flarm-ifile") && more) {
        FlarmConfig.enabled = 1;
        strncpy(FlarmConfig.ifile_path, argv[++j], sizeof(FlarmConfig.ifile_path) - 1);
    } else if (!strcmp(argv[j], "--flarm-ifile-once")) {
        FlarmConfig.enabled = 1;
        FlarmConfig.ifile_once = 1;
    } else if (!strcmp(argv[j], "--flarm-gain") && more) {
        float gain_db = atof(argv[++j]);
        FlarmConfig.gain = (int32_t)(gain_db * 10);
    } else if (!strcmp(argv[j], "--flarm-ppm") && more) {
        FlarmConfig.ppm_error = atoi(argv[++j]);
    } else if (!strcmp(argv[j], "--flarm-keys") && more) {
        strncpy(FlarmConfig.keys_file, argv[++j], sizeof(FlarmConfig.keys_file) - 1);
    } else if (!strcmp(argv[j], "--ogn-station") && more) {
        strncpy(FlarmConfig.ogn_station, argv[++j], sizeof(FlarmConfig.ogn_station) - 1);
    } else if (!strcmp(argv[j], "--ogn-server") && more) {
        strncpy(FlarmConfig.ogn_server, argv[++j], sizeof(FlarmConfig.ogn_server) - 1);
    } else if (!strcmp(argv[j], "--ogn-port") && more) {
        FlarmConfig.ogn_port = atoi(argv[++j]);
    } else if (!strcmp(argv[j], "--p3i")) {
        FlarmConfig.p3i_enabled = 1;
    } else {
        return false;
    }

    *jptr = j;
    return true;
}
