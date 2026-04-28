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
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <math.h>

#include "dump1090.h"
#include "ogntp_decode.h"

#ifdef ENABLE_RTLSDR
#include <rtl-sdr.h>
#endif

// ======================== Globals ========================

flarm_reader_config_t FlarmConfig;

// Thread-safe message queue: decoded FLARM messages to be consumed by main thread
#define FLARM_MSG_QUEUE_SIZE 64

static struct {
    pthread_t        thread;
    int              thread_running;

#ifdef ENABLE_RTLSDR
    rtlsdr_dev_t    *dev;
#endif

    struct flarm_state *demod;

    // Lock-free SPSC queue (single producer = demod callback, single consumer = main thread)
    flarm_message_t  msg_queue[FLARM_MSG_QUEUE_SIZE];
    volatile unsigned msg_queue_head;   // written by producer
    volatile unsigned msg_queue_tail;   // written by consumer

    // OGNTP message queue (same lock-free SPSC pattern)
    ogntp_message_t  ogntp_queue[FLARM_MSG_QUEUE_SIZE];
    volatile unsigned ogntp_queue_head;
    volatile unsigned ogntp_queue_tail;

    volatile int     stop_flag;
} Flarm;

// ======================== Config defaults ========================

void flarmReaderInitConfig(void)
{
    memset(&FlarmConfig, 0, sizeof(FlarmConfig));
    FlarmConfig.enabled = 0;
    FlarmConfig.gain = 0;  // auto
    FlarmConfig.ppm_error = 0;
    strncpy(FlarmConfig.ogn_server, "aprs.glidernet.org", sizeof(FlarmConfig.ogn_server) - 1);
    FlarmConfig.ogn_port = 14580;

    memset(&Flarm, 0, sizeof(Flarm));
}

// ======================== Message queue ========================

static void flarm_enqueue_message(const flarm_message_t *msg, void *ctx)
{
    (void)ctx;
    unsigned next_head = (Flarm.msg_queue_head + 1) % FLARM_MSG_QUEUE_SIZE;
    if (next_head == Flarm.msg_queue_tail) {
        // Queue full, drop oldest
        return;
    }
    Flarm.msg_queue[Flarm.msg_queue_head] = *msg;
    __sync_synchronize();  // memory barrier
    Flarm.msg_queue_head = next_head;
}

static bool flarm_dequeue_message(flarm_message_t *msg)
{
    if (Flarm.msg_queue_tail == Flarm.msg_queue_head) {
        return false;  // empty
    }
    *msg = Flarm.msg_queue[Flarm.msg_queue_tail];
    __sync_synchronize();
    Flarm.msg_queue_tail = (Flarm.msg_queue_tail + 1) % FLARM_MSG_QUEUE_SIZE;
    return true;
}

static void ogntp_enqueue_message(const ogntp_message_t *msg, void *ctx)
{
    (void)ctx;
    unsigned next_head = (Flarm.ogntp_queue_head + 1) % FLARM_MSG_QUEUE_SIZE;
    if (next_head == Flarm.ogntp_queue_tail) {
        return; // queue full, drop
    }
    Flarm.ogntp_queue[Flarm.ogntp_queue_head] = *msg;
    __sync_synchronize();
    Flarm.ogntp_queue_head = next_head;
}

static bool ogntp_dequeue_message(ogntp_message_t *msg)
{
    if (Flarm.ogntp_queue_tail == Flarm.ogntp_queue_head) {
        return false;
    }
    *msg = Flarm.ogntp_queue[Flarm.ogntp_queue_tail];
    __sync_synchronize();
    Flarm.ogntp_queue_tail = (Flarm.ogntp_queue_tail + 1) % FLARM_MSG_QUEUE_SIZE;
    return true;
}

// ======================== RTL-SDR callback ========================

#ifdef ENABLE_RTLSDR
static void flarm_rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
    (void)ctx;

    if (Flarm.stop_flag || Modes.exit) {
        rtlsdr_cancel_async(Flarm.dev);
        return;
    }

    if (Flarm.demod && len > 0) {
        flarm_demod_process(Flarm.demod, buf, len);
    }
}
#endif

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
static int find_flarm_device(const char *serial)
{
    int count = rtlsdr_get_device_count();
    if (count == 0) return -1;

    for (int i = 0; i < count; i++) {
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

#ifdef ENABLE_RTLSDR
    if (FlarmConfig.device_serial[0] == '\0') {
        fprintf(stderr, "flarm: no device serial specified (use --flarm-device)\n");
        return false;
    }

    int dev_index = find_flarm_device(FlarmConfig.device_serial);
    if (dev_index < 0) {
        fprintf(stderr, "flarm: no RTL-SDR device with serial '%s' found\n", FlarmConfig.device_serial);

        // List available devices
        int count = rtlsdr_get_device_count();
        fprintf(stderr, "flarm: available devices:\n");
        for (int i = 0; i < count; i++) {
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

    // Configure for FLARM reception
    rtlsdr_set_freq_correction(Flarm.dev, FlarmConfig.ppm_error);
    rtlsdr_set_center_freq(Flarm.dev, FLARM_CENTER_FREQ);
    rtlsdr_set_sample_rate(Flarm.dev, FLARM_SAMPLE_RATE);

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
        .callback     = flarm_enqueue_message,
        .callback_ctx = NULL,
        .ogntp_callback     = ogntp_enqueue_message,
        .ogntp_callback_ctx = NULL
    };

    Flarm.demod = flarm_demod_create(&demod_config);
    if (!Flarm.demod) {
        fprintf(stderr, "flarm: failed to create demodulator\n");
        rtlsdr_close(Flarm.dev);
        Flarm.dev = NULL;
        return false;
    }

    fprintf(stderr, "flarm: initialized successfully (center freq: %.1f MHz, sample rate: %.1f MSPS)\n",
            FLARM_CENTER_FREQ / 1e6, FLARM_SAMPLE_RATE / 1e6);
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

#ifdef ENABLE_RTLSDR
    if (!Flarm.thread_running) return;

    Flarm.stop_flag = 1;
    if (Flarm.dev) {
        rtlsdr_cancel_async(Flarm.dev);
    }

    pthread_join(Flarm.thread, NULL);
    Flarm.thread_running = 0;
    fprintf(stderr, "flarm: reader thread stopped\n");
#endif
}

// ======================== Close ========================

void flarmReaderClose(void)
{
    flarmReaderStop();

    if (Flarm.demod) {
        // Print final stats
        flarm_demod_stats_t stats;
        flarm_demod_get_stats(Flarm.demod, &stats);
        fprintf(stderr, "flarm: final stats: samples=%llu detected=%llu crc_ok=%llu decoded=%llu failed=%llu\n",
                (unsigned long long)stats.samples_processed,
                (unsigned long long)stats.packets_detected,
                (unsigned long long)stats.packets_crc_ok,
                (unsigned long long)stats.packets_decoded,
                (unsigned long long)stats.packets_failed);
        fprintf(stderr, "flarm: OGNTP stats: detected=%llu ldpc_ok=%llu decoded=%llu failed=%llu\n",
                (unsigned long long)stats.ogntp_packets_detected,
                (unsigned long long)stats.ogntp_packets_ldpc_ok,
                (unsigned long long)stats.ogntp_packets_decoded,
                (unsigned long long)stats.ogntp_packets_failed);

        flarm_demod_destroy(Flarm.demod);
        Flarm.demod = NULL;
    }

#ifdef ENABLE_RTLSDR
    if (Flarm.dev) {
        rtlsdr_close(Flarm.dev);
        Flarm.dev = NULL;
    }
#endif
}

// ======================== Synthetic Mode S message generation =================

// CPR NL function (number of longitude zones for a given latitude)
static int flarm_cprNL(double lat)
{
    if (lat < 0) lat = -lat;
    if (lat < 10.47047130)  return 59;
    if (lat < 14.82817437)  return 58;
    if (lat < 18.18626357)  return 57;
    if (lat < 21.02939493)  return 56;
    if (lat < 23.54504487)  return 55;
    if (lat < 25.82924707)  return 54;
    if (lat < 27.93898710)  return 53;
    if (lat < 29.91135686)  return 52;
    if (lat < 31.77209708)  return 51;
    if (lat < 33.53993436)  return 50;
    if (lat < 35.22899598)  return 49;
    if (lat < 36.85025108)  return 48;
    if (lat < 38.41241892)  return 47;
    if (lat < 39.92256684)  return 46;
    if (lat < 41.38651832)  return 45;
    if (lat < 42.80914012)  return 44;
    if (lat < 44.19454951)  return 43;
    if (lat < 45.54626723)  return 42;
    if (lat < 46.86733252)  return 41;
    if (lat < 48.16039128)  return 40;
    if (lat < 49.42776439)  return 39;
    if (lat < 50.67150166)  return 38;
    if (lat < 51.89342469)  return 37;
    if (lat < 53.09516153)  return 36;
    if (lat < 54.27817472)  return 35;
    if (lat < 55.44378444)  return 34;
    if (lat < 56.59318756)  return 33;
    if (lat < 57.72747354)  return 32;
    if (lat < 58.84763776)  return 31;
    if (lat < 59.95459277)  return 30;
    if (lat < 61.04917774)  return 29;
    if (lat < 62.13216659)  return 28;
    if (lat < 63.20427479)  return 27;
    if (lat < 64.26616523)  return 26;
    if (lat < 65.31845310)  return 25;
    if (lat < 66.36171008)  return 24;
    if (lat < 67.39646774)  return 23;
    if (lat < 68.42322022)  return 22;
    if (lat < 69.44242631)  return 21;
    if (lat < 70.45451075)  return 20;
    if (lat < 71.45986473)  return 19;
    if (lat < 72.45884545)  return 18;
    if (lat < 73.45177442)  return 17;
    if (lat < 74.43893416)  return 16;
    if (lat < 75.42056257)  return 15;
    if (lat < 76.39684391)  return 14;
    if (lat < 77.36789461)  return 13;
    if (lat < 78.33374083)  return 12;
    if (lat < 79.29428225)  return 11;
    if (lat < 80.24923213)  return 10;
    if (lat < 81.19801349)  return 9;
    if (lat < 82.13956981)  return 8;
    if (lat < 83.07199445)  return 7;
    if (lat < 83.99173563)  return 6;
    if (lat < 84.89166191)  return 5;
    if (lat < 85.75541621)  return 4;
    if (lat < 86.53536998)  return 3;
    if (lat < 87.00000000)  return 2;
    return 1;
}

// Positive fmod
static double cprMod(double a, double b)
{
    double res = fmod(a, b);
    if (res < 0) res += b;
    return res;
}

// Encode latitude/longitude into 17-bit CPR format
static void flarm_cpr_encode(double lat, double lon, int fflag,
                              unsigned *cpr_lat, unsigned *cpr_lon)
{
    double Dlat = fflag ? (360.0 / 59.0) : (360.0 / 60.0);

    double yz = floor(131072.0 * cprMod(lat, Dlat) / Dlat + 0.5);
    *cpr_lat = ((unsigned)yz) & 0x1FFFF;

    int nl = flarm_cprNL(lat) - fflag;
    if (nl < 1) nl = 1;
    double Dlon = 360.0 / nl;

    double xz = floor(131072.0 * cprMod(lon, Dlon) / Dlon + 0.5);
    *cpr_lon = ((unsigned)xz) & 0x1FFFF;
}

// Encode altitude in feet to AC12 field (25ft resolution, Q-bit encoding)
static unsigned flarm_encode_ac12(int alt_ft)
{
    int n = (alt_ft + 1000) / 25;
    if (n < 0) n = 0;
    if (n > 0x7FF) n = 0x7FF;
    // Re-insert Q bit at position 4
    return ((n & 0x7F0) << 1) | 0x10 | (n & 0x0F);
}

// ADS-B char → 6-bit AIS index
static unsigned char flarm_ais_encode(char c)
{
    // AIS charset: @ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_ !"#$%&'()*+,-./0123456789:;<=>?
    if (c >= 'A' && c <= 'Z') return (unsigned char)(c - 'A' + 1);
    if (c >= '0' && c <= '9') return (unsigned char)(c - '0' + 48);
    if (c == ' ') return 32;
    if (c == '@') return 0;
    return 32; // space for unknown
}

// Compute Mode S CRC-24 and set the PI field for DF18.
// For DF17/18, the PI field should make modesChecksum(msg, 112) == 0.
static void flarm_set_crc(unsigned char *msg, int bytes)
{
    // Set last 3 bytes (PI field) to 0 for CRC calculation
    msg[bytes - 3] = 0;
    msg[bytes - 2] = 0;
    msg[bytes - 1] = 0;

    // modesChecksum with PI=0 gives us: CRC(first 11 bytes) XOR 0 = raw CRC
    uint32_t crc = modesChecksum(msg, bytes * 8);

    // Set PI = raw CRC so that modesChecksum(msg, 112) = rawCRC XOR PI = 0
    msg[bytes - 3] = (crc >> 16) & 0xFF;
    msg[bytes - 2] = (crc >> 8) & 0xFF;
    msg[bytes - 1] = crc & 0xFF;
}

// Build a DF18 CF=5 (Fine TIS-B, non-ICAO) identity & category message
static void flarm_build_ident_msg(unsigned char *msg, uint32_t addr,
                                   const char *callsign, unsigned category)
{
    memset(msg, 0, 14);

    // DF=18 (5 bits), CF=5 (3 bits) → byte 0 = (18 << 3) | 5 = 0x95
    msg[0] = 0x95;

    // AA field (24-bit address)
    msg[1] = (addr >> 16) & 0xFF;
    msg[2] = (addr >> 8) & 0xFF;
    msg[3] = addr & 0xFF;

    // ME field (7 bytes, bytes 4-10)
    // metype from category: metype = 0x0E - (category >> 4)
    unsigned metype = 0x0E - (category >> 4);
    if (metype < 1) metype = 1;
    if (metype > 4) metype = 4;
    unsigned mesub = category & 0x0F;

    // ME bits 1-5: metype, bits 6-8: mesub
    msg[4] = (metype << 3) | (mesub & 0x07);

    // ME bits 9-56: 8 chars × 6 bits each
    // Callsign chars packed into bytes 5-10
    char cs[9];
    memset(cs, ' ', 8);
    cs[8] = 0;
    memcpy(cs, callsign, strlen(callsign) < 8 ? strlen(callsign) : 8);

    unsigned char ais[8];
    for (int i = 0; i < 8; i++)
        ais[i] = flarm_ais_encode(cs[i]);

    // Pack 8 × 6-bit values into 48 bits (6 bytes), starting at bit 9 of ME
    // Byte 5: bits 9-16 → ais[0](6) + ais[1] high 2
    msg[5] = (ais[0] << 2) | (ais[1] >> 4);
    // Byte 6: bits 17-24 → ais[1] low 4 + ais[2] high 4
    msg[6] = ((ais[1] & 0x0F) << 4) | (ais[2] >> 2);
    // Byte 7: bits 25-32 → ais[2] low 2 + ais[3](6)
    msg[7] = ((ais[2] & 0x03) << 6) | ais[3];
    // Byte 8: bits 33-40 → ais[4](6) + ais[5] high 2
    msg[8] = (ais[4] << 2) | (ais[5] >> 4);
    // Byte 9: bits 41-48 → ais[5] low 4 + ais[6] high 4
    msg[9] = ((ais[5] & 0x0F) << 4) | (ais[6] >> 2);
    // Byte 10: bits 49-56 → ais[6] low 2 + ais[7](6)
    msg[10] = ((ais[6] & 0x03) << 6) | ais[7];

    flarm_set_crc(msg, 14);
}

// Build a DF18 CF=5 airborne position message (metype=11)
static void flarm_build_position_msg(unsigned char *msg, uint32_t addr,
                                      double lat, double lon, int alt_ft,
                                      int fflag)
{
    memset(msg, 0, 14);

    msg[0] = 0x95;  // DF=18, CF=5
    msg[1] = (addr >> 16) & 0xFF;
    msg[2] = (addr >> 8) & 0xFF;
    msg[3] = addr & 0xFF;

    // ME field
    // metype=11 (airborne position, barometric altitude)
    unsigned metype = 11;
    unsigned ac12 = flarm_encode_ac12(alt_ft);

    unsigned cpr_lat, cpr_lon;
    flarm_cpr_encode(lat, lon, fflag, &cpr_lat, &cpr_lon);

    // ME byte 4 (bits 1-8): metype(5) + SS(2) + NIC-B(1)
    msg[4] = (metype << 3);  // SS=0, NIC-B=0

    // ME byte 5 (bits 9-16): AC12 high 8 bits
    msg[5] = (ac12 >> 4) & 0xFF;

    // ME byte 6 (bits 17-24): AC12 low 4 + T(1) + F(1) + CPR_LAT high 2
    msg[6] = ((ac12 & 0x0F) << 4) | (0 << 3) | (fflag << 2) | ((cpr_lat >> 15) & 0x03);

    // ME byte 7 (bits 25-32): CPR_LAT bits 14-7
    msg[7] = (cpr_lat >> 7) & 0xFF;

    // ME byte 8 (bits 33-40): CPR_LAT low 7 + CPR_LON bit 16
    msg[8] = ((cpr_lat & 0x7F) << 1) | ((cpr_lon >> 16) & 0x01);

    // ME byte 9 (bits 41-48): CPR_LON bits 15-8
    msg[9] = (cpr_lon >> 8) & 0xFF;

    // ME byte 10 (bits 49-56): CPR_LON low 8
    msg[10] = cpr_lon & 0xFF;

    flarm_set_crc(msg, 14);
}

// Build a DF18 CF=5 airborne velocity message (metype=19, subtype=1)
static void flarm_build_velocity_msg(unsigned char *msg, uint32_t addr,
                                      float gs_kts, float track_deg,
                                      int vrate_fpm)
{
    memset(msg, 0, 14);

    msg[0] = 0x95;  // DF=18, CF=5
    msg[1] = (addr >> 16) & 0xFF;
    msg[2] = (addr >> 8) & 0xFF;
    msg[3] = addr & 0xFF;

    // ME field: metype=19, subtype=1 (ground speed, subsonic)
    unsigned metype = 19;
    unsigned subtype = 1;

    // Decompose ground speed into E/W and N/S components
    double track_rad = track_deg * M_PI / 180.0;
    double ew_vel = gs_kts * sin(track_rad);
    double ns_vel = gs_kts * cos(track_rad);

    unsigned ew_sign = (ew_vel < 0) ? 1 : 0;
    unsigned ew_raw = (unsigned)(fabs(ew_vel) + 0.5) + 1;
    if (ew_raw > 1023) ew_raw = 1023;

    unsigned ns_sign = (ns_vel < 0) ? 1 : 0;
    unsigned ns_raw = (unsigned)(fabs(ns_vel) + 0.5) + 1;
    if (ns_raw > 1023) ns_raw = 1023;

    // Vertical rate encoding
    unsigned vr_source = 0;  // 0 = geometric
    unsigned vr_sign = (vrate_fpm < 0) ? 1 : 0;
    unsigned vr_raw = (unsigned)(abs(vrate_fpm) / 64) + 1;
    if (vr_raw > 511) vr_raw = 511;

    // Pack ME bytes 4-10
    // Byte 4 (ME bits 1-8): metype(5) + subtype(3)
    msg[4] = (metype << 3) | (subtype & 0x07);

    // Byte 5 (ME bits 9-16): IC(1) + resv(1) + NACv(3) + ew_sign(1) + ew_raw high 2
    // IC=0, resv=0, NACv=0
    msg[5] = (ew_sign << 2) | ((ew_raw >> 8) & 0x03);

    // Byte 6 (ME bits 17-24): ew_raw low 8
    msg[6] = ew_raw & 0xFF;

    // Byte 7 (ME bits 25-32): ns_sign(1) + ns_raw(10) → ns_sign(1) + ns_raw high 7
    msg[7] = (ns_sign << 7) | ((ns_raw >> 3) & 0x7F);

    // Byte 8 (ME bits 33-40): ns_raw low 3 + vr_source(1) + vr_sign(1) + vr_raw high 3
    msg[8] = ((ns_raw & 0x07) << 5) | (vr_source << 4) | (vr_sign << 3) | ((vr_raw >> 6) & 0x07);

    // Byte 9 (ME bits 41-48): vr_raw low 6 + reserved(2)
    msg[9] = ((vr_raw & 0x3F) << 2);

    // Byte 10 (ME bits 49-56): delta_sign(1) + delta(7) = 0 (no baro/geom delta)
    msg[10] = 0;

    flarm_set_crc(msg, 14);
}

// Submit a synthetic modesMessage through the standard decode pipeline
static void flarm_submit_synthetic(unsigned char *raw_msg, double signal_level)
{
    struct modesMessage mm;
    memset(&mm, 0, sizeof(mm));

    mm.timestampMsg = 12000000ULL * (mstime() / 1000);  // Approximate 12MHz timestamp
    mm.sysTimestampMsg = mstime();
    mm.signalLevel = signal_level;
    mm.score = SR_NOT_SET;  // Let scoreModesMessage() score based on CRC
    mm.remote = 0;

    // Let the standard Mode S decoder process this message
    // It will: decode DF18→extract fields, track aircraft, output to Beast/SBS/etc.
    int result = decodeModesMessage(&mm, raw_msg);
    if (result >= 0) {
        useModesMessage(&mm);
    }
}

// ======================== Periodic work (main thread) ========================

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

        // Use the raw FLARM address (24-bit) as the DF18 AA field
        uint32_t addr = msg.addr & 0xFFFFFF;

        double signal = msg.signal_level;
        if (signal <= 0) signal = 0.001;
        if (signal > 1.0) signal = 1.0;

        // Map FLARM aircraft type to ADS-B category
        unsigned category;
        switch (msg.aircraft_type) {
            case FLARM_ACFT_GLIDER:
            case FLARM_ACFT_HANGGLIDER:
            case FLARM_ACFT_PARAGLIDER:
                category = 0xB1;  // Glider/sailplane
                break;
            case FLARM_ACFT_HELICOPTER:
                category = 0xA7;  // Rotorcraft
                break;
            case FLARM_ACFT_BALLOON:
            case FLARM_ACFT_ZEPPELIN:
                category = 0xB2;  // Lighter-than-air
                break;
            case FLARM_ACFT_PARACHUTE:
                category = 0xB3;  // Parachutist
                break;
            case FLARM_ACFT_UAV:
                category = 0xB6;  // UAV
                break;
            case FLARM_ACFT_POWERED:
            case FLARM_ACFT_TOWPLANE:
            case FLARM_ACFT_DROPPLANE:
                category = 0xA1;  // Light aircraft
                break;
            case FLARM_ACFT_JET:
                category = 0xA3;  // Large aircraft
                break;
            default:
                category = 0xC0;  // No info
                break;
        }

        // Build callsign from FLARM address
        char callsign[9];
        snprintf(callsign, sizeof(callsign), "FLR%05X", addr & 0xFFFFF);

        unsigned char raw[14];

        // 1. Send identity & category message
        flarm_build_ident_msg(raw, addr, callsign, category);
        flarm_submit_synthetic(raw, signal);

        // 2. Send airborne position messages (both even AND odd for CPR resolution)
        if (msg.latitude != 0 && msg.longitude != 0) {
            int alt_ft = (int)(msg.altitude * 3.28084);
            // Even frame
            flarm_build_position_msg(raw, addr, msg.latitude, msg.longitude, alt_ft, 0);
            flarm_submit_synthetic(raw, signal);
            // Odd frame
            flarm_build_position_msg(raw, addr, msg.latitude, msg.longitude, alt_ft, 1);
            flarm_submit_synthetic(raw, signal);
        }

        // 3. Send velocity message
        if (msg.speed > 0.1f || fabsf(msg.vs) > 0.1f) {
            float gs_kts = msg.speed * 1.94384f;
            int vrate_fpm = (int)(msg.vs * 196.85f);
            flarm_build_velocity_msg(raw, addr, gs_kts, msg.course, vrate_fpm);
            flarm_submit_synthetic(raw, signal);
        }
    }

    // ---- Drain OGNTP queue ----
    ogntp_message_t omsg;
    while (ogntp_dequeue_message(&omsg)) {
        if (!omsg.valid) continue;

        uint32_t addr = omsg.addr & 0xFFFFFF;

        double signal = omsg.signal_level;
        if (signal <= 0) signal = 0.001;
        if (signal > 1.0) signal = 1.0;

        // Map OGN aircraft type to ADS-B category (same table as FLARM)
        unsigned category;
        switch (omsg.aircraft_type) {
            case FLARM_ACFT_GLIDER:
            case FLARM_ACFT_HANGGLIDER:
            case FLARM_ACFT_PARAGLIDER:
                category = 0xB1; break;
            case FLARM_ACFT_HELICOPTER:
                category = 0xA7; break;
            case FLARM_ACFT_BALLOON:
            case FLARM_ACFT_ZEPPELIN:
                category = 0xB2; break;
            case FLARM_ACFT_PARACHUTE:
                category = 0xB3; break;
            case FLARM_ACFT_UAV:
                category = 0xB6; break;
            case FLARM_ACFT_POWERED:
            case FLARM_ACFT_TOWPLANE:
            case FLARM_ACFT_DROPPLANE:
                category = 0xA1; break;
            case FLARM_ACFT_JET:
                category = 0xA3; break;
            default:
                category = 0xC0; break;
        }

        // OGN callsign: OGN prefix + lower 20 bits of address
        char callsign[9];
        snprintf(callsign, sizeof(callsign), "OGN%05X", addr & 0xFFFFF);

        unsigned char raw[14];

        // 1. Identity message
        flarm_build_ident_msg(raw, addr, callsign, category);
        flarm_submit_synthetic(raw, signal);

        // 2. Position messages (even and odd for CPR resolution)
        if (omsg.latitude != 0 && omsg.longitude != 0) {
            int alt_ft = (int)(omsg.altitude * 3.28084);
            flarm_build_position_msg(raw, addr, omsg.latitude, omsg.longitude, alt_ft, 0);
            flarm_submit_synthetic(raw, signal);
            flarm_build_position_msg(raw, addr, omsg.latitude, omsg.longitude, alt_ft, 1);
            flarm_submit_synthetic(raw, signal);
        }

        // 3. Velocity message
        if (omsg.speed > 0.1f || fabsf(omsg.vs) > 0.1f) {
            float gs_kts = omsg.speed * 1.94384f;
            int vrate_fpm = (int)(omsg.vs * 196.85f);
            flarm_build_velocity_msg(raw, addr, gs_kts, omsg.course, vrate_fpm);
            flarm_submit_synthetic(raw, signal);
        }
    }
}

// ======================== SdrManager decoder_ops for FLARM ========================
//
// These allow FLARM reception to be managed by SdrManager instead of the
// standalone Flarm.dev / Flarm.thread subsystem. The RTL-SDR device is
// opened and the reader thread is managed by SdrManager; these ops only
// handle the demodulator, SPSC queue, and message processing.

#include "sdr_receiver.h"
#include "ogn_client.h"

#define FLARM_DEC_QUEUE_SIZE 64

typedef struct {
    struct flarm_state *demod;
    flarm_message_t     queue[FLARM_DEC_QUEUE_SIZE];
    volatile unsigned   head;
    volatile unsigned   tail;
} flarm_decoder_state_t;

static void flarm_dec_enqueue(const flarm_message_t *msg, void *ctx)
{
    flarm_decoder_state_t *st = (flarm_decoder_state_t *)ctx;
    unsigned next = (st->head + 1) % FLARM_DEC_QUEUE_SIZE;
    if (next == st->tail) return;  // queue full, drop
    st->queue[st->head] = *msg;
    __sync_synchronize();
    st->head = next;
}

static bool flarm_dec_dequeue(flarm_decoder_state_t *st, flarm_message_t *out)
{
    if (st->tail == st->head) return false;
    *out = st->queue[st->tail];
    __sync_synchronize();
    st->tail = (st->tail + 1) % FLARM_DEC_QUEUE_SIZE;
    return true;
}

bool flarmDecoderInit(struct sdr_receiver *rx)
{
    flarm_decoder_state_t *st = calloc(1, sizeof(*st));
    if (!st) return false;

    flarm_demod_config_t cfg = {
        .ref_lat      = Modes.fUserLat,
        .ref_lon      = Modes.fUserLon,
        .ref_alt_geoid = 0,
        .callback     = flarm_dec_enqueue,
        .callback_ctx = st
    };

    st->demod = flarm_demod_create(&cfg);
    if (!st->demod) {
        free(st);
        return false;
    }

    rx->decoder_state = st;

    // Initialize OGN client if station is configured
    if (FlarmConfig.ogn_station[0]) {
        ognClientInit();
    }

    fprintf(stderr, "rx[%d]: FLARM decoder created (868 MHz GFSK)\n", rx->id);
    return true;
}

void flarmDecoderProcess(struct sdr_receiver *rx, const uint8_t *iq, uint32_t len)
{
    flarm_decoder_state_t *st = (flarm_decoder_state_t *)rx->decoder_state;
    if (!st || !st->demod) return;
    flarm_demod_process(st->demod, iq, len);
}

void flarmDecoderDrain(struct sdr_receiver *rx)
{
    flarm_decoder_state_t *st = (flarm_decoder_state_t *)rx->decoder_state;
    if (!st) return;

    flarm_message_t msg;
    while (flarm_dec_dequeue(st, &msg)) {
        if (!msg.valid) continue;

        // Submit to OGN
        ognClientSubmit(&msg);

        uint32_t addr = msg.addr & 0xFFFFFF;
        double signal = msg.signal_level;
        if (signal <= 0) signal = 0.001;
        if (signal > 1.0) signal = 1.0;

        // Map aircraft type to ADS-B category
        unsigned category;
        switch (msg.aircraft_type) {
            case FLARM_ACFT_GLIDER: case FLARM_ACFT_HANGGLIDER: case FLARM_ACFT_PARAGLIDER:
                category = 0xB1; break;
            case FLARM_ACFT_HELICOPTER: category = 0xA7; break;
            case FLARM_ACFT_BALLOON: case FLARM_ACFT_ZEPPELIN: category = 0xB2; break;
            case FLARM_ACFT_PARACHUTE: category = 0xB3; break;
            case FLARM_ACFT_UAV: category = 0xB6; break;
            case FLARM_ACFT_POWERED: case FLARM_ACFT_TOWPLANE: case FLARM_ACFT_DROPPLANE:
                category = 0xA1; break;
            case FLARM_ACFT_JET: category = 0xA3; break;
            default: category = 0xC0; break;
        }

        char callsign[9];
        snprintf(callsign, sizeof(callsign), "FLR%05X", addr & 0xFFFFF);

        unsigned char raw[14];

        // Identity & category
        flarm_build_ident_msg(raw, addr, callsign, category);
        flarm_submit_synthetic(raw, signal);

        // Position (even + odd CPR)
        if (msg.latitude != 0 && msg.longitude != 0) {
            int alt_ft = (int)(msg.altitude * 3.28084);
            flarm_build_position_msg(raw, addr, msg.latitude, msg.longitude, alt_ft, 0);
            flarm_submit_synthetic(raw, signal);
            flarm_build_position_msg(raw, addr, msg.latitude, msg.longitude, alt_ft, 1);
            flarm_submit_synthetic(raw, signal);
        }

        // Velocity
        if (msg.speed > 0.1f || fabsf(msg.vs) > 0.1f) {
            float gs_kts = msg.speed * 1.94384f;
            int vrate_fpm = (int)(msg.vs * 196.85f);
            flarm_build_velocity_msg(raw, addr, gs_kts, msg.course, vrate_fpm);
            flarm_submit_synthetic(raw, signal);
        }
    }
}

void flarmDecoderStop(struct sdr_receiver *rx)
{
    flarm_decoder_state_t *st = (flarm_decoder_state_t *)rx->decoder_state;
    if (!st) return;

    if (st->demod) {
        flarm_demod_stats_t stats;
        flarm_demod_get_stats(st->demod, &stats);
        fprintf(stderr, "rx[%d]: FLARM stats: samples=%llu detected=%llu crc_ok=%llu decoded=%llu failed=%llu\n",
                rx->id,
                (unsigned long long)stats.samples_processed,
                (unsigned long long)stats.packets_detected,
                (unsigned long long)stats.packets_crc_ok,
                (unsigned long long)stats.packets_decoded,
                (unsigned long long)stats.packets_failed);
        flarm_demod_destroy(st->demod);
    }

    free(st);
    rx->decoder_state = NULL;
    fprintf(stderr, "rx[%d]: FLARM decoder destroyed\n", rx->id);
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
        "--flarm-gain <dB>            Gain in dB (0 = auto, default: auto)\n"
        "--flarm-ppm <correction>     Frequency correction in PPM\n"
        "--ogn-station <name>         OGN station name for APRS-IS feed\n"
        "--ogn-server <host>          OGN APRS-IS server (default: aprs.glidernet.org)\n"
        "--ogn-port <port>            OGN APRS-IS port (default: 14580)\n"
        "\n"
    );
}

bool flarmReaderHandleOption(int argc, char **argv, int *jptr)
{
    int j = *jptr;
    bool more = (j + 1 < argc);

    if (!strcmp(argv[j], "--flarm")) {
        FlarmConfig.enabled = 1;
    } else if (!strcmp(argv[j], "--flarm-device") && more) {
        FlarmConfig.enabled = 1;
        strncpy(FlarmConfig.device_serial, argv[++j], sizeof(FlarmConfig.device_serial) - 1);
    } else if (!strcmp(argv[j], "--flarm-gain") && more) {
        float gain_db = atof(argv[++j]);
        FlarmConfig.gain = (int)(gain_db * 10);
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
    } else {
        return false;
    }

    *jptr = j;
    return true;
}
