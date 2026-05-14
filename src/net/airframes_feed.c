// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// airframes_feed.c: ACARS/VDL2 UDP JSON feed to airframes.io
//
// Sends decoded ACARS messages in acarsdec-compatible JSON format
// and VDL2 messages in dumpvdl2-compatible JSON format via UDP.
//
// ACARS default: feed.acars.io:5550
// VDL2  default: feed.acars.io:5552
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#include "dump1090.h"
#include "airframes_feed.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

static int acars_fd = -1;
static int vdl2_fd  = -1;
static struct sockaddr_in acars_addr;
static struct sockaddr_in vdl2_addr;
static uint64_t acars_msgs_sent = 0;
static uint64_t vdl2_msgs_sent  = 0;

// Resolve hostname to sockaddr_in, returns 0 on success
static int resolve_host(const char *host, int port, struct sockaddr_in *out)
{
    memset(out, 0, sizeof(*out));
    out->sin_family = AF_INET;
    out->sin_port = htons((uint16_t)port);

    // Try numeric first
    if (inet_pton(AF_INET, host, &out->sin_addr) == 1)
        return 0;

    // DNS lookup
    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    int err = getaddrinfo(host, NULL, &hints, &res);
    if (err != 0 || !res)
        return -1;

    struct sockaddr_in *sin = (struct sockaddr_in *)res->ai_addr;
    out->sin_addr = sin->sin_addr;
    freeaddrinfo(res);
    return 0;
}

void airframesFeedInit(void)
{
    // Initialize ACARS feed socket
    if (Modes.airframes_acars_feed.enabled && Modes.airframes_acars_feed.host) {
        if (resolve_host(Modes.airframes_acars_feed.host,
                         Modes.airframes_acars_feed.port, &acars_addr) == 0) {
            acars_fd = socket(AF_INET, SOCK_DGRAM, 0);
            if (acars_fd >= 0) {
                fprintf(stderr, "Airframes ACARS feed: %s:%d (station: %s)\n",
                        Modes.airframes_acars_feed.host,
                        Modes.airframes_acars_feed.port,
                        Modes.airframes_station_id);
            } else {
                fprintf(stderr, "Airframes ACARS feed: socket failed: %s\n",
                        strerror(errno));
            }
        } else {
            fprintf(stderr, "Airframes ACARS feed: cannot resolve %s\n",
                    Modes.airframes_acars_feed.host);
        }
    }

    // Initialize VDL2 feed socket
    if (Modes.airframes_vdl2_feed.enabled && Modes.airframes_vdl2_feed.host) {
        if (resolve_host(Modes.airframes_vdl2_feed.host,
                         Modes.airframes_vdl2_feed.port, &vdl2_addr) == 0) {
            vdl2_fd = socket(AF_INET, SOCK_DGRAM, 0);
            if (vdl2_fd >= 0) {
                fprintf(stderr, "Airframes VDL2 feed: %s:%d (station: %s)\n",
                        Modes.airframes_vdl2_feed.host,
                        Modes.airframes_vdl2_feed.port,
                        Modes.airframes_station_id);
            } else {
                fprintf(stderr, "Airframes VDL2 feed: socket failed: %s\n",
                        strerror(errno));
            }
        } else {
            fprintf(stderr, "Airframes VDL2 feed: cannot resolve %s\n",
                    Modes.airframes_vdl2_feed.host);
        }
    }
}

// Escape a string for JSON output. Returns number of bytes written (excluding NUL).
static int json_escape_str(char *out, int out_size, const char *in)
{
    int pos = 0;
    for (; *in && pos < out_size - 2; in++) {
        unsigned char c = (unsigned char)*in;
        if (c == '"' || c == '\\') {
            if (pos + 2 >= out_size) break;
            out[pos++] = '\\';
            out[pos++] = (char)c;
        } else if (c == '\n') {
            if (pos + 2 >= out_size) break;
            out[pos++] = '\\';
            out[pos++] = 'n';
        } else if (c == '\r') {
            if (pos + 2 >= out_size) break;
            out[pos++] = '\\';
            out[pos++] = 'r';
        } else if (c == '\t') {
            if (pos + 2 >= out_size) break;
            out[pos++] = '\\';
            out[pos++] = 't';
        } else if (c < 0x20) {
            // Skip other control characters
        } else {
            out[pos++] = (char)c;
        }
    }
    out[pos] = '\0';
    return pos;
}

// Format and send ACARS message in acarsdec JSON format via UDP
void airframesFeedSendAcars(const acars_msg_t *msg)
{
    if (acars_fd < 0 || !Modes.airframes_acars_feed.enabled)
        return;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    double timestamp = ts.tv_sec + ts.tv_nsec / 1e9;

    char text_esc[2048];
    json_escape_str(text_esc, sizeof(text_esc), msg->text);

    char reg_esc[32];
    json_escape_str(reg_esc, sizeof(reg_esc), msg->reg);

    char flight_esc[16];
    json_escape_str(flight_esc, sizeof(flight_esc), msg->flight);

    // acarsdec JSON format
    char buf[4096];
    int len = snprintf(buf, sizeof(buf),
        "{\"timestamp\":%.3f,"
        "\"station_id\":\"%s\","
        "\"channel\":%d,"
        "\"freq\":%.3f,"
        "\"level\":%.1f,"
        "\"error\":%d,"
        "\"mode\":\"%c\","
        "\"label\":\"%s\","
        "\"block_id\":\"%c\","
        "\"ack\":%s,"
        "\"tail\":\".%s\","
        "\"flight\":\"%s\","
        "\"msgno\":\"%s\","
        "\"text\":\"%s\"}\n",
        timestamp,
        Modes.airframes_station_id,
        msg->channel,
        msg->freq / 1e6,
        msg->level,
        msg->errors,
        msg->mode ? msg->mode : '2',
        msg->label,
        msg->block_id ? msg->block_id : ' ',
        (msg->ack == 0x15) ? "true" : "false",   // NAK = no ack
        reg_esc,
        flight_esc,
        msg->msgno,
        text_esc);

    if (len > 0 && len < (int)sizeof(buf)) {
        ssize_t n = sendto(acars_fd, buf, (size_t)len, 0,
                           (struct sockaddr *)&acars_addr, sizeof(acars_addr));
        if (n > 0)
            acars_msgs_sent++;
    }
}

// Format and send VDL2 message in dumpvdl2-compatible JSON format via UDP
void airframesFeedSendVdl2(const vdl2_msg_t *msg)
{
    if (vdl2_fd < 0 || !Modes.airframes_vdl2_feed.enabled)
        return;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    char text_esc[2048];
    json_escape_str(text_esc, sizeof(text_esc), msg->text);

    char reg_esc[32];
    json_escape_str(reg_esc, sizeof(reg_esc), msg->reg);

    char flight_esc[16];
    json_escape_str(flight_esc, sizeof(flight_esc), msg->flight);

    char buf[4096];
    int len;

    if (msg->has_acars) {
        // VDL2 frame with ACARS payload — full dumpvdl2-compatible JSON
        len = snprintf(buf, sizeof(buf),
            "{\"vdl2\":{"
            "\"app\":{\"name\":\"dump1090-gg\",\"ver\":\"" MODES_DUMP1090_VERSION "\"},"
            "\"t\":{\"sec\":%ld,\"usec\":%ld},"
            "\"freq\":%ld,"
            "\"sig_level\":%.1f,"
            "\"noise_level\":%.1f,"
            "\"station\":\"%s\","
            "\"avlc\":{"
            "\"src\":{\"addr\":\"%06X\",\"type\":\"Aircraft\"},"
            "\"dst\":{\"addr\":\"%06X\",\"type\":\"Ground station\"},"
            "\"acars\":{"
            "\"err\":false,"
            "\"crc_ok\":true,"
            "\"more\":false,"
            "\"reg\":\".%s\","
            "\"mode\":\"2\","
            "\"label\":\"%s\","
            "\"blk_id\":\"%c\","
            "\"ack\":false,"
            "\"flight\":\"%s\","
            "\"msg_text\":\"%s\""
            "}"   // acars
            "}"   // avlc
            "}}\n",  // vdl2
            (long)ts.tv_sec, (long)(ts.tv_nsec / 1000),
            (long)msg->freq,
            msg->level,
            msg->level - msg->snr,
            Modes.airframes_station_id,
            msg->src.addr,
            msg->dst.addr,
            reg_esc,
            msg->label,
            ' ',
            flight_esc,
            text_esc);
    } else {
        // Non-ACARS VDL2 frame — simplified format
        len = snprintf(buf, sizeof(buf),
            "{\"vdl2\":{"
            "\"app\":{\"name\":\"dump1090-gg\",\"ver\":\"" MODES_DUMP1090_VERSION "\"},"
            "\"t\":{\"sec\":%ld,\"usec\":%ld},"
            "\"freq\":%ld,"
            "\"sig_level\":%.1f,"
            "\"noise_level\":%.1f,"
            "\"station\":\"%s\","
            "\"avlc\":{"
            "\"src\":{\"addr\":\"%06X\"},"
            "\"dst\":{\"addr\":\"%06X\"},"
            "\"frame_type\":\"%s\""
            "}"   // avlc
            "}}\n",  // vdl2
            (long)ts.tv_sec, (long)(ts.tv_nsec / 1000),
            (long)msg->freq,
            msg->level,
            msg->level - msg->snr,
            Modes.airframes_station_id,
            msg->src.addr,
            msg->dst.addr,
            msg->frame_type);
    }

    if (len > 0 && len < (int)sizeof(buf)) {
        ssize_t n = sendto(vdl2_fd, buf, (size_t)len, 0,
                           (struct sockaddr *)&vdl2_addr, sizeof(vdl2_addr));
        if (n > 0)
            vdl2_msgs_sent++;
    }
}

void airframesFeedCleanup(void)
{
    if (acars_fd >= 0) {
        fprintf(stderr, "Airframes ACARS feed: %" PRIu64 " messages sent\n", acars_msgs_sent);
        close(acars_fd);
        acars_fd = -1;
    }
    if (vdl2_fd >= 0) {
        fprintf(stderr, "Airframes VDL2 feed: %" PRIu64 " messages sent\n", vdl2_msgs_sent);
        close(vdl2_fd);
        vdl2_fd = -1;
    }
}
