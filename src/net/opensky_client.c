// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// opensky_client.c: OpenSky Network feeder client (native protocol)
//
// Protocol reimplemented from the open-source opensky-sensor daemon (v2.1.7, BSD 3-Clause).
// Source: https://github.com/openskynetwork/opensky-sensor
// Connects to collector.opensky-network.org:10004 and sends:
//   1. LOGIN frame: device type + version
//   2. SERIAL frame: stored serial or request new one
//   3. GPS frame: latitude, longitude, altitude (IEEE 754 doubles)
//   4. USERNAME frame
//   5. Beast binary ADS-B frames (same as beast feed)
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "dump1090.h"
#include <stdint.h>
#include "feeder_thread.h"
#include "opensky_client.h"

#include <sys/socket.h>
#include <netdb.h>
#include <fcntl.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <arpa/inet.h>

// ===================== Protocol constants =====================

#define OPENSKY_FRAME_SYNC     0x1A

// Client -> Server frame types
#define OPENSKY_TYPE_LOGIN     0x41  // 'A' — device type + version
#define OPENSKY_TYPE_SERIAL_REQ 0x42 // 'B' — request serial number
#define OPENSKY_TYPE_SERIAL    0x35  // '5' — send known serial
#define OPENSKY_TYPE_GPS       0x37  // '7' — GPS position (3x double)
#define OPENSKY_TYPE_USERNAME  0x43  // 'C' — username string

// Server -> Client: TLV with 2-byte type (BE) + 2-byte length (BE) + payload
#define OPENSKY_SRV_SERIAL     0x0005  // serial number assignment

// Device constants (matching openskyd-dump1090 v2.1.7)
#define OPENSKY_DEVICE_TYPE    5
#define OPENSKY_VERSION_MAJOR  2
#define OPENSKY_VERSION_MINOR  1
#define OPENSKY_VERSION_PATCH  7

// ===================== Global config =====================

struct opensky_config OpenSkyConfig;
msg_queue_t opensky_queue;

// ===================== Helpers =====================

static int opensky_tcp_connect(const char *host, int port) {
    struct addrinfo hints, *res, *rp;
    char port_str[16];
    int32_t fd = -1;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    snprintf(port_str, sizeof(port_str), "%d", port);
    if (getaddrinfo(host, port_str, &hints, &res) != 0)
        return -1;

    for (rp = res; rp; rp = rp->ai_next) {
        fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (fd < 0) continue;

        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0)
            break;

        if (errno == EINPROGRESS) {
            fd_set wfds;
            struct timeval tv = {5, 0};
            FD_ZERO(&wfds);
            FD_SET(fd, &wfds);
            if (select(fd + 1, NULL, &wfds, NULL, &tv) > 0) {
                int32_t so_error;
                socklen_t len = sizeof(so_error);
                getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &len);
                if (so_error == 0)
                    break;
            }
        }
        close(fd);
        fd = -1;
    }

    freeaddrinfo(res);

    if (fd >= 0) {
        int32_t val = 1;
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &val, sizeof(val));
    }

    return fd;
}

// Write all bytes to fd (blocking-style with select)
static int32_t opensky_write_all(int32_t fd, const void *buf, size_t len) {
    const uint8_t *p = buf;
    size_t remaining = len;

    while (remaining > 0) {
        ssize_t w = write(fd, p, remaining);
        if (w > 0) {
            p += w;
            remaining -= (size_t)w;
        } else if (w < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                fd_set wfds;
                struct timeval tv = {5, 0};
                FD_ZERO(&wfds);
                FD_SET(fd, &wfds);
                if (select(fd + 1, NULL, &wfds, NULL, &tv) <= 0)
                    return -1;
            } else {
                return -1;
            }
        }
    }
    return 0;
}

// Read exactly n bytes from fd (with timeout)
static int32_t opensky_read_all(int32_t fd, void *buf, size_t len, int32_t timeout_sec) {
    uint8_t *p = buf;
    size_t remaining = len;

    while (remaining > 0) {
        fd_set rfds;
        struct timeval tv = {timeout_sec, 0};
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        int sel = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (sel <= 0) return -1;

        ssize_t r = read(fd, p, remaining);
        if (r <= 0) return -1;
        p += r;
        remaining -= (size_t)r;
    }
    return 0;
}

// Store uint32 big-endian
static void put_be32(uint8_t *p, uint32_t v) {
    p[0] = (v >> 24) & 0xFF;
    p[1] = (v >> 16) & 0xFF;
    p[2] = (v >>  8) & 0xFF;
    p[3] =  v        & 0xFF;
}

// Read uint32 big-endian
static uint32_t get_be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}

// Store IEEE 754 double big-endian
static void put_be_double(uint8_t *p, double v) {
    union { double d; uint64_t u; } conv;
    conv.d = v;
    p[0] = (conv.u >> 56) & 0xFF;
    p[1] = (conv.u >> 48) & 0xFF;
    p[2] = (conv.u >> 40) & 0xFF;
    p[3] = (conv.u >> 32) & 0xFF;
    p[4] = (conv.u >> 24) & 0xFF;
    p[5] = (conv.u >> 16) & 0xFF;
    p[6] = (conv.u >>  8) & 0xFF;
    p[7] =  conv.u        & 0xFF;
}

// ===================== Serial persistence =====================

static int32_t opensky_load_serial(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return 0;

    char line[64];
    int32_t serial = 0;
    while (fgets(line, sizeof(line), f)) {
        // Format: "serial = <number>"
        if (strncmp(line, "serial", 6) == 0) {
            char *eq = strchr(line, '=');
            if (eq) {
                serial = (int32_t)atol(eq + 1);
            }
        }
    }
    fclose(f);
    return serial;
}

static void opensky_save_serial(const char *path, int32_t serial) {
    FILE *f = fopen(path, "w");
    if (!f) {
        fprintf(stderr, "OpenSky: could not save serial to %s: %s\n", path, strerror(errno));
        return;
    }
    fprintf(f, "[Device]\nserial = %d\n", serial);
    fclose(f);
}

// ===================== Protocol frames =====================

// Send LOGIN frame: 1A 41 + device_type(4) + major(4) + minor(4) + patch(4) = 18 bytes
static int32_t opensky_send_login(int32_t fd) {
    uint8_t buf[18];
    buf[0] = OPENSKY_FRAME_SYNC;
    buf[1] = OPENSKY_TYPE_LOGIN;
    put_be32(buf + 2,  OPENSKY_DEVICE_TYPE);
    put_be32(buf + 6,  OPENSKY_VERSION_MAJOR);
    put_be32(buf + 10, OPENSKY_VERSION_MINOR);
    put_be32(buf + 14, OPENSKY_VERSION_PATCH);
    return opensky_write_all(fd, buf, 18);
}

// Send SERIAL frame: 1A 35 + serial(4) = 6 bytes
static int32_t opensky_send_serial(int32_t fd, int32_t serial) {
    uint8_t buf[6];
    buf[0] = OPENSKY_FRAME_SYNC;
    buf[1] = OPENSKY_TYPE_SERIAL;
    put_be32(buf + 2, (uint32_t)serial);
    return opensky_write_all(fd, buf, 6);
}

// Send SERIAL_REQ frame: 1A 42 = 2 bytes
static int32_t opensky_send_serial_req(int32_t fd) {
    uint8_t buf[2];
    buf[0] = OPENSKY_FRAME_SYNC;
    buf[1] = OPENSKY_TYPE_SERIAL_REQ;
    return opensky_write_all(fd, buf, 2);
}

// Read server serial response: TLV header (type=2, len=2) then payload
static int32_t opensky_recv_serial(int32_t fd) {
    uint8_t hdr[4];
    if (opensky_read_all(fd, hdr, 4, 60) != 0) {
        fprintf(stderr, "OpenSky: timeout waiting for serial number\n");
        return 0;
    }

    uint16_t type = ((uint16_t)hdr[0] << 8) | hdr[1];
    uint16_t len  = ((uint16_t)hdr[2] << 8) | hdr[3];

    fprintf(stderr, "OpenSky: server TLV type=%u len=%u\n", type, len);

    // Read the payload (len includes the 4-byte header)
    uint8_t payload[256];
    uint16_t payload_len = len - 4;
    if (payload_len == 0 || payload_len > sizeof(payload)) {
        fprintf(stderr, "OpenSky: bad TLV payload length %u\n", payload_len);
        return 0;
    }
    if (opensky_read_all(fd, payload, payload_len, 10) != 0) {
        fprintf(stderr, "OpenSky: timeout reading TLV payload\n");
        return 0;
    }

    if (type != OPENSKY_SRV_SERIAL) {
        fprintf(stderr, "OpenSky: unexpected server response type=%u\n", type);
        return 0;
    }

    return (int32_t)get_be32(payload);
}

// Send GPS frame: 1A 37 + lat(8) + lon(8) + alt(8) = 26 bytes
static int32_t opensky_send_gps(int32_t fd, double lat, double lon, double alt) {
    uint8_t buf[26];
    buf[0] = OPENSKY_FRAME_SYNC;
    buf[1] = OPENSKY_TYPE_GPS;
    put_be_double(buf + 2,  lat);
    put_be_double(buf + 10, lon);
    put_be_double(buf + 18, alt);
    return opensky_write_all(fd, buf, 26);
}

// Send USERNAME frame: 1A 43 + username (40 bytes, null-padded)
static int32_t opensky_send_username(int32_t fd, const char *username) {
    uint8_t buf[42];
    buf[0] = OPENSKY_FRAME_SYNC;
    buf[1] = OPENSKY_TYPE_USERNAME;
    memset(buf + 2, 0, 40);
    size_t ulen = strlen(username);
    if (ulen > 39) ulen = 39;
    memcpy(buf + 2, username, ulen);
    return opensky_write_all(fd, buf, 2 + 40);
}

// ===================== Beast binary encoding (same as feeder_thread.c) =====================

static int32_t opensky_encode_beast(const struct modesMessage *mm, uint8_t *buf, int32_t bufsize) {
    uint8_t *p = buf;
    uint8_t *end = buf + bufsize;
    int32_t msgLen = mm->msgbits / 8;

    if (msgLen != MODES_SHORT_MSG_BYTES && msgLen != MODES_LONG_MSG_BYTES && msgLen != MODEAC_MSG_BYTES)
        return 0;

    if (mm->source == SOURCE_MLAT && !Modes.forward_mlat)
        return 0;

#define BEAST_PUSH(b) do { \
    uint8_t _b = (uint8_t)(b); \
    if (p >= end) return 0; \
    *p++ = _b; \
    if (_b == 0x1a) { if (p >= end) return 0; *p++ = 0x1a; } \
} while(0)

    if (p >= end) return 0;
    *p++ = 0x1a;
    if (p >= end) return 0;
    if (msgLen == MODEAC_MSG_BYTES) *p++ = '1';
    else if (msgLen == MODES_SHORT_MSG_BYTES) *p++ = '2';
    else *p++ = '3';

    BEAST_PUSH((mm->timestampMsg >> 40) & 0xFF);
    BEAST_PUSH((mm->timestampMsg >> 32) & 0xFF);
    BEAST_PUSH((mm->timestampMsg >> 24) & 0xFF);
    BEAST_PUSH((mm->timestampMsg >> 16) & 0xFF);
    BEAST_PUSH((mm->timestampMsg >> 8) & 0xFF);
    BEAST_PUSH(mm->timestampMsg & 0xFF);

    int32_t sig = (int32_t)(sqrt(mm->signalLevel) * 255 + 0.5);
    if (mm->signalLevel > 0 && sig < 1) sig = 1;
    if (sig > 255) sig = 255;
    BEAST_PUSH(sig);

    for (int32_t i = 0; i < msgLen; i++) {
        BEAST_PUSH(mm->verbatim[i]);
    }

#undef BEAST_PUSH

    return (int32_t)(p - buf);
}

// ===================== OpenSky feeder thread =====================

void openskyClientInit(void) {
    memset(&OpenSkyConfig, 0, sizeof(OpenSkyConfig));
    strncpy(OpenSkyConfig.host, "collector.opensky-network.org", sizeof(OpenSkyConfig.host) - 1);
    OpenSkyConfig.port = 10004;
    strncpy(OpenSkyConfig.serial_file, "/var/lib/dump1090-gg/opensky-serial.conf",
            sizeof(OpenSkyConfig.serial_file) - 1);
    if (!opensky_queue)
        opensky_queue = msg_queue_create(sizeof(struct modesMessage), 4096);
}

void *opensky_thread_entry(void *arg) {
    MODES_NOTUSED(arg);

    struct modesMessage mm;
    uint8_t beast_buf[256];
    static const uint8_t heartbeat[] = { 0x1a, '1', 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    int32_t fd = -1;
    uint64_t next_reconnect = 0;
    uint64_t last_heartbeat = 0;
    int32_t logged_in = 0;

    while (atomic_load(&feeders_running)) {
        uint64_t now = mstime();

        // When internet is offline, drain queue and close connection
        if (!atomic_load(&net_available)) {
            while (msg_queue_pop(opensky_queue, &mm)) { /* discard */ }
            if (fd >= 0) {
                close(fd);
                fd = -1;
                logged_in = 0;
                fprintf(stderr, "OpenSky: disconnected (internet offline)\n");
            }
            usleep(500000);
            continue;
        }

        // ---- Connect and login ----
        if (fd < 0 && now >= next_reconnect) {
            fd = opensky_tcp_connect(OpenSkyConfig.host, OpenSkyConfig.port);
            if (fd < 0) {
                fprintf(stderr, "OpenSky: failed to connect to %s:%d, retry in 30s\n",
                        OpenSkyConfig.host, OpenSkyConfig.port);
                next_reconnect = now + 30000;
                continue;
            }

            fprintf(stderr, "OpenSky: connected to %s:%d\n",
                    OpenSkyConfig.host, OpenSkyConfig.port);
            logged_in = 0;

            // Step 1: Send LOGIN
            if (opensky_send_login(fd) != 0) {
                fprintf(stderr, "OpenSky: login send failed\n");
                goto reconnect;
            }
            fprintf(stderr, "OpenSky: sent login (Device=%d, Version=%d.%d.%d)\n",
                    OPENSKY_DEVICE_TYPE, OPENSKY_VERSION_MAJOR,
                    OPENSKY_VERSION_MINOR, OPENSKY_VERSION_PATCH);

            // Step 2: Serial number
            if (OpenSkyConfig.serial == 0) {
                // Try loading from file
                OpenSkyConfig.serial = opensky_load_serial(OpenSkyConfig.serial_file);
            }

            if (OpenSkyConfig.serial != 0) {
                // Send known serial
                if (opensky_send_serial(fd, OpenSkyConfig.serial) != 0) {
                    fprintf(stderr, "OpenSky: serial send failed\n");
                    goto reconnect;
                }
                fprintf(stderr, "OpenSky: sent serial %d\n", OpenSkyConfig.serial);
            } else {
                // Request new serial
                if (opensky_send_serial_req(fd) != 0) {
                    fprintf(stderr, "OpenSky: serial request failed\n");
                    goto reconnect;
                }
                fprintf(stderr, "OpenSky: requesting new serial number\n");

                int32_t new_serial = opensky_recv_serial(fd);
                if (new_serial == 0) {
                    fprintf(stderr, "OpenSky: failed to get serial number\n");
                    goto reconnect;
                }

                OpenSkyConfig.serial = new_serial;
                fprintf(stderr, "OpenSky: got serial %d\n", new_serial);
                opensky_save_serial(OpenSkyConfig.serial_file, new_serial);

                // Echo serial back to server
                if (opensky_send_serial(fd, OpenSkyConfig.serial) != 0) {
                    fprintf(stderr, "OpenSky: serial echo failed\n");
                    goto reconnect;
                }
            }

            // Step 3: GPS position
            if (opensky_send_gps(fd, OpenSkyConfig.lat, OpenSkyConfig.lon, OpenSkyConfig.alt) != 0) {
                fprintf(stderr, "OpenSky: GPS send failed\n");
                goto reconnect;
            }
            fprintf(stderr, "OpenSky: sent position %+.4f, %+.4f, %.0fm\n",
                    OpenSkyConfig.lat, OpenSkyConfig.lon, OpenSkyConfig.alt);

            // Step 4: Username
            if (OpenSkyConfig.username[0]) {
                if (opensky_send_username(fd, OpenSkyConfig.username) != 0) {
                    fprintf(stderr, "OpenSky: username send failed\n");
                    goto reconnect;
                }
                fprintf(stderr, "OpenSky: sent username '%s'\n", OpenSkyConfig.username);
            }

            logged_in = 1;
            last_heartbeat = mstime();

            fprintf(stderr, "OpenSky: login complete (serial=%d, user=%s)\n",
                    OpenSkyConfig.serial, OpenSkyConfig.username);
        }

        // ---- Stream data ----
        if (fd >= 0 && logged_in) {
            int32_t sent = 0;

            while (msg_queue_pop(opensky_queue, &mm)) {
                int32_t len = opensky_encode_beast(&mm, beast_buf, sizeof(beast_buf));
                if (len <= 0) continue;

                int w = write(fd, beast_buf, len);
                if (w < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    fprintf(stderr, "OpenSky: write error: %s\n", strerror(errno));
                    goto reconnect;
                }
                sent++;
            }

            // Heartbeat every 30s
            now = mstime();
            if (now - last_heartbeat >= 30000) {
                if (write(fd, heartbeat, sizeof(heartbeat)) < 0) {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        fprintf(stderr, "OpenSky: heartbeat error: %s\n", strerror(errno));
                        goto reconnect;
                    }
                }
                last_heartbeat = now;
            }

            // Drain any server responses
            char discard[256];
            int r = read(fd, discard, sizeof(discard));
            if (r == 0) {
                fprintf(stderr, "OpenSky: connection closed by server\n");
                goto reconnect;
            } else if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                fprintf(stderr, "OpenSky: read error: %s\n", strerror(errno));
                goto reconnect;
            }

            {
                struct timespec ts = {0, 5 * 1000 * 1000};
                nanosleep(&ts, NULL);
            }
        } else if (fd < 0) {
            struct timespec ts = {0, 100 * 1000 * 1000};
            nanosleep(&ts, NULL);
        }
        continue;

reconnect:
        if (fd >= 0) {
            close(fd);
            fd = -1;
        }
        logged_in = 0;
        next_reconnect = mstime() + 30000;
    }

    if (fd >= 0)
        close(fd);

    return NULL;
}
