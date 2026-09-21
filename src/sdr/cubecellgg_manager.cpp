// cubecellgg_manager.cpp — CubeCell IoT receiver manager for dump1090-gg
//
// All-in-one: USB detection, firmware flash, serial communication.
// No external scripts — everything is embedded C++.

#include "cubecellgg_manager.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>
#include <limits.h>

// ====================== Constants ======================

#define CP2102_VID       "10c4"
#define CP2102_PID       "ea60"
#define FLASH_BAUD       B921600
#define COMM_BAUD        B115200
#define FLASH_KEY        "\xD9\x1F\x5D\xC3\x8D\x2A"
#define FLASH_KEY_LEN    6
#define FLASH_CHUNK      25
#define MAX_CYACD_ROWS   600
#define MAX_ROW_DATA     256
#define SERIAL_TIMEOUT_S 5

// ====================== Bootloader protocol ======================

static uint16_t bl_checksum(const uint8_t *d, int n) {
    uint32_t s = 0;
    for (int i = 0; i < n; i++) s += d[i];
    return (~s + 1) & 0xFFFF;
}

static int bl_make_packet(uint8_t *buf, uint8_t cmd, const uint8_t *data, int dlen) {
    buf[0] = 0x01;
    buf[1] = cmd;
    buf[2] = dlen & 0xFF;
    buf[3] = (dlen >> 8) & 0xFF;
    if (dlen > 0 && data) memcpy(buf + 4, data, dlen);
    uint16_t cs = bl_checksum(buf, 4 + dlen);
    buf[4 + dlen] = cs & 0xFF;
    buf[5 + dlen] = (cs >> 8) & 0xFF;
    buf[6 + dlen] = 0x17;
    return 7 + dlen;
}

// ====================== .cyacd parser ======================

struct cyacd_row {
    uint8_t  array_id;
    uint16_t row_num;
    uint16_t data_len;
    uint8_t  data[MAX_ROW_DATA];
};

static int hex2byte(const char *h) {
    auto nib = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    int hi = nib(h[0]), lo = nib(h[1]);
    return (hi < 0 || lo < 0) ? -1 : (hi << 4) | lo;
}

static int cyacd_parse(const char *path, cyacd_row *rows, int max_rows) {
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char line[1024];
    int count = 0;
    if (!fgets(line, sizeof(line), f)) { fclose(f); return -1; }  // skip header
    while (fgets(line, sizeof(line), f) && count < max_rows) {
        char *h = line;
        while (*h == ':' || *h == ' ') h++;
        int hlen = strlen(h);
        while (hlen > 0 && (h[hlen-1] == '\n' || h[hlen-1] == '\r')) h[--hlen] = 0;
        if (hlen < 12) continue;
        uint8_t raw[280];
        int rlen = 0;
        for (int i = 0; i + 1 < hlen && rlen < (int)sizeof(raw); i += 2) {
            int b = hex2byte(h + i);
            if (b < 0) break;
            raw[rlen++] = (uint8_t)b;
        }
        if (rlen < 6) continue;
        rows[count].array_id = raw[0];
        rows[count].row_num = (raw[1] << 8) | raw[2];
        rows[count].data_len = (raw[3] << 8) | raw[4];
        if (rows[count].data_len > MAX_ROW_DATA) rows[count].data_len = MAX_ROW_DATA;
        memcpy(rows[count].data, raw + 5, rows[count].data_len);
        count++;
    }
    fclose(f);
    return count;
}

// ====================== Serial helpers ======================

static int serial_open(const char *path, speed_t baud) {
    int fd = open(path, O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    cfsetispeed(&tio, baud);
    cfsetospeed(&tio, baud);
    tio.c_cflag = CS8 | CLOCAL | CREAD | baud;
    tio.c_iflag = IGNPAR;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 10;  // 1s timeout
    tcsetattr(fd, TCSANOW, &tio);
    tcflush(fd, TCIOFLUSH);
    return fd;
}

static void serial_set_dtr(int fd, int state) {
    int bits = TIOCM_DTR;
    ioctl(fd, state ? TIOCMBIS : TIOCMBIC, &bits);
}

static void serial_set_rts(int fd, int state) {
    int bits = TIOCM_RTS;
    ioctl(fd, state ? TIOCMBIS : TIOCMBIC, &bits);
}

static int serial_read_packet(int fd, uint8_t *buf, int max_len, int timeout_ms) {
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    int total = 0;
    bool found_sop = false;

    while (total < max_len) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        int elapsed = (now.tv_sec - start.tv_sec) * 1000 + (now.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsed > timeout_ms) break;

        uint8_t byte;
        int n = read(fd, &byte, 1);
        if (n <= 0) { usleep(1000); continue; }

        if (!found_sop) {
            if (byte == 0x01) {
                buf[0] = byte;
                total = 1;
                found_sop = true;
            }
            continue;
        }

        buf[total++] = byte;
        if (byte == 0x17 && total >= 7) break;  // EOP
    }
    return found_sop ? total : 0;
}

// ====================== USB detection ======================

static bool find_cp2102_tty(char *tty_path, int max_len) {
    // Scan /sys/class/tty/ttyUSB* and check if the parent USB device is CP2102
    DIR *d = opendir("/sys/class/tty");
    if (!d) return false;
    struct dirent *e;
    while ((e = readdir(d))) {
        if (strncmp(e->d_name, "ttyUSB", 6) != 0) continue;
        // Read the device symlink to find the USB device path
        char syslink[300], resolved[PATH_MAX];
        snprintf(syslink, sizeof(syslink), "/sys/class/tty/%.20s/device", e->d_name);
        if (!realpath(syslink, resolved)) continue;
        // Go up TWO levels: ttyUSB0 -> interface:1.0 -> USB device
        char *slash = strrchr(resolved, '/');
        if (slash) *slash = 0;
        slash = strrchr(resolved, '/');
        if (slash) *slash = 0;
        // Read idVendor + idProduct
        char vpath[PATH_MAX + 32], vid[8] = {}, pid[8] = {};
        snprintf(vpath, sizeof(vpath), "%s/idVendor", resolved);
        FILE *vf = fopen(vpath, "r");
        if (!vf) continue;
        if (fgets(vid, sizeof(vid), vf)) vid[strcspn(vid, "\n")] = 0;
        fclose(vf);
        snprintf(vpath, sizeof(vpath), "%s/idProduct", resolved);
        FILE *pf = fopen(vpath, "r");
        if (!pf) continue;
        if (fgets(pid, sizeof(pid), pf)) pid[strcspn(pid, "\n")] = 0;
        fclose(pf);

        if (strcasecmp(vid, CP2102_VID) == 0 && strcasecmp(pid, CP2102_PID) == 0) {
            snprintf(tty_path, max_len, "/dev/%.20s", e->d_name);
            closedir(d);
            return true;
        }
    }
    closedir(d);
    return false;
}

// ====================== Firmware flash ======================

static bool flash_firmware(const char *tty_path, const char *cyacd_path) {
    static cyacd_row rows[MAX_CYACD_ROWS];
    int row_count = cyacd_parse(cyacd_path, rows, MAX_CYACD_ROWS);
    if (row_count <= 0) {
        fprintf(stderr, "cubecellgg: failed to parse %s\n", cyacd_path);
        return false;
    }
    fprintf(stderr, "cubecellgg: flashing %d rows from %s\n", row_count, cyacd_path);

    // Open at 921600 for bootloader
    int fd = serial_open(tty_path, FLASH_BAUD);
    if (fd < 0) {
        fprintf(stderr, "cubecellgg: cannot open %s for flashing\n", tty_path);
        return false;
    }

    // DTR/RTS bootloader entry sequence
    serial_set_dtr(fd, 0);
    serial_set_rts(fd, 1);
    usleep(100000);
    serial_set_dtr(fd, 1);
    serial_set_rts(fd, 0);
    usleep(100000);
    serial_set_dtr(fd, 0);
    tcflush(fd, TCIOFLUSH);

    // EnterBootloader with security key
    uint8_t pkt[300], resp[64];
    int plen = bl_make_packet(pkt, 0x38, (const uint8_t *)FLASH_KEY, FLASH_KEY_LEN);
    if (write(fd, pkt, plen) < 0) { /* ignore */ }

    int rlen = serial_read_packet(fd, resp, sizeof(resp), SERIAL_TIMEOUT_S * 1000);
    if (rlen < 7 || resp[0] != 0x01 || resp[1] != 0x00) {
        fprintf(stderr, "cubecellgg: bootloader not responding (rlen=%d first=0x%02x status=0x%02x)\n",
                rlen, rlen > 0 ? resp[0] : 0, rlen >= 2 ? resp[1] : 0xFF);
        close(fd);
        return false;
    }
    fprintf(stderr, "cubecellgg: bootloader active\n");

    // Flash all rows
    int errors = 0;
    for (int i = 0; i < row_count; i++) {
        const cyacd_row *r = &rows[i];
        int off = 0;
        bool row_ok = true;

        // SendData chunks
        while (off < r->data_len - FLASH_CHUNK) {
            plen = bl_make_packet(pkt, 0x37, r->data + off, FLASH_CHUNK);
            if (write(fd, pkt, plen) < 0) { /* ignore */ }
            rlen = serial_read_packet(fd, resp, sizeof(resp), SERIAL_TIMEOUT_S * 1000);
            if (rlen < 7 || resp[0] != 0x01 || resp[1] != 0x00) {
                row_ok = false;
                break;
            }
            off += FLASH_CHUNK;
        }

        if (row_ok) {
            // ProgramRow with last chunk
            int last = r->data_len - off;
            uint8_t prog[3 + MAX_ROW_DATA];
            prog[0] = r->array_id;
            prog[1] = r->row_num & 0xFF;
            prog[2] = (r->row_num >> 8) & 0xFF;
            memcpy(prog + 3, r->data + off, last);
            plen = bl_make_packet(pkt, 0x39, prog, 3 + last);
            if (write(fd, pkt, plen) < 0) { /* ignore */ }
            rlen = serial_read_packet(fd, resp, sizeof(resp), SERIAL_TIMEOUT_S * 1000);
            if (rlen < 7 || resp[0] != 0x01 || resp[1] != 0x00) row_ok = false;
        }

        if (!row_ok) {
            errors++;
            fprintf(stderr, "cubecellgg: row %d (rn=%d) failed\n", i + 1, r->row_num);
            if (errors > 3) {
                fprintf(stderr, "cubecellgg: too many errors, aborting flash\n");
                close(fd);
                return false;
            }
        }

        int pct = (i + 1) * 100 / row_count;
        int prev = i * 100 / row_count;
        if (pct / 10 != prev / 10)
            fprintf(stderr, "cubecellgg: flash %d%%\n", pct);
    }

    // VerifyChecksum
    plen = bl_make_packet(pkt, 0x31, NULL, 0);
    if (write(fd, pkt, plen) < 0) { /* ignore */ }
    rlen = serial_read_packet(fd, resp, sizeof(resp), SERIAL_TIMEOUT_S * 1000);
    bool verify_ok = (rlen >= 7 && resp[0] == 0x01 && resp[1] == 0x00);
    fprintf(stderr, "cubecellgg: verify %s\n", verify_ok ? "OK" : "FAIL");

    // ExitBootloader
    plen = bl_make_packet(pkt, 0x3B, NULL, 0);
    if (write(fd, pkt, plen) < 0) { /* ignore */ }

    close(fd);
    fprintf(stderr, "cubecellgg: flash complete, errors=%d\n", errors);
    return errors == 0 && verify_ok;
}

// ====================== Manager state ======================

static struct {
    pthread_t        thread;
    bool             thread_running;
    volatile bool    shutdown;

    char             firmware_dir[256];
    char             tty_path[64];
    int              serial_fd;
    ccgg_state_t     state;
    char             fw_version[16];
    char             local_version[16];
    char             profile[16];
    uint32_t         frequency;
    uint32_t         iot_packets;
    uint32_t         crc_failures;
    ccgg_cap_t       caps[CCGG_MAX_CAPS];
    int32_t          num_caps;
    bool             paused;  // true when user clicked Stop

    // Waterfall line buffer (latest line only)
    char             wf_line[2048];
    bool             wf_new;

    ccgg_iot_callback_t  iot_cb;
    void                *iot_ctx;
    ccgg_scan_callback_t scan_cb;
    void                *scan_ctx;

    pthread_mutex_t  lock;
} g_ccgg;

// ====================== Version comparison ======================

static bool read_file_line(const char *path, char *buf, int max) {
    FILE *f = fopen(path, "r");
    if (!f) return false;
    if (!fgets(buf, max, f)) { fclose(f); return false; }
    buf[strcspn(buf, "\r\n")] = 0;
    fclose(f);
    return true;
}

static int version_compare(const char *a, const char *b) {
    int a1=0, a2=0, a3=0, b1=0, b2=0, b3=0;
    sscanf(a, "%d.%d.%d", &a1, &a2, &a3);
    sscanf(b, "%d.%d.%d", &b1, &b2, &b3);
    if (a1 != b1) return a1 - b1;
    if (a2 != b2) return a2 - b2;
    return a3 - b3;
}

// ====================== Communication ======================

static bool query_device_version(int fd, char *version, int max_len) {
    tcflush(fd, TCIOFLUSH);
    const char *cmd = "CMD:STATUS\r\n";
    if (write(fd, cmd, strlen(cmd)) < 0) { /* ignore */ }
    usleep(500000);

    char buf[512];
    int total = 0;
    for (int attempt = 0; attempt < 10 && total < (int)sizeof(buf) - 1; attempt++) {
        int n = read(fd, buf + total, sizeof(buf) - 1 - total);
        if (n > 0) total += n;
        else usleep(100000);
    }
    buf[total] = 0;

    // Find "version":"X.Y.Z" in JSON
    const char *vp = strstr(buf, "\"version\":\"");
    if (!vp) return false;
    vp += 11;
    const char *end = strchr(vp, '"');
    if (!end || end - vp >= max_len) return false;
    memcpy(version, vp, end - vp);
    version[end - vp] = 0;
    return true;
}

static void query_device_caps(int fd) {
    tcflush(fd, TCIOFLUSH);
    const char *cmd = "CMD:CAPS\r\n";
    if (write(fd, cmd, strlen(cmd)) < 0) { return; }
    usleep(500000);
    char buf[1024];
    int total = 0;
    for (int a = 0; a < 10 && total < (int)sizeof(buf) - 1; a++) {
        int n = read(fd, buf + total, sizeof(buf) - 1 - total);
        if (n > 0) total += n; else usleep(100000);
    }
    buf[total] = 0;

    pthread_mutex_lock(&g_ccgg.lock);
    g_ccgg.num_caps = 0;
    // Simple JSON parse: find each {"id":N,"name":"...","desc":"...","freq":N}
    const char *p = buf;
    while ((p = strstr(p, "\"id\":")) && g_ccgg.num_caps < CCGG_MAX_CAPS) {
        ccgg_cap_t *c = &g_ccgg.caps[g_ccgg.num_caps];
        memset(c, 0, sizeof(*c));
        c->id = atoi(p + 5);
        const char *np = strstr(p, "\"name\":\"");
        if (np) { np += 8; const char *e = strchr(np, '"'); if (e) { int l = e-np; if(l>15)l=15; memcpy(c->name, np, l); } }
        const char *dp = strstr(p, "\"desc\":\"");
        if (dp) { dp += 8; const char *e = strchr(dp, '"'); if (e) { int l = e-dp; if(l>63)l=63; memcpy(c->desc, dp, l); } }
        const char *fp = strstr(p, "\"freq\":");
        if (fp) c->freq = strtoul(fp + 7, NULL, 10);
        g_ccgg.num_caps++;
        p++;
    }
    pthread_mutex_unlock(&g_ccgg.lock);
    fprintf(stderr, "cubecellgg: %d capabilities reported\n", g_ccgg.num_caps);
}

static void parse_serial_line(const char *line) {
    if (g_ccgg.paused) return;  // discard all data when paused
    // Parse JSON from cubecellgg firmware
    if (strstr(line, "\"proto\":\"") && g_ccgg.iot_cb) {
        ccgg_iot_msg_t msg = {};
        // Extract fields from JSON
        const char *p;
        if ((p = strstr(line, "\"proto\":\""))) {
            p += 9;
            const char *e = strchr(p, '"');
            if (e) { int len = e - p; if (len > 31) len = 31; memcpy(msg.proto, p, len); }
        }
        if ((p = strstr(line, "\"id\":"))) msg.sensor_id = atoi(p + 5);
        if ((p = strstr(line, "\"temp\":"))) msg.temperature = atof(p + 7);
        if ((p = strstr(line, "\"hum\":"))) msg.humidity = atoi(p + 6); else msg.humidity = -1;
        if ((p = strstr(line, "\"rssi\":"))) msg.rssi = atoi(p + 7);
        if ((p = strstr(line, "\"batt\":"))) msg.battery_low = atoi(p + 7);
        if ((p = strstr(line, "\"new\":"))) msg.new_battery = atoi(p + 6);
        if ((p = strstr(line, "\"pkts\":"))) msg.pkt_count = atoi(p + 7);

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        msg.timestamp_ms = (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;

        pthread_mutex_lock(&g_ccgg.lock);
        g_ccgg.iot_packets++;
        pthread_mutex_unlock(&g_ccgg.lock);

        g_ccgg.iot_cb(&msg, g_ccgg.iot_ctx);
    }
    else if (strstr(line, "\"event\":\"wf\"")) {
        // Store latest waterfall line for API polling
        pthread_mutex_lock(&g_ccgg.lock);
        snprintf(g_ccgg.wf_line, sizeof(g_ccgg.wf_line), "%s", line);
        g_ccgg.wf_new = true;
        pthread_mutex_unlock(&g_ccgg.lock);
    }
    else if (strstr(line, "\"event\":\"scan\"") && g_ccgg.scan_cb) {
        ccgg_scan_result_t scan = {};
        // Parse scan data array
        const char *arr = strstr(line, "\"data\":[");
        if (arr) {
            arr += 8;
            int bin = 0;
            while (*arr && bin < 256) {
                if (*arr == '[') {
                    arr++;
                    uint32_t freq = strtoul(arr, (char**)&arr, 10);
                    if (*arr == ',') arr++;
                    int rssi = strtol(arr, (char**)&arr, 10);
                    if (bin == 0) scan.freq_start = freq;
                    else if (bin == 1) scan.freq_step = freq - scan.freq_start;
                    scan.rssi[bin++] = (int8_t)rssi;
                    while (*arr && *arr != ']') arr++;
                    if (*arr == ']') arr++;
                    if (*arr == ',') arr++;
                }
                else arr++;
            }
            scan.num_bins = bin;
            g_ccgg.scan_cb(&scan, g_ccgg.scan_ctx);
        }
    }
}

// ====================== Manager thread ======================

static void *ccgg_thread(void *arg) {
    (void)arg;
    int line_pos = 0; (void)line_pos;

    while (!g_ccgg.shutdown) {
        // Phase 1: Detect device
        pthread_mutex_lock(&g_ccgg.lock);
        g_ccgg.state = CCGG_STATE_DETECTING;
        pthread_mutex_unlock(&g_ccgg.lock);

        char tty[64] = {};
        while (!g_ccgg.shutdown && !find_cp2102_tty(tty, sizeof(tty))) {
            sleep(5);
        }
        if (g_ccgg.shutdown) break;

        pthread_mutex_lock(&g_ccgg.lock);
        snprintf(g_ccgg.tty_path, sizeof(g_ccgg.tty_path), "%s", tty);
        pthread_mutex_unlock(&g_ccgg.lock);

        fprintf(stderr, "cubecellgg: detected on %s\n", tty);

        // Phase 2: Check version and flash if needed
        char cyacd_path[300], ver_path[300];
        snprintf(cyacd_path, sizeof(cyacd_path), "%s/cubecellgg.cyacd", g_ccgg.firmware_dir);
        snprintf(ver_path, sizeof(ver_path), "%s/version.txt", g_ccgg.firmware_dir);

        char local_ver[16] = "0.0.0";
        bool has_firmware = false;
        struct stat st;
        if (stat(cyacd_path, &st) == 0 && read_file_line(ver_path, local_ver, sizeof(local_ver))) {
            has_firmware = true;
        }

        pthread_mutex_lock(&g_ccgg.lock);
        snprintf(g_ccgg.local_version, sizeof(g_ccgg.local_version), "%s", local_ver);
        pthread_mutex_unlock(&g_ccgg.lock);

        // Open at 115200 to query version
        int fd = serial_open(tty, COMM_BAUD);
        if (fd < 0) {
            fprintf(stderr, "cubecellgg: cannot open %s\n", tty);
            sleep(5);
            continue;
        }

        // Wait for boot + query version
        sleep(2);
        char dev_ver[16] = "0.0.0";
        query_device_version(fd, dev_ver, sizeof(dev_ver));
        close(fd);

        fprintf(stderr, "cubecellgg: device=%s local=%s\n", dev_ver, local_ver);

        pthread_mutex_lock(&g_ccgg.lock);
        snprintf(g_ccgg.fw_version, sizeof(g_ccgg.fw_version), "%s", dev_ver);
        pthread_mutex_unlock(&g_ccgg.lock);

        // Flash if local is newer
        if (has_firmware && version_compare(local_ver, dev_ver) > 0) {
            fprintf(stderr, "cubecellgg: firmware update %s -> %s\n", dev_ver, local_ver);
            pthread_mutex_lock(&g_ccgg.lock);
            g_ccgg.state = CCGG_STATE_FLASHING;
            pthread_mutex_unlock(&g_ccgg.lock);

            if (flash_firmware(tty, cyacd_path)) {
                fprintf(stderr, "cubecellgg: flash successful\n");
                snprintf(g_ccgg.fw_version, sizeof(g_ccgg.fw_version), "%s", local_ver);
                sleep(3);  // wait for reboot
            } else {
                fprintf(stderr, "cubecellgg: flash FAILED\n");
                pthread_mutex_lock(&g_ccgg.lock);
                g_ccgg.state = CCGG_STATE_ERROR;
                pthread_mutex_unlock(&g_ccgg.lock);
                sleep(10);
                continue;
            }
        }

        // Phase 3: Normal operation — read decoded data
        fd = serial_open(tty, COMM_BAUD);
        if (fd < 0) {
            fprintf(stderr, "cubecellgg: cannot open %s for communication\n", tty);
            sleep(5);
            continue;
        }

        pthread_mutex_lock(&g_ccgg.lock);
        g_ccgg.serial_fd = fd;
        g_ccgg.state = CCGG_STATE_RUNNING;
        snprintf(g_ccgg.profile, sizeof(g_ccgg.profile), "%s", "lacrosse");
        g_ccgg.frequency = 868300000;
        pthread_mutex_unlock(&g_ccgg.lock);

        fprintf(stderr, "cubecellgg: running on %s (fw %s)\n", tty, g_ccgg.fw_version);

        // Query capabilities
        query_device_caps(fd);

        // Read serial data line by line
        char bigline[2048];
        int bigline_pos = 0;
        (void)line_pos;
        while (!g_ccgg.shutdown) {
            uint8_t byte;
            int n = read(fd, &byte, 1);
            if (n <= 0) {
                // Check if device still exists
                if (access(tty, F_OK) != 0) {
                    fprintf(stderr, "cubecellgg: device disconnected\n");
                    break;
                }
                usleep(10000);
                continue;
            }

            if (byte == '\n' || byte == '\r') {
                if (bigline_pos > 0) {
                    bigline[bigline_pos] = 0;
                    parse_serial_line(bigline);
                    bigline_pos = 0;
                }
            } else if (bigline_pos < (int)sizeof(bigline) - 1) {
                bigline[bigline_pos++] = byte;
            }
        }

        close(fd);
        pthread_mutex_lock(&g_ccgg.lock);
        g_ccgg.serial_fd = -1;
        g_ccgg.state = CCGG_STATE_DISCONNECTED;
        pthread_mutex_unlock(&g_ccgg.lock);
    }

    return NULL;
}

// ====================== Public API ======================

void ccggInit(const char *firmware_dir) {
    memset(&g_ccgg, 0, sizeof(g_ccgg));
    g_ccgg.serial_fd = -1;
    g_ccgg.state = CCGG_STATE_DISCONNECTED;
    pthread_mutex_init(&g_ccgg.lock, NULL);

    if (firmware_dir)
        snprintf(g_ccgg.firmware_dir, sizeof(g_ccgg.firmware_dir), "%s", firmware_dir);
    else
        snprintf(g_ccgg.firmware_dir, sizeof(g_ccgg.firmware_dir), "%s", "/usr/share/dump1090-gg/cubecellgg");

    g_ccgg.thread_running = true;
    pthread_create(&g_ccgg.thread, NULL, ccgg_thread, NULL);
    fprintf(stderr, "cubecellgg: manager initialized (firmware_dir=%s)\n", g_ccgg.firmware_dir);
}

void ccggShutdown(void) {
    g_ccgg.shutdown = true;
    if (g_ccgg.thread_running) {
        pthread_join(g_ccgg.thread, NULL);
        g_ccgg.thread_running = false;
    }
    if (g_ccgg.serial_fd >= 0) {
        close(g_ccgg.serial_fd);
        g_ccgg.serial_fd = -1;
    }
    pthread_mutex_destroy(&g_ccgg.lock);
}

void ccggSetIoTCallback(ccgg_iot_callback_t cb, void *ctx) {
    g_ccgg.iot_cb = cb;
    g_ccgg.iot_ctx = ctx;
}

void ccggSetScanCallback(ccgg_scan_callback_t cb, void *ctx) {
    g_ccgg.scan_cb = cb;
    g_ccgg.scan_ctx = ctx;
}

void ccggGetDeviceInfo(ccgg_device_info_t *info) {
    pthread_mutex_lock(&g_ccgg.lock);
    memset(info, 0, sizeof(*info));
    snprintf(info->tty_path, sizeof(info->tty_path), "%s", g_ccgg.tty_path);
    snprintf(info->fw_version, sizeof(info->fw_version), "%s", g_ccgg.fw_version);
    snprintf(info->local_version, sizeof(info->local_version), "%s", g_ccgg.local_version);
    snprintf(info->profile, sizeof(info->profile), "%s", g_ccgg.profile);
    info->frequency = g_ccgg.frequency;
    info->state = g_ccgg.paused ? CCGG_STATE_STOPPED : g_ccgg.state;
    info->iot_packets = g_ccgg.iot_packets;
    info->crc_failures = g_ccgg.crc_failures;
    info->num_caps = g_ccgg.num_caps;
    memcpy(info->caps, g_ccgg.caps, sizeof(info->caps));
    pthread_mutex_unlock(&g_ccgg.lock);
}

int ccggSendCommand(const char *cmd) {
    pthread_mutex_lock(&g_ccgg.lock);
    int fd = g_ccgg.serial_fd;
    pthread_mutex_unlock(&g_ccgg.lock);
    if (fd < 0) return -1;
    char buf[256];
    int len = snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    return write(fd, buf, len) > 0 ? 0 : -1;
}

int ccggRequestScan(void) {
    return ccggSendCommand("CMD:SCAN");
}

int ccggSetProfile(const char *profile) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "CMD:PROFILE:%s", profile);
    int rc = ccggSendCommand(cmd);
    if (rc == 0) {
        pthread_mutex_lock(&g_ccgg.lock);
        snprintf(g_ccgg.profile, sizeof(g_ccgg.profile), "%s", profile);
        pthread_mutex_unlock(&g_ccgg.lock);
    }
    return rc;
}

int ccggSetFrequency(uint32_t freq_hz) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "CMD:FREQ:%u", freq_hz);
    int rc = ccggSendCommand(cmd);
    if (rc == 0) {
        pthread_mutex_lock(&g_ccgg.lock);
        g_ccgg.frequency = freq_hz;
        pthread_mutex_unlock(&g_ccgg.lock);
    }
    return rc;
}

void ccggSetPaused(bool paused) {
    pthread_mutex_lock(&g_ccgg.lock);
    g_ccgg.paused = paused;
    pthread_mutex_unlock(&g_ccgg.lock);
    fprintf(stderr, "cubecellgg: %s\n", paused ? "stopped" : "resumed");
}

bool ccggIsRunning(void) {
    pthread_mutex_lock(&g_ccgg.lock);
    bool r = (g_ccgg.state == CCGG_STATE_RUNNING);
    pthread_mutex_unlock(&g_ccgg.lock);
    return r;
}

int ccggWaterfallStart(uint32_t start_hz, uint32_t end_hz, uint32_t step_hz) {
    if (start_hz == 0) start_hz = 863000000;
    if (end_hz == 0) end_hz = 870000000;
    if (step_hz == 0) step_hz = 50000;
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "CMD:WATERFALL:RANGE:%u:%u:%u", start_hz, end_hz, step_hz);
    ccggSendCommand(cmd);
    usleep(50000);
    return ccggSendCommand("CMD:WATERFALL:START");
}

int ccggWaterfallStop(void) {
    return ccggSendCommand("CMD:WATERFALL:STOP");
}

char *ccggWaterfallGetLine(void) {
    pthread_mutex_lock(&g_ccgg.lock);
    if (!g_ccgg.wf_new || g_ccgg.wf_line[0] == 0) {
        pthread_mutex_unlock(&g_ccgg.lock);
        return NULL;
    }
    char *copy = strdup(g_ccgg.wf_line);
    g_ccgg.wf_new = false;
    pthread_mutex_unlock(&g_ccgg.lock);
    return copy;
}
