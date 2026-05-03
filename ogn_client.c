// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// ogn_client.c: OGN (Open Glider Network) APRS-IS feed client
//
// Connects to aprs.glidernet.org:14580 and sends decoded FLARM positions
// in APRS format. The OGN protocol uses TCP with simple text lines.
//
// Login format: user STATION pass -1 vers dump1090-gg VERSION filter r/LAT/LON/RANGE
// Position format: FLARM_ID>APRS,qAS,STATION:/HHMMSS h DDMM.mmN/DDDMM.mmE'/A=AAAAAA ...
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <ctype.h>
#include <sys/socket.h>
#include <sys/sysinfo.h>
#include <netdb.h>
#include <fcntl.h>

#include "dump1090.h"

// ======================== State ========================

#define OGN_QUEUE_SIZE 128
#define OGN_RECONNECT_INTERVAL 30000  // ms
#define OGN_KEEPALIVE_INTERVAL 120000 // ms
#define OGN_BEACON_INTERVAL   300000 // ms (5 minutes)
#define OGN_ACFT_HASH_SIZE    256
#define OGN_ACFT_MAX_AGE      3600    // seconds

static struct {
    int           fd;
    int           connected;
    uint64_t      next_reconnect;
    uint64_t      last_keepalive;
    uint64_t      last_beacon;

    flarm_message_t queue[OGN_QUEUE_SIZE];
    unsigned      queue_head;
    unsigned      queue_tail;

    uint64_t      packets_sent;

    // Aircraft tracking for beacon stats
    struct {
        uint32_t addr;
        time_t   last_seen;
    } acft[OGN_ACFT_HASH_SIZE];
    unsigned      acft_total;      // total positions received since last beacon
    float         signal_max;      // max signal level since last beacon
    float         signal_sum;      // sum of signal levels
    unsigned      signal_count;    // number of signal samples
} OgnClient;

// ======================== Init ========================

void ognClientInit(void)
{
    memset(&OgnClient, 0, sizeof(OgnClient));
    OgnClient.fd = -1;
    OgnClient.connected = 0;
    OgnClient.next_reconnect = 0;
}

// ======================== Queue ========================

void ognClientSubmit(const flarm_message_t *msg)
{
    if (!FlarmConfig.enabled || FlarmConfig.ogn_station[0] == '\0') return;

    unsigned next = (OgnClient.queue_head + 1) % OGN_QUEUE_SIZE;
    if (next == OgnClient.queue_tail) return;  // full

    OgnClient.queue[OgnClient.queue_head] = *msg;
    OgnClient.queue_head = next;

    // Track unique aircraft (simple hash table)
    unsigned slot = msg->addr % OGN_ACFT_HASH_SIZE;
    OgnClient.acft[slot].addr = msg->addr;
    OgnClient.acft[slot].last_seen = time(NULL);
    OgnClient.acft_total++;

    // Track signal stats
    if (msg->signal_level > 0) {
        if (msg->signal_level > OgnClient.signal_max)
            OgnClient.signal_max = msg->signal_level;
        OgnClient.signal_sum += msg->signal_level;
        OgnClient.signal_count++;
    }
}

// ======================== TCP connect ========================

static int ogn_connect(void)
{
    struct addrinfo hints, *res, *rp;
    char port_str[16];

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    snprintf(port_str, sizeof(port_str), "%d", FlarmConfig.ogn_port);

    int err = getaddrinfo(FlarmConfig.ogn_server, port_str, &hints, &res);
    if (err != 0) {
        fprintf(stderr, "ogn: DNS resolution failed for %s: %s\n",
                FlarmConfig.ogn_server, gai_strerror(err));
        return -1;
    }

    int fd = -1;
    for (rp = res; rp; rp = rp->ai_next) {
        fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (fd < 0) continue;

        // Set non-blocking temporarily for connect timeout
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0 || errno == EINPROGRESS) {
            // Wait for connect with timeout
            fd_set wfds;
            struct timeval tv = {5, 0};  // 5 seconds
            FD_ZERO(&wfds);
            FD_SET(fd, &wfds);

            if (select(fd + 1, NULL, &wfds, NULL, &tv) > 0) {
                int so_error;
                socklen_t len = sizeof(so_error);
                getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &len);
                if (so_error == 0) {
                    // Connected
                    fcntl(fd, F_SETFL, flags);  // restore blocking
                    break;
                }
            }
        }

        close(fd);
        fd = -1;
    }

    freeaddrinfo(res);

    if (fd < 0) {
        fprintf(stderr, "ogn: failed to connect to %s:%d\n",
                FlarmConfig.ogn_server, FlarmConfig.ogn_port);
        return -1;
    }

    // Set non-blocking for normal operation
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    return fd;
}

// ======================== APRS Passcode ========================

static int aprs_passcode(const char *callsign)
{
    int hash = 0x73e2;
    int i = 0;
    int len = 0;
    char call[16];

    // Copy callsign without SSID (strip everything after '-'), uppercase
    for (len = 0; len < 15 && callsign[len] && callsign[len] != '-'; len++)
        call[len] = toupper((unsigned char)callsign[len]);
    call[len] = 0;

    for (i = 0; i < len; i += 2) {
        hash ^= ((unsigned char)call[i]) << 8;
        if (i + 1 < len)
            hash ^= (unsigned char)call[i + 1];
    }
    return hash & 0x7FFF;
}

// ======================== Login ========================

static bool ogn_login(int fd)
{
    int passcode = aprs_passcode(FlarmConfig.ogn_station);
    char login[256];
    int n = snprintf(login, sizeof(login),
                     "user %s pass %d vers dump1090-gg " MODES_DUMP1090_VERSION
                     " filter r/%.4f/%.4f/100\r\n",
                     FlarmConfig.ogn_station,
                     passcode,
                     Modes.fUserLat, Modes.fUserLon);

    if (n <= 0 || n >= (int)sizeof(login)) return false;

    // Blocking send for login line
    int sent = 0;
    while (sent < n) {
        int w = write(fd, login + sent, n - sent);
        if (w <= 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            return false;
        }
        sent += w;
    }

    fprintf(stderr, "ogn: logged in as %s to %s:%d\n",
            FlarmConfig.ogn_station, FlarmConfig.ogn_server, FlarmConfig.ogn_port);
    return true;
}

// ======================== Format APRS position ========================

static const char *flarm_addr_prefix(uint8_t addr_type)
{
    switch (addr_type) {
        case FLARM_ADDR_ICAO:  return "ICA";
        case FLARM_ADDR_FLARM: return "FLR";
        case FLARM_ADDR_RANDOM:
        case FLARM_ADDR_ANONYMOUS:
        default:               return "OGN";
    }
}

static const char *flarm_acft_symbol(uint8_t acft_type)
{
    // APRS symbols for aircraft types (primary table '/')
    switch (acft_type) {
        case FLARM_ACFT_GLIDER:     return "/'";  // glider
        case FLARM_ACFT_TOWPLANE:   return "/^";  // aircraft
        case FLARM_ACFT_HELICOPTER: return "/X";  // helicopter
        case FLARM_ACFT_PARACHUTE:  return "/O";  // balloon
        case FLARM_ACFT_HANGGLIDER: return "/'";  // glider
        case FLARM_ACFT_PARAGLIDER: return "/'";  // glider
        case FLARM_ACFT_POWERED:    return "/^";  // aircraft
        case FLARM_ACFT_JET:        return "/^";  // aircraft
        case FLARM_ACFT_BALLOON:    return "/O";  // balloon
        case FLARM_ACFT_UAV:        return "/^";  // aircraft
        default:                    return "/^";  // aircraft
    }
}

static int format_aprs_position(const flarm_message_t *msg, char *buf, size_t bufsize)
{
    time_t now = time(NULL);
    struct tm *utc = gmtime(&now);

    // Latitude to DDMM.mm format
    double lat_abs = fabs(msg->latitude);
    int lat_deg = (int)lat_abs;
    double lat_min = (lat_abs - lat_deg) * 60.0;
    char lat_ns = (msg->latitude >= 0) ? 'N' : 'S';

    // Longitude to DDDMM.mm format
    double lon_abs = fabs(msg->longitude);
    int lon_deg = (int)lon_abs;
    double lon_min = (lon_abs - lon_deg) * 60.0;
    char lon_ew = (msg->longitude >= 0) ? 'E' : 'W';

    // Altitude in feet
    int alt_feet = (int)(msg->altitude * 3.28084);
    if (alt_feet < 0) alt_feet = 0;

    // Ground speed in knots
    int speed_kts = (int)(msg->speed * 1.94384);

    // Course
    int course = (int)msg->course;
    if (course < 0) course += 360;
    if (course > 360) course = 0;

    // Climb rate in fpm
    int climb_fpm = (int)(msg->vs * 196.85);

    const char *prefix = flarm_addr_prefix(msg->addr_type);
    const char *symbol = flarm_acft_symbol(msg->aircraft_type);

    // Do NOT include qAS in path — the APRS-IS server adds qAS,login automatically
    int n = snprintf(buf, bufsize,
                     "%s%06X>OGFLR:/%02d%02d%02dh%02d%05.2f%c%c%03d%05.2f%c%c%03d/%03d/A=%06d !W00! id%02X%06X %+d0fpm +0.0rot\r\n",
                     prefix, msg->addr,
                     utc->tm_hour, utc->tm_min, utc->tm_sec,
                     lat_deg, lat_min, lat_ns,
                     symbol[0],
                     lon_deg, lon_min, lon_ew,
                     symbol[1],
                     course, speed_kts, alt_feet,
                     (unsigned)(((msg->stealth & 1) << 7) | ((msg->no_track & 1) << 6) | ((msg->addr_type & 3) << 4) | (msg->aircraft_type & 0x0F)),
                     msg->addr,
                     climb_fpm / 10);

    return n;
}

// ======================== System info helpers ========================

static float read_cpu_temperature(void)
{
    FILE *f = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (!f) return 0;
    int millideg = 0;
    if (fscanf(f, "%d", &millideg) != 1) millideg = 0;
    fclose(f);
    return millideg / 1000.0f;
}

static float read_cpu_load(void)
{
    FILE *f = fopen("/proc/loadavg", "r");
    if (!f) return 0;
    float load = 0;
    if (fscanf(f, "%f", &load) != 1) load = 0;
    fclose(f);
    return load;
}

static void read_ram_info(float *used_mb, float *total_mb)
{
    *used_mb = 0;
    *total_mb = 0;
    struct sysinfo si;
    if (sysinfo(&si) == 0) {
        unsigned long unit = si.mem_unit;
        *total_mb = (float)(si.totalram * unit) / (1024.0f * 1024.0f);
        float free_mb = (float)((si.freeram + si.bufferram) * unit) / (1024.0f * 1024.0f);
        *used_mb = *total_mb - free_mb;
    }
}

static void read_ntp_info(float *offset_ms, float *ppm)
{
    *offset_ms = 0;
    *ppm = 0;
    FILE *f = popen("chronyc tracking 2>/dev/null || ntpq -c rv 2>/dev/null", "r");
    if (!f) return;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        // chronyc: "System time     : 0.000001234 seconds slow of NTP time"
        // chronyc: "Frequency       : 1.234 ppm slow"
        float val;
        if (sscanf(line, "System time : %f seconds", &val) == 1) {
            *offset_ms = val * 1000.0f;
        } else if (sscanf(line, "Frequency : %f ppm", &val) == 1) {
            *ppm = val;
            if (strstr(line, "slow")) *ppm = -*ppm;
        }
        // ntpq: offset=1.234, frequency=5.678
        char *p;
        if ((p = strstr(line, "offset=")) != NULL) {
            if (sscanf(p, "offset=%f", &val) == 1)
                *offset_ms = val;
        }
        if ((p = strstr(line, "frequency=")) != NULL) {
            if (sscanf(p, "frequency=%f", &val) == 1)
                *ppm = val;
        }
    }
    pclose(f);
}

static int count_unique_aircraft(void)
{
    time_t now = time(NULL);
    int count = 0;
    for (unsigned i = 0; i < OGN_ACFT_HASH_SIZE; i++) {
        if (OgnClient.acft[i].addr != 0 &&
            (now - OgnClient.acft[i].last_seen) < OGN_ACFT_MAX_AGE) {
            count++;
        }
    }
    return count;
}

// ======================== Station beacon ========================

static void ogn_send_station_beacon(void)
{
    if (OgnClient.fd < 0 || !OgnClient.connected) return;
    if (Modes.fUserLat == 0.0 && Modes.fUserLon == 0.0) return;

    time_t now = time(NULL);
    struct tm *utc = gmtime(&now);

    // Latitude to DDMM.mm format
    double lat_abs = fabs(Modes.fUserLat);
    int lat_deg = (int)lat_abs;
    double lat_min = (lat_abs - lat_deg) * 60.0;
    char lat_ns = (Modes.fUserLat >= 0) ? 'N' : 'S';

    // Longitude to DDDMM.mm format
    double lon_abs = fabs(Modes.fUserLon);
    int lon_deg = (int)lon_abs;
    double lon_min = (lon_abs - lon_deg) * 60.0;
    char lon_ew = (Modes.fUserLon >= 0) ? 'E' : 'W';

    // Altitude in feet from MLAT config (--alt), fallback to 497m
    int alt_feet = (MlatConfig.alt > 0) ? (int)(MlatConfig.alt * 3.28084) : 1631;

    // Position beacon: symbol table 'I' (overlay IGate) + symbol '&' = OGN receiver
    char beacon[512];
    int n = snprintf(beacon, sizeof(beacon),
                     "%s>OGNSDR,TCPIP*:/%02d%02d%02dh%02d%05.2f%cI%03d%05.2f%c&/A=%06d\r\n",
                     FlarmConfig.ogn_station,
                     utc->tm_hour, utc->tm_min, utc->tm_sec,
                     lat_deg, lat_min, lat_ns,
                     lon_deg, lon_min, lon_ew,
                     alt_feet);

    if (n > 0 && n < (int)sizeof(beacon)) {
        int w = write(OgnClient.fd, beacon, n);
        if (w < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            fprintf(stderr, "ogn: beacon write error: %s\n", strerror(errno));
            close(OgnClient.fd);
            OgnClient.fd = -1;
            OgnClient.connected = 0;
            return;
        }
        if (w > 0)
            fprintf(stderr, "ogn: sent station beacon for %s\n", FlarmConfig.ogn_station);
    }

    // Gather system stats
    float cpu_temp = read_cpu_temperature();
    float cpu_load = read_cpu_load();
    float ram_used, ram_total;
    read_ram_info(&ram_used, &ram_total);
    float ntp_offset, ntp_ppm;
    read_ntp_info(&ntp_offset, &ntp_ppm);
    int acft_unique = count_unique_aircraft();
    int acft_total = acft_unique; // unique in last hour

    // Signal stats from FLARM (dB)
    float signal_peak_db = -999;
    float signal_avg_db = -999;
    if (OgnClient.signal_count > 0) {
        float avg = OgnClient.signal_sum / (float)OgnClient.signal_count;
        if (avg > 0) signal_avg_db = 10.0f * log10f(avg);
        if (OgnClient.signal_max > 0) signal_peak_db = 10.0f * log10f(OgnClient.signal_max);
    }

    // Noise floor from ADS-B stats (dB)
    float noise_db = -999;
    if (Modes.stats_latest.noise_power_count > 0) {
        double noise = Modes.stats_latest.noise_power_sum / (double)Modes.stats_latest.noise_power_count;
        if (noise > 0) noise_db = (float)(10.0 * log10(noise));
    }

    // Build OGN standard status beacon
    // Format: STATION>OGNSDR,TCPIP*:>HHMMSSh vSOFTWARE CPU:load RAM:used/totalMB NTP:offset/ppm +tempC N/NAcfts[1h] RF:noise+ppm/avgdB/peakdB
    char status[512];
    int pos = 0;
    pos += snprintf(status + pos, sizeof(status) - (size_t)pos,
                    "%s>OGNSDR,TCPIP*:>%02d%02d%02dh",
                    FlarmConfig.ogn_station,
                    utc->tm_hour, utc->tm_min, utc->tm_sec);

    // Software version
    pos += snprintf(status + pos, sizeof(status) - (size_t)pos,
                    " vdump1090-gg " MODES_DUMP1090_VERSION);

    // CPU load
    pos += snprintf(status + pos, sizeof(status) - (size_t)pos,
                    " CPU:%.1f", cpu_load);

    // RAM
    if (ram_total > 0) {
        pos += snprintf(status + pos, sizeof(status) - (size_t)pos,
                        " RAM:%.1f/%.1fMB", ram_used, ram_total);
    }

    // NTP
    pos += snprintf(status + pos, sizeof(status) - (size_t)pos,
                    " NTP:%.1fms/%+.1fppm", ntp_offset, ntp_ppm);

    // Temperature
    if (cpu_temp > 0) {
        pos += snprintf(status + pos, sizeof(status) - (size_t)pos,
                        " %+.1fC", cpu_temp);
    }

    // Aircraft count
    pos += snprintf(status + pos, sizeof(status) - (size_t)pos,
                    " %d/%dAcfts[1h]", acft_unique, acft_total);

    // RF stats
    if (noise_db > -999 || signal_avg_db > -999) {
        pos += snprintf(status + pos, sizeof(status) - (size_t)pos, " RF:");
        if (noise_db > -999)
            pos += snprintf(status + pos, sizeof(status) - (size_t)pos, "%+.0f", noise_db);
        pos += snprintf(status + pos, sizeof(status) - (size_t)pos, "+0.0ppm");
        if (signal_avg_db > -999)
            pos += snprintf(status + pos, sizeof(status) - (size_t)pos, "/%.1fdB", signal_avg_db);
        if (signal_peak_db > -999)
            pos += snprintf(status + pos, sizeof(status) - (size_t)pos, "/%.1fdB", signal_peak_db);
    }

    pos += snprintf(status + pos, sizeof(status) - (size_t)pos, "\r\n");

    if (pos > 0 && pos < (int)sizeof(status)) {
        if (write(OgnClient.fd, status, (size_t)pos) < 0) { /* ignore */ }
    }

    // Reset signal stats for next period
    OgnClient.signal_max = 0;
    OgnClient.signal_sum = 0;
    OgnClient.signal_count = 0;
    OgnClient.acft_total = 0;
}

// ======================== Send queued positions ========================

static void ogn_send_queued(void)
{
    if (OgnClient.fd < 0 || !OgnClient.connected) return;

    while (OgnClient.queue_tail != OgnClient.queue_head) {
        flarm_message_t *msg = &OgnClient.queue[OgnClient.queue_tail];

        char aprs_line[512];
        int len = format_aprs_position(msg, aprs_line, sizeof(aprs_line));
        if (len <= 0 || len >= (int)sizeof(aprs_line)) {
            OgnClient.queue_tail = (OgnClient.queue_tail + 1) % OGN_QUEUE_SIZE;
            continue;
        }

        int w = write(OgnClient.fd, aprs_line, len);
        if (w < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Socket buffer full, try later
                return;
            }
            // Connection lost
            fprintf(stderr, "ogn: write error: %s\n", strerror(errno));
            close(OgnClient.fd);
            OgnClient.fd = -1;
            OgnClient.connected = 0;
            return;
        }

        OgnClient.packets_sent++;
        OgnClient.queue_tail = (OgnClient.queue_tail + 1) % OGN_QUEUE_SIZE;
    }
}

// ======================== Periodic work ========================

void ognClientPeriodicWork(void)
{
    if (!FlarmConfig.enabled || FlarmConfig.ogn_station[0] == '\0') return;

    uint64_t now = mstime();

    if (!OgnClient.connected) {
        // Try to reconnect
        if (now < OgnClient.next_reconnect) return;
        OgnClient.next_reconnect = now + OGN_RECONNECT_INTERVAL;

        int fd = ogn_connect();
        if (fd < 0) return;

        if (!ogn_login(fd)) {
            close(fd);
            return;
        }

        OgnClient.fd = fd;
        OgnClient.connected = 1;
        OgnClient.last_keepalive = now;
        OgnClient.last_beacon = 0;  // force immediate beacon
    }

    // Read and discard any data from server (keepalive / server messages)
    if (OgnClient.fd >= 0) {
        char discard[1024];
        while (read(OgnClient.fd, discard, sizeof(discard)) > 0) {
            // discard server data
        }
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
            // Connection lost
            fprintf(stderr, "ogn: connection lost: %s\n", strerror(errno));
            close(OgnClient.fd);
            OgnClient.fd = -1;
            OgnClient.connected = 0;
            return;
        }
    }

    // Send station beacon periodically
    if (now - OgnClient.last_beacon >= OGN_BEACON_INTERVAL) {
        ogn_send_station_beacon();
        OgnClient.last_beacon = now;
    }

    // Send queued positions
    ogn_send_queued();

    // Keepalive
    if (now - OgnClient.last_keepalive >= OGN_KEEPALIVE_INTERVAL) {
        const char *ka = "# keepalive\r\n";
        int w = write(OgnClient.fd, ka, strlen(ka));
        if (w < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            fprintf(stderr, "ogn: keepalive failed\n");
            close(OgnClient.fd);
            OgnClient.fd = -1;
            OgnClient.connected = 0;
            return;
        }
        OgnClient.last_keepalive = now;
    }
}

// ======================== Cleanup ========================

void ognClientCleanup(void)
{
    if (OgnClient.fd >= 0) {
        close(OgnClient.fd);
        OgnClient.fd = -1;
    }
    OgnClient.connected = 0;

    if (OgnClient.packets_sent > 0) {
        fprintf(stderr, "ogn: total packets sent: %llu\n",
                (unsigned long long)OgnClient.packets_sent);
    }
}
