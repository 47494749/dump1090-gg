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

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>
#include <unistd.h>
#include <cerrno>
#include <ctime>
#include <cmath>
#include <ctype.h>
#include <sys/socket.h>
#include <sys/sysinfo.h>
#include <netdb.h>
#include <fcntl.h>

#include "dump1090.h"
#include "gg_format.h"

// ======================== State ========================

#define OGN_QUEUE_SIZE 128
#define OGN_RECONNECT_INTERVAL 30000  // ms
#define OGN_KEEPALIVE_INTERVAL 120000 // ms
#define OGN_BEACON_INTERVAL   300000 // ms (5 minutes)
#define OGN_ACFT_HASH_SIZE    256
#define OGN_ACFT_MAX_AGE      3600    // seconds

static struct {
    int32_t           fd;
    int32_t           connected;
    uint64_t      next_reconnect;
    uint64_t      last_keepalive;
    uint64_t      last_beacon;

    flarm_message_t queue[OGN_QUEUE_SIZE];
    uint32_t      queue_head;
    uint32_t      queue_tail;

    uint64_t      packets_sent;

    // Aircraft tracking for beacon stats
    struct {
        uint32_t addr;
        time_t   last_seen;
    } acft[OGN_ACFT_HASH_SIZE];
    uint32_t      acft_total;      // total positions received since last beacon
    float         signal_max;      // max signal level since last beacon
    float         signal_sum;      // sum of signal levels
    uint32_t      signal_count;    // number of signal samples
} OgnClient;

// ======================== Init ========================

void ognClientInit(void)
{
    OgnClient = {};
    OgnClient.fd = -1;
    OgnClient.connected = 0;
    OgnClient.next_reconnect = 0;
}

// ======================== Queue ========================

void ognClientSubmit(const flarm_message_t *msg)
{
    if (!FlarmConfig.enabled || FlarmConfig.ogn_station.empty()) return;

    uint32_t next = (OgnClient.queue_head + 1) % OGN_QUEUE_SIZE;
    if (next == OgnClient.queue_tail) return;  // full

    OgnClient.queue[OgnClient.queue_head] = *msg;
    OgnClient.queue_head = next;

    // Track unique aircraft (simple hash table)
    uint32_t slot = msg->addr % OGN_ACFT_HASH_SIZE;
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
    struct addrinfo hints = {}, *res, *rp;

    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    std::string port_str = std::to_string(FlarmConfig.ogn_port);

    int err = getaddrinfo(FlarmConfig.ogn_server.c_str(), port_str.c_str(), &hints, &res);
    if (err != 0) {
        fprintf(stderr, "ogn: DNS resolution failed for %s: %s\n",
                FlarmConfig.ogn_server.c_str(), gai_strerror(err));
        return -1;
    }

    int32_t fd = -1;
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
                int32_t so_error;
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
                FlarmConfig.ogn_server.c_str(), FlarmConfig.ogn_port);
        return -1;
    }

    // Set non-blocking for normal operation
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    return fd;
}

// ======================== APRS Passcode ========================

static int32_t aprs_passcode(const char *callsign)
{
    int32_t hash = 0x73e2;
    std::string call;

    // Copy callsign without SSID (strip everything after '-'), uppercase
    for (int32_t i = 0; callsign[i] && callsign[i] != '-' && i < 15; i++)
        call += (char)toupper((uint8_t)callsign[i]);

    for (size_t i = 0; i < call.size(); i += 2) {
        hash ^= ((uint8_t)call[i]) << 8;
        if (i + 1 < call.size())
            hash ^= (uint8_t)call[i + 1];
    }
    return hash & 0x7FFF;
}

// ======================== Login ========================

static bool ogn_login(int32_t fd)
{
    int32_t passcode = aprs_passcode(FlarmConfig.ogn_station.c_str());
    char tmp[256];
    snprintf(tmp, sizeof(tmp),
             "user %s pass %d vers dump1090-gg " MODES_DUMP1090_VERSION
             " filter r/%.4f/%.4f/100\r\n",
             FlarmConfig.ogn_station.c_str(),
                     passcode,
                     Modes.fUserLat, Modes.fUserLon);
    std::string login(tmp);

    // Blocking send for login line
    size_t sent = 0;
    while (sent < login.size()) {
        int w = write(fd, login.data() + sent, login.size() - sent);
        if (w <= 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            return false;
        }
        sent += w;
    }

    fprintf(stderr, "ogn: logged in as %s to %s:%d\n",
            FlarmConfig.ogn_station.c_str(), FlarmConfig.ogn_server.c_str(), FlarmConfig.ogn_port);
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

static std::string format_aprs_position(const flarm_message_t *msg)
{
    time_t now = time(NULL);
    struct tm *utc = gmtime(&now);

    // Latitude to DDMM.mm format
    double lat_abs = fabs(msg->latitude);
    int32_t lat_deg = (int32_t)lat_abs;
    double lat_min = (lat_abs - lat_deg) * 60.0;
    char lat_ns = (msg->latitude >= 0) ? 'N' : 'S';

    // Longitude to DDDMM.mm format
    double lon_abs = fabs(msg->longitude);
    int32_t lon_deg = (int32_t)lon_abs;
    double lon_min = (lon_abs - lon_deg) * 60.0;
    char lon_ew = (msg->longitude >= 0) ? 'E' : 'W';

    // Altitude in feet
    int32_t alt_feet = (int32_t)(msg->altitude * 3.28084);
    if (alt_feet < 0) alt_feet = 0;

    // Ground speed in knots
    int32_t speed_kts = (int32_t)(msg->speed * 1.94384);

    // Course
    int32_t course = (int32_t)msg->course;
    if (course < 0) course += 360;
    if (course > 360) course = 0;

    // Climb rate in fpm
    int32_t climb_fpm = (int32_t)(msg->vs * 196.85);

    const char *prefix = flarm_addr_prefix(msg->addr_type);
    const char *symbol = flarm_acft_symbol(msg->aircraft_type);

    char tmp[512];
    snprintf(tmp, sizeof(tmp),
                     "%s%06X>OGFLR:/%02d%02d%02dh%02d%05.2f%c%c%03d%05.2f%c%c%03d/%03d/A=%06d !W00! id%02X%06X %+d0fpm +0.0rot\r\n",
                     prefix, msg->addr,
                     utc->tm_hour, utc->tm_min, utc->tm_sec,
                     lat_deg, lat_min, lat_ns,
                     symbol[0],
                     lon_deg, lon_min, lon_ew,
                     symbol[1],
                     course, speed_kts, alt_feet,
                     (uint32_t)(((msg->stealth & 1) << 7) | ((msg->no_track & 1) << 6) | ((msg->addr_type & 3) << 4) | (msg->aircraft_type & 0x0F)),
                     msg->addr,
                     climb_fpm / 10);

    return tmp;
}

// ======================== System info helpers ========================

static float read_cpu_temperature(void)
{
    FILE *f = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (!f) return 0;
    int32_t millideg = 0;
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
        uint64_t unit = si.mem_unit;
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
        std::string_view line_sv(line);
        if (sscanf(line, "System time : %f seconds", &val) == 1) {
            *offset_ms = val * 1000.0f;
        } else if (sscanf(line, "Frequency : %f ppm", &val) == 1) {
            *ppm = val;
            if (line_sv.find("slow") != std::string_view::npos) *ppm = -*ppm;
        }
        // ntpq: offset=1.234, frequency=5.678
        size_t pos;
        if ((pos = line_sv.find("offset=")) != std::string_view::npos) {
            if (sscanf(line + pos, "offset=%f", &val) == 1)
                *offset_ms = val;
        }
        if ((pos = line_sv.find("frequency=")) != std::string_view::npos) {
            if (sscanf(line + pos, "frequency=%f", &val) == 1)
                *ppm = val;
        }
    }
    pclose(f);
}

static int32_t count_unique_aircraft(void)
{
    time_t now = time(NULL);
    int32_t count = 0;
    for (uint32_t i = 0; i < OGN_ACFT_HASH_SIZE; i++) {
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
    int32_t lat_deg = (int32_t)lat_abs;
    double lat_min = (lat_abs - lat_deg) * 60.0;
    char lat_ns = (Modes.fUserLat >= 0) ? 'N' : 'S';

    // Longitude to DDDMM.mm format
    double lon_abs = fabs(Modes.fUserLon);
    int32_t lon_deg = (int32_t)lon_abs;
    double lon_min = (lon_abs - lon_deg) * 60.0;
    char lon_ew = (Modes.fUserLon >= 0) ? 'E' : 'W';

    // Altitude in feet from MLAT config (--alt), fallback to 497m
    int32_t alt_feet = (MlatConfig.alt > 0) ? (int32_t)(MlatConfig.alt * 3.28084) : 1631;

    // Position beacon: symbol table 'I' (overlay IGate) + symbol '&' = OGN receiver
    char bcn_tmp[512];
    snprintf(bcn_tmp, sizeof(bcn_tmp),
                     "%s>OGNSDR,TCPIP*:/%02d%02d%02dh%02d%05.2f%cI%03d%05.2f%c&/A=%06d\r\n",
                     FlarmConfig.ogn_station.c_str(),
                     utc->tm_hour, utc->tm_min, utc->tm_sec,
                     lat_deg, lat_min, lat_ns,
                     lon_deg, lon_min, lon_ew,
                     alt_feet);
    std::string beacon(bcn_tmp);

    {
        int w = write(OgnClient.fd, beacon.data(), beacon.size());
        if (w < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            gg::eprint("ogn: beacon write error: %s\n", strerror(errno));
            close(OgnClient.fd);
            OgnClient.fd = -1;
            OgnClient.connected = 0;
            return;
        }
        if (w > 0)
            gg::eprint("ogn: sent station beacon for %s\n", FlarmConfig.ogn_station.c_str());
    }

    // Gather system stats
    float cpu_temp = read_cpu_temperature();
    float cpu_load = read_cpu_load();
    float ram_used, ram_total;
    read_ram_info(&ram_used, &ram_total);
    float ntp_offset, ntp_ppm;
    read_ntp_info(&ntp_offset, &ntp_ppm);
    int32_t acft_unique = count_unique_aircraft();
    int32_t acft_total = acft_unique; // unique in last hour

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

    // Build OGN standard status beacon using std::string
    char st_tmp[128];
    snprintf(st_tmp, sizeof(st_tmp), "%s>OGNSDR,TCPIP*:>%02d%02d%02dh",
             FlarmConfig.ogn_station.c_str(), utc->tm_hour, utc->tm_min, utc->tm_sec);
    std::string status(st_tmp);

    // Software version
    status += " vdump1090-gg " MODES_DUMP1090_VERSION;

    // CPU load
    snprintf(st_tmp, sizeof(st_tmp), " CPU:%.1f", cpu_load);
    status += st_tmp;

    // RAM
    if (ram_total > 0) {
        snprintf(st_tmp, sizeof(st_tmp), " RAM:%.1f/%.1fMB", ram_used, ram_total);
        status += st_tmp;
    }

    // NTP
    snprintf(st_tmp, sizeof(st_tmp), " NTP:%.1fms/%+.1fppm", ntp_offset, ntp_ppm);
    status += st_tmp;

    // Temperature
    if (cpu_temp > 0) {
        snprintf(st_tmp, sizeof(st_tmp), " %+.1fC", cpu_temp);
        status += st_tmp;
    }

    // Aircraft count
    status += " " + std::to_string(acft_unique) + "/" + std::to_string(acft_total) + "Acfts[1h]";

    // RF stats
    if (noise_db > -999 || signal_avg_db > -999) {
        status += " RF:";
        if (noise_db > -999) {
            snprintf(st_tmp, sizeof(st_tmp), "%+.0f", noise_db);
            status += st_tmp;
        }
        status += "+0.0ppm";
        if (signal_avg_db > -999) {
            snprintf(st_tmp, sizeof(st_tmp), "/%.1fdB", signal_avg_db);
            status += st_tmp;
        }
        if (signal_peak_db > -999) {
            snprintf(st_tmp, sizeof(st_tmp), "/%.1fdB", signal_peak_db);
            status += st_tmp;
        }
    }

    status += "\r\n";

    if (write(OgnClient.fd, status.data(), status.size()) < 0) { /* ignore */ }

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

        std::string aprs_line = format_aprs_position(msg);
        if (aprs_line.empty()) {
            OgnClient.queue_tail = (OgnClient.queue_tail + 1) % OGN_QUEUE_SIZE;
            continue;
        }

        int w = write(OgnClient.fd, aprs_line.data(), aprs_line.size());
        if (w < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Socket buffer full, try later
                return;
            }
            // Connection lost
            gg::eprint("ogn: write error: %s\n", strerror(errno));
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
    if (!FlarmConfig.enabled || FlarmConfig.ogn_station.empty()) return;

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
            gg::eprint("ogn: connection lost: %s\n", strerror(errno));
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
            gg::eprint("ogn: keepalive failed\n");
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
        fprintf(stderr, "ogn: total packets sent: %" PRIu64 "\n",
                (uint64_t)OgnClient.packets_sent);
    }
}
