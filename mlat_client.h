// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// mlat_client.h: built-in MLAT (multilateration) client
//
// Implements the JSON-over-TCP protocol used by mlat-server
// (https://github.com/adsbexchange/mlat-server) so that dump1090
// can participate in multilateration without an external mlat-client process.
//
// Supports up to MAX_MLAT_SERVERS independent MLAT server connections.
//
// Protocol reference: mlat-client by Oliver Jowett, GPL-3+

#ifndef MLAT_CLIENT_H
#define MLAT_CLIENT_H

#include <stdint.h>
#include <stdbool.h>

// Forward declaration (full definition in dump1090.h)
struct modesMessage;

// ============================= Configuration =============================

#define MAX_MLAT_SERVERS        16       // max simultaneous MLAT server connections
#define MLAT_READ_BUF_SIZE      65536    // TCP read buffer
#define MLAT_WRITE_BUF_SIZE     131072   // TCP write buffer (JSON lines can be bursty)
#define MLAT_HASH_SIZE          4096     // aircraft hash table size (power of 2)
#define MLAT_HASH_MASK          (MLAT_HASH_SIZE - 1)

// Timing intervals (milliseconds)
#define MLAT_RECONNECT_INTERVAL     10000    // 10s between reconnect attempts
#define MLAT_HEARTBEAT_INTERVAL     120000   // 120s heartbeat to server
#define MLAT_INACTIVITY_TIMEOUT     300000   // 300s (5 min) no data = disconnect
#define MLAT_UPDATE_INTERVAL        4500     // 4.5s between aircraft list updates
#define MLAT_REPORT_INTERVAL        4        // report every N updates (4 * 4.5s ≈ 18s)
#define MLAT_POSITION_EXPIRY        30000    // 30s before ADS-B "good" expires
#define MLAT_AIRCRAFT_EXPIRY        120000   // 120s before aircraft expires from MLAT tracker
#define MLAT_MIN_MESSAGES           10       // min messages before sending MLAT data

// ============================= Data Types ================================

typedef enum {
    MLAT_DISCONNECTED = 0,
    MLAT_CONNECTING,
    MLAT_HANDSHAKING,
    MLAT_READY
} mlat_state_t;

// Per-aircraft MLAT tracking data (stored in hash table)
struct mlat_aircraft {
    uint32_t addr;                   // ICAO address (0 = empty slot)
    int      messages;               // total messages seen
    uint64_t last_message;           // last message time (mstime)

    // ADS-B position tracking (for sync messages)
    bool     adsb_good;              // has recent ADS-B positions (even+odd)
    uint64_t last_even_time;         // last even CPR time (mstime)
    uint64_t last_odd_time;          // last odd CPR time (mstime)
    uint64_t even_timestamp;         // 12MHz timestamp of last even CPR msg
    uint64_t odd_timestamp;          // 12MHz timestamp of last odd CPR msg
    unsigned char even_msg[14];      // raw even CPR message bytes
    unsigned char odd_msg[14];       // raw odd CPR message bytes
    int      even_msgbits;           // bit length of even message
    int      odd_msgbits;            // bit length of odd message

    // Rate tracking
    int      recent_adsb_positions;
    uint64_t rate_measurement_start;

    // Flags per-server (bitmask, bit N = server N)
    uint32_t requested;              // server wants MLAT data for this aircraft
    uint32_t reported;               // we've reported this aircraft to server
};

// One MLAT server connection
struct mlat_server {
    // --- Configuration (set once) ---
    char    *host;
    int      port;

    // --- Connection state ---
    int      fd;                     // TCP socket (-1 if disconnected)
    mlat_state_t state;

    // --- I/O buffers ---
    char     readbuf[MLAT_READ_BUF_SIZE];
    int      readbuf_len;
    char     writebuf[MLAT_WRITE_BUF_SIZE];
    int      writebuf_len;

    // --- Timers ---
    uint64_t next_reconnect;
    uint64_t next_heartbeat;
    uint64_t last_data_received;
    uint64_t next_aircraft_update;
    int      report_counter;         // counts up to MLAT_REPORT_INTERVAL

    // --- Server-controlled settings ---
    bool     split_sync;             // server wants split sync messages

    // --- Server index (0..MAX_MLAT_SERVERS-1) ---
    int      index;

    // --- Mutual exclusion: peer server that shares the same backend ---
    int      peer_index;             // index of peer server (-1 = none)
};

// Global MLAT state
struct mlat_config {
    int      server_count;
    struct mlat_server servers[MAX_MLAT_SERVERS];

    // Feeder identity
    char    *user;                   // feeder name (e.g. "blau1")
    char    *uuid;                   // feeder UUID (read from uuid_file)
    char    *uuid_file;              // path to UUID file

    // Receiver position
    double   lat;
    double   lon;
    double   alt;                    // altitude in meters
    bool     position_set;           // true if lat/lon/alt configured

    // Options
    bool     return_results;         // request MLAT results back from server

    // Per-aircraft hash table (shared across all servers)
    struct mlat_aircraft aircraft[MLAT_HASH_SIZE];

    // Timing
    uint64_t last_aircraft_update;
};

extern struct mlat_config MlatConfig;

// ============================= Public API ================================

// Initialize MLAT client (called from modesInitNet)
void mlatClientInit(void);

// Periodic work: reconnect, heartbeat, aircraft updates, read/write (called from modesNetPeriodicWork)
void mlatClientPeriodicWork(void);

// Process a decoded message for MLAT (called from useModesMessage, after tracking)
void mlatClientProcessMessage(struct modesMessage *mm);

// Clean up on exit
void mlatClientCleanup(void);

// Disconnect all servers (e.g. when internet goes offline)
void mlatClientDisconnectAll(const char *reason);

// Add an MLAT server from CLI (returns 0 on success, -1 if full)
int mlatClientAddServer(const char *hostport);

#endif // MLAT_CLIENT_H
