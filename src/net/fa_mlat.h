// SPDX-License-Identifier: GPL-3.0-or-later
//
// fa_mlat.h: Built-in FlightAware MLAT client for dump1090-gg
//
// Replaces the external fa-mlat-client Python program.
// Runs as a thread, reads decoded messages from SPSC queue,
// sends MLAT data via UDP binary protocol to FA's MLAT server.
//
// Protocol reference: fa-mlat-client by Oliver Jowett, GPL-3+
// https://github.com/mutability/mlat-client (flightaware/client/adeptclient.py)

#ifndef FA_MLAT_H
#define FA_MLAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <pthread.h>

// Forward declaration
struct modesMessage;

// ============================= Constants ================================

#define FA_MLAT_MAX_WANTED      4096     // max tracked ICAO addresses
#define FA_MLAT_MAX_MODEAC      256      // max Mode A/C codes
#define FA_MLAT_HASH_SIZE       2048     // aircraft hash table size (power of 2)
#define FA_MLAT_HASH_MASK       (FA_MLAT_HASH_SIZE - 1)

#define FA_MLAT_STATUS_LINES    16       // max pending status lines
#define FA_MLAT_STATUS_LINE_LEN 2048     // max bytes per status line

#define FA_MLAT_REPORT_INTERVAL 30000    // 30s between seen/lost/rate reports (ms)
#define FA_MLAT_EXPIRY_AGE      60000    // 60s aircraft expiry (ms)
#define FA_MLAT_POSITION_EXPIRY 30000    // 30s position expiry (ms)
#define FA_MLAT_MIN_MESSAGES    10       // min messages before sending MLAT data
#define FA_MLAT_UDP_REPORT_INTERVAL 60000 // 60s between UDP count reports (ms)
#define FA_MLAT_HEARTBEAT_MS    500      // 500ms heartbeat / flush interval

// UDP protocol message types
#define FA_MLAT_TYPE_SYNC       1
#define FA_MLAT_TYPE_MLAT_SHORT 2
#define FA_MLAT_TYPE_MLAT_LONG  3
#define FA_MLAT_TYPE_REBASE     5
#define FA_MLAT_TYPE_ABS_SYNC   6
#define FA_MLAT_TYPE_MLAT_MODEAC 7

// UDP struct sizes (big-endian packed, no padding)
#define FA_MLAT_HDR_SIZE        14       // key(4)+seq(2)+base_ts(8)
#define FA_MLAT_SYNC_SIZE       40       // type(1)+icao(3)+edelta(4)+odelta(4)+even(14)+odd(14)
#define FA_MLAT_SHORT_SIZE      15       // type(1)+icao(3)+delta(4)+msg(7)
#define FA_MLAT_LONG_SIZE       22       // type(1)+icao(3)+delta(4)+msg(14)
#define FA_MLAT_REBASE_SIZE     9        // type(1)+ts(8)
#define FA_MLAT_ABS_SYNC_SIZE   48       // type(1)+icao(3)+ts1(8)+ts2(8)+even(14)+odd(14)
#define FA_MLAT_MODEAC_SIZE     7        // type(1)+delta(4)+msg(2)

// Beast clock frequency (12 MHz)
#define FA_MLAT_CLOCK_FREQ      12000000ULL
// Max timestamp delta for sync pairs: 5 seconds worth of 12MHz ticks
#define FA_MLAT_SYNC_MAX_DELTA  (5 * FA_MLAT_CLOCK_FREQ)

// ============================= Data Types ================================

// Per-aircraft tracking for FA MLAT coordinator
struct fa_mlat_aircraft {
    uint32_t addr;                   // ICAO address (0 = empty slot)
    int      messages;               // total messages seen
    uint64_t last_message_time;      // monotonic ms
    uint64_t last_position_time;     // monotonic ms (last DF17 position)
    int      requested;              // server wants data for this aircraft
    int      reported;               // we've reported seeing this aircraft

    // DF17 sync pair tracking
    int      has_even;
    int      has_odd;
    uint64_t even_timestamp;         // 12MHz Beast timestamp
    uint64_t odd_timestamp;          // 12MHz Beast timestamp
    uint8_t even_msg[14];      // raw even CPR message
    uint8_t odd_msg[14];       // raw odd CPR message
    int      even_msgbits;
    int      odd_msgbits;
    uint32_t even_nucp;              // NUCp of even message
    uint32_t odd_nucp;               // NUCp of odd message

    // Rate measurement
    uint64_t rate_measurement_start; // monotonic ms
    int      recent_adsb_positions;
};

// Shared state between PiAware thread and FA MLAT thread
struct fa_mlat_state {
    // --- Control (PiAware sets, FA MLAT reads) ---
    pthread_mutex_t ctl_mutex;
    int      enabled;                // FA MLAT active
    int      stop_requested;         // signal thread to stop

    // UDP transport config
    char     udp_host[256];
    int      udp_port;
    uint32_t udp_key;

    // Traffic filter
    uint32_t wanted_icao[FA_MLAT_MAX_WANTED];
    int      wanted_count;
    uint32_t wanted_modeac[FA_MLAT_MAX_MODEAC];
    int      wanted_modeac_count;

    // --- Status output (FA MLAT writes, PiAware reads) ---
    pthread_mutex_t status_mutex;
    char     status_lines[FA_MLAT_STATUS_LINES][FA_MLAT_STATUS_LINE_LEN];
    int      status_head;
    int      status_tail;

    // --- Thread ---
    pthread_t thread;
    int       thread_running;

    // --- Aircraft hash table (FA MLAT thread only) ---
    struct fa_mlat_aircraft aircraft[FA_MLAT_HASH_SIZE];
};

extern struct fa_mlat_state FaMlat;

// ============================= Public API ================================

// Initialize (call once at startup)
void faMlatInit(void);

// Enable FA MLAT with UDP transport parameters (called from PiAware thread)
void faMlatEnable(const char *host, int port, uint32_t key);

// Disable FA MLAT (called from PiAware thread)
void faMlatDisable(void);

// Update wanted ICAO set: add these to the wanted set
void faMlatStartSending(const uint32_t *icao, int count,
                        const uint32_t *modeac, int modeac_count);

// Update wanted ICAO set: remove these from the wanted set
void faMlatStopSending(const uint32_t *icao, int count,
                       const uint32_t *modeac, int modeac_count);

// Process a decoded message (called from FA MLAT thread via SPSC queue)
void faMlatProcessMessage(struct modesMessage *mm);

// Poll for status lines to forward to FA server
// Returns 1 if a line was retrieved (copied to buf), 0 if none pending
int faMlatPollStatus(char *buf, int bufsize);

// Inject an MLAT result position (called from PiAware thread when FA sends mlat_result)
void faMlatInjectResult(uint32_t addr, double lat, double lon, double alt,
                        double nsvel, double ewvel, double vrate,
                        int anon, int modeac);

// Cleanup
void faMlatCleanup(void);

#ifdef __cplusplus
}
#endif

#endif // FA_MLAT_H
