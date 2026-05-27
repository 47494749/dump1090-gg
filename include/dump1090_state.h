// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// dump1090_state.h: global program state structure
//
// Requires: dump1090_defs.h, dump1090_types.h, net_io.h, stats.h, elm.h
//           to be included before this header.

#ifndef DUMP1090_STATE_H
#define DUMP1090_STATE_H

// Program global state
struct _Modes {                             // Internal state
    pthread_t       reader_thread;

    pthread_mutex_t reader_cpu_mutex;                     // mutex protecting reader_cpu_accumulator
    struct timespec reader_cpu_accumulator;               // accumulated CPU time used by the reader thread
    struct timespec reader_cpu_start;                     // start time for the last reader thread CPU measurement

    uint32_t        trailing_samples;                     // extra trailing samples in magnitude buffers
    double          sample_rate;                          // actual sample rate in use (in hz)

    uint16_t       *log10lut;        // Magnitude -> log10 lookup table
    atomic_int      exit;            // Exit from the main loop when true (2 = unclean exit)

    // Sample conversion
    int            dc_filter;        // should we apply a DC filter?

    // RTLSDR and some other SDRs
    char *        dev_name;
    float         gain;              // value in dB, or MODES_AUTO_GAIN, or MODES_MAX_GAIN
    int           freq;

    // Networking
    char           aneterr[ANET_ERR_LEN];
    struct net_service *services;    // Active services
    struct client *clients;          // Our clients

    struct net_service *beast_verbatim_service;        // Beast-format output service, verbatim mode
    struct net_service *beast_verbatim_local_service;  // Beast-format output service, verbatim+local mode
    struct net_service *beast_cooked_service;          // Beast-format output service, "cooked" mode

    struct net_writer raw_out;                   // AVR-format output
    struct net_writer beast_verbatim_out;        // Beast-format output, verbatim mode
    struct net_writer beast_verbatim_local_out;  // Beast-format output, verbatim+local mode
    struct net_writer beast_cooked_out;          // Beast-format output, "cooked" mode
    struct net_writer sbs_out;                   // SBS-format output
    struct net_writer stratux_out;               // Stratux-format output
    struct net_writer fatsv_out;                 // FATSV-format output

#ifdef _WIN32
    WSADATA        wsaData;          // Windows socket initialisation
#endif

    // Configuration
    sdr_type_t sdr_type;             // where are we getting data from?
    int   nfix_crc;                  // Number of crc bit error(s) to correct
    int   check_crc;                 // Only display messages with good CRC
    int   fix_df;                    // Try to correct damage to the DF field, as well as the main message body
    int   enable_df24;               // Enable decoding of DF24..DF31 (Comm-D ELM)
    int   raw;                       // Raw output format
    int   mode_ac;                   // Enable decoding of SSR Modes A & C
    int   mode_ac_auto;              // allow toggling of A/C by Beast commands
    int   net;                       // Enable networking
    int   net_only;                  // Enable just networking
    uint64_t net_heartbeat_interval; // TCP heartbeat interval (milliseconds)
    int   net_output_flush_size;     // Minimum Size of output data
    uint64_t net_output_flush_interval; // Maximum interval (in milliseconds) between outputwrites
    char *net_output_raw_ports;      // List of raw output TCP ports
    char *net_input_raw_ports;       // List of raw input TCP ports
    char *net_output_sbs_ports;      // List of SBS output TCP ports
    char *net_output_stratux_ports;  // List of Stratux output TCP ports
    char *net_input_beast_ports;     // List of Beast input TCP ports
    char *net_output_beast_ports;    // List of Beast output TCP ports
    char *net_bind_address;          // Bind address
    int   net_sndbuf_size;           // TCP output buffer size (64Kb * 2^n)
    int   net_verbatim;              // if true, Beast output connections default to verbatim mode
    int   forward_mlat;              // allow forwarding of mlat messages to output ports

    // Beast feed outputs (ADSBx, OpenSky, adsb.fi, etc.)
    #define MAX_BEAST_FEEDS 16
    #define FEED_FORMAT_BEAST        0
    #define FEED_FORMAT_RAW          1
    #define FEED_FORMAT_SBS          2
    #define FEED_FORMAT_BEAST_REDUCE 3
    struct {
        char  name[32];              // display name (e.g. "ADSBx", "OpenSky")
        char *host;                  // feed host
        int   port;                  // feed port
        int   format;                // FEED_FORMAT_BEAST, FEED_FORMAT_RAW, or FEED_FORMAT_SBS
        int   enabled;               // 1=active, 0=disabled by user
    } beast_feeds[MAX_BEAST_FEEDS];
    int beast_feed_count;
    int beast_reduce_interval;       // BeastReduce min interval in ms (default 250)
    char *adsbhub_ckey;              // ADSBHub station ckey for dynamic IP update

    // Airframes.io ACARS/VDL2 UDP feed
    struct {
        char *host;                  // feed host (default: feed.acars.io)
        int   port;                  // feed port (default: 5550 for ACARS, 5552 for VDL2)
        int   enabled;               // 1=active, 0=disabled
    } airframes_acars_feed, airframes_vdl2_feed;
    char  airframes_station_id[32];  // station identifier for airframes.io

    int   quiet;                     // Suppress stdout
    uint32_t show_only;              // Only show messages from this ICAO
    int   interactive;               // Interactive mode
    uint64_t interactive_display_ttl;// Interactive mode: TTL display
    int interactive_display_size;    // Size of TTL display
    int   interactive_show_distance; // Show aircraft distance and bearing instead of lat/lon
    interactive_distance_unit_t interactive_distance_units; // Units for interactive distance display
    char *interactive_callsign_filter; // Filter for interactive display callsigns
    uint64_t stats;                  // Interval (millis) between stats dumps,
    int   stats_range_histo;         // Collect/show a range histogram?
    int   onlyaddr;                  // Print only ICAO addresses
    int   tisb_verbose;              // Log TIS-B/ADS-R messages to stderr
    int   crc_rescue;                // CRC-based message rescue: accept msgs with corrupted preambles
    int   metric;                    // Use metric units
    int   use_gnss;                  // Use GNSS altitudes with H suffix ("HAE", though it isn't always) when available
    int   mlat;                      // Use Beast ascii format for raw data output, i.e. @...; iso *...;
    char *json_dir;                  // Path to json base directory, or NULL not to write json.
    uint64_t json_interval;          // Interval between rewriting the json aircraft file, in milliseconds; also the advertised map refresh interval
    uint64_t json_stats_interval;    // Interval between rewriting the json stats file, in milliseconds
    int   json_location_accuracy;    // Accuracy of location metadata: 0=none, 1=approx, 2=exact
    double faup_rate_multiplier;     // Multiplier to adjust rate of faup1090 messages emitted
    bool faup_upload_unknown_commb;  // faup1090: should we upload Comm-B messages that weren't in a recognized format?

    int   json_aircraft_history_next;
    struct {
        char *content;
        int clen;
    } json_aircraft_history[HISTORY_SIZE];

    // User details
    double fUserLat;                // Users receiver/antenna lat/lon needed for initial surface location
    double fUserLon;                // Users receiver/antenna lat/lon needed for initial surface location
    int    bUserFlags;              // Flags relating to the user details
    double maxRange;                // Absolute maximum decoding range, in *metres*

    // State tracking
    struct aircraft *aircrafts;

    // ELM (Comm-D) reassembly
    struct elm_state elm;

    // Statistics
    struct stats stats_current;     // Currently accumulating stats, this is where all stats are initially collected
    struct stats stats_alltime;     // Accumulated stats since the start of the process
    struct stats stats_periodic;    // Accumulated stats since the last periodic stats display (--stats-every)
    struct stats stats_latest;      // Accumulated stats since the end of the last 1-minute period
    struct stats stats_1min[15];    // Accumulated stats for a full 1-minute window; this is a ring buffer maintaining a history of 15 minutes
    int stats_newest_1min;          // Index into stats_1min of the most recent 1-minute window
    struct stats stats_5min;        // Accumulated stats from the last 5 complete 1-minute windows
    struct stats stats_15min;       // Accumulated stats from the last 15 complete 1-minute windows

    // Adaptive gain config
    float adaptive_min_gain_db;
    float adaptive_max_gain_db;

    float adaptive_duty_cycle;

    bool adaptive_burst_control;
    float adaptive_burst_alpha;
    uint32_t adaptive_burst_change_delay;
    float adaptive_burst_loud_rate;
    uint32_t adaptive_burst_loud_runlength;
    float adaptive_burst_quiet_rate;
    uint32_t adaptive_burst_quiet_runlength;

    bool adaptive_range_control;
    float adaptive_range_alpha;
    uint32_t adaptive_range_percentile;
    float adaptive_range_target;
    uint32_t adaptive_range_change_delay;
    uint32_t adaptive_range_scan_delay;
    uint32_t adaptive_range_rescan_delay;
};

extern struct _Modes Modes;

#endif // DUMP1090_STATE_H
