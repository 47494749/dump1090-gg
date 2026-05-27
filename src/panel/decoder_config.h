// Part of dump1090-gg-light
//
// decoder_config.h: Per-decoder configuration structs
//
// Each decoder type has its own configuration struct with its specific options.
// Each USB dongle has its own hardware config struct.
// The SDR Manager holds the association between dongles and decoders.

#ifndef DECODER_CONFIG_H
#define DECODER_CONFIG_H


#include <stdint.h>

// ======================== Decoder Types ========================

typedef enum {
    DECODER_ADSB = 0,
    DECODER_FLARM,
    DECODER_ACARS,
    DECODER_VDL2,
    DECODER_RADIOSONDE,
    DECODER_POCSAG,
    DECODER_GSM,
    DECODER_LTE,
    DECODER_IOT868,
    DECODER_FANET,
    DECODER_SARSAT,
    DECODER_TYPE_COUNT
} decoder_type_t;

// ======================== ADS-B Decoder Config ========================

typedef struct {
    // Decode options
    int32_t     fix_crc;            // 0 = off, 1 = 1-bit fix, 2 = 2-bit fix
    bool    check_crc;          // only accept messages with good CRC
    bool    fix_df;             // try to correct DF field damage
    bool    enable_df24;        // decode DF24..DF31 (Comm-D ELM)
    bool    mode_ac;            // decode SSR Mode A/C
    bool    mode_ac_auto;       // allow toggling Mode A/C via Beast commands
    bool    crc_rescue;         // accept msgs with corrupted preambles
    bool    use_gnss;           // use GNSS altitudes when available
    bool    mlat;               // MLAT timestamp mode for Beast output

    // Adaptive gain
    bool    adaptive_range;
    bool    adaptive_burst;
    float   adaptive_min_gain;
    float   adaptive_max_gain;
    float   adaptive_duty_cycle;
    // Burst control
    float   adaptive_burst_alpha;
    uint32_t adaptive_burst_change_delay;
    float   adaptive_burst_loud_rate;
    uint32_t adaptive_burst_loud_runlength;
    float   adaptive_burst_quiet_rate;
    uint32_t adaptive_burst_quiet_runlength;
    // Range control
    float   adaptive_range_alpha;
    uint32_t adaptive_range_percentile;
    float   adaptive_range_target;
    uint32_t adaptive_range_change_delay;
    uint32_t adaptive_range_scan_delay;
    uint32_t adaptive_range_rescan_delay;
} adsb_decoder_config_t;

// ======================== FLARM Decoder Config ========================

typedef struct {
    bool        enabled;
    bool        ogn_only;
    char        ogn_server[128];
    int32_t         ogn_port;
    char        ogn_station[64];
    // Encryption keys (loaded from file or set via API)
    uint32_t    key_table[12];
    uint32_t    key2;
    uint32_t    key3;
    uint32_t    key4;
    uint32_t    key5[4];
    bool        keys_loaded;
    char        keys_file[256];     // path to keys file (for reference)
} flarm_decoder_config_t;

// ======================== ACARS Decoder Config ========================

typedef struct {
    bool    enabled;
    double  channel_freqs[8];   // up to 8 ACARS channels
    int32_t     num_channels;
    double  center_freq;
} acars_decoder_config_t;

// ======================== VDL2 Decoder Config ========================

typedef struct {
    bool    enabled;
    double  channel_freqs[8];   // up to 8 VDL2 channels
    int32_t     num_channels;
    double  center_freq;
    float   squelch_level;      // dBFS
} vdl2_decoder_config_t;

// ======================== Radiosonde Decoder Config ========================

typedef struct {
    bool    enabled;
    bool    sondehub_upload;
    bool    radiosondy_upload;
    bool    wettersonde_upload;
    char    callsign[64];
    double  center_freq;
} radiosonde_decoder_config_t;

// ======================== POCSAG Decoder Config ========================

typedef struct {
    bool    enabled;
    bool    output_enabled;     // show decoded messages
    double  channel_freqs[8];   // multi-channel frequencies
    int32_t     num_channels;
    double  center_freq;
} pocsag_decoder_config_t;

// ======================== GSM Decoder Config ========================

typedef struct {
    bool    enabled;
    bool    output_enabled;
    double  arfcn_freq;
    int32_t     tsc;                // -1 = auto-detect
} gsm_decoder_config_t;

// ======================== LTE Decoder Config ========================

typedef struct {
    bool    enabled;
    bool    output_enabled;
    bool    hop_enabled;        // Band 20 frequency hopping
    double  center_freq;
} lte_decoder_config_t;

// ======================== IoT 868 Decoder Config ========================

typedef struct {
    bool    enabled;
    bool    output_enabled;
    double  center_freq;
} iot868_decoder_config_t;

// ======================== FANET Decoder Config ========================

typedef struct {
    bool    enabled;
    bool    output_enabled;
    double  center_freq;
} fanet_decoder_config_t;

// ======================== Sarsat Decoder Config ========================

typedef struct {
    bool    enabled;
    bool    output_enabled;
    double  center_freq;
} sarsat_decoder_config_t;

// ======================== USB Dongle Config ========================

typedef struct {
    char    serial[64];         // RTL-SDR serial number
    float   gain;               // gain in dB
    int32_t     ppm;                // oscillator PPM correction
    bool    digital_agc;        // enable digital AGC
    int32_t     direct_sampling;    // 0=off, 1=I, 2=Q
    // Runtime info (filled after device open, not persisted)
    char    tuner_name[32];     // e.g. "R820T", "FC0012"
    char    freq_range[32];     // e.g. "24-1766 MHz"
    int32_t     tuner_type;         // RTLSDR_TUNER_* enum
} dongle_config_t;

// ======================== Global Decoder Configs ========================

typedef struct {
    adsb_decoder_config_t       adsb;
    flarm_decoder_config_t      flarm;
    acars_decoder_config_t      acars;
    vdl2_decoder_config_t       vdl2;
    radiosonde_decoder_config_t radiosonde;
    pocsag_decoder_config_t     pocsag;
    gsm_decoder_config_t        gsm;
    lte_decoder_config_t        lte;
    iot868_decoder_config_t     iot868;
    fanet_decoder_config_t      fanet;
    sarsat_decoder_config_t     sarsat;
} all_decoder_configs_t;

extern all_decoder_configs_t DecoderConfigs;

// ======================== API ========================

// Initialize all decoder configs with defaults
void decoderConfigInit(void);

// Load decoder configs from /etc/dump1090-gg/decoders.json
// Returns true on success
bool decoderConfigLoad(void);

// Save decoder configs to /etc/dump1090-gg/decoders.json
bool decoderConfigSave(void);

// Get decoder type name string
const char *decoderTypeName(decoder_type_t type);

// Get decoder type from string name (returns DECODER_TYPE_COUNT on failure)
decoder_type_t decoderTypeFromName(const char *name);

// Load FLARM keys from file into DecoderConfigs.flarm
bool decoderConfigLoadFlarmKeys(const char *path);

// Save FLARM keys from DecoderConfigs.flarm to file
bool decoderConfigSaveFlarmKeys(const char *path);

// Parse a JSON string and update DecoderConfigs in-place
// Returns true on success
bool decoderConfigParseJson(const char *json);

#define DECODER_CONFIG_PATH "/etc/dump1090-gg/decoders.json"

#endif // DECODER_CONFIG_H
