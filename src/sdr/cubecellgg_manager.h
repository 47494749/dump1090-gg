// cubecellgg_manager.h — CubeCell IoT receiver manager for dump1090-gg
//
// Detects, flashes, and communicates with cubecellgg devices (Heltec HTCC-AB01)
// connected via USB serial (CP2102, VID 10C4:EA60).

#ifndef CUBECELLGG_MANAGER_H
#define CUBECELLGG_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

// Device state
typedef enum {
    CCGG_STATE_DISCONNECTED = 0,
    CCGG_STATE_DETECTING,
    CCGG_STATE_FLASHING,
    CCGG_STATE_RUNNING,
    CCGG_STATE_STOPPED,
    CCGG_STATE_ERROR
} ccgg_state_t;

// IoT decoded message from cubecellgg
typedef struct {
    char     proto[32];     // "lacrosse", "honeywell", "fineoffset", etc.
    int32_t  sensor_id;
    float    temperature;
    int32_t  humidity;      // -1 = no sensor
    int32_t  rssi;
    int32_t  battery_low;
    int32_t  new_battery;
    int32_t  channel;       // Honeywell: cmd byte; LaCrosse: 0
    float    wind_speed;    // m/s, -1 = not available
    int32_t  wind_dir;      // degrees, -1 = not available
    float    rain;          // mm, -1 = not available
    uint32_t pkt_count;
    uint64_t timestamp_ms;
} ccgg_iot_msg_t;

// Callback for decoded IoT messages
typedef void (*ccgg_iot_callback_t)(const ccgg_iot_msg_t *msg, void *ctx);

// RSSI scan result
typedef struct {
    uint32_t freq_start;
    uint32_t freq_step;
    int32_t  num_bins;
    int8_t   rssi[256];     // dBm per bin
} ccgg_scan_result_t;

// Callback for scan results
typedef void (*ccgg_scan_callback_t)(const ccgg_scan_result_t *scan, void *ctx);

// Capability descriptor
typedef struct {
    int32_t  id;
    char     name[16];
    char     desc[64];
    uint32_t freq;
} ccgg_cap_t;

#define CCGG_MAX_CAPS 8

// Device info
typedef struct {
    char     tty_path[64];
    char     fw_version[16];
    char     local_version[16];
    uint32_t frequency;
    char     profile[16];       // current active mode
    ccgg_state_t state;
    uint32_t iot_packets;
    uint32_t crc_failures;
    ccgg_cap_t caps[CCGG_MAX_CAPS];
    int32_t  num_caps;
} ccgg_device_info_t;

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the cubecellgg manager. Call once at startup.
// firmware_dir: path to directory containing cubecellgg.cyacd + version.txt
//               (typically /usr/share/dump1090-gg/cubecellgg/)
void ccggInit(const char *firmware_dir);

// Shutdown and release resources
void ccggShutdown(void);

// Set callback for decoded IoT messages
void ccggSetIoTCallback(ccgg_iot_callback_t cb, void *ctx);

// Set callback for RSSI scan results
void ccggSetScanCallback(ccgg_scan_callback_t cb, void *ctx);

// Get current device info (thread-safe snapshot)
void ccggGetDeviceInfo(ccgg_device_info_t *info);

// Send a command to the device (e.g. "CMD:FREQ:869000000")
int ccggSendCommand(const char *cmd);

// Request a single RSSI scan
int ccggRequestScan(void);

// Set operating profile ("lacrosse", "scan")
int ccggSetProfile(const char *profile);

// Set frequency
int ccggSetFrequency(uint32_t freq_hz);

// Check if device is connected and running
bool ccggIsRunning(void);

// Pause/resume data processing (Stop/Run buttons)
void ccggSetPaused(bool paused);

// Waterfall control
int ccggWaterfallStart(uint32_t start_hz, uint32_t end_hz, uint32_t step_hz);
int ccggWaterfallStop(void);

// Get latest waterfall line as JSON string (caller must free)
// Returns NULL if no data. Format: {"s":freq_start,"t":step,"d":[-110,-105,...]}
char *ccggWaterfallGetLine(void);

#ifdef __cplusplus
}
#endif

#endif // CUBECELLGG_MANAGER_H
