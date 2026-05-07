// Part of dump1090-gg-light
//
// iot_tracker.h: ISM 868 MHz IoT device tracking and JSON export.
//
// Maintains a list of discovered IoT devices, updated from decoder callbacks,
// and provides JSON serialization for the /api/iot868 endpoint.

#ifndef IOT_TRACKER_H
#define IOT_TRACKER_H

#include <stdint.h>
#include <stdbool.h>
#include "iot_decode.h"

#define IOT_TRACKER_MAX_DEVICES  128
#define IOT_DEVICE_TIMEOUT       600  // seconds before device is considered stale

// Tracked device entry
typedef struct {
    iot_protocol_t   protocol;
    iot_modulation_t modulation;
    uint32_t         device_id;
    uint8_t          channel;

    // Last known measurements
    float            temperature_c;
    float            humidity_pct;
    float            pressure_hpa;
    float            wind_speed_ms;
    float            wind_dir_deg;
    float            rain_mm;
    float            power_w;
    float            energy_kwh;
    float            battery_v;
    uint8_t          battery_ok;

    // Signal
    float            rssi_db;
    float            freq_offset_hz;
    double           freq_hz;

    // Statistics
    uint64_t         msg_count;      // total messages received
    uint64_t         first_seen_ms;
    uint64_t         last_seen_ms;

    // Last raw payload
    uint8_t          payload[64];
    uint8_t          payload_len;

    bool             active;         // slot in use
} iot_tracked_device_t;

// Initialize the tracker (call once at startup)
void iotTrackerInit(void);

// Update device from decoder message (thread-safe)
void iotTrackerUpdate(const iot_device_msg_t *msg);

// Get number of active (non-stale) devices
int iotTrackerActiveCount(void);

// Generate JSON for /api/iot868 endpoint. Returns malloc'd string (caller frees).
char *iotTrackerToJSON(void);

// Clean up
void iotTrackerDestroy(void);

#endif // IOT_TRACKER_H
