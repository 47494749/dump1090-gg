#ifndef IOT_HISTORY_H
#define IOT_HISTORY_H

#include <stdint.h>
#include <string>

#define IOT_HISTORY_FILE         "/etc/dump1090-gg/iot_history.dat"
#define IOT_HISTORY_MAGIC        0x49483031  // "IH01"
#define IOT_HISTORY_VERSION      1
#define IOT_HISTORY_MAX_RECORDS  500000  // ~12 MB at 24 bytes per record
#define IOT_HISTORY_SAVE_INTERVAL 50     // save to disk every N records
#define IOT_HISTORY_MAX_SENSORS  256

struct iot_history_record {
    uint64_t ts;            // epoch milliseconds
    uint16_t sensor_id;     // LaCrosse sensor ID
    int16_t  temperature;   // temperature × 10 (e.g. 235 = 23.5°C)
    int16_t  humidity;      // humidity % (0-100, -1 = not available)
    int16_t  rssi;          // RSSI in dBm (e.g. -94)
    uint8_t  battery_ok;    // 1 = OK, 0 = LOW
    uint8_t  source;        // 0 = SDR, 1 = CubeCell
    uint16_t reserved;
};

struct iot_history_header {
    uint32_t magic;
    uint32_t version;
    int32_t  count;         // total records in file
    int32_t  head;          // ring buffer write position
    int32_t  capacity;      // max records (IOT_HISTORY_MAX_RECORDS)
    int32_t  reserved;
};

void iotHistoryInit(void);
void iotHistoryShutdown(void);

// Record a LaCrosse packet (thread-safe)
// source: 0 = SDR decoder, 1 = CubeCell
void iotHistoryRecord(uint16_t sensor_id, float temperature_c,
                      float humidity_pct, float rssi_db,
                      uint8_t battery_ok, uint8_t source);

// JSON for /api/iot-history?sensor=ID&max_points=N
std::string iotHistoryToJSON(uint16_t sensor_id, int32_t max_points);

// JSON list of all sensor IDs with record counts
std::string iotHistorySensorsJSON(void);

#endif // IOT_HISTORY_H
