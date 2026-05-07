// Part of dump1090-gg-light
//
// iot_decode.h: ISM 868 MHz multi-protocol IoT device decoder.
//
// Monitors the 868 MHz ISM band for OOK and FSK signals from common
// IoT devices: weather stations, smart meters (wMBus), thermostats,
// door/window sensors, smoke detectors, etc.
//
// Uses a single RTL-SDR at 868.3 MHz / 2 MSPS to cover the entire
// 868.0–868.6 MHz band simultaneously.
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#ifndef IOT_DECODE_H
#define IOT_DECODE_H

#include <stdint.h>
#include <stdbool.h>

// ======================== Constants ========================

#define IOT_SAMPLE_RATE     2000000   // 2 MSPS — covers ~2 MHz bandwidth
#define IOT_CENTER_FREQ     868300000 // 868.3 MHz center
#define IOT_MAX_DEVICES     128       // max tracked devices

// ======================== Protocol IDs ========================

typedef enum {
    IOT_PROTO_UNKNOWN = 0,
    IOT_PROTO_LACROSSE_TX,       // LaCrosse TX29/TX35 temperature/humidity
    IOT_PROTO_WMBUS_C,           // Wireless M-Bus Mode C (smart meters)
    IOT_PROTO_WMBUS_T,           // Wireless M-Bus Mode T
    IOT_PROTO_HONEYWELL_CM,      // Honeywell CM921/CM927 thermostat
    IOT_PROTO_BRESSER_5IN1,      // Bresser 5-in-1 weather station
    IOT_PROTO_BRESSER_6IN1,      // Bresser 6-in-1 weather station
    IOT_PROTO_OREGON_V3,         // Oregon Scientific v3
    IOT_PROTO_DAVIS_VANTAGE,     // Davis Vantage Vue/Pro2
    IOT_PROTO_FINE_OFFSET_WH,    // Fine Offset WH1080/WH2600
    IOT_PROTO_NETATMO,           // Netatmo weather station
    IOT_PROTO_AURIOL_HG,         // Auriol/Lidl HG0xxxx
    IOT_PROTO_HIDEKI_TS04,       // Hideki TS04 temp/humidity
    IOT_PROTO_TFA_DOSTMANN,      // TFA Dostmann sensors
    IOT_PROTO_ELV_EM1000,        // ELV EM1000 energy meter
    IOT_PROTO_REVOLT_NC5462,     // Revolt NC-5462 power meter
    IOT_PROTO_SMOKE_GS558,       // GS 558 smoke detector (868 MHz)
    IOT_PROTO_ELRO_DB286A,       // Elro DB286A doorbell
    IOT_PROTO_FRIEDLAND_868,     // Friedland Libra+ 868 MHz doorbell
    IOT_PROTO_SOMFY_RTS,         // Somfy RTS blinds/shutters
    IOT_PROTO_NICE_FLOR_S,       // Nice Flor-S gate remote
    IOT_PROTO_EQ3_RADIATOR,      // eQ-3/Homematic radiator thermostat
    IOT_PROTO_DANFOSS_CFR,       // Danfoss CFR thermostat
    IOT_PROTO_MAVERICK_ET73,     // Maverick ET-73x BBQ thermometer
    IOT_PROTO_EFERGY_E2,         // Efergy e2 energy monitor
    IOT_PROTO_CURRENT_COST,      // Current Cost energy monitor
    IOT_PROTO_OWL_CM160,         // OWL CM160 energy monitor
    IOT_PROTO_FSK_GENERIC,       // Unidentified FSK packet
    IOT_PROTO_OOK_GENERIC,       // Unidentified OOK packet
    IOT_PROTO_COUNT
} iot_protocol_t;

// ======================== Modulation type ========================

typedef enum {
    IOT_MOD_OOK = 0,    // On-Off Keying
    IOT_MOD_FSK,         // 2-FSK
    IOT_MOD_GFSK,        // Gaussian FSK
    IOT_MOD_UNKNOWN
} iot_modulation_t;

// ======================== Device info (from decoder) ========================

typedef struct {
    iot_protocol_t  protocol;
    iot_modulation_t modulation;
    uint32_t        device_id;       // protocol-specific device ID
    uint8_t         channel;         // sub-channel (0 if N/A)

    // Measurements (protocol-dependent, NAN if unavailable)
    float           temperature_c;   // degrees Celsius
    float           humidity_pct;    // percent (0-100)
    float           pressure_hpa;    // hectopascal
    float           wind_speed_ms;   // m/s
    float           wind_dir_deg;    // degrees (0-360)
    float           rain_mm;         // total rainfall mm
    float           power_w;         // power in watts
    float           energy_kwh;      // cumulative energy kWh
    float           battery_v;       // battery voltage
    uint8_t         battery_ok;      // 1=ok, 0=low, 255=unknown

    // Signal info
    float           rssi_db;         // estimated signal strength
    float           freq_offset_hz;  // offset from nominal frequency
    double          freq_hz;         // actual detected frequency

    // Raw payload
    uint8_t         payload[64];
    uint8_t         payload_len;

    // Timing
    uint64_t        timestamp_ms;    // detection time (msec since epoch)
} iot_device_msg_t;

// ======================== Decoder state (opaque) ========================

typedef struct iot_decoder_state iot_decoder_state_t;

// ======================== API ========================

// Create decoder state. Returns NULL on failure.
iot_decoder_state_t *iotDecoderCreate(uint32_t sample_rate);

// Process IQ samples (uint8_t interleaved I/Q, length in bytes).
// Calls iotTrackerUpdate() internally when a device message is decoded.
void iotDecoderProcess(iot_decoder_state_t *state, const uint8_t *iq, uint32_t len);

// Destroy decoder state.
void iotDecoderDestroy(iot_decoder_state_t *state);

// Get protocol name string
const char *iotProtocolName(iot_protocol_t proto);

// Get modulation name string
const char *iotModulationName(iot_modulation_t mod);

#endif // IOT_DECODE_H
