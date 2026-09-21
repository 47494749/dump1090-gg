// Part of dump1090-gg-light
//
// iot_decode.c: ISM 868 MHz multi-protocol IoT device decoder.
//
// This decoder processes raw IQ samples at 2 MSPS centered on 868.3 MHz
// and detects OOK and FSK signals from common IoT devices.
//
// Architecture:
//   1. Envelope detection (AM demod) for OOK signals
//   2. FM demodulation for FSK signals
//   3. Pulse/gap timing analysis to identify protocols
//   4. Protocol-specific bit decoding and CRC verification
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#include <cstdlib>
#include <stdint.h>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <ctime>
using std::isnan;

#include "iot_decode.h"
#include "msg_queue.h"

// ======================== Internal constants ========================

#define IOT_BLOCK_SIZE       16384    // samples per processing block
#define IOT_OOK_THRESHOLD    40       // AM envelope threshold (0-255 scale)
#define IOT_MIN_PULSE_US     50       // minimum valid pulse width (us)
#define IOT_MAX_PULSE_US     10000    // maximum valid pulse width (us)
#define IOT_MAX_PULSES       512      // max pulses in a burst
#define IOT_GAP_TIMEOUT_US   50000    // gap > this = end of message (50ms)
#define IOT_FSK_DEVIATION    50000    // typical FSK deviation (Hz)

// Pulse timing constants (in microseconds at 2 MSPS → 1 sample = 0.5 µs)
#define US_TO_SAMPLES(us) ((us) * 2)
#define SAMPLES_TO_US(s)  ((s) / 2)  // 2 MSPS: 2 samples = 1 µs

// ======================== Pulse representation ========================

typedef struct {
    uint16_t pulse_us;   // high duration in µs
    uint16_t gap_us;     // low duration in µs
} pulse_t;

// ======================== Decoder state ========================

struct iot_decoder_state {
    uint32_t sample_rate;
    msg_queue_t out_queue;        // output queue for decoded messages

    // OOK envelope detector state
    uint8_t  ook_level;           // current smoothed AM level
    bool     ook_high;            // currently in pulse?
    uint32_t ook_count;           // sample counter within current pulse/gap
    uint32_t ook_pulse_samples;   // samples in current pulse

    // Pulse buffer
    pulse_t  pulses[IOT_MAX_PULSES];
    int32_t      pulse_count;

    // FSK demodulator state (2:1 decimated to 1 MSPS effective)
    int8_t   fsk_prev_i;
    int8_t   fsk_prev_q;
    int32_t  fsk_dec_i;       // decimation accumulator I
    int32_t  fsk_dec_q;       // decimation accumulator Q
    int32_t  fsk_dec_phase;   // 0 or 1 — decimation phase
    int32_t  fsk_fm_dc[8];
    int32_t  fsk_accum[8];
    int32_t  fsk_count[8];
    uint8_t  fsk_ring[8][4096];
    int32_t  fsk_ring_len[8];

    // Statistics
    uint64_t samples_processed;
    uint64_t packets_decoded;
};

// ======================== Protocol name table ========================

static const char *protocol_names[IOT_PROTO_COUNT] = {
    [IOT_PROTO_UNKNOWN]       = "Unknown",
    [IOT_PROTO_LACROSSE_TX]   = "LaCrosse TX",
    [IOT_PROTO_WMBUS_C]       = "wMBus Mode C",
    [IOT_PROTO_WMBUS_T]       = "wMBus Mode T",
    [IOT_PROTO_HONEYWELL_CM]  = "Honeywell CM9xx",
    [IOT_PROTO_BRESSER_5IN1]  = "Bresser 5-in-1",
    [IOT_PROTO_BRESSER_6IN1]  = "Bresser 6-in-1",
    [IOT_PROTO_OREGON_V3]     = "Oregon Sci v3",
    [IOT_PROTO_DAVIS_VANTAGE] = "Davis Vantage",
    [IOT_PROTO_FINE_OFFSET_WH]= "Fine Offset WH",
    [IOT_PROTO_NETATMO]       = "Netatmo",
    [IOT_PROTO_AURIOL_HG]     = "Auriol HG",
    [IOT_PROTO_HIDEKI_TS04]   = "Hideki TS04",
    [IOT_PROTO_TFA_DOSTMANN]  = "TFA Dostmann",
    [IOT_PROTO_ELV_EM1000]    = "ELV EM1000",
    [IOT_PROTO_REVOLT_NC5462] = "Revolt NC-5462",
    [IOT_PROTO_SMOKE_GS558]   = "Smoke GS558",
    [IOT_PROTO_ELRO_DB286A]   = "Elro DB286A",
    [IOT_PROTO_FRIEDLAND_868] = "Friedland 868",
    [IOT_PROTO_SOMFY_RTS]     = "Somfy RTS",
    [IOT_PROTO_NICE_FLOR_S]   = "Nice Flor-S",
    [IOT_PROTO_EQ3_RADIATOR]  = "eQ-3 Radiator",
    [IOT_PROTO_DANFOSS_CFR]   = "Danfoss CFR",
    [IOT_PROTO_MAVERICK_ET73] = "Maverick ET-73",
    [IOT_PROTO_EFERGY_E2]     = "Efergy e2",
    [IOT_PROTO_CURRENT_COST]  = "Current Cost",
    [IOT_PROTO_OWL_CM160]     = "OWL CM160",
    [IOT_PROTO_FSK_GENERIC]   = "FSK (unknown)",
    [IOT_PROTO_OOK_GENERIC]   = "OOK (unknown)",
};

static const char *modulation_names[] = {
    [IOT_MOD_OOK]     = "OOK",
    [IOT_MOD_FSK]     = "FSK",
    [IOT_MOD_GFSK]    = "GFSK",
    [IOT_MOD_UNKNOWN] = "?",
};

// ======================== Utility ========================

const char *iotProtocolName(iot_protocol_t proto)
{
    if (proto < IOT_PROTO_COUNT)
        return protocol_names[proto];
    return "?";
}

const char *iotModulationName(iot_modulation_t mod)
{
    if (mod <= IOT_MOD_UNKNOWN)
        return modulation_names[mod];
    return "?";
}

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

static uint8_t crc8(const uint8_t *data, int32_t len, uint8_t poly, uint8_t init)
{
    uint8_t crc = init;
    for (int32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int32_t b = 0; b < 8; b++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// CRC-16/EN-13757 for wMBus (poly 0x3D65, init 0x0000, final XOR 0xFFFF)
static uint16_t crc16_wmbus(const uint8_t *data, int32_t len)
{
    uint16_t crc = 0x0000;
    for (int32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int32_t b = 0; b < 8; b++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x3D65;
            else
                crc <<= 1;
        }
    }
    return crc ^ 0xFFFF;
}

// ======================== OOK Protocol decoders ========================

// LaCrosse TX29/TX35: pulse ~500µs, gap ~1000µs for '1', ~2000µs for '0'
// Preamble: 10 x '0' bits, then 0x0A sync nibble
static bool decode_lacrosse_tx(const pulse_t *pulses, int32_t count, iot_device_msg_t *msg)
{
    if (count < 40) return false;

    // Find preamble: pulses ~500µs, gaps alternating ~1000/2000µs
    int32_t start = -1;
    for (int32_t i = 0; i <= count - 40; i++) {
        if (pulses[i].pulse_us >= 400 && pulses[i].pulse_us <= 600 &&
            pulses[i].gap_us >= 1800 && pulses[i].gap_us <= 2200) {
            // Possible '0' preamble bit
            int32_t good = 0;
            for (int32_t j = 0; j < 4 && (i+j) < count; j++) {
                if (pulses[i+j].pulse_us >= 400 && pulses[i+j].pulse_us <= 600 &&
                    pulses[i+j].gap_us >= 1800 && pulses[i+j].gap_us <= 2200)
                    good++;
            }
            if (good >= 3) { start = i + good; break; }
        }
    }
    if (start < 0 || start + 32 > count) return false;

    // Decode bits: short gap (~1000µs) = 1, long gap (~2000µs) = 0
    uint8_t bytes[5] = {0};
    int32_t bit_idx = 0;
    for (int32_t i = start; i < count && bit_idx < 40; i++) {
        if (pulses[i].pulse_us < 300 || pulses[i].pulse_us > 700) break;
        int32_t bit = (pulses[i].gap_us < 1500) ? 1 : 0;
        bytes[bit_idx / 8] |= (bit << (7 - (bit_idx % 8)));
        bit_idx++;
    }
    if (bit_idx < 32) return false;

    // Verify: first nibble should be 0x0A (sync)
    uint8_t sync_nibble = (bytes[0] >> 4) & 0x0F;
    if (sync_nibble != 0x0A) return false;

    // Checksum: XOR of nibbles should be 0
    uint8_t chk = 0;
    for (int32_t i = 0; i < bit_idx / 4; i++) {
        uint8_t nibble = (bytes[i/2] >> ((i%2)?0:4)) & 0x0F;
        chk ^= nibble;
    }
    // Simple validation: type nibble should be temperature or humidity
    uint8_t type_nibble = bytes[0] & 0x0F;
    if (type_nibble != 0x00 && type_nibble != 0x0E) return false;

    msg->protocol = IOT_PROTO_LACROSSE_TX;
    msg->modulation = IOT_MOD_OOK;
    msg->device_id = ((bytes[1] & 0xF0) >> 4) | ((bytes[1] & 0x0F) << 4);
    msg->channel = 0;
    msg->battery_ok = 255;  // unknown

    if (type_nibble == 0x00) {
        // Temperature: raw = nibbles 3-5, temp = raw/10 - 40
        int32_t raw = ((bytes[2] & 0x0F) * 100) + ((bytes[3] >> 4) * 10) + (bytes[3] & 0x0F);
        msg->temperature_c = raw / 10.0f - 40.0f;
        msg->humidity_pct = NAN;
    } else {
        // Humidity
        int32_t raw = ((bytes[2] & 0x0F) * 100) + ((bytes[3] >> 4) * 10) + (bytes[3] & 0x0F);
        msg->humidity_pct = (float)raw / 10.0f;
        msg->temperature_c = NAN;
    }
    msg->pressure_hpa = NAN;
    msg->wind_speed_ms = NAN;
    msg->wind_dir_deg = NAN;
    msg->rain_mm = NAN;
    msg->power_w = NAN;
    msg->energy_kwh = NAN;
    msg->battery_v = NAN;
    msg->freq_hz = 868.9e6;

    memcpy(msg->payload, bytes, bit_idx / 8);
    msg->payload_len = bit_idx / 8;
    msg->timestamp_ms = now_ms();
    return true;
}

// Bresser 5-in-1: OOK, 17 byte message, bit rate ~8 kbps
// Pulse: ~125µs high, gap: ~125µs = '1', ~375µs = '0'
static bool decode_bresser_5in1(const pulse_t *pulses, int32_t count, iot_device_msg_t *msg)
{
    if (count < 136) return false;  // 17 bytes * 8 bits

    // Look for consistent short pulses (~125µs)
    int32_t start = -1;
    for (int32_t i = 0; i <= count - 136; i++) {
        if (pulses[i].pulse_us >= 80 && pulses[i].pulse_us <= 180) {
            int32_t good = 0;
            for (int32_t j = 0; j < 8 && (i+j) < count; j++) {
                if (pulses[i+j].pulse_us >= 80 && pulses[i+j].pulse_us <= 180)
                    good++;
            }
            if (good >= 7) { start = i; break; }
        }
    }
    if (start < 0 || start + 136 > count) return false;

    // Decode: short gap = 1, long gap = 0
    uint8_t bytes[17] = {0};
    for (int32_t i = 0; i < 136 && (start + i) < count; i++) {
        int32_t bit = (pulses[start + i].gap_us < 250) ? 1 : 0;
        bytes[i / 8] |= (bit << (7 - (i % 8)));
    }

    // CRC check (byte 16 is CRC-8 poly 0x31)
    uint8_t computed_crc = crc8(bytes, 16, 0x31, 0x00);
    if (computed_crc != bytes[16]) return false;

    msg->protocol = IOT_PROTO_BRESSER_5IN1;
    msg->modulation = IOT_MOD_OOK;
    msg->device_id = (bytes[1] << 8) | bytes[2];
    msg->channel = bytes[3] & 0x07;
    msg->battery_ok = (bytes[3] & 0x80) ? 0 : 1;

    // Temperature: bytes 4-5, signed, /10
    int16_t raw_temp = (int16_t)((bytes[4] << 8) | bytes[5]);
    msg->temperature_c = raw_temp / 10.0f;
    msg->humidity_pct = (float)bytes[6];
    // Wind: byte 7 = speed *0.1 m/s, byte 8 = gust, byte 9 = direction/22.5
    msg->wind_speed_ms = bytes[7] * 0.1f;
    msg->wind_dir_deg = bytes[9] * 22.5f;
    msg->rain_mm = ((bytes[10] << 8) | bytes[11]) * 0.1f;
    msg->pressure_hpa = NAN;
    msg->power_w = NAN;
    msg->energy_kwh = NAN;
    msg->battery_v = NAN;
    msg->freq_hz = 868.3e6;

    memcpy(msg->payload, bytes, 17);
    msg->payload_len = 17;
    msg->timestamp_ms = now_ms();
    return true;
}

// (decode_ook_generic removed — too many false positives from noise)

// ======================== FSK Protocol decoders ========================

// LaCrosse TX29/TX35 FSK (868 MHz "IT+" instant transmission protocol)
// Based on rtl_433 lacrosse_tx35.c by Sébastien Blanc.
//
// Modulation: FSK PCM, ~17.2 kbps (55 µs per bit for TX29, 105 µs for TX35)
// Preamble: 1010... (4+ bits), Sync: 0x2DD4, Model nibble: 0x9
// After model: 5 bytes of data + CRC-8 (poly 0x31, init 0x00)
//
// Data layout (5 bytes, extracted after the model nibble 0x9):
//   b[0]: [id5:2]       — upper 4 bits of 6-bit sensor ID
//   b[1]: [id1:0][new_batt][0][temp_digit1] — lower 2 bits of ID, battery, BCD temp
//   b[2]: [temp_digit2][temp_digit3]        — BCD temperature (tens, units, tenths)
//   b[3]: [batt_low][humidity6:0]           — battery low, humidity %
//   b[4]: CRC-8 (poly 0x31, init 0x00) of b[0..3]
//
// Temperature: digit1*10 + digit2 + digit3*0.1 - 40.0 (BCD, in Celsius)
// Humidity: 0x6A = no sensor, 0x7D = probe channel, otherwise %RH
// LaCrosse TX29/TX35 FSK decoder — works directly on FM discriminator output.
// Uses correlator-based sync detection instead of bit slicer for robustness.
// sync_word = 0xA2DD49 (24 bits) as ±1 pattern
static const int8_t lacrosse_sync_pattern[24] = {
    +1,-1,+1,-1, // preamble 1010
    +1,-1,+1,-1, -1,+1,-1,+1, +1,+1,-1,+1, -1,+1,-1,-1, // sync 0x2DD4
    +1,-1,-1,+1  // model 0x9
};

static bool decode_lacrosse_fm(const int16_t *fm, int32_t fm_len,
                               int32_t samples_per_bit, iot_device_msg_t *msg)
{
    int32_t total_bits_needed = 24 + 44;  // sync(24) + model_skip(4) + data(40)
    int32_t total_samples = total_bits_needed * samples_per_bit;
    if (fm_len < total_samples + samples_per_bit) return false;

    // Correlate sync pattern against FM output, trying both polarities.
    // The FM discriminator polarity depends on the signal's frequency offset
    // relative to the tuner center. Try both +1/-1 and -1/+1 mappings.
    int32_t fm_mean = 0;
    int32_t best_pos = -1;
    int64_t best_corr = 0;
    int32_t best_polarity = 1;

    for (int32_t pos = 0; pos <= fm_len - total_samples; pos++) {
        int64_t corr = 0;
        for (int32_t b = 0; b < 24; b++) {
            int32_t sample_center = pos + b * samples_per_bit + samples_per_bit / 2;
            if (sample_center >= fm_len) break;
            int32_t half = samples_per_bit / 4;
            if (half < 1) half = 1;
            int64_t bit_val = 0;
            for (int32_t k = -half; k <= half; k++) {
                int32_t idx = sample_center + k;
                if (idx >= 0 && idx < fm_len) bit_val += fm[idx];
            }
            corr += bit_val * lacrosse_sync_pattern[b];
        }
        // Check both polarities: positive and negative correlation
        int64_t abs_corr = (corr > 0) ? corr : -corr;
        if (abs_corr > best_corr) {
            best_corr = abs_corr;
            best_pos = pos;
            best_polarity = (corr > 0) ? 1 : -1;
        }
    }

    if (best_pos < 0) return false;

    // Threshold: correlator must be significantly above noise
    int64_t norm = 55000;
    if (best_corr < norm) return false;

    // Compute local DC from the sync region for payload extraction
    {
        int32_t sync_end = best_pos + 24 * samples_per_bit;
        int64_t local_sum = 0;
        int32_t local_cnt = 0;
        for (int32_t k = best_pos; k < sync_end && k < fm_len; k++) {
            local_sum += fm[k]; local_cnt++;
        }
        if (local_cnt > 0) fm_mean = (int32_t)(local_sum / local_cnt);
    }

    // Extract bits using sub-sample clock calibration from the sync word.
    //
    // The correlator gives best_pos at sample granularity. To find the
    // exact sub-sample phase, we test multiple offsets within one bit
    // period and pick the one that best matches the known sync bits.
    // Then use that calibrated phase for the payload extraction with
    // zero-crossing clock recovery.
    int32_t best_phase = 0;
    int32_t best_sync_score = -1;
    int32_t quarter_bit = samples_per_bit / 4;
    if (quarter_bit < 1) quarter_bit = 1;

    for (int32_t phase = -quarter_bit; phase <= quarter_bit; phase++) {
        int32_t score = 0;
        int32_t cpos = best_pos + phase;
        for (int32_t b = 0; b < 24; b++) {
            if (cpos + samples_per_bit > fm_len) break;
            int64_t bit_sum = 0;
            for (int32_t k = 0; k < samples_per_bit; k++)
                bit_sum += (fm[cpos + k] - fm_mean);
            // Apply polarity: if signal is inverted, flip the comparison
            int32_t got = ((bit_sum * best_polarity) > 0) ? 1 : 0;
            int32_t expect = (lacrosse_sync_pattern[b] > 0) ? 1 : 0;
            if (got == expect) score++;
            cpos += samples_per_bit;
        }
        if (score > best_sync_score) {
            best_sync_score = score;
            best_phase = phase;
        }
    }

    if (best_sync_score < 18) return false;

    // Now extract data bits (skip preamble+sync = 20 bits) with the
    // calibrated phase and zero-crossing clock recovery.
    int32_t data_start = best_pos + best_phase + 20 * samples_per_bit;
    uint8_t raw_bits[64];
    int32_t clock_pos = data_start;

    for (int32_t b = 0; b < 48; b++) {
        if (clock_pos + samples_per_bit > fm_len) break;

        int64_t bit_sum = 0;
        for (int32_t k = 0; k < samples_per_bit; k++)
            bit_sum += (fm[clock_pos + k] - fm_mean);
        raw_bits[b] = ((bit_sum * best_polarity) > 0) ? 1 : 0;

        int32_t next_boundary = clock_pos + samples_per_bit;

        // Clock recovery: snap to nearest zero-crossing
        int32_t best_zc = next_boundary;
        int32_t best_zc_dist = quarter_bit + 1;
        for (int32_t k = -quarter_bit; k <= quarter_bit; k++) {
            int32_t p = next_boundary + k;
            if (p > 0 && p < fm_len - 1) {
                int32_t v0 = fm[p - 1] - fm_mean;
                int32_t v1 = fm[p] - fm_mean;
                if ((v0 > 0) != (v1 > 0)) {
                    if (abs(k) < best_zc_dist) {
                        best_zc_dist = abs(k);
                        best_zc = p;
                    }
                }
            }
        }
        clock_pos = (best_zc_dist <= quarter_bit) ? best_zc : next_boundary;
    }

    // Skip model nibble (4 bits), extract 5 data bytes (40 bits)
    uint8_t b5[5] = {0};
    for (int32_t i = 0; i < 40; i++) {
        int32_t bit_pos = 4 + i;  // skip model nibble
        if (bit_pos >= 48) return false;
        b5[i / 8] |= (raw_bits[bit_pos] & 1) << (7 - (i % 8));
    }

    // CRC-8 check (poly 0x31, init 0x00) over first 4 bytes
    uint8_t crc = crc8(b5, 4, 0x31, 0x00);
    if (crc != b5[4]) return false;

    // Parse fields
    int32_t sensor_id   = ((b5[0] & 0x0F) << 2) | (b5[1] >> 6);
    float   temp_c      = 10.0f * (b5[1] & 0x0F) + 1.0f * ((b5[2] >> 4) & 0x0F) + 0.1f * (b5[2] & 0x0F) - 40.0f;
    int32_t battery_low = b5[3] >> 7;
    int32_t humidity    = b5[3] & 0x7F;

    // Plausibility check: reject values outside physical range.
    // CRC-8 has only 256 values — with frequent correlator triggers,
    // ~0.4% of random data passes CRC by chance.
    if (temp_c < -30.0f || temp_c > 60.0f) return false;
    if (humidity != 0x6A && humidity != 0x7D && humidity > 100) return false;

    // BCD digit validation: each nibble of temperature must be 0-9
    if ((b5[1] & 0x0F) > 9) return false;
    if (((b5[2] >> 4) & 0x0F) > 9) return false;
    if ((b5[2] & 0x0F) > 9) return false;

    msg->protocol = IOT_PROTO_LACROSSE_TX;
    msg->modulation = IOT_MOD_FSK;
    msg->device_id = sensor_id;
    msg->channel = 0;
    msg->battery_ok = battery_low ? 0 : 1;
    msg->temperature_c = temp_c;

    if (humidity == 0x6A || humidity == 0x7D)
        msg->humidity_pct = NAN;
    else
        msg->humidity_pct = (float)humidity;

    msg->pressure_hpa = NAN;
    msg->wind_speed_ms = NAN;
    msg->wind_dir_deg = NAN;
    msg->rain_mm = NAN;
    msg->power_w = NAN;
    msg->energy_kwh = NAN;
    msg->battery_v = NAN;
    msg->freq_hz = 868.3e6;

    memcpy(msg->payload, b5, 5);
    msg->payload_len = 5;
    msg->timestamp_ms = now_ms();
    return true;
}

// Legacy bit-based decoder (for other FSK protocols that still use the ring)
static bool decode_lacrosse_fsk(const uint8_t *bits, int32_t bit_count, iot_device_msg_t *msg)
{
    (void)bits; (void)bit_count; (void)msg;
    return false;  // replaced by decode_lacrosse_fm correlator
}

// Fine Offset WH1080/WH3080 FSK variant (868.3 MHz)
// Protocol from rtl_433 fineoffset.c
// FSK PCM, 58µs/bit (~17.2 kbps), sync 0xAA2DD4
// Data: 11 bytes (weather) or 8 bytes (UV/light)
// CRC-8 poly 0x31, init 0xFF
static bool decode_fineoffset_fsk(const uint8_t *bits, int32_t bit_count, iot_device_msg_t *msg)
{
    if (bit_count < 100) return false;

    // Search for preamble 0xAA + sync 0x2DD4 (24 bits)
    int32_t start = -1;
    for (int32_t i = 0; i <= bit_count - 100; i++) {
        uint32_t word = 0;
        for (int32_t b = 0; b < 24; b++)
            word = (word << 1) | (bits[i + b] & 1);
        if (word == 0xAA2DD4) {
            start = i + 24;
            break;
        }
    }
    if (start < 0) return false;

    // Try 88-bit weather packet (11 bytes)
    if (start + 88 <= bit_count) {
        uint8_t b[11] = {};
        for (int32_t i = 0; i < 88; i++)
            b[i/8] |= (bits[start + i] & 1) << (7 - (i % 8));

        // CRC-8 check
        uint8_t crc = 0xFF;
        for (int32_t i = 0; i < 11; i++) {
            crc ^= b[i];
            for (int32_t j = 0; j < 8; j++)
                crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : crc << 1;
        }
        if (crc == 0) {
            uint8_t type_nibble = b[0] >> 4;
            if (type_nibble == 0x0A) {  // Weather data
                msg->protocol = IOT_PROTO_FINE_OFFSET_WH;
                msg->modulation = IOT_MOD_FSK;
                msg->device_id = ((b[1] & 0x0F) << 4) | (b[2] >> 4);
                // FSK temperature: sign-magnitude, bit 11 = sign
                int32_t temp_raw = ((b[2] & 0x0F) << 8) | b[3];
                if (temp_raw & 0x800) temp_raw = -(temp_raw & 0x7FF);
                msg->temperature_c = temp_raw * 0.1f;
                msg->humidity_pct = (float)b[4];
                msg->wind_speed_ms = b[5] * 0.34f;
                msg->wind_dir_deg = (float)(uint16_t[]){0,23,45,68,90,113,135,158,180,203,225,248,270,293,315,338}[b[9] & 0x0F];
                msg->rain_mm = (((b[7] & 0x0F) << 8) | b[8]) * 0.3f;
                msg->battery_ok = ((b[9] >> 4) == 1) ? 0 : 1;
                msg->channel = 0;
                msg->pressure_hpa = NAN;
                msg->power_w = NAN;
                msg->energy_kwh = NAN;
                msg->battery_v = NAN;
                msg->rssi_db = NAN;
                msg->freq_offset_hz = NAN;
                msg->freq_hz = 868.3e6;
                memcpy(msg->payload, b, 11);
                msg->payload_len = 11;
                msg->timestamp_ms = now_ms();
                return true;
            }
        }
    }
    return false;
}

// Bresser 6-in-1 Weather Center FSK (868 MHz)
// Protocol from rtl_433 bresser_6in1.c
// FSK PCM, 124µs/bit (~8065 bps), sync 0xAA2DD4
// Data: 18 bytes, LFSR-16 digest + additive checksum
static bool decode_bresser_6in1(const uint8_t *bits, int32_t bit_count, iot_device_msg_t *msg)
{
    if (bit_count < 168) return false;  // need 24 sync + 144 data

    // Search for sync 0xAA2DD4
    int32_t start = -1;
    for (int32_t i = 0; i <= bit_count - 168; i++) {
        uint32_t word = 0;
        for (int32_t b = 0; b < 24; b++)
            word = (word << 1) | (bits[i + b] & 1);
        if (word == 0xAA2DD4) {
            start = i + 24;
            break;
        }
    }
    if (start < 0 || start + 144 > bit_count) return false;

    uint8_t m[18] = {};
    for (int32_t i = 0; i < 144; i++)
        m[i/8] |= (bits[start + i] & 1) << (7 - (i % 8));

    // Additive checksum: sum of bytes 2..17 must be 0xFF
    uint8_t sum = 0;
    for (int32_t i = 2; i < 18; i++) sum += m[i];
    if (sum != 0xFF) return false;

    // Parse sensor type
    uint8_t stype = m[6] >> 4;
    if (stype != 1 && stype != 2) return false;  // 1=weather, 2=thermo/hygro

    msg->protocol = IOT_PROTO_BRESSER_6IN1;
    msg->modulation = IOT_MOD_FSK;
    msg->device_id = ((uint32_t)m[2] << 24) | ((uint32_t)m[3] << 16) | ((uint32_t)m[4] << 8) | m[5];
    msg->channel = m[6] & 0x07;

    // Temperature: BCD in bytes 12-13, sign bit at m[13] bit 3
    int32_t temp_bcd = (m[12] >> 4) * 100 + (m[12] & 0x0F) * 10 + (m[13] >> 4);
    float temp_c = temp_bcd * 0.1f;
    if (m[13] & 0x08) temp_c = -temp_c;
    msg->temperature_c = temp_c;

    // Humidity: BCD in byte 14
    msg->humidity_pct = (float)((m[14] >> 4) * 10 + (m[14] & 0x0F));

    // Wind speed: inverted BCD in bytes 7-9
    if (stype == 1) {
        uint8_t w8 = m[8] ^ 0xFF, w9 = m[9] ^ 0xFF;
        msg->wind_speed_ms = ((w9 >> 4) * 100 + (w9 & 0x0F) * 10 + (w8 & 0x0F)) * 0.1f;
        msg->wind_dir_deg = (float)((m[10] >> 4) * 100 + (m[10] & 0x0F) * 10 + (m[11] >> 4));
        // Rain: bytes 12-14 XOR 0xFF when rain flag set
        if (m[16] & 0x01) {
            uint8_t r12 = m[12] ^ 0xFF, r13 = m[13] ^ 0xFF, r14 = m[14] ^ 0xFF;
            int32_t rain_bcd = (r12 >> 4) * 100000 + (r12 & 0x0F) * 10000 +
                               (r13 >> 4) * 1000 + (r13 & 0x0F) * 100 +
                               (r14 >> 4) * 10 + (r14 & 0x0F);
            msg->rain_mm = rain_bcd * 0.1f;
        } else {
            msg->rain_mm = NAN;
        }
    } else {
        msg->wind_speed_ms = NAN;
        msg->wind_dir_deg = NAN;
        msg->rain_mm = NAN;
    }

    msg->battery_ok = (m[13] & 0x02) ? 1 : 0;
    msg->pressure_hpa = NAN;
    msg->power_w = NAN;
    msg->energy_kwh = NAN;
    msg->battery_v = NAN;
    msg->rssi_db = NAN;
    msg->freq_offset_hz = NAN;
    msg->freq_hz = 868.3e6;
    memcpy(msg->payload, m, 18);
    msg->payload_len = 18;
    msg->timestamp_ms = now_ms();
    return true;
}

// wMBus Mode C/T: GFSK ±50 kHz, 100 kbps (Mode C) or ~32.768 kbps (Mode T)
// Preamble: Mode C = 0101...0101 + 0x543D, Mode T = 1010...1010 + 0x3965543D
// We require the FULL 16-bit sync 0x543D (Mode C) to avoid false positives.
static bool decode_wmbus(const uint8_t *bits, int32_t bit_count, iot_device_msg_t *msg)
{
    if (bit_count < 120) return false;  // need preamble + header + CRC

    // Search for wMBus Mode C sync: 0101 0100 0011 1101 = 0x543D
    // Also require at least 8 bits of alternating preamble before sync
    int32_t start = -1;
    iot_protocol_t proto = IOT_PROTO_WMBUS_C;
    for (int32_t i = 8; i <= bit_count - 120; i++) {
        uint16_t word = 0;
        for (int32_t b = 0; b < 16; b++)
            word = (word << 1) | (bits[i + b] & 1);
        if (word == 0x543D) {
            // Verify preamble: at least 6 of 8 preceding bits should alternate (0101...)
            int32_t alt_count = 0;
            for (int32_t p = 0; p < 8 && (i - 8 + p) >= 0; p++) {
                int32_t expected = (p % 2 == 0) ? 0 : 1;
                if ((bits[i - 8 + p] & 1) == expected) alt_count++;
            }
            if (alt_count >= 6) {
                start = i + 16;
                break;
            }
        }
        // Mode T: require 32-bit sync 0x3965543D
        if (i + 32 <= bit_count) {
            uint32_t word32 = 0;
            for (int32_t b = 0; b < 32; b++)
                word32 = (word32 << 1) | (bits[i + b] & 1);
            if (word32 == 0x3965543D) {
                start = i + 32;
                proto = IOT_PROTO_WMBUS_T;
                break;
            }
        }
    }
    if (start < 0) return false;
    msg->protocol = proto;

    // Decode bytes after sync
    uint8_t bytes[64] = {0};
    int32_t byte_count = 0;
    for (int32_t i = start; i + 8 <= bit_count && byte_count < 64; i += 8) {
        uint8_t byte = 0;
        for (int32_t b = 0; b < 8; b++)
            byte = (byte << 1) | (bits[i + b] & 1);
        bytes[byte_count++] = byte;
    }
    if (byte_count < 10) return false;  // too short

    // wMBus header: L-field (length), C-field, M-field(2), A-field(6)
    uint8_t l_field = bytes[0];
    if (l_field < 9 || l_field > 60) return false;
    // uint8_t c_field = bytes[1];
    uint16_t m_field = (bytes[3] << 8) | bytes[2];  // manufacturer (little-endian)
    uint32_t a_field = ((uint32_t)bytes[7] << 24) | ((uint32_t)bytes[6] << 16) | ((uint32_t)bytes[5] << 8) | bytes[4];

    // CRC-16 check of first block (first 10 bytes, CRC at 10-11) — MANDATORY
    if (byte_count < 12) return false;
    uint16_t crc_calc = crc16_wmbus(bytes, 10);
    uint16_t crc_recv = (bytes[10] << 8) | bytes[11];
    if (crc_calc != crc_recv) return false;  // strict CRC — reject noise

    msg->modulation = IOT_MOD_GFSK;
    msg->device_id = a_field;
    msg->channel = m_field & 0xFF;  // use manufacturer LSB as "channel" for display
    msg->temperature_c = NAN;
    msg->humidity_pct = NAN;
    msg->pressure_hpa = NAN;
    msg->wind_speed_ms = NAN;
    msg->wind_dir_deg = NAN;
    msg->rain_mm = NAN;
    msg->power_w = NAN;
    msg->energy_kwh = NAN;
    msg->battery_v = NAN;
    msg->battery_ok = 255;
    msg->freq_hz = 868.95e6;

    int32_t copy_len = byte_count < 64 ? byte_count : 64;
    memcpy(msg->payload, bytes, copy_len);
    msg->payload_len = copy_len;
    msg->timestamp_ms = now_ms();
    return true;
}

// Honeywell CM921/CM927 (Evohome/RAMSES II): FSK, ~38.4 kbps, Manchester encoded
static bool decode_honeywell_cm(const uint8_t *bits, int32_t bit_count, iot_device_msg_t *msg)
{
    if (bit_count < 120) return false;

    // Sync pattern: 0xFF 0x00 0x33 (preamble 1010... then sync word)
    int32_t start = -1;
    for (int32_t i = 0; i <= bit_count - 120; i += 8) {
        uint8_t b0 = 0, b1 = 0, b2 = 0;
        for (int32_t b = 0; b < 8 && (i+b) < bit_count; b++)
            b0 = (b0 << 1) | (bits[i+b] & 1);
        for (int32_t b = 0; b < 8 && (i+8+b) < bit_count; b++)
            b1 = (b1 << 1) | (bits[i+8+b] & 1);
        for (int32_t b = 0; b < 8 && (i+16+b) < bit_count; b++)
            b2 = (b2 << 1) | (bits[i+16+b] & 1);
        if (b0 == 0xFF && b1 == 0x00 && b2 == 0x33) {
            start = i + 24;
            break;
        }
    }
    if (start < 0 || start + 96 > bit_count) return false;

    // Manchester decode
    uint8_t bytes[32] = {0};
    int32_t byte_count = 0;
    int32_t bit_idx = 0;
    for (int32_t i = start; i + 1 < bit_count && byte_count < 32; i += 2) {
        int32_t bit = (bits[i] > bits[i+1]) ? 1 : 0;
        bytes[byte_count] |= (bit << (7 - bit_idx));
        bit_idx++;
        if (bit_idx == 8) { bit_idx = 0; byte_count++; }
    }
    if (byte_count < 6) return false;

    // Device ID from first 3 bytes, command byte as channel (original working parse)
    uint32_t dev_id = ((uint32_t)bytes[0] << 16) | ((uint32_t)bytes[1] << 8) | bytes[2];

    msg->protocol = IOT_PROTO_HONEYWELL_CM;
    msg->modulation = IOT_MOD_FSK;
    msg->device_id = dev_id;
    msg->channel = bytes[3];
    msg->temperature_c = NAN;
    msg->humidity_pct = NAN;
    msg->pressure_hpa = NAN;
    msg->wind_speed_ms = NAN;
    msg->wind_dir_deg = NAN;
    msg->rain_mm = NAN;
    msg->power_w = NAN;
    msg->energy_kwh = NAN;
    msg->battery_v = NAN;
    msg->battery_ok = 255;
    msg->freq_hz = 868.3e6;

    // Try to find temperature in the payload using known Evohome patterns:
    // Pattern 1: command 0x30C9 anywhere → next 3 bytes = [zone][temp_hi][temp_lo]
    // Pattern 2: two consecutive bytes that look like temp/100 in range -20..60°C
    for (int32_t i = 3; i + 4 < byte_count; i++) {
        if (bytes[i] == 0x30 && bytes[i+1] == 0xC9 && i + 5 < byte_count) {
            // 0x30C9 command found: payload = [zone_id][temp_hi][temp_lo]
            int16_t raw = (int16_t)((bytes[i+3] << 8) | bytes[i+4]);
            if (raw > -2000 && raw < 6000 && raw != 0x7FFF) {
                msg->temperature_c = raw / 100.0f;
                break;
            }
        }
        if (bytes[i] == 0x23 && bytes[i+1] == 0x09 && i + 5 < byte_count) {
            // 0x2309 setpoint: payload = [zone_id][sp_hi][sp_lo]
            int16_t raw = (int16_t)((bytes[i+3] << 8) | bytes[i+4]);
            if (raw > 0 && raw < 5000 && raw != 0x7FFF) {
                msg->temperature_c = raw / 100.0f;
                break;
            }
        }
    }

    // Fallback: if bytes[3]==0x30 and enough bytes, try old single-byte-command interpretation
    if (isnan(msg->temperature_c) && bytes[3] == 0x30 && byte_count >= 8) {
        int16_t raw = (int16_t)((bytes[5] << 8) | bytes[6]);
        if (raw > -2000 && raw < 6000)
            msg->temperature_c = raw / 100.0f;
    }

    int32_t copy_len = byte_count < 32 ? byte_count : 32;
    memcpy(msg->payload, bytes, copy_len);
    msg->payload_len = copy_len;
    msg->timestamp_ms = now_ms();
    return true;
}

// ======================== Main processing ========================

iot_decoder_state_t *iotDecoderCreate(uint32_t sample_rate)
{
    iot_decoder_state_t *state = static_cast<iot_decoder_state_t*>(calloc(1, sizeof(*state)));
    if (!state) return NULL;
    state->sample_rate = sample_rate;
    state->out_queue = msg_queue_create(sizeof(iot_device_msg_t), 32);
    if (!state->out_queue) { free(state); return NULL; }
    return state;
}

void iotDecoderDestroy(iot_decoder_state_t *state)
{
    if (!state) return;
    msg_queue_destroy(state->out_queue);
    free(state);
}

uint64_t iotDecoderGetPacketsDecoded(iot_decoder_state_t *state)
{
    return state ? state->packets_decoded : 0;
}

int32_t iotDecoderDequeue(iot_decoder_state_t *state, iot_device_msg_t *msg)
{
    if (!state || !state->out_queue) return 0;
    return msg_queue_pop(state->out_queue, msg);
}

// AM envelope detection: sqrt(I² + Q²) approximated
static inline uint8_t envelope(int8_t i, int8_t q)
{
    int32_t ai = abs(i);
    int32_t aq = abs(q);
    // Fast magnitude approximation: max(|I|,|Q|) + min(|I|,|Q|)/4
    if (ai > aq)
        return (uint8_t)(ai + (aq >> 2));
    else
        return (uint8_t)(aq + (ai >> 2));
}

// FM discriminator: arg(conj(prev) * curr)
static inline int16_t fm_demod(int8_t i0, int8_t q0, int8_t i1, int8_t q1)
{
    // Cross product gives sin(phase_diff) ≈ phase_diff for small angles
    return (int16_t)(i0 * q1 - q0 * i1);
}

// Process one block of IQ data
static void process_block(iot_decoder_state_t *state, const uint8_t *iq, uint32_t sample_count)
{
    // Pass 1: OOK envelope detection + pulse extraction
    for (uint32_t i = 0; i < sample_count; i++) {
        int8_t si = (int8_t)(iq[i*2]   - 128);
        int8_t sq = (int8_t)(iq[i*2+1] - 128);
        uint8_t env = envelope(si, sq);

        // Low-pass filter
        state->ook_level = (uint8_t)((state->ook_level * 7 + env) / 8);

        bool above = (state->ook_level > IOT_OOK_THRESHOLD);

        if (above && !state->ook_high) {
            // Rising edge — end of gap
            if (state->ook_count > 0 && state->pulse_count > 0) {
                uint32_t gap_us = SAMPLES_TO_US(state->ook_count);
                if (gap_us < IOT_GAP_TIMEOUT_US) {
                    state->pulses[state->pulse_count - 1].gap_us = (uint16_t)(gap_us > 65535 ? 65535 : gap_us);
                } else {
                    // End of message — try to decode
                    if (state->pulse_count >= 20) {
                        iot_device_msg_t msg;
                        msg = {};
                        msg.temperature_c = NAN;
                        msg.humidity_pct = NAN;
                        msg.pressure_hpa = NAN;
                        msg.wind_speed_ms = NAN;
                        msg.wind_dir_deg = NAN;
                        msg.rain_mm = NAN;
                        msg.power_w = NAN;
                        msg.energy_kwh = NAN;
                        msg.battery_v = NAN;
                        msg.rssi_db = NAN;
                        msg.freq_offset_hz = NAN;

                        bool decoded = false;
                        if (!decoded) decoded = decode_lacrosse_tx(state->pulses, state->pulse_count, &msg);
                        if (!decoded) decoded = decode_bresser_5in1(state->pulses, state->pulse_count, &msg);
                        // No generic OOK — too many false positives from noise

                        if (decoded) {
                            msg_queue_push(state->out_queue, &msg);
                            state->packets_decoded++;
                        }
                    }
                    state->pulse_count = 0;
                }
            }
            state->ook_high = true;
            state->ook_count = 0;
        } else if (!above && state->ook_high) {
            // Falling edge — end of pulse
            uint32_t pulse_us = SAMPLES_TO_US(state->ook_count);
            if (pulse_us >= IOT_MIN_PULSE_US && pulse_us <= IOT_MAX_PULSE_US &&
                state->pulse_count < IOT_MAX_PULSES) {
                state->pulses[state->pulse_count].pulse_us = (uint16_t)(pulse_us > 65535 ? 65535 : pulse_us);
                state->pulses[state->pulse_count].gap_us = 0;
                state->pulse_count++;
            }
            state->ook_high = false;
            state->ook_count = 0;
        } else {
            state->ook_count++;
        }
    }

    // Pass 2: FSK demodulation at 2 MSPS with delay-1 FM discriminator.
    // Bit periods at 2 MSPS:
    static const int32_t bit_periods[] = { 20, 52, 61, 110, 210, 248 };
    static const int32_t num_rates = 6;

    int8_t prev_i = state->fsk_prev_i;
    int8_t prev_q = state->fsk_prev_q;

    int16_t fm_buf[IOT_BLOCK_SIZE + 1];
    int32_t fm_len = 0;

    for (uint32_t i = 0; i < sample_count; i++) {
        int8_t si = (int8_t)((int32_t)iq[i * 2]     - 128);
        int8_t sq = (int8_t)((int32_t)iq[i * 2 + 1] - 128);

        fm_buf[fm_len++] = fm_demod(prev_i, prev_q, si, sq);
        prev_i = si;
        prev_q = sq;
    }

    state->fsk_prev_i = prev_i;
    state->fsk_prev_q = prev_q;

    // Try LaCrosse correlator directly on FM buffer (TX29: 14 samp/bit, TX35: 26 samp/bit at 250 kHz)
    {
        static const int32_t lacrosse_rates[] = { 110, 210 };
        for (int32_t lr = 0; lr < 2; lr++) {
            iot_device_msg_t msg = {};
            msg.temperature_c = NAN; msg.humidity_pct = NAN; msg.pressure_hpa = NAN;
            msg.wind_speed_ms = NAN; msg.wind_dir_deg = NAN; msg.rain_mm = NAN;
            msg.power_w = NAN; msg.energy_kwh = NAN; msg.battery_v = NAN;
            msg.rssi_db = NAN; msg.freq_offset_hz = NAN;
            if (decode_lacrosse_fm(fm_buf, fm_len, lacrosse_rates[lr], &msg)) {
                msg_queue_push(state->out_queue, &msg);
                state->packets_decoded++;
            }
        }
    }

    // Bit-slice the decimated FM output at each rate
    for (int32_t rate_idx = 0; rate_idx < num_rates; rate_idx++) {
        int32_t samples_per_bit = bit_periods[rate_idx];
        int32_t fm_dc = state->fsk_fm_dc[rate_idx];
        int32_t accum = state->fsk_accum[rate_idx];
        int32_t cnt = state->fsk_count[rate_idx];
        int32_t ring_len = state->fsk_ring_len[rate_idx];
        uint8_t *ring = state->fsk_ring[rate_idx];

        for (int32_t fi = 0; fi < fm_len; fi++) {
            int32_t freq = fm_buf[fi];
            fm_dc += (freq - fm_dc + 128) >> 8;
            accum += (freq - fm_dc);
            cnt++;

            if (cnt >= samples_per_bit) {
                if (ring_len < 4096)
                    ring[ring_len++] = (accum > 0) ? 1 : 0;
                accum = 0;
                cnt = 0;
            }
        }

        state->fsk_fm_dc[rate_idx] = fm_dc;
        state->fsk_accum[rate_idx] = accum;
        state->fsk_count[rate_idx] = cnt;
        state->fsk_ring_len[rate_idx] = ring_len;

        // Use the persistent ring buffer for decoding
        uint8_t *fsk_bits_local = ring;
        int32_t fsk_bit_count = ring_len;

        // Try FSK protocol decoders on accumulated bits
        if (fsk_bit_count >= 120) {
            iot_device_msg_t msg;
            msg = {};
            msg.temperature_c = NAN;
            msg.humidity_pct = NAN;
            msg.pressure_hpa = NAN;
            msg.wind_speed_ms = NAN;
            msg.wind_dir_deg = NAN;
            msg.rain_mm = NAN;
            msg.power_w = NAN;
            msg.energy_kwh = NAN;
            msg.battery_v = NAN;
            msg.rssi_db = NAN;
            msg.freq_offset_hz = NAN;

            bool decoded = false;
            if (!decoded) decoded = decode_lacrosse_fsk(fsk_bits_local, fsk_bit_count, &msg);
            if (!decoded) decoded = decode_fineoffset_fsk(fsk_bits_local, fsk_bit_count, &msg);
            if (!decoded) decoded = decode_bresser_6in1(fsk_bits_local, fsk_bit_count, &msg);
            if (!decoded) decoded = decode_wmbus(fsk_bits_local, fsk_bit_count, &msg);
            if (!decoded) decoded = decode_honeywell_cm(fsk_bits_local, fsk_bit_count, &msg);

            if (decoded) {
                msg_queue_push(state->out_queue, &msg);
                state->packets_decoded++;
                state->fsk_ring_len[rate_idx] = 0;
                break;
            }
        }

        // Prevent ring overflow: when full, shift out old bits keeping last 256
        if (state->fsk_ring_len[rate_idx] >= 3840) {
            int32_t keep = 256;
            memmove(state->fsk_ring[rate_idx],
                    state->fsk_ring[rate_idx] + state->fsk_ring_len[rate_idx] - keep,
                    keep);
            state->fsk_ring_len[rate_idx] = keep;
        }
    }
    state->samples_processed += sample_count;
}

void iotDecoderProcess(iot_decoder_state_t *state, const uint8_t *iq, uint32_t len)
{
    if (!state || !iq || len < 2) return;

    uint32_t sample_count = len / 2;  // IQ pairs
    uint32_t offset = 0;

    while (offset < sample_count) {
        uint32_t block = sample_count - offset;
        if (block > IOT_BLOCK_SIZE)
            block = IOT_BLOCK_SIZE;
        process_block(state, iq + offset * 2, block);
        offset += block;
    }
}
