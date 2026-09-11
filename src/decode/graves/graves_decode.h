// Part of dump1090-gg-light
//
// graves_decode.h: GRAVES passive radar decoder
//
// Detects aircraft reflections of the French GRAVES VHF radar (143.050 MHz).
// Each reflection appears as a Doppler-shifted peak in the spectrum.
// Peaks are tracked over time and optionally correlated with ADS-B targets.
//
// GRAVES transmitter: 47.3481°N, 5.5134°E (Plateau d'Albion, near Apt, France)
// Power: ~20 MW EIRP, operating in VHF band
// Range: detectable up to ~800 km from transmitter with good antenna
//
// This file is free software: GPL-3.0-or-later

#ifndef GRAVES_DECODE_H
#define GRAVES_DECODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ======================== Constants ========================

#define GRAVES_FREQ             143050000   // 143.050 MHz
#define GRAVES_SAMPLE_RATE      1000000     // 1 MHz (covers ±500 kHz, min for sdrgg/RTL-SDR)
#define GRAVES_MAX_TARGETS      64          // max simultaneous tracked targets
#define GRAVES_FFT_SIZE         8192        // FFT size (~30 Hz/bin at 250 kHz)
#define GRAVES_OVERLAP          4096        // 50% overlap
#define GRAVES_MAX_PEAKS        32          // max peaks per FFT frame

// GRAVES transmitter location (Plateau d'Albion)
#define GRAVES_TX_LAT           43.9303
#define GRAVES_TX_LON           5.7164

// ======================== Target info ========================

typedef struct {
    uint32_t    track_id;           // unique track identifier
    float       doppler_hz;         // current Doppler shift (Hz)
    float       velocity_ms;        // estimated bistatic velocity (m/s)
    float       amplitude_db;       // signal strength relative to noise (dB)
    float       doppler_rate;       // rate of change of Doppler (Hz/s)
    int32_t     age_frames;         // frames since first detection
    int32_t     updates;            // total number of FFT frames with this target
    int32_t     missed;             // consecutive frames without detection
    uint64_t    first_seen_ms;      // timestamp of first detection
    uint64_t    last_seen_ms;       // timestamp of last detection

    // ADS-B correlation
    uint32_t    matched_icao;       // ICAO address of matched ADS-B target (0 = unmatched)
    char        matched_callsign[9];// callsign of matched target
    float       match_score;        // correlation confidence (0-1)
} graves_target_t;

// ======================== Statistics ========================

typedef struct {
    uint64_t    samples_processed;
    uint64_t    fft_frames;
    uint64_t    peaks_detected;
    uint64_t    targets_created;
    uint64_t    targets_lost;
    int32_t     active_targets;
    int32_t     matched_targets;    // correlated with ADS-B
    int32_t     unmatched_targets;  // no ADS-B match (potential military/stealth)
    float       noise_floor_db;
    float       direct_signal_db;   // strength of direct GRAVES signal (0 Hz)
} graves_stats_t;

// ======================== Configuration ========================

typedef struct {
    double      center_freq;        // should be GRAVES_FREQ
    double      sample_rate;        // should be GRAVES_SAMPLE_RATE
    double      rx_lat;             // receiver latitude
    double      rx_lon;             // receiver longitude
    float       min_snr_db;         // minimum SNR for peak detection (default 8)
    float       min_doppler_hz;     // ignore peaks below this Doppler (default 200)
    float       max_doppler_hz;     // ignore peaks above this Doppler (default 15000)
    int32_t     track_timeout_sec;  // drop track after N seconds without update (default 30)
    int32_t     adsb_correlate;     // enable ADS-B correlation (default 1)
} graves_config_t;

// ======================== Opaque state ========================

struct graves_state;

// ======================== API ========================

struct graves_state *graves_create(const graves_config_t *config);
void graves_destroy(struct graves_state *state);

// Process IQ samples (uint8_t interleaved I/Q)
void graves_process(struct graves_state *state, const uint8_t *iq_data, uint32_t len);

// Get current target list (returns number of active targets)
int32_t graves_get_targets(struct graves_state *state, graves_target_t *targets, int32_t max_targets);

// Get statistics
void graves_get_stats(struct graves_state *state, graves_stats_t *stats);

// Set ADS-B aircraft list for correlation
// Called periodically by the main thread with current ADS-B positions
typedef struct {
    uint32_t icao;
    char     callsign[9];
    double   lat, lon, alt;
    float    speed_kt;          // ground speed in knots
    float    heading_deg;       // track angle
    float    vert_rate_fpm;     // vertical rate ft/min
} graves_adsb_aircraft_t;

void graves_set_adsb_list(struct graves_state *state,
                          const graves_adsb_aircraft_t *aircraft, int32_t count);

#ifdef __cplusplus
}
#endif

#endif // GRAVES_DECODE_H
