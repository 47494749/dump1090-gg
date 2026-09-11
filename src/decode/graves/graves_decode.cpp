// Part of dump1090-gg-light
//
// graves_decode.cpp: GRAVES passive radar decoder
//
// Signal processing pipeline:
//   IQ samples (250 kHz) → DC removal → Hann window → FFT → peak detection
//   → target tracking → optional ADS-B correlation
//
// The GRAVES space surveillance radar (Apt, France) transmits a strong VHF
// signal at 143.050 MHz. Aircraft reflect this signal with a Doppler shift
// proportional to their bistatic velocity. By performing spectral analysis
// on the received signal, we detect these reflections as peaks offset from
// the direct-path carrier.
//
// This file is free software: GPL-3.0-or-later

#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <algorithm>
#include "graves_decode.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Speed of light (m/s)
#define C_LIGHT 299792458.0

// ======================== FFT (in-place radix-2 Cooley-Tukey) ========================

typedef struct { float re, im; } cplx_t;

static void fft_dit(cplx_t *x, int32_t N)
{
    // Bit-reversal permutation
    for (int32_t i = 1, j = 0; i < N; i++) {
        int32_t bit = N >> 1;
        while (j & bit) { j ^= bit; bit >>= 1; }
        j ^= bit;
        if (i < j) { cplx_t tmp = x[i]; x[i] = x[j]; x[j] = tmp; }
    }

    // Butterfly stages
    for (int32_t len = 2; len <= N; len <<= 1) {
        float ang = -2.0f * (float)M_PI / len;
        cplx_t wlen = { cosf(ang), sinf(ang) };
        for (int32_t i = 0; i < N; i += len) {
            cplx_t w = { 1.0f, 0.0f };
            for (int32_t j = 0; j < len / 2; j++) {
                cplx_t u = x[i + j];
                cplx_t v = { w.re * x[i + j + len/2].re - w.im * x[i + j + len/2].im,
                             w.re * x[i + j + len/2].im + w.im * x[i + j + len/2].re };
                x[i + j].re = u.re + v.re;
                x[i + j].im = u.im + v.im;
                x[i + j + len/2].re = u.re - v.re;
                x[i + j + len/2].im = u.im - v.im;
                float wnew_re = w.re * wlen.re - w.im * wlen.im;
                w.im = w.re * wlen.im + w.im * wlen.re;
                w.re = wnew_re;
            }
        }
    }
}

// ======================== Hann window ========================

static float hann_window[GRAVES_FFT_SIZE];
static volatile int32_t hann_initialized = 0;

static void init_hann_window(void)
{
    if (hann_initialized) return;
    for (int32_t i = 0; i < GRAVES_FFT_SIZE; i++)
        hann_window[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (GRAVES_FFT_SIZE - 1)));
    __sync_synchronize();
    hann_initialized = 1;
}

// ======================== Peak detection ========================

typedef struct {
    float freq_hz;      // Doppler frequency (Hz)
    float power_db;     // power relative to noise floor (dB)
    float raw_power;    // linear power
    int32_t bin;        // FFT bin index
} peak_t;

// ======================== Target tracker ========================

typedef struct {
    graves_target_t info;
    int32_t active;         // 1 = active, 0 = slot free
    float   predicted_doppler;  // predicted Doppler for next frame
} track_slot_t;

// ======================== State ========================

struct graves_state {
    graves_config_t config;

    // IQ accumulation buffer (for overlapping FFT)
    float   *iq_buf_i;      // I channel ring buffer
    float   *iq_buf_q;      // Q channel ring buffer
    int32_t  iq_wr;         // write position
    int32_t  iq_count;      // total samples in buffer

    // FFT workspace
    cplx_t  *fft_buf;
    float   *power_spectrum; // |FFT|^2 in dB

    // DC removal
    float dc_i, dc_q;

    // Noise floor estimation
    float noise_floor_db;
    float direct_signal_db;

    // Target tracker
    track_slot_t tracks[GRAVES_MAX_TARGETS];
    uint32_t next_track_id;

    // ADS-B aircraft list (for correlation)
    graves_adsb_aircraft_t adsb_list[256];
    int32_t adsb_count;

    // Stats
    graves_stats_t stats;

    // Timing
    uint64_t last_fft_ms;
    float    bin_hz;        // Hz per FFT bin
};

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

// ======================== Create / Destroy ========================

struct graves_state *graves_create(const graves_config_t *config)
{
    init_hann_window();

    struct graves_state *s = (struct graves_state *)calloc(1, sizeof(*s));
    if (!s) return NULL;

    s->config = *config;
    if (s->config.min_snr_db <= 0) s->config.min_snr_db = 8.0f;
    if (s->config.min_doppler_hz <= 0) s->config.min_doppler_hz = 200.0f;
    if (s->config.max_doppler_hz <= 0) s->config.max_doppler_hz = 15000.0f;
    if (s->config.track_timeout_sec <= 0) s->config.track_timeout_sec = 30;
    if (s->config.sample_rate <= 0) s->config.sample_rate = GRAVES_SAMPLE_RATE;

    s->iq_buf_i = (float *)calloc(GRAVES_FFT_SIZE * 2, sizeof(float));
    s->iq_buf_q = (float *)calloc(GRAVES_FFT_SIZE * 2, sizeof(float));
    s->fft_buf = (cplx_t *)calloc(GRAVES_FFT_SIZE, sizeof(cplx_t));
    s->power_spectrum = (float *)calloc(GRAVES_FFT_SIZE, sizeof(float));

    if (!s->iq_buf_i || !s->iq_buf_q || !s->fft_buf || !s->power_spectrum) {
        graves_destroy(s);
        return NULL;
    }

    s->bin_hz = (float)config->sample_rate / GRAVES_FFT_SIZE;
    s->next_track_id = 1;
    s->noise_floor_db = -40.0f;

    fprintf(stderr, "GRAVES: decoder created (freq=%.3f MHz, rate=%u, FFT=%d, bin=%.1f Hz)\n",
            config->center_freq / 1e6, (uint32_t)config->sample_rate,
            GRAVES_FFT_SIZE, (double)s->bin_hz);

    return s;
}

void graves_destroy(struct graves_state *state)
{
    if (!state) return;
    free(state->iq_buf_i);
    free(state->iq_buf_q);
    free(state->fft_buf);
    free(state->power_spectrum);
    free(state);
}

// ======================== Noise floor estimation ========================
// Use median of power spectrum (excluding DC and strong peaks)

static float estimate_noise_floor(const float *power_db, int32_t N)
{
    // Simple approach: sort a subset and take the 25th percentile
    static float sorted[1024];
    int32_t count = 0;
    int32_t step = N / 1024;
    if (step < 1) step = 1;

    for (int32_t i = 10; i < N - 10 && count < 1024; i += step)
        sorted[count++] = power_db[i];

    // Partial sort for 25th percentile
    int32_t target = count / 4;
    std::nth_element(sorted, sorted + target, sorted + count);
    return sorted[target];
}

// ======================== Peak detection ========================

static int32_t detect_peaks(const float *power_db, float noise_floor,
                            float bin_hz, const graves_config_t *cfg,
                            peak_t *peaks, int32_t max_peaks)
{
    int32_t N = GRAVES_FFT_SIZE;
    int32_t count = 0;
    float threshold = noise_floor + cfg->min_snr_db;

    // Convert Hz limits to bin indices
    // FFT layout: bin 0 = DC, bin 1..N/2-1 = positive freq, bin N/2..N-1 = negative freq
    int32_t min_bin = (int32_t)(cfg->min_doppler_hz / bin_hz);
    int32_t max_bin = (int32_t)(cfg->max_doppler_hz / bin_hz);
    if (min_bin < 3) min_bin = 3;  // skip DC region
    if (max_bin > N / 2 - 1) max_bin = N / 2 - 1;

    // Scan positive Doppler (approaching targets)
    for (int32_t i = min_bin; i <= max_bin && count < max_peaks; i++) {
        if (power_db[i] > threshold &&
            power_db[i] > power_db[i-1] && power_db[i] > power_db[i+1] &&
            power_db[i] > power_db[i-2] && power_db[i] > power_db[i+2]) {
            // Parabolic interpolation for sub-bin accuracy
            float alpha = power_db[i-1];
            float beta  = power_db[i];
            float gamma = power_db[i+1];
            float delta = 0.5f * (alpha - gamma) / (alpha - 2.0f * beta + gamma);
            peaks[count].bin = i;
            peaks[count].freq_hz = (i + delta) * bin_hz;
            peaks[count].power_db = beta - 0.25f * (alpha - gamma) * delta;
            peaks[count].raw_power = powf(10.0f, peaks[count].power_db / 10.0f);
            count++;
        }
    }

    // Scan negative Doppler (receding targets)
    for (int32_t i = N - max_bin; i <= N - min_bin && count < max_peaks; i++) {
        if (power_db[i] > threshold &&
            power_db[i] > power_db[i-1] && power_db[i] > power_db[i+1] &&
            power_db[i] > power_db[i-2] && power_db[i] > power_db[i+2]) {
            float alpha = power_db[i-1];
            float beta  = power_db[i];
            float gamma = power_db[i+1];
            float delta = 0.5f * (alpha - gamma) / (alpha - 2.0f * beta + gamma);
            // Negative Doppler: bin N-k corresponds to -k*bin_hz
            peaks[count].bin = i;
            peaks[count].freq_hz = -((N - i) - delta) * bin_hz;
            peaks[count].power_db = beta - 0.25f * (alpha - gamma) * delta;
            peaks[count].raw_power = powf(10.0f, peaks[count].power_db / 10.0f);
            count++;
        }
    }

    return count;
}

// ======================== Target tracking ========================

static void update_tracks(struct graves_state *s, const peak_t *peaks, int32_t npk)
{
    uint64_t now = now_ms();
    float gate_hz = s->bin_hz * 3.0f;  // association gate: ±3 bins

    // Mark all tracks as "not updated this frame"
    bool peak_used[GRAVES_MAX_PEAKS] = {false};
    bool track_updated[GRAVES_MAX_TARGETS] = {false};

    // Associate peaks with existing tracks (nearest-neighbor)
    for (int32_t t = 0; t < GRAVES_MAX_TARGETS; t++) {
        if (!s->tracks[t].active) continue;
        float best_dist = gate_hz;
        int32_t best_pk = -1;
        for (int32_t p = 0; p < npk; p++) {
            if (peak_used[p]) continue;
            float dist = fabsf(peaks[p].freq_hz - s->tracks[t].predicted_doppler);
            if (dist < best_dist) {
                best_dist = dist;
                best_pk = p;
            }
        }
        if (best_pk >= 0) {
            peak_used[best_pk] = true;
            track_updated[t] = true;
            graves_target_t *tgt = &s->tracks[t].info;
            float old_doppler = tgt->doppler_hz;
            tgt->doppler_hz = peaks[best_pk].freq_hz;
            tgt->amplitude_db = peaks[best_pk].power_db - s->noise_floor_db;
            tgt->velocity_ms = (float)(tgt->doppler_hz * C_LIGHT / (double)GRAVES_FREQ);
            float dt = (now - tgt->last_seen_ms) / 1000.0f;
            if (dt > 0.01f)
                tgt->doppler_rate = (tgt->doppler_hz - old_doppler) / dt;
            tgt->last_seen_ms = now;
            tgt->updates++;
            tgt->missed = 0;
            tgt->age_frames++;
            s->tracks[t].predicted_doppler = tgt->doppler_hz + tgt->doppler_rate * 0.033f;
        }
    }

    // Age out tracks that weren't updated
    for (int32_t t = 0; t < GRAVES_MAX_TARGETS; t++) {
        if (!s->tracks[t].active) continue;
        if (!track_updated[t]) {
            s->tracks[t].info.missed++;
            s->tracks[t].info.age_frames++;
            float timeout_ms = s->config.track_timeout_sec * 1000.0f;
            if ((now - s->tracks[t].info.last_seen_ms) > (uint64_t)timeout_ms) {
                s->tracks[t].active = 0;
                s->stats.targets_lost++;
                s->stats.active_targets--;
            }
        }
    }

    // Create new tracks for unassociated peaks
    for (int32_t p = 0; p < npk; p++) {
        if (peak_used[p]) continue;
        // Find free slot
        for (int32_t t = 0; t < GRAVES_MAX_TARGETS; t++) {
            if (s->tracks[t].active) continue;
            memset(&s->tracks[t], 0, sizeof(track_slot_t));
            s->tracks[t].active = 1;
            s->tracks[t].info.track_id = s->next_track_id++;
            s->tracks[t].info.doppler_hz = peaks[p].freq_hz;
            s->tracks[t].info.amplitude_db = peaks[p].power_db - s->noise_floor_db;
            s->tracks[t].info.velocity_ms = (float)(peaks[p].freq_hz * C_LIGHT / (double)GRAVES_FREQ);
            s->tracks[t].info.first_seen_ms = now;
            s->tracks[t].info.last_seen_ms = now;
            s->tracks[t].info.updates = 1;
            s->tracks[t].info.age_frames = 1;
            s->tracks[t].predicted_doppler = peaks[p].freq_hz;
            s->stats.targets_created++;
            s->stats.active_targets++;
            break;
        }
    }
}

// ======================== ADS-B correlation ========================

static void correlate_adsb(struct graves_state *s)
{
    if (!s->config.adsb_correlate || s->adsb_count == 0) return;

    double rx_lat = s->config.rx_lat * M_PI / 180.0;
    double rx_lon = s->config.rx_lon * M_PI / 180.0;
    double tx_lat = GRAVES_TX_LAT * M_PI / 180.0;
    double tx_lon = GRAVES_TX_LON * M_PI / 180.0;

    int32_t matched = 0, unmatched = 0;

    for (int32_t t = 0; t < GRAVES_MAX_TARGETS; t++) {
        if (!s->tracks[t].active) continue;
        graves_target_t *tgt = &s->tracks[t].info;
        tgt->matched_icao = 0;
        tgt->matched_callsign[0] = '\0';
        tgt->match_score = 0;

        float best_score = 0;
        int32_t best_idx = -1;

        for (int32_t a = 0; a < s->adsb_count; a++) {
            const graves_adsb_aircraft_t *ac = &s->adsb_list[a];
            if (ac->lat == 0 && ac->lon == 0) continue;

            // Compute expected bistatic Doppler for this aircraft
            double ac_lat = ac->lat * M_PI / 180.0;
            double ac_lon = ac->lon * M_PI / 180.0;
            double ac_hdg = ac->heading_deg * M_PI / 180.0;
            double ac_spd = ac->speed_kt * 0.514444;  // knots to m/s

            // Velocity vector in local ENU
            double ve = ac_spd * sinf((float)ac_hdg);
            double vn = ac_spd * cosf((float)ac_hdg);

            // Direction from aircraft to receiver (simplified flat-earth)
            double dlat_rx = rx_lat - ac_lat;
            double dlon_rx = (rx_lon - ac_lon) * cos(ac_lat);
            double dist_rx = sqrt(dlat_rx * dlat_rx + dlon_rx * dlon_rx);
            if (dist_rx < 1e-10) continue;
            double ur_n = dlat_rx / dist_rx;
            double ur_e = dlon_rx / dist_rx;

            // Direction from aircraft to GRAVES TX
            double dlat_tx = tx_lat - ac_lat;
            double dlon_tx = (tx_lon - ac_lon) * cos(ac_lat);
            double dist_tx = sqrt(dlat_tx * dlat_tx + dlon_tx * dlon_tx);
            if (dist_tx < 1e-10) continue;
            double ut_n = dlat_tx / dist_tx;
            double ut_e = dlon_tx / dist_tx;

            // Bistatic Doppler = f/c * (v·u_rx + v·u_tx)
            double vr_rx = vn * ur_n + ve * ur_e;
            double vr_tx = vn * ut_n + ve * ut_e;
            double expected_doppler = (double)GRAVES_FREQ / C_LIGHT * (vr_rx + vr_tx);

            // Compare with measured Doppler
            float diff = fabsf(tgt->doppler_hz - (float)expected_doppler);
            float tolerance = s->bin_hz * 5.0f;  // ±5 bins tolerance
            if (diff < tolerance) {
                float score = 1.0f - diff / tolerance;
                if (score > best_score) {
                    best_score = score;
                    best_idx = a;
                }
            }
        }

        if (best_idx >= 0 && best_score > 0.3f) {
            tgt->matched_icao = s->adsb_list[best_idx].icao;
            strncpy(tgt->matched_callsign, s->adsb_list[best_idx].callsign, 8);
            tgt->matched_callsign[8] = '\0';
            tgt->match_score = best_score;
            matched++;
        } else {
            unmatched++;
        }
    }

    s->stats.matched_targets = matched;
    s->stats.unmatched_targets = unmatched;
}

// ======================== Process IQ ========================

void graves_process(struct graves_state *state, const uint8_t *iq_data, uint32_t len)
{
    uint32_t samples = len / 2;
    state->stats.samples_processed += samples;

    for (uint32_t i = 0; i < samples; i++) {
        // Convert UC8 to float with DC removal
        float si = (float)iq_data[i * 2]     - 127.5f;
        float sq = (float)iq_data[i * 2 + 1] - 127.5f;

        // IIR DC removal (alpha ~0.995)
        state->dc_i = 0.995f * state->dc_i + 0.005f * si;
        state->dc_q = 0.995f * state->dc_q + 0.005f * sq;
        si -= state->dc_i;
        sq -= state->dc_q;

        // Store in ring buffer
        int32_t pos = state->iq_wr;
        state->iq_buf_i[pos] = si;
        state->iq_buf_q[pos] = sq;
        state->iq_wr = (pos + 1) % (GRAVES_FFT_SIZE * 2);
        state->iq_count++;

        // When we have enough samples for a new FFT frame
        if (state->iq_count >= GRAVES_FFT_SIZE &&
            (state->iq_count % GRAVES_OVERLAP) == 0) {

            // Fill FFT buffer with windowed samples
            int32_t start = (state->iq_wr - GRAVES_FFT_SIZE + GRAVES_FFT_SIZE * 2)
                            % (GRAVES_FFT_SIZE * 2);
            for (int32_t k = 0; k < GRAVES_FFT_SIZE; k++) {
                int32_t idx = (start + k) % (GRAVES_FFT_SIZE * 2);
                state->fft_buf[k].re = state->iq_buf_i[idx] * hann_window[k];
                state->fft_buf[k].im = state->iq_buf_q[idx] * hann_window[k];
            }

            // FFT
            fft_dit(state->fft_buf, GRAVES_FFT_SIZE);

            // Compute power spectrum in dB
            for (int32_t k = 0; k < GRAVES_FFT_SIZE; k++) {
                float pwr = state->fft_buf[k].re * state->fft_buf[k].re +
                            state->fft_buf[k].im * state->fft_buf[k].im;
                state->power_spectrum[k] = 10.0f * log10f(pwr + 1e-20f);
            }

            // Estimate noise floor
            state->noise_floor_db = estimate_noise_floor(state->power_spectrum,
                                                          GRAVES_FFT_SIZE);
            state->stats.noise_floor_db = state->noise_floor_db;

            // Direct signal strength (DC bin region)
            state->direct_signal_db = state->power_spectrum[0];
            for (int32_t k = 1; k <= 3; k++) {
                if (state->power_spectrum[k] > state->direct_signal_db)
                    state->direct_signal_db = state->power_spectrum[k];
            }
            state->stats.direct_signal_db = state->direct_signal_db - state->noise_floor_db;

            // Detect peaks
            peak_t peaks[GRAVES_MAX_PEAKS];
            int32_t npk = detect_peaks(state->power_spectrum, state->noise_floor_db,
                                       state->bin_hz, &state->config,
                                       peaks, GRAVES_MAX_PEAKS);
            state->stats.peaks_detected += npk;

            // Update tracker
            update_tracks(state, peaks, npk);

            // ADS-B correlation (every frame)
            correlate_adsb(state);

            state->stats.fft_frames++;
            state->last_fft_ms = now_ms();
        }
    }
}

// ======================== API ========================

int32_t graves_get_targets(struct graves_state *state, graves_target_t *targets, int32_t max_targets)
{
    int32_t count = 0;
    for (int32_t t = 0; t < GRAVES_MAX_TARGETS && count < max_targets; t++) {
        if (state->tracks[t].active) {
            targets[count++] = state->tracks[t].info;
        }
    }
    return count;
}

void graves_get_stats(struct graves_state *state, graves_stats_t *stats)
{
    *stats = state->stats;
}

void graves_set_adsb_list(struct graves_state *state,
                          const graves_adsb_aircraft_t *aircraft, int32_t count)
{
    if (count > 256) count = 256;
    memcpy(state->adsb_list, aircraft, count * sizeof(graves_adsb_aircraft_t));
    state->adsb_count = count;
}
