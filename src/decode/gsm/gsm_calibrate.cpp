// gsm_calibrate.c: GSM-based crystal frequency calibration for RTL-SDR
//
// Algorithm replicated from ogn-rf gsm_scan (glidernet/ogn-rf):
//
// GSM base stations transmit at precise, known frequencies (multiples of 200 kHz).
// GSM uses GMSK modulation at 270833 bps.  The power spectrum of GMSK has a
// characteristic peak at +DataRate/4 (~67.7 kHz) from the carrier center.
// By measuring where this peak actually appears vs where it should be, we
// determine the frequency error of the RTL-SDR crystal oscillator in PPM.
//
// Procedure:
//   1. Scan the E-GSM-900 downlink band (920-960 MHz) in 2 MHz steps
//   2. For each step, capture IQ samples and compute averaged power spectrum via FFT
//   3. For each GSM channel (every 200 kHz), find the spectral peak
//   4. Compute PPM offset = -1e6 * (peak_offset_hz - DataRate/4) / channel_freq
//   5. Collect measurements, trim outliers, compute mean and RMS
//   6. corrected_ppm = current_ppm + mean_offset

#include "gsm_calibrate.h"
#include "dump1090_defs.h"
#include <stdint.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include "gg_format.h"
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if MODES_ENABLE_DIAGNOSTICS
#define GSM_CAL_DIAG(...) gg::eprint(__VA_ARGS__)
#else
#define GSM_CAL_DIAG(...) do {} while (0)
#endif

// GSM constants
#define GSM_CHAN_WIDTH    200000      // Hz — GSM channel spacing
#define GSM_DATA_RATE    270833      // bps — GMSK bit rate

// Scan parameters (matching ogn-rf gsm_scan defaults)
#define SAMPLE_RATE      2000000     // Hz
#define FFT_SIZE         1024        // bins => ~1953 Hz/bin
#define FFT_HALF         (FFT_SIZE / 2)
#define SAMPLES_PER_READ (16384 * 64)  // 1048576 IQ pairs = 2097152 bytes per read (~0.52 sec)
#define READS_PER_SCAN   4             // Multiple reads per frequency for better SNR
#define LOWER_FREQ       920000000   // Hz — E-GSM-900 downlink start
#define UPPER_FREQ       960000000   // Hz — E-GSM-900 downlink end
#define GUARD_BAND       100000      // Hz
#define MAX_PPM_VALUES   512

// ===================== Radix-2 Cooley-Tukey FFT (N=1024) =====================

typedef struct { float re, im; } cplx_t;

static void fft_forward(cplx_t *x, int32_t n)
{
    // Bit-reversal permutation
    for (int32_t i = 1, j = 0; i < n; i++) {
        int32_t bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) {
            cplx_t t = x[i];
            x[i] = x[j];
            x[j] = t;
        }
    }

    // Butterfly stages
    for (int32_t len = 2; len <= n; len <<= 1) {
        float ang = -2.0f * (float)M_PI / len;
        cplx_t wn = { cosf(ang), sinf(ang) };
        for (int32_t i = 0; i < n; i += len) {
            cplx_t w = { 1.0f, 0.0f };
            for (int32_t j = 0; j < len / 2; j++) {
                cplx_t u = x[i + j];
                cplx_t v = {
                    x[i + j + len/2].re * w.re - x[i + j + len/2].im * w.im,
                    x[i + j + len/2].re * w.im + x[i + j + len/2].im * w.re
                };
                x[i + j].re = u.re + v.re;
                x[i + j].im = u.im + v.im;
                x[i + j + len/2].re = u.re - v.re;
                x[i + j + len/2].im = u.im - v.im;
                float wr = w.re * wn.re - w.im * wn.im;
                w.im = w.re * wn.im + w.im * wn.re;
                w.re = wr;
            }
        }
    }
}

// ===================== Sine window (same as ogn-rf) =====================

static void make_sine_window(float *win, int32_t n)
{
    float scale = 1.0f / sqrtf((float)n);
    for (int32_t i = 0; i < n; i++)
        win[i] = sinf((float)M_PI * i / n) * scale;
}

// ===================== Float comparison for qsort =====================

static int32_t cmp_float(const void *a, const void *b)
{
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

// ===================== Main calibration function =====================

gsm_cal_result_t gsm_calibrate(const char *serial, int32_t current_ppm, float gain_db,
                                sdr_backend_type_t backend)
{
    gsm_cal_result_t result;
    result = {};

    sdr_device_t *dev = NULL;
    uint8_t *iq_buf = NULL;
    float *power_avg = NULL;
    cplx_t *fft_buf = NULL;
    float *ppm_values = NULL;

    // Open device via backend layer — automatically uses the right library
    const sdr_backend_ops_t *ops = sdrBackendResolve(backend);
    if (!ops) {
        snprintf(result.error, sizeof(result.error), "No SDR backend available");
        return result;
    }

    GSM_CAL_DIAG("gsm_cal: opening device %s via backend '%s'\n", serial, ops->name);
    dev = ops->open_by_serial(serial);
    if (!dev) {
        snprintf(result.error, sizeof(result.error),
                 "Failed to open SDR device %s via %s", serial, ops->name);
        return result;
    }
    // Store ops so dispatch functions work
    dev->ops = ops;

    // Configure SDR
    sdr_set_sample_rate(dev, SAMPLE_RATE);
    sdr_set_freq_correction(dev, current_ppm);

    // Use manual gain at max for reliable GSM detection.
    // Auto-gain is unreliable on FC0012 tuners and some RTL-SDR clones.
    sdr_set_gain_mode(dev, 1);  // manual gain
    int32_t gains[64];
    int32_t n_gains = sdr_get_tuner_gains(dev, gains, 64);
    if (n_gains > 0) {
        sdr_set_gain(dev, gains[n_gains - 1]);  // max gain
        GSM_CAL_DIAG("gsm_cal: %s gain set to %d (tenth dB), %d steps available\n",
                 serial, gains[n_gains - 1], n_gains);
    } else {
        gg::eprint("gsm_cal: %s WARNING: no gain steps available!\n", serial);
    }
    (void)gain_db;

    // Allocate buffers
    iq_buf = static_cast<decltype(iq_buf)>(malloc(SAMPLES_PER_READ * 2));
    power_avg = static_cast<decltype(power_avg)>(calloc(FFT_SIZE, sizeof(float)));
    fft_buf = static_cast<decltype(fft_buf)>(malloc(FFT_SIZE * sizeof(cplx_t)));
    ppm_values = static_cast<decltype(ppm_values)>(malloc(MAX_PPM_VALUES * sizeof(float)));
    int32_t ppm_count = 0;

    if (!iq_buf || !power_avg || !fft_buf || !ppm_values) {
        snprintf(result.error, sizeof(result.error), "Memory allocation failed");
        goto cleanup;
    }

    { // block scope to avoid goto-crosses-initialization
    // Prepare sine window
    float window[FFT_SIZE];
    make_sine_window(window, FFT_SIZE);
    float bin_width = (float)SAMPLE_RATE / FFT_SIZE;

    // Scan plan: step through the E-GSM band in SAMPLE_RATE (2 MHz) steps
    int32_t freq_step = SAMPLE_RATE;
    int32_t scans = (UPPER_FREQ - LOWER_FREQ - 2 * GUARD_BAND + freq_step - 1) / freq_step;
    int32_t read_ok_count = 0, read_fail_count = 0;
    float best_ratio_overall = 0;

    int32_t freq = LOWER_FREQ + GUARD_BAND + freq_step / 2;
    for (int32_t scan = 0; scan < scans && ppm_count < MAX_PPM_VALUES; scan++, freq += freq_step) {
        // Tune to scan frequency
        sdr_set_frequency(dev, (uint32_t)freq);
        int32_t actual_freq = (int32_t)sdr_get_frequency(dev);
        sdr_reset_buffer(dev);

        // Compute averaged power spectrum from multiple reads for better SNR
        memset(power_avg, 0, FFT_SIZE * sizeof(float));
        int32_t slide_step = FFT_HALF;   // 50% overlap
        int32_t num_slides = 0;
        int32_t scan_read_ok = 0;

        for (int32_t rd = 0; rd < READS_PER_SCAN; rd++) {
            // Read IQ samples synchronously via backend layer
            int32_t n_read = 0;
            int32_t rc = sdr_read_sync(dev, iq_buf, SAMPLES_PER_READ * 2, &n_read);
            (void)rc;
            // Accept data even with LIBUSB_ERROR_OVERFLOW (-8)
            if (n_read < (int32_t)(FFT_SIZE * 4)) {
                if (rd == 0)
                    GSM_CAL_DIAG("gsm_cal: scan %d freq=%dHz: read failed rc=%d n_read=%d\n", scan, freq, rc, n_read);
                continue;
            }
            scan_read_ok++;
            int32_t num_iq_pairs = n_read / 2;

            // Log first read of first scan to verify data quality
            if (scan == 0 && rd == 0) {
                int32_t dc_count = 0;
                for (int32_t k = 0; k < 1000 && k < n_read; k++)
                    if (iq_buf[k] == 127 || iq_buf[k] == 128) dc_count++;
                GSM_CAL_DIAG("gsm_cal: %s scan0 freq=%d n_read=%d dc_pct=%d%%\n",
                         serial, freq, n_read, dc_count * 100 / (n_read < 1000 ? n_read : 1000));
            }

            // Accumulate FFT power from this read
            for (int32_t off = 0; off + FFT_SIZE <= num_iq_pairs; off += slide_step) {
                for (int32_t i = 0; i < FFT_SIZE; i++) {
                    float re = (iq_buf[(off + i) * 2    ] - 127.5f) / 128.0f;
                    float im = (iq_buf[(off + i) * 2 + 1] - 127.5f) / 128.0f;
                    fft_buf[i].re = re * window[i];
                    fft_buf[i].im = im * window[i];
                }

                fft_forward(fft_buf, FFT_SIZE);

                for (int32_t i = 0; i < FFT_SIZE; i++) {
                    int32_t si = (i + FFT_HALF) % FFT_SIZE;
                    float pwr = fft_buf[i].re * fft_buf[i].re
                              + fft_buf[i].im * fft_buf[i].im;
                    power_avg[si] += pwr;
                }
                num_slides++;
            }
        }

        if (scan_read_ok == 0) {
            read_fail_count++;
            continue;
        }
        read_ok_count++;

        if (num_slides < 2)
            continue;

        // Normalize
        float inv_slides = 1.0f / num_slides;
        for (int32_t i = 0; i < FFT_SIZE; i++)
            power_avg[i] *= inv_slides;

        // Analyze GSM channels in this scan
        // After fftshift: bin 0 = lowest freq, bin FFT_HALF = DC (center)
        float first_bin_freq = (float)actual_freq - bin_width * FFT_HALF;
        float last_bin_freq  = (float)actual_freq + bin_width * FFT_HALF;

        // Expected GMSK peak offset from carrier in bins
        float exp_peak_bins = (float)GSM_DATA_RATE / (4.0f * bin_width);  // ~34.7 bins

        int32_t chan = (int32_t)ceilf(first_bin_freq / GSM_CHAN_WIDTH);
        for (; ppm_count < MAX_PPM_VALUES; chan++) {
            float chan_freq = chan * (float)GSM_CHAN_WIDTH;
            if (chan_freq >= last_bin_freq)
                break;

            // Expected GMSK peak position: channel_center + DataRate/4
            float center_bin = (chan_freq - first_bin_freq) / bin_width;
            int32_t exp_bin = (int32_t)(center_bin + exp_peak_bins + 0.5f);

            // Search window: ±4 bins (~±8 kHz) around expected peak
            // A genuine GMSK DataRate/4 peak should be within ±2 bins of expected
            int32_t search_lo = exp_bin - 4;
            int32_t search_hi = exp_bin + 4;
            if (search_lo < 2 || search_hi >= FFT_SIZE - 2)
                continue;

            // Find peak near expected position
            float peak_val = 0;
            int32_t peak_idx = exp_bin;
            for (int32_t i = search_lo; i <= search_hi; i++) {
                if (power_avg[i] > peak_val) {
                    peak_val = power_avg[i];
                    peak_idx = i;
                }
            }

            // Can't interpolate at edges of search window
            if (peak_idx <= search_lo || peak_idx >= search_hi)
                continue;

            // Background: average power in the lower half of the channel
            // (away from the GMSK peak, between center-90kHz and center+20kHz)
            int32_t bkg_lo = (int32_t)(center_bin - 0.45f * GSM_CHAN_WIDTH / bin_width + 0.5f);
            int32_t bkg_hi = (int32_t)(center_bin + 0.10f * GSM_CHAN_WIDTH / bin_width + 0.5f);
            if (bkg_lo < 0) bkg_lo = 0;
            if (bkg_hi >= FFT_SIZE) bkg_hi = FFT_SIZE - 1;
            float bkg = 0;
            int32_t bkg_n = 0;
            for (int32_t i = bkg_lo; i <= bkg_hi; i++) {
                bkg += power_avg[i];
                bkg_n++;
            }
            if (bkg_n > 0) bkg /= bkg_n;

            // Signal quality: peak at expected GMSK position must be >> background
            float ratio = (bkg > 1e-12f) ? peak_val / bkg : 0;
            if (ratio > best_ratio_overall) best_ratio_overall = ratio;
            if (bkg < 1e-12f || ratio < 2.0f)
                continue;

            // Sub-bin interpolation (parabolic)
            float y_m = power_avg[peak_idx - 1];
            float y_0 = power_avg[peak_idx];
            float y_p = power_avg[peak_idx + 1];
            float peak_pos;
            float peak_sum = y_m + y_0 + y_p;
            if (peak_sum > 1e-12f)
                peak_pos = (float)peak_idx + (y_p - y_m) / peak_sum;
            else
                peak_pos = (float)peak_idx;

            // PPM calculation:
            // measured offset from channel center in Hz
            float offset_bins = peak_pos - center_bin;
            float measured_hz = offset_bins * bin_width;
            float expected_hz = (float)GSM_DATA_RATE / 4.0f;
            float ppm = -1e6f * (measured_hz - expected_hz) / chan_freq;

            // Reject obvious outliers
            if (fabsf(ppm) > 200.0f)
                continue;

            ppm_values[ppm_count++] = ppm;
        }
    }

    // Close device
    ops->close(dev);
    dev = NULL;

    GSM_CAL_DIAG("gsm_cal: device %s via %s: %d scans, %d reads OK, %d reads failed, %d PPM measurements, best_ratio=%.1f (need>2.0)\n",
                 serial, ops->name, scans, read_ok_count, read_fail_count, ppm_count, best_ratio_overall);

    // ---- Statistical processing: median + MAD filtering ----

    if (ppm_count < 4) {
        snprintf(result.error, sizeof(result.error),
                 "Not enough GSM signals: only %d measurements (need 4+). "
                 "Ensure antenna can receive 920-960 MHz.", ppm_count);
        goto cleanup;
    }

    // Sort
    qsort(ppm_values, ppm_count, sizeof(float), cmp_float);

    // Median
    float median;
    if (ppm_count % 2 == 0)
        median = (ppm_values[ppm_count/2 - 1] + ppm_values[ppm_count/2]) / 2.0f;
    else
        median = ppm_values[ppm_count/2];

    // MAD (Median Absolute Deviation)
    float *abs_dev = static_cast<float*>(malloc(ppm_count * sizeof(float)));
    if (!abs_dev) { snprintf(result.error, sizeof(result.error), "Memory allocation failed"); goto cleanup; }
    for (int32_t i = 0; i < ppm_count; i++)
        abs_dev[i] = fabsf(ppm_values[i] - median);
    qsort(abs_dev, ppm_count, sizeof(float), cmp_float);
    float mad;
    if (ppm_count % 2 == 0)
        mad = (abs_dev[ppm_count/2 - 1] + abs_dev[ppm_count/2]) / 2.0f;
    else
        mad = abs_dev[ppm_count/2];
    free(abs_dev);

    // Keep only values within 2*MAD of median (or 3 ppm if MAD is tiny)
    float threshold = fmaxf(2.0f * mad, 3.0f);
    float sum = 0;
    int32_t kept = 0;
    for (int32_t i = 0; i < ppm_count; i++) {
        if (fabsf(ppm_values[i] - median) <= threshold) {
            sum += ppm_values[i];
            kept++;
        }
    }

    if (kept < 3) {
        // Fall back to median only
        sum = median;
        kept = 1;
    }
    float mean = sum / kept;

    // RMS of kept values
    float rms_sum = 0;
    for (int32_t i = 0; i < ppm_count; i++) {
        if (fabsf(ppm_values[i] - median) <= threshold) {
            float d = ppm_values[i] - mean;
            rms_sum += d * d;
        }
    }
    float rms = (kept > 1) ? sqrtf(rms_sum / kept) : 0.0f;

    result.success = 1;
    result.measured_offset = mean;
    result.corrected_ppm = (double)current_ppm + mean;
    result.rms = rms;
    result.samples = kept;

    if (rms > 0.3f) {
        snprintf(result.error, sizeof(result.error),
                 "Warning: noisy measurements (RMS=%.2f ppm). "
                 "Result may be inaccurate — retry after warmup.", rms);
    }

    } // end block scope

cleanup:
    if (dev) ops->close(dev);
    free(iq_buf);
    free(power_avg);
    free(fft_buf);
    free(ppm_values);
    return result;
}
