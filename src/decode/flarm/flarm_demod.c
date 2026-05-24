// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// flarm_demod.c: GFSK demodulator for FLARM 868 MHz packets
//
// FLARM uses 2-FSK (GFSK) modulation at 100 kbps with +/-50 kHz deviation.
// The radio link uses Manchester encoding, an 8-byte syncword, and
// CRC-16 CCITT. After demodulation and CRC check, packets are passed
// to the FLARM protocol decoder for XXTEA decryption.
//
// Demodulation pipeline:
//   IQ samples (1.6 MSPS) -> NCO channelization -> FIR LPF (33-tap)
//   -> FM discriminator -> DC block -> sample-level matched filter (NCC)
//   -> payload extraction -> Manchester decode -> CRC -> decrypt/decode
//
// Key design: uses sample-level normalized cross-correlation (NCC) for sync
// detection. With 1024-sample sync template (64 chips x 16 samples/chip),
// noise floor std = 1/sqrt(1024) = 0.031, giving robust detection even at
// low SNR (real signals correlate at 0.35-0.60).
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "flarm_demod.h"
#include "flarm_decode.h"
#include "ogntp_decode.h"
#include "p3i_decode.h"
#include "adsl_decode.h"

// ======================== Constants ========================

// FLARM on-air chip rate: 100 kbps
#define FLARM_CHIP_RATE     100000

// At default 1.6 MSPS with 100 kbps on-air chip rate, each chip is 16 samples
// At 2.4 MSPS (P3I mode), each chip is 24 samples
// We use MAX for static array sizing, actual value is stored per-instance
#define SAMPLES_PER_BIT_DEFAULT 16
#define MAX_SAMPLES_PER_BIT     24

// Sync template length in samples (64 chips x max_samples/chip)
// Static arrays are sized for MAX; runtime uses actual length from state
#define SYNC_TEMPLATE_LEN_DEFAULT (FLARM_SYNCWORD_SIZE * 8 * SAMPLES_PER_BIT_DEFAULT)  // 1024
#define SYNC_TEMPLATE_LEN_MAX     (FLARM_SYNCWORD_SIZE * 8 * MAX_SAMPLES_PER_BIT)      // 1536

// After sync: 26 bytes x 8 bits x 2 (Manchester) = 416 chips
#define PAYLOAD_CHIPS       (FLARM_PACKET_TOTAL * 8 * 2)

// ADS-L: 24 bytes x 8 bits x 2 (Manchester) = 384 chips
#define ADSL_PAYLOAD_CHIPS  (ADSL_PACKET_TOTAL * 8 * 2)

// Payload length in samples (default; runtime uses state->payload_samples)
#define PAYLOAD_SAMPLES_DEFAULT (PAYLOAD_CHIPS * SAMPLES_PER_BIT_DEFAULT)  // 6656
#define PAYLOAD_SAMPLES_MAX     (PAYLOAD_CHIPS * MAX_SAMPLES_PER_BIT)      // 9984

// FM sample ring buffer: power of 2, must hold sync + payload + margin
// At max 24 spb: sync=1536 + payload=9984 = 11520, need margin → 32768
#define FM_RING_BITS        15
#define FM_RING_SIZE        (1 << FM_RING_BITS)   // 32768
#define FM_RING_MASK        (FM_RING_SIZE - 1)

// Sync NCC threshold.  Noise floor: std = 1/sqrt(1024) = 0.031.
// Peak noise over 100K trials (5s at 400K checks/s): ~4.5 sigma = 0.14.
// Real FLARM signals in captures show NCC 0.25-0.39.
// Threshold at 0.25 catches weak signals; CRC-16 provides final validation.
#define SYNC_NCC_THRESHOLD  0.25f

// Check correlation every N samples (timing resolution: +/-N/2 samples)
// Larger interval reduces CPU but may miss some peaks; refine compensates.
#define CORR_CHECK_INTERVAL 8

// After sync detection, refine position +/- this many samples
#define SYNC_REFINE_RANGE   8

// Minimum samples between sync detections to prevent re-triggering
// Short lockout: just skip past current sync pattern. The 'collecting' flag
// prevents re-detection during payload collection (6656+ samples).
#define SYNC_LOCKOUT_SAMPLES_DEFAULT (SYNC_TEMPLATE_LEN_DEFAULT + 128)

// DC-blocking filter coefficient (time constant ~6ms at 1.6 Msps)
#define DC_BLOCK_ALPHA      0.9999f

// ======================== Channelizer ========================

#define FLARM_NUM_CHANNELS   2

static const uint32_t FLARM_CHANNEL_FREQS[FLARM_NUM_CHANNELS] = {
    868200000,  // 868.2 MHz (EU freq 1)
    868400000,  // 868.4 MHz (EU freq 2)
};

// Channel filter: 65-tap FIR low-pass, Hamming window, 120 kHz cutoff.
// Linear phase: no waveform distortion.  Used for payload extraction only
// (NCC sync detection uses unfiltered FM for maximum correlation).
// Tight filtering maximizes noise rejection for bit decisions.
#define FIR_TAPS             65
#define CHANNEL_LPF_CUTOFF   120000.0

static float fir_coeffs[FIR_TAPS];  // shared by all channels (same filter)
static int fir_initialized = 0;

static void design_fir_lpf(float *h, int N, double cutoff_hz, double sample_rate)
{
    int M = (N - 1) / 2;
    double fc = cutoff_hz / sample_rate;  // normalized cutoff (0..0.5)
    for (int n = 0; n < N; n++) {
        // Hamming window
        double w = 0.54 - 0.46 * cos(2.0 * M_PI * n / (N - 1));
        // sinc
        double sinc_val;
        if (n == M) {
            sinc_val = 2.0 * fc;
        } else {
            double x = 2.0 * M_PI * fc * (n - M);
            sinc_val = sin(x) / (M_PI * (n - M));
        }
        h[n] = (float)(sinc_val * w);
    }
}

typedef struct {
    float delay_i[FIR_TAPS];
    float delay_q[FIR_TAPS];
    int pos;  // circular write position
} fir_state_t;

// ======================== Per-channel state ========================

typedef struct {
    // NCO (complex rotation)
    float nco_cos;           // current cos(phase)
    float nco_sin;           // current sin(phase)
    float nco_cos_inc;       // cos(phase_increment)
    float nco_sin_inc;       // sin(phase_increment)
    uint32_t nco_count;      // counter for periodic renormalization

    // LPF (FIR)
    fir_state_t fir;

    // FM discriminator (raw path — no FIR, for NCC sync detection)
    float prev_i_raw, prev_q_raw;

    // FM discriminator (filtered path — with FIR, for payload extraction)
    float prev_i_filt, prev_q_filt;

    // DC block
    float dc_avg;

    // FM sample ring buffers:
    //   fm_ring[]      = raw FM (no filter) — used for NCC sync detection
    //   fm_filt_ring[] = FIR-filtered FM — used for payload bit extraction
    float fm_ring[FM_RING_SIZE];
    float fm_filt_ring[FM_RING_SIZE];
    uint32_t ring_wr;

    // Correlation timing
    uint32_t corr_countdown;

    // Sync lockout
    uint32_t lockout_remaining;

    // FLARM packet state
    int      flarm_collecting;
    int      flarm_polarity;
    float    flarm_det_ncc;     // NCC at detection for diagnostics
    uint32_t flarm_payload_start;
    uint32_t flarm_samples_needed;

    // FLARM peak-hold sync detection (finds true NCC peak at low SNR)
    int      flarm_peak_searching;     // 1 = scanning for NCC peak after threshold crossing
    float    flarm_peak_best_ncc;      // best |NCC| found during peak search
    uint32_t flarm_peak_best_start;    // sync_start at best NCC position
    int      flarm_peak_countdown;     // remaining coarse checks in search window

    // OGNTP packet state
    int      ogntp_collecting;
    int      ogntp_polarity;
    uint32_t ogntp_payload_start;
    uint32_t ogntp_samples_needed;

    // ADS-L packet state
    int      adsl_collecting;
    int      adsl_polarity;
    uint32_t adsl_payload_start;
    uint32_t adsl_samples_needed;

    // ADS-L peak-hold sync detection (same strategy as FLARM)
    int      adsl_peak_searching;
    float    adsl_peak_best_ncc;
    uint32_t adsl_peak_best_start;
    int      adsl_peak_countdown;

    // Diagnostics
    float    diag_best_flarm_ncc;
    float    diag_best_ogntp_ncc;
    float    diag_best_adsl_ncc;
    float    diag_fm_sum;
    uint32_t diag_fm_count;
} flarm_channel_t;

// ======================== Overall state ========================

struct flarm_state {
    flarm_demod_config_t config;
    uint32_t sample_rate;           // actual SDR sample rate
    int      samples_per_bit;       // sample_rate / FLARM_CHIP_RATE
    int      sync_template_len;     // 64 * samples_per_bit
    int      payload_samples;       // PAYLOAD_CHIPS * samples_per_bit
    int      adsl_payload_samples;  // ADSL_PAYLOAD_CHIPS * samples_per_bit
    int      sync_lockout;          // sync_template_len + 128
    flarm_channel_t channels[FLARM_NUM_CHANNELS];
    uint64_t diag_report_interval;
    uint64_t diag_last_report;
    flarm_demod_stats_t stats;
    uint32_t time_override;  // If != 0, use this instead of time(NULL) for XXTEA
};

// ======================== FIR filter processing ========================

static inline void fir_process(fir_state_t *fir, float in_i, float in_q,
                               float *out_i, float *out_q)
{
    // Write new sample to delay line
    fir->delay_i[fir->pos] = in_i;
    fir->delay_q[fir->pos] = in_q;

    // Convolve: sum over taps
    float sum_i = 0, sum_q = 0;
    int idx = fir->pos;
    for (int k = 0; k < FIR_TAPS; k++) {
        sum_i += fir_coeffs[k] * fir->delay_i[idx];
        sum_q += fir_coeffs[k] * fir->delay_q[idx];
        idx--;
        if (idx < 0) idx = FIR_TAPS - 1;
    }

    *out_i = sum_i;
    *out_q = sum_q;

    fir->pos++;
    if (fir->pos >= FIR_TAPS) fir->pos = 0;
}

// ======================== Sync templates ========================

static float flarm_sync_template[SYNC_TEMPLATE_LEN_MAX];
static float ogntp_sync_template[SYNC_TEMPLATE_LEN_MAX];
static float adsl_sync_template[SYNC_TEMPLATE_LEN_MAX];
static float flarm_template_energy;   // sum(template²)
static float ogntp_template_energy;
static float adsl_template_energy;
static int templates_initialized = 0;
static int templates_spb = 0;          // samples_per_bit used for templates
static int templates_sync_len = 0;     // actual sync template length

// (No pre-filtering needed: FIR has linear phase, rectangular ±1 template works directly)

static void init_templates(uint32_t sample_rate)
{
    int spb = sample_rate / FLARM_CHIP_RATE;
    int sync_len = FLARM_SYNCWORD_SIZE * 8 * spb;

    // Reinitialize if sample rate changed
    if (templates_initialized && templates_spb == spb) return;

    // Initialize FIR coefficients (once, shared by all channels)
    if (!fir_initialized || templates_spb != spb) {
        design_fir_lpf(fir_coeffs, FIR_TAPS, CHANNEL_LPF_CUTOFF, (float)sample_rate);
        fir_initialized = 1;
    }

    // Generate ideal ±1 FLARM sync template
    uint64_t pattern = 0;
    for (int i = 0; i < FLARM_SYNCWORD_SIZE; i++)
        pattern = (pattern << 8) | FLARM_SYNCWORD[i];
    for (int chip = 0; chip < 64; chip++) {
        float val = ((pattern >> (63 - chip)) & 1) ? 1.0f : -1.0f;
        for (int s = 0; s < spb; s++)
            flarm_sync_template[chip * spb + s] = val;
    }

    // Generate ideal ±1 OGNTP sync template
    uint64_t ogntp_pattern = 0;
    for (int i = 0; i < OGNTP_SYNCWORD_SIZE; i++)
        ogntp_pattern = (ogntp_pattern << 8) | OGNTP_SYNCWORD[i];
    for (int chip = 0; chip < 64; chip++) {
        float val = ((ogntp_pattern >> (63 - chip)) & 1) ? 1.0f : -1.0f;
        for (int s = 0; s < spb; s++)
            ogntp_sync_template[chip * spb + s] = val;
    }

    // Generate ideal ±1 ADS-L sync template
    uint64_t adsl_pattern = 0;
    for (int i = 0; i < ADSL_SYNCWORD_SIZE; i++)
        adsl_pattern = (adsl_pattern << 8) | ADSL_SYNCWORD[i];
    for (int chip = 0; chip < 64; chip++) {
        float val = ((adsl_pattern >> (63 - chip)) & 1) ? 1.0f : -1.0f;
        for (int s = 0; s < spb; s++)
            adsl_sync_template[chip * spb + s] = val;
    }

    // Template energy = sync_len since all values are ±1
    flarm_template_energy = (float)sync_len;
    ogntp_template_energy = (float)sync_len;
    adsl_template_energy = (float)sync_len;

    templates_spb = spb;
    templates_sync_len = sync_len;
    templates_initialized = 1;
}

// ======================== NCC computation ========================
// Uses DC-robust formula: subtracts windowed mean from energy denominator.
// Since template is zero-mean (sum=0), numerator is unaffected by signal DC.
// NCC = Σ(s·t) / sqrt((Σs² - (Σs)²/N) · N)

static float compute_ncc_flat(const float *signal, const float *tmpl, float tmpl_energy, int tmpl_len)
{
    float corr = 0, energy = 0, sig_sum = 0;
    for (int i = 0; i < tmpl_len; i++) {
        float v = signal[i];
        corr += v * tmpl[i];
        energy += v * v;
        sig_sum += v;
    }
    // Remove DC component from energy: var = E[x²] - E[x]²
    float mean_sq = (sig_sum * sig_sum) / (float)tmpl_len;
    float var_energy = energy - mean_sq;
    if (var_energy < 1e-10f) return 0;
    return corr / sqrtf(var_energy * tmpl_energy);
}

static float compute_ncc_ring(const float *ring, uint32_t start,
                              const float *tmpl, float tmpl_energy, int tmpl_len)
{
    start &= FM_RING_MASK;
    if (start + (uint32_t)tmpl_len <= FM_RING_SIZE) {
        return compute_ncc_flat(&ring[start], tmpl, tmpl_energy, tmpl_len);
    }
    float temp[SYNC_TEMPLATE_LEN_MAX];
    uint32_t first_part = FM_RING_SIZE - start;
    memcpy(temp, &ring[start], first_part * sizeof(float));
    memcpy(temp + first_part, ring, ((uint32_t)tmpl_len - first_part) * sizeof(float));
    return compute_ncc_flat(temp, tmpl, tmpl_energy, tmpl_len);
}

static uint32_t refine_sync(const float *ring, uint32_t initial_start,
                            const float *tmpl, float tmpl_energy, int tmpl_len,
                            float *best_ncc_out)
{
    uint32_t best_pos = initial_start;
    float best_ncc = fabsf(compute_ncc_ring(ring, initial_start, tmpl, tmpl_energy, tmpl_len));

    for (int offset = -SYNC_REFINE_RANGE; offset <= SYNC_REFINE_RANGE; offset++) {
        if (offset == 0) continue;
        uint32_t pos = (initial_start + offset) & FM_RING_MASK;
        float ncc = fabsf(compute_ncc_ring(ring, pos, tmpl, tmpl_energy, tmpl_len));
        if (ncc > best_ncc) {
            best_ncc = ncc;
            best_pos = pos;
        }
    }
    *best_ncc_out = best_ncc;
    return best_pos;
}

// ======================== Payload extraction ========================

static void extract_payload_bits(const float *ring, uint32_t start, int polarity,
                                 uint8_t *manchester_bits, int samples_per_bit,
                                 int num_chips)
{
    // Compute local DC estimate over the full payload region.
    // Manchester encoding is balanced (equal +1 and -1 chips), so the mean ≈ DC offset.
    float dc_sum = 0;
    for (int s = 0; s < num_chips * samples_per_bit; s++) {
        dc_sum += ring[(start + s) & FM_RING_MASK];
    }
    float dc_offset = dc_sum / (float)(num_chips * samples_per_bit);

    for (int chip = 0; chip < num_chips; chip++) {
        uint32_t chip_start = (start + chip * samples_per_bit) & FM_RING_MASK;
        float acc = 0;
        for (int s = 0; s < samples_per_bit; s++) {
            acc += ring[(chip_start + s) & FM_RING_MASK];
        }
        acc -= dc_offset * samples_per_bit;  // remove DC bias
        manchester_bits[chip] = ((acc * polarity) > 0) ? 1 : 0;
    }
}

// ======================== Manchester decode ========================

static bool manchester_decode_payload(const uint8_t *manchester_bits, uint32_t n_bits,
                                      uint8_t *out_bytes, uint32_t *out_len,
                                      uint32_t *violations)
{
    uint32_t n_data_bits = n_bits / 2;
    uint32_t n_bytes = n_data_bits / 8;
    if (n_data_bits % 8 != 0) return false;

    *out_len = n_bytes;
    *violations = 0;
    memset(out_bytes, 0, n_bytes);

    for (uint32_t i = 0; i < n_data_bits; i++) {
        uint8_t first  = manchester_bits[i * 2];
        uint8_t second = manchester_bits[i * 2 + 1];

        uint8_t data_bit;
        if (first == 1 && second == 0)
            data_bit = 0;
        else if (first == 0 && second == 1)
            data_bit = 1;
        else {
            (*violations)++;
            data_bit = second;
        }

        out_bytes[i / 8] |= (data_bit << (7 - (i % 8)));
    }
    return true;
}

// ======================== Bit error correction ========================

// Try CRC with up to max_corrections bit flips on the payload (excluding CRC bytes).
// Returns the number of corrections applied (0 = no error, -1 = uncorrectable).
static int try_bit_correction(uint8_t *payload, uint32_t len, uint32_t max_corrections)
{
    // 0 corrections: direct check
    if (flarm_check_crc(payload, len))
        return 0;

    uint32_t data_bits = (len - 2) * 8;

    // 1-bit correction
    if (max_corrections >= 1) {
        for (uint32_t b1 = 0; b1 < data_bits; b1++) {
            payload[b1 / 8] ^= (0x80 >> (b1 % 8));
            if (flarm_check_crc(payload, len))
                return 1;
            payload[b1 / 8] ^= (0x80 >> (b1 % 8));  // undo
        }
    }

    // 2-bit correction
    if (max_corrections >= 2) {
        for (uint32_t b1 = 0; b1 < data_bits - 1; b1++) {
            payload[b1 / 8] ^= (0x80 >> (b1 % 8));
            for (uint32_t b2 = b1 + 1; b2 < data_bits; b2++) {
                payload[b2 / 8] ^= (0x80 >> (b2 % 8));
                if (flarm_check_crc(payload, len))
                    return 2;
                payload[b2 / 8] ^= (0x80 >> (b2 % 8));  // undo b2
            }
            payload[b1 / 8] ^= (0x80 >> (b1 % 8));  // undo b1
        }
    }

    return -1;  // uncorrectable
}

// ======================== Try decode FLARM ========================

// Attempt FLARM payload decode at a given ring buffer offset.
// Returns: number of corrections applied (0-2), or -1 if CRC failed.
static int try_decode_flarm_at_offset(struct flarm_state *state, flarm_channel_t *ch,
                                      uint32_t start, uint8_t *payload_out,
                                      uint32_t *violations_out)
{
    uint8_t manchester_bits[PAYLOAD_CHIPS];
    extract_payload_bits(ch->fm_filt_ring, start,
                         ch->flarm_polarity, manchester_bits, state->samples_per_bit,
                         PAYLOAD_CHIPS);

    uint32_t payload_len = 0;
    *violations_out = 0;

    if (!manchester_decode_payload(manchester_bits, PAYLOAD_CHIPS, payload_out,
                                   &payload_len, violations_out))
        return -1;
    if (payload_len < FLARM_PACKET_TOTAL)
        return -1;

    uint32_t max_corr = (*violations_out <= 5) ? 2 : 0;
    return try_bit_correction(payload_out, FLARM_PACKET_TOTAL, max_corr);
}

static void try_decode_flarm(struct flarm_state *state, flarm_channel_t *ch)
{
    uint8_t payload[FLARM_PACKET_TOTAL];
    uint32_t violations = 0;
    int corrections;

    // ---- Try OGN-TP LDPC first (same sync word, stricter check) ----
    // LDPC requires exact alignment (no corrections possible), so try multiple offsets.
    {
        static const int ogntp_offsets[] = { 0, 1, -1, 2, 3 };
        int samples_per_premanchbyte = 16 * state->samples_per_bit;

        for (int oi = 0; oi < 5; oi++) {
            uint32_t start = (ch->flarm_payload_start +
                             ogntp_offsets[oi] * samples_per_premanchbyte) & FM_RING_MASK;
            uint8_t manchester_bits[PAYLOAD_CHIPS];
            extract_payload_bits(ch->fm_filt_ring, start,
                                 ch->flarm_polarity, manchester_bits, state->samples_per_bit,
                                 PAYLOAD_CHIPS);
            uint8_t ogntp_payload[OGNTP_PACKET_TOTAL];
            uint32_t ogntp_len = 0;
            uint32_t ogntp_viol = 0;
            if (!manchester_decode_payload(manchester_bits, PAYLOAD_CHIPS, ogntp_payload,
                                           &ogntp_len, &ogntp_viol))
                continue;
            if (ogntp_len < OGNTP_PACKET_TOTAL)
                continue;
            // No violation threshold — LDPC check (48 parity equations) is strict enough
            if (ogntp_ldpc_check(ogntp_payload) != 0)
                continue;

            // LDPC passed — this is OGN-TP!
            state->stats.ogntp_packets_detected++;
            state->stats.ogntp_packets_ldpc_ok++;
            fprintf(stderr, "OGNTP LDPC OK! offset=%+d ncc=%.3f viol=%u payload=%02x%02x%02x%02x%02x%02x\n",
                    ogntp_offsets[oi], ch->flarm_det_ncc, ogntp_viol,
                    ogntp_payload[0], ogntp_payload[1], ogntp_payload[2],
                    ogntp_payload[3], ogntp_payload[4], ogntp_payload[5]);

            ogntp_message_t msg;
            if (ogntp_decode_packet(ogntp_payload, state->config.ref_lat,
                                    state->config.ref_lon, &msg)) {
                state->stats.ogntp_packets_decoded++;
                msg.signal_level = 0.0f;
                if (state->config.ogntp_callback)
                    state->config.ogntp_callback(&msg, state->config.ogntp_callback_ctx);
            }
            return;
        }
    }

    // ---- FLARM CRC-16 path ----
    // Try at the detected position first
    corrections = try_decode_flarm_at_offset(state, ch, ch->flarm_payload_start,
                                             payload, &violations);

    if (corrections < 0 && violations <= 5) {
        // CRC failed with low violations — NCC sync may be misaligned.
        // Try forward offsets in pre-Manchester byte steps (16 chips = 256 samples each).
        // Also try one step backward for robustness.
        static const int byte_offsets[] = { 1, 2, 3, -1 };
        int samples_per_premanchbyte = 16 * state->samples_per_bit;

        for (int i = 0; i < 4; i++) {
            uint32_t start = (ch->flarm_payload_start +
                             byte_offsets[i] * samples_per_premanchbyte) & FM_RING_MASK;
            uint32_t off_viol = 0;
            int off_corr = try_decode_flarm_at_offset(state, ch, start, payload, &off_viol);
            if (off_corr >= 0) {
                corrections = off_corr;
                violations = off_viol;
                fprintf(stderr, "flarm: sync realigned by %+d bytes (ncc=%.3f)\n",
                        byte_offsets[i], ch->flarm_det_ncc);
                break;
            }
        }
    }

    if (corrections < 0) {
        // CRC failed. Type=3/4 packets use an undocumented protocol variant
        // with unknown encryption keys - log for analysis but cannot decode
        uint8_t type_field = (payload[3] & 0x0F);
        if (violations <= 2 && (type_field == 3 || type_field == 4)) {
            uint32_t addr = payload[0] | (payload[1] << 8) | (payload[2] << 16);
            if (type_field == 3) {
                state->stats.packets_type3++;
                fprintf(stderr, "flarm: type=3 addr=%06X (undocumented protocol, not decodable)\n", addr);
            } else {
                state->stats.packets_type4++;
                fprintf(stderr, "flarm: type=%u addr=%06X (unknown protocol, not decodable)\n",
                        type_field, addr);
            }
            state->stats.packets_failed++;
            return;
        }
        // Debug: dump failed payload for analysis
        fprintf(stderr, "flarm-crc-fail ncc=%.3f pol=%d viol=%u start=%u bytes=%02x%02x%02x%02x%02x%02x%02x%02x"
                "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
                ch->flarm_det_ncc, ch->flarm_polarity, violations,
                ch->flarm_payload_start,
                payload[0],payload[1],payload[2],payload[3],payload[4],payload[5],
                payload[6],payload[7],payload[8],payload[9],payload[10],payload[11],
                payload[12],payload[13],payload[14],payload[15],payload[16],payload[17],
                payload[18],payload[19],payload[20],payload[21],payload[22],payload[23],
                payload[24],payload[25]);
        state->stats.packets_failed++;
        return;
    }

    state->stats.packets_crc_ok++;
    fprintf(stderr, "FLARM CRC OK! corrections=%d ncc=%.3f payload=%02x%02x%02x%02x%02x%02x "
            "(detected=%" PRIu64 " crc_ok=%" PRIu64 ")\n",
            corrections, ch->flarm_det_ncc,
            payload[0], payload[1], payload[2], payload[3], payload[4], payload[5],
            (uint64_t)state->stats.packets_detected,
            (uint64_t)state->stats.packets_crc_ok);

    uint32_t now = state->time_override ? state->time_override : (uint32_t)time(NULL);
    flarm_message_t msg;

    // When replaying a file, scan a wider time window (file mtime = start of capture)
    // File duration up to ~120s + GPS/UTC offset ~18s + margin → scan ±150s
    // V7 tstamp field validation prevents false positives despite wide range
    int scan_range = state->time_override ? 150 : 1;

    for (int dt = 0; dt <= scan_range; dt++) {
        // Try dt=0 first, then -1,+1,-2,+2,...
        int offsets[2] = { -dt, dt };
        int n_offsets = (dt == 0) ? 1 : 2;
        for (int k = 0; k < n_offsets; k++) {
            if (flarm_decode_packet(payload, state->config.ref_lat, state->config.ref_lon,
                                    state->config.ref_alt_geoid, now + offsets[k], &msg)) {
                msg.signal_level = 0;

                // Address sanity checks (apply to all types)
                if (msg.addr == 0x000000 || msg.addr == 0xFFFFFF) continue;
                if ((msg.addr & 0xFF) == ((msg.addr >> 8) & 0xFF) &&
                    (msg.addr & 0xFF) == ((msg.addr >> 16) & 0xFF)) continue;

                state->stats.packets_decoded++;

                // Type 1 = non-traffic message (header-only, no position)
                // Cannot validate position fields — deliver based on address only
                if (msg.header_only) {
                    state->stats.packets_type1++;
                    fprintf(stderr, "FLARM TYPE1 addr=%06X addr_type=%u (non-traffic message, header only)\n",
                            msg.addr, msg.addr_type);
                    if (state->config.callback)
                        state->config.callback(&msg, state->config.callback_ctx);
                    return;
                }

                // Position validation for type 0/2: wrong timestamp produces garbage
                // Skip stealth check for file replay (time_override != 0)
                if (msg.stealth && !state->time_override) continue;
                if (msg.altitude < -500 || msg.altitude > 15000) continue;
                if (msg.speed < 0 || msg.speed > 150) continue;  // 150 m/s ≈ 540 km/h max
                // Distance filter: max ~3° (≈330km) from receiver
                {
                    double dlat = msg.latitude - state->config.ref_lat;
                    double dlon = msg.longitude - state->config.ref_lon;
                    if (sqrt(dlat*dlat + dlon*dlon) > 3.0) continue;
                }

                fprintf(stderr, "FLARM DECODED! addr=%06X lat=%.4f lon=%.4f alt=%d vs=%.0f spd=%.0f dt=%d\n",
                        msg.addr, msg.latitude, msg.longitude,
                        msg.altitude, (double)msg.vs, msg.speed, offsets[k]);

                if (state->config.callback)
                    state->config.callback(&msg, state->config.callback_ctx);
                return;
            }
        }
    }
    state->stats.packets_failed++;
}

// ======================== Try decode OGNTP ========================

static void try_decode_ogntp(struct flarm_state *state, flarm_channel_t *ch)
{
    uint8_t manchester_bits[PAYLOAD_CHIPS];
    extract_payload_bits(ch->fm_filt_ring, ch->ogntp_payload_start,
                         ch->ogntp_polarity, manchester_bits, state->samples_per_bit,
                         PAYLOAD_CHIPS);

    uint8_t payload[OGNTP_PACKET_TOTAL];
    uint32_t payload_len = 0;

    uint32_t ogntp_violations = 0;
    if (!manchester_decode_payload(manchester_bits, PAYLOAD_CHIPS, payload, &payload_len, &ogntp_violations)) {
        state->stats.ogntp_packets_failed++;
        return;
    }
    if (payload_len < OGNTP_PACKET_TOTAL) {
        state->stats.ogntp_packets_failed++;
        return;
    }

    if (ogntp_ldpc_check(payload) != 0) {
        state->stats.ogntp_packets_failed++;
        return;
    }
    state->stats.ogntp_packets_ldpc_ok++;

    ogntp_message_t msg;
    if (ogntp_decode_packet(payload, state->config.ref_lat, state->config.ref_lon, &msg)) {
        state->stats.ogntp_packets_decoded++;
        msg.signal_level = 0.0f;
        if (state->config.ogntp_callback)
            state->config.ogntp_callback(&msg, state->config.ogntp_callback_ctx);
    } else {
        state->stats.ogntp_packets_failed++;
    }
}

// ======================== Try decode ADS-L ========================

static void try_decode_adsl(struct flarm_state *state, flarm_channel_t *ch)
{
    uint8_t manchester_bits[ADSL_PAYLOAD_CHIPS];
    extract_payload_bits(ch->fm_filt_ring, ch->adsl_payload_start,
                         ch->adsl_polarity, manchester_bits, state->samples_per_bit,
                         ADSL_PAYLOAD_CHIPS);

    uint8_t payload[ADSL_PACKET_TOTAL];
    uint32_t payload_len = 0;
    uint32_t violations = 0;

    if (!manchester_decode_payload(manchester_bits, ADSL_PAYLOAD_CHIPS, payload, &payload_len, &violations)) {
        fprintf(stderr, "ADSL-DBG manchester fail len=%u viol=%u\n", payload_len, violations);
        state->stats.adsl_packets_failed++;
        return;
    }
    if (payload_len < ADSL_PACKET_TOTAL) {
        fprintf(stderr, "ADSL-DBG short payload len=%u (need %d)\n", payload_len, ADSL_PACKET_TOTAL);
        state->stats.adsl_packets_failed++;
        return;
    }
    fprintf(stderr, "ADSL-DBG manchester OK len=%u viol=%u pol=%d\n", payload_len, violations, ch->adsl_polarity);

    // ADS-L payload is INVERTED on-air: flip all bytes
    for (uint32_t i = 0; i < ADSL_PACKET_TOTAL; i++) {
        payload[i] ^= 0xFF;
    }

    // CRC-24 check
    fprintf(stderr, "ADSL-DBG pre-crc: %02X %02X %02X %02X %02X %02X %02X %02X ... %02X %02X %02X\n",
            payload[0], payload[1], payload[2], payload[3],
            payload[4], payload[5], payload[6], payload[7],
            payload[ADSL_PACKET_TOTAL-3], payload[ADSL_PACKET_TOTAL-2], payload[ADSL_PACKET_TOTAL-1]);
    if (!adsl_check_crc(payload)) {
        fprintf(stderr, "ADSL-DBG CRC-24 FAIL\n");
        state->stats.adsl_packets_failed++;
        return;
    }
    fprintf(stderr, "ADSL-DBG CRC-24 PASS!\n");
    state->stats.adsl_packets_crc_ok++;

    adsl_message_t msg;
    if (adsl_decode_packet(payload, state->config.ref_lat, state->config.ref_lon,
                           state->config.ref_alt_geoid, &msg)) {
        state->stats.adsl_packets_decoded++;
        msg.signal_level = 0.0f;

        fprintf(stderr, "ADSL DECODED! addr=%06X lat=%.4f lon=%.4f alt=%d spd=%.1f crs=%.0f vs=%.1f\n",
                msg.addr, msg.latitude, msg.longitude, msg.altitude,
                (double)msg.speed, (double)msg.course, (double)msg.vs);

        if (state->config.adsl_callback)
            state->config.adsl_callback(&msg, state->config.adsl_callback_ctx);
    } else {
        state->stats.adsl_packets_failed++;
    }
}

// ======================== Create / Destroy ========================

struct flarm_state *flarm_demod_create(const flarm_demod_config_t *config)
{
    struct flarm_state *s = calloc(1, sizeof(struct flarm_state));
    if (!s) return NULL;

    s->config = *config;

    // Determine actual sample rate
    uint32_t sr = config->sample_rate;
    if (sr == 0) sr = FLARM_SAMPLE_RATE;
    s->sample_rate = sr;

    // Compute runtime timing parameters
    s->samples_per_bit = sr / FLARM_CHIP_RATE;
    s->sync_template_len = FLARM_SYNCWORD_SIZE * 8 * s->samples_per_bit;
    s->payload_samples = PAYLOAD_CHIPS * s->samples_per_bit;
    s->adsl_payload_samples = ADSL_PAYLOAD_CHIPS * s->samples_per_bit;
    s->sync_lockout = s->sync_template_len + 128;

    init_templates(sr);

    uint32_t center = config->center_freq;
    if (center == 0) center = FLARM_CENTER_FREQ;

    for (int ch = 0; ch < FLARM_NUM_CHANNELS; ch++) {
        flarm_channel_t *c = &s->channels[ch];

        double offset_hz = (double)FLARM_CHANNEL_FREQS[ch] - (double)center;
        double phase_inc = -2.0 * M_PI * offset_hz / (double)sr;
        c->nco_cos = 1.0f;
        c->nco_sin = 0.0f;
        c->nco_cos_inc = (float)cos(phase_inc);
        c->nco_sin_inc = (float)sin(phase_inc);
        c->nco_count = 0;

        memset(&c->fir, 0, sizeof(c->fir));
        c->fir.pos = 0;

        c->prev_i_raw = 0;
        c->prev_q_raw = 0;
        c->prev_i_filt = 0;
        c->prev_q_filt = 0;
        c->dc_avg = 0;

        memset(c->fm_ring, 0, sizeof(c->fm_ring));
        memset(c->fm_filt_ring, 0, sizeof(c->fm_filt_ring));
        c->ring_wr = 0;
        c->corr_countdown = CORR_CHECK_INTERVAL;
        c->lockout_remaining = 0;

        c->flarm_collecting = 0;
        c->flarm_peak_searching = 0;
        c->ogntp_collecting = 0;
        c->adsl_collecting = 0;

        c->diag_best_flarm_ncc = 0;
        c->diag_best_ogntp_ncc = 0;
        c->diag_best_adsl_ncc = 0;
        c->diag_fm_sum = 0;
        c->diag_fm_count = 0;

        fprintf(stderr, "flarm-ch%d: %.3f MHz, NCO offset %+.1f kHz, LPF %.0f kHz, "
                "NCC threshold %.2f, %d samp/bit, check every %d samp\n",
                ch, FLARM_CHANNEL_FREQS[ch] / 1e6, offset_hz / 1e3,
                CHANNEL_LPF_CUTOFF / 1e3, SYNC_NCC_THRESHOLD, s->samples_per_bit, CORR_CHECK_INTERVAL);
    }

    s->diag_report_interval = 5 * (uint64_t)sr;
    s->diag_last_report = 0;

    return s;
}

void flarm_demod_destroy(struct flarm_state *state)
{
    free(state);
}

void flarm_demod_set_position(struct flarm_state *state, double lat, double lon, float alt_geoid)
{
    state->config.ref_lat = lat;
    state->config.ref_lon = lon;
    state->config.ref_alt_geoid = alt_geoid;
}

void flarm_demod_set_time_override(struct flarm_state *state, uint32_t base_time)
{
    state->time_override = base_time;
}

void flarm_demod_get_stats(struct flarm_state *state, flarm_demod_stats_t *stats)
{
    *stats = state->stats;
}

// ======================== Main IQ processing ========================

void flarm_demod_process(struct flarm_state *state, const uint8_t *iq_data, uint32_t len)
{
    uint32_t n_samples = len / 2;
    state->stats.samples_processed += n_samples;

    for (uint32_t i = 0; i < n_samples; i++) {
        float raw_i = (float)((int)iq_data[i * 2]     - 128);
        float raw_q = (float)((int)iq_data[i * 2 + 1] - 128);

        for (int ch_idx = 0; ch_idx < FLARM_NUM_CHANNELS; ch_idx++) {
            flarm_channel_t *ch = &state->channels[ch_idx];

            // ---- NCO mixing (complex rotation) ----
            float mix_i = raw_i * ch->nco_cos - raw_q * ch->nco_sin;
            float mix_q = raw_i * ch->nco_sin + raw_q * ch->nco_cos;

            // Advance NCO phase via complex multiplication
            float new_cos = ch->nco_cos * ch->nco_cos_inc - ch->nco_sin * ch->nco_sin_inc;
            float new_sin = ch->nco_sin * ch->nco_cos_inc + ch->nco_cos * ch->nco_sin_inc;
            ch->nco_cos = new_cos;
            ch->nco_sin = new_sin;

            // Periodic renormalization to prevent amplitude drift
            if (++ch->nco_count >= 1024) {
                float mag = ch->nco_cos * ch->nco_cos + ch->nco_sin * ch->nco_sin;
                float scale = 1.5f - 0.5f * mag;  // fast inverse sqrt approximation
                ch->nco_cos *= scale;
                ch->nco_sin *= scale;
                ch->nco_count = 0;
            }

            // ---- FIR LPF (linear phase) ----
            float filt_i, filt_q;
            fir_process(&ch->fir, mix_i, mix_q, &filt_i, &filt_q);

            // ---- FM discriminator (RAW path — no filter, for NCC) ----
            float cross_raw = mix_q * ch->prev_i_raw - mix_i * ch->prev_q_raw;
            float dot_raw   = mix_i * ch->prev_i_raw + mix_q * ch->prev_q_raw;
            ch->prev_i_raw = mix_i;
            ch->prev_q_raw = mix_q;
            float fm_raw = atan2f(cross_raw, dot_raw);

            // ---- FM discriminator (FILTERED path — with FIR, for payload) ----
            float cross_filt = filt_q * ch->prev_i_filt - filt_i * ch->prev_q_filt;
            float dot_filt   = filt_i * ch->prev_i_filt + filt_q * ch->prev_q_filt;
            ch->prev_i_filt = filt_i;
            ch->prev_q_filt = filt_q;
            float fm_filt = atan2f(cross_filt, dot_filt);

            // ---- Store raw FM in ring buffer (no DC block) ----
            // DC is handled per-window in NCC (robust formula) and in payload extraction
            ch->dc_avg = DC_BLOCK_ALPHA * ch->dc_avg + (1.0f - DC_BLOCK_ALPHA) * fm_raw;  // track for diagnostics only

            // ---- Store in ring buffers ----
            ch->fm_ring[ch->ring_wr] = fm_raw;
            ch->fm_filt_ring[ch->ring_wr] = fm_filt;
            ch->ring_wr = (ch->ring_wr + 1) & FM_RING_MASK;

            // ---- Diagnostics ----
            ch->diag_fm_sum += fabsf(fm_raw);
            ch->diag_fm_count++;

            // ---- Payload collection countdown ----
            if (ch->flarm_collecting) {
                ch->flarm_samples_needed--;
                if (ch->flarm_samples_needed == 0) {
                    try_decode_flarm(state, ch);
                    ch->flarm_collecting = 0;
                }
            }
            if (ch->ogntp_collecting) {
                ch->ogntp_samples_needed--;
                if (ch->ogntp_samples_needed == 0) {
                    try_decode_ogntp(state, ch);
                    ch->ogntp_collecting = 0;
                }
            }
            if (ch->adsl_collecting) {
                ch->adsl_samples_needed--;
                if (ch->adsl_samples_needed == 0) {
                    try_decode_adsl(state, ch);
                    ch->adsl_collecting = 0;
                }
            }

            // ---- Lockout (FLARM only — other protocols check independently) ----
            int flarm_locked_out = 0;
            if (ch->lockout_remaining > 0) {
                ch->lockout_remaining--;
                flarm_locked_out = 1;
            }

            // ---- Periodic NCC check ----
            ch->corr_countdown--;
            if (ch->corr_countdown > 0)
                continue;
            ch->corr_countdown = CORR_CHECK_INTERVAL;

            int stl = state->sync_template_len;

            // Sync would end at current ring_wr, starts sync_template_len before
            uint32_t sync_start = (ch->ring_wr - stl) & FM_RING_MASK;

            // ---- FLARM sync detection (blocked during lockout) ----
            if (!flarm_locked_out && !ch->flarm_collecting) {
                float ncc = compute_ncc_ring(ch->fm_ring, sync_start,
                                            flarm_sync_template, flarm_template_energy, stl);
                float abs_ncc = fabsf(ncc);

                if (abs_ncc > ch->diag_best_flarm_ncc)
                    ch->diag_best_flarm_ncc = abs_ncc;

                if (ch->flarm_peak_searching) {
                    // Continue peak search — track maximum NCC position
                    if (abs_ncc > ch->flarm_peak_best_ncc) {
                        ch->flarm_peak_best_ncc = abs_ncc;
                        ch->flarm_peak_best_start = sync_start;
                    }
                    ch->flarm_peak_countdown--;

                    // End search when countdown expires or NCC drops well below peak
                    if (ch->flarm_peak_countdown <= 0 ||
                        (abs_ncc < SYNC_NCC_THRESHOLD * 0.8f &&
                         abs_ncc < ch->flarm_peak_best_ncc * 0.7f)) {

                        // Refine peak to single-sample accuracy
                        float refined_ncc;
                        uint32_t best_start = refine_sync(ch->fm_ring,
                                                         ch->flarm_peak_best_start,
                                                         flarm_sync_template,
                                                         flarm_template_energy, stl,
                                                         &refined_ncc);
                        float signed_ncc = compute_ncc_ring(ch->fm_ring, best_start,
                                                           flarm_sync_template,
                                                           flarm_template_energy, stl);

                        state->stats.packets_detected++;
                        ch->flarm_polarity = (signed_ncc > 0) ? 1 : -1;
                        ch->flarm_det_ncc = refined_ncc;
                        ch->flarm_payload_start = (best_start + stl + (FIR_TAPS-1)/2) & FM_RING_MASK;
                        ch->lockout_remaining = state->sync_lockout;
                        ch->flarm_peak_searching = 0;

                        // Compute remaining samples (ring_wr advanced during peak search)
                        // Add margin for sync realignment offset search (+3 pre-Manchester bytes)
                        uint32_t offset_margin = 3 * 16 * (uint32_t)state->samples_per_bit;
                        uint32_t payload_end = (ch->flarm_payload_start +
                                               (uint32_t)state->payload_samples +
                                               offset_margin) & FM_RING_MASK;
                        uint32_t remaining = (payload_end - ch->ring_wr) & FM_RING_MASK;
                        if (remaining == 0 ||
                            remaining > (uint32_t)(state->payload_samples + stl + offset_margin)) {
                            // ring_wr already past payload end — process immediately
                            try_decode_flarm(state, ch);
                        } else {
                            ch->flarm_collecting = 1;
                            ch->flarm_samples_needed = remaining;
                        }
                    }
                } else if (abs_ncc >= SYNC_NCC_THRESHOLD) {
                    // First threshold crossing — start peak search
                    ch->flarm_peak_searching = 1;
                    ch->flarm_peak_best_ncc = abs_ncc;
                    ch->flarm_peak_best_start = sync_start;
                    // Search for one full sync template length (128 coarse checks)
                    ch->flarm_peak_countdown = stl / CORR_CHECK_INTERVAL;
                }
            }

            // ---- OGNTP sync detection ----
            if (!ch->ogntp_collecting) {
                float oncc = compute_ncc_ring(ch->fm_ring, sync_start,
                                             ogntp_sync_template, ogntp_template_energy, stl);
                float abs_oncc = fabsf(oncc);

                if (abs_oncc > ch->diag_best_ogntp_ncc)
                    ch->diag_best_ogntp_ncc = abs_oncc;

                if (abs_oncc >= SYNC_NCC_THRESHOLD) {
                    float refined_ncc;
                    uint32_t best_start = refine_sync(ch->fm_ring, sync_start,
                                                     ogntp_sync_template,
                                                     ogntp_template_energy, stl, &refined_ncc);
                    float signed_ncc = compute_ncc_ring(ch->fm_ring, best_start,
                                                       ogntp_sync_template,
                                                       ogntp_template_energy, stl);

                    state->stats.ogntp_packets_detected++;
                    ch->ogntp_collecting = 1;
                    ch->ogntp_polarity = (signed_ncc > 0) ? 1 : -1;
                    // Compensate FIR group delay: filtered ring is delayed by (FIR_TAPS-1)/2 samples
                    ch->ogntp_payload_start = (best_start + stl + (FIR_TAPS-1)/2) & FM_RING_MASK;
                    ch->ogntp_samples_needed = state->payload_samples;
                }
            }

            // ---- ADS-L sync detection ----
            if (!ch->adsl_collecting) {
                float ancc = compute_ncc_ring(ch->fm_ring, sync_start,
                                             adsl_sync_template, adsl_template_energy, stl);
                float abs_ancc = fabsf(ancc);

                if (abs_ancc > ch->diag_best_adsl_ncc)
                    ch->diag_best_adsl_ncc = abs_ancc;

                if (ch->adsl_peak_searching) {
                    // Continue peak search — track maximum NCC position
                    if (abs_ancc > ch->adsl_peak_best_ncc) {
                        ch->adsl_peak_best_ncc = abs_ancc;
                        ch->adsl_peak_best_start = sync_start;
                    }
                    ch->adsl_peak_countdown--;

                    // End search when countdown expires or NCC drops well below peak
                    if (ch->adsl_peak_countdown <= 0 ||
                        (abs_ancc < SYNC_NCC_THRESHOLD * 0.8f &&
                         abs_ancc < ch->adsl_peak_best_ncc * 0.7f)) {

                        // Refine peak to single-sample accuracy
                        float refined_ncc;
                        uint32_t best_start = refine_sync(ch->fm_ring,
                                                         ch->adsl_peak_best_start,
                                                         adsl_sync_template,
                                                         adsl_template_energy, stl,
                                                         &refined_ncc);
                        float signed_ncc = compute_ncc_ring(ch->fm_ring, best_start,
                                                           adsl_sync_template,
                                                           adsl_template_energy, stl);

                        fprintf(stderr, "ADSL-DBG peak ch=%d coarse=%.3f refined=%.3f start=%u\n",
                                ch_idx, ch->adsl_peak_best_ncc, refined_ncc, best_start);

                        state->stats.adsl_packets_detected++;
                        ch->adsl_polarity = (signed_ncc > 0) ? 1 : -1;
                        ch->adsl_payload_start = (best_start + stl + (FIR_TAPS-1)/2) & FM_RING_MASK;
                        ch->adsl_peak_searching = 0;

                        // Compute remaining samples for payload
                        uint32_t payload_end = (ch->adsl_payload_start +
                                               (uint32_t)state->adsl_payload_samples) & FM_RING_MASK;
                        uint32_t remaining = (payload_end - ch->ring_wr) & FM_RING_MASK;
                        if (remaining == 0 ||
                            remaining > (uint32_t)(state->adsl_payload_samples + stl)) {
                            // ring_wr already past payload end — process immediately
                            try_decode_adsl(state, ch);
                        } else {
                            ch->adsl_collecting = 1;
                            ch->adsl_samples_needed = remaining;
                        }
                    }
                } else if (abs_ancc >= SYNC_NCC_THRESHOLD) {
                    // First threshold crossing — start peak search
                    ch->adsl_peak_searching = 1;
                    ch->adsl_peak_best_ncc = abs_ancc;
                    ch->adsl_peak_best_start = sync_start;
                    // Search for one full sync template length
                    ch->adsl_peak_countdown = stl / CORR_CHECK_INTERVAL;
                }
            }
        }
    }

    // ---- Periodic diagnostic report ----
    if (state->stats.samples_processed - state->diag_last_report >= state->diag_report_interval) {
        for (int ch = 0; ch < FLARM_NUM_CHANNELS; ch++) {
            flarm_channel_t *c = &state->channels[ch];
            float avg_fm = (c->diag_fm_count > 0) ?
                            c->diag_fm_sum / c->diag_fm_count : 0;
            fprintf(stderr, "flarm-diag ch%d (%.1fMHz): best_ncc=%.3f ogntp_ncc=%.3f adsl_ncc=%.3f "
                    "avg_fm=%.4f dc=%.4f det=%" PRIu64 " crc=%" PRIu64 " fail=%" PRIu64 "\n",
                    ch, FLARM_CHANNEL_FREQS[ch] / 1e6,
                    c->diag_best_flarm_ncc, c->diag_best_ogntp_ncc, c->diag_best_adsl_ncc,
                    avg_fm, c->dc_avg,
                    (uint64_t)state->stats.packets_detected,
                    (uint64_t)state->stats.packets_crc_ok,
                    (uint64_t)state->stats.packets_failed);
            c->diag_best_flarm_ncc = 0;
            c->diag_best_ogntp_ncc = 0;
            c->diag_best_adsl_ncc = 0;
            c->diag_fm_sum = 0;
            c->diag_fm_count = 0;
        }
        state->diag_last_report = state->stats.samples_processed;
    }
}
