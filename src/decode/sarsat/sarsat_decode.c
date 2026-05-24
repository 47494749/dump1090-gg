// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sarsat_decode.c: Cospas-Sarsat 406 MHz beacon decoder
//
// Signal processing chain:
//   IQ samples (2.4 MSPS, uint8_t) → FM discriminator → decimation (factor 60)
//   → DC removal → AGC → PLL bit clock recovery at 400 baud
//   → Biphase-L (Manchester) decoding → preamble/sync detection
//   → BCH error correction → protocol field parsing → callback
//
// Protocol: Cospas-Sarsat C/S T.001 (1st-generation 406 MHz beacons)
//   - FM deviation: ±3.5 kHz
//   - Biphase-L encoding at 400 baud (800 half-symbols/s)
//   - Preamble: 15 bits of unmodulated carrier (all 1s after Biphase-L)
//   - Frame sync: 9 bits (011010000 = normal, 000101111 = test)
//   - PDF-1: 61 data bits → BCH(82,61) t=3
//   - PDF-2: 26 data bits → BCH(38,26) t=2 (long format only)
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "sarsat_decode.h"

// ======================== DSP Constants ========================

// Decimation: 2.4 MHz → 40 kHz (factor 60)
// Need at least 20× oversampling for 800 half-symbol/s
#define SARSAT_DECIM          60
#define SARSAT_IF_RATE        (SARSAT_SAMPLE_RATE / SARSAT_DECIM)  // 40000 Hz
#define SARSAT_HALF_SYM_RATE  (SARSAT_BAUD_RATE * 2)               // 800 half-sym/s
#define SARSAT_SPS            ((double)SARSAT_IF_RATE / SARSAT_HALF_SYM_RATE) // 50 samples/half-sym

// FM discriminator gain
#define FM_GAIN               0.8f

// Lowpass filter
#define SARSAT_LPF_TAPS       21

// PLL bit clock recovery
#define PLL_BW                0.008f
#define PLL_DAMP              0.707f

// Preamble detection: expect ~15 consecutive 1-bits in Biphase-L
#define PREAMBLE_MIN_ONES     12

// Minimum burst energy threshold (normalized)
#define BURST_SQUELCH         0.02f

// Maximum frame buffer for collecting bits
#define MAX_FRAME_BITS        200

// BCH generator polynomials (per C/S T.001 Annex B)
// BCH(82,61) shortened from BCH(127,106), t=3
// g(x) = m1(x)*m3(x)*m5(x) with primitive poly p(x) = x^7 + x^3 + 1
// Per C/S T.001 Annex B
#define BCH1_POLY             0x26D9E3u  // degree 21
#define BCH1_DEG              21
#define BCH1_N                82         // 61 data + 21 parity
#define BCH1_T                3          // Corrects up to 3 errors

// BCH(38,26) shortened from BCH(63,51), t=2
#define BCH2_POLY             0x1539u    // x^12 + ... (12 bits)
#define BCH2_DEG              12
#define BCH2_N                38         // 26 data + 12 parity
#define BCH2_T                2          // Corrects up to 2 errors

// ======================== Country code table ========================

static const struct { int code; const char *name; } country_table[] = {
    // Europe 201-279
    {201,"Albania"},{203,"Austria"},{205,"Belgium"},{206,"Belarus"},
    {207,"Bulgaria"},{209,"Cyprus"},{211,"Germany"},{213,"Georgia"},
    {214,"Moldova"},{215,"Malta"},{216,"Armenia"},{219,"Denmark"},
    {220,"Denmark"},{224,"Spain"},{225,"Spain"},{226,"France"},
    {227,"France"},{228,"France"},{229,"Malta"},{230,"Finland"},
    {231,"Faroe Islands"},{232,"United Kingdom"},{233,"United Kingdom"},
    {234,"United Kingdom"},{235,"United Kingdom"},{236,"Gibraltar"},
    {237,"Greece"},{238,"Croatia"},{239,"Greece"},{240,"Greece"},
    {242,"Morocco"},{243,"Hungary"},{244,"Netherlands"},{245,"Netherlands"},
    {246,"Netherlands"},{247,"Italy"},{248,"Malta"},{249,"Malta"},
    {250,"Ireland"},{251,"Iceland"},{252,"Liechtenstein"},{253,"Luxembourg"},
    {254,"Monaco"},{255,"Madeira"},{257,"Norway"},{258,"Norway"},
    {259,"Norway"},{261,"Poland"},{262,"Montenegro"},{263,"Portugal"},
    {264,"Romania"},{265,"Sweden"},{266,"Sweden"},{267,"Slovakia"},
    {268,"San Marino"},{269,"Switzerland"},{270,"Czech Republic"},
    {271,"Turkey"},{272,"Ukraine"},{273,"Russia"},{274,"North Macedonia"},
    {275,"Latvia"},{276,"Estonia"},{277,"Lithuania"},{278,"Slovenia"},
    {279,"Serbia"},
    // Americas 301-379
    {303,"Alaska"},{304,"Antigua"},{308,"Bahamas"},{312,"Belize"},
    {314,"Barbados"},{316,"Canada"},{319,"Cayman Islands"},
    {321,"Costa Rica"},{323,"Cuba"},{325,"Dominica"},
    {327,"Dominican Republic"},{330,"Grenada"},{331,"Greenland"},
    {332,"Guatemala"},{334,"Honduras"},{336,"Haiti"},{338,"USA"},
    {339,"Jamaica"},{345,"Mexico"},{350,"Nicaragua"},
    {351,"Panama"},{352,"Panama"},{353,"Panama"},{354,"Panama"},
    {355,"Panama"},{356,"Panama"},{357,"Panama"},{358,"Puerto Rico"},
    {359,"El Salvador"},{362,"Trinidad and Tobago"},{366,"USA"},
    {367,"USA"},{368,"USA"},{369,"USA"},{370,"Panama"},
    {371,"Panama"},{372,"Panama"},{373,"Panama"},{374,"Panama"},
    {379,"US Virgin Islands"},
    // Asia 401-499
    {401,"Afghanistan"},{403,"Saudi Arabia"},{405,"Bangladesh"},
    {408,"Bahrain"},{412,"China"},{413,"China"},{414,"China"},
    {416,"Taiwan"},{417,"Sri Lanka"},{419,"India"},{422,"Iran"},
    {425,"Iraq"},{428,"Israel"},{431,"Japan"},{432,"Japan"},
    {436,"Kazakhstan"},{438,"Jordan"},{440,"South Korea"},
    {441,"South Korea"},{445,"North Korea"},{447,"Kuwait"},
    {450,"Lebanon"},{455,"Maldives"},{457,"Mongolia"},
    {461,"Oman"},{463,"Pakistan"},{466,"Qatar"},
    {470,"UAE"},{471,"UAE"},{477,"Hong Kong"},
    // Pacific 501-579
    {501,"Adelie Land"},{503,"Australia"},{506,"Myanmar"},
    {508,"Brunei"},{512,"New Zealand"},{514,"Cambodia"},
    {525,"Indonesia"},{531,"Laos"},{533,"Malaysia"},
    {538,"Marshall Islands"},{540,"New Caledonia"},
    {548,"Philippines"},{553,"Papua New Guinea"},
    {557,"Solomon Islands"},{561,"Samoa"},{563,"Singapore"},
    {564,"Singapore"},{567,"Thailand"},{570,"Tonga"},
    {574,"Vietnam"},{576,"Vanuatu"},
    // Africa 601-679
    {601,"South Africa"},{603,"Angola"},{605,"Algeria"},
    {610,"Benin"},{611,"Botswana"},{613,"Cameroon"},
    {617,"Cape Verde"},{619,"Ivory Coast"},{621,"Djibouti"},
    {622,"Egypt"},{624,"Ethiopia"},{626,"Gabon"},{627,"Ghana"},
    {632,"Guinea"},{634,"Kenya"},{636,"Liberia"},{637,"Liberia"},
    {642,"Libya"},{645,"Mauritius"},{647,"Madagascar"},
    {649,"Mali"},{650,"Mozambique"},{655,"Malawi"},
    {657,"Nigeria"},{659,"Namibia"},{661,"Rwanda"},
    {662,"Sudan"},{663,"Senegal"},{664,"Seychelles"},
    {666,"Somalia"},{667,"Sierra Leone"},{672,"Tunisia"},
    {674,"Tanzania"},{675,"Uganda"},{676,"DR Congo"},
    {678,"Zambia"},{679,"Zimbabwe"},
    // South America 701-775
    {701,"Argentina"},{710,"Brazil"},{720,"Bolivia"},
    {725,"Chile"},{730,"Colombia"},{735,"Ecuador"},
    {740,"Falkland Islands"},{750,"Guyana"},{755,"Paraguay"},
    {760,"Peru"},{765,"Suriname"},{770,"Uruguay"},{775,"Venezuela"},
    {0, NULL}
};

// ======================== Internal state ========================

struct sarsat_state {
    sarsat_config_t config;

    // FM discriminator state
    float prev_i, prev_q;

    // Decimation counter
    int decim_count;
    float decim_accum;

    // DC removal (single-pole IIR)
    float dc_offset;

    // AGC
    float agc_level;
    float agc_gain;

    // PLL bit clock recovery
    float pll_phase;
    float pll_freq;        // samples per half-symbol (nominal = SARSAT_SPS)
    float prev_sample;

    // Biphase-L decoder
    int half_sym_buf[MAX_FRAME_BITS * 2 + 48]; // half-symbol buffer
    int half_sym_count;
    bool in_burst;
    int idle_count;        // consecutive low-energy samples

    // Lowpass filter
    float lpf_coeffs[SARSAT_LPF_TAPS];
    float lpf_hist[SARSAT_LPF_TAPS];
    int   lpf_idx;

    // Statistics
    sarsat_stats_t stats;
};

// ======================== Forward declarations ========================

static void sarsat_init_lpf(struct sarsat_state *st);
static float sarsat_apply_lpf(struct sarsat_state *st, float sample);
static void sarsat_process_half_symbol(struct sarsat_state *st, int level);
static void sarsat_try_decode(struct sarsat_state *st);
static bool sarsat_decode_frame(const uint8_t *bits, int nbits, bool is_test,
                                sarsat_msg_t *msg);
static uint32_t bch_syndrome(const uint8_t *bits, int n, uint32_t poly, int deg);
static bool bch_correct(uint8_t *bits, int n, uint32_t poly, int deg,
                        int max_errors, int *corrected);
static void sarsat_decode_identification(const uint8_t *frame, sarsat_msg_t *msg);
static void sarsat_decode_position(const uint8_t *frame, sarsat_msg_t *msg);
static uint32_t bits_to_int(const uint8_t *bits, int start, int len);
static void bits_to_hex(const uint8_t *bits, int start, int nbits, char *out);

// ======================== Create / Destroy ========================

struct sarsat_state *sarsat_create(const sarsat_config_t *config)
{
    struct sarsat_state *st = calloc(1, sizeof(*st));
    if (!st) return NULL;

    st->config = *config;

    // Init PLL
    st->pll_freq = (float)SARSAT_SPS;
    st->pll_phase = 0.0f;

    // Init AGC
    st->agc_gain = 1.0f;
    st->agc_level = 0.0f;

    // Init LPF
    sarsat_init_lpf(st);

    fprintf(stderr, "sarsat: decoder created, freq=%.3f MHz, rate=%.0f, IF=%d Hz, SPS=%.1f\n",
            config->center_freq / 1e6, config->sample_rate,
            SARSAT_IF_RATE, (double)SARSAT_SPS);

    return st;
}

void sarsat_destroy(struct sarsat_state *state)
{
    free(state);
}

// ======================== Lowpass filter ========================

static void sarsat_init_lpf(struct sarsat_state *st)
{
    // Blackman-windowed sinc LPF
    // Cutoff at ~2 kHz (well above 400 baud, below Nyquist of 20 kHz)
    double fc = 2000.0 / SARSAT_IF_RATE;  // normalized cutoff
    int M = SARSAT_LPF_TAPS - 1;
    double sum = 0;

    for (int i = 0; i < SARSAT_LPF_TAPS; i++) {
        double n = i - M / 2.0;
        double sinc = (fabs(n) < 1e-6) ? 1.0 : sin(2.0 * M_PI * fc * n) / (M_PI * n);
        // Blackman window
        double w = 0.42 - 0.5 * cos(2.0 * M_PI * i / M) + 0.08 * cos(4.0 * M_PI * i / M);
        st->lpf_coeffs[i] = (float)(sinc * w);
        sum += st->lpf_coeffs[i];
    }
    // Normalize
    for (int i = 0; i < SARSAT_LPF_TAPS; i++)
        st->lpf_coeffs[i] /= (float)sum;

    memset(st->lpf_hist, 0, sizeof(st->lpf_hist));
    st->lpf_idx = 0;
}

static float sarsat_apply_lpf(struct sarsat_state *st, float sample)
{
    st->lpf_hist[st->lpf_idx] = sample;
    float out = 0;
    int idx = st->lpf_idx;
    for (int i = 0; i < SARSAT_LPF_TAPS; i++) {
        out += st->lpf_coeffs[i] * st->lpf_hist[idx];
        if (--idx < 0) idx = SARSAT_LPF_TAPS - 1;
    }
    st->lpf_idx = (st->lpf_idx + 1) % SARSAT_LPF_TAPS;
    return out;
}

// ======================== Main IQ processing ========================

void sarsat_process(struct sarsat_state *state, const uint8_t *iq_data, uint32_t len)
{
    // Process IQ sample pairs
    for (uint32_t i = 0; i + 1 < len; i += 2) {
        float I = ((float)iq_data[i]     - 127.5f) / 127.5f;
        float Q = ((float)iq_data[i + 1] - 127.5f) / 127.5f;

        // FM discriminator: atan2-based frequency estimation
        // freq ∝ (I_prev * Q - Q_prev * I) / (I^2 + Q^2)
        float denom = I * I + Q * Q;
        float fm;
        if (denom > 1e-12f) {
            fm = (state->prev_i * Q - state->prev_q * I) / denom;
            fm *= FM_GAIN;
        } else {
            fm = 0.0f;
        }
        state->prev_i = I;
        state->prev_q = Q;

        // Decimation: accumulate and average
        state->decim_accum += fm;
        state->decim_count++;

        if (state->decim_count >= SARSAT_DECIM) {
            float decimated = state->decim_accum / SARSAT_DECIM;
            state->decim_accum = 0;
            state->decim_count = 0;

            state->stats.samples_processed += SARSAT_DECIM;

            // DC removal (single-pole IIR, alpha=0.001)
            state->dc_offset = 0.999f * state->dc_offset + 0.001f * decimated;
            decimated -= state->dc_offset;

            // Lowpass filter
            float filtered = sarsat_apply_lpf(state, decimated);

            // AGC
            float abs_val = fabsf(filtered);
            if (abs_val > state->agc_level)
                state->agc_level = 0.9f * state->agc_level + 0.1f * abs_val;
            else
                state->agc_level = 0.9999f * state->agc_level + 0.0001f * abs_val;

            if (state->agc_level > 0.001f) {
                float target = 0.5f / state->agc_level;
                state->agc_gain = 0.99f * state->agc_gain + 0.01f * target;
                if (state->agc_gain > 100.0f) state->agc_gain = 100.0f;
                if (state->agc_gain < 0.01f) state->agc_gain = 0.01f;
            }

            float normalized = filtered * state->agc_gain;
            if (normalized > 1.0f) normalized = 1.0f;
            if (normalized < -1.0f) normalized = -1.0f;

            // PLL bit clock recovery
            // Detect zero crossings for clock synchronization
            if ((normalized > 0 && state->prev_sample < 0) ||
                (normalized < 0 && state->prev_sample > 0)) {
                // Zero crossing: adjust PLL phase
                float phase_error = state->pll_phase - state->pll_freq / 2.0f;
                state->pll_phase -= phase_error * PLL_BW;
            }
            state->prev_sample = normalized;

            state->pll_phase += 1.0f;
            if (state->pll_phase >= state->pll_freq) {
                state->pll_phase -= state->pll_freq;
                // Sample at the center of the half-symbol
                int level = (normalized > 0) ? 1 : 0;
                sarsat_process_half_symbol(state, level);
            }
        }
    }
}

// ======================== Half-symbol processing ========================

static void sarsat_process_half_symbol(struct sarsat_state *st, int level)
{
    // Burst detection: track if we're in a signal burst
    if (!st->in_burst) {
        // Wait for activity: enough energy over recent half-symbols
        st->half_sym_buf[st->half_sym_count % (MAX_FRAME_BITS * 2 + 48)] = level;
        st->half_sym_count++;

        // Simple burst detection: look for the Biphase-L preamble pattern
        // In Biphase-L, a constant '1' bit produces alternating 1,0,1,0...
        // The preamble is 15 continuous 1-bits → 30 half-syms: 1,0,1,0,...
        if (st->half_sym_count >= 30) {
            int transitions = 0;
            int base = st->half_sym_count - 30;
            for (int i = 0; i < 29; i++) {
                int idx1 = (base + i)     % (MAX_FRAME_BITS * 2 + 48);
                int idx2 = (base + i + 1) % (MAX_FRAME_BITS * 2 + 48);
                if (st->half_sym_buf[idx1] != st->half_sym_buf[idx2])
                    transitions++;
            }
            // Preamble: expect ~29 transitions out of 29 possible
            if (transitions >= 26) {
                st->in_burst = true;
                // Reset: start collecting from here
                // Copy the last 30 half-syms to the beginning
                for (int i = 0; i < 30; i++) {
                    int src = (base + i) % (MAX_FRAME_BITS * 2 + 48);
                    st->half_sym_buf[i] = st->half_sym_buf[src];
                }
                st->half_sym_count = 30;
                st->idle_count = 0;
            }
        }

        // Keep buffer from overflowing
        if (st->half_sym_count > MAX_FRAME_BITS * 2 + 40) {
            st->half_sym_count = 0;
        }
        return;
    }

    // In burst: collect half-symbols
    if (st->half_sym_count < MAX_FRAME_BITS * 2 + 48) {
        st->half_sym_buf[st->half_sym_count++] = level;
    }

    // Collect enough half-symbols then decode.
    // Preamble length varies (real beacons ~64 bits, test signals 20-40 bits)
    // so we wait until the buffer is full, then attempt decode.
    if (st->half_sym_count >= MAX_FRAME_BITS * 2 + 40) {
        sarsat_try_decode(st);
        st->in_burst = false;
        st->half_sym_count = 0;
    }
}

// ======================== Frame decode attempt ========================

static void sarsat_try_decode(struct sarsat_state *st)
{
    // Convert half-symbols to Biphase-L bits
    // Biphase-L: 1 → high-low (1,0), 0 → low-high (0,1)
    int n_half = st->half_sym_count;
    uint8_t bits[MAX_FRAME_BITS];
    int n_bits = 0;

    for (int i = 0; i + 1 < n_half && n_bits < MAX_FRAME_BITS; i += 2) {
        int first  = st->half_sym_buf[i];
        int second = st->half_sym_buf[i + 1];
        if (first == 1 && second == 0) {
            bits[n_bits++] = 1;
        } else if (first == 0 && second == 1) {
            bits[n_bits++] = 0;
        } else {
            // Invalid Biphase-L pair — insert best guess
            bits[n_bits++] = first;
        }
    }

    if (n_bits < SARSAT_PREAMBLE_LEN + SARSAT_FRAMESYNC_LEN + 82) {
        return;  // Not enough bits for even a short message
    }

    // Search for frame sync pattern after preamble
    // Preamble length varies: 20-64+ bits in the buffer after burst detection
    // Only require minimum data for short format (9 sync + 82 data = 91 bits)
    for (int start = 0; start + SARSAT_FRAMESYNC_LEN + 82 <= n_bits && start < 100; start++) {
        // Check preamble: expect ~15 ones before frame sync
        int ones = 0;
        int pream_start = (start >= SARSAT_PREAMBLE_LEN) ? start - SARSAT_PREAMBLE_LEN : 0;
        for (int j = pream_start; j < start; j++) {
            if (bits[j] == 1) ones++;
        }
        if (ones < PREAMBLE_MIN_ONES && start >= SARSAT_PREAMBLE_LEN)
            continue;

        // Check frame sync (9 bits at position 'start')
        if (start + SARSAT_FRAMESYNC_LEN > n_bits)
            continue;

        uint16_t sync = 0;
        for (int j = 0; j < SARSAT_FRAMESYNC_LEN; j++) {
            sync = (sync << 1) | bits[start + j];
        }

        bool is_test;
        if (sync == SARSAT_SYNC_NORMAL) {
            is_test = false;
        } else if (sync == SARSAT_SYNC_TEST) {
            is_test = true;
        } else {
            continue;  // No valid sync
        }

        st->stats.bursts_detected++;

        // Extract frame data bits (after sync)
        int data_start = start + SARSAT_FRAMESYNC_LEN;
        int data_avail = n_bits - data_start;
        if (data_avail < 82) continue;  // Need at least PDF-1 + BCH-1

        int frame_len = (data_avail >= 120) ? 120 : 82;
        uint8_t frame[120];
        memcpy(frame, &bits[data_start], frame_len);

        sarsat_msg_t msg;
        if (sarsat_decode_frame(frame, frame_len, is_test, &msg)) {
            st->stats.frames_decoded++;
            st->stats.bch1_corrected += (uint64_t)msg.bch1_errors;
            st->stats.bch2_corrected += (uint64_t)msg.bch2_errors;

            if (st->config.callback) {
                st->config.callback(&msg, st->config.callback_ctx);
            }
            return;  // Success — stop searching
        } else {
            if (!msg.bch1_valid) st->stats.bch1_failed++;
        }
    }
}

// ======================== Frame decode + BCH ========================

static bool sarsat_decode_frame(const uint8_t *bits, int nbits, bool is_test,
                                sarsat_msg_t *msg)
{
    memset(msg, 0, sizeof(*msg));
    msg->is_test = is_test;

    // Copy raw bits
    int copy_len = (nbits > SARSAT_FRAME_BITS) ? SARSAT_FRAME_BITS : nbits;
    memcpy(msg->raw_bits, bits, copy_len);

    // Bit 0 (bit 25 in full frame): Format flag  0=short, 1=long
    msg->long_message = (bits[0] == 1);

    // Extract and verify BCH-1: bits 0-81 (PDF-1 61 bits + BCH1 21 bits)
    uint8_t bch1_block[BCH1_N];
    memcpy(bch1_block, bits, BCH1_N);

    uint32_t s1 = bch_syndrome(bch1_block, BCH1_N, BCH1_POLY, BCH1_DEG);
    if (s1 == 0) {
        msg->bch1_valid = true;
        msg->bch1_errors = 0;
    } else {
        int corrected = 0;
        if (bch_correct(bch1_block, BCH1_N, BCH1_POLY, BCH1_DEG, BCH1_T, &corrected)) {
            msg->bch1_valid = true;
            msg->bch1_errors = corrected;
            // Copy corrected bits back
            memcpy((uint8_t *)bits, bch1_block, BCH1_N);
        } else {
            msg->bch1_valid = false;
            msg->valid = false;
            return false;
        }
    }

    // Decode identification fields from PDF-1
    sarsat_decode_identification(bits, msg);

    // If long format, verify BCH-2 and decode position
    if (msg->long_message && nbits >= 120) {
        uint8_t bch2_block[BCH2_N];
        memcpy(bch2_block, bits + BCH1_N, BCH2_N);

        uint32_t s2 = bch_syndrome(bch2_block, BCH2_N, BCH2_POLY, BCH2_DEG);
        if (s2 == 0) {
            msg->bch2_valid = true;
            msg->bch2_errors = 0;
        } else {
            int corrected = 0;
            if (bch_correct(bch2_block, BCH2_N, BCH2_POLY, BCH2_DEG, BCH2_T, &corrected)) {
                msg->bch2_valid = true;
                msg->bch2_errors = corrected;
                memcpy((uint8_t *)(bits + BCH1_N), bch2_block, BCH2_N);
            } else {
                msg->bch2_valid = false;
            }
        }

        // Decode position from PDF-1 coarse + PDF-2 fine
        sarsat_decode_position(bits, msg);
    }

    // Generate 15-character Hex ID (bits 1-60 → 60 bits → 15 hex chars)
    bits_to_hex(bits, 1, 60, msg->hex_id);

    msg->valid = msg->bch1_valid;
    return msg->valid;
}

// ======================== BCH error correction ========================

static uint32_t bch_syndrome(const uint8_t *bits, int n, uint32_t poly, int deg)
{
    uint32_t remainder = 0;
    for (int i = 0; i < n; i++) {
        remainder = (remainder << 1) | bits[i];
        if (remainder & (1u << deg))
            remainder ^= poly;
    }
    return remainder;
}

static bool bch_correct(uint8_t *bits, int n, uint32_t poly, int deg,
                         int max_errors, int *corrected)
{
    *corrected = 0;

    // Try single-bit error correction
    for (int i = 0; i < n; i++) {
        bits[i] ^= 1;
        if (bch_syndrome(bits, n, poly, deg) == 0) {
            *corrected = 1;
            return true;
        }
        bits[i] ^= 1;
    }

    if (max_errors < 2) return false;

    // Try double-bit error correction
    for (int i = 0; i < n - 1; i++) {
        bits[i] ^= 1;
        for (int j = i + 1; j < n; j++) {
            bits[j] ^= 1;
            if (bch_syndrome(bits, n, poly, deg) == 0) {
                *corrected = 2;
                return true;
            }
            bits[j] ^= 1;
        }
        bits[i] ^= 1;
    }

    if (max_errors < 3) return false;

    // Try triple-bit error correction
    for (int i = 0; i < n - 2; i++) {
        bits[i] ^= 1;
        for (int j = i + 1; j < n - 1; j++) {
            bits[j] ^= 1;
            for (int k = j + 1; k < n; k++) {
                bits[k] ^= 1;
                if (bch_syndrome(bits, n, poly, deg) == 0) {
                    *corrected = 3;
                    return true;
                }
                bits[k] ^= 1;
            }
            bits[j] ^= 1;
        }
        bits[i] ^= 1;
    }

    return false;
}

// ======================== Protocol field decoding ========================

static uint32_t bits_to_int(const uint8_t *bits, int start, int len)
{
    uint32_t val = 0;
    for (int i = 0; i < len; i++)
        val = (val << 1) | bits[start + i];
    return val;
}

static void bits_to_hex(const uint8_t *bits, int start, int nbits, char *out)
{
    int pos = 0;
    for (int i = 0; i < nbits; i += 4) {
        uint8_t nibble = 0;
        for (int j = 0; j < 4 && (i + j) < nbits; j++)
            nibble = (nibble << 1) | bits[start + i + j];
        out[pos++] = "0123456789ABCDEF"[nibble];
    }
    out[pos] = '\0';
}

static void sarsat_decode_identification(const uint8_t *frame, sarsat_msg_t *msg)
{
    // frame[0]  = format flag (bit 25)
    // frame[1]  = protocol flag (bit 26): 0=standard location, 1=user
    bool user_protocol = (frame[1] == 1);

    // frame[2..11] = country code (bits 27-36, 10 bits)
    msg->country_code = (int)bits_to_int(frame, 2, 10);
    const char *cname = sarsat_country_name(msg->country_code);
    strncpy(msg->country_name, cname, sizeof(msg->country_name) - 1);

    // frame[12..15] = protocol code (bits 37-40, 4 bits)
    uint8_t proto = (uint8_t)bits_to_int(frame, 12, 4);

    // Determine protocol and beacon type
    if (!user_protocol) {
        // Standard location protocols
        switch (proto) {
        case 0x02: msg->protocol = SARSAT_PROTO_ORBITOGRAPHY; break;
        case 0x03: msg->protocol = SARSAT_PROTO_ELT_SERIAL; break;
        case 0x04: msg->protocol = SARSAT_PROTO_ELT_OPERATOR; break;
        case 0x05: msg->protocol = SARSAT_PROTO_ELT_AIRCRAFT; break;
        case 0x06: msg->protocol = SARSAT_PROTO_EPIRB_MMSI; break;
        case 0x0E: msg->protocol = SARSAT_PROTO_STD_TEST; break;
        case 0x0F: msg->protocol = SARSAT_PROTO_NAT_TEST; break;
        case 0x01: msg->protocol = SARSAT_PROTO_ELT_DT; break;
        default:   msg->protocol = SARSAT_PROTO_UNKNOWN; break;
        }
    } else {
        // User protocols
        switch (proto) {
        case 0x08: msg->protocol = SARSAT_PROTO_EPIRB_RADIO; break;
        case 0x09: msg->protocol = SARSAT_PROTO_SHIP_MMSI; break;
        case 0x0B: msg->protocol = SARSAT_PROTO_PLB_SERIAL; break;
        case 0x0C: msg->protocol = SARSAT_PROTO_NAT_LOC; break;
        case 0x0E: msg->protocol = SARSAT_PROTO_STD_TEST; break;
        case 0x0F: msg->protocol = SARSAT_PROTO_NAT_TEST; break;
        default:   msg->protocol = SARSAT_PROTO_UNKNOWN; break;
        }
    }

    // Determine beacon type
    switch (msg->protocol) {
    case SARSAT_PROTO_ELT_SERIAL:
    case SARSAT_PROTO_ELT_AIRCRAFT:
    case SARSAT_PROTO_ELT_OPERATOR:
        msg->beacon_type = SARSAT_BEACON_ELT;
        break;
    case SARSAT_PROTO_EPIRB_MMSI:
    case SARSAT_PROTO_EPIRB_RADIO:
        msg->beacon_type = SARSAT_BEACON_EPIRB;
        break;
    case SARSAT_PROTO_PLB_SERIAL:
        msg->beacon_type = SARSAT_BEACON_PLB;
        break;
    case SARSAT_PROTO_SHIP_MMSI:
        msg->beacon_type = SARSAT_BEACON_SSAS;
        break;
    case SARSAT_PROTO_ELT_DT:
        msg->beacon_type = SARSAT_BEACON_ELT_DT;
        break;
    default:
        msg->beacon_type = SARSAT_BEACON_UNKNOWN;
        break;
    }

    // Decode identification data based on protocol
    // frame[16..] = identification (bits 41+)
    switch (msg->protocol) {
    case SARSAT_PROTO_ELT_SERIAL:
    case SARSAT_PROTO_PLB_SERIAL:
        // Certificate number (10 bits) + Serial (14 bits)
        msg->cert_number = bits_to_int(frame, 16, 10);
        msg->serial_number = bits_to_int(frame, 26, 14);
        break;

    case SARSAT_PROTO_ELT_AIRCRAFT:
        // 24-bit ICAO aircraft address
        msg->icao_address = bits_to_int(frame, 16, 24);
        break;

    case SARSAT_PROTO_ELT_OPERATOR: {
        // Operator designator (18 bits, 3 chars × 6-bit Baudot)
        // + additional serial (6 bits)
        uint32_t op_bits = bits_to_int(frame, 16, 18);
        char c1 = (char)((op_bits >> 12) & 0x3F);
        char c2 = (char)((op_bits >>  6) & 0x3F);
        char c3 = (char)((op_bits      ) & 0x3F);
        // Convert from modified Baudot to ASCII (letters = +0x40)
        msg->aircraft_operator[0] = (c1 >= 1 && c1 <= 26) ? (char)(c1 + 'A' - 1) : '?';
        msg->aircraft_operator[1] = (c2 >= 1 && c2 <= 26) ? (char)(c2 + 'A' - 1) : '?';
        msg->aircraft_operator[2] = (c3 >= 1 && c3 <= 26) ? (char)(c3 + 'A' - 1) : '?';
        msg->aircraft_operator[3] = '\0';
        msg->serial_number = bits_to_int(frame, 34, 6);
        break;
    }

    case SARSAT_PROTO_EPIRB_MMSI:
    case SARSAT_PROTO_SHIP_MMSI:
    case SARSAT_PROTO_ORBITOGRAPHY: {
        // MMSI (20 bits) + beacon number (4 bits)
        uint32_t mmsi_part = bits_to_int(frame, 16, 20);
        snprintf(msg->mmsi, sizeof(msg->mmsi), "%u", mmsi_part);
        msg->serial_number = bits_to_int(frame, 36, 4); // beacon number
        break;
    }

    case SARSAT_PROTO_EPIRB_RADIO: {
        // Radio call sign: modified Baudot, up to 7 chars
        // Bits 41-82 = 42 bits → 7 × 6-bit chars
        char cs[8];
        for (int i = 0; i < 7; i++) {
            int code = (int)bits_to_int(frame, 16 + i * 6, 6);
            if (code >= 1 && code <= 26)
                cs[i] = (char)(code + 'A' - 1);
            else if (code >= 27 && code <= 36)
                cs[i] = (char)(code - 27 + '0');
            else if (code == 0)
                cs[i] = ' ';
            else
                cs[i] = '?';
        }
        cs[7] = '\0';
        // Trim trailing spaces
        for (int i = 6; i >= 0 && cs[i] == ' '; i--)
            cs[i] = '\0';
        strncpy(msg->call_sign, cs, sizeof(msg->call_sign) - 1);
        break;
    }

    case SARSAT_PROTO_ELT_DT:
    case SARSAT_PROTO_STD_TEST:
        // Certificate number (10 bits) + Serial (14 bits)
        msg->cert_number = bits_to_int(frame, 16, 10);
        msg->serial_number = bits_to_int(frame, 26, 14);
        break;

    case SARSAT_PROTO_NAT_LOC:
    case SARSAT_PROTO_NAT_TEST:
        // National format: 24-bit serial
        msg->serial_number = bits_to_int(frame, 16, 24);
        break;

    default:
        // Generic serial extraction
        msg->serial_number = bits_to_int(frame, 16, 24);
        break;
    }
}

static void sarsat_decode_position(const uint8_t *frame, sarsat_msg_t *msg)
{
    // Coarse position from PDF-1 (bits 65-85 = frame[40..60])
    // Bit 40: Lat N/S (0=N, 1=S)
    // Bits 41-47: Lat degrees (7 bits, 0-90)
    // Bits 48-49: Lat quarter-degrees (2 bits, 0-3, ×15 minutes)
    // Bit 50: Lon E/W (0=E, 1=W)
    // Bits 51-58: Lon degrees (8 bits, 0-180)
    // Bits 59-60: Lon quarter-degrees (2 bits, 0-3, ×15 minutes)

    bool north = (frame[40] == 0);
    int lat_deg = (int)bits_to_int(frame, 41, 7);
    int lat_min_coarse = (int)bits_to_int(frame, 48, 2) * 15;

    bool east = (frame[50] == 0);
    int lon_deg = (int)bits_to_int(frame, 51, 8);
    int lon_min_coarse = (int)bits_to_int(frame, 59, 2) * 15;

    int lat_min = lat_min_coarse;
    int lat_sec = 0;
    int lon_min = lon_min_coarse;
    int lon_sec = 0;

    msg->position_from_gps = false;
    msg->homing_121_5 = false;

    // Fine position from PDF-2 (bits 107-132 = frame[82..107])
    if (msg->long_message && msg->bch2_valid) {
        // PDF-2 starts at frame[82]
        int pdf2 = 82;

        // Bits 0-3: type field, must be 0b1101 (0x0D) for position data
        uint8_t pdf2_type = (uint8_t)bits_to_int(frame, pdf2, 4);
        if (pdf2_type != 0x0D) {
            msg->position_valid = (lat_deg > 0 || lon_deg > 0);
        } else {

        // Bit 4: position source (0=internal, 1=external GPS)
        // Bit 5: 121.5 MHz homing device
        msg->position_from_gps = (frame[pdf2 + 4] == 1);
        msg->homing_121_5 = (frame[pdf2 + 5] == 1);

        // Latitude offset: sign(1) + minutes(5) + seconds/4(4)
        bool lat_offset_pos = (frame[pdf2 + 6] == 1);
        int lat_min_offset = (int)bits_to_int(frame, pdf2 + 7, 5);
        int lat_sec_4 = (int)bits_to_int(frame, pdf2 + 12, 4);

        lat_min = lat_min_coarse + (lat_offset_pos ? lat_min_offset : -lat_min_offset);
        if (lat_min < 0) { lat_min += 60; lat_deg--; }
        if (lat_min >= 60) { lat_min -= 60; lat_deg++; }
        lat_sec = lat_sec_4 * 4;

        // Longitude offset: sign(1) + minutes(5) + seconds/4(4)
        bool lon_offset_pos = (frame[pdf2 + 16] == 1);
        int lon_min_offset = (int)bits_to_int(frame, pdf2 + 17, 5);
        int lon_sec_4 = (int)bits_to_int(frame, pdf2 + 22, 4);

        lon_min = lon_min_coarse + (lon_offset_pos ? lon_min_offset : -lon_min_offset);
        if (lon_min < 0) { lon_min += 60; lon_deg--; }
        if (lon_min >= 60) { lon_min -= 60; lon_deg++; }
        lon_sec = lon_sec_4 * 4;

        msg->position_valid = true;
        } // end pdf2_type == 0x0D
    } else {
        // Coarse position only — still valid at ~25 km resolution
        msg->position_valid = (lat_deg > 0 || lon_deg > 0);
    }

    // Convert to decimal degrees
    double lat = lat_deg + lat_min / 60.0 + lat_sec / 3600.0;
    double lon = lon_deg + lon_min / 60.0 + lon_sec / 3600.0;
    msg->latitude  = north ? lat : -lat;
    msg->longitude = east  ? lon : -lon;
}

// ======================== Utility functions ========================

void sarsat_flush(struct sarsat_state *state)
{
    if (state && state->in_burst && state->half_sym_count >= 200) {
        sarsat_try_decode(state);
        state->in_burst = false;
        state->half_sym_count = 0;
    }
}

void sarsat_get_stats(struct sarsat_state *state, sarsat_stats_t *stats)
{
    if (state && stats) *stats = state->stats;
}

const char *sarsat_beacon_type_name(sarsat_beacon_type_t type)
{
    switch (type) {
    case SARSAT_BEACON_ELT:     return "ELT";
    case SARSAT_BEACON_EPIRB:   return "EPIRB";
    case SARSAT_BEACON_PLB:     return "PLB";
    case SARSAT_BEACON_SSAS:    return "SSAS";
    case SARSAT_BEACON_ELT_DT:  return "ELT(DT)";
    default:                    return "Unknown";
    }
}

const char *sarsat_protocol_name(sarsat_protocol_t proto)
{
    switch (proto) {
    case SARSAT_PROTO_ORBITOGRAPHY:  return "Orbitography";
    case SARSAT_PROTO_ELT_SERIAL:   return "ELT Serial";
    case SARSAT_PROTO_ELT_AIRCRAFT: return "ELT Aircraft 24-bit";
    case SARSAT_PROTO_ELT_OPERATOR: return "ELT Operator";
    case SARSAT_PROTO_EPIRB_MMSI:   return "EPIRB MMSI";
    case SARSAT_PROTO_EPIRB_RADIO:  return "EPIRB Radio Call";
    case SARSAT_PROTO_SHIP_MMSI:    return "Ship MMSI";
    case SARSAT_PROTO_PLB_SERIAL:   return "PLB Serial";
    case SARSAT_PROTO_NAT_LOC:      return "National Location";
    case SARSAT_PROTO_STD_TEST:     return "Standard Test";
    case SARSAT_PROTO_NAT_TEST:     return "National Test";
    case SARSAT_PROTO_ELT_DT:      return "ELT(DT)";
    default:                        return "Unknown";
    }
}

const char *sarsat_country_name(int mid)
{
    for (int i = 0; country_table[i].name != NULL; i++) {
        if (country_table[i].code == mid)
            return country_table[i].name;
    }
    // Range-based fallback for common multi-code countries
    if (mid >= 211 && mid <= 218) return "Germany";
    if (mid >= 224 && mid <= 225) return "Spain";
    if (mid >= 226 && mid <= 228) return "France";
    if (mid >= 232 && mid <= 235) return "United Kingdom";
    if (mid >= 237 && mid <= 241) return "Greece";
    if (mid >= 244 && mid <= 246) return "Netherlands";
    if (mid >= 257 && mid <= 259) return "Norway";
    if (mid >= 265 && mid <= 266) return "Sweden";
    if (mid >= 338 && mid <= 339) return "USA";
    if (mid >= 366 && mid <= 369) return "USA";
    if (mid >= 351 && mid <= 357) return "Panama";
    if (mid >= 370 && mid <= 374) return "Panama";
    if (mid >= 412 && mid <= 414) return "China";
    if (mid >= 431 && mid <= 432) return "Japan";
    if (mid >= 440 && mid <= 441) return "South Korea";
    if (mid >= 563 && mid <= 566) return "Singapore";
    if (mid >= 636 && mid <= 637) return "Liberia";
    return "Unknown";
}
