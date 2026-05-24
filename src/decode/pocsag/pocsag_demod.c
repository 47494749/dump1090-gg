// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// pocsag_demod.c: POCSAG pager decoder — multi-channel
//
// Implements a complete POCSAG receiver chain with channelizer:
//   Wideband IQ (2.4 MHz) → per-channel freq translation → FM discriminator →
//   clock recovery → bit slicer → preamble → sync → BCH → message extraction
//
// Supports 512, 1200, and 2400 baud auto-detection on each channel.
// Supports up to POCSAG_MAX_CHANNELS simultaneous frequencies.
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "pocsag_demod.h"

// ======================== Internal Constants ========================

// Supported baud rates for auto-detection
static const int BAUD_RATES[] = { 512, 1200, 2400 };
#define NUM_BAUD_RATES  3

// BCH(31,21) generator polynomial for POCSAG
// g(x) = x^10 + x^9 + x^8 + x^6 + x^5 + x^3 + 1 = 0x769
#define BCH_POLY  0x769u

// Number of data bits in a codeword (excluding parity + even parity bit)
#define BCH_DATA_BITS   21
#define BCH_PARITY_BITS 10
#define BCH_TOTAL_BITS  31  // +1 even parity = 32

// Decimation factor: reduces sample rate before clock recovery
// 2.4 MSPS / 50 = 48 kHz, giving 40 samples/bit at 1200 baud
#define POCSAG_DECIMATION  50

// ======================== Per-baud-rate state ========================

typedef struct {
    int      baud_rate;
    double   samples_per_bit;

    // Clock recovery
    double   clock_phase;        // fractional sample counter within a bit
    double   clock_freq;         // current estimated samples/bit

    // Preamble detection
    int      preamble_count;     // consecutive alternating bits
    bool     preamble_found;

    // Sync + batch reception
    uint32_t shift_reg;          // 32-bit shift register for sync search
    bool     synced;
    int      batch_bit_count;    // bits received in current batch
    uint32_t batch_words[POCSAG_BATCH_WORDS]; // current batch
    int      batch_word_idx;     // current word index in batch

    // Message assembly
    bool     in_message;
    uint32_t msg_address;
    int      msg_function;
    char     alpha_buf[POCSAG_MSG_MAX_LEN];
    int      alpha_len;
    char     numeric_buf[POCSAG_MSG_MAX_LEN];
    int      numeric_len;
    int      alpha_bit_buf;      // partial character bits
    int      alpha_bit_count;
    int      errors_total;
    bool     inverted;           // true if FSK polarity is inverted
} baud_state_t;

// ======================== Per-channel state ========================
// Each channel has its own frequency translation, FM demod, LPF, and baud decoders.

typedef struct {
    double   channel_freq;              // this channel's frequency in Hz
    double   freq_offset;               // offset from SDR center freq in Hz

    // NCO (Numerically Controlled Oscillator) for frequency translation
    double   nco_phase;                 // current oscillator phase (radians)
    double   nco_phase_inc;             // phase increment per sample

    // FM discriminator state
    float    prev_i, prev_q;

    // Low-pass filter state (simple IIR)
    float    lpf_out;
    float    lpf_alpha;

    // Decimation accumulator (integrate POCSAG_DECIMATION samples)
    float    decim_acc;
    int      decim_count;

    // DC offset removal (operates on decimated stream)
    float    dc_avg;

    // 3 baud-rate decoders
    baud_state_t baud[NUM_BAUD_RATES];

    // Clock recovery: previous sample per baud rate
    float    prev_sample[NUM_BAUD_RATES];

    // Per-channel signal level
    float    signal_acc;
    int      sample_count;
} channel_state_t;

// ======================== Main decoder state ========================

struct pocsag_state {
    pocsag_config_t config;
    pocsag_stats_t  stats;

    int              num_channels;
    channel_state_t  channels[POCSAG_MAX_CHANNELS];
};

// ======================== BCH(31,21) decode ========================

// Compute BCH syndrome for a 32-bit codeword.
// Returns syndrome (0 = no error in first 31 bits).
static uint32_t bch_syndrome(uint32_t cw)
{
    // The codeword is bits [31..1] = BCH(31,21), bit [0] = even parity
    uint32_t data = cw >> 1;  // strip parity bit

    uint32_t syndrome = 0;
    for (int i = 30; i >= 0; i--) {
        syndrome <<= 1;
        syndrome |= ((data >> i) & 1);
        if (syndrome & (1u << BCH_PARITY_BITS))
            syndrome ^= BCH_POLY;
    }
    return syndrome;
}

static bool pocsag_even_parity_ok(uint32_t cw)
{
    uint32_t p = cw;
    p ^= p >> 16;
    p ^= p >> 8;
    p ^= p >> 4;
    p ^= p >> 2;
    p ^= p >> 1;
    return (p & 1u) == 0;
}

// Try to correct up to two bit errors in the BCH(31,21)+parity codeword.
// Returns number of bits corrected (0, 1, or 2), or -1 if uncorrectable.
static int bch_correct(uint32_t *cw)
{
    uint32_t syndrome = bch_syndrome(*cw);
    if (syndrome == 0) {
        if (!pocsag_even_parity_ok(*cw)) {
            *cw ^= 1u;   // parity bit only
            return 1;
        }
        return 0;
    }

    // Try all single-bit flips first, including the explicit parity bit.
    for (int i = 0; i <= 31; i++) {
        uint32_t trial = *cw ^ (1u << i);
        if (bch_syndrome(trial) == 0 && pocsag_even_parity_ok(trial)) {
            *cw = trial;
            return 1;
        }
    }

    // BCH(31,21) has minimum distance 6, so up to 2 bit errors are correctable.
    // Exhaustive search is small enough here: C(32,2)=496 trials.
    for (int i = 0; i <= 31; i++) {
        for (int j = i + 1; j <= 31; j++) {
            uint32_t trial = *cw ^ (1u << i) ^ (1u << j);
            if (bch_syndrome(trial) == 0 && pocsag_even_parity_ok(trial)) {
                *cw = trial;
                return 2;
            }
        }
    }

    return -1;  // uncorrectable
}

// ======================== Baud-rate state helpers ========================

static void baud_state_init(baud_state_t *bs, int baud, double sample_rate)
{
    memset(bs, 0, sizeof(*bs));
    bs->baud_rate = baud;
    // Use decimated sample rate for clock recovery
    double decimated_rate = sample_rate / POCSAG_DECIMATION;
    bs->samples_per_bit = decimated_rate / baud;
    bs->clock_freq = bs->samples_per_bit;
    bs->clock_phase = 0;
}

// ======================== Message delivery ========================

static void deliver_message(struct pocsag_state *st, baud_state_t *bs, float sig, double channel_freq)
{
    if (bs->alpha_len == 0 && bs->numeric_len == 0 && !bs->in_message)
        return;

    pocsag_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.address = bs->msg_address;
    msg.function = bs->msg_function;
    msg.baud_rate = bs->baud_rate;
    msg.signal_level = sig;
    msg.errors_corrected = bs->errors_total;
    msg.channel_freq = channel_freq;

    if (bs->msg_function == POCSAG_FUNC_TONE) {
        msg.is_tone_only = true;
    } else if (bs->msg_function == POCSAG_FUNC_NUMERIC) {
        msg.is_numeric = true;
        if (bs->numeric_len > 0) {
            memcpy(msg.numeric_msg, bs->numeric_buf, bs->numeric_len);
            msg.numeric_msg[bs->numeric_len] = '\0';
            msg.numeric_len = bs->numeric_len;
        }
    } else {
        // Function 1,2 = alpha
        msg.is_alpha = true;
        if (bs->alpha_len > 0) {
            memcpy(msg.alpha_msg, bs->alpha_buf, bs->alpha_len);
            msg.alpha_msg[bs->alpha_len] = '\0';
            msg.alpha_len = bs->alpha_len;
        }
    }

    st->stats.messages_decoded++;
    if (st->config.callback)
        st->config.callback(&msg, st->config.callback_ctx);

    // Reset message assembly
    bs->in_message = false;
    bs->alpha_len = 0;
    bs->numeric_len = 0;
    bs->alpha_bit_buf = 0;
    bs->alpha_bit_count = 0;
    bs->errors_total = 0;
}

// ======================== PN9 de-whitening ========================

// Some transmitters (e.g. SX1262-based) apply PN9 data whitening to the payload.
// Polynomial: x^9 + x^5 + 1 (Fibonacci LFSR)
// The whitening is applied after the sync word, with MSB-first bit packing.
static void pn9_dewhiten_batch(uint32_t *codewords, int n_codewords, uint16_t seed) {
    uint16_t state = seed & 0x1FF;
    // Skip first 32 PN9 bits (they correspond to the sync word, which is not whitened)
    for (int i = 0; i < 32; i++) {
        uint8_t feedback = ((state >> 0) ^ (state >> 5)) & 1;
        state = (state >> 1) | (feedback << 8);
    }
    for (int w = 0; w < n_codewords; w++) {
        uint32_t mask = 0;
        for (int b = 0; b < 32; b++) {
            uint8_t out_bit = state & 1;
            uint8_t feedback = ((state >> 0) ^ (state >> 5)) & 1;
            state = (state >> 1) | (feedback << 8);
            mask = (mask << 1) | out_bit;
        }
        codewords[w] ^= mask;
    }
}

// ======================== Batch processing ========================

// POCSAG numeric character set (BCD table)
static const char POCSAG_NUMERIC_CHARS[] = "0123456789*U -)( ";

// Process a complete batch of 17 codewords (sync + 8 pairs of 2 codewords)
static void process_batch(struct pocsag_state *st, baud_state_t *bs, float sig, double channel_freq)
{
    int bch_ok = 0, bch_fail = 0, idle_cnt = 0, addr_cnt = 0, msg_cnt = 0;

    // Adaptive realignment: check if batch has 1-bit clock/sync offset.
    // Try BCH on word[1] as-is; if it fails, try right-shifted version.
    uint32_t test_cw = bs->batch_words[1];
    int need_realign = (bch_syndrome(test_cw) != 0);
    if (need_realign) {
        // Check if realigned word passes BCH
        uint32_t sync_lsb = bs->batch_words[0] & 1;
        uint32_t realigned = (test_cw >> 1) | (sync_lsb << 31);
        if (bch_syndrome(realigned) == 0 || realigned == POCSAG_IDLE_WORD) {
            // Apply realignment to all data words
            for (int i = POCSAG_BATCH_WORDS - 1; i >= 1; i--) {
                uint32_t msb_from_prev = bs->batch_words[i - 1] & 1;
                bs->batch_words[i] = (bs->batch_words[i] >> 1) | (msb_from_prev << 31);
            }
        }
    }

    // Auto-detect PN9 whitening (e.g. SX1262-based transmitters)
    // If first data word fails BCH raw, try PN9 de-whitening with known seeds
    if (bch_syndrome(bs->batch_words[1]) != 0 && bs->batch_words[1] != POCSAG_IDLE_WORD) {
        static const uint16_t pn9_seeds[] = { 0x02E, 0x1FF, 0x100 };
        for (int s = 0; s < 3; s++) {
            uint32_t test_copy[16];
            memcpy(test_copy, &bs->batch_words[1], 16 * sizeof(uint32_t));
            pn9_dewhiten_batch(test_copy, 16, pn9_seeds[s]);
            if (bch_syndrome(test_copy[0]) == 0 || test_copy[0] == POCSAG_IDLE_WORD) {
                memcpy(&bs->batch_words[1], test_copy, 16 * sizeof(uint32_t));
                break;
            }
        }
    }

    // Words 0 = sync word (already verified), words 1..16 = 8 pairs
    for (int i = 1; i < POCSAG_BATCH_WORDS; i++) {
        uint32_t cw = bs->batch_words[i];

        // BCH error correction
        int corr = bch_correct(&cw);
        if (corr < 0) {
            bch_fail++;
            st->stats.bch_failures++;
            // If we're in a message, terminate it
            if (bs->in_message) {
                deliver_message(st, bs, sig, channel_freq);
            }
            continue;
        }
        bch_ok++;
        if (corr > 0) {
            st->stats.bch_corrections++;
            bs->errors_total += corr;
        }

        // Check idle codeword
        if (cw == POCSAG_IDLE_WORD || (cw >> 1) == (POCSAG_IDLE_WORD >> 1)) {
            idle_cnt++;
            if (bs->in_message) {
                deliver_message(st, bs, sig, channel_freq);
            }
            continue;
        }

        bool is_address = ((cw >> 31) & 1) == 0;

        if (is_address) {
            addr_cnt++;
            // Deliver previous message if any
            if (bs->in_message) {
                deliver_message(st, bs, sig, channel_freq);
            }

            // Start new message
            // Address: bits 30..13 = 18 address bits, bits 12..11 = function
            // Full address = (18-bit address << 3) | frame position (i/2)
            uint32_t addr18 = (cw >> 13) & 0x3FFFF;
            int frame_pos = (i - 1) / 2;  // 0..7
            bs->msg_address = (addr18 << 3) | frame_pos;
            bs->msg_function = (cw >> 11) & 3;
            bs->in_message = true;
            bs->alpha_len = 0;
            bs->numeric_len = 0;
            bs->alpha_bit_buf = 0;
            bs->alpha_bit_count = 0;
            bs->errors_total = 0;

            if (bs->msg_function == POCSAG_FUNC_TONE) {
                // Tone-only: deliver immediately
                deliver_message(st, bs, sig, channel_freq);
            }
        } else {
            // Message codeword
            msg_cnt++;
            if (!bs->in_message)
                continue;  // orphan data word, skip

            // Collect codewords for decoding
            // Decode on the fly
            uint32_t data = (cw >> 11) & 0xFFFFF;

            if (bs->msg_function == POCSAG_FUNC_NUMERIC) {
                // Numeric: 5 BCD digits per codeword (ITU-R M.584: nibble bits reversed)
                for (int d = 0; d < 5 && bs->numeric_len < POCSAG_MSG_MAX_LEN - 1; d++) {
                    int nibble = (data >> (16 - d * 4)) & 0xF;
                    nibble = ((nibble & 1) << 3) | ((nibble & 2) << 1) |
                             ((nibble & 4) >> 1) | ((nibble & 8) >> 3);
                    bs->numeric_buf[bs->numeric_len++] = POCSAG_NUMERIC_CHARS[nibble];
                }
            } else {
                // Alpha: 20 bits → 7-bit chars
                for (int b = 19; b >= 0; b--) {
                    bs->alpha_bit_buf |= ((data >> b) & 1) << bs->alpha_bit_count;
                    bs->alpha_bit_count++;
                    if (bs->alpha_bit_count == 7) {
                        char ch = (char)(bs->alpha_bit_buf & 0x7F);
                        if (ch >= 32 && ch < 127 && bs->alpha_len < POCSAG_MSG_MAX_LEN - 1) {
                            bs->alpha_buf[bs->alpha_len++] = ch;
                        } else if ((ch == '\n' || ch == '\r') && bs->alpha_len < POCSAG_MSG_MAX_LEN - 1) {
                            bs->alpha_buf[bs->alpha_len++] = ' ';
                        }
                        bs->alpha_bit_buf = 0;
                        bs->alpha_bit_count = 0;
                    }
                }
            }
        }
    }

    (void)bch_ok; (void)bch_fail; (void)idle_cnt; (void)addr_cnt; (void)msg_cnt;
}

// ======================== Bit processing ========================

// Count number of differing bits (Hamming distance)
static inline int popcount32(uint32_t x) {
    x = x - ((x >> 1) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
    return (int)(((x + (x >> 4)) & 0x0F0F0F0Fu) * 0x01010101u >> 24);
}

// Maximum allowed bit errors in sync word detection
#define POCSAG_SYNC_MAX_ERRORS  2

static void process_bit(struct pocsag_state *st, baud_state_t *bs, int bit, float sig, double channel_freq)
{
    // --- Preamble detection ---
    if (!bs->synced) {
        // Look for alternating 1/0 pattern
        int expected = (bs->preamble_count & 1) ? 0 : 1;
        if (bs->preamble_count == 0) expected = bit;  // accept either polarity to start

        if (bit == expected || bs->preamble_count == 0) {
            bs->preamble_count++;
            // Latch: once we've seen enough alternating bits, remember it
            if (bs->preamble_count >= POCSAG_PREAMBLE_MIN && !bs->preamble_found) {
                bs->preamble_found = true;
            }
        } else {
            // Reset but keep this bit as potential start
            bs->preamble_count = 1;
        }

        // Shift into sync search register
        bs->shift_reg = (bs->shift_reg << 1) | (bit & 1);

        // Check for sync word with fuzzy matching (require preamble seen earlier)
        if (bs->preamble_found) {
            int dist_normal = popcount32(bs->shift_reg ^ POCSAG_SYNC_WORD);
            int dist_inv = popcount32(bs->shift_reg ^ ~POCSAG_SYNC_WORD);

            if (dist_normal <= POCSAG_SYNC_MAX_ERRORS) {
                bs->synced = true;
                bs->batch_bit_count = 0;
                bs->batch_word_idx = 1;  // word 0 = sync (already matched)
                bs->batch_words[0] = POCSAG_SYNC_WORD;
                bs->inverted = false;
                st->stats.syncs_detected++;
                if (bs->preamble_count >= POCSAG_PREAMBLE_BITS)
                    st->stats.preambles_detected++;
                bs->preamble_count = 0;
                bs->preamble_found = false;
                return;
            }

            if (dist_inv <= POCSAG_SYNC_MAX_ERRORS) {
                bs->synced = true;
                bs->batch_bit_count = 0;
                bs->batch_word_idx = 1;
                bs->batch_words[0] = POCSAG_SYNC_WORD;
                bs->inverted = true;
                st->stats.syncs_detected++;
                if (bs->preamble_count >= POCSAG_PREAMBLE_BITS)
                    st->stats.preambles_detected++;
                bs->preamble_count = 0;
                bs->preamble_found = false;
                return;
            }
        }

        return;
    }

    // --- Batch bit collection ---
    // We have sync, now collecting the 16 remaining codewords (16 × 32 = 512 bits)
    if (bs->inverted) bit = !bit;  // Correct inverted polarity

    int word_bit = bs->batch_bit_count % 32;
    int word_idx = bs->batch_word_idx;

    if (word_idx < POCSAG_BATCH_WORDS) {
        if (word_bit == 0)
            bs->batch_words[word_idx] = 0;
        bs->batch_words[word_idx] = (bs->batch_words[word_idx] << 1) | (bit & 1);
    }

    bs->batch_bit_count++;

    if (bs->batch_bit_count % 32 == 0) {
        bs->batch_word_idx++;
    }

    // Check if batch is complete (16 words × 32 bits = 512 bits)
    if (bs->batch_word_idx >= POCSAG_BATCH_WORDS) {
        // Process this batch
        process_batch(st, bs, sig, channel_freq);

        // Sync + batch reset (keep preamble_found for consecutive batches)
        bs->synced = false;
        bs->shift_reg = 0;
        bs->batch_bit_count = 0;
        bs->batch_word_idx = 0;
        bs->preamble_count = 0;
        bs->preamble_found = true;  // Allow next sync without new preamble
        bs->inverted = false;
    }
}

// ======================== FM discriminator ========================

// Simple FM discriminator: atan2 phase difference
// Input: I/Q uint8_t samples (0-255, center at 128)
// Output: frequency deviation estimate (-1..+1)
static inline float fm_discriminator(float i0, float q0, float i1, float q1)
{
    // Conjugate multiply: (i1 + jq1) * conj(i0 + jq0) = (i1*i0 + q1*q0) + j(q1*i0 - i1*q0)
    float re = i1 * i0 + q1 * q0;
    float im = q1 * i0 - i1 * q0;

    // Fast atan2 approximation
    if (re == 0.0f && im == 0.0f) return 0.0f;

    // Use atan2f for accuracy
    return atan2f(im, re);
}

// ======================== Channel initialization ========================

static void channel_state_init(channel_state_t *ch, double channel_freq, double center_freq,
                                double sample_rate)
{
    memset(ch, 0, sizeof(*ch));
    ch->channel_freq = channel_freq;
    ch->freq_offset = channel_freq - center_freq;

    // NCO: compute phase increment per sample for frequency translation
    // We mix the signal down by -freq_offset to center the channel at DC
    ch->nco_phase = 0.0;
    ch->nco_phase_inc = -2.0 * M_PI * ch->freq_offset / sample_rate;

    ch->prev_i = 0;
    ch->prev_q = 0;
    ch->lpf_out = 0;
    ch->dc_avg = 0;

    // LPF cutoff ~= max baud rate * 3 = 3600 Hz
    double fc = 3600.0;
    ch->lpf_alpha = (float)(1.0 - exp(-2.0 * M_PI * fc / sample_rate));

    // Initialize 3 baud-rate decoders for this channel
    for (int i = 0; i < NUM_BAUD_RATES; i++) {
        baud_state_init(&ch->baud[i], BAUD_RATES[i], sample_rate);
    }

    ch->signal_acc = 0;
    ch->sample_count = 0;
}

// ======================== API ========================

struct pocsag_state *pocsag_create(const pocsag_config_t *cfg)
{
    if (!cfg || cfg->sample_rate <= 0) return NULL;

    struct pocsag_state *st = calloc(1, sizeof(*st));
    if (!st) return NULL;

    st->config = *cfg;
    memset(&st->stats, 0, sizeof(st->stats));

    if (cfg->num_channels > 0 && cfg->num_channels <= POCSAG_MAX_CHANNELS) {
        // Multi-channel mode
        st->num_channels = cfg->num_channels;
        for (int i = 0; i < st->num_channels; i++) {
            channel_state_init(&st->channels[i], cfg->channel_freqs[i],
                               cfg->center_freq, cfg->sample_rate);
            fprintf(stderr, "  POCSAG ch%d: %.3f MHz (offset %+.1f kHz)\n",
                    i, cfg->channel_freqs[i] / 1e6,
                    (cfg->channel_freqs[i] - cfg->center_freq) / 1e3);
        }
    } else {
        // Legacy single-channel mode: center_freq IS the channel freq
        st->num_channels = 1;
        channel_state_init(&st->channels[0], cfg->center_freq,
                           cfg->center_freq, cfg->sample_rate);
    }

    return st;
}

void pocsag_destroy(struct pocsag_state *st)
{
    if (!st) return;

    // Flush any in-progress messages on all channels
    for (int c = 0; c < st->num_channels; c++) {
        channel_state_t *ch = &st->channels[c];
        for (int i = 0; i < NUM_BAUD_RATES; i++) {
            if (ch->baud[i].in_message) {
                deliver_message(st, &ch->baud[i], 0.0f, ch->channel_freq);
            }
        }
    }

    free(st);
}

void pocsag_process(struct pocsag_state *st, const uint8_t *iq_data, uint32_t len)
{
    if (!st || !iq_data || len < 2) return;

    // Process I/Q sample pairs
    for (uint32_t k = 0; k + 1 < len; k += 2) {
        float i_raw = ((float)iq_data[k]     - 127.5f) / 127.5f;
        float q_raw = ((float)iq_data[k + 1] - 127.5f) / 127.5f;

        // Feed each channel
        for (int c = 0; c < st->num_channels; c++) {
            channel_state_t *ch = &st->channels[c];

            float i_val, q_val;

            if (st->num_channels == 1 && ch->freq_offset == 0.0) {
                // Single-channel legacy: no mixing needed
                i_val = i_raw;
                q_val = q_raw;
            } else {
                // Frequency-translate: mix with NCO to shift channel to baseband
                float cos_nco = cosf((float)ch->nco_phase);
                float sin_nco = sinf((float)ch->nco_phase);

                i_val = i_raw * cos_nco - q_raw * sin_nco;
                q_val = i_raw * sin_nco + q_raw * cos_nco;

                ch->nco_phase += ch->nco_phase_inc;
                // Keep phase in [-pi, pi] to avoid float precision loss
                if (ch->nco_phase > M_PI) ch->nco_phase -= 2.0 * M_PI;
                else if (ch->nco_phase < -M_PI) ch->nco_phase += 2.0 * M_PI;
            }

            // Per-channel signal level estimation
            float mag = i_val * i_val + q_val * q_val;
            ch->signal_acc += mag;
            ch->sample_count++;

            // FM discriminator
            float fm = fm_discriminator(ch->prev_i, ch->prev_q, i_val, q_val);
            ch->prev_i = i_val;
            ch->prev_q = q_val;

            // Low-pass filter (anti-aliasing before decimation)
            ch->lpf_out += ch->lpf_alpha * (fm - ch->lpf_out);

            // Decimation: accumulate POCSAG_DECIMATION samples, then process
            ch->decim_acc += ch->lpf_out;
            ch->decim_count++;

            if (ch->decim_count >= POCSAG_DECIMATION) {
                float decimated = ch->decim_acc / POCSAG_DECIMATION;
                ch->decim_acc = 0;
                ch->decim_count = 0;

                // DC offset removal on decimated stream
                ch->dc_avg += 0.001f * (decimated - ch->dc_avg);
                float sample = decimated - ch->dc_avg;

                // Feed to each baud-rate decoder (now at decimated rate)
                for (int b = 0; b < NUM_BAUD_RATES; b++) {
                    baud_state_t *bs = &ch->baud[b];

                    bs->clock_phase += 1.0;

                    if (bs->clock_phase >= bs->clock_freq) {
                        bs->clock_phase -= bs->clock_freq;

                        // Bit decision: FSK — positive = 1, negative = 0
                        int bit = (sample > 0.0f) ? 1 : 0;

                        float sig = (ch->sample_count > 0) ?
                            10.0f * log10f(ch->signal_acc / ch->sample_count + 1e-10f) : -40.0f;

                        process_bit(st, bs, bit, sig, ch->channel_freq);
                    }

                    // Clock recovery: nudge phase based on zero crossings
                    if ((ch->prev_sample[b] > 0 && sample < 0) ||
                        (ch->prev_sample[b] < 0 && sample > 0)) {
                        double phase_error = bs->clock_phase - (bs->clock_freq / 2.0);
                        bs->clock_phase -= phase_error * 0.1;
                    }
                    ch->prev_sample[b] = sample;
                }
            }
        }

        st->stats.samples_processed++;
    }
}

void pocsag_get_stats(const struct pocsag_state *st, pocsag_stats_t *out)
{
    if (!st || !out) return;
    *out = st->stats;
}
