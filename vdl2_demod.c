// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// vdl2_demod.c: VDL Mode 2 D8PSK demodulator
//
// Demodulation pipeline:
//   IQ samples (2.0 MSPS) → frequency shift to baseband → decimate to ~126 kHz
//   → D8PSK demodulation (phase tracking + differential decode)
//   → AVLC deframing (flag detection, bit unstuffing, FCS)
//   → ACARS message extraction
//
// VDL2 uses Differential 8-PSK at 31500 symbols/sec, 3 bits per symbol.
// AVLC framing is HDLC-like with 0x7E flags and bit stuffing.
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include "vdl2_demod.h"

// ======================== Constants ========================

// Decimation: from sample_rate to ~4x symbol rate
// 2.0 MHz / 16 = 125 kHz ≈ 4 * 31500
#define VDL2_DECIM        16
#define VDL2_IF_RATE      (2000000 / VDL2_DECIM)  // 125000 Hz
#define VDL2_SPS          (VDL2_IF_RATE / VDL2_SYMBOL_RATE)  // ~3.97 samples/symbol

// AVLC flag
#define AVLC_FLAG         0x7E

// FCS polynomial (CRC-CCITT)
#define FCS_INIT          0xFFFF
#define FCS_GOOD          0xF0B8  // Expected residue after processing frame + FCS

// Lowpass filter taps for decimation
#define VDL2_LPF_TAPS     33

// Phase states for D8PSK (8 phases, 0..7)
// Gray coding: phase_change → 3 bits (standard 3-bit reflected Gray code)
static const uint8_t dpsk_gray[8] = { 0, 1, 3, 2, 6, 7, 5, 4 };

// ======================== FCS (CRC-16 CCITT) ========================

static uint16_t fcs_table[256];
static int fcs_table_init = 0;

static void init_fcs_table(void)
{
    for (int i = 0; i < 256; i++) {
        uint16_t crc = (uint16_t)i;
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8408; // Reversed polynomial
            else
                crc >>= 1;
        }
        fcs_table[i] = crc;
    }
    fcs_table_init = 1;
}

static uint16_t fcs_update(uint16_t fcs, uint8_t byte)
{
    return (fcs >> 8) ^ fcs_table[(fcs ^ byte) & 0xFF];
}

// ======================== Decimation lowpass filter ========================

static float vdl2_lpf[VDL2_LPF_TAPS];
static int vdl2_lpf_init = 0;

static void init_vdl2_lpf(void)
{
    // Blackman-windowed sinc lowpass, cutoff at ~40 kHz
    // Filter applied at sample_rate (2 MHz), normalized to Nyquist = sample_rate/2
    int M = VDL2_LPF_TAPS - 1;
    double fc = 40000.0 / ((double)VDL2_IF_RATE * VDL2_DECIM / 2.0);  // Normalized cutoff
    double sum = 0;
    for (int i = 0; i < VDL2_LPF_TAPS; i++) {
        double n = i - M / 2.0;
        double h;
        if (fabs(n) < 1e-10)
            h = 2.0 * fc;
        else
            h = sin(2.0 * M_PI * fc * n) / (M_PI * n);
        // Blackman window
        double w = 0.42 - 0.5 * cos(2.0 * M_PI * i / M) + 0.08 * cos(4.0 * M_PI * i / M);
        vdl2_lpf[i] = (float)(h * w);
        sum += vdl2_lpf[i];
    }
    // Normalize
    for (int i = 0; i < VDL2_LPF_TAPS; i++)
        vdl2_lpf[i] /= (float)sum;

    vdl2_lpf_init = 1;
}

// ======================== State ========================

typedef enum {
    VDL2_HUNT = 0,     // Searching for preamble/flag
    VDL2_SYNC,         // Got flag, receiving frame
} vdl2_rx_state_t;

struct vdl2_state {
    vdl2_config_t config;

    // Mixer LO for frequency shift
    float complex *mixer_lo;
    int      mixer_len;
    int      mixer_pos;

    // Decimation filter state
    float complex decim_buf[VDL2_LPF_TAPS];
    int      decim_idx;
    int      decim_count;

    // D8PSK demodulator
    float complex prev_symbol;   // Previous symbol for differential decode
    float    sym_clock;          // Symbol clock accumulator
    float    sym_rate;           // Samples per symbol

    // Bit assembly
    uint8_t  shift_reg;          // 8-bit shift register for flag/data detection
    int      bit_count;          // Bits in current byte
    int      ones_count;         // Consecutive 1s for bit unstuffing

    // AVLC frame assembly
    vdl2_rx_state_t rx_state;
    uint8_t  frame_buf[VDL2_MAX_FRAME_LEN];
    int      frame_len;
    int      frame_bit_idx;

    // Stats
    vdl2_stats_t stats;
};

// ======================== Create / Destroy ========================

struct vdl2_state *vdl2_create(const vdl2_config_t *config)
{
    struct vdl2_state *s = calloc(1, sizeof(struct vdl2_state));
    if (!s) return NULL;

    s->config = *config;

    if (!fcs_table_init) init_fcs_table();
    if (!vdl2_lpf_init) init_vdl2_lpf();

    // Precompute mixer LO (period = sample_rate / gcd(sample_rate, offset))
    // For simplicity, use a block of VDL2_DECIM samples and repeat
    double f_offset = (config->channel_freqs[0] - config->center_freq);
    s->mixer_len = VDL2_DECIM;
    s->mixer_lo = calloc((unsigned)s->mixer_len, sizeof(float complex));
    if (!s->mixer_lo) { free(s); return NULL; }

    for (int k = 0; k < s->mixer_len; k++) {
        double phase = -2.0 * M_PI * f_offset * k / config->sample_rate;
        s->mixer_lo[k] = (float complex)cexp(I * phase);
    }
    s->mixer_pos = 0;

    s->prev_symbol = 1.0f + 0.0f * I;
    s->sym_rate = (float)VDL2_IF_RATE / VDL2_SYMBOL_RATE;
    s->sym_clock = 0;

    s->rx_state = VDL2_HUNT;
    s->shift_reg = 0;
    s->ones_count = 0;
    s->frame_len = 0;

    fprintf(stderr, "VDL2: freq=%.3f MHz, sr=%.0f, decim=%d, IF=%d, sps=%.2f\n",
            config->channel_freqs[0] / 1e6, config->sample_rate,
            VDL2_DECIM, VDL2_IF_RATE, (double)s->sym_rate);

    return s;
}

void vdl2_destroy(struct vdl2_state *state)
{
    if (!state) return;
    free(state->mixer_lo);
    free(state);
}

void vdl2_get_stats(struct vdl2_state *state, vdl2_stats_t *stats)
{
    *stats = state->stats;
}

// ======================== ACARS extraction from AVLC ========================

static void vdl2_extract_acars(struct vdl2_state *state, const uint8_t *frame, int len)
{
    // AVLC frame structure:
    // Bytes 0-3: Address field (2 bytes src + 2 bytes dst)
    // Byte 4: Control field
    // Bytes 5+: Information field (may contain ACARS)
    // Last 2 bytes: FCS (already verified)

    if (len < 8) return;  // Too short for meaningful content

    int info_start = 4;  // Skip address + control
    // Check for extended address
    if (!(frame[1] & 0x01)) info_start = 5;  // Extended address
    if (info_start >= len - 2) return;

    int info_len = len - 2 - info_start;  // Subtract FCS
    if (info_len < 5) return;

    // Look for ACARS-like content in the information field
    // ACARS over AVLC: the info field contains an ACARS message
    const uint8_t *info = &frame[info_start];

    vdl2_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.freq = state->config.channel_freqs[0];
    msg.info_len = info_len;

    // Extract AVLC addresses (3 bytes each, big-endian 24-bit)
    if (len >= 4) {
        msg.dst.addr = ((uint32_t)(frame[0] & 0xFE) << 16) |
                       ((uint32_t)frame[1] << 9) |
                       ((uint32_t)frame[2] << 1) |
                       ((uint32_t)(frame[3] >> 7));
        // For extended address, source is in bytes 4-6
        if (info_start == 5 && len >= 7) {
            msg.src.addr = ((uint32_t)(frame[4] & 0xFE) << 16) |
                           ((uint32_t)frame[5] << 9) |
                           ((uint32_t)frame[6] << 1) |
                           ((uint32_t)(frame[7] >> 7));
        }
    }

    // Determine frame type from control byte
    uint8_t ctrl = frame[info_start - 1];
    if ((ctrl & 0x01) == 0)
        snprintf(msg.frame_type, sizeof(msg.frame_type), "I");
    else if ((ctrl & 0x03) == 0x01)
        snprintf(msg.frame_type, sizeof(msg.frame_type), "S");
    else
        snprintf(msg.frame_type, sizeof(msg.frame_type), "U");

    // Try to extract ACARS fields
    // Look for SOH (0x01) marker
    int acars_start = -1;
    for (int i = 0; i < info_len - 5; i++) {
        if (info[i] == 0x01) {  // SOH
            acars_start = i + 1;
            break;
        }
    }

    if (acars_start >= 0 && acars_start + 12 < info_len) {
        msg.has_acars = true;
        // Standard ACARS format after SOH
        int p = acars_start;

        // Skip mode byte
        if (p < info_len) p++;

        // Registration (7 chars)
        for (int i = 0; i < 7 && p < info_len; i++)
            msg.reg[i] = info[p++] & 0x7F;

        // Skip ACK
        if (p < info_len) p++;

        // Label (2 chars)
        if (p + 1 < info_len) {
            msg.label[0] = info[p++] & 0x7F;
            msg.label[1] = info[p++] & 0x7F;
        }

        // Skip block_id and STX
        if (p < info_len) p++;  // block_id
        if (p < info_len) p++;  // STX

        // Text
        int tlen = 0;
        while (p < info_len && tlen < VDL2_MAX_MSG_LEN) {
            uint8_t c = info[p++];
            if (c == 0x03 || c == 0x83 || c == 0x17 || c == 0x97)
                break;  // ETX or ETB
            msg.text[tlen++] = c & 0x7F;
        }
        msg.text[tlen] = '\0';
        msg.text_len = tlen;
    } else {
        // No ACARS structure found, dump raw info as hex summary
        int tlen = 0;
        for (int i = 0; i < info_len && tlen < VDL2_MAX_MSG_LEN - 3; i++) {
            int n = snprintf(msg.text + tlen, (unsigned)(VDL2_MAX_MSG_LEN - tlen),
                             "%02X ", info[i]);
            if (n > 0) tlen += n;
        }
        msg.text[tlen] = '\0';
        msg.text_len = tlen;
    }

    state->stats.messages_decoded++;

    if (state->config.callback)
        state->config.callback(&msg, state->config.callback_ctx);
}

// ======================== AVLC Frame Processing ========================

static void vdl2_process_frame(struct vdl2_state *state)
{
    int len = state->frame_len;
    if (len < 6) return;  // Min: 4 addr + 2 FCS

    state->stats.frames_detected++;

    // Verify FCS
    uint16_t fcs = FCS_INIT;
    for (int i = 0; i < len; i++)
        fcs = fcs_update(fcs, state->frame_buf[i]);

    if (fcs != FCS_GOOD) {
        state->stats.fcs_errors++;
        return;
    }

    vdl2_extract_acars(state, state->frame_buf, len);
}

// ======================== Bit Processing (AVLC deframing) ========================

static void vdl2_process_bit(struct vdl2_state *state, int bit)
{
    state->shift_reg = (uint8_t)((state->shift_reg >> 1) | (bit ? 0x80 : 0));

    if (state->shift_reg == AVLC_FLAG) {
        // Flag detected
        if (state->rx_state == VDL2_SYNC && state->frame_len > 0) {
            // End of frame
            vdl2_process_frame(state);
        }
        // Start new frame
        state->rx_state = VDL2_SYNC;
        state->frame_len = 0;
        state->frame_bit_idx = 0;
        state->ones_count = 0;
        state->bit_count = 0;
        return;
    }

    if (state->rx_state != VDL2_SYNC) return;

    // Check for abort (7+ consecutive 1s)
    if (bit) {
        state->ones_count++;
        if (state->ones_count >= 7) {
            state->rx_state = VDL2_HUNT;
            return;
        }
    }

    // Bit unstuffing: after 5 consecutive 1s, a 0 is stuffed and should be removed
    if (state->ones_count == 5) {
        state->ones_count = 0;
        if (!bit) return;  // Remove stuffed 0
        // 6 consecutive 1s: could be flag or abort, handled above
    }
    if (!bit) state->ones_count = 0;

    // Assemble byte
    if (state->frame_len < VDL2_MAX_FRAME_LEN) {
        if (bit)
            state->frame_buf[state->frame_len] |= (uint8_t)(1 << state->bit_count);

        state->bit_count++;
        if (state->bit_count >= 8) {
            state->frame_len++;
            state->bit_count = 0;
            if (state->frame_len < VDL2_MAX_FRAME_LEN)
                state->frame_buf[state->frame_len] = 0;
        }
    }
}

// ======================== D8PSK Symbol Processing ========================

static void vdl2_process_symbol(struct vdl2_state *state, float complex sym)
{
    // Normalize
    float mag = cabsf(sym);
    if (mag < 1e-10f) return;
    sym /= mag;

    // Differential decode: multiply by conjugate of previous symbol
    float complex diff = sym * conjf(state->prev_symbol);
    state->prev_symbol = sym;

    // Get phase of differential symbol (0..2*PI)
    float phase = cargf(diff);
    if (phase < 0) phase += 2.0f * (float)M_PI;

    // Quantize to nearest octant (0..7)
    int octant = (int)(phase / ((float)M_PI / 4.0f) + 0.5f) % 8;

    // Gray decode to 3 bits
    uint8_t bits3 = dpsk_gray[octant];

    // Output 3 bits (MSB first)
    vdl2_process_bit(state, (bits3 >> 2) & 1);
    vdl2_process_bit(state, (bits3 >> 1) & 1);
    vdl2_process_bit(state, bits3 & 1);
}

// ======================== IQ Processing ========================

void vdl2_process(struct vdl2_state *state, const uint8_t *iq_data, unsigned len)
{
    unsigned samples = len / 2;

    state->stats.samples_processed += samples;

    for (unsigned i = 0; i < samples; i++) {
        // Convert UC8 to float complex
        float r = (float)iq_data[i * 2]     - 127.5f;
        float g = (float)iq_data[i * 2 + 1] - 127.5f;
        float complex z = (r + g * I);

        // Frequency shift to baseband
        z *= state->mixer_lo[state->mixer_pos];
        state->mixer_pos = (state->mixer_pos + 1) % state->mixer_len;

        // Decimation filter
        state->decim_buf[state->decim_idx] = z;
        state->decim_idx = (state->decim_idx + 1) % VDL2_LPF_TAPS;
        state->decim_count++;

        if (state->decim_count >= VDL2_DECIM) {
            state->decim_count = 0;

            // Apply lowpass filter
            float complex filtered = 0;
            for (int k = 0; k < VDL2_LPF_TAPS; k++) {
                filtered += state->decim_buf[(state->decim_idx + k) % VDL2_LPF_TAPS]
                            * vdl2_lpf[k];
            }

            // Symbol timing: accumulate until we have one symbol's worth
            state->sym_clock += 1.0f;
            if (state->sym_clock >= state->sym_rate) {
                state->sym_clock -= state->sym_rate;
                vdl2_process_symbol(state, filtered);
            }
        }
    }
}
