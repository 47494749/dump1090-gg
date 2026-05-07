// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// acars_demod.c: ACARS AM-MSK demodulator
//
// Demodulation pipeline:
//   IQ samples (2.0 MSPS) → per-channel mixer+decimate to 12.5 kHz
//   → AM envelope → MSK demodulator (VCO + matched filter + PLL)
//   → bit assembly → ACARS framing (SYN-SYN-SOH-text-ETX-CRC)
//   → CRC-16 check → decoded message output
//
// Based on algorithms from acarsdec by Thierry Leconte (GPLv2)
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
#include "acars_demod.h"

// ======================== Constants ========================

#define SYN_CHAR 0x16
#define SOH_CHAR 0x01
#define STX_CHAR 0x02
#define ETX_CHAR 0x83
#define ETB_CHAR 0x97
#define DEL_CHAR 0x7f

// MSK filter parameters
#define MFLT_LEN    ((ACARS_INTRATE / 1200) + 1)
#define MFLT_OVER   12
#define MFLT_TOTAL  (MFLT_LEN * MFLT_OVER + 1)

// PLL constants (from acarsdec)
#define PLL_GAIN    38e-4f
#define PLL_COEFF   0.52f

#define MAXPERR 3  // Max parity errors to attempt correction

// ======================== ACARS CRC-16/IBM (poly 0xA001) ========================

static const uint16_t crc_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

#define UPDATE_CRC(crc, c) \
    (crc) = ((crc) >> 8) ^ crc_table[((crc) ^ (c)) & 0xFF]

// Number of 1-bits in a byte (for parity checking)
static const uint8_t numbits[256] = {
    0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4,1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,
    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
    3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,4,5,5,6,5,6,6,7,5,6,6,7,6,7,7,8
};



// ======================== Per-channel state ========================

typedef enum {
    ACARS_WSYN = 0,   // Waiting for SYN
    ACARS_SYN2,       // Got first SYN, waiting for second
    ACARS_SOH1,       // Got SYN-SYN, waiting for SOH
    ACARS_TXT,        // Receiving message text
    ACARS_CRC1,       // Waiting for CRC byte 1
    ACARS_CRC2,       // Waiting for CRC byte 2
    ACARS_END         // Message complete
} acars_frame_state_t;

typedef struct {
    int      chn;           // Channel index
    double   freq;          // Channel frequency (Hz)

    // Channelizer: complex mixer weights (precomputed)
    float complex *mixer;   // mixer[decim_factor] = exp(-j*2*pi*f_offset*n)
    int      decim_factor;  // Decimation factor

    // Decimated AM envelope buffer
    float   *dm_buffer;     // Decimated AM samples
    int      dm_len;        // Number of decimated samples in current block

    // MSK demodulator state
    double   MskPhi;        // VCO phase
    double   MskDf;         // PLL frequency correction
    float    MskClk;        // Bit clock accumulator
    double   MskLvlSum;     // Signal level accumulator
    int      MskBitCount;   // Signal level sample count
    unsigned MskS;          // Symbol counter (I/Q alternation)
    int      idx;           // Matched filter input buffer index
    float complex *inb;     // Matched filter input buffer

    // Bit assembly
    unsigned char outbits;  // Shift register for bit assembly
    int      nbits;         // Bits remaining before byte complete

    // ACARS framing
    acars_frame_state_t frame_state;
    uint8_t  msg_buf[ACARS_MAX_MSGLEN + 16];
    int      msg_len;
    int      msg_err;       // Parity error count
    uint8_t  crc_bytes[2];
} acars_channel_t;

// ======================== Main state ========================

struct acars_state {
    acars_config_t config;
    acars_channel_t channels[ACARS_MAX_CHANNELS];

    // MSK matched filter (shared across channels)
    float    msk_filter[MFLT_TOTAL];

    // Stats
    acars_stats_t stats;

    // IQ buffer for block processing
    int      iq_block_size;  // IQ samples per decimation block
};

// ======================== Create / Destroy ========================

struct acars_state *acars_create(const acars_config_t *config)
{
    struct acars_state *s = calloc(1, sizeof(struct acars_state));
    if (!s) return NULL;

    s->config = *config;

    // Compute decimation factor
    int decim = (int)(config->sample_rate / ACARS_INTRATE + 0.5);
    if (decim < 1) decim = 1;
    s->iq_block_size = decim;

    fprintf(stderr, "ACARS: sample_rate=%.0f, intrate=%d, decim=%d, channels=%d\n",
            config->sample_rate, ACARS_INTRATE, decim, config->num_channels);

    // Compute MSK matched filter (cosine at 600 Hz)
    for (int i = 0; i < MFLT_TOTAL; i++) {
        float v = cosf(2.0f * (float)M_PI * 600.0f / ACARS_INTRATE / MFLT_OVER *
                        (float)(i - (MFLT_TOTAL - 1) / 2));
        s->msk_filter[i] = (v < 0) ? 0 : v;
    }

    // Initialize per-channel state
    for (int n = 0; n < config->num_channels; n++) {
        acars_channel_t *ch = &s->channels[n];
        ch->chn = n;
        ch->freq = config->channel_freqs[n];
        ch->decim_factor = decim;

        // Allocate mixer weights
        ch->mixer = calloc((unsigned)decim, sizeof(float complex));
        if (!ch->mixer) { acars_destroy(s); return NULL; }

        // Precompute mixer: e^(-j*2*pi*f_offset*n / sample_rate) / (decim * 127.5)
        double f_offset = (ch->freq - config->center_freq) / config->sample_rate * 2.0 * M_PI;
        double norm = 1.0 / (decim * 127.5);
        for (int k = 0; k < decim; k++) {
            ch->mixer[k] = (float complex)(norm * cexp(-I * f_offset * k));
        }

        // Allocate decimated buffer
        ch->dm_buffer = calloc(ACARS_DECIM_BUFSZ, sizeof(float));
        if (!ch->dm_buffer) { acars_destroy(s); return NULL; }

        // Allocate matched filter input buffer
        ch->inb = calloc((unsigned)MFLT_LEN, sizeof(float complex));
        if (!ch->inb) { acars_destroy(s); return NULL; }

        // Initial state
        ch->MskPhi = 0;
        ch->MskDf = 0;
        ch->MskClk = 0;
        ch->MskS = 0;
        ch->idx = 0;
        ch->outbits = 0;
        ch->nbits = 1;
        ch->frame_state = ACARS_WSYN;
        ch->msg_len = 0;
        ch->msg_err = 0;

        fprintf(stderr, "ACARS: channel %d: %.3f MHz, offset=%.0f Hz\n",
                n, ch->freq / 1e6, ch->freq - config->center_freq);
    }

    return s;
}

void acars_destroy(struct acars_state *state)
{
    if (!state) return;
    for (int n = 0; n < state->config.num_channels; n++) {
        free(state->channels[n].mixer);
        free(state->channels[n].dm_buffer);
        free(state->channels[n].inb);
    }
    free(state);
}

void acars_get_stats(struct acars_state *state, acars_stats_t *stats)
{
    *stats = state->stats;
}

// ======================== ACARS Message Output ========================

static void acars_output_message(struct acars_state *state, acars_channel_t *ch)
{
    acars_msg_t msg;
    memset(&msg, 0, sizeof(msg));

    msg.channel = ch->chn;
    msg.freq = ch->freq;
    msg.level = (ch->MskBitCount > 0)
        ? 10.0f * log10f((float)(ch->MskLvlSum / ch->MskBitCount))
        : -99.0f;
    msg.errors = ch->msg_err;

    if (ch->msg_len < 13) return;  // Too short

    // Parse ACARS fields from message buffer
    // Byte 0: mode, 1-7: reg, 8: ack, 9-10: label, 11: block_id, 12: STX/ETX
    msg.mode = ch->msg_buf[0] & 0x7F;

    for (int i = 0; i < 7 && i + 1 < ch->msg_len; i++)
        msg.reg[i] = ch->msg_buf[i + 1] & 0x7F;
    msg.reg[7] = '\0';

    msg.ack = (ch->msg_len > 8) ? (ch->msg_buf[8] & 0x7F) : ' ';

    if (ch->msg_len > 9)  msg.label[0] = ch->msg_buf[9]  & 0x7F;
    if (ch->msg_len > 10) msg.label[1] = ch->msg_buf[10] & 0x7F;
    msg.label[2] = '\0';

    msg.block_id = (ch->msg_len > 11) ? (ch->msg_buf[11] & 0x7F) : ' ';

    // If there's text after STX (byte 12)
    int text_start = 13;
    int text_end = ch->msg_len;

    // Extract message number and flight from beginning of text
    if (text_end - text_start >= 4) {
        for (int i = 0; i < 4 && text_start + i < text_end; i++)
            msg.msgno[i] = ch->msg_buf[text_start + i] & 0x7F;
        msg.msgno[4] = '\0';
        text_start += 4;
    }
    if (text_end - text_start >= 6) {
        for (int i = 0; i < 6 && text_start + i < text_end; i++)
            msg.flight[i] = ch->msg_buf[text_start + i] & 0x7F;
        msg.flight[6] = '\0';
        text_start += 6;
    }

    // Remaining text
    int tlen = text_end - text_start;
    if (tlen > ACARS_MAX_MSGLEN) tlen = ACARS_MAX_MSGLEN;
    if (tlen > 0) {
        for (int i = 0; i < tlen; i++)
            msg.text[i] = ch->msg_buf[text_start + i] & 0x7F;
    }
    msg.text[tlen > 0 ? tlen : 0] = '\0';
    msg.text_len = tlen > 0 ? tlen : 0;

    state->stats.messages_decoded++;

    if (state->config.callback) {
        state->config.callback(&msg, state->config.callback_ctx);
    }
}

// ======================== ACARS Frame Decoder ========================

static void acars_reset_frame(acars_channel_t *ch)
{
    ch->frame_state = ACARS_WSYN;
    ch->MskDf = 0;
    ch->nbits = 1;
}

static void acars_decode_byte(struct acars_state *state, acars_channel_t *ch)
{
    unsigned char r = ch->outbits;

    switch (ch->frame_state) {
    case ACARS_WSYN:
        if (r == SYN_CHAR) {
            ch->frame_state = ACARS_SYN2;
            ch->nbits = 8;
            return;
        }
        if (r == (unsigned char)~SYN_CHAR) {
            ch->MskS ^= 2;
            ch->frame_state = ACARS_SYN2;
            ch->nbits = 8;
            return;
        }
        ch->nbits = 1;
        return;

    case ACARS_SYN2:
        if (r == SYN_CHAR) {
            ch->frame_state = ACARS_SOH1;
            ch->nbits = 8;
            return;
        }
        if (r == (unsigned char)~SYN_CHAR) {
            ch->MskS ^= 2;
            ch->nbits = 8;
            return;
        }
        acars_reset_frame(ch);
        return;

    case ACARS_SOH1:
        if (r == SOH_CHAR) {
            ch->frame_state = ACARS_TXT;
            ch->msg_len = 0;
            ch->msg_err = 0;
            ch->nbits = 8;
            ch->MskLvlSum = 0;
            ch->MskBitCount = 0;
            return;
        }
        acars_reset_frame(ch);
        return;

    case ACARS_TXT:
        ch->msg_buf[ch->msg_len] = r;
        ch->msg_len++;
        if ((numbits[r] & 1) == 0) {
            ch->msg_err++;
            if (ch->msg_err > MAXPERR + 1) {
                acars_reset_frame(ch);
                return;
            }
        }
        if (r == ETX_CHAR || r == ETB_CHAR) {
            ch->frame_state = ACARS_CRC1;
            ch->nbits = 8;
            return;
        }
        if (ch->msg_len > 20 && r == DEL_CHAR) {
            // Missed end marker, try to recover
            ch->msg_len -= 3;
            ch->crc_bytes[0] = ch->msg_buf[ch->msg_len];
            ch->crc_bytes[1] = ch->msg_buf[ch->msg_len + 1];
            ch->frame_state = ACARS_CRC2;
            goto process_msg;
        }
        if (ch->msg_len >= ACARS_MAX_MSGLEN) {
            acars_reset_frame(ch);
            return;
        }
        ch->nbits = 8;
        return;

    case ACARS_CRC1:
        ch->crc_bytes[0] = r;
        ch->frame_state = ACARS_CRC2;
        ch->nbits = 8;
        return;

    case ACARS_CRC2:
        ch->crc_bytes[1] = r;
    process_msg:
        {
            // Force STX/ETX markers
            if (ch->msg_len >= 13)
                ch->msg_buf[12] = (ch->msg_buf[12] & (ETX_CHAR | STX_CHAR)) |
                                  (ETX_CHAR & STX_CHAR);

            // CRC check
            uint16_t crc = 0;
            for (int i = 0; i < ch->msg_len; i++) {
                UPDATE_CRC(crc, ch->msg_buf[i]);
            }
            UPDATE_CRC(crc, ch->crc_bytes[0]);
            UPDATE_CRC(crc, ch->crc_bytes[1]);

            if (crc != 0) {
                state->stats.crc_errors++;
                // Try to fix with parity info — simplified, just reject for now
                acars_reset_frame(ch);
                ch->nbits = 8;
                return;
            }

            // Strip parity bits
            for (int i = 0; i < ch->msg_len; i++)
                ch->msg_buf[i] &= 0x7F;

            acars_output_message(state, ch);
        }
        ch->frame_state = ACARS_END;
        ch->nbits = 8;
        return;

    case ACARS_END:
        acars_reset_frame(ch);
        ch->nbits = 8;
        return;
    }
}

// ======================== MSK Demodulator (per channel) ========================

static void acars_demod_msk(struct acars_state *state, acars_channel_t *ch, int len)
{
    int idx = ch->idx;
    double p = ch->MskPhi;
    const float *h = state->msk_filter;

    for (int n = 0; n < len; n++) {
        float in;
        double s;
        float complex v;
        int j, o;

        // VCO: center frequency 1800 Hz
        s = 1800.0 / ACARS_INTRATE * 2.0 * M_PI + ch->MskDf;
        p += s;
        if (p >= 2.0 * M_PI) p -= 2.0 * M_PI;

        // Mixer: multiply AM envelope by complex VCO
        in = ch->dm_buffer[n];
        ch->inb[idx] = in * cexpf((float)(-p) * I);
        idx = (idx + 1) % MFLT_LEN;

        // Bit clock
        ch->MskClk += (float)s;
        if (ch->MskClk >= 3.0f * (float)M_PI / 2.0f - (float)s / 2.0f) {
            double dphi;
            float vo, lvl;
            ch->MskClk -= 3.0f * (float)M_PI / 2.0f;

            // Matched filter
            o = (int)(MFLT_OVER * (ch->MskClk / s + 0.5));
            if (o > MFLT_OVER) o = MFLT_OVER;
            v = 0;
            for (j = 0; j < MFLT_LEN; j++, o += MFLT_OVER) {
                v += h[o] * ch->inb[(j + idx) % MFLT_LEN];
            }

            // Normalize
            lvl = cabsf(v);
            v /= (lvl + 1e-8f);
            ch->MskLvlSum += (double)(lvl * lvl / 4.0f);
            ch->MskBitCount++;

            // Extract bit (I/Q alternation for MSK)
            if (ch->MskS & 1) {
                vo = cimagf(v);
                dphi = (vo >= 0) ? (double)-crealf(v) : (double)crealf(v);
            } else {
                vo = crealf(v);
                dphi = (vo >= 0) ? (double)cimagf(v) : (double)-cimagf(v);
            }

            // Output bit
            if (ch->MskS & 2) {
                ch->outbits >>= 1;
                if (-vo > 0) ch->outbits |= 0x80;
            } else {
                ch->outbits >>= 1;
                if (vo > 0) ch->outbits |= 0x80;
            }
            ch->MskS++;

            ch->nbits--;
            if (ch->nbits <= 0)
                acars_decode_byte(state, ch);

            // PLL filter
            ch->MskDf = PLL_COEFF * ch->MskDf + (1.0 - PLL_COEFF) * PLL_GAIN * dphi;
        }
    }

    ch->idx = idx;
    ch->MskPhi = p;
}

// ======================== IQ Processing ========================

void acars_process(struct acars_state *state, const uint8_t *iq_data, unsigned len)
{
    unsigned samples = len / 2;  // IQ pairs
    int decim = state->iq_block_size;
    int nch = state->config.num_channels;

    state->stats.samples_processed += samples;

    // Process in blocks of decim IQ samples → 1 output sample per channel
    unsigned pos = 0;
    while (pos + (unsigned)decim * 2 <= len) {
        // For each channel: mix, integrate (decimate), take AM envelope
        for (int c = 0; c < nch; c++) {
            acars_channel_t *ch = &state->channels[c];
            float complex D = 0;
            const float complex *wf = ch->mixer;

            for (int k = 0; k < decim; k++) {
                float r = (float)iq_data[pos + k * 2]     - 127.37f;
                float g = (float)iq_data[pos + k * 2 + 1] - 127.37f;
                D += (r + g * I) * wf[k];
            }

            // AM envelope detection
            ch->dm_buffer[ch->dm_len] = cabsf(D);
        }

        // All channels get the same output index
        for (int c = 0; c < nch; c++)
            state->channels[c].dm_len++;

        pos += (unsigned)(decim * 2);

        // When we have a block of decimated samples, demodulate
        if (state->channels[0].dm_len >= ACARS_DECIM_BUFSZ) {
            for (int c = 0; c < nch; c++) {
                acars_demod_msk(state, &state->channels[c], state->channels[c].dm_len);
                state->channels[c].dm_len = 0;
            }
        }
    }

    // Process remaining decimated samples
    if (state->channels[0].dm_len > 0) {
        for (int c = 0; c < nch; c++) {
            acars_demod_msk(state, &state->channels[c], state->channels[c].dm_len);
            state->channels[c].dm_len = 0;
        }
    }
}
