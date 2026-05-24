// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// mode_s.h: Mode S message decoding (prototypes)
//
// Copyright (c) 2017-2021 FlightAware, LLC
// Copyright (c) 2017 Oliver Jowett <oliver@mutability.co.uk>
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef MODE_S_H
#define MODE_S_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <assert.h>

//
// Functions exported from mode_s.c
//

// Possible return values of scoreModesMessage,
// ordered from worst to best
typedef enum {
    SR_NOT_SET = 0,               // message has not been scored yet

    SR_ALL_ZEROS,                 // a message that's all zeros
    SR_UNKNOWN_DF,                // message with unrecognized DF
    SR_UNCORRECTABLE,             // message with uncorrectable errors

    SR_UNKNOWN_THRESHOLD,         // cutoff for message that might be valid, but don't match an existing aircraft

    SR_UNRELIABLE_UNKNOWN,        // Address/Parity,                 unknown aircraft

    SR_DF11_IID_1ERROR_UNKNOWN,   // DF11, non-zero IID, 1 error,    unknown aircraft
    SR_DF11_ACQ_1ERROR_UNKNOWN,   // DF11, zero IID,     1 error,    unknown aircraft
    SR_DF11_IID_UNKNOWN,          // DF11, non-zero IID, no errors,  unknown aircraft

    SR_DF18_2ERROR_UNKNOWN,       // DF17,               2 errors,   unknown aircraft
    SR_DF17_2ERROR_UNKNOWN,       // DF17,               2 errors,   unknown aircraft

    SR_ACCEPT_THRESHOLD,          // cutoff for accepting messages

    // Address/Parity is unreliable, prefer anything else but this
    SR_UNRELIABLE_KNOWN,          // Address/Parity,                 known aircraft

    // 2-bit error correction is quite unreliable, put it low down the ranking even for known aircraft
    SR_DF18_2ERROR_KNOWN,         // DF18,               2 errors,   known aircraft
    SR_DF17_2ERROR_KNOWN,         // DF17,               2 errors,   known aircraft

    // 1-bit error when we haven't previously seen anything from this address, low priority
    SR_DF18_1ERROR_UNKNOWN,       // DF18,               1 error,    unknown aircraft
    SR_DF17_1ERROR_UNKNOWN,       // DF17,               1 error,    unknown aircraft

    // We need to accept at least one non-ES message type from unknown aircraft
    // or else we'd never accept message from Mode-S only aircraft
    SR_DF11_ACQ_UNKNOWN,          // DF11, zero IID,     no errors,  unknown aircraft

    SR_DF11_IID_1ERROR_KNOWN,     // DF11, non-zero IID, 1 error,    known aircraft
    SR_DF11_ACQ_1ERROR_KNOWN,     // DF11, zero IID,     1 error,    known aircraft
    SR_DF11_IID_KNOWN,            // DF11, non-zero IID, no errors,  known aircraft

    SR_DF18_1ERROR_KNOWN,         // DF18,               1 error,    known aircraft
    SR_DF17_1ERROR_KNOWN,         // DF17,               1 error,    known aircraft

    SR_DF11_ACQ_KNOWN,            // DF11, zero IID,     no errors,  known aircraft

    SR_DF18_UNKNOWN,              // DF18,               no errors,  unknown aircraft
    SR_DF17_UNKNOWN,              // DF17,               no errors,  unknown aircraft
    SR_DF18_KNOWN,                // DF18,               no errors,  known aircraft
    SR_DF17_KNOWN,                // DF17,               no errors,  known aircraft
} score_rank;

int32_t modesMessageLenByType(int32_t type);
score_rank scoreModesMessage(const uint8_t *msg);
int32_t decodeModesMessage (struct modesMessage *mm, const uint8_t *msg);
void displayModesMessage(struct modesMessage *mm);
void useModesMessage    (struct modesMessage *mm);

// datafield extraction helpers

// The first bit (MSB of the first byte) is numbered 1, for consistency
// with how the specs number them.

// Extract one bit from a message.
static inline  __attribute__((always_inline)) uint32_t getbit(const uint8_t *data, uint32_t bitnum)
{
    uint32_t bi = bitnum - 1;
    uint32_t by = bi >> 3;
    uint32_t mask = 1 << (7 - (bi & 7));

    return (data[by] & mask) != 0;
}

// Extract some bits (firstbit .. lastbit inclusive) from a message.
static inline  __attribute__((always_inline)) uint32_t getbits(const uint8_t *data, uint32_t firstbit, uint32_t lastbit)
{
    uint32_t fbi = firstbit - 1;
    uint32_t lbi = lastbit - 1;
    uint32_t nbi = (lastbit - firstbit + 1);

    uint32_t fby = fbi >> 3;
    uint32_t lby = lbi >> 3;
    uint32_t nby = (lby - fby) + 1;

    uint32_t shift = 7 - (lbi & 7);
    uint32_t topmask = 0xFF >> (fbi & 7);

    assert (fbi <= lbi);
    assert (nbi <= 32);
    assert (nby <= 5);

    if (nby == 5) {
        return
            ((data[fby] & topmask) << (32 - shift)) |
            (data[fby + 1] << (24 - shift)) |
            (data[fby + 2] << (16 - shift)) |
            (data[fby + 3] << (8 - shift)) |
            (data[fby + 4] >> shift);
    } else if (nby == 4) {
        return
            ((data[fby] & topmask) << (24 - shift)) |
            (data[fby + 1] << (16 - shift)) |
            (data[fby + 2] << (8 - shift)) |
            (data[fby + 3] >> shift);
    } else if (nby == 3) {
        return
            ((data[fby] & topmask) << (16 - shift)) |
            (data[fby + 1] << (8 - shift)) |
            (data[fby + 2] >> shift);
    } else if (nby == 2) {
        return
            ((data[fby] & topmask) << (8 - shift)) |
            (data[fby + 1] >> shift);
    } else if (nby == 1) {
        return
            (data[fby] & topmask) >> shift;
    } else {
        return 0;
    }
}

#ifdef __cplusplus
}
#endif

#endif
