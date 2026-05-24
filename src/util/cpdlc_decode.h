// SPDX-License-Identifier: GPL-3.0-or-later
//
// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// cpdlc_decode.h: FANS-1/A CPDLC message decoder (ASN.1 UPER)
//

#ifndef CPDLC_DECODE_H
#define CPDLC_DECODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Try to decode a non-ACARS ELM payload as FANS-1/A CPDLC.
// Returns 1 if successfully decoded (output printed to stdout), 0 if not CPDLC.
int32_t cpdlc_try_decode(uint32_t addr, const uint8_t *data, int32_t len);

// Direction-aware version: dir = 0 (downlink/DM), 1 (uplink/UM), -1 (try both)
int32_t cpdlc_try_decode_dir(uint32_t addr, const uint8_t *data, int32_t len, int32_t dir);

#ifdef __cplusplus
}
#endif

#endif
