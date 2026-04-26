// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// cpdlc_decode.h: FANS-1/A CPDLC message decoder (ASN.1 UPER)
//

#ifndef CPDLC_DECODE_H
#define CPDLC_DECODE_H

#include <stdint.h>

// Try to decode a non-ACARS ELM payload as FANS-1/A CPDLC.
// Returns 1 if successfully decoded (output printed to stdout), 0 if not CPDLC.
int cpdlc_try_decode(uint32_t addr, const unsigned char *data, int len);

// Direction-aware version: dir = 0 (downlink/DM), 1 (uplink/UM), -1 (try both)
int cpdlc_try_decode_dir(uint32_t addr, const unsigned char *data, int len, int dir);

#endif
