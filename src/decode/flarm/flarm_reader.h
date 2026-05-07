// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// flarm_reader.h: Second RTL-SDR reader for FLARM 868 MHz reception
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#ifndef FLARM_READER_H
#define FLARM_READER_H

#include <stdbool.h>
#include <stdint.h>

// FLARM reader configuration (set before calling flarmReaderInit)
typedef struct {
    int    enabled;              // FLARM reception enabled
    char   device_serial[64];   // RTL-SDR device serial number for 868 MHz dongle
    int    gain;                // Gain in tenths of dB (0 = auto)
    int    ppm_error;           // Frequency correction in PPM

    // Keys file
    char   keys_file[256];     // Path to FLARM decryption keys file
    int    flarm_ogn_only;     // If set, FLARM data goes to OGN only (not other feeders)

    // OGN station info
    char   ogn_station[32];    // OGN station name (e.g. "MYSTATION")
    char   ogn_server[128];    // OGN APRS-IS server (default: aprs.glidernet.org)
    int    ogn_port;           // OGN APRS-IS port (default: 14580)

    // Virtual IQ file input (--flarm-ifile)
    char   ifile_path[512];    // Path to raw IQ file (uint8 I/Q pairs, 1.6 MSPS)
    uint32_t ifile_time;       // Unix timestamp of file (mtime), used for XXTEA decrypt
} flarm_reader_config_t;

extern flarm_reader_config_t FlarmConfig;

// Initialize FLARM reader config defaults
void flarmReaderInitConfig(void);

// Open the second RTL-SDR and create demodulator
// Returns true on success
bool flarmReaderOpen(void);

// Start the FLARM reader thread
void flarmReaderStart(void);

// Stop the FLARM reader thread
void flarmReaderStop(void);

// Cleanup all FLARM resources
void flarmReaderClose(void);

// Called from backgroundTasks() to check FLARM packet queue and integrate into aircraft list
void flarmReaderPeriodicWork(void);

// Handle FLARM-specific CLI options
// Returns true if the option was consumed
bool flarmReaderHandleOption(int argc, char **argv, int *jptr);

// Print FLARM help
void flarmReaderShowHelp(void);

// ======================== SdrManager decoder_ops for FLARM ========================
// These allow FLARM to run as a decoder plugin under SdrManager instead of
// as a standalone subsystem with its own RTL-SDR management.

struct sdr_receiver;  // forward decl

// decoder_ops callbacks (registered via decoderOpsForRole(SDR_ROLE_FLARM))
bool flarmDecoderInit(struct sdr_receiver *rx);
void flarmDecoderProcess(struct sdr_receiver *rx, const uint8_t *iq, uint32_t len);
void flarmDecoderDrain(struct sdr_receiver *rx);
void flarmDecoderStop(struct sdr_receiver *rx);

#endif // FLARM_READER_H
