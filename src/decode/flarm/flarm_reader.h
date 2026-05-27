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


#include <stdint.h>
#include <string>
#include "flarm_demod.h"

// FLARM reader configuration (set before calling flarmReaderInit)
typedef struct {
    int32_t    enabled;              // FLARM reception enabled
    std::string device_serial;      // RTL-SDR device serial number for 868 MHz dongle
    int32_t    gain;                // Gain in tenths of dB (0 = auto)
    int32_t    ppm_error;           // Frequency correction in PPM

    // Keys file
    std::string keys_file;         // Path to FLARM decryption keys file
    int32_t    flarm_ogn_only;     // If set, FLARM data goes to OGN only (not other feeders)

    // OGN station info
    std::string ogn_station;       // OGN station name (e.g. "MYSTATION")
    std::string ogn_server;        // OGN APRS-IS server (default: aprs.glidernet.org)
    int32_t    ogn_port;           // OGN APRS-IS port (default: 14580)

    // Virtual IQ file input (--flarm-ifile)
    std::string ifile_path;        // Path to raw IQ file (uint8 I/Q pairs, 1.6 MSPS)
    uint32_t ifile_time;       // Unix timestamp of file (mtime), used for XXTEA decrypt
    int32_t    ifile_once;         // If set, stop after a single file replay pass

    // P3I (PilotAware) decoder
    int32_t    p3i_enabled;        // Enable P3I 869.525 MHz decoder (widens SDR bandwidth)
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
bool flarmDecoderDrain(struct sdr_receiver *rx);
void flarmDecoderStop(struct sdr_receiver *rx);
bool flarmDecoderGetStats(struct sdr_receiver *rx, flarm_demod_stats_t *stats);

#endif // FLARM_READER_H
