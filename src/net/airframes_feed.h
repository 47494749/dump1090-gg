// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// airframes_feed.h: ACARS/VDL2 UDP JSON feed to airframes.io
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#ifndef AIRFRAMES_FEED_H
#define AIRFRAMES_FEED_H

#include "acars_demod.h"
#include "vdl2_demod.h"

// Initialize airframes feed UDP sockets (call once at startup)
void airframesFeedInit(void);

// Send an ACARS message as acarsdec-compatible JSON via UDP
void airframesFeedSendAcars(const acars_msg_t *msg);

// Send a VDL2 message as dumpvdl2-compatible JSON via UDP
void airframesFeedSendVdl2(const vdl2_msg_t *msg);

// Cleanup airframes feed sockets (call at shutdown)
void airframesFeedCleanup(void);

#endif // AIRFRAMES_FEED_H
