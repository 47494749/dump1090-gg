// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// ogn_client.h: OGN (Open Glider Network) APRS-IS feed client
//
// Connects to the OGN APRS-IS server and submits decoded FLARM positions.
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#ifndef OGN_CLIENT_H
#define OGN_CLIENT_H


#include "flarm_decode.h"

// Initialize the OGN client (call once at startup, after config is parsed)
void ognClientInit(void);

// Periodic work: manage connection, send queued positions
// Called from backgroundTasks()
void ognClientPeriodicWork(void);

// Submit a decoded FLARM position to the OGN client queue
void ognClientSubmit(const flarm_message_t *msg);

// Cleanup OGN client resources
void ognClientCleanup(void);

#endif // OGN_CLIENT_H
