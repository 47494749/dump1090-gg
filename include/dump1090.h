// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// dump1090.h: master program header (orchestrator)
//
// Copyright (c) 2014-2016 Oliver Jowett <oliver@mutability.co.uk>
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

// This file incorporates work covered by the following copyright and
// permission notice:
//
//   Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//    *  Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//    *  Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef __DUMP1090_H
#define __DUMP1090_H

// ============================= System includes ==========================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#ifdef __cplusplus
#include <atomic>
using std::atomic_int;
using std::atomic_uint;
#else
#include <stdatomic.h>
#endif
#include <pthread.h>
#include <stdint.h>
#include <inttypes.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <time.h>
#include <limits.h>
#include <strings.h>

// ============================= Platform / DSP ===========================

#include "compat/compat.h"
#include "dsp/generated/starch.h"

// ============================= Domain headers ===========================

// Constants and type definitions (no dependencies)
#include "dump1090_defs.h"
#include "dump1090_types.h"

// Utility module headers
#include "util.h"
#include "crc.h"
#include "convert.h"
#include "ais_charset.h"
#include "cpu.h"
#include "cpdlc_decode.h"

// Network module headers
#include "anet.h"
#include "net_io.h"

// SDR module headers
#include "sdr.h"
#include "fifo.h"
#include "sdr_receiver.h"

// ADS-B module headers
#include "demod_2400.h"
#include "stats.h"
#include "cpr.h"
#include "icao_filter.h"
#include "elm.h"
#include "adaptive.h"

// Global state (needs net_io.h, stats.h, elm.h types)
#include "dump1090_state.h"

// Message struct (needs types)
#include "dump1090_message.h"

// Dispatcher types (needed by track.h for aircraft_update_t)
#include "decoder_types.h"

// Headers that need struct modesMessage
#include "track.h"
#include "mode_s.h"
#include "comm_b.h"

// Feeder thread (needs modesMessage for message queues)
#include "feeder_thread.h"

// FA MLAT built-in client (needs feeder_thread.h)
#include "fa_mlat.h"

// Feeder client headers
#include "mlat_client.h"
#include "piaware_client.h"
#include "planefinder_client.h"
#include "fr24_client.h"
#include "radarbox_client.h"
#include "opensky_client.h"
#include "ogn_client.h"
#include "sondehub_client.h"

// Decoder headers
#include "flarm_decode.h"
#include "flarm_demod.h"
#include "flarm_reader.h"

// Panel / config headers
#include "config_panel.h"
#include "decoder_config.h"
#include "station_config.h"
#include "network_config.h"
#include "display_config.h"
#include "json_config.h"
#include "app_config.h"

// ======================== function declarations =========================

#ifdef __cplusplus
extern "C" {
#endif

//
// Functions exported from mode_ac.c
//
int  detectModeA       (uint16_t *m, struct modesMessage *mm);
void decodeModeAMessage(struct modesMessage *mm, int ModeA);
void modeACInit();
int modeAToModeC (uint32_t modeA);
uint32_t modeCToModeA (int modeC);

//
// Functions exported from interactive.c
//
void  interactiveInit(void);
void  interactiveShowData(void);
void  interactiveCleanup(void);
void  interactiveNoConnection(void);

// Provided by dump1090.c / view1090.c / faup1090.c
void receiverPositionChanged(float lat, float lon, float alt);

#ifdef __cplusplus
}
#endif

#endif // __DUMP1090_H
