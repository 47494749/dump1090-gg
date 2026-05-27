// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// net_io.c: network handling.
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

#include "dump1090.h"
#include <cstdint>
#include "dispatcher.h"

/* for PRIX64 */
#include <inttypes.h>

static adsb_queue_handle_t net_adsb_queue = NULL;

#include <cassert>
#include <cstdarg>
#include <string>
#include <new>
#include "gg_format.h"

// Helper: format into std::string
static std::string sfmt(const char *fmt, ...) __attribute__((format(printf,1,2)));
static std::string sfmt(const char *fmt, ...) {
    char tmp[1024];
    va_list ap;
    va_start(ap, fmt);
    int32_t n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n < 0) return {};
    if ((size_t)n < sizeof(tmp)) return std::string(tmp, n);
    std::string s(n, '\0');
    va_start(ap, fmt);
    vsnprintf(&s[0], n + 1, fmt, ap);
    va_end(ap);
    return s;
}

//
// ============================= Networking =============================
//
// Note: here we disregard any kind of good coding practice in favor of
// extreme simplicity, that is:
//
// 1) We only rely on the kernel buffers for our I/O without any kind of
//    user space buffering.
// 2) We don't register any kind of event handler, from time to time a
//    function gets called and we accept new connections. All the rest is
//    handled via non-blocking I/O and manually polling clients to see if
//    they have something new to share with us when reading is needed.

static int32_t handleBeastCommand(struct client *c, char *p);
static int32_t decodeBinMessage(struct client *c, char *p);
static int32_t decodeHexMessage(struct client *c, char *hex);
static int32_t handleFaupCommand(struct client *c, char *hex);

static void moveNetClient(struct client *c, struct net_service *new_service);

static void send_raw_heartbeat(struct net_service *service);
static void send_beast_heartbeat(struct net_service *service);
static void send_sbs_heartbeat(struct net_service *service);
static void send_stratux_heartbeat(struct net_service *service);

static void writeBeastMessage(struct net_writer *writer, uint64_t timestamp, double signalLevel, uint8_t *msg, int32_t msgLen);

static void writeFATSVEvent(struct modesMessage *mm, struct aircraft *a);
static void writeFATSVPositionUpdate(float lat, float lon, float alt);

static void autoset_modeac();

__attribute__ ((format (printf,3,0))) static char *safe_vsnprintf(char *p, char *end, const char *format, va_list ap);
__attribute__ ((format (printf,3,4))) static char *safe_snprintf(char *p, char *end, const char *format, ...);

static std::string jsonEscapeString(const char *str);

//
//=========================================================================
//
// Networking "stack" initialization
//

// Init a service with the given read/write characteristics, return the new service.
// Doesn't arrange for the service to listen or connect
struct net_service *serviceInit(const char *descr, struct net_writer *writer, heartbeat_fn hb, read_mode_t mode, const char *sep, read_fn handler)
{
    struct net_service *service = new net_service{};
    if (!service) {
        gg::eprint("Out of memory allocating service %s\n", descr);
        exit(1);
    }

    service->next = Modes.services;
    Modes.services = service;

    service->descr = descr;
    service->listener_count = 0;
    service->connections = 0;
    service->writer = writer;
    service->read_sep = sep;
    service->read_mode = mode;
    service->read_handler = handler;

    if (service->writer) {
        if (! (service->writer->data = new (std::nothrow) char[MODES_OUT_BUF_SIZE]) ) {
            gg::eprint("Out of memory allocating output buffer for service %s\n", descr);
            exit(1);
        }

        service->writer->service = service;
        service->writer->dataUsed = 0;
        service->writer->lastWrite = mstime();
        service->writer->send_heartbeat = hb;
    }

    return service;
}

// Create a client attached to the given service using the provided socket FD
struct client *createSocketClient(struct net_service *service, int32_t fd)
{
    anetSetSendBuffer(Modes.aneterr, fd, (MODES_NET_SNDBUF_SIZE << Modes.net_sndbuf_size));
    return createGenericClient(service, fd);
}

// Create a client attached to the given service using the provided FD (might not be a socket!)
struct client *createGenericClient(struct net_service *service, int32_t fd)
{
    struct client *c;

    anetNonBlock(Modes.aneterr, fd);

    if (!(c = new (std::nothrow) client)) {
        gg::eprint("Out of memory allocating a new %s network client\n", service->descr);
        exit(1);
    }

    c->service    = NULL;
    c->next       = Modes.clients;
    c->fd         = fd;
    c->buflen     = 0;
    c->modeac_requested = 0;
    c->verbatim_requested = (service == Modes.beast_verbatim_service || service == Modes.beast_verbatim_local_service);
    c->local_requested = (service == Modes.beast_verbatim_local_service);
    Modes.clients = c;

    moveNetClient(c, service);

    return c;
}

// Initiate an outgoing connection which will use the given service.
// Return the new client or NULL if the connection failed
struct client *serviceConnect(struct net_service *service, char *addr, int32_t port)
{
    int32_t s;

    std::string port_str = std::to_string(port);
    s = anetTcpConnect(Modes.aneterr, addr, port_str.data());
    if (s == ANET_ERR)
        return NULL;

    return createSocketClient(service, s);
}

// Set up the given service to listen on an address/port.
// _exits_ on failure!
void serviceListen(struct net_service *service, char *bind_addr, char *bind_ports)
{
    int32_t *fds = NULL;
    int32_t n = 0;
    char *p, *end;

    if (service->listener_count > 0) {
        gg::eprint("Tried to set up the service %s twice!\n", service->descr);
        exit(1);
    }

    if (!bind_ports || !strcmp(bind_ports, "") || !strcmp(bind_ports, "0"))
        return;

    p = bind_ports;
    while (p && *p) {
        int32_t newfds[16];
        int32_t nfds, i;

        end = strpbrk(p, ", ");
        std::string port_token;
        if (!end) {
            port_token = p;
            p = NULL;
        } else {
            port_token.assign(p, end - p);
            p = end + 1;
        }

        nfds = anetTcpServer(Modes.aneterr, port_token.data(), bind_addr, newfds, sizeof(newfds));
        if (nfds == ANET_ERR) {
            fprintf(stderr, "Error opening the listening port %s (%s): %s\n",
                    port_token.c_str(), service->descr, Modes.aneterr);
            exit(1);
        }

        fds = (int32_t*)realloc(fds, (n+nfds) * sizeof(int32_t));
        if (!fds) {
            gg::eprint("out of memory\n");
            exit(1);
        }

        for (i = 0; i < nfds; ++i) {
            anetNonBlock(Modes.aneterr, newfds[i]);
            fds[n++] = newfds[i];
        }
    }

    service->listener_count = n;
    service->listener_fds = fds;
}

struct net_service *makeBeastInputService(void)
{
    return serviceInit("Beast TCP input", NULL, NULL, READ_MODE_BEAST, NULL, decodeBinMessage);
}

struct net_service *makeFatsvOutputService(void)
{
    return serviceInit("FATSV TCP output", &Modes.fatsv_out, NULL, READ_MODE_IGNORE, NULL, NULL);
}

struct net_service *makeFaCmdInputService(void)
{
    return serviceInit("faup Command input", NULL, NULL, READ_MODE_ASCII, "\n", handleFaupCommand);
}

void modesInitNet(void) {
    struct net_service *s;

    signal(SIGPIPE, SIG_IGN);
    Modes.clients = NULL;
    Modes.services = NULL;

    if (!net_adsb_queue)
        net_adsb_queue = dispatcher_register_adsb_queue("net_io");

    // set up listeners
    s = serviceInit("Raw TCP output", &Modes.raw_out, send_raw_heartbeat, READ_MODE_IGNORE, NULL, NULL);
    serviceListen(s, Modes.net_bind_address, Modes.net_output_raw_ports);

    // we maintain three output services for the different option setting combinations we support
    // and switch clients between them if they request a change in mode
    Modes.beast_cooked_service = serviceInit("Beast TCP output (cooked mode)", &Modes.beast_cooked_out, send_beast_heartbeat, READ_MODE_BEAST_COMMAND, NULL, handleBeastCommand);
    Modes.beast_verbatim_service = serviceInit("Beast TCP output (verbatim mode)", &Modes.beast_verbatim_out, send_beast_heartbeat, READ_MODE_BEAST_COMMAND, NULL, handleBeastCommand);
    Modes.beast_verbatim_local_service = serviceInit("Beast TCP output (verbatim+local mode)", &Modes.beast_verbatim_local_out, send_beast_heartbeat, READ_MODE_BEAST_COMMAND, NULL, handleBeastCommand);

    if (Modes.net_verbatim)
        serviceListen(Modes.beast_verbatim_service, Modes.net_bind_address, Modes.net_output_beast_ports);
    else
        serviceListen(Modes.beast_cooked_service, Modes.net_bind_address, Modes.net_output_beast_ports);

    s = serviceInit("Basestation TCP output", &Modes.sbs_out, send_sbs_heartbeat, READ_MODE_IGNORE, NULL, NULL);
    serviceListen(s, Modes.net_bind_address, Modes.net_output_sbs_ports);

    s = serviceInit("Stratux TCP output", &Modes.stratux_out, send_stratux_heartbeat, READ_MODE_IGNORE, NULL, NULL);
    serviceListen(s, Modes.net_bind_address, Modes.net_output_stratux_ports);

    s = serviceInit("Raw TCP input", NULL, NULL, READ_MODE_ASCII, "\n", decodeHexMessage);
    serviceListen(s, Modes.net_bind_address, Modes.net_input_raw_ports);

    s = makeBeastInputService();
    serviceListen(s, Modes.net_bind_address, Modes.net_input_beast_ports);

    // ADSBexchange direct feed now runs in its own feeder thread
    // (see feeder_thread.c)

    // Initialize built-in MLAT client
    mlatClientInit();

    // Initialize built-in PiAware client
    piawareClientInit();

    // RadarBox client removed in light version
}
//
//=========================================================================
//
// This function gets called from time to time when the decoding thread is
// awakened by new data arriving. This usually happens a few times every second
//
static struct client * modesAcceptClients(void) {
    int32_t fd;
    struct net_service *s;

    for (s = Modes.services; s; s = s->next) {
        int32_t i;
        for (i = 0; i < s->listener_count; ++i) {
            while ((fd = anetTcpAccept(Modes.aneterr, s->listener_fds[i])) >= 0) {
                createSocketClient(s, fd);
            }
        }
    }

    return Modes.clients;
}
//
//=========================================================================
//
// On error free the client, collect the structure, adjust maxfd if needed.
//
static void modesCloseClient(struct client *c) {
    if (!c->service) {
        gg::eprint("warning: double close of net client\n");
        return;
    }

    // Clean up, but defer removing from the list until modesNetCleanup().
    // This is because there may be stackframes still pointing at this
    // client (unpredictably: reading from client A may cause client B to
    // be freed)

    close(c->fd);
    c->service->connections--;

    // mark it as inactive and ready to be freed
    c->fd = -1;
    c->service = NULL;
    c->modeac_requested = 0;

    autoset_modeac();
}
//
//=========================================================================
//
// Send the write buffer for the specified writer to all connected clients
//
static void flushWrites(struct net_writer *writer) {
    struct client *c;

    for (c = Modes.clients; c; c = c->next) {
        if (!c->service)
            continue;
        if (c->service == writer->service) {
#ifndef _WIN32
            int nwritten = write(c->fd, writer->data, writer->dataUsed);
#else
            int32_t nwritten = send(c->fd, writer->data, writer->dataUsed, 0 );
#endif
            if (nwritten > 0) {
                __atomic_fetch_add(&writer->service->bytes_out_total,
                                   (uint64_t)nwritten, __ATOMIC_RELAXED);
            }
            if (nwritten != writer->dataUsed) {
                modesCloseClient(c);
            }
        }
    }

    writer->dataUsed = 0;
    writer->lastWrite = mstime();
}

// Prepare to write up to 'len' bytes to the given net_writer.
// Returns a pointer to write to, or NULL to skip this write.
static char *prepareWrite(struct net_writer *writer, int32_t len) {
    if (!writer ||
        !writer->service ||
        !writer->service->connections ||
        !writer->data)
        return NULL;

    if (len > MODES_OUT_BUF_SIZE)
        return NULL;

    if (writer->dataUsed + len >= MODES_OUT_BUF_SIZE) {
        // Flush now to free some space
        flushWrites(writer);
    }

    return writer->data + writer->dataUsed;
}

// Complete a write previously begun by prepareWrite.
// endptr should point one byte past the last byte written
// to the buffer returned from prepareWrite.
static void completeWrite(struct net_writer *writer, char *endptr) {
    writer->dataUsed = endptr - writer->data;

    if (writer->dataUsed >= Modes.net_output_flush_size) {
        flushWrites(writer);
    }
}

//
//=========================================================================
//
// Write raw output in Beast Binary format with Timestamp to TCP clients
//
static void modesSendBeastVerbatimOutput(struct modesMessage *mm) {
    // Don't forward mlat messages, unless --forward-mlat is set
    if (mm->source == SOURCE_MLAT && !Modes.forward_mlat)
        return;

    // Do verbatim output for all messages
    writeBeastMessage(&Modes.beast_verbatim_out, mm->timestampMsg, mm->signalLevel, mm->verbatim, mm->msgbits / 8);
}

static void modesSendBeastVerbatimLocalOutput(struct modesMessage *mm) {
    // Never forward remote messages
    if (mm->remote)
        return;

    // Do verbatim output for all messages
    writeBeastMessage(&Modes.beast_verbatim_local_out, mm->timestampMsg, mm->signalLevel, mm->verbatim, mm->msgbits / 8);
}

static void modesSendBeastCookedOutput(struct modesMessage *mm, struct aircraft *a) {
    // Don't forward mlat messages, unless --forward-mlat is set
    if (mm->source == SOURCE_MLAT && !Modes.forward_mlat)
        return;

    // Filter some messages from cooked output
    // Don't forward 2-bit-corrected messages
    if (mm->correctedbits >= 2)
        return;

    // Don't forward unreliable messages
    if ((a && !a->reliable) && !mm->reliable)
        return;

    writeBeastMessage(&Modes.beast_cooked_out, mm->timestampMsg, mm->signalLevel, mm->msg, mm->msgbits / 8);
}

static void writeBeastMessage(struct net_writer *writer, uint64_t timestamp, double signalLevel, uint8_t *msg, int32_t msgLen) {
    char ch;
    int32_t  j;
    int32_t sig;

    char *p = prepareWrite(writer, 2 + 2 * (7 + msgLen));
    if (!p)
        return;

    *p++ = 0x1a;
    if      (msgLen == MODES_SHORT_MSG_BYTES)
      {*p++ = '2';}
    else if (msgLen == MODES_LONG_MSG_BYTES)
      {*p++ = '3';}
    else if (msgLen == MODEAC_MSG_BYTES)
      {*p++ = '1';}
    else
      {return;}

    /* timestamp, big-endian */
    *p++ = (ch = (timestamp >> 40));
    if (0x1A == ch) {*p++ = ch; }
    *p++ = (ch = (timestamp >> 32));
    if (0x1A == ch) {*p++ = ch; }
    *p++ = (ch = (timestamp >> 24));
    if (0x1A == ch) {*p++ = ch; }
    *p++ = (ch = (timestamp >> 16));
    if (0x1A == ch) {*p++ = ch; }
    *p++ = (ch = (timestamp >> 8));
    if (0x1A == ch) {*p++ = ch; }
    *p++ = (ch = (timestamp));
    if (0x1A == ch) {*p++ = ch; }

    sig = round(sqrt(signalLevel) * 255);
    if (signalLevel > 0 && sig < 1)
        sig = 1;
    if (sig > 255)
        sig = 255;
    *p++ = ch = (char)sig;
    if (0x1A == ch) {*p++ = ch; }

    for (j = 0; j < msgLen; j++) {
        *p++ = (ch = msg[j]);
        if (0x1A == ch) {*p++ = ch; }
    }

    completeWrite(writer, p);
}

static void send_beast_heartbeat(struct net_service *service)
{
    static char heartbeat_message[] = { 0x1a, '1', 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    char *data;

    if (!service->writer)
        return;

    data = prepareWrite(service->writer, sizeof(heartbeat_message));
    if (!data)
        return;

    memcpy(data, heartbeat_message, sizeof(heartbeat_message));
    completeWrite(service->writer, data + sizeof(heartbeat_message));
}

//
//=========================================================================
//
// Write raw output to TCP clients
//
static void modesSendRawOutput(struct modesMessage *mm, struct aircraft *a) {
    // Don't ever forward mlat messages via raw output.
    if (mm->source == SOURCE_MLAT)
        return;

    // Filter some messages
    // Don't forward 2-bit-corrected messages
    if (mm->correctedbits >= 2)
        return;

    // Don't forward unreliable messages
    if ((a && !a->reliable) && !mm->reliable)
        return;

    int32_t msgLen = mm->msgbits / 8;
    char *p = prepareWrite(&Modes.raw_out, msgLen*2 + 15);
    if (!p)
        return;

    static const char hexchars[] = "0123456789ABCDEF";

    if (Modes.mlat && mm->timestampMsg) {
        /* timestamp, big-endian */
        *p++ = '@';
        uint64_t ts = mm->timestampMsg;
        for (int32_t i = 11; i >= 0; --i) {
            p[i] = hexchars[ts & 0xF];
            ts >>= 4;
        }
        p += 12;
    } else
        *p++ = '*';

    uint8_t *msg = mm->msg;
    for (int32_t j = 0; j < msgLen; j++) {
        *p++ = hexchars[msg[j] >> 4];
        *p++ = hexchars[msg[j] & 0x0F];
    }

    *p++ = ';';
    *p++ = '\n';

    completeWrite(&Modes.raw_out, p);
}

static void send_raw_heartbeat(struct net_service *service)
{
    static const char *heartbeat_message = "*0000;\n";
    char *data;
    int32_t len = strlen(heartbeat_message);

    if (!service->writer)
        return;

    data = prepareWrite(service->writer, len);
    if (!data)
        return;

    memcpy(data, heartbeat_message, len);
    completeWrite(service->writer, data + len);
}

//
//=========================================================================
//
// Write SBS output to TCP clients
//
static void modesSendSBSOutput(struct modesMessage *mm, struct aircraft *a) {
    struct timespec now;
    struct tm    stTime_receive, stTime_now;
    int32_t          msgType;

    // We require a tracked aircraft for SBS output
    if (!a)
        return;

    // Don't ever forward 2-bit-corrected messages via SBS output.
    if (mm->correctedbits >= 2)
        return;

    // Don't ever forward mlat messages via SBS output.
    if (mm->source == SOURCE_MLAT)
        return;

    // Don't ever send unreliable messages via SBS output
    if (!mm->reliable && !a->reliable)
        return;

    // For now, suppress non-ICAO addresses
    if (mm->addr & MODES_NON_ICAO_ADDRESS)
        return;

    //
    // SBS BS style output checked against the following reference
    // http://www.homepages.mcb.net/bones/SBS/Article/Barebones42_Socket_Data.htm - seems comprehensive
    //

    // Decide on the basic SBS Message Type
    switch (mm->msgtype) {
    case 4:
    case 20:
        msgType = 5;
        break;
        break;

    case 5:
    case 21:
        msgType = 6;
        break;

    case 0:
    case 16:
        msgType = 7;
        break;

    case 11:
        msgType = 8;
        break;

    case 17:
    case 18:
    case 19:
        if (mm->metype >= 1 && mm->metype <= 4) {
            msgType = 1;
        } else if (mm->metype >= 5 && mm->metype <=  8) {
            msgType = 2;
        } else if (mm->metype == 19) {
            msgType = 4;
        } else if ((mm->metype >= 9 && mm->metype <= 18) || (mm->metype >= 20 && mm->metype <= 22)) {
            msgType = 3;
        } else {
            return;
        }
        break;

    default:
        return;
    }

    // Build SBS line using std::string
    std::string s;
    s.reserve(200);

    // Fields 1 to 6 : SBS message type and ICAO address of the aircraft and some other stuff
    s += sfmt("MSG,%d,1,1,%06X,1,", msgType, mm->addr);

    // Find current system time
    clock_gettime(CLOCK_REALTIME, &now);
    localtime_r(&now.tv_sec, &stTime_now);

    // Find message reception time
    time_t received = (time_t) (mm->sysTimestampMsg / 1000);
    localtime_r(&received, &stTime_receive);

    // Fields 7 & 8 are the message reception time and date
    s += sfmt("%04d/%02d/%02d,", (stTime_receive.tm_year+1900),(stTime_receive.tm_mon+1), stTime_receive.tm_mday);
    s += sfmt("%02d:%02d:%02d.%03u,", stTime_receive.tm_hour, stTime_receive.tm_min, stTime_receive.tm_sec, (uint32_t) (mm->sysTimestampMsg % 1000));

    // Fields 9 & 10 are the current time and date
    s += sfmt("%04d/%02d/%02d,", (stTime_now.tm_year+1900),(stTime_now.tm_mon+1), stTime_now.tm_mday);
    s += sfmt("%02d:%02d:%02d.%03u", stTime_now.tm_hour, stTime_now.tm_min, stTime_now.tm_sec, (uint32_t) (now.tv_nsec / 1000000U));

    // Field 11 is the callsign (if we have it)
    if (mm->callsign_valid) { s += sfmt(",%s", mm->callsign); }
    else                    { s += ","; }

    // Field 12 is the altitude (if we have it)
    if (Modes.use_gnss) {
        if (mm->altitude_geom_valid) {
            s += sfmt(",%dH", mm->altitude_geom);
        } else if (mm->altitude_baro_valid && trackDataValid(&a->geom_delta_valid)) {
            s += sfmt(",%dH", mm->altitude_baro + a->geom_delta);
        } else if (mm->altitude_baro_valid) {
            s += sfmt(",%d", mm->altitude_baro);
        } else {
            s += ",";
        }
    } else {
        if (mm->altitude_baro_valid) {
            s += sfmt(",%d", mm->altitude_baro);
        } else if (mm->altitude_geom_valid && trackDataValid(&a->geom_delta_valid)) {
            s += sfmt(",%d", mm->altitude_geom - a->geom_delta);
        } else {
            s += ",";
        }
    }

    // Field 13 is the ground Speed (if we have it)
    if (mm->gs_valid) {
        s += sfmt(",%.0f", mm->gs.selected);
    } else {
        s += ",";
    }

    // Field 14 is the ground Heading (if we have it)
    if (mm->heading_valid && mm->heading_type == HEADING_GROUND_TRACK) {
        s += sfmt(",%.0f", mm->heading);
    } else {
        s += ",";
    }

    // Fields 15 and 16 are the Lat/Lon (if we have it)
    if (mm->cpr_decoded) {
        s += sfmt(",%1.5f,%1.5f", mm->decoded_lat, mm->decoded_lon);
    } else {
        s += ",,";
    }

    // Field 17 is the VerticalRate (if we have it)
    if (Modes.use_gnss) {
        if (mm->geom_rate_valid) {
            s += sfmt(",%dH", mm->geom_rate);
        } else if (mm->baro_rate_valid) {
            s += sfmt(",%d", mm->baro_rate);
        } else {
            s += ",";
        }
    } else {
        if (mm->baro_rate_valid) {
            s += sfmt(",%d", mm->baro_rate);
        } else if (mm->geom_rate_valid) {
            s += sfmt(",%d", mm->geom_rate);
        } else {
            s += ",";
        }
    }

    // Field 18 is the Squawk (if we have it)
    if (mm->squawk_valid) {
        s += sfmt(",%04x", mm->squawk);
    } else {
        s += ",";
    }

    // Field 19 is the Squawk Changing Alert flag (if we have it)
    if (mm->alert_valid) {
        s += mm->alert ? ",-1" : ",0";
    } else {
        s += ",";
    }

    // Field 20 is the Squawk Emergency flag (if we have it)
    if (mm->squawk_valid) {
        if ((mm->squawk == 0x7500) || (mm->squawk == 0x7600) || (mm->squawk == 0x7700)) {
            s += ",-1";
        } else {
            s += ",0";
        }
    } else {
        s += ",";
    }

    // Field 21 is the Squawk Ident flag (if we have it)
    if (mm->spi_valid) {
        s += mm->spi ? ",-1" : ",0";
    } else {
        s += ",";
    }

    // Field 22 is the OnTheGround flag (if we have it)
    switch (mm->airground) {
    case AG_GROUND:
        s += ",-1";
        break;
    case AG_AIRBORNE:
        s += ",0";
        break;
    default:
        s += ",";
        break;
    }

    s += "\r\n";

    // Copy into network output buffer
    char *p = prepareWrite(&Modes.sbs_out, s.size());
    if (!p)
        return;
    memcpy(p, s.data(), s.size());
    completeWrite(&Modes.sbs_out, p + s.size());
}

static void send_sbs_heartbeat(struct net_service *service)
{
    static const char *heartbeat_message = "\r\n";  // is there a better one?
    char *data;
    int32_t len = strlen(heartbeat_message);

    if (!service->writer)
        return;

    data = prepareWrite(service->writer, len);
    if (!data)
        return;

    memcpy(data, heartbeat_message, len);
    completeWrite(service->writer, data + len);
}

//
//=========================================================================
//
// Write Stratux output to TCP clients
//

#define STRATUX_MAX_PACKET_SIZE 1000
static void modesSendStratuxOutput(struct modesMessage *mm, struct aircraft *a) {
    char *p;

    // We require a tracked aircraft for Stratux output
    if (!a)
        return;

    // Don't ever forward 2-bit-corrected messages via Stratux output.
    if (mm->correctedbits >= 2)
        return;

    // Don't ever send unreliable messages via Stratux output
    if (!mm->reliable && !a->reliable)
        return;

    p = prepareWrite(&Modes.stratux_out, STRATUX_MAX_PACKET_SIZE); // larger buffer size needed vs SBS
    if (!p)
        return;

    char *end = p + STRATUX_MAX_PACKET_SIZE;

    // Begin populating the traffic.go fields.
    // ICAO address, Mode S message types, and signal level

    int32_t cacf = 0; // overload the JSON "CA" field to report CA (DF11 or DF17), CF (DF18), or zero (all other DF types)
    if ((mm->msgtype == 11) || (mm->msgtype == 17)) {
        cacf = mm->CA;
    } else if (mm->msgtype == 18) {
        cacf = mm->CF;
    }

    const char* is_mlat_str = "false";
    if (mm->source == SOURCE_MLAT)
        is_mlat_str = "true";

    p = safe_snprintf(p, end,
            "{\"Icao_addr\":%u,"
            "\"DF\":%d,\"CA\":%d,"
            "\"TypeCode\":%u,"
            "\"SubtypeCode\":%u,"
            "\"SignalLevel\":%f,"
            "\"Gain\":%f,"
            "\"IsMlat\":%s,",
            mm->addr,
            mm->msgtype, cacf,
            mm->metype,
            mm->mesub,
            mm->signalLevel, // what precision and range is needed for RSSI?
            sdrGetGainDb(sdrGetGain()),
            is_mlat_str);

    //// callsign
    if (mm->callsign_valid)
        p = safe_snprintf(p, end, "\"Tail\":\"%s\",", jsonEscapeString(mm->callsign).c_str());
    else
        p = safe_snprintf(p, end, "\"Tail\":null,");

    //// altitude & gnss
    bool alt_is_geom;
    if (mm->altitude_baro_valid) {
        p = safe_snprintf(p, end, "\"Alt\":%d,",mm->altitude_baro);
        alt_is_geom = false;
    } else if (mm->altitude_geom_valid) {
        p = safe_snprintf(p, end, "\"Alt\":%d,",mm->altitude_geom);
        alt_is_geom = true;
    } else {
        p = safe_snprintf(p, end, "\"Alt\":null,");
        alt_is_geom = false;
    }

    // altitude source
    if (alt_is_geom)
        p = safe_snprintf(p, end, "\"AltIsGNSS\":true,");
    else
        p = safe_snprintf(p, end, "\"AltIsGNSS\":false,");

    // GNSS alt. delta From baro alt.
    if (trackDataValid(&a->geom_delta_valid))
        p = safe_snprintf(p, end, "\"GnssDiffFromBaroAlt\":%d,",a->geom_delta);
    else
        p = safe_snprintf(p, end, "\"GnssDiffFromBaroAlt\":null,");
    ////

    //// ground speed and track
    if (mm->gs_valid)
        p = safe_snprintf(p, end, "\"Speed_valid\":true,\"Speed\":%.0f,", mm->gs.selected);
    else
        p = safe_snprintf(p, end, "\"Speed_valid\":false,\"Speed\":null,");

    //// ground heading
    if (mm->heading_valid && mm->heading_type == HEADING_GROUND_TRACK)
        p = safe_snprintf(p, end, "\"Track\":%.0f,", mm->heading);
    else
        p = safe_snprintf(p, end, "\"Track\":null,");

    //// position
    if (mm->cpr_decoded)
        p = safe_snprintf(p, end, "\"Lat\":%.6f,\"Lng\":%.6f,\"Position_valid\":true,",
                    mm->decoded_lat, mm->decoded_lon);
    else
        p = safe_snprintf(p, end, "\"Lat\":null,\"Lng\":null,\"Position_valid\":false,");

    //// vrate (use barometric if possible)
    if (mm->baro_rate_valid)
        p = safe_snprintf(p, end, "\"Vvel\":%d,", mm->baro_rate);
    else if (mm->geom_rate_valid)
        p = safe_snprintf(p, end, "\"Vvel\":%d,", mm->geom_rate);
    else
        p = safe_snprintf(p, end, "\"Vvel\":null,");

    //// squawk
    if (mm->squawk_valid)
        p = safe_snprintf(p, end, "\"Squawk\":%x,", mm->squawk);
    else
        p = safe_snprintf(p, end, "\"Squawk\":null,");

    // TODO: squawk changing alert support in stratux?
    // TODO: squawk emergency flag?
    // TODO: squawk ident flag?

    // airground
    switch (mm->airground) {
        case AG_GROUND:
            p = safe_snprintf(p, end, "\"OnGround\":true,");
            break;
        case AG_AIRBORNE:
            p = safe_snprintf(p, end, "\"OnGround\":false,");
            break;
        default:
            p = safe_snprintf(p, end, "\"OnGround\":null,");
    }

    // navigation accuracy category - position
    if (mm->accuracy.nac_p_valid) {
        p = safe_snprintf(p, end, "\"NACp\":%u,", mm->accuracy.nac_p);
    } else {
        p = safe_snprintf(p, end, "\"NACp\":null,");
    }

    // emitter type
    int32_t emitter = -1;
    if ((mm->msgtype ==  17) || (mm->msgtype == 18)) {
        switch (mm->metype) {
            case 1:
                emitter = ((mm->mesub) | 0x18);
                break;
            case 2:
                emitter = ((mm->mesub) | 0x10);
                break;
            case 3:
                emitter = ((mm->mesub) | 0x08);
                break;
            case 4:
                emitter = (mm->mesub);
                break;
        }
    }

    if (emitter >= 0)
        p = safe_snprintf(p, end, "\"Emitter_category\":%d,", emitter);
    else
        p = safe_snprintf(p, end, "\"Emitter_category\":null,");

    // Time message received (based on system clock). Format is 2016-02-20T06:35:43.155Z
    struct tm stTime_receive;
    time_t received = (time_t) (mm->sysTimestampMsg / 1000);
    gmtime_r(&received, &stTime_receive);
    p = safe_snprintf(p, end, "\"Timestamp\":\"%04d-%02d-%02dT%02d:%02d:%02d.%03uZ\"",
            (stTime_receive.tm_year+1900),(stTime_receive.tm_mon+1),
            stTime_receive.tm_mday, stTime_receive.tm_hour,
            stTime_receive.tm_min, stTime_receive.tm_sec,
            (uint32_t)(mm->sysTimestampMsg % 1000));

    p = safe_snprintf(p, end, "}\r\n");

    if (p < end)
        completeWrite(&Modes.stratux_out, p);
    else
        gg::eprint("stratux: output too large (max %d, overran by %d)\n", STRATUX_MAX_PACKET_SIZE, (int32_t) (p - end));
}

static void send_stratux_heartbeat(struct net_service *service)
{
    static const char *heartbeat_message = "{\"Icao_addr\":134217727}\r\n";  // 0x07FFFFFF. Overflows 24-bit ICAO to signal invalic #, need to validate that this won't cause problems with traffic.go
    char *data;
    int32_t len = strlen(heartbeat_message);

    if (!service->writer)
        return;

    data = prepareWrite(service->writer, len);
    if (!data)
        return;

    memcpy(data, heartbeat_message, len);
    completeWrite(service->writer, data + len);
}

//
//=========================================================================
//
void modesQueueOutput(struct modesMessage *mm, struct aircraft *a) {

    // Delegate to the format-specific outputs, each of which makes its own decision about filtering messages
    modesSendSBSOutput(mm, a);
    modesSendStratuxOutput(mm, a);
    modesSendRawOutput(mm, a);
    modesSendBeastVerbatimOutput(mm);
    modesSendBeastVerbatimLocalOutput(mm);
    modesSendBeastCookedOutput(mm, a);
    writeFATSVEvent(mm, a);
}

// Decode a little-endian IEEE754 float (binary32)
static float ieee754_binary32_le_to_float(uint8_t *data)
{
    double sign = (data[3] & 0x80) ? -1.0 : 1.0;
    int16_t raw_exponent = ((data[3] & 0x7f) << 1) | ((data[2] & 0x80) >> 7);
    uint32_t raw_significand = ((data[2] & 0x7f) << 16) | (data[1]  << 8) | data[0];

    if (raw_exponent == 0) {
        if (raw_significand == 0) {
            /* -0 is treated like +0 */
            return 0;
        } else {
            /* denormal */
            return ldexp(sign * raw_significand, -126 - 23);
        }
    }

    if (raw_exponent == 255) {
        if (raw_significand == 0) {
            /* +/-infinity */
            return sign < 0 ? -INFINITY : INFINITY;
        } else {
            /* NaN */
#ifdef NAN
            return NAN;
#else
            return 0.0f;
#endif
        }
    }

    /* normalized value */
    return ldexp(sign * ((1 << 23) | raw_significand), raw_exponent - 127 - 23);
}

static void handle_radarcape_position(float lat, float lon, float alt)
{
    if (!isfinite(lat) || lat < -90 || lat > 90 || !isfinite(lon) || lon < -180 || lon > 180 || !isfinite(alt))
        return;

    writeFATSVPositionUpdate(lat, lon, alt);

    if (!(Modes.bUserFlags & MODES_USER_LATLON_VALID)) {
        Modes.fUserLat = lat;
        Modes.fUserLon = lon;
        Modes.bUserFlags |= MODES_USER_LATLON_VALID;
        receiverPositionChanged(lat, lon, alt);
    }
}

// recompute global Mode A/C setting
static void autoset_modeac() {
    struct client *c;

    if (!Modes.mode_ac_auto)
        return;

    Modes.mode_ac = 0;
    for (c = Modes.clients; c; c = c->next) {
        if (c->modeac_requested) {
            Modes.mode_ac = 1;
            break;
        }
    }
}

// Send some Beast settings commands to a client
void sendBeastSettings(struct client *c, const char *settings)
{
    int32_t len;

    len = strlen(settings) * 3;
    std::string buf(len, '\0');
    char *p = buf.data();

    while (*settings) {
        *p++ = 0x1a;
        *p++ = '1';
        *p++ = *settings++;
    }

    anetWrite(c->fd, buf.data(), len);
}

// Move a network client to a new service
static void moveNetClient(struct client *c, struct net_service *new_service)
{
    if (c->service == new_service)
        return;

    if (c->service) {
        // Flush to ensure correct message framing
        if (c->service->writer)
            flushWrites(c->service->writer);
        --c->service->connections;
    }

    if (new_service) {
        // Flush to ensure correct message framing
        if (new_service->writer)
            flushWrites(new_service->writer);
        ++new_service->connections;
    }

    c->service = new_service;
}

static int32_t handleFaupCommand(struct client *c, char *p) {
    if (p == NULL)
        return 0;

    MODES_NOTUSED(c);
    char* msg_field;
    double multiplier;
    msg_field = strtok (p, "\t");

    // Traverse through message for commands
    while (msg_field != NULL) {
        if (!strcmp(msg_field, "upload_rate_multiplier")) {
            msg_field = strtok (NULL, "\t");
            multiplier = atof(msg_field);

            // Sanity check on multiplier value
            if (!(multiplier > 0 && multiplier <= 100)) {
                gg::eprint("handleFaupCommand(): upload_rate_multiplier (%0.2f) out of range\n", multiplier);
                return 0;
            }

            gg::eprint("handleFaupCommand(): Adjusting message rate to FlightAware by %0.2fx\n", multiplier);
            Modes.faup_rate_multiplier = multiplier;
            break;
        }

        if (!strcmp(msg_field, "upload_unknown_commb")) {
            msg_field = strtok (NULL, "\t");
            uint32_t enable = atoi(msg_field);
            gg::eprint("handleFaupCommand(): %s upload of unknown Comm-B messages\n", enable ? "Enabling" : "Disabling");
            Modes.faup_upload_unknown_commb = enable;
            break;
        }
        msg_field = strtok (NULL, "\t");
    }

    return 0;
}

// Move a client to the right output service based on
// the currently requested options
static void handleOptionsChange(struct client *c) {
    if (c->local_requested)
        moveNetClient(c, Modes.beast_verbatim_local_service);
    else if (c->verbatim_requested)
        moveNetClient(c, Modes.beast_verbatim_service);
    else
        moveNetClient(c, Modes.beast_cooked_service);
}

//
// Handle a Beast command message. Currently we support only j/J, l/L, v/V
// and ignore other options
//
static int32_t handleBeastCommand(struct client *c, char *p) {
    if (p[0] != '1') {
        // huh?
        return 0;
    }

    switch (p[1]) {
    case 'j':
        c->modeac_requested = 0;
        autoset_modeac();
        break;
    case 'J':
        c->modeac_requested = 1;
        autoset_modeac();
        break;
    case 'v':
        c->verbatim_requested = 0;
        handleOptionsChange(c);
        break;
    case 'V':
        c->verbatim_requested = 1;
        handleOptionsChange(c);
        break;
    case 'l':
        c->local_requested = 0;
        handleOptionsChange(c);
        break;
    case 'L':
        c->local_requested = 1;
        handleOptionsChange(c);
        break;
    }

    return 0;
}

//
//=========================================================================
//
// This function decodes a Beast binary format message
//
// The message is passed to the higher level layers, so it feeds
// the selected screen output, the network output and so forth.
//
// If the message looks invalid it is silently discarded.
//
// The function always returns 0 (success) to the caller as there is no
// case where we want broken messages here to close the client connection.
//
static int32_t decodeBinMessage(struct client *c, char *p) {
    int32_t msgLen = 0;
    int32_t  j;
    char ch;
    uint8_t msg[MODES_LONG_MSG_BYTES + 7];
    static struct modesMessage zeroMessage;
    struct modesMessage mm = {};
    MODES_NOTUSED(c);

    ch = *p++; /// Get the message type

    if (ch == '1' && Modes.mode_ac) {
        msgLen = MODEAC_MSG_BYTES;
    } else if (ch == '2') {
        msgLen = MODES_SHORT_MSG_BYTES;
    } else if (ch == '3') {
        msgLen = MODES_LONG_MSG_BYTES;
    } else if (ch == '5') {
        // Special case for Radarcape position messages.
        float lat, lon, alt;

        for (j = 0; j < 21; j++) { // and the data
            msg[j] = ch = *p++;
            if (0x1A == ch) {p++;}
        }

        lat = ieee754_binary32_le_to_float(msg + 4);
        lon = ieee754_binary32_le_to_float(msg + 8);
        alt = ieee754_binary32_le_to_float(msg + 12);

        handle_radarcape_position(lat, lon, alt);
    } else {
        // Ignore this.
        return 0;
    }

    if (msgLen) {
        mm = zeroMessage;

        // Mark messages received over the internet as remote so that we don't try to
        // pass them off as being received by this instance when forwarding them
        mm.remote      =    1;

        // Grab the timestamp (big endian format)
        mm.timestampMsg = 0;
        for (j = 0; j < 6; j++) {
            ch = *p++;
            mm.timestampMsg = mm.timestampMsg << 8 | (ch & 255);
            if (0x1A == ch) {p++;}
        }

        // record reception time as the time we read it.
        mm.sysTimestampMsg = mstime();

        ch = *p++;  // Grab the signal level
        mm.signalLevel = ((uint8_t)ch / 255.0);
        mm.signalLevel = mm.signalLevel * mm.signalLevel;
        if (0x1A == ch) {p++;}

        for (j = 0; j < msgLen; j++) { // and the data
            msg[j] = ch = *p++;
            if (0x1A == ch) {p++;}
        }

        if (msgLen == MODEAC_MSG_BYTES) { // ModeA or ModeC
            Modes.stats_current.remote_received_modeac++;
            decodeModeAMessage(&mm, ((msg[0] << 8) | msg[1]));
        } else {
            int32_t result;

            Modes.stats_current.remote_received_modes++;
            result = decodeModesMessage(&mm, msg);
            if (result < 0) {
                if (result == -1)
                    Modes.stats_current.remote_rejected_unknown_icao++;
                else
                    Modes.stats_current.remote_rejected_bad++;
                return 0;
            } else {
                Modes.stats_current.remote_accepted[mm.correctedbits]++;
            }
        }

        dispatcher_push_adsb(net_adsb_queue, &mm);
    }
    return (0);
}
//
//=========================================================================
//
// Turn an hex digit into its 4 bit decimal value.
// Returns -1 if the digit is not in the 0-F range.
//
static int32_t hexDigitVal(int32_t c) {
    c = tolower(c);
    if (c >= '0' && c <= '9') return c-'0';
    else if (c >= 'a' && c <= 'f') return c-'a'+10;
    else return -1;
}


// decode 12 hex digits as a 48-bit timestamp
static bool timestampFromHex(const char *hex, uint64_t *timestamp)
{
    uint64_t ts = 0;
    for (uint32_t i = 0; i < 12; ++i) {
        int32_t v = hexDigitVal(hex[i]);
        if (v < 0)
            return false;
        ts = (ts << 4) | v;
    }

    *timestamp = ts;
    return true;
}

// decode 2 hex digits as a signal level
static bool signalFromHex(const char *hex, double *signal)
{
    int32_t d1 = hexDigitVal(hex[0]);
    int32_t d2 = hexDigitVal(hex[1]);
    if (d1 < 0 || d2 < 0)
        return false;

    double sig = ((d1 << 4) | d2) / 255.0;
    *signal = sig * sig;
    return true;
}

//
//=========================================================================
//
// This function decodes a string representing message in raw hex format
// like: *8D4B969699155600E87406F5B69F; The string is null-terminated.
//
// The message is passed to the higher level layers, so it feeds
// the selected screen output, the network output and so forth.
//
// If the message looks invalid it is silently discarded.
//
// The function always returns 0 (success) to the caller as there is no
// case where we want broken messages here to close the client connection.
//
static int32_t decodeHexMessage(struct client *c, char *hex) {
    int32_t l = strlen(hex), j;
    uint8_t msg[MODES_LONG_MSG_BYTES];
    struct modesMessage mm;
    static struct modesMessage zeroMessage;

    MODES_NOTUSED(c);
    mm = zeroMessage;

    // Mark messages received over the internet as remote so that we don't try to
    // pass them off as being received by this instance when forwarding them
    mm.remote      =    1;
    mm.signalLevel =    0;

    // Remove spaces on the left and on the right
    while(l && isspace(hex[l-1])) {
        hex[l-1] = '\0'; l--;
    }
    while(isspace(*hex)) {
        hex++; l--;
    }

    // Turn the message into binary.
    // Accept *-AVR raw @-AVR/BEAST timeS+raw %-AVR timeS+raw (CRC good) <-BEAST timeS+sigL+raw
    // and some AVR records that we can understand
    if (hex[l-1] != ';') {return (0);} // not complete - abort

    switch (hex[0]) {
        case '<':
            // [0]       '<'
            // [1..12]   timestamp
            // [13..14]  signal level
            // [15..l-2] data
            // [l-1]     ';'
            if (l < 18)
                return 0; // truncated
            if (!timestampFromHex(hex + 1, &mm.timestampMsg))
                return 0; // malformed timestamp
            if (!signalFromHex(hex + 13, &mm.signalLevel))
                return 0; // malformed signal level
            hex += 15;
            l -= 16;
            break;

        case '@':     // No CRC check
        case '%':     // CRC is OK
            // [0]       '@' or '%'
            // [1..12]   timestamp
            // [13..l-2] data
            // [l-1]     ';'
            if (l < 16)
                return 0; // truncated
            if (!timestampFromHex(hex + 1, &mm.timestampMsg))
                return 0; // malformed timestamp
            hex += 13;
            l -= 14;
            break;

        case '*':
        case ':':
            // [0]       '*' or ':'
            // [1..l-2]  data
            // [l-1]     ';'
            if (l < 4)
                return 0; // truncated
            hex++;
            l -= 2;
            break;

        default:
            return 0;
    }

    if ( (l != (MODEAC_MSG_BYTES      * 2))
      && (l != (MODES_SHORT_MSG_BYTES * 2))
      && (l != (MODES_LONG_MSG_BYTES  * 2)) )
        {return (0);} // Too int16_t or int64_t message... broken

    if ( (0 == Modes.mode_ac)
      && (l == (MODEAC_MSG_BYTES * 2)) )
        {return (0);} // Right length for ModeA/C, but not enabled

    for (j = 0; j < l; j += 2) {
        int32_t high = hexDigitVal(hex[j]);
        int32_t low  = hexDigitVal(hex[j+1]);

        if (high == -1 || low == -1) return 0;
        msg[j/2] = (high << 4) | low;
    }

    // record reception time as the time we read it.
    mm.sysTimestampMsg = mstime();

    if (l == (MODEAC_MSG_BYTES * 2)) {  // ModeA or ModeC
        Modes.stats_current.remote_received_modeac++;
        decodeModeAMessage(&mm, ((msg[0] << 8) | msg[1]));
    } else {       // Assume ModeS
        int32_t result;

        Modes.stats_current.remote_received_modes++;
        result = decodeModesMessage(&mm, msg);
        if (result < 0) {
            if (result == -1)
                Modes.stats_current.remote_rejected_unknown_icao++;
            else
                Modes.stats_current.remote_rejected_bad++;
            return 0;
        } else {
            Modes.stats_current.remote_accepted[mm.correctedbits]++;
        }
    }

    dispatcher_push_adsb(net_adsb_queue, &mm);
    return (0);
}

__attribute__ ((format (printf,3,0))) static char *safe_vsnprintf(char *p, char *end, const char *format, va_list ap)
{
    p += vsnprintf(p < end ? p : NULL, p < end ? (size_t)(end - p) : 0, format, ap);
    return p;
}

 __attribute__ ((format (printf,3,4))) static char *safe_snprintf(char *p, char *end, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    p += vsnprintf(p < end ? p : NULL, p < end ? (size_t)(end - p) : 0, format, ap);
    va_end(ap);
    return p;
}

//
//=========================================================================
//
// Return a description of planes in json. No metric conversion
//

// usual caveats about function-returning-pointer-to-static-buffer apply
static std::string jsonEscapeString(const char *str) {
    std::string buf;
    buf.reserve(256);
    const char *in = str;

    for (; *in; ++in) {
        uint8_t ch = *in;
        if (ch == '"' || ch == '\\') {
            buf += '\\';
            buf += (char)ch;
        } else if (ch < 32 || ch > 127) {
            char tmp[8];
            snprintf(tmp, sizeof(tmp), "\\u%04x", ch);
            buf += tmp;
        } else {
            buf += (char)ch;
        }
    }

    return buf;
}

static std::string append_flags(struct aircraft *a, datasource_t source)
{
    std::string s = "[";

    size_t start = s.size();
    if (a->callsign_valid.source == source)
        s += "\"callsign\",";
    if (a->altitude_baro_valid.source == source)
        s += "\"altitude\",";
    if (a->altitude_geom_valid.source == source)
        s += "\"alt_geom\",";
    if (a->gs_valid.source == source)
        s += "\"gs\",";
    if (a->ias_valid.source == source)
        s += "\"ias\",";
    if (a->tas_valid.source == source)
        s += "\"tas\",";
    if (a->mach_valid.source == source)
        s += "\"mach\",";
    if (a->track_valid.source == source)
        s += "\"track\",";
    if (a->track_rate_valid.source == source)
        s += "\"track_rate\",";
    if (a->roll_valid.source == source)
        s += "\"roll\",";
    if (a->mag_heading_valid.source == source)
        s += "\"mag_heading\",";
    if (a->true_heading_valid.source == source)
        s += "\"true_heading\",";
    if (a->baro_rate_valid.source == source)
        s += "\"baro_rate\",";
    if (a->geom_rate_valid.source == source)
        s += "\"geom_rate\",";
    if (a->squawk_valid.source == source)
        s += "\"squawk\",";
    if (a->emergency_valid.source == source)
        s += "\"emergency\",";
    if (a->nav_qnh_valid.source == source)
        s += "\"nav_qnh\",";
    if (a->nav_altitude_mcp_valid.source == source)
        s += "\"nav_altitude_mcp\",";
    if (a->nav_altitude_fms_valid.source == source)
        s += "\"nav_altitude_fms\",";
    if (a->nav_heading_valid.source == source)
        s += "\"nav_heading\",";
    if (a->nav_modes_valid.source == source)
        s += "\"nav_modes\",";
    if (a->position_valid.source == source)
        s += "\"lat\",\"lon\",\"nic\",\"rc\",";
    if (a->nic_baro_valid.source == source)
        s += "\"nic_baro\",";
    if (a->nac_p_valid.source == source)
        s += "\"nac_p\",";
    if (a->nac_v_valid.source == source)
        s += "\"nac_v\",";
    if (a->sil_valid.source == source)
        s += "\"sil\",\"sil_type\",";
    if (a->gva_valid.source == source)
        s += "\"gva\",";
    if (a->sda_valid.source == source)
        s += "\"sda\",";
    if (s.size() != start)
        s.pop_back();  // remove trailing comma
    s += ']';
    return s;
}

static struct {
    nav_modes_t flag;
    const char *name;
} nav_modes_names[] = {
    { NAV_MODE_AUTOPILOT, "autopilot" },
    { NAV_MODE_VNAV,      "vnav" },
    { NAV_MODE_ALT_HOLD,  "althold" },
    { NAV_MODE_APPROACH,  "approach" },
    { NAV_MODE_LNAV,      "lnav" },
    { NAV_MODE_TCAS,      "tcas" },
    { (nav_modes_t)0, NULL }
};

static std::string append_nav_modes(nav_modes_t flags, const char *quote, const char *sep)
{
    std::string s;
    bool first = true;
    for (int32_t i = 0; nav_modes_names[i].name; ++i) {
        if (!(flags & nav_modes_names[i].flag))
            continue;
        if (!first)
            s += sep;
        first = false;
        s += quote;
        s += nav_modes_names[i].name;
        s += quote;
    }
    return s;
}

static const char *nav_modes_flags_string(nav_modes_t flags) {
    static std::string buf;
    buf = append_nav_modes(flags, "", " ");
    return buf.c_str();
}

static const char *addrtype_enum_string(addrtype_t type) {
    switch (type) {
    case ADDR_ADSB_ICAO:
        return "adsb_icao";
    case ADDR_ADSB_ICAO_NT:
        return "adsb_icao_nt";
    case ADDR_ADSR_ICAO:
        return "adsr_icao";
    case ADDR_TISB_ICAO:
        return "tisb_icao";
    case ADDR_ADSB_OTHER:
        return "adsb_other";
    case ADDR_ADSR_OTHER:
        return "adsr_other";
    case ADDR_TISB_OTHER:
        return "tisb_other";
    case ADDR_TISB_TRACKFILE:
        return "tisb_trackfile";
    default:
        return "unknown";
    }
}

static const char *emergency_enum_string(emergency_t emergency)
{
    switch (emergency) {
    case EMERGENCY_NONE:      return "none";
    case EMERGENCY_GENERAL:   return "general";
    case EMERGENCY_LIFEGUARD: return "lifeguard";
    case EMERGENCY_MINFUEL:   return "minfuel";
    case EMERGENCY_NORDO:     return "nordo";
    case EMERGENCY_UNLAWFUL:  return "unlawful";
    case EMERGENCY_DOWNED:    return "downed";
    default:                  return "reserved";
    }
}

static const char *sil_type_enum_string(sil_type_t type)
{
    switch (type) {
    case SIL_UNKNOWN: return "unknown";
    case SIL_PER_HOUR: return "perhour";
    case SIL_PER_SAMPLE: return "persample";
    default: return "invalid";
    }
}

static const char *nav_altitude_source_enum_string(nav_altitude_source_t src)
{
    switch (src) {
    case NAV_ALT_INVALID:  return "invalid";
    case NAV_ALT_UNKNOWN:  return "unknown";
    case NAV_ALT_AIRCRAFT: return "aircraft";
    case NAV_ALT_MCP:      return "mcp";
    case NAV_ALT_FMS:      return "fms";
    default:               return "invalid";
    }
}

static const char *mrar_source_enum_string(mrar_source_t src)
{
    switch (src) {
    case MRAR_SOURCE_INVALID:  return "invalid";
    case MRAR_SOURCE_INS:      return "ins";
    case MRAR_SOURCE_GNSS:     return "gnss";
    case MRAR_SOURCE_DMEDME:   return "dmedme";
    case MRAR_SOURCE_VORDME:   return "vordme";
    default:                   return "reserved";
    }
}

static const char *hazard_enum_string(hazard_t hazard)
{
    switch (hazard) {
    case HAZARD_NIL:       return "nil";
    case HAZARD_LIGHT:     return "light";
    case HAZARD_MODERATE:  return "moderate";
    case HAZARD_SEVERE:    return "severe";
    default:               return "invalid";
    }
}

char *generateAircraftJson(const char *url_path, int32_t *len) {
    uint64_t now = mstime();
    struct aircraft *a;
    int32_t first = 1;

    MODES_NOTUSED(url_path);

    _messageNow = now;

    std::string s = sfmt(
                       "{ \"now\" : %.1f,\n"
                       "  \"messages\" : %u,\n"
                       "  \"aircraft\" : [",
                       now / 1000.0,
                       Modes.stats_current.messages_total + Modes.stats_alltime.messages_total);

    for (a = Modes.aircrafts; a; a = a->next) {
        if (!a->reliable) {
            continue;
        }

        if (first)
            first = 0;
        else
            s += ',';

        s += sfmt("\n    {\"hex\":\"%s%06x\"", (a->addr & MODES_NON_ICAO_ADDRESS) ? "~" : "", a->addr & 0xFFFFFF);
        if (a->addrtype != ADDR_ADSB_ICAO)
            s += sfmt(",\"type\":\"%s\"", addrtype_enum_string(a->addrtype));
        if (trackDataValid(&a->callsign_valid))
            s += sfmt(",\"flight\":\"%s\"", jsonEscapeString(a->callsign).c_str());
        if (trackDataValid(&a->airground_valid) && a->airground_valid.source >= SOURCE_MODE_S_CHECKED && a->airground == AG_GROUND)
            s += ",\"alt_baro\":\"ground\"";
        else {
            if (trackDataValid(&a->altitude_baro_valid))
                s += sfmt(",\"alt_baro\":%d", a->altitude_baro);
            if (trackDataValid(&a->altitude_geom_valid))
                s += sfmt(",\"alt_geom\":%d", a->altitude_geom);
        }
        if (trackDataValid(&a->gs_valid))
            s += sfmt(",\"gs\":%.1f", a->gs);
        if (trackDataValid(&a->ias_valid))
            s += sfmt(",\"ias\":%u", a->ias);
        if (trackDataValid(&a->tas_valid))
            s += sfmt(",\"tas\":%u", a->tas);
        if (trackDataValid(&a->mach_valid))
            s += sfmt(",\"mach\":%.3f", a->mach);
        if (trackDataValid(&a->track_valid))
            s += sfmt(",\"track\":%.1f", a->track);
        if (trackDataValid(&a->track_rate_valid))
            s += sfmt(",\"track_rate\":%.2f", a->track_rate);
        if (trackDataValid(&a->roll_valid))
            s += sfmt(",\"roll\":%.1f", a->roll);
        if (trackDataValid(&a->mag_heading_valid))
            s += sfmt(",\"mag_heading\":%.1f", a->mag_heading);
        if (trackDataValid(&a->true_heading_valid))
            s += sfmt(",\"true_heading\":%.1f", a->true_heading);
        if (trackDataValid(&a->baro_rate_valid))
            s += sfmt(",\"baro_rate\":%d", a->baro_rate);
        if (trackDataValid(&a->geom_rate_valid))
            s += sfmt(",\"geom_rate\":%d", a->geom_rate);
        if (trackDataValid(&a->squawk_valid))
            s += sfmt(",\"squawk\":\"%04x\"", a->squawk);
        if (trackDataValid(&a->emergency_valid))
            s += sfmt(",\"emergency\":\"%s\"", emergency_enum_string(a->emergency));
        if (a->category != 0)
            s += sfmt(",\"category\":\"%02X\"", a->category);
        if (a->flarm_acft_type != 0) {
            static const char *flarm_type_names[] = {
                "unknown", "glider", "towplane", "helicopter", "parachute",
                "dropplane", "hangglider", "paraglider", "powered", "jet",
                "ufo", "balloon", "airship", "uav", "ground", "static"
            };
            static const char *flarm_addr_type_names[] = {
                "random", "icao", "flarm", "anonymous"
            };
            uint32_t ft = (uint32_t)a->flarm_acft_type;
            s += sfmt(",\"flarm_type\":%u", ft);
            if (ft <= 15)
                s += sfmt(",\"flarm_type_name\":\"%s\"", flarm_type_names[ft]);
            if (a->flarm_addr_type <= 3)
                s += sfmt(",\"flarm_addr_type\":\"%s\"", flarm_addr_type_names[a->flarm_addr_type]);
            if (a->flarm_proto_version > 0)
                s += sfmt(",\"flarm_proto\":%u", (uint32_t)a->flarm_proto_version);
        }
        // FANET HW info (type 8)
        if (a->fanet_hwinfo.valid) {
            s += sfmt(",\"fanet_hw\":{\"dev\":%u,\"uptime\":%u,\"rssi\":%d}",
                          (uint32_t)a->fanet_hwinfo.device_type,
                          (uint32_t)a->fanet_hwinfo.uptime_minutes,
                          (int32_t)a->fanet_hwinfo.rssi);
        }
        // FANET thermal info (type 9)
        if (a->fanet_thermal.valid) {
            s += sfmt(",\"fanet_thermal\":{\"lat\":%.5f,\"lon\":%.5f,\"alt\":%d,\"climb\":%.1f,\"wind_spd\":%.0f,\"wind_hdg\":%.0f,\"conf\":%u}",
                          a->fanet_thermal.latitude, a->fanet_thermal.longitude,
                          a->fanet_thermal.altitude, a->fanet_thermal.climb,
                          a->fanet_thermal.wind_speed, a->fanet_thermal.wind_heading,
                          (uint32_t)a->fanet_thermal.confidence);
        }
        // Radiosonde info
        if (a->sonde_info.valid) {
            s += sfmt(",\"sonde_type\":\"%s\",\"sonde_serial\":\"%s\",\"sonde_frame\":%d",
                          a->sonde_info.sonde_type, a->sonde_info.serial,
                          a->sonde_info.frame_num);
            if (a->sonde_info.rs_errors >= 0)
                s += sfmt(",\"sonde_rs\":%d", a->sonde_info.rs_errors);
            s += sfmt(",\"sonde_freq\":%.3f", (double)a->sonde_info.freq_mhz);
            s += sfmt(",\"sonde_climb\":%.1f", a->sonde_info.vel_v);
            if (a->sonde_info.satellites > 0)
                s += sfmt(",\"sonde_sat\":%d", a->sonde_info.satellites);
        }
        if (trackDataValid(&a->nav_qnh_valid))
            s += sfmt(",\"nav_qnh\":%.1f", a->nav_qnh);
         if (trackDataValid(&a->nav_altitude_mcp_valid))
            s += sfmt(",\"nav_altitude_mcp\":%d", a->nav_altitude_mcp);
         if (trackDataValid(&a->nav_altitude_fms_valid))
            s += sfmt(",\"nav_altitude_fms\":%d", a->nav_altitude_fms);
        if (trackDataValid(&a->nav_heading_valid))
            s += sfmt(",\"nav_heading\":%.1f", a->nav_heading);
        if (trackDataValid(&a->nav_modes_valid)) {
            s += ",\"nav_modes\":[";
            s += append_nav_modes(a->nav_modes, "\"", ",");
            s += ']';
        }
        if (trackDataValid(&a->position_valid)) {
            s += sfmt(",\"lat\":%f,\"lon\":%f,\"nic\":%u,\"rc\":%u,\"seen_pos\":%.1f", a->lat, a->lon, a->pos_nic, a->pos_rc, (now - a->position_valid.updated)/1000.0);
            // Position data source
            const char *ds = "unknown";
            switch (a->position_valid.source) {
                case SOURCE_ADSB:           ds = "adsb"; break;
                case SOURCE_ADSR:           ds = "adsr"; break;
                case SOURCE_TISB:           ds = "tisb"; break;
                case SOURCE_MLAT:           ds = "mlat"; break;
                case SOURCE_MODE_S_CHECKED: ds = "mode_s"; break;
                case SOURCE_MODE_S:         ds = "mode_s"; break;
                case SOURCE_MODE_AC:        ds = "mode_ac"; break;
                default:                    break;
            }
            s += sfmt(",\"datasource\":\"%s\"", ds);
        }
        if (a->adsb_version >= 0)
            s += sfmt(",\"version\":%d", a->adsb_version);
        if (trackDataValid(&a->nic_baro_valid))
            s += sfmt(",\"nic_baro\":%u", (uint32_t) a->nic_baro);
        if (trackDataValid(&a->nac_p_valid))
            s += sfmt(",\"nac_p\":%u", a->nac_p);
        if (trackDataValid(&a->nac_v_valid))
            s += sfmt(",\"nac_v\":%u", a->nac_v);
        if (trackDataValid(&a->sil_valid))
            s += sfmt(",\"sil\":%u", a->sil);
        if (a->sil_type != SIL_INVALID)
            s += sfmt(",\"sil_type\":\"%s\"", sil_type_enum_string(a->sil_type));
        if (trackDataValid(&a->gva_valid))
            s += sfmt(",\"gva\":%u", a->gva);
        if (trackDataValid(&a->sda_valid))
            s += sfmt(",\"sda\":%u", a->sda);
        if (trackDataValid(&a->mrar_source_valid))
            s += sfmt(",\"mrar_source\":\"%s\"", mrar_source_enum_string(a->mrar_source));
        if (trackDataValid(&a->wind_valid))
            s += sfmt(",\"wind_speed\":%.0f,\"wind_dir\":%.1f", a->wind_speed, a->wind_dir);
        if (trackDataValid(&a->temperature_valid))
            s += sfmt(",\"temperature\":%.2f", a->temperature);
        if (trackDataValid(&a->pressure_valid))
            s += sfmt(",\"pressure\":%.0f", a->pressure);
        if (trackDataValid(&a->turbulence_valid))
            s += sfmt(",\"turbulence\":\"%s\"", hazard_enum_string(a->turbulence));
        if (trackDataValid(&a->humidity_valid))
            s += sfmt(",\"humidity\":%.1f", a->humidity);

        // Derived wind (from TAS + heading + GS + track)
        if (a->derived_wind_updated > 0 && (now - a->derived_wind_updated) < 30000)
            s += sfmt(",\"wd_speed\":%.0f,\"wd_dir\":%.1f,\"wd_alt\":%d",
                          a->derived_wind_speed, a->derived_wind_dir, a->derived_wind_altitude);

        // Derived temperature (from TAS + Mach)
        if (a->oat_updated > 0 && (now - a->oat_updated) < 30000)
            s += sfmt(",\"oat\":%.1f,\"tat\":%.1f", a->oat, a->tat);

        // Calculated track from position pairs
        if (a->calc_track_updated > 0 && (now - a->calc_track_updated) < 30000)
            s += sfmt(",\"calc_track\":%.1f", a->calc_track);

        // Altitude reliability
        if (a->alt_reliable > 0)
            s += sfmt(",\"alt_reliable\":%d", a->alt_reliable);

        // Magnetic declination
        if (a->mag_declination_updated > 0)
            s += sfmt(",\"mag_dec\":%.1f", a->mag_declination);

        // MHAR (BDS 4,5) hazard data
        if (trackDataValid(&a->mhar_turbulence_valid))
            s += sfmt(",\"mhar_turbulence\":\"%s\"", hazard_enum_string(a->mhar_turbulence));
        if (trackDataValid(&a->mhar_windshear_valid))
            s += sfmt(",\"mhar_windshear\":\"%s\"", hazard_enum_string(a->mhar_windshear));
        if (trackDataValid(&a->mhar_microburst_valid))
            s += sfmt(",\"mhar_microburst\":\"%s\"", hazard_enum_string(a->mhar_microburst));
        if (trackDataValid(&a->mhar_icing_valid))
            s += sfmt(",\"mhar_icing\":\"%s\"", hazard_enum_string(a->mhar_icing));
        if (trackDataValid(&a->mhar_wake_valid))
            s += sfmt(",\"mhar_wake\":\"%s\"", hazard_enum_string(a->mhar_wake));
        if (trackDataValid(&a->mhar_sat_valid))
            s += sfmt(",\"mhar_temperature\":%.2f", a->mhar_sat);
        if (trackDataValid(&a->mhar_asp_valid))
            s += sfmt(",\"mhar_pressure\":%.0f", a->mhar_asp);
        if (trackDataValid(&a->mhar_rh_valid))
            s += sfmt(",\"mhar_radio_height\":%d", (int)a->mhar_rh);

        // Waypoint data (BDS 4,1/4,2/4,3)
        if (trackDataValid(&a->waypoint_valid))
            s += sfmt(",\"waypoint\":\"%s\"", jsonEscapeString(a->waypoint_id).c_str());
        if (trackDataValid(&a->waypoint_pos_valid)) {
            s += sfmt(",\"waypoint_lat\":%f,\"waypoint_lon\":%f", a->waypoint_lat, a->waypoint_lon);
            if (a->waypoint_alt)
                s += sfmt(",\"waypoint_alt\":%d", a->waypoint_alt);
        }
        if (trackDataValid(&a->waypoint_info_valid)) {
            if (a->waypoint_crossing_alt)
                s += sfmt(",\"waypoint_crossing_alt\":%d", a->waypoint_crossing_alt);
            if (a->waypoint_crossing_speed)
                s += sfmt(",\"waypoint_crossing_speed\":%u", a->waypoint_crossing_speed);
        }

        // ACAS RA data
        if (trackDataValid(&a->acas_ra_valid)) {
            s += sfmt(",\"acas_ra\":{\"ara\":%u,\"rac\":%u,\"rat\":%u,\"mte\":%u,\"tti\":%u,\"threat\":\"%06X\"}",
                          a->acas_ara, a->acas_rac, a->acas_rat, a->acas_mte, a->acas_tti, a->acas_threat_id);
        }

        // Operational Status capabilities (TC31)
        if (trackDataValid(&a->opstatus_valid)) {
            s += sfmt(",\"opstatus\":{\"version\":%u", a->opstatus_version);
            s += sfmt(",\"om_acas_ra\":%u,\"om_ident\":%u,\"om_atc\":%u,\"om_saf\":%u",
                          a->opstatus_om_acas_ra, a->opstatus_om_ident, a->opstatus_om_atc, a->opstatus_om_saf);
            s += sfmt(",\"cc_acas\":%u,\"cc_cdti\":%u,\"cc_1090_in\":%u,\"cc_arv\":%u,\"cc_ts\":%u,\"cc_tc\":%u",
                          a->opstatus_cc_acas, a->opstatus_cc_cdti, a->opstatus_cc_1090_in, a->opstatus_cc_arv, a->opstatus_cc_ts, a->opstatus_cc_tc);
            s += sfmt(",\"cc_uat_in\":%u,\"cc_poa\":%u,\"cc_b2_low\":%u,\"cc_lw\":%u,\"cc_antenna_offset\":%u}",
                          a->opstatus_cc_uat_in, a->opstatus_cc_poa, a->opstatus_cc_b2_low, a->opstatus_cc_lw, a->opstatus_cc_antenna_offset);
        }

        if (a->modeA_hit)
            s += ",\"modea\":true";
        if (a->modeC_hit)
            s += ",\"modec\":true";

        // GPS integrity: 0=normal (omitted), 1=degraded, 2=suspect
        if (a->gps_integrity == 1)
            s += ",\"gps_integrity\":\"degraded\"";
        else if (a->gps_integrity == 2)
            s += ",\"gps_integrity\":\"suspect\"";

        // Circling detection
        if (a->circling)
            s += ",\"circling\":true";

        // DF19 (Military Extended Squitter)
        if (a->seen_df19)
            s += ",\"df19\":true";

        s += ",\"mlat\":";
        s += append_flags(a, SOURCE_MLAT);
        s += ",\"tisb\":";
        s += append_flags(a, SOURCE_TISB);

        s += sfmt(",\"messages\":%" PRId64 ",\"seen\":%.1f,\"rssi\":%.1f}",
                      a->messages, (now - a->seen)/1000.0,
                      10 * log10((a->signalLevel[0] + a->signalLevel[1] + a->signalLevel[2] + a->signalLevel[3] +
                                  a->signalLevel[4] + a->signalLevel[5] + a->signalLevel[6] + a->signalLevel[7] + 1e-5) / 8));
    }

    s += "\n  ]\n}\n";
    *len = (int)s.size();
    return strdup(s.c_str());
}

static void appendStatsJson(std::string &s,
                            struct stats *st,
                            const char *key)
{
    int32_t i;

    s += sfmt("\"%s\":{\"start\":%.1f,\"end\":%.1f",
              key,
              st->start / 1000.0,
              st->end / 1000.0);

    if (!Modes.net_only) {
        s += sfmt(",\"local\":{\"samples_processed\":%" PRIu64 ""
                           ",\"samples_dropped\":%" PRIu64 ""
                           ",\"modeac\":%u"
                           ",\"modes\":%u"
                           ",\"bad\":%u"
                           ",\"unknown_icao\":%u",
                           (uint64_t)st->samples_processed,
                           (uint64_t)st->samples_dropped,
                           st->demod_modeac,
                           st->demod_preambles,
                           st->demod_rejected_bad,
                           st->demod_rejected_unknown_icao);

        if (st->demod_crc_rescued)
            s += sfmt(",\"crc_rescued\":%u", st->demod_crc_rescued);

        for (i=0; i <= Modes.nfix_crc; ++i) {
            if (i == 0) s += sfmt(",\"accepted\":[%u", st->demod_accepted[i]);
            else s += sfmt(",%u", st->demod_accepted[i]);
        }

        s += ']';

        if (st->signal_power_sum > 0 && st->signal_power_count > 0)
            s += sfmt(",\"signal\":%.1f", 10 * log10(st->signal_power_sum / st->signal_power_count));
        if (st->noise_power_sum > 0 && st->noise_power_count > 0)
            s += sfmt(",\"noise\":%.1f", 10 * log10(st->noise_power_sum / st->noise_power_count));
        if (st->peak_signal_power > 0)
            s += sfmt(",\"peak_signal\":%.1f", 10 * log10(st->peak_signal_power));

        s += sfmt(",\"strong_signals\":%u", st->strong_signal_count);
        if (st->sdr_gain >= 0)
            s += sfmt(",\"gain_db\":%.1f", sdrGetGainDb(st->sdr_gain));
        s += '}';
    }

    if (Modes.net) {
        s += sfmt(",\"remote\":{\"modeac\":%u"
                           ",\"modes\":%u"
                           ",\"bad\":%u"
                           ",\"unknown_icao\":%u",
                           st->remote_received_modeac,
                           st->remote_received_modes,
                           st->remote_rejected_bad,
                           st->remote_rejected_unknown_icao);

        for (i=0; i <= Modes.nfix_crc; ++i) {
            if (i == 0) s += sfmt(",\"accepted\":[%u", st->remote_accepted[i]);
            else s += sfmt(",%u", st->remote_accepted[i]);
        }

        s += "]}";
    }

    uint64_t demod_cpu_millis = (uint64_t)st->demod_cpu.tv_sec*1000UL + st->demod_cpu.tv_nsec/1000000UL;
    uint64_t reader_cpu_millis = (uint64_t)st->reader_cpu.tv_sec*1000UL + st->reader_cpu.tv_nsec/1000000UL;
    uint64_t background_cpu_millis = (uint64_t)st->background_cpu.tv_sec*1000UL + st->background_cpu.tv_nsec/1000000UL;

    s += sfmt(",\"cpr\":{\"surface\":%u"
                      ",\"airborne\":%u"
                      ",\"global_ok\":%u"
                      ",\"global_bad\":%u"
                      ",\"global_range\":%u"
                      ",\"global_speed\":%u"
                      ",\"global_skipped\":%u"
                      ",\"local_ok\":%u"
                      ",\"local_aircraft_relative\":%u"
                      ",\"local_receiver_relative\":%u"
                      ",\"local_skipped\":%u"
                      ",\"local_range\":%u"
                      ",\"local_speed\":%u"
                      ",\"filtered\":%u}"
                      ",\"altitude_suppressed\":%u"
                      ",\"cpu\":{\"demod\":%" PRIu64 ",\"reader\":%" PRIu64 ",\"background\":%" PRIu64 "}"
                      ",\"tracks\":{\"all\":%u"
                      ",\"single_message\":%u"
                      ",\"unreliable\":%u}"
                      ",\"messages\":%u",
                      st->cpr_surface,
                      st->cpr_airborne,
                      st->cpr_global_ok,
                      st->cpr_global_bad,
                      st->cpr_global_range_checks,
                      st->cpr_global_speed_checks,
                      st->cpr_global_skipped,
                      st->cpr_local_ok,
                      st->cpr_local_aircraft_relative,
                      st->cpr_local_receiver_relative,
                      st->cpr_local_skipped,
                      st->cpr_local_range_checks,
                      st->cpr_local_speed_checks,
                      st->cpr_filtered,
                      st->suppressed_altitude_messages,
                      (uint64_t)demod_cpu_millis,
                      (uint64_t)reader_cpu_millis,
                      (uint64_t)background_cpu_millis,
                      st->unique_aircraft,
                      st->single_message_aircraft,
                      st->unreliable_aircraft,
                      st->messages_total);

    for (i = 0; i < 32; ++i) {
        if (i == 0)
            s += sfmt(",\"messages_by_df\":[%u", st->messages_by_df[i]);
        else
            s += sfmt(",%u", st->messages_by_df[i]);
    }
    s += ']';

    if (st->adaptive_valid) {
        s += sfmt(",\"adaptive\":"
                          "{\"gain_db\":%.1f"
                          ",\"dynamic_range_limit_db\":%.1f"
                          ",\"gain_changes\":%u"
                          ",\"loud_undecoded\":%u"
                          ",\"loud_decoded\":%u"
                          ",\"noise_dbfs\":%.1f"
                          ",\"gain_seconds\":[",
                          sdrGetGainDb(st->sdr_gain),
                          sdrGetGainDb(st->adaptive_range_gain_limit),
                          st->adaptive_gain_changes,
                          st->adaptive_loud_undecoded,
                          st->adaptive_loud_decoded,
                          st->adaptive_noise_dbfs);
        bool first = true;
        for (uint32_t i = 0; i < STATS_GAIN_COUNT; ++i) {
            if (st->adaptive_gain_seconds[i] > 0) {
                s += sfmt("%s[%.1f,%u]",
                                  first ? "" : ",",
                                  sdrGetGainDb(i), st->adaptive_gain_seconds[i]);
                first = false;
            }
        }
        s += "]}";
    }
    s += '}';
}

char *generateStatsJson(const char *url_path, int32_t *len) {
    MODES_NOTUSED(url_path);

    std::string s = "{\n";
    appendStatsJson(s, &Modes.stats_latest, "latest");
    s += ",\n";

    appendStatsJson(s, &Modes.stats_1min[Modes.stats_newest_1min], "last1min");
    s += ",\n";

    appendStatsJson(s, &Modes.stats_5min, "last5min");
    s += ",\n";

    appendStatsJson(s, &Modes.stats_15min, "last15min");
    s += ",\n";

    appendStatsJson(s, &Modes.stats_alltime, "total");
    s += "\n}\n";

    *len = (int)s.size();
    return strdup(s.c_str());
}

//
// Return a description of the receiver in json.
//
char *generateReceiverJson(const char *url_path, int32_t *len)
{
    int32_t history_size;

    MODES_NOTUSED(url_path);

    // work out number of valid history entries
    if (Modes.json_aircraft_history[HISTORY_SIZE-1].content == NULL)
        history_size = Modes.json_aircraft_history_next;
    else
        history_size = HISTORY_SIZE;

    std::string s = sfmt("{ "
                 "\"version\" : \"%s\", "
                 "\"refresh\" : %.0f, "
                 "\"history\" : %d",
                 MODES_DUMP1090_VERSION, 1.0*Modes.json_interval, history_size);

    if (Modes.json_location_accuracy && (Modes.fUserLat != 0.0 || Modes.fUserLon != 0.0)) {
        if (Modes.json_location_accuracy == 1) {
            s += sfmt(", "
                         "\"lat\" : %.2f, "
                         "\"lon\" : %.2f",
                         Modes.fUserLat, Modes.fUserLon);
        } else {
            s += sfmt(", "
                         "\"lat\" : %.6f, "
                         "\"lon\" : %.6f",
                         Modes.fUserLat, Modes.fUserLon);
        }
    }

    s += " }\n";

    *len = (int)s.size();
    return strdup(s.c_str());
}

char *generateHistoryJson(const char *url_path, int32_t *len)
{
    int32_t history_index = -1;

    if (sscanf(url_path, "/data/history_%d.json", &history_index) != 1)
        return NULL;

    if (history_index < 0 || history_index >= HISTORY_SIZE)
        return NULL;

    if (!Modes.json_aircraft_history[history_index].content)
        return NULL;

    *len = Modes.json_aircraft_history[history_index].clen;
    return strdup(Modes.json_aircraft_history[history_index].content);
}

static void ratelimitWriteError(const char *format, ...)
{
    static uint64_t lastError = 0;
    static uint32_t suppressed = 0;

    uint64_t now = mstime();
    if (now - lastError < 60000) {
        ++suppressed;
        return;
    }

    lastError = now;

    va_list ap;
    va_start(ap, format);
    vfprintf(stderr, format, ap);
    if (suppressed) {
        gg::eprint(" (%u more error messages suppressed)", suppressed);
        suppressed = 0;
    }
    gg::eprint("\n");
    va_end(ap);
}

// Write JSON to file
void writeJsonToFile(const char *file, char * (*generator) (const char *,int32_t*))
{
#ifndef _WIN32
    int32_t fd;
    int32_t len = 0;
    mode_t mask;
    char *content;

    if (!Modes.json_dir)
        return;

    std::string tmppath = std::string(Modes.json_dir) + "/" + file + ".XXXXXX";
    fd = mkstemp(tmppath.data());
    if (fd < 0) {
        ratelimitWriteError("failed to create %s (while updating %s/%s): %s", tmppath.c_str(), Modes.json_dir, file, strerror(errno));
        return;
    }

    mask = umask(0);
    umask(mask);
    fchmod(fd, 0644 & ~mask);

    std::string url_path = std::string("/data/") + file;
    content = generator(url_path.c_str(), &len);

    if (write(fd, content, len) != len) {
        ratelimitWriteError("failed to write to %s (while updating %s/%s): %s", tmppath.c_str(), Modes.json_dir, file, strerror(errno));
        goto error_1;
    }

    if (close(fd) < 0) {
        ratelimitWriteError("failed to write to %s (while updating %s/%s): %s", tmppath.c_str(), Modes.json_dir, file, strerror(errno));
        goto error_2;
    }

    {
        std::string destpath = std::string(Modes.json_dir) + "/" + file;
        if (rename(tmppath.c_str(), destpath.c_str()) < 0) {
            ratelimitWriteError("failed to rename %s to %s: %s", tmppath.c_str(), destpath.c_str(), strerror(errno));
            goto error_2;
        }
    }

    free(content);
    return;

 error_1:
    close(fd);
 error_2:
    unlink(tmppath.c_str());
    free(content);
    return;
#endif
}


//
//=========================================================================
//
// This function polls the clients using read() in order to receive new
// messages from the net.
//
// The message is supposed to be separated from the next message by the
// separator 'sep', which is a null-terminated C string.
//
// Every full message received is decoded and passed to the higher layers
// calling the function's 'handler'.
//
// The handler returns 0 on success, or 1 to signal this function we should
// close the connection with the client in case of non-recoverable errors.
//
static void modesReadFromClient(struct client *c) {
    int32_t left;
    int32_t nread;
    int32_t bContinue = 1;

    while (bContinue) {
        left = MODES_CLIENT_BUF_SIZE - c->buflen - 1; // leave 1 extra byte for NUL termination in the ASCII case

        // If our buffer is full discard it, this is some badly formatted shit
        if (left <= 0) {
            c->buflen = 0;
            left = MODES_CLIENT_BUF_SIZE;
            // If there is garbage, read more to discard it ASAP
        }
#ifndef _WIN32
        nread = read(c->fd, c->buf+c->buflen, left);
#else
        nread = recv(c->fd, c->buf+c->buflen, left, 0);
        if (nread < 0) {errno = WSAGetLastError();}
#endif

        // If we didn't get all the data we asked for, then return once we've processed what we did get.
        if (nread != left) {
            bContinue = 0;
        }

        if (nread == 0) { // End of file
            modesCloseClient(c);
            return;
        }

#ifndef _WIN32
        if (nread < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) // No data available (not really an error)
#else
        if (nread < 0 && errno == EWOULDBLOCK) // No data available (not really an error)
#endif
        {
            return;
        }

        if (nread < 0) { // Other errors
            modesCloseClient(c);
            return;
        }

        __atomic_fetch_add(&c->service->bytes_in_total,
                           (uint64_t)nread, __ATOMIC_RELAXED);

        c->buflen += nread;

        char *som = c->buf;           // first byte of next message
        char *eod = som + c->buflen;  // one byte past end of data
        char *p;

        switch (c->service->read_mode) {
        case READ_MODE_IGNORE:
            // drop the bytes on the floor
            som = eod;
            break;

        case READ_MODE_BEAST:
            // This is the Beast Binary scanning case.
            // If there is a complete message still in the buffer, there must be the separator 'sep'
            // in the buffer, note that we full-scan the buffer at every read for simplicity.

            while (som < eod && ((p = (char*)memchr(som, (char) 0x1a, eod - som)) != NULL)) { // The first byte of buffer 'should' be 0x1a
                som = p; // consume garbage up to the 0x1a
                ++p; // skip 0x1a

                if (p >= eod) {
                    // Incomplete message in buffer, retry later
                    break;
                }

                char *eom; // one byte past end of message
                if        (*p == '1') {
                    eom = p + MODEAC_MSG_BYTES      + 8;         // point past remainder of message
                } else if (*p == '2') {
                    eom = p + MODES_SHORT_MSG_BYTES + 8;
                } else if (*p == '3') {
                    eom = p + MODES_LONG_MSG_BYTES  + 8;
                } else if (*p == '4') {
                    eom = p + MODES_LONG_MSG_BYTES  + 8;
                } else if (*p == '5') {
                    eom = p + MODES_LONG_MSG_BYTES  + 8;
                } else {
                    // Not a valid beast message, skip 0x1a and try again
                    ++som;
                    continue;
                }

                // we need to be careful of double escape characters in the message body
                for (p = som + 1; p < eod && p < eom; p++) {
                    if (0x1A == *p) {
                        p++;
                        eom++;
                    }
                }

                if (eom > eod) { // Incomplete message in buffer, retry later
                    break;
                }

                // Have a 0x1a followed by 1/2/3/4/5 - pass message to handler.
                if (c->service->read_handler(c, som + 1)) {
                    modesCloseClient(c);
                    return;
                }

                // advance to next message
                som = eom;
            }
            break;

        case READ_MODE_BEAST_COMMAND:
            while (som < eod && ((p = (char*)memchr(som, (char) 0x1a, eod - som)) != NULL)) { // The first byte of buffer 'should' be 0x1a
                char *eom; // one byte past end of message

                som = p; // consume garbage up to the 0x1a
                ++p; // skip 0x1a

                if (p >= eod) {
                    // Incomplete message in buffer, retry later
                    break;
                }

                if (*p == '1') {
                    eom = p + 2;
                } else {
                    // Not a valid beast command, skip 0x1a and try again
                    ++som;
                    continue;
                }

                // we need to be careful of double escape characters in the message body
                for (p = som + 1; p < eod && p < eom; p++) {
                    if (0x1A == *p) {
                        p++;
                        eom++;
                    }
                }

                if (eom > eod) { // Incomplete message in buffer, retry later
                    break;
                }

                // Have a 0x1a followed by 1 - pass message to handler.
                if (c->service->read_handler(c, som + 1)) {
                    modesCloseClient(c);
                    return;
                }

                // advance to next message
                som = eom;
            }
            break;

        case READ_MODE_ASCII:
            //
            // This is the ASCII scanning case, AVR RAW or HTTP at present
            // If there is a complete message still in the buffer, there must be the separator 'sep'
            // in the buffer, note that we full-scan the buffer at every read for simplicity.

            // Always NUL-terminate so we are free to use strstr()
            // nb: we never fill the last byte of the buffer with read data (see above) so this is safe
            *eod = '\0';

            while (som < eod && (p = strstr(som, c->service->read_sep)) != NULL) { // end of first message if found
                *p = '\0';                         // The handler expects null terminated strings
                if (c->service->read_handler(c, som)) {         // Pass message to handler.
                    modesCloseClient(c);           // Handler returns 1 on error to signal we .
                    return;                        // should close the client connection
                }
                som = p + strlen(c->service->read_sep);               // Move to start of next message
            }

            break;
        }

        if (som > c->buf) {                        // We processed something - so
            c->buflen = eod - som;                 //     Update the unprocessed buffer length
            memmove(c->buf, som, c->buflen);       //     Move what's remaining to the start of the buffer
        } else {                                   // If no message was decoded process the next client
            return;
        }
    }
}

__attribute__ ((format (printf,4,5))) static char *appendFATSV(char *p, char *end, const char *field, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);

    p = safe_snprintf(p, end, "%s\t", field);
    p = safe_vsnprintf(p, end, format, ap);
    p = safe_snprintf(p, end, "\t");

    va_end(ap);
    return p;
}

#define TSV_MAX_PACKET_SIZE 800
#define TSV_VERSION "9E"

static void writeFATSVPositionUpdate(float lat, float lon, float alt)
{
    static float last_lat, last_lon, last_alt;

    if (lat == last_lat && lon == last_lon && alt == last_alt)
        return;

    last_lat = lat;
    last_lon = lon;
    last_alt = alt;

    char *p = prepareWrite(&Modes.fatsv_out, TSV_MAX_PACKET_SIZE);
    if (!p)
        return;

    char *end = p + TSV_MAX_PACKET_SIZE;

    p = appendFATSV(p, end, "_v",     "%s", TSV_VERSION);
    p = appendFATSV(p, end, "clock",  "%" PRIu64, messageNow() / 1000);
    p = appendFATSV(p, end, "type",   "%s",       "location_update");
    p = appendFATSV(p, end, "lat",    "%.5f",     lat);
    p = appendFATSV(p, end, "lon",    "%.5f",     lon);
    p = appendFATSV(p, end, "alt",    "%.0f",     alt);
    p = appendFATSV(p, end, "altref", "%s",       "egm96_meters");
    --p; // remove last tab
    p = safe_snprintf(p, end, "\n");

    if (p < end)
        completeWrite(&Modes.fatsv_out, p);
    else
        gg::eprint("fatsv: output too large (max %d, overran by %d)\n", TSV_MAX_PACKET_SIZE, (int32_t) (p - end));
}

static void writeFATSVEventMessage(struct modesMessage *mm, const char *datafield, uint8_t *data, size_t len)
{
    char *p = prepareWrite(&Modes.fatsv_out, TSV_MAX_PACKET_SIZE);
    if (!p)
        return;

    char *end = p + TSV_MAX_PACKET_SIZE;

    p = appendFATSV(p, end, "_v",    "%s", TSV_VERSION);
    p = appendFATSV(p, end, "clock", "%" PRIu64, messageNow() / 1000);
    p = appendFATSV(p, end, (mm->addr & MODES_NON_ICAO_ADDRESS) ? "otherid" : "hexid", "%06X", mm->addr & 0xFFFFFF);
    if (mm->addrtype != ADDR_ADSB_ICAO) {
        p = appendFATSV(p, end, "addrtype", "%s", addrtype_enum_string(mm->addrtype));
    }

    p = safe_snprintf(p, end, "%s\t", datafield);
    for (size_t i = 0; i < len; ++i) {
        p = safe_snprintf(p, end, "%02X", data[i]);
    }
    p = safe_snprintf(p, end, "\n");

    if (p < end)
        completeWrite(&Modes.fatsv_out, p);
    else
        gg::eprint("fatsv: output too large (max %d, overran by %d)\n", TSV_MAX_PACKET_SIZE, (int32_t) (p - end));
#       undef bufsize
}

static void writeFATSVEvent(struct modesMessage *mm, struct aircraft *a)
{
    // Write event records for a couple of message types.

    if (!Modes.fatsv_out.service || !Modes.fatsv_out.service->connections) {
        return; // not enabled or no active connections
    }

    if (!a || mm->source == SOURCE_MLAT || (!a->reliable && !mm->reliable))
        return;

    switch (mm->msgtype) {
    case 20:
    case 21:
        // DF 20/21: Comm-B: emit if they've changed since we last sent them
        switch (mm->commb_format) {
        case COMMB_DATALINK_CAPS:
            // BDS 1,0: data link capability report
            if (memcmp(mm->MB, a->fatsv_emitted_bds_10, 7) != 0) {
                memcpy(a->fatsv_emitted_bds_10, mm->MB, 7);
                writeFATSVEventMessage(mm, "datalink_caps", mm->MB, 7);
            }
            break;

        case COMMB_ACAS_RA:
            // BDS 3,0: ACAS RA report
            if (memcmp(mm->MB, a->fatsv_emitted_bds_30, 7) != 0) {
                memcpy(a->fatsv_emitted_bds_30, mm->MB, 7);
                writeFATSVEventMessage(mm, "commb_acas_ra", mm->MB, 7);
            }
            break;

        case COMMB_GICB_CAPS:
            // BDS 1,7: common usage GICB capability report
            if (memcmp(mm->MB, a->fatsv_emitted_bds_17, 7) != 0) {
                memcpy(a->fatsv_emitted_bds_17, mm->MB, 7);
                writeFATSVEventMessage(mm, "gicb_caps", mm->MB, 7);
            }
            break;

        case COMMB_UNKNOWN:
            // If enabled, upload raw unrecognized Comm-B messages
            // for server-side analysis
            if (Modes.faup_upload_unknown_commb && memcmp(mm->MB, a->fatsv_emitted_unknown_commb, 7) != 0) {
                memcpy(a->fatsv_emitted_unknown_commb, mm->MB, 7);
                writeFATSVEventMessage(mm, "unknown_commb", mm->MB, 7);
            }
            break;

        default:
            // nothing
            break;
        }
        break;

    case 17:
    case 18:
        // DF 17/18: extended squitter
        if (mm->metype == 28 && mm->mesub == 2 && memcmp(mm->ME, &a->fatsv_emitted_es_acas_ra, 7) != 0) {
            // type 28 subtype 2: ACAS RA report
            // first byte has the type/subtype, remaining bytes match the BDS 3,0 format
            memcpy(a->fatsv_emitted_es_acas_ra, mm->ME, 7);
            writeFATSVEventMessage(mm, "es_acas_ra", mm->ME, 7);
        } else if (mm->metype == 31 && (mm->mesub == 0 || mm->mesub == 1) && memcmp(mm->ME, a->fatsv_emitted_es_status, 7) != 0) {
            // aircraft operational status
            memcpy(a->fatsv_emitted_es_status, mm->ME, 7);
            writeFATSVEventMessage(mm, "es_op_status", mm->ME, 7);
        }
        break;
    }
}

static inline uint32_t unsigned_difference(uint32_t v1, uint32_t v2)
{
    return (v1 > v2) ? (v1 - v2) : (v2 - v1);
}

static inline float heading_difference(float h1, float h2)
{
    float d = fabs(h1 - h2);
    return (d < 180) ? d : (360 - d);
}

 __attribute__ ((format (printf,6,7))) static char *appendFATSVMeta(char *p, char *end, const char *field, struct aircraft *a, const data_validity *source, const char *format, ...)
{
    const char *sourcetype;
    switch (source->source) {
    case SOURCE_MODE_S:
        sourcetype = "U";
        break;
    case SOURCE_MODE_S_CHECKED:
        sourcetype = "S";
        break;
    case SOURCE_TISB:
        sourcetype = "T";
        break;
    case SOURCE_ADSR:
        sourcetype = "R";
        break;
    case SOURCE_ADSB:
        sourcetype = "A";
        break;
    default:
        // don't want to forward data sourced from these
        return p;
    }

    if (!trackDataValid(source)) {
        // expired data
        return p;
    }

    if (source->updated > messageNow()) {
        // data in the future
        return p;
    }

    if (source->updated < a->fatsv_last_emitted) {
        // not updated since last time
        return p;
    }

    uint64_t age = (messageNow() - source->updated) / 1000;
    if (age > 255) {
        // too old
        return p;
    }

    p = safe_snprintf(p, end, "%s\t", field);

    va_list ap;
    va_start(ap, format);
    p = safe_vsnprintf(p, end, format, ap);
    va_end(ap);

    p = safe_snprintf(p, end, " %" PRIu64 " %s\t", age, sourcetype);
    return p;
}

static const char *airground_enum_string(airground_t ag)
{
    switch (ag) {
    case AG_AIRBORNE:
        return "A+";
    case AG_GROUND:
        return "G+";
    default:
        return "?";
    }
}

static void writeFATSV()
{
    struct aircraft *a;
    static uint64_t next_update;

    if (!Modes.fatsv_out.service || !Modes.fatsv_out.service->connections) {
        return; // not enabled or no active connections
    }

    uint64_t now = mstime();
    if (now < next_update) {
        return;
    }

    // scan once a second at most
    next_update = now + 1000;

    for (a = Modes.aircrafts; a; a = a->next) {
        if (!a->reliable)
            continue;

        // don't emit if it hasn't updated since last time
        if (a->seen < a->fatsv_last_emitted) {
            continue;
        }

        // Pretend we are "processing a message" so the validity checks work as expected
        _messageNow = a->seen;

        // some special cases:
        int32_t altValid = trackDataValid(&a->altitude_baro_valid);
        int32_t airgroundValid = trackDataValid(&a->airground_valid) && a->airground_valid.source >= SOURCE_MODE_S_CHECKED; // for non-ADS-B transponders, only trust DF11 CA field
        int32_t gsValid = trackDataValid(&a->gs_valid);
        int32_t squawkValid = trackDataValid(&a->squawk_valid);
        int32_t callsignValid = trackDataValid(&a->callsign_valid) && strcmp(a->callsign, "        ") != 0;
        int32_t positionValid = trackDataValid(&a->position_valid);

        // If we are definitely on the ground, suppress any unreliable altitude info.
        // When on the ground, ADS-B transponders don't emit an ADS-B message that includes
        // altitude, so a corrupted Mode S altitude response from some other in-the-air AC
        // might be taken as the "best available altitude" and produce e.g. "airGround G+ alt 31000".
        if (airgroundValid && a->airground == AG_GROUND && a->altitude_baro_valid.source < SOURCE_MODE_S_CHECKED)
            altValid = 0;

        // if it hasn't changed altitude, heading, or speed much,
        // don't update so often
        int32_t changed =
            (altValid && abs(a->altitude_baro - a->fatsv_emitted_altitude_baro) >= 50) ||
            (trackDataValid(&a->altitude_geom_valid) && abs(a->altitude_geom - a->fatsv_emitted_altitude_geom) >= 50) ||
            (trackDataValid(&a->baro_rate_valid) && abs(a->baro_rate - a->fatsv_emitted_baro_rate) > 500) ||
            (trackDataValid(&a->geom_rate_valid) && abs(a->geom_rate - a->fatsv_emitted_geom_rate) > 500) ||
            (trackDataValid(&a->track_valid) && heading_difference(a->track, a->fatsv_emitted_track) >= 2) ||
            (trackDataValid(&a->track_rate_valid) && fabs(a->track_rate - a->fatsv_emitted_track_rate) >= 0.5) ||
            (trackDataValid(&a->roll_valid) && fabs(a->roll - a->fatsv_emitted_roll) >= 5.0) ||
            (trackDataValid(&a->mag_heading_valid) && heading_difference(a->mag_heading, a->fatsv_emitted_mag_heading) >= 2) ||
            (trackDataValid(&a->true_heading_valid) && heading_difference(a->true_heading, a->fatsv_emitted_true_heading) >= 2) ||
            (gsValid && fabs(a->gs - a->fatsv_emitted_gs) >= 25) ||
            (trackDataValid(&a->ias_valid) && unsigned_difference(a->ias, a->fatsv_emitted_ias) >= 25) ||
            (trackDataValid(&a->tas_valid) && unsigned_difference(a->tas, a->fatsv_emitted_tas) >= 25) ||
            (trackDataValid(&a->mach_valid) && fabs(a->mach - a->fatsv_emitted_mach) >= 0.02);

        int32_t immediate =
            (trackDataValid(&a->nav_altitude_mcp_valid) && abs(a->nav_altitude_mcp - a->fatsv_emitted_nav_altitude_mcp) > 50) ||
            (trackDataValid(&a->nav_altitude_fms_valid) && abs(a->nav_altitude_fms - a->fatsv_emitted_nav_altitude_fms) > 50) ||
            (trackDataValid(&a->nav_altitude_src_valid) && a->nav_altitude_src != a->fatsv_emitted_nav_altitude_src) ||
            (trackDataValid(&a->nav_heading_valid) && heading_difference(a->nav_heading, a->fatsv_emitted_nav_heading) > 2) ||
            (trackDataValid(&a->nav_modes_valid) && a->nav_modes != a->fatsv_emitted_nav_modes) ||
            (trackDataValid(&a->nav_qnh_valid) && fabs(a->nav_qnh - a->fatsv_emitted_nav_qnh) > 0.8) || // 0.8 is the ES message resolution
            (callsignValid && strcmp(a->callsign, a->fatsv_emitted_callsign) != 0) ||
            (airgroundValid && a->airground == AG_AIRBORNE && a->fatsv_emitted_airground == AG_GROUND) ||
            (airgroundValid && a->airground == AG_GROUND && a->fatsv_emitted_airground == AG_AIRBORNE) ||
            (squawkValid && a->squawk != a->fatsv_emitted_squawk) ||
            (trackDataValid(&a->emergency_valid) && a->emergency != a->fatsv_emitted_emergency) ||
            (trackDataValid(&a->mrar_source_valid) && a->mrar_source_valid.updated > a->fatsv_last_emitted) ||
            (trackDataValid(&a->wind_valid) && a->wind_valid.updated > a->fatsv_last_emitted) ||
            (trackDataValid(&a->pressure_valid) && a->pressure_valid.updated > a->fatsv_last_emitted) ||
            (trackDataValid(&a->temperature_valid) && a->temperature_valid.updated > a->fatsv_last_emitted) ||
            (trackDataValid(&a->turbulence_valid) && a->turbulence_valid.updated > a->fatsv_last_emitted) ||
            (trackDataValid(&a->humidity_valid) && a->humidity_valid.updated > a->fatsv_last_emitted);

        uint64_t minAge;
        double adjustedMinAge;
        if (immediate) {
            // a change we want to emit right away
            minAge = 0;
        } else if (!positionValid) {
            // don't send mode S very often
            minAge = 30000;
        } else if ((airgroundValid && a->airground == AG_GROUND) ||
                   (altValid && a->altitude_baro < 500 && (!gsValid || a->gs < 200)) ||
                   (gsValid && a->gs < 100 && (!altValid || a->altitude_baro < 1000))) {
            // we are probably on the ground, increase the update rate
            minAge = 1000;
        } else if (!altValid || a->altitude_baro < 10000) {
            // Below 10000 feet, emit up to every 5s when changing, 10s otherwise
            minAge = (changed ? 5000 : 10000);
        } else {
            // Above 10000 feet, emit up to every 10s when changing, 30s otherwise
            minAge = (changed ? 10000 : 30000);
        }

        // Adjust rate for multiplier
        adjustedMinAge = minAge / Modes.faup_rate_multiplier;

        if ((now - a->fatsv_last_emitted) < adjustedMinAge) {
            continue;
        }

        char *p = prepareWrite(&Modes.fatsv_out, TSV_MAX_PACKET_SIZE);
        if (!p)
            return;
        char *end = p + TSV_MAX_PACKET_SIZE;

        p = appendFATSV(p, end, "_v",    "%s", TSV_VERSION);
        p = appendFATSV(p, end, "clock", "%" PRIu64, messageNow() / 1000);
        p = appendFATSV(p, end, (a->addr & MODES_NON_ICAO_ADDRESS) ? "otherid" : "hexid", "%06X", a->addr & 0xFFFFFF);

        // for fields we only emit on change,
        // occasionally re-emit them all
        int32_t forceEmit = (now - a->fatsv_last_force_emit) > 600000;

        // these don't change often / at all, only emit when they change
        if (forceEmit || a->addrtype != a->fatsv_emitted_addrtype) {
            p = appendFATSV(p, end, "addrtype", "%s", addrtype_enum_string(a->addrtype));
        }
        if (forceEmit || a->adsb_version != a->fatsv_emitted_adsb_version) {
            p = appendFATSV(p, end, "adsb_version", "%d", a->adsb_version);
        }
        if (forceEmit || a->category != a->fatsv_emitted_category) {
            p = appendFATSV(p, end, "category", "%02X", a->category);
        }
        if (trackDataValid(&a->nac_p_valid) && (forceEmit || a->nac_p != a->fatsv_emitted_nac_p)) {
            p = appendFATSVMeta(p, end, "nac_p",       a, &a->nac_p_valid,         "%u",       a->nac_p);
        }
        if (trackDataValid(&a->nac_v_valid) && (forceEmit || a->nac_v != a->fatsv_emitted_nac_v)) {
            p = appendFATSVMeta(p, end, "nac_v",       a, &a->nac_v_valid,         "%u",       a->nac_v);
        }
        if (trackDataValid(&a->sil_valid) && (forceEmit || a->sil != a->fatsv_emitted_sil)) {
            p = appendFATSVMeta(p, end, "sil",         a, &a->sil_valid,           "%u",       a->sil);
        }
        if (trackDataValid(&a->sil_valid) && (forceEmit || a->sil_type != a->fatsv_emitted_sil_type)) {
            p = appendFATSVMeta(p, end, "sil_type",    a, &a->sil_valid,           "%s",       sil_type_enum_string(a->sil_type));
        }
        if (trackDataValid(&a->nic_baro_valid) && (forceEmit || a->nic_baro != a->fatsv_emitted_nic_baro)) {
            p = appendFATSVMeta(p, end, "nic_baro",    a, &a->nic_baro_valid,      "%u",       (uint32_t) a->nic_baro);
        }

        // only emit alt, speed, latlon, track etc if they have been received since the last time
        // and are not stale

        char *dataStart = p;

        // special cases
        if (airgroundValid)
            p = appendFATSVMeta(p, end, "airGround", a, &a->airground_valid,      "%s",   airground_enum_string(a->airground));
        if (squawkValid)
            p = appendFATSVMeta(p, end, "squawk", a, &a->squawk_valid,        "%04x", a->squawk);
        if (callsignValid)
            p = appendFATSVMeta(p, end, "ident", a, &a->callsign_valid,       "{%s}", a->callsign);
        if (altValid)
            p = appendFATSVMeta(p, end, "alt",   a, &a->altitude_baro_valid,  "%d",   a->altitude_baro);
        if (positionValid) {
            p = appendFATSVMeta(p, end, "position", a, &a->position_valid,  "{%.5f %.5f %u %u}", a->lat, a->lon, a->pos_nic, a->pos_rc);
        }

        p = appendFATSVMeta(p, end, "alt_gnss",    a, &a->altitude_geom_valid,  "%d",   a->altitude_geom);
        p = appendFATSVMeta(p, end, "vrate",       a, &a->baro_rate_valid,      "%d",   a->baro_rate);
        p = appendFATSVMeta(p, end, "vrate_geom",  a, &a->geom_rate_valid,      "%d",   a->geom_rate);
        p = appendFATSVMeta(p, end, "speed",       a, &a->gs_valid,             "%.1f", a->gs);
        p = appendFATSVMeta(p, end, "speed_ias",   a, &a->ias_valid,            "%u",   a->ias);
        p = appendFATSVMeta(p, end, "speed_tas",   a, &a->tas_valid,            "%u",   a->tas);
        p = appendFATSVMeta(p, end, "mach",        a, &a->mach_valid,           "%.3f", a->mach);
        p = appendFATSVMeta(p, end, "track",       a, &a->track_valid,          "%.1f", a->track);
        p = appendFATSVMeta(p, end, "track_rate",  a, &a->track_rate_valid,     "%.2f", a->track_rate);
        p = appendFATSVMeta(p, end, "roll",        a, &a->roll_valid,           "%.1f", a->roll);
        p = appendFATSVMeta(p, end, "heading_magnetic", a, &a->mag_heading_valid, "%.1f", a->mag_heading);
        p = appendFATSVMeta(p, end, "heading_true", a, &a->true_heading_valid,    "%.1f", a->true_heading);
        p = appendFATSVMeta(p, end, "nav_alt_mcp", a, &a->nav_altitude_mcp_valid, "%d",   a->nav_altitude_mcp);
        p = appendFATSVMeta(p, end, "nav_alt_fms", a, &a->nav_altitude_fms_valid, "%d",   a->nav_altitude_fms);
        p = appendFATSVMeta(p, end, "nav_alt_src", a, &a->nav_altitude_src_valid, "%s", nav_altitude_source_enum_string(a->nav_altitude_src));
        p = appendFATSVMeta(p, end, "nav_heading", a, &a->nav_heading_valid,    "%.1f", a->nav_heading);
        p = appendFATSVMeta(p, end, "nav_modes",   a, &a->nav_modes_valid,      "{%s}", nav_modes_flags_string(a->nav_modes));
        p = appendFATSVMeta(p, end, "nav_qnh",     a, &a->nav_qnh_valid,        "%.1f", a->nav_qnh);
        p = appendFATSVMeta(p, end, "emergency",   a, &a->emergency_valid,      "%s",   emergency_enum_string(a->emergency));
        p = appendFATSVMeta(p, end, "mrar_source", a, &a->mrar_source_valid,    "%s",   mrar_source_enum_string(a->mrar_source));
        p = appendFATSVMeta(p, end, "wind_speed",  a, &a->wind_valid,           "%.0f", a->wind_speed);
        p = appendFATSVMeta(p, end, "wind_dir",    a, &a->wind_valid,           "%.1f", a->wind_dir);
        p = appendFATSVMeta(p, end, "temperature", a, &a->temperature_valid,    "%.2f", a->temperature);
        p = appendFATSVMeta(p, end, "pressure",    a, &a->pressure_valid,       "%.0f", a->pressure);
        p = appendFATSVMeta(p, end, "turbulence",  a, &a->turbulence_valid,     "%s",   hazard_enum_string(a->turbulence));
        p = appendFATSVMeta(p, end, "humidity",    a, &a->humidity_valid,       "%.0f", a->humidity);

        // if we didn't get anything interesting, bail out.
        // We don't need to do anything special to unwind prepareWrite().
        if (p == dataStart) {
            continue;
        }

        --p; // remove last tab
        p = safe_snprintf(p, end, "\n");

        if (p < end)
            completeWrite(&Modes.fatsv_out, p);
        else
            gg::eprint("fatsv: output too large (max %d, overran by %d)\n", TSV_MAX_PACKET_SIZE, (int32_t) (p - end));

        a->fatsv_emitted_altitude_baro = a->altitude_baro;
        a->fatsv_emitted_altitude_geom = a->altitude_geom;
        a->fatsv_emitted_baro_rate = a->baro_rate;
        a->fatsv_emitted_geom_rate = a->geom_rate;
        a->fatsv_emitted_gs = a->gs;
        a->fatsv_emitted_ias = a->ias;
        a->fatsv_emitted_tas = a->tas;
        a->fatsv_emitted_mach = a->mach;
        a->fatsv_emitted_track = a->track;
        a->fatsv_emitted_track_rate = a->track_rate;
        a->fatsv_emitted_roll = a->roll;
        a->fatsv_emitted_mag_heading = a->mag_heading;
        a->fatsv_emitted_true_heading = a->true_heading;
        a->fatsv_emitted_airground = a->airground;
        a->fatsv_emitted_nav_altitude_mcp = a->nav_altitude_mcp;
        a->fatsv_emitted_nav_altitude_fms = a->nav_altitude_fms;
        a->fatsv_emitted_nav_altitude_src = a->nav_altitude_src;
        a->fatsv_emitted_nav_heading = a->nav_heading;
        a->fatsv_emitted_nav_modes = a->nav_modes;
        a->fatsv_emitted_nav_qnh = a->nav_qnh;
        memcpy(a->fatsv_emitted_callsign, a->callsign, sizeof(a->fatsv_emitted_callsign));
        a->fatsv_emitted_addrtype = a->addrtype;
        a->fatsv_emitted_adsb_version = a->adsb_version;
        a->fatsv_emitted_category = a->category;
        a->fatsv_emitted_squawk = a->squawk;
        a->fatsv_emitted_nac_p = a->nac_p;
        a->fatsv_emitted_nac_v = a->nac_v;
        a->fatsv_emitted_sil = a->sil;
        a->fatsv_emitted_sil_type = a->sil_type;
        a->fatsv_emitted_nic_baro = a->nic_baro;
        a->fatsv_emitted_emergency = a->emergency;
        a->fatsv_last_emitted = now;
        if (forceEmit) {
            a->fatsv_last_force_emit = now;
        }
    }
}

//
// Perform periodic network work
//
void modesNetPeriodicWork(void) {
    struct client *c, **prev;
    struct net_service *s;
    uint64_t now = mstime();
    int32_t need_flush = 0;

    // Accept new connections
    modesAcceptClients();

    // Built-in MLAT client now runs in its own thread (see feeder_thread.c)

    // Built-in PiAware client now runs in its own thread (see feeder_thread.c)

    // Read from clients
    for (c = Modes.clients; c; c = c->next) {
        if (!c->service)
            continue;
        if (c->service->read_handler)
            modesReadFromClient(c);
    }

    // Generate FATSV output
    writeFATSV();

    // If we have generated no messages for a while, send
    // a heartbeat
    if (Modes.net_heartbeat_interval) {
        for (s = Modes.services; s; s = s->next) {
            if (s->writer &&
                s->connections &&
                s->writer->send_heartbeat &&
                (s->writer->lastWrite + Modes.net_heartbeat_interval) <= now) {
                s->writer->send_heartbeat(s);
            }
        }
    }

    // If we have data that has been waiting to be written for a while,
    // write it now.
    for (s = Modes.services; s; s = s->next) {
        if (s->writer &&
            s->writer->dataUsed &&
            (need_flush || (s->writer->lastWrite + Modes.net_output_flush_interval) <= now)) {
            flushWrites(s->writer);
        }
    }

    // Unlink and free closed clients
    for (prev = &Modes.clients, c = *prev; c; c = *prev) {
        if (c->fd == -1) {
            // ADSBx reconnection now handled by feeder thread
            // Recently closed, prune from list
            *prev = c->next;
            delete c;
        } else {
            prev = &c->next;
        }
    }
}

//
// =============================== Network IO ===========================
//
