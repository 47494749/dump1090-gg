# dump1090-gg-light

**dump1090-gg-light** is an all-in-one ADS-B / Mode S / FLARM / OGNTP / ACARS / VDL2 /
Radiosonde / CPDLC receiver and multi-feed relay for Linux.
It is a fork of [dump1090-fa](https://github.com/flightaware/dump1090) by FlightAware,
extended with native threaded feeder clients, a multi-SDR receiver architecture,
decoders for aeronautical communication and weather sounding signals, and a built-in
web control panel with version display.

This is the **light version** for public distribution. The proprietary feeder
protocols (Flightradar24, PlaneFinder, RadarBox) have been replaced with inert
compatibility stubs (the source files still exist so that legacy CLI options
do not break the build, but they contain no functional code) because their
terms of service regarding third-party client implementations are ambiguous or
unclear — to avoid any potential legal issues, the working implementations are
not included. All open-source feeders and 11 Beast-binary tracking networks are
fully functional.

## About this project — AI as a developer

This project is an **experiment to test the effectiveness of AI as a software
developer**. The repository contains substantial upstream code from **dump1090-fa**
and its predecessors (by Oliver Jowett / FlightAware), which retains its original
authorship and copyright. All of the **new functionality** — the native feeder
clients, multi-SDR architecture, FLARM / OGNTP / ACARS / VDL2 / Radiosonde / CPDLC
decoders, the web control panel, Makefile modifications, protocol implementations,
the information gathering, and this README itself — was **written entirely by AI**,
under the **continuous supervision of a human** who directed what to do, how to do
it, which design decisions to make, and which approaches to take.

**This is not "vibe coding".** The human operator is an **IT security
specialist** with deep software development expertise. Every design choice, protocol behavior, and implementation
detail was verified against that hands-on technical knowledge — the AI was
used as a precision tool to write code faster, not as a substitute for
engineering judgment.

The human operator provided:
- The overall architecture and all major design decisions
- Specific technical directions and implementation strategies
- Debugging decisions and root-cause analysis guidance
- Code review and validation of every change before deployment
- Continuous oversight — the AI's output had to be checked constantly,
  because it would silently change architecture or approach without warning
- Simpler, more logical solutions when the AI over-complicated things —
  in many cases the human had to suggest the straightforward approach
  that the AI failed to see on its own
- Firm direction when the AI drifted — at times the AI had to be explicitly
  forced to follow the given directives instead of inventing its own plan

The AI performed:
- All code writing (C, Python helper scripts, Makefile edits)
- Protocol research and information gathering from open-source references
- File transfers, remote compilation, and deployment to the target device
- Diagnostic reading of logs, journal output
- Writing this documentation

### Conclusion

**AI is an excellent tool for writing and modifying code quickly** — it can
produce working C implementations of complex protocols, handle build systems,
diagnose compiler errors, and iterate rapidly. However, **AI is not capable of
making correct autonomous decisions**. Without human direction, it tends to:
- Over-engineer solutions or add unnecessary abstractions
- Make wrong assumptions about protocol behavior
- Miss subtle issues
- Fail to prioritize or choose the right approach among alternatives
- **Get the simplest architectures wrong** — even straightforward designs
  that any developer would get right on the first try
- **Silently change architecture** without notice or justification, requiring
  the human to catch the deviation and force a rollback

The human-AI collaboration model — where the human makes architectural and
strategic decisions while the AI handles the mechanical work of writing,
compiling, and deploying code — proved highly effective for this project.

---

## What's different from dump1090-fa

| Feature | dump1090-fa (upstream) | dump1090-gg-light |
|---|---|---|
| ADS-B decoding | ✅ | ✅ (enhanced Comm-B/TIS-B/ELM/CPDLC) |
| PiAware / FlightAware feed | external `piaware` + `faup1090` | **native thread** (TLS, ADEPT protocol) |
| FlightAware MLAT | external `fa-mlat-client` (Python) | **native thread** (UDP binary protocol) |
| ADSBexchange MLAT | external `mlat-client` (Python) | **native thread** (JSON-over-TCP) |
| OpenSky Network feed | external daemon | **native thread** (binary protocol) |
| Beast feeds (11 networks) | not supported | **native thread** (ADSBx, adsb.fi, FlyItaly, adsb.one, adsb.lol, airplanes.live, Planespotters, TheAirTraffic, AVDelphi, PlaneWatch, ADSBHub) |
| SondeHub upload | not supported | **native thread** (HTTPS PUT to SondeHub v2 API) |
| FLARM / OGNTP / OGN feed | not supported | **native** (2nd RTL-SDR, GFSK demod, LDPC, OGN APRS-IS) |
| ACARS decoding | not supported | **native** (AM envelope + MSK demod, 5 channels) |
| VDL2 decoding | not supported | **native** (D8PSK demod, AVLC parsing, ACARS extraction) |
| Radiosonde decoding | not supported | **native** (RS41/RS92/DFM/M10, RS ECC, GPS, PTU) |
| CPDLC decoding | not supported | **native** (FANS-1/A ASN.1 UPER via ELM) |
| ELM reassembly (DF24–31) | not supported | **native** (ACARS/CPDLC extraction from Comm-D) |
| Multi-SDR management | single dongle | **dynamic role assignment** (up to 8 RTL-SDR) |
| Web control panel | not supported | **native** (HTTP REST API, live configuration) |

All feeder threads use **lock-free SPSC ring buffers** to receive data from the main
decode loop, so slow or failing network I/O on any single feed cannot block decoding.

![Web control panel — Status page showing all active feeders and their connection state](docs/images/status.png)

### Architecture overview

```
RTL-SDR #1 (1090 MHz) ──► ADS-B decode ──┬──► PiAware thread   → piaware.flightaware.com:1200 (TLS)
                           DF24 → ELM    │    FA-MLAT thread   → FlightAware MLAT server (UDP)
                           ELM → CPDLC   │    ADSBx MLAT       → mlat.adsbexchange.com (JSON-TCP)
                                          ├──► OpenSky thread   → collector.opensky-network.org:10004
                                          ├──► Beast feed thread → 11 networks simultaneously
                                          │     ADSBx, adsb.fi, FlyItaly, PlaneWatch, adsb.one,
                                          │     adsb.lol, airplanes.live, Planespotters,
                                          │     TheAirTraffic, AVDelphi, ADSBHub
                                          └──► JSON files       → /run/dump1090-gg/

RTL-SDR #2 (868 MHz)  ──► FLARM demod ───┬──► OGN thread       → aprs.glidernet.org:14580 (APRS-IS)
                          OGNTP demod ────┤
                                          └──► Synthetic DF18   → ADS-B pipeline (merged)

RTL-SDR #3 (131 MHz)  ──► ACARS demod ───┬──► Message display  (5 EU channels, AM-MSK)
                          VDL2 demod  ────┤    (3 EU channels, D8PSK 10.5 ksym/s)
                                          └──► aircraft.json integration

RTL-SDR #4 (403 MHz)  ──► Sonde demod ───┬──► SondeHub thread  → api.v2.sondehub.org (HTTPS PUT)
                          (RS41/RS92/    │
                           DFM/M10)      └──► Message display  (FFT freq scan, RS ECC)

Web control panel (port 8888) ──► Live config, status, logs, device management
```

---

## Building

### Prerequisites

```bash
sudo apt-get install build-essential librtlsdr-dev pkg-config libncurses5-dev \
    libbladerf-dev libhackrf-dev liblimesuite-dev libsoapysdr-dev \
    libssl-dev zlib1g-dev
```

### Compile

```bash
make clean
make -j4 DUMP1090_VERSION=custom-threads
```

`DUMP1090_VERSION` sets the version string embedded in the binary (shown in
`--help` output and the SkyAware web interface). Any value can be used; omit it
to default to the git tag.

This produces three binaries: `dump1090`, `view1090`, `starch-benchmark`.

### Install

```bash
sudo cp dump1090 /usr/bin/dump1090-gg
sudo chmod 755 /usr/bin/dump1090-gg
```

---

## Configuration

The configuration file is `/etc/default/dump1090-gg` (shell-script fragment,
sourced by the start script). The systemd service is `dump1090-gg.service`.
Live configuration is also possible via the web control panel.

### Command-line options

#### SDR and decoding

| Option | Description |
|---|---|
| `--device-type <type>` | SDR type: `rtlsdr`, `bladerf`, `hackrf`, `limesdr`, `ifile` |
| `--gain <dB>` | Tuner gain (default: max). `-10` for AGC |
| `--freq <Hz>` | Override frequency (default: 1090 MHz) |
| `--fix` | Enable single-bit CRC error correction |
| `--enable-df24` | Enable DF24–31 Comm-D ELM decoding (required for CPDLC) |
| `--receiver <spec>` | Add SDR receiver: `serial:role[:gain=X][:ppm=Y][:agc]` |
| | Role: `adsb`, `flarm`, `acars`, `vdl2`, `radiosonde` |

#### FLARM / OGN

| Option | Description |
|---|---|
| `--flarm` | Enable FLARM 868 MHz decoder |
| `--flarm-device <serial>` | RTL-SDR serial for 868 MHz dongle |
| `--flarm-gain <dB>` | FLARM dongle gain (0 = auto) |
| `--flarm-ppm <ppm>` | Frequency correction in PPM |
| `--flarm-keys <file>` | Load FLARM decryption keys from file |
| `--ogn-station <name>` | OGN station callsign for APRS-IS feed |
| `--ogn-server <host>` | OGN APRS-IS server (default: aprs.glidernet.org) |
| `--ogn-port <port>` | OGN APRS-IS port (default: 14580) |

#### Feeder clients

| Option | Description |
|---|---|
| `--piaware` | Enable built-in PiAware ADEPT client |
| `--piaware-feeder-id <uuid>` | FlightAware feeder ID (no external piaware needed) |
| `--opensky` | Enable OpenSky Network feed |
| `--opensky-user <name>` | OpenSky username |
| `--opensky-serial <file>` | OpenSky serial number file |
| `--sondehub <callsign>` | Enable SondeHub upload with this callsign |

#### Beast feed networks (11 supported)

Each network can be enabled individually. Host and port are overridable.

| Option | Default destination |
|---|---|
| `--adsbx` | feed.adsbexchange.com:30005 |
| `--adsbfi` | feed.adsb.fi:30004 |
| `--flyitaly` | dati.flyitalyadsb.com:4905 |
| `--planewatch` | atc.plane.watch:30004 |
| `--adsbone` | feed.adsb.one:64004 |
| `--adsblol` | feed.adsb.lol:30004 |
| `--airplaneslive` | feed.airplanes.live:30004 |
| `--planespotters` | feed.planespotters.net:30004 |
| `--theairtraffic` | feed.theairtraffic.com:30004 |
| `--avdelphi` | data.avdelphi.com:24999 |
| `--adsbhub` | data.adsbhub.org:5001 |

#### Web control panel

| Option | Description |
|---|---|
| `--panel` | Enable the web control panel |
| `--panel-port <port>` | HTTP port (default: 8888) |
| `--panel-password <pw>` | Require Basic Auth (user: `admin`) |
| `--panel-html-dir <dir>` | Custom HTML directory |

These options can also be set via `EXTRA_OPTIONS` in `/etc/default/dump1090-gg`.

---

## Technical details of added features

### ADS-B / Mode S core decoder enhancements

These changes extend the **core decoder pipeline** (Mode S message parsing,
Comm-B register decoding, aircraft tracking, and JSON output) beyond what
stock dump1090-fa implements.

#### New Comm-B (BDS) register decoders

Stock dump1090-fa decodes BDS 1,0 / 1,7 / 2,0 / 3,0 / 4,0 / 4,4 / 5,0 / 6,0.
dump1090-gg adds four additional registers:

| Register | Name | Decoded fields |
|---|---|---|
| **BDS 4,5** | Meteorological Hazard Report (MHAR) | Turbulence, windshear, microburst, icing, wake vortex severity (NIL/LIGHT/MODERATE/SEVERE), static air temperature, average static pressure, relative humidity |
| **BDS 4,1** | Next Waypoint Identifier | 8-character AIS-encoded waypoint name |
| **BDS 4,2** | Next Waypoint Position | Latitude/longitude (11-bit, ~0.088° resolution), altitude (10 ft resolution) |
| **BDS 4,3** | Next Waypoint Crossing Info | Crossing altitude, crossing speed |

#### Enhanced ACAS RA extraction

- **BDS 3,0**: Stock only recognizes the register. dump1090-gg extracts all
  individual fields: ARA (14-bit Active Resolution Advisory), RAC (4-bit RA
  Complement), RAT (RA Terminated), MTE (Multiple Threat Encounter), TTI
  (2-bit Threat Type Indicator), Threat Identity (26-bit).
- **DF16 MV field**: The long air-air surveillance reply MV field is parsed
  for the same ACAS RA fields when the aircraft is airborne.

#### New downlink formats

| DF | Description | Notes |
|---|---|---|
| **DF19** | Military Extended Squitter | Decoded identically to DF17. Stock dump1090-fa ignores it. |
| **DF24–31** | Comm-D Extended Length Message (ELM) | Gated by `--enable-df24`. Feeds the ELM reassembly system. |

#### ELM (Comm-D) reassembly (`elm.c/.h`)

Entirely new subsystem for DF24–31 messages:
- Per-aircraft hash table for segment reassembly (up to 16 segments ×
  10 bytes = 160 bytes per complete message)
- Bitmask-based segment tracking with 60-second TTL for incomplete messages
- Separate decode thread with producer/consumer queue
- **ACARS framing detection**: SOH/STX/ETX markers, mode character,
  aircraft registration, label, and message body extraction
- **CPDLC detection and handoff**: recognized ELM payloads are forwarded
  to the CPDLC decoder for full FANS-1/A message decoding
- Fallback hex+ASCII dump for unrecognized payloads

#### CPDLC decoder (`cpdlc_decode.c/.h`)

Full **FANS-1/A CPDLC** (Controller-Pilot Data Link Communications) decoder.
Decodes CPDLC messages carried in Comm-D ELM frames (DF24–31) using
**ASN.1 UPER** (Unaligned Packed Encoding Rules).

- **Complete message tables**: 129 downlink messages (DM0–DM128) and
  183 uplink messages (UM0–UM182) per ICAO Doc 9705 / RTCA DO-258A
- **Direction-aware decoding**: `cpdlc_try_decode_dir()` API disambiguates
  between uplink (UM, controller→pilot) and downlink (DM, pilot→controller)
  messages that share the same message reference numbers
- **41 parameter types** fully implemented:
  - **Primitives**: altitude (8 forms: FL, feet/meters QNH/QFE, GNSS, metric FL),
    speed (8 forms: IAS/TAS/GS in kt/kmh, Mach), time (HH:MM),
    frequency (HF kHz / VHF MHz / UHF MHz / freetext), degrees (magnetic/true),
    vertical rate, altimeter setting (inHg/hPa), beacon code (squawk)
  - **Position**: fix name, navaid, airport, lat/lon, place+bearing,
    place+bearing+distance — all with PER constraint decoding
  - **Complex structures**: route clearance (up to 128 waypoints with
    level/speed constraints), hold at waypoint (with speed/direction/time/distance),
    position report (19 optional fields including winds, turbulence, icing),
    procedure name (SID/STAR/approach/departure/arrival), unit name + frequency,
    fuel quantity + persons on board, offset (left/right + distance)
  - **Compound types**: altitude+speed, altitude+time, position+altitude,
    position+speed, position+time, position+altitude+speed,
    time+position+altitude, position+position+altitude, and 20+ other
    multi-field combinations
- **Self-contained implementation**: hand-rolled ASN.1 UPER bitstream reader,
  no external ASN.1 library dependency. PER constraints extracted from
  libacars/asn1 generated specifications
- **57 test vectors** (`test_cpdlc.c`): hand-crafted UPER bit-level encoder
  exercising all parameter types, both DM and UM directions

**Standards:** ICAO Doc 9705 (FANS-1/A), RTCA DO-258A, EUROCAE ED-100A,
ASN.1 UPER (ITU-T X.691).

#### Coarse TIS-B decoder (DF18 CF=3)

Stock dump1090-fa has `case 3: // TODO: decode me!` and returns early.
dump1090-gg implements `decodeCoarseTISB()` which decodes altitude (12-bit AC
code), CPR position (12-bit lat/lon scaled to 17-bit for the standard CPR
decoder), ground speed (8-bit), and ground track (7-bit, 360/128° resolution).

Reference: DO-260B §2.2.4.4 (TIS-B Coarse Position).

#### Derived meteorological calculations

| Calculation | Method |
|---|---|
| **Wind speed and direction** | Wind triangle from TAS, heading, GS, and track within a 2.5-second window. Requires BDS 5,0 (TAS, track angle, GS) and BDS 6,0 (magnetic heading) or true heading. Sanity-capped at 250 kt. |
| **OAT / TAT** | Derived from TAS and Mach number: $T_{OAT} = 288.15 \cdot (TAS / (661.47 \cdot M))^2 - 273.15$. Requires Mach ≥ 0.395, TAS ≥ 100 kt. |

#### Magnetic declination model

Tilted dipole approximation using WMM2020-epoch Gauss coefficients
($g_1^0 = -29404.8$, $g_1^1 = -1450.9$, $h_1^1 = 4652.5$ nT). Converts
magnetic heading to true heading for derived wind calculation. Recomputed
every 30 seconds per aircraft.

#### Calculated track from positions

Ground track independently computed from successive CPR-decoded positions
(minimum 100 m apart) using great-circle bearing. Provides a cross-check
against the ADS-B reported track angle.

#### Squawk debounce

New squawk values must be received **twice, at least 250 ms apart**, before
being accepted. Prevents spurious squawk changes from single-bit errors in
Mode A replies.

#### Full Operational Status extraction (TC31)

Stock dump1090-fa decodes TC31 internally for version/accuracy bits but does
not expose the Operational Mode (OM) and Capability Class (CC) bitfields.
dump1090-gg tracks and outputs 16 individual fields:
- **OM**: ACAS RA active, IDENT active, ATC services, SAF (single antenna)
- **CC**: ACAS operational, CDTI, 1090ES-IN, ARV, TS (target state), TC
  (target change), UAT-IN, POA (position offset applied), B2-low, L/W code,
  antenna offset

#### New `aircraft.json` output fields

All new data is exposed via the standard `aircraft.json` HTTP endpoint:

| Field(s) | Source |
|---|---|
| `mhar_turbulence`, `mhar_windshear`, `mhar_microburst`, `mhar_icing`, `mhar_wake`, `mhar_temperature`, `mhar_pressure`, `mhar_humidity` | BDS 4,5 |
| `waypoint`, `waypoint_lat`, `waypoint_lon`, `waypoint_alt` | BDS 4,1 / 4,2 |
| `waypoint_crossing_alt`, `waypoint_crossing_speed` | BDS 4,3 |
| `acas_ra` (object: ara, rac, rat, mte, tti, threat) | BDS 3,0 / DF16 / TC28 |
| `opstatus` (object: 16 OM/CC fields) | TC31 |
| `wd_speed`, `wd_dir`, `wd_alt` | Derived wind |
| `oat`, `tat` | Derived OAT/TAT |
| `calc_track` | Position-derived track |
| `mag_dec` | Magnetic declination |
| `alt_reliable` | Altitude reliability score |

---

### Lock-free feeder architecture (`feeder_thread.c/.h`)

All feeder clients run in independent pthreads, decoupled from the main decode
loop via **SPSC (Single-Producer Single-Consumer) ring buffers** with 4096
entries. The producer (main decode thread) and consumers (feeder threads) use
C11 `stdatomic` operations (`memory_order_acquire` / `memory_order_release`)
for lock-free synchronization. A `pthread_rwlock_t` protects the aircraft list
for feeders that need to iterate it.

**Internet connectivity watchdog**: the beast feed thread periodically probes
DNS resolution and sets an atomic `net_available` flag. All feeder threads
(OGN, OpenSky, SondeHub, etc.) pause automatically when the network is down
and resume when connectivity returns.

### PiAware / ADEPT client (`piaware_client.c/.h`)

Native TLS client implementing FlightAware's **ADEPT** (Aviation Data Exchange
Protocol). Replaces the external `piaware` daemon and `faup1090` helper.

- Connects to `piaware.flightaware.com:1200` over TLS (OpenSSL)
- Authenticates with feeder ID (UUID) and MAC address. The feeder ID can be
  provided directly via `--piaware-feeder-id <uuid>` without needing piaware
  installed, or read from a file via `--piaware-feeder-id-file <path>`
  (default: `/var/cache/piaware/feeder_id`)
- Uploads aircraft data in **FATSV** (FlightAware Tab-Separated Values) format,
  including: position, barometric and geometric altitude, ground speed, IAS, TAS,
  Mach, track, squawk, callsign, emitter category, emergency status, NACp, NACv,
  SIL, SDA, and full BDS register data
- Health reports every 5 minutes, FATSV updates every 1 second
- Handles MLAT coordination commands (`mlat_wanted`, `mlat_unwanted`, `mlat_result`)
  from the FlightAware server

**Standards:** ADS-B 1090ES (DO-260B DF17/18), Mode S (DF11), TIS-B, ADS-R, Mode A/C.

### FlightAware MLAT client (`fa_mlat.c/.h`)

Native UDP client implementing FlightAware's binary MLAT protocol. Replaces the
external `fa-mlat-client` (Python).

- Custom UDP binary protocol with big-endian packed submessages:
  `TYPE_SYNC` (even/odd CPR pair), `TYPE_MLAT_SHORT` (7-byte Mode S),
  `TYPE_MLAT_LONG` (14-byte Mode S), `TYPE_REBASE` (timestamp reset),
  `TYPE_MLAT_MODEAC` (Mode A/C)
- Per-aircraft hash table (2048 slots, open addressing) tracking message
  counts, timestamps, and CPR pairs for sync generation
- 12 MHz Beast clock timestamps
- Controlled dynamically by the PiAware thread based on server commands

**Standards:** Multilateration using ADS-B 1090ES CPR position pairs and Mode S timestamps.

### ADSBexchange feeder & MLAT (`feeder_thread.c`, `mlat_client.c/.h`)

Dual-function ADSBexchange integration:

1. **Beast binary feed** to `feed.adsbexchange.com:30005` — standard Beast
   binary encoding of raw Mode S messages with 12 MHz timestamps
2. **JSON-over-TCP MLAT** — newline-delimited JSON protocol to up to 4 MLAT
   servers simultaneously, with handshake, sync pairs, rate reports,
   and MLAT result injection back into the main aircraft list

MLAT results are re-encoded as synthetic DF17 Beast frames (CPR-encoded
position + velocity) and injected into the main decode pipeline.

**Standards:** Beast binary protocol, ADS-B 1090ES (DF17 CPR), MLAT.

### Beast feed network support (`feeder_thread.c`)

A unified Beast binary feed system supporting **up to 16 concurrent feed
destinations**. Each feed runs within a single multiplexed thread with
independent reconnect, heartbeat, and statistics.

11 tracking networks are preconfigured:

| Network | Default endpoint |
|---|---|
| ADSBexchange | feed.adsbexchange.com:30005 |
| adsb.fi | feed.adsb.fi:30004 |
| Fly Italy ADSB | dati.flyitalyadsb.com:4905 |
| plane.watch | atc.plane.watch:30004 |
| adsb.one | feed.adsb.one:64004 |
| adsb.lol | feed.adsb.lol:30004 |
| airplanes.live | feed.airplanes.live:30004 |
| Planespotters | feed.planespotters.net:30004 |
| TheAirTraffic | feed.theairtraffic.com:30004 |
| AVDelphi | data.avdelphi.com:24999 |
| ADSBHub | data.adsbhub.org:5001 |

Each network can be individually enabled/disabled via CLI flags or the web
control panel. Host and port are overridable per network. ADSBHub supports
a dynamic IP registration protocol (`--adsbhub-ckey`) for NAT traversal.

Three feed formats are supported: **Beast binary** (default), **Raw hex**,
and **SBS (BaseStation)**.

### OpenSky Network client (`opensky_client.c/.h`)

Native client implementing the OpenSky Network's **binary upload protocol**,
reimplemented by reading the open-source
[opensky-sensor](https://github.com/openskynetwork/opensky-sensor) daemon
(v2.1.7), published by OpenSky Network under the **BSD 3-Clause** license.
The OpenSky [Terms of Use §7](https://opensky-network.org/about/terms-of-use)
allow voluntary data feeding and only prohibit sending modified or fabricated
data — there is no requirement to use OpenSky's own feeder software.

- Connects to `collector.opensky-network.org:10004`
- Custom framing: `0x1A` escape byte + type byte, with TLV responses
- Login sequence: device type/version → serial number negotiation →
  GPS position (IEEE 754 doubles, big-endian) → username → Beast frames
- Serial number persistence: auto-assigned by server on first connect,
  saved to file for subsequent sessions
- Configurable via `--opensky`, `--opensky-user`, `--opensky-serial`,
  `--opensky-host`, `--opensky-port`

### FLARM receiver (`flarm_reader.c/.h`, `flarm_demod.c/.h`, `flarm_decode.c/.h`)

Complete FLARM reception chain using a **second RTL-SDR dongle** at 868.3 MHz:

1. **RTL-SDR reader** (`flarm_reader.c`): Async IQ capture at 1.6 MSPS,
   identified by serial number
2. **GFSK demodulator** (`flarm_demod.c`): FM discriminator → DC-blocking IIR
   filter (α=0.9999) → bit slicer → Manchester decoding → 64-bit syncword
   correlation (Hamming distance ≤ 4) → packet assembly
3. **Protocol decoder** (`flarm_decode.c`): CRC-16 CCITT verification →
   XXTEA decryption (v6 and v7 key schedules) → coordinate decoding
   (relative to receiver position) → sanity checks (≤300 km, altitude
   −500 m to 15000 m, speed ≤500 m/s)
4. **DF18 synthesis**: Decoded FLARM positions are re-encoded as **DF18 CF=5
   (Fine TIS-B, non-ICAO)** messages with CPR position, AC12 altitude, and
   velocity components, then injected into the main Mode S pipeline

This allows FLARM aircraft to appear in `aircraft.json` and be shared with
all feeder networks alongside ADS-B traffic.

**Standards:** FLARM Legacy v6/v7 (868 MHz ISM band), DF18 TIS-B (DO-260B §2.2.4.4).

### OGNTP receiver (`ogntp_decode.c/.h`)

**OGN Tracking Protocol** decoder, running in parallel with the FLARM decoder
on the same 868 MHz RTL-SDR dongle.

- **LDPC(208,160) parity check** — 48 parity bits validated to ensure data
  integrity before position extraction
- **8-byte syncword detection**: `0xAA 0x66 0x55 0xA5 0x96 0x99 0x96 0x5A`
  (Hamming distance ≤ 4)
- **Parallel demodulation**: OGNTP sync detection runs alongside FLARM in the
  same `process_bit()` pipeline — the first sync match determines the protocol
- **Position decoding**: latitude, longitude (scaled integers), altitude,
  aircraft type, address type
- **DF18 synthesis**: decoded OGNTP positions are injected into the main
  pipeline as synthetic DF18 frames, identical to FLARM integration
- **Callsign prefix**: `OGN` + 5-hex-digit address (vs `FLR` for FLARM)
- **Lock-free SPSC queue**: separate from FLARM queue, drained in
  `flarmReaderPeriodicWork()`

**Standards:** OGN Tracking Protocol (community-documented), LDPC(208,160).

### OGN / APRS-IS client (`ogn_client.c/.h`)

Submits decoded FLARM positions to the Open Glider Network.

- APRS-IS TCP connection to `aprs.glidernet.org:14580`
- APRS position reports with aircraft type symbols (glider, helicopter,
  balloon, etc.), climb rate, and address prefix (ICAO/FLARM/OGN)
- Station beacon every 5 minutes with CPU temperature
  (`/sys/class/thermal/thermal_zone0/temp`), CPU load (`/proc/loadavg`),
  RAM usage (`/proc/meminfo`), NTP offset (`adjtimex(2)` `time.tv_usec`),
  and unique aircraft count
- 128-entry message queue fed by FLARM decoder callback

**Standards:** APRS-IS protocol, OGN position report format.

---

### Multi-SDR receiver architecture (`sdr_receiver.c/.h`)

![Web panel — SDR device management with role assignment per dongle](docs/images/devices.png)

Dynamic assignment of multiple RTL-SDR dongles to different decoder roles.
Each receiver is identified by its USB serial number and configured via
`--receiver <serial>:<role>` where role is one of `adsb`, `flarm`, `acars`,
`vdl2`, or `radiosonde`.

- Supports up to **8 simultaneous RTL-SDR devices**
- Each receiver runs its own async IQ reader thread
- Role can be changed via the web configuration panel dropdown
- Automatic gain control and sample rate configuration per role
- Clean start/stop lifecycle for hot role switching
- Receiver configuration persisted to `/etc/dump1090-gg/receivers.json`

### ACARS decoder (`acars_demod.c/.h`)

Multi-channel **ACARS** (Aircraft Communications Addressing and Reporting System)
demodulator operating on VHF frequencies around 131 MHz.

- **AM envelope detection** for ACARS amplitude modulation
- **MSK (Minimum Shift Keying) demodulation** at 2400 baud
- **Multi-channel reception**: simultaneous monitoring of up to 5 European
  ACARS frequencies (131.525, 131.550, 131.725, 131.825, 131.850 MHz)
- **Odd parity** bit checking per character
- **CRC-16** frame validation
- Message parsing: mode character, aircraft registration, label field,
  block ID, sequence number, and message body
- Pre-key and sync-word detection for burst acquisition

**Standards:** ARINC 618 (ACARS character encoding and framing), ARINC 620
(ACARS media access), ITU-R M.1234 (VHF datalink).

### VDL2 decoder (`vdl2_demod.c/.h`)

Multi-channel **VDL Mode 2** (VHF Digital Link Mode 2) demodulator — the
digital successor to ACARS, carrying ACARS, CPDLC, ADS-C, and ATIS messages
over a higher-bandwidth D8PSK link.

- **D8PSK demodulation** (Differential 8-Phase Shift Keying) at **10,500 symbols/s**
  (31,500 bits/s) using Gray-coded constellation {0, 1, 3, 2, 6, 7, 5, 4}
- **16-symbol preamble detection** via differential phase correlation with
  configurable threshold
- **Burst-mode TDMA** state machine: `DM_INIT` → `DM_SYNC` (preamble acquired)
  → frame processing → back to scanning
- **Gardner timing recovery** for symbol clock synchronization
- **AGC** (Automatic Gain Control) with noise floor tracking for adaptive
  squelch threshold
- **Multi-channel reception**: 3 European VDL2 frequencies
  (136.725, 136.775, 136.875 MHz) monitored simultaneously
- **AFC** (Automatic Frequency Correction) — frequency offset estimated from
  preamble phase linear regression, compensated during burst demodulation
- **FCS (Frame Check Sequence)** — CRC-16 with polynomial 0x1021 (bit-reversed),
  **1-bit error correction** via precomputed syndrome table (80 unique syndromes
  for frames up to 5 bytes header)
- **Bit de-stuffing** per HDLC rules (zero removed after 5 consecutive ones)
- **Squelch** — frames below noise floor + threshold are rejected
- **2-pole Chebyshev Type I IIR low-pass filter** for channel isolation
- **Full AVLC frame parsing**:
  - Source and destination address extraction (24-bit ICAO + type/SSR/C/R/SAPI)
  - All 12 frame types: I, RR, RNR, REJ, SABM, DISC, DM, UA, UI, XID, TEST, FRMR
  - **ACARS-over-AVLC extraction**: automatic detection and parsing of ACARS
    payloads within AVLC information frames

**Standards:** ICAO Doc 9776 (VDL Mode 2 Technical Manual), ICAO Annex 10 Vol III,
ARINC 618 (ACARS over AVLC), ISO/IEC 13239 (HDLC/AVLC framing).

### Radiosonde decoder (`sonde_demod.c/.h`)

Multi-protocol radiosonde decoder supporting **RS41, RS92, DFM, and M10**
sondes on 403 MHz.

#### Vaisala RS41
- **PLL bit clock recovery** for GFSK demodulation
- **FFT frequency scanning** to locate the signal within receiver bandwidth
- **Reed-Solomon RS(255,231) error correction** — corrects up to 12 symbol
  errors; hard fail on uncorrectable frames
- **XOR whitening** removal (64-byte mask)
- **CRC-16 CCITT** validation on all data sub-blocks (always enforced)
- **GPS decoding**: ECEF coordinates → WGS84 lat/lon/alt, velocity vector
- **PTU**: calibrated temperature and humidity from per-sonde coefficients
- **Serial number** required before emitting position (`serial[0] != '\0'`)

#### Vaisala RS92
- **Reed-Solomon RS(255,231)** with hard fail on uncorrectable frames
- **GPS decoding**: ECEF → WGS84
- **Serial number** required before emitting position

#### Graw DFM (DFM06/DFM09)
- **Manchester decoding**: 66 raw bytes → 33 decoded bytes
- **Nibble XOR parity** validation per subframe (2-byte header + 6×5-byte subframes)
- **GPS decoding**: lat/lon/alt from decoded subframes
- **Serial number** required before emitting position

#### Meteomodem M10
- **CRC-16 CCITT** over bytes 0..98, stored little-endian at bytes 99..100
- **Frame type byte** must be `0x9A` (M10 GPS frame)
- **Sync threshold** 22/24 bits (raised from 20 to reduce false positives)
- **GPS decoding**: lat/lon/alt, velocity (cm/s → m/s)
- **101-byte frame** length

**Standards:** Vaisala RS41/RS92 telemetry format (community-documented by
[rs1729](https://github.com/rs1729/RS)), Graw DFM (community RE), Meteomodem
M10 (community RE), WGS84 geodetic reference system.

### SondeHub client (`sondehub_client.c/.h`)

Uploads decoded radiosonde telemetry to the **SondeHub v2 API**, the global
collaborative radiosonde tracking platform.

- **HTTPS PUT** to `api.v2.sondehub.org` using OpenSSL TLS (no libcurl dependency)
- **Two endpoints**:
  - `/sondes/telemetry` — batched telemetry upload every 30 seconds (JSON array)
  - `/listeners` — station position upload every 10 minutes (JSON object)
- **Mandatory telemetry fields**: software name/version, uploader callsign,
  time received (ISO 8601), manufacturer, type ("RS41", "RS92", "DFM", "M10"),
  serial, frame number, datetime, lat/lon/alt
- **Optional fields**: horizontal/vertical velocity, heading, satellite count,
  frequency, SNR, temperature, humidity, uploader position
- **64-entry ring buffer** for non-blocking telemetry submission from the
  sonde decoder callback
- **Exponential backoff** on failure (30s → 60s → 120s → ... up to 10 min)
- **Only active** when `--sondehub <callsign>` is specified and a radiosonde
  receiver is producing valid GPS positions
- Enabled via `--sondehub <callsign>` CLI option
- Upload statistics reported at shutdown

**Standards:** SondeHub v2 API (https://github.com/projecthorus/sondehub-infra/wiki).

---

### Web control panel (`config_panel.c/.h`)

Built-in HTTP server providing a REST API for live configuration, monitoring,
and management. Replaces the need for manual config file editing and service
restarts for most settings.

- **HTTP server** on configurable port (default 8888)
- **Optional Basic Auth** (username: `admin`, configurable password)
- **REST API endpoints**:
  | Endpoint | Method | Description |
  |---|---|---|
  | `/api/config` | GET | Full configuration dump |
  | `/api/config` | POST | Live configuration update (no restart needed) |
  | `/api/status` | GET | System status, uptime, version, variant |
  | `/api/aircraft` | GET | Live aircraft table |
  | `/api/connections` | GET | Feeder connection status (all networks) |
  | `/api/stats` | GET | Decode and feed statistics |
  | `/api/logs` | GET | Ring-buffered log viewer (2000 lines) |
  | `/api/messages` | GET | Decoded message ring buffer (500 messages) |
  | `/api/devices` | GET | SDR device enumeration and tuner info |
- **Configurable items** (via POST, applied at runtime):
  - Beast feed networks: enable/disable per network
  - OpenSky Network: enable/disable, username
  - PiAware: enable/disable, feeder ID
  - ADSBHub: enable/disable, ckey
  - SDR receiver roles and gain
  - Station position (lat/lon)
- **Security**: path traversal protection (realpath validation + character
  whitelist), shell metacharacter sanitization, JSON input validation,
  request size limits
- **Thread-safe logging**: ring buffer with 2000 entries, accessible via API
- **RTL-SDR device probing**: enumerates all connected dongles with serial
  numbers and tuner types on startup

![Web panel — Live connections overview with per-feeder statistics](docs/images/connections.png)

![Web panel — Real-time aircraft table with squawk decoding and ICAO lookup](docs/images/aircraft.png)

![Web panel — Decoded messages view (ACARS, VDL2, CPDLC, FLARM, Radiosonde)](docs/images/messages.png)

![Web panel — Live log viewer with auto-scroll](docs/images/logs.png)

### Aircraft table features (`panel/aircraft.html`)

The aircraft table page includes several interactive features:

- **Source column**: color-coded badges showing how each aircraft was acquired:
  ADS-B (cyan), FLARM (green), OGNTP (blue), MLAT (purple), TIS-B (orange),
  ADS-R (teal), Mode-S (gray). The source is derived from the `type` field
  in `aircraft.json` (omitted for standard ADS-B ICAO addresses)
- **Military aircraft detection**: heuristic `isMilitary()` function using:
  - ICAO address ranges (US 0xAE-0xAF, UK RAF 0x43C, French FAF 0x3B,
    Australian RAAF 0x7C7, Turkish ThAF 0x71)
  - Military squawk codes (7001–7007)
  - Known military callsign prefixes (SPARO, GRIFONE, FALCO, COBRA, VIPER,
    AWACS, NATO, REAPER, etc.)
  - Owner keywords from ICAO database lookups (air force, navy, army, etc.)
- **GPS integrity flags**: GPS! (suspect) and GPS↓ (degraded) badges
- **Version badge**: all pages display the running program version (fetched
  from `/api/status`) in the top-right corner of the navigation bar

---

## Original code and licenses

### Base project

| Component | Author | License | Source |
|---|---|---|---|
| **dump1090** (original) | Salvatore Sanfilippo (antirez) | BSD 2-Clause | <https://github.com/antirez/dump1090> |
| **dump1090** (Malcolm Robb fork) | Malcolm Robb | BSD 2-Clause | <https://github.com/MalcolmRobb/dump1090> |
| **dump1090-mutability** | Oliver Jowett | GPL-2.0-or-later | <https://github.com/mutability/dump1090> |
| **dump1090-fa** | FlightAware LLC | GPL-2.0-or-later | <https://github.com/flightaware/dump1090> |

The base codebase (ADS-B demodulation, Mode S decoding, network I/O, adaptive gain,
interactive display, SkyAware web interface) is **entirely from dump1090-fa** and is
used unmodified except for the build variant name. All original copyright headers
are preserved.

### New modules added in dump1090-gg

| File(s) | Purpose | Protocol source | License |
|---|---|---|---|
| `piaware_client.c/.h` | Native FlightAware ADEPT client | Open-source [piaware](https://github.com/flightaware/piaware) (BSD) | GPL-3.0-or-later |
| `fa_mlat.c/.h` | Native FlightAware MLAT (UDP binary) | Open-source [fa-mlat-client](https://github.com/mutability/mlat-client) by Oliver Jowett (GPL-3.0+) | GPL-3.0-or-later |
| `mlat_client.c/.h` | ADSBexchange MLAT (JSON-over-TCP) | Open-source [mlat-server](https://github.com/adsbexchange/mlat-server) | GPL-3.0-or-later |
| `flarm_decode.c/.h` | FLARM Legacy protocol decoder (XXTEA) | Open-source [SoftRF](https://github.com/lyusupov/SoftRF) by Linar Yusupov (GPL-3.0), protocol RE by Stanislaw Pusep | GPL-3.0-or-later |
| `flarm_demod.c/.h` | GFSK demodulator for 868 MHz FLARM | Original implementation based on published radio parameters | GPL-3.0-or-later |
| `flarm_reader.c/.h` | Second RTL-SDR reader (868 MHz) | librtlsdr API | GPL-3.0-or-later |
| `ogn_client.c/.h` | OGN APRS-IS feed client | [OGN protocol wiki](http://wiki.glidernet.org/) and public APRS-IS documentation | GPL-3.0-or-later |
| `opensky_client.c/.h` | OpenSky Network native feed client | Open-source [opensky-sensor](https://github.com/openskynetwork/opensky-sensor) v2.1.7 (BSD 3-Clause) | GPL-3.0-or-later |
| `sdr_receiver.c/.h` | Multi-SDR receiver manager | Original implementation | GPL-3.0-or-later |
| `config_panel.c/.h` | Web control panel (HTTP REST) | Original implementation | GPL-3.0-or-later |
| `acars_demod.c/.h` | ACARS AM-MSK demodulator | ARINC 618/620 public standards | GPL-3.0-or-later |
| `vdl2_demod.c/.h` | VDL Mode 2 D8PSK demodulator | ICAO Doc 9776, Annex 10 Vol III | GPL-3.0-or-later |
| `sonde_demod.c/.h` | Multi-protocol radiosonde decoder (RS41/RS92/DFM/M10) | Community-documented formats ([rs1729/RS](https://github.com/rs1729/RS)) | GPL-3.0-or-later |
| `ogntp_decode.c/.h` | OGN Tracking Protocol decoder (LDPC) | Community-documented OGN-TP format | GPL-3.0-or-later |
| `sondehub_client.c/.h` | SondeHub v2 telemetry uploader | [SondeHub API](https://github.com/projecthorus/sondehub-infra/wiki), reference: [radiosonde_auto_rx](https://github.com/projecthorus/radiosonde_auto_rx) | GPL-3.0-or-later |
| `elm.c/.h` | Comm-D ELM reassembly | ICAO Annex 10 Vol IV (Comm-D framing) | GPL-3.0-or-later |
| `cpdlc_decode.c/.h` | FANS-1/A CPDLC message decoder | ICAO Doc 9705, RTCA DO-258A, ASN.1 constraints from [libacars](https://github.com/szpajder/libacars) | GPL-3.0-or-later |
| `feeder_thread.c/.h` | Thread management, lock-free SPSC queues, beast feeds | Original implementation | GPL-3.0-or-later |
| `feeder_thread_stub.c` | Stub for view1090/faup1090 linking | — | GPL-3.0-or-later |

### Linked libraries

| Library | License | Purpose |
|---|---|---|
| librtlsdr | GPL-2.0 | RTL-SDR device access |
| OpenSSL (libssl, libcrypto) | Apache-2.0 | TLS for PiAware, SondeHub, ADSBHub |
| zlib (libz) | zlib license | Gzip compression |
| libncurses | MIT | Interactive terminal display |
| libpthread, libm, librt | glibc (LGPL) | Threading, math, timers |

---

## How protocol information was obtained

> **Important**: All protocol information was obtained from one of
> these sources:
>
> 1. **Open-source reference implementations** (reading published source code)
> 2. **Publicly documented protocols** (APRS-IS, OGN wiki, HTTP standards, ICAO docs)
> 3. **Published aviation standards** (ARINC 618/620, ICAO Annex 10, DO-260B)

### PiAware / ADEPT protocol

The ADEPT (Aviation Data Exchange Protocol) was reimplemented by reading the
**open-source piaware client** published by FlightAware under the BSD license:

- <https://github.com/flightaware/piaware>

The piaware source code documents the complete ADEPT protocol: TLS connection
setup, TSV-based message framing, login handshake, FATSV aircraft data format,
MLAT result forwarding, and health/alive messages. All field names and message
types are taken directly from the published Tcl source.

### FlightAware MLAT (UDP binary)

Reimplemented from the **open-source fa-mlat-client** by Oliver Jowett,
published under GPL-3.0+:

- <https://github.com/mutability/mlat-client>

The Python source files `flightaware/client/adeptclient.py` and
`mlat/client/coordinator.py` fully document the UDP binary protocol: message
types (SYNC, MLAT_SHORT, MLAT_LONG, REBASE), big-endian framing, MTU
constraints, heartbeat interval, and 12-MHz clock encoding.

### ADSBexchange MLAT (JSON-over-TCP)

Reimplemented from the **open-source mlat-server** by ADSBexchange:

- <https://github.com/adsbexchange/mlat-server>

The server's Python source documents the JSON-over-TCP protocol: newline-delimited
JSON messages, handshake fields, sync/MLAT message formats, and clock handling.
The ADSBexchange Beast feed (port 30005) uses the standard, publicly documented
Beast binary protocol.

### OpenSky Network (binary protocol)

Reimplemented by reading the **open-source opensky-sensor** daemon (v2.1.7),
published by OpenSky Network under the BSD 3-Clause license:

- <https://github.com/openskynetwork/opensky-sensor>

The C source code (in `src/`) documents the complete upload protocol:
protocol framing (`0x1A` escape + type byte), login sequence, serial
number negotiation, GPS position upload (IEEE 754 doubles, big-endian),
and TLV server responses. All message types and field layouts were
taken directly from the published source files.

### FLARM protocol

The FLARM radio protocol was implemented using **published open-source work**:

- **Protocol reverse-engineering** by [Stanislaw Pusep](https://github.com/creaktive)
  who published his findings on the FLARM Legacy packet structure
- **SoftRF project** by [Linar Yusupov](https://github.com/lyusupov/SoftRF),
  published under GPL-3.0, which contains a complete open-source implementation
  of FLARM packet encoding/decoding in `Legacy.cpp`

From these sources, the following protocol details were obtained:
- Radio parameters: 868.2/868.4 MHz, 100 kbps GFSK, ±50 kHz deviation
- Manchester-encoded syncword: `0x55 0x99 0xA5 0xA9 0x55 0x66 0x65 0x96`
- Packet structure: 24-byte payload + 2-byte CRC-16 CCITT
- XXTEA encryption keys and key-scheduling algorithm
- Coordinate encoding (latitude/longitude delta from a grid reference)
- Aircraft type, address type, stealth flag, and vertical speed fields

The GFSK demodulator (`flarm_demod.c`) is an **original implementation** based on
standard DSP techniques (FM discriminator, Manchester decoding, syncword
correlation) applied to the published radio parameters.

### OGN / APRS-IS protocol

The OGN (Open Glider Network) protocol is **publicly documented**:

- <http://wiki.glidernet.org/>
- APRS-IS protocol: <http://www.aprs-is.net/Connecting.aspx>

The APRS position format, login procedure, passcode algorithm, station beacon
format, and aircraft symbol table are all part of the public APRS specification.

### CPDLC / FANS-1/A protocol

The CPDLC decoder was implemented using **published aviation standards** and
**open-source reference code**:

- **ICAO Doc 9705** (Manual of Technical Provisions for the Aeronautical
  Telecommunication Network) — defines the FANS-1/A CPDLC message set,
  message reference numbers, and parameter types
- **RTCA DO-258A / EUROCAE ED-100A** — FANS-1/A interoperability standards
  defining the complete DM and UM message tables
- **ASN.1 PER constraints** were extracted from the open-source
  [libacars](https://github.com/szpajder/libacars) project by Tomasz Lemiech,
  which contains ASN.1 module definitions generated from the ICAO specifications

### SondeHub API

The SondeHub uploader was implemented using the **publicly documented API** and
**open-source reference implementation**:

- SondeHub API wiki: <https://github.com/projecthorus/sondehub-infra/wiki>
- Reference implementation: [radiosonde_auto_rx](https://github.com/projecthorus/radiosonde_auto_rx)
  `sondehub.py` by Mark Jessop VK5QI (GPL-3.0)

### ACARS / VDL2 / Radiosonde

These decoders are implemented from **published aviation standards** and
**community-documented formats**:

- **ACARS**: ARINC 618 (character encoding, framing), ARINC 620 (media access)
- **VDL Mode 2**: ICAO Doc 9776 (VDL Mode 2 Technical Manual), ICAO Annex 10
  Volume III (Digital Data Communication Systems), ISO/IEC 13239 (HDLC/AVLC)
- **RS41 Radiosonde**: Community-documented telemetry format, based on
  publicly available reverse-engineering efforts by the radiosonde community
  (notably [rs1729](https://github.com/rs1729/RS))
- **RS92 / DFM / M10**: Community-documented formats from the same
  radiosonde reverse-engineering community
- **OGNTP**: OGN Tracking Protocol, documented on the
  [OGN protocol wiki](http://wiki.glidernet.org/) and in the
  [ogn-decode](https://github.com/glidernet) source code

---

## License

dump1090-gg-light is free software.

The **base codebase** (from dump1090-fa) is licensed under the
**GNU General Public License, version 2 or later** (GPL-2.0-or-later).
It incorporates BSD-licensed code by Salvatore Sanfilippo and Malcolm Robb
(see `LICENSE` file for details).

The **new modules** (feeder threads, FLARM decoder/demodulator, native clients,
ACARS/VDL2/Radiosonde decoders, CPDLC decoder, SondeHub client, web panel) are
licensed under the **GNU General Public License, version 3 or later**
(GPL-3.0-or-later), which is compatible with GPL-2.0-or-later.

The combined work is therefore distributed under **GPL-3.0-or-later**.

### License compatibility

| Original license | Compatible with GPL-3? | Notes |
|---|---|---|
| BSD 2-Clause (antirez, Malcolm Robb) | ✅ Yes | Permissive, compatible with any GPL |
| GPL-2.0-or-later (Oliver Jowett, FlightAware) | ✅ Yes | "or later" allows GPL-3 |
| GPL-3.0 (SoftRF / Linar Yusupov) | ✅ Yes | Same license |
| GPL-3.0 (radiosonde_auto_rx / Mark Jessop) | ✅ Yes | Reference only (API reimplementation) |
| BSD (piaware reference) | ✅ Yes | Permissive |
| BSD 3-Clause (opensky-sensor / OpenSky Network) | ✅ Yes | Permissive; new code is original GPL-3 work, not a derivative |
| Apache-2.0 (OpenSSL) | ✅ Yes | Compatible with GPL-3 (not GPL-2-only) |

Since the upstream dump1090-fa uses "GPL version 2 **or later**", and the new
FLARM code is derived from GPL-3.0 sources (SoftRF), the combined binary is
distributed under GPL-3.0-or-later. This is fully compatible — no license
violation occurs.

**Contact for non-GPL licensing of the original dump1090 base:**
Oliver Jowett `<oliver@mutability.co.uk>` (see `LICENSE` file).

---

## Credits

- **Salvatore Sanfilippo** (antirez) — original dump1090
- **Malcolm Robb** — dump1090 Mode S decoder improvements, view1090
- **Oliver Jowett** — dump1090-mutability, fa-mlat-client
- **FlightAware LLC** — dump1090-fa, adaptive gain, PiAware
- **Stanislaw Pusep** — FLARM protocol reverse-engineering
- **Linar Yusupov** — SoftRF FLARM implementation (GPL-3.0)
- **ADSBexchange** — mlat-server (JSON MLAT protocol reference)
- **Tomasz Lemiech** (szpajder) — libacars (ASN.1 CPDLC specifications)
- **Mark Jessop** (VK5QI) — radiosonde_auto_rx, SondeHub API
- **Project Horus** — SondeHub infrastructure
- **Radiosonde community** — RS41/RS92/DFM/M10 telemetry format documentation
- **rs1729** — Comprehensive radiosonde decoder reference implementations
- **Open Glider Network** — OGN Tracking Protocol documentation
