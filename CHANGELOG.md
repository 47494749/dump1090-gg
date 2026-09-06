# Changelog

All notable changes to this project will be documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/).

Historical entries keep the module and directory names that were current when
they were written. Internal test harnesses mentioned in those notes were used
during development and are only part of the public tree when the files are
actually present here.

---

### v1.0.9 (2026-09-06)

**Multi-decoder stats & charts:**
- Stats history format upgraded to v3 (`SH03`): new fields for OGNTP, ADS-L,
  P3I, and IoT 868 decoded packet counters
- Automatic v2→v3 migration preserves existing history data
- `stats.html` charts now show all active decoders dynamically instead of
  hardcoded ADS-B/FLARM only
- New chart color assignments for OGNTP, ADS-L, P3I, and IoT 868 decoders

**IoT 868 counter API:**
- Added `iotDecoderGetPacketsDecoded()` to expose IoT 868 decoded packet count
- Stats snapshot (`rxGetStatsSnapshot`) now includes IoT 868 counters

**FLARM stats granularity:**
- Stats snapshot now reports OGNTP, ADS-L, and P3I decoded counts separately
  (previously aggregated under FLARM)

**Sharing monitor improvements:**
- New `sharing_count_localhost_connections()` helper to detect stale feeders
  that are running but not connected
- Two-phase monitor: Phase 1 restarts stopped services, Phase 2 restarts
  services that are running but have no localhost connections
- Extracted `sharing_restart_service()` for cleaner restart logic

**Build fix:**
- Moved `gg_format.h` include outside `#ifdef ENABLE_RTLSDR` in
  `sdr_backend.cpp` — fixes SDRGG-only cross-compilation builds

---

### v1.0.8 (2026-07-27)

**Sharing programs page (new):**
- New `/sharing.html` panel page for managing third-party ADS-B feed programs
  (ADSBexchange, adsb.fi, adsb.lol, etc.)
- Backend API (`/api/sharing`) to detect installed feed programs, show their
  status, and control them via systemd
- Navigation bar updated across all panel pages to include the Sharing link

**ELM / Comm-D decoder hardening:**
- ACARS framing validation now enforces ARINC 622 structural rules: explicit
  ETX/ETB terminator required, minimum header length, printable address field
  check — prevents false-positive ACARS detection from random SOH bytes
- ELM RAW output label changed from `ELM` to `ELM RAW` for clarity when
  neither ACARS nor CPDLC decoding succeeds
- KE=1 (uplink acknowledgement) frames now correctly rejected from downlink
  reassembly — these carry TAS data, not downlink segments
- Stale sequence restart: a new segment 0 arriving after a gap now correctly
  starts a fresh message instead of appending to an expired entry
- Completion logic simplified: messages flush only when all 16 segments are
  present (full ELM); shorter messages rely on TTL expiry in cleanup

**CPDLC decoder improvements:**
- New `cpdlc_try_decode_dir()` API with explicit direction parameter,
  replacing undirected `cpdlc_try_decode()`
- Structural validation rules: plausible timestamp check, zero-padding
  verification, minimum message length (16 bits), reserved-slot rejection
- Scoring system for decode confidence (used by ELM to select best direction)
- Range-check fix for constrained integers with negative lower bounds
  (GNSS altitude, temperature) — previously could accept out-of-range values

**COSPAS-SARSAT decoder hardening:**
- Reject frames with unassigned protocol code (noise passing BCH brute-force)
- Long-format frames with BCH-1 bit corrections now also require BCH-2 valid
- Fix position-source bit polarity (0=internal GPS, not 1)

**Statistics dashboard improvements:**
- Stats history API supports `max_points` query parameter for server-side
  downsampling — reduces payload size for long retention periods
- Stats page uses capped point count (600) to avoid browser performance issues
- Effective snapshot interval reported to client for correct time-axis scaling

**Panel log improvements:**
- SQUAWK change messages now show the ICAO hex address (e.g. `[4BA90D  ]`)
  instead of `[?]` when the aircraft callsign has not yet been received

---

### v1.0.7 (2026-07-06)

**Panel restructure:**
- New home page with OpenStreetMap aircraft map (zoom, pan, hover tooltips)
- Configuration page moved from `/` to `/config.html`
- New Feeders page (`/feeders.html`) with all feeder/hub settings
- Version badge on map overlay (bottom-right)
- Updated navigation bar across all pages (static and inline)

**Multi-SDR enhancements:**
- Full libsdrgg backend support for FC0012 tuner (dongle 00000103): frequency
  tuning, gain control, waterfall spectrum display
- Receiver health monitor: detect stalled receivers (30s no ADS-B with active
  USB) and auto-recover via reset cascade
- `dump_registers` and `read_demod_reg` backend operations for runtime
  diagnostics panel
- Tuner AGC capability flag per device (R820T/R820T2 only)

**Radiosonde decoder improvements:**
- Configurable sonde frequency via decoder config (no longer hardcoded)

**Bug fixes:**
- Remove unconditional USB trace logging from libsdrgg backend
- Remove waterfall power diagnostic print (wf-diag)
- Fix panel nav bar consistency across all pages

**Cleanup:**
- Remove `rtl_demod_dump.cpp`, `rtl_init_stress.cpp`, `rtl_register_test.py`
  (development test tools, not part of public distribution)

---

### v1.0.6 (2026-05-27)

**BDS 4,4 / 4,5 meteorological decoders (new):**
- Decode Comm-B BDS 4,4 (Meteorological Routine Air Report — MRAR):
  wind speed/direction, static air temperature, pressure, humidity, turbulence
- Decode Comm-B BDS 4,5 (Meteorological Hazard Report — MHAR):
  turbulence, windshear, microburst, icing, wake vortex, temperature, pressure,
  humidity severity levels
- New fields tracked per aircraft and exposed in `/api/aircraft` JSON
- Aircraft panel page: hover tooltip showing real-time MRAR/MHAR weather data
  with severity colour coding

**Waterfall spectrum analyzer (new):**
- Real-time FFT spectrum display and scrolling waterfall via WebSocket
  (`panel/waterfall.html`, `/ws/waterfall` endpoint)
- Per-device SDR selection: tap any connected dongle for live spectrum view
- Adjustable gain slider and sample rate (1.0 / 1.6 / 2.0 / 2.4 Msps)
- "Take Ownership" mode: pause decoder, retune frequency, release and restore
- IQ ring-buffer tap in `sdr_receiver.cpp`, 256-point Hann-windowed FFT in
  `config_panel.cpp`

**Statistics dashboard (new):**
- New `panel/stats.html` page with Chart.js time-series graphs for message
  rates, signal levels, and per-decoder counters
- Stats history engine: periodic snapshots saved to disk, survives restarts
- New API endpoints: `/api/system-stats`, `/api/decoder-stats`,
  `/api/stats-history`

**System warnings (new):**
- Warning popup system (`panel/warnings.js`) loaded on all panel pages
- `/api/warnings` endpoint returns active warnings as JSON
- DVB kernel module conflict detection: warns when `dvb_usb_rtl28xxu` is
  loaded and provides the fix command

**Diagnostics page (new):**
- Runtime diagnostics panel page (`/diagnostics.html`) with per-dongle signal
  metrics and frequency sweep, served inline by `config_panel.cpp`
- `/api/diagnostics` and `/api/diagnostics/start` endpoints

**BeastReduce rate limiting (new):**
- Per-aircraft rate limiting for bandwidth-efficient beast feeding
  (default 250 ms interval, configurable via `--beast-reduce-interval`)
- All default beast feed networks now use BeastReduce format
- Non-state messages (DF11, DF17 identity, etc.) always forwarded immediately

**CPDLC decoder improvements:**
- Expanded UPER decoding coverage for additional CPDLC message types
- Improved altitude, speed, and position element parsing

**C/C++ migration:**
- Renamed all `.c` source files to `.cpp` (C++17 compilation throughout)
- Replaced `printf`/`fprintf` with `gg::print`/`gg::eprint` wrappers
- Migrated `int`/`unsigned` to explicit `int32_t`/`uint32_t` types
- Added `gg_format.h` utility header for type-safe formatted output

**Build system:**
- `DUMP1090_DIAGNOSTICS=yes` build flag: opt-in verbose decoder trace output
  (ACARS, VDL2, FANET, GSM calibration, adaptive-gain, panel, decoder-config)
- Fix static linking of `libsdrgg.a` when `SDRGG_PREFIX` is set

**Cleanup:**
- Add SPDX GPL-3.0-or-later headers to `src/dispatch/` and `src/main/app_config.h`
- Add GPL-2.0-or-later headers to `src/decode/iot/iot_tracker.cpp/.h`
- Remove stale `src/panel/panel_index.html`
- Fix ACARS typo in README feature overview
- Align IoT license text with actual per-file GPL-2.0-or-later notices
- Document LTE, IoT, waterfall, diagnostics, stats, and warnings features in
  README

---

### v1.0.5 (2026-05-14)

**FANET+ LoRa decoder (new):**
- Full FANET+ (Flying Ad-hoc NETwork) decoder for paragliders, hang-gliders,
  and light aircraft on 868.2 MHz (`fanet_decode.c/.h`)
- LoRa CSS demodulation: SF7, BW250kHz, 500 kSPS, 128 chips/symbol
- PHY: preamble detection (8 upchirps), sync word (0xF1), 2.25 downchirp SFD,
  explicit header (CR=4/8), payload with variable coding rate
- Dechirping via complex multiplication + FFT peak detection, gray decode,
  deinterleave, Hamming FEC, CRC-16 CCITT
- All FANET message types: tracking (type 1), name (type 2), text (type 3),
  service/weather (type 4), landmark (type 5), ground tracking (type 7),
  hardware info (type 0xA), thermal (type 9)
- Name cache (64 entries, ~1h TTL) for pilot/aircraft identification
- Synthetic ADS-B track integration: FANET positions appear on Aircraft page
  with `FNT` callsign prefix and FLARM-style type icons
- Dedicated FANET Monitor panel page with decoder statistics
- SDR role `SDR_ROLE_FANET` for dedicated dongle assignment

**COSPAS-SARSAT 406 MHz beacon decoder (new):**
- Emergency distress beacon decoder for ELT, EPIRB, PLB, SSAS, ELT-DT
  (`sarsat_decode.c/.h`)
- Signal chain: 2.4 MSPS IQ → FM discriminator → 60× decimation (40 kHz IF)
  → DC removal → 21-tap Blackman LPF → AGC → PLL (800 half-sym/s) →
  Biphase-L/Manchester → preamble detect → frame sync → BCH → protocol decode
- BCH-1(82,61) t=3 and BCH-2(38,26) t=2 forward error correction
- Protocol fields: beacon type, country code (200+ countries), MMSI,
  ICAO address, position (long-format), certificate/serial numbers
- SDR role `SDR_ROLE_SARSAT` for dedicated dongle assignment

**PilotAware P3I decoder (new):**
- PilotAware P3I anti-collision protocol decoder (`p3i_decode.c/.h`,
  `p3i_demod.c/.h`)
- 869.525 MHz, 2-FSK, 38.4 kbps, ±10 kHz deviation
- Syncword correlation, net ID validation, CRC integrity check
- Integrated into 868 MHz FLARM multi-protocol demodulator

**ADS-L decoder (new):**
- EASA SRD-860 Electronic Conspicuity standard decoder
  (`adsl_decode.c/.h`)
- 868.2/868.4 MHz, 100 kbps 2-FSK, Manchester encoding
- XXTEA descramble (zero key, 5 words, 6 rounds), Mode-S CRC-24 check
- FANET cordic position decoding, UnsVR speed/altitude encoding
- Integrated into 868 MHz FLARM demodulator as 3rd NCC sync template
- Synthetic DF18 CF=5 messages for aircraft tracking integration

**Airframes.io feed (new):**
- UDP JSON feeder for ACARS and VDL2 messages to airframes.io
  (`airframes_feed.c/.h`)
- ACARS: acarsdec-compatible JSON format → feed.acars.io:5550
- VDL2: dumpvdl2-compatible JSON format → feed.acars.io:5552
- Configurable station ID, host, port; enable/disable per feed
- Panel UI with per-feed toggle controls (active when decoder running)
- CLI: `--airframes-acars`, `--airframes-vdl2`, `--airframes-station-id`

**ACARS label semantic lookup (new):**
- Human-readable descriptions for ACARS message labels (`acars_label.c/.h`)
- ARINC 618/620 label table with 8 categories (ATC, AOC, AAC, Service,
  Emergency, Weather, Printer, Unknown)
- Used by panel Messages page for label enrichment

**Central message dispatcher (new):**
- C++17 dispatcher architecture (`src/dispatch/`) replacing direct function
  calls between decoders, aircraft tracker, and feeders
- `DecoderQueue<T>` template-based thread-safe SPSC queues
- Unified `aircraft_update_t`, `text_message_t`, `ground_track_t`,
  `raw_modes_t` data types across all decoder sources
- Integrated into main loop via `backgroundTasks()` polling

**C → C++ migration:**
- Core networking and panel modules migrated to C++17 for dispatcher
  integration and RAII resource management:
  `feeder_thread`, `net_io`, `mlat_client`, `ogn_client`, `piaware_client`,
  `sondehub_client`, `config_panel`, `gsm_tracker`, `iot_tracker`,
  `lte_tracker`

**868 MHz multi-protocol demodulator enhancements:**
- FLARM demodulator now runs four parallel NCC sync templates:
  FLARM V6/V7, OGN-TP, ADS-L, P3I
- OGNTP decoder: added LDPC(208,160) FEC, comprehensive message struct
  with position/velocity/aircraft type fields, relay counting, status
  validation flags
- FLARM reader: unified SPSC queue pattern for all 868 MHz protocols

**ACARS/VDL2 demodulator improvements:**
- ACARS: enhanced multi-channel reception, improved AM-MSK handling
- VDL2: enhanced D8PSK demodulation, better frame synchronization

**Aircraft tracking enhancements:**
- Extended aircraft state for new decoder sources (FANET, P3I, ADS-L)
- FLARM-style type mapping for FANET aircraft categories
- Improved track management for mixed-source targets

**Panel improvements:**
- Airframes.io feed section in config page with per-feed enable/disable
  checkboxes (grayed out when decoder inactive)
- SARSAT added to Decoder Roles description in Devices page
- `roleLabels` and `roleIcons` updated for all 11 decoder roles
- FANET Monitor page with real-time decoder statistics
- Status page groups airframes feeds under "Other" category

**Build system:**
- New source directories: `src/decode/fanet/`, `src/decode/sarsat/`,
  `src/dispatch/`
- C++17 compilation for dispatch module
- New object targets: `fanet_decode.o`, `sarsat_decode.o`, `p3i_decode.o`,
  `p3i_demod.o`, `adsl_decode.o`, `acars_label.o`, `airframes_feed.o`,
  `dispatcher.o`, `msg_queue.o`

---

### v1.0.4 (2026-05-07)

**SDR backend abstraction layer (new):**
- New hardware abstraction layer (`sdr_backend.c/.h`) decoupling all SDR access
  from direct `rtlsdr_*` calls — defines `sdr_backend_ops_t` vtable with ~20
  operations (enumerate, open, close, set_frequency, set_gain, read_async, etc.)
- Unified enums `sdr_tuner_type_t`, `sdr_backend_type_t` and extended
  capabilities struct `sdr_tuner_caps_t` (per-stage gain, bandwidth control,
  PLL lock detection)
- `rtlsdr` backend wrapping all `rtlsdr_*` calls into the new vtable
- Per-receiver `backend=<name>` option in receiver config string

**libsdrgg support (new):**
- New C++ backend (`sdr_backend_sdrgg.cpp`) for libsdrgg — an optimized
  USB SDR driver with R820T-specific IF frequency reprogramming after DDC
  reset, per-stage gain control (LNA/mixer/VGA), PLL lock detection, and
  stream mutex protection
- Build with `make SDRGG=yes` (optionally `SDRGG_PREFIX=/path/to/lib`)
- Prints libsdrgg version at startup when enabled

**LTE SIB decoding (new):**
- Full SIB1 and SI message decoding after MIB (`lte_sib.c/.h`, 1430 lines):
  PDCCH blind detection (SI-RNTI), PDSCH QPSK demod, convolutional/turbo
  decode, rate dematching, CRC-16, ASN.1 UPER bit reader
- SIB1 field extraction: MCC, MNC, TAC, CellID, cell_barred, q_rxlevmin,
  si_window_length
- Emergency alert scanning on SI subframes (4,6,7,8,9) for SIB10/SIB11
  (ETWS) and SIB12 (CMAS), CBS 7-bit text decode

**IQ file replay for all decoders (new):**
- Standalone IQ-file replay infrastructure with per-decoder options:
  `--pocsag-ifile`, `--sonde-ifile`/`--sonde-freq`,
  `--acars-ifile`/`--acars-freq`, `--gsm-ifile`/`--gsm-freq`,
  `--lte-ifile`/`--lte-freq`
- Each decoder gets its own reader thread with real-time throttling, stats
  on completion, and proper cleanup on shutdown

**Source tree reorganization:**
- All source files moved into a structured directory layout:
  `src/main/`, `src/adsb/`, `src/net/`, `src/sdr/`, `src/panel/`,
  `src/decode/{flarm,acars,vdl2,sonde,pocsag,gsm,lte,iot}/`,
  `src/util/`, `src/stubs/`, `include/`, `tests/`, `docs/`,
  `web/legacy-map/`
- Makefile rewritten with VPATH-based build, `obj/` output directory,
  per-module `SRCDIR_*` variables, and named object lists

**Header refactoring:**
- Monolithic `dump1090.h` split into focused headers in `include/`:
  `dump1090_types.h` (enums), `dump1090_defs.h` (constants),
  `dump1090_message.h` (`struct modesMessage`), `dump1090_state.h`
  (`struct _Modes`), and umbrella `dump1090.h`

**SDR receiver migration:**
- `sdr_receiver.c` fully migrated from direct `rtlsdr_*` calls to the backend
  abstraction layer: `rtlsdr_dev_t` → `sdr_device_t`, all API calls through
  `rx->backend_ops->` vtable dispatch
- Device enumeration via `ops->enumerate()` returning `sdr_dev_info_t[]`
- Guard broadened from `#ifdef ENABLE_RTLSDR` to
  `#if defined(ENABLE_RTLSDR) || defined(ENABLE_SDRGG)`

**Config panel migration:**
- Diagnostics and tuner probing migrated from `rtlsdr_*` to backend layer:
  `panelProbeAllTuners()` uses `sdrBackendEnumerateAll()` +
  `ops->open_by_serial()` loop
- Tuner name/range functions use unified `sdr_tuner_type_t` enums

**ELM / Comm-D improvements:**
- `elm_decode_acars()` output refactored from `printf()`/`putchar()` to buffer
  (`char *outbuf, int outbuf_size`) with `snprintf()`
- CPDLC decode uses `open_memstream()` to capture output into buffer
- Simplified panel logging via `panelLogMessage("[ELM] %s", decoded)`

**GSM calibrator improvements:**
- Migrated from `rtlsdr_*` to backend abstraction layer
- Multiple reads per scan (`READS_PER_SCAN=4`) for better SNR through averaging
- Gain strategy changed from auto-gain to max manual gain (FC0012 reliability)
- Lowered FCCH detection threshold ratio from 5.0 to 2.0
- Enhanced diagnostics: DC percentage, per-scan read stats, best ratio tracking

**IoT 868 MHz decoder improvements:**
- Honeywell CM921/CM927 (RAMSES II) multi-pattern temperature extraction:
  scans payload for command codes `0x30C9` (zone temp) and `0x2309` (setpoint)
- Manchester decode buffer increased from 16 to 32 bytes
- Range validation on extracted temperatures (-20°C to 60°C)
- Cast fix for correct 24-bit device ID construction

**Signal handling:**
- `sigintHandler`/`sigtermHandler` now use async-signal-safe `write()` instead
  of `log_with_timestamp()`/`fprintf()`

**Tests:**
- `test_r820t_iq.cpp` — R820T IQ quality test via libsdrgg: per-stage gain
  control, PLL lock check, IQ capture statistics at 3 frequencies

---

### v1.0.3 (2026-05-03)

**FLARM demodulator rewrite:**
- Replaced bit-level shift-register syncword correlator with sample-level
  normalized cross-correlation (NCC) on a 1024-sample sync template
- New 65-tap FIR low-pass channel filter (Hamming window, 120 kHz cutoff)
  replacing single-pole IIR — matches Python v7 decoder design
- FM sample ring buffer with lockout-based re-trigger prevention
- NCO uses direct cos/sin rotation with periodic renormalization
  (eliminates cumulative phase drift from atan2-based approach)
- Corrected FLARM syncword byte order (now matches OGN pre-Manchester
  encoding: `0x0AF3656C`)
- Added `flarm_demod_set_time_override()` API for IQ file replay with
  correct XXTEA decryption timestamps (uses file mtime)
- FLARM identity messages submitted twice to reach reliability threshold faster

**IoT 868 MHz decoder (new):**
- New receiver role `iot868` with dedicated SDR dongle support
- IoT device tracker with JSON API (`/api/iot868`)
- Panel: new "IoT 868" page and nav bar link on all pages
- Panel config: IoT 868 output enable/disable toggle

**SDR receiver manager:**
- Virtual file device support (`--receiver FILE:role:path=/path/to/file.raw`)
  with real-time paced IQ replay in a loop, enabling offline decoder testing
- File replay reader thread with configurable `path=` option in receiver config

**Panel improvements:**
- Aircraft table: new "Dist" (distance) column with client-side Haversine
  calculation from station coordinates, sortable
- Panel log timestamps now include day/month (`dd/mm HH:MM:SS.ms`)
- Log messages prefixed with `[ADSB]` for squawk changes, emergencies, and
  ident events
- SDR config page reorganized: decoder settings separated from hardware
  settings, fetched from new `/api/decoders` endpoint
- Removed keys section from main config API (moved to `/api/decoders`)
- Removed inline ADS-B/FLARM hardware details from config API (moved to
  `/api/receivers`)
- IoT 868 status card added to config API response

**OGN client fix:**
- Corrected APRS position report address-type field encoding: now includes
  stealth/no-track bits and full 4-bit aircraft type (was truncating to 2 bits)

**Configuration architecture:**
- New `decoderConfigInit()` / `decoderConfigLoad()` startup sequence
- `appConfigSyncFromModes()` migration step for legacy settings
- Panel decoder config cards for all roles (ADS-B, FLARM, ACARS, VDL2,
  Radiosonde, POCSAG, GSM, LTE, IoT 868) with live toggle support

**Cleanup:**
- Removed active debug fprintf in FLARM demodulator (fired on every dt=0 packet)
- GSM SCH self-test now logs only on failure (silent when passing)
- Added `receivers.json` to `.gitignore` (instance-specific hardware config)
- Version history moved from README.md to CHANGELOG.md

---

### v1.0.2 (2026-04-30)

**New decoders:**
- **GSM broadcast channel decoder** (`gsm_decode.c/.h`) — passive GSM-900
  downlink scanner: GMSK demod at 270.833 ksym/s, FCCH/SCH detection, Viterbi
  convolutional decoding, Fire code CRC, SI1–SI4 parsing, Cell Broadcast,
  paging request decode, FCCH-only zombie cell tracking
- **GSM cell tracker** (`gsm_tracker.c/.h`) — maintains table of up to 64
  discovered cells with JSON API at `/api/gsm`
- **GSM PPM calibrator** (`gsm_calibrate.c/.h`) — measures RTL-SDR crystal
  offset using GSM carriers (approach from ogn-rf)
- **POCSAG pager decoder** (`pocsag_demod.c/.h`) — multi-baud FSK decoder
  (512/1200/2400 baud), BCH(31,21) ECC, numeric and alpha message extraction
- **LTE cell scanner** (`lte_decode.c/.h`, `lte_tracker.c/.h`) — PSS/SSS
  Zadoff-Chu correlation, OFDM PBCH decoding, MIB extraction (bandwidth,
  PHICH, SFN), SIB1 PLMN/TAC/CellID decode, cell tracker with JSON API

**Multi-SDR enhancements:**
- New receiver roles: `gsm`, `pocsag`, `lte` (in addition to existing `adsb`,
  `flarm`, `acars`, `vdl2`, `radiosonde`)
- SDR receiver manager extended to support up to 8 RTL-SDR dongles with
  per-role sample rate, frequency, and gain configuration
- Panel device management dropdown updated for new roles

**FLARM/OGNTP improvements:**
- **Dual-channel channelizer**: simultaneous reception on 868.2 MHz and
  868.4 MHz using NCO frequency shifting and per-channel LPF, replacing
  single-frequency tuning
- Per-channel diagnostics and syncword match tracking

**ELM (Comm-D) hardening:**
- Content validation filter: rejects reassembled ELM payloads that don't
  contain recognizable ACARS framing, CPDLC/ASN.1 patterns, or structured text
- DF24–31 false-positive rejection: frames that required CRC bit correction
  are now discarded (Address/Parity has no independent CRC)

**Mode S decoder improvements:**
- DF18 false-positive filter: reject frames with valid CRC-24 but no decoded
  data fields (CRC-24 collision filter)
- Squawk change logging to panel log (with special squawk descriptions)

**Panel:**
- All pages updated with consistent nav bar and layout
- New GSM and LTE status pages
- **SDR Diagnostics page** (`/diagnostics.html`) — interactive dongle health
  check: enumerates all connected RTL-SDR devices, reads tuner type/serial/gains,
  runs a short IQ capture to measure noise floor and signal presence, reports
  results in real-time via `/api/diagnostics` REST endpoint

**Build:**
- Makefile updated for new modules (gsm_decode, gsm_tracker, gsm_calibrate,
  lte_decode, lte_tracker, pocsag_demod)
- `COPYING.GPLv3` added for GPL-3.0 license text

**Tests:**
- `gsm_test.c` — GSM decoder unit tests (GMSK, Viterbi, Fire code, SI parse)
- `pocsag_test.c` — POCSAG BCH, address extraction, alpha/numeric decode

---

### v1.0.1 (2026-04-28)

**New features:**
- **OGNTP decoder** (`ogntp_decode.c/.h`) — OGN Tracking Protocol demodulation
  alongside FLARM on 868.4 MHz: syncword detection, Manchester decoding,
  LDPC(208,160) parity check, position extraction, DF18 synthesis for map
  display and feeder sharing

**Bug fixes:**
- HTTP/1.1 ALPN forcing in feeder threads and SondeHub client to prevent h2
  protocol negotiation failures with some endpoints
- Fixed OpenSky dashboard URL in panel (now points to sensors view)

**Improvements:**
- Panel: added Source column with color-coded protocol badges
  (ADS-B/FLARM/OGNTP/MLAT/TIS-B/ADS-R/Mode-S) in aircraft table
- Panel: tooltip CSS, version badge in nav bar across all pages
- config_panel: SondeHub config parsing, OGN station name sync
- flarm_reader: lock-free SPSC message queue for OGNTP packets
- test_sonde: added Test 7 for M10 CRC validation and frame parsing

**Cleanup:**
- Removed obsolete `_rpi_sonde_demod.c` (superseded multi-sonde prototype)

---

### v1.0.0 (2026-04-26)

**Initial public release.** Complete all-in-one ADS-B / Mode S receiver and
multi-feed relay with the following capabilities:

**Core ADS-B / Mode S:**
- Full dump1090-fa decoder with enhanced Comm-B (BDS 4,1 / 4,2 / 4,3 / 4,5),
  ACAS RA extraction (BDS 3,0 / DF16), DF19 military squitter, coarse TIS-B
  (DF18 CF=3), full TC31 Operational Status, derived wind/OAT/TAT, magnetic
  declination model, calculated track, squawk debounce, altitude reliability

**Native feeder threads (lock-free SPSC architecture):**
- PiAware / ADEPT (TLS, FATSV format)
- FlightAware MLAT (UDP binary protocol)
- ADSBexchange feed + MLAT (JSON-over-TCP)
- OpenSky Network (binary upload protocol)
- Beast-binary feed to 11 networks (ADSBx, adsb.fi, FlyItaly, plane.watch,
  adsb.one, adsb.lol, airplanes.live, Planespotters, TheAirTraffic, AVDelphi,
  ADSBHub)
- OGN / APRS-IS (FLARM → OGN position reports)
- SondeHub v2 (HTTPS PUT telemetry upload)

**Decoders:**
- FLARM (868 MHz, GFSK, XXTEA v6/v7, DF18 synthesis)
- ACARS (AM-MSK, 5 EU channels, CRC-16)
- VDL2 (D8PSK 10.5 ksym/s, AVLC/HDLC, ACARS extraction, 1-bit FCS correction)
- Radiosonde (RS41/RS92/DFM/M10 — Reed-Solomon, GPS, PTU)
- ELM / Comm-D reassembly (DF24–31, 16-segment bitmask, 60s TTL)
- CPDLC / FANS-1/A (ASN.1 UPER, 129 DM + 183 UM messages, 41 parameter types)

**Infrastructure:**
- Multi-SDR receiver manager (roles: adsb, flarm, acars, vdl2, radiosonde)
- Web control panel (HTTP REST API on port 8888, live config, status, logs,
  messages, aircraft table, device management)
- Internet connectivity watchdog with automatic feeder pause/resume
- Stub files for proprietary feeders (FR24, PlaneFinder, RadarBox) — build
  compatibility without functional code

**Tests:**
- `test_cpdlc.c` — 57 UPER test vectors for FANS-1/A messages
- `test_acars.c` — CRC-16, parity, frame extraction tests
- `test_vdl2.c` — FCS, D8PSK Gray decode, HDLC unstuffing, AVLC tests
- `test_sonde.c` — RS41 Reed-Solomon, CRC, XOR whitening, ECEF→LLA, full pipeline
