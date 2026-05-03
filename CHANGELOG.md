# Changelog

All notable changes to this project will be documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/).

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
