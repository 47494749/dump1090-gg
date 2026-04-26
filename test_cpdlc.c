// test_cpdlc.c — CPDLC decoder unit test with hand-crafted UPER test vectors
//
// Compile: cc -I. -O2 -o test_cpdlc test_cpdlc.c cpdlc_decode.c -lm
// Run:     ./test_cpdlc
//
// Each test vector is a raw UPER bitstream encoding a FANS-1/A CPDLC message.
// We redirect stdout, call cpdlc_try_decode(), and check the output.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "cpdlc_decode.h"

// ========== Bit-level UPER encoder for building test vectors ==========

typedef struct {
    unsigned char buf[512];
    int bit_pos;
} uper_enc_t;

static void enc_init(uper_enc_t *e) {
    memset(e->buf, 0, sizeof(e->buf));
    e->bit_pos = 0;
}

static void enc_bits(uper_enc_t *e, uint32_t val, int nbits) {
    for (int i = nbits - 1; i >= 0; i--) {
        int byte_idx = e->bit_pos / 8;
        int bit_idx = 7 - (e->bit_pos % 8);
        if (val & (1u << i))
            e->buf[byte_idx] |= (1 << bit_idx);
        e->bit_pos++;
    }
}

static void enc_constrained(uper_enc_t *e, int val, int lower, int upper) {
    int range = upper - lower + 1;
    int nbits = 0, tmp = range - 1;
    while (tmp > 0) { nbits++; tmp >>= 1; }
    if (nbits > 0)
        enc_bits(e, (uint32_t)(val - lower), nbits);
}

static void enc_ia5string(uper_enc_t *e, const char *s, int lmin, int lmax) {
    int slen = (int)strlen(s);
    if (lmin != lmax)
        enc_constrained(e, slen, lmin, lmax);
    for (int i = 0; i < slen; i++)
        enc_bits(e, (unsigned char)s[i], 7);
}

static int enc_bytes(const uper_enc_t *e) {
    return (e->bit_pos + 7) / 8;
}

// ========== Message Builders ==========
// FANS-1/A message structure:
//   opt_bitmap:     2 bits  (bit1=has_msgref, bit0=has_timestamp)
//   msg_id:         constrained(0,63) = 6 bits
//   [msg_ref]:      constrained(0,63) = 6 bits (if has_msgref)
//   [timestamp]:    year(0..99)+month(1..12)+day(1..31)+hour(0..23)+min(0..59)
//   elem_count:     constrained(0,4) = 3 bits  (actual = encoded + 1)
//   elem[i]:        constrained(0,128) for DM, constrained(0,182) for UM = 8 bits

static void enc_msg_header(uper_enc_t *e, int msg_id, int msg_ref) {
    int has_ref = (msg_ref >= 0) ? 1 : 0;
    enc_bits(e, has_ref, 1);    // has_msgref
    enc_bits(e, 0, 1);          // has_timestamp = 0
    enc_constrained(e, msg_id, 0, 63);
    if (has_ref)
        enc_constrained(e, msg_ref, 0, 63);
    enc_constrained(e, 0, 0, 4); // 1 element
}

static void enc_dm_elem(uper_enc_t *e, int elem_idx) {
    enc_constrained(e, elem_idx, 0, 128);
}

static void enc_um_elem(uper_enc_t *e, int elem_idx) {
    enc_constrained(e, elem_idx, 0, 182);
}

// Altitude: CHOICE(3b) + value
static void enc_altitude_fl(uper_enc_t *e, int fl) {
    enc_bits(e, 6, 3);  // choice 6 = FL
    enc_constrained(e, fl, 30, 600);
}

static void enc_altitude_feet(uper_enc_t *e, int feet) {
    enc_bits(e, 0, 3);  // choice 0 = feet QNH
    enc_constrained(e, feet / 10, 0, 2500);
}

// Speed: CHOICE(3b) + value
static void enc_speed_ias_kt(uper_enc_t *e, int kt) {
    enc_bits(e, 0, 3);  // choice 0 = IAS knots (×10)
    enc_constrained(e, kt / 10, 7, 38);
}

static void enc_speed_mach(uper_enc_t *e, int mach100) {
    enc_bits(e, 6, 3);  // choice 6 = Mach
    enc_constrained(e, mach100, 61, 92);
}

// Time: hours(0..23) + minutes(0..59)
static void enc_time(uper_enc_t *e, int h, int m) {
    enc_constrained(e, h, 0, 23);
    enc_constrained(e, m, 0, 59);
}

// Degrees: CHOICE(1b) + constrained(1,360)
static void enc_degrees_magnetic(uper_enc_t *e, int deg) {
    enc_bits(e, 0, 1);
    enc_constrained(e, deg, 1, 360);
}

// Position: CHOICE(3b) + ...
static void enc_position_fixname(uper_enc_t *e, const char *fix) {
    enc_bits(e, 0, 3);  // choice 0 = fixname
    enc_ia5string(e, fix, 1, 5);
}

static void enc_position_navaid(uper_enc_t *e, const char *nav) {
    enc_bits(e, 1, 3);  // choice 1 = navaid
    enc_ia5string(e, nav, 1, 4);
}

static void enc_position_airport(uper_enc_t *e, const char *apt) {
    enc_bits(e, 2, 3);  // choice 2 = airport
    enc_ia5string(e, apt, 4, 4);
}

static void enc_position_latlon(uper_enc_t *e, int lat_d, int lat_m, int lat_s,
                                 int lon_d, int lon_m, int lon_e) {
    enc_bits(e, 3, 3); // choice 3 = latlon
    enc_constrained(e, lat_d, 0, 90);
    enc_constrained(e, lat_m, 0, 59);
    enc_bits(e, lat_s, 1);  // 0=N, 1=S
    enc_constrained(e, lon_d, 0, 180);
    enc_constrained(e, lon_m, 0, 59);
    enc_bits(e, lon_e, 1);  // 0=E, 1=W
}

// Beacon code: 4 × constrained(0,7) each 3 bits
static void enc_beacon_code(uper_enc_t *e, int d0, int d1, int d2, int d3) {
    enc_constrained(e, d0, 0, 7);
    enc_constrained(e, d1, 0, 7);
    enc_constrained(e, d2, 0, 7);
    enc_constrained(e, d3, 0, 7);
}

// Offset: direction(ENUM 0..10, 4b) + distance_offset(CHOICE 1b + val)
static void enc_offset_nm(uper_enc_t *e, int dir, int nm) {
    enc_constrained(e, dir, 0, 10);  // direction
    enc_bits(e, 0, 1);               // choice = NM
    enc_constrained(e, nm, 1, 128);
}

// Frequency: CHOICE(2b) + val
static void enc_frequency_vhf(uper_enc_t *e, int khz) {
    enc_bits(e, 1, 2);  // choice 1 = VHF
    enc_constrained(e, khz, 117000, 138000);
}

// ICAO Unit Name: CHOICE(1b) + IA5 + func(0..7)
static void enc_icao_unit_name(uper_enc_t *e, const char *name, int is_long, int func) {
    enc_bits(e, is_long, 1);
    if (is_long)
        enc_ia5string(e, name, 3, 18);
    else
        enc_ia5string(e, name, 4, 4);
    enc_constrained(e, func, 0, 7);
}

// VerticalRate: CHOICE(1b) + val
static void enc_vertical_rate_fpm(uper_enc_t *e, int fpm) {
    enc_bits(e, 0, 1);  // English
    enc_constrained(e, fpm / 100, 0, 60);
}

// Altimeter: CHOICE(1b) + val
static void enc_altimeter_inhg(uper_enc_t *e, int inhg100) {
    enc_bits(e, 0, 1);
    enc_constrained(e, inhg100, 2200, 3200);
}

// Version: constrained(0,15)
static void enc_version(uper_enc_t *e, int v) {
    enc_constrained(e, v, 0, 15);
}

// Error: constrained(0,9)
static void enc_error(uper_enc_t *e, int v) {
    enc_constrained(e, v, 0, 9);
}

// ATIS: IA5 1..1
static void enc_atis(uper_enc_t *e, char c) {
    enc_ia5string(e, (char[]){c, '\0'}, 1, 1);
}

// ProcedureName: opt(1b) + type(0..2) + proc(IA5 1..6) + [trans(IA5 1..5)]
static void enc_procedure_name(uper_enc_t *e, int ptype, const char *proc, const char *trans) {
    int has_trans = (trans != NULL);
    enc_bits(e, has_trans, 1);
    enc_constrained(e, ptype, 0, 2);
    enc_ia5string(e, proc, 1, 6);
    if (has_trans)
        enc_ia5string(e, trans, 1, 5);
}

// RouteClearance: 10-bit opt bitmap + optional fields
static void enc_route_clearance_simple(uper_enc_t *e, const char *dep_apt, const char *dst_apt) {
    // Set bits 9 and 8 only (departure + destination airports)
    int bitmask = 0;
    if (dep_apt) bitmask |= (1 << 9);
    if (dst_apt) bitmask |= (1 << 8);
    enc_bits(e, bitmask, 10);
    if (dep_apt) enc_ia5string(e, dep_apt, 4, 4);
    if (dst_apt) enc_ia5string(e, dst_apt, 4, 4);
}

// Route clearance with route (bit 1 = RouteInformationSequence)
static void enc_route_clearance_with_route(uper_enc_t *e,
    const char *dep_apt, const char *dst_apt, const char **fixes, int nfixes) {
    int bitmask = 0;
    if (dep_apt) bitmask |= (1 << 9);
    if (dst_apt) bitmask |= (1 << 8);
    bitmask |= (1 << 1);  // bit 1 = RouteInformationSequence
    enc_bits(e, bitmask, 10);
    if (dep_apt) enc_ia5string(e, dep_apt, 4, 4);
    if (dst_apt) enc_ia5string(e, dst_apt, 4, 4);
    // RouteInformationSequence: count constrained(1,128) + N × RouteInformation
    enc_constrained(e, nfixes, 1, 128);
    for (int i = 0; i < nfixes; i++) {
        // RouteInformation CHOICE(3b,0..5): 0 = publishedIdentifier
        enc_bits(e, 0, 3);  // publishedIdentifier
        // PublishedIdentifier: fixName(IA5 1..5) + OPT latlon
        enc_bits(e, 0, 1);  // no optional latlon
        enc_ia5string(e, fixes[i], 1, 5);
    }
}

// FuelPersons: time(h,m) + persons(1..1024)
static void enc_fuel_persons(uper_enc_t *e, int h, int m, int pob) {
    enc_constrained(e, h, 0, 23);
    enc_constrained(e, m, 0, 59);
    enc_constrained(e, pob, 1, 1024);
}

// HoldAtWaypoint: 7-bit opt + position + optional fields
static void enc_hold_simple(uper_enc_t *e, const char *fix, int alt_fl) {
    // Just speed_lo=0, alt=1, rest=0 => bitmap = 0100000 = 0x20
    enc_bits(e, 0x20, 7);
    enc_position_fixname(e, fix);
    enc_altitude_fl(e, alt_fl);
}

// Unit name + frequency
static void enc_unit_name_freq(uper_enc_t *e, const char *name, int is_long, int func, int freq_khz) {
    enc_icao_unit_name(e, name, is_long, func);
    enc_frequency_vhf(e, freq_khz);
}

// ========== Test Framework ==========

static int test_pass = 0, test_fail = 0;

static int run_test(const char *name, const unsigned char *data, int len,
                    int is_uplink, const char *expected) {
    // Redirect stdout to pipe
    int pipefd[2];
    if (pipe(pipefd) < 0) { perror("pipe"); return -1; }

    fflush(stdout);
    int saved_stdout = dup(STDOUT_FILENO);
    dup2(pipefd[1], STDOUT_FILENO);
    close(pipefd[1]);

    // Call decoder with known direction
    uint32_t addr = 0xABCDEF;
    int decoded = cpdlc_try_decode_dir(addr, data, len, is_uplink ? 1 : 0);

    fflush(stdout);
    dup2(saved_stdout, STDOUT_FILENO);
    close(saved_stdout);

    // Read captured output
    char output[2048];
    int n = read(pipefd[0], output, sizeof(output) - 1);
    close(pipefd[0]);
    if (n < 0) n = 0;
    output[n] = '\0';

    // Remove trailing newline for comparison
    if (n > 0 && output[n-1] == '\n') output[n-1] = '\0';

    // Check
    int pass = 0;
    if (decoded && strstr(output, expected) != NULL) {
        pass = 1;
        test_pass++;
    } else {
        test_fail++;
    }

    fprintf(stderr, "%s %-50s %s\n", pass ? "PASS" : "FAIL", name,
            pass ? "" : "");
    if (!pass) {
        fprintf(stderr, "     expected: %s\n", expected);
        fprintf(stderr, "     got:      %s\n", decoded ? output : "(not decoded)");
        fprintf(stderr, "     hex(%d):  ", len);
        for (int i = 0; i < len; i++) fprintf(stderr, "%02X", data[i]);
        fprintf(stderr, "\n");
    }
    return pass;
}

// ========== Test Cases ==========

int main(void) {
    uper_enc_t e;
    fprintf(stderr, "===== CPDLC Decoder Unit Tests =====\n\n");

    // --- 1. DM0 WILCO ---
    enc_init(&e);
    enc_msg_header(&e, 5, -1);
    enc_dm_elem(&e, 0);
    run_test("DM0 WILCO", e.buf, enc_bytes(&e), 0, "DM0 WILCO");

    // --- 2. DM1 UNABLE ---
    enc_init(&e);
    enc_msg_header(&e, 6, -1);
    enc_dm_elem(&e, 1);
    run_test("DM1 UNABLE", e.buf, enc_bytes(&e), 0, "DM1 UNABLE");

    // --- 3. DM0 WILCO with msg ref ---
    enc_init(&e);
    enc_msg_header(&e, 5, 12);
    enc_dm_elem(&e, 0);
    run_test("DM0 WILCO REF=12", e.buf, enc_bytes(&e), 0, "REF=12");

    // --- 4. DM6 REQUEST FL350 ---
    enc_init(&e);
    enc_msg_header(&e, 10, -1);
    enc_dm_elem(&e, 6);
    enc_altitude_fl(&e, 350);
    run_test("DM6 REQUEST FL350", e.buf, enc_bytes(&e), 0, "DM6 REQUEST FL350");

    // --- 5. DM9 REQUEST CLIMB TO FL380 ---
    enc_init(&e);
    enc_msg_header(&e, 11, -1);
    enc_dm_elem(&e, 9);
    enc_altitude_fl(&e, 380);
    run_test("DM9 REQUEST CLIMB TO FL380", e.buf, enc_bytes(&e), 0,
             "DM9 REQUEST CLIMB TO FL380");

    // --- 6. DM18 REQUEST 250kt ---
    enc_init(&e);
    enc_msg_header(&e, 7, -1);
    enc_dm_elem(&e, 18);
    enc_speed_ias_kt(&e, 250);
    run_test("DM18 REQUEST 250kt", e.buf, enc_bytes(&e), 0,
             "DM18 REQUEST 250kt");

    // --- 7. DM19 REQUEST M.82 TO M.84 ---
    enc_init(&e);
    enc_msg_header(&e, 8, -1);
    enc_dm_elem(&e, 19);
    enc_speed_mach(&e, 82);
    enc_speed_mach(&e, 84);
    run_test("DM19 REQUEST M.82 TO M.84", e.buf, enc_bytes(&e), 0,
             "DM19 REQUEST M.82 TO M.84");

    // --- 8. DM22 REQUEST DIRECT TO ABCDE ---
    enc_init(&e);
    enc_msg_header(&e, 12, -1);
    enc_dm_elem(&e, 22);
    enc_position_fixname(&e, "ABCDE");
    run_test("DM22 REQUEST DIRECT TO ABCDE", e.buf, enc_bytes(&e), 0,
             "DM22 REQUEST DIRECT TO ABCDE");

    // --- 9. DM32 PRESENT LEVEL FL350 ---
    enc_init(&e);
    enc_msg_header(&e, 15, -1);
    enc_dm_elem(&e, 32);
    enc_altitude_fl(&e, 350);
    run_test("DM32 PRESENT LEVEL FL350", e.buf, enc_bytes(&e), 0,
             "DM32 PRESENT LEVEL FL350");

    // --- 10. DM35 PRESENT HEADING 270 (magnetic) ---
    enc_init(&e);
    enc_msg_header(&e, 20, -1);
    enc_dm_elem(&e, 35);
    enc_degrees_magnetic(&e, 270);
    run_test("DM35 PRESENT HEADING 270", e.buf, enc_bytes(&e), 0,
             "DM35 PRESENT HEADING 270");

    // --- 11. DM33 PRESENT POSITION (latlon) ---
    enc_init(&e);
    enc_msg_header(&e, 21, -1);
    enc_dm_elem(&e, 33);
    enc_position_latlon(&e, 48, 24, 0, 9, 56, 0);  // N4824/E00956
    run_test("DM33 PRESENT POSITION 4824N/00956E", e.buf, enc_bytes(&e), 0,
             "DM33 PRESENT POSITION 4824N/00956E");

    // --- 12. DM47 SQUAWKING 1234 ---
    enc_init(&e);
    enc_msg_header(&e, 25, -1);
    enc_dm_elem(&e, 47);
    enc_beacon_code(&e, 1, 2, 3, 4);
    run_test("DM47 SQUAWKING 1234", e.buf, enc_bytes(&e), 0,
             "DM47 SQUAWKING 1234");

    // --- 13. DM7 REQUEST BLOCK FL350 TO FL370 ---
    enc_init(&e);
    enc_msg_header(&e, 13, -1);
    enc_dm_elem(&e, 7);
    enc_altitude_fl(&e, 350);
    enc_altitude_fl(&e, 370);
    run_test("DM7 REQUEST BLOCK FL350 TO FL370", e.buf, enc_bytes(&e), 0,
             "DM7 REQUEST BLOCK FL350 TO FL370");

    // --- 14. DM11 AT ABCDE REQUEST CLIMB TO FL380 (POS_ALT) ---
    enc_init(&e);
    enc_msg_header(&e, 14, -1);
    enc_dm_elem(&e, 11);
    enc_position_fixname(&e, "ABCDE");
    enc_altitude_fl(&e, 380);
    run_test("DM11 AT ABCDE CLIMB FL380", e.buf, enc_bytes(&e), 0,
             "DM11 AT ABCDE REQUEST CLIMB TO FL380");

    // --- 15. DM43 NEXT WAYPOINT ETA 14:30Z ---
    enc_init(&e);
    enc_msg_header(&e, 16, -1);
    enc_dm_elem(&e, 43);
    enc_time(&e, 14, 30);
    run_test("DM43 NEXT WAYPOINT ETA 14:30Z", e.buf, enc_bytes(&e), 0,
             "DM43 NEXT WAYPOINT ETA 14:30Z");

    // --- 16. DM73 VERSION 3 ---
    enc_init(&e);
    enc_msg_header(&e, 30, -1);
    enc_dm_elem(&e, 73);
    enc_version(&e, 3);
    run_test("DM73 VERSION 3", e.buf, enc_bytes(&e), 0, "DM73 VERSION 3");

    // --- 17. DM79 ATIS B ---
    enc_init(&e);
    enc_msg_header(&e, 31, -1);
    enc_dm_elem(&e, 79);
    enc_atis(&e, 'B');
    run_test("DM79 ATIS B", e.buf, enc_bytes(&e), 0, "DM79 ATIS B");

    // --- 18. DM62 ERROR undefinedError ---
    enc_init(&e);
    enc_msg_header(&e, 32, -1);
    enc_dm_elem(&e, 62);
    enc_error(&e, 9);  // undefinedError
    run_test("DM62 ERROR undefinedError", e.buf, enc_bytes(&e), 0,
             "DM62 ERROR undefinedError");

    // --- 19. DM14 REQUEST OFFSET LEFT 5NM OF ROUTE ---
    enc_init(&e);
    enc_msg_header(&e, 33, -1);
    enc_dm_elem(&e, 14);
    enc_offset_nm(&e, 0, 5);  // dir=LEFT=0, 5NM
    run_test("DM14 REQUEST OFFSET LEFT 5NM", e.buf, enc_bytes(&e), 0,
             "DM14 REQUEST OFFSET LEFT 5NM");

    // --- 20. DM23 REQUEST PROCEDURE (APP RNAV1.ILS) ---
    enc_init(&e);
    enc_msg_header(&e, 34, -1);
    enc_dm_elem(&e, 23);
    enc_procedure_name(&e, 1, "RNAV1", "ILS");  // APP
    run_test("DM23 REQUEST APP RNAV1.ILS", e.buf, enc_bytes(&e), 0,
             "DM23 REQUEST APP RNAV1.ILS");

    // --- 21. DM24 REQUEST ROUTE (dep=EGLL, dst=LFPG) ---
    enc_init(&e);
    enc_msg_header(&e, 35, -1);
    enc_dm_elem(&e, 24);
    enc_route_clearance_simple(&e, "EGLL", "LFPG");
    run_test("DM24 REQUEST ROUTE DEP:EGLL DST:LFPG", e.buf, enc_bytes(&e), 0,
             "DEP:EGLL DST:LFPG");

    // --- 22. DM57 FUEL AND PERSONS ---
    enc_init(&e);
    enc_msg_header(&e, 36, -1);
    enc_dm_elem(&e, 57);
    enc_fuel_persons(&e, 2, 30, 185);
    run_test("DM57 FUEL 02:30 185 POB", e.buf, enc_bytes(&e), 0,
             "DM57 02:30 FUEL 185 POB");

    // --- 23. DM90 REQUEST APPROACH (NULL) ---
    enc_init(&e);
    enc_msg_header(&e, 37, -1);
    enc_dm_elem(&e, 90);
    run_test("DM90 REQUEST APPROACH", e.buf, enc_bytes(&e), 0,
             "DM90 REQUEST APPROACH");

    // --- 24. DM92 REQUEST PROCEDURE APPROACH (NULL) ---
    enc_init(&e);
    enc_msg_header(&e, 38, -1);
    enc_dm_elem(&e, 92);
    run_test("DM92 REQUEST PROCEDURE APPROACH", e.buf, enc_bytes(&e), 0,
             "DM92 REQUEST PROCEDURE APPROACH");

    // --- 25. DM102 LANDING REPORT (NULL) ---
    enc_init(&e);
    enc_msg_header(&e, 39, -1);
    enc_dm_elem(&e, 102);
    run_test("DM102 LANDING REPORT", e.buf, enc_bytes(&e), 0,
             "DM102 LANDING REPORT");

    // ===== UPLINK MESSAGES =====

    // --- 26. UM0 UNABLE ---
    enc_init(&e);
    enc_msg_header(&e, 1, -1);
    enc_um_elem(&e, 0);
    run_test("UM0 UNABLE", e.buf, enc_bytes(&e), 1, "UM0 UNABLE");

    // --- 27. UM19 MAINTAIN FL370 ---
    enc_init(&e);
    enc_msg_header(&e, 3, -1);
    enc_um_elem(&e, 19);
    enc_altitude_fl(&e, 370);
    run_test("UM19 MAINTAIN FL370", e.buf, enc_bytes(&e), 1,
             "UM19 MAINTAIN FL370");

    // --- 28. UM20 CLIMB TO AND MAINTAIN FL380 ---
    enc_init(&e);
    enc_msg_header(&e, 8, -1);
    enc_um_elem(&e, 20);
    enc_altitude_fl(&e, 380);
    run_test("UM20 CLIMB TO AND MAINTAIN FL380", e.buf, enc_bytes(&e), 1,
             "UM20 CLIMB TO AND MAINTAIN FL380");

    // --- 29. UM46 CROSS ABCDE AT FL350 ---
    enc_init(&e);
    enc_msg_header(&e, 9, -1);
    enc_um_elem(&e, 46);
    enc_position_fixname(&e, "ABCDE");
    enc_altitude_fl(&e, 350);
    run_test("UM46 CROSS ABCDE AT FL350", e.buf, enc_bytes(&e), 1,
             "UM46 CROSS ABCDE AT FL350");

    // --- 30. UM75 WHEN ABLE PROCEED DIRECT TO ABCDE (PT_POS) ---
    enc_init(&e);
    enc_msg_header(&e, 30, -1);
    enc_um_elem(&e, 75);
    enc_position_fixname(&e, "ABCDE");
    run_test("UM75 WHEN ABLE PROCEED DIRECT TO ABCDE", e.buf, enc_bytes(&e), 1,
             "UM75 WHEN ABLE PROCEED DIRECT TO ABCDE");

    // --- 31. UM76 AT 12:30Z PROCEED DIRECT TO XYZ (TIME_POS) ---
    enc_init(&e);
    enc_msg_header(&e, 40, -1);
    enc_um_elem(&e, 76);
    enc_time(&e, 12, 30);
    enc_position_fixname(&e, "XYZ");
    run_test("UM76 AT 12:30Z PROCEED DIRECT TO XYZ", e.buf, enc_bytes(&e), 1,
             "UM76 AT 12:30Z PROCEED DIRECT TO XYZ");

    // --- 32. UM77 AT VICKY PROCEED DIRECT TO BAKER (POS_POS) ---
    enc_init(&e);
    enc_msg_header(&e, 41, -1);
    enc_um_elem(&e, 77);
    enc_position_fixname(&e, "VICKY");
    enc_position_fixname(&e, "BAKER");
    run_test("UM77 AT VICKY PROCEED DIRECT TO BAKER", e.buf, enc_bytes(&e), 1,
             "UM77 AT VICKY PROCEED DIRECT TO BAKER");

    // --- 33. UM78 AT FL350 PROCEED DIRECT TO ABCDE (ALT_POS) ---
    enc_init(&e);
    enc_msg_header(&e, 42, -1);
    enc_um_elem(&e, 78);
    enc_altitude_fl(&e, 350);
    enc_position_fixname(&e, "ABCDE");
    run_test("UM78 AT FL350 PROCEED DIRECT TO ABCDE", e.buf, enc_bytes(&e), 1,
             "UM78 AT FL350 PROCEED DIRECT TO ABCDE");

    // --- 34. UM88 AT VICKY EXPECT DIRECT TO BAKER (POS_POS) ---
    enc_init(&e);
    enc_msg_header(&e, 43, -1);
    enc_um_elem(&e, 88);
    enc_position_fixname(&e, "VICKY");
    enc_position_fixname(&e, "BAKER");
    run_test("UM88 AT VICKY EXPECT DIRECT TO BAKER", e.buf, enc_bytes(&e), 1,
             "UM88 AT VICKY EXPECT DIRECT TO BAKER");

    // --- 35. UM89 AT 14:00Z EXPECT DIRECT TO ABCDE (TIME_POS) ---
    enc_init(&e);
    enc_msg_header(&e, 44, -1);
    enc_um_elem(&e, 89);
    enc_time(&e, 14, 0);
    enc_position_fixname(&e, "ABCDE");
    run_test("UM89 AT 14:00Z EXPECT DIRECT TO ABCDE", e.buf, enc_bytes(&e), 1,
             "UM89 AT 14:00Z EXPECT DIRECT TO ABCDE");

    // --- 36. UM90 AT FL350 EXPECT DIRECT TO ABCDE (ALT_POS) ---
    enc_init(&e);
    enc_msg_header(&e, 45, -1);
    enc_um_elem(&e, 90);
    enc_altitude_fl(&e, 350);
    enc_position_fixname(&e, "ABCDE");
    run_test("UM90 AT FL350 EXPECT DIRECT TO ABCDE", e.buf, enc_bytes(&e), 1,
             "UM90 AT FL350 EXPECT DIRECT TO ABCDE");

    // --- 37. UM79 CLEARED TO ABCDE VIA route (POS_ROUTECLR) ---
    enc_init(&e);
    enc_msg_header(&e, 46, -1);
    enc_um_elem(&e, 79);
    enc_position_fixname(&e, "ABCDE");
    enc_route_clearance_simple(&e, "EGLL", "LFPG");
    run_test("UM79 CLEARED TO ABCDE VIA EGLL-LFPG", e.buf, enc_bytes(&e), 1,
             "UM79 CLEARED TO ABCDE VIA DEP:EGLL DST:LFPG");

    // --- 38. UM83 AT ABCDE CLEARED route (POS_ROUTECLR) ---
    enc_init(&e);
    enc_msg_header(&e, 47, -1);
    enc_um_elem(&e, 83);
    enc_position_fixname(&e, "ABCDE");
    enc_route_clearance_simple(&e, "EGLL", NULL);
    run_test("UM83 AT ABCDE CLEARED DEP:EGLL", e.buf, enc_bytes(&e), 1,
             "UM83 AT ABCDE CLEARED DEP:EGLL");

    // --- 39. UM84 AT ABCDE CLEARED procname (POS_PROCNAME) ---
    enc_init(&e);
    enc_msg_header(&e, 48, -1);
    enc_um_elem(&e, 84);
    enc_position_fixname(&e, "ABCDE");
    enc_procedure_name(&e, 0, "STAR1", NULL);  // ARR
    run_test("UM84 AT ABCDE CLEARED ARR STAR1", e.buf, enc_bytes(&e), 1,
             "UM84 AT ABCDE CLEARED ARR STAR1");

    // --- 40. UM86 AT ABCDE EXPECT route (POS_ROUTECLR) ---
    enc_init(&e);
    enc_msg_header(&e, 49, -1);
    enc_um_elem(&e, 86);
    enc_position_fixname(&e, "ABCDE");
    enc_route_clearance_simple(&e, NULL, "EDDF");
    run_test("UM86 AT ABCDE EXPECT DST:EDDF", e.buf, enc_bytes(&e), 1,
             "UM86 AT ABCDE EXPECT DST:EDDF");

    // --- 41. UM101 AT ABCDE EXPECT SPEED 250kt (POS_SPD) ---
    enc_init(&e);
    enc_msg_header(&e, 50, -1);
    enc_um_elem(&e, 101);
    enc_position_fixname(&e, "ABCDE");
    enc_speed_ias_kt(&e, 250);
    run_test("UM101 AT ABCDE EXPECT SPEED 250kt", e.buf, enc_bytes(&e), 1,
             "UM101 AT ABCDE EXPECT SPEED 250kt");

    // --- 42. UM102 AT FL350 EXPECT SPEED M.82 (ALT_SPD) ---
    enc_init(&e);
    enc_msg_header(&e, 51, -1);
    enc_um_elem(&e, 102);
    enc_altitude_fl(&e, 350);
    enc_speed_mach(&e, 82);
    run_test("UM102 AT FL350 EXPECT SPEED M.82", e.buf, enc_bytes(&e), 1,
             "UM102 AT FL350 EXPECT SPEED M.82");

    // --- 43. UM103 AT 15:00Z EXPECT SPEED M.80 TO M.84 (TIME_SPD_SPD) ---
    enc_init(&e);
    enc_msg_header(&e, 52, -1);
    enc_um_elem(&e, 103);
    enc_time(&e, 15, 0);
    enc_speed_mach(&e, 80);
    enc_speed_mach(&e, 84);
    run_test("UM103 AT 15:00Z EXPECT M.80 TO M.84", e.buf, enc_bytes(&e), 1,
             "UM103 AT 15:00Z EXPECT SPEED M.80 TO M.84");

    // --- 44. UM117 CONTACT EGLL APP 121.500MHz ---
    enc_init(&e);
    enc_msg_header(&e, 53, -1);
    enc_um_elem(&e, 117);
    enc_unit_name_freq(&e, "EGLL", 0, 1, 121500);  // func=1=APP
    run_test("UM117 CONTACT EGLL APP 121.500MHz", e.buf, enc_bytes(&e), 1,
             "UM117 CONTACT EGLL APP 121.500MHz");

    // --- 45. UM123 SQUAWK 7000 ---
    enc_init(&e);
    enc_msg_header(&e, 54, -1);
    enc_um_elem(&e, 123);
    enc_beacon_code(&e, 7, 0, 0, 0);
    run_test("UM123 SQUAWK 7000", e.buf, enc_bytes(&e), 1,
             "UM123 SQUAWK 7000");

    // --- 46. UM153 ALTIMETER 29.92inHg ---
    enc_init(&e);
    enc_msg_header(&e, 55, -1);
    enc_um_elem(&e, 153);
    enc_altimeter_inhg(&e, 2992);
    run_test("UM153 ALTIMETER 29.92inHg", e.buf, enc_bytes(&e), 1,
             "UM153 ALTIMETER 29.92inHg");

    // --- 47. UM171 CLIMB AT 1500fpm MINIMUM ---
    enc_init(&e);
    enc_msg_header(&e, 56, -1);
    enc_um_elem(&e, 171);
    enc_vertical_rate_fpm(&e, 1500);
    run_test("UM171 CLIMB AT 1500fpm MINIMUM", e.buf, enc_bytes(&e), 1,
             "UM171 CLIMB AT 1500fpm MINIMUM");

    // --- 48. UM178 (reserved) ---
    enc_init(&e);
    enc_msg_header(&e, 57, -1);
    enc_um_elem(&e, 178);
    run_test("UM178 reserved", e.buf, enc_bytes(&e), 1,
             "UM178 (reserved UM178)");

    // --- 49. UM91 HOLD AT ABCDE (HoldClearance) ---
    enc_init(&e);
    enc_msg_header(&e, 58, -1);
    enc_um_elem(&e, 91);
    enc_hold_simple(&e, "ABCDE", 350);
    run_test("UM91 HOLD AT ABCDE FL350", e.buf, enc_bytes(&e), 1,
             "UM91 HOLD AT ABCDE ALT:FL350");

    // --- 50. Route clearance with RouteInformationSequence ---
    enc_init(&e);
    enc_msg_header(&e, 59, -1);
    enc_dm_elem(&e, 24);
    const char *rte_fixes[] = {"ABCDE", "FGHIJ", "KLMNO"};
    enc_route_clearance_with_route(&e, "EGLL", "LFPG", rte_fixes, 3);
    run_test("DM24 ROUTE with fixes", e.buf, enc_bytes(&e), 0,
             "ROUTE:ABCDE FGHIJ KLMNO");

    // --- 51. UM80 CLEARED route ---
    enc_init(&e);
    enc_msg_header(&e, 60, -1);
    enc_um_elem(&e, 80);
    enc_route_clearance_simple(&e, "EDDF", "LIMC");
    run_test("UM80 CLEARED EDDF-LIMC", e.buf, enc_bytes(&e), 1,
             "UM80 CLEARED DEP:EDDF DST:LIMC");

    // --- 52. UM158 ATIS A ---
    enc_init(&e);
    enc_msg_header(&e, 61, -1);
    enc_um_elem(&e, 158);
    enc_atis(&e, 'A');
    run_test("UM158 ATIS A", e.buf, enc_bytes(&e), 1, "UM158 ATIS A");

    // --- 53. UM159 ERROR unrecognizedMsgRefNum ---
    enc_init(&e);
    enc_msg_header(&e, 62, -1);
    enc_um_elem(&e, 159);
    enc_error(&e, 0);
    run_test("UM159 ERROR unrecognizedMsgRefNum", e.buf, enc_bytes(&e), 1,
             "UM159 ERROR unrecognizedMsgRefNum");

    // --- 54. DM21 REQUEST VOICE CONTACT 121.500MHz ---
    enc_init(&e);
    enc_msg_header(&e, 40, -1);
    enc_dm_elem(&e, 21);
    enc_frequency_vhf(&e, 121500);
    run_test("DM21 VOICE CONTACT 121.500MHz", e.buf, enc_bytes(&e), 0,
             "DM21 REQUEST VOICE CONTACT 121.500MHz");

    // --- 55. DM22 REQUEST DIRECT TO airport EGLL ---
    enc_init(&e);
    enc_msg_header(&e, 41, -1);
    enc_dm_elem(&e, 22);
    enc_position_airport(&e, "EGLL");
    run_test("DM22 REQUEST DIRECT TO EGLL", e.buf, enc_bytes(&e), 0,
             "DM22 REQUEST DIRECT TO EGLL");

    // --- 56. DM22 REQUEST DIRECT TO navaid VOR ---
    enc_init(&e);
    enc_msg_header(&e, 42, -1);
    enc_dm_elem(&e, 22);
    enc_position_navaid(&e, "BNN");
    run_test("DM22 REQUEST DIRECT TO BNN (navaid)", e.buf, enc_bytes(&e), 0,
             "DM22 REQUEST DIRECT TO BNN");

    // --- 57. UM94 TURN LEFT HEADING 270 ---
    // UM94 is DirectionDegrees: direction enum + degrees
    enc_init(&e);
    enc_msg_header(&e, 43, -1);
    enc_um_elem(&e, 94);
    enc_constrained(&e, 0, 0, 10); // LEFT = 0, 4 bits
    enc_degrees_magnetic(&e, 270);
    run_test("UM94 TURN LEFT HEADING 270", e.buf, enc_bytes(&e), 1,
             "UM94 TURN LEFT HEADING");

    // --- Summary ---
    fprintf(stderr, "\n===== Results: %d passed, %d failed =====\n",
            test_pass, test_fail);

    return test_fail > 0 ? 1 : 0;
}
