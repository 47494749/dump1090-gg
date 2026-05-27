// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// cpdlc_decode.c: FANS-1/A CPDLC message decoder (ASN.1 UPER)
//
// Decodes CPDLC messages carried via Comm-D ELM (DF24-31).
// FANS-1/A uses ASN.1 UPER (Unaligned Packed Encoding Rules).
//
// PER constraints extracted from libacars/asn1/FANS*.c generated files.
// Reference: ICAO Doc 9705 (FANS-1/A), RTCA DO-258A
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include <cstdio>
#include <stdint.h>
#include <cstring>
#include <ctype.h>
#include "cpdlc_decode.h"
#include "gg_format.h"
#include <string>

// ========== UPER Bitstream Reader ==========

typedef struct {
    const uint8_t *data;
    int32_t len;       // total bytes
    int32_t bit_pos;   // current bit position
} uper_t;

static void uper_init(uper_t *u, const uint8_t *data, int32_t len) {
    u->data = data;
    u->len = len;
    u->bit_pos = 0;
}

static int32_t uper_bits_left(const uper_t *u) {
    return u->len * 8 - u->bit_pos;
}

static int32_t uper_read(uper_t *u, int nbits) {
    if (nbits <= 0 || nbits > 32 || uper_bits_left(u) < nbits)
        return -1;
    uint32_t val = 0;
    for (int32_t i = 0; i < nbits; i++) {
        int32_t byte_idx = u->bit_pos / 8;
        int32_t bit_idx = 7 - (u->bit_pos % 8);
        val = (val << 1) | ((u->data[byte_idx] >> bit_idx) & 1);
        u->bit_pos++;
    }
    return (int32_t)val;
}

static int32_t uper_skip(uper_t *u, int32_t nbits) {
    if (uper_bits_left(u) < nbits) return -1;
    u->bit_pos += nbits;
    return 0;
}

static int32_t uper_read_constrained(uper_t *u, int32_t lower, int32_t upper) {
    if (upper < lower) return -1;
    int32_t range = upper - lower + 1;
    int32_t nbits = 0;
    int32_t tmp = range - 1;
    while (tmp > 0) { nbits++; tmp >>= 1; }
    if (nbits == 0) return lower;
    int32_t v = uper_read(u, nbits);
    if (v < 0) return -1;
    return v + lower;
}

static int32_t uper_read_ia5string(uper_t *u, char *buf, int32_t bufsize, int32_t lmin, int32_t lmax) {
    int32_t slen;
    if (lmin == lmax) {
        slen = lmin;
    } else {
        slen = uper_read_constrained(u, lmin, lmax);
        if (slen < 0) return -1;
    }
    if (slen >= bufsize) slen = bufsize - 1;
    for (int32_t i = 0; i < slen; i++) {
        int32_t ch = uper_read(u, 7);
        if (ch < 0) return -1;
        buf[i] = (char)(ch & 0x7f);
    }
    buf[slen] = '\0';
    return slen;
}

// ========== Parameter Types ==========

typedef enum {
    PT_NULL = 0,
    PT_ALT, PT_SPD, PT_TIME, PT_POS, PT_FREETEXT, PT_FREQ, PT_DEG,
    PT_ALT_ALT, PT_SPD_SPD, PT_ALT_TIME, PT_POS_ALT, PT_POS_SPD,
    PT_POS_TIME, PT_POS_TIME_ALT,
    PT_VERSION, PT_ERROR, PT_ATIS,
    PT_BEACON, PT_OFFSET, PT_PROCNAME, PT_ROUTECLR, PT_UNITFREQ,
    PT_VRATE, PT_ALTIMETER, PT_HOLD, PT_HOLD2, PT_POS_RPT,
    PT_FUEL_PERSONS, PT_POS_OFFSET,
    PT_POS_ALT_ALT, PT_POS_TIME_TIME, PT_POS_ALT_SPD, PT_POS_SPD_ALT,
    PT_TIME_POS_ALT, PT_POS_POS_ALT,
    PT_POS_UNITFREQ, PT_TIME_UNITFREQ, PT_POS_DEG,
    PT_TIME_POS, PT_POS_POS, PT_ALT_POS,
    PT_POS_ROUTECLR, PT_POS_PROCNAME,
    PT_ALT_SPD, PT_TIME_SPD_SPD,
    PT_COMPLEX,
} param_type_t;

typedef struct {
    const char *text;
    param_type_t ptype;
} msg_element_t;

// ========== DM Table (DM0..DM128) ==========

static const msg_element_t dm_table[] = {
    /* DM0   */ {"WILCO", PT_NULL},
    /* DM1   */ {"UNABLE", PT_NULL},
    /* DM2   */ {"STANDBY", PT_NULL},
    /* DM3   */ {"ROGER", PT_NULL},
    /* DM4   */ {"AFFIRM", PT_NULL},
    /* DM5   */ {"NEGATIVE", PT_NULL},
    /* DM6   */ {"REQUEST %s", PT_ALT},
    /* DM7   */ {"REQUEST BLOCK %s TO %s", PT_ALT_ALT},
    /* DM8   */ {"REQUEST CRUISE CLIMB TO %s", PT_ALT},
    /* DM9   */ {"REQUEST CLIMB TO %s", PT_ALT},
    /* DM10  */ {"REQUEST DESCENT TO %s", PT_ALT},
    /* DM11  */ {"AT %s REQUEST CLIMB TO %s", PT_POS_ALT},
    /* DM12  */ {"AT %s REQUEST DESCENT TO %s", PT_POS_ALT},
    /* DM13  */ {"AT %s REQUEST CRUISE CLIMB TO %s", PT_POS_ALT},
    /* DM14  */ {"REQUEST OFFSET %s OF ROUTE", PT_OFFSET},
    /* DM15  */ {"REQUEST OFFSET %s OF ROUTE", PT_OFFSET},
    /* DM16  */ {"AT %s REQUEST OFFSET %s OF ROUTE", PT_POS_OFFSET},
    /* DM17  */ {"AT %s REQUEST OFFSET %s OF ROUTE", PT_POS_OFFSET},
    /* DM18  */ {"REQUEST %s", PT_SPD},
    /* DM19  */ {"REQUEST %s TO %s", PT_SPD_SPD},
    /* DM20  */ {"REQUEST VOICE CONTACT", PT_NULL},
    /* DM21  */ {"REQUEST VOICE CONTACT %s", PT_FREQ},
    /* DM22  */ {"REQUEST DIRECT TO %s", PT_POS},
    /* DM23  */ {"REQUEST %s", PT_PROCNAME},
    /* DM24  */ {"REQUEST %s", PT_ROUTECLR},
    /* DM25  */ {"REQUEST CLEARANCE", PT_NULL},
    /* DM26  */ {"REQUEST WEATHER DEVIATION TO %s", PT_POS},
    /* DM27  */ {"REQUEST WEATHER DEVIATION UP TO %s OF ROUTE", PT_OFFSET},
    /* DM28  */ {"LEAVING %s", PT_ALT},
    /* DM29  */ {"CLIMBING TO %s", PT_ALT},
    /* DM30  */ {"DESCENDING TO %s", PT_ALT},
    /* DM31  */ {"PASSING %s", PT_POS},
    /* DM32  */ {"PRESENT LEVEL %s", PT_ALT},
    /* DM33  */ {"PRESENT POSITION %s", PT_POS},
    /* DM34  */ {"PRESENT SPEED %s", PT_SPD},
    /* DM35  */ {"PRESENT HEADING %s", PT_DEG},
    /* DM36  */ {"PRESENT GROUND TRACK %s", PT_DEG},
    /* DM37  */ {"MAINTAINING %s", PT_ALT},
    /* DM38  */ {"ASSIGNED LEVEL %s", PT_ALT},
    /* DM39  */ {"ASSIGNED SPEED %s", PT_SPD},
    /* DM40  */ {"ASSIGNED ROUTE %s", PT_ROUTECLR},
    /* DM41  */ {"BACK ON ROUTE", PT_NULL},
    /* DM42  */ {"NEXT WAYPOINT %s", PT_POS},
    /* DM43  */ {"NEXT WAYPOINT ETA %s", PT_TIME},
    /* DM44  */ {"ENSUING WAYPOINT %s", PT_POS},
    /* DM45  */ {"REPORTED WAYPOINT %s", PT_POS},
    /* DM46  */ {"REPORTED WAYPOINT %s %s", PT_POS_TIME},
    /* DM47  */ {"SQUAWKING %s", PT_BEACON},
    /* DM48  */ {"POSITION REPORT %s", PT_POS_RPT},
    /* DM49  */ {"WHEN CAN WE EXPECT %s", PT_SPD},
    /* DM50  */ {"WHEN CAN WE EXPECT %s TO %s", PT_SPD_SPD},
    /* DM51  */ {"WHEN CAN WE EXPECT BACK ON ROUTE", PT_NULL},
    /* DM52  */ {"WHEN CAN WE EXPECT LOWER ALTITUDE", PT_NULL},
    /* DM53  */ {"WHEN CAN WE EXPECT HIGHER ALTITUDE", PT_NULL},
    /* DM54  */ {"WHEN CAN WE EXPECT CRUISE CLIMB", PT_NULL},
    /* DM55  */ {"PAN PAN PAN", PT_NULL},
    /* DM56  */ {"MAYDAY MAYDAY MAYDAY", PT_NULL},
    /* DM57  */ {"%s REMAINING FUEL AND PERSONS ON BOARD", PT_FUEL_PERSONS},
    /* DM58  */ {"CANCEL EMERGENCY", PT_NULL},
    /* DM59  */ {"DIVERTING TO %s", PT_POS},
    /* DM60  */ {"OFFSETTING %s OF ROUTE", PT_OFFSET},
    /* DM61  */ {"DESCENDING TO %s", PT_ALT},
    /* DM62  */ {"ERROR %s", PT_ERROR},
    /* DM63  */ {"NOT CURRENT DATA AUTHORITY", PT_NULL},
    /* DM64  */ {"%s", PT_FREETEXT},
    /* DM65  */ {"DUE TO WEATHER", PT_NULL},
    /* DM66  */ {"DUE TO AIRCRAFT PERFORMANCE", PT_NULL},
    /* DM67  */ {"FREETEXT %s", PT_FREETEXT},
    /* DM68  */ {"FREETEXT %s", PT_FREETEXT},
    /* DM69  */ {"REQUEST VMC DESCENT", PT_NULL},
    /* DM70  */ {"REQUEST HEADING %s", PT_DEG},
    /* DM71  */ {"REQUEST GROUND TRACK %s", PT_DEG},
    /* DM72  */ {"REACHING %s", PT_ALT},
    /* DM73  */ {"VERSION %s", PT_VERSION},
    /* DM74  */ {"MAINTAIN OWN SEPARATION AND VMC", PT_NULL},
    /* DM75  */ {"AT PILOTS DISCRETION", PT_NULL},
    /* DM76  */ {"REACHING BLOCK %s TO %s", PT_ALT_ALT},
    /* DM77  */ {"ASSIGNED BLOCK %s TO %s", PT_ALT_ALT},
    /* DM78  */ {"AT %s", PT_TIME},
    /* DM79  */ {"ATIS %s", PT_ATIS},
    /* DM80  */ {"DEVIATING UP TO %s OF ROUTE", PT_OFFSET},
    /* DM81  */ {"WE CAN ACCEPT %s AT %s", PT_ALT_TIME},
    /* DM82  */ {"WE CANNOT ACCEPT %s", PT_ALT},
    /* DM83  */ {"WE CAN ACCEPT %s AT %s", PT_ALT_TIME},
    /* DM84  */ {"WE CANNOT ACCEPT %s", PT_SPD},
    /* DM85  */ {"WE CANNOT ACCEPT %s", PT_OFFSET},
    /* DM86  */ {"WE CAN ACCEPT %s AT %s", PT_ALT_TIME},
    /* DM87  */ {"WHEN CAN WE EXPECT CLIMB TO %s", PT_ALT},
    /* DM88  */ {"WHEN CAN WE EXPECT DESCENT TO %s", PT_ALT},
    /* DM89  */ {"MONITORING %s", PT_UNITFREQ},
    /* DM90  */ {"REQUEST APPROACH", PT_NULL},
    /* DM91  */ {"HOLDING AT %s", PT_POS},
    /* DM92  */ {"REQUEST PROCEDURE APPROACH", PT_NULL},
    /* DM93  */ {"REQUEST CLEARANCE", PT_NULL},
    /* DM94  */ {"ETA %s %s", PT_POS_TIME},
    /* DM95  */ {"(reserved DM95)", PT_NULL},
    /* DM96  */ {"(reserved DM96)", PT_NULL},
    /* DM97  */ {"(reserved DM97)", PT_NULL},
    /* DM98  */ {"(reserved DM98)", PT_NULL},
    /* DM99  */ {"CURRENT DATA AUTHORITY %s", PT_FREETEXT},
    /* DM100 */ {"LOGICAL ACKNOWLEDGEMENT", PT_NULL},
    /* DM101 */ {"REQUEST END OF SERVICE", PT_NULL},
    /* DM102 */ {"LANDING REPORT", PT_NULL},
    /* DM103 */ {"CANCELLING IFR", PT_NULL},
    /* DM104 */ {"(reserved DM104)", PT_NULL},
    /* DM105 */ {"(reserved DM105)", PT_NULL},
    /* DM106 */ {"PREFERRED LEVEL %s", PT_ALT},
    /* DM107 */ {"NOT AUTHORIZED NEXT DATA AUTHORITY", PT_NULL},
    /* DM108 */ {"DE-ICING COMPLETE", PT_NULL},
    /* DM109 */ {"TOP OF DESCENT %s %s", PT_POS_TIME},
    /* DM110 */ {"REQUEST SPEED OFFSET", PT_NULL},
    /* DM111 */ {"WHEN CAN WE EXPECT %s", PT_ALT},
    /* DM112 */ {"WHEN CAN WE EXPECT %s", PT_SPD},
    /* DM113 */ {"SPEED %s", PT_SPD},
    /* DM114 */ {"(reserved DM114)", PT_NULL},
    /* DM115 */ {"(reserved DM115)", PT_NULL},
    /* DM116 */ {"(reserved DM116)", PT_NULL},
    /* DM117 */ {"(reserved DM117)", PT_NULL},
    /* DM118 */ {"(reserved DM118)", PT_NULL},
    /* DM119 */ {"(reserved DM119)", PT_NULL},
    /* DM120 */ {"(reserved DM120)", PT_NULL},
    /* DM121 */ {"(reserved DM121)", PT_NULL},
    /* DM122 */ {"(reserved DM122)", PT_NULL},
    /* DM123 */ {"(reserved DM123)", PT_NULL},
    /* DM124 */ {"(reserved DM124)", PT_NULL},
    /* DM125 */ {"(reserved DM125)", PT_NULL},
    /* DM126 */ {"(reserved DM126)", PT_NULL},
    /* DM127 */ {"(reserved DM127)", PT_NULL},
    /* DM128 */ {"(reserved DM128)", PT_NULL},
};
#define DM_TABLE_SIZE (sizeof(dm_table) / sizeof(dm_table[0]))

// ========== UM Table (UM0..UM182) ==========

static const msg_element_t um_table[] = {
    /* UM0   */ {"UNABLE", PT_NULL},
    /* UM1   */ {"STANDBY", PT_NULL},
    /* UM2   */ {"REQUEST DEFERRED", PT_NULL},
    /* UM3   */ {"ROGER", PT_NULL},
    /* UM4   */ {"AFFIRM", PT_NULL},
    /* UM5   */ {"NEGATIVE", PT_NULL},
    /* UM6   */ {"EXPECT %s", PT_ALT},
    /* UM7   */ {"EXPECT CLIMB AT %s", PT_TIME},
    /* UM8   */ {"EXPECT CLIMB AT %s", PT_POS},
    /* UM9   */ {"EXPECT DESCENT AT %s", PT_TIME},
    /* UM10  */ {"EXPECT DESCENT AT %s", PT_POS},
    /* UM11  */ {"EXPECT CRUISE CLIMB AT %s", PT_TIME},
    /* UM12  */ {"EXPECT CRUISE CLIMB AT %s", PT_POS},
    /* UM13  */ {"AT %s EXPECT CLIMB TO %s", PT_POS_ALT},
    /* UM14  */ {"AT %s EXPECT DESCENT TO %s", PT_POS_ALT},
    /* UM15  */ {"AT %s EXPECT CRUISE CLIMB TO %s", PT_POS_ALT},
    /* UM16  */ {"AT %s EXPECT CLIMB TO %s", PT_ALT_TIME},
    /* UM17  */ {"AT %s EXPECT DESCENT TO %s", PT_ALT_TIME},
    /* UM18  */ {"AT %s EXPECT CRUISE CLIMB TO %s", PT_ALT_TIME},
    /* UM19  */ {"MAINTAIN %s", PT_ALT},
    /* UM20  */ {"CLIMB TO AND MAINTAIN %s", PT_ALT},
    /* UM21  */ {"AT %s CLIMB TO AND MAINTAIN %s", PT_POS_ALT},
    /* UM22  */ {"AT %s CLIMB TO AND MAINTAIN %s", PT_ALT_TIME},
    /* UM23  */ {"DESCEND TO AND MAINTAIN %s", PT_ALT},
    /* UM24  */ {"AT %s DESCEND TO AND MAINTAIN %s", PT_POS_ALT},
    /* UM25  */ {"AT %s DESCEND TO AND MAINTAIN %s", PT_ALT_TIME},
    /* UM26  */ {"CLIMB TO REACH %s BY %s", PT_ALT_TIME},
    /* UM27  */ {"CLIMB TO REACH %s BY %s", PT_POS_ALT},
    /* UM28  */ {"DESCEND TO REACH %s BY %s", PT_ALT_TIME},
    /* UM29  */ {"DESCEND TO REACH %s BY %s", PT_POS_ALT},
    /* UM30  */ {"MAINTAIN BLOCK %s TO %s", PT_ALT_ALT},
    /* UM31  */ {"CLIMB TO AND MAINTAIN BLOCK %s TO %s", PT_ALT_ALT},
    /* UM32  */ {"DESCEND TO AND MAINTAIN BLOCK %s TO %s", PT_ALT_ALT},
    /* UM33  */ {"CRUISE %s", PT_ALT},
    /* UM34  */ {"CRUISE CLIMB TO %s", PT_ALT},
    /* UM35  */ {"CRUISE CLIMB ABOVE %s", PT_ALT},
    /* UM36  */ {"EXPEDITE CLIMB TO %s", PT_ALT},
    /* UM37  */ {"EXPEDITE DESCENT TO %s", PT_ALT},
    /* UM38  */ {"IMMEDIATELY CLIMB TO %s", PT_ALT},
    /* UM39  */ {"IMMEDIATELY DESCEND TO %s", PT_ALT},
    /* UM40  */ {"IMMEDIATELY STOP CLIMB AT %s", PT_ALT},
    /* UM41  */ {"IMMEDIATELY STOP DESCENT AT %s", PT_ALT},
    /* UM42  */ {"EXPECT TO CROSS %s AT %s", PT_POS_ALT},
    /* UM43  */ {"EXPECT TO CROSS %s AT OR ABOVE %s", PT_POS_ALT},
    /* UM44  */ {"EXPECT TO CROSS %s AT OR BELOW %s", PT_POS_ALT},
    /* UM45  */ {"EXPECT TO CROSS %s AT AND MAINTAIN %s", PT_POS_ALT},
    /* UM46  */ {"CROSS %s AT %s", PT_POS_ALT},
    /* UM47  */ {"CROSS %s AT OR ABOVE %s", PT_POS_ALT},
    /* UM48  */ {"CROSS %s AT OR BELOW %s", PT_POS_ALT},
    /* UM49  */ {"CROSS %s AT AND MAINTAIN %s", PT_POS_ALT},
    /* UM50  */ {"CROSS %s BETWEEN %s AND %s", PT_POS_ALT_ALT},
    /* UM51  */ {"CROSS %s AT %s", PT_POS_TIME},
    /* UM52  */ {"CROSS %s AT OR BEFORE %s", PT_POS_TIME},
    /* UM53  */ {"CROSS %s AT OR AFTER %s", PT_POS_TIME},
    /* UM54  */ {"CROSS %s BETWEEN %s AND %s", PT_POS_TIME_TIME},
    /* UM55  */ {"CROSS %s AT %s AT %s", PT_POS_ALT_SPD},
    /* UM56  */ {"CROSS %s AT OR ABOVE %s AT %s", PT_POS_ALT_SPD},
    /* UM57  */ {"CROSS %s AT %s AT %s", PT_POS_SPD_ALT},
    /* UM58  */ {"CROSS %s AT OR BEFORE %s AT %s", PT_POS_TIME_ALT},
    /* UM59  */ {"CROSS %s AT OR AFTER %s AT %s", PT_POS_TIME_ALT},
    /* UM60  */ {"CROSS %s AT AND MAINTAIN %s AT %s", PT_POS_ALT_SPD},
    /* UM61  */ {"CROSS %s AT %s AT AND MAINTAIN %s", PT_POS_SPD_ALT},
    /* UM62  */ {"AT %s CROSS %s AT AND MAINTAIN %s", PT_TIME_POS_ALT},
    /* UM63  */ {"AT %s CROSS %s AT AND MAINTAIN %s", PT_POS_POS_ALT},
    /* UM64  */ {"OFFSET %s OF ROUTE", PT_OFFSET},
    /* UM65  */ {"AT %s OFFSET %s OF ROUTE", PT_POS_OFFSET},
    /* UM66  */ {"AT %s OFFSET %s OF ROUTE", PT_POS_OFFSET},
    /* UM67  */ {"PROCEED BACK ON ROUTE", PT_NULL},
    /* UM68  */ {"REJOIN ROUTE BY %s", PT_POS},
    /* UM69  */ {"REJOIN ROUTE BY %s", PT_TIME},
    /* UM70  */ {"EXPECT BACK ON ROUTE BY %s", PT_POS},
    /* UM71  */ {"EXPECT BACK ON ROUTE BY %s", PT_TIME},
    /* UM72  */ {"RESUME OWN NAVIGATION", PT_NULL},
    /* UM73  */ {"PROCEED DIRECT TO %s", PT_POS},
    /* UM74  */ {"PROCEED DIRECT TO %s", PT_POS},
    /* UM75  */ {"WHEN ABLE PROCEED DIRECT TO %s", PT_POS},
    /* UM76  */ {"AT %s PROCEED DIRECT TO %s", PT_TIME_POS},
    /* UM77  */ {"AT %s PROCEED DIRECT TO %s", PT_POS_POS},
    /* UM78  */ {"AT %s PROCEED DIRECT TO %s", PT_ALT_POS},
    /* UM79  */ {"CLEARED TO %s VIA %s", PT_POS_ROUTECLR},
    /* UM80  */ {"CLEARED %s", PT_ROUTECLR},
    /* UM81  */ {"CLEARED %s", PT_PROCNAME},
    /* UM82  */ {"CLEARED TO DEVIATE UP TO %s OF ROUTE", PT_OFFSET},
    /* UM83  */ {"AT %s CLEARED %s", PT_POS_ROUTECLR},
    /* UM84  */ {"AT %s CLEARED %s", PT_POS_PROCNAME},
    /* UM85  */ {"EXPECT %s", PT_ROUTECLR},
    /* UM86  */ {"AT %s EXPECT %s", PT_POS_ROUTECLR},
    /* UM87  */ {"EXPECT DIRECT TO %s", PT_POS},
    /* UM88  */ {"AT %s EXPECT DIRECT TO %s", PT_POS_POS},
    /* UM89  */ {"AT %s EXPECT DIRECT TO %s", PT_TIME_POS},
    /* UM90  */ {"AT %s EXPECT DIRECT TO %s", PT_ALT_POS},
    /* UM91  */ {"HOLD AT %s", PT_HOLD},
    /* UM92  */ {"HOLD AT %s AS PUBLISHED MAINTAIN %s", PT_HOLD2},
    /* UM93  */ {"EXPECT FURTHER CLEARANCE AT %s", PT_TIME},
    /* UM94  */ {"TURN LEFT HEADING %s", PT_DEG},
    /* UM95  */ {"TURN RIGHT HEADING %s", PT_DEG},
    /* UM96  */ {"FLY HEADING %s", PT_DEG},
    /* UM97  */ {"AT %s FLY HEADING %s", PT_POS_DEG},
    /* UM98  */ {"IMMEDIATELY TURN LEFT HEADING %s", PT_DEG},
    /* UM99  */ {"EXPECT %s", PT_SPD},
    /* UM100 */ {"AT %s EXPECT SPEED %s", PT_POS_SPD},
    /* UM101 */ {"AT %s EXPECT SPEED %s", PT_POS_SPD},
    /* UM102 */ {"AT %s EXPECT SPEED %s", PT_ALT_SPD},
    /* UM103 */ {"AT %s EXPECT SPEED %s TO %s", PT_TIME_SPD_SPD},
    /* UM104 */ {"IMMEDIATELY TURN RIGHT HEADING %s", PT_DEG},
    /* UM105 */ {"AT %s EXPECT SPEED %s", PT_POS_SPD},
    /* UM106 */ {"MAINTAIN %s", PT_SPD},
    /* UM107 */ {"MAINTAIN PRESENT SPEED", PT_NULL},
    /* UM108 */ {"MAINTAIN %s OR GREATER", PT_SPD},
    /* UM109 */ {"MAINTAIN %s OR LESS", PT_SPD},
    /* UM110 */ {"MAINTAIN %s TO %s", PT_SPD_SPD},
    /* UM111 */ {"INCREASE SPEED TO %s", PT_SPD},
    /* UM112 */ {"INCREASE SPEED TO %s OR GREATER", PT_SPD},
    /* UM113 */ {"REDUCE SPEED TO %s", PT_SPD},
    /* UM114 */ {"REDUCE SPEED TO %s OR LESS", PT_SPD},
    /* UM115 */ {"DO NOT EXCEED %s", PT_SPD},
    /* UM116 */ {"RESUME NORMAL SPEED", PT_NULL},
    /* UM117 */ {"CONTACT %s", PT_UNITFREQ},
    /* UM118 */ {"AT %s CONTACT %s", PT_POS_UNITFREQ},
    /* UM119 */ {"AT %s CONTACT %s", PT_TIME_UNITFREQ},
    /* UM120 */ {"MONITOR %s", PT_UNITFREQ},
    /* UM121 */ {"AT %s MONITOR %s", PT_POS_UNITFREQ},
    /* UM122 */ {"AT %s MONITOR %s", PT_TIME_UNITFREQ},
    /* UM123 */ {"SQUAWK %s", PT_BEACON},
    /* UM124 */ {"REPORT BACK ON ROUTE", PT_NULL},
    /* UM125 */ {"YOU ARE CLEARED TO LAND", PT_NULL},
    /* UM126 */ {"REPORT LEAVING %s", PT_ALT},
    /* UM127 */ {"REPORT MAINTAINING %s", PT_ALT},
    /* UM128 */ {"REPORT PASSING %s", PT_POS},
    /* UM129 */ {"REPORT REMAINING FUEL AND PERSONS ON BOARD", PT_NULL},
    /* UM130 */ {"REPORT POSITION", PT_NULL},
    /* UM131 */ {"REPORT PRESENT LEVEL", PT_NULL},
    /* UM132 */ {"CONFIRM ASSIGNED LEVEL", PT_NULL},
    /* UM133 */ {"CONFIRM ASSIGNED SPEED", PT_NULL},
    /* UM134 */ {"CONFIRM ASSIGNED ROUTE", PT_NULL},
    /* UM135 */ {"CONFIRM TIME OVER REPORTED WAYPOINT", PT_NULL},
    /* UM136 */ {"CONFIRM REPORTED WAYPOINT", PT_NULL},
    /* UM137 */ {"CONFIRM SQUAWK", PT_NULL},
    /* UM138 */ {"CONFIRM HEADING", PT_NULL},
    /* UM139 */ {"CONFIRM GROUND TRACK", PT_NULL},
    /* UM140 */ {"CONFIRM SPEED", PT_NULL},
    /* UM141 */ {"CONFIRM ALTITUDE", PT_NULL},
    /* UM142 */ {"CONFIRM ATIS CODE", PT_NULL},
    /* UM143 */ {"(reserved UM143)", PT_NULL},
    /* UM144 */ {"(reserved UM144)", PT_NULL},
    /* UM145 */ {"CONFIRM ASSIGNED ALTITUDE", PT_NULL},
    /* UM146 */ {"(reserved UM146)", PT_NULL},
    /* UM147 */ {"REQUEST POSITION REPORT", PT_NULL},
    /* UM148 */ {"WHEN CAN YOU ACCEPT %s", PT_ALT},
    /* UM149 */ {"CAN YOU ACCEPT %s AT %s", PT_ALT_TIME},
    /* UM150 */ {"CAN YOU ACCEPT %s AT %s", PT_ALT_TIME},
    /* UM151 */ {"WHEN CAN YOU ACCEPT %s", PT_SPD},
    /* UM152 */ {"WHEN CAN YOU ACCEPT %s TO %s", PT_SPD_SPD},
    /* UM153 */ {"ALTIMETER %s", PT_ALTIMETER},
    /* UM154 */ {"RADAR SERVICE TERMINATED", PT_NULL},
    /* UM155 */ {"RADAR CONTACT AT %s", PT_POS},
    /* UM156 */ {"RADAR CONTACT LOST", PT_NULL},
    /* UM157 */ {"CHECK STUCK MICROPHONE %s", PT_FREQ},
    /* UM158 */ {"ATIS %s", PT_ATIS},
    /* UM159 */ {"ERROR %s", PT_ERROR},
    /* UM160 */ {"NEXT DATA AUTHORITY %s", PT_FREETEXT},
    /* UM161 */ {"END SERVICE", PT_NULL},
    /* UM162 */ {"SERVICE UNAVAILABLE", PT_NULL},
    /* UM163 */ {"(reserved UM163)", PT_NULL},
    /* UM164 */ {"(reserved UM164)", PT_NULL},
    /* UM165 */ {"(reserved UM165)", PT_NULL},
    /* UM166 */ {"(reserved UM166)", PT_NULL},
    /* UM167 */ {"FLIGHT PLAN NOT HELD", PT_NULL},
    /* UM168 */ {"REPORT REACHING BLOCK %s TO %s", PT_ALT_ALT},
    /* UM169 */ {"FREETEXT %s", PT_FREETEXT},
    /* UM170 */ {"FREETEXT %s", PT_FREETEXT},
    /* UM171 */ {"CLIMB AT %s MINIMUM", PT_VRATE},
    /* UM172 */ {"CLIMB AT %s MAXIMUM", PT_VRATE},
    /* UM173 */ {"DESCEND AT %s MINIMUM", PT_VRATE},
    /* UM174 */ {"DESCEND AT %s MAXIMUM", PT_VRATE},
    /* UM175 */ {"REPORT REACHING %s", PT_ALT},
    /* UM176 */ {"MAINTAIN OWN SEPARATION AND VMC", PT_NULL},
    /* UM177 */ {"AT PILOTS DISCRETION", PT_NULL},
    /* UM178 */ {"(reserved UM178)", PT_NULL},
    /* UM179 */ {"SQUAWK IDENT", PT_NULL},
    /* UM180 */ {"REPORT REACHING BLOCK %s TO %s", PT_ALT_ALT},
    /* UM181 */ {"(reserved UM181)", PT_NULL},
    /* UM182 */ {"(reserved UM182)", PT_NULL},
};
#define UM_TABLE_SIZE (sizeof(um_table) / sizeof(um_table[0]))

// ========== Primitive Decoders ==========

static int32_t decode_altitude(uper_t *u, char *buf, int32_t sz) {
    int32_t c = uper_read(u, 3);
    if (c < 0) return -1;
    int32_t v;
    switch (c) {
    case 0: v=uper_read_constrained(u,0,2500); if(v<0) return -1; snprintf(buf, sz, "%dft", v*10); break;
    case 1: v=uper_read_constrained(u,0,25000); if(v<0) return -1; snprintf(buf, sz, "%dm", v); break;
    case 2: v=uper_read_constrained(u,0,2500); if(v<0) return -1; snprintf(buf, sz, "%dft(QFE)", v*10); break;
    case 3: v=uper_read_constrained(u,0,25000); if(v<0) return -1; snprintf(buf, sz, "%dm(QFE)", v); break;
    case 4: v=uper_read_constrained(u,-1000,100000); if(v==-1) return -1; snprintf(buf, sz, "%dft(GNSS)", v); break;
    case 5: v=uper_read_constrained(u,-300,30000); if(v==-1) return -1; snprintf(buf, sz, "%dm(GNSS)", v); break;
    case 6: v=uper_read_constrained(u,30,600); if(v<0) return -1; snprintf(buf, sz, "FL%d", v); break;
    case 7: v=uper_read_constrained(u,30,2000); if(v<0) return -1; snprintf(buf, sz, "FL%dM", v); break;
    default: return -1;
    }
    return 0;
}

static int32_t decode_speed(uper_t *u, char *buf, int32_t sz) {
    int32_t c = uper_read(u, 3);
    if (c < 0) return -1;
    int32_t v;
    switch (c) {
    case 0: v=uper_read_constrained(u,7,38); if(v<0) return -1; snprintf(buf, sz, "%dkt", v*10); break;
    case 1: v=uper_read_constrained(u,1,70); if(v<0) return -1; snprintf(buf, sz, "%dkmh", v*10); break;
    case 2: v=uper_read_constrained(u,7,38); if(v<0) return -1; snprintf(buf, sz, "%dktTAS", v*10); break;
    case 3: v=uper_read_constrained(u,1,70); if(v<0) return -1; snprintf(buf, sz, "%dkmhTAS", v*10); break;
    case 4: v=uper_read_constrained(u,0,2000); if(v<0) return -1; snprintf(buf, sz, "%dktGS", v); break;
    case 5: v=uper_read_constrained(u,0,4000); if(v<0) return -1; snprintf(buf, sz, "%dkmhGS", v); break;
    case 6: v=uper_read_constrained(u,61,92); if(v<0) return -1; snprintf(buf, sz, "M.%02d", v); break;
    case 7: v=uper_read_constrained(u,500,4000); if(v<0) return -1; snprintf(buf, sz, "M.%d", v); break;
    default: return -1;
    }
    return 0;
}

static int32_t decode_time(uper_t *u, char *buf, int32_t sz) {
    int32_t h = uper_read_constrained(u,0,23); if(h<0) return -1;
    int32_t m = uper_read_constrained(u,0,59); if(m<0) return -1;
    snprintf(buf, sz, "%02d:%02dZ", h, m);
    return 0;
}

static int32_t decode_frequency(uper_t *u, char *buf, int32_t sz) {
    int32_t c = uper_read(u,2); if(c<0) return -1;
    int32_t v;
    switch (c) {
    case 0: v=uper_read_constrained(u,2850,28000); if(v<0) return -1; snprintf(buf, sz, "%d.%dkHz", v/10, v%10); break;
    case 1: v=uper_read_constrained(u,117000,138000); if(v<0) return -1; snprintf(buf, sz, "%d.%03dMHz", v/1000, v%1000); break;
    case 2: v=uper_read_constrained(u,9000,15999); if(v<0) return -1; snprintf(buf, sz, "%d.%03dMHz", v/1000, v%1000); break;
    case 3: if(uper_read_ia5string(u,buf,sz,1,12)<0) return -1; break;
    default: return -1;
    }
    return 0;
}

static int32_t decode_degrees(uper_t *u, char *buf, int32_t sz) {
    int32_t c = uper_read(u,1); if(c<0) return -1;
    int32_t v = uper_read_constrained(u,1,360); if(v<0) return -1;
    snprintf(buf, sz, c==0 ? "%03d" : "%03dT", v);
    return 0;
}

static int32_t decode_position(uper_t *u, char *buf, int32_t sz) {
    int32_t c = uper_read(u,3); if(c<0) return -1;
    switch (c) {
    case 0: if(uper_read_ia5string(u,buf,sz,1,5)<0) return -1; break;
    case 1: if(uper_read_ia5string(u,buf,sz,1,4)<0) return -1; break;
    case 2: if(uper_read_ia5string(u,buf,sz,4,4)<0) return -1; break;
    case 3: {
        int32_t lat_d=uper_read_constrained(u,0,90);
        int32_t lat_m=uper_read_constrained(u,0,59);
        int32_t lat_dir=uper_read(u,1);
        int32_t lon_d=uper_read_constrained(u,0,180);
        int32_t lon_m=uper_read_constrained(u,0,59);
        int32_t lon_dir=uper_read(u,1);
        if(lat_d<0||lon_d<0) return -1;
        snprintf(buf,sz,"%02d%02d%c/%03d%02d%c",
                 lat_d,lat_m,lat_dir?'S':'N',lon_d,lon_m,lon_dir?'W':'E');
        break;
    }
    case 4: { // PlaceBearing: SEQUENCE { fixName(IA5 1..5), OPT latlon, degrees }
        int32_t opt = uper_read(u,1); if(opt<0) return -1;
        char fix[16]; if(uper_read_ia5string(u,fix,sizeof(fix),1,5)<0) return -1;
        if (opt) uper_skip(u, 7+6+1+8+6+1); // skip latlon (29b)
        char deg[16]; if(decode_degrees(u,deg,sizeof(deg))<0) return -1;
        snprintf(buf, sz, "%s/%s", fix, deg);
        break;
    }
    case 5: { // PlaceBearingDistance: same + distance
        int32_t opt = uper_read(u,1); if(opt<0) return -1;
        char fix[16]; if(uper_read_ia5string(u,fix,sizeof(fix),1,5)<0) return -1;
        if (opt) uper_skip(u, 29);
        char deg[16]; if(decode_degrees(u,deg,sizeof(deg))<0) return -1;
        int32_t dc = uper_read(u,1); if(dc<0) return -1;
        int32_t dv;
        if (dc==0) { dv=uper_read_constrained(u,0,9999); if(dv<0) return -1; snprintf(buf, sz, "%s/%s/%dNM", fix, deg, dv); }
        else { dv=uper_read_constrained(u,1,1024); if(dv<0) return -1; snprintf(buf, sz, "%s/%s/%dKM", fix, deg, dv); }
        break;
    }
    default: snprintf(buf, sz, "(pos?%d)", c); return -1;
    }
    return 0;
}

static int32_t decode_version(uper_t *u, char *buf, int32_t sz) {
    int32_t v = uper_read_constrained(u,0,15); if(v<0) return -1;
    snprintf(buf, sz, "%d", v); return 0;
}

static int32_t decode_error(uper_t *u, char *buf, int32_t sz) {
    static const char *e[] = {"unrecognizedMsgRefNum","logicalAckNotAccepted",
        "insufficientResources","invalidMsgElement","flightPlanNotHeld",
        "intentConflict","msgRefNumInUse","versionNotSupported",
        "dataLinkNotEnabled","undefinedError"};
    int32_t v = uper_read_constrained(u,0,9); if(v<0) return -1;
    snprintf(buf, sz, "%s", v<10 ? e[v] : "err?");
    return 0;
}

static int32_t decode_atis(uper_t *u, char *buf, int32_t sz) {
    return uper_read_ia5string(u,buf,sz,1,1);
}

// ========== New Decoders ==========

static int32_t decode_beacon_code(uper_t *u, char *buf, int32_t sz) {
    int32_t d0=uper_read_constrained(u,0,7); if(d0<0) return -1;
    int32_t d1=uper_read_constrained(u,0,7); if(d1<0) return -1;
    int32_t d2=uper_read_constrained(u,0,7); if(d2<0) return -1;
    int32_t d3=uper_read_constrained(u,0,7); if(d3<0) return -1;
    snprintf(buf, sz, "%d%d%d%d", d0, d1, d2, d3);
    return 0;
}

static int32_t decode_direction(uper_t *u, char *buf, int32_t sz) {
    static const char *dirs[] = {"LEFT","RIGHT","EITHER","N","S","E","W","NE","NW","SE","SW"};
    int32_t v = uper_read_constrained(u,0,10); if(v<0) return -1;
    snprintf(buf, sz, "%s", dirs[v]);
    return 0;
}

static int32_t decode_distance_offset(uper_t *u, char *buf, int32_t sz) {
    int32_t c = uper_read(u,1); if(c<0) return -1;
    if (c==0) { int32_t v=uper_read_constrained(u,1,128); if(v<0) return -1; snprintf(buf, sz, "%dNM", v); }
    else { int32_t v=uper_read_constrained(u,1,256); if(v<0) return -1; snprintf(buf, sz, "%dKM", v); }
    return 0;
}

static int32_t decode_offset(uper_t *u, char *buf, int32_t sz) {
    char dir[16], dist[16];
    if(decode_direction(u,dir,sizeof(dir))<0) return -1;
    if(decode_distance_offset(u,dist,sizeof(dist))<0) return -1;
    snprintf(buf, sz, "%s %s", dir, dist);
    return 0;
}

static int32_t decode_vertical_rate(uper_t *u, char *buf, int32_t sz) {
    int32_t c = uper_read(u,1); if(c<0) return -1;
    if (c==0) { int32_t v=uper_read_constrained(u,0,60); if(v<0) return -1; snprintf(buf, sz, "%dfpm", v*100); }
    else { int32_t v=uper_read_constrained(u,0,200); if(v<0) return -1; snprintf(buf, sz, "%dm/min", v*10); }
    return 0;
}

static int32_t decode_altimeter(uper_t *u, char *buf, int32_t sz) {
    int32_t c = uper_read(u,1); if(c<0) return -1;
    if (c==0) { int32_t v=uper_read_constrained(u,2200,3200); if(v<0) return -1; snprintf(buf, sz, "%d.%02dinHg", v/100, v%100); }
    else { int32_t v=uper_read_constrained(u,7500,12500); if(v<0) return -1; snprintf(buf, sz, "%d.%dhPa", v/10, v%10); }
    return 0;
}

static int32_t decode_runway(uper_t *u, char *buf, int32_t sz) {
    static const char *cfg[] = {"L","R","C",""};
    int32_t dir=uper_read_constrained(u,1,36); if(dir<0) return -1;
    int32_t ci=uper_read_constrained(u,0,3); if(ci<0) return -1;
    snprintf(buf, sz, "RWY%02d%s", dir, cfg[ci]);
    return 0;
}

static const char *facility_functions[] = {
    "CTR","APP","TWR","FINAL","GND","CLR","DEP","CTRL"
};

static int32_t decode_icao_unit_name(uper_t *u, char *buf, int32_t sz) {
    int32_t idc = uper_read(u,1); if(idc<0) return -1;
    char name[32];
    if (idc==0) { if(uper_read_ia5string(u,name,sizeof(name),4,4)<0) return -1; }
    else { if(uper_read_ia5string(u,name,sizeof(name),3,18)<0) return -1; }
    int32_t func = uper_read_constrained(u,0,7); if(func<0) return -1;
    snprintf(buf, sz, "%s %s", name, facility_functions[func]);
    return 0;
}

static int32_t decode_unit_name_freq(uper_t *u, char *buf, int32_t sz) {
    char unit[64], freq[32];
    if(decode_icao_unit_name(u,unit,sizeof(unit))<0) return -1;
    if(decode_frequency(u,freq,sizeof(freq))<0) return -1;
    snprintf(buf, sz, "%s %s", unit, freq);
    return 0;
}

static int32_t decode_procedure_name(uper_t *u, char *buf, int32_t sz) {
    static const char *ptypes[] = {"ARR","APP","DEP"};
    int32_t opt = uper_read(u,1); if(opt<0) return -1;
    int32_t pt = uper_read_constrained(u,0,2); if(pt<0) return -1;
    char proc[16]; if(uper_read_ia5string(u,proc,sizeof(proc),1,6)<0) return -1;
    if (opt) {
        char trans[16]; if(uper_read_ia5string(u,trans,sizeof(trans),1,5)<0) return -1;
        snprintf(buf, sz, "%s %s.%s", ptypes[pt], proc, trans);
    } else {
        snprintf(buf, sz, "%s %s", ptypes[pt], proc);
    }
    return 0;
}

static int32_t decode_distance(uper_t *u, char *buf, int32_t sz) {
    int32_t c = uper_read(u,1); if(c<0) return -1;
    if (c==0) { int32_t v=uper_read_constrained(u,0,9999); if(v<0) return -1; snprintf(buf, sz, "%dNM", v); }
    else { int32_t v=uper_read_constrained(u,1,1024); if(v<0) return -1; snprintf(buf, sz, "%dKM", v); }
    return 0;
}

static int32_t decode_temperature(uper_t *u, char *buf, int32_t sz) {
    int32_t c = uper_read(u,1); if(c<0) return -1;
    if (c==0) { int32_t v=uper_read_constrained(u,-80,47); if(v==-81) return -1; snprintf(buf, sz, "%dC", v); }
    else { int32_t v=uper_read_constrained(u,-105,150); if(v==-106) return -1; snprintf(buf, sz, "%dF", v); }
    return 0;
}

static int32_t decode_winds(uper_t *u, char *buf, int32_t sz) {
    int32_t dir = uper_read_constrained(u,1,360); if(dir<0) return -1;
    int32_t sc = uper_read(u,1); if(sc<0) return -1;
    if (sc==0) { int32_t v=uper_read_constrained(u,0,255); if(v<0) return -1; snprintf(buf, sz, "%03d/%dkt", dir, v); }
    else { int32_t v=uper_read_constrained(u,0,511); if(v<0) return -1; snprintf(buf, sz, "%03d/%dkmh", dir, v); }
    return 0;
}

static int32_t decode_vertical_change(uper_t *u, char *buf, int32_t sz) {
    int32_t vdir = uper_read(u,1); if(vdir<0) return -1;
    char rate[16]; if(decode_vertical_rate(u,rate,sizeof(rate))<0) return -1;
    snprintf(buf, sz, "%s%s", vdir?"DN":"UP", rate);
    return 0;
}

static int32_t decode_position_report(uper_t *u, char *buf, int32_t sz) {
    int32_t opt = uper_read(u,19); if(opt<0) return -1;
    char pos[64], time[16], alt[32];
    if(decode_position(u,pos,sizeof(pos))<0) return -1;
    if(decode_time(u,time,sizeof(time))<0) return -1;
    if(decode_altitude(u,alt,sizeof(alt))<0) return -1;
    int32_t n = snprintf(buf, sz, "%s %s %s", pos, time, alt);
    if (opt & (1<<18)) { char t[64]; if(decode_position(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " NEXT:%s", t); }
    if (opt & (1<<17)) { char t[16]; if(decode_time(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " ETA:%s", t); }
    if (opt & (1<<16)) { char t[64]; if(decode_position(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " NEXT+1:%s", t); }
    if (opt & (1<<15)) { char t[16]; if(decode_time(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " ETADST:%s", t); }
    if (opt & (1<<14)) { char t[16]; if(decode_time(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " FUEL:%s", t); }
    if (opt & (1<<13)) { char t[16]; if(decode_temperature(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " T:%s", t); }
    if (opt & (1<<12)) { char t[32]; if(decode_winds(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " WIND:%s", t); }
    if (opt & (1<<11)) { static const char *tb[]={"LIGHT","MOD","SEVERE"}; int32_t v=uper_read_constrained(u,0,2); if(v>=0&&n<sz) n+=snprintf(buf+n, sz-n, " TURB:%s", tb[v]); }
    if (opt & (1<<10)) { static const char *ic[]={"TRACE","LIGHT","MOD","SEVERE"}; int32_t v=uper_read_constrained(u,0,3); if(v>=0&&n<sz) n+=snprintf(buf+n, sz-n, " ICE:%s", ic[v]); }
    if (opt & (1<<9))  { char t[32]; if(decode_speed(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " SPD:%s", t); }
    if (opt & (1<<8))  { int32_t v=uper_read_constrained(u,7,70); if(v>=0&&n<sz) n+=snprintf(buf+n, sz-n, " GS:%dkt", v*10); }
    if (opt & (1<<7))  { char t[32]; if(decode_vertical_change(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " VS:%s", t); }
    if (opt & (1<<6))  { char t[16]; if(decode_degrees(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " TRK:%s", t); }
    if (opt & (1<<5))  { char t[16]; if(decode_degrees(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " HDG:%s", t); }
    if (opt & (1<<4))  { char t[32]; if(decode_distance(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " DIST:%s", t); }
    if (opt & (1<<3))  { char t[260]; if(uper_read_ia5string(u,t,sizeof(t),1,256)>=0&&n<sz) n+=snprintf(buf+n, sz-n, " INFO:%s", t); }
    if (opt & (1<<2))  { char t[64]; if(decode_position(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " RWPT:%s", t); }
    if (opt & (1<<1))  { char t[16]; if(decode_time(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " RWPTT:%s", t); }
    if (opt & (1<<0))  { char t[32]; if(decode_altitude(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " RWPTA:%s", t); }
    return 0;
}

static int32_t decode_route_info(uper_t *u, char *buf, int32_t sz) {
    int32_t c = uper_read(u, 3); if (c < 0) return -1;
    switch (c) {
    case 0: { // publishedIdentifier: fixName(IA5 1..5) + OPT latlon
        int32_t opt = uper_read(u, 1); if (opt < 0) return -1;
        if (uper_read_ia5string(u, buf, sz, 1, 5) < 0) return -1;
        if (opt) uper_skip(u, 29);
        break;
    }
    case 1: { // latitudeLongitude
        int32_t la_d=uper_read_constrained(u,0,90), la_m=uper_read_constrained(u,0,59), la_s=uper_read(u,1);
        int32_t lo_d=uper_read_constrained(u,0,180), lo_m=uper_read_constrained(u,0,59), lo_s=uper_read(u,1);
        if (la_d<0||lo_d<0) return -1;
        snprintf(buf, sz, "%02d%02d%c/%03d%02d%c", la_d, la_m, la_s?'S':'N', lo_d, lo_m, lo_s?'W':'E');
        break;
    }
    case 2: { // placeBearingPlaceBearing: SIZE(2..2) x PlaceBearing{fix,OPT latlon,deg}
        char f1[8],f2[8],d1[16],d2[16];
        int32_t o1=uper_read(u,1); if(o1<0) return -1;
        if(uper_read_ia5string(u,f1,sizeof(f1),1,5)<0) return -1;
        if(o1) uper_skip(u,29);
        if(decode_degrees(u,d1,sizeof(d1))<0) return -1;
        int32_t o2=uper_read(u,1); if(o2<0) return -1;
        if(uper_read_ia5string(u,f2,sizeof(f2),1,5)<0) return -1;
        if(o2) uper_skip(u,29);
        if(decode_degrees(u,d2,sizeof(d2))<0) return -1;
        snprintf(buf, sz, "%s/%s-%s/%s", f1, d1, f2, d2);
        break;
    }
    case 3: { // placeBearingDistance
        int32_t opt=uper_read(u,1); if(opt<0) return -1;
        char fix[8]; if(uper_read_ia5string(u,fix,sizeof(fix),1,5)<0) return -1;
        if(opt) uper_skip(u,29);
        char deg[16]; if(decode_degrees(u,deg,sizeof(deg))<0) return -1;
        int32_t dc=uper_read(u,1); if(dc<0) return -1;
        int32_t dv;
        if(dc==0){dv=uper_read_constrained(u,0,9999);if(dv<0)return -1;snprintf(buf, sz, "%s/%s/%dNM", fix, deg, dv);}
        else{dv=uper_read_constrained(u,1,1024);if(dv<0)return -1;snprintf(buf, sz, "%s/%s/%dKM", fix, deg, dv);}
        break;
    }
    case 4: // airwayIdentifier (IA5 1..5)
        if (uper_read_ia5string(u, buf, sz, 1, 5) < 0) return -1;
        break;
    case 5: { // trackDetail: trackName(IA5 3..6) + LatLonSeq(1..128)
        char name[8]; if(uper_read_ia5string(u,name,sizeof(name),3,6)<0) return -1;
        int32_t cnt=uper_read_constrained(u,1,128); if(cnt<0) return -1;
        int32_t n=snprintf(buf, sz, "TRK:%s", name);
        for(int32_t i=0;i<cnt&&i<4;i++){
            int32_t la_d=uper_read_constrained(u,0,90),la_m=uper_read_constrained(u,0,59),la_s=uper_read(u,1);
            int32_t lo_d=uper_read_constrained(u,0,180),lo_m=uper_read_constrained(u,0,59),lo_s=uper_read(u,1);
            if(la_d<0||lo_d<0) return -1;
            if(n<sz) n+=snprintf(buf+n, sz-n, " %02d%02d%c/%03d%02d%c", la_d, la_m, la_s?'S':'N', lo_d, lo_m, lo_s?'W':'E');
        }
        for(int32_t i=4;i<cnt;i++) uper_skip(u,29);
        if(cnt>4&&n<sz) n+=snprintf(buf+n, sz-n, " +%d", cnt-4);
        break;
    }
    default: snprintf(buf, sz, "(rte?%d)", c); return -1;
    }
    return 0;
}

static int32_t decode_route_info_seq(uper_t *u, char *buf, int32_t sz) {
    int32_t cnt = uper_read_constrained(u, 1, 128); if (cnt < 0) return -1;
    int32_t n = 0;
    for (int32_t i = 0; i < cnt; i++) {
        char ri[128];
        if (decode_route_info(u, ri, sizeof(ri)) < 0) {
            if (n < sz) n += snprintf(buf+n, sz-n, "%s(?)", n ? " " : "");
            break;
        }
        if (n < sz) n += snprintf(buf+n, sz-n, "%s%s", n ? " " : "", ri);
    }
    if (n == 0) snprintf(buf, sz, "(empty)");
    return 0;
}

static int32_t decode_route_clearance(uper_t *u, char *buf, int32_t sz) {
    int32_t opt = uper_read(u,10); if(opt<0) return -1;
    int32_t n = 0; buf[0] = '\0';
    if (opt & (1<<9)) { char t[8]; if(uper_read_ia5string(u,t,sizeof(t),4,4)>=0) n+=snprintf(buf+n, sz-n, "%sDEP:%s", n?" ":"", t); }
    if (opt & (1<<8)) { char t[8]; if(uper_read_ia5string(u,t,sizeof(t),4,4)>=0) n+=snprintf(buf+n, sz-n, "%sDST:%s", n?" ":"", t); }
    if (opt & (1<<7)) { char t[16]; if(decode_runway(u,t,sizeof(t))==0) n+=snprintf(buf+n, sz-n, "%sRWYDEP:%s", n?" ":"", t); }
    if (opt & (1<<6)) { char t[32]; if(decode_procedure_name(u,t,sizeof(t))==0) n+=snprintf(buf+n, sz-n, "%sPROCDEP:%s", n?" ":"", t); }
    if (opt & (1<<5)) { char t[16]; if(decode_runway(u,t,sizeof(t))==0) n+=snprintf(buf+n, sz-n, "%sRWYARR:%s", n?" ":"", t); }
    if (opt & (1<<4)) { char t[32]; if(decode_procedure_name(u,t,sizeof(t))==0) n+=snprintf(buf+n, sz-n, "%sPROCAPP:%s", n?" ":"", t); }
    if (opt & (1<<3)) { char t[32]; if(decode_procedure_name(u,t,sizeof(t))==0) n+=snprintf(buf+n, sz-n, "%sPROCARR:%s", n?" ":"", t); }
    if (opt & (1<<2)) { char t[8]; if(uper_read_ia5string(u,t,sizeof(t),1,5)>=0) n+=snprintf(buf+n, sz-n, "%sAWY:%s", n?" ":"", t); }
    if (opt & (1<<1)) { char t[256]; if(decode_route_info_seq(u,t,sizeof(t))==0) n+=snprintf(buf+n, sz-n, "%sROUTE:%s", n?" ":"", t); else if(n<sz) n+=snprintf(buf+n,sz-n,"%sROUTE:(?)",n?" ":""); }
    if (opt & (1<<0)) { int32_t ao=uper_read(u,6); if(ao>=0&&n<sz) n+=snprintf(buf+n, sz-n, "%sADD:%02X", n?" ":"", (uint32_t)ao); }
    if (n==0) snprintf(buf, sz, "(empty)");
    return 0;
}

static int32_t decode_hold_at_waypoint(uper_t *u, char *buf, int32_t sz) {
    int32_t opt = uper_read(u,7); if(opt<0) return -1;
    char pos[64]; if(decode_position(u,pos,sizeof(pos))<0) return -1;
    int32_t n = snprintf(buf, sz, "%s", pos);
    if (opt & (1<<6)) { char t[32]; if(decode_speed(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " SPDLO:%s", t); }
    if (opt & (1<<5)) { char t[32]; if(decode_altitude(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " ALT:%s", t); }
    if (opt & (1<<4)) { char t[32]; if(decode_speed(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " SPDHI:%s", t); }
    if (opt & (1<<3)) { char t[16]; if(decode_direction(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " %sTURNS", t); }
    if (opt & (1<<2)) { char t[16]; if(decode_degrees(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " INBD:%s", t); }
    if (opt & (1<<1)) { char t[16]; if(decode_time(u,t,sizeof(t))==0&&n<sz) n+=snprintf(buf+n, sz-n, " EFC:%s", t); }
    if (opt & (1<<0)) {
        int32_t lc = uper_read(u,1);
        if (lc==0) { int32_t dc=uper_read(u,1); int32_t dv;
            if(dc==0){dv=uper_read_constrained(u,1,999);if(dv>=0&&n<sz) n+=snprintf(buf+n, sz-n, " LEG:%dNM", dv);}
            else{dv=uper_read_constrained(u,1,128);if(dv>=0&&n<sz) n+=snprintf(buf+n, sz-n, " LEG:%dKM", dv);}
        } else { int32_t v=uper_read_constrained(u,1,99); if(v>=0&&n<sz) n+=snprintf(buf+n, sz-n, " LEG:%dmin", v); }
    }
    return 0;
}

static int32_t decode_fuel_persons(uper_t *u, char *buf, int32_t sz) {
    int32_t h = uper_read_constrained(u,0,23); if(h<0) return -1;
    int32_t m = uper_read_constrained(u,0,59); if(m<0) return -1;
    int32_t pob = uper_read_constrained(u,1,1024); if(pob<0) return -1;
    snprintf(buf, sz, "%02d:%02d FUEL %d POB", h, m, pob);
    return 0;
}

// ========== Master Decode Dispatcher ==========
// Takes the format string, decodes params, and fills buf with COMPLETE formatted text.

static int32_t decode_param(uper_t *u, param_type_t ptype, const char *fmt, char *buf, int32_t bufsize) {
    char t1[128], t2[128], t3[128];
    switch (ptype) {
    case PT_NULL:
        snprintf(buf, bufsize, "%s", fmt);
        return 0;
    case PT_ALT:
        if(decode_altitude(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_SPD:
        if(decode_speed(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_TIME:
        if(decode_time(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_POS:
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_FREETEXT:
        if(uper_read_ia5string(u,t1,sizeof(t1),1,256)<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_FREQ:
        if(decode_frequency(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_DEG:
        if(decode_degrees(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_VERSION:
        if(decode_version(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_ERROR:
        if(decode_error(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_ATIS:
        if(decode_atis(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_BEACON:
        if(decode_beacon_code(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_OFFSET:
        if(decode_offset(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_PROCNAME:
        if(decode_procedure_name(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_ROUTECLR:
        if(decode_route_clearance(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_UNITFREQ:
        if(decode_unit_name_freq(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_VRATE:
        if(decode_vertical_rate(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_ALTIMETER:
        if(decode_altimeter(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_POS_RPT:
        if(decode_position_report(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_FUEL_PERSONS:
        if(decode_fuel_persons(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    case PT_HOLD:
        if(decode_hold_at_waypoint(u,t1,sizeof(t1))<0) return -1;
        snprintf(buf, bufsize, fmt, t1); return 0;
    // Two-parameter types
    case PT_ALT_ALT:
        if(decode_altitude(u,t1,sizeof(t1))<0) return -1;
        if(decode_altitude(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    case PT_SPD_SPD:
        if(decode_speed(u,t1,sizeof(t1))<0) return -1;
        if(decode_speed(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    case PT_ALT_TIME:
        if(decode_time(u,t1,sizeof(t1))<0) return -1;
        if(decode_altitude(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    case PT_POS_ALT:
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        if(decode_altitude(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    case PT_POS_SPD:
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        if(decode_speed(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    case PT_POS_TIME:
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        if(decode_time(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    case PT_POS_DEG:
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        if(decode_degrees(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    case PT_POS_OFFSET:
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        if(decode_offset(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    case PT_POS_UNITFREQ:
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        if(decode_unit_name_freq(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    case PT_TIME_UNITFREQ:
        if(decode_time(u,t1,sizeof(t1))<0) return -1;
        if(decode_unit_name_freq(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    case PT_HOLD2: // pos + alt
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        if(decode_altitude(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    // Three-parameter types
    case PT_POS_TIME_ALT:
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        if(decode_time(u,t2,sizeof(t2))<0) return -1;
        if(decode_altitude(u,t3,sizeof(t3))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2, t3); return 0;
    case PT_POS_ALT_ALT:
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        if(decode_altitude(u,t2,sizeof(t2))<0) return -1;
        if(decode_altitude(u,t3,sizeof(t3))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2, t3); return 0;
    case PT_POS_TIME_TIME:
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        if(decode_time(u,t2,sizeof(t2))<0) return -1;
        if(decode_time(u,t3,sizeof(t3))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2, t3); return 0;
    case PT_POS_ALT_SPD:
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        if(decode_altitude(u,t2,sizeof(t2))<0) return -1;
        if(decode_speed(u,t3,sizeof(t3))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2, t3); return 0;
    case PT_POS_SPD_ALT:
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        if(decode_speed(u,t2,sizeof(t2))<0) return -1;
        if(decode_altitude(u,t3,sizeof(t3))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2, t3); return 0;
    case PT_TIME_POS_ALT:
        if(decode_time(u,t1,sizeof(t1))<0) return -1;
        if(decode_position(u,t2,sizeof(t2))<0) return -1;
        if(decode_altitude(u,t3,sizeof(t3))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2, t3); return 0;
    case PT_POS_POS_ALT:
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        if(decode_position(u,t2,sizeof(t2))<0) return -1;
        if(decode_altitude(u,t3,sizeof(t3))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2, t3); return 0;
    case PT_TIME_POS:
        if(decode_time(u,t1,sizeof(t1))<0) return -1;
        if(decode_position(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    case PT_POS_POS:
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        if(decode_position(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    case PT_ALT_POS:
        if(decode_altitude(u,t1,sizeof(t1))<0) return -1;
        if(decode_position(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    case PT_POS_ROUTECLR:
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        if(decode_route_clearance(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    case PT_POS_PROCNAME:
        if(decode_position(u,t1,sizeof(t1))<0) return -1;
        if(decode_procedure_name(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    case PT_ALT_SPD:
        if(decode_altitude(u,t1,sizeof(t1))<0) return -1;
        if(decode_speed(u,t2,sizeof(t2))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2); return 0;
    case PT_TIME_SPD_SPD:
        if(decode_time(u,t1,sizeof(t1))<0) return -1;
        if(decode_speed(u,t2,sizeof(t2))<0) return -1;
        if(decode_speed(u,t3,sizeof(t3))<0) return -1;
        snprintf(buf, bufsize, fmt, t1, t2, t3); return 0;
    case PT_COMPLEX:
    default:
        snprintf(buf, bufsize, "%s (...)", fmt);
        return 0;
    }
}

// ========== Top-Level Message Decoder ==========

static int32_t try_decode_message(uint32_t addr, const uint8_t *data, int32_t len,
                              int32_t is_uplink) {
    uper_t u;
    uper_init(&u, data, len);

    const char *dir = is_uplink ? "UP" : "DN";
    int32_t max_choice = is_uplink ? 182 : 128;
    const msg_element_t *table = is_uplink ? um_table : dm_table;
    int32_t table_size = is_uplink ? (int32_t)UM_TABLE_SIZE : (int32_t)DM_TABLE_SIZE;

    int32_t opt_bitmap = uper_read(&u, 2);
    if (opt_bitmap < 0) return 0;
    int32_t has_msgref = (opt_bitmap >> 1) & 1;
    int32_t has_timestamp = opt_bitmap & 1;

    int32_t msg_id = uper_read_constrained(&u, 0, 63);
    if (msg_id < 0) return 0;

    int32_t msg_ref = -1;
    if (has_msgref) {
        msg_ref = uper_read_constrained(&u, 0, 63);
        if (msg_ref < 0) return 0;
    }

    int32_t ts_year=-1, ts_month=-1, ts_day=-1, ts_hour=-1, ts_min=-1;
    if (has_timestamp) {
        ts_year = uper_read_constrained(&u, 0, 99);
        ts_month = uper_read_constrained(&u, 1, 12);
        ts_day = uper_read_constrained(&u, 1, 31);
        ts_hour = uper_read_constrained(&u, 0, 23);
        ts_min = uper_read_constrained(&u, 0, 59);
        if (ts_year<0 || ts_month<0 || ts_day<0 || ts_hour<0 || ts_min<0)
            return 0;
    }

    int32_t elem_count = uper_read_constrained(&u, 0, 4);
    if (elem_count < 0) return 0;
    elem_count += 1;
    if (elem_count > 5) return 0;
    if (uper_bits_left(&u) < elem_count * 8) return 0;

    gg::print("CPDLC %s %06X MID=%d", dir, addr, msg_id);
    if (msg_ref >= 0) gg::print(" REF=%d", msg_ref);
    if (has_timestamp)
        printf(" TS=%02d-%02d-%02d %02d:%02dZ",
               2000+ts_year, ts_month, ts_day, ts_hour, ts_min);
    gg::print(": ");

    for (int32_t i = 0; i < elem_count; i++) {
        if (i > 0) gg::print("; ");

        int32_t elem_idx = uper_read_constrained(&u, 0, max_choice);
        if (elem_idx < 0) { gg::print("(decode error at element %d)", i); break; }

        const char *pfx = is_uplink ? "UM" : "DM";
        if (elem_idx < table_size && table[elem_idx].text) {
            const msg_element_t *elem = &table[elem_idx];
            char param_buf[512];
            if (decode_param(&u, elem->ptype, elem->text, param_buf, sizeof(param_buf)) < 0)
                gg::print("%s%d %s (?)", pfx, elem_idx, elem->text);
            else
                gg::print("%s%d %s", pfx, elem_idx, param_buf);
        } else {
            gg::print("%s%d", pfx, elem_idx);
        }
    }

    gg::print("\n");
    return 1;
}

// ========== Public API ==========

int32_t cpdlc_try_decode(uint32_t addr, const uint8_t *data, int32_t len) {
    if (!data || len < 3) return 0;

    int32_t decoded = try_decode_message(addr, data, len, 0);
    if (!decoded)
        decoded = try_decode_message(addr, data, len, 1);

    return decoded;
}

int32_t cpdlc_try_decode_dir(uint32_t addr, const uint8_t *data, int32_t len, int32_t dir) {
    if (!data || len < 3) return 0;

    if (dir == 0)
        return try_decode_message(addr, data, len, 0);
    else if (dir == 1)
        return try_decode_message(addr, data, len, 1);
    else
        return cpdlc_try_decode(addr, data, len);
}
