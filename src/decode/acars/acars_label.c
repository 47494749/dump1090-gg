// Part of dump1090-gg-light
//
// acars_label.c: ACARS label semantic lookup table
//
// Label definitions derived from ARINC 618/620 standards and
// publicly available ACARS label references (airframes.io, ARINC specs).
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#include <string.h>
#include "acars_label.h"

// ======================== Label table ========================
// Sorted by label for binary search

typedef struct {
    char              label[3];
    const char       *description;
    acars_category_t  category;
} label_entry_t;

static const label_entry_t label_table[] = {
    // Emergency
    {"00", "Emergency Situation Report",                 ACARS_CAT_EMERGENCY},
    {"99", "Emergency Situation Report",                 ACARS_CAT_EMERGENCY},

    // General aviation / AOC
    {"10", "Airline Operational",                        ACARS_CAT_AOC},
    {"11", "In Range Arrival Report",                    ACARS_CAT_AOC},
    {"13", "CMU Loadsheet Uplink",                       ACARS_CAT_AOC},
    {"14", "General Aviation Free Text",                 ACARS_CAT_AOC},
    {"15", "Position Report",                            ACARS_CAT_AOC},
    {"16", "Weather Request",                            ACARS_CAT_WEATHER},
    {"17", "Position/Weather Report",                    ACARS_CAT_AOC},
    {"18", "Weather Report",                             ACARS_CAT_WEATHER},
    {"19", "ATIS Report",                                ACARS_CAT_ATC},
    {"1C", "Flight Plan Request",                        ACARS_CAT_AOC},
    {"1M", "ETA Report",                                 ACARS_CAT_AOC},

    // Initialization / Loadsheet
    {"20", "Initialization",                             ACARS_CAT_AOC},
    {"21", "Takeoff Data Calculation",                   ACARS_CAT_AOC},
    {"23", "Loadsheet Uplink",                           ACARS_CAT_AOC},
    {"24", "Weather Request",                            ACARS_CAT_WEATHER},
    {"25", "Weather Request",                            ACARS_CAT_WEATHER},
    {"26", "SIGMET Message",                             ACARS_CAT_WEATHER},
    {"27", "Weather Request",                            ACARS_CAT_WEATHER},
    {"28", "Weather Request",                            ACARS_CAT_WEATHER},
    {"29", "Flow Message",                               ACARS_CAT_ATC},
    {"2S", "Weather Request",                            ACARS_CAT_WEATHER},
    {"2U", "Weather",                                    ACARS_CAT_WEATHER},

    // Position / Weather reports
    {"30", "Position Report",                            ACARS_CAT_AOC},
    {"31", "Weather Report",                             ACARS_CAT_WEATHER},
    {"36", "In Range Message",                           ACARS_CAT_AOC},
    {"39", "Arrival Free Text Message",                  ACARS_CAT_AOC},
    {"3F", "ETA Downlink Message",                       ACARS_CAT_AOC},
    {"3G", "Free Text Message Format",                   ACARS_CAT_AOC},
    {"3S", "Downlink Message",                           ACARS_CAT_AOC},
    {"3U", "Uplink Acknowledgement",                     ACARS_CAT_AOC},

    // Flow / Status
    {"41", "Flow Message",                               ACARS_CAT_ATC},
    {"46", "Pilot Report",                               ACARS_CAT_AOC},
    {"47", "Airway Position Downlink",                   ACARS_CAT_AOC},
    {"48", "Miscellaneous Messages",                     ACARS_CAT_AOC},
    {"49", "Status Reports",                             ACARS_CAT_AOC},
    {"4A", "Latest New Format",                          ACARS_CAT_AOC},
    {"4M", "Cargo Information",                          ACARS_CAT_AAC},
    {"4P", "Weather Report / Forecast",                  ACARS_CAT_WEATHER},
    {"4Q", "Departure Format",                           ACARS_CAT_AOC},
    {"4R", "Off Report",                                 ACARS_CAT_AOC},
    {"4S", "Weather Report",                             ACARS_CAT_WEATHER},
    {"4T", "ETA Report",                                 ACARS_CAT_AOC},
    {"4X", "Frequency List",                             ACARS_CAT_AOC},

    // HFDL / System
    {"50", "HFDL Message Router",                        ACARS_CAT_SERVICE},
    {"51", "Ground GMT Request/Response",                ACARS_CAT_SERVICE},
    {"52", "Ground UTC Request/Response",                ACARS_CAT_SERVICE},
    {"53", "Reserved",                                   ACARS_CAT_SERVICE},
    {"54", "Aircrew Voice Contact Request",              ACARS_CAT_AOC},
    {"57", "Alternate Position Report",                  ACARS_CAT_AOC},
    {"58", "Aircraft Tracking Control",                  ACARS_CAT_AOC},
    {"5D", "ATIS Request",                               ACARS_CAT_ATC},
    {"5P", "Temporary Suspension of ACARS",              ACARS_CAT_SERVICE},
    {"5R", "Aircraft Initiated Position Report",         ACARS_CAT_AOC},
    {"5U", "Downlink Weather Request",                   ACARS_CAT_WEATHER},
    {"5V", "VDL Switch Advisory",                        ACARS_CAT_SERVICE},
    {"5Y", "ETA Revision / Diversion",                   ACARS_CAT_AOC},
    {"5Z", "Airline Designated Downlink",                ACARS_CAT_AOC},

    // Engine / Aircrew
    {"7A", "Aircraft Initiated Engine Data",             ACARS_CAT_AOC},
    {"7B", "Aircrew Initiated Messages",                 ACARS_CAT_AOC},

    // Airline defined (80-89)
    {"80", "Airline Defined",                            ACARS_CAT_AOC},
    {"81", "Airline Defined",                            ACARS_CAT_AOC},
    {"82", "Airline Defined",                            ACARS_CAT_AOC},
    {"83", "Airline Defined",                            ACARS_CAT_AOC},
    {"84", "Airline Defined",                            ACARS_CAT_AOC},
    {"85", "Airline Defined",                            ACARS_CAT_AOC},
    {"86", "Airline Defined",                            ACARS_CAT_AOC},
    {"87", "Airline Defined",                            ACARS_CAT_AOC},
    {"88", "Airline Defined",                            ACARS_CAT_AOC},
    {"89", "Airline Defined",                            ACARS_CAT_AOC},
    {"8A", "Airline Defined / Out Report",               ACARS_CAT_AOC},
    {"8B", "Airline Defined / Off Report",               ACARS_CAT_AOC},
    {"8C", "Airline Defined / On Report",                ACARS_CAT_AOC},
    {"8D", "Airline Defined / In Report",                ACARS_CAT_AOC},
    {"8E", "Airline Defined / Out-Return-In Report",     ACARS_CAT_AOC},
    {"8G", "Airline Defined / Takeoff Data",             ACARS_CAT_AOC},
    {"8H", "Airline Defined / Loadsheet Req",            ACARS_CAT_AOC},
    {"8I", "Airline Defined / Flightplan Req",           ACARS_CAT_AOC},
    {"8J", "Airline Defined / Crewlist Req",             ACARS_CAT_AAC},
    {"8X", "Uplink ATIS Information",                    ACARS_CAT_ATC},
    {"8Z", "Avionics Unable to Process Data",            ACARS_CAT_SERVICE},

    // Addressed Downlinks
    {"90", "Aircraft Addressed Downlinks",               ACARS_CAT_AOC},

    // ATC clearances (A0-AF uplink, B0-BG downlink)
    {"A1", "Deliver Oceanic Clearance",                  ACARS_CAT_ATC},
    {"A2", "Deliver Departure Clearance",                ACARS_CAT_ATC},
    {"A3", "Departure Clearance",                        ACARS_CAT_ATC},
    {"A4", "Acknowledge PDC",                            ACARS_CAT_ATC},
    {"A5", "Request Position Report",                    ACARS_CAT_ATC},
    {"A6", "Request ADS Reports",                        ACARS_CAT_ATC},
    {"A7", "Free Text from ATC",                         ACARS_CAT_ATC},
    {"A8", "Deliver Departure Slot",                     ACARS_CAT_ATC},
    {"A9", "Deliver ATIS Information",                   ACARS_CAT_ATC},
    {"AA", "ATC Communication",                          ACARS_CAT_ATC},
    {"AB", "Terminal Weather Info for Pilots",            ACARS_CAT_WEATHER},
    {"AC", "Pushback Clearance Uplink",                  ACARS_CAT_ATC},
    {"AD", "Expected Taxi Clearance",                    ACARS_CAT_ATC},
    {"AF", "CPC Command Response",                       ACARS_CAT_ATC},

    {"B0", "ATS Facility Notification",                  ACARS_CAT_ATC},
    {"B1", "Request Oceanic Clearance",                  ACARS_CAT_ATC},
    {"B2", "Oceanic Clearance Readback",                 ACARS_CAT_ATC},
    {"B3", "Request Departure Clearance",                ACARS_CAT_ATC},
    {"B4", "Departure Clearance Readback",               ACARS_CAT_ATC},
    {"B5", "Waypoint Position Report",                   ACARS_CAT_ATC},
    {"B6", "Provide ADS Report",                         ACARS_CAT_ATC},
    {"B7", "Free Text to ATC",                           ACARS_CAT_ATC},
    {"B8", "Request Departure Slot",                     ACARS_CAT_ATC},
    {"B9", "Request ATIS Report",                        ACARS_CAT_ATC},
    {"BA", "ATC Communication",                          ACARS_CAT_ATC},
    {"BB", "Terminal Weather Info for Pilots",            ACARS_CAT_WEATHER},
    {"BC", "Pushback Clearance Request",                 ACARS_CAT_ATC},
    {"BD", "Expected Taxi Clearance Request",            ACARS_CAT_ATC},
    {"BE", "CPC Logon/Logoff Request",                   ACARS_CAT_ATC},
    {"BF", "CPC WILCO",                                  ACARS_CAT_ATC},

    // Cockpit printer (C0-CF)
    {"C0", "Cockpit Printer Message",                    ACARS_CAT_PRINTER},
    {"C1", "Cockpit Printer Message",                    ACARS_CAT_PRINTER},
    {"C2", "Cockpit Printer Message",                    ACARS_CAT_PRINTER},
    {"C3", "Cockpit Printer Message",                    ACARS_CAT_PRINTER},
    {"C4", "Cockpit Printer Message",                    ACARS_CAT_PRINTER},
    {"C5", "Cockpit Printer Message",                    ACARS_CAT_PRINTER},
    {"C6", "Cockpit Printer Message",                    ACARS_CAT_PRINTER},
    {"C7", "Cockpit Printer Message",                    ACARS_CAT_PRINTER},
    {"C8", "Cockpit Printer Message",                    ACARS_CAT_PRINTER},
    {"C9", "Cockpit Printer Message",                    ACARS_CAT_PRINTER},
    {"CA", "Cockpit Printer Message",                    ACARS_CAT_PRINTER},
    {"CB", "Printer Busy",                               ACARS_CAT_PRINTER},
    {"CC", "Printer in Local or Test Mode",              ACARS_CAT_PRINTER},
    {"CD", "Printer Out of Paper",                       ACARS_CAT_PRINTER},
    {"CE", "Printer Buffer Overrun",                     ACARS_CAT_PRINTER},
    {"CF", "Printer Init Before Completion",             ACARS_CAT_PRINTER},

    // Special
    {"EI", "Internet Email Message",                     ACARS_CAT_AOC},
    {"F3", "Dedicated Transceiver Advisory",             ACARS_CAT_SERVICE},
    {"H1", "Message To/From Terminal",                   ACARS_CAT_AOC},
    {"H2", "Meteorological Report",                      ACARS_CAT_WEATHER},
    {"H3", "Icing Report",                               ACARS_CAT_WEATHER},
    {"HF", "HFDL Messages",                              ACARS_CAT_SERVICE},
    {"HX", "Undelivered Uplink Report",                  ACARS_CAT_SERVICE},
    {"M2", "User Defined Message",                       ACARS_CAT_AOC},

    // OOOI / Fuel reports (Q-labels)
    {"Q0", "ACARS Link Test",                            ACARS_CAT_SERVICE},
    {"Q1", "ETA Departure/Arrival Report",               ACARS_CAT_AOC},
    {"Q2", "ETA Report",                                 ACARS_CAT_AOC},
    {"Q3", "Clock Update Advisory",                      ACARS_CAT_SERVICE},
    {"Q4", "Voice Circuit Busy",                         ACARS_CAT_SERVICE},
    {"Q5", "Unable to Deliver Uplink",                   ACARS_CAT_SERVICE},
    {"Q6", "Voice-to-Data Changeover Advisory",          ACARS_CAT_SERVICE},
    {"Q7", "Delay Message",                              ACARS_CAT_AOC},
    {"QA", "OUT Fuel Report",                            ACARS_CAT_AOC},
    {"QB", "OFF Report",                                 ACARS_CAT_AOC},
    {"QC", "ON Report",                                  ACARS_CAT_AOC},
    {"QD", "IN Fuel Report",                             ACARS_CAT_AOC},
    {"QE", "OUT Fuel/Destination Report",                ACARS_CAT_AOC},
    {"QF", "OFF Destination Report",                     ACARS_CAT_AOC},
    {"QG", "OUT Return In Report",                       ACARS_CAT_AOC},
    {"QH", "OUT Report",                                 ACARS_CAT_AOC},
    {"QK", "Landing Report",                             ACARS_CAT_AOC},
    {"QL", "Arrival Report",                             ACARS_CAT_AOC},
    {"QM", "Arrival Information Report",                 ACARS_CAT_AOC},
    {"QN", "Diversion Report",                           ACARS_CAT_AOC},
    {"QP", "OUT Report",                                 ACARS_CAT_AOC},
    {"QQ", "OFF Report",                                 ACARS_CAT_AOC},
    {"QR", "ON Report",                                  ACARS_CAT_AOC},
    {"QS", "IN Report",                                  ACARS_CAT_AOC},
    {"QT", "OUT Return In Report",                       ACARS_CAT_AOC},
    {"QX", "Intercept / Unable to Process",              ACARS_CAT_SERVICE},

    // Command/Response
    {"RA", "Command/Response",                           ACARS_CAT_AOC},
    {"RB", "Command/Response",                           ACARS_CAT_AOC},

    // VHF statistics
    {"S1", "VHF Network Statistics Report",              ACARS_CAT_SERVICE},
    {"S2", "VHF Performance Report",                     ACARS_CAT_SERVICE},
    {"S3", "LRU Configuration Report",                   ACARS_CAT_SERVICE},
    {"SA", "Media Advisory",                             ACARS_CAT_SERVICE},
    {"SQ", "Ground Station Squitter",                    ACARS_CAT_SERVICE},

    // System
    {"UP", "Message Acknowledgement",                    ACARS_CAT_SERVICE},
    {"_d", "No Information To Transmit",                 ACARS_CAT_SERVICE},
    {":;", "Command Transceiver Change Freq",            ACARS_CAT_SERVICE},
    {"::", "Reserved",                                   ACARS_CAT_SERVICE},
};

#define LABEL_TABLE_SIZE (sizeof(label_table) / sizeof(label_table[0]))

static const acars_label_info_t unknown_label = {
    .description = NULL,
    .category = ACARS_CAT_UNKNOWN
};

const acars_label_info_t *acars_label_lookup(const char label[2])
{
    if (!label) return &unknown_label;

    // Linear search (table is ~150 entries — fast enough)
    for (uint32_t i = 0; i < LABEL_TABLE_SIZE; i++) {
        if (label[0] == label_table[i].label[0] &&
            label[1] == label_table[i].label[1]) {
            static acars_label_info_t result;
            result.description = label_table[i].description;
            result.category = label_table[i].category;
            return &result;
        }
    }

    return &unknown_label;
}

const char *acars_category_name(acars_category_t cat)
{
    switch (cat) {
    case ACARS_CAT_ATC:       return "ATC";
    case ACARS_CAT_AOC:       return "AOC";
    case ACARS_CAT_AAC:       return "AAC";
    case ACARS_CAT_SERVICE:   return "Service";
    case ACARS_CAT_EMERGENCY: return "Emergency";
    case ACARS_CAT_WEATHER:   return "Weather";
    case ACARS_CAT_PRINTER:   return "Printer";
    case ACARS_CAT_UNKNOWN:
    default:                  return "Unknown";
    }
}
