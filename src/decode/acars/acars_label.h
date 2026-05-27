// Part of dump1090-gg-light
//
// acars_label.h: ACARS label semantic lookup
//
// Provides human-readable descriptions and category classification
// for ACARS message labels per ARINC 618/620 and industry practice.
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#ifndef ACARS_LABEL_H
#define ACARS_LABEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ACARS message categories
typedef enum {
    ACARS_CAT_UNKNOWN   = 0,   // Unknown / unrecognized label
    ACARS_CAT_ATC       = 1,   // Air Traffic Control (clearances, position reports)
    ACARS_CAT_AOC       = 2,   // Airline Operational Control (OOOI, fuel, weather req)
    ACARS_CAT_AAC       = 3,   // Airline Administrative Control (crew, cargo)
    ACARS_CAT_SERVICE   = 4,   // System service messages (link test, media advisory)
    ACARS_CAT_EMERGENCY = 5,   // Emergency situations
    ACARS_CAT_WEATHER   = 6,   // Weather-related messages
    ACARS_CAT_PRINTER   = 7,   // Cockpit printer messages
} acars_category_t;

// Label lookup result
typedef struct {
    const char       *description;  // Human-readable description (NULL if unknown)
    acars_category_t  category;     // Message category
} acars_label_info_t;

// Look up an ACARS label (2-char string).
// Always returns a valid pointer (static storage, never NULL).
// If the label is unknown, description will be NULL and category ACARS_CAT_UNKNOWN.
const acars_label_info_t *acars_label_lookup(const char label[2]);

// Get category name string
const char *acars_category_name(acars_category_t cat);

#ifdef __cplusplus
}
#endif

#endif // ACARS_LABEL_H
