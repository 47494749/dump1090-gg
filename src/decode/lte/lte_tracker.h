// Part of dump1090-gg-light
//
// lte_tracker.h: Thread-safe LTE cell tracking and JSON export.
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.

#ifndef LTE_TRACKER_H
#define LTE_TRACKER_H
#include <stdint.h>

#include "lte_decode.h"

// Initialize the global LTE cell tracker
void lteTrackerInit(void);

// Update cell info (called from decoder callback, any thread)
void lteTrackerUpdate(const lte_cell_info_t *cell);

// Get number of tracked cells
int32_t lteTrackerCount(void);

// Clean up
void lteTrackerDestroy(void);

// Returns std::string
#include <string>
std::string lteTrackerToJSON(void);

#endif // LTE_TRACKER_H
