// Part of dump1090-gg-light, a Mode S message decoder.
//
// app_config.h: Application configuration initialization
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

// Sync new config structs from legacy Modes struct.
// Call this after CLI argument parsing is complete.
void appConfigSyncFromModes(void);

#endif // APP_CONFIG_H
