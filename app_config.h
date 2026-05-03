// Part of dump1090-gg, a Mode S message decoder.
//
// app_config.h: Application configuration initialization

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

// Sync new config structs from legacy Modes struct.
// Call this after CLI argument parsing is complete.
void appConfigSyncFromModes(void);

#endif // APP_CONFIG_H
