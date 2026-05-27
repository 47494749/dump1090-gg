// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// config_panel.h: Built-in configuration and monitoring web panel
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#ifndef CONFIG_PANEL_H
#define CONFIG_PANEL_H


#include <stdint.h>
#include <pthread.h>

#define PANEL_DEFAULT_PORT    8888
#define PANEL_LOG_LINES       2000
#define PANEL_LOG_LINE_LEN    256
#define PANEL_MSG_LINES       10000
#define PANEL_MSG_LINE_LEN    256
#define PANEL_HTML_DIR        "/usr/share/dump1090-gg/panel"
#define PANEL_CONF_PATH       "/etc/dump1090-gg/panel.conf"
#define PANEL_WS_MAX_CLIENTS  4

typedef struct {
    int32_t      enabled;
    int32_t      port;
    char     password[64];      // Basic Auth password (user: admin)
    char     html_dir[256];     // Directory with panel HTML files

    // Log ring buffer
    char     log_buf[PANEL_LOG_LINES][PANEL_LOG_LINE_LEN];
    int32_t      log_head;
    int32_t      log_count;
    int32_t      log_seq;
    pthread_mutex_t log_mutex;

    // Decoded message ring buffer
    char     msg_buf[PANEL_MSG_LINES][PANEL_MSG_LINE_LEN];
    int32_t      msg_head;
    int32_t      msg_count;
    int32_t      msg_seq;
    pthread_mutex_t msg_mutex;

    // Server state
    int32_t      listen_fd;
    pthread_t thread;
    int32_t      running;

    // WebSocket state
    int32_t      ws_fds[PANEL_WS_MAX_CLIENTS];
    int32_t      ws_count;
    pthread_mutex_t ws_mutex;
} panel_state_t;

extern panel_state_t PanelState;

// Initialize panel defaults
void panelInitConfig(void);

// Start the panel HTTP server thread
void panelStart(void);

// Stop the panel
void panelStop(void);

// Add a log line to the ring buffer (thread-safe, also writes to stderr)
void panelLog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Add a decoded message summary to the ring buffer (thread-safe)
void panelLogMessage(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Handle panel CLI options (returns true if handled)
bool panelHandleOption(int argc, char **argv, int *jptr);

// Load saved beast feed enabled/disabled state from panel.conf
void panelEnsureDefaultBeastFeeds(void);
void panelLoadBeastFeedState(void);

// Probe and cache tuner types for all RTL-SDR devices (call before any device is opened)
void panelProbeAllTuners(void);

#endif // CONFIG_PANEL_H
