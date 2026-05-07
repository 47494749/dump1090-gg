// Stub for view1090/faup1090 which don't need the panel
#include "config_panel.h"
#include <stdbool.h>

panel_state_t PanelState;

void panelInitConfig(void) {}
void panelStart(void) {}
void panelStop(void) {}
void panelProbeAllTuners(void) {}
void panelLog(const char *fmt, ...) { (void)fmt; }
void panelLogMessage(const char *fmt, ...) { (void)fmt; }
bool panelHandleOption(int argc, char **argv, int *jptr) { (void)argc; (void)argv; (void)jptr; return false; }
