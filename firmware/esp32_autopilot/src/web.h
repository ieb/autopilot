#ifndef WEB_H
#define WEB_H

#include "app_state.h"

// Start WiFi AP and AsyncWebServer
void web_init();

// Call periodically to handle any web-triggered state changes
void web_apply_commands(AppState& state);

// Set port for sim web server (sim builds only)
#ifdef SIM_BUILD
void sim_web_set_port(int port);
#endif

#endif // WEB_H
