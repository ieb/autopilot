#ifndef N2K_H
#define N2K_H

#include "app_state.h"

// Initialize NMEA2000 library on TWAI CAN bus
void n2k_init();

// Call every loop() — handles RX parsing and periodic TX
void n2k_update(AppState& state);

// Send rudder + heading/track control PGNs
void n2k_send(const AppState& state);

#endif // N2K_H
