#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "app_state.h"

// Initialize PWM channels, GPIO, load calibration from NVS
void actuator_init();

// Read rudder ADC + current sense, update state
void actuator_read_sensors(AppState& state);

// Run position P-controller, drive H-bridge, manage clutch
void actuator_update(AppState& state);

// Save rudder calibration to NVS
void actuator_save_calibration();

// Calibration commands
void actuator_calibrate_center();
void actuator_calibrate_port();
void actuator_calibrate_stbd();

#endif // ACTUATOR_H
