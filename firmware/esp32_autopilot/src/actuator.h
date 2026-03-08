#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <stdint.h>
#include "app_state.h"

// Initialize PWM channels, GPIO, load calibration from NVS
void actuator_init();

// Read rudder ADC + current sense, update state
void actuator_read_sensors(AppState& state);

// Run position P-controller, drive H-bridge, manage clutch
void actuator_update(AppState& state);

// Save rudder calibration to NVS
void actuator_save_calibration();

// Calibration commands — read current ADC position and save as reference point
void actuator_calibrate_center();
void actuator_calibrate_port();
void actuator_calibrate_stbd();

// Adjust port and starboard limits to make the range symmetric
void actuator_calibration_linearise();

// Get current calibration values (millivolts)
void actuator_get_calibration(uint32_t& center, uint32_t& port, uint32_t& stbd);

// Read current rudder ADC as calibrated millivolts (for UI display)
uint32_t actuator_read_raw_mv();

#endif // ACTUATOR_H
