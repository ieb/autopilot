#ifndef PILOT_MANAGER_H
#define PILOT_MANAGER_H

#include "app_state.h"

// Initialize pilot instances and load saved config from NVS
void pilot_manager_init();

// Build features from state, run active pilot, write rudder_target
void pilot_manager_update(AppState& state);

// Set pilot type (PD/PID/Smooth/Adaptive)
void pilot_manager_set_type(PilotType type);

// Set pilot mode + target value, engage clutch
void pilot_manager_set_mode(PilotMode mode, float target);

// Set gains on active pilot
void pilot_manager_set_gains(float kp, float ki, float kd);

// Get current gains
void pilot_manager_get_gains(float& kp, float& ki, float& kd);

// Get active pilot type
PilotType pilot_manager_get_type();

#endif // PILOT_MANAGER_H
