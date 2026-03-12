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

// Get adaptive pilot confidence (0..1, higher = more certain of gains)
// Returns -1.0 if not in adaptive mode.
float pilot_manager_get_adaptive_confidence();

// Get adaptive pilot estimated plant parameters (K_r, K_d)
// Returns false if not in adaptive mode.
bool pilot_manager_get_plant_params(float& kr, float& kd_plant);

// Latch VMG tack side from current TWA (call when entering VMG mode)
void pilot_manager_latch_tack(float twa);

// Flip VMG tack side (for tack commands from p70/web)
void pilot_manager_tack();

#endif // PILOT_MANAGER_H
