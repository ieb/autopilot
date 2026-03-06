#include "pilot_manager.h"
#include "config.h"
#include "polar.h"
#include "pilots/pilot_base.h"
#include "pilots/pd_pilot.h"
#include "pilots/pid_pilot.h"
#include "pilots/smooth_pilot.h"
#include "pilots/adaptive_pilot.h"
#include <math.h>

#ifndef NATIVE_BUILD
#include <Preferences.h>
static Preferences prefs;
#endif

// Pilot instances (statically allocated)
static PDPilot pd_pilot(DEFAULT_KP, DEFAULT_KD);
static PIDPilot pid_pilot(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
static SmoothPilot smooth_pilot(&pd_pilot, 1.0f / PILOT_RATE_HZ);
static AdaptivePilot adaptive_pilot(&pd_pilot, false, 1.0f / PILOT_RATE_HZ);

static BasePilot* active_pilot = &pd_pilot;
static PilotType active_type = PILOT_PD;

// Wrap heading difference to [-180, 180]
static float wrap_180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float compute_heading_error(const AppState& state) {
    switch (state.pilot_mode) {
        case MODE_COMPASS: {
            return wrap_180(state.target_value - state.heading);
        }
        case MODE_WIND_AWA: {
            // Target heading = current_heading + (target_awa - current_awa)
            // Error: target_heading - current_heading = target_awa - current_awa
            // But we want: positive error = target to starboard = positive rudder
            // AWA convention: positive = starboard
            // If current_awa > target_awa, wind too far aft, head up (positive rudder)
            return state.awa - state.target_value;
        }
        case MODE_WIND_TWA: {
            return state.twa - state.target_value;
        }
        case MODE_VMG_UP:
        case MODE_VMG_DOWN:
            // target_value is dynamically set to optimal TWA each cycle
            return state.twa - state.target_value;
        default:
            return 0.0f;
    }
}

void pilot_manager_init() {
#ifndef NATIVE_BUILD
    prefs.begin(NVS_NAMESPACE, true);
    float kp = prefs.getFloat("kp", DEFAULT_KP);
    float ki = prefs.getFloat("ki", DEFAULT_KI);
    float kd = prefs.getFloat("kd", DEFAULT_KD);
    active_type = (PilotType)prefs.getUChar("pilot_type", PILOT_PD);
    prefs.end();

    pd_pilot.kp = kp;
    pd_pilot.kd = kd;
    pid_pilot.kp = kp;
    pid_pilot.ki = ki;
    pid_pilot.kd = kd;
#endif

    switch (active_type) {
        case PILOT_PD:       active_pilot = &pd_pilot; break;
        case PILOT_PID:      active_pilot = &pid_pilot; break;
        case PILOT_SMOOTH:   active_pilot = &smooth_pilot; break;
        case PILOT_ADAPTIVE: active_pilot = &adaptive_pilot; break;
    }
}

void pilot_manager_update(AppState& state) {
    if (state.pilot_mode == MODE_STANDBY) {
        state.rudder_target = 0.0f;
        return;
    }

    // For VMG modes, update target TWA from polar each cycle
    if (state.pilot_mode == MODE_VMG_UP) {
        PolarResult r = polar_get_optimal_vmg_upwind(state.tws);
        // Preserve tack side: if TWA is negative (port tack), target is negative
        state.target_value = (state.twa < 0) ? -r.optimal_twa : r.optimal_twa;
    } else if (state.pilot_mode == MODE_VMG_DOWN) {
        PolarResult r = polar_get_optimal_vmg_downwind(state.tws);
        state.target_value = (state.twa < 0) ? -r.optimal_twa : r.optimal_twa;
    }

    float heading_error = compute_heading_error(state);

    // Build feature struct
    PilotFeatures f;
    f.heading_error = heading_error / 90.0f;
    f.heading_rate = state.yaw_rate / 30.0f;
    f.stw = state.stw / 25.0f;
    f.rudder_position = state.rudder_actual;
    f.roll = state.roll / 45.0f;

    // Mode flag
    switch (state.pilot_mode) {
        case MODE_COMPASS:   f.mode_flag = 0.0f; break;
        case MODE_WIND_AWA:  f.mode_flag = 0.5f; break;
        case MODE_WIND_TWA:
        case MODE_VMG_UP:
        case MODE_VMG_DOWN:  f.mode_flag = 1.0f; break;
        default:             f.mode_flag = 0.0f; break;
    }

    // PD suggestion (feature[19])
    float h_err_deg = heading_error;
    float h_rate_deg = state.yaw_rate;
    f.pd_suggestion = clampf(
        (DEFAULT_KP * h_err_deg + DEFAULT_KD * (-h_rate_deg)) / 25.0f,
        -1.0f, 1.0f
    );

    state.rudder_target = active_pilot->steer(f);
    state.last_pilot_ms = millis();
}

void pilot_manager_set_type(PilotType type) {
    if (type == active_type) return;

    // Reset new pilot before switching
    switch (type) {
        case PILOT_PD:       pd_pilot.reset(); active_pilot = &pd_pilot; break;
        case PILOT_PID:      pid_pilot.reset(); active_pilot = &pid_pilot; break;
        case PILOT_SMOOTH:   smooth_pilot.reset(); active_pilot = &smooth_pilot; break;
        case PILOT_ADAPTIVE: adaptive_pilot.reset(); active_pilot = &adaptive_pilot; break;
    }
    active_type = type;

#ifndef NATIVE_BUILD
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putUChar("pilot_type", (uint8_t)type);
    prefs.end();
#endif
}

void pilot_manager_set_mode(PilotMode mode, float target) {
    // Reset pilot state on mode change
    active_pilot->reset();
}

void pilot_manager_set_gains(float kp, float ki, float kd) {
    active_pilot->configure(kp, ki, kd);

#ifndef NATIVE_BUILD
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putFloat("kp", kp);
    prefs.putFloat("ki", ki);
    prefs.putFloat("kd", kd);
    prefs.end();
#endif
}

void pilot_manager_get_gains(float& kp, float& ki, float& kd) {
    kp = pd_pilot.kp;
    ki = pid_pilot.ki;
    kd = pd_pilot.kd;
}

PilotType pilot_manager_get_type() {
    return active_type;
}
