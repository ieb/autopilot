#ifdef SIM_BUILD

#include "actuator.h"
#include "config.h"
#include "sim_hal.h"
#include <math.h>

// Simulated rudder state
static float sim_rudder_pos = 0.0f;   // normalized -1..+1
static float sim_rudder_prev = 0.0f;
static bool sim_clutch = false;

// Calibration (not meaningful in sim, but API must work)
static uint32_t sim_cal_center = ADC_RUDDER_CENTER_MV;
static uint32_t sim_cal_port = ADC_RUDDER_PORT_MV;
static uint32_t sim_cal_stbd = ADC_RUDDER_STBD_MV;

// Actuator rate limit: 1.7 deg/s = 1.7/25 = 0.068 normalized/s
static constexpr float ACTUATOR_RATE = 1.7f / 25.0f;  // normalized per second

void actuator_init() {
    printf("[SIM] Actuator init (fake BTS7960, rate=1.7 deg/s)\n");
}

void actuator_read_sensors(AppState& state) {
    float dt = ACTUATOR_INTERVAL_MS / 1000.0f;
    state.rudder_velocity = (sim_rudder_pos - sim_rudder_prev) / dt;
    sim_rudder_prev = sim_rudder_pos;
    state.rudder_actual = sim_rudder_pos;

    // Simulate motor current: ~3A at full duty, proportional to velocity
    // (motor draws current when moving, near-zero when at target)
    state.motor_current = sim_clutch ? fabsf(state.rudder_velocity) / ACTUATOR_RATE * 3.0f : 0.0f;
    state.supply_voltage = 12.8f;
}

void actuator_update(AppState& state) {
    // Watchdog
    if (sim_millis() - state.last_pilot_ms > WATCHDOG_TIMEOUT_MS) {
        if (state.clutch_engaged) {
            state.clutch_requested = false;
            state.fault_code = FAULT_WATCHDOG;
        }
    }

    // Clutch
    if (state.clutch_requested && !state.clutch_engaged) {
        state.clutch_engaged = true;
        state.fault_code = FAULT_NONE;
        sim_clutch = true;
    } else if (!state.clutch_requested && state.clutch_engaged) {
        state.clutch_engaged = false;
        sim_clutch = false;
    }

    if (!sim_clutch) {
        return;
    }

    // Rate-limited position tracking
    float dt = ACTUATOR_INTERVAL_MS / 1000.0f;
    float error = state.rudder_target - sim_rudder_pos;
    float max_move = ACTUATOR_RATE * dt;

    if (error > max_move) {
        sim_rudder_pos += max_move;
    } else if (error < -max_move) {
        sim_rudder_pos -= max_move;
    } else {
        sim_rudder_pos = state.rudder_target;
    }

    // Clamp to position limits
    float limit = POSITION_LIMIT_NORM;
    if (sim_rudder_pos > limit) sim_rudder_pos = limit;
    if (sim_rudder_pos < -limit) sim_rudder_pos = -limit;
}

void actuator_save_calibration() {
    printf("[SIM] Calibration saved: center=%u port=%u stbd=%u mV\n",
           sim_cal_center, sim_cal_port, sim_cal_stbd);
}

void actuator_calibrate_center() {
    printf("[SIM] Calibrate center (no-op in sim)\n");
}

void actuator_calibrate_port() {
    printf("[SIM] Calibrate port (no-op in sim)\n");
}

void actuator_calibrate_stbd() {
    printf("[SIM] Calibrate stbd (no-op in sim)\n");
}

void actuator_calibration_linearise() {
    uint32_t port_range = sim_cal_center - sim_cal_port;
    uint32_t stbd_range = sim_cal_stbd - sim_cal_center;
    uint32_t min_range = (port_range < stbd_range) ? port_range : stbd_range;
    sim_cal_port = sim_cal_center - min_range;
    sim_cal_stbd = sim_cal_center + min_range;
    actuator_save_calibration();
}

void actuator_get_calibration(uint32_t& center, uint32_t& port, uint32_t& stbd) {
    center = sim_cal_center;
    port = sim_cal_port;
    stbd = sim_cal_stbd;
}

uint32_t actuator_read_raw_mv() {
    // Convert current sim_rudder_pos back to a fake mV reading
    if (sim_rudder_pos < 0) {
        float frac = -sim_rudder_pos;
        return (uint32_t)(sim_cal_center - frac * (sim_cal_center - sim_cal_port));
    } else {
        float frac = sim_rudder_pos;
        return (uint32_t)(sim_cal_center + frac * (sim_cal_stbd - sim_cal_center));
    }
}

#endif // SIM_BUILD
