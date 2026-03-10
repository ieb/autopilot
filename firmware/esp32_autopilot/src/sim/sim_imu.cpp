#ifdef SIM_BUILD

#include "imu.h"
#include "config.h"
#include "sim_hal.h"
#include <math.h>

// Internal sim state
static float sim_true_heading = 180.0f;  // degrees, 0-360
static float sim_roll = 0.0f;
static float sim_pitch = 0.0f;
static float sim_yaw_rate = 0.0f;
static float prev_heading = 180.0f;
static float prev_roll = 0.0f;

static float wrap_360(float deg) {
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg;
}

static float wrap_180(float deg) {
    deg = fmodf(deg, 360.0f);
    if (deg > 180.0f) deg -= 360.0f;
    if (deg < -180.0f) deg += 360.0f;
    return deg;
}

bool imu_init() {
    printf("[SIM] IMU init (fake BNO055) heading=%.0f\n", sim_true_heading);
    return true;
}

// Update heading based on rudder position (simple yaw dynamics)
void sim_imu_update_dynamics(float rudder_actual_norm, float dt) {
    // Turn rate: ~3 deg/s per 10 deg rudder (at ~6 kts)
    float rudder_deg = rudder_actual_norm * 25.0f;
    float turn_rate = rudder_deg * 0.3f;  // deg/s per deg of rudder
    sim_yaw_rate = turn_rate;
    sim_true_heading = wrap_360(sim_true_heading + turn_rate * dt);
}

void imu_read(AppState& state) {
    float dt = IMU_INTERVAL_MS / 1000.0f;

    // Heading: if N2K heading is valid, use fusion (same as real imu.cpp)
    // In sim, n2k provides the "truth" heading and imu provides the same
    // since we control both. The fusion in the real code will work transparently.
    float imu_heading = sim_true_heading;

    if (state.n2k_heading_valid) {
        float imu_delta = wrap_180(imu_heading - state.imu_heading_at_n2k);
        state.heading = wrap_360(state.n2k_heading + imu_delta);
    } else {
        state.heading = imu_heading;
    }

    // Roll: simplified heel model from AWA and speed
    // ~10 deg heel at 15 deg AWA and 7 kts
    float target_roll = 0.0f;
    if (state.stw > 0.5f) {
        float awa_rad = state.awa * (M_PI / 180.0f);
        target_roll = sinf(awa_rad) * state.stw * 0.15f;
    }
    sim_roll += (target_roll - sim_roll) * 0.1f;  // smooth
    state.roll = sim_roll;

    // Pitch: small wave oscillation
    static uint32_t pitch_phase = 0;
    pitch_phase++;
    sim_pitch = 2.0f * sinf(pitch_phase * 0.05f);  // ~1 Hz wave
    state.pitch = sim_pitch;

    // Rates
    state.yaw_rate = sim_yaw_rate;
    state.roll_rate = (state.roll - prev_roll) / dt;
    prev_roll = state.roll;

    state.imu_last_ms = sim_millis();
}

// Expose for sim_main to call
float sim_imu_get_heading() {
    return sim_true_heading;
}

#endif // SIM_BUILD
