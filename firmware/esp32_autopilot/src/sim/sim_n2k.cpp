#ifdef SIM_BUILD

#include "n2k.h"
#include "config.h"
#include "polar.h"
#include "sim_hal.h"
#include <math.h>
#include <stdio.h>

// Simulated environment
static float sim_tws = 12.0f;   // true wind speed (kts)
static float sim_twd = 225.0f;  // true wind direction (degrees, 0-360)

// Expose for sim_main CLI args
void sim_n2k_set_wind(float tws, float twd) {
    sim_tws = tws;
    sim_twd = twd;
}

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

void n2k_init() {
    printf("[SIM] N2K init (fake NMEA2000, TWS=%.1f TWD=%.0f)\n", sim_tws, sim_twd);
}

// Forward declaration from sim_imu.cpp
float sim_imu_get_heading();

void n2k_update(AppState& state) {
    float heading = sim_imu_get_heading();

    // TWA: signed, positive = starboard
    float twa = wrap_180(sim_twd - heading);
    state.twa = twa;
    state.tws = sim_tws;

    // STW from polar
    float abs_twa = fabsf(twa);
    if (abs_twa < 1.0f) abs_twa = 1.0f;
    state.stw = polar_get_target_speed(abs_twa, sim_tws) * 0.95f;  // 95% of polar
    if (state.stw < 0.5f) state.stw = 0.5f;

    // AWA/AWS from wind triangle
    // V_apparent = V_true + V_boat (vector addition)
    float twa_rad = twa * (M_PI / 180.0f);
    float wx = sim_tws * cosf(twa_rad) + state.stw;  // apparent wind x (headwind component)
    float wy = sim_tws * sinf(twa_rad);               // apparent wind y (cross component)
    state.aws = sqrtf(wx * wx + wy * wy);
    state.awa = atan2f(wy, wx) * (180.0f / M_PI);

    // SOG/COG
    state.sog = state.stw * 0.98f;
    state.cog = heading;

    // N2K heading (fluxgate sim): provide true heading with small noise
    state.n2k_heading = heading;
    state.imu_heading_at_n2k = heading;  // snapshot
    state.n2k_heading_valid = true;

    // Timestamps
    uint32_t now = sim_millis();
    state.n2k_wind_ms = now;
    state.n2k_speed_ms = now;
    state.n2k_cog_ms = now;
    state.n2k_heading_ms = now;
}

void n2k_send(const AppState& state) {
    // No-op in simulation (no CAN bus)
}

#endif // SIM_BUILD
