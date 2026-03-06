#ifndef POLAR_H
#define POLAR_H

struct PolarResult {
    float optimal_twa;   // degrees
    float target_stw;    // knots
    float vmg;           // knots
};

// Get target boat speed from polar via bilinear interpolation
float polar_get_target_speed(float twa, float tws);

// Find optimal upwind VMG angle (searches 25-69 deg)
PolarResult polar_get_optimal_vmg_upwind(float tws);

// Find optimal downwind VMG angle (searches 110-180 deg)
PolarResult polar_get_optimal_vmg_downwind(float tws);

// Performance ratio: actual_stw / polar_target
float polar_get_performance_ratio(float twa, float tws, float actual_stw);

#endif // POLAR_H
