#include "polar.h"
#include "polar_data.h"
#include <math.h>

struct IndexResult {
    int i0, i1;
    float frac;
};

static IndexResult find_indices(const float* arr, int len, float val) {
    if (val <= arr[0]) return {0, 0, 0.0f};
    if (val >= arr[len - 1]) return {len - 1, len - 1, 0.0f};

    for (int i = 0; i < len - 1; i++) {
        if (arr[i] <= val && val <= arr[i + 1]) {
            float denom = arr[i + 1] - arr[i];
            float frac = (denom != 0.0f) ? (val - arr[i]) / denom : 0.0f;
            return {i, i + 1, frac};
        }
    }
    return {len - 1, len - 1, 0.0f};
}

float polar_get_target_speed(float twa, float tws) {
    twa = fabsf(twa);
    if (twa > 180.0f) twa = 360.0f - twa;

    IndexResult twa_idx = find_indices(POLAR_TWA, POLAR_NUM_TWA, twa);
    IndexResult tws_idx = find_indices(POLAR_TWS, POLAR_NUM_TWS, tws);

    float v00 = POLAR_STW[twa_idx.i0][tws_idx.i0];
    float v01 = POLAR_STW[twa_idx.i0][tws_idx.i1];
    float v10 = POLAR_STW[twa_idx.i1][tws_idx.i0];
    float v11 = POLAR_STW[twa_idx.i1][tws_idx.i1];

    float v0 = v00 + (v01 - v00) * tws_idx.frac;
    float v1 = v10 + (v11 - v10) * tws_idx.frac;

    return v0 + (v1 - v0) * twa_idx.frac;
}

PolarResult polar_get_optimal_vmg_upwind(float tws) {
    float best_vmg = 0.0f;
    float best_twa = 45.0f;
    float best_stw = 0.0f;

    for (int twa_deg = 25; twa_deg < 70; twa_deg++) {
        float stw = polar_get_target_speed((float)twa_deg, tws);
        float vmg = stw * cosf(twa_deg * (float)M_PI / 180.0f);
        if (vmg > best_vmg) {
            best_vmg = vmg;
            best_twa = (float)twa_deg;
            best_stw = stw;
        }
    }
    return {best_twa, best_stw, best_vmg};
}

PolarResult polar_get_optimal_vmg_downwind(float tws) {
    float best_vmg = 0.0f;
    float best_twa = 150.0f;
    float best_stw = 0.0f;

    for (int twa_deg = 110; twa_deg <= 180; twa_deg++) {
        float stw = polar_get_target_speed((float)twa_deg, tws);
        float vmg = stw * cosf((180 - twa_deg) * (float)M_PI / 180.0f);
        if (vmg > best_vmg) {
            best_vmg = vmg;
            best_twa = (float)twa_deg;
            best_stw = stw;
        }
    }
    return {best_twa, best_stw, best_vmg};
}

float polar_get_performance_ratio(float twa, float tws, float actual_stw) {
    float target = polar_get_target_speed(twa, tws);
    if (target <= 0.0f) return 0.0f;
    return actual_stw / target;
}
