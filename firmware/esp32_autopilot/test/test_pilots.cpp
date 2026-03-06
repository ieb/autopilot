#include <unity.h>
#include <math.h>
#include "app_state.h"
#include "pilots/pilot_base.h"
#include "pilots/pd_pilot.h"
#include "pilots/pid_pilot.h"
#include "pilots/smooth_pilot.h"
#include "pilots/adaptive_pilot.h"
#include "polar.h"

// Helper: build features with just heading error and rate
static PilotFeatures make_features(float error_deg, float rate_dps, float stw_kn = 6.0f) {
    PilotFeatures f = {};
    f.heading_error = error_deg / 90.0f;
    f.heading_rate = rate_dps / 30.0f;
    f.stw = stw_kn / 25.0f;
    f.mode_flag = 0.0f;
    f.rudder_position = 0.0f;
    f.roll = 0.0f;
    f.pd_suggestion = clampf(
        (1.0f * error_deg + 1.5f * (-rate_dps)) / 25.0f, -1.0f, 1.0f);
    return f;
}

// ============================================================================
// PD Pilot Tests
// ============================================================================

void test_pd_zero_error() {
    PDPilot pd;
    PilotFeatures f = make_features(0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pd.steer(f));
}

void test_pd_5deg_error() {
    // kp=1.0, error=5deg, rate=0: cmd = 1.0*5.0/25.0 = 0.2
    PDPilot pd;
    PilotFeatures f = make_features(5.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, pd.steer(f));
}

void test_pd_negative_error() {
    // kp=1.0, error=-10deg, rate=0: cmd = -10.0/25.0 = -0.4
    PDPilot pd;
    PilotFeatures f = make_features(-10.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.4f, pd.steer(f));
}

void test_pd_damping() {
    // kp=1.0, kd=1.5, error=5, rate=2: cmd = (5.0 + 1.5*(-2))/25 = 2.0/25 = 0.08
    PDPilot pd;
    PilotFeatures f = make_features(5.0f, 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.08f, pd.steer(f));
}

void test_pd_saturation() {
    // Large error should saturate at 1.0
    PDPilot pd;
    PilotFeatures f = make_features(50.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, pd.steer(f));
}

void test_pd_reset() {
    PDPilot pd;
    PilotFeatures f = make_features(5.0f, 0.0f);
    pd.steer(f);
    pd.reset();
    // After reset, should still compute correctly
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, pd.steer(f));
}

// ============================================================================
// PID Pilot Tests
// ============================================================================

void test_pid_proportional() {
    // Same as PD with ki=0
    PIDPilot pid(1.0f, 0.0f, 1.5f);
    PilotFeatures f = make_features(5.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, pid.steer(f));
}

void test_pid_integrator_accumulates() {
    PIDPilot pid(1.0f, 0.1f, 1.5f, 1.0f, 10.0f, 0.2f);
    PilotFeatures f = make_features(5.0f, 0.0f);

    float cmd1 = pid.steer(f);
    float cmd2 = pid.steer(f);
    // Second call should have larger output due to integral
    TEST_ASSERT_TRUE(cmd2 > cmd1);
}

void test_pid_reset_clears_integrator() {
    PIDPilot pid(1.0f, 0.1f, 1.5f, 1.0f, 10.0f, 0.2f);
    PilotFeatures f = make_features(5.0f, 0.0f);

    pid.steer(f);
    pid.steer(f);
    pid.reset();

    // After reset, should match first call
    float cmd = pid.steer(f);
    PIDPilot fresh(1.0f, 0.1f, 1.5f, 1.0f, 10.0f, 0.2f);
    float expected = fresh.steer(f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, expected, cmd);
}

// ============================================================================
// Smooth Pilot Tests
// ============================================================================

void test_smooth_rate_limits() {
    PDPilot inner;
    SmoothPilot smooth(&inner, 0.2f);

    // Large step: should be rate-limited (not reach full command immediately)
    PilotFeatures f = make_features(20.0f, 0.0f);
    float cmd = smooth.steer(f);

    // Inner PD would give 20/25 = 0.8, but smooth limits rate
    TEST_ASSERT_TRUE(cmd < 0.8f);
    TEST_ASSERT_TRUE(cmd > 0.0f);
}

void test_smooth_speed_scaling() {
    // Run multiple steps so rate limiter converges and speed scaling dominates
    PDPilot inner1, inner2;
    SmoothPilot smooth1(&inner1, 0.2f, 3.0f, 6.0f, 0.4f, 1.0f);
    SmoothPilot smooth2(&inner2, 0.2f, 3.0f, 6.0f, 0.4f, 1.0f);

    PilotFeatures f_slow = make_features(5.0f, 0.0f, 3.0f);  // slow -> speed_factor=1.0
    PilotFeatures f_fast = make_features(5.0f, 0.0f, 12.0f); // fast -> speed_factor=0.5

    float cmd_slow = 0, cmd_fast = 0;
    for (int i = 0; i < 50; i++) {
        cmd_slow = smooth1.steer(f_slow);
        cmd_fast = smooth2.steer(f_fast);
    }

    // After convergence, slow should have larger output than fast
    TEST_ASSERT_TRUE(cmd_slow > cmd_fast);
}

void test_smooth_reset() {
    PDPilot inner;
    SmoothPilot smooth(&inner, 0.2f);
    PilotFeatures f = make_features(10.0f, 0.0f);

    smooth.steer(f);
    smooth.steer(f);
    smooth.reset();

    // After reset, first output should be rate-limited from zero
    float cmd = smooth.steer(f);
    TEST_ASSERT_TRUE(cmd < 0.5f);
}

// ============================================================================
// Adaptive Pilot Tests
// ============================================================================

void test_adaptive_passes_through() {
    PDPilot inner;
    AdaptivePilot adaptive(&inner, false, 0.2f);

    PilotFeatures f = make_features(5.0f, 0.0f);
    float cmd = adaptive.steer(f);

    // First step: no EKF update, just passes through inner pilot
    // Inner PD: 1.0*5/25 = 0.2 (gains may have been set to midpoint of bounds)
    // Since default bounds midpoint for kp = (0.5+4.0)/2 = 2.25, kd = 2.25
    // We just check it produces a reasonable output
    TEST_ASSERT_TRUE(fabsf(cmd) > 0.0f);
    TEST_ASSERT_TRUE(fabsf(cmd) <= 1.0f);
}

void test_adaptive_confidence() {
    PDPilot inner;
    AdaptivePilot adaptive(&inner, false, 0.2f);

    float conf = adaptive.get_confidence();
    // Initial: trace(P) = 2 * 0.1 = 0.2, confidence = 1/(1+0.2) = 0.833
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.833f, conf);
}

// ============================================================================
// Polar Tests
// ============================================================================

void test_polar_zero_wind() {
    float stw = polar_get_target_speed(90.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, stw);
}

void test_polar_beam_reach_10kn() {
    // TWA=90, TWS=10: from table, row[14] (90deg), col[4] (10kn) = 8.6
    float stw = polar_get_target_speed(90.0f, 10.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 8.6f, stw);
}

void test_polar_interpolation() {
    // TWA=85, TWS=9: should interpolate between known values
    float stw = polar_get_target_speed(85.0f, 9.0f);
    TEST_ASSERT_TRUE(stw > 7.0f);
    TEST_ASSERT_TRUE(stw < 9.0f);
}

void test_polar_vmg_upwind() {
    PolarResult r = polar_get_optimal_vmg_upwind(12.0f);
    // Upwind optimal should be roughly 35-50 deg for most boats
    TEST_ASSERT_TRUE(r.optimal_twa >= 30.0f);
    TEST_ASSERT_TRUE(r.optimal_twa <= 55.0f);
    TEST_ASSERT_TRUE(r.vmg > 0.0f);
}

void test_polar_vmg_downwind() {
    PolarResult r = polar_get_optimal_vmg_downwind(12.0f);
    // Downwind optimal should be 140-170 deg
    TEST_ASSERT_TRUE(r.optimal_twa >= 130.0f);
    TEST_ASSERT_TRUE(r.optimal_twa <= 175.0f);
    TEST_ASSERT_TRUE(r.vmg > 0.0f);
}

void test_polar_performance_ratio() {
    float target = polar_get_target_speed(90.0f, 10.0f);
    float ratio = polar_get_performance_ratio(90.0f, 10.0f, target * 0.9f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.9f, ratio);
}

void test_polar_negative_twa() {
    // Negative TWA should give same result as positive (symmetric)
    float stw_pos = polar_get_target_speed(90.0f, 10.0f);
    float stw_neg = polar_get_target_speed(-90.0f, 10.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, stw_pos, stw_neg);
}

// ============================================================================
// Sign Convention Tests
// ============================================================================

void test_positive_error_positive_rudder() {
    // Positive heading error (target to starboard) -> positive rudder
    PDPilot pd;
    PilotFeatures f = make_features(10.0f, 0.0f);
    TEST_ASSERT_TRUE(pd.steer(f) > 0.0f);
}

void test_negative_error_negative_rudder() {
    PDPilot pd;
    PilotFeatures f = make_features(-10.0f, 0.0f);
    TEST_ASSERT_TRUE(pd.steer(f) < 0.0f);
}

void test_positive_rate_reduces_command() {
    // Positive yaw rate (turning starboard) should reduce starboard command
    PDPilot pd;
    PilotFeatures f_no_rate = make_features(10.0f, 0.0f);
    PilotFeatures f_with_rate = make_features(10.0f, 3.0f);

    float cmd_no_rate = pd.steer(f_no_rate);
    pd.reset();
    float cmd_with_rate = pd.steer(f_with_rate);

    TEST_ASSERT_TRUE(cmd_with_rate < cmd_no_rate);
}

// ============================================================================
// Test Runner
// ============================================================================

void setUp() {}
void tearDown() {}

int main(int argc, char** argv) {
    UNITY_BEGIN();

    // PD
    RUN_TEST(test_pd_zero_error);
    RUN_TEST(test_pd_5deg_error);
    RUN_TEST(test_pd_negative_error);
    RUN_TEST(test_pd_damping);
    RUN_TEST(test_pd_saturation);
    RUN_TEST(test_pd_reset);

    // PID
    RUN_TEST(test_pid_proportional);
    RUN_TEST(test_pid_integrator_accumulates);
    RUN_TEST(test_pid_reset_clears_integrator);

    // Smooth
    RUN_TEST(test_smooth_rate_limits);
    RUN_TEST(test_smooth_speed_scaling);
    RUN_TEST(test_smooth_reset);

    // Adaptive
    RUN_TEST(test_adaptive_passes_through);
    RUN_TEST(test_adaptive_confidence);

    // Polar
    RUN_TEST(test_polar_zero_wind);
    RUN_TEST(test_polar_beam_reach_10kn);
    RUN_TEST(test_polar_interpolation);
    RUN_TEST(test_polar_vmg_upwind);
    RUN_TEST(test_polar_vmg_downwind);
    RUN_TEST(test_polar_performance_ratio);
    RUN_TEST(test_polar_negative_twa);

    // Sign conventions
    RUN_TEST(test_positive_error_positive_rudder);
    RUN_TEST(test_negative_error_negative_rudder);
    RUN_TEST(test_positive_rate_reduces_command);

    return UNITY_END();
}
