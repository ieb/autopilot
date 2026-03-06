"""
Tests for SmoothPilot (progressive rudder application wrapper).
"""

import numpy as np
import pytest

from src.pilots.pd_pilot import PDPilot
from src.pilots.adaptive_pilot import AdaptivePilot
from src.pilots.smooth_pilot import SmoothPilot
from src.pilots.evaluate import evaluate_pilot


def _make_features(heading_error_deg=0.0, heading_rate_dps=0.0,
                   mode=0.0, stw_kn=6.0, pd_suggestion=0.0):
    """Build a 22-element feature vector with specified values."""
    f = np.zeros(22, dtype=np.float32)
    f[0] = heading_error_deg / 90.0
    f[1] = mode
    f[2] = heading_rate_dps / 30.0
    f[11] = stw_kn / 25.0
    f[19] = pd_suggestion
    return f


class TestRateLimiting:
    def test_rate_limits_step_change(self):
        """First step output for large error should be much less than raw PD."""
        inner = PDPilot()
        smooth = SmoothPilot(inner=PDPilot())

        features = _make_features(heading_error_deg=90.0)
        raw = inner.steer(features)
        smoothed = smooth.steer(features)

        assert abs(raw) > 0.5, "Raw PD should give large command for 90° error"
        assert abs(smoothed) < abs(raw) * 0.5, (
            f"Smoothed {smoothed:.3f} should be much less than raw {raw:.3f}"
        )

    def test_ramps_up_over_time(self):
        """Constant error should produce monotonically increasing output."""
        smooth = SmoothPilot(inner=PDPilot())
        features = _make_features(heading_error_deg=15.0)

        outputs = []
        for _ in range(20):
            outputs.append(smooth.steer(features))

        # Each output should be >= the previous (monotonically increasing)
        for i in range(1, len(outputs)):
            assert outputs[i] >= outputs[i - 1] - 1e-6, (
                f"Step {i}: {outputs[i]:.4f} < {outputs[i-1]:.4f}"
            )

    def test_urgency_increases_rate(self):
        """Output should ramp faster after sustained error."""
        # Run two smooth pilots: one with pre-accumulated urgency
        smooth_fresh = SmoothPilot(inner=PDPilot())
        smooth_urgent = SmoothPilot(inner=PDPilot())

        features = _make_features(heading_error_deg=15.0)

        # Build up urgency in one pilot
        for _ in range(60):  # 30s at 2Hz
            smooth_urgent.steer(features)
        smooth_urgent._prev_output = 0.0  # reset output but keep urgency

        # Now compare single-step change
        fresh_step = smooth_fresh.steer(features)
        urgent_step = smooth_urgent.steer(features)

        assert abs(urgent_step) > abs(fresh_step) * 1.5, (
            f"Urgent step {urgent_step:.4f} should be much larger than "
            f"fresh step {fresh_step:.4f}"
        )


class TestSpeedScaling:
    def test_high_speed_reduces_output(self):
        """At 10kn output should be less than at 6kn for same error."""
        smooth_6kn = SmoothPilot(inner=PDPilot())
        smooth_10kn = SmoothPilot(inner=PDPilot())

        features_6kn = _make_features(heading_error_deg=15.0, stw_kn=6.0)
        features_10kn = _make_features(heading_error_deg=15.0, stw_kn=10.0)

        # Run several steps to get past initial rate limiting
        for _ in range(40):
            out_6kn = smooth_6kn.steer(features_6kn)
            out_10kn = smooth_10kn.steer(features_10kn)

        assert abs(out_10kn) < abs(out_6kn), (
            f"10kn output {out_10kn:.4f} should be less than "
            f"6kn output {out_6kn:.4f}"
        )

    def test_low_speed_no_amplification(self):
        """At 3kn output should match unscaled (speed_gain capped at 1.0)."""
        smooth_3kn = SmoothPilot(inner=PDPilot())
        smooth_6kn = SmoothPilot(inner=PDPilot())

        features_3kn = _make_features(heading_error_deg=15.0, stw_kn=3.0)
        features_6kn = _make_features(heading_error_deg=15.0, stw_kn=6.0)

        # Run several steps
        for _ in range(40):
            out_3kn = smooth_3kn.steer(features_3kn)
            out_6kn = smooth_6kn.steer(features_6kn)

        # 3kn speed_factor = 6/3 = 2.0, clamped to 1.0 = same as 6kn
        assert abs(out_3kn - out_6kn) < 0.01, (
            f"3kn output {out_3kn:.4f} should match 6kn output {out_6kn:.4f}"
        )


class TestTurnRateAwareness:
    def test_fast_turn_slows_application(self):
        """High heading rate should reduce rate of rudder change."""
        smooth_calm = SmoothPilot(inner=PDPilot())
        smooth_turning = SmoothPilot(inner=PDPilot())

        features_calm = _make_features(heading_error_deg=15.0,
                                       heading_rate_dps=0.0)
        features_turning = _make_features(heading_error_deg=15.0,
                                          heading_rate_dps=6.0)

        out_calm = smooth_calm.steer(features_calm)
        out_turning = smooth_turning.steer(features_turning)

        assert abs(out_turning) < abs(out_calm), (
            f"Turning output {out_turning:.4f} should be less than "
            f"calm output {out_calm:.4f}"
        )


class TestReset:
    def test_reset_clears_state(self):
        smooth = SmoothPilot(inner=PDPilot())
        features = _make_features(heading_error_deg=15.0)

        for _ in range(20):
            smooth.steer(features)

        assert smooth._prev_output != 0.0
        assert smooth._time_at_error > 0.0

        smooth.reset()
        assert smooth._prev_output == 0.0
        assert smooth._time_at_error == 0.0


class TestStacking:
    def test_stacks_with_adaptive(self):
        """SmoothPilot(AdaptivePilot(PDPilot())) should work."""
        inner = PDPilot()
        adaptive = AdaptivePilot(inner=inner)
        smooth = SmoothPilot(inner=adaptive)

        features = _make_features(heading_error_deg=5.0)
        cmd = smooth.steer(features)
        assert isinstance(cmd, float)
        assert -1.0 <= cmd <= 1.0

        # Reset should propagate
        smooth.reset()
        assert smooth._prev_output == 0.0
        assert adaptive._step == 0


class TestCL:
    def test_cl_all_scenarios_pass(self):
        """Smooth PD should pass all 3 CL scenarios."""
        pilot = SmoothPilot(inner=PDPilot())
        results = evaluate_pilot(pilot, verbose=True)

        for r in results:
            assert r["passed"], (
                f"{r['name']} failed: SS error {r['mean_ss_error_deg']:.1f}° "
                f"(accept <{r['acceptable_deg']}°)"
            )
