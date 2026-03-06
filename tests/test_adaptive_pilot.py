"""
Tests for AdaptivePilot (EKF gain tuning wrapper).
"""

import json
import tempfile

import numpy as np
import pytest

from src.pilots.pd_pilot import PDPilot
from src.pilots.pid_pilot import PIDPilot
from src.pilots.adaptive_pilot import AdaptivePilot
from src.pilots.evaluate import evaluate_pilot


def _make_features(heading_error_deg=0.0, heading_rate_dps=0.0,
                   mode=0.0, pd_suggestion=0.0):
    """Build a 22-element feature vector with specified values."""
    f = np.zeros(22, dtype=np.float32)
    f[0] = heading_error_deg / 90.0
    f[1] = mode
    f[2] = heading_rate_dps / 30.0
    f[19] = pd_suggestion
    return f


class TestWrapping:
    def test_wraps_pd_pilot(self):
        inner = PDPilot()
        pilot = AdaptivePilot(inner=inner)
        features = _make_features(heading_error_deg=5.0)
        cmd = pilot.steer(features)
        assert isinstance(cmd, float)
        assert -1.0 <= cmd <= 1.0

    def test_wraps_pid_pilot(self):
        inner = PIDPilot()
        pilot = AdaptivePilot(inner=inner)
        assert pilot._n == 3
        assert pilot._has_ki is True
        features = _make_features(heading_error_deg=5.0)
        cmd = pilot.steer(features)
        assert isinstance(cmd, float)
        assert -1.0 <= cmd <= 1.0


class TestGainBounds:
    def test_gains_within_bounds(self):
        inner = PDPilot()
        pilot = AdaptivePilot(inner=inner, kp_bounds=(0.5, 4.0),
                              kd_bounds=(0.5, 4.0))
        # Feed extreme features to stress the EKF
        for i in range(200):
            sign = 1 if i % 2 == 0 else -1
            f = _make_features(heading_error_deg=sign * 80.0,
                               heading_rate_dps=sign * 25.0)
            pilot.steer(f)

        gains = pilot.gains
        assert 0.5 <= gains["kp"] <= 4.0
        assert 0.5 <= gains["kd"] <= 4.0

    def test_pid_gains_within_bounds(self):
        inner = PIDPilot()
        pilot = AdaptivePilot(inner=inner)
        for i in range(200):
            sign = 1 if i % 2 == 0 else -1
            f = _make_features(heading_error_deg=sign * 80.0,
                               heading_rate_dps=sign * 25.0)
            pilot.steer(f)

        gains = pilot.gains
        assert 0.5 <= gains["kp"] <= 4.0
        assert 0.0 <= gains["ki"] <= 0.5
        assert 0.5 <= gains["kd"] <= 4.0


class TestDivergenceReset:
    def test_divergence_resets(self):
        inner = PDPilot()
        pilot = AdaptivePilot(inner=inner, max_p_trace=10.0)
        # Artificially inflate P to trigger divergence reset
        pilot._P = np.eye(pilot._n) * 100.0
        f = _make_features(heading_error_deg=5.0)
        pilot._step = 1  # ensure update runs
        pilot._prev_error = 4.0
        pilot.steer(f)

        # After reset, gains should be back to defaults
        assert abs(pilot.gains["kp"] - 1.0) < 0.01
        assert abs(pilot.gains["kd"] - 1.5) < 0.01


class TestSaveLoad:
    def test_save_load_roundtrip(self):
        inner = PDPilot()
        pilot = AdaptivePilot(inner=inner)
        # Run a few steps to get non-default state
        for _ in range(10):
            pilot.steer(_make_features(heading_error_deg=3.0,
                                       heading_rate_dps=1.0))

        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            path = f.name
        pilot.save(path)

        inner2 = PDPilot()
        loaded = AdaptivePilot.load(path, inner=inner2)
        assert loaded._step == pilot._step
        np.testing.assert_allclose(loaded._x, pilot._x)
        np.testing.assert_allclose(loaded._P, pilot._P)
        assert abs(loaded._prev_error - pilot._prev_error) < 1e-10


class TestReset:
    def test_reset_clears_state(self):
        inner = PDPilot()
        pilot = AdaptivePilot(inner=inner)
        for _ in range(20):
            pilot.steer(_make_features(heading_error_deg=5.0))

        assert pilot._step > 0
        pilot.reset()
        assert pilot._step == 0
        assert abs(pilot._prev_error) < 1e-10
        assert abs(pilot._integrator_est) < 1e-10
        # Gains back to defaults
        assert abs(pilot.gains["kp"] - 1.0) < 0.01
        assert abs(pilot.gains["kd"] - 1.5) < 0.01


class TestCL:
    def test_cl_compass_small(self):
        inner = PDPilot()
        pilot = AdaptivePilot(inner=inner)
        results = evaluate_pilot(pilot, scenario_name="compass_small")
        assert results[0]["passed"], (
            f"compass_small failed: {results[0]['mean_ss_error_deg']:.1f}°"
        )

    def test_cl_no_regression(self):
        """Adaptive PD should not be much worse than static PD."""
        static = PDPilot()
        adaptive = AdaptivePilot(inner=PDPilot())

        static_results = evaluate_pilot(static)
        adaptive_results = evaluate_pilot(adaptive)

        for sr, ar in zip(static_results, adaptive_results):
            # Allow up to 1.5x the static error (with floor of 2°)
            limit = max(sr["mean_ss_error_deg"] * 1.5, 2.0)
            assert ar["mean_ss_error_deg"] <= limit, (
                f"{ar['name']}: adaptive {ar['mean_ss_error_deg']:.1f}° > "
                f"limit {limit:.1f}° (static {sr['mean_ss_error_deg']:.1f}°)"
            )
