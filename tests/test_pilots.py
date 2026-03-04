"""
Tests for the pilots package.
"""

import numpy as np
import pytest

from src.pilots import get_pilot, list_pilots, PILOT_REGISTRY
from src.pilots.base import BasePilot
from src.pilots.pd_pilot import PDPilot
from src.pilots.pid_pilot import PIDPilot


def _make_features(**overrides) -> np.ndarray:
    """Build a 22-element feature vector with specified overrides.

    Defaults to all zeros.  Keys are human-readable names mapped to indices:
        heading_error=0, mode=1, heading_rate=2, roll=3, pitch=4,
        roll_rate=5, awa=6, awa_rate=7, aws=8, twa=9, tws=10, stw=11,
        sog=12, cog_error=13, rudder_pos=14, rudder_vel=15,
        computed_heading=16, vmg_up=17, vmg_down=18, pd_suggestion=19,
        placeholder=20, wave_period=21
    """
    _NAMES = {
        "heading_error": 0, "mode": 1, "heading_rate": 2, "roll": 3,
        "pitch": 4, "roll_rate": 5, "awa": 6, "awa_rate": 7, "aws": 8,
        "twa": 9, "tws": 10, "stw": 11, "sog": 12, "cog_error": 13,
        "rudder_pos": 14, "rudder_vel": 15, "computed_heading": 16,
        "vmg_up": 17, "vmg_down": 18, "pd_suggestion": 19,
        "placeholder": 20, "wave_period": 21,
    }
    feat = np.zeros(22, dtype=np.float32)
    for k, v in overrides.items():
        feat[_NAMES[k]] = v
    return feat


# ── PD Pilot ──────────────────────────────────────────────────


class TestPDPilot:
    def test_zero_error_zero_output(self):
        pilot = PDPilot()
        features = _make_features()
        assert pilot.steer(features) == pytest.approx(0.0)

    def test_positive_error_positive_rudder(self):
        """Starboard error → starboard rudder (CLAUDE.md sign convention)."""
        pilot = PDPilot()
        features = _make_features(heading_error=0.5)  # 45° error
        result = pilot.steer(features)
        assert result > 0.0

    def test_negative_error_negative_rudder(self):
        """Port error → port rudder."""
        pilot = PDPilot()
        features = _make_features(heading_error=-0.5)
        result = pilot.steer(features)
        assert result < 0.0

    def test_heading_rate_damping(self):
        """Positive heading_rate (turning starboard) should oppose P term.

        With a positive heading error, positive heading_rate reduces output.
        """
        pilot = PDPilot()
        # Use small error so output doesn't saturate at clip boundary
        f_no_rate = _make_features(heading_error=0.1, heading_rate=0.0)
        f_with_rate = _make_features(heading_error=0.1, heading_rate=0.2)
        r_no_rate = pilot.steer(f_no_rate)
        r_with_rate = pilot.steer(f_with_rate)
        assert r_with_rate < r_no_rate

    def test_output_clipped(self):
        """Large error → output clamped to [-1, 1]."""
        pilot = PDPilot()
        features = _make_features(heading_error=1.0)  # 90° error
        result = pilot.steer(features)
        assert -1.0 <= result <= 1.0

    def test_sign_convention(self):
        """Validate against CLAUDE.md: positive error → positive rudder → starboard turn."""
        pilot = PDPilot()
        # 10° starboard error (normalised: 10/90 ≈ 0.111)
        features = _make_features(heading_error=10.0 / 90.0)
        result = pilot.steer(features)
        assert result > 0.0, "Positive heading error must produce positive (starboard) rudder"

    def test_reset(self):
        pilot = PDPilot()
        pilot.steer(_make_features(heading_error=0.5))
        pilot.reset()
        assert pilot._prev_command == 0.0

    def test_max_rudder_limits_output(self):
        pilot = PDPilot(max_rudder=0.5)
        features = _make_features(heading_error=1.0)
        result = pilot.steer(features)
        assert abs(result) <= 0.5


# ── PID Pilot ─────────────────────────────────────────────────


class TestPIDPilot:
    def test_integrator_accumulates(self):
        """Sustained error grows integrator."""
        pilot = PIDPilot(ki=0.5, dt=0.5)
        features = _make_features(heading_error=0.1)  # 9° error
        pilot.steer(features)
        assert pilot._integrator != 0.0

    def test_integrator_clamped(self):
        """Integrator doesn't exceed limit."""
        pilot = PIDPilot(ki=1.0, integrator_limit=5.0, dt=0.5)
        features = _make_features(heading_error=1.0)  # 90° error
        for _ in range(100):
            pilot.steer(features)
        assert abs(pilot._integrator) <= 5.0

    def test_antiwindup(self):
        """When output saturates, integrator stops growing in that direction."""
        pilot = PIDPilot(kp=1.6, ki=1.0, kd=0.0, dt=0.5,
                         integrator_limit=100.0)  # high limit so clamping doesn't mask
        features = _make_features(heading_error=1.0)  # 90° → will saturate

        # Run enough steps for integrator to grow
        for _ in range(20):
            pilot.steer(features)

        # Integrator should be bounded because anti-windup kicks in
        # The exact value depends on the anti-windup implementation,
        # but it should not keep growing linearly (20 * 90 * 0.5 = 900)
        assert pilot._integrator < 100.0

    def test_reset_clears_integrator(self):
        pilot = PIDPilot()
        pilot.steer(_make_features(heading_error=0.5))
        pilot.reset()
        assert pilot._integrator == 0.0
        assert pilot._prev_command == 0.0

    def test_ki_zero_matches_pd(self):
        """PID with ki=0 should equal PD output."""
        pd = PDPilot(kp=1.6, kd=1.5)
        pid = PIDPilot(kp=1.6, ki=0.0, kd=1.5)
        features = _make_features(heading_error=0.3, heading_rate=0.2)
        assert pd.steer(features) == pytest.approx(pid.steer(features))


# ── Registry ──────────────────────────────────────────────────


class TestPilotRegistry:
    def test_get_pilot(self):
        pilot = get_pilot("pd")
        assert isinstance(pilot, PDPilot)
        pilot = get_pilot("pid")
        assert isinstance(pilot, PIDPilot)

    def test_get_pilot_unknown(self):
        with pytest.raises(KeyError, match="Unknown pilot"):
            get_pilot("nonexistent")

    def test_list_pilots(self):
        names = list_pilots()
        assert "pd" in names
        assert "pid" in names

    def test_configure(self):
        pilot = get_pilot("pd")
        pilot.configure(kp=2.0, kd=1.8)
        assert pilot.kp == 2.0
        assert pilot.kd == 1.8

    def test_configure_ignores_unknown(self):
        """configure() silently ignores attributes that don't exist."""
        pilot = get_pilot("pd")
        pilot.configure(nonexistent=42)
        assert not hasattr(pilot, "nonexistent")

    def test_get_pilot_with_kwargs(self):
        pilot = get_pilot("pid", ki=0.5, integrator_limit=20.0)
        assert pilot.ki == 0.5
        assert pilot.integrator_limit == 20.0


# ── CL Runner ─────────────────────────────────────────────────


class TestCLRunner:
    def test_pd_passes_compass_small(self):
        """PD pilot passes the easiest CL scenario."""
        from src.pilots.evaluate import evaluate_pilot

        pilot = PDPilot()
        results = evaluate_pilot(pilot, scenario_name="compass_small")
        assert len(results) == 1
        r = results[0]
        assert r["name"] == "compass_small"
        assert r["passed"], (
            f"PD pilot failed compass_small: SS error {r['mean_ss_error_deg']}°"
        )
