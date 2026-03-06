"""
Smooth Pilot
=============

Progressive rudder application wrapper. Sits between an inner PD/PID
pilot and the output, applying three mechanisms:

1. Speed-dependent gain scaling — reduce command at higher boat speeds
2. Rate limiting with urgency ramp — limit how fast rudder changes
3. Turn-rate awareness — slow further application when already turning
"""

import numpy as np

from .base import BasePilot


class SmoothPilot(BasePilot):
    """Progressive rudder application wrapper.

    Args:
        inner: Inner pilot (PD, PID, etc.).
        dt: Expected time between steer() calls (seconds).
        base_rate_limit: Maximum rudder rate in deg/s at zero urgency.
        speed_ref: Reference boat speed (kn) where no scaling occurs.
        speed_gain_min: Minimum speed scaling factor (at high speed).
        speed_gain_max: Maximum speed scaling factor (at low speed).
        turn_rate_threshold: Turn rate (deg/s) above which backoff starts.
        turn_rate_gain: How aggressively to back off when turning fast.
    """

    name = "smooth"

    def __init__(
        self,
        inner: BasePilot,
        dt: float = 0.5,
        base_rate_limit: float = 3.0,
        speed_ref: float = 6.0,
        speed_gain_min: float = 0.4,
        speed_gain_max: float = 1.0,
        turn_rate_threshold: float = 3.0,
        turn_rate_gain: float = 0.5,
    ):
        self.inner = inner
        self.dt = dt
        self.base_rate_limit = base_rate_limit
        self.speed_ref = speed_ref
        self.speed_gain_min = speed_gain_min
        self.speed_gain_max = speed_gain_max
        self.turn_rate_threshold = turn_rate_threshold
        self.turn_rate_gain = turn_rate_gain

        self._prev_output = 0.0
        self._time_at_error = 0.0

    def steer(self, features: np.ndarray) -> float:
        heading_error_deg = abs(features[0] * 90.0)

        # Track time at error for urgency ramp
        if heading_error_deg > 1.0:
            self._time_at_error += self.dt
        else:
            self._time_at_error = max(0.0, self._time_at_error - self.dt)

        # 1. Speed-dependent gain scaling
        stw = features[11] * 25.0  # boat speed (knots)
        speed_factor = self.speed_ref / max(stw, 1.0)
        speed_factor = np.clip(speed_factor, self.speed_gain_min,
                               self.speed_gain_max)
        target = self.inner.steer(features) * speed_factor

        # 2. Rate limiting with urgency ramp and error scaling
        urgency = min(1.0, self._time_at_error / 30.0)
        # Scale rate by error: small errors stay smooth, large errors ramp fast
        error_scale = min(3.0, 1.0 + heading_error_deg / 15.0)
        effective_rate = self.base_rate_limit * error_scale * (1.0 + 2.0 * urgency)
        max_change = (effective_rate / 25.0) * self.dt

        # 3. Turn-rate awareness (only when increasing rudder magnitude)
        turn_rate = abs(features[2] * 30.0)  # deg/s
        if turn_rate > self.turn_rate_threshold:
            excess = (turn_rate - self.turn_rate_threshold) / 3.0
            max_change *= 1.0 / (1.0 + self.turn_rate_gain * excess)

        # Asymmetric rate limit: full rate limit when building rudder,
        # 3x faster when reducing or reversing (allows quick correction)
        delta = target - self._prev_output
        reducing = abs(target) < abs(self._prev_output)
        reversing = (target * self._prev_output < 0) and abs(self._prev_output) > 0.02
        effective_max = max_change * 3.0 if (reducing or reversing) else max_change
        delta = np.clip(delta, -effective_max, effective_max)
        self._prev_output += delta

        return float(np.clip(self._prev_output, -1.0, 1.0))

    def reset(self) -> None:
        self._prev_output = 0.0
        self._time_at_error = 0.0
        self.inner.reset()
