"""
PD Pilot
========

Clean PD controller for heading/wind hold.
Stripped from helm_controller.py — no reaction delay, noise, fatigue,
anticipation, or skill scaling.
"""

import numpy as np

from .base import BasePilot


class PDPilot(BasePilot):
    """Proportional-Derivative autopilot controller.

    Uses heading_error (feature 0) and heading_rate (feature 2)
    from the normalised feature vector.  Sign convention matches
    CLAUDE.md: positive error → positive (starboard) rudder.
    """

    name = "pd"

    def __init__(self, kp: float = 1.0, kd: float = 1.5,
                 max_rudder: float = 1.0):
        self.kp = kp
        self.kd = kd
        self.max_rudder = max_rudder  # normalised, 1.0 = 25°
        self._prev_command = 0.0

    def steer(self, features: np.ndarray) -> float:
        heading_error = features[0] * 90.0    # denormalise to degrees
        heading_rate = features[2] * 30.0     # denormalise to deg/s

        # PD control: derivative = -heading_rate (damping opposes rotation)
        command_deg = self.kp * heading_error + self.kd * (-heading_rate)
        command_norm = np.clip(command_deg / 25.0,
                               -self.max_rudder, self.max_rudder)
        self._prev_command = float(command_norm)
        return float(command_norm)

    def reset(self) -> None:
        self._prev_command = 0.0
