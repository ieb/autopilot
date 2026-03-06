"""
PID Pilot
=========

PID controller with integral term and anti-windup.
Extends the PD pilot with accumulated error correction.
"""

import numpy as np

from .base import BasePilot


class PIDPilot(BasePilot):
    """Proportional-Integral-Derivative autopilot controller.

    Adds an integral term to the PD controller for eliminating
    steady-state error.  Anti-windup prevents integrator saturation:
    the integrator is clamped, and accumulation is undone when
    the output is saturated in the same direction.
    """

    name = "pid"

    def __init__(self, kp: float = 1.0, ki: float = 0.1, kd: float = 1.5,
                 max_rudder: float = 1.0, integrator_limit: float = 10.0,
                 dt: float = 0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_rudder = max_rudder
        self.integrator_limit = integrator_limit  # degrees
        self.dt = dt  # seconds between calls (CL runner calls at inference_hz=2)
        self._integrator = 0.0
        self._prev_command = 0.0

    def steer(self, features: np.ndarray) -> float:
        heading_error = features[0] * 90.0    # denormalise to degrees
        heading_rate = features[2] * 30.0     # denormalise to deg/s

        # Accumulate integral, tracking actual delta after clamping
        prev_integrator = self._integrator
        self._integrator += heading_error * self.dt
        self._integrator = float(np.clip(self._integrator,
                                          -self.integrator_limit,
                                          self.integrator_limit))
        actual_delta = self._integrator - prev_integrator

        command_deg = (self.kp * heading_error
                       + self.ki * self._integrator
                       + self.kd * (-heading_rate))
        command_norm = float(np.clip(command_deg / 25.0,
                                      -self.max_rudder, self.max_rudder))

        # Anti-windup back-calculation: if output is saturated,
        # undo this step's accumulation to prevent integrator growth.
        if abs(command_norm) >= self.max_rudder:
            if np.sign(heading_error) == np.sign(self._integrator):
                self._integrator -= actual_delta

        self._prev_command = command_norm
        return command_norm

    def reset(self) -> None:
        self._integrator = 0.0
        self._prev_command = 0.0
