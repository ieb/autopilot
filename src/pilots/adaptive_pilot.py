"""
Adaptive Pilot
==============

EKF-based online gain tuning wrapper for PD/PID pilots.
Treats [kp, kd] (or [kp, ki, kd]) as hidden state and adapts
them based on observed heading error dynamics.
"""

import json
import logging
from pathlib import Path

import numpy as np

from .base import BasePilot
from .pid_pilot import PIDPilot

logger = logging.getLogger(__name__)


class AdaptivePilot(BasePilot):
    """Adaptive gain tuning via Extended Kalman Filter.

    Wraps a PD or PID inner pilot and adjusts its gains online
    by treating them as EKF hidden state.  The measurement model
    uses the heading error dynamics: how much the error changed
    vs. what the rudder command should have produced.

    Args:
        inner: A PDPilot or PIDPilot instance.
        dt: Expected time between steer() calls (seconds).
        q_diag: Process noise variance per gain (random walk).
        r_scalar: Measurement noise variance.
        rudder_eff: Estimated deg/s heading change per deg rudder.
        kp_bounds: (min, max) for proportional gain.
        kd_bounds: (min, max) for derivative gain.
        ki_bounds: (min, max) for integral gain (PID only).
        max_p_trace: If trace(P) exceeds this, reset to defaults.
        log_interval: Log adapted gains every N steps.
    """

    name = "adaptive"

    def __init__(
        self,
        inner: BasePilot,
        dt: float = 0.5,
        q_diag: float = 1e-6,
        r_scalar: float = 1.0,
        rudder_eff: float = 0.5,
        kp_bounds: tuple[float, float] = (0.5, 4.0),
        kd_bounds: tuple[float, float] = (0.5, 4.0),
        ki_bounds: tuple[float, float] = (0.0, 0.5),
        max_p_trace: float = 10.0,
        log_interval: int = 20,
    ):
        self.inner = inner
        self.dt = dt
        self.q_diag = q_diag
        self.r_scalar = r_scalar
        self.rudder_eff = rudder_eff
        self.kp_bounds = kp_bounds
        self.kd_bounds = kd_bounds
        self.ki_bounds = ki_bounds
        self.max_p_trace = max_p_trace
        self.log_interval = log_interval

        # Detect dimensionality from inner pilot type
        self._has_ki = isinstance(inner, PIDPilot)
        self._n = 3 if self._has_ki else 2

        # Store original gains as defaults for reset
        self._default_kp = inner.kp
        self._default_kd = inner.kd
        self._default_ki = getattr(inner, "ki", 0.0)

        self._init_ekf()

    def _init_ekf(self) -> None:
        """Initialise EKF state from inner pilot's current gains."""
        if self._has_ki:
            self._x = np.array([self._default_kp, self._default_ki,
                                self._default_kd], dtype=np.float64)
            self._bounds = np.array([
                self.kp_bounds, self.ki_bounds, self.kd_bounds
            ], dtype=np.float64)
        else:
            self._x = np.array([self._default_kp, self._default_kd],
                               dtype=np.float64)
            self._bounds = np.array([
                self.kp_bounds, self.kd_bounds
            ], dtype=np.float64)

        self._P = np.eye(self._n, dtype=np.float64) * 0.1
        self._Q = np.eye(self._n, dtype=np.float64) * self.q_diag
        self._R = np.array([[self.r_scalar]], dtype=np.float64)

        self._prev_error = 0.0
        self._integrator_est = 0.0
        self._step = 0

    def _clamp_gains(self) -> None:
        """Clamp EKF state to safety bounds."""
        self._x = np.clip(self._x, self._bounds[:, 0], self._bounds[:, 1])

    def _apply_gains(self) -> None:
        """Push current EKF gain estimates to the inner pilot."""
        if self._has_ki:
            self.inner.configure(kp=float(self._x[0]),
                                 ki=float(self._x[1]),
                                 kd=float(self._x[2]))
        else:
            self.inner.configure(kp=float(self._x[0]),
                                 kd=float(self._x[1]))

    def steer(self, features: np.ndarray) -> float:
        e = features[0] * 90.0      # heading error (deg)
        rate = features[2] * 30.0   # heading rate (deg/s)

        # Build regressor: what each gain multiplies
        if self._has_ki:
            self._integrator_est += e * self.dt
            phi = np.array([e, self._integrator_est, -rate],
                           dtype=np.float64)
        else:
            phi = np.array([e, -rate], dtype=np.float64)

        # EKF update (skip during large manoeuvres or fast transients)
        if self._step > 0 and abs(e) < 45.0 and abs(rate) < 15.0:
            # Innovation: observed error change vs free-dynamics prediction
            # de/dt = -heading_rate, so predicted e = prev_error - rate*dt
            innovation = e - self._prev_error + rate * self.dt

            # Jacobian of measurement w.r.t. gains
            H = (-self.rudder_eff * self.dt * phi).reshape(1, self._n)

            # Standard EKF update
            S = H @ self._P @ H.T + self._R
            K = self._P @ H.T / float(S[0, 0])
            self._x = self._x + K.flatten() * innovation
            self._P = self._P - K @ H @ self._P + self._Q

            # Clamp and check for divergence
            self._clamp_gains()
            if np.trace(self._P) > self.max_p_trace:
                logger.warning("EKF divergence detected (trace=%.2f), "
                               "resetting to defaults", np.trace(self._P))
                self._reset_to_defaults()

        self._prev_error = e
        self._step += 1

        # Apply adapted gains to inner pilot
        self._apply_gains()

        # Log periodically
        if self._step % self.log_interval == 0:
            logger.debug("Adaptive step %d: gains=%s trace(P)=%.4f",
                         self._step, self.gains, np.trace(self._P))

        return self.inner.steer(features)

    def _reset_to_defaults(self) -> None:
        """Reset EKF state to inner pilot's original default gains."""
        if self._has_ki:
            self._x = np.array([self._default_kp, self._default_ki,
                                self._default_kd], dtype=np.float64)
        else:
            self._x = np.array([self._default_kp, self._default_kd],
                               dtype=np.float64)
        self._P = np.eye(self._n, dtype=np.float64) * 0.1

    def reset(self) -> None:
        self._init_ekf()
        self.inner.reset()

    def save(self, path: str) -> None:
        """Save EKF state to JSON."""
        state = {
            "x": self._x.tolist(),
            "P": self._P.tolist(),
            "step": int(self._step),
            "prev_error": float(self._prev_error),
            "integrator_est": float(self._integrator_est),
            "has_ki": self._has_ki,
        }
        Path(path).write_text(json.dumps(state, indent=2))

    @classmethod
    def load(cls, path: str, inner: BasePilot) -> "AdaptivePilot":
        """Load EKF state from JSON, wrapping the given inner pilot."""
        state = json.loads(Path(path).read_text())
        pilot = cls(inner=inner)
        pilot._x = np.array(state["x"], dtype=np.float64)
        pilot._P = np.array(state["P"], dtype=np.float64)
        pilot._step = state["step"]
        pilot._prev_error = state["prev_error"]
        pilot._integrator_est = state["integrator_est"]
        pilot._apply_gains()
        return pilot

    @property
    def gains(self) -> dict:
        if self._has_ki:
            return {"kp": float(self._x[0]), "ki": float(self._x[1]),
                    "kd": float(self._x[2])}
        return {"kp": float(self._x[0]), "kd": float(self._x[1])}

    @property
    def confidence(self) -> float:
        """Inverse of trace(P) — higher means more certain."""
        return 1.0 / (1.0 + np.trace(self._P))
