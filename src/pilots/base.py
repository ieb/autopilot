"""
Base Pilot
==========

Abstract base class for autopilot controllers.
"""

from abc import ABC, abstractmethod
import numpy as np


class BasePilot(ABC):
    """Base class for autopilot controllers.

    All pilots receive a 22-element normalised feature vector
    (see data_loader.py:compute_features) and return a rudder
    command in [-1, 1] (±25°).
    """

    name: str  # e.g. "pd", "pid"

    @abstractmethod
    def steer(self, features: np.ndarray) -> float:
        """Compute rudder command from a 22-element feature vector.

        Args:
            features: normalised feature vector (see data_loader.py)
                [0] heading_error   (error / 90°)
                [1] mode_flag       (compass=0, awa=0.5, twa=1.0)
                [2] heading_rate    (rate / 30 deg/s)
                [6] AWA             (awa / 180°)
                [9] TWA             (twa / 180°)
                [14] rudder_pos     (pos / 25°, zeroed)
                [19] pd_suggestion  (the sim PD value, for reference)

        Returns:
            Rudder command in [-1, 1] (±25°)
        """
        ...

    @abstractmethod
    def reset(self) -> None:
        """Reset internal state (integrator, etc.) between scenarios."""
        ...

    def configure(self, **kwargs) -> None:
        """Override gains or parameters at runtime (from CLI flags)."""
        for k, v in kwargs.items():
            if hasattr(self, k):
                setattr(self, k, v)
