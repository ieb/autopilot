"""
Mode Manager Module
===================

Manages autopilot operating modes and target calculations.
Handles mode transitions and VMG optimization.
"""

import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple, Callable
import math
import logging

from ..ml.polar import Polar
from ..sensors.nmea2000_interface import N2KData, calculate_true_wind

logger = logging.getLogger(__name__)


class AutopilotMode(Enum):
    """Autopilot operating modes."""
    STANDBY = auto()        # Not steering, observing only
    COMPASS = auto()        # Steer to magnetic heading
    WIND_AWA = auto()       # Steer to apparent wind angle
    WIND_TWA = auto()       # Steer to true wind angle
    VMG_UP = auto()         # Optimize upwind VMG
    VMG_DOWN = auto()       # Optimize downwind VMG
    TRACK = auto()          # Steer to waypoint (future)


@dataclass
class ModeState:
    """Current mode state."""
    mode: AutopilotMode = AutopilotMode.STANDBY
    target_value: float = 0.0          # Target heading/angle for current mode
    computed_heading: float = 0.0      # Computed target heading
    tack_side: int = 0                 # 1 for starboard tack, -1 for port
    
    # Mode info
    mode_name: str = "STANDBY"
    mode_description: str = "Autopilot disengaged"
    
    # Timing
    mode_start_time: float = 0.0
    mode_duration_s: float = 0.0


class ModeManager:
    """
    Manages autopilot modes and target calculations.
    
    Modes:
    - STANDBY: Not actively steering, observe and learn
    - COMPASS: Hold a magnetic heading
    - WIND_AWA: Hold an apparent wind angle (close-hauled)
    - WIND_TWA: Hold a true wind angle
    - VMG_UP/DOWN: Optimize velocity made good
    """
    
    def __init__(self, polar: Optional[Polar] = None):
        self.polar = polar or Polar.pogo_1250()
        self._state = ModeState()
        self._callbacks: list[Callable[[ModeState], None]] = []
        
        # VMG optimization state
        self._vmg_update_interval = 10.0  # Seconds between VMG angle updates
        self._last_vmg_update = 0.0
        self._optimal_twa = 45.0
        
    def set_mode(self, mode: AutopilotMode, target: float = 0.0, tack_side: int = 1):
        """
        Set autopilot mode.
        
        Args:
            mode: Target mode
            target: Target value (heading for COMPASS, angle for wind modes)
            tack_side: 1 for starboard tack, -1 for port (for VMG modes)
        """
        old_mode = self._state.mode
        
        self._state.mode = mode
        self._state.target_value = target
        self._state.tack_side = tack_side
        self._state.mode_start_time = time.time()
        
        # Update mode info
        self._update_mode_info()
        
        logger.info(
            f"Mode change: {old_mode.name} → {mode.name}, "
            f"target={target:.1f}, tack={tack_side}"
        )
        
        # Notify callbacks
        for callback in self._callbacks:
            try:
                callback(self._state)
            except Exception as e:
                logger.warning(f"Mode callback error: {e}")
                
    def _update_mode_info(self):
        """Update mode description strings."""
        mode = self._state.mode
        target = self._state.target_value
        
        if mode == AutopilotMode.STANDBY:
            self._state.mode_name = "STANDBY"
            self._state.mode_description = "Autopilot disengaged"
            
        elif mode == AutopilotMode.COMPASS:
            self._state.mode_name = "COMPASS"
            self._state.mode_description = f"Heading {target:.0f}°M"
            
        elif mode == AutopilotMode.WIND_AWA:
            side = "S" if target > 0 else "P"
            self._state.mode_name = f"AWA {side}"
            self._state.mode_description = f"Wind {abs(target):.0f}°{side}"
            
        elif mode == AutopilotMode.WIND_TWA:
            side = "S" if target > 0 else "P"
            self._state.mode_name = f"TWA {side}"
            self._state.mode_description = f"True {abs(target):.0f}°{side}"
            
        elif mode == AutopilotMode.VMG_UP:
            side = "STBD" if self._state.tack_side > 0 else "PORT"
            self._state.mode_name = f"VMG UP {side}"
            self._state.mode_description = f"Upwind VMG {side}"
            
        elif mode == AutopilotMode.VMG_DOWN:
            side = "STBD" if self._state.tack_side > 0 else "PORT"
            self._state.mode_name = f"VMG DN {side}"
            self._state.mode_description = f"Downwind VMG {side}"
            
    def update(self, heading: float, n2k: N2KData) -> float:
        """
        Update mode calculations and return target heading.
        
        Args:
            heading: Current magnetic heading (from IMU)
            n2k: Current NMEA2000 data
            
        Returns:
            Target heading to steer (degrees magnetic)
        """
        now = time.time()
        self._state.mode_duration_s = now - self._state.mode_start_time
        
        mode = self._state.mode
        target = self._state.target_value
        
        # Calculate true wind
        twa, tws = calculate_true_wind(n2k.awa, n2k.aws, n2k.stw, heading)
        
        if mode == AutopilotMode.STANDBY:
            # No target in standby
            self._state.computed_heading = heading
            return heading
            
        elif mode == AutopilotMode.COMPASS:
            # Direct heading target
            self._state.computed_heading = target
            return target
            
        elif mode == AutopilotMode.WIND_AWA:
            # Compute heading to achieve target AWA
            # target_heading = heading + (target_awa - current_awa)
            awa_error = target - n2k.awa
            target_heading = self._normalize_heading(heading + awa_error)
            self._state.computed_heading = target_heading
            return target_heading
            
        elif mode == AutopilotMode.WIND_TWA:
            # Compute heading to achieve target TWA
            twa_error = target - twa
            target_heading = self._normalize_heading(heading + twa_error)
            self._state.computed_heading = target_heading
            return target_heading
            
        elif mode == AutopilotMode.VMG_UP:
            # Optimize upwind VMG
            return self._compute_vmg_target(heading, tws, upwind=True)
            
        elif mode == AutopilotMode.VMG_DOWN:
            # Optimize downwind VMG
            return self._compute_vmg_target(heading, tws, upwind=False)
            
        return heading
        
    def _compute_vmg_target(self, heading: float, tws: float, upwind: bool) -> float:
        """Compute optimal VMG target heading."""
        now = time.time()
        
        # Periodically update optimal angle from polar
        if now - self._last_vmg_update > self._vmg_update_interval:
            if upwind:
                self._optimal_twa, _, _ = self.polar.get_optimal_vmg_upwind(tws)
            else:
                self._optimal_twa, _, _ = self.polar.get_optimal_vmg_downwind(tws)
            self._last_vmg_update = now
            logger.debug(f"VMG optimal TWA updated: {self._optimal_twa:.1f}°")
            
        # Apply tack side
        target_twa = self._optimal_twa * self._state.tack_side
        
        # Update stored target
        self._state.target_value = target_twa
        
        # This is now equivalent to TWA mode
        # TODO: Calculate heading from TWA
        # For now, return current heading (should compute from wind)
        self._state.computed_heading = heading
        return heading
        
    def _normalize_heading(self, heading: float) -> float:
        """Normalize heading to 0-360."""
        while heading < 0:
            heading += 360
        while heading >= 360:
            heading -= 360
        return heading
        
    def tack(self):
        """Execute a tack (flip tack side)."""
        if self._state.mode in [AutopilotMode.VMG_UP, AutopilotMode.VMG_DOWN,
                                 AutopilotMode.WIND_AWA, AutopilotMode.WIND_TWA]:
            self._state.tack_side *= -1
            self._state.target_value *= -1
            self._update_mode_info()
            logger.info(f"Tack to {'starboard' if self._state.tack_side > 0 else 'port'}")
            
    def adjust_target(self, delta: float):
        """
        Adjust target value.
        
        Args:
            delta: Amount to adjust (degrees)
        """
        if self._state.mode == AutopilotMode.STANDBY:
            return
            
        self._state.target_value += delta
        
        # Normalize based on mode
        if self._state.mode == AutopilotMode.COMPASS:
            self._state.target_value = self._normalize_heading(self._state.target_value)
        else:
            # Wind angles: clamp to valid range
            self._state.target_value = max(-180, min(180, self._state.target_value))
            
        self._update_mode_info()
        logger.info(f"Target adjusted to {self._state.target_value:.1f}")
        
    def add_callback(self, callback: Callable[[ModeState], None]):
        """Register callback for mode changes."""
        self._callbacks.append(callback)
        
    def get_state(self) -> ModeState:
        """Get current mode state."""
        return self._state
        
    @property
    def is_active(self) -> bool:
        """Check if autopilot is actively steering."""
        return self._state.mode != AutopilotMode.STANDBY
