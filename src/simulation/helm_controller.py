"""
Helm Controller
===============

Simulates human-like steering behavior with PD control,
rate limiting, and realistic variations.
"""

import math
import random
from dataclasses import dataclass
from typing import Optional, Tuple
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class SteeringMode(Enum):
    """Steering mode determines control behavior."""
    COMPASS = "compass"      # Maintain magnetic heading
    WIND_AWA = "wind_awa"    # Maintain apparent wind angle
    WIND_TWA = "wind_twa"    # Maintain true wind angle
    STANDBY = "standby"      # No active steering


@dataclass
class HelmConfig:
    """Configuration for helm controller."""
    # PD gains for compass mode
    compass_kp: float = 0.3          # Proportional gain
    compass_kd: float = 0.5          # Derivative gain
    
    # PD gains for AWA mode (more responsive)
    awa_kp: float = 0.25
    awa_kd: float = 0.6
    
    # PD gains for TWA mode (smoother downwind)
    twa_kp: float = 0.2
    twa_kd: float = 0.4
    
    # Rudder limits
    max_rudder_angle: float = 25.0   # Maximum rudder deflection (degrees)
    max_rudder_rate: float = 4.0     # Maximum rate (deg/s) - Jefa LD100 spec
    
    # Human-like behavior
    reaction_delay: float = 0.15     # Reaction delay (seconds)
    noise_std: float = 0.3           # Random noise std dev (degrees)
    anticipation_factor: float = 0.1 # Anticipation of wind shifts
    
    # Fatigue model
    fatigue_enabled: bool = True
    fatigue_time_constant: float = 3600.0  # Time for full fatigue (seconds)
    fatigue_noise_factor: float = 2.0      # Noise multiplier at full fatigue
    fatigue_delay_factor: float = 1.5      # Delay multiplier at full fatigue
    
    # Skill level (0.5 = novice, 1.0 = expert)
    skill_level: float = 0.85


@dataclass
class HelmState:
    """Current helm state."""
    mode: SteeringMode = SteeringMode.COMPASS
    target_heading: float = 0.0      # Target for compass mode
    target_awa: float = 45.0         # Target for AWA mode
    target_twa: float = 135.0        # Target for TWA mode
    
    rudder_command: float = 0.0      # Current rudder command (degrees)
    last_error: float = 0.0          # Previous error for derivative
    error_integral: float = 0.0      # Integrated error (not used currently)
    
    elapsed_time: float = 0.0        # Total elapsed time (for fatigue)
    
    # Delayed inputs buffer (for reaction delay simulation)
    delayed_heading: float = 0.0
    delayed_awa: float = 0.0
    delayed_twa: float = 0.0


class HelmController:
    """
    Human-like helm controller.
    
    Simulates realistic steering behavior including:
    - Mode-specific PD control
    - Reaction delays
    - Natural variation and noise
    - Fatigue effects over time
    """
    
    def __init__(self, config: Optional[HelmConfig] = None, seed: Optional[int] = None):
        """
        Initialize helm controller.
        
        Args:
            config: Helm configuration
            seed: Random seed for reproducibility
        """
        self.config = config or HelmConfig()
        self.state = HelmState()
        
        if seed is not None:
            random.seed(seed)
            
        # Input buffer for delay simulation
        self._input_buffer = []
        self._buffer_time = 0.0
        
    def compute_rudder(self, heading: float, awa: float, twa: float,
                       heading_rate: float, dt: float) -> float:
        """
        Compute rudder command based on current state.
        
        Args:
            heading: Current magnetic heading (degrees)
            awa: Current apparent wind angle (degrees)
            twa: Current true wind angle (degrees)
            heading_rate: Current heading rate (deg/s)
            dt: Time step (seconds)
            
        Returns:
            Rudder command (degrees)
        """
        if self.state.mode == SteeringMode.STANDBY:
            return 0.0
            
        # Update elapsed time for fatigue
        self.state.elapsed_time += dt
        
        # Apply reaction delay
        delayed = self._apply_delay(heading, awa, twa, dt)
        
        # Compute error based on mode
        error = self._compute_error(delayed)
        
        # Get appropriate gains
        kp, kd = self._get_gains()
        
        # Apply skill level
        kp *= self.config.skill_level
        kd *= self.config.skill_level
        
        # PD control
        # For compass mode, error = target - heading, so d(error)/dt = -heading_rate
        # The derivative term should oppose the current motion to provide damping
        if self.state.mode == SteeringMode.COMPASS:
            # Negative sign because error derivative = -heading_rate
            derivative = -heading_rate
        else:
            derivative = (error - self.state.last_error) / dt if dt > 0 else 0.0
        
        command = kp * error + kd * derivative
        
        # Apply anticipation (slightly lead the error)
        anticipation = self._compute_anticipation(error, derivative)
        command += anticipation
        
        # Add human noise
        noise = self._compute_noise()
        command += noise
        
        # Rate limit
        command = self._apply_rate_limit(command, dt)
        
        # Clamp to rudder limits
        command = max(-self.config.max_rudder_angle,
                     min(self.config.max_rudder_angle, command))
        
        # Update state
        self.state.rudder_command = command
        self.state.last_error = error
        
        return command
        
    def _apply_delay(self, heading: float, awa: float, twa: float, 
                     dt: float) -> Tuple[float, float, float]:
        """Apply reaction delay to inputs."""
        # Get effective delay with fatigue
        delay = self.config.reaction_delay
        if self.config.fatigue_enabled:
            fatigue = self._get_fatigue_factor()
            delay *= (1.0 + (self.config.fatigue_delay_factor - 1.0) * fatigue)
            
        # Add to buffer
        self._input_buffer.append((self._buffer_time, heading, awa, twa))
        self._buffer_time += dt
        
        # Find delayed values
        target_time = self._buffer_time - delay
        
        # Remove old entries and find closest
        while len(self._input_buffer) > 1 and self._input_buffer[0][0] < target_time - 1.0:
            self._input_buffer.pop(0)
            
        if len(self._input_buffer) == 0:
            return (heading, awa, twa)
            
        # Find entry closest to target time
        for entry in self._input_buffer:
            if entry[0] >= target_time:
                self.state.delayed_heading = entry[1]
                self.state.delayed_awa = entry[2]
                self.state.delayed_twa = entry[3]
                return (entry[1], entry[2], entry[3])
                
        # Use most recent if no delayed entry available
        entry = self._input_buffer[-1]
        return (entry[1], entry[2], entry[3])
        
    def _compute_error(self, delayed: Tuple[float, float, float]) -> float:
        """
        Compute steering error based on mode.
        
        Sign convention: positive error means we need to turn to port (left).
        This requires NEGATIVE rudder to produce the correct turn direction.
        Error = target - current, so positive error = target is to port.
        """
        heading, awa, twa = delayed
        
        if self.state.mode == SteeringMode.COMPASS:
            # target - heading: positive when target is to port (left)
            return self._angle_diff(self.state.target_heading, heading)
        elif self.state.mode == SteeringMode.WIND_AWA:
            return self._angle_diff(self.state.target_awa, awa)
        elif self.state.mode == SteeringMode.WIND_TWA:
            return self._angle_diff(self.state.target_twa, twa)
        return 0.0
        
    def _get_gains(self) -> Tuple[float, float]:
        """Get PD gains for current mode."""
        if self.state.mode == SteeringMode.COMPASS:
            return (self.config.compass_kp, self.config.compass_kd)
        elif self.state.mode == SteeringMode.WIND_AWA:
            return (self.config.awa_kp, self.config.awa_kd)
        elif self.state.mode == SteeringMode.WIND_TWA:
            return (self.config.twa_kp, self.config.twa_kd)
        return (0.0, 0.0)
        
    def _compute_anticipation(self, error: float, derivative: float) -> float:
        """Compute anticipatory correction."""
        # Anticipate based on error trend
        if abs(derivative) > 1.0:  # Only anticipate significant changes
            anticipation = self.config.anticipation_factor * derivative
            return anticipation * self.config.skill_level
        return 0.0
        
    def _compute_noise(self) -> float:
        """Compute human-like noise."""
        noise_std = self.config.noise_std
        
        # Increase noise with fatigue
        if self.config.fatigue_enabled:
            fatigue = self._get_fatigue_factor()
            noise_std *= (1.0 + (self.config.fatigue_noise_factor - 1.0) * fatigue)
            
        # Reduce noise with skill
        noise_std /= self.config.skill_level
        
        return random.gauss(0, noise_std)
        
    def _apply_rate_limit(self, command: float, dt: float) -> float:
        """Apply rate limiting to rudder command."""
        max_change = self.config.max_rudder_rate * dt
        diff = command - self.state.rudder_command
        diff = max(-max_change, min(max_change, diff))
        return self.state.rudder_command + diff
        
    def _get_fatigue_factor(self) -> float:
        """Get current fatigue factor (0 = fresh, 1 = fully fatigued)."""
        return min(1.0, self.state.elapsed_time / self.config.fatigue_time_constant)
        
    def _angle_diff(self, a: float, b: float) -> float:
        """Compute signed angle difference (a - b)."""
        diff = a - b
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff
        
    def set_mode(self, mode: SteeringMode, target: float = 0.0):
        """
        Set steering mode and target.
        
        Args:
            mode: New steering mode
            target: Target value (heading, AWA, or TWA depending on mode)
        """
        self.state.mode = mode
        
        if mode == SteeringMode.COMPASS:
            self.state.target_heading = target % 360
        elif mode == SteeringMode.WIND_AWA:
            self.state.target_awa = target
        elif mode == SteeringMode.WIND_TWA:
            self.state.target_twa = target
            
        # Reset error state on mode change
        self.state.last_error = 0.0
        self.state.error_integral = 0.0
        
    def set_target(self, target: float):
        """Update target value for current mode."""
        if self.state.mode == SteeringMode.COMPASS:
            self.state.target_heading = target % 360
        elif self.state.mode == SteeringMode.WIND_AWA:
            self.state.target_awa = target
        elif self.state.mode == SteeringMode.WIND_TWA:
            self.state.target_twa = target
            
    def get_target(self) -> float:
        """Get current target value."""
        if self.state.mode == SteeringMode.COMPASS:
            return self.state.target_heading
        elif self.state.mode == SteeringMode.WIND_AWA:
            return self.state.target_awa
        elif self.state.mode == SteeringMode.WIND_TWA:
            return self.state.target_twa
        return 0.0
        
    def reset(self):
        """Reset helm state."""
        self.state = HelmState()
        self._input_buffer = []
        self._buffer_time = 0.0
        
    @classmethod
    def novice(cls) -> 'HelmController':
        """Create a novice helm controller."""
        config = HelmConfig(
            skill_level=0.6,
            noise_std=0.5,
            reaction_delay=0.25,
        )
        return cls(config)
        
    @classmethod
    def expert(cls) -> 'HelmController':
        """Create an expert helm controller."""
        config = HelmConfig(
            skill_level=1.0,
            noise_std=0.2,
            reaction_delay=0.1,
            anticipation_factor=0.15,
        )
        return cls(config)
