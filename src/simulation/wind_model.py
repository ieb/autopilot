"""
Wind Model
==========

Realistic wind simulation with shifts, gusts, oscillations, and lulls.
"""

import math
import random
from dataclasses import dataclass
from typing import Optional
import logging

logger = logging.getLogger(__name__)


@dataclass
class WindConfig:
    """Configuration for wind model."""
    # Base wind
    base_tws_min: float = 8.0           # Minimum TWS (knots)
    base_tws_max: float = 25.0          # Maximum TWS (knots)
    base_twd: float = 180.0             # Initial TWD (degrees)
    
    # Gradual shifts
    shift_rate: float = 0.5             # Average shift rate (deg/minute)
    shift_persistence: float = 0.98     # How long shifts persist (0-1)
    
    # Oscillations (geographic/thermal effects)
    oscillation_enabled: bool = True
    oscillation_period: float = 300.0   # Period in seconds (5 min)
    oscillation_amplitude: float = 10.0 # Amplitude in degrees
    
    # Gusts
    gust_probability: float = 0.02      # Probability per second
    gust_intensity_min: float = 1.2     # Minimum gust multiplier
    gust_intensity_max: float = 1.5     # Maximum gust multiplier
    gust_duration: float = 10.0         # Typical gust duration (seconds)
    gust_decay: float = 0.9             # Decay rate per timestep
    
    # Lulls
    lull_probability: float = 0.01      # Probability per second
    lull_intensity_min: float = 0.6     # Minimum lull multiplier
    lull_intensity_max: float = 0.8     # Maximum lull multiplier
    lull_duration: float = 15.0         # Typical lull duration (seconds)


@dataclass
class WindState:
    """Current wind state."""
    twd: float = 180.0          # True wind direction (degrees)
    tws: float = 15.0           # True wind speed (knots)
    
    # Internal state
    base_tws: float = 15.0      # Base wind speed before gusts/lulls
    shift_velocity: float = 0.0 # Current shift rate (deg/s)
    gust_factor: float = 1.0    # Current gust/lull multiplier
    gust_time_remaining: float = 0.0  # Time until gust/lull ends
    oscillation_phase: float = 0.0    # Current oscillation phase


class WindModel:
    """
    Realistic wind simulation model.
    
    Generates time-varying wind conditions with:
    - Random walk direction shifts
    - Sinusoidal oscillations
    - Probabilistic gusts and lulls
    """
    
    def __init__(self, config: Optional[WindConfig] = None, seed: Optional[int] = None):
        """
        Initialize wind model.
        
        Args:
            config: Wind configuration
            seed: Random seed for reproducibility
        """
        self.config = config or WindConfig()
        self.state = WindState()
        
        if seed is not None:
            random.seed(seed)
            
        self._initialize()
        
    def _initialize(self):
        """Initialize wind state."""
        self.state.twd = self.config.base_twd
        self.state.base_tws = random.uniform(
            self.config.base_tws_min, 
            self.config.base_tws_max
        )
        self.state.tws = self.state.base_tws
        self.state.gust_factor = 1.0
        self.state.gust_time_remaining = 0.0
        self.state.oscillation_phase = random.uniform(0, 2 * math.pi)
        
    def step(self, dt: float) -> WindState:
        """
        Advance wind state by one timestep.
        
        Args:
            dt: Time step in seconds
            
        Returns:
            Updated wind state
        """
        self._update_shift(dt)
        self._update_oscillation(dt)
        self._update_gusts_lulls(dt)
        self._compute_final_wind()
        
        return self.state
        
    def _update_shift(self, dt: float):
        """Update gradual wind shifts using random walk."""
        # Add random acceleration to shift velocity
        acceleration = random.gauss(0, self.config.shift_rate * dt / 60.0)
        self.state.shift_velocity += acceleration
        
        # Apply persistence (damping)
        self.state.shift_velocity *= self.config.shift_persistence
        
        # Limit maximum shift rate
        max_rate = self.config.shift_rate * 3 / 60.0  # deg/s
        self.state.shift_velocity = max(-max_rate, 
                                        min(max_rate, self.state.shift_velocity))
        
        # Update direction
        self.state.twd += self.state.shift_velocity * dt
        self.state.twd %= 360
        
    def _update_oscillation(self, dt: float):
        """Update sinusoidal oscillation component."""
        if not self.config.oscillation_enabled:
            return
            
        # Advance phase
        omega = 2 * math.pi / self.config.oscillation_period
        self.state.oscillation_phase += omega * dt
        self.state.oscillation_phase %= (2 * math.pi)
        
    def _update_gusts_lulls(self, dt: float):
        """Update gust and lull events."""
        # If currently in a gust/lull, decay it
        if self.state.gust_time_remaining > 0:
            self.state.gust_time_remaining -= dt
            
            if self.state.gust_time_remaining <= 0:
                # Gust/lull ended, return to normal
                self.state.gust_factor = 1.0
                self.state.gust_time_remaining = 0.0
            else:
                # Decay gust intensity toward 1.0
                decay = self.config.gust_decay ** dt
                self.state.gust_factor = 1.0 + (self.state.gust_factor - 1.0) * decay
        else:
            # Check for new gust
            if random.random() < self.config.gust_probability * dt:
                self.state.gust_factor = random.uniform(
                    self.config.gust_intensity_min,
                    self.config.gust_intensity_max
                )
                self.state.gust_time_remaining = random.uniform(
                    self.config.gust_duration * 0.5,
                    self.config.gust_duration * 1.5
                )
            # Check for new lull
            elif random.random() < self.config.lull_probability * dt:
                self.state.gust_factor = random.uniform(
                    self.config.lull_intensity_min,
                    self.config.lull_intensity_max
                )
                self.state.gust_time_remaining = random.uniform(
                    self.config.lull_duration * 0.5,
                    self.config.lull_duration * 1.5
                )
                
    def _compute_final_wind(self):
        """Compute final wind values with all effects."""
        # Apply oscillation to direction
        if self.config.oscillation_enabled:
            oscillation = self.config.oscillation_amplitude * math.sin(
                self.state.oscillation_phase
            )
            effective_twd = self.state.twd + oscillation
        else:
            effective_twd = self.state.twd
            
        self.state.twd = effective_twd % 360
        
        # Apply gust/lull factor to speed
        self.state.tws = self.state.base_tws * self.state.gust_factor
        
        # Clamp to reasonable range
        self.state.tws = max(0, min(60, self.state.tws))
        
    def set_wind(self, twd: float, tws: float):
        """Set wind conditions directly."""
        self.state.twd = twd % 360
        self.state.tws = tws
        self.state.base_tws = tws
        
    def get_wind(self) -> tuple:
        """Get current wind as (TWD, TWS) tuple."""
        return (self.state.twd, self.state.tws)
        
    def trigger_gust(self, intensity: float = 1.3, duration: float = 10.0):
        """Manually trigger a gust event."""
        self.state.gust_factor = intensity
        self.state.gust_time_remaining = duration
        
    def trigger_lull(self, intensity: float = 0.7, duration: float = 15.0):
        """Manually trigger a lull event."""
        self.state.gust_factor = intensity
        self.state.gust_time_remaining = duration
        
    def reset(self, twd: Optional[float] = None, tws: Optional[float] = None):
        """Reset wind to initial conditions."""
        self._initialize()
        if twd is not None:
            self.state.twd = twd % 360
        if tws is not None:
            self.state.tws = tws
            self.state.base_tws = tws
