"""
Wave Model
==========

Simulates wave-induced pitch and roll motion for realistic IMU data.
"""

import math
import random
from dataclasses import dataclass
from typing import Optional, Tuple
import logging

logger = logging.getLogger(__name__)


@dataclass
class WaveConfig:
    """Configuration for wave model."""
    # Dominant wave (swell)
    swell_enabled: bool = True
    swell_period_min: float = 6.0        # Minimum period (seconds)
    swell_period_max: float = 12.0       # Maximum period (seconds)
    swell_amplitude_min: float = 2.0     # Minimum amplitude (degrees)
    swell_amplitude_max: float = 8.0     # Maximum amplitude (degrees)
    
    # Secondary swell
    secondary_swell_enabled: bool = True
    secondary_period_factor: float = 1.5  # Period relative to primary
    secondary_amplitude_factor: float = 0.4  # Amplitude relative to primary
    secondary_phase_offset: float = 0.7   # Phase offset (radians)
    
    # Wind chop (high frequency)
    chop_enabled: bool = True
    chop_period_min: float = 2.0         # Minimum period (seconds)
    chop_period_max: float = 4.0         # Maximum period (seconds)
    chop_amplitude_factor: float = 0.3   # Amplitude relative to swell
    
    # Wave-wind correlation
    amplitude_per_knot: float = 0.15     # Additional amplitude per knot of wind
    
    # Pitch-roll phase relationship
    pitch_roll_phase: float = math.pi / 2  # Pitch leads roll by 90 degrees
    
    # Wave-induced yaw (heading perturbation)
    # Downwind: stern lift causes boat to round up toward wind
    # Upwind: waves knock boat off wind (away from wind)
    yaw_effect_enabled: bool = True
    downwind_yaw_factor: float = 0.1     # deg/s per degree heel per meter swell (downwind)
    upwind_yaw_factor: float = 0.025     # deg/s per degree heel per meter swell (upwind)


@dataclass
class WaveState:
    """Current wave state."""
    # Primary swell
    swell_period: float = 8.0        # Current swell period (seconds)
    swell_amplitude: float = 4.0     # Current swell amplitude (degrees)
    swell_phase: float = 0.0         # Current swell phase (radians)
    
    # Secondary swell
    secondary_phase: float = 0.0     # Secondary swell phase
    
    # Chop
    chop_period: float = 3.0         # Current chop period
    chop_phase: float = 0.0          # Current chop phase
    
    # Output motion
    wave_roll: float = 0.0           # Wave-induced roll (degrees)
    wave_pitch: float = 0.0          # Wave-induced pitch (degrees)
    wave_heave: float = 0.0          # Vertical motion (m/s^2 for accel)
    wave_yaw: float = 0.0            # Wave-induced yaw rate (deg/s)


class WaveModel:
    """
    Wave-induced motion model.
    
    Generates realistic pitch and roll variations from:
    - Primary ocean swell
    - Secondary cross-swell
    - Wind-driven chop
    """
    
    def __init__(self, config: Optional[WaveConfig] = None, seed: Optional[int] = None):
        """
        Initialize wave model.
        
        Args:
            config: Wave configuration
            seed: Random seed for reproducibility
        """
        self.config = config or WaveConfig()
        self.state = WaveState()
        
        if seed is not None:
            random.seed(seed)
            
        self._initialize()
        
    def _initialize(self):
        """Initialize wave state with random parameters."""
        # Randomize swell characteristics
        self.state.swell_period = random.uniform(
            self.config.swell_period_min,
            self.config.swell_period_max
        )
        self.state.swell_amplitude = random.uniform(
            self.config.swell_amplitude_min,
            self.config.swell_amplitude_max
        )
        
        # Random initial phases
        self.state.swell_phase = random.uniform(0, 2 * math.pi)
        self.state.secondary_phase = random.uniform(0, 2 * math.pi)
        self.state.chop_phase = random.uniform(0, 2 * math.pi)
        
        # Chop period
        self.state.chop_period = random.uniform(
            self.config.chop_period_min,
            self.config.chop_period_max
        )
        
    def step(self, dt: float, tws: float = 15.0, heading: float = 0.0,
             wave_direction: Optional[float] = None,
             twa: float = 90.0, heel: float = 0.0) -> Tuple[float, float, float]:
        """
        Advance wave state by one timestep.
        
        Args:
            dt: Time step in seconds
            tws: True wind speed (knots) - affects chop amplitude
            heading: Boat heading (degrees) - affects wave impact angle
            wave_direction: Direction waves coming from (degrees), defaults to downwind
            twa: True wind angle (degrees) - affects yaw behavior
            heel: Current heel angle (degrees) - affects yaw magnitude
            
        Returns:
            Tuple of (wave_roll, wave_pitch, wave_yaw) in degrees/deg/s
        """
        # Default wave direction from downwind
        if wave_direction is None:
            wave_direction = 0.0  # Will be combined with swell
            
        self._update_phases(dt)
        self._compute_motion(tws, heading, wave_direction)
        self._compute_wave_yaw(twa, heel)
        
        return (self.state.wave_roll, self.state.wave_pitch, self.state.wave_yaw)
        
    def _update_phases(self, dt: float):
        """Update all wave phases."""
        # Primary swell
        omega_swell = 2 * math.pi / self.state.swell_period
        self.state.swell_phase += omega_swell * dt
        self.state.swell_phase %= (2 * math.pi)
        
        # Secondary swell
        omega_secondary = omega_swell / self.config.secondary_period_factor
        self.state.secondary_phase += omega_secondary * dt
        self.state.secondary_phase %= (2 * math.pi)
        
        # Chop
        omega_chop = 2 * math.pi / self.state.chop_period
        self.state.chop_phase += omega_chop * dt
        self.state.chop_phase %= (2 * math.pi)
        
    def _compute_motion(self, tws: float, heading: float, wave_direction: float):
        """Compute total wave-induced motion."""
        # Adjust amplitude for wind speed
        wind_amplitude = self.state.swell_amplitude + tws * self.config.amplitude_per_knot
        
        # Primary swell contribution
        if self.config.swell_enabled:
            swell_roll = wind_amplitude * math.sin(self.state.swell_phase)
            swell_pitch = wind_amplitude * 0.5 * math.sin(
                self.state.swell_phase + self.config.pitch_roll_phase
            )
        else:
            swell_roll = 0.0
            swell_pitch = 0.0
            
        # Secondary swell contribution
        if self.config.secondary_swell_enabled:
            secondary_amp = wind_amplitude * self.config.secondary_amplitude_factor
            secondary_roll = secondary_amp * math.sin(
                self.state.secondary_phase + self.config.secondary_phase_offset
            )
            secondary_pitch = secondary_amp * 0.4 * math.sin(
                self.state.secondary_phase + self.config.secondary_phase_offset + 
                self.config.pitch_roll_phase
            )
        else:
            secondary_roll = 0.0
            secondary_pitch = 0.0
            
        # Chop contribution (correlated with wind)
        if self.config.chop_enabled and tws > 5:
            chop_amp = wind_amplitude * self.config.chop_amplitude_factor
            chop_amp *= min(1.0, tws / 15.0)  # Scale with wind
            
            # Add some randomness to chop
            chop_roll = chop_amp * math.sin(self.state.chop_phase)
            chop_roll += random.gauss(0, chop_amp * 0.3)
            
            chop_pitch = chop_amp * 0.3 * math.sin(
                self.state.chop_phase + self.config.pitch_roll_phase
            )
            chop_pitch += random.gauss(0, chop_amp * 0.2)
        else:
            chop_roll = 0.0
            chop_pitch = 0.0
            
        # Combine all components
        self.state.wave_roll = swell_roll + secondary_roll + chop_roll
        self.state.wave_pitch = swell_pitch + secondary_pitch + chop_pitch
        
        # Compute approximate heave acceleration (for IMU simulation)
        # Heave is related to vertical component of wave motion
        self.state.wave_heave = (
            wind_amplitude * 0.1 * 
            math.sin(self.state.swell_phase + math.pi / 4)
        )
        
    def _compute_wave_yaw(self, twa: float, heel: float):
        """
        Compute wave-induced yaw rate based on point of sail and heel angle.
        
        Physics:
        - Downwind (TWA > 120°): Stern lift causes boat to round up toward wind.
          More heel amplifies this effect. Requires counter-rudder.
        - Upwind (TWA < 60°): Waves knock boat off wind (away from wind).
          Weather helm from sails partially counteracts. Effect varies with wave phase.
        - Beam reach: Minimal heading effect from waves.
        
        Args:
            twa: True wind angle (degrees, -180 to 180)
            heel: Current heel angle (degrees, negative = heel to port)
        """
        if not self.config.yaw_effect_enabled:
            self.state.wave_yaw = 0.0
            return
            
        abs_twa = abs(twa)
        abs_heel = abs(heel)
        
        # Estimate swell height from amplitude (rough approximation)
        # Amplitude in degrees relates to swell height - typically 2-4° per meter
        swell_height_m = self.state.swell_amplitude / 3.0
        
        # Wave phase factor: positive when stern lifting (phase near π)
        # sin(phase) oscillates between -1 and 1
        phase_factor = math.sin(self.state.swell_phase)
        
        # Determine yaw effect based on point of sail
        if abs_twa > 120:
            # Downwind (running): stern lift causes rounding up toward wind
            # Positive yaw rate means heading increases (turning to starboard)
            # When on port tack (twa > 0), rounding up = heading toward wind = positive
            # When on starboard tack (twa < 0), rounding up = heading toward wind = negative
            base_yaw = self.config.downwind_yaw_factor * abs_heel * swell_height_m
            
            # Sign: positive phase_factor means stern lifting
            # Stern lift causes rounding up toward wind
            # If TWA > 0 (wind from starboard), round up = turn to starboard = positive heading change
            # If TWA < 0 (wind from port), round up = turn to port = negative heading change
            if twa > 0:
                self.state.wave_yaw = base_yaw * phase_factor
            else:
                self.state.wave_yaw = -base_yaw * phase_factor
                
        elif abs_twa < 60:
            # Upwind (beating): waves knock boat away from wind
            # Falling off = heading moving away from wind direction
            # If TWA > 0 (wind from starboard), fall off = turn to port = negative heading change
            # If TWA < 0 (wind from port), fall off = turn to starboard = positive heading change
            base_yaw = self.config.upwind_yaw_factor * abs_heel * swell_height_m
            
            # Waves knock bow off when climbing wave (bow lifting)
            # Bow lifts when phase_factor is negative (opposite of stern lift)
            if twa > 0:
                self.state.wave_yaw = -base_yaw * (-phase_factor)  # Fall off to port
            else:
                self.state.wave_yaw = base_yaw * (-phase_factor)   # Fall off to starboard
                
        else:
            # Beam reach: transition zone, reduced effect
            # Linear interpolation between upwind and downwind effects
            if abs_twa <= 90:
                # Closer to upwind - reduce upwind effect
                blend = (abs_twa - 60) / 30.0  # 0 at 60°, 1 at 90°
                base_yaw = self.config.upwind_yaw_factor * abs_heel * swell_height_m
                base_yaw *= (1 - blend)  # Fade out toward beam reach
                if twa > 0:
                    self.state.wave_yaw = -base_yaw * (-phase_factor)
                else:
                    self.state.wave_yaw = base_yaw * (-phase_factor)
            else:
                # Closer to downwind - reduce downwind effect
                blend = (120 - abs_twa) / 30.0  # 0 at 120°, 1 at 90°
                base_yaw = self.config.downwind_yaw_factor * abs_heel * swell_height_m
                base_yaw *= (1 - blend)  # Fade out toward beam reach
                if twa > 0:
                    self.state.wave_yaw = base_yaw * phase_factor
                else:
                    self.state.wave_yaw = -base_yaw * phase_factor
        
    def get_motion(self) -> Tuple[float, float, float, float]:
        """Get current wave motion as (roll, pitch, heave, yaw) tuple."""
        return (self.state.wave_roll, self.state.wave_pitch, self.state.wave_heave, self.state.wave_yaw)
        
    def set_sea_state(self, swell_period: float, swell_amplitude: float):
        """Set sea state directly."""
        self.state.swell_period = swell_period
        self.state.swell_amplitude = swell_amplitude
        
    def reset(self):
        """Reset wave model to random initial conditions."""
        self._initialize()
        
    @classmethod
    def calm(cls) -> 'WaveModel':
        """Create a calm sea state model."""
        config = WaveConfig(
            swell_amplitude_min=0.5,
            swell_amplitude_max=2.0,
            chop_enabled=False,
            secondary_swell_enabled=False,
        )
        return cls(config)
        
    @classmethod
    def moderate(cls) -> 'WaveModel':
        """Create a moderate sea state model."""
        config = WaveConfig(
            swell_amplitude_min=3.0,
            swell_amplitude_max=6.0,
        )
        return cls(config)
        
    @classmethod
    def rough(cls) -> 'WaveModel':
        """Create a rough sea state model."""
        config = WaveConfig(
            swell_amplitude_min=6.0,
            swell_amplitude_max=12.0,
            swell_period_min=8.0,
            swell_period_max=14.0,
            chop_amplitude_factor=0.5,
        )
        return cls(config)
