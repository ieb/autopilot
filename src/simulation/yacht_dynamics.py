"""
Yacht Dynamics Model
====================

Physics-based model for yacht response to rudder and wind.
Computes heading rate, heel angle, speed, and leeway.
"""

import math
from dataclasses import dataclass, field
from typing import Optional
import logging

from ..ml.polar import Polar

logger = logging.getLogger(__name__)


@dataclass
class YachtConfig:
    """Configuration for yacht dynamics."""
    # Yacht characteristics (Pogo 1250 defaults)
    displacement_kg: float = 7500
    lwl_m: float = 11.5
    beam_m: float = 4.0
    max_speed_kts: float = 20.0
    
    # Rudder response
    rudder_effectiveness: float = 2.0      # deg/s heading rate per deg rudder
    heading_damping: float = 0.8           # First-order response damping
    max_rudder_angle: float = 25.0         # Maximum rudder deflection
    
    # Heel characteristics
    max_heel: float = 30.0                 # Maximum heel angle (degrees)
    heel_stiffness: float = 0.02           # Heel per unit wind pressure
    heel_damping: float = 0.9              # Heel response damping
    
    # Weather helm
    weather_helm_factor: float = 0.1       # deg/s yaw rate per deg heel
    
    # Leeway
    leeway_coefficient: float = 0.15       # Leeway degrees per deg heel
    
    # Speed response
    speed_damping: float = 0.95            # Speed change damping


@dataclass
class YachtState:
    """Current yacht state."""
    # Position
    latitude: float = 60.0
    longitude: float = 22.0
    
    # Heading and rates
    heading: float = 0.0           # Magnetic heading (degrees)
    heading_rate: float = 0.0      # Yaw rate (deg/s)
    
    # Attitude
    roll: float = 0.0              # Heel angle (degrees, negative = port)
    pitch: float = 0.0             # Pitch angle (degrees)
    roll_rate: float = 0.0         # Roll rate (deg/s)
    
    # Rudder
    rudder_angle: float = 0.0      # Current rudder position (degrees)
    
    # Speed
    stw: float = 6.0               # Speed through water (knots)
    sog: float = 6.0               # Speed over ground (knots)
    cog: float = 0.0               # Course over ground (degrees)
    leeway: float = 0.0            # Leeway angle (degrees)
    
    # Wind (set by wind model)
    twd: float = 180.0             # True wind direction (degrees)
    tws: float = 15.0              # True wind speed (knots)
    awa: float = 45.0              # Apparent wind angle (degrees)
    aws: float = 18.0              # Apparent wind speed (knots)


class YachtDynamics:
    """
    Physics-based yacht dynamics model.
    
    Simulates yacht response to rudder input and wind conditions.
    Uses polar data for speed calculations.
    """
    
    def __init__(self, config: Optional[YachtConfig] = None,
                 polar: Optional[Polar] = None,
                 variation: float = 0.0):
        """
        Initialize yacht dynamics.
        
        Args:
            config: Yacht configuration
            polar: Polar diagram for speed lookup
            variation: Random variation factor (0.0-1.0) for domain randomization
        """
        self.config = config or YachtConfig()
        self.polar = polar or Polar.pogo_1250()
        self.state = YachtState()
        
        # Apply domain randomization
        if variation > 0:
            self._apply_variation(variation)
            
    def _apply_variation(self, variation: float):
        """Apply random variation to yacht parameters."""
        import random
        
        def vary(value: float) -> float:
            return value * (1.0 + random.uniform(-variation, variation))
        
        self.config.rudder_effectiveness = vary(self.config.rudder_effectiveness)
        self.config.heading_damping = min(0.95, max(0.5, vary(self.config.heading_damping)))
        self.config.heel_stiffness = vary(self.config.heel_stiffness)
        self.config.weather_helm_factor = vary(self.config.weather_helm_factor)
        
    def step(self, rudder_command: float, dt: float) -> YachtState:
        """
        Advance yacht state by one timestep.
        
        Args:
            rudder_command: Commanded rudder angle (degrees)
            dt: Time step (seconds)
            
        Returns:
            Updated yacht state
        """
        # Apply rudder with rate limiting
        self._apply_rudder(rudder_command, dt)
        
        # Compute forces and moments
        self._update_heading(dt)
        self._update_heel(dt)
        self._update_speed(dt)
        self._update_apparent_wind()
        self._update_leeway()
        self._update_position(dt)
        
        return self.state
        
    def _apply_rudder(self, command: float, dt: float):
        """Apply rudder command with rate limiting."""
        max_rate = 4.0  # deg/s (Jefa LD100 spec)
        
        # Clamp command to limits
        command = max(-self.config.max_rudder_angle,
                     min(self.config.max_rudder_angle, command))
        
        # Rate limit
        diff = command - self.state.rudder_angle
        max_change = max_rate * dt
        diff = max(-max_change, min(max_change, diff))
        
        self.state.rudder_angle += diff
        
    def _update_heading(self, dt: float):
        """Update heading based on rudder and other forces."""
        # Target heading rate from rudder
        rudder_effect = self.config.rudder_effectiveness * self.state.rudder_angle
        
        # Speed factor - rudder is less effective at low speed
        speed_factor = min(1.0, self.state.stw / 3.0) if self.state.stw > 0 else 0.0
        rudder_effect *= speed_factor
        
        # Weather helm from heel
        weather_helm = self.config.weather_helm_factor * self.state.roll
        
        # Target rate
        target_rate = rudder_effect + weather_helm
        
        # First-order response
        self.state.heading_rate = (
            self.config.heading_damping * self.state.heading_rate +
            (1 - self.config.heading_damping) * target_rate
        )
        
        # Update heading
        self.state.heading += self.state.heading_rate * dt
        self.state.heading %= 360
        
    def _update_heel(self, dt: float):
        """Update heel angle based on wind pressure."""
        # Compute true wind angle
        twa = self._compute_twa()
        
        # Wind heeling moment proportional to AWS^2 * sin(AWA)
        awa_rad = math.radians(self.state.awa)
        wind_pressure = self.state.aws ** 2 * abs(math.sin(awa_rad))
        
        # Target heel
        target_heel = min(self.config.max_heel, 
                         wind_pressure * self.config.heel_stiffness)
        
        # Sign based on which tack (positive AWA = starboard tack = heel to port = negative roll)
        if self.state.awa > 0:
            target_heel = -target_heel
            
        # Damped response
        old_roll = self.state.roll
        self.state.roll = (
            self.config.heel_damping * self.state.roll +
            (1 - self.config.heel_damping) * target_heel
        )
        
        # Compute roll rate
        self.state.roll_rate = (self.state.roll - old_roll) / dt if dt > 0 else 0.0
        
    def _update_speed(self, dt: float):
        """Update boat speed from polar."""
        twa = abs(self._compute_twa())
        
        # Target speed from polar
        target_stw = self.polar.get_target_speed(twa, self.state.tws)
        
        # Reduce speed for high heel angles
        heel_factor = math.cos(math.radians(abs(self.state.roll) * 0.5))
        target_stw *= heel_factor
        
        # Reduce speed during maneuvers (high rudder angle)
        rudder_drag = 1.0 - 0.01 * abs(self.state.rudder_angle)
        target_stw *= rudder_drag
        
        # Damped response
        self.state.stw = (
            self.config.speed_damping * self.state.stw +
            (1 - self.config.speed_damping) * target_stw
        )
        
        # SOG includes current effect (simplified)
        self.state.sog = self.state.stw * 0.98  # Slight loss
        
    def _update_apparent_wind(self):
        """Compute apparent wind from true wind and boat motion."""
        twa = self._compute_twa()
        twa_rad = math.radians(twa)
        
        # True wind components relative to boat
        tw_x = self.state.tws * math.cos(twa_rad)
        tw_y = self.state.tws * math.sin(twa_rad)
        
        # Subtract boat velocity (boat moves into the wind)
        aw_x = tw_x - self.state.stw
        aw_y = tw_y
        
        # Apparent wind
        self.state.aws = math.sqrt(aw_x**2 + aw_y**2)
        self.state.awa = math.degrees(math.atan2(aw_y, aw_x))
        
    def _update_leeway(self):
        """Compute leeway angle."""
        # Leeway proportional to heel and inversely proportional to speed
        if self.state.stw > 0.5:
            self.state.leeway = (
                self.config.leeway_coefficient * abs(self.state.roll) / 
                math.sqrt(self.state.stw)
            )
        else:
            self.state.leeway = 0.0
            
        # Sign: leeway is always to leeward
        if self.state.roll < 0:  # Heeling to port
            self.state.leeway = -self.state.leeway
            
        # COG differs from heading by leeway
        self.state.cog = (self.state.heading + self.state.leeway) % 360
        
    def _update_position(self, dt: float):
        """Update position based on COG and SOG."""
        if self.state.sog < 0.1:
            return
            
        # Convert SOG from knots to degrees/second
        # 1 knot = 1 nm/hour, 1 nm ≈ 1' of latitude = 1/60 degree
        # So: 1 knot = (1/60 degree)/hour = (1/60)/3600 degree/second = 1/216000 deg/s
        speed_deg_per_sec = self.state.sog / 216000.0
        
        # Distance traveled
        distance_deg = speed_deg_per_sec * dt
        
        # Update position
        cog_rad = math.radians(self.state.cog)
        self.state.latitude += distance_deg * math.cos(cog_rad)
        self.state.longitude += distance_deg * math.sin(cog_rad) / math.cos(
            math.radians(self.state.latitude)
        )
        
    def _compute_twa(self) -> float:
        """Compute true wind angle relative to heading."""
        twa = self.state.twd - self.state.heading
        while twa > 180:
            twa -= 360
        while twa < -180:
            twa += 360
        return twa
        
    def set_wind(self, twd: float, tws: float):
        """Set true wind conditions."""
        self.state.twd = twd % 360
        self.state.tws = max(0, tws)
        self._update_apparent_wind()
        
    def set_heading(self, heading: float):
        """Set yacht heading directly."""
        self.state.heading = heading % 360
        self._update_apparent_wind()
        
    def reset(self, heading: float = 0.0, twd: float = 180.0, tws: float = 15.0):
        """Reset yacht state to initial conditions."""
        self.state = YachtState()
        self.state.heading = heading % 360
        self.state.cog = heading % 360
        self.state.twd = twd % 360
        self.state.tws = tws
        self._update_apparent_wind()
        self._update_heel(0.1)
        self._update_speed(0.1)
