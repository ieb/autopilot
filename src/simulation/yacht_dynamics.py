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


# Reef reduction factors for sail area
REEF_FACTORS = {0: 1.0, 1: 0.75, 2: 0.55, 3: 0.35}


@dataclass
class SailConfig:
    """
    Sail configuration for heel calculations.
    
    Pogo 1250 default values:
    - Main: 55 m² (full)
    - Genoa: 52 m² (full)
    - Total upwind: ~107 m²
    """
    main_area: float = 55.0        # m² full mainsail
    jib_area: float = 52.0         # m² genoa/jib
    reef_level: int = 0            # 0=full, 1=1st reef, 2=2nd reef, 3=storm
    center_of_effort: float = 6.0  # m above center of lateral resistance
    
    @property
    def effective_area(self) -> float:
        """Get effective sail area accounting for reef level."""
        return (self.main_area + self.jib_area) * REEF_FACTORS.get(self.reef_level, 1.0)
    
    def reef_up(self) -> bool:
        """Increase reef level (reduce sail). Returns True if changed."""
        if self.reef_level < 3:
            self.reef_level += 1
            return True
        return False
    
    def reef_down(self) -> bool:
        """Decrease reef level (more sail). Returns True if changed."""
        if self.reef_level > 0:
            self.reef_level -= 1
            return True
        return False


@dataclass
class YachtConfig:
    """Configuration for yacht dynamics."""
    # Yacht characteristics (Pogo 1250 defaults)
    displacement_kg: float = 5500          # Total displacement in kg
    lwl_m: float = 11.5                    # Waterline length
    beam_m: float = 4.5                    # Beam
    max_speed_kts: float = 20.0
    
    # Stability parameters (physics-based heel model)
    metacentric_height: float = 0.72       # GM in meters – tuned to give ~30° heel upwind in 15kt
    max_heel: float = 45.0                 # Absolute maximum heel angle (degrees)
    max_heel_before_reef: float = 35.0     # Heel threshold to trigger reefing
    auto_reef: bool = True                 # Enable automatic reefing
    heel_damping: float = 0.85             # Heel response damping (0-1)

    # Rudder response
    rudder_effectiveness: float = 0.5      # deg/s heading rate per deg rudder
    heading_damping: float = 0.8           # First-order response damping
    max_rudder_angle: float = 25.0         # Maximum rudder deflection
    
    # Weather helm
    weather_helm_factor: float = 0.05      # deg/s yaw rate per deg heel
    
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
    
    # Sail state
    reef_level: int = 0            # Current reef level (0=full, 1-3=reefed)


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
        self.sail_config = SailConfig()
        self._reef_cooldown = 0.0  # Seconds remaining before next reef change allowed

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
        self.config.metacentric_height = vary(self.config.metacentric_height)
        self.config.weather_helm_factor = vary(self.config.weather_helm_factor)
        
    def step(self, rudder_command: float, dt: float, wave_yaw: float = 0.0) -> YachtState:
        """
        Advance yacht state by one timestep.
        
        Args:
            rudder_command: Commanded rudder angle (degrees)
            dt: Time step (seconds)
            wave_yaw: Wave-induced yaw rate (deg/s) from wave model
            
        Returns:
            Updated yacht state
        """
        # Apply rudder with rate limiting
        self._apply_rudder(rudder_command, dt)
        
        # Compute forces and moments
        self._update_heading(dt, wave_yaw)
        self._update_heel(dt)
        self._check_reefing(dt)  # Auto-adjust sails based on heel
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
        
    def _update_heading(self, dt: float, wave_yaw: float = 0.0):
        """
        Update heading based on rudder, weather helm, and wave effects.
        
        Args:
            dt: Time step (seconds)
            wave_yaw: Wave-induced yaw rate (deg/s) - accounts for:
                - Downwind: stern lift causing boat to round up toward wind
                - Upwind: waves knocking boat off wind
        """
        # Target heading rate from rudder
        rudder_effect = self.config.rudder_effectiveness * self.state.rudder_angle
        
        # Speed factor - rudder is less effective at low speed
        speed_factor = min(1.0, self.state.stw / 3.0) if self.state.stw > 0 else 0.0
        rudder_effect *= speed_factor
        
        # Weather helm from heel – heeling to port (negative roll) on starboard
        # tack pushes the bow toward the wind (starboard = positive heading rate).
        # Negate so that negative roll → positive weather helm.
        weather_helm = -self.config.weather_helm_factor * self.state.roll
        
        # Target rate including wave-induced yaw
        target_rate = rudder_effect + weather_helm + wave_yaw
        
        # First-order response
        self.state.heading_rate = (
            self.config.heading_damping * self.state.heading_rate +
            (1 - self.config.heading_damping) * target_rate
        )
        
        # Update heading
        self.state.heading += self.state.heading_rate * dt
        self.state.heading %= 360
        
    def _update_heel(self, dt: float):
        """
        Update heel angle using physics-based heeling/righting moment balance.
        
        Heeling Moment: M_h = 0.5 * rho * A_sail * C_L * V^2 * h_CE * cos(theta)
        Righting Moment: M_r = displacement * g * GM * sin(theta)
        
        At equilibrium: M_h = M_r, solve for theta (heel angle)
        """
        # Physical constants
        RHO_AIR = 1.225  # kg/m³ air density
        G = 9.81  # m/s² gravity
        
        # Get effective sail area from sail configuration
        sail_area = self.sail_config.effective_area
        center_of_effort = self.sail_config.center_of_effort
        
        # Convert apparent wind speed to m/s
        aws_ms = self.state.aws * 0.5144  # knots to m/s
        awa_rad = math.radians(abs(self.state.awa))
        
        # Lift coefficient varies with AWA
        # Upwind: high side-force from sails generating lift → high heel.
        # Real sails produce high CL across AWA 25-80°, with a ramp up
        # from luffing (AWA<20) and a drop past beam reach.
        # Downwind: mostly drag, low side-force → low heel.
        # Tuned with GM=0.72 to give ~28° upwind / ~8° downwind at 15kt.
        awa_deg = math.degrees(awa_rad)
        if awa_deg < 20:
            # Close to head-to-wind: sails luffing, low force
            cl = 1.5 * (awa_deg / 20.0)
        elif awa_deg < 80:
            # Upwind plateau: high lift from close-hauled to near beam reach
            cl = 1.5
        elif awa_deg < 100:
            # Transition: lift drops past beam reach
            cl = 1.5 * (100.0 - awa_deg) / 20.0
        else:
            # Downwind: mostly drag, low side-force
            cl = 0.2 * math.sin(math.pi - awa_rad)
        
        # Heeling force from sails: F = 0.5 * rho * A * Cl * V^2
        heeling_force = 0.5 * RHO_AIR * sail_area * cl * aws_ms ** 2

        # Heeling moment = force * lever arm (center of effort height)
        heeling_moment = heeling_force * center_of_effort

        # Solve for equilibrium heel angle analytically.
        # At equilibrium: M_heel * cos(θ) = M_right * sin(θ)
        # where M_right = displacement * g * GM
        # This gives: tan(θ) = M_heel / M_right, so θ = atan(M_heel / M_right)
        righting_arm_factor = self.config.displacement_kg * G * self.config.metacentric_height

        if righting_arm_factor > 0:
            target_heel_rad = math.atan(heeling_moment / righting_arm_factor)
        else:
            target_heel_rad = 0.0
        
        # Convert to degrees and cap at max heel
        target_heel = math.degrees(target_heel_rad)
        target_heel = min(self.config.max_heel, target_heel)
        
        # Sign based on which tack (positive AWA = starboard tack = heel to port = negative roll)
        if self.state.awa > 0:
            target_heel = -target_heel
            
        # Damped response for smooth transitions
        old_roll = self.state.roll
        self.state.roll = (
            self.config.heel_damping * self.state.roll +
            (1 - self.config.heel_damping) * target_heel
        )
        
        # Compute roll rate
        self.state.roll_rate = (self.state.roll - old_roll) / dt if dt > 0 else 0.0
        
        # Sync reef level to state for logging
        self.state.reef_level = self.sail_config.reef_level
        
    def _check_reefing(self, dt: float):
        """
        Automatically adjust reef level based on heel angle.

        Reef up (reduce sail) when heel exceeds max_heel_before_reef.
        Shake out reef (more sail) when heel is well below threshold.
        A cooldown timer prevents chain-reefing during transient spikes.
        """
        if not self.config.auto_reef:
            return

        # Decrement cooldown
        if self._reef_cooldown > 0:
            self._reef_cooldown -= dt
            return

        current_heel = abs(self.state.roll)

        # Reef up if heel exceeds threshold
        if current_heel > self.config.max_heel_before_reef:
            if self.sail_config.reef_up():
                self._reef_cooldown = 5.0  # Wait 5s before next change
                logger.debug(
                    f"Reefing up to level {self.sail_config.reef_level} "
                    f"(heel {current_heel:.1f}° > {self.config.max_heel_before_reef}°)"
                )

        # Shake out reef if heel is well under threshold (65% of limit)
        elif current_heel < self.config.max_heel_before_reef * 0.65:
            if self.sail_config.reef_down():
                self._reef_cooldown = 10.0  # Longer cooldown for shake-out
                logger.debug(
                    f"Shaking out to reef level {self.sail_config.reef_level} "
                    f"(heel {current_heel:.1f}°)"
                )
        
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
        self.sail_config = SailConfig()  # Reset sail configuration
        self._reef_cooldown = 0.0
        self.state.heading = heading % 360
        self.state.cog = heading % 360
        self.state.twd = twd % 360
        self.state.tws = tws
        self._update_apparent_wind()
        self._update_heel(0.1)
        self._update_speed(0.1)
