"""
Navigator
=========

Manages route following, waypoint tracking, and steering mode selection.
Determines the appropriate steering mode based on wind angle.
"""

import math
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from typing import List, Optional, Tuple
import logging

from .route_parser import Waypoint

logger = logging.getLogger(__name__)


# Earth radius in nautical miles
EARTH_RADIUS_NM = 3440.065


class SteeringMode(Enum):
    """Steering mode for the autopilot."""
    COMPASS = "compass"       # Hold magnetic heading
    WIND_AWA = "wind_awa"     # Hold apparent wind angle
    WIND_TWA = "wind_twa"     # Hold true wind angle
    MOTORING = "motoring"     # Motor mode - hold COG
    

@dataclass
class NavigationState:
    """Current navigation state."""
    # Current leg
    current_leg_index: int = 0
    from_waypoint: Optional[Waypoint] = None
    to_waypoint: Optional[Waypoint] = None
    
    # Target values
    target_heading: float = 0.0       # For COMPASS/MOTORING mode
    target_awa: float = 0.0           # For WIND_AWA mode
    target_twa: float = 0.0           # For WIND_TWA mode
    
    # Mode
    steering_mode: SteeringMode = SteeringMode.COMPASS
    is_motoring: bool = False
    
    # Route progress
    bearing_to_waypoint: float = 0.0  # Bearing to next waypoint
    distance_to_waypoint: float = 0.0 # Distance to next waypoint (nm)
    cross_track_error: float = 0.0    # XTE (nm, positive = right of track)
    
    # Status
    arrived_at_destination: bool = False
    leg_complete: bool = False


class Navigator:
    """
    Manages route following and steering mode selection.
    
    Features:
    - Tracks current position along route
    - Computes bearing and distance to next waypoint
    - Determines steering mode based on wind angle
    - Calculates cross-track error
    """
    
    # Steering mode thresholds (conventional sailing logic)
    AWA_MODE_MIN = 30   # Below this, too close to wind
    AWA_MODE_MAX = 120  # Above this, use TWA
    MOTORING_STW = 6.0  # Speed when motoring (knots)
    
    # Waypoint arrival threshold
    ARRIVAL_THRESHOLD_NM = 0.1  # 0.1 nm = ~185 meters
    
    def __init__(self, waypoints: List[Waypoint]):
        """
        Initialize navigator with route waypoints.
        
        Args:
            waypoints: List of Waypoint objects defining the route
        """
        if not waypoints:
            raise ValueError("Route must have at least one waypoint")
            
        self.waypoints = waypoints
        self.state = NavigationState()
        
        # Initialize to first leg
        self._start_leg(0)
        
    def _start_leg(self, leg_index: int):
        """Start a new leg of the route."""
        if leg_index >= len(self.waypoints):
            self.state.arrived_at_destination = True
            return
            
        self.state.current_leg_index = leg_index
        self.state.from_waypoint = self.waypoints[leg_index]
        
        if leg_index < len(self.waypoints) - 1:
            self.state.to_waypoint = self.waypoints[leg_index + 1]
        else:
            # Last waypoint - use destination
            self.state.to_waypoint = self.waypoints[leg_index]
            
        self.state.leg_complete = False
        self.state.is_motoring = self.state.from_waypoint.is_motoring
        
        # Set initial target based on waypoint data
        if self.state.is_motoring:
            self.state.steering_mode = SteeringMode.MOTORING
            self.state.target_heading = self.state.from_waypoint.cog
        else:
            # Will be updated in update() based on wind
            self.state.steering_mode = SteeringMode.COMPASS
            self.state.target_heading = self.state.from_waypoint.cog
            
        logger.debug(f"Started leg {leg_index}: {self.state.from_waypoint.cog}° -> "
                    f"{self.state.to_waypoint.to_lat:.4f}, {self.state.to_waypoint.to_lon:.4f}")
        
    def update(self, lat: float, lon: float, twd: float, 
               heading: float, stw: float) -> NavigationState:
        """
        Update navigation state based on current position and conditions.
        
        Args:
            lat: Current latitude
            lon: Current longitude
            twd: True wind direction (degrees)
            heading: Current heading (degrees)
            stw: Speed through water (knots)
            
        Returns:
            Updated NavigationState
        """
        if self.state.arrived_at_destination:
            return self.state
            
        # Compute bearing and distance to waypoint
        self._update_bearing_distance(lat, lon)
        
        # Check if we've arrived at the waypoint
        if self.state.distance_to_waypoint < self.ARRIVAL_THRESHOLD_NM:
            self._advance_to_next_leg()
            if not self.state.arrived_at_destination:
                self._update_bearing_distance(lat, lon)
                
        # Compute cross-track error
        self._update_cross_track_error(lat, lon)
        
        # Determine steering mode and target
        if self.state.is_motoring:
            self._set_motoring_mode()
        else:
            self._determine_steering_mode(twd, heading)
            
        return self.state
        
    def _update_bearing_distance(self, lat: float, lon: float):
        """Update bearing and distance to next waypoint."""
        if self.state.to_waypoint is None:
            return
            
        # Target is the 'to' position of current waypoint
        target_lat = self.state.to_waypoint.to_lat
        target_lon = self.state.to_waypoint.to_lon
        
        self.state.bearing_to_waypoint = self._compute_bearing(
            lat, lon, target_lat, target_lon
        )
        self.state.distance_to_waypoint = self._compute_distance(
            lat, lon, target_lat, target_lon
        )
        
    def _update_cross_track_error(self, lat: float, lon: float):
        """
        Compute cross-track error from the planned track line.
        
        Positive XTE means boat is to the right of track.
        """
        if self.state.from_waypoint is None or self.state.to_waypoint is None:
            return
            
        # Track line: from -> to
        from_lat = self.state.from_waypoint.from_lat
        from_lon = self.state.from_waypoint.from_lon
        to_lat = self.state.to_waypoint.to_lat
        to_lon = self.state.to_waypoint.to_lon
        
        # Use cross-track distance formula
        self.state.cross_track_error = self._compute_cross_track(
            from_lat, from_lon, to_lat, to_lon, lat, lon
        )
        
    def _advance_to_next_leg(self):
        """Advance to the next leg of the route."""
        next_leg = self.state.current_leg_index + 1
        
        if next_leg >= len(self.waypoints):
            self.state.arrived_at_destination = True
            logger.info("Arrived at destination!")
        else:
            self.state.leg_complete = True
            self._start_leg(next_leg)
            logger.debug(f"Advanced to leg {next_leg}")
            
    def _determine_steering_mode(self, twd: float, heading: float):
        """
        Determine the appropriate steering mode based on wind angle.
        
        Conventional logic:
        - AWA mode for upwind/reaching (30-120 degrees)
        - TWA mode for running/broad reaching (>120 degrees)
        """
        # Compute true wind angle
        twa = self._compute_twa(twd, heading)
        abs_twa = abs(twa)
        
        # Target bearing (what we want to sail)
        target_cog = self.state.bearing_to_waypoint
        
        # Compute what wind angle we would have on the target course
        target_twa = self._compute_twa(twd, target_cog)
        abs_target_twa = abs(target_twa)
        
        if abs_target_twa < self.AWA_MODE_MIN:
            # Too close to wind - need to tack
            # For now, use AWA mode at the minimum angle
            self.state.steering_mode = SteeringMode.WIND_AWA
            # Maintain the current tack (sign of TWA)
            if twa >= 0:
                self.state.target_awa = self._twa_to_awa(self.AWA_MODE_MIN, twd, heading)
            else:
                self.state.target_awa = self._twa_to_awa(-self.AWA_MODE_MIN, twd, heading)
            self.state.target_twa = self.AWA_MODE_MIN if twa >= 0 else -self.AWA_MODE_MIN
                
        elif abs_target_twa <= self.AWA_MODE_MAX:
            # Upwind or reaching - use AWA mode
            self.state.steering_mode = SteeringMode.WIND_AWA
            # Target AWA that gives us the right heading
            self.state.target_awa = self._heading_to_target_awa(target_cog, twd)
            self.state.target_twa = target_twa
            
        else:
            # Running - use TWA mode
            self.state.steering_mode = SteeringMode.WIND_TWA
            self.state.target_twa = target_twa
            self.state.target_awa = self._twa_to_awa(target_twa, twd, heading)
            
        self.state.target_heading = target_cog
        
    def _set_motoring_mode(self):
        """Set up motoring mode."""
        self.state.steering_mode = SteeringMode.MOTORING
        self.state.target_heading = self.state.bearing_to_waypoint
        
    def _compute_twa(self, twd: float, heading: float) -> float:
        """Compute true wind angle from TWD and heading."""
        twa = twd - heading
        while twa > 180:
            twa -= 360
        while twa < -180:
            twa += 360
        return twa
        
    def _twa_to_awa(self, twa: float, twd: float, heading: float) -> float:
        """
        Convert TWA to approximate AWA.
        
        This is a simplification - actual AWA depends on boat speed.
        """
        # For rough estimation, AWA ≈ TWA * 0.8-0.9 depending on conditions
        # More accurate would use the wind triangle
        return twa * 0.85
        
    def _heading_to_target_awa(self, target_heading: float, twd: float) -> float:
        """Compute the AWA needed to achieve a target heading."""
        target_twa = self._compute_twa(twd, target_heading)
        # Approximate AWA from TWA
        return target_twa * 0.85
        
    def _compute_bearing(self, lat1: float, lon1: float, 
                        lat2: float, lon2: float) -> float:
        """
        Compute initial bearing from point 1 to point 2.
        
        Returns bearing in degrees (0-360).
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlon_rad = math.radians(lon2 - lon1)
        
        x = math.sin(dlon_rad) * math.cos(lat2_rad)
        y = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad))
        
        bearing = math.degrees(math.atan2(x, y))
        return bearing % 360
        
    def _compute_distance(self, lat1: float, lon1: float,
                         lat2: float, lon2: float) -> float:
        """
        Compute great circle distance in nautical miles.
        
        Uses Haversine formula.
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlat_rad = math.radians(lat2 - lat1)
        dlon_rad = math.radians(lon2 - lon1)
        
        a = (math.sin(dlat_rad / 2) ** 2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) * 
             math.sin(dlon_rad / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return EARTH_RADIUS_NM * c
        
    def _compute_cross_track(self, lat1: float, lon1: float,
                            lat2: float, lon2: float,
                            lat3: float, lon3: float) -> float:
        """
        Compute cross-track distance from point 3 to line 1-2.
        
        Returns distance in nautical miles.
        Positive means point 3 is to the right of line 1->2.
        """
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        lat3_rad = math.radians(lat3)
        lon3_rad = math.radians(lon3)
        
        # Angular distance from point 1 to point 3
        d13 = self._compute_distance(lat1, lon1, lat3, lon3) / EARTH_RADIUS_NM
        
        # Bearing from point 1 to point 3
        bearing_13 = math.radians(self._compute_bearing(lat1, lon1, lat3, lon3))
        
        # Bearing from point 1 to point 2 (track line)
        bearing_12 = math.radians(self._compute_bearing(lat1, lon1, lat2, lon2))
        
        # Cross-track distance (spherical)
        xte = math.asin(math.sin(d13) * math.sin(bearing_13 - bearing_12))
        
        return xte * EARTH_RADIUS_NM
        
    def get_current_waypoint(self) -> Optional[Waypoint]:
        """Get the current 'from' waypoint."""
        return self.state.from_waypoint
        
    def get_target_waypoint(self) -> Optional[Waypoint]:
        """Get the current target 'to' waypoint."""
        return self.state.to_waypoint
        
    def get_leg_progress(self) -> float:
        """
        Get progress along the current leg (0-1).
        
        Based on distance remaining vs total leg distance.
        """
        if self.state.from_waypoint is None:
            return 0.0
            
        total_distance = self.state.from_waypoint.distance
        if total_distance <= 0:
            return 1.0
            
        remaining = self.state.distance_to_waypoint
        return max(0.0, min(1.0, 1.0 - remaining / total_distance))
        
    def get_route_progress(self) -> float:
        """
        Get overall route progress (0-1).
        
        Based on completed legs and current leg progress.
        """
        if not self.waypoints:
            return 0.0
            
        total_distance = self.waypoints[0].distance_to_finish
        if total_distance <= 0:
            return 1.0
            
        if self.state.arrived_at_destination:
            return 1.0
            
        if self.state.from_waypoint:
            remaining = self.state.from_waypoint.distance_to_finish
            # Subtract distance already covered on current leg
            remaining -= (self.state.from_waypoint.distance - 
                         self.state.distance_to_waypoint)
            return max(0.0, min(1.0, 1.0 - remaining / total_distance))
            
        return 0.0
        
    @property
    def total_waypoints(self) -> int:
        """Total number of waypoints."""
        return len(self.waypoints)
        
    @property
    def current_leg(self) -> int:
        """Current leg index (0-based)."""
        return self.state.current_leg_index
