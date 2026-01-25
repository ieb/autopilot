"""
Wave Provider
=============

Provides wave data at any position and time by interpolating
from GRIB files or using default/estimated values.
"""

import math
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Tuple
import logging

from .grib_reader import GribReader
from .route_parser import Waypoint

logger = logging.getLogger(__name__)


@dataclass
class WaveConditions:
    """Wave conditions at a specific point."""
    significant_height: float  # Hs in meters
    mean_period: float         # Tp in seconds
    direction: float           # Wave direction (degrees, from)
    source: str                # 'grib', 'estimated', or 'default'
    
    @property
    def amplitude_degrees(self) -> float:
        """
        Estimate wave-induced roll amplitude in degrees.
        
        Rough approximation based on significant wave height.
        Actual roll depends heavily on vessel characteristics.
        """
        # Rough formula: roll amplitude ~ 3-5 degrees per meter of Hs
        return self.significant_height * 4.0


class WaveProvider:
    """
    Provides wave data for simulation.
    
    Priority:
    1. GRIB data (if available)
    2. Estimated from wind speed (fetch-limited wind waves)
    3. Default calm conditions
    """
    
    def __init__(self,
                 grib_reader: Optional[GribReader] = None,
                 waypoints: Optional[List[Waypoint]] = None):
        """
        Initialize wave provider.
        
        Args:
            grib_reader: GribReader with loaded wave data
            waypoints: List of waypoints (for wind-based estimation)
        """
        self.grib_reader = grib_reader
        self.waypoints = waypoints or []
        
        # Cache
        self._last_query: Optional[Tuple[float, float, datetime]] = None
        self._last_result: Optional[WaveConditions] = None
        
        # Defaults
        self.default_hs = 1.0   # meters
        self.default_tp = 6.0  # seconds
        self.default_dir = 0.0 # degrees
        
    def get_waves(self, lat: float, lon: float, time: datetime,
                  tws: Optional[float] = None,
                  twd: Optional[float] = None) -> WaveConditions:
        """
        Get wave conditions at a position and time.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            time: Query time
            tws: True wind speed (optional, for estimation)
            twd: True wind direction (optional, for estimation)
            
        Returns:
            WaveConditions with Hs, Tp, direction, and source
        """
        # Check cache
        if self._last_query is not None:
            last_lat, last_lon, last_time = self._last_query
            if (abs(lat - last_lat) < 0.001 and
                abs(lon - last_lon) < 0.001 and
                abs((time - last_time).total_seconds()) < 1):
                return self._last_result
                
        # Try GRIB data first
        if self.grib_reader and self.grib_reader.get_wave_times():
            try:
                hs, tp, direction = self.grib_reader.get_waves_at(lat, lon, time)
                if hs > 0:  # Valid data
                    result = WaveConditions(
                        significant_height=hs,
                        mean_period=tp,
                        direction=direction,
                        source='grib'
                    )
                    self._cache_result(lat, lon, time, result)
                    return result
            except Exception as e:
                logger.debug(f"GRIB wave lookup failed: {e}")
                
        # Estimate from wind if available
        if tws is not None and twd is not None:
            result = self._estimate_from_wind(tws, twd)
            self._cache_result(lat, lon, time, result)
            return result
            
        # Try to get wind from waypoints for estimation
        if self.waypoints:
            wp_wind = self._get_wind_from_waypoints(time)
            if wp_wind:
                result = self._estimate_from_wind(wp_wind[0], wp_wind[1])
                self._cache_result(lat, lon, time, result)
                return result
                
        # Default values
        result = WaveConditions(
            significant_height=self.default_hs,
            mean_period=self.default_tp,
            direction=self.default_dir,
            source='default'
        )
        self._cache_result(lat, lon, time, result)
        return result
        
    def _estimate_from_wind(self, tws: float, twd: float) -> WaveConditions:
        """
        Estimate wave parameters from wind using simplified SMB method.
        
        This is a rough approximation for fetch-limited wind waves.
        For fully-developed seas, waves would be larger.
        
        Args:
            tws: True wind speed in knots
            twd: True wind direction in degrees
            
        Returns:
            Estimated wave conditions
        """
        # Convert wind speed to m/s
        wind_mps = tws / 1.94384
        
        # Simplified wave height estimation
        # Hs ~ 0.02 * U^2 / g for developed seas (roughly)
        # For the English Channel with typical fetch, use a moderate factor
        g = 9.81
        if wind_mps > 0:
            # Approximate Hs in meters
            hs = 0.015 * (wind_mps ** 2) / g
            hs = max(0.2, min(hs, 8.0))  # Clamp to reasonable range
            
            # Approximate period from Hs (empirical relationship)
            # Tp ~ 3.5 * sqrt(Hs) for wind waves
            tp = 3.5 * math.sqrt(hs)
            tp = max(3.0, min(tp, 12.0))
        else:
            hs = 0.2
            tp = 4.0
            
        # Waves generally come from the same direction as wind
        # (simplified - in reality depends on fetch geometry)
        direction = twd
        
        return WaveConditions(
            significant_height=hs,
            mean_period=tp,
            direction=direction,
            source='estimated'
        )
        
    def _get_wind_from_waypoints(self, time: datetime) -> Optional[Tuple[float, float]]:
        """Get wind from waypoints by interpolating in time."""
        if not self.waypoints:
            return None
            
        for i, wp in enumerate(self.waypoints):
            if i < len(self.waypoints) - 1:
                next_wp = self.waypoints[i + 1]
                if wp.timestamp <= time <= next_wp.timestamp:
                    dt_total = (next_wp.timestamp - wp.timestamp).total_seconds()
                    dt_query = (time - wp.timestamp).total_seconds()
                    frac = dt_query / dt_total if dt_total > 0 else 0.0
                    
                    tws = wp.tws + (next_wp.tws - wp.tws) * frac
                    twd = wp.twd  # Simple, don't interpolate direction
                    return (tws, twd)
                    
        # Use closest waypoint
        if time < self.waypoints[0].timestamp:
            wp = self.waypoints[0]
        else:
            wp = self.waypoints[-1]
        return (wp.tws, wp.twd)
        
    def _cache_result(self, lat: float, lon: float, time: datetime,
                      result: WaveConditions):
        """Cache the last query result."""
        self._last_query = (lat, lon, time)
        self._last_result = result
        
    def get_sea_state_category(self, hs: float) -> str:
        """
        Get Douglas sea state category from significant height.
        
        Args:
            hs: Significant wave height in meters
            
        Returns:
            Sea state description
        """
        if hs < 0.1:
            return "Calm (glassy)"
        elif hs < 0.5:
            return "Calm (rippled)"
        elif hs < 1.25:
            return "Smooth"
        elif hs < 2.5:
            return "Slight"
        elif hs < 4.0:
            return "Moderate"
        elif hs < 6.0:
            return "Rough"
        elif hs < 9.0:
            return "Very rough"
        elif hs < 14.0:
            return "High"
        else:
            return "Phenomenal"
