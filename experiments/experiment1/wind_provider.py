"""
Wind Provider
=============

Provides wind data at any position and time by interpolating
from GRIB files or falling back to route-predicted values.
"""

from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Tuple
import logging

from .grib_reader import GribReader
from .route_parser import Waypoint

logger = logging.getLogger(__name__)


@dataclass
class WindConditions:
    """Wind conditions at a specific point."""
    tws: float          # True wind speed (knots)
    twd: float          # True wind direction (degrees)
    source: str         # 'grib', 'route', or 'default'
    
    @property
    def tws_mps(self) -> float:
        """Wind speed in m/s."""
        return self.tws / 1.94384


class WindProvider:
    """
    Provides wind data for simulation.
    
    Priority:
    1. GRIB data (if available and covers the position/time)
    2. Route predicted values (from the route planner)
    3. Default values
    """
    
    def __init__(self, 
                 grib_reader: Optional[GribReader] = None,
                 waypoints: Optional[List[Waypoint]] = None):
        """
        Initialize wind provider.
        
        Args:
            grib_reader: GribReader with loaded data
            waypoints: List of waypoints with predicted wind
        """
        self.grib_reader = grib_reader
        self.waypoints = waypoints or []
        
        # Cache for performance
        self._last_query: Optional[Tuple[float, float, datetime]] = None
        self._last_result: Optional[WindConditions] = None
        
        # Default wind if nothing else available
        self.default_tws = 15.0
        self.default_twd = 180.0
        
    def get_wind(self, lat: float, lon: float, time: datetime) -> WindConditions:
        """
        Get wind conditions at a position and time.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            time: Query time
            
        Returns:
            WindConditions with TWS, TWD, and source
        """
        # Check cache (same position within ~100m and same second)
        if self._last_query is not None:
            last_lat, last_lon, last_time = self._last_query
            if (abs(lat - last_lat) < 0.001 and 
                abs(lon - last_lon) < 0.001 and
                abs((time - last_time).total_seconds()) < 1):
                return self._last_result
                
        # Try GRIB data first
        if self.grib_reader and self.grib_reader.get_wind_times():
            try:
                tws, twd = self.grib_reader.get_wind_at(lat, lon, time)
                result = WindConditions(tws=tws, twd=twd, source='grib')
                self._cache_result(lat, lon, time, result)
                return result
            except Exception as e:
                logger.debug(f"GRIB wind lookup failed: {e}")
                
        # Fall back to route predicted values
        if self.waypoints:
            result = self._get_from_route(time)
            if result:
                self._cache_result(lat, lon, time, result)
                return result
                
        # Default values
        result = WindConditions(
            tws=self.default_tws,
            twd=self.default_twd,
            source='default'
        )
        self._cache_result(lat, lon, time, result)
        return result
        
    def _get_from_route(self, time: datetime) -> Optional[WindConditions]:
        """Get wind from route waypoints by interpolating in time."""
        if not self.waypoints:
            return None
            
        # Find bracketing waypoints
        for i, wp in enumerate(self.waypoints):
            if i < len(self.waypoints) - 1:
                next_wp = self.waypoints[i + 1]
                if wp.timestamp <= time <= next_wp.timestamp:
                    # Interpolate between waypoints
                    dt_total = (next_wp.timestamp - wp.timestamp).total_seconds()
                    dt_query = (time - wp.timestamp).total_seconds()
                    frac = dt_query / dt_total if dt_total > 0 else 0.0
                    
                    tws = wp.tws + (next_wp.tws - wp.tws) * frac
                    
                    # Handle direction wraparound
                    twd0 = wp.twd
                    twd1 = next_wp.twd
                    diff = twd1 - twd0
                    if diff > 180:
                        diff -= 360
                    elif diff < -180:
                        diff += 360
                    twd = (twd0 + diff * frac) % 360
                    
                    return WindConditions(tws=tws, twd=twd, source='route')
                    
        # Before first waypoint or after last
        if time < self.waypoints[0].timestamp:
            wp = self.waypoints[0]
            return WindConditions(tws=wp.tws, twd=wp.twd, source='route')
        else:
            wp = self.waypoints[-1]
            return WindConditions(tws=wp.tws, twd=wp.twd, source='route')
            
    def _cache_result(self, lat: float, lon: float, time: datetime,
                      result: WindConditions):
        """Cache the last query result."""
        self._last_query = (lat, lon, time)
        self._last_result = result
        
    def get_tws_range(self) -> Tuple[float, float]:
        """Get the range of TWS values from waypoints."""
        if not self.waypoints:
            return (self.default_tws, self.default_tws)
        tws_values = [wp.tws for wp in self.waypoints]
        return (min(tws_values), max(tws_values))
        
    def get_twd_at_start(self) -> float:
        """Get the TWD at the start of the route."""
        if self.waypoints:
            return self.waypoints[0].twd
        return self.default_twd
