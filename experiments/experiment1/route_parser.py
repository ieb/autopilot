"""
Route Parser
=============

Parses route CSV files from weather routing software.
Extracts waypoints with navigation data for passage simulation.
"""

import csv
import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple
import logging

logger = logging.getLogger(__name__)


@dataclass
class Waypoint:
    """A waypoint in the planned route."""
    # Timing
    timestamp: datetime
    
    # Position (decimal degrees)
    from_lat: float
    from_lon: float
    to_lat: float
    to_lon: float
    
    # Navigation
    cog: float              # Course over ground (degrees)
    sog: float              # Speed over ground (knots)
    distance: float         # Distance for this leg (nm)
    
    # Wind conditions (from route planner)
    tws: float              # True wind speed (knots)
    twd: float              # True wind direction (degrees)
    surface_wind_angle: float  # Surface wind angle (degrees)
    
    # Route info
    distance_to_finish: float  # Remaining distance (nm)
    is_motoring: bool          # Whether motoring on this leg
    
    # Current (if available)
    current_speed: float = 0.0  # Current speed (knots)
    current_direction: float = 0.0  # Current direction (degrees)
    
    @property
    def twa(self) -> float:
        """Compute true wind angle from TWD and COG."""
        twa = self.twd - self.cog
        while twa > 180:
            twa -= 360
        while twa < -180:
            twa += 360
        return twa
    
    @property
    def duration_seconds(self) -> float:
        """Estimated duration for this leg in seconds."""
        if self.sog > 0:
            return (self.distance / self.sog) * 3600
        return 0.0


class RouteParser:
    """
    Parser for weather routing CSV files.
    
    Handles the specific format from weather routing software with:
    - DMS coordinate format (e.g., 51¡56.578'N)
    - Multi-line headers
    - Various navigation parameters
    """
    
    def __init__(self, filepath: str):
        """
        Initialize parser with route file path.
        
        Args:
            filepath: Path to the CSV route file
        """
        self.filepath = Path(filepath)
        self.waypoints: List[Waypoint] = []
        
    def parse(self) -> List[Waypoint]:
        """
        Parse the route file and return list of waypoints.
        
        Returns:
            List of Waypoint objects representing the planned route
        """
        if not self.filepath.exists():
            raise FileNotFoundError(f"Route file not found: {self.filepath}")
            
        self.waypoints = []
        
        # Try different encodings - file may use latin-1 or similar
        for encoding in ['utf-8', 'latin-1', 'cp1252', 'iso-8859-1']:
            try:
                with open(self.filepath, 'r', encoding=encoding) as f:
                    lines = f.readlines()
                break
            except UnicodeDecodeError:
                continue
        else:
            raise ValueError(f"Could not decode file with any supported encoding")
            
        # Skip header (first 2 lines based on the format)
        data_lines = lines[2:]
        
        for line_num, line in enumerate(data_lines, start=3):
            line = line.strip()
            if not line:
                continue
                
            try:
                waypoint = self._parse_line(line)
                if waypoint:
                    self.waypoints.append(waypoint)
            except Exception as e:
                logger.warning(f"Failed to parse line {line_num}: {e}")
                continue
                
        logger.info(f"Parsed {len(self.waypoints)} waypoints from {self.filepath.name}")
        
        return self.waypoints
        
    def _parse_line(self, line: str) -> Optional[Waypoint]:
        """Parse a single CSV line into a Waypoint."""
        # Split by comma, handling potential edge cases
        parts = line.split(',')
        
        if len(parts) < 15:
            logger.warning(f"Line has insufficient columns: {len(parts)}")
            return None
            
        # Parse timestamp
        timestamp = self._parse_timestamp(parts[0])
        if not timestamp:
            return None
            
        # Parse coordinates (DMS format)
        from_lat = self._parse_coordinate(parts[1])
        from_lon = self._parse_coordinate(parts[2])
        to_lat = self._parse_coordinate(parts[3])
        to_lon = self._parse_coordinate(parts[4])
        
        # Parse navigation data
        cog = float(parts[5])
        sog = float(parts[6])
        distance = float(parts[7])
        
        # Parse wind data
        tws = float(parts[8])
        twd = float(parts[9])
        surface_wind_angle = float(parts[10])
        
        # Parse distance to finish
        distance_to_finish = float(parts[11])
        
        # Parse motoring flag (column 12, empty or 0/1)
        is_motoring = self._parse_motoring(parts[12] if len(parts) > 12 else '')
        
        # Parse current data if available
        current_speed = 0.0
        current_direction = 0.0
        if len(parts) > 13:
            try:
                current_speed = float(parts[13]) if parts[13] else 0.0
            except ValueError:
                pass
        if len(parts) > 14:
            try:
                current_direction = float(parts[14]) if parts[14] else 0.0
            except ValueError:
                pass
        
        return Waypoint(
            timestamp=timestamp,
            from_lat=from_lat,
            from_lon=from_lon,
            to_lat=to_lat,
            to_lon=to_lon,
            cog=cog,
            sog=sog,
            distance=distance,
            tws=tws,
            twd=twd,
            surface_wind_angle=surface_wind_angle,
            distance_to_finish=distance_to_finish,
            is_motoring=is_motoring,
            current_speed=current_speed,
            current_direction=current_direction,
        )
        
    def _parse_timestamp(self, s: str) -> Optional[datetime]:
        """Parse timestamp string like '25 Jan 2026 00:00:00'."""
        s = s.strip()
        try:
            return datetime.strptime(s, '%d %b %Y %H:%M:%S')
        except ValueError:
            logger.warning(f"Failed to parse timestamp: {s}")
            return None
            
    def _parse_coordinate(self, s: str) -> float:
        """
        Parse coordinate string in DMS format.
        
        Examples:
            - "51¡56.578'N" -> 51.94296667
            - "001¡23.998'E" -> 1.39996667
            
        Args:
            s: Coordinate string in DMS format
            
        Returns:
            Decimal degrees (positive for N/E, negative for S/W)
        """
        s = s.strip()
        
        # Pattern to match DMS format: degrees¡minutes'direction
        # The ¡ character is a special degree symbol used in this format
        pattern = r"(\d+)[¡°](\d+\.?\d*)'([NSEW])"
        match = re.match(pattern, s)
        
        if match:
            degrees = int(match.group(1))
            minutes = float(match.group(2))
            direction = match.group(3)
            
            # Convert to decimal degrees
            decimal = degrees + (minutes / 60.0)
            
            # Apply sign based on direction
            if direction in ('S', 'W'):
                decimal = -decimal
                
            return decimal
        else:
            # Try parsing as plain decimal (fallback)
            try:
                return float(s)
            except ValueError:
                logger.warning(f"Failed to parse coordinate: {s}")
                return 0.0
                
    def _parse_motoring(self, s: str) -> bool:
        """Parse motoring flag (empty, 0, or 1)."""
        s = s.strip()
        if not s:
            return False
        try:
            return int(s) == 1
        except ValueError:
            return False
            
    @property
    def start_time(self) -> Optional[datetime]:
        """Get the start time of the route."""
        if self.waypoints:
            return self.waypoints[0].timestamp
        return None
        
    @property
    def end_time(self) -> Optional[datetime]:
        """Get the planned end time of the route."""
        if self.waypoints:
            return self.waypoints[-1].timestamp
        return None
        
    @property
    def total_distance(self) -> float:
        """Get total route distance in nautical miles."""
        if self.waypoints:
            return self.waypoints[0].distance_to_finish
        return 0.0
        
    @property
    def planned_duration_seconds(self) -> float:
        """Get planned duration in seconds."""
        if self.start_time and self.end_time:
            return (self.end_time - self.start_time).total_seconds()
        return 0.0
        
    def get_waypoint_at_time(self, time: datetime) -> Tuple[Optional[Waypoint], int]:
        """
        Find the active waypoint at a given time.
        
        Args:
            time: The time to query
            
        Returns:
            Tuple of (waypoint, index) or (None, -1) if not found
        """
        for i, wp in enumerate(self.waypoints):
            # Check if time is within this waypoint's time range
            if i < len(self.waypoints) - 1:
                next_wp = self.waypoints[i + 1]
                if wp.timestamp <= time < next_wp.timestamp:
                    return (wp, i)
            else:
                # Last waypoint
                if wp.timestamp <= time:
                    return (wp, i)
        return (None, -1)
        
    def get_route_bounds(self) -> Tuple[float, float, float, float]:
        """
        Get the bounding box of the route.
        
        Returns:
            Tuple of (min_lat, max_lat, min_lon, max_lon)
        """
        if not self.waypoints:
            return (0.0, 0.0, 0.0, 0.0)
            
        lats = []
        lons = []
        for wp in self.waypoints:
            lats.extend([wp.from_lat, wp.to_lat])
            lons.extend([wp.from_lon, wp.to_lon])
            
        return (min(lats), max(lats), min(lons), max(lons))


def parse_route(filepath: str) -> List[Waypoint]:
    """
    Convenience function to parse a route file.
    
    Args:
        filepath: Path to the CSV route file
        
    Returns:
        List of Waypoint objects
    """
    parser = RouteParser(filepath)
    return parser.parse()
