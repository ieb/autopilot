"""
Route Parser
=============

Parses route files from weather routing software.
Supports CSV (e.g. Expedition, qtVlm) and KML (SailGrib) formats.
Extracts waypoints with navigation data for passage simulation.
"""

import csv
import math
import re
import xml.etree.ElementTree as ET
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
    Parser for weather routing route files.
    
    Supports:
    - CSV files (e.g. Expedition, qtVlm) with DMS coordinates and multi-line headers
    - KML files from SailGrib with sg: namespace extensions
    
    File format is auto-detected from the extension.
    """
    
    EARTH_RADIUS_NM = 3440.065
    
    def __init__(self, filepath: str):
        """
        Initialize parser with route file path.
        
        Args:
            filepath: Path to route file (.csv or .kml)
        """
        self.filepath = Path(filepath)
        self.waypoints: List[Waypoint] = []
        
    def parse(self) -> List[Waypoint]:
        """
        Parse the route file and return list of waypoints.
        
        Auto-detects CSV vs KML based on file extension.
        
        Returns:
            List of Waypoint objects representing the planned route
        """
        if not self.filepath.exists():
            raise FileNotFoundError(f"Route file not found: {self.filepath}")
            
        self.waypoints = []
        
        if self.filepath.suffix.lower() == '.kml':
            self._parse_kml()
        else:
            self._parse_csv()
            
        logger.info(f"Parsed {len(self.waypoints)} waypoints from {self.filepath.name}")
        return self.waypoints
    
    def _parse_csv(self):
        """Parse a CSV route file."""
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
                waypoint = self._parse_csv_line(line)
                if waypoint:
                    self.waypoints.append(waypoint)
            except Exception as e:
                logger.warning(f"Failed to parse line {line_num}: {e}")
                continue
    
    def _parse_kml(self):
        """Parse a SailGrib KML route file."""
        tree = ET.parse(self.filepath)
        root = tree.getroot()
        
        ns = {'kml': 'http://www.opengis.net/kml/2.2'}
        sg_ns = 'http://www.sailgrib.com'
        
        # Find the routing folder (contains the isochrone placemarks)
        routing_folder = None
        for folder in root.findall('.//kml:Folder', ns):
            ext_data = folder.find('kml:ExtendedData', ns)
            if ext_data is not None:
                for child in ext_data:
                    if child.tag == f'{{{sg_ns}}}type' and child.text == 'routing':
                        routing_folder = folder
                        break
            if routing_folder is not None:
                break
        
        if routing_folder is None:
            raise ValueError("No SailGrib routing folder found in KML file")
        
        # Collect isochrone placemarks (skip the LineString placemark at the end)
        raw_points = []
        for pm in routing_folder.findall('kml:Placemark', ns):
            coords_elem = pm.find('kml:Point/kml:coordinates', ns)
            ts_elem = pm.find('kml:TimeStamp/kml:when', ns)
            if coords_elem is None or ts_elem is None:
                continue
            
            lon_str, lat_str = coords_elem.text.strip().split(',')[:2]
            lat = float(lat_str)
            lon = float(lon_str)
            
            timestamp = datetime.fromisoformat(ts_elem.text.replace('Z', '+00:00'))
            # Strip timezone for consistency with CSV parser
            timestamp = timestamp.replace(tzinfo=None)
            
            ext_data = pm.find('kml:ExtendedData', ns)
            boat_vars = {}
            weather_vars = {}
            if ext_data is not None:
                for child in ext_data:
                    tag = child.tag.split('}')[-1] if '}' in child.tag else child.tag
                    name = child.get('name', '')
                    if tag == 'BoatVariable':
                        boat_vars[name] = child.text
                    elif tag == 'WeatherVariable':
                        weather_vars[name] = child.text
            
            raw_points.append({
                'lat': lat, 'lon': lon,
                'timestamp': timestamp,
                'cog': float(boat_vars.get('cog', 0)),
                'sog': float(boat_vars.get('sog', 0)),
                'dog': float(boat_vars.get('dog', 0)),
                'stw': float(boat_vars.get('stw', 0)),
                'swa': float(boat_vars.get('swa', 0)),
                'dtw': float(boat_vars.get('dtw', 0)),
                'ismotoring': float(boat_vars.get('ismotoring', 0)) > 0.5,
                'tws': float(weather_vars.get('tws', 0)),
                'twd': float(weather_vars.get('twd', 0)),
                'cs': float(weather_vars.get('cs', 0)),
                'cd': float(weather_vars.get('cd', 0)),
            })
        
        if len(raw_points) < 2:
            raise ValueError(f"KML file has fewer than 2 routing placemarks ({len(raw_points)} found)")
        
        # Skip the final placemark if it has zero wind/speed (arrival marker)
        last = raw_points[-1]
        if last['tws'] == 0 and last['sog'] == 0 and last['cog'] == 0:
            arrival_point = raw_points.pop()
        else:
            arrival_point = None
        
        # Build Waypoint objects: consecutive points form from/to pairs
        for i, pt in enumerate(raw_points):
            if i < len(raw_points) - 1:
                next_pt = raw_points[i + 1]
                to_lat = next_pt['lat']
                to_lon = next_pt['lon']
            elif arrival_point is not None:
                to_lat = arrival_point['lat']
                to_lon = arrival_point['lon']
            else:
                # Last point with no arrival marker - estimate destination from COG/DOG
                to_lat, to_lon = self._project_position(
                    pt['lat'], pt['lon'], pt['cog'], pt['dog']
                )
            
            self.waypoints.append(Waypoint(
                timestamp=pt['timestamp'],
                from_lat=pt['lat'],
                from_lon=pt['lon'],
                to_lat=to_lat,
                to_lon=to_lon,
                cog=pt['cog'],
                sog=pt['sog'],
                distance=pt['dog'],
                tws=pt['tws'],
                twd=pt['twd'],
                surface_wind_angle=pt['swa'],
                distance_to_finish=pt['dtw'],
                is_motoring=pt['ismotoring'],
                current_speed=pt['cs'],
                current_direction=pt['cd'],
            ))
    
    @staticmethod
    def _project_position(lat: float, lon: float, bearing_deg: float, distance_nm: float) -> Tuple[float, float]:
        """Project a position along a bearing for a given distance."""
        R = RouteParser.EARTH_RADIUS_NM
        d = distance_nm / R
        brng = math.radians(bearing_deg)
        lat1 = math.radians(lat)
        lon1 = math.radians(lon)
        
        lat2 = math.asin(
            math.sin(lat1) * math.cos(d) +
            math.cos(lat1) * math.sin(d) * math.cos(brng)
        )
        lon2 = lon1 + math.atan2(
            math.sin(brng) * math.sin(d) * math.cos(lat1),
            math.cos(d) - math.sin(lat1) * math.sin(lat2)
        )
        
        return math.degrees(lat2), math.degrees(lon2)
        
    def _parse_csv_line(self, line: str) -> Optional[Waypoint]:
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


def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Compute initial bearing from (lat1, lon1) to (lat2, lon2) in degrees [0, 360)."""
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlon_rad = math.radians(lon2 - lon1)

    x = math.sin(dlon_rad) * math.cos(lat2_rad)
    y = (math.cos(lat1_rad) * math.sin(lat2_rad) -
         math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad))

    return math.degrees(math.atan2(x, y)) % 360


def _distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Compute great-circle distance in nautical miles."""
    EARTH_RADIUS_NM = 3440.065
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlat_rad = math.radians(lat2 - lat1)
    dlon_rad = math.radians(lon2 - lon1)

    a = (math.sin(dlat_rad / 2) ** 2 +
         math.cos(lat1_rad) * math.cos(lat2_rad) *
         math.sin(dlon_rad / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return EARTH_RADIUS_NM * c


def _bearing_diff(a: float, b: float) -> float:
    """Absolute angular difference between two bearings in degrees."""
    d = abs(a - b) % 360
    return d if d <= 180 else 360 - d


def consolidate_legs(waypoints: List[Waypoint], threshold_deg: float = 10.0) -> List[Waypoint]:
    """
    Merge consecutive waypoints with similar bearings into single longer legs.

    Two checks prevent over-merging:
    1. Consecutive leg-to-leg bearing change must be within threshold (catches zigzags)
    2. Overall bearing from group start to candidate endpoint must stay within
       threshold of the initial group bearing (catches gradual curves)

    Merging never crosses a motoring/sailing boundary.

    Args:
        waypoints: List of Waypoint objects from route parser
        threshold_deg: Maximum bearing change to allow merging

    Returns:
        Consolidated list of Waypoint objects
    """
    if len(waypoints) <= 1:
        return list(waypoints)

    result: List[Waypoint] = []

    group_start = 0
    group_end = 0  # inclusive
    # Fixed reference: bearing of the first leg in the current group
    initial_bearing = _bearing(
        waypoints[0].from_lat, waypoints[0].from_lon,
        waypoints[0].to_lat, waypoints[0].to_lon,
    )

    for i in range(1, len(waypoints)):
        wp = waypoints[i]
        first = waypoints[group_start]
        prev = waypoints[group_end]

        # Never merge across motoring/sailing boundary
        if wp.is_motoring != first.is_motoring:
            result.append(_merge_group(waypoints, group_start, group_end))
            group_start = i
            group_end = i
            initial_bearing = _bearing(wp.from_lat, wp.from_lon, wp.to_lat, wp.to_lon)
            continue

        # Check 1: Consecutive leg bearing change (catches zigzag patterns)
        leg_bearing = _bearing(wp.from_lat, wp.from_lon, wp.to_lat, wp.to_lon)
        prev_bearing = _bearing(prev.from_lat, prev.from_lon, prev.to_lat, prev.to_lon)
        if _bearing_diff(prev_bearing, leg_bearing) > threshold_deg:
            result.append(_merge_group(waypoints, group_start, group_end))
            group_start = i
            group_end = i
            initial_bearing = leg_bearing
            continue

        # Check 2: Overall bearing drift from initial group bearing
        candidate_bearing = _bearing(
            first.from_lat, first.from_lon,
            wp.to_lat, wp.to_lon,
        )
        if _bearing_diff(initial_bearing, candidate_bearing) > threshold_deg:
            result.append(_merge_group(waypoints, group_start, group_end))
            group_start = i
            group_end = i
            initial_bearing = leg_bearing
            continue

        # Extend group
        group_end = i

    # Emit final group
    result.append(_merge_group(waypoints, group_start, group_end))

    logger.info(f"Consolidated {len(waypoints)} legs -> {len(result)} legs (threshold={threshold_deg}°)")
    return result


def _merge_group(waypoints: List[Waypoint], start: int, end: int) -> Waypoint:
    """Merge waypoints[start..end] into a single waypoint."""
    first = waypoints[start]
    last = waypoints[end]

    if start == end:
        return first

    from_lat = first.from_lat
    from_lon = first.from_lon
    to_lat = last.to_lat
    to_lon = last.to_lon

    return Waypoint(
        timestamp=first.timestamp,
        from_lat=from_lat,
        from_lon=from_lon,
        to_lat=to_lat,
        to_lon=to_lon,
        cog=_bearing(from_lat, from_lon, to_lat, to_lon),
        sog=first.sog,
        distance=_distance(from_lat, from_lon, to_lat, to_lon),
        tws=first.tws,
        twd=first.twd,
        surface_wind_angle=first.surface_wind_angle,
        distance_to_finish=last.distance_to_finish,
        is_motoring=first.is_motoring,
        current_speed=first.current_speed,
        current_direction=first.current_direction,
    )


def parse_route(filepath: str, consolidate: bool = True) -> List[Waypoint]:
    """
    Convenience function to parse a route file.

    Args:
        filepath: Path to route file (.csv or .kml)
        consolidate: If True, merge collinear legs (default True)

    Returns:
        List of Waypoint objects
    """
    parser = RouteParser(filepath)
    waypoints = parser.parse()
    if consolidate and waypoints:
        waypoints = consolidate_legs(waypoints)
    return waypoints
