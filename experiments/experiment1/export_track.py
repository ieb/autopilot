"""
Track Exporter
==============

Exports simulation track data to GPX format for use with OpenSeaMap
and other mapping applications.

Usage:
    uv run python -m experiments.experiment1.export_track results/experiment1/time_series.csv
    
    # With custom output path
    uv run python -m experiments.experiment1.export_track results/experiment1/time_series.csv -o track.gpx
    
    # Include waypoints from route
    uv run python -m experiments.experiment1.export_track results/experiment1/time_series.csv \\
        --route data/experiment1/route/wr_route_1_20260125_005338.csv
"""

import argparse
import csv
import sys
from datetime import datetime, timedelta
from pathlib import Path
from typing import List, Optional, Tuple
import xml.etree.ElementTree as ET
from xml.dom import minidom


def parse_time_series(csv_path: Path) -> List[dict]:
    """Parse time series CSV file."""
    points = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append({
                'timestamp': float(row['timestamp']),
                'latitude': float(row['latitude']),
                'longitude': float(row['longitude']),
                'heading': float(row.get('heading', 0)),
                'sog': float(row.get('sog', 0)),
                'stw': float(row.get('stw', 0)),
                'tws': float(row.get('tws', 0)),
                'twd': float(row.get('twd', 0)),
                'steering_mode': row.get('steering_mode', ''),
            })
    return points


def create_gpx(
    points: List[dict],
    track_name: str = "Simulated Passage",
    start_time: Optional[datetime] = None,
    waypoints: Optional[List[Tuple[float, float, str]]] = None
) -> str:
    """
    Create GPX XML from track points.
    
    Args:
        points: List of track points with lat, lon, timestamp
        track_name: Name for the track
        start_time: Start time for the track (defaults to now)
        waypoints: Optional list of (lat, lon, name) tuples for route waypoints
        
    Returns:
        GPX XML string
    """
    if start_time is None:
        start_time = datetime.utcnow()
    
    # Create root element with namespaces
    gpx = ET.Element('gpx')
    gpx.set('version', '1.1')
    gpx.set('creator', 'ML Yacht Autopilot Simulator')
    gpx.set('xmlns', 'http://www.topografix.com/GPX/1/1')
    gpx.set('xmlns:xsi', 'http://www.w3.org/2001/XMLSchema-instance')
    gpx.set('xsi:schemaLocation', 
            'http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd')
    
    # Add metadata
    metadata = ET.SubElement(gpx, 'metadata')
    name_elem = ET.SubElement(metadata, 'name')
    name_elem.text = track_name
    time_elem = ET.SubElement(metadata, 'time')
    time_elem.text = start_time.strftime('%Y-%m-%dT%H:%M:%SZ')
    
    # Add waypoints if provided
    if waypoints:
        for lat, lon, wpt_name in waypoints:
            wpt = ET.SubElement(gpx, 'wpt')
            wpt.set('lat', f'{lat:.6f}')
            wpt.set('lon', f'{lon:.6f}')
            wpt_name_elem = ET.SubElement(wpt, 'name')
            wpt_name_elem.text = wpt_name
    
    # Create track
    trk = ET.SubElement(gpx, 'trk')
    trk_name = ET.SubElement(trk, 'name')
    trk_name.text = track_name
    
    # Track segment
    trkseg = ET.SubElement(trk, 'trkseg')
    
    # Add track points
    for point in points:
        trkpt = ET.SubElement(trkseg, 'trkpt')
        trkpt.set('lat', f'{point["latitude"]:.6f}')
        trkpt.set('lon', f'{point["longitude"]:.6f}')
        
        # Add time
        point_time = start_time + timedelta(seconds=point['timestamp'])
        time_elem = ET.SubElement(trkpt, 'time')
        time_elem.text = point_time.strftime('%Y-%m-%dT%H:%M:%SZ')
        
        # Add speed as extension (SOG in m/s for GPX standard)
        # 1 knot = 0.514444 m/s
        if point.get('sog', 0) > 0:
            speed_ms = point['sog'] * 0.514444
            # Use extensions for additional data
            extensions = ET.SubElement(trkpt, 'extensions')
            
            # Course/heading
            course = ET.SubElement(extensions, 'course')
            course.text = f'{point.get("heading", 0):.1f}'
            
            # Speed
            speed = ET.SubElement(extensions, 'speed')
            speed.text = f'{speed_ms:.2f}'
    
    # Pretty print
    xml_str = ET.tostring(gpx, encoding='unicode')
    dom = minidom.parseString(xml_str)
    return dom.toprettyxml(indent='  ')


def parse_route_waypoints(route_path: Path) -> List[Tuple[float, float, str]]:
    """Parse waypoints from route CSV file."""
    from .route_parser import RouteParser
    
    parser = RouteParser(str(route_path))
    waypoints = parser.parse()
    
    result = []
    for i, wp in enumerate(waypoints):
        name = f"WPT{i:02d}"
        if i == 0:
            name = "START"
        elif i == len(waypoints) - 1:
            name = "FINISH"
        result.append((wp.to_lat, wp.to_lon, name))
    
    return result


def main():
    parser = argparse.ArgumentParser(
        description='Export simulation track to GPX format for OpenSeaMap',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s results/experiment1/time_series.csv
  %(prog)s results/experiment1/time_series.csv -o passage_track.gpx
  %(prog)s results/experiment1/time_series.csv --route data/experiment1/route/route.csv
        """
    )
    
    parser.add_argument(
        'input',
        type=str,
        help='Path to time_series.csv from experiment output'
    )
    
    parser.add_argument(
        '-o', '--output',
        type=str,
        default=None,
        help='Output GPX file path (default: same directory as input, .gpx extension)'
    )
    
    parser.add_argument(
        '--route',
        type=str,
        default=None,
        help='Optional route CSV file to include waypoints'
    )
    
    parser.add_argument(
        '--name',
        type=str,
        default='Simulated Passage',
        help='Track name in GPX file'
    )
    
    parser.add_argument(
        '--start-time',
        type=str,
        default=None,
        help='Start time for track (ISO format, e.g., 2026-01-25T00:00:00Z)'
    )
    
    parser.add_argument(
        '--sample-interval',
        type=int,
        default=60,
        help='Sample interval in seconds (default: 60, reduces file size)'
    )
    
    args = parser.parse_args()
    
    # Validate input
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"Error: Input file not found: {input_path}", file=sys.stderr)
        sys.exit(1)
    
    # Determine output path
    if args.output:
        output_path = Path(args.output)
    else:
        output_path = input_path.with_suffix('.gpx')
    
    # Parse start time
    start_time = None
    if args.start_time:
        try:
            start_time = datetime.fromisoformat(args.start_time.replace('Z', '+00:00'))
        except ValueError:
            print(f"Error: Invalid start time format: {args.start_time}", file=sys.stderr)
            sys.exit(1)
    
    # Read track data
    print(f"Reading track data from {input_path}...")
    points = parse_time_series(input_path)
    
    if not points:
        print("Error: No track points found in input file", file=sys.stderr)
        sys.exit(1)
    
    print(f"  Found {len(points)} track points")
    
    # Sample points to reduce file size
    if args.sample_interval > 0:
        sampled = []
        last_time = -args.sample_interval
        for point in points:
            if point['timestamp'] - last_time >= args.sample_interval:
                sampled.append(point)
                last_time = point['timestamp']
        # Always include last point
        if points[-1] not in sampled:
            sampled.append(points[-1])
        print(f"  Sampled to {len(sampled)} points (interval: {args.sample_interval}s)")
        points = sampled
    
    # Parse waypoints if route provided
    waypoints = None
    if args.route:
        route_path = Path(args.route)
        if route_path.exists():
            print(f"Reading waypoints from {route_path}...")
            try:
                waypoints = parse_route_waypoints(route_path)
                print(f"  Found {len(waypoints)} waypoints")
            except Exception as e:
                print(f"  Warning: Could not parse route: {e}", file=sys.stderr)
    
    # Create GPX
    print(f"Creating GPX file...")
    gpx_content = create_gpx(
        points=points,
        track_name=args.name,
        start_time=start_time,
        waypoints=waypoints
    )
    
    # Write output
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(gpx_content)
    
    print(f"Track exported to: {output_path}")
    print(f"\nTo view on OpenSeaMap:")
    print(f"  1. Go to https://map.openseamap.org/")
    print(f"  2. Use a GPX viewer overlay or import the file")
    print(f"  3. Or upload to https://www.gpsvisualizer.com/ to view on OpenSeaMap")


if __name__ == '__main__':
    main()
