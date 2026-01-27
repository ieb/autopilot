#!/usr/bin/env python3
"""
GRIB Visualization Server
=========================

Flask server that serves GRIB weather data visualization.
Provides API endpoints for wind and wave data from GRIB files.
"""

import argparse
import logging
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
from flask import Flask, jsonify, request, send_from_directory

# Add project root to path for imports
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from experiments.experiment1.grib_reader import GribReader
from experiments.experiment1.route_parser import RouteParser

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__, static_folder='static')

# Global GRIB reader instance
grib_reader: Optional[GribReader] = None

# Global directories for routes and results
routes_dir: Optional[Path] = None
results_dir: Optional[Path] = None


def sanitize_path(base_dir: Path, name: str) -> Optional[Path]:
    """
    Sanitize a user-provided filename to prevent path traversal attacks.
    
    Args:
        base_dir: The allowed base directory
        name: User-provided filename or directory name
        
    Returns:
        Sanitized Path if valid, None if the path is unsafe
    """
    import os
    
    # Reject empty names
    if not name or not name.strip():
        return None
    
    # Get absolute base path
    base_path = os.path.abspath(str(base_dir))
    
    # Normalize the full path to resolve any .. or . components
    full_path = os.path.normpath(os.path.join(base_path, name))
    
    # Verify the normalized path is still under the base directory
    # Must start with base_path + separator to prevent prefix attacks
    # (e.g., /base/dir vs /base/dir_other)
    if not full_path.startswith(base_path + os.sep) and full_path != base_path:
        return None
    
    return Path(full_path)


def clear_cfgrib_cache() -> int:
    """
    Clear cfgrib index cache files from temp directory.
    
    These .idx files can become corrupted and cause EOFError on restart.
    Clearing them forces cfgrib to regenerate the index.
    
    Returns:
        Number of files deleted
    """
    import tempfile
    import glob
    
    temp_dir = tempfile.gettempdir()
    patterns = [
        '*.grb.*.idx',
        '*.grb2.*.idx',
    ]
    
    deleted = 0
    for pattern in patterns:
        for idx_file in glob.glob(str(Path(temp_dir) / pattern)):
            try:
                Path(idx_file).unlink()
                deleted += 1
                logger.debug(f"Deleted cfgrib cache: {idx_file}")
            except OSError as e:
                logger.warning(f"Failed to delete {idx_file}: {e}")
    
    if deleted > 0:
        logger.info(f"Cleared {deleted} cfgrib cache file(s)")
    
    return deleted


def init_grib_reader(grib_dir: str) -> bool:
    """Initialize the GRIB reader with data from the specified directory."""
    global grib_reader
    
    # Clear potentially corrupted cfgrib cache files
    clear_cfgrib_cache()
    
    grib_path = Path(grib_dir)
    if not grib_path.exists():
        logger.error(f"GRIB directory not found: {grib_dir}")
        return False
    
    grib_reader = GribReader(grib_dir)
    try:
        files_loaded = grib_reader.load_directory()
        logger.info(f"Loaded {files_loaded} GRIB files")
        return files_loaded > 0
    except Exception as e:
        logger.error(f"Failed to load GRIB files: {e}")
        return False


@app.route('/')
def index():
    """Serve the main HTML page."""
    return send_from_directory(app.static_folder, 'index.html')


@app.route('/api/debug/grib')
def debug_grib() -> Dict[str, Any]:
    """
    Debug endpoint to show what data is available in the GRIB files.
    Scans raw GRIB files to list ALL available parameters.
    """
    if grib_reader is None:
        return jsonify({'error': 'No GRIB data loaded'}), 500
    
    result = {
        'wind_times_count': len(grib_reader.wind_data),
        'wave_times_count': len(grib_reader.wave_data),
        'loaded_wave_fields': [],
        'raw_grib_parameters': [],
    }
    
    # Check what wave fields were loaded
    for time, wave_list in list(grib_reader.wave_data.items())[:2]:
        for wave in wave_list:
            result['loaded_wave_fields'].append({
                'time': time.isoformat(),
                'has_swh': wave.swh is not None,
                'has_mwp': wave.mwp is not None,
                'has_mwd': wave.mwd is not None,
                'swh_name': wave.swh.name if wave.swh else None,
                'mwp_name': wave.mwp.name if wave.mwp else None,
                'mwd_name': wave.mwd.name if wave.mwd else None,
            })
    
    # Scan raw GRIB files to find ALL parameters
    try:
        import cfgrib
        grib_path = Path(grib_reader.grib_dir)
        
        for pattern in ['*.grb', '*.grib', '*.grb2', '*.grb.bz2', '*.grib.bz2']:
            for grib_file in list(grib_path.glob(pattern))[:3]:  # Limit to first 3 files
                file_info = {'file': grib_file.name, 'datasets': []}
                
                # Handle bz2 compression
                if grib_file.suffix == '.bz2':
                    actual_file = grib_reader._decompress_bz2(grib_file)
                else:
                    actual_file = grib_file
                
                try:
                    datasets = cfgrib.open_datasets(str(actual_file))
                    for ds in datasets:
                        ds_info = {
                            'variables': list(ds.data_vars),
                            'coords': list(ds.coords),
                            'attrs': {}
                        }
                        # Get variable attributes
                        for var in ds.data_vars:
                            da = ds[var]
                            ds_info['attrs'][var] = {
                                'long_name': da.attrs.get('long_name', ''),
                                'units': da.attrs.get('units', ''),
                                'standard_name': da.attrs.get('standard_name', ''),
                            }
                        file_info['datasets'].append(ds_info)
                except Exception as e:
                    file_info['error'] = str(e)
                
                result['raw_grib_parameters'].append(file_info)
                
    except ImportError:
        result['raw_grib_parameters'] = 'cfgrib not available'
    except Exception as e:
        result['raw_grib_parameters'] = f'Error scanning: {str(e)}'
    
    return jsonify(result)


@app.route('/api/metadata')
def get_metadata() -> Dict[str, Any]:
    """
    Return metadata about available GRIB data.
    
    Returns:
        JSON with available times, bounding box, and grid info
    """
    if grib_reader is None:
        return jsonify({'error': 'No GRIB data loaded'}), 500
    
    wind_times = grib_reader.get_wind_times()
    wave_times = grib_reader.get_wave_times()
    
    # Get bounding box from first available data
    bounds = None
    resolution = None
    
    if wind_times and wind_times[0] in grib_reader.wind_data:
        wind_list = grib_reader.wind_data[wind_times[0]]
        if wind_list:
            field = wind_list[0].u10
            bounds = {
                'lat_min': field.lat_min,
                'lat_max': field.lat_max,
                'lon_min': field.lon_min,
                'lon_max': field.lon_max,
            }
            resolution = field.resolution
    
    return jsonify({
        'wind_times': [t.isoformat() for t in wind_times],
        'wave_times': [t.isoformat() for t in wave_times],
        'bounds': bounds,
        'resolution': resolution,
    })


@app.route('/api/wind')
def get_wind() -> Dict[str, Any]:
    """
    Return wind data for a specific time.
    
    Query params:
        time: ISO format timestamp
        
    Returns:
        JSON with grid of wind data points
    """
    if grib_reader is None:
        return jsonify({'error': 'No GRIB data loaded'}), 500
    
    time_str = request.args.get('time')
    if not time_str:
        return jsonify({'error': 'Missing time parameter'}), 400
    
    try:
        query_time = datetime.fromisoformat(time_str)
    except ValueError:
        return jsonify({'error': 'Invalid time format'}), 400
    
    wind_times = grib_reader.get_wind_times()
    if not wind_times:
        return jsonify({'error': 'No wind data available'}), 404
    
    # Find closest time
    closest_time = min(wind_times, key=lambda t: abs((t - query_time).total_seconds()))
    
    if closest_time not in grib_reader.wind_data:
        return jsonify({'error': 'Wind data not found for time'}), 404
    
    wind_list = grib_reader.wind_data[closest_time]
    if not wind_list:
        return jsonify({'error': 'No wind grids available'}), 404
    
    # Use first (or highest resolution) grid
    wind = wind_list[0]
    u10 = wind.u10
    v10 = wind.v10
    
    # Convert to list of data points
    points = []
    lats = u10.lats
    lons = u10.lons
    
    # Handle decreasing latitudes
    if lats[0] > lats[-1]:
        lats = lats[::-1]
        u_data = u10.data[::-1, :]
        v_data = v10.data[::-1, :]
    else:
        u_data = u10.data
        v_data = v10.data
    
    for i, lat in enumerate(lats):
        for j, lon in enumerate(lons):
            u = float(u_data[i, j])
            v = float(v_data[i, j])
            
            # Skip NaN values
            if np.isnan(u) or np.isnan(v):
                continue
            
            # Calculate speed (m/s -> knots) and direction
            speed_mps = math.sqrt(u**2 + v**2)
            speed_knots = speed_mps * 1.94384
            
            # Wind direction (meteorological: direction wind is FROM)
            twd = math.degrees(math.atan2(-u, -v)) % 360
            
            points.append({
                'lat': float(lat),
                'lon': float(lon),
                'u': u,
                'v': v,
                'speed': round(speed_knots, 1),
                'direction': round(twd, 1),
            })
    
    return jsonify({
        'time': closest_time.isoformat(),
        'points': points,
        'bounds': {
            'lat_min': float(np.min(lats)),
            'lat_max': float(np.max(lats)),
            'lon_min': float(np.min(lons)),
            'lon_max': float(np.max(lons)),
        },
        'grid_size': {
            'rows': len(lats),
            'cols': len(lons),
        }
    })


@app.route('/api/waves')
def get_waves() -> Dict[str, Any]:
    """
    Return wave data for a specific time.
    Scans GRIB files directly to get all available wave parameters.
    
    Query params:
        time: ISO format timestamp
        
    Returns:
        JSON with grid of wave data points including all available parameters
    """
    if grib_reader is None:
        return jsonify({'error': 'No GRIB data loaded'}), 500
    
    time_str = request.args.get('time')
    if not time_str:
        return jsonify({'error': 'Missing time parameter'}), 400
    
    try:
        query_time = datetime.fromisoformat(time_str)
    except ValueError:
        return jsonify({'error': 'Invalid time format'}), 400
    
    # Try to load wave data directly from GRIB files for more complete data
    try:
        wave_data = load_all_wave_parameters(query_time)
        if wave_data:
            return jsonify(wave_data)
    except Exception as e:
        logger.warning(f"Direct wave loading failed: {e}, falling back to GribReader")
    
    # Fallback to GribReader data
    wave_times = grib_reader.get_wave_times()
    if not wave_times:
        return jsonify({'error': 'No wave data available'}), 404
    
    closest_time = min(wave_times, key=lambda t: abs((t - query_time).total_seconds()))
    
    if closest_time not in grib_reader.wave_data:
        return jsonify({'error': 'Wave data not found for time'}), 404
    
    wave_list = grib_reader.wave_data[closest_time]
    if not wave_list:
        return jsonify({'error': 'No wave grids available'}), 404
    
    wave = None
    for w in wave_list:
        if w.swh is not None:
            wave = w
            break
    
    if wave is None or wave.swh is None:
        return jsonify({'error': 'No significant wave height data available'}), 404
    
    swh = wave.swh
    mwp = wave.mwp
    mwd = wave.mwd
    
    points = []
    lats = swh.lats
    lons = swh.lons
    
    if lats[0] > lats[-1]:
        lats = lats[::-1]
        height_data = swh.data[::-1, :]
        period_data = mwp.data[::-1, :] if mwp else None
        dir_data = mwd.data[::-1, :] if mwd else None
    else:
        height_data = swh.data
        period_data = mwp.data if mwp else None
        dir_data = mwd.data if mwd else None
    
    for i, lat in enumerate(lats):
        for j, lon in enumerate(lons):
            height = float(height_data[i, j])
            if np.isnan(height):
                continue
            
            point = {
                'lat': float(lat),
                'lon': float(lon),
                'height': round(height, 2),
            }
            
            if period_data is not None:
                period = float(period_data[i, j])
                if not np.isnan(period):
                    point['period'] = round(period, 1)
            
            if dir_data is not None:
                direction = float(dir_data[i, j])
                if not np.isnan(direction):
                    point['direction'] = round(direction, 1)
            
            points.append(point)
    
    return jsonify({
        'time': closest_time.isoformat(),
        'points': points,
        'bounds': {
            'lat_min': float(np.min(lats)),
            'lat_max': float(np.max(lats)),
            'lon_min': float(np.min(lons)),
            'lon_max': float(np.max(lons)),
        },
        'grid_size': {
            'rows': len(lats),
            'cols': len(lons),
        },
        'available_fields': ['height', 'period', 'direction'] if period_data is not None else ['height'],
    })


def load_all_wave_parameters(query_time: datetime) -> Optional[Dict[str, Any]]:
    """
    Load all wave parameters directly from GRIB files.
    Returns more complete wave data than the standard GribReader.
    """
    try:
        import cfgrib
    except ImportError:
        return None
    
    grib_path = Path(grib_reader.grib_dir)
    
    # Wave parameter mapping: variable name patterns -> display name
    wave_params = {
        'swh': 'height',           # Significant wave height
        'shww': 'wind_wave_height', # Wind wave height
        'shts': 'swell_height',     # Swell height
        'mwp': 'period',           # Mean wave period
        'mpww': 'wind_wave_period', # Wind wave period
        'mpts': 'swell_period',     # Swell period
        'mwd': 'direction',        # Mean wave direction
        'mdww': 'wind_wave_dir',    # Wind wave direction
        'mdts': 'swell_dir',        # Swell direction
        'pp1d': 'peak_period',      # Peak period
        'mp2': 'mean_period_2',     # Secondary mean period
    }
    
    # Collect all wave data from files
    all_data: Dict[str, Dict] = {}  # param_name -> {lats, lons, data}
    
    for pattern in ['*.grb', '*.grib', '*.grb2', '*.grb.bz2', '*.grib.bz2']:
        for grib_file in grib_path.glob(pattern):
            # Handle bz2 compression
            if grib_file.suffix == '.bz2':
                actual_file = grib_reader._decompress_bz2(grib_file)
            else:
                actual_file = grib_file
            
            try:
                datasets = cfgrib.open_datasets(str(actual_file))
                for ds in datasets:
                    for var in ds.data_vars:
                        var_lower = var.lower()
                        
                        # Check if this is a wave parameter we want
                        display_name = None
                        for pattern_name, disp in wave_params.items():
                            if pattern_name in var_lower:
                                display_name = disp
                                break
                        
                        if display_name and display_name not in all_data:
                            da = ds[var]
                            
                            # Get coordinates
                            if 'latitude' in da.coords:
                                lats = da.latitude.values
                            elif 'lat' in da.coords:
                                lats = da.lat.values
                            else:
                                continue
                                
                            if 'longitude' in da.coords:
                                lons = da.longitude.values
                            elif 'lon' in da.coords:
                                lons = da.lon.values
                            else:
                                continue
                            
                            # Get data (squeeze extra dimensions)
                            data = np.squeeze(da.values)
                            if data.ndim != 2:
                                continue
                            
                            all_data[display_name] = {
                                'lats': lats,
                                'lons': lons,
                                'data': data,
                                'units': da.attrs.get('units', ''),
                            }
                            
            except Exception as e:
                logger.debug(f"Error reading {grib_file}: {e}")
                continue
    
    if not all_data or 'height' not in all_data:
        return None
    
    # Use height grid as reference
    ref = all_data['height']
    lats = ref['lats']
    lons = ref['lons']
    
    # Handle decreasing latitudes
    if lats[0] > lats[-1]:
        lats = lats[::-1]
        for key in all_data:
            all_data[key]['data'] = all_data[key]['data'][::-1, :]
    
    # Build points list
    points = []
    for i, lat in enumerate(lats):
        for j, lon in enumerate(lons):
            height = float(all_data['height']['data'][i, j])
            if np.isnan(height):
                continue
            
            point = {
                'lat': float(lat),
                'lon': float(lon),
                'height': round(height, 2),
            }
            
            # Add all other available parameters
            for param_name, param_data in all_data.items():
                if param_name == 'height':
                    continue
                try:
                    val = float(param_data['data'][i, j])
                    if not np.isnan(val):
                        point[param_name] = round(val, 2)
                except (IndexError, ValueError):
                    pass
            
            points.append(point)
    
    return {
        'time': query_time.isoformat(),
        'points': points,
        'bounds': {
            'lat_min': float(np.min(lats)),
            'lat_max': float(np.max(lats)),
            'lon_min': float(np.min(lons)),
            'lon_max': float(np.max(lons)),
        },
        'grid_size': {
            'rows': len(lats),
            'cols': len(lons),
        },
        'available_fields': list(all_data.keys()),
    }


# ============================================================================
# Route and Result API Endpoints
# ============================================================================

@app.route('/api/routes')
def list_routes() -> Dict[str, Any]:
    """List available route files."""
    if routes_dir is None or not routes_dir.exists():
        return jsonify({'routes': [], 'error': 'Routes directory not configured'})
    
    route_files = []
    for pattern in ['*.csv']:
        for f in routes_dir.glob(pattern):
            route_files.append(f.name)
    
    return jsonify({'routes': sorted(route_files)})


@app.route('/api/route/<name>')
def get_route(name: str) -> Dict[str, Any]:
    """
    Get parsed route data.
    
    Args:
        name: Route file name
        
    Returns:
        JSON with waypoints and route metadata
    """
    if routes_dir is None or not routes_dir.exists():
        return jsonify({'error': 'Routes directory not configured'}), 500
    
    # Sanitize the path to prevent directory traversal
    route_path = sanitize_path(routes_dir, name)
    if route_path is None:
        return jsonify({'error': 'Invalid route name'}), 400
    
    if not route_path.exists():
        return jsonify({'error': f'Route not found: {name}'}), 404
    
    try:
        parser = RouteParser(str(route_path))
        waypoints = parser.parse()
        
        if not waypoints:
            return jsonify({'error': 'No waypoints found in route'}), 400
        
        # Convert waypoints to JSON-serializable format
        waypoint_data = []
        for wp in waypoints:
            waypoint_data.append({
                'timestamp': wp.timestamp.isoformat(),
                'from_lat': wp.from_lat,
                'from_lon': wp.from_lon,
                'to_lat': wp.to_lat,
                'to_lon': wp.to_lon,
                'cog': wp.cog,
                'sog': wp.sog,
                'tws': wp.tws,
                'twd': wp.twd,
                'twa': wp.twa,
                'distance': wp.distance,
                'distance_to_finish': wp.distance_to_finish,
            })
        
        bounds = parser.get_route_bounds()
        
        return jsonify({
            'name': name,
            'waypoints': waypoint_data,
            'start_time': parser.start_time.isoformat() if parser.start_time else None,
            'end_time': parser.end_time.isoformat() if parser.end_time else None,
            'total_distance': parser.total_distance,
            'bounds': {
                'lat_min': bounds[0],
                'lat_max': bounds[1],
                'lon_min': bounds[2],
                'lon_max': bounds[3],
            }
        })
        
    except Exception as e:
        logger.error(f"Failed to parse route {name}: {e}")
        return jsonify({'error': "Failed to parse route"}), 500


@app.route('/api/results')
def list_results() -> Dict[str, Any]:
    """List available result directories."""
    if results_dir is None or not results_dir.exists():
        return jsonify({'results': [], 'error': 'Results directory not configured'})
    
    result_dirs = []
    for d in results_dir.iterdir():
        if d.is_dir():
            # Check if it has a time_series.csv
            ts_file = d / 'time_series.csv'
            if ts_file.exists():
                result_dirs.append(d.name)
    
    return jsonify({'results': sorted(result_dirs)})


@app.route('/api/result/<name>')
def get_result(name: str) -> Dict[str, Any]:
    """
    Get experiment result track data.
    
    Args:
        name: Result directory name
        
    Returns:
        JSON with track points
    """
    if results_dir is None or not results_dir.exists():
        return jsonify({'error': 'Results directory not configured'}), 500
    
    # Sanitize the path to prevent directory traversal
    result_path = sanitize_path(results_dir, name)
    if result_path is None:
        return jsonify({'error': 'Invalid result name'}), 400
    
    ts_file = result_path / 'time_series.csv'
    
    if not ts_file.exists():
        return jsonify({'error': f'Result not found: {name}'}), 404
    
    try:
        import csv
        
        points = []
        start_time = None
        
        # Also try to load summary for metadata
        summary_file = result_path / 'summary.json'
        summary = None
        if summary_file.exists():
            import json
            with open(summary_file, 'r') as f:
                summary = json.load(f)
                if 'start_time' in summary:
                    start_time = summary['start_time']
        
        with open(ts_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    point = {
                        'timestamp': float(row['timestamp']),
                        'lat': float(row['latitude']),
                        'lon': float(row['longitude']),
                        'heading': float(row.get('heading', 0)),
                        'sog': float(row.get('sog', 0)),
                        'stw': float(row.get('stw', 0)),
                        'tws': float(row.get('tws', 0)),
                        'twd': float(row.get('twd', 0)),
                        'steering_mode': row.get('steering_mode', ''),
                    }
                    # Add optional fields if present
                    if 'target_heading' in row and row['target_heading']:
                        point['target_heading'] = float(row['target_heading'])
                    if 'twa' in row and row['twa']:
                        point['twa'] = float(row['twa'])
                    if 'awa' in row and row['awa']:
                        point['awa'] = float(row['awa'])
                    if 'aws' in row and row['aws']:
                        point['aws'] = float(row['aws'])
                    if 'heading_error' in row and row['heading_error']:
                        point['heading_error'] = float(row['heading_error'])
                    points.append(point)
                except (ValueError, KeyError) as e:
                    continue
        
        if not points:
            return jsonify({'error': 'No track points found'}), 400
        
        # Calculate bounds
        lats = [p['lat'] for p in points]
        lons = [p['lon'] for p in points]
        
        # Subsample if too many points (keep every Nth point)
        max_points = 5000
        if len(points) > max_points:
            step = len(points) // max_points
            sampled = points[::step]
            # Always include last point
            if points[-1] not in sampled:
                sampled.append(points[-1])
            points = sampled
        
        return jsonify({
            'name': name,
            'points': points,
            'start_time': start_time,
            'duration_seconds': points[-1]['timestamp'] if points else 0,
            'point_count': len(points),
            'bounds': {
                'lat_min': min(lats),
                'lat_max': max(lats),
                'lon_min': min(lons),
                'lon_max': max(lons),
            }
        })
        
    except Exception as e:
        logger.error(f"Failed to load result {name}: {e}")
        return jsonify({'error': 'Failed to load result'}), 500


def main():
    global routes_dir, results_dir
    
    parser = argparse.ArgumentParser(description='GRIB Visualization Server')
    parser.add_argument('--grib-dir', '-g', required=True,
                        help='Directory containing GRIB files')
    parser.add_argument('--routes-dir', '-r',
                        default='data/experiment1/route',
                        help='Directory containing route CSV files (default: data/experiment1/route)')
    parser.add_argument('--results-dir',
                        default='results',
                        help='Directory containing result folders (default: results)')
    parser.add_argument('--port', '-p', type=int, default=8080,
                        help='Port to run server on (default: 8080)')
    parser.add_argument('--host', default='127.0.0.1',
                        help='Host to bind to (default: 127.0.0.1)')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug mode')
    
    args = parser.parse_args()
    
    if not init_grib_reader(args.grib_dir):
        logger.error("Failed to initialize GRIB reader. Exiting.")
        sys.exit(1)
    
    # Set routes and results directories
    routes_dir = Path(args.routes_dir)
    if routes_dir.exists():
        logger.info(f"Routes directory: {routes_dir}")
    else:
        logger.warning(f"Routes directory not found: {routes_dir}")
    
    results_dir = Path(args.results_dir)
    if results_dir.exists():
        logger.info(f"Results directory: {results_dir}")
    else:
        logger.warning(f"Results directory not found: {results_dir}")
    
    logger.info(f"Starting server at http://{args.host}:{args.port}")
    app.run(host=args.host, port=args.port, debug=args.debug)


if __name__ == '__main__':
    main()
