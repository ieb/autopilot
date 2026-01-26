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

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__, static_folder='static')

# Global GRIB reader instance
grib_reader: Optional[GribReader] = None


def init_grib_reader(grib_dir: str) -> bool:
    """Initialize the GRIB reader with data from the specified directory."""
    global grib_reader
    
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


def main():
    parser = argparse.ArgumentParser(description='GRIB Visualization Server')
    parser.add_argument('--grib-dir', '-g', required=True,
                        help='Directory containing GRIB files')
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
    
    logger.info(f"Starting server at http://{args.host}:{args.port}")
    app.run(host=args.host, port=args.port, debug=args.debug)


if __name__ == '__main__':
    main()
