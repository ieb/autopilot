"""
GRIB Reader
===========

Reads GRIB weather files containing wind and wave data.
Supports bz2 compressed files and multiple resolution grids.
"""

import bz2
import logging
import math
import os
import tempfile
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# Disable cfgrib index file caching to avoid "older than GRIB file" warnings
# when decompressing bz2 files to temp directories
os.environ.setdefault('GRIB_INDEX_PATH', '')

# Optional imports for GRIB handling
try:
    import xarray as xr
    import cfgrib
    HAS_CFGRIB = True
    # Opt into new xarray defaults to silence FutureWarning from cfgrib
    xr.set_options(use_new_combine_kwarg_defaults=True)
except ImportError:
    HAS_CFGRIB = False
    logger.warning("cfgrib/xarray not available. GRIB reading disabled.")


@dataclass
class GribField:
    """A single GRIB field with spatial data."""
    name: str
    short_name: str
    units: str
    valid_time: datetime
    data: np.ndarray  # 2D array [lat, lon]
    lats: np.ndarray  # 1D array of latitudes
    lons: np.ndarray  # 1D array of longitudes
    
    @property
    def lat_min(self) -> float:
        return float(np.min(self.lats))
        
    @property
    def lat_max(self) -> float:
        return float(np.max(self.lats))
        
    @property
    def lon_min(self) -> float:
        return float(np.min(self.lons))
        
    @property
    def lon_max(self) -> float:
        return float(np.max(self.lons))
        
    @property
    def resolution(self) -> float:
        """Approximate grid resolution in degrees."""
        if len(self.lats) > 1:
            return abs(float(self.lats[1] - self.lats[0]))
        return 0.0


@dataclass
class WindData:
    """Wind data at a specific time."""
    valid_time: datetime
    u10: GribField  # U-component of 10m wind (m/s)
    v10: GribField  # V-component of 10m wind (m/s)
    
    def get_wind_at(self, lat: float, lon: float) -> Tuple[float, float]:
        """
        Get wind speed and direction at a point.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            
        Returns:
            Tuple of (TWS in knots, TWD in degrees)
        """
        u = self._interpolate(self.u10, lat, lon)
        v = self._interpolate(self.v10, lat, lon)
        
        # Convert m/s to knots
        tws = math.sqrt(u**2 + v**2) * 1.94384
        
        # Wind direction (meteorological convention: direction wind is FROM)
        twd = math.degrees(math.atan2(-u, -v)) % 360
        
        return (tws, twd)
        
    def _interpolate(self, field: GribField, lat: float, lon: float) -> float:
        """Bilinear interpolation of field value at a point."""
        return bilinear_interpolate(field.data, field.lats, field.lons, lat, lon)


@dataclass
class WaveData:
    """Wave data at a specific time."""
    valid_time: datetime
    swh: Optional[GribField] = None    # Significant wave height (m)
    mwp: Optional[GribField] = None    # Mean wave period (s)
    mwd: Optional[GribField] = None    # Mean wave direction (degrees)
    
    def get_waves_at(self, lat: float, lon: float) -> Tuple[float, float, float]:
        """
        Get wave parameters at a point.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            
        Returns:
            Tuple of (Hs in meters, Tp in seconds, Dir in degrees)
        """
        hs = 0.0
        tp = 5.0  # Default period
        direction = 0.0
        
        if self.swh is not None:
            hs = bilinear_interpolate(self.swh.data, self.swh.lats, self.swh.lons, lat, lon)
        if self.mwp is not None:
            tp = bilinear_interpolate(self.mwp.data, self.mwp.lats, self.mwp.lons, lat, lon)
        if self.mwd is not None:
            direction = bilinear_interpolate(self.mwd.data, self.mwd.lats, self.mwd.lons, lat, lon)
            
        return (hs, tp, direction)


def bilinear_interpolate(data: np.ndarray, lats: np.ndarray, lons: np.ndarray,
                         lat: float, lon: float) -> float:
    """
    Bilinear interpolation on a regular lat/lon grid.
    
    Args:
        data: 2D array [lat, lon]
        lats: 1D array of latitudes (can be increasing or decreasing)
        lons: 1D array of longitudes
        lat: Query latitude
        lon: Query longitude
        
    Returns:
        Interpolated value
    """
    # Handle latitude direction (may be decreasing)
    if lats[0] > lats[-1]:
        lats = lats[::-1]
        data = data[::-1, :]
        
    # Find bounding indices
    lat_idx = np.searchsorted(lats, lat)
    lon_idx = np.searchsorted(lons, lon)
    
    # Clamp to valid range
    lat_idx = max(1, min(lat_idx, len(lats) - 1))
    lon_idx = max(1, min(lon_idx, len(lons) - 1))
    
    # Get corner indices
    i0, i1 = lat_idx - 1, lat_idx
    j0, j1 = lon_idx - 1, lon_idx
    
    # Get corner values
    lat0, lat1 = lats[i0], lats[i1]
    lon0, lon1 = lons[j0], lons[j1]
    
    # Compute interpolation weights
    if lat1 != lat0:
        lat_frac = (lat - lat0) / (lat1 - lat0)
    else:
        lat_frac = 0.0
    if lon1 != lon0:
        lon_frac = (lon - lon0) / (lon1 - lon0)
    else:
        lon_frac = 0.0
        
    # Clamp fractions
    lat_frac = max(0.0, min(1.0, lat_frac))
    lon_frac = max(0.0, min(1.0, lon_frac))
    
    # Get corner values (handle NaN)
    v00 = float(data[i0, j0])
    v01 = float(data[i0, j1])
    v10 = float(data[i1, j0])
    v11 = float(data[i1, j1])
    
    # Replace NaN with nearest valid
    if np.isnan(v00):
        v00 = 0.0
    if np.isnan(v01):
        v01 = 0.0
    if np.isnan(v10):
        v10 = 0.0
    if np.isnan(v11):
        v11 = 0.0
    
    # Bilinear interpolation
    v0 = v00 + (v01 - v00) * lon_frac
    v1 = v10 + (v11 - v10) * lon_frac
    
    return v0 + (v1 - v0) * lat_frac


class GribReader:
    """
    Reader for GRIB weather files.
    
    Handles:
    - bz2 compressed files
    - Multiple GRIB files (different resolutions/areas)
    - Wind and wave parameters
    """
    
    # Common parameter short names for wind
    WIND_U_NAMES = ['u10', '10u', 'U10', 'UGRD']
    WIND_V_NAMES = ['v10', '10v', 'V10', 'VGRD']
    
    # Common parameter short names for waves
    WAVE_HEIGHT_NAMES = ['swh', 'hs', 'HTSGW', 'shww']
    WAVE_PERIOD_NAMES = ['mwp', 'pp1d', 'PERPW', 'mpww']
    WAVE_DIR_NAMES = ['mwd', 'DIRPW', 'mdww']
    
    def __init__(self, grib_dir: Optional[str] = None):
        """
        Initialize GRIB reader.
        
        Args:
            grib_dir: Directory containing GRIB files
        """
        self.grib_dir = Path(grib_dir) if grib_dir else None
        self.wind_data: Dict[datetime, List[WindData]] = {}
        self.wave_data: Dict[datetime, List[WaveData]] = {}
        self._temp_files: List[Path] = []
        
    def load_directory(self, grib_dir: Optional[str] = None) -> int:
        """
        Load all GRIB files from a directory.
        
        Args:
            grib_dir: Directory to load from (uses init dir if not specified)
            
        Returns:
            Number of files loaded
        """
        if not HAS_CFGRIB:
            raise ImportError("cfgrib is required to read GRIB files. "
                            "Install with: pip install cfgrib xarray eccodes")
            
        grib_path = Path(grib_dir) if grib_dir else self.grib_dir
        if not grib_path:
            raise ValueError("No GRIB directory specified")
            
        if not grib_path.exists():
            raise FileNotFoundError(f"GRIB directory not found: {grib_path}")
            
        files_loaded = 0
        
        # Find all GRIB files (including compressed)
        for pattern in ['*.grb', '*.grib', '*.grb2', '*.grb.bz2', '*.grib.bz2']:
            for grib_file in grib_path.glob(pattern):
                try:
                    self.load_file(grib_file)
                    files_loaded += 1
                except Exception as e:
                    logger.warning(f"Failed to load {grib_file.name}: {e}")
                    
        logger.info(f"Loaded {files_loaded} GRIB files with "
                   f"{len(self.wind_data)} wind times, {len(self.wave_data)} wave times")
        
        return files_loaded
        
    def load_file(self, filepath: Path):
        """
        Load a single GRIB file.
        
        Args:
            filepath: Path to GRIB file (may be bz2 compressed)
        """
        if not HAS_CFGRIB:
            raise ImportError("cfgrib is required")
            
        filepath = Path(filepath)
        
        # Handle bz2 compression
        if filepath.suffix == '.bz2':
            actual_file = self._decompress_bz2(filepath)
        else:
            actual_file = filepath
            
        try:
            # Open with xarray/cfgrib
            # Try different filter combinations for wind and wave data
            self._load_wind_data(actual_file)
            self._load_wave_data(actual_file)
            
        finally:
            # Clean up temp file if we decompressed
            if filepath.suffix == '.bz2' and actual_file in self._temp_files:
                pass  # Keep for now, clean up in __del__
                
    def _decompress_bz2(self, filepath: Path) -> Path:
        """Decompress a bz2 file to a temp location."""
        # Create temp file
        temp_file = Path(tempfile.gettempdir()) / filepath.stem
        
        if not temp_file.exists():
            logger.debug(f"Decompressing {filepath.name}...")
            with bz2.open(filepath, 'rb') as f_in:
                with open(temp_file, 'wb') as f_out:
                    f_out.write(f_in.read())
            self._temp_files.append(temp_file)
        
        # Delete any stale index files to avoid "older than GRIB file" warnings
        # cfgrib creates .idx files with hash suffixes
        for idx_file in Path(tempfile.gettempdir()).glob(f"{filepath.stem}.*.idx"):
            try:
                idx_file.unlink()
                logger.debug(f"Removed stale index file: {idx_file.name}")
            except OSError:
                pass
            
        return temp_file
        
    def _load_wind_data(self, filepath: Path):
        """Load wind data from a GRIB file."""
        found_wind = False
        try:
            # Try opening with different filter keys
            datasets = cfgrib.open_datasets(str(filepath))
            
            for ds in datasets:
                # Look for wind components
                u_var = None
                v_var = None
                
                for var in ds.data_vars:
                    var_lower = var.lower()
                    if any(name.lower() in var_lower for name in self.WIND_U_NAMES):
                        u_var = var
                    elif any(name.lower() in var_lower for name in self.WIND_V_NAMES):
                        v_var = var
                        
                if u_var and v_var:
                    self._extract_wind_fields(ds, u_var, v_var)
                    found_wind = True
                    
        except Exception as e:
            logger.debug(f"cfgrib wind loading failed: {e}")
        
        # Fallback: use eccodes directly for unrecognized GRIB1 parameters
        # indicatorOfParameter 33 = U wind, 34 = V wind
        if not found_wind:
            self._load_wind_data_eccodes(filepath)
    
    def _load_wind_data_eccodes(self, filepath: Path):
        """
        Load wind data using eccodes directly.
        
        Handles GRIB files where cfgrib can't identify parameters.
        Uses GRIB1 indicatorOfParameter: 33=U wind, 34=V wind
        """
        try:
            import eccodes
        except ImportError:
            logger.debug("eccodes not available for fallback wind loading")
            return
        
        # Group messages by valid_time
        u_messages = {}  # valid_time -> (data, lats, lons)
        v_messages = {}
        
        try:
            with open(filepath, 'rb') as f:
                while True:
                    gid = eccodes.codes_grib_new_from_file(f)
                    if gid is None:
                        break
                    
                    try:
                        level_type = eccodes.codes_get(gid, 'typeOfLevel')
                        level = eccodes.codes_get(gid, 'level')
                        
                        # Only 10m wind data
                        if level_type != 'heightAboveGround' or level != 10:
                            continue
                        
                        # Get parameter indicator (GRIB1: 33=U, 34=V)
                        try:
                            param = eccodes.codes_get(gid, 'indicatorOfParameter')
                        except:
                            continue
                        
                        if param not in (33, 34):
                            continue
                        
                        # Get valid time
                        data_date = eccodes.codes_get(gid, 'dataDate')
                        data_time = eccodes.codes_get(gid, 'dataTime')
                        step = eccodes.codes_get(gid, 'step')
                        
                        # Parse date/time
                        year = data_date // 10000
                        month = (data_date % 10000) // 100
                        day = data_date % 100
                        hour = data_time // 100
                        minute = data_time % 100
                        
                        from datetime import timedelta
                        base_time = datetime(year, month, day, hour, minute)
                        valid_time = base_time + timedelta(hours=step)
                        
                        # Get grid data
                        ni = eccodes.codes_get(gid, 'Ni')
                        nj = eccodes.codes_get(gid, 'Nj')
                        lat_first = eccodes.codes_get(gid, 'latitudeOfFirstGridPointInDegrees')
                        lat_last = eccodes.codes_get(gid, 'latitudeOfLastGridPointInDegrees')
                        lon_first = eccodes.codes_get(gid, 'longitudeOfFirstGridPointInDegrees')
                        lon_last = eccodes.codes_get(gid, 'longitudeOfLastGridPointInDegrees')
                        
                        # Handle longitude wraparound (359 -> 2 means crossing 0)
                        if lon_first > 180:
                            lon_first -= 360
                        if lon_last > 180:
                            lon_last -= 360
                        
                        lats = np.linspace(lat_first, lat_last, nj)
                        lons = np.linspace(lon_first, lon_last, ni)
                        
                        # Get values
                        values = eccodes.codes_get_values(gid)
                        data = values.reshape((nj, ni))
                        
                        if param == 33:
                            u_messages[valid_time] = (data, lats, lons)
                        else:
                            v_messages[valid_time] = (data, lats, lons)
                            
                    finally:
                        eccodes.codes_release(gid)
            
            # Create WindData for times with both U and V
            for valid_time in u_messages:
                if valid_time not in v_messages:
                    continue
                
                u_data, lats, lons = u_messages[valid_time]
                v_data, _, _ = v_messages[valid_time]
                
                u_field = GribField(
                    name='u10', short_name='u10', units='m/s',
                    valid_time=valid_time, data=u_data, lats=lats, lons=lons,
                )
                v_field = GribField(
                    name='v10', short_name='v10', units='m/s',
                    valid_time=valid_time, data=v_data, lats=lats, lons=lons,
                )
                
                wind = WindData(valid_time=valid_time, u10=u_field, v10=v_field)
                
                if valid_time not in self.wind_data:
                    self.wind_data[valid_time] = []
                self.wind_data[valid_time].append(wind)
            
            if u_messages:
                logger.debug(f"Loaded {len(u_messages)} wind times from {filepath.name} via eccodes")
                
        except Exception as e:
            logger.debug(f"eccodes wind loading failed: {e}")
            
    def _load_wave_data(self, filepath: Path):
        """Load wave data from a GRIB file."""
        try:
            datasets = cfgrib.open_datasets(str(filepath))
            
            for ds in datasets:
                # Look for wave parameters
                swh_var = None
                mwp_var = None
                mwd_var = None
                
                for var in ds.data_vars:
                    var_lower = var.lower()
                    if any(name.lower() in var_lower for name in self.WAVE_HEIGHT_NAMES):
                        swh_var = var
                    elif any(name.lower() in var_lower for name in self.WAVE_PERIOD_NAMES):
                        mwp_var = var
                    elif any(name.lower() in var_lower for name in self.WAVE_DIR_NAMES):
                        mwd_var = var
                        
                if swh_var or mwp_var:
                    self._extract_wave_fields(ds, swh_var, mwp_var, mwd_var)
                    
        except Exception as e:
            logger.debug(f"No wave data in {filepath.name}: {e}")
            
    def _extract_wind_fields(self, ds: 'xr.Dataset', u_var: str, v_var: str):
        """Extract wind fields from dataset."""
        u_da = ds[u_var]
        v_da = ds[v_var]
        
        # Get coordinates
        lats = u_da.latitude.values if 'latitude' in u_da.coords else u_da.lat.values
        lons = u_da.longitude.values if 'longitude' in u_da.coords else u_da.lon.values
        
        # Get valid times
        if 'valid_time' in u_da.coords:
            times = u_da.valid_time.values
        elif 'time' in u_da.coords:
            times = u_da.time.values
        else:
            times = [np.datetime64('now')]
            
        # Handle single time vs multiple times
        if np.ndim(times) == 0:
            times = [times]
            
        for i, t in enumerate(times):
            valid_time = self._numpy_to_datetime(t)
            
            # Extract data for this time
            if len(times) > 1:
                u_data = u_da.isel(time=i).values if 'time' in u_da.dims else u_da.values
                v_data = v_da.isel(time=i).values if 'time' in v_da.dims else v_da.values
            else:
                u_data = u_da.values
                v_data = v_da.values
                
            # Remove extra dimensions
            u_data = np.squeeze(u_data)
            v_data = np.squeeze(v_data)
            
            # Create GribFields
            u_field = GribField(
                name=u_var,
                short_name='u10',
                units='m/s',
                valid_time=valid_time,
                data=u_data,
                lats=lats,
                lons=lons,
            )
            v_field = GribField(
                name=v_var,
                short_name='v10',
                units='m/s',
                valid_time=valid_time,
                data=v_data,
                lats=lats,
                lons=lons,
            )
            
            wind = WindData(valid_time=valid_time, u10=u_field, v10=v_field)
            
            if valid_time not in self.wind_data:
                self.wind_data[valid_time] = []
            self.wind_data[valid_time].append(wind)
            
    def _extract_wave_fields(self, ds: 'xr.Dataset', 
                            swh_var: Optional[str],
                            mwp_var: Optional[str],
                            mwd_var: Optional[str]):
        """Extract wave fields from dataset."""
        # Get reference variable for coords
        ref_var = swh_var or mwp_var or mwd_var
        if not ref_var:
            return
            
        ref_da = ds[ref_var]
        
        # Get coordinates
        lats = ref_da.latitude.values if 'latitude' in ref_da.coords else ref_da.lat.values
        lons = ref_da.longitude.values if 'longitude' in ref_da.coords else ref_da.lon.values
        
        # Get valid times
        if 'valid_time' in ref_da.coords:
            times = ref_da.valid_time.values
        elif 'time' in ref_da.coords:
            times = ref_da.time.values
        else:
            times = [np.datetime64('now')]
            
        if np.ndim(times) == 0:
            times = [times]
            
        for i, t in enumerate(times):
            valid_time = self._numpy_to_datetime(t)
            
            # Extract fields
            swh_field = None
            mwp_field = None
            mwd_field = None
            
            if swh_var:
                da = ds[swh_var]
                data = da.isel(time=i).values if 'time' in da.dims and len(times) > 1 else da.values
                swh_field = GribField(
                    name=swh_var, short_name='swh', units='m',
                    valid_time=valid_time, data=np.squeeze(data),
                    lats=lats, lons=lons,
                )
                
            if mwp_var:
                da = ds[mwp_var]
                data = da.isel(time=i).values if 'time' in da.dims and len(times) > 1 else da.values
                mwp_field = GribField(
                    name=mwp_var, short_name='mwp', units='s',
                    valid_time=valid_time, data=np.squeeze(data),
                    lats=lats, lons=lons,
                )
                
            if mwd_var:
                da = ds[mwd_var]
                data = da.isel(time=i).values if 'time' in da.dims and len(times) > 1 else da.values
                mwd_field = GribField(
                    name=mwd_var, short_name='mwd', units='degrees',
                    valid_time=valid_time, data=np.squeeze(data),
                    lats=lats, lons=lons,
                )
                
            wave = WaveData(
                valid_time=valid_time,
                swh=swh_field,
                mwp=mwp_field,
                mwd=mwd_field,
            )
            
            if valid_time not in self.wave_data:
                self.wave_data[valid_time] = []
            self.wave_data[valid_time].append(wave)
            
    def _numpy_to_datetime(self, np_time) -> datetime:
        """Convert numpy datetime64 to Python datetime."""
        if isinstance(np_time, datetime):
            return np_time
        if isinstance(np_time, np.datetime64):
            # Convert to timestamp then to datetime
            ts = (np_time - np.datetime64('1970-01-01T00:00:00')) / np.timedelta64(1, 's')
            return datetime.utcfromtimestamp(ts)
        return datetime.now()
        
    def get_wind_times(self) -> List[datetime]:
        """Get list of available wind forecast times."""
        return sorted(self.wind_data.keys())
        
    def get_wave_times(self) -> List[datetime]:
        """Get list of available wave forecast times."""
        return sorted(self.wave_data.keys())
        
    def get_wind_at(self, lat: float, lon: float, time: datetime) -> Tuple[float, float]:
        """
        Get wind at a specific location and time.
        
        Uses the best available resolution grid and interpolates in time.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            time: Query time
            
        Returns:
            Tuple of (TWS in knots, TWD in degrees)
        """
        # Find bracketing times
        times = self.get_wind_times()
        if not times:
            logger.warning("No wind data available")
            return (15.0, 180.0)  # Default wind
            
        # Find best time match
        t0, t1, frac = self._find_time_bracket(times, time)
        
        # Get wind at both times
        tws0, twd0 = self._get_wind_at_time(lat, lon, t0)
        
        if t0 == t1 or frac == 0:
            return (tws0, twd0)
            
        tws1, twd1 = self._get_wind_at_time(lat, lon, t1)
        
        # Interpolate
        tws = tws0 + (tws1 - tws0) * frac
        
        # Handle direction wraparound
        diff = twd1 - twd0
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        twd = (twd0 + diff * frac) % 360
        
        return (tws, twd)
        
    def _get_wind_at_time(self, lat: float, lon: float, time: datetime) -> Tuple[float, float]:
        """Get wind at a specific time from available grids."""
        if time not in self.wind_data:
            return (15.0, 180.0)
            
        # Use highest resolution grid that contains the point
        best_wind = None
        best_res = float('inf')
        
        for wind in self.wind_data[time]:
            if (wind.u10.lat_min <= lat <= wind.u10.lat_max and
                wind.u10.lon_min <= lon <= wind.u10.lon_max):
                res = wind.u10.resolution
                if res < best_res:
                    best_res = res
                    best_wind = wind
                    
        if best_wind:
            return best_wind.get_wind_at(lat, lon)
            
        # Fallback to any available grid
        if self.wind_data[time]:
            return self.wind_data[time][0].get_wind_at(lat, lon)
            
        return (15.0, 180.0)
        
    def get_waves_at(self, lat: float, lon: float, time: datetime) -> Tuple[float, float, float]:
        """
        Get wave data at a specific location and time.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            time: Query time
            
        Returns:
            Tuple of (Hs in meters, Tp in seconds, Dir in degrees)
        """
        times = self.get_wave_times()
        if not times:
            return (1.0, 6.0, 0.0)  # Default waves
            
        t0, t1, frac = self._find_time_bracket(times, time)
        
        hs0, tp0, dir0 = self._get_waves_at_time(lat, lon, t0)
        
        if t0 == t1 or frac == 0:
            return (hs0, tp0, dir0)
            
        hs1, tp1, dir1 = self._get_waves_at_time(lat, lon, t1)
        
        # Interpolate
        hs = hs0 + (hs1 - hs0) * frac
        tp = tp0 + (tp1 - tp0) * frac
        
        # Handle direction wraparound
        diff = dir1 - dir0
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        direction = (dir0 + diff * frac) % 360
        
        return (hs, tp, direction)
        
    def _get_waves_at_time(self, lat: float, lon: float, time: datetime) -> Tuple[float, float, float]:
        """Get waves at a specific time from available grids."""
        if time not in self.wave_data:
            return (1.0, 6.0, 0.0)
            
        # Use highest resolution grid
        best_wave = None
        best_res = float('inf')
        
        for wave in self.wave_data[time]:
            ref_field = wave.swh or wave.mwp
            if ref_field is None:
                continue
            if (ref_field.lat_min <= lat <= ref_field.lat_max and
                ref_field.lon_min <= lon <= ref_field.lon_max):
                res = ref_field.resolution
                if res < best_res:
                    best_res = res
                    best_wave = wave
                    
        if best_wave:
            return best_wave.get_waves_at(lat, lon)
            
        if self.wave_data[time]:
            return self.wave_data[time][0].get_waves_at(lat, lon)
            
        return (1.0, 6.0, 0.0)
        
    def _find_time_bracket(self, times: List[datetime], query: datetime
                          ) -> Tuple[datetime, datetime, float]:
        """Find the two times that bracket the query time."""
        if not times:
            return (datetime.now(), datetime.now(), 0.0)
            
        if query <= times[0]:
            return (times[0], times[0], 0.0)
        if query >= times[-1]:
            return (times[-1], times[-1], 0.0)
            
        for i in range(len(times) - 1):
            if times[i] <= query <= times[i + 1]:
                dt = (times[i + 1] - times[i]).total_seconds()
                frac = (query - times[i]).total_seconds() / dt if dt > 0 else 0.0
                return (times[i], times[i + 1], frac)
                
        return (times[-1], times[-1], 0.0)
        
    def cleanup(self):
        """Clean up temporary files."""
        for temp_file in self._temp_files:
            try:
                if temp_file.exists():
                    temp_file.unlink()
            except Exception:
                pass
        self._temp_files.clear()
        
    def __del__(self):
        """Cleanup on deletion."""
        self.cleanup()
