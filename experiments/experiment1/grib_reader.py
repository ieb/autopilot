"""
GRIB Reader
===========

Reads GRIB weather files containing wind and wave data.
Supports bz2 compressed files and multiple resolution grids.
"""

import bz2
import contextlib
import json
import logging
import math
import os
import sys
import tempfile
import threading
from dataclasses import dataclass
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

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
    # Silence cfgrib's noisy stderr output for truncated/corrupt files
    logging.getLogger('cfgrib').setLevel(logging.CRITICAL)
    logging.getLogger('cfgrib.messages').setLevel(logging.CRITICAL)
except ImportError:
    HAS_CFGRIB = False
    logger.warning("cfgrib/xarray not available. GRIB reading disabled.")


@contextlib.contextmanager
def _suppress_stderr():
    """Suppress stderr from both Python and the eccodes C library.

    eccodes writes ERROR messages (e.g. "Can't create file") directly to
    the C-level file descriptor, bypassing Python's sys.stderr.  We dup
    the fd to /dev/null for the duration of the block.
    """
    stderr_fd = sys.stderr.fileno()
    saved_fd = os.dup(stderr_fd)
    try:
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, stderr_fd)
        os.close(devnull)
        yield
    finally:
        os.dup2(saved_fd, stderr_fd)
        os.close(saved_fd)


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
    source_file: Optional[str] = None
    
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
    source_file: Optional[str] = None
    
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
    
    METADATA_FILENAME = 'grib_metadata.json'
    METADATA_VERSION = 1

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
        self._metadata: Dict[str, Any] = {'version': self.METADATA_VERSION, 'files': {}}
        self._lazy: bool = False
        self._loaded_files: Set[str] = set()
        self._lock = threading.Lock()
        
    def load_directory(self, grib_dir: Optional[str] = None,
                       lazy: bool = False) -> int:
        """
        Load GRIB files from a directory.

        Args:
            grib_dir: Directory to load from (uses init dir if not specified)
            lazy: If True, only scan metadata — files are loaded on demand
                  via ensure_time_loaded(). If False, load all files eagerly.

        Returns:
            Number of files found (lazy) or loaded (eager)
        """
        if not HAS_CFGRIB:
            raise ImportError("cfgrib is required to read GRIB files. "
                            "Install with: pip install cfgrib xarray eccodes")

        grib_path = Path(grib_dir) if grib_dir else self.grib_dir
        if not grib_path:
            raise ValueError("No GRIB directory specified")

        if not grib_path.exists():
            raise FileNotFoundError(f"GRIB directory not found: {grib_path}")

        self.grib_dir = grib_path
        self._lazy = lazy

        # Build / load the metadata cache
        self._metadata = self._load_metadata_cache(grib_path)

        if lazy:
            n_files = len(self._metadata['files'])
            logger.info(f"Lazy mode: indexed {n_files} GRIB files from metadata cache")
            return n_files

        # Eager mode — load all files
        files_loaded = 0
        for pattern in ['*.grb', '*.grib', '*.grb2', '*.grb.bz2', '*.grib.bz2']:
            for grib_file in grib_path.glob(pattern):
                try:
                    self.load_file(grib_file)
                    self._loaded_files.add(grib_file.name)
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
        source_file = filepath.name

        # Handle bz2 compression
        if filepath.suffix == '.bz2':
            actual_file = self._decompress_bz2(filepath)
        else:
            actual_file = filepath

        try:
            # Open with xarray/cfgrib
            # Try different filter combinations for wind and wave data
            self._load_wind_data(actual_file, source_file=source_file)
            self._load_wave_data(actual_file, source_file=source_file)

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
        
    def _load_wind_data(self, filepath: Path, source_file: str = None):
        """Load wind data from a GRIB file."""
        found_wind = False
        try:
            with _suppress_stderr():
                datasets = cfgrib.open_datasets(str(filepath))

            for ds in datasets:
                u_var = None
                v_var = None

                for var in ds.data_vars:
                    var_lower = var.lower()
                    if any(name.lower() in var_lower for name in self.WIND_U_NAMES):
                        u_var = var
                    elif any(name.lower() in var_lower for name in self.WIND_V_NAMES):
                        v_var = var

                if u_var and v_var:
                    self._extract_wind_fields(ds, u_var, v_var, source_file=source_file)
                    found_wind = True

        except Exception as e:
            logger.debug(f"cfgrib wind loading failed: {e}")

        # Fallback: use eccodes directly for unrecognized GRIB1 parameters
        # indicatorOfParameter 33 = U wind, 34 = V wind
        if not found_wind:
            self._load_wind_data_eccodes(filepath, source_file=source_file)
    
    def _load_wind_data_eccodes(self, filepath: Path, source_file: str = None):
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
                
                wind = WindData(valid_time=valid_time, u10=u_field, v10=v_field,
                               source_file=source_file)

                if valid_time not in self.wind_data:
                    self.wind_data[valid_time] = []
                self.wind_data[valid_time].append(wind)

            if u_messages:
                logger.debug(f"Loaded {len(u_messages)} wind times from {filepath.name} via eccodes")
                
        except Exception as e:
            logger.debug(f"eccodes wind loading failed: {e}")
            
    def _load_wave_data(self, filepath: Path, source_file: str = None):
        """Load wave data from a GRIB file."""
        try:
            with _suppress_stderr():
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
                    self._extract_wave_fields(ds, swh_var, mwp_var, mwd_var,
                                              source_file=source_file)

        except Exception as e:
            logger.debug(f"No wave data in {filepath.name}: {e}")
            
    def _extract_wind_fields(self, ds: 'xr.Dataset', u_var: str, v_var: str,
                              source_file: str = None):
        """Extract wind fields from dataset."""
        u_da = ds[u_var]
        v_da = ds[v_var]

        # Get coordinates
        lats = u_da.latitude.values if 'latitude' in u_da.coords else u_da.lat.values
        lons = u_da.longitude.values if 'longitude' in u_da.coords else u_da.lon.values
        # Normalise longitudes from 0-360 to -180/180
        lons = np.where(lons > 180, lons - 360, lons)
        
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
            
        # Find the actual time dimension name (may be 'time', 'valid_time', 'step', etc.)
        time_dim = None
        for dim in u_da.dims:
            if dim not in ('latitude', 'longitude', 'lat', 'lon', 'x', 'y'):
                time_dim = dim
                break

        for i, t in enumerate(times):
            valid_time = self._numpy_to_datetime(t)

            # Extract data for this time
            if len(times) > 1 and time_dim and time_dim in u_da.dims:
                u_data = u_da.isel({time_dim: i}).values
                v_data = v_da.isel({time_dim: i}).values
            else:
                u_data = u_da.values
                v_data = v_da.values

            # Remove extra dimensions
            u_data = np.squeeze(u_data)
            v_data = np.squeeze(v_data)

            # Validate shape — must be 2D (lat, lon)
            if u_data.ndim != 2 or v_data.ndim != 2:
                logger.warning(
                    f"Skipping wind field at {valid_time}: "
                    f"expected 2D data, got u={u_data.shape} v={v_data.shape}"
                )
                continue
            
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
            
            wind = WindData(valid_time=valid_time, u10=u_field, v10=v_field,
                           source_file=source_file)

            if valid_time not in self.wind_data:
                self.wind_data[valid_time] = []
            self.wind_data[valid_time].append(wind)
            
    def _extract_wave_fields(self, ds: 'xr.Dataset',
                            swh_var: Optional[str],
                            mwp_var: Optional[str],
                            mwd_var: Optional[str],
                            source_file: str = None):
        """Extract wave fields from dataset."""
        # Get reference variable for coords
        ref_var = swh_var or mwp_var or mwd_var
        if not ref_var:
            return

        ref_da = ds[ref_var]

        # Get coordinates
        lats = ref_da.latitude.values if 'latitude' in ref_da.coords else ref_da.lat.values
        lons = ref_da.longitude.values if 'longitude' in ref_da.coords else ref_da.lon.values
        # Normalise longitudes from 0-360 to -180/180
        lons = np.where(lons > 180, lons - 360, lons)
        
        # Get valid times
        if 'valid_time' in ref_da.coords:
            times = ref_da.valid_time.values
        elif 'time' in ref_da.coords:
            times = ref_da.time.values
        else:
            times = [np.datetime64('now')]
            
        if np.ndim(times) == 0:
            times = [times]
            
        # Find the actual time dimension name
        time_dim = None
        for dim in ref_da.dims:
            if dim not in ('latitude', 'longitude', 'lat', 'lon', 'x', 'y'):
                time_dim = dim
                break

        for i, t in enumerate(times):
            valid_time = self._numpy_to_datetime(t)

            # Extract fields
            swh_field = None
            mwp_field = None
            mwd_field = None

            if swh_var:
                da = ds[swh_var]
                data = self._extract_time_slice(da, i, len(times), time_dim)
                if data.ndim == 2:
                    swh_field = GribField(
                        name=swh_var, short_name='swh', units='m',
                        valid_time=valid_time, data=data,
                        lats=lats, lons=lons,
                    )

            if mwp_var:
                da = ds[mwp_var]
                data = self._extract_time_slice(da, i, len(times), time_dim)
                if data.ndim == 2:
                    mwp_field = GribField(
                        name=mwp_var, short_name='mwp', units='s',
                        valid_time=valid_time, data=data,
                        lats=lats, lons=lons,
                    )

            if mwd_var:
                da = ds[mwd_var]
                data = self._extract_time_slice(da, i, len(times), time_dim)
                if data.ndim == 2:
                    mwd_field = GribField(
                        name=mwd_var, short_name='mwd', units='degrees',
                        valid_time=valid_time, data=data,
                        lats=lats, lons=lons,
                    )
                
            wave = WaveData(
                valid_time=valid_time,
                swh=swh_field,
                mwp=mwp_field,
                mwd=mwd_field,
                source_file=source_file,
            )

            if valid_time not in self.wave_data:
                self.wave_data[valid_time] = []
            self.wave_data[valid_time].append(wave)
            
    @staticmethod
    def _extract_time_slice(da, time_index: int, n_times: int,
                            time_dim: Optional[str]) -> np.ndarray:
        """Extract a 2D (lat, lon) slice from a data array.

        Handles datasets where the time dimension may be named 'time',
        'valid_time', 'step', etc.
        """
        if n_times > 1 and time_dim and time_dim in da.dims:
            data = da.isel({time_dim: time_index}).values
        else:
            data = da.values
        return np.squeeze(data)

    # ------------------------------------------------------------------
    # Metadata scanning and lazy loading
    # ------------------------------------------------------------------

    def _load_metadata_cache(self, grib_dir: Path) -> Dict[str, Any]:
        """Load or build the metadata cache for *grib_dir*."""
        cache_path = grib_dir / self.METADATA_FILENAME
        metadata: Dict[str, Any] = {'version': self.METADATA_VERSION, 'files': {}}

        # Try loading existing cache
        if cache_path.exists():
            try:
                with open(cache_path, 'r') as f:
                    cached = json.load(f)
                if cached.get('version') == self.METADATA_VERSION:
                    metadata = cached
            except Exception as e:
                logger.warning(f"Failed to read metadata cache: {e}")

        # Enumerate current GRIB files on disk
        grib_files: List[Path] = []
        for pattern in ['*.grb', '*.grib', '*.grb2', '*.grb.bz2', '*.grib.bz2']:
            grib_files.extend(grib_dir.glob(pattern))

        current_names = {f.name for f in grib_files}
        file_map = {f.name: f for f in grib_files}

        # Remove entries for files that no longer exist
        stale_keys = [k for k in metadata['files'] if k not in current_names]
        for k in stale_keys:
            del metadata['files'][k]

        # Validate / rescan each file
        changed = bool(stale_keys)
        for name, filepath in file_map.items():
            stat = filepath.stat()
            entry = metadata['files'].get(name)
            if (entry
                    and entry.get('size_bytes') == stat.st_size
                    and entry.get('mtime') == stat.st_mtime):
                continue  # still fresh
            # Scan this file
            try:
                new_entry = self._scan_file_metadata(filepath)
                new_entry['size_bytes'] = stat.st_size
                new_entry['mtime'] = stat.st_mtime
                metadata['files'][name] = new_entry
                changed = True
                logger.debug(f"Scanned metadata for {name}")
            except Exception as e:
                logger.warning(f"Failed to scan metadata for {name}: {e}")

        if changed:
            self._save_metadata_cache(grib_dir, metadata)

        return metadata

    @staticmethod
    def _save_metadata_cache(grib_dir: Path, metadata: Dict[str, Any]):
        """Persist *metadata* to grib_metadata.json."""
        cache_path = grib_dir / GribReader.METADATA_FILENAME
        try:
            with open(cache_path, 'w') as f:
                json.dump(metadata, f, indent=2, default=str)
            logger.debug(f"Saved metadata cache to {cache_path}")
        except Exception as e:
            logger.warning(f"Failed to save metadata cache: {e}")

    @staticmethod
    def _scan_file_metadata(filepath: Path) -> Dict[str, Any]:
        """Scan GRIB headers with eccodes — no data values loaded."""
        try:
            import eccodes
        except ImportError:
            raise ImportError("eccodes is required for metadata scanning")

        params: set = set()
        valid_times: list = []
        lat_min = float('inf')
        lat_max = float('-inf')
        lon_min = float('inf')
        lon_max = float('-inf')
        resolution_deg: Optional[float] = None
        has_wind = False
        has_waves = False

        # Wind & wave parameter short names (lowercase)
        wind_names = {'u10', '10u', 'ugrd', 'v10', '10v', 'vgrd'}
        wave_names = {'swh', 'hs', 'htsgw', 'shww', 'mwp', 'pp1d', 'perpw',
                      'mpww', 'mwd', 'dirpw', 'mdww', 'shts', 'mpts', 'mdts', 'mp2'}

        # Decompress bz2 if needed (to a temp file)
        if filepath.suffix == '.bz2':
            tmp = Path(tempfile.gettempdir()) / filepath.stem
            if not tmp.exists():
                with bz2.open(filepath, 'rb') as fin:
                    with open(tmp, 'wb') as fout:
                        fout.write(fin.read())
            scan_path = tmp
        else:
            scan_path = filepath

        with open(scan_path, 'rb') as f:
            while True:
                gid = eccodes.codes_grib_new_from_file(f)
                if gid is None:
                    break
                try:
                    # Parameter short name
                    try:
                        sn = eccodes.codes_get(gid, 'shortName')
                        params.add(sn)
                        if sn.lower() in wind_names:
                            has_wind = True
                        if sn.lower() in wave_names:
                            has_waves = True
                        # Fallback for GRIB1 files where shortName is
                        # 'unknown': check indicatorOfParameter at 10m
                        # (33 = U wind, 34 = V wind)
                        if sn == 'unknown' and not has_wind:
                            try:
                                indicator = eccodes.codes_get(gid, 'indicatorOfParameter')
                                level_type = eccodes.codes_get(gid, 'typeOfLevel')
                                level = eccodes.codes_get(gid, 'level')
                                if indicator in (33, 34) and level_type == 'heightAboveGround' and level == 10:
                                    has_wind = True
                            except Exception:
                                pass
                    except Exception:
                        pass

                    # Valid time
                    try:
                        data_date = eccodes.codes_get(gid, 'dataDate')
                        data_time = eccodes.codes_get(gid, 'dataTime')
                        step = eccodes.codes_get(gid, 'step')
                        year = data_date // 10000
                        month = (data_date % 10000) // 100
                        day = data_date % 100
                        hour = data_time // 100
                        minute = data_time % 100
                        base = datetime(year, month, day, hour, minute)
                        vt = base + timedelta(hours=step)
                        vt_iso = vt.isoformat()
                        if vt_iso not in valid_times:
                            valid_times.append(vt_iso)
                    except Exception:
                        pass

                    # Spatial bounds
                    try:
                        lf = eccodes.codes_get(gid, 'latitudeOfFirstGridPointInDegrees')
                        ll = eccodes.codes_get(gid, 'latitudeOfLastGridPointInDegrees')
                        of = eccodes.codes_get(gid, 'longitudeOfFirstGridPointInDegrees')
                        ol = eccodes.codes_get(gid, 'longitudeOfLastGridPointInDegrees')
                        if of > 180:
                            of -= 360
                        if ol > 180:
                            ol -= 360
                        lat_min = min(lat_min, lf, ll)
                        lat_max = max(lat_max, lf, ll)
                        lon_min = min(lon_min, of, ol)
                        lon_max = max(lon_max, of, ol)
                    except Exception:
                        pass

                    # Resolution
                    if resolution_deg is None:
                        try:
                            ni = eccodes.codes_get(gid, 'Ni')
                            nj = eccodes.codes_get(gid, 'Nj')
                            if ni > 1 and nj > 1:
                                resolution_deg = abs(lat_max - lat_min) / (nj - 1)
                        except Exception:
                            pass
                finally:
                    eccodes.codes_release(gid)

        time_min = min(valid_times) if valid_times else None
        time_max = max(valid_times) if valid_times else None

        return {
            'parameters': sorted(params),
            'valid_times': sorted(valid_times),
            'time_min': time_min,
            'time_max': time_max,
            'lat_min': lat_min if lat_min != float('inf') else None,
            'lat_max': lat_max if lat_max != float('-inf') else None,
            'lon_min': lon_min if lon_min != float('inf') else None,
            'lon_max': lon_max if lon_max != float('-inf') else None,
            'resolution_deg': resolution_deg,
            'has_wind': has_wind,
            'has_waves': has_waves,
        }

    # ------------------------------------------------------------------
    # Lazy-load helpers
    # ------------------------------------------------------------------

    def get_file_metadata(self) -> Dict[str, Any]:
        """Return per-file metadata for the frontend file selector."""
        files = {}
        for name, entry in self._metadata.get('files', {}).items():
            files[name] = {
                'valid_times': entry.get('valid_times', []),
                'time_min': entry.get('time_min'),
                'time_max': entry.get('time_max'),
                'has_wind': entry.get('has_wind', False),
                'has_waves': entry.get('has_waves', False),
                'resolution_deg': entry.get('resolution_deg'),
            }
        return files

    def ensure_time_loaded(self, query_time: datetime, filename: str = None):
        """Ensure data covering *query_time* is loaded.

        In eager mode this is a no-op.  In lazy mode it finds matching
        files from metadata and loads them on demand.

        If *filename* is given, only that specific file is loaded.
        """
        if not self._lazy:
            return

        if filename:
            self._ensure_file_loaded(filename)
        else:
            filenames = self._files_for_time(query_time)
            for name in filenames:
                self._ensure_file_loaded(name)

    def _files_for_time(self, query_time: datetime) -> List[str]:
        """Return filenames from metadata whose time range covers *query_time*."""
        iso = query_time.isoformat()
        result = []
        for name, entry in self._metadata.get('files', {}).items():
            t_min = entry.get('time_min')
            t_max = entry.get('time_max')
            if t_min is None or t_max is None:
                # Unknown time range — include to be safe
                result.append(name)
                continue
            if t_min <= iso <= t_max:
                result.append(name)
        return result

    def _ensure_file_loaded(self, filename: str):
        """Decompress + load a single file if not already loaded."""
        if filename in self._loaded_files:
            return

        with self._lock:
            # Double-check after acquiring lock
            if filename in self._loaded_files:
                return

            filepath = self.grib_dir / filename
            if not filepath.exists():
                logger.warning(f"GRIB file not found: {filepath}")
                return

            try:
                logger.info(f"Lazy-loading {filename}")
                self.load_file(filepath)
                self._loaded_files.add(filename)
            except Exception as e:
                logger.warning(f"Failed to lazy-load {filename}: {e}")

    def get_bounds_from_metadata(self) -> Dict[str, Any]:
        """Return spatial/temporal bounds from metadata without loading data."""
        wind_times: List[str] = []
        wave_times: List[str] = []
        lat_min = float('inf')
        lat_max = float('-inf')
        lon_min = float('inf')
        lon_max = float('-inf')
        has_wind = False
        has_waves = False

        for entry in self._metadata.get('files', {}).values():
            times = entry.get('valid_times', [])
            if entry.get('lat_min') is not None:
                lat_min = min(lat_min, entry['lat_min'])
            if entry.get('lat_max') is not None:
                lat_max = max(lat_max, entry['lat_max'])
            if entry.get('lon_min') is not None:
                lon_min = min(lon_min, entry['lon_min'])
            if entry.get('lon_max') is not None:
                lon_max = max(lon_max, entry['lon_max'])
            if entry.get('has_wind'):
                has_wind = True
                wind_times.extend(times)
            if entry.get('has_waves'):
                has_waves = True
                wave_times.extend(times)

        # Separate time lists so the frontend only offers wave overlay
        # for times that actually have wave data (and likewise for wind).
        # The time slider uses the union so it covers the full range.
        unique_wind = sorted(set(wind_times))
        unique_wave = sorted(set(wave_times))

        return {
            'wind_times': unique_wind,
            'wave_times': unique_wave,
            'bounds': {
                'lat_min': lat_min if lat_min != float('inf') else None,
                'lat_max': lat_max if lat_max != float('-inf') else None,
                'lon_min': lon_min if lon_min != float('inf') else None,
                'lon_max': lon_max if lon_max != float('-inf') else None,
            },
            'has_wind': has_wind,
            'has_waves': has_waves,
        }

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
