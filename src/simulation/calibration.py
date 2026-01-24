"""
Calibration Module
==================

Extracts statistics from real log data to calibrate simulator parameters.
"""

import math
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple
import logging

from ..training.data_loader import CANLogParser, LoggedFrame
from .wind_model import WindConfig
from .wave_model import WaveConfig
from .helm_controller import HelmConfig
from .yacht_dynamics import YachtConfig

logger = logging.getLogger(__name__)


@dataclass
class CalibrationStats:
    """Statistics extracted from real log data."""
    # Sample counts
    total_frames: int = 0
    duration_seconds: float = 0.0
    
    # Heading statistics
    heading_mean: float = 0.0
    heading_std: float = 0.0
    heading_rate_mean: float = 0.0
    heading_rate_std: float = 0.0
    
    # Wind statistics
    awa_mean: float = 0.0
    awa_std: float = 0.0
    aws_mean: float = 0.0
    aws_std: float = 0.0
    tws_estimated_mean: float = 0.0
    tws_estimated_std: float = 0.0
    
    # Speed statistics
    stw_mean: float = 0.0
    stw_std: float = 0.0
    sog_mean: float = 0.0
    sog_std: float = 0.0
    
    # Rudder statistics
    rudder_mean: float = 0.0
    rudder_std: float = 0.0
    rudder_rate_mean: float = 0.0
    rudder_rate_std: float = 0.0
    
    # Heel/pitch statistics
    roll_mean: float = 0.0
    roll_std: float = 0.0
    pitch_mean: float = 0.0
    pitch_std: float = 0.0
    
    # Derived
    rudder_activity: float = 0.0  # How often rudder is being moved
    helm_responsiveness: float = 0.0  # Correlation between error and rudder


@dataclass 
class CalibratedConfig:
    """Calibrated configuration from real data."""
    wind_config: WindConfig = field(default_factory=WindConfig)
    wave_config: WaveConfig = field(default_factory=WaveConfig)
    helm_config: HelmConfig = field(default_factory=HelmConfig)
    yacht_config: YachtConfig = field(default_factory=YachtConfig)
    stats: CalibrationStats = field(default_factory=CalibrationStats)


def calibrate_from_logs(log_path: str, 
                        recursive: bool = True) -> CalibratedConfig:
    """
    Analyze real log files and generate calibrated simulation config.
    
    Args:
        log_path: Path to log file or directory
        recursive: If directory, search recursively
        
    Returns:
        CalibratedConfig with parameters matched to real data
    """
    path = Path(log_path)
    
    if path.is_file():
        frames = _parse_log_file(str(path))
    elif path.is_dir():
        frames = _parse_log_directory(str(path), recursive)
    else:
        logger.warning(f"Path does not exist: {path}")
        return CalibratedConfig()
        
    if not frames:
        logger.warning("No frames parsed from logs")
        return CalibratedConfig()
        
    # Compute statistics
    stats = _compute_statistics(frames)
    
    # Generate calibrated configs
    wind_config = _calibrate_wind(stats)
    wave_config = _calibrate_waves(stats)
    helm_config = _calibrate_helm(stats)
    yacht_config = _calibrate_yacht(stats)
    
    logger.info(f"Calibrated from {stats.total_frames} frames, "
                f"{stats.duration_seconds/3600:.2f} hours")
    
    return CalibratedConfig(
        wind_config=wind_config,
        wave_config=wave_config,
        helm_config=helm_config,
        yacht_config=yacht_config,
        stats=stats,
    )


def _parse_log_file(filepath: str) -> List[LoggedFrame]:
    """Parse a single log file."""
    parser = CANLogParser()
    try:
        return list(parser.parse_file(filepath))
    except Exception as e:
        logger.error(f"Failed to parse {filepath}: {e}")
        return []


def _parse_log_directory(directory: str, recursive: bool) -> List[LoggedFrame]:
    """Parse all log files in a directory."""
    parser = CANLogParser()
    all_frames = []
    
    path = Path(directory)
    pattern = '**/*.log' if recursive else '*.log'
    
    for log_file in path.glob(pattern):
        if log_file.is_file():
            try:
                frames = list(parser.parse_file(str(log_file)))
                all_frames.extend(frames)
                logger.info(f"Parsed {len(frames)} frames from {log_file.name}")
            except Exception as e:
                logger.error(f"Failed to parse {log_file}: {e}")
                
    return all_frames


def _compute_statistics(frames: List[LoggedFrame]) -> CalibrationStats:
    """Compute statistics from parsed frames."""
    if not frames:
        return CalibrationStats()
        
    stats = CalibrationStats()
    stats.total_frames = len(frames)
    
    # Duration
    if len(frames) > 1:
        stats.duration_seconds = frames[-1].timestamp - frames[0].timestamp
        
    # Collect values
    headings = [f.heading for f in frames if f.heading != 0]
    heading_rates = [f.yaw_rate for f in frames if f.yaw_rate != 0]
    awas = [f.awa for f in frames if f.awa != 0]
    awss = [f.aws for f in frames if f.aws != 0]
    stws = [f.stw for f in frames if f.stw != 0]
    sogs = [f.sog for f in frames if f.sog != 0]
    rudders = [f.rudder_angle for f in frames]
    rolls = [f.roll for f in frames if f.roll != 0]
    pitches = [f.pitch for f in frames if f.pitch != 0]
    
    # Compute rudder rates
    rudder_rates = []
    for i in range(1, len(frames)):
        dt = frames[i].timestamp - frames[i-1].timestamp
        if dt > 0:
            rate = (frames[i].rudder_angle - frames[i-1].rudder_angle) / dt
            rudder_rates.append(rate)
            
    # Estimate TWS from AWA, AWS, STW
    tws_estimates = []
    for f in frames:
        if f.aws > 0 and f.stw > 0:
            tws = _compute_tws(f.awa, f.aws, f.stw)
            if 0 < tws < 60:
                tws_estimates.append(tws)
                
    # Heading statistics (circular)
    if headings:
        stats.heading_mean = _circular_mean(headings)
        stats.heading_std = _circular_std(headings)
        
    if heading_rates:
        stats.heading_rate_mean = statistics.mean(heading_rates)
        stats.heading_rate_std = statistics.stdev(heading_rates) if len(heading_rates) > 1 else 0
        
    # Wind statistics
    if awas:
        stats.awa_mean = statistics.mean(awas)
        stats.awa_std = statistics.stdev(awas) if len(awas) > 1 else 0
        
    if awss:
        stats.aws_mean = statistics.mean(awss)
        stats.aws_std = statistics.stdev(awss) if len(awss) > 1 else 0
        
    if tws_estimates:
        stats.tws_estimated_mean = statistics.mean(tws_estimates)
        stats.tws_estimated_std = statistics.stdev(tws_estimates) if len(tws_estimates) > 1 else 0
        
    # Speed statistics
    if stws:
        stats.stw_mean = statistics.mean(stws)
        stats.stw_std = statistics.stdev(stws) if len(stws) > 1 else 0
        
    if sogs:
        stats.sog_mean = statistics.mean(sogs)
        stats.sog_std = statistics.stdev(sogs) if len(sogs) > 1 else 0
        
    # Rudder statistics
    if rudders:
        stats.rudder_mean = statistics.mean(rudders)
        stats.rudder_std = statistics.stdev(rudders) if len(rudders) > 1 else 0
        
    if rudder_rates:
        stats.rudder_rate_mean = statistics.mean(rudder_rates)
        stats.rudder_rate_std = statistics.stdev(rudder_rates) if len(rudder_rates) > 1 else 0
        
    # Roll/pitch statistics
    if rolls:
        stats.roll_mean = statistics.mean(rolls)
        stats.roll_std = statistics.stdev(rolls) if len(rolls) > 1 else 0
        
    if pitches:
        stats.pitch_mean = statistics.mean(pitches)
        stats.pitch_std = statistics.stdev(pitches) if len(pitches) > 1 else 0
        
    # Derived metrics
    if rudder_rates:
        # Activity = how often rudder is moving more than 0.5 deg/s
        active_count = sum(1 for r in rudder_rates if abs(r) > 0.5)
        stats.rudder_activity = active_count / len(rudder_rates)
        
    return stats


def _calibrate_wind(stats: CalibrationStats) -> WindConfig:
    """Generate wind config from statistics."""
    config = WindConfig()
    
    # Set TWS range based on observed data
    if stats.tws_estimated_mean > 0:
        mean = stats.tws_estimated_mean
        std = stats.tws_estimated_std
        
        config.base_tws_min = max(0, mean - 2 * std)
        config.base_tws_max = mean + 2 * std
        
    # Estimate shift rate from AWA variation
    if stats.awa_std > 0:
        # AWA variation indicates wind shifts
        config.shift_rate = min(2.0, stats.awa_std / 10.0)
        
    # Gust probability from AWS variation
    if stats.aws_std > 0 and stats.aws_mean > 0:
        cv = stats.aws_std / stats.aws_mean  # Coefficient of variation
        config.gust_probability = min(0.05, cv * 0.1)
        
    logger.info(f"Calibrated wind: TWS {config.base_tws_min:.1f}-{config.base_tws_max:.1f} kts, "
                f"shift rate {config.shift_rate:.2f} deg/min")
    
    return config


def _calibrate_waves(stats: CalibrationStats) -> WaveConfig:
    """Generate wave config from statistics."""
    config = WaveConfig()
    
    # Estimate wave amplitude from roll/pitch variation
    if stats.roll_std > 0:
        # Roll std includes wave motion
        config.swell_amplitude_min = max(0.5, stats.roll_std - 2)
        config.swell_amplitude_max = stats.roll_std + 3
        
    # Chop depends on wind
    if stats.tws_estimated_mean > 15:
        config.chop_enabled = True
        config.chop_amplitude_factor = 0.4
    elif stats.tws_estimated_mean < 8:
        config.chop_enabled = False
        
    logger.info(f"Calibrated waves: amplitude {config.swell_amplitude_min:.1f}-"
                f"{config.swell_amplitude_max:.1f} deg")
    
    return config


def _calibrate_helm(stats: CalibrationStats) -> HelmConfig:
    """Generate helm config from statistics."""
    config = HelmConfig()
    
    # Estimate noise from rudder variation
    if stats.rudder_std > 0:
        config.noise_std = min(1.0, stats.rudder_std * 0.1)
        
    # Estimate skill from rudder activity and rate
    if stats.rudder_activity > 0:
        # High activity with low rate = skilled
        # Low activity with high rate = less skilled
        if stats.rudder_rate_std > 0:
            smoothness = 1.0 / (1.0 + stats.rudder_rate_std / 5.0)
            config.skill_level = 0.5 + 0.5 * smoothness
            
    logger.info(f"Calibrated helm: skill {config.skill_level:.2f}, "
                f"noise std {config.noise_std:.2f}")
    
    return config


def _calibrate_yacht(stats: CalibrationStats) -> YachtConfig:
    """Generate yacht config from statistics."""
    config = YachtConfig()
    
    # Estimate rudder effectiveness from heading rate vs rudder
    # This would need more sophisticated analysis
    
    # Estimate heel stiffness from roll vs wind
    if stats.roll_std > 0 and stats.aws_mean > 0:
        # Higher roll per unit wind = less stiff
        roll_per_wind = stats.roll_std / (stats.aws_mean ** 2)
        config.heel_stiffness = roll_per_wind * 10
        
    return config


def _compute_tws(awa: float, aws: float, stw: float) -> float:
    """Compute true wind speed from apparent wind."""
    awa_rad = math.radians(awa)
    tw_x = aws * math.cos(awa_rad) - stw
    tw_y = aws * math.sin(awa_rad)
    return math.sqrt(tw_x**2 + tw_y**2)


def _circular_mean(angles: List[float]) -> float:
    """Compute circular mean of angles in degrees."""
    if not angles:
        return 0.0
    sin_sum = sum(math.sin(math.radians(a)) for a in angles)
    cos_sum = sum(math.cos(math.radians(a)) for a in angles)
    return math.degrees(math.atan2(sin_sum, cos_sum)) % 360


def _circular_std(angles: List[float]) -> float:
    """Compute circular standard deviation in degrees."""
    if len(angles) < 2:
        return 0.0
    sin_sum = sum(math.sin(math.radians(a)) for a in angles)
    cos_sum = sum(math.cos(math.radians(a)) for a in angles)
    r = math.sqrt(sin_sum**2 + cos_sum**2) / len(angles)
    if r >= 1.0:
        return 0.0
    return math.degrees(math.sqrt(-2 * math.log(r)))


def print_calibration_report(config: CalibratedConfig) -> None:
    """Print a human-readable calibration report."""
    stats = config.stats
    
    print("\n" + "=" * 60)
    print("CALIBRATION REPORT")
    print("=" * 60)
    
    print(f"\nData Summary:")
    print(f"  Total frames:  {stats.total_frames:,}")
    print(f"  Duration:      {stats.duration_seconds/3600:.2f} hours")
    
    print(f"\nWind Conditions:")
    print(f"  AWA:           {stats.awa_mean:.1f}° ± {stats.awa_std:.1f}°")
    print(f"  AWS:           {stats.aws_mean:.1f} kts ± {stats.aws_std:.1f}")
    print(f"  TWS (est):     {stats.tws_estimated_mean:.1f} kts ± {stats.tws_estimated_std:.1f}")
    
    print(f"\nBoat Performance:")
    print(f"  STW:           {stats.stw_mean:.1f} kts ± {stats.stw_std:.1f}")
    print(f"  SOG:           {stats.sog_mean:.1f} kts ± {stats.sog_std:.1f}")
    print(f"  Heel:          {stats.roll_mean:.1f}° ± {stats.roll_std:.1f}°")
    
    print(f"\nHelm Behavior:")
    print(f"  Rudder:        {stats.rudder_mean:.1f}° ± {stats.rudder_std:.1f}°")
    print(f"  Rudder rate:   {stats.rudder_rate_mean:.2f} deg/s ± {stats.rudder_rate_std:.2f}")
    print(f"  Activity:      {stats.rudder_activity*100:.1f}%")
    
    print(f"\nCalibrated Parameters:")
    wc = config.wind_config
    print(f"  Wind TWS:      {wc.base_tws_min:.1f} - {wc.base_tws_max:.1f} kts")
    print(f"  Shift rate:    {wc.shift_rate:.2f} deg/min")
    print(f"  Gust prob:     {wc.gust_probability:.3f}/s")
    
    hc = config.helm_config
    print(f"  Helm skill:    {hc.skill_level:.2f}")
    print(f"  Helm noise:    {hc.noise_std:.2f}°")
    
    print("=" * 60)
