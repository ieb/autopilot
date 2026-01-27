"""
Metrics Tracker
===============

Tracks performance metrics during passage simulation.
Computes ETA error, cross-track error, polar performance, and course accuracy.
"""

import json
import math
from dataclasses import dataclass, field, asdict
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Dict, List, Optional
import logging

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class TimeSeriesPoint:
    """A single point in the time series."""
    timestamp: float           # Elapsed time in seconds
    latitude: float
    longitude: float
    heading: float
    target_heading: float
    heading_error: float
    stw: float
    sog: float
    tws: float
    twd: float
    twa: float
    awa: float                 # Apparent wind angle (degrees)
    aws: float                 # Apparent wind speed (knots)
    cross_track_error: float   # nm
    polar_performance: float   # ratio (0-1.2+)
    rudder_angle: float
    steering_mode: str
    leg_index: int


@dataclass
class LegMetrics:
    """Metrics for a single leg of the route."""
    leg_index: int
    planned_start_time: datetime
    actual_start_time: datetime
    planned_duration_sec: float
    actual_duration_sec: float
    
    # ETA
    time_error_sec: float      # positive = late
    
    # Cross-track
    mean_xte: float            # Mean absolute XTE (nm)
    max_xte: float             # Maximum absolute XTE (nm)
    rms_xte: float             # RMS XTE (nm)
    
    # Polar performance
    mean_polar_perf: float     # Mean polar performance ratio
    min_polar_perf: float      # Minimum polar performance
    
    # Course accuracy
    mean_heading_error: float  # Mean absolute heading error (degrees)
    rms_heading_error: float   # RMS heading error (degrees)


@dataclass
class SummaryMetrics:
    """Summary metrics for the entire passage."""
    # Overall timing
    planned_duration_sec: float
    actual_duration_sec: float
    eta_error_sec: float       # positive = late
    eta_error_percent: float
    
    # Cross-track error
    mean_xte_nm: float
    max_xte_nm: float
    rms_xte_nm: float
    
    # Polar performance
    mean_polar_performance: float
    polar_performance_std: float
    
    # Course accuracy
    mean_heading_error_deg: float
    rms_heading_error_deg: float
    max_heading_error_deg: float
    
    # Route completion
    total_legs: int
    completed_legs: int
    total_distance_nm: float
    covered_distance_nm: float
    
    # Steering mode distribution
    time_in_awa_mode_sec: float
    time_in_twa_mode_sec: float
    time_in_compass_mode_sec: float
    time_in_motoring_sec: float


class MetricsTracker:
    """
    Tracks and computes performance metrics during simulation.
    
    Records time series data and computes summary statistics.
    """
    
    def __init__(self, planned_start_time: datetime, planned_end_time: datetime):
        """
        Initialize metrics tracker.
        
        Args:
            planned_start_time: Planned departure time
            planned_end_time: Planned arrival time
        """
        self.planned_start_time = planned_start_time
        self.planned_end_time = planned_end_time
        self.planned_duration = (planned_end_time - planned_start_time).total_seconds()
        
        # Time series data
        self.time_series: List[TimeSeriesPoint] = []
        
        # Leg tracking
        self.leg_metrics: List[LegMetrics] = []
        self._current_leg_index = 0
        self._leg_start_time: Optional[float] = None
        self._leg_start_actual: Optional[datetime] = None
        self._leg_points: List[TimeSeriesPoint] = []
        
        # Steering mode time tracking
        self._mode_times: Dict[str, float] = {
            'wind_awa': 0.0,
            'wind_twa': 0.0,
            'compass': 0.0,
            'motoring': 0.0,
        }
        self._last_sample_time = 0.0
        
        # Distance tracking
        self._total_distance_nm = 0.0
        self._last_lat: Optional[float] = None
        self._last_lon: Optional[float] = None
        
    def record(self, 
               elapsed_time: float,
               lat: float, lon: float,
               heading: float, target_heading: float,
               stw: float, sog: float,
               tws: float, twd: float,
               awa: float, aws: float,
               cross_track_error: float,
               polar_performance: float,
               rudder_angle: float,
               steering_mode: str,
               leg_index: int):
        """
        Record a single measurement point.
        
        Args:
            elapsed_time: Elapsed time in seconds
            lat, lon: Current position
            heading: Current heading (degrees)
            target_heading: Target heading (degrees)
            stw: Speed through water (knots)
            sog: Speed over ground (knots)
            tws: True wind speed (knots)
            twd: True wind direction (degrees)
            awa: Apparent wind angle (degrees)
            aws: Apparent wind speed (knots)
            cross_track_error: Cross-track error (nm)
            polar_performance: Polar performance ratio
            rudder_angle: Current rudder angle (degrees)
            steering_mode: Current steering mode
            leg_index: Current leg index
        """
        # Compute derived values
        twa = self._compute_twa(twd, heading)
        heading_error = self._angle_diff(heading, target_heading)
        
        point = TimeSeriesPoint(
            timestamp=elapsed_time,
            latitude=lat,
            longitude=lon,
            heading=heading,
            target_heading=target_heading,
            heading_error=heading_error,
            stw=stw,
            sog=sog,
            tws=tws,
            twd=twd,
            twa=twa,
            awa=awa,
            aws=aws,
            cross_track_error=cross_track_error,
            polar_performance=polar_performance,
            rudder_angle=rudder_angle,
            steering_mode=steering_mode,
            leg_index=leg_index,
        )
        
        self.time_series.append(point)
        
        # Track steering mode time
        dt = elapsed_time - self._last_sample_time
        if steering_mode in self._mode_times:
            self._mode_times[steering_mode] += dt
        self._last_sample_time = elapsed_time
        
        # Track distance
        if self._last_lat is not None:
            dist = self._compute_distance(self._last_lat, self._last_lon, lat, lon)
            self._total_distance_nm += dist
        self._last_lat = lat
        self._last_lon = lon
        
        # Track leg changes
        if leg_index != self._current_leg_index:
            self._finish_leg(elapsed_time)
            self._start_leg(leg_index, elapsed_time)
        
        self._leg_points.append(point)
        
    def _start_leg(self, leg_index: int, elapsed_time: float):
        """Start tracking a new leg."""
        self._current_leg_index = leg_index
        self._leg_start_time = elapsed_time
        self._leg_start_actual = self.planned_start_time + timedelta(seconds=elapsed_time)
        self._leg_points = []
        
    def _finish_leg(self, elapsed_time: float):
        """Finish tracking current leg and compute leg metrics."""
        if not self._leg_points or self._leg_start_time is None:
            return
            
        actual_duration = elapsed_time - self._leg_start_time
        
        # Compute leg statistics
        xte_values = [abs(p.cross_track_error) for p in self._leg_points]
        polar_values = [p.polar_performance for p in self._leg_points if p.polar_performance > 0]
        heading_errors = [abs(p.heading_error) for p in self._leg_points]
        
        leg = LegMetrics(
            leg_index=self._current_leg_index,
            planned_start_time=self.planned_start_time + timedelta(seconds=self._leg_start_time),
            actual_start_time=self._leg_start_actual or datetime.now(),
            planned_duration_sec=0.0,  # Would need planned data
            actual_duration_sec=actual_duration,
            time_error_sec=0.0,  # Would need planned data
            mean_xte=np.mean(xte_values) if xte_values else 0.0,
            max_xte=np.max(xte_values) if xte_values else 0.0,
            rms_xte=np.sqrt(np.mean(np.array(xte_values)**2)) if xte_values else 0.0,
            mean_polar_perf=np.mean(polar_values) if polar_values else 0.0,
            min_polar_perf=np.min(polar_values) if polar_values else 0.0,
            mean_heading_error=np.mean(heading_errors) if heading_errors else 0.0,
            rms_heading_error=np.sqrt(np.mean(np.array(heading_errors)**2)) if heading_errors else 0.0,
        )
        
        self.leg_metrics.append(leg)
        
    def compute_summary(self) -> SummaryMetrics:
        """
        Compute summary metrics from recorded data.
        
        Returns:
            SummaryMetrics with overall statistics
        """
        # Finish last leg
        if self._leg_points and self.time_series:
            self._finish_leg(self.time_series[-1].timestamp)
            
        # Get actual duration
        actual_duration = self.time_series[-1].timestamp if self.time_series else 0.0
        
        # Cross-track error statistics
        xte_values = [abs(p.cross_track_error) for p in self.time_series]
        
        # Polar performance statistics
        polar_values = [p.polar_performance for p in self.time_series 
                       if p.polar_performance > 0]
        
        # Heading error statistics
        heading_errors = [abs(p.heading_error) for p in self.time_series]
        
        return SummaryMetrics(
            planned_duration_sec=self.planned_duration,
            actual_duration_sec=actual_duration,
            eta_error_sec=actual_duration - self.planned_duration,
            eta_error_percent=((actual_duration - self.planned_duration) / 
                              self.planned_duration * 100) if self.planned_duration > 0 else 0.0,
            
            mean_xte_nm=np.mean(xte_values) if xte_values else 0.0,
            max_xte_nm=np.max(xte_values) if xte_values else 0.0,
            rms_xte_nm=np.sqrt(np.mean(np.array(xte_values)**2)) if xte_values else 0.0,
            
            mean_polar_performance=np.mean(polar_values) if polar_values else 0.0,
            polar_performance_std=np.std(polar_values) if polar_values else 0.0,
            
            mean_heading_error_deg=np.mean(heading_errors) if heading_errors else 0.0,
            rms_heading_error_deg=np.sqrt(np.mean(np.array(heading_errors)**2)) if heading_errors else 0.0,
            max_heading_error_deg=np.max(heading_errors) if heading_errors else 0.0,
            
            total_legs=len(self.leg_metrics),
            completed_legs=len(self.leg_metrics),
            total_distance_nm=self._total_distance_nm,
            covered_distance_nm=self._total_distance_nm,
            
            time_in_awa_mode_sec=self._mode_times.get('wind_awa', 0.0),
            time_in_twa_mode_sec=self._mode_times.get('wind_twa', 0.0),
            time_in_compass_mode_sec=self._mode_times.get('compass', 0.0),
            time_in_motoring_sec=self._mode_times.get('motoring', 0.0),
        )
        
    def save_results(self, output_dir: str):
        """
        Save metrics to files.
        
        Creates:
        - summary.json: Summary statistics
        - time_series.csv: Full time series data
        - report.md: Human-readable report
        
        Args:
            output_dir: Directory to save files to
        """
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        summary = self.compute_summary()
        
        # Save summary JSON
        summary_file = output_path / 'summary.json'
        with open(summary_file, 'w') as f:
            json.dump(asdict(summary), f, indent=2)
        logger.info(f"Saved summary to {summary_file}")
        
        # Save time series CSV
        ts_file = output_path / 'time_series.csv'
        self._save_time_series_csv(ts_file)
        logger.info(f"Saved time series to {ts_file}")
        
        # Save report
        report_file = output_path / 'report.md'
        self._save_report(report_file, summary)
        logger.info(f"Saved report to {report_file}")
        
    def _save_time_series_csv(self, filepath: Path):
        """Save time series to CSV."""
        import csv
        
        if not self.time_series:
            return
            
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            
            # Header
            writer.writerow([
                'timestamp', 'latitude', 'longitude', 'heading', 'target_heading',
                'heading_error', 'stw', 'sog', 'tws', 'twd', 'twa', 'awa', 'aws',
                'cross_track_error', 'polar_performance', 'rudder_angle',
                'steering_mode', 'leg_index'
            ])
            
            # Data - subsample for large datasets
            step = max(1, len(self.time_series) // 10000)
            for i in range(0, len(self.time_series), step):
                p = self.time_series[i]
                writer.writerow([
                    f"{p.timestamp:.1f}",
                    f"{p.latitude:.6f}",
                    f"{p.longitude:.6f}",
                    f"{p.heading:.1f}",
                    f"{p.target_heading:.1f}",
                    f"{p.heading_error:.1f}",
                    f"{p.stw:.2f}",
                    f"{p.sog:.2f}",
                    f"{p.tws:.1f}",
                    f"{p.twd:.1f}",
                    f"{p.twa:.1f}",
                    f"{p.awa:.1f}",
                    f"{p.aws:.1f}",
                    f"{p.cross_track_error:.4f}",
                    f"{p.polar_performance:.3f}",
                    f"{p.rudder_angle:.1f}",
                    p.steering_mode,
                    p.leg_index,
                ])
                
    def _save_report(self, filepath: Path, summary: SummaryMetrics):
        """Save human-readable report."""
        with open(filepath, 'w') as f:
            f.write("# Passage Simulation Report\n\n")
            
            f.write("## Summary\n\n")
            f.write(f"- **Planned Duration**: {self._format_duration(summary.planned_duration_sec)}\n")
            f.write(f"- **Actual Duration**: {self._format_duration(summary.actual_duration_sec)}\n")
            f.write(f"- **ETA Error**: {summary.eta_error_sec:.0f} seconds ({summary.eta_error_percent:.1f}%)\n")
            f.write(f"- **Total Distance**: {summary.total_distance_nm:.1f} nm\n")
            f.write(f"- **Legs Completed**: {summary.completed_legs}/{summary.total_legs}\n\n")
            
            f.write("## Performance Metrics\n\n")
            f.write("### Cross-Track Error\n\n")
            f.write(f"- Mean: {summary.mean_xte_nm:.4f} nm ({summary.mean_xte_nm * 1852:.0f} m)\n")
            f.write(f"- Max: {summary.max_xte_nm:.4f} nm ({summary.max_xte_nm * 1852:.0f} m)\n")
            f.write(f"- RMS: {summary.rms_xte_nm:.4f} nm\n\n")
            
            f.write("### Polar Performance\n\n")
            f.write(f"- Mean: {summary.mean_polar_performance:.1%}\n")
            f.write(f"- Std Dev: {summary.polar_performance_std:.1%}\n\n")
            
            f.write("### Course Accuracy\n\n")
            f.write(f"- Mean Heading Error: {summary.mean_heading_error_deg:.1f}°\n")
            f.write(f"- RMS Heading Error: {summary.rms_heading_error_deg:.1f}°\n")
            f.write(f"- Max Heading Error: {summary.max_heading_error_deg:.1f}°\n\n")
            
            f.write("## Steering Mode Distribution\n\n")
            total_time = (summary.time_in_awa_mode_sec + summary.time_in_twa_mode_sec +
                         summary.time_in_compass_mode_sec + summary.time_in_motoring_sec)
            if total_time > 0:
                f.write(f"- AWA Mode: {self._format_duration(summary.time_in_awa_mode_sec)} ")
                f.write(f"({summary.time_in_awa_mode_sec / total_time * 100:.1f}%)\n")
                f.write(f"- TWA Mode: {self._format_duration(summary.time_in_twa_mode_sec)} ")
                f.write(f"({summary.time_in_twa_mode_sec / total_time * 100:.1f}%)\n")
                f.write(f"- Compass Mode: {self._format_duration(summary.time_in_compass_mode_sec)} ")
                f.write(f"({summary.time_in_compass_mode_sec / total_time * 100:.1f}%)\n")
                f.write(f"- Motoring: {self._format_duration(summary.time_in_motoring_sec)} ")
                f.write(f"({summary.time_in_motoring_sec / total_time * 100:.1f}%)\n")
                
    def _format_duration(self, seconds: float) -> str:
        """Format duration in hours:minutes:seconds."""
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        secs = int(seconds % 60)
        return f"{hours:02d}:{minutes:02d}:{secs:02d}"
        
    def _compute_twa(self, twd: float, heading: float) -> float:
        """Compute true wind angle."""
        twa = twd - heading
        while twa > 180:
            twa -= 360
        while twa < -180:
            twa += 360
        return twa
        
    def _angle_diff(self, a: float, b: float) -> float:
        """Compute signed angle difference."""
        diff = a - b
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff
        
    def _compute_distance(self, lat1: float, lon1: float,
                         lat2: float, lon2: float) -> float:
        """Compute distance in nautical miles (Haversine)."""
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
        
    def print_summary(self):
        """Print summary to console."""
        summary = self.compute_summary()
        
        print("\n" + "=" * 60)
        print("PASSAGE SIMULATION RESULTS")
        print("=" * 60)
        
        print(f"\nTiming:")
        print(f"  Planned:  {self._format_duration(summary.planned_duration_sec)}")
        print(f"  Actual:   {self._format_duration(summary.actual_duration_sec)}")
        print(f"  Error:    {summary.eta_error_sec:.0f}s ({summary.eta_error_percent:+.1f}%)")
        
        print(f"\nCross-Track Error:")
        print(f"  Mean: {summary.mean_xte_nm:.4f} nm ({summary.mean_xte_nm * 1852:.0f} m)")
        print(f"  Max:  {summary.max_xte_nm:.4f} nm ({summary.max_xte_nm * 1852:.0f} m)")
        
        print(f"\nPolar Performance:")
        print(f"  Mean: {summary.mean_polar_performance:.1%} (± {summary.polar_performance_std:.1%})")
        
        print(f"\nCourse Accuracy:")
        print(f"  RMS Error: {summary.rms_heading_error_deg:.1f}°")
        print(f"  Max Error: {summary.max_heading_error_deg:.1f}°")
        
        print("\n" + "=" * 60)
