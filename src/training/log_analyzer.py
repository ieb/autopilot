"""
Log Analyzer Module
====================

Analyzes CAN log files to detect boat operation modes (anchor/motoring/sailing)
and steering targets (heading/AWA/TWA), generating metadata files for training.
"""

import argparse
import json
import math
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import List, Optional, Dict, Any, Tuple
import logging

from .data_loader import CANLogParser, LoggedFrame

logger = logging.getLogger(__name__)


class OperationMode(Enum):
    """Boat operation mode."""
    ANCHOR = "anchor"
    MOTORING = "motoring"
    SAILING = "sailing"
    UNKNOWN = "unknown"


class SteeringMode(Enum):
    """Steering target mode."""
    HEADING = "heading"
    AWA = "awa"
    TWA = "twa"
    NONE = "none"


@dataclass
class AnalysisConfig:
    """Configuration for log analysis."""
    # Operation mode detection thresholds
    anchor_sog_threshold: float = 0.5  # knots - below this considered stationary
    anchor_position_radius: float = 50.0  # meters - position wander radius
    motoring_min_rpm: float = 100.0  # RPM threshold for engine running
    motoring_max_stw: float = 7.0  # knots - max speed under engine
    motoring_low_wind_threshold: float = 6.0  # TWS below this likely motoring
    motoring_close_wind_threshold: float = 20.0  # AWA below this can't sail
    sailing_min_roll_offset: float = 5.0  # degrees - min heel angle for sailing
    
    # Steering mode detection thresholds
    heading_stability_threshold: float = 5.0  # degrees - heading variation tolerance
    awa_stability_threshold: float = 5.0  # degrees - AWA variation tolerance
    twa_stability_threshold: float = 8.0  # degrees - TWA variation tolerance
    twa_downwind_threshold: float = 130.0  # degrees - above this likely TWA mode
    
    # Time windows
    min_segment_duration: float = 60.0  # seconds - minimum segment length
    analysis_window: float = 60.0  # seconds - rolling analysis window
    sample_interval: float = 1.0  # seconds - analysis sample rate


# Required input features for training and their source fields
REQUIRED_FEATURES = {
    "heading": "heading",
    "yaw_rate": "yaw_rate",
    "roll": "roll",
    "pitch": "pitch",
    "awa": "awa",
    "aws": "aws",
    "stw": "stw",
    "sog": "sog",
    "cog": "cog",
    "rudder_angle": "rudder_angle",
}

# Optional but useful features
OPTIONAL_FEATURES = {
    "engine_rpm": "engine_rpm",
    "latitude": "latitude",
    "longitude": "longitude",
    "pilot_heading": "pilot_heading",
}


@dataclass
class FeatureCoverage:
    """Coverage statistics for a feature."""
    name: str
    present_count: int = 0
    total_count: int = 0
    
    @property
    def coverage_pct(self) -> float:
        """Percentage of frames with valid data."""
        if self.total_count == 0:
            return 0.0
        return 100.0 * self.present_count / self.total_count
    
    @property
    def is_available(self) -> bool:
        """Feature is considered available if > 50% coverage."""
        return self.coverage_pct > 50.0
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "present_count": self.present_count,
            "total_count": self.total_count,
            "coverage_pct": round(self.coverage_pct, 1),
            "available": self.is_available,
        }


@dataclass
class Segment:
    """A time segment with detected modes."""
    start_time: float
    end_time: float
    operation_mode: str
    steering_mode: str
    target_value: float
    confidence: float
    notes: str = ""
    frame_count: int = 0
    feature_coverage: Dict[str, FeatureCoverage] = field(default_factory=dict)
    
    def duration(self) -> float:
        """Return segment duration in seconds."""
        return self.end_time - self.start_time
    
    def duration_hours(self) -> float:
        """Return segment duration in hours."""
        return self.duration() / 3600.0
    
    def missing_features(self) -> List[str]:
        """Return list of required features that are missing or have low coverage."""
        missing = []
        for name in REQUIRED_FEATURES:
            if name in self.feature_coverage:
                if not self.feature_coverage[name].is_available:
                    missing.append(name)
            else:
                missing.append(name)
        return missing
    
    def is_usable_for_training(self) -> bool:
        """Segment is usable if operation mode is valid and required features present."""
        if self.operation_mode in ('anchor', 'unknown'):
            return False
        if self.steering_mode == 'none':
            return False
        # Need at least heading, wind, and speed
        critical_features = ['heading', 'awa', 'aws', 'stw']
        for feat in critical_features:
            if feat in self.missing_features():
                return False
        return True
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "start_time": self.start_time,
            "end_time": self.end_time,
            "duration_sec": round(self.duration(), 1),
            "duration_hours": round(self.duration_hours(), 4),
            "operation_mode": self.operation_mode,
            "steering_mode": self.steering_mode,
            "target_value": round(self.target_value, 1),
            "confidence": round(self.confidence, 2),
            "notes": self.notes,
            "frame_count": self.frame_count,
            "usable_for_training": self.is_usable_for_training(),
            "missing_features": self.missing_features(),
            "feature_coverage": {k: v.to_dict() for k, v in self.feature_coverage.items()},
        }


@dataclass
class AnalysisResult:
    """Result of log file analysis."""
    source_file: str
    analyzed_at: str
    total_duration_sec: float
    total_frame_count: int = 0
    segments: List[Segment] = field(default_factory=list)
    summary: Dict[str, Any] = field(default_factory=dict)
    overall_feature_coverage: Dict[str, FeatureCoverage] = field(default_factory=dict)
    
    def total_duration_hours(self) -> float:
        """Total duration in hours."""
        return self.total_duration_sec / 3600.0
    
    def usable_segments(self) -> List[Segment]:
        """Return segments usable for training."""
        return [s for s in self.segments if s.is_usable_for_training()]
    
    def usable_duration_sec(self) -> float:
        """Total usable duration in seconds."""
        return sum(s.duration() for s in self.usable_segments())
    
    def usable_duration_hours(self) -> float:
        """Total usable duration in hours."""
        return self.usable_duration_sec() / 3600.0
    
    def usable_frame_count(self) -> int:
        """Total usable frames for training."""
        return sum(s.frame_count for s in self.usable_segments())
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "source_file": self.source_file,
            "analyzed_at": self.analyzed_at,
            "total_duration_sec": round(self.total_duration_sec, 1),
            "total_duration_hours": round(self.total_duration_hours(), 4),
            "total_frame_count": self.total_frame_count,
            "usable_duration_sec": round(self.usable_duration_sec(), 1),
            "usable_duration_hours": round(self.usable_duration_hours(), 4),
            "usable_frame_count": self.usable_frame_count(),
            "segments": [s.to_dict() for s in self.segments],
            "summary": self.summary,
            "overall_feature_coverage": {k: v.to_dict() for k, v in self.overall_feature_coverage.items()},
        }


class OperationDetector:
    """Detects boat operation mode from sensor data."""
    
    def __init__(self, config: AnalysisConfig):
        self.config = config
        
    def detect(self, frames: List[LoggedFrame]) -> OperationMode:
        """
        Detect operation mode from a window of frames.
        
        Returns the most likely operation mode based on sensor data.
        """
        if not frames:
            return OperationMode.UNKNOWN
            
        # Calculate statistics over the window
        avg_sog = self._mean([f.sog for f in frames])
        avg_rpm = self._mean([f.engine_rpm for f in frames])
        avg_roll = self._mean([f.roll for f in frames])
        abs_avg_roll = abs(avg_roll)
        
        # Compute true wind speed
        tws_values = []
        for f in frames:
            tws = self._compute_tws(f.awa, f.aws, f.stw)
            tws_values.append(tws)
        avg_tws = self._mean(tws_values)
        avg_awa = self._mean([abs(f.awa) for f in frames])
        
        # Check for anchor
        if self._is_at_anchor(frames, avg_sog):
            return OperationMode.ANCHOR
            
        # Check for motoring
        if self._is_motoring(avg_rpm, avg_tws, avg_awa, avg_sog, abs_avg_roll):
            return OperationMode.MOTORING
            
        # Check for sailing
        if self._is_sailing(abs_avg_roll, avg_tws, avg_awa):
            return OperationMode.SAILING
            
        return OperationMode.UNKNOWN
        
    def _is_at_anchor(self, frames: List[LoggedFrame], avg_sog: float) -> bool:
        """Check if boat is at anchor."""
        # Low SOG
        if avg_sog > self.config.anchor_sog_threshold:
            return False
            
        # Check position wander - should be within small radius
        lats = [f.latitude for f in frames if f.latitude != 0]
        lons = [f.longitude for f in frames if f.longitude != 0]
        
        if lats and lons:
            lat_range = max(lats) - min(lats)
            lon_range = max(lons) - min(lons)
            # Rough conversion to meters (1 degree lat ~ 111km)
            lat_meters = lat_range * 111000
            lon_meters = lon_range * 111000 * math.cos(math.radians(self._mean(lats)))
            max_wander = max(lat_meters, lon_meters)
            
            if max_wander < self.config.anchor_position_radius:
                return True
                
        return False
        
    def _is_motoring(self, avg_rpm: float, avg_tws: float, avg_awa: float, 
                     avg_sog: float, abs_avg_roll: float) -> bool:
        """Check if boat is motoring."""
        # Engine RPM detected
        if avg_rpm > self.config.motoring_min_rpm:
            return True
            
        # Low wind - can't sail
        if avg_tws < self.config.motoring_low_wind_threshold:
            return True
            
        # Close to wind - can't sail
        if avg_awa < self.config.motoring_close_wind_threshold:
            return True
            
        # Exceeding engine max speed - actually sailing
        if avg_sog > self.config.motoring_max_stw:
            return False
            
        # Flat (no heel) but moving - likely motoring
        if abs_avg_roll < 3.0 and avg_sog > 1.0:
            return True
            
        return False
        
    def _is_sailing(self, abs_avg_roll: float, avg_tws: float, avg_awa: float) -> bool:
        """Check if boat is sailing."""
        # Has heel angle
        if abs_avg_roll > self.config.sailing_min_roll_offset:
            return True
            
        # Good wind and valid sailing angle
        if avg_tws > self.config.motoring_low_wind_threshold:
            if avg_awa > self.config.motoring_close_wind_threshold:
                return True
                
        return False
        
    def _compute_tws(self, awa: float, aws: float, stw: float) -> float:
        """Compute true wind speed from apparent wind."""
        awa_rad = math.radians(awa)
        aw_x = aws * math.cos(awa_rad) - stw
        aw_y = aws * math.sin(awa_rad)
        return math.sqrt(aw_x**2 + aw_y**2)
        
    def _mean(self, values: List[float]) -> float:
        """Calculate mean, returning 0 for empty list."""
        if not values:
            return 0.0
        return sum(values) / len(values)


class TargetDetector:
    """Detects steering target mode from sensor data."""
    
    def __init__(self, config: AnalysisConfig):
        self.config = config
        
    def detect(self, frames: List[LoggedFrame], operation_mode: OperationMode
               ) -> Tuple[SteeringMode, float, float]:
        """
        Detect steering mode and target value from a window of frames.
        
        Returns (steering_mode, target_value, confidence).
        """
        if not frames or operation_mode == OperationMode.ANCHOR:
            return SteeringMode.NONE, 0.0, 0.0
            
        # Extract angle series
        headings = [f.heading for f in frames]
        awas = [f.awa for f in frames]
        
        # Compute TWA for each frame
        twas = []
        for f in frames:
            twa = self._compute_twa(f.awa, f.aws, f.stw)
            twas.append(twa)
            
        # Calculate stability (standard deviation)
        heading_std = self._circular_std(headings)
        awa_std = self._std(awas)
        twa_std = self._std(twas)
        
        avg_twa = self._mean([abs(t) for t in twas])
        
        # Determine mode based on which is most stable
        heading_stable = heading_std < self.config.heading_stability_threshold
        awa_stable = awa_std < self.config.awa_stability_threshold
        twa_stable = twa_std < self.config.twa_stability_threshold
        
        # For motoring, only heading mode makes sense
        if operation_mode == OperationMode.MOTORING:
            if heading_stable:
                target = self._circular_mean(headings)
                confidence = 1.0 - (heading_std / self.config.heading_stability_threshold)
                return SteeringMode.HEADING, target, max(0.5, confidence)
            return SteeringMode.NONE, 0.0, 0.3
            
        # For sailing, check TWA first for downwind
        if avg_twa > self.config.twa_downwind_threshold:
            if twa_stable:
                target = self._mean(twas)
                confidence = 1.0 - (twa_std / self.config.twa_stability_threshold)
                return SteeringMode.TWA, target, max(0.5, confidence)
                
        # Check AWA for upwind/reaching
        if awa_stable and not heading_stable:
            target = self._mean(awas)
            confidence = 1.0 - (awa_std / self.config.awa_stability_threshold)
            return SteeringMode.AWA, target, max(0.5, confidence)
            
        # Check heading
        if heading_stable:
            target = self._circular_mean(headings)
            confidence = 1.0 - (heading_std / self.config.heading_stability_threshold)
            return SteeringMode.HEADING, target, max(0.5, confidence)
            
        # Nothing stable
        return SteeringMode.NONE, 0.0, 0.2
        
    def _compute_twa(self, awa: float, aws: float, stw: float) -> float:
        """Compute true wind angle from apparent wind."""
        awa_rad = math.radians(awa)
        aw_x = aws * math.cos(awa_rad) - stw
        aw_y = aws * math.sin(awa_rad)
        return math.degrees(math.atan2(aw_y, aw_x))
        
    def _mean(self, values: List[float]) -> float:
        """Calculate mean, returning 0 for empty list."""
        if not values:
            return 0.0
        return sum(values) / len(values)
        
    def _std(self, values: List[float]) -> float:
        """Calculate standard deviation."""
        if len(values) < 2:
            return 0.0
        mean = self._mean(values)
        variance = sum((x - mean) ** 2 for x in values) / len(values)
        return math.sqrt(variance)
        
    def _circular_mean(self, angles: List[float]) -> float:
        """Calculate circular mean of angles in degrees."""
        if not angles:
            return 0.0
        sin_sum = sum(math.sin(math.radians(a)) for a in angles)
        cos_sum = sum(math.cos(math.radians(a)) for a in angles)
        return math.degrees(math.atan2(sin_sum, cos_sum)) % 360
        
    def _circular_std(self, angles: List[float]) -> float:
        """Calculate circular standard deviation of angles in degrees."""
        if len(angles) < 2:
            return 0.0
        sin_sum = sum(math.sin(math.radians(a)) for a in angles)
        cos_sum = sum(math.cos(math.radians(a)) for a in angles)
        r = math.sqrt(sin_sum**2 + cos_sum**2) / len(angles)
        # Circular std approximation
        if r >= 1.0:
            return 0.0
        return math.degrees(math.sqrt(-2 * math.log(r)))


class LogAnalyzer:
    """
    Analyzes CAN log files to detect operation modes and steering targets.
    
    Produces metadata files that can be used by TrainingDataLoader.
    """
    
    def __init__(self, config: Optional[AnalysisConfig] = None):
        self.config = config or AnalysisConfig()
        self.parser = CANLogParser()
        self.operation_detector = OperationDetector(self.config)
        self.target_detector = TargetDetector(self.config)
        
    def analyze_file(self, filepath: str) -> AnalysisResult:
        """
        Analyze a single log file.
        
        Returns AnalysisResult with detected segments.
        """
        path = Path(filepath)
        logger.info(f"Analyzing {path.name}...")
        
        # Parse all frames
        frames = list(self.parser.parse_file(filepath))
        
        if not frames:
            logger.warning(f"No frames parsed from {filepath}")
            return AnalysisResult(
                source_file=path.name,
                analyzed_at=datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                total_duration_sec=0,
                total_frame_count=0,
                segments=[],
                summary={"usable_for_training": False, "reason": "No frames parsed"}
            )
            
        # Calculate total duration
        start_time = frames[0].timestamp
        end_time = frames[-1].timestamp
        total_duration = end_time - start_time
        
        # Calculate overall feature coverage
        overall_coverage = self._compute_feature_coverage(frames)
        
        # Analyze in windows
        segments = self._analyze_windows(frames)
        
        # Merge adjacent similar segments
        merged_segments = self._merge_segments(segments)
        
        # Recompute accurate frame counts and coverage for merged segments
        merged_segments = self._recompute_segment_stats(merged_segments, frames)
        # Calculate summary
        summary = self._calculate_summary(merged_segments, total_duration, overall_coverage)

        for seg in merged_segments:
            missing_features = seg.missing_features()
            if len(missing_features) > 0:
                logger.info(f"  Skipping segment, missing features: {missing_features}")
                merged_segments.remove(seg)



        result = AnalysisResult(
            source_file=path.name,
            analyzed_at=datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            total_duration_sec=total_duration,
            total_frame_count=len(frames),
            segments=merged_segments,
            summary=summary,
            overall_feature_coverage=overall_coverage,
        )
        for seg in merged_segments:
            logger.info(f"Segment: {seg.operation_mode} {seg.steering_mode} {seg.target_value} {seg.confidence}% {seg.duration():.2f}s {seg.notes} {seg.frame_count}")
            for feature_name, feature_coverage in seg.feature_coverage.items():
                logger.info(f"  {feature_name}: {feature_coverage.present_count} / {feature_coverage.total_count} ({feature_coverage.coverage_pct:.1f}%)")

        logger.info(f"Found {len(merged_segments)} segments, "
            f"{result.usable_duration_hours():.2f}h usable, "
            f"{result.usable_frame_count()} frames in {path.name}")

        return result
    
    def _compute_feature_coverage(self, frames: List[LoggedFrame]) -> Dict[str, FeatureCoverage]:
        """Compute feature coverage statistics for a set of frames."""
        all_features = {**REQUIRED_FEATURES, **OPTIONAL_FEATURES}
        coverage = {}
        
        for feature_name, field_name in all_features.items():
            fc = FeatureCoverage(name=feature_name, total_count=len(frames))
            
            for frame in frames:
                value = getattr(frame, field_name, 0)
                # Consider a value "present" if it's non-zero
                # (for angles, 0 could be valid, but we check for any data presence)
                if value != 0:
                    fc.present_count += 1
                    
            coverage[feature_name] = fc
            
        return coverage
    
    def _recompute_segment_stats(self, segments: List[Segment], 
                                  all_frames: List[LoggedFrame]) -> List[Segment]:
        """
        Recompute accurate frame counts and coverage for merged segments.
        
        After merging, the frame counts can be inaccurate due to overlapping windows.
        This method recalculates based on actual frames within each segment's time range.
        """
        updated_segments = []
        
        for seg in segments:
            # Get actual frames in this segment's time range
            seg_frames = [f for f in all_frames 
                         if seg.start_time <= f.timestamp < seg.end_time]
            
            # Recompute coverage
            coverage = self._compute_feature_coverage(seg_frames)
            
            # Create updated segment
            updated_seg = Segment(
                start_time=seg.start_time,
                end_time=seg.end_time,
                operation_mode=seg.operation_mode,
                steering_mode=seg.steering_mode,
                target_value=seg.target_value,
                confidence=seg.confidence,
                notes=seg.notes,
                frame_count=len(seg_frames),
                feature_coverage=coverage,
            )
            updated_segments.append(updated_seg)
            
        return updated_segments
        
    def _analyze_windows(self, frames: List[LoggedFrame]) -> List[Segment]:
        """Analyze frames using sliding windows."""
        segments = []
        
        if not frames:
            return segments
            
        window_size = self.config.analysis_window
        step_size = self.config.sample_interval
        
        start_time = frames[0].timestamp
        end_time = frames[-1].timestamp
        
        current_time = start_time
        
        while current_time < end_time:
            # Get frames in current window
            window_end = current_time + window_size
            window_frames = [f for f in frames 
                           if current_time <= f.timestamp < window_end]
            
            if window_frames:
                # Detect modes
                op_mode = self.operation_detector.detect(window_frames)
                steering_mode, target_value, confidence = \
                    self.target_detector.detect(window_frames, op_mode)
                
                # Create notes
                notes = self._generate_notes(window_frames, op_mode, steering_mode)
                
                # Compute feature coverage for this window
                feature_coverage = self._compute_feature_coverage(window_frames)
                
                segment = Segment(
                    start_time=current_time,
                    end_time=window_end,
                    operation_mode=op_mode.value,
                    steering_mode=steering_mode.value,
                    target_value=target_value,
                    confidence=confidence,
                    notes=notes,
                    frame_count=len(window_frames),
                    feature_coverage=feature_coverage,
                )
                segments.append(segment)
                
            current_time += step_size
            
        return segments
        
    def _generate_notes(self, frames: List[LoggedFrame], 
                        op_mode: OperationMode, 
                        steering_mode: SteeringMode) -> str:
        """Generate human-readable notes about detection."""
        notes = []
        
        avg_rpm = sum(f.engine_rpm for f in frames) / len(frames) if frames else 0
        avg_roll = sum(f.roll for f in frames) / len(frames) if frames else 0
        avg_sog = sum(f.sog for f in frames) / len(frames) if frames else 0
        
        if op_mode == OperationMode.MOTORING:
            if avg_rpm > 0:
                notes.append(f"Engine {avg_rpm:.0f} RPM")
            else:
                notes.append("Low wind/close hauled")
        elif op_mode == OperationMode.SAILING:
            notes.append(f"Heel {avg_roll:.1f}deg")
        elif op_mode == OperationMode.ANCHOR:
            notes.append(f"SOG {avg_sog:.1f}kn")
            
        if steering_mode == SteeringMode.HEADING:
            notes.append("Heading stable")
        elif steering_mode == SteeringMode.AWA:
            notes.append("AWA stable")
        elif steering_mode == SteeringMode.TWA:
            notes.append("TWA stable (downwind)")
            
        return ", ".join(notes)
        
    def _merge_segments(self, segments: List[Segment]) -> List[Segment]:
        """Merge adjacent segments with same modes."""
        if not segments:
            return []
            
        merged = []
        current = segments[0]
        accumulated_frames = [current]
        
        for next_seg in segments[1:]:
            # Check if modes match
            same_op = current.operation_mode == next_seg.operation_mode
            same_steering = current.steering_mode == next_seg.steering_mode
            
            # Check if target values are close (for heading, use circular difference)
            if current.steering_mode == SteeringMode.HEADING.value:
                target_diff = abs(self._angle_diff(
                    current.target_value, next_seg.target_value))
            else:
                target_diff = abs(current.target_value - next_seg.target_value)
            similar_target = target_diff < 10.0  # Within 10 degrees
            
            if same_op and same_steering and similar_target:
                # Extend current segment
                accumulated_frames.append(next_seg)
                current = self._merge_two_segments(current, next_seg, accumulated_frames)
            else:
                # Save current and start new
                if current.duration() >= self.config.min_segment_duration:
                    merged.append(current)
                current = next_seg
                accumulated_frames = [next_seg]
                
        # Don't forget last segment
        if current.duration() >= self.config.min_segment_duration:
            merged.append(current)
            
        return merged
    
    def _merge_two_segments(self, current: Segment, next_seg: Segment, 
                            all_segments: List[Segment]) -> Segment:
        """Merge two segments, aggregating frame counts and feature coverage."""
        # Aggregate frame counts
        total_frames = sum(s.frame_count for s in all_segments)
        
        # Aggregate feature coverage
        merged_coverage = {}
        for feature_name in REQUIRED_FEATURES:
            fc = FeatureCoverage(name=feature_name)
            for seg in all_segments:
                if feature_name in seg.feature_coverage:
                    fc.present_count += seg.feature_coverage[feature_name].present_count
                    fc.total_count += seg.feature_coverage[feature_name].total_count
            merged_coverage[feature_name] = fc
            
        for feature_name in OPTIONAL_FEATURES:
            fc = FeatureCoverage(name=feature_name)
            for seg in all_segments:
                if feature_name in seg.feature_coverage:
                    fc.present_count += seg.feature_coverage[feature_name].present_count
                    fc.total_count += seg.feature_coverage[feature_name].total_count
            merged_coverage[feature_name] = fc
        
        return Segment(
            start_time=current.start_time,
            end_time=next_seg.end_time,
            operation_mode=current.operation_mode,
            steering_mode=current.steering_mode,
            target_value=current.target_value,
            confidence=(current.confidence + next_seg.confidence) / 2,
            notes=current.notes,
            frame_count=total_frames,
            feature_coverage=merged_coverage,
        )
        
    def _angle_diff(self, a: float, b: float) -> float:
        """Compute signed angle difference."""
        diff = a - b
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff
        
    def _calculate_summary(self, segments: List[Segment], 
                          total_duration: float,
                          overall_coverage: Dict[str, FeatureCoverage] = None
                          ) -> Dict[str, Any]:
        """Calculate summary statistics."""
        if total_duration <= 0:
            return {"usable_for_training": False, "reason": "No duration"}
            
        # Count time in each mode
        anchor_time = sum(s.duration() for s in segments 
                         if s.operation_mode == OperationMode.ANCHOR.value)
        motoring_time = sum(s.duration() for s in segments 
                           if s.operation_mode == OperationMode.MOTORING.value)
        sailing_time = sum(s.duration() for s in segments 
                          if s.operation_mode == OperationMode.SAILING.value)
        unknown_time = sum(s.duration() for s in segments 
                          if s.operation_mode == OperationMode.UNKNOWN.value)
        
        # Calculate percentages
        anchor_pct = round(100 * anchor_time / total_duration, 1)
        motoring_pct = round(100 * motoring_time / total_duration, 1)
        sailing_pct = round(100 * sailing_time / total_duration, 1)
        unknown_pct = round(100 * unknown_time / total_duration, 1)
        
        # Count usable segments and frames
        usable_segments = [s for s in segments if s.is_usable_for_training()]
        usable_time = sum(s.duration() for s in usable_segments)
        usable_frames = sum(s.frame_count for s in usable_segments)
        total_frames = sum(s.frame_count for s in segments)
        
        # Check for missing required features
        missing_required = []
        if overall_coverage:
            for name in REQUIRED_FEATURES:
                if name in overall_coverage:
                    if not overall_coverage[name].is_available:
                        missing_required.append(name)
        
        # Determine if usable for training
        usable = usable_time >= 60.0 and len(missing_required) <= 2  # Allow some missing
        
        return {
            # Time breakdown
            "total_duration_hours": round(total_duration / 3600, 4),
            "anchor_hours": round(anchor_time / 3600, 4),
            "motoring_hours": round(motoring_time / 3600, 4),
            "sailing_hours": round(sailing_time / 3600, 4),
            "unknown_hours": round(unknown_time / 3600, 4),
            
            # Percentages
            "anchor_pct": anchor_pct,
            "motoring_pct": motoring_pct,
            "sailing_pct": sailing_pct,
            "unknown_pct": unknown_pct,
            
            # Usable data
            "usable_for_training": usable,
            "usable_duration_sec": round(usable_time, 1),
            "usable_duration_hours": round(usable_time / 3600, 4),
            "usable_segment_count": len(usable_segments),
            "total_segment_count": len(segments),
            
            # Frame counts
            "total_frame_count": total_frames,
            "usable_frame_count": usable_frames,
            "training_records_estimate": max(0, usable_frames - 20),  # Subtract sequence length
            
            # Feature coverage
            "missing_required_features": missing_required,
            "required_features_available": len(missing_required) == 0,
        }
        
    def save_metadata(self, result: AnalysisResult, output_path: Optional[str] = None
                     ) -> str:
        """
        Save analysis result to metadata JSON file.
        
        If output_path is not specified, creates .meta.json alongside source file.
        """
        if output_path:
            meta_path = Path(output_path)
        else:
            # Create path alongside source file
            source_path = Path(result.source_file)
            meta_path = source_path.with_suffix('.meta.json')
            
        with open(meta_path, 'w') as f:
            json.dump(result.to_dict(), f, indent=2)
            
        logger.info(f"Saved metadata to {meta_path}")
        return str(meta_path)


def analyze_directory(directory: str, config: Optional[AnalysisConfig] = None,
                     recursive: bool = True, dry_run: bool = False
                     ) -> List[AnalysisResult]:
    """
    Analyze all log files in a directory.
    
    Args:
        directory: Path to directory containing log files
        config: Analysis configuration
        recursive: Whether to search subdirectories
        dry_run: If True, don't write metadata files
        
    Returns:
        List of AnalysisResult objects
    """
    analyzer = LogAnalyzer(config)
    results = []
    
    path = Path(directory)
    pattern = '**/*.log' if recursive else '*.log'
    
    for log_file in sorted(path.glob(pattern)):
        if log_file.is_file():
            try:
                result = analyzer.analyze_file(str(log_file))
                results.append(result)
                
                if not dry_run and result.segments:
                    # Save metadata alongside log file
                    meta_path = log_file.with_suffix('.meta.json')
                    analyzer.save_metadata(result, str(meta_path))
                    
            except Exception as e:
                logger.error(f"Failed to analyze {log_file}: {e}")
                
    return results


def main() -> None:
    """CLI entry point for log analyzer."""
    parser = argparse.ArgumentParser(
        description="Analyze CAN log files to detect operation modes and steering targets"
    )
    parser.add_argument(
        "path",
        type=Path,
        help="Path to log file or directory"
    )
    parser.add_argument(
        "--recursive", "-r",
        action="store_true",
        default=True,
        help="Search directories recursively (default: True)"
    )
    parser.add_argument(
        "--dry-run", "-n",
        action="store_true",
        help="Show analysis without writing metadata files"
    )
    parser.add_argument(
        "--interactive", "-i",
        action="store_true",
        help="Interactive mode to review and adjust segments"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Verbose output"
    )
    
    args = parser.parse_args()
    
    # Setup logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=log_level, format='%(levelname)s: %(message)s')
    
    config = AnalysisConfig()
    analyzer = LogAnalyzer(config)
    
    if args.path.is_file():
        # Single file
        result = analyzer.analyze_file(str(args.path))
        _print_result(result)
        
        if not args.dry_run and result.segments:
            meta_path = args.path.with_suffix('.meta.json')
            analyzer.save_metadata(result, str(meta_path))
            print(f"\nMetadata saved to: {meta_path}")
            
        if args.interactive:
            _interactive_review(result, analyzer, args.path, args.dry_run)
            
    elif args.path.is_dir():
        # Directory
        results = analyze_directory(
            str(args.path),
            config=config,
            recursive=args.recursive,
            dry_run=args.dry_run
        )
        
        _print_directory_summary(results)
        
        if args.dry_run:
            print("\nDry run - no metadata files written")
        else:
            print(f"\nMetadata files written alongside log files")
    else:
        print(f"Error: {args.path} does not exist")
        return


def _print_directory_summary(results: List[AnalysisResult]) -> None:
    """Print summary for directory analysis."""
    print(f"\n{'='*70}")
    print(f"DIRECTORY ANALYSIS SUMMARY")
    print(f"{'='*70}")
    print(f"Files analyzed: {len(results)}")
    
    # Aggregate statistics
    total_hours = sum(r.total_duration_hours() for r in results)
    usable_hours = sum(r.usable_duration_hours() for r in results)
    total_frames = sum(r.total_frame_count for r in results)
    usable_frames = sum(r.usable_frame_count() for r in results)
    usable_files = sum(1 for r in results if r.summary.get('usable_for_training'))
    
    print(f"\n--- Data Volume ---")
    print(f"Total log duration:    {total_hours:.2f} hours")
    print(f"Usable for training:   {usable_hours:.2f} hours ({100*usable_hours/total_hours:.1f}%)" if total_hours > 0 else "")
    print(f"Total frames:          {total_frames:,}")
    print(f"Usable frames:         {usable_frames:,}")
    print(f"Training records est:  {max(0, usable_frames - 20 * len(results)):,}")
    
    print(f"\n--- Files ---")
    print(f"Usable for training:   {usable_files}/{len(results)} files")
    
    # Aggregate by operation mode
    anchor_hours = sum(r.summary.get('anchor_hours', 0) for r in results)
    motoring_hours = sum(r.summary.get('motoring_hours', 0) for r in results)
    sailing_hours = sum(r.summary.get('sailing_hours', 0) for r in results)
    
    print(f"\n--- Time by Operation Mode ---")
    print(f"Anchor:    {anchor_hours:.2f} hours")
    print(f"Motoring:  {motoring_hours:.2f} hours")
    print(f"Sailing:   {sailing_hours:.2f} hours")
    
    # Feature coverage across all files
    print(f"\n--- Feature Coverage (across all files) ---")
    feature_counts = {}
    for r in results:
        for name, fc in r.overall_feature_coverage.items():
            if name not in feature_counts:
                feature_counts[name] = {'present': 0, 'total': 0}
            feature_counts[name]['present'] += fc.present_count
            feature_counts[name]['total'] += fc.total_count
    
    print(f"{'Feature':<15} {'Coverage':>10} {'Status':>12}")
    print(f"{'-'*15} {'-'*10} {'-'*12}")
    for name in REQUIRED_FEATURES:
        if name in feature_counts:
            fc = feature_counts[name]
            pct = 100 * fc['present'] / fc['total'] if fc['total'] > 0 else 0
            status = "OK" if pct > 50 else "MISSING"
            marker = "" if pct > 50 else " <--"
            print(f"{name:<15} {pct:>9.1f}% {status:>12}{marker}")
    
    print(f"\nOptional features:")
    for name in OPTIONAL_FEATURES:
        if name in feature_counts:
            fc = feature_counts[name]
            pct = 100 * fc['present'] / fc['total'] if fc['total'] > 0 else 0
            print(f"  {name:<15} {pct:>9.1f}%")
    
    print(f"{'='*70}")


def _print_result(result: AnalysisResult) -> None:
    """Print analysis result to console."""
    print(f"\n{'='*70}")
    print(f"FILE ANALYSIS: {result.source_file}")
    print(f"{'='*70}")
    
    # Overview
    print(f"\n--- Overview ---")
    print(f"Total duration:  {result.total_duration_hours():.4f} hours ({result.total_duration_sec:.1f} sec)")
    print(f"Total frames:    {result.total_frame_count:,}")
    print(f"Segments found:  {len(result.segments)}")
    
    # Usable data summary
    usable_segs = result.usable_segments()
    print(f"\n--- Usable Data ---")
    print(f"Usable duration: {result.usable_duration_hours():.4f} hours ({result.usable_duration_sec():.1f} sec)")
    print(f"Usable frames:   {result.usable_frame_count():,}")
    print(f"Usable segments: {len(usable_segs)}/{len(result.segments)}")
    training_records = max(0, result.usable_frame_count() - 20)  # Subtract sequence length
    print(f"Training records estimate: {training_records:,}")
    
    # Feature coverage
    print(f"\n--- Feature Coverage ---")
    print(f"{'Feature':<15} {'Coverage':>10} {'Status':>12}")
    print(f"{'-'*15} {'-'*10} {'-'*12}")
    for name in REQUIRED_FEATURES:
        if name in result.overall_feature_coverage:
            fc = result.overall_feature_coverage[name]
            status = "OK" if fc.is_available else "MISSING"
            marker = "" if fc.is_available else " <--"
            print(f"{name:<15} {fc.coverage_pct:>9.1f}% {status:>12}{marker}")
    
    missing = result.summary.get('missing_required_features', [])
    if missing:
        print(f"\nWARNING: Missing required features: {', '.join(missing)}")
    
    # Segments detail
    print(f"\n--- Segments ---")
    for i, seg in enumerate(result.segments, 1):
        usable_marker = "[USABLE]" if seg.is_usable_for_training() else "[skip]"
        print(f"\n[{i}] {seg.operation_mode.upper()} / {seg.steering_mode.upper()} {usable_marker}")
        print(f"    Duration:    {seg.duration_hours():.4f} hours ({seg.duration():.1f} sec)")
        print(f"    Frames:      {seg.frame_count:,}")
        print(f"    Target:      {seg.target_value:.1f}° (confidence: {seg.confidence:.0%})")
        
        # Show missing features for this segment
        seg_missing = seg.missing_features()
        if seg_missing:
            print(f"    Missing:     {', '.join(seg_missing)}")
        
        print(f"    Notes:       {seg.notes}")
        
    # Summary
    print(f"\n--- Summary ---")
    summary = result.summary
    print(f"Anchor:      {summary.get('anchor_hours', 0):.4f} hours ({summary.get('anchor_pct', 0):.1f}%)")
    print(f"Motoring:    {summary.get('motoring_hours', 0):.4f} hours ({summary.get('motoring_pct', 0):.1f}%)")
    print(f"Sailing:     {summary.get('sailing_hours', 0):.4f} hours ({summary.get('sailing_pct', 0):.1f}%)")
    print(f"Unknown:     {summary.get('unknown_hours', 0):.4f} hours ({summary.get('unknown_pct', 0):.1f}%)")
    
    print(f"\nUsable for training: {'YES' if summary.get('usable_for_training') else 'NO'}")
    print(f"{'='*70}")


def _interactive_review(result: AnalysisResult, analyzer: LogAnalyzer,
                        filepath: Path, dry_run: bool) -> None:
    """Interactive mode to review and adjust segments."""
    print("\n[Interactive Mode]")
    print("Commands: (n)ext, (p)rev, (e)dit, (d)elete, (s)ave, (q)uit")
    
    segments = list(result.segments)
    idx = 0
    modified = False
    
    while True:
        if segments:
            seg = segments[idx]
            print(f"\nSegment {idx+1}/{len(segments)}:")
            print(f"  Operation: {seg.operation_mode}")
            print(f"  Steering: {seg.steering_mode}")
            print(f"  Target: {seg.target_value:.1f}°")
            print(f"  Confidence: {seg.confidence:.0%}")
            
        cmd = input("\nCommand: ").strip().lower()
        
        if cmd in ('q', 'quit'):
            if modified and not dry_run:
                save = input("Save changes? (y/n): ").strip().lower()
                if save == 'y':
                    result.segments = segments
                    meta_path = filepath.with_suffix('.meta.json')
                    analyzer.save_metadata(result, str(meta_path))
                    print(f"Saved to {meta_path}")
            break
            
        elif cmd in ('n', 'next'):
            if idx < len(segments) - 1:
                idx += 1
                
        elif cmd in ('p', 'prev'):
            if idx > 0:
                idx -= 1
                
        elif cmd in ('e', 'edit') and segments:
            print("Edit (leave blank to keep current):")
            
            new_op = input(f"  Operation [{seg.operation_mode}]: ").strip()
            if new_op in ('anchor', 'motoring', 'sailing', 'unknown'):
                seg.operation_mode = new_op
                modified = True
                
            new_steer = input(f"  Steering [{seg.steering_mode}]: ").strip()
            if new_steer in ('heading', 'awa', 'twa', 'none'):
                seg.steering_mode = new_steer
                modified = True
                
            new_target = input(f"  Target [{seg.target_value:.1f}]: ").strip()
            if new_target:
                try:
                    seg.target_value = float(new_target)
                    modified = True
                except ValueError:
                    print("Invalid number")
                    
        elif cmd in ('d', 'delete') and segments:
            segments.pop(idx)
            modified = True
            if idx >= len(segments) and segments:
                idx = len(segments) - 1
                
        elif cmd in ('s', 'save') and not dry_run:
            result.segments = segments
            meta_path = filepath.with_suffix('.meta.json')
            analyzer.save_metadata(result, str(meta_path))
            print(f"Saved to {meta_path}")
            modified = False


if __name__ == "__main__":
    main()
