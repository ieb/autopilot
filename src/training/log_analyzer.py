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
    
    def duration(self) -> float:
        """Return segment duration in seconds."""
        return self.end_time - self.start_time
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "start_time": self.start_time,
            "end_time": self.end_time,
            "operation_mode": self.operation_mode,
            "steering_mode": self.steering_mode,
            "target_value": round(self.target_value, 1),
            "confidence": round(self.confidence, 2),
            "notes": self.notes,
        }


@dataclass
class AnalysisResult:
    """Result of log file analysis."""
    source_file: str
    analyzed_at: str
    total_duration_sec: float
    segments: List[Segment] = field(default_factory=list)
    summary: Dict[str, Any] = field(default_factory=dict)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "source_file": self.source_file,
            "analyzed_at": self.analyzed_at,
            "total_duration_sec": round(self.total_duration_sec, 1),
            "segments": [s.to_dict() for s in self.segments],
            "summary": self.summary,
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
                segments=[],
                summary={"usable_for_training": False, "reason": "No frames parsed"}
            )
            
        # Calculate total duration
        start_time = frames[0].timestamp
        end_time = frames[-1].timestamp
        total_duration = end_time - start_time
        
        # Analyze in windows
        segments = self._analyze_windows(frames)
        
        # Merge adjacent similar segments
        merged_segments = self._merge_segments(segments)
        
        # Calculate summary
        summary = self._calculate_summary(merged_segments, total_duration)
        
        result = AnalysisResult(
            source_file=path.name,
            analyzed_at=datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            total_duration_sec=total_duration,
            segments=merged_segments,
            summary=summary,
        )
        
        logger.info(f"Found {len(merged_segments)} segments in {path.name}")
        return result
        
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
                
                segment = Segment(
                    start_time=current_time,
                    end_time=window_end,
                    operation_mode=op_mode.value,
                    steering_mode=steering_mode.value,
                    target_value=target_value,
                    confidence=confidence,
                    notes=notes,
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
                current = Segment(
                    start_time=current.start_time,
                    end_time=next_seg.end_time,
                    operation_mode=current.operation_mode,
                    steering_mode=current.steering_mode,
                    target_value=current.target_value,  # Keep first target
                    confidence=(current.confidence + next_seg.confidence) / 2,
                    notes=current.notes,
                )
            else:
                # Save current and start new
                if current.duration() >= self.config.min_segment_duration:
                    merged.append(current)
                current = next_seg
                
        # Don't forget last segment
        if current.duration() >= self.config.min_segment_duration:
            merged.append(current)
            
        return merged
        
    def _angle_diff(self, a: float, b: float) -> float:
        """Compute signed angle difference."""
        diff = a - b
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff
        
    def _calculate_summary(self, segments: List[Segment], 
                          total_duration: float) -> Dict[str, Any]:
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
        
        # Determine if usable for training
        usable_time = motoring_time + sailing_time
        usable = usable_time >= 60.0  # At least 1 minute of usable data
        
        return {
            "anchor_pct": anchor_pct,
            "motoring_pct": motoring_pct,
            "sailing_pct": sailing_pct,
            "unknown_pct": unknown_pct,
            "usable_for_training": usable,
            "usable_duration_sec": round(usable_time, 1),
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
        
        print(f"\n{'='*60}")
        print(f"Analyzed {len(results)} files")
        
        # Summary
        total_usable = sum(1 for r in results if r.summary.get('usable_for_training'))
        print(f"Usable for training: {total_usable}/{len(results)}")
        
        if args.dry_run:
            print("\nDry run - no metadata files written")
        else:
            print(f"\nMetadata files written alongside log files")
    else:
        print(f"Error: {args.path} does not exist")
        return


def _print_result(result: AnalysisResult) -> None:
    """Print analysis result to console."""
    print(f"\n{'='*60}")
    print(f"File: {result.source_file}")
    print(f"Duration: {result.total_duration_sec:.1f} seconds")
    print(f"Segments: {len(result.segments)}")
    print(f"{'='*60}")
    
    for i, seg in enumerate(result.segments, 1):
        duration = seg.duration()
        print(f"\n[{i}] {seg.operation_mode.upper()} / {seg.steering_mode.upper()}")
        print(f"    Time: {duration:.1f}s")
        print(f"    Target: {seg.target_value:.1f}° (confidence: {seg.confidence:.0%})")
        print(f"    Notes: {seg.notes}")
        
    print(f"\n{'='*60}")
    print("Summary:")
    for key, value in result.summary.items():
        print(f"  {key}: {value}")


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
