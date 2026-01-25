"""
NMEA2000 Data Logger Module
===========================

Standalone data logger service that captures NMEA2000 bus data for offline
ML model fine-tuning. Detects operation modes (anchor/motoring/sailing) and
helm control source (human/Raymarine autopilot/ML autopilot).

Features:
- Compressed JSONL output with gzip compression
- Log rotation based on file size or age
- Anchor detection to pause logging when not relevant
- Pilot mode detection from Raymarine proprietary PGNs
"""

import gzip
import json
import logging
import os
import time
import math
import argparse
import signal
import sys
from dataclasses import dataclass, asdict, field
from datetime import datetime
from enum import Enum, auto
from pathlib import Path
from typing import Optional, Dict, Any, List, Deque
from collections import deque

from ..sensors.nmea2000_interface import (
    NMEA2000Interface,
    N2KConfig,
    N2KData,
    PGN,
    PilotMode,
    calculate_true_wind,
)

logger = logging.getLogger(__name__)


@dataclass
class DataLoggerConfig:
    """Configuration for the NMEA2000 data logger."""
    
    can_channel: str = "can0"
    log_dir: str = "logs/n2k"
    max_file_size_mb: float = 256.0
    max_file_age_minutes: float = 60.0
    sample_rate_hz: float = 1.0
    compression: bool = True
    
    # Anchor detection thresholds
    anchor_sog_threshold: float = 0.5    # knots
    anchor_position_radius: float = 50.0  # meters
    anchor_detection_window_s: float = 300.0  # 5 minutes
    
    # Motoring detection thresholds
    motoring_min_rpm: float = 500.0
    motoring_low_wind_threshold: float = 5.0  # knots TWS
    
    # Position buffer for anchor detection
    position_buffer_size: int = 60  # 1 minute at 1 Hz


class OperationMode(Enum):
    """Boat operation mode."""
    ANCHOR = auto()     # At anchor, skip logging
    MOTORING = auto()   # Engine running, log for motoring behavior
    SAILING = auto()    # Sailing, log for sailing behavior
    UNKNOWN = auto()    # Default, log conservatively


class HelmSource(Enum):
    """Who/what is controlling the helm."""
    HUMAN = auto()              # Human at helm (Raymarine pilot in standby)
    RAYMARINE_COMPASS = auto()  # Raymarine steering to compass heading
    RAYMARINE_WIND = auto()     # Raymarine steering to wind angle
    ML_PILOT = auto()           # Our ML autopilot (via proprietary PGN, TBD)
    UNKNOWN = auto()            # Unable to determine


@dataclass
class LogRecord:
    """
    Single record of logged data.
    
    Compatible with LoggedFrame structure from data_loader.py for training.
    """
    timestamp: float
    
    # Navigation data
    heading: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    yaw_rate: float = 0.0
    
    # Wind data
    awa: float = 0.0
    aws: float = 0.0
    twa: float = 0.0
    tws: float = 0.0
    
    # Speed data
    stw: float = 0.0
    cog: float = 0.0
    sog: float = 0.0
    
    # Position
    latitude: float = 0.0
    longitude: float = 0.0
    
    # Rudder
    rudder_angle: float = 0.0
    
    # Engine
    engine_rpm: float = 0.0
    
    # Raymarine autopilot
    pilot_heading: float = 0.0
    pilot_wind_angle: float = 0.0
    pilot_mode: int = 0
    pilot_submode: int = 0
    
    # Computed fields
    helm_source: str = "unknown"
    operation_mode: str = "unknown"
    
    # ML pilot status (TBD - placeholder)
    ml_pilot_engaged: bool = False
    
    # Target values (for training)
    target_heading: float = 0.0
    target_awa: float = 0.0
    target_twa: float = 0.0
    mode: str = "unknown"  # Steering mode: compass, wind_awa, wind_twa


@dataclass
class PositionSample:
    """Position sample for anchor detection."""
    timestamp: float
    latitude: float
    longitude: float
    sog: float


class DataLogger:
    """
    Standalone NMEA2000 data logger for ML fine-tuning.
    
    Captures all relevant NMEA2000 data, detects operation modes,
    and writes compressed logs with automatic rotation.
    """
    
    def __init__(self, config: Optional[DataLoggerConfig] = None):
        self.config = config or DataLoggerConfig()
        
        # N2K interface
        self._n2k_config = N2KConfig(channel=self.config.can_channel)
        self._n2k = NMEA2000Interface(self._n2k_config)
        
        # Current state
        self._operation_mode = OperationMode.UNKNOWN
        self._helm_source = HelmSource.UNKNOWN
        self._at_anchor = False
        self._running = False
        
        # Position buffer for anchor detection
        self._position_buffer: Deque[PositionSample] = deque(
            maxlen=self.config.position_buffer_size
        )
        
        # Log file state
        self._current_file: Optional[gzip.GzipFile] = None
        self._current_path: Optional[Path] = None
        self._file_start_time: float = 0.0
        self._record_count: int = 0
        
        # Statistics
        self._total_records: int = 0
        self._files_written: int = 0
        self._skipped_anchor: int = 0
        
        # Ensure log directory exists
        Path(self.config.log_dir).mkdir(parents=True, exist_ok=True)
        
    def start(self) -> bool:
        """Start the data logger."""
        if not self._n2k.start():
            logger.error("Failed to start NMEA2000 interface")
            return False
            
        self._running = True
        logger.info(f"Data logger started, output: {self.config.log_dir}")
        return True
        
    def stop(self):
        """Stop the data logger and close files."""
        self._running = False
        self._close_current_file()
        self._n2k.stop()
        logger.info(
            f"Data logger stopped. Records: {self._total_records}, "
            f"Files: {self._files_written}, Skipped (anchor): {self._skipped_anchor}"
        )
        
    def run(self):
        """Main logging loop."""
        sample_interval = 1.0 / self.config.sample_rate_hz
        last_sample_time = 0.0
        
        while self._running:
            now = time.time()
            
            # Check if it's time for a new sample
            if now - last_sample_time >= sample_interval:
                last_sample_time = now
                self._process_sample()
                
            # Small sleep to avoid busy loop
            time.sleep(0.01)
            
    def _process_sample(self):
        """Process a single sample from the NMEA2000 bus."""
        # Get current data
        n2k_data = self._n2k.get_data()
        
        # Update position buffer for anchor detection
        if n2k_data.latitude != 0 and n2k_data.longitude != 0:
            self._position_buffer.append(PositionSample(
                timestamp=n2k_data.timestamp,
                latitude=n2k_data.latitude,
                longitude=n2k_data.longitude,
                sog=n2k_data.sog,
            ))
            
        # Detect operation mode
        self._update_operation_mode(n2k_data)
        
        # Skip logging if at anchor
        if self._operation_mode == OperationMode.ANCHOR:
            self._skipped_anchor += 1
            return
            
        # Detect helm source
        self._update_helm_source(n2k_data)
        
        # Build log record
        record = self._build_record(n2k_data)
        
        # Write to log
        self._write_record(record)
        
    def _update_operation_mode(self, data: N2KData):
        """Update the operation mode based on sensor data."""
        # Check for anchor
        if self._is_at_anchor():
            self._operation_mode = OperationMode.ANCHOR
            self._at_anchor = True
            return
            
        self._at_anchor = False
        
        # Check for motoring (engine RPM > threshold)
        if data.engine_rpm > self.config.motoring_min_rpm:
            self._operation_mode = OperationMode.MOTORING
            return
            
        # Calculate true wind for sailing detection
        if data.aws > 0 and data.stw > 0:
            twa, tws = calculate_true_wind(
                data.awa, data.aws, data.stw, data.heading_n2k
            )
            # Low wind - likely motoring even without RPM
            if tws < self.config.motoring_low_wind_threshold:
                self._operation_mode = OperationMode.MOTORING
                return
                
        # Default to sailing if moving and not motoring
        if data.sog > 1.0:
            self._operation_mode = OperationMode.SAILING
        else:
            self._operation_mode = OperationMode.UNKNOWN
            
    def _is_at_anchor(self) -> bool:
        """Check if the boat is at anchor based on position buffer."""
        if len(self._position_buffer) < 10:
            return False
            
        # Get recent samples
        now = time.time()
        window_start = now - self.config.anchor_detection_window_s
        recent = [p for p in self._position_buffer if p.timestamp > window_start]
        
        if len(recent) < 10:
            return False
            
        # Check average SOG
        avg_sog = sum(p.sog for p in recent) / len(recent)
        if avg_sog > self.config.anchor_sog_threshold:
            return False
            
        # Check position wander
        lats = [p.latitude for p in recent]
        lons = [p.longitude for p in recent]
        
        lat_range = max(lats) - min(lats)
        lon_range = max(lons) - min(lons)
        
        # Convert to meters (1 degree lat ~ 111km)
        avg_lat = sum(lats) / len(lats)
        lat_meters = lat_range * 111000
        lon_meters = lon_range * 111000 * math.cos(math.radians(avg_lat))
        max_wander = max(lat_meters, lon_meters)
        
        return max_wander < self.config.anchor_position_radius
        
    def _update_helm_source(self, data: N2KData):
        """Update helm source based on Raymarine pilot mode."""
        # Check for ML pilot first (TBD - placeholder logic)
        # In future, this will check a proprietary PGN
        # For now, ML pilot is never detected via N2K
        
        # Check Raymarine pilot mode from PGN 65379
        pilot_mode_age = data.get_age_ms(data.pilot_mode_timestamp)
        
        if pilot_mode_age < 5000:  # Valid if < 5 seconds old
            if data.pilot_mode == PilotMode.STANDBY:
                self._helm_source = HelmSource.HUMAN
            elif data.pilot_mode == PilotMode.AUTO:
                self._helm_source = HelmSource.RAYMARINE_COMPASS
            elif data.pilot_mode == PilotMode.WIND:
                self._helm_source = HelmSource.RAYMARINE_WIND
            elif data.pilot_mode in (PilotMode.TRACK, PilotMode.NO_DRIFT):
                # Track and no-drift modes treated as compass-like
                self._helm_source = HelmSource.RAYMARINE_COMPASS
            else:
                self._helm_source = HelmSource.UNKNOWN
        else:
            # Fallback: check PGN 127237 (Heading/Track Control)
            track_age = data.get_age_ms(data.track_control_timestamp)
            if track_age < 5000:
                # Steering mode present suggests autopilot active
                if data.steering_mode > 0:
                    self._helm_source = HelmSource.RAYMARINE_COMPASS  # Assume compass
                else:
                    self._helm_source = HelmSource.HUMAN
            else:
                self._helm_source = HelmSource.UNKNOWN
                
    def _build_record(self, data: N2KData) -> LogRecord:
        """Build a log record from NMEA2000 data."""
        # Calculate true wind
        twa, tws = 0.0, 0.0
        if data.aws > 0 and data.stw > 0:
            twa, tws = calculate_true_wind(
                data.awa, data.aws, data.stw, data.heading_n2k
            )
            
        # Determine steering mode and target from helm source
        mode = "unknown"
        target_heading = 0.0
        target_awa = 0.0
        target_twa = 0.0
        
        if self._helm_source == HelmSource.RAYMARINE_COMPASS:
            mode = "compass"
            target_heading = data.pilot_heading
        elif self._helm_source == HelmSource.RAYMARINE_WIND:
            mode = "wind_awa"
            target_awa = data.pilot_wind_angle
            # Estimate TWA target from current conditions if needed
            
        return LogRecord(
            timestamp=data.timestamp,
            heading=data.heading_n2k,
            pitch=0.0,  # Not available from NMEA2000 in current setup
            roll=0.0,   # Not available from NMEA2000 in current setup
            yaw_rate=0.0,  # Would need Rate of Turn PGN 127251
            awa=data.awa,
            aws=data.aws,
            twa=twa,
            tws=tws,
            stw=data.stw,
            cog=data.cog,
            sog=data.sog,
            latitude=data.latitude,
            longitude=data.longitude,
            rudder_angle=data.rudder_angle_n2k,
            engine_rpm=data.engine_rpm,
            pilot_heading=data.pilot_heading,
            pilot_wind_angle=data.pilot_wind_angle,
            pilot_mode=data.pilot_mode,
            pilot_submode=data.pilot_submode,
            helm_source=self._helm_source.name.lower(),
            operation_mode=self._operation_mode.name.lower(),
            ml_pilot_engaged=False,  # TBD
            target_heading=target_heading,
            target_awa=target_awa,
            target_twa=target_twa,
            mode=mode,
        )
        
    def _write_record(self, record: LogRecord):
        """Write a record to the current log file."""
        # Check if we need to rotate
        if self._should_rotate():
            self._rotate_file()
            
        # Ensure file is open
        if self._current_file is None:
            self._open_new_file()
            
        # Write record as JSON line
        try:
            record_dict = asdict(record)
            line = json.dumps(record_dict) + "\n"
            self._current_file.write(line.encode('utf-8'))
            self._record_count += 1
            self._total_records += 1
        except Exception as e:
            logger.error(f"Failed to write record: {e}")
            
    def _should_rotate(self) -> bool:
        """Check if log file should be rotated."""
        if self._current_file is None or self._current_path is None:
            return True
            
        # Check age
        file_age_min = (time.time() - self._file_start_time) / 60
        if file_age_min >= self.config.max_file_age_minutes:
            return True
            
        # Check size
        try:
            file_size_mb = os.path.getsize(self._current_path) / (1024 * 1024)
            if file_size_mb >= self.config.max_file_size_mb:
                return True
        except OSError:
            pass
            
        return False
        
    def _open_new_file(self):
        """Open a new log file."""
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        if self.config.compression:
            filename = f"n2k_{timestamp_str}.jsonlog.gz"
            self._current_path = Path(self.config.log_dir) / filename
            self._current_file = gzip.open(self._current_path, 'wb')
        else:
            filename = f"n2k_{timestamp_str}.jsonlog"
            self._current_path = Path(self.config.log_dir) / filename
            self._current_file = open(self._current_path, 'wb')
            
        self._file_start_time = time.time()
        self._record_count = 0
        self._files_written += 1
        
        logger.info(f"Opened new log file: {filename}")
        
    def _close_current_file(self):
        """Close the current log file."""
        if self._current_file is not None:
            try:
                self._current_file.close()
                logger.info(
                    f"Closed log file: {self._current_path.name}, "
                    f"records: {self._record_count}"
                )
            except Exception as e:
                logger.error(f"Error closing file: {e}")
            finally:
                self._current_file = None
                self._current_path = None
                
    def _rotate_file(self):
        """Rotate to a new log file."""
        self._close_current_file()
        self._open_new_file()
        
    @property
    def stats(self) -> Dict[str, Any]:
        """Get current statistics."""
        return {
            "total_records": self._total_records,
            "files_written": self._files_written,
            "skipped_anchor": self._skipped_anchor,
            "operation_mode": self._operation_mode.name,
            "helm_source": self._helm_source.name,
            "at_anchor": self._at_anchor,
            "current_file": str(self._current_path) if self._current_path else None,
            "n2k_stats": self._n2k.stats,
        }


def main():
    """CLI entry point for standalone data logger."""
    parser = argparse.ArgumentParser(
        description="NMEA2000 Data Logger for ML Fine-Tuning"
    )
    parser.add_argument(
        "--channel", "-c",
        default="can0",
        help="CAN interface (default: can0)"
    )
    parser.add_argument(
        "--output", "-o",
        default="logs/n2k",
        help="Output directory (default: logs/n2k)"
    )
    parser.add_argument(
        "--max-size",
        type=float,
        default=256.0,
        help="Max file size in MB (default: 256)"
    )
    parser.add_argument(
        "--max-age",
        type=float,
        default=60.0,
        help="Max file age in minutes (default: 60)"
    )
    parser.add_argument(
        "--sample-rate",
        type=float,
        default=1.0,
        help="Sample rate in Hz (default: 1.0)"
    )
    parser.add_argument(
        "--no-compress",
        action="store_true",
        help="Disable gzip compression"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable debug logging"
    )
    
    args = parser.parse_args()
    
    # Configure logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )
    
    # Create config
    config = DataLoggerConfig(
        can_channel=args.channel,
        log_dir=args.output,
        max_file_size_mb=args.max_size,
        max_file_age_minutes=args.max_age,
        sample_rate_hz=args.sample_rate,
        compression=not args.no_compress,
    )
    
    # Create logger
    data_logger = DataLogger(config)
    
    # Handle graceful shutdown
    def signal_handler(sig, frame):
        logger.info("Shutdown requested...")
        data_logger.stop()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Start logging
    if not data_logger.start():
        logger.error("Failed to start data logger")
        sys.exit(1)
        
    logger.info(f"Logging to {config.log_dir} at {config.sample_rate_hz} Hz")
    logger.info("Press Ctrl+C to stop")
    
    # Run main loop
    try:
        data_logger.run()
    except Exception as e:
        logger.exception(f"Data logger error: {e}")
        data_logger.stop()
        sys.exit(1)


if __name__ == "__main__":
    main()
