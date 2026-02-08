"""
Data Loader Module
==================

Loads and processes training data from logged CAN/sensor data.
Creates training sequences for imitation learning.
"""

import os
import json
import struct
import time
from dataclasses import dataclass
from typing import Optional, Iterator, Tuple, List, Dict, Any
from pathlib import Path
import numpy as np
import logging

logger = logging.getLogger(__name__)


@dataclass
class DataConfig:
    """Configuration for data loading."""
    sequence_length: int = 20      # Timesteps per sequence
    feature_dim: int = 25          # Features per timestep
    sample_rate_hz: float = 10.0   # Target sample rate
    train_split: float = 0.8       # Train/validation split
    shuffle: bool = True           # Shuffle training data
    normalize: bool = True         # Apply feature normalization
    mode_transition_filter_s: float = 15.0  # Seconds to filter after mode changes


@dataclass
class LoggedFrame:
    """Single frame of logged data."""
    timestamp: float
    
    # IMU data
    heading: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    yaw_rate: float = 0.0
    
    # NMEA2000 data
    awa: float = 0.0
    aws: float = 0.0
    stw: float = 0.0
    cog: float = 0.0
    sog: float = 0.0
    
    # Rudder (the label for training)
    rudder_angle: float = 0.0
    
    # Mode info
    target_heading: float = 0.0
    target_awa: float = 0.0
    target_twa: float = 0.0
    mode: str = "unknown"
    
    # Engine data (for operation mode detection)
    engine_rpm: float = 0.0
    engine_hours: float = 0.0
    engine_temp: float = 0.0
    
    # Position data (for anchor detection)
    latitude: float = 0.0
    longitude: float = 0.0
    
    # Raymarine autopilot data
    pilot_heading: float = 0.0
    pilot_mode: str = "unknown"


class CANLogParser:
    """
    Parser for raw CAN bus log files.
    
    Supports multiple log formats - override for your specific format.
    """
    
    def __init__(self):
        # PGN to field mapping
        self._current_frame = LoggedFrame(timestamp=0)
        
    def parse_file(self, filepath: str) -> Iterator[LoggedFrame]:
        """
        Parse a CAN log file and yield LoggedFrames.
        
        Automatically detects format from file content.
        """
        path = Path(filepath)
        
        # Detect format from first line of file
        detected_format = self._detect_format(filepath)
        
        if detected_format == 'json':
            yield from self._parse_json(filepath)
        elif detected_format == 'csv':
            yield from self._parse_csv(filepath)
        elif detected_format == 'candump':
            yield from self._parse_candump(filepath)
        elif detected_format == 'analyzed_csv':
            yield from self._parse_analyzed_csv(filepath)
        else:
            logger.warning(f"Unknown log format for {path.name}")
            
    def _detect_format(self, filepath: str) -> str:
        """Detect log format from file content."""
        try:
            with open(filepath, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                        
                    # JSON format - starts with {
                    if line.startswith('{'):
                        return 'json'
                        
                    # Standard candump - starts with (timestamp)
                    if line.startswith('(') and ')' in line:
                        return 'candump'
                        
                    # Analyzed CSV format: YYYY-MM-DD-HH:MM:SS.mmm,priority,pgn,...
                    if len(line) > 23 and line[4] == '-' and line[7] == '-' and line[10] == '-':
                        parts = line.split(',')
                        if len(parts) >= 7:
                            return 'analyzed_csv'
                            
                    # Break after first non-empty line
                    break
        except Exception:
            pass
            
        # Fall back to extension-based detection
        path = Path(filepath)
        if path.suffix in ('.json', '.jsonlog'):
            return 'json'
        elif path.suffix == '.csv':
            return 'csv'
        elif path.suffix == '.log':
            return 'candump'
            
        return 'unknown'
            
    def _parse_json(self, filepath: str) -> Iterator[LoggedFrame]:
        """Parse JSON format logs."""
        with open(filepath, 'r') as f:
            for line in f:
                try:
                    record = json.loads(line.strip())
                    frame = self._record_to_frame(record)
                    if frame:
                        yield frame
                except json.JSONDecodeError:
                    continue
                    
    def _parse_csv(self, filepath: str) -> Iterator[LoggedFrame]:
        """Parse CSV format logs."""
        import csv
        with open(filepath, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                frame = self._dict_to_frame(row)
                if frame:
                    yield frame
                    
    def _parse_analyzed_csv(self, filepath: str) -> Iterator[LoggedFrame]:
        """
        Parse analyzed CSV format logs.
        
        Format: YYYY-MM-DD-HH:MM:SS.mmm,priority,pgn,source,dest,len,data_bytes...
        Example: 2018-05-12-14:44:03.893,2,127250,204,255,8,ff,be,07,ff,7f,ff,7f,fd
        """
        from datetime import datetime
        
        current_data: Dict[str, Any] = {}
        last_emit_time = 0.0
        emit_interval = 0.1  # 10Hz
        base_timestamp = None
        
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                    
                try:
                    parts = line.split(',')
                    if len(parts) < 7:
                        continue
                        
                    # Parse timestamp: YYYY-MM-DD-HH:MM:SS.mmm
                    timestamp_str = parts[0]
                    try:
                        dt = datetime.strptime(timestamp_str[:23], "%Y-%m-%d-%H:%M:%S.%f")
                        timestamp = dt.timestamp()
                    except ValueError:
                        continue
                        
                    if base_timestamp is None:
                        base_timestamp = timestamp
                        
                    # Parse PGN directly (already decoded in analyzed format)
                    pgn = int(parts[2])
                    
                    # Parse data bytes
                    data_bytes = []
                    for i in range(6, len(parts)):
                        try:
                            data_bytes.append(int(parts[i], 16))
                        except ValueError:
                            break
                    data = bytes(data_bytes)
                    
                    # Decode PGN data directly (no need to extract from CAN ID)
                    self._decode_pgn_data(pgn, data, current_data, timestamp)
                    
                    # Emit at target rate
                    if timestamp - last_emit_time >= emit_interval:
                        frame = self._data_to_frame(current_data, timestamp)
                        if frame:
                            yield frame
                        last_emit_time = timestamp
                        
                except (ValueError, IndexError):
                    continue
                    
    def _decode_pgn_data(self, pgn: int, data: bytes, 
                         current_data: Dict, timestamp: float):
        """Decode PGN data directly (for analyzed CSV format)."""
        current_data['timestamp'] = timestamp
        
        # Wind
        if pgn == 130306:
            if len(data) >= 6:
                speed_raw = struct.unpack('<H', data[1:3])[0]
                angle_raw = struct.unpack('<H', data[3:5])[0]
                reference = data[5] & 0x07
                
                if reference == 2:  # Apparent
                    if speed_raw != 0xFFFF:
                        current_data['aws'] = speed_raw * 0.01 * 1.94384
                    if angle_raw != 0xFFFF:
                        awa = angle_raw * 0.0001 * 180 / 3.14159
                        if awa > 180:
                            awa -= 360
                        current_data['awa'] = awa
                        
        # Speed
        elif pgn == 128259:
            if len(data) >= 4:
                stw_raw = struct.unpack('<H', data[1:3])[0]
                if stw_raw != 0xFFFF:
                    current_data['stw'] = stw_raw * 0.01 * 1.94384
                    
        # COG/SOG
        elif pgn == 129026:
            if len(data) >= 6:
                cog_raw = struct.unpack('<H', data[2:4])[0]
                sog_raw = struct.unpack('<H', data[4:6])[0]
                if cog_raw != 0xFFFF:
                    current_data['cog'] = cog_raw * 0.0001 * 180 / 3.14159
                if sog_raw != 0xFFFF:
                    current_data['sog'] = sog_raw * 0.01 * 1.94384
                    
        # Heading
        elif pgn == 127250:
            if len(data) >= 4:
                heading_raw = struct.unpack('<H', data[1:3])[0]
                if heading_raw != 0xFFFF:
                    current_data['heading'] = heading_raw * 0.0001 * 180 / 3.14159
                    
        # Rudder
        elif pgn == 127245:
            if len(data) >= 6:
                position_raw = struct.unpack('<h', data[4:6])[0]
                if position_raw != 0x7FFF:
                    current_data['rudder_angle'] = position_raw * 0.0001 * 180 / 3.14159
                    
        # Engine RPM
        elif pgn == 127488:
            if len(data) >= 4:
                rpm_raw = struct.unpack('<H', data[1:3])[0]
                if rpm_raw != 0xFFFF:
                    current_data['engine_rpm'] = rpm_raw * 0.25
                    
        # Position
        elif pgn == 129025:
            if len(data) >= 8:
                lat_raw = struct.unpack('<i', data[0:4])[0]
                lon_raw = struct.unpack('<i', data[4:8])[0]
                if lat_raw != 0x7FFFFFFF:
                    current_data['latitude'] = lat_raw * 1e-7
                if lon_raw != 0x7FFFFFFF:
                    current_data['longitude'] = lon_raw * 1e-7
                    
        # Rate of Turn
        elif pgn == 127251:
            if len(data) >= 5:
                rot_raw = struct.unpack('<i', data[1:5])[0]
                if rot_raw != 0x7FFFFFFF:
                    current_data['yaw_rate'] = rot_raw * 3.125e-8 * 180 / 3.14159
                    
        # Attitude
        elif pgn == 127257:
            if len(data) >= 7:
                pitch_raw = struct.unpack('<h', data[3:5])[0]
                roll_raw = struct.unpack('<h', data[5:7])[0]
                if pitch_raw != 0x7FFF:
                    current_data['pitch'] = pitch_raw * 0.0001 * 180 / 3.14159
                if roll_raw != 0x7FFF:
                    current_data['roll'] = roll_raw * 0.0001 * 180 / 3.14159
                    
        # Seatalk Pilot Heading
        elif pgn == 65359:
            if len(data) >= 8:
                heading_raw = struct.unpack('<H', data[5:7])[0]
                if heading_raw != 0xFFFF:
                    current_data['pilot_heading'] = heading_raw * 0.0001 * 180 / 3.14159
                    
    def _parse_candump(self, filepath: str) -> Iterator[LoggedFrame]:
        """
        Parse candump -L format logs.
        
        Format: (timestamp) interface candid#data
        Example: (1234567890.123456) can0 19F51200#FFFFFFFFFFFF00FF
        """
        current_data: Dict[str, Any] = {}
        last_emit_time = 0.0
        emit_interval = 0.1  # 10Hz
        
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or not line.startswith('('):
                    continue
                    
                try:
                    # Parse candump format
                    parts = line.split()
                    timestamp = float(parts[0].strip('()'))
                    can_data = parts[2] if len(parts) > 2 else ""
                    
                    if '#' in can_data:
                        can_id_hex, data_hex = can_data.split('#')
                        can_id = int(can_id_hex, 16)
                        data = bytes.fromhex(data_hex)
                        
                        # Decode PGN and update current data
                        self._decode_can_frame(can_id, data, current_data, timestamp)
                        
                    # Emit at target rate
                    if timestamp - last_emit_time >= emit_interval:
                        frame = self._data_to_frame(current_data, timestamp)
                        if frame:
                            yield frame
                        last_emit_time = timestamp
                        
                except (ValueError, IndexError) as e:
                    continue
                    
    def _decode_can_frame(self, can_id: int, data: bytes, 
                          current_data: Dict, timestamp: float):
        """Decode CAN frame and update current data dict."""
        # Extract PGN from CAN ID
        pdu_format = (can_id >> 16) & 0xFF
        pdu_specific = (can_id >> 8) & 0xFF
        
        if pdu_format < 240:
            pgn = pdu_format << 8
        else:
            pgn = (pdu_format << 8) | pdu_specific
            
        if (can_id >> 24) & 0x01:
            pgn |= 0x10000
            
        current_data['timestamp'] = timestamp
            
        # Decode known PGNs
        if pgn == 130306:  # Wind
            if len(data) >= 6:
                speed_raw = struct.unpack('<H', data[1:3])[0]
                angle_raw = struct.unpack('<H', data[3:5])[0]
                reference = data[5] & 0x07
                
                if reference == 2:  # Apparent
                    if speed_raw != 0xFFFF:
                        current_data['aws'] = speed_raw * 0.01 * 1.94384
                    if angle_raw != 0xFFFF:
                        awa = angle_raw * 0.0001 * 180 / 3.14159
                        if awa > 180:
                            awa -= 360
                        current_data['awa'] = awa
                        
        elif pgn == 128259:  # Speed
            if len(data) >= 4:
                stw_raw = struct.unpack('<H', data[1:3])[0]
                if stw_raw != 0xFFFF:
                    current_data['stw'] = stw_raw * 0.01 * 1.94384
                    
        elif pgn == 129026:  # COG/SOG
            if len(data) >= 6:
                cog_raw = struct.unpack('<H', data[2:4])[0]
                sog_raw = struct.unpack('<H', data[4:6])[0]
                if cog_raw != 0xFFFF:
                    current_data['cog'] = cog_raw * 0.0001 * 180 / 3.14159
                if sog_raw != 0xFFFF:
                    current_data['sog'] = sog_raw * 0.01 * 1.94384
                    
        elif pgn == 127250:  # Heading
            if len(data) >= 4:
                heading_raw = struct.unpack('<H', data[1:3])[0]
                if heading_raw != 0xFFFF:
                    current_data['heading'] = heading_raw * 0.0001 * 180 / 3.14159
                    
        elif pgn == 127245:  # Rudder
            if len(data) >= 6:
                position_raw = struct.unpack('<h', data[4:6])[0]
                if position_raw != 0x7FFF:
                    current_data['rudder_angle'] = position_raw * 0.0001 * 180 / 3.14159
                    
        elif pgn == 127488:  # Engine Parameters, Rapid Update
            if len(data) >= 4:
                # Byte 0: Engine instance
                # Bytes 1-2: Engine speed (RPM * 0.25)
                rpm_raw = struct.unpack('<H', data[1:3])[0]
                if rpm_raw != 0xFFFF:
                    current_data['engine_rpm'] = rpm_raw * 0.25
                    
        elif pgn == 127489:  # Engine Parameters, Dynamic
            if len(data) >= 8:
                # Byte 0: Engine instance
                # Bytes 3-4: Engine temperature (K * 0.01)
                # Bytes 5-6: Alternator potential (V * 0.01)
                # Skip to temperature at offset 3-4
                temp_raw = struct.unpack('<H', data[3:5])[0]
                if temp_raw != 0xFFFF:
                    # Convert from K * 0.01 to Celsius
                    current_data['engine_temp'] = (temp_raw * 0.01) - 273.15
                    
        elif pgn == 129025:  # Position, Rapid Update
            if len(data) >= 8:
                # Bytes 0-3: Latitude (degrees * 1e-7)
                # Bytes 4-7: Longitude (degrees * 1e-7)
                lat_raw = struct.unpack('<i', data[0:4])[0]
                lon_raw = struct.unpack('<i', data[4:8])[0]
                if lat_raw != 0x7FFFFFFF:
                    current_data['latitude'] = lat_raw * 1e-7
                if lon_raw != 0x7FFFFFFF:
                    current_data['longitude'] = lon_raw * 1e-7
                    
        elif pgn == 127251:  # Rate of Turn
            if len(data) >= 5:
                # Byte 0: SID
                # Bytes 1-4: Rate of turn (rad/s * 3.125e-8, signed)
                rot_raw = struct.unpack('<i', data[1:5])[0]
                if rot_raw != 0x7FFFFFFF:
                    # Convert to degrees per second
                    current_data['yaw_rate'] = rot_raw * 3.125e-8 * 180 / 3.14159
                    
        elif pgn == 127257:  # Attitude
            if len(data) >= 7:
                # Byte 0: SID
                # Bytes 1-2: Yaw (not used, use heading instead)
                # Bytes 3-4: Pitch (rad * 0.0001)
                # Bytes 5-6: Roll (rad * 0.0001)
                pitch_raw = struct.unpack('<h', data[3:5])[0]
                roll_raw = struct.unpack('<h', data[5:7])[0]
                if pitch_raw != 0x7FFF:
                    current_data['pitch'] = pitch_raw * 0.0001 * 180 / 3.14159
                if roll_raw != 0x7FFF:
                    current_data['roll'] = roll_raw * 0.0001 * 180 / 3.14159
                    
        elif pgn == 65359:  # Seatalk: Pilot Heading (Raymarine proprietary)
            if len(data) >= 8:
                # Bytes 0-1: Manufacturer code (Raymarine = 0x9F3B)
                # Bytes 5-6: Heading magnetic (rad * 0.0001)
                heading_raw = struct.unpack('<H', data[5:7])[0]
                if heading_raw != 0xFFFF:
                    current_data['pilot_heading'] = heading_raw * 0.0001 * 180 / 3.14159
                    
    def _data_to_frame(self, data: Dict, timestamp: float) -> Optional[LoggedFrame]:
        """Convert accumulated data dict to LoggedFrame."""
        return LoggedFrame(
            timestamp=timestamp,
            heading=data.get('heading', 0),
            pitch=data.get('pitch', 0),
            roll=data.get('roll', 0),
            yaw_rate=data.get('yaw_rate', 0),
            awa=data.get('awa', 0),
            aws=data.get('aws', 0),
            stw=data.get('stw', 0),
            cog=data.get('cog', 0),
            sog=data.get('sog', 0),
            rudder_angle=data.get('rudder_angle', 0),
            target_heading=data.get('target_heading', data.get('heading', 0)),
            mode=data.get('mode', 'unknown'),
            engine_rpm=data.get('engine_rpm', 0),
            engine_hours=data.get('engine_hours', 0),
            engine_temp=data.get('engine_temp', 0),
            latitude=data.get('latitude', 0),
            longitude=data.get('longitude', 0),
            pilot_heading=data.get('pilot_heading', 0),
            pilot_mode=data.get('pilot_mode', 'unknown'),
        )
        
    def _record_to_frame(self, record: dict) -> Optional[LoggedFrame]:
        """Convert JSON record to LoggedFrame."""
        try:
            return LoggedFrame(
                timestamp=record.get('timestamp', time.time()),
                heading=record.get('heading', 0),
                pitch=record.get('pitch', 0),
                roll=record.get('roll', 0),
                yaw_rate=record.get('yaw_rate', 0),
                awa=record.get('awa', 0),
                aws=record.get('aws', 0),
                stw=record.get('stw', 0),
                cog=record.get('cog', 0),
                sog=record.get('sog', 0),
                rudder_angle=record.get('rudder_angle', 0),
                target_heading=record.get('target_heading', 0),
                target_awa=record.get('target_awa', 0),
                target_twa=record.get('target_twa', 0),
                mode=record.get('mode', 'unknown'),
                engine_rpm=record.get('engine_rpm', 0),
                engine_hours=record.get('engine_hours', 0),
                engine_temp=record.get('engine_temp', 0),
                latitude=record.get('latitude', 0),
                longitude=record.get('longitude', 0),
                pilot_heading=record.get('pilot_heading', 0),
                pilot_mode=record.get('pilot_mode', 'unknown'),
            )
        except Exception:
            return None
            
    def _dict_to_frame(self, row: dict) -> Optional[LoggedFrame]:
        """Convert CSV row to LoggedFrame."""
        return self._record_to_frame(row)


@dataclass
class MetadataSegment:
    """A segment from metadata file."""
    start_time: float
    end_time: float
    operation_mode: str
    steering_mode: str
    target_value: float
    confidence: float = 0.0
    notes: str = ""


class TrainingDataLoader:
    """
    Loads and prepares training data from log files.
    
    Creates (state_sequence, rudder_command) pairs for supervised learning.
    Uses metadata files when available to set mode and target values.
    """
    
    def __init__(self, config: Optional[DataConfig] = None):
        self.config = config or DataConfig()
        self._parser = CANLogParser()
        
    def _load_metadata(self, filepath: str) -> Optional[List[MetadataSegment]]:
        """
        Load metadata from .meta.json file if it exists.
        
        Returns list of segments or None if no metadata file exists.
        """
        meta_path = Path(filepath).with_suffix('.meta.json')
        
        if not meta_path.exists():
            return None
            
        try:
            with open(meta_path, 'r') as f:
                data = json.load(f)
                
            segments = []
            for seg_data in data.get('segments', []):
                segment = MetadataSegment(
                    start_time=seg_data['start_time'],
                    end_time=seg_data['end_time'],
                    operation_mode=seg_data['operation_mode'],
                    steering_mode=seg_data['steering_mode'],
                    target_value=seg_data['target_value'],
                    confidence=seg_data.get('confidence', 0.0),
                    notes=seg_data.get('notes', ''),
                )
                segments.append(segment)
                
            logger.info(f"Loaded {len(segments)} segments from {meta_path.name}")
            return segments
            
        except (json.JSONDecodeError, KeyError) as e:
            logger.warning(f"Failed to parse metadata {meta_path}: {e}")
            return None
            
    def _apply_metadata(self, frames: List[LoggedFrame], 
                        segments: List[MetadataSegment]) -> List[LoggedFrame]:
        """
        Apply metadata segments to frames, setting mode and target values.
        
        Filters out frames that are not usable for training (anchor, unknown).
        """
        if not segments:
            return frames
            
        updated_frames = []
        
        for frame in frames:
            # Find matching segment for this frame's timestamp
            matching_segment = None
            for seg in segments:
                if seg.start_time <= frame.timestamp < seg.end_time:
                    matching_segment = seg
                    break
                    
            if matching_segment:
                # Skip anchor and unknown operation modes
                if matching_segment.operation_mode in ('anchor', 'unknown'):
                    continue
                    
                # Map steering mode to training mode
                if matching_segment.steering_mode == 'heading':
                    frame.mode = 'compass'
                    frame.target_heading = matching_segment.target_value
                elif matching_segment.steering_mode == 'awa':
                    frame.mode = 'wind_awa'
                    # For AWA mode, target_heading is computed from current heading + target AWA
                    # But we store the target AWA in target_heading for feature engineering
                    frame.target_heading = matching_segment.target_value
                elif matching_segment.steering_mode == 'twa':
                    frame.mode = 'wind_twa'
                    frame.target_heading = matching_segment.target_value
                else:
                    # No steering mode - skip
                    continue
                    
                updated_frames.append(frame)
            else:
                # No matching segment - include with default mode
                updated_frames.append(frame)
                
        return updated_frames
        
    def load_directory(self, directory: str) -> Tuple[np.ndarray, np.ndarray]:
        """
        Load all log files from a directory.
        
        Returns:
            (X, y): Training data and labels
            X shape: [n_samples, sequence_length, feature_dim]
            y shape: [n_samples, 1]
        """
        all_X = []
        all_y = []
        
        path = Path(directory)
        for log_file in path.glob('**/*'):
            if log_file.is_file() and log_file.suffix in ['.json', '.jsonlog', '.csv', '.log']:
                try:
                    X, y = self.load_file(str(log_file))
                    if len(X) > 0:
                        all_X.append(X)
                        all_y.append(y)
                        logger.info(f"Loaded {len(X)} sequences from {log_file.name}")
                except Exception as e:
                    logger.warning(f"Failed to load {log_file}: {e}")
                    
        if not all_X:
            return np.array([]), np.array([])
            
        X = np.concatenate(all_X, axis=0)
        y = np.concatenate(all_y, axis=0)
        
        if self.config.shuffle:
            indices = np.random.permutation(len(X))
            X = X[indices]
            y = y[indices]
            
        logger.info(f"Total: {len(X)} training sequences")
        return X, y
        
    def load_file(self, filepath: str) -> Tuple[np.ndarray, np.ndarray]:
        """
        Load a single log file.
        
        If a .meta.json file exists alongside the log file, it will be used
        to set mode and target values for each frame.
        
        Returns:
            (X, y): Sequences and labels
        """
        frames = list(self._parser.parse_file(filepath))
        
        if len(frames) < self.config.sequence_length + 1:
            return np.array([]), np.array([])
            
        # Load and apply metadata if available
        metadata = self._load_metadata(filepath)
        if metadata:
            frames = self._apply_metadata(frames, metadata)
            
        if len(frames) < self.config.sequence_length + 1:
            return np.array([]), np.array([])
        
        # Mark frames in mode transition periods for filtering
        # After a mode change, the helm controller needs time to stabilize
        transition_mask = self._compute_transition_mask(frames)
        
        # Convert frames to feature sequences
        X_list = []
        y_list = []
        
        skipped_transitions = 0
        for i in range(len(frames) - self.config.sequence_length):
            # Check if any frame in the sequence or label is in transition
            seq_end = i + self.config.sequence_length + 1  # Include label frame
            if any(transition_mask[i:seq_end]):
                skipped_transitions += 1
                continue
                
            sequence = frames[i:i + self.config.sequence_length]
            label_frame = frames[i + self.config.sequence_length]
            
            # Build feature array for sequence
            features = np.zeros((self.config.sequence_length, self.config.feature_dim))
            
            for j, frame in enumerate(sequence):
                features[j] = self._frame_to_features(frame, sequence[0])
                
            # Label is the rudder position (normalized to 25° max)
            label = label_frame.rudder_angle / 25.0  # Normalize to [-1, 1]
            label = np.clip(label, -1.0, 1.0)
            
            X_list.append(features)
            y_list.append([label])
        
        if skipped_transitions > 0:
            logger.info(f"Filtered {skipped_transitions} sequences in mode transition periods")
            
        return np.array(X_list), np.array(y_list)
    
    def _compute_transition_mask(self, frames: List[LoggedFrame]) -> List[bool]:
        """
        Compute mask indicating frames in mode transition periods.
        
        After a mode change, the helm controller PD response creates transient
        behavior where error/rudder relationships are dominated by derivative
        damping rather than proportional control. We filter these periods to
        give the ML model cleaner training signal.
        
        Returns:
            List of booleans, True if frame is in transition period
        """
        if len(frames) == 0:
            return []
            
        mask = [False] * len(frames)
        filter_seconds = self.config.mode_transition_filter_s
        
        if filter_seconds <= 0:
            return mask  # No filtering
            
        # Find mode changes and mark transition periods
        current_mode = frames[0].mode
        last_change_time = frames[0].timestamp - filter_seconds - 1  # Before any data
        
        for i, frame in enumerate(frames):
            if frame.mode != current_mode:
                # Mode changed - start new transition period
                current_mode = frame.mode
                last_change_time = frame.timestamp
            
            # Mark if within transition period
            if frame.timestamp - last_change_time < filter_seconds:
                mask[i] = True
                
        return mask
        
    def _frame_to_features(self, frame: LoggedFrame, 
                           ref_frame: LoggedFrame) -> np.ndarray:
        """Convert a logged frame to normalized features.
        
        ARCHITECTURE: Feature[0] is ALWAYS heading error (HelmController convention).
        error = computed_heading - heading (target - current)
        Positive error = target to starboard → positive rudder.
        In wind modes, computed_heading is derived from the wind target.
        """
        features = np.zeros(self.config.feature_dim)
        
        # Compute TWA for use in wind modes
        twa, _ = self._compute_true_wind(frame.awa, frame.aws, frame.stw)
        
        # ARCHITECTURE: Compute target heading from mode-specific target
        # Wind targets are converted to heading targets so feature[0] is always heading error.
        # This matches passage_simulator._compute_features() and ModeManager.update()
        if frame.mode == "compass":
            # Direct heading target
            computed_heading = frame.target_heading
        elif frame.mode == "wind_awa":
            # Compute heading that would achieve target AWA
            # If awa > target_awa: need to head up → increase heading → computed > heading
            awa_delta = frame.awa - frame.target_awa
            computed_heading = frame.heading + awa_delta
        elif frame.mode in ["wind_twa", "vmg"]:
            # Compute heading that would achieve target TWA
            twa_delta = twa - frame.target_twa
            computed_heading = frame.heading + twa_delta
        else:
            # Default to compass mode behavior
            computed_heading = frame.target_heading
            
        # Normalize computed_heading to [0, 360)
        while computed_heading < 0:
            computed_heading += 360
        while computed_heading >= 360:
            computed_heading -= 360
        
        # FEAT_ERROR (0): ALWAYS heading error (HelmController convention)
        # error = computed_heading - heading (target - current)
        # Positive error = target to starboard → positive rudder
        error = self._angle_diff(computed_heading, frame.heading)
        features[0] = error / 180.0
        
        # Skip integral for now (would need state tracking)
        features[1] = 0.0
        
        # Heading rate
        features[2] = frame.yaw_rate / 30.0
        
        # Roll, pitch
        features[3] = frame.roll / 45.0
        features[4] = frame.pitch / 30.0
        
        # Roll rate (would need computation)
        features[5] = 0.0
        
        # AWA, AWS
        features[6] = frame.awa / 180.0
        features[7] = 0.0  # AWA rate
        features[8] = frame.aws / 60.0
        
        # TWA, TWS (computed from AWA/AWS/STW)
        twa, tws = self._compute_true_wind(frame.awa, frame.aws, frame.stw)
        features[9] = twa / 180.0
        features[10] = tws / 60.0
        
        # Speed
        features[11] = frame.stw / 25.0
        features[12] = frame.sog / 25.0
        
        # COG error (relative to compass target heading)
        cog_error = self._angle_diff(frame.cog, frame.target_heading)
        features[13] = cog_error / 180.0
        
        # Rudder position (normalized to 25° max)
        features[14] = frame.rudder_angle / 25.0
        features[15] = 0.0  # Rudder velocity
        
        # Target angle (16): The computed target heading (always heading-based)
        features[16] = computed_heading / 180.0
        
        # VMG (computed)
        vmg_up = frame.stw * np.cos(np.radians(abs(twa))) if frame.stw > 0 else 0
        vmg_down = frame.stw * np.cos(np.radians(180 - abs(twa))) if frame.stw > 0 else 0
        features[17] = vmg_up / 15.0
        features[18] = vmg_down / 20.0
        
        # Polar target (would need polar lookup)
        features[19] = 0.5  # Placeholder
        features[20] = 0.8  # Placeholder performance
        
        # Mode flags
        features[21] = 1.0 if frame.mode == "compass" else 0.0
        features[22] = 1.0 if frame.mode == "wind_awa" else 0.0
        features[23] = 1.0 if frame.mode in ["wind_twa", "vmg"] else 0.0
        
        # Wave period (placeholder)
        features[24] = 5.0 / 15.0
        
        return np.clip(features, -1.0, 1.0)
        
    def _angle_diff(self, a: float, b: float) -> float:
        """Compute signed angle difference."""
        diff = a - b
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff
        
    def _compute_true_wind(self, awa: float, aws: float, stw: float) -> Tuple[float, float]:
        """Compute true wind from apparent wind."""
        awa_rad = np.radians(awa)
        aw_x = aws * np.cos(awa_rad) - stw
        aw_y = aws * np.sin(awa_rad)
        
        tws = np.sqrt(aw_x**2 + aw_y**2)
        twa = np.degrees(np.arctan2(aw_y, aw_x))
        
        return twa, tws
        
    def split_data(self, X: np.ndarray, y: np.ndarray
                   ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Split data into train and validation sets."""
        split_idx = int(len(X) * self.config.train_split)
        
        X_train = X[:split_idx]
        y_train = y[:split_idx]
        X_val = X[split_idx:]
        y_val = y[split_idx:]
        
        logger.info(f"Train: {len(X_train)}, Validation: {len(X_val)}")
        return X_train, y_train, X_val, y_val


# PyTorch Dataset wrapper (optional, for custom data loading)
try:
    import torch
    from torch.utils.data import Dataset
    
    class AutopilotDataset(Dataset):
        """
        PyTorch Dataset wrapper for autopilot training data.
        
        Converts NumPy arrays to PyTorch tensors for use with DataLoader.
        """
        
        def __init__(self, X: np.ndarray, y: np.ndarray):
            """
            Initialize dataset.
            
            Args:
                X: Input sequences of shape [n_samples, sequence_length, feature_dim]
                y: Target labels of shape [n_samples, 1] or [n_samples]
            """
            self.X = torch.from_numpy(X).float()
            self.y = torch.from_numpy(y).float()
            if self.y.ndim == 1:
                self.y = self.y.unsqueeze(1)
        
        def __len__(self) -> int:
            return len(self.X)
        
        def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
            return self.X[idx], self.y[idx]
    
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False
    AutopilotDataset = None
