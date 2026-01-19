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
    mode: str = "unknown"


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
        
        Override this method for your specific log format.
        """
        # Try to detect format from extension
        path = Path(filepath)
        
        if path.suffix == '.json':
            yield from self._parse_json(filepath)
        elif path.suffix == '.csv':
            yield from self._parse_csv(filepath)
        elif path.suffix == '.log':
            yield from self._parse_candump(filepath)
        else:
            logger.warning(f"Unknown log format: {path.suffix}")
            
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
                    
    def _data_to_frame(self, data: Dict, timestamp: float) -> Optional[LoggedFrame]:
        """Convert accumulated data dict to LoggedFrame."""
        return LoggedFrame(
            timestamp=timestamp,
            heading=data.get('heading', 0),
            awa=data.get('awa', 0),
            aws=data.get('aws', 0),
            stw=data.get('stw', 0),
            cog=data.get('cog', 0),
            sog=data.get('sog', 0),
            rudder_angle=data.get('rudder_angle', 0),
            target_heading=data.get('target_heading', data.get('heading', 0))
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
                mode=record.get('mode', 'unknown')
            )
        except Exception:
            return None
            
    def _dict_to_frame(self, row: dict) -> Optional[LoggedFrame]:
        """Convert CSV row to LoggedFrame."""
        return self._record_to_frame(row)


class TrainingDataLoader:
    """
    Loads and prepares training data from log files.
    
    Creates (state_sequence, rudder_command) pairs for supervised learning.
    """
    
    def __init__(self, config: Optional[DataConfig] = None):
        self.config = config or DataConfig()
        self._parser = CANLogParser()
        
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
            if log_file.is_file() and log_file.suffix in ['.json', '.csv', '.log']:
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
        
        Returns:
            (X, y): Sequences and labels
        """
        frames = list(self._parser.parse_file(filepath))
        
        if len(frames) < self.config.sequence_length + 1:
            return np.array([]), np.array([])
            
        # Convert frames to feature sequences
        X_list = []
        y_list = []
        
        for i in range(len(frames) - self.config.sequence_length):
            sequence = frames[i:i + self.config.sequence_length]
            label_frame = frames[i + self.config.sequence_length]
            
            # Build feature array for sequence
            features = np.zeros((self.config.sequence_length, self.config.feature_dim))
            
            for j, frame in enumerate(sequence):
                features[j] = self._frame_to_features(frame, sequence[0])
                
            # Label is the rudder position (normalized)
            label = label_frame.rudder_angle / 30.0  # Normalize to [-1, 1]
            label = np.clip(label, -1.0, 1.0)
            
            X_list.append(features)
            y_list.append([label])
            
        return np.array(X_list), np.array(y_list)
        
    def _frame_to_features(self, frame: LoggedFrame, 
                           ref_frame: LoggedFrame) -> np.ndarray:
        """Convert a logged frame to normalized features."""
        features = np.zeros(self.config.feature_dim)
        
        # Heading error (using target_heading if available)
        heading_error = self._angle_diff(frame.heading, frame.target_heading)
        features[0] = heading_error / 180.0
        
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
        
        # COG error
        cog_error = self._angle_diff(frame.cog, frame.target_heading)
        features[13] = cog_error / 180.0
        
        # Rudder position
        features[14] = frame.rudder_angle / 30.0
        features[15] = 0.0  # Rudder velocity
        
        # Target angle
        features[16] = frame.target_heading / 180.0
        
        # VMG (computed)
        vmg_up = frame.stw * np.cos(np.radians(abs(twa))) if frame.stw > 0 else 0
        vmg_down = frame.stw * np.cos(np.radians(180 - abs(twa))) if frame.stw > 0 else 0
        features[17] = vmg_up / 15.0
        features[18] = vmg_down / 20.0
        
        # Polar target (would need polar lookup)
        features[19] = 0.5  # Placeholder
        features[20] = 0.8  # Placeholder performance
        
        # Mode flags (assume compass mode for historical data)
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
