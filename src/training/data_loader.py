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

FEATURE_DIM = 22

# Features whose sign flips when mirroring port ↔ starboard.
MIRROR_NEGATE_FEATURES = [
    0,   # heading_error
    # 1 = mode flag (not negated)
    2,   # heading_rate
    3,   # roll (heel flips)
    5,   # roll_rate
    6,   # AWA
    7,   # AWA_rate
    9,   # TWA
    13,  # COG_error
    14,  # rudder_position
    15,  # rudder_velocity
    19,  # PD suggestion (flips with error and rate)
]


def _angle_diff(a: float, b: float) -> float:
    """Compute signed angle difference a - b, normalized to [-180, 180]."""
    diff = a - b
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    return diff


def _compute_true_wind(awa: float, aws: float, stw: float) -> Tuple[float, float]:
    """Compute true wind angle and speed from apparent wind and boat speed."""
    awa_rad = np.radians(awa)
    aw_x = aws * np.cos(awa_rad) - stw
    aw_y = aws * np.sin(awa_rad)
    tws = np.sqrt(aw_x**2 + aw_y**2)
    twa = np.degrees(np.arctan2(aw_y, aw_x))
    return twa, tws


def compute_features(heading: float, pitch: float, roll: float,
                     yaw_rate: float, awa: float, aws: float,
                     stw: float, cog: float, sog: float,
                     rudder_angle: float, target_heading: float,
                     target_awa: float, target_twa: float,
                     mode: str, *,
                     roll_rate: float = 0.0,
                     awa_rate: float = 0.0,
                     rudder_velocity: float = 0.0,
                     wave_period: float = 0.0) -> np.ndarray:
    """Compute the 22-element normalized feature vector from raw sensor values.

    ARCHITECTURE: The model always steers to a computed compass heading.
    For wind modes, the target wind angle is converted to a target TWA
    (AWA is converted via the wind triangle first), then the heading
    delta is computed from TWA.  The model never steers to a wind angle
    directly; TWA/TWS/STW are informational context features only.

    This is the single source of truth for feature computation.  It is called
    by the binary frame writer, the passage simulator, and the CL validator.

    Returns:
        np.ndarray of shape (22,) with values clipped to [-1, 1].
    """
    features = np.zeros(FEATURE_DIM, dtype=np.float32)

    twa, tws = _compute_true_wind(awa, aws, stw)

    # Compute target heading -- ALL modes reduce to a compass heading.
    if mode == "compass":
        computed_heading = target_heading
    elif mode == "wind_awa":
        # Convert target AWA → target TWA via the wind triangle, then
        # compute heading delta from TWA (same as TWA mode below).
        target_twa_from_awa, _ = _compute_true_wind(target_awa, aws, stw)
        twa_delta = twa - target_twa_from_awa
        computed_heading = heading + twa_delta
    elif mode in ("wind_twa", "vmg"):
        twa_delta = twa - target_twa
        computed_heading = heading + twa_delta
    else:
        computed_heading = target_heading

    computed_heading %= 360.0

    error = _angle_diff(computed_heading, heading)
    features[0] = np.clip(error / 90.0, -1.0, 1.0)     # ±90° range (primary signal)
    # Mode encoding: compass=0.0, wind_awa=0.5, wind_twa/vmg=1.0
    _MODE_ENC = {"compass": 0.0, "wind_awa": 0.5, "wind_twa": 1.0, "vmg": 1.0}
    features[1] = _MODE_ENC.get(mode, 0.0)
    features[2] = np.clip(yaw_rate / 30.0, -1.0, 1.0)
    features[3] = np.clip(roll / 45.0, -1.0, 1.0)
    features[4] = np.clip(pitch / 30.0, -1.0, 1.0)
    features[5] = np.clip(roll_rate / 30.0, -1.0, 1.0)  # deg/s, same scale as heading_rate
    features[6] = np.clip(awa / 180.0, -1.0, 1.0)
    features[7] = np.clip(awa_rate / 10.0, -1.0, 1.0)   # deg/s
    features[8] = np.clip(aws / 60.0, 0.0, 1.0)
    features[9] = np.clip(twa / 180.0, -1.0, 1.0)
    features[10] = np.clip(tws / 60.0, 0.0, 1.0)
    features[11] = np.clip(stw / 25.0, 0.0, 1.0)
    features[12] = np.clip(sog / 25.0, 0.0, 1.0)

    cog_error = _angle_diff(cog, computed_heading)
    features[13] = np.clip(cog_error / 45.0, -1.0, 1.0) # ±45° range (matches heading_error)
    features[14] = 0.0   # rudder_position zeroed to prevent label leak
    features[15] = np.clip(rudder_velocity / 10.0, -1.0, 1.0)  # deg/s
    features[16] = computed_heading / 360.0  # [0, 1)
    vmg_up = stw * np.cos(np.radians(abs(twa))) if stw > 0 else 0.0
    vmg_down = stw * np.cos(np.radians(180 - abs(twa))) if stw > 0 else 0.0
    features[17] = np.clip(vmg_up / 15.0, -1.0, 1.0)
    features[18] = np.clip(vmg_down / 20.0, -1.0, 1.0)
    # PD suggestion: pre-compute damped control signal from heading error and
    # heading rate using default compass PD gains (kp=1.0, kd=1.5).
    # This bakes in the correct error+rate combination so the model doesn't
    # have to learn the phase relationship from scratch.
    pd_rudder = 1.0 * error + 1.5 * (-yaw_rate)
    features[19] = np.clip(pd_rudder / 25.0, -1.0, 1.0)
    features[20] = 0.0   # polar_performance placeholder (zero, not informative)
    features[21] = np.clip(wave_period / 15.0, 0.0, 1.0)  # seconds, 0-15s range

    return np.clip(features, -1.0, 1.0)


@dataclass
class DataConfig:
    """Configuration for data loading."""
    sequence_length: int = 20      # Timesteps per sequence
    feature_dim: int = 22          # Features per timestep
    sample_rate_hz: float = 2.0    # Target sample rate (matches recording_rate_hz)
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

        X = np.array(X_list)
        y = np.array(y_list)

        # Port/starboard symmetry augmentation: mirror every sample so the
        # model sees equal positive and negative rudder commands.  This
        # prevents directional bias caused by weather helm or predominant
        # tack in the training data.
        if len(X) > 0:
            X_mirror, y_mirror = self._mirror_sequences(X, y)
            X = np.concatenate([X, X_mirror], axis=0)
            y = np.concatenate([y, y_mirror], axis=0)
            logger.info(f"Symmetry augmentation: {len(X_list)} → {len(X)} sequences")

        return X, y
    
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
        
    def _mirror_sequences(self, X: np.ndarray, y: np.ndarray
                          ) -> Tuple[np.ndarray, np.ndarray]:
        """Create port/starboard mirrored copies of all sequences.

        Negates directional features and rudder labels so the model
        learns symmetrical behaviour and cannot develop a tack bias.
        """
        X_m = X.copy()
        y_m = -y
        for idx in MIRROR_NEGATE_FEATURES:
            X_m[:, :, idx] = -X_m[:, :, idx]
        X_m[:, :, 16] = 0.0  # zero computed_heading to prevent mirror leak
        return X_m, y_m

    def _frame_to_features(self, frame: LoggedFrame,
                           ref_frame: LoggedFrame) -> np.ndarray:
        """Convert a logged frame to normalized features.

        Delegates to the module-level compute_features() function.
        """
        return compute_features(
            heading=frame.heading, pitch=frame.pitch, roll=frame.roll,
            yaw_rate=frame.yaw_rate, awa=frame.awa, aws=frame.aws,
            stw=frame.stw, cog=frame.cog, sog=frame.sog,
            rudder_angle=frame.rudder_angle,
            target_heading=frame.target_heading,
            target_awa=frame.target_awa,
            target_twa=frame.target_twa,
            mode=frame.mode,
        )
        
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

    # -----------------------------------------------------------------
    # Binary frame file loading
    # -----------------------------------------------------------------

    @staticmethod
    def load_binary(filepath: str, mmap: bool = True
                    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Load a binary .bin feature file.

        Returns:
            (features, labels, valid_mask)
            features: (N, 25) float32
            labels:   (N,)   float32
            valid_mask: (N,) bool  -- True where frame is NOT in transition
        """
        path = Path(filepath)
        file_size = path.stat().st_size
        from src.simulation.data_generator import _BIN_HEADER_SIZE, _BIN_COLS, _BIN_MAGIC
        data_bytes = file_size - _BIN_HEADER_SIZE
        n_frames = data_bytes // (_BIN_COLS * 4)

        mode = 'r' if mmap else None
        raw = np.memmap(str(path), dtype=np.float32, mode='r',
                        offset=_BIN_HEADER_SIZE,
                        shape=(n_frames, _BIN_COLS)) if mmap else None
        if not mmap:
            with open(path, 'rb') as fh:
                hdr = fh.read(_BIN_HEADER_SIZE)
                if hdr[:4] != _BIN_MAGIC:
                    raise ValueError(f"Not a binary frame file: {path}")
                raw = np.frombuffer(fh.read(), dtype=np.float32).reshape(
                    n_frames, _BIN_COLS)

        features = raw[:, :FEATURE_DIM]
        labels = raw[:, FEATURE_DIM]

        # Build valid_mask from transition_times in companion meta
        valid_mask = np.ones(n_frames, dtype=bool)
        meta_path = path.with_suffix('.meta.json')
        if meta_path.exists():
            with open(meta_path) as f:
                meta = json.load(f)
            transitions = meta.get('transition_times', [])
            if transitions:
                sample_rate = 10.0  # default
                filter_s = 15.0
                for t_time in transitions:
                    start_idx = max(0, int(t_time * sample_rate) - 1)
                    end_idx = min(n_frames,
                                  int((t_time + filter_s) * sample_rate))
                    valid_mask[start_idx:end_idx] = False

        logger.info(f"Loaded binary {path.name}: {n_frames:,} frames, "
                     f"{valid_mask.sum():,} valid")
        return features, labels, valid_mask

    @staticmethod
    def load_binary_directory(directory: str, mmap: bool = True
                              ) -> List[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """Load all .bin files from a directory.

        Returns a list of (features, labels, valid_mask) tuples, one per file.
        Keeping files separate enables per-file train/val splitting.
        """
        path = Path(directory)
        bin_files = sorted(path.glob('**/*.bin'))
        if not bin_files:
            raise FileNotFoundError(f"No .bin files found in {directory}")

        result = []
        for bf in bin_files:
            try:
                result.append(TrainingDataLoader.load_binary(str(bf), mmap=mmap))
            except Exception as e:
                logger.warning(f"Failed to load {bf}: {e}")

        total = sum(f.shape[0] for f, _, _ in result)
        logger.info(f"Loaded {len(result)} binary files, {total:,} total frames")
        return result

    def preprocess_to_disk(self, directory: str, output_dir: str) -> Dict[str, Any]:
        """
        Preprocess log files to memory-mappable .npy files on disk.
        
        This avoids loading all training data into RAM at once. Each log file
        is processed into a pair of .npy files (X_sequences, y_labels) that
        can be memory-mapped during training.
        
        Args:
            directory: Directory containing log files
            output_dir: Directory for preprocessed .npy files
            
        Returns:
            Dict with preprocessing stats and manifest info
        """
        out_path = Path(output_dir)
        out_path.mkdir(parents=True, exist_ok=True)
        
        src_path = Path(directory)
        log_files = sorted([
            f for f in src_path.glob('**/*')
            if f.is_file() and f.suffix in ['.json', '.jsonlog', '.csv', '.log']
        ])
        
        if not log_files:
            raise ValueError(f"No log files found in {directory}")
        
        manifest = {
            'source_dir': str(directory),
            'config': {
                'sequence_length': self.config.sequence_length,
                'feature_dim': self.config.feature_dim,
                'sample_rate_hz': self.config.sample_rate_hz,
                'mode_transition_filter_s': self.config.mode_transition_filter_s,
            },
            'files': [],
            'total_sequences': 0,
        }
        
        for log_file in log_files:
            try:
                X, y = self.load_file(str(log_file))
                if len(X) == 0:
                    continue
                    
                # Save as .npy files
                stem = log_file.stem
                x_path = out_path / f"{stem}_X.npy"
                y_path = out_path / f"{stem}_y.npy"
                
                np.save(x_path, X.astype(np.float32))
                np.save(y_path, y.astype(np.float32))
                
                file_info = {
                    'source': str(log_file),
                    'x_file': x_path.name,
                    'y_file': y_path.name,
                    'num_sequences': len(X),
                }
                manifest['files'].append(file_info)
                manifest['total_sequences'] += len(X)
                
                logger.info(f"Preprocessed {log_file.name}: {len(X):,} sequences")
                
            except Exception as e:
                logger.warning(f"Failed to preprocess {log_file}: {e}")
        
        # Save manifest
        manifest_path = out_path / 'manifest.json'
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)
        
        logger.info(f"Preprocessed {len(manifest['files'])} files, "
                     f"{manifest['total_sequences']:,} total sequences -> {output_dir}")
        
        return manifest


# PyTorch Dataset wrappers
try:
    import torch
    from torch.utils.data import Dataset
    
    class AutopilotDataset(Dataset):
        """
        PyTorch Dataset wrapper for autopilot training data.
        
        Converts NumPy arrays to PyTorch tensors for use with DataLoader.
        """
        
        def __init__(self, X: np.ndarray, y: np.ndarray):
            self.X = torch.from_numpy(X).float()
            self.y = torch.from_numpy(y).float()
            if self.y.ndim == 1:
                self.y = self.y.unsqueeze(1)
        
        def __len__(self) -> int:
            return len(self.X)
        
        def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
            return self.X[idx], self.y[idx]

    class MemmapDataset(Dataset):
        """
        Memory-mapped dataset for large training data.
        
        Loads preprocessed .npy files using memory mapping so that only the
        active batch occupies RAM. Suitable for datasets that exceed available
        memory.
        
        Usage:
            dataset = MemmapDataset.from_manifest("data/preprocessed/manifest.json")
            loader = DataLoader(dataset, batch_size=64, shuffle=True)
        """
        
        def __init__(self, x_arrays: List[np.ndarray], y_arrays: List[np.ndarray],
                     cumulative_lengths: List[int]):
            self._x_arrays = x_arrays
            self._y_arrays = y_arrays
            self._cumulative = cumulative_lengths
            self._total = cumulative_lengths[-1] if cumulative_lengths else 0
        
        @classmethod
        def from_manifest(cls, manifest_path: str) -> 'MemmapDataset':
            """Load dataset from a preprocessing manifest."""
            manifest_path = Path(manifest_path)
            with open(manifest_path) as f:
                manifest = json.load(f)
            
            base_dir = manifest_path.parent
            x_arrays = []
            y_arrays = []
            cumulative = []
            running_total = 0
            
            for file_info in manifest['files']:
                x_path = base_dir / file_info['x_file']
                y_path = base_dir / file_info['y_file']
                
                x_mmap = np.load(str(x_path), mmap_mode='r')
                y_mmap = np.load(str(y_path), mmap_mode='r')
                
                x_arrays.append(x_mmap)
                y_arrays.append(y_mmap)
                running_total += len(x_mmap)
                cumulative.append(running_total)
            
            logger.info(f"Memory-mapped {len(x_arrays)} files, "
                         f"{running_total:,} total sequences")
            
            return cls(x_arrays, y_arrays, cumulative)
        
        @classmethod
        def from_directory(cls, preprocessed_dir: str) -> 'MemmapDataset':
            """Load from a preprocessed directory containing manifest.json."""
            manifest_path = Path(preprocessed_dir) / 'manifest.json'
            if not manifest_path.exists():
                raise FileNotFoundError(
                    f"No manifest.json in {preprocessed_dir}. "
                    f"Run preprocessing first with --preprocess.")
            return cls.from_manifest(str(manifest_path))
        
        def __len__(self) -> int:
            return self._total
        
        def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
            if idx < 0:
                idx = self._total + idx
            
            # Binary search for the file containing this index
            file_idx = 0
            local_idx = idx
            for i, cum_len in enumerate(self._cumulative):
                if idx < cum_len:
                    file_idx = i
                    local_idx = idx - (self._cumulative[i - 1] if i > 0 else 0)
                    break
            
            x = torch.from_numpy(self._x_arrays[file_idx][local_idx].copy()).float()
            y = torch.from_numpy(self._y_arrays[file_idx][local_idx].copy()).float()
            
            if y.ndim == 0:
                y = y.unsqueeze(0)
            
            return x, y
        
        def split(self, train_fraction: float = 0.8
                  ) -> Tuple['MemmapDataset', 'MemmapDataset']:
            """
            Split into train/validation sets by interleaving files.
            
            Files are dealt round-robin into train and val sets so that
            both sets contain data from all routes/conditions. This avoids
            the overfitting problem where train sees one route and val
            sees another.
            """
            n_files = len(self._x_arrays)
            if n_files < 2:
                raise ValueError(
                    f"Need at least 2 preprocessed files to split, got {n_files}")
            
            # Deal files round-robin: every Nth file goes to val
            # e.g. with 6 files and 80/20 split, val_every=5 means files
            # 4,5 go to val (indices 4,5 from 0-based) -- but round-robin
            # is more even: pick every 5th file for val
            val_every = max(2, round(1.0 / (1.0 - train_fraction)))
            
            train_x, train_y = [], []
            val_x, val_y = [], []
            
            for i in range(n_files):
                if i % val_every == (val_every - 1):
                    val_x.append(self._x_arrays[i])
                    val_y.append(self._y_arrays[i])
                else:
                    train_x.append(self._x_arrays[i])
                    train_y.append(self._y_arrays[i])
            
            # Ensure val is non-empty
            if not val_x:
                val_x.append(train_x.pop())
                val_y.append(train_y.pop())
            
            def _make_cumulative(arrays):
                cum = []
                running = 0
                for a in arrays:
                    running += len(a)
                    cum.append(running)
                return cum
            
            logger.info(f"Split: {len(train_x)} train files, "
                         f"{len(val_x)} val files (interleaved)")
            
            return (MemmapDataset(train_x, train_y, _make_cumulative(train_x)),
                    MemmapDataset(val_x, val_y, _make_cumulative(val_x)))
    
    class FrameDataset(Dataset):
        """On-the-fly sequence construction from flat per-frame feature arrays.

        Instead of pre-building all overlapping (seq_len, feature_dim) windows
        and storing them in RAM, this dataset keeps only the (N, feature_dim)
        feature array and constructs each window at access time.

        Port/starboard mirror augmentation is applied on-the-fly: the second
        half of the index space returns sign-flipped copies, doubling the
        effective dataset size with zero extra memory.
        """

        def __init__(self, features: np.ndarray, labels: np.ndarray,
                     valid_mask: np.ndarray, seq_len: int = 20,
                     mirror: bool = True):
            self._features = features   # (N, 25)
            self._labels = labels       # (N,)
            self._seq_len = seq_len
            self._mirror = mirror
            self._indices = self._build_valid_indices(valid_mask, seq_len)
            self._n_real = len(self._indices)

        @staticmethod
        def _build_valid_indices(valid_mask: np.ndarray,
                                 seq_len: int) -> np.ndarray:
            """Find start positions where all frames in [i, i+seq_len] are valid."""
            n = len(valid_mask)
            if n <= seq_len:
                return np.array([], dtype=np.int64)
            # Sliding window: a start is valid when all seq_len+1 frames are True
            window = seq_len + 1
            cumsum = np.cumsum(np.concatenate(([0], valid_mask.astype(np.int64))))
            counts = cumsum[window:] - cumsum[:n - window + 1]
            return np.nonzero(counts == window)[0]

        @classmethod
        def from_file_list(cls,
                           file_data: 'List[Tuple[np.ndarray, np.ndarray, np.ndarray]]',
                           seq_len: int = 20,
                           mirror: bool = True) -> 'FrameDataset':
            """Create a FrameDataset from multiple binary files.

            Concatenates features/labels/masks while inserting seq_len invalid
            frames at file boundaries to prevent cross-file sequences.
            """
            all_f, all_l, all_m = [], [], []
            gap = np.zeros(seq_len, dtype=bool)
            for i, (f, l, m) in enumerate(file_data):
                if i > 0:
                    all_f.append(np.zeros((seq_len, f.shape[1]), dtype=np.float32))
                    all_l.append(np.zeros(seq_len, dtype=np.float32))
                    all_m.append(gap)
                all_f.append(np.asarray(f))
                all_l.append(np.asarray(l))
                all_m.append(np.asarray(m))
            features = np.concatenate(all_f)
            labels = np.concatenate(all_l)
            mask = np.concatenate(all_m)
            return cls(features, labels, mask, seq_len=seq_len, mirror=mirror)

        def __len__(self) -> int:
            return self._n_real * 2 if self._mirror else self._n_real

        def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
            is_mirror = idx >= self._n_real
            real_idx = idx - self._n_real if is_mirror else idx
            start = self._indices[real_idx]
            x = self._features[start: start + self._seq_len].copy()
            y_val = float(self._labels[start + self._seq_len])
            if is_mirror:
                x[:, MIRROR_NEGATE_FEATURES] *= -1
                x[:, 16] = 0.0  # computed_heading is a compass bearing; zero it
                                 # to prevent leaking real-vs-mirrored state
                y_val = -y_val
            # Residual label: model learns correction on top of PD controller.
            # At inference: rudder = pd_suggestion + model_output
            y_val = y_val - float(x[-1, 19])
            return (torch.from_numpy(x).float(),
                    torch.tensor([y_val], dtype=torch.float32))

    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False
    AutopilotDataset = None
    MemmapDataset = None
    FrameDataset = None
