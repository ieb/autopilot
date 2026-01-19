"""
Feature Engineering Module
==========================

Transforms raw sensor data into normalized features for the ML model.
Maintains a sliding window of historical observations.
"""

import time
import math
from dataclasses import dataclass, field
from typing import Optional, Deque
from collections import deque
import numpy as np
import logging

from ..sensors.imu_fusion import IMUData
from ..sensors.nmea2000_interface import N2KData, calculate_true_wind
from ..sensors.adc_reader import RudderData
from .polar import Polar

logger = logging.getLogger(__name__)


@dataclass
class FeatureConfig:
    """Configuration for feature engineering."""
    sequence_length: int = 20      # Number of timesteps in sequence
    feature_dim: int = 25          # Number of features per timestep
    sample_rate_hz: float = 10.0   # Target sample rate for ML
    
    # Normalization constants
    max_heading_error: float = 180.0
    max_heading_rate: float = 30.0   # deg/s
    max_roll: float = 45.0
    max_pitch: float = 30.0
    max_roll_rate: float = 30.0      # deg/s
    max_wind_angle: float = 180.0
    max_wind_speed: float = 60.0     # knots
    max_boat_speed: float = 25.0     # knots (Pogo can hit 20+)
    max_rudder: float = 30.0         # degrees
    max_rudder_rate: float = 10.0    # deg/s
    max_vmg: float = 15.0            # knots
    max_wave_period: float = 15.0    # seconds


@dataclass
class TargetState:
    """Current autopilot target."""
    mode: str = "standby"  # standby, compass, wind_awa, wind_twa, vmg_up, vmg_down
    target_heading: float = 0.0      # For compass mode
    target_awa: float = 0.0          # For wind_awa mode  
    target_twa: float = 0.0          # For wind_twa mode


@dataclass
class FeatureFrame:
    """Single frame of features at one timestep."""
    timestamp: float = 0.0
    features: np.ndarray = field(default_factory=lambda: np.zeros(25))
    valid: bool = False


class FeatureEngineering:
    """
    Transforms sensor data into ML model input features.
    
    Maintains a sliding window of feature frames to create
    the sequence input for the LSTM model.
    """
    
    # Feature indices (for reference)
    FEAT_HEADING_ERROR = 0
    FEAT_HEADING_ERROR_INTEGRAL = 1
    FEAT_HEADING_RATE = 2
    FEAT_ROLL = 3
    FEAT_PITCH = 4
    FEAT_ROLL_RATE = 5
    FEAT_AWA = 6
    FEAT_AWA_RATE = 7
    FEAT_AWS = 8
    FEAT_TWA = 9
    FEAT_TWS = 10
    FEAT_STW = 11
    FEAT_SOG = 12
    FEAT_COG_ERROR = 13
    FEAT_RUDDER_POSITION = 14
    FEAT_RUDDER_VELOCITY = 15
    FEAT_TARGET_ANGLE = 16
    FEAT_VMG_UP = 17
    FEAT_VMG_DOWN = 18
    FEAT_POLAR_TARGET = 19
    FEAT_POLAR_PERFORMANCE = 20
    FEAT_MODE_COMPASS = 21
    FEAT_MODE_WIND_AWA = 22
    FEAT_MODE_WIND_TWA = 23
    FEAT_WAVE_PERIOD = 24
    
    def __init__(self, config: Optional[FeatureConfig] = None,
                 polar: Optional[Polar] = None):
        self.config = config or FeatureConfig()
        self.polar = polar or Polar.pogo_1250()
        
        # Feature history buffer
        self._buffer: Deque[FeatureFrame] = deque(maxlen=self.config.sequence_length)
        
        # State for derived features
        self._last_awa = 0.0
        self._last_awa_time = 0.0
        self._last_rudder = 0.0
        self._last_rudder_time = 0.0
        self._heading_error_integral = 0.0
        
        # Wave estimation from accelerometer
        self._accel_buffer: Deque[float] = deque(maxlen=100)  # 1 second @ 100Hz
        
        # Target state
        self.target = TargetState()
        
        # Fill buffer with zeros initially
        for _ in range(self.config.sequence_length):
            self._buffer.append(FeatureFrame())
            
    def update(self, imu: IMUData, n2k: N2KData, rudder: RudderData) -> np.ndarray:
        """
        Compute features from current sensor data.
        
        Args:
            imu: Current IMU data (100Hz source)
            n2k: Current NMEA2000 data (1Hz source, may be stale)
            rudder: Current rudder position (50Hz source)
            
        Returns:
            Feature sequence as numpy array [sequence_length, feature_dim]
        """
        now = time.time()
        features = np.zeros(self.config.feature_dim)
        
        # 1. Heading error based on mode
        heading_error = self._compute_heading_error(imu, n2k)
        features[self.FEAT_HEADING_ERROR] = self._normalize(
            heading_error, self.config.max_heading_error
        )
        
        # 2. Heading error integral (for wind-up)
        dt = now - self._last_rudder_time if self._last_rudder_time > 0 else 0.1
        self._heading_error_integral += heading_error * dt
        self._heading_error_integral = np.clip(self._heading_error_integral, -180, 180)
        features[self.FEAT_HEADING_ERROR_INTEGRAL] = self._normalize(
            self._heading_error_integral, 180.0
        )
        
        # 3. Heading rate (yaw rate from IMU)
        features[self.FEAT_HEADING_RATE] = self._normalize(
            imu.yaw_rate, self.config.max_heading_rate
        )
        
        # 4-5. Roll and pitch
        features[self.FEAT_ROLL] = self._normalize(imu.roll, self.config.max_roll)
        features[self.FEAT_PITCH] = self._normalize(imu.pitch, self.config.max_pitch)
        
        # 6. Roll rate
        features[self.FEAT_ROLL_RATE] = self._normalize(
            imu.roll_rate, self.config.max_roll_rate
        )
        
        # 7. AWA
        features[self.FEAT_AWA] = self._normalize(n2k.awa, self.config.max_wind_angle)
        
        # 8. AWA rate (computed)
        if self._last_awa_time > 0:
            awa_rate = (n2k.awa - self._last_awa) / max(0.01, now - self._last_awa_time)
        else:
            awa_rate = 0.0
        features[self.FEAT_AWA_RATE] = self._normalize(awa_rate, 10.0)
        self._last_awa = n2k.awa
        self._last_awa_time = now
        
        # 9. AWS
        features[self.FEAT_AWS] = self._normalize(n2k.aws, self.config.max_wind_speed)
        
        # 10-11. True wind (computed)
        twa, tws = calculate_true_wind(n2k.awa, n2k.aws, n2k.stw, imu.heading)
        features[self.FEAT_TWA] = self._normalize(twa, self.config.max_wind_angle)
        features[self.FEAT_TWS] = self._normalize(tws, self.config.max_wind_speed)
        
        # 12-13. Boat speed
        features[self.FEAT_STW] = self._normalize(n2k.stw, self.config.max_boat_speed)
        features[self.FEAT_SOG] = self._normalize(n2k.sog, self.config.max_boat_speed)
        
        # 14. COG error (if target is magnetic heading)
        cog_error = self._angle_diff(n2k.cog, self.target.target_heading)
        features[self.FEAT_COG_ERROR] = self._normalize(cog_error, 180.0)
        
        # 15. Rudder position
        features[self.FEAT_RUDDER_POSITION] = self._normalize(
            rudder.angle_deg, self.config.max_rudder
        )
        
        # 16. Rudder velocity (computed)
        if self._last_rudder_time > 0:
            rudder_dt = now - self._last_rudder_time
            rudder_vel = (rudder.angle_deg - self._last_rudder) / max(0.01, rudder_dt)
        else:
            rudder_vel = 0.0
        features[self.FEAT_RUDDER_VELOCITY] = self._normalize(
            rudder_vel, self.config.max_rudder_rate
        )
        self._last_rudder = rudder.angle_deg
        self._last_rudder_time = now
        
        # 17. Target angle (mode-dependent)
        target_angle = self._get_target_angle()
        features[self.FEAT_TARGET_ANGLE] = self._normalize(target_angle, 180.0)
        
        # 18-19. VMG
        vmg_up = n2k.stw * math.cos(math.radians(abs(twa))) if n2k.stw > 0 else 0
        vmg_down = n2k.stw * math.cos(math.radians(180 - abs(twa))) if n2k.stw > 0 else 0
        features[self.FEAT_VMG_UP] = self._normalize(vmg_up, self.config.max_vmg)
        features[self.FEAT_VMG_DOWN] = self._normalize(vmg_down, 20.0)
        
        # 20-21. Polar performance
        polar_target = self.polar.get_target_speed(twa, tws)
        polar_perf = self.polar.get_performance_ratio(twa, tws, n2k.stw)
        features[self.FEAT_POLAR_TARGET] = self._normalize(
            polar_target, self.config.max_boat_speed
        )
        features[self.FEAT_POLAR_PERFORMANCE] = np.clip(polar_perf, 0, 1.2)
        
        # 22-24. Mode flags
        features[self.FEAT_MODE_COMPASS] = 1.0 if self.target.mode == "compass" else 0.0
        features[self.FEAT_MODE_WIND_AWA] = 1.0 if self.target.mode == "wind_awa" else 0.0
        features[self.FEAT_MODE_WIND_TWA] = 1.0 if self.target.mode in ["wind_twa", "vmg_up", "vmg_down"] else 0.0
        
        # 25. Wave period estimate (from accelerometer FFT)
        self._accel_buffer.append(imu.accel_z)
        wave_period = self._estimate_wave_period()
        features[self.FEAT_WAVE_PERIOD] = self._normalize(
            wave_period, self.config.max_wave_period
        )
        
        # Add to buffer
        frame = FeatureFrame(
            timestamp=now,
            features=features,
            valid=imu.valid and rudder.valid
        )
        self._buffer.append(frame)
        
        # Return sequence
        return self.get_sequence()
        
    def get_sequence(self) -> np.ndarray:
        """Get current feature sequence for ML model."""
        sequence = np.zeros((self.config.sequence_length, self.config.feature_dim))
        for i, frame in enumerate(self._buffer):
            sequence[i] = frame.features
        return sequence
        
    def set_target(self, mode: str, value: float = 0.0):
        """
        Set autopilot target.
        
        Args:
            mode: One of 'standby', 'compass', 'wind_awa', 'wind_twa', 'vmg_up', 'vmg_down'
            value: Target value (heading, AWA, or TWA depending on mode)
        """
        self.target.mode = mode
        if mode == "compass":
            self.target.target_heading = value
        elif mode == "wind_awa":
            self.target.target_awa = value
        elif mode in ["wind_twa", "vmg_up", "vmg_down"]:
            self.target.target_twa = value
            
        # Reset integral on mode change
        self._heading_error_integral = 0.0
        
    def _compute_heading_error(self, imu: IMUData, n2k: N2KData) -> float:
        """Compute heading error based on current mode."""
        if self.target.mode == "compass":
            return self._angle_diff(imu.heading, self.target.target_heading)
        elif self.target.mode == "wind_awa":
            return self._angle_diff(n2k.awa, self.target.target_awa)
        elif self.target.mode in ["wind_twa", "vmg_up", "vmg_down"]:
            twa, _ = calculate_true_wind(n2k.awa, n2k.aws, n2k.stw, imu.heading)
            return self._angle_diff(twa, self.target.target_twa)
        return 0.0
        
    def _get_target_angle(self) -> float:
        """Get target angle for current mode."""
        if self.target.mode == "compass":
            return self.target.target_heading
        elif self.target.mode == "wind_awa":
            return self.target.target_awa
        elif self.target.mode in ["wind_twa", "vmg_up", "vmg_down"]:
            return self.target.target_twa
        return 0.0
        
    def _angle_diff(self, a: float, b: float) -> float:
        """Compute signed angle difference (a - b), normalized to [-180, 180]."""
        diff = a - b
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff
        
    def _normalize(self, value: float, max_val: float) -> float:
        """Normalize value to [-1, 1] range."""
        return np.clip(value / max_val, -1.0, 1.0)
        
    def _estimate_wave_period(self) -> float:
        """Estimate dominant wave period from accelerometer data."""
        if len(self._accel_buffer) < 50:
            return 5.0  # Default
            
        try:
            # Simple zero-crossing period estimate
            data = np.array(self._accel_buffer)
            mean = np.mean(data)
            crossings = np.where(np.diff(np.signbit(data - mean)))[0]
            
            if len(crossings) >= 2:
                # Average half-period between crossings
                half_periods = np.diff(crossings) / 100.0  # 100Hz sample rate
                full_period = np.mean(half_periods) * 2
                return np.clip(full_period, 2.0, 15.0)
        except Exception:
            pass
            
        return 5.0  # Default
        
    @property
    def is_valid(self) -> bool:
        """Check if we have enough valid data for inference."""
        valid_count = sum(1 for f in self._buffer if f.valid)
        return valid_count >= self.config.sequence_length // 2
