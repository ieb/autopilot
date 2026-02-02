"""
IMU Fusion I2C Module
=====================

Direct I2C interface to ICM-20948 IMU on Raspberry Pi.
Runs Madgwick AHRS filter in Python, eliminating the need for a separate MCU.

This is a drop-in replacement for imu_fusion.py when the ICM-20948 is connected
directly to the Pi's I2C bus instead of via a dedicated MCU.

Features:
- Direct I2C sensor reading at 100Hz
- Madgwick AHRS filter for sensor fusion
- Magnetometer calibration
- Off-center mounting acceleration correction
- Same API as IMUFusion for serial/socket

Requirements:
- smbus2: pip install smbus2
- numpy: pip install numpy
- I2C enabled on Pi: sudo raspi-config -> Interface Options -> I2C

Wiring:
    ICM-20948  ->  Raspberry Pi
    VCC        ->  3.3V (Pin 1)
    GND        ->  GND (Pin 6)
    SDA        ->  SDA (Pin 3, GPIO 2)
    SCL        ->  SCL (Pin 5, GPIO 3)
"""

import math
import json
import threading
import time
from dataclasses import dataclass
from typing import Optional, Callable, Tuple
from pathlib import Path
import logging

import numpy as np

logger = logging.getLogger(__name__)

# Try to import smbus2 (optional for testing without hardware)
try:
    import smbus2
    HAS_SMBUS = True
except ImportError:
    smbus2 = None
    HAS_SMBUS = False


# =============================================================================
# Constants
# =============================================================================

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi
GRAVITY = 9.81  # m/s²


# =============================================================================
# ICM-20948 Register Definitions
# =============================================================================

# ICM-20948 I2C addresses
ICM20948_ADDR_LOW = 0x68   # AD0 = LOW
ICM20948_ADDR_HIGH = 0x69  # AD0 = HIGH

# AK09916 magnetometer (inside ICM-20948)
AK09916_ADDR = 0x0C

# Bank 0 registers
REG_BANK_SEL = 0x7F
WHO_AM_I = 0x00
USER_CTRL = 0x03
LP_CONFIG = 0x05
PWR_MGMT_1 = 0x06
PWR_MGMT_2 = 0x07
INT_PIN_CFG = 0x0F
INT_ENABLE = 0x10
INT_ENABLE_1 = 0x11
INT_STATUS = 0x19
INT_STATUS_1 = 0x1A

# Sensor data registers (Bank 0)
ACCEL_XOUT_H = 0x2D
ACCEL_XOUT_L = 0x2E
ACCEL_YOUT_H = 0x2F
ACCEL_YOUT_L = 0x30
ACCEL_ZOUT_H = 0x31
ACCEL_ZOUT_L = 0x32
GYRO_XOUT_H = 0x33
GYRO_XOUT_L = 0x34
GYRO_YOUT_H = 0x35
GYRO_YOUT_L = 0x36
GYRO_ZOUT_H = 0x37
GYRO_ZOUT_L = 0x38
TEMP_OUT_H = 0x39
TEMP_OUT_L = 0x3A

# External sensor data (magnetometer via I2C master)
EXT_SLV_SENS_DATA_00 = 0x3B

# Bank 2 registers (accel/gyro config)
GYRO_SMPLRT_DIV = 0x00
GYRO_CONFIG_1 = 0x01
GYRO_CONFIG_2 = 0x02
ACCEL_SMPLRT_DIV_1 = 0x10
ACCEL_SMPLRT_DIV_2 = 0x11
ACCEL_CONFIG = 0x14
ACCEL_CONFIG_2 = 0x15

# Bank 3 registers (I2C master for magnetometer)
I2C_MST_ODR_CONFIG = 0x00
I2C_MST_CTRL = 0x01
I2C_MST_DELAY_CTRL = 0x02
I2C_SLV0_ADDR = 0x03
I2C_SLV0_REG = 0x04
I2C_SLV0_CTRL = 0x05
I2C_SLV0_DO = 0x06

# AK09916 magnetometer registers
AK09916_WIA2 = 0x01
AK09916_ST1 = 0x10
AK09916_HXL = 0x11
AK09916_HXH = 0x12
AK09916_HYL = 0x13
AK09916_HYH = 0x14
AK09916_HZL = 0x15
AK09916_HZH = 0x16
AK09916_ST2 = 0x18
AK09916_CNTL2 = 0x31
AK09916_CNTL3 = 0x32

# ICM-20948 expected WHO_AM_I value
ICM20948_WHO_AM_I_VALUE = 0xEA

# Sensor scale factors
ACCEL_SCALE_2G = 16384.0   # LSB/g
ACCEL_SCALE_4G = 8192.0
ACCEL_SCALE_8G = 4096.0
ACCEL_SCALE_16G = 2048.0

GYRO_SCALE_250DPS = 131.0   # LSB/(deg/s)
GYRO_SCALE_500DPS = 65.5
GYRO_SCALE_1000DPS = 32.8
GYRO_SCALE_2000DPS = 16.4

MAG_SCALE = 0.15  # uT/LSB for AK09916


# =============================================================================
# Data Classes (same as imu_fusion.py for API compatibility)
# =============================================================================

@dataclass
class IMUData:
    """Fused IMU orientation and rate data."""
    timestamp: float = 0.0
    
    # Orientation (degrees)
    heading: float = 0.0      # Magnetic heading, 0-360
    pitch: float = 0.0        # Bow up positive
    roll: float = 0.0         # Starboard down positive
    
    # Angular rates (deg/s)
    yaw_rate: float = 0.0     # Rate of turn
    pitch_rate: float = 0.0
    roll_rate: float = 0.0
    
    # Linear accelerations (m/s²)
    accel_x: float = 0.0      # Forward positive
    accel_y: float = 0.0      # Starboard positive  
    accel_z: float = 0.0      # Down positive
    
    # Status
    valid: bool = False
    age_ms: float = float('inf')
    
    def update_age(self, current_time: float):
        """Update the age of this reading."""
        self.age_ms = (current_time - self.timestamp) * 1000


@dataclass
class IMUCalibrationData:
    """Stored calibration data for IMU."""
    # Magnetometer calibration
    mag_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    mag_scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    
    # IMU mounting position offset from center of rotation (meters)
    position_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return {
            "mag_offset": list(self.mag_offset),
            "mag_scale": list(self.mag_scale),
            "position_offset": list(self.position_offset)
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'IMUCalibrationData':
        """Create from dictionary."""
        return cls(
            mag_offset=tuple(data.get("mag_offset", [0.0, 0.0, 0.0])),
            mag_scale=tuple(data.get("mag_scale", [1.0, 1.0, 1.0])),
            position_offset=tuple(data.get("position_offset", [0.0, 0.0, 0.0]))
        )


@dataclass
class IMUConfigI2C:
    """Configuration for I2C IMU connection."""
    i2c_bus: int = 1           # I2C bus number (1 for Pi)
    i2c_address: int = ICM20948_ADDR_LOW  # ICM-20948 address
    update_rate_hz: float = 100.0  # Target update rate
    max_age_ms: float = 200.0  # Max age before data considered stale
    
    # Filter tuning
    madgwick_beta: float = 0.1  # Filter gain (0.01-0.5)
    
    # Sensor ranges
    accel_range: int = 4       # g (2, 4, 8, or 16)
    gyro_range: int = 500      # deg/s (250, 500, 1000, or 2000)
    
    # Calibration storage
    calibration_file: Optional[str] = None


# Alias for backwards compatibility
IMUConfig = IMUConfigI2C


# =============================================================================
# Madgwick AHRS Filter (Python Port)
# =============================================================================

class MadgwickAHRS:
    """
    Madgwick AHRS filter for 9-DOF sensor fusion.
    
    Ported from C++ implementation in firmware/mcu/src/MadgwickAHRS.h
    """
    
    def __init__(self, sample_freq: float = 100.0, beta: float = 0.1):
        """
        Initialize the Madgwick filter.
        
        Args:
            sample_freq: Expected sample frequency in Hz
            beta: Filter gain (higher = faster but noisier)
        """
        self.sample_freq = sample_freq
        self.inv_sample_freq = 1.0 / sample_freq
        self.beta = beta
        
        # Quaternion (w, x, y, z)
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        
        # Cached Euler angles (degrees)
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
    
    def update(self, gx: float, gy: float, gz: float,
               ax: float, ay: float, az: float,
               mx: float, my: float, mz: float):
        """
        Update filter with 9-DOF sensor data.
        
        Args:
            gx, gy, gz: Gyroscope in degrees/second
            ax, ay, az: Accelerometer (any unit, normalized internally)
            mx, my, mz: Magnetometer (any unit, normalized internally)
        """
        q0, q1, q2, q3 = self.q
        
        # Convert gyro to radians/sec
        gx *= DEG_TO_RAD
        gy *= DEG_TO_RAD
        gz *= DEG_TO_RAD
        
        # Rate of change of quaternion from gyroscope
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
        
        # Compute feedback only if accelerometer measurement valid
        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
            # Normalize accelerometer
            norm = math.sqrt(ax * ax + ay * ay + az * az)
            ax /= norm
            ay /= norm
            az /= norm
            
            # Normalize magnetometer
            norm = math.sqrt(mx * mx + my * my + mz * mz)
            if norm > 0:
                mx /= norm
                my /= norm
                mz /= norm
            
            # Auxiliary variables
            _2q0mx = 2.0 * q0 * mx
            _2q0my = 2.0 * q0 * my
            _2q0mz = 2.0 * q0 * mz
            _2q1mx = 2.0 * q1 * mx
            _2q0 = 2.0 * q0
            _2q1 = 2.0 * q1
            _2q2 = 2.0 * q2
            _2q3 = 2.0 * q3
            _2q0q2 = 2.0 * q0 * q2
            _2q2q3 = 2.0 * q2 * q3
            q0q0 = q0 * q0
            q0q1 = q0 * q1
            q0q2 = q0 * q2
            q0q3 = q0 * q3
            q1q1 = q1 * q1
            q1q2 = q1 * q2
            q1q3 = q1 * q3
            q2q2 = q2 * q2
            q2q3 = q2 * q3
            q3q3 = q3 * q3
            
            # Reference direction of Earth's magnetic field
            hx = (mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + 
                  _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3)
            hy = (_2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - 
                  my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3)
            _2bx = math.sqrt(hx * hx + hy * hy)
            _2bz = (-_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - 
                    mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3)
            _4bx = 2.0 * _2bx
            _4bz = 2.0 * _2bz
            
            # Gradient descent corrective step
            s0 = (-_2q2 * (2.0 * q1q3 - _2q0q2 - ax) +
                  _2q1 * (2.0 * q0q1 + _2q2q3 - ay) -
                  _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                  (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                  _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz))
            
            s1 = (_2q3 * (2.0 * q1q3 - _2q0q2 - ax) +
                  _2q0 * (2.0 * q0q1 + _2q2q3 - ay) -
                  4.0 * q1 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az) +
                  _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                  (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                  (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz))
            
            s2 = (-_2q0 * (2.0 * q1q3 - _2q0q2 - ax) +
                  _2q3 * (2.0 * q0q1 + _2q2q3 - ay) -
                  4.0 * q2 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az) +
                  (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                  (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                  (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz))
            
            s3 = (_2q1 * (2.0 * q1q3 - _2q0q2 - ax) +
                  _2q2 * (2.0 * q0q1 + _2q2q3 - ay) +
                  (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                  (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                  _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz))
            
            # Normalize step magnitude
            norm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
            if norm > 0:
                s0 /= norm
                s1 /= norm
                s2 /= norm
                s3 /= norm
            
            # Apply feedback step
            qDot1 -= self.beta * s0
            qDot2 -= self.beta * s1
            qDot3 -= self.beta * s2
            qDot4 -= self.beta * s3
        
        # Integrate rate of change
        q0 += qDot1 * self.inv_sample_freq
        q1 += qDot2 * self.inv_sample_freq
        q2 += qDot3 * self.inv_sample_freq
        q3 += qDot4 * self.inv_sample_freq
        
        # Normalize quaternion
        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.q[0] = q0 / norm
        self.q[1] = q1 / norm
        self.q[2] = q2 / norm
        self.q[3] = q3 / norm
        
        # Compute Euler angles
        self._compute_angles()
    
    def _compute_angles(self):
        """Compute Euler angles from quaternion."""
        q0, q1, q2, q3 = self.q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
        self._roll = math.atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1.0:
            self._pitch = math.copysign(90.0, sinp)
        else:
            self._pitch = math.asin(sinp) * RAD_TO_DEG
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        self._yaw = math.atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG
        
        # Normalize yaw to 0-360
        if self._yaw < 0:
            self._yaw += 360.0
    
    @property
    def roll(self) -> float:
        return self._roll
    
    @property
    def pitch(self) -> float:
        return self._pitch
    
    @property
    def yaw(self) -> float:
        return self._yaw


# =============================================================================
# ICM-20948 I2C Driver
# =============================================================================

class ICM20948Driver:
    """
    I2C driver for ICM-20948 9-DOF IMU sensor.
    
    Handles direct register read/write and sensor configuration.
    """
    
    def __init__(self, bus: int = 1, address: int = ICM20948_ADDR_LOW):
        """
        Initialize the ICM-20948 driver.
        
        Args:
            bus: I2C bus number (1 for Pi)
            address: I2C address (0x68 or 0x69)
        """
        self.bus_num = bus
        self.address = address
        self._bus: Optional['smbus2.SMBus'] = None
        
        # Current register bank
        self._current_bank = -1
        
        # Sensor scale factors
        self._accel_scale = ACCEL_SCALE_4G
        self._gyro_scale = GYRO_SCALE_500DPS
        
        # Last read values
        self.accel = np.zeros(3, dtype=np.float64)  # g
        self.gyro = np.zeros(3, dtype=np.float64)   # deg/s
        self.mag = np.zeros(3, dtype=np.float64)    # uT
        self.temperature = 0.0  # °C
    
    def begin(self) -> bool:
        """
        Initialize the sensor.
        
        Returns:
            True if initialization successful
        """
        if not HAS_SMBUS:
            logger.error("smbus2 not installed. Run: pip install smbus2")
            return False
        
        try:
            self._bus = smbus2.SMBus(self.bus_num)
            
            # Check WHO_AM_I
            self._set_bank(0)
            who_am_i = self._read_byte(WHO_AM_I)
            if who_am_i != ICM20948_WHO_AM_I_VALUE:
                logger.error(f"ICM-20948 not found. WHO_AM_I = 0x{who_am_i:02X}, expected 0x{ICM20948_WHO_AM_I_VALUE:02X}")
                return False
            
            # Reset device
            self._write_byte(PWR_MGMT_1, 0x80)
            time.sleep(0.1)
            
            # Wake up, auto-select clock
            self._write_byte(PWR_MGMT_1, 0x01)
            time.sleep(0.05)
            
            # Enable all sensors
            self._write_byte(PWR_MGMT_2, 0x00)
            
            # Configure sample rate divider (Bank 2)
            self._set_bank(2)
            self._write_byte(GYRO_SMPLRT_DIV, 0x00)  # Max rate
            self._write_byte(ACCEL_SMPLRT_DIV_1, 0x00)
            self._write_byte(ACCEL_SMPLRT_DIV_2, 0x00)
            
            # Configure gyro: ±500 dps, DLPF enabled
            self._write_byte(GYRO_CONFIG_1, 0x19)  # ±500 dps, DLPF
            self._gyro_scale = GYRO_SCALE_500DPS
            
            # Configure accel: ±4g, DLPF enabled
            self._write_byte(ACCEL_CONFIG, 0x09)  # ±4g, DLPF
            self._accel_scale = ACCEL_SCALE_4G
            
            # Setup magnetometer via I2C master
            self._setup_magnetometer()
            
            self._set_bank(0)
            logger.info("ICM-20948 initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize ICM-20948: {e}")
            return False
    
    def _setup_magnetometer(self):
        """Configure I2C master to read magnetometer."""
        # Enable I2C bypass to access magnetometer directly
        self._set_bank(0)
        self._write_byte(INT_PIN_CFG, 0x02)  # Bypass enable
        time.sleep(0.01)
        
        # Reset magnetometer
        try:
            self._bus.write_byte_data(AK09916_ADDR, AK09916_CNTL3, 0x01)
            time.sleep(0.01)
        except Exception:
            logger.warning("Could not reset magnetometer")
        
        # Set magnetometer to continuous mode 4 (100Hz)
        try:
            self._bus.write_byte_data(AK09916_ADDR, AK09916_CNTL2, 0x08)
            time.sleep(0.01)
            logger.debug("Magnetometer configured for 100Hz")
        except Exception as e:
            logger.warning(f"Could not configure magnetometer: {e}")
    
    def _set_bank(self, bank: int):
        """Switch register bank."""
        if bank != self._current_bank:
            self._bus.write_byte_data(self.address, REG_BANK_SEL, bank << 4)
            self._current_bank = bank
    
    def _read_byte(self, reg: int) -> int:
        """Read a single byte from register."""
        return self._bus.read_byte_data(self.address, reg)
    
    def _write_byte(self, reg: int, value: int):
        """Write a single byte to register."""
        self._bus.write_byte_data(self.address, reg, value)
    
    def _read_bytes(self, reg: int, length: int) -> bytes:
        """Read multiple bytes from consecutive registers."""
        return bytes(self._bus.read_i2c_block_data(self.address, reg, length))
    
    def read(self) -> bool:
        """
        Read all sensor data.
        
        Returns:
            True if read successful
        """
        try:
            self._set_bank(0)
            
            # Read accel and gyro (14 bytes starting at ACCEL_XOUT_H)
            data = self._read_bytes(ACCEL_XOUT_H, 14)
            
            # Parse accelerometer (big-endian signed 16-bit)
            ax = self._parse_int16(data[0], data[1]) / self._accel_scale
            ay = self._parse_int16(data[2], data[3]) / self._accel_scale
            az = self._parse_int16(data[4], data[5]) / self._accel_scale
            
            # Parse gyroscope
            gx = self._parse_int16(data[6], data[7]) / self._gyro_scale
            gy = self._parse_int16(data[8], data[9]) / self._gyro_scale
            gz = self._parse_int16(data[10], data[11]) / self._gyro_scale
            
            # Parse temperature
            temp_raw = self._parse_int16(data[12], data[13])
            self.temperature = (temp_raw / 333.87) + 21.0
            
            self.accel[0] = ax
            self.accel[1] = ay
            self.accel[2] = az
            self.gyro[0] = gx
            self.gyro[1] = gy
            self.gyro[2] = gz
            
            # Read magnetometer via bypass
            self._read_magnetometer()
            
            return True
            
        except Exception as e:
            logger.warning(f"Failed to read sensor data: {e}")
            return False
    
    def _read_magnetometer(self):
        """Read magnetometer data via I2C bypass."""
        try:
            # Check if data is ready
            st1 = self._bus.read_byte_data(AK09916_ADDR, AK09916_ST1)
            if not (st1 & 0x01):
                return  # Data not ready
            
            # Read 8 bytes (HXL through ST2)
            data = bytes(self._bus.read_i2c_block_data(AK09916_ADDR, AK09916_HXL, 8))
            
            # Parse (little-endian signed 16-bit)
            mx = self._parse_int16_le(data[0], data[1]) * MAG_SCALE
            my = self._parse_int16_le(data[2], data[3]) * MAG_SCALE
            mz = self._parse_int16_le(data[4], data[5]) * MAG_SCALE
            
            self.mag[0] = mx
            self.mag[1] = my
            self.mag[2] = mz
            
        except Exception as e:
            logger.debug(f"Magnetometer read failed: {e}")
    
    @staticmethod
    def _parse_int16(high: int, low: int) -> int:
        """Parse big-endian signed 16-bit integer."""
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 0x10000
        return value
    
    @staticmethod
    def _parse_int16_le(low: int, high: int) -> int:
        """Parse little-endian signed 16-bit integer."""
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 0x10000
        return value
    
    def close(self):
        """Close the I2C bus."""
        if self._bus:
            try:
                self._bus.close()
            except Exception:
                pass
            self._bus = None


# =============================================================================
# IMU Fusion I2C Class
# =============================================================================

class IMUFusionI2C:
    """
    Direct I2C interface to ICM-20948 with Madgwick AHRS filter.
    
    Drop-in replacement for IMUFusion (serial version).
    """
    
    def __init__(self, config: Optional[IMUConfigI2C] = None,
                 calibration: Optional[IMUCalibrationData] = None):
        self.config = config or IMUConfigI2C()
        self.calibration = calibration or IMUCalibrationData()
        
        self._driver: Optional[ICM20948Driver] = None
        self._filter: Optional[MadgwickAHRS] = None
        self._data = IMUData()
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._callbacks: list[Callable[[IMUData], None]] = []
        
        # Previous gyro for angular acceleration
        self._prev_gyro = np.zeros(3)
        self._prev_gyro_time = 0.0
        
        # Statistics
        self._msg_count = 0
        self._error_count = 0
        self._last_msg_time = 0.0
        
        # Version info (for API compatibility)
        self.firmware_version = "i2c-direct-1.0"
        
        # Load calibration if specified
        if self.config.calibration_file and not calibration:
            self._load_calibration()
    
    def _load_calibration(self):
        """Load calibration from JSON file."""
        if not self.config.calibration_file:
            return
        
        path = Path(self.config.calibration_file)
        if path.exists():
            try:
                with open(path, 'r') as f:
                    data = json.load(f)
                self.calibration = IMUCalibrationData.from_dict(data)
                logger.info(f"Loaded IMU calibration from {path}")
            except Exception as e:
                logger.warning(f"Failed to load IMU calibration: {e}")
    
    def save_calibration(self) -> bool:
        """Save calibration to JSON file."""
        if not self.config.calibration_file:
            logger.warning("No calibration file configured")
            return False
        
        path = Path(self.config.calibration_file)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'w') as f:
                json.dump(self.calibration.to_dict(), f, indent=2)
            logger.info(f"Saved IMU calibration to {path}")
            return True
        except Exception as e:
            logger.error(f"Failed to save IMU calibration: {e}")
            return False
    
    def start(self) -> bool:
        """
        Start the IMU interface.
        
        Returns:
            True if started successfully
        """
        try:
            # Initialize I2C driver
            self._driver = ICM20948Driver(
                bus=self.config.i2c_bus,
                address=self.config.i2c_address
            )
            if not self._driver.begin():
                return False
            
            # Initialize Madgwick filter
            self._filter = MadgwickAHRS(
                sample_freq=self.config.update_rate_hz,
                beta=self.config.madgwick_beta
            )
            
            # Start update thread
            self._running = True
            self._thread = threading.Thread(target=self._update_loop, daemon=True)
            self._thread.start()
            
            logger.info(f"IMU I2C started at {self.config.update_rate_hz}Hz")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start IMU I2C: {e}")
            return False
    
    def stop(self):
        """Stop the IMU interface."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._driver:
            self._driver.close()
            self._driver = None
        logger.info("IMU I2C stopped")
    
    def get_data(self) -> IMUData:
        """Get the latest IMU data (thread-safe)."""
        with self._lock:
            data = IMUData(
                timestamp=self._data.timestamp,
                heading=self._data.heading,
                pitch=self._data.pitch,
                roll=self._data.roll,
                yaw_rate=self._data.yaw_rate,
                pitch_rate=self._data.pitch_rate,
                roll_rate=self._data.roll_rate,
                accel_x=self._data.accel_x,
                accel_y=self._data.accel_y,
                accel_z=self._data.accel_z,
                valid=self._data.valid
            )
            data.update_age(time.time())
            data.valid = data.age_ms < self.config.max_age_ms
            return data
    
    def add_callback(self, callback: Callable[[IMUData], None]):
        """Register a callback for new IMU data."""
        self._callbacks.append(callback)
    
    def _update_loop(self):
        """Background thread running sensor fusion at target rate."""
        interval = 1.0 / self.config.update_rate_hz
        next_update = time.time()
        
        while self._running:
            now = time.time()
            
            if now >= next_update:
                try:
                    self._update_step()
                except Exception as e:
                    self._error_count += 1
                    logger.warning(f"IMU update error: {e}")
                
                next_update += interval
                if next_update < now:
                    next_update = now + interval
            else:
                time.sleep(min(0.001, next_update - now))
    
    def _update_step(self):
        """Perform one update step: read sensors, run filter, update data."""
        if not self._driver.read():
            return
        
        now = time.time()
        
        # Get raw sensor values
        ax, ay, az = self._driver.accel
        gx, gy, gz = self._driver.gyro
        mx, my, mz = self._driver.mag
        
        # Apply magnetometer calibration
        ox, oy, oz = self.calibration.mag_offset
        sx, sy, sz = self.calibration.mag_scale
        mx = (mx - ox) * sx
        my = (my - oy) * sy
        mz = (mz - oz) * sz
        
        # Update Madgwick filter
        self._filter.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        
        # Get orientation from filter
        heading = self._filter.yaw
        pitch = self._filter.pitch
        roll = self._filter.roll
        
        # Calculate angular acceleration for offset correction
        dt = now - self._prev_gyro_time if self._prev_gyro_time > 0 else 0.01
        if 0 < dt < 0.1:
            alpha_x = (gx - self._prev_gyro[0]) / dt
            alpha_y = (gy - self._prev_gyro[1]) / dt
            alpha_z = (gz - self._prev_gyro[2]) / dt
        else:
            alpha_x = alpha_y = alpha_z = 0.0
        
        self._prev_gyro[0] = gx
        self._prev_gyro[1] = gy
        self._prev_gyro[2] = gz
        self._prev_gyro_time = now
        
        # Convert acceleration to m/s²
        accel_x = ax * GRAVITY
        accel_y = ay * GRAVITY
        accel_z = az * GRAVITY
        
        # Apply off-center mounting correction
        accel_x, accel_y, accel_z = self._correct_accel_for_offset(
            accel_x, accel_y, accel_z,
            gx, gy, gz,
            alpha_x, alpha_y, alpha_z
        )
        
        # Update stored data
        with self._lock:
            self._data.timestamp = now
            self._data.heading = heading
            self._data.pitch = pitch
            self._data.roll = roll
            self._data.yaw_rate = gz
            self._data.pitch_rate = gy
            self._data.roll_rate = gx
            self._data.accel_x = accel_x
            self._data.accel_y = accel_y
            self._data.accel_z = accel_z
            self._data.valid = True
        
        self._msg_count += 1
        self._last_msg_time = now
        
        # Notify callbacks
        data = self.get_data()
        for callback in self._callbacks:
            try:
                callback(data)
            except Exception as e:
                logger.warning(f"IMU callback error: {e}")
    
    def _correct_accel_for_offset(self, ax: float, ay: float, az: float,
                                   gx: float, gy: float, gz: float,
                                   alpha_x: float, alpha_y: float, alpha_z: float
                                   ) -> Tuple[float, float, float]:
        """
        Correct acceleration for off-center IMU mounting.
        
        Subtracts centripetal and tangential accelerations caused by
        rotational motion when the IMU is not at the center of rotation.
        """
        dx, dy, dz = self.calibration.position_offset
        
        if dx == 0.0 and dy == 0.0 and dz == 0.0:
            return ax, ay, az
        
        # Convert to rad/s and rad/s²
        wx = gx * DEG_TO_RAD
        wy = gy * DEG_TO_RAD
        wz = gz * DEG_TO_RAD
        ax_rad = alpha_x * DEG_TO_RAD
        ay_rad = alpha_y * DEG_TO_RAD
        az_rad = alpha_z * DEG_TO_RAD
        
        # Centripetal acceleration: a_c = omega x (omega x r)
        cent_x = wy * (wy * dx - wx * dy) + wz * (wz * dx - wx * dz)
        cent_y = wx * (wx * dy - wy * dx) + wz * (wz * dy - wy * dz)
        cent_z = wx * (wx * dz - wz * dx) + wy * (wy * dz - wz * dy)
        
        # Tangential acceleration: a_t = alpha x r
        tang_x = ay_rad * dz - az_rad * dy
        tang_y = az_rad * dx - ax_rad * dz
        tang_z = ax_rad * dy - ay_rad * dx
        
        # Subtract spurious accelerations
        return (ax - cent_x - tang_x,
                ay - cent_y - tang_y,
                az - cent_z - tang_z)
    
    @property
    def stats(self) -> dict:
        """Get statistics about IMU operation."""
        return {
            "message_count": self._msg_count,
            "error_count": self._error_count,
            "last_message_age_ms": (time.time() - self._last_msg_time) * 1000 if self._last_msg_time else float('inf'),
            "connected": self._driver is not None and self._running,
            "firmware_version": self.firmware_version
        }


# =============================================================================
# Calibration Helper
# =============================================================================

class IMUCalibrationI2C:
    """
    Interactive magnetometer calibration for I2C-connected IMU.
    """
    
    @staticmethod
    def run_calibration(config: Optional[IMUConfigI2C] = None,
                        duration: float = 30.0) -> Optional[IMUCalibrationData]:
        """
        Run interactive magnetometer calibration.
        
        Slowly rotate the sensor through all orientations during calibration.
        
        Args:
            config: I2C configuration (uses defaults if None)
            duration: Calibration duration in seconds
            
        Returns:
            IMUCalibrationData with mag calibration, or None on failure
        """
        config = config or IMUConfigI2C()
        
        try:
            driver = ICM20948Driver(
                bus=config.i2c_bus,
                address=config.i2c_address
            )
            if not driver.begin():
                return None
            
            logger.info(f"Starting calibration - rotate sensor slowly for {duration} seconds")
            
            # Track min/max
            mag_min = np.array([99999.0, 99999.0, 99999.0])
            mag_max = np.array([-99999.0, -99999.0, -99999.0])
            
            start_time = time.time()
            sample_count = 0
            
            while time.time() - start_time < duration:
                if driver.read():
                    mx, my, mz = driver.mag
                    
                    mag_min[0] = min(mag_min[0], mx)
                    mag_max[0] = max(mag_max[0], mx)
                    mag_min[1] = min(mag_min[1], my)
                    mag_max[1] = max(mag_max[1], my)
                    mag_min[2] = min(mag_min[2], mz)
                    mag_max[2] = max(mag_max[2], mz)
                    
                    sample_count += 1
                
                time.sleep(0.01)  # 100Hz sampling
            
            driver.close()
            
            if sample_count < 100:
                logger.error("Insufficient samples collected")
                return None
            
            # Calculate offsets (center of min/max)
            offset = (mag_min + mag_max) / 2.0
            
            # Calculate scales
            ranges = mag_max - mag_min
            ranges = np.maximum(ranges, 0.001)  # Avoid division by zero
            avg_range = np.mean(ranges)
            scale = avg_range / ranges
            
            cal_data = IMUCalibrationData(
                mag_offset=(float(offset[0]), float(offset[1]), float(offset[2])),
                mag_scale=(float(scale[0]), float(scale[1]), float(scale[2]))
            )
            
            logger.info(f"Calibration complete: offset={cal_data.mag_offset}, scale={cal_data.mag_scale}")
            logger.info(f"Collected {sample_count} samples")
            
            return cal_data
            
        except Exception as e:
            logger.error(f"Calibration failed: {e}")
            return None


# =============================================================================
# Compatibility Aliases
# =============================================================================

# For drop-in replacement compatibility
IMUFusion = IMUFusionI2C
IMUCalibration = IMUCalibrationI2C
