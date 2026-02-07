"""
IMU Fusion BNO055 Module
========================

Direct I2C interface to BNO055 9-DOF Absolute Orientation IMU on Raspberry Pi.
Uses the BNO055's built-in sensor fusion processor for orientation output.

This is a drop-in replacement for imu_fusion.py when the BNO055 is connected
directly to the Pi's I2C bus.

Key Advantages of BNO055:
- Built-in sensor fusion processor (no Madgwick filter needed on Pi)
- Outputs Euler angles directly at 100Hz
- Built-in calibration status reporting
- Lower CPU usage than software fusion
- Consistent timing (fusion runs on sensor's ARM Cortex-M0)

Requirements:
- smbus2: pip install smbus2
- numpy: pip install numpy
- I2C enabled on Pi: sudo raspi-config -> Interface Options -> I2C

Wiring:
    BNO055     ->  Raspberry Pi
    VIN        ->  3.3V (Pin 1)
    GND        ->  GND (Pin 6)
    SDA        ->  SDA (Pin 3, GPIO 2)
    SCL        ->  SCL (Pin 5, GPIO 3)
    
Note: Some BNO055 breakouts have a voltage regulator and can use 5V on VIN.
Check your specific board's documentation.
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
# BNO055 Register Definitions
# =============================================================================

# BNO055 I2C addresses
BNO055_ADDR_LOW = 0x28   # COM3 = LOW (default)
BNO055_ADDR_HIGH = 0x29  # COM3 = HIGH

# Page 0 registers
BNO055_CHIP_ID = 0x00
BNO055_ACC_ID = 0x01
BNO055_MAG_ID = 0x02
BNO055_GYR_ID = 0x03
BNO055_SW_REV_ID_LSB = 0x04
BNO055_SW_REV_ID_MSB = 0x05
BNO055_BL_REV_ID = 0x06
BNO055_PAGE_ID = 0x07

# Accelerometer data
BNO055_ACC_DATA_X_LSB = 0x08
BNO055_ACC_DATA_X_MSB = 0x09
BNO055_ACC_DATA_Y_LSB = 0x0A
BNO055_ACC_DATA_Y_MSB = 0x0B
BNO055_ACC_DATA_Z_LSB = 0x0C
BNO055_ACC_DATA_Z_MSB = 0x0D

# Magnetometer data
BNO055_MAG_DATA_X_LSB = 0x0E
BNO055_MAG_DATA_X_MSB = 0x0F
BNO055_MAG_DATA_Y_LSB = 0x10
BNO055_MAG_DATA_Y_MSB = 0x11
BNO055_MAG_DATA_Z_LSB = 0x12
BNO055_MAG_DATA_Z_MSB = 0x13

# Gyroscope data
BNO055_GYR_DATA_X_LSB = 0x14
BNO055_GYR_DATA_X_MSB = 0x15
BNO055_GYR_DATA_Y_LSB = 0x16
BNO055_GYR_DATA_Y_MSB = 0x17
BNO055_GYR_DATA_Z_LSB = 0x18
BNO055_GYR_DATA_Z_MSB = 0x19

# Euler angles (fusion output)
BNO055_EUL_HEADING_LSB = 0x1A
BNO055_EUL_HEADING_MSB = 0x1B
BNO055_EUL_ROLL_LSB = 0x1C
BNO055_EUL_ROLL_MSB = 0x1D
BNO055_EUL_PITCH_LSB = 0x1E
BNO055_EUL_PITCH_MSB = 0x1F

# Quaternion data
BNO055_QUA_DATA_W_LSB = 0x20
BNO055_QUA_DATA_W_MSB = 0x21
BNO055_QUA_DATA_X_LSB = 0x22
BNO055_QUA_DATA_X_MSB = 0x23
BNO055_QUA_DATA_Y_LSB = 0x24
BNO055_QUA_DATA_Y_MSB = 0x25
BNO055_QUA_DATA_Z_LSB = 0x26
BNO055_QUA_DATA_Z_MSB = 0x27

# Linear acceleration (gravity removed)
BNO055_LIA_DATA_X_LSB = 0x28
BNO055_LIA_DATA_X_MSB = 0x29
BNO055_LIA_DATA_Y_LSB = 0x2A
BNO055_LIA_DATA_Y_MSB = 0x2B
BNO055_LIA_DATA_Z_LSB = 0x2C
BNO055_LIA_DATA_Z_MSB = 0x2D

# Gravity vector
BNO055_GRV_DATA_X_LSB = 0x2E
BNO055_GRV_DATA_X_MSB = 0x2F
BNO055_GRV_DATA_Y_LSB = 0x30
BNO055_GRV_DATA_Y_MSB = 0x31
BNO055_GRV_DATA_Z_LSB = 0x32
BNO055_GRV_DATA_Z_MSB = 0x33

# Temperature
BNO055_TEMP = 0x34

# Calibration status
BNO055_CALIB_STAT = 0x35
BNO055_ST_RESULT = 0x36
BNO055_INT_STA = 0x37

# System status
BNO055_SYS_CLK_STATUS = 0x38
BNO055_SYS_STATUS = 0x39
BNO055_SYS_ERR = 0x3A

# Unit selection
BNO055_UNIT_SEL = 0x3B
BNO055_DATA_SELECT = 0x3C

# Operating mode
BNO055_OPR_MODE = 0x3D
BNO055_PWR_MODE = 0x3E

# System trigger
BNO055_SYS_TRIGGER = 0x3F

# Temperature source
BNO055_TEMP_SOURCE = 0x40

# Axis mapping
BNO055_AXIS_MAP_CONFIG = 0x41
BNO055_AXIS_MAP_SIGN = 0x42

# Calibration offsets (stored in sensor)
BNO055_ACC_OFFSET_X_LSB = 0x55
BNO055_ACC_OFFSET_X_MSB = 0x56
BNO055_ACC_OFFSET_Y_LSB = 0x57
BNO055_ACC_OFFSET_Y_MSB = 0x58
BNO055_ACC_OFFSET_Z_LSB = 0x59
BNO055_ACC_OFFSET_Z_MSB = 0x5A
BNO055_MAG_OFFSET_X_LSB = 0x5B
BNO055_MAG_OFFSET_X_MSB = 0x5C
BNO055_MAG_OFFSET_Y_LSB = 0x5D
BNO055_MAG_OFFSET_Y_MSB = 0x5E
BNO055_MAG_OFFSET_Z_LSB = 0x5F
BNO055_MAG_OFFSET_Z_MSB = 0x60
BNO055_GYR_OFFSET_X_LSB = 0x61
BNO055_GYR_OFFSET_X_MSB = 0x62
BNO055_GYR_OFFSET_Y_LSB = 0x63
BNO055_GYR_OFFSET_Y_MSB = 0x64
BNO055_GYR_OFFSET_Z_LSB = 0x65
BNO055_GYR_OFFSET_Z_MSB = 0x66
BNO055_ACC_RADIUS_LSB = 0x67
BNO055_ACC_RADIUS_MSB = 0x68
BNO055_MAG_RADIUS_LSB = 0x69
BNO055_MAG_RADIUS_MSB = 0x6A

# Expected chip ID
BNO055_CHIP_ID_VALUE = 0xA0

# Operating modes
OPR_MODE_CONFIG = 0x00
OPR_MODE_ACCONLY = 0x01
OPR_MODE_MAGONLY = 0x02
OPR_MODE_GYROONLY = 0x03
OPR_MODE_ACCMAG = 0x04
OPR_MODE_ACCGYRO = 0x05
OPR_MODE_MAGGYRO = 0x06
OPR_MODE_AMG = 0x07
OPR_MODE_IMU = 0x08
OPR_MODE_COMPASS = 0x09
OPR_MODE_M4G = 0x0A
OPR_MODE_NDOF_FMC_OFF = 0x0B
OPR_MODE_NDOF = 0x0C  # 9-DOF fusion mode (default for autopilot)

# Power modes
PWR_MODE_NORMAL = 0x00
PWR_MODE_LOW = 0x01
PWR_MODE_SUSPEND = 0x02

# Scale factors
EULER_SCALE = 16.0      # LSB per degree
ACCEL_SCALE = 100.0     # LSB per m/s²
GYRO_SCALE = 16.0       # LSB per deg/s
MAG_SCALE = 16.0        # LSB per uT
QUAT_SCALE = 16384.0    # LSB per unit quaternion
LIA_SCALE = 100.0       # LSB per m/s² (linear acceleration)


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
    
    # BNO055-specific: calibration status (0-3 for each sensor)
    cal_sys: int = 0
    cal_gyro: int = 0
    cal_accel: int = 0
    cal_mag: int = 0
    
    def update_age(self, current_time: float):
        """Update the age of this reading."""
        self.age_ms = (current_time - self.timestamp) * 1000
    
    @property
    def is_calibrated(self) -> bool:
        """Check if sensor is fully calibrated."""
        return self.cal_sys == 3 and self.cal_gyro == 3 and self.cal_mag == 3


@dataclass
class IMUCalibrationData:
    """Stored calibration data for BNO055."""
    # BNO055 stores 22 bytes of calibration offsets
    calibration_bytes: bytes = b'\x00' * 22
    
    # IMU mounting position offset from center of rotation (meters)
    position_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        import base64
        return {
            "calibration_bytes": base64.b64encode(self.calibration_bytes).decode('ascii'),
            "position_offset": list(self.position_offset)
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'IMUCalibrationData':
        """Create from dictionary."""
        import base64
        cal_bytes = b'\x00' * 22
        if "calibration_bytes" in data:
            cal_bytes = base64.b64decode(data["calibration_bytes"])
        return cls(
            calibration_bytes=cal_bytes,
            position_offset=tuple(data.get("position_offset", [0.0, 0.0, 0.0]))
        )


@dataclass
class IMUConfigBNO055:
    """Configuration for BNO055 I2C connection."""
    i2c_bus: int = 1           # I2C bus number (1 for Pi)
    i2c_address: int = BNO055_ADDR_LOW  # BNO055 address
    update_rate_hz: float = 100.0  # Target update rate
    max_age_ms: float = 200.0  # Max age before data considered stale
    
    # Operating mode (NDOF = 9-DOF fusion)
    operation_mode: int = OPR_MODE_NDOF
    
    # Use linear acceleration (gravity removed) instead of raw accel
    use_linear_accel: bool = True
    
    # Calibration storage
    calibration_file: Optional[str] = None


# Alias for backwards compatibility
IMUConfig = IMUConfigBNO055


# =============================================================================
# BNO055 I2C Driver
# =============================================================================

class BNO055Driver:
    """
    I2C driver for BNO055 9-DOF Absolute Orientation IMU.
    
    Uses the sensor's built-in fusion processor for orientation output.
    """
    
    def __init__(self, bus: int = 1, address: int = BNO055_ADDR_LOW):
        """
        Initialize the BNO055 driver.
        
        Args:
            bus: I2C bus number (1 for Pi)
            address: I2C address (0x28 or 0x29)
        """
        self.bus_num = bus
        self.address = address
        self._bus: Optional['smbus2.SMBus'] = None
        
        # Last read values
        self.euler = np.zeros(3, dtype=np.float64)   # heading, roll, pitch (deg)
        self.gyro = np.zeros(3, dtype=np.float64)    # deg/s
        self.accel = np.zeros(3, dtype=np.float64)   # m/s²
        self.linear_accel = np.zeros(3, dtype=np.float64)  # m/s² (gravity removed)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.temperature = 0  # °C
        
        # Calibration status (0-3 for each)
        self.cal_sys = 0
        self.cal_gyro = 0
        self.cal_accel = 0
        self.cal_mag = 0
    
    def begin(self, mode: int = OPR_MODE_NDOF) -> bool:
        """
        Initialize the sensor.
        
        Args:
            mode: Operating mode (default NDOF for 9-DOF fusion)
            
        Returns:
            True if initialization successful
        """
        if not HAS_SMBUS:
            logger.error("smbus2 not installed. Run: pip install smbus2")
            return False
        
        try:
            self._bus = smbus2.SMBus(self.bus_num)
            
            # Check chip ID
            chip_id = self._read_byte(BNO055_CHIP_ID)
            if chip_id != BNO055_CHIP_ID_VALUE:
                logger.error(f"BNO055 not found. CHIP_ID = 0x{chip_id:02X}, expected 0x{BNO055_CHIP_ID_VALUE:02X}")
                return False
            
            # Switch to config mode for setup
            self._set_mode(OPR_MODE_CONFIG)
            time.sleep(0.025)
            
            # Reset
            self._write_byte(BNO055_SYS_TRIGGER, 0x20)
            time.sleep(0.65)  # Wait for reset
            
            # Wait for chip ID to be readable again
            for _ in range(10):
                try:
                    if self._read_byte(BNO055_CHIP_ID) == BNO055_CHIP_ID_VALUE:
                        break
                except Exception:
                    pass
                time.sleep(0.1)
            
            # Set to config mode
            self._set_mode(OPR_MODE_CONFIG)
            time.sleep(0.025)
            
            # Set power mode to normal
            self._write_byte(BNO055_PWR_MODE, PWR_MODE_NORMAL)
            time.sleep(0.01)
            
            # Set page 0
            self._write_byte(BNO055_PAGE_ID, 0)
            
            # Set units: degrees, m/s², Celsius
            # Bit 0: Accel = m/s², Bit 1: Gyro = deg/s, Bit 2: Euler = degrees
            # Bit 4: Temp = Celsius, Bit 7: Orientation = Windows (pitch increasing nose down)
            self._write_byte(BNO055_UNIT_SEL, 0x00)
            
            # External crystal for better accuracy (if available)
            self._write_byte(BNO055_SYS_TRIGGER, 0x80)
            time.sleep(0.01)
            
            # Set operating mode
            self._set_mode(mode)
            time.sleep(0.02)
            
            logger.info(f"BNO055 initialized in mode 0x{mode:02X}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize BNO055: {e}")
            return False
    
    def _set_mode(self, mode: int):
        """Set operating mode."""
        self._write_byte(BNO055_OPR_MODE, mode)
    
    def _read_byte(self, reg: int) -> int:
        """Read a single byte from register."""
        return self._bus.read_byte_data(self.address, reg)
    
    def _write_byte(self, reg: int, value: int):
        """Write a single byte to register."""
        self._bus.write_byte_data(self.address, reg, value)
    
    def _read_bytes(self, reg: int, length: int) -> bytes:
        """Read multiple bytes from consecutive registers."""
        return bytes(self._bus.read_i2c_block_data(self.address, reg, length))
    
    def _write_bytes(self, reg: int, data: bytes):
        """Write multiple bytes to consecutive registers."""
        self._bus.write_i2c_block_data(self.address, reg, list(data))
    
    def read(self) -> bool:
        """
        Read all sensor data.
        
        Returns:
            True if read successful
        """
        try:
            # Read Euler angles (6 bytes: heading, roll, pitch)
            data = self._read_bytes(BNO055_EUL_HEADING_LSB, 6)
            self.euler[0] = self._parse_int16_le(data[0], data[1]) / EULER_SCALE  # heading
            self.euler[1] = self._parse_int16_le(data[2], data[3]) / EULER_SCALE  # roll
            self.euler[2] = self._parse_int16_le(data[4], data[5]) / EULER_SCALE  # pitch
            
            # Read gyroscope (6 bytes)
            data = self._read_bytes(BNO055_GYR_DATA_X_LSB, 6)
            self.gyro[0] = self._parse_int16_le(data[0], data[1]) / GYRO_SCALE
            self.gyro[1] = self._parse_int16_le(data[2], data[3]) / GYRO_SCALE
            self.gyro[2] = self._parse_int16_le(data[4], data[5]) / GYRO_SCALE
            
            # Read accelerometer (6 bytes)
            data = self._read_bytes(BNO055_ACC_DATA_X_LSB, 6)
            self.accel[0] = self._parse_int16_le(data[0], data[1]) / ACCEL_SCALE
            self.accel[1] = self._parse_int16_le(data[2], data[3]) / ACCEL_SCALE
            self.accel[2] = self._parse_int16_le(data[4], data[5]) / ACCEL_SCALE
            
            # Read linear acceleration (gravity removed) (6 bytes)
            data = self._read_bytes(BNO055_LIA_DATA_X_LSB, 6)
            self.linear_accel[0] = self._parse_int16_le(data[0], data[1]) / LIA_SCALE
            self.linear_accel[1] = self._parse_int16_le(data[2], data[3]) / LIA_SCALE
            self.linear_accel[2] = self._parse_int16_le(data[4], data[5]) / LIA_SCALE
            
            # Read calibration status
            cal_stat = self._read_byte(BNO055_CALIB_STAT)
            self.cal_sys = (cal_stat >> 6) & 0x03
            self.cal_gyro = (cal_stat >> 4) & 0x03
            self.cal_accel = (cal_stat >> 2) & 0x03
            self.cal_mag = cal_stat & 0x03
            
            return True
            
        except Exception as e:
            logger.warning(f"Failed to read sensor data: {e}")
            return False
    
    def get_calibration(self) -> bytes:
        """
        Read calibration offsets from sensor (22 bytes).
        
        Must be in CONFIG mode to read calibration.
        """
        try:
            # Switch to config mode
            current_mode = self._read_byte(BNO055_OPR_MODE)
            self._set_mode(OPR_MODE_CONFIG)
            time.sleep(0.025)
            
            # Read 22 bytes of calibration data
            cal_data = self._read_bytes(BNO055_ACC_OFFSET_X_LSB, 22)
            
            # Restore mode
            self._set_mode(current_mode)
            time.sleep(0.02)
            
            return cal_data
            
        except Exception as e:
            logger.error(f"Failed to read calibration: {e}")
            return b'\x00' * 22
    
    def set_calibration(self, cal_data: bytes) -> bool:
        """
        Write calibration offsets to sensor.
        
        Args:
            cal_data: 22 bytes of calibration data
            
        Returns:
            True if successful
        """
        if len(cal_data) != 22:
            logger.error(f"Calibration data must be 22 bytes, got {len(cal_data)}")
            return False
        
        try:
            # Switch to config mode
            current_mode = self._read_byte(BNO055_OPR_MODE)
            self._set_mode(OPR_MODE_CONFIG)
            time.sleep(0.025)
            
            # Write calibration data
            self._write_bytes(BNO055_ACC_OFFSET_X_LSB, cal_data)
            
            # Restore mode
            self._set_mode(current_mode)
            time.sleep(0.02)
            
            logger.info("Calibration data written to sensor")
            return True
            
        except Exception as e:
            logger.error(f"Failed to write calibration: {e}")
            return False
    
    def get_system_status(self) -> Tuple[int, int, int]:
        """
        Get system status.
        
        Returns:
            Tuple of (status, self_test_result, error)
        """
        status = self._read_byte(BNO055_SYS_STATUS)
        self_test = self._read_byte(BNO055_ST_RESULT)
        error = self._read_byte(BNO055_SYS_ERR)
        return status, self_test, error
    
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
# IMU Fusion BNO055 Class
# =============================================================================

class IMUFusionBNO055:
    """
    Direct I2C interface to BNO055 with built-in sensor fusion.
    
    Drop-in replacement for IMUFusion (serial version) and IMUFusionI2C.
    """
    
    def __init__(self, config: Optional[IMUConfigBNO055] = None,
                 calibration: Optional[IMUCalibrationData] = None):
        self.config = config or IMUConfigBNO055()
        self.calibration = calibration or IMUCalibrationData()
        
        self._driver: Optional[BNO055Driver] = None
        self._data = IMUData()
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._callbacks: list[Callable[[IMUData], None]] = []
        
        # Previous gyro for angular acceleration (offset correction)
        self._prev_gyro = np.zeros(3)
        self._prev_gyro_time = 0.0
        
        # Statistics
        self._msg_count = 0
        self._error_count = 0
        self._last_msg_time = 0.0
        
        # Version info (for API compatibility)
        self.firmware_version = "bno055-i2c-1.0"
        
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
        """
        Save calibration to JSON file.
        
        Reads current calibration from sensor and saves to file.
        """
        if not self.config.calibration_file:
            logger.warning("No calibration file configured")
            return False
        
        # Read current calibration from sensor
        if self._driver:
            cal_bytes = self._driver.get_calibration()
            self.calibration = IMUCalibrationData(
                calibration_bytes=cal_bytes,
                position_offset=self.calibration.position_offset
            )
        
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
            self._driver = BNO055Driver(
                bus=self.config.i2c_bus,
                address=self.config.i2c_address
            )
            if not self._driver.begin(self.config.operation_mode):
                return False
            
            # Apply stored calibration if available
            if self.calibration.calibration_bytes != b'\x00' * 22:
                self._driver.set_calibration(self.calibration.calibration_bytes)
            
            # Start update thread
            self._running = True
            self._thread = threading.Thread(target=self._update_loop, daemon=True)
            self._thread.start()
            
            logger.info(f"IMU BNO055 started at {self.config.update_rate_hz}Hz")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start IMU BNO055: {e}")
            return False
    
    def stop(self):
        """Stop the IMU interface."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._driver:
            self._driver.close()
            self._driver = None
        logger.info("IMU BNO055 stopped")
    
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
                valid=self._data.valid,
                cal_sys=self._data.cal_sys,
                cal_gyro=self._data.cal_gyro,
                cal_accel=self._data.cal_accel,
                cal_mag=self._data.cal_mag
            )
            data.update_age(time.time())
            data.valid = data.age_ms < self.config.max_age_ms
            return data
    
    def add_callback(self, callback: Callable[[IMUData], None]):
        """Register a callback for new IMU data."""
        self._callbacks.append(callback)
    
    def _update_loop(self):
        """Background thread reading sensor at target rate."""
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
        """Perform one update step: read sensor, apply corrections, update data."""
        if not self._driver.read():
            return
        
        now = time.time()
        
        # Get orientation directly from BNO055 fusion
        heading = self._driver.euler[0]  # 0-360
        roll = self._driver.euler[1]
        pitch = self._driver.euler[2]
        
        # Get angular rates
        gx, gy, gz = self._driver.gyro
        
        # Get acceleration (linear or raw based on config)
        if self.config.use_linear_accel:
            ax, ay, az = self._driver.linear_accel
        else:
            ax, ay, az = self._driver.accel
        
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
        
        # Apply off-center mounting correction
        ax, ay, az = self._correct_accel_for_offset(
            ax, ay, az,
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
            self._data.accel_x = ax
            self._data.accel_y = ay
            self._data.accel_z = az
            self._data.valid = True
            self._data.cal_sys = self._driver.cal_sys
            self._data.cal_gyro = self._driver.cal_gyro
            self._data.cal_accel = self._driver.cal_accel
            self._data.cal_mag = self._driver.cal_mag
        
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
    
    def get_calibration_status(self) -> dict:
        """
        Get detailed calibration status.
        
        Returns dict with 'sys', 'gyro', 'accel', 'mag' values (0-3 each).
        3 = fully calibrated, 0 = not calibrated.
        """
        data = self.get_data()
        return {
            "sys": data.cal_sys,
            "gyro": data.cal_gyro,
            "accel": data.cal_accel,
            "mag": data.cal_mag,
            "fully_calibrated": data.is_calibrated
        }
    
    @property
    def stats(self) -> dict:
        """Get statistics about IMU operation."""
        return {
            "message_count": self._msg_count,
            "error_count": self._error_count,
            "last_message_age_ms": (time.time() - self._last_msg_time) * 1000 if self._last_msg_time else float('inf'),
            "connected": self._driver is not None and self._running,
            "firmware_version": self.firmware_version,
            "calibration": self.get_calibration_status()
        }


# =============================================================================
# Calibration Helper
# =============================================================================

class IMUCalibrationBNO055:
    """
    Calibration helper for BNO055.
    
    The BNO055 performs automatic calibration. This class helps monitor
    calibration status and save/restore calibration data.
    """
    
    @staticmethod
    def wait_for_calibration(imu: IMUFusionBNO055, 
                              timeout: float = 60.0,
                              callback: Optional[Callable[[dict], None]] = None) -> bool:
        """
        Wait for the BNO055 to become fully calibrated.
        
        During calibration:
        - Keep gyro still for a few seconds
        - Move accelerometer to several positions and hold still
        - Wave magnetometer in figure-8 pattern
        
        Args:
            imu: Running IMUFusionBNO055 instance
            timeout: Maximum time to wait
            callback: Optional callback for status updates
            
        Returns:
            True if fully calibrated within timeout
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            status = imu.get_calibration_status()
            
            if callback:
                callback(status)
            
            if status["fully_calibrated"]:
                return True
            
            time.sleep(0.5)
        
        return False
    
    @staticmethod
    def run_calibration(config: Optional[IMUConfigBNO055] = None,
                        timeout: float = 60.0) -> Optional[IMUCalibrationData]:
        """
        Run interactive calibration session.
        
        Args:
            config: I2C configuration (uses defaults if None)
            timeout: Maximum calibration time
            
        Returns:
            IMUCalibrationData with sensor offsets, or None on failure
        """
        config = config or IMUConfigBNO055()
        
        try:
            imu = IMUFusionBNO055(config)
            if not imu.start():
                return None
            
            logger.info("Starting BNO055 calibration...")
            logger.info("- Keep gyro still for a few seconds")
            logger.info("- Move accelerometer to 6 positions (each axis up/down)")
            logger.info("- Wave magnetometer in figure-8 pattern")
            
            def status_callback(status):
                logger.info(f"Calibration: sys={status['sys']}, gyro={status['gyro']}, "
                           f"accel={status['accel']}, mag={status['mag']}")
            
            if IMUCalibrationBNO055.wait_for_calibration(imu, timeout, status_callback):
                # Read calibration data from sensor
                cal_bytes = imu._driver.get_calibration()
                
                cal_data = IMUCalibrationData(
                    calibration_bytes=cal_bytes,
                    position_offset=(0.0, 0.0, 0.0)
                )
                
                logger.info("Calibration complete!")
                imu.stop()
                return cal_data
            else:
                logger.warning("Calibration timeout - not fully calibrated")
                imu.stop()
                return None
                
        except Exception as e:
            logger.error(f"Calibration failed: {e}")
            return None


# =============================================================================
# Compatibility Aliases
# =============================================================================

# For drop-in replacement compatibility
IMUFusion = IMUFusionBNO055
IMUCalibration = IMUCalibrationBNO055
