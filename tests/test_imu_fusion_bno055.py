"""
Tests for IMU Fusion BNO055 Module
==================================

Tests for the BNO055 I2C-based IMU interface including:
- BNO055 driver (mocked I2C)
- IMUFusionBNO055 class
- Calibration
- Offset correction
"""

import pytest
import math
import time
import json
from unittest.mock import MagicMock, patch
from pathlib import Path

import numpy as np

from src.sensors.imu_fusion_bno055 import (
    IMUData,
    IMUCalibrationData,
    IMUConfigBNO055,
    BNO055Driver,
    IMUFusionBNO055,
    IMUCalibrationBNO055,
    DEG_TO_RAD,
    BNO055_CHIP_ID_VALUE,
    OPR_MODE_NDOF,
)


# =============================================================================
# Test Data Classes
# =============================================================================

class TestIMUData:
    """Tests for IMUData dataclass."""
    
    def test_default_values(self):
        """Test default IMUData values."""
        data = IMUData()
        assert data.heading == 0.0
        assert data.pitch == 0.0
        assert data.roll == 0.0
        assert data.valid is False
        assert data.age_ms == float('inf')
        assert data.cal_sys == 0
        assert data.cal_gyro == 0
        assert data.cal_accel == 0
        assert data.cal_mag == 0
    
    def test_update_age(self):
        """Test age calculation."""
        data = IMUData(timestamp=100.0)
        data.update_age(100.05)
        assert data.age_ms == pytest.approx(50.0, abs=0.1)
    
    def test_is_calibrated(self):
        """Test calibration status check."""
        data = IMUData(cal_sys=3, cal_gyro=3, cal_accel=2, cal_mag=3)
        assert data.is_calibrated is True
        
        data = IMUData(cal_sys=2, cal_gyro=3, cal_accel=3, cal_mag=3)
        assert data.is_calibrated is False


class TestIMUCalibrationData:
    """Tests for IMUCalibrationData dataclass."""
    
    def test_default_values(self):
        """Test default calibration values."""
        cal = IMUCalibrationData()
        assert cal.calibration_bytes == b'\x00' * 22
        assert cal.position_offset == (0.0, 0.0, 0.0)
    
    def test_to_dict(self):
        """Test serialization to dict."""
        cal = IMUCalibrationData(
            calibration_bytes=b'\x01\x02\x03' + b'\x00' * 19,
            position_offset=(0.5, 0.6, 0.7)
        )
        d = cal.to_dict()
        assert "calibration_bytes" in d
        assert d["position_offset"] == [0.5, 0.6, 0.7]
    
    def test_from_dict(self):
        """Test deserialization from dict."""
        import base64
        cal_bytes = b'\x01\x02\x03' + b'\x00' * 19
        d = {
            "calibration_bytes": base64.b64encode(cal_bytes).decode('ascii'),
            "position_offset": [0.5, 0.6, 0.7]
        }
        cal = IMUCalibrationData.from_dict(d)
        assert cal.calibration_bytes == cal_bytes
        assert cal.position_offset == (0.5, 0.6, 0.7)
    
    def test_from_dict_missing_keys(self):
        """Test deserialization with missing keys uses defaults."""
        cal = IMUCalibrationData.from_dict({})
        assert cal.calibration_bytes == b'\x00' * 22
        assert cal.position_offset == (0.0, 0.0, 0.0)


class TestIMUConfigBNO055:
    """Tests for IMUConfigBNO055 dataclass."""
    
    def test_default_values(self):
        """Test default configuration values."""
        config = IMUConfigBNO055()
        assert config.i2c_bus == 1
        assert config.i2c_address == 0x28
        assert config.update_rate_hz == 100.0
        assert config.operation_mode == OPR_MODE_NDOF
        assert config.use_linear_accel is True


# =============================================================================
# Test BNO055 Driver
# =============================================================================

class TestBNO055Driver:
    """Tests for BNO055 I2C driver with mocked I2C bus."""
    
    @pytest.fixture
    def mock_smbus(self):
        """Create a mock SMBus."""
        with patch('src.sensors.imu_fusion_bno055.smbus2') as mock:
            mock_bus = MagicMock()
            mock.SMBus.return_value = mock_bus
            yield mock_bus
    
    @pytest.fixture
    def mock_has_smbus(self):
        """Mock HAS_SMBUS to True."""
        with patch('src.sensors.imu_fusion_bno055.HAS_SMBUS', True):
            yield
    
    def test_parse_int16_le(self):
        """Test little-endian signed integer parsing."""
        assert BNO055Driver._parse_int16_le(0x00, 0x00) == 0
        assert BNO055Driver._parse_int16_le(0x01, 0x00) == 1
        assert BNO055Driver._parse_int16_le(0x00, 0x01) == 256
        assert BNO055Driver._parse_int16_le(0xFF, 0xFF) == -1
        assert BNO055Driver._parse_int16_le(0x00, 0x80) == -32768
        assert BNO055Driver._parse_int16_le(0xFF, 0x7F) == 32767
    
    def test_begin_success(self, mock_smbus, mock_has_smbus):
        """Test successful initialization."""
        # Mock chip ID response
        mock_smbus.read_byte_data.return_value = BNO055_CHIP_ID_VALUE
        
        driver = BNO055Driver()
        assert driver.begin() is True
        
        # Verify chip ID was checked
        mock_smbus.read_byte_data.assert_any_call(0x28, 0x00)
    
    def test_begin_wrong_device(self, mock_smbus, mock_has_smbus):
        """Test initialization fails with wrong device."""
        mock_smbus.read_byte_data.return_value = 0x00  # Wrong ID
        
        driver = BNO055Driver()
        assert driver.begin() is False
    
    def test_read_sensors(self, mock_smbus, mock_has_smbus):
        """Test sensor data reading."""
        mock_smbus.read_byte_data.return_value = BNO055_CHIP_ID_VALUE
        
        driver = BNO055Driver()
        driver.begin()
        
        # Mock sensor data reads
        # Euler: heading=180°, roll=10°, pitch=5° (in 1/16 degree units)
        euler_data = [0x40, 0x0B, 0xA0, 0x00, 0x50, 0x00]  # 2880, 160, 80
        gyro_data = [0x10, 0x00, 0x20, 0x00, 0x30, 0x00]
        accel_data = [0x00, 0x01, 0x00, 0x02, 0x00, 0x03]
        lia_data = [0x10, 0x00, 0x20, 0x00, 0x30, 0x00]
        
        mock_smbus.read_i2c_block_data.side_effect = [
            euler_data,
            gyro_data,
            accel_data,
            lia_data
        ]
        mock_smbus.read_byte_data.return_value = 0xFF  # Full calibration
        
        result = driver.read()
        assert result is True
        
        # Verify Euler angle parsing (2880 / 16 = 180°)
        assert driver.euler[0] == pytest.approx(180.0, rel=0.01)
    
    def test_calibration_status_parsing(self, mock_smbus, mock_has_smbus):
        """Test calibration status byte parsing."""
        mock_smbus.read_byte_data.return_value = BNO055_CHIP_ID_VALUE
        
        driver = BNO055Driver()
        driver.begin()
        
        # Set up sensor reads
        mock_smbus.read_i2c_block_data.return_value = [0] * 6
        
        # Cal status: sys=3, gyro=2, accel=1, mag=0 = 0b11_10_01_00 = 0xE4
        mock_smbus.read_byte_data.return_value = 0xE4
        
        driver.read()
        
        assert driver.cal_sys == 3
        assert driver.cal_gyro == 2
        assert driver.cal_accel == 1
        assert driver.cal_mag == 0


# =============================================================================
# Test Offset Correction
# =============================================================================

class TestOffsetCorrection:
    """Tests for off-center mounting acceleration correction."""
    
    def test_no_correction_at_zero_offset(self):
        """Test no correction when offset is zero."""
        config = IMUConfigBNO055()
        cal = IMUCalibrationData(position_offset=(0.0, 0.0, 0.0))
        
        with patch.object(BNO055Driver, 'begin', return_value=True):
            imu = IMUFusionBNO055(config, cal)
            
            ax, ay, az = imu._correct_accel_for_offset(
                1.0, 2.0, 3.0,  # accel
                10.0, 20.0, 30.0,  # gyro
                5.0, 10.0, 15.0  # alpha
            )
            
            assert ax == 1.0
            assert ay == 2.0
            assert az == 3.0
    
    def test_centripetal_correction(self):
        """Test centripetal acceleration is subtracted."""
        config = IMUConfigBNO055()
        cal = IMUCalibrationData(position_offset=(1.0, 0.0, 0.0))  # 1m forward
        
        with patch.object(BNO055Driver, 'begin', return_value=True):
            imu = IMUFusionBNO055(config, cal)
            
            # Pure yaw rotation at 57.3 deg/s = 1 rad/s
            ax, ay, az = imu._correct_accel_for_offset(
                1.0, 0.0, 0.0,  # Initial accel
                0.0, 0.0, 57.2957795,  # 1 rad/s yaw
                0.0, 0.0, 0.0  # No angular acceleration
            )
            
            # Should subtract centripetal (≈1 m/s²)
            assert ax == pytest.approx(0.0, abs=0.1)


# =============================================================================
# Test IMUFusionBNO055 Class
# =============================================================================

class TestIMUFusionBNO055:
    """Tests for IMUFusionBNO055 class."""
    
    @pytest.fixture
    def mock_driver(self):
        """Create a mock BNO055 driver."""
        with patch('src.sensors.imu_fusion_bno055.BNO055Driver') as MockDriver:
            driver = MagicMock()
            driver.begin.return_value = True
            driver.read.return_value = True
            driver.euler = np.array([180.0, 5.0, 2.0])  # heading, roll, pitch
            driver.gyro = np.array([0.0, 0.0, 0.0])
            driver.accel = np.array([0.0, 0.0, 9.81])
            driver.linear_accel = np.array([0.0, 0.0, 0.0])
            driver.cal_sys = 3
            driver.cal_gyro = 3
            driver.cal_accel = 3
            driver.cal_mag = 3
            driver.get_calibration.return_value = b'\x00' * 22
            MockDriver.return_value = driver
            yield driver
    
    def test_start_success(self, mock_driver):
        """Test successful start."""
        imu = IMUFusionBNO055()
        assert imu.start() is True
        assert imu._running is True
        imu.stop()
    
    def test_stop(self, mock_driver):
        """Test stop."""
        imu = IMUFusionBNO055()
        imu.start()
        imu.stop()
        assert imu._running is False
    
    def test_get_data(self, mock_driver):
        """Test get_data returns IMUData with BNO055-specific fields."""
        imu = IMUFusionBNO055()
        imu.start()
        time.sleep(0.05)  # Let update thread run
        
        data = imu.get_data()
        assert isinstance(data, IMUData)
        assert hasattr(data, 'cal_sys')
        assert hasattr(data, 'cal_gyro')
        assert hasattr(data, 'cal_accel')
        assert hasattr(data, 'cal_mag')
        
        imu.stop()
    
    def test_callbacks(self, mock_driver):
        """Test callback registration and invocation."""
        imu = IMUFusionBNO055()
        
        callback_data = []
        def callback(data):
            callback_data.append(data)
        
        imu.add_callback(callback)
        imu.start()
        time.sleep(0.1)  # Let update thread run
        imu.stop()
        
        assert len(callback_data) > 0
    
    def test_stats(self, mock_driver):
        """Test stats property includes calibration."""
        imu = IMUFusionBNO055()
        imu.start()
        time.sleep(0.05)
        
        stats = imu.stats
        assert "message_count" in stats
        assert "error_count" in stats
        assert "connected" in stats
        assert "firmware_version" in stats
        assert "calibration" in stats
        
        imu.stop()
    
    def test_get_calibration_status(self, mock_driver):
        """Test calibration status retrieval."""
        imu = IMUFusionBNO055()
        imu.start()
        time.sleep(0.05)
        
        status = imu.get_calibration_status()
        assert "sys" in status
        assert "gyro" in status
        assert "accel" in status
        assert "mag" in status
        assert "fully_calibrated" in status
        
        imu.stop()
    
    def test_calibration_save_load(self, mock_driver, tmp_path):
        """Test calibration save and load."""
        cal_file = tmp_path / "cal.json"
        
        config = IMUConfigBNO055(calibration_file=str(cal_file))
        cal = IMUCalibrationData(
            calibration_bytes=b'\x01\x02\x03' + b'\x00' * 19,
            position_offset=(0.5, 0.6, 0.7)
        )
        
        imu = IMUFusionBNO055(config, cal)
        imu.start()
        time.sleep(0.02)
        assert imu.save_calibration() is True
        assert cal_file.exists()
        imu.stop()
        
        # Create new instance that loads calibration
        imu2 = IMUFusionBNO055(config)
        assert imu2.calibration.position_offset == (0.5, 0.6, 0.7)


# =============================================================================
# Test API Compatibility
# =============================================================================

class TestAPICompatibility:
    """Test that IMUFusionBNO055 has same API as IMUFusion."""
    
    def test_has_required_methods(self):
        """Test that all required methods exist."""
        # Check class has the methods
        assert hasattr(IMUFusionBNO055, 'start')
        assert hasattr(IMUFusionBNO055, 'stop')
        assert hasattr(IMUFusionBNO055, 'get_data')
        assert hasattr(IMUFusionBNO055, 'add_callback')
        assert hasattr(IMUFusionBNO055, 'save_calibration')
        assert hasattr(IMUFusionBNO055, 'stats')
        
        # Check instance has attributes
        imu = IMUFusionBNO055()
        assert hasattr(imu, 'calibration')
        assert hasattr(imu, 'firmware_version')
    
    def test_data_class_compatibility(self):
        """Test IMUData has all required fields."""
        data = IMUData()
        assert hasattr(data, 'timestamp')
        assert hasattr(data, 'heading')
        assert hasattr(data, 'pitch')
        assert hasattr(data, 'roll')
        assert hasattr(data, 'yaw_rate')
        assert hasattr(data, 'pitch_rate')
        assert hasattr(data, 'roll_rate')
        assert hasattr(data, 'accel_x')
        assert hasattr(data, 'accel_y')
        assert hasattr(data, 'accel_z')
        assert hasattr(data, 'valid')
        assert hasattr(data, 'age_ms')
        # BNO055-specific
        assert hasattr(data, 'cal_sys')
        assert hasattr(data, 'cal_mag')
