"""
Tests for IMU Fusion I2C Module
===============================

Tests for the I2C-based IMU interface including:
- Madgwick AHRS filter
- ICM-20948 driver (mocked I2C)
- IMUFusionI2C class
- Calibration
- Offset correction
"""

import pytest
import math
import time
import json
from unittest.mock import MagicMock, patch, PropertyMock
from pathlib import Path

import numpy as np

from src.sensors.imu_fusion_i2c import (
    IMUData,
    IMUCalibrationData,
    IMUConfigI2C,
    MadgwickAHRS,
    ICM20948Driver,
    IMUFusionI2C,
    IMUCalibrationI2C,
    DEG_TO_RAD,
    RAD_TO_DEG,
    ICM20948_WHO_AM_I_VALUE,
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
    
    def test_update_age(self):
        """Test age calculation."""
        data = IMUData(timestamp=100.0)
        data.update_age(100.05)
        assert data.age_ms == pytest.approx(50.0, abs=0.1)


class TestIMUCalibrationData:
    """Tests for IMUCalibrationData dataclass."""
    
    def test_default_values(self):
        """Test default calibration values."""
        cal = IMUCalibrationData()
        assert cal.mag_offset == (0.0, 0.0, 0.0)
        assert cal.mag_scale == (1.0, 1.0, 1.0)
        assert cal.position_offset == (0.0, 0.0, 0.0)
    
    def test_to_dict(self):
        """Test serialization to dict."""
        cal = IMUCalibrationData(
            mag_offset=(1.0, 2.0, 3.0),
            mag_scale=(1.1, 1.2, 1.3),
            position_offset=(0.5, 0.6, 0.7)
        )
        d = cal.to_dict()
        assert d["mag_offset"] == [1.0, 2.0, 3.0]
        assert d["mag_scale"] == [1.1, 1.2, 1.3]
        assert d["position_offset"] == [0.5, 0.6, 0.7]
    
    def test_from_dict(self):
        """Test deserialization from dict."""
        d = {
            "mag_offset": [1.0, 2.0, 3.0],
            "mag_scale": [1.1, 1.2, 1.3],
            "position_offset": [0.5, 0.6, 0.7]
        }
        cal = IMUCalibrationData.from_dict(d)
        assert cal.mag_offset == (1.0, 2.0, 3.0)
        assert cal.mag_scale == (1.1, 1.2, 1.3)
        assert cal.position_offset == (0.5, 0.6, 0.7)
    
    def test_from_dict_missing_keys(self):
        """Test deserialization with missing keys uses defaults."""
        cal = IMUCalibrationData.from_dict({})
        assert cal.mag_offset == (0.0, 0.0, 0.0)
        assert cal.mag_scale == (1.0, 1.0, 1.0)


class TestIMUConfigI2C:
    """Tests for IMUConfigI2C dataclass."""
    
    def test_default_values(self):
        """Test default configuration values."""
        config = IMUConfigI2C()
        assert config.i2c_bus == 1
        assert config.i2c_address == 0x68
        assert config.update_rate_hz == 100.0
        assert config.madgwick_beta == 0.1


# =============================================================================
# Test Madgwick AHRS Filter
# =============================================================================

class TestMadgwickAHRS:
    """Tests for Madgwick AHRS filter."""
    
    def test_initialization(self):
        """Test filter initialization."""
        filter = MadgwickAHRS(sample_freq=100.0, beta=0.1)
        assert filter.sample_freq == 100.0
        assert filter.beta == 0.1
        # Quaternion should be identity
        assert filter.q[0] == pytest.approx(1.0)
        assert filter.q[1] == pytest.approx(0.0)
        assert filter.q[2] == pytest.approx(0.0)
        assert filter.q[3] == pytest.approx(0.0)
    
    def test_initial_angles(self):
        """Test initial Euler angles are zero."""
        filter = MadgwickAHRS()
        assert filter.roll == pytest.approx(0.0, abs=0.1)
        assert filter.pitch == pytest.approx(0.0, abs=0.1)
        # Yaw is undefined without magnetometer update
    
    def test_update_with_gravity(self):
        """Test filter converges with gravity-only input."""
        filter = MadgwickAHRS(sample_freq=100.0, beta=0.5)
        
        # Simulate sensor at rest: gravity pointing down (0, 0, 1)
        # Gyro = 0, Accel = (0, 0, 1g), Mag = (1, 0, 0) pointing north
        for _ in range(100):
            filter.update(0, 0, 0, 0, 0, 1, 1, 0, 0)
        
        # Should converge to level orientation
        assert abs(filter.roll) < 5.0
        assert abs(filter.pitch) < 5.0
    
    def test_update_with_tilt(self):
        """Test filter detects tilt."""
        filter = MadgwickAHRS(sample_freq=100.0, beta=0.5)
        
        # Simulate 45-degree roll (gravity at 45° between Y and Z)
        sin45 = math.sin(math.radians(45))
        cos45 = math.cos(math.radians(45))
        
        for _ in range(200):
            filter.update(0, 0, 0, 0, sin45, cos45, 1, 0, 0)
        
        # Should detect approximately 45-degree roll
        assert filter.roll == pytest.approx(45.0, abs=10.0)
    
    def test_gyro_integration(self):
        """Test pure gyro integration without corrections."""
        filter = MadgwickAHRS(sample_freq=100.0, beta=0.0)  # Zero beta = pure gyro
        
        # Rotate 90 deg/s around Z for 1 second
        for _ in range(100):
            filter.update(0, 0, 90, 0, 0, 0, 0, 0, 0)
        
        # Should have rotated approximately 90 degrees in yaw
        # Note: with zero beta, accelerometer doesn't correct, so heading drifts
        # This is just testing that gyro integration works
        assert filter.yaw != pytest.approx(0.0, abs=1.0)
    
    def test_quaternion_remains_normalized(self):
        """Test quaternion stays normalized after updates."""
        filter = MadgwickAHRS()
        
        for _ in range(1000):
            filter.update(
                np.random.randn() * 10,  # Random gyro
                np.random.randn() * 10,
                np.random.randn() * 10,
                np.random.randn(),  # Random accel
                np.random.randn(),
                np.random.randn() + 1,  # Bias toward gravity
                np.random.randn(),  # Random mag
                np.random.randn(),
                np.random.randn()
            )
        
        norm = np.linalg.norm(filter.q)
        assert norm == pytest.approx(1.0, abs=1e-6)


# =============================================================================
# Test ICM-20948 Driver
# =============================================================================

class TestICM20948Driver:
    """Tests for ICM-20948 I2C driver with mocked I2C bus."""
    
    @pytest.fixture
    def mock_smbus(self):
        """Create a mock SMBus."""
        with patch('src.sensors.imu_fusion_i2c.smbus2') as mock:
            mock_bus = MagicMock()
            mock.SMBus.return_value = mock_bus
            yield mock_bus
    
    @pytest.fixture
    def mock_has_smbus(self):
        """Mock HAS_SMBUS to True."""
        with patch('src.sensors.imu_fusion_i2c.HAS_SMBUS', True):
            yield
    
    def test_parse_int16(self):
        """Test big-endian signed integer parsing."""
        assert ICM20948Driver._parse_int16(0x00, 0x00) == 0
        assert ICM20948Driver._parse_int16(0x00, 0x01) == 1
        assert ICM20948Driver._parse_int16(0x01, 0x00) == 256
        assert ICM20948Driver._parse_int16(0xFF, 0xFF) == -1
        assert ICM20948Driver._parse_int16(0x80, 0x00) == -32768
        assert ICM20948Driver._parse_int16(0x7F, 0xFF) == 32767
    
    def test_parse_int16_le(self):
        """Test little-endian signed integer parsing."""
        assert ICM20948Driver._parse_int16_le(0x00, 0x00) == 0
        assert ICM20948Driver._parse_int16_le(0x01, 0x00) == 1
        assert ICM20948Driver._parse_int16_le(0x00, 0x01) == 256
        assert ICM20948Driver._parse_int16_le(0xFF, 0xFF) == -1
    
    def test_begin_success(self, mock_smbus, mock_has_smbus):
        """Test successful initialization."""
        # Mock WHO_AM_I response
        mock_smbus.read_byte_data.return_value = ICM20948_WHO_AM_I_VALUE
        
        driver = ICM20948Driver()
        assert driver.begin() is True
        
        # Verify WHO_AM_I was checked
        mock_smbus.read_byte_data.assert_any_call(0x68, 0x00)
    
    def test_begin_wrong_device(self, mock_smbus, mock_has_smbus):
        """Test initialization fails with wrong device."""
        mock_smbus.read_byte_data.return_value = 0x00  # Wrong ID
        
        driver = ICM20948Driver()
        assert driver.begin() is False
    
    def test_read_sensors(self, mock_smbus, mock_has_smbus):
        """Test sensor data reading."""
        mock_smbus.read_byte_data.return_value = ICM20948_WHO_AM_I_VALUE
        
        # Mock sensor data: 14 bytes for accel + gyro + temp
        # Accel: (0x0100, 0x0200, 0x0300) = (256, 512, 768) raw
        # Gyro: (0x0400, 0x0500, 0x0600) = (1024, 1280, 1536) raw
        # Temp: (0x0700) = 1792 raw
        mock_smbus.read_i2c_block_data.side_effect = [
            [0x01, 0x00, 0x02, 0x00, 0x03, 0x00,  # Accel
             0x04, 0x00, 0x05, 0x00, 0x06, 0x00,  # Gyro
             0x07, 0x00],  # Temp
            [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]  # Mag (ST1 + data)
        ]
        mock_smbus.read_byte_data.side_effect = [
            ICM20948_WHO_AM_I_VALUE,  # WHO_AM_I
            0x01  # Mag ST1 (data ready)
        ]
        
        driver = ICM20948Driver()
        driver.begin()
        
        # Reset mock for read test
        mock_smbus.read_i2c_block_data.side_effect = [
            [0x01, 0x00, 0x02, 0x00, 0x03, 0x00,
             0x04, 0x00, 0x05, 0x00, 0x06, 0x00,
             0x07, 0x00],
            [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        ]
        mock_smbus.read_byte_data.return_value = 0x01
        
        result = driver.read()
        assert result is True
        
        # Verify accel values (raw / scale)
        # 256 / 8192 (4g scale) ≈ 0.031 g
        assert driver.accel[0] == pytest.approx(256 / 8192.0, rel=0.01)


# =============================================================================
# Test Offset Correction
# =============================================================================

class TestOffsetCorrection:
    """Tests for off-center mounting acceleration correction."""
    
    def test_no_correction_at_zero_offset(self):
        """Test no correction when offset is zero."""
        config = IMUConfigI2C()
        cal = IMUCalibrationData(position_offset=(0.0, 0.0, 0.0))
        
        with patch.object(ICM20948Driver, 'begin', return_value=True):
            with patch.object(ICM20948Driver, 'read', return_value=True):
                imu = IMUFusionI2C(config, cal)
                
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
        config = IMUConfigI2C()
        cal = IMUCalibrationData(position_offset=(1.0, 0.0, 0.0))  # 1m forward
        
        with patch.object(ICM20948Driver, 'begin', return_value=True):
            imu = IMUFusionI2C(config, cal)
            
            # Pure yaw rotation at 57.3 deg/s = 1 rad/s
            # Centripetal at 1m = omega^2 * r = 1^2 * 1 = 1 m/s²
            ax, ay, az = imu._correct_accel_for_offset(
                1.0, 0.0, 0.0,  # Initial accel
                0.0, 0.0, 57.2957795,  # 1 rad/s yaw
                0.0, 0.0, 0.0  # No angular acceleration
            )
            
            # Should subtract centripetal (≈1 m/s²)
            assert ax == pytest.approx(0.0, abs=0.1)


# =============================================================================
# Test IMUFusionI2C Class
# =============================================================================

class TestIMUFusionI2C:
    """Tests for IMUFusionI2C class."""
    
    @pytest.fixture
    def mock_driver(self):
        """Create a mock ICM20948 driver."""
        with patch('src.sensors.imu_fusion_i2c.ICM20948Driver') as MockDriver:
            driver = MagicMock()
            driver.begin.return_value = True
            driver.read.return_value = True
            driver.accel = np.array([0.0, 0.0, 1.0])  # 1g down
            driver.gyro = np.array([0.0, 0.0, 0.0])
            driver.mag = np.array([1.0, 0.0, 0.0])
            MockDriver.return_value = driver
            yield driver
    
    def test_start_success(self, mock_driver):
        """Test successful start."""
        imu = IMUFusionI2C()
        assert imu.start() is True
        assert imu._running is True
        imu.stop()
    
    def test_stop(self, mock_driver):
        """Test stop."""
        imu = IMUFusionI2C()
        imu.start()
        imu.stop()
        assert imu._running is False
    
    def test_get_data(self, mock_driver):
        """Test get_data returns IMUData."""
        imu = IMUFusionI2C()
        imu.start()
        time.sleep(0.05)  # Let update thread run
        
        data = imu.get_data()
        assert isinstance(data, IMUData)
        
        imu.stop()
    
    def test_callbacks(self, mock_driver):
        """Test callback registration and invocation."""
        imu = IMUFusionI2C()
        
        callback_data = []
        def callback(data):
            callback_data.append(data)
        
        imu.add_callback(callback)
        imu.start()
        time.sleep(0.1)  # Let update thread run
        imu.stop()
        
        assert len(callback_data) > 0
    
    def test_stats(self, mock_driver):
        """Test stats property."""
        imu = IMUFusionI2C()
        imu.start()
        time.sleep(0.05)
        
        stats = imu.stats
        assert "message_count" in stats
        assert "error_count" in stats
        assert "connected" in stats
        assert "firmware_version" in stats
        
        imu.stop()
    
    def test_calibration_save_load(self, mock_driver, tmp_path):
        """Test calibration save and load."""
        cal_file = tmp_path / "cal.json"
        
        config = IMUConfigI2C(calibration_file=str(cal_file))
        cal = IMUCalibrationData(
            mag_offset=(1.0, 2.0, 3.0),
            mag_scale=(1.1, 1.2, 1.3),
            position_offset=(0.5, 0.6, 0.7)
        )
        
        imu = IMUFusionI2C(config, cal)
        assert imu.save_calibration() is True
        assert cal_file.exists()
        
        # Create new instance that loads calibration
        imu2 = IMUFusionI2C(config)
        assert imu2.calibration.mag_offset == (1.0, 2.0, 3.0)
        assert imu2.calibration.mag_scale == (1.1, 1.2, 1.3)


# =============================================================================
# Test API Compatibility
# =============================================================================

class TestAPICompatibility:
    """Test that IMUFusionI2C has same API as IMUFusion."""
    
    def test_has_required_methods(self):
        """Test that all required methods exist."""
        # Check class has the methods
        assert hasattr(IMUFusionI2C, 'start')
        assert hasattr(IMUFusionI2C, 'stop')
        assert hasattr(IMUFusionI2C, 'get_data')
        assert hasattr(IMUFusionI2C, 'add_callback')
        assert hasattr(IMUFusionI2C, 'save_calibration')
        assert hasattr(IMUFusionI2C, 'stats')
        
        # Check instance has attributes
        imu = IMUFusionI2C()
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
