"""
Unit tests for IMU Fusion module.

Tests serial message parsing, checksum validation, and data handling.
"""

import time
import pytest
from unittest.mock import Mock, patch, MagicMock

from src.sensors.imu_fusion import (
    IMUFusion, IMUConfig, IMUData
)


class TestIMUConfig:
    """Tests for IMUConfig dataclass."""
    
    def test_default_config(self):
        """Default config should have expected values."""
        config = IMUConfig()
        
        assert config.port == "/dev/ttyUSB0"
        assert config.baudrate == 115200
        assert config.timeout == 0.1
        assert config.max_age_ms == 200.0
    
    def test_custom_config(self):
        """Custom config should override defaults."""
        config = IMUConfig(port="/dev/ttyAMA0", baudrate=9600)
        
        assert config.port == "/dev/ttyAMA0"
        assert config.baudrate == 9600


class TestIMUData:
    """Tests for IMUData dataclass."""
    
    def test_default_data(self):
        """Default data should have expected values."""
        data = IMUData()
        
        assert data.heading == 0.0
        assert data.pitch == 0.0
        assert data.roll == 0.0
        assert data.yaw_rate == 0.0
        assert data.pitch_rate == 0.0
        assert data.roll_rate == 0.0
        assert data.accel_x == 0.0
        assert data.accel_y == 0.0
        assert data.accel_z == 0.0
        assert data.valid is False
        assert data.age_ms == float('inf')
    
    def test_update_age(self):
        """update_age should calculate age correctly."""
        data = IMUData()
        data.timestamp = time.time() - 0.1  # 100ms ago
        
        data.update_age(time.time())
        
        assert 90 < data.age_ms < 110  # ~100ms with tolerance


class TestParseIMUMessage:
    """Tests for IMU message parsing."""
    
    def test_parse_valid_message_no_checksum(self):
        """Valid message without checksum should parse."""
        imu = IMUFusion()
        
        # $IMU,heading,pitch,roll,yaw_rate,pitch_rate,roll_rate,ax,ay,az
        message = "$IMU,180.5,5.2,-3.1,2.5,0.5,1.0,0.1,-0.2,9.81"
        
        imu._parse_imu_message(message)
        
        with imu._lock:
            assert imu._data.heading == pytest.approx(180.5)
            assert imu._data.pitch == pytest.approx(5.2)
            assert imu._data.roll == pytest.approx(-3.1)
            assert imu._data.yaw_rate == pytest.approx(2.5)
            assert imu._data.pitch_rate == pytest.approx(0.5)
            assert imu._data.roll_rate == pytest.approx(1.0)
            assert imu._data.accel_x == pytest.approx(0.1)
            assert imu._data.accel_y == pytest.approx(-0.2)
            assert imu._data.accel_z == pytest.approx(9.81)
            assert imu._data.valid is True
    
    def test_parse_valid_message_with_checksum(self):
        """Valid message with valid checksum should parse."""
        imu = IMUFusion()
        
        # Build message with checksum
        payload = "IMU,180.5,5.2,-3.1,2.5,0.5,1.0,0.1,-0.2,9.81"
        checksum = 0
        for char in payload:
            checksum ^= ord(char)
        
        message = f"${payload}*{checksum:02X}"
        
        imu._parse_imu_message(message)
        
        with imu._lock:
            assert imu._data.heading == pytest.approx(180.5)
            assert imu._data.valid is True
    
    def test_parse_invalid_checksum(self):
        """Invalid checksum should be rejected."""
        imu = IMUFusion()
        
        message = "$IMU,180.5,5.2,-3.1,2.5,0.5,1.0,0.1,-0.2,9.81*FF"  # Wrong checksum
        
        initial_errors = imu._error_count
        imu._parse_imu_message(message)
        
        assert imu._error_count == initial_errors + 1
    
    def test_parse_incomplete_message(self):
        """Incomplete message should be silently ignored (not crash)."""
        imu = IMUFusion()
        
        message = "$IMU,180.5,5.2,-3.1"  # Missing fields
        
        # Parser requires >= 10 fields, so this is silently ignored
        # The important thing is it doesn't crash or update data
        initial_msg_count = imu._msg_count
        imu._parse_imu_message(message)
        
        # Message count should not increase (message was ignored)
        assert imu._msg_count == initial_msg_count
    
    def test_parse_invalid_values(self):
        """Non-numeric values should be rejected."""
        imu = IMUFusion()
        
        message = "$IMU,invalid,5.2,-3.1,2.5,0.5,1.0,0.1,-0.2,9.81"
        
        initial_errors = imu._error_count
        imu._parse_imu_message(message)
        
        assert imu._error_count == initial_errors + 1


class TestGetData:
    """Tests for get_data method."""
    
    def test_get_data_returns_copy(self):
        """get_data should return a copy of data."""
        imu = IMUFusion()
        
        data1 = imu.get_data()
        data1.heading = 999.0
        
        data2 = imu.get_data()
        
        assert data2.heading != 999.0
    
    def test_get_data_updates_age(self):
        """get_data should update age field."""
        imu = IMUFusion()
        
        # Set a known timestamp
        with imu._lock:
            imu._data.timestamp = time.time() - 0.1
            imu._data.valid = True
        
        data = imu.get_data()
        
        # Age should be ~100ms
        assert 50 < data.age_ms < 150
    
    def test_get_data_validity_based_on_age(self):
        """get_data should set valid=False if data is stale."""
        imu = IMUFusion()
        
        # Set old timestamp
        with imu._lock:
            imu._data.timestamp = time.time() - 1.0  # 1 second old
            imu._data.valid = True
        
        data = imu.get_data()
        
        # Should be invalid due to age > 200ms
        assert data.valid is False
    
    def test_get_data_valid_when_fresh(self):
        """get_data should set valid=True if data is fresh."""
        imu = IMUFusion()
        
        # Set recent timestamp
        with imu._lock:
            imu._data.timestamp = time.time() - 0.05  # 50ms old
            imu._data.valid = True
        
        data = imu.get_data()
        
        # Should be valid
        assert data.valid is True


class TestIMUFusionStats:
    """Tests for IMU statistics."""
    
    def test_stats_structure(self):
        """Stats should contain expected fields."""
        imu = IMUFusion()
        stats = imu.stats
        
        assert "message_count" in stats
        assert "error_count" in stats
        assert "last_message_age_ms" in stats
        assert "connected" in stats
    
    def test_initial_stats(self):
        """Initial stats should be zero/disconnected."""
        imu = IMUFusion()
        stats = imu.stats
        
        assert stats["message_count"] == 0
        assert stats["error_count"] == 0
        assert stats["connected"] is False
    
    def test_stats_update_after_parse(self):
        """Stats should update after parsing message."""
        imu = IMUFusion()
        
        message = "$IMU,180.5,5.2,-3.1,2.5,0.5,1.0,0.1,-0.2,9.81"
        imu._parse_imu_message(message)
        
        stats = imu.stats
        assert stats["message_count"] == 1


class TestCallbacks:
    """Tests for IMU data callbacks."""
    
    def test_add_callback(self):
        """Callbacks should be addable."""
        imu = IMUFusion()
        callback = Mock()
        
        imu.add_callback(callback)
        
        assert callback in imu._callbacks
    
    def test_callback_called_on_parse(self):
        """Callbacks should be called when message is parsed."""
        imu = IMUFusion()
        callback = Mock()
        imu.add_callback(callback)
        
        message = "$IMU,180.5,5.2,-3.1,2.5,0.5,1.0,0.1,-0.2,9.81"
        imu._parse_imu_message(message)
        
        callback.assert_called_once()
    
    def test_callback_receives_data(self):
        """Callback should receive IMUData."""
        imu = IMUFusion()
        callback = Mock()
        imu.add_callback(callback)
        
        message = "$IMU,180.5,5.2,-3.1,2.5,0.5,1.0,0.1,-0.2,9.81"
        imu._parse_imu_message(message)
        
        call_arg = callback.call_args[0][0]
        assert isinstance(call_arg, IMUData)
        assert call_arg.heading == pytest.approx(180.5)
    
    def test_callback_exception_handled(self):
        """Exception in callback should not break parsing."""
        imu = IMUFusion()
        bad_callback = Mock(side_effect=Exception("Test error"))
        good_callback = Mock()
        imu.add_callback(bad_callback)
        imu.add_callback(good_callback)
        
        message = "$IMU,180.5,5.2,-3.1,2.5,0.5,1.0,0.1,-0.2,9.81"
        imu._parse_imu_message(message)
        
        # Good callback should still be called
        good_callback.assert_called_once()


class TestIMUCalibration:
    """Tests for IMU calibration commands."""
    
    def test_calibration_import(self):
        """IMUCalibration should be importable."""
        from src.sensors.imu_fusion import IMUCalibration
        
        assert IMUCalibration is not None
