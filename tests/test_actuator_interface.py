"""
Unit tests for Actuator Interface module.

Tests serial message parsing, checksum validation, mock actuator behavior,
and status handling.
"""

import time
import pytest
from unittest.mock import Mock, patch, MagicMock

from src.control.actuator_interface import (
    ActuatorInterface, ActuatorConfig, ActuatorStatus,
    MockActuatorInterface, MCUFaultCode
)


class TestMCUFaultCode:
    """Tests for MCUFaultCode enumeration."""
    
    def test_all_fault_codes_defined(self):
        """All expected fault codes should be defined."""
        assert MCUFaultCode.NONE == 0
        assert MCUFaultCode.OVERCURRENT == 1
        assert MCUFaultCode.STALL == 2
        assert MCUFaultCode.POSITION_LIMIT == 3
        assert MCUFaultCode.SENSOR == 4
        assert MCUFaultCode.WATCHDOG == 5


class TestActuatorConfig:
    """Tests for ActuatorConfig dataclass."""
    
    def test_default_config(self):
        """Default config should have expected values."""
        config = ActuatorConfig()
        
        assert config.port == "/dev/ttyAMA1"
        assert config.baudrate == 115200
        assert config.timeout == 0.1
        assert config.watchdog_timeout_ms == 2000
        assert config.max_status_age_s == 0.2
        assert config.min_command_interval_s == 0.02
    
    def test_custom_config(self):
        """Custom config should override defaults."""
        config = ActuatorConfig(port="/dev/ttyUSB0", baudrate=9600)
        
        assert config.port == "/dev/ttyUSB0"
        assert config.baudrate == 9600


class TestActuatorStatus:
    """Tests for ActuatorStatus dataclass."""
    
    def test_default_status(self):
        """Default status should have expected values."""
        status = ActuatorStatus()
        
        assert status.target_angle == 0.0
        assert status.actual_angle == 0.0
        assert status.clutch_engaged is False
        assert status.voltage == 0.0
        assert status.current == 0.0
        assert status.fault_code == MCUFaultCode.NONE
        assert status.valid is False
    
    def test_get_angle_degrees(self):
        """get_angle_degrees should convert normalized to degrees."""
        status = ActuatorStatus(actual_angle=0.5)
        
        # 0.5 normalized with max_angle=30 = 15 degrees
        assert status.get_angle_degrees() == 15.0
        assert status.get_angle_degrees(max_angle=25.0) == 12.5
    
    def test_get_target_degrees(self):
        """get_target_degrees should convert normalized to degrees."""
        status = ActuatorStatus(target_angle=-0.5)
        
        assert status.get_target_degrees() == -15.0


class TestComputeChecksum:
    """Tests for checksum calculation."""
    
    def test_checksum_empty_string(self):
        """Checksum of empty string should be 0."""
        interface = ActuatorInterface()
        
        assert interface._compute_checksum("") == 0
    
    def test_checksum_single_char(self):
        """Checksum of single char should be its ASCII value."""
        interface = ActuatorInterface()
        
        assert interface._compute_checksum("A") == ord("A")
    
    def test_checksum_xor_operation(self):
        """Checksum should XOR all characters."""
        interface = ActuatorInterface()
        
        # XOR of 'A' and 'B' = 65 ^ 66 = 3
        assert interface._compute_checksum("AB") == 65 ^ 66
    
    def test_checksum_known_message(self):
        """Verify checksum for a known message."""
        interface = ActuatorInterface()
        
        payload = "STS,0.500,0.480,1,12.6,0.5,0"
        checksum = interface._compute_checksum(payload)
        
        # Checksum should be consistent
        assert checksum == interface._compute_checksum(payload)


class TestParseStatus:
    """Tests for status message parsing."""
    
    def test_parse_valid_status(self):
        """Valid status message should update internal state."""
        interface = ActuatorInterface()
        
        # Compute checksum for message
        payload = "STS,0.500,0.480,1,12.6,0.5,0"
        checksum = interface._compute_checksum(payload)
        message = f"${payload}*{checksum:02X}"
        
        interface._parse_status(message)
        
        with interface._lock:
            assert interface._status.target_angle == pytest.approx(0.5)
            assert interface._status.actual_angle == pytest.approx(0.48)
            assert interface._status.clutch_engaged is True
            assert interface._status.voltage == pytest.approx(12.6)
            assert interface._status.current == pytest.approx(0.5)
            assert interface._status.fault_code == MCUFaultCode.NONE
            assert interface._status.valid is True
    
    def test_parse_status_with_fault(self):
        """Status with fault code should be parsed correctly."""
        interface = ActuatorInterface()
        
        payload = "STS,0.000,0.000,0,12.4,15.0,1"  # Fault code 1 = OVERCURRENT
        checksum = interface._compute_checksum(payload)
        message = f"${payload}*{checksum:02X}"
        
        interface._parse_status(message)
        
        with interface._lock:
            assert interface._status.fault_code == MCUFaultCode.OVERCURRENT
    
    def test_parse_invalid_checksum(self):
        """Invalid checksum should be rejected."""
        interface = ActuatorInterface()
        
        payload = "STS,0.500,0.480,1,12.6,0.5,0"
        message = f"${payload}*FF"  # Wrong checksum
        
        initial_count = interface._parse_errors
        interface._parse_status(message)
        
        assert interface._parse_errors == initial_count + 1
    
    def test_parse_missing_checksum(self):
        """Message without checksum should be rejected."""
        interface = ActuatorInterface()
        
        message = "$STS,0.500,0.480,1,12.6,0.5,0"  # No *XX
        
        initial_count = interface._parse_errors
        interface._parse_status(message)
        
        assert interface._parse_errors == initial_count + 1
    
    def test_parse_wrong_message_type(self):
        """Non-STS message should be ignored."""
        interface = ActuatorInterface()
        
        payload = "RUD,0.500,1"
        checksum = interface._compute_checksum(payload)
        message = f"${payload}*{checksum:02X}"
        
        initial_count = interface._parse_errors
        interface._parse_status(message)
        
        # Wrong message type
        assert interface._parse_errors == initial_count + 1
    
    def test_parse_incomplete_message(self):
        """Incomplete message should be rejected."""
        interface = ActuatorInterface()
        
        payload = "STS,0.500,0.480"  # Missing fields
        checksum = interface._compute_checksum(payload)
        message = f"${payload}*{checksum:02X}"
        
        initial_count = interface._parse_errors
        interface._parse_status(message)
        
        assert interface._parse_errors == initial_count + 1


class TestGetStatus:
    """Tests for get_status method."""
    
    def test_get_status_returns_copy(self):
        """get_status should return a copy of status."""
        interface = ActuatorInterface()
        
        status1 = interface.get_status()
        status1.target_angle = 999.0
        
        status2 = interface.get_status()
        
        assert status2.target_angle != 999.0
    
    def test_get_status_marks_stale(self):
        """get_status should mark stale status as invalid."""
        interface = ActuatorInterface()
        
        # Set old timestamp
        with interface._lock:
            interface._status.timestamp = time.time() - 1.0
            interface._status.valid = True
        
        status = interface.get_status()
        
        # Should be marked invalid due to age
        assert status.valid is False


class TestActuatorInterfaceStats:
    """Tests for interface statistics."""
    
    def test_stats_structure(self):
        """Stats should contain expected fields."""
        interface = ActuatorInterface()
        stats = interface.stats
        
        assert "commands_sent" in stats
        assert "status_received" in stats
        assert "parse_errors" in stats
    
    def test_initial_stats(self):
        """Initial stats should be zero."""
        interface = ActuatorInterface()
        stats = interface.stats
        
        assert stats["commands_sent"] == 0
        assert stats["status_received"] == 0
        assert stats["parse_errors"] == 0


class TestMockActuatorInterface:
    """Tests for MockActuatorInterface class."""
    
    def test_mock_start_success(self):
        """Mock interface should start successfully."""
        mock = MockActuatorInterface()
        
        result = mock.start()
        
        assert result is True
        assert mock._running is True
        
        mock.stop()
    
    def test_mock_send_command(self):
        """Mock should accept commands."""
        mock = MockActuatorInterface()
        mock.start()
        
        mock.send_command(0.5, engage=True)
        
        assert mock._mock_target == pytest.approx(0.5)
        assert mock._mock_clutch is True
        assert mock._commands_sent == 1
        
        mock.stop()
    
    def test_mock_command_clamping(self):
        """Mock should clamp target to [-1, 1]."""
        mock = MockActuatorInterface()
        mock.start()
        
        mock.send_command(1.5, engage=True)
        assert mock._mock_target == 1.0
        
        mock.send_command(-1.5, engage=True)
        assert mock._mock_target == -1.0
        
        mock.stop()
    
    def test_mock_simulates_movement(self):
        """Mock should simulate rudder movement over time."""
        mock = MockActuatorInterface()
        mock.start()
        
        mock.send_command(0.5, engage=True)
        
        # Wait for simulated movement
        time.sleep(0.2)
        
        status = mock.get_status()
        
        # Position should be moving toward target
        assert status.actual_angle > 0
        assert status.valid is True
        
        mock.stop()
    
    def test_mock_disengage_stops_movement(self):
        """Mock should stop movement when disengaged."""
        mock = MockActuatorInterface()
        mock.start()
        
        mock.send_command(0.5, engage=True)
        time.sleep(0.1)
        
        mock.send_command(0.5, engage=False)  # Disengage
        
        position1 = mock._mock_position
        time.sleep(0.1)
        position2 = mock._mock_position
        
        # Position should not have moved
        assert position1 == pytest.approx(position2)
        
        mock.stop()
    
    def test_mock_status_updates(self):
        """Mock should update status at regular intervals."""
        mock = MockActuatorInterface()
        mock.start()
        
        time.sleep(0.1)
        
        stats = mock.stats
        
        # Should have received some status updates
        assert stats["status_received"] > 0
        
        mock.stop()
    
    def test_mock_status_voltage_current(self):
        """Mock should report realistic voltage and current."""
        mock = MockActuatorInterface()
        mock.start()
        
        mock.send_command(0.5, engage=True)
        time.sleep(0.1)
        
        status = mock.get_status()
        
        assert status.voltage == pytest.approx(12.6)
        # Current should be non-zero when moving
        
        mock.stop()
    
    def test_mock_no_fault(self):
        """Mock should report no fault in normal operation."""
        mock = MockActuatorInterface()
        mock.start()
        
        mock.send_command(0.5, engage=True)
        time.sleep(0.1)
        
        status = mock.get_status()
        
        assert status.fault_code == MCUFaultCode.NONE
        
        mock.stop()


class TestCallbacks:
    """Tests for status and fault callbacks."""
    
    def test_status_callback_set(self):
        """Status callback should be settable."""
        interface = ActuatorInterface()
        callback = Mock()
        
        interface.set_status_callback(callback)
        
        assert interface._status_callback == callback
    
    def test_fault_callback_set(self):
        """Fault callback should be settable."""
        interface = ActuatorInterface()
        callback = Mock()
        
        interface.set_fault_callback(callback)
        
        assert interface._fault_callback == callback
