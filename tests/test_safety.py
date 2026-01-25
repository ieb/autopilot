"""
Unit tests for Safety Layer module.

Tests safety limit enforcement, sensor timeouts, heading deviations,
manual override detection, and rate limiting.
"""

import time
import pytest
from unittest.mock import Mock, patch

from src.control.safety import (
    SafetyLayer, SafetyConfig, SafetyState, AlarmCode,
    SystemSafety, EmergencyStop
)
from src.control.actuator_interface import ActuatorStatus, MCUFaultCode


class TestSafetyConfig:
    """Tests for SafetyConfig defaults."""
    
    def test_default_rudder_limits(self):
        """Default rudder limits should match specification."""
        config = SafetyConfig()
        
        assert config.rudder_limit_deg == 24.0
        assert config.rudder_warning_deg == 22.0
    
    def test_default_rate_limit(self):
        """Default rate limit should be 5 deg/s."""
        config = SafetyConfig()
        
        assert config.max_rudder_rate == 5.0
    
    def test_default_sensor_timeouts(self):
        """Default sensor timeouts should match specification."""
        config = SafetyConfig()
        
        assert config.imu_max_age_ms == 200.0
        assert config.rudder_max_age_ms == 100.0


class TestRudderLimits:
    """Tests for rudder position limit enforcement."""
    
    def test_output_within_limits_unchanged(self, safety_layer):
        """Output within limits should pass through (after filtering)."""
        output, state = safety_layer.validate(
            ml_output=0.5,
            heading_error=0,
            rudder_position=0,
            imu_age_ms=50,
            rudder_age_ms=20,
            commanded_position=0
        )
        
        # First call might be affected by filter, but should be proportional
        assert state.alarm_code in [AlarmCode.OK, AlarmCode.RATE_LIMIT]
        assert state.is_safe is True
    
    def test_output_clamped_to_limit(self, safety_layer):
        """Output exceeding limit should be clamped."""
        # Request full +30° (normalized = 1.0)
        # After multiple calls to allow filter to settle
        for _ in range(10):
            output, state = safety_layer.validate(
                ml_output=1.0,
                heading_error=0,
                rudder_position=0,
                imu_age_ms=50,
                rudder_age_ms=20,
                commanded_position=0
            )
        
        # Should be clamped to 24/25 = 0.96
        max_normalized = 24.0 / 25.0
        assert abs(output) <= max_normalized + 0.01  # Small tolerance for filter
    
    def test_negative_output_clamped(self, safety_layer):
        """Negative output should also be clamped."""
        for _ in range(10):
            output, state = safety_layer.validate(
                ml_output=-1.0,
                heading_error=0,
                rudder_position=0,
                imu_age_ms=50,
                rudder_age_ms=20,
                commanded_position=0
            )
        
        max_normalized = 24.0 / 25.0
        assert abs(output) <= max_normalized + 0.01


class TestRateLimiting:
    """Tests for rudder rate limit enforcement."""
    
    def test_rate_limit_applied_on_large_change(self, safety_layer):
        """Large sudden changes should be rate limited."""
        # First call with zero output
        safety_layer.validate(
            ml_output=0.0,
            heading_error=0,
            rudder_position=0,
            imu_age_ms=50,
            rudder_age_ms=20,
            commanded_position=0
        )
        
        # Immediate jump to full output should be limited
        output, state = safety_layer.validate(
            ml_output=1.0,
            heading_error=0,
            rudder_position=0,
            imu_age_ms=50,
            rudder_age_ms=20,
            commanded_position=0
        )
        
        # Output should be less than requested due to rate limiting
        assert abs(output) < 1.0
    
    def test_gradual_change_not_limited(self):
        """Gradual changes with realistic timing should allow output to build up."""
        import time as time_module
        
        # Create a fresh safety layer for this test
        sl = SafetyLayer()
        
        # Simulate realistic timing between calls (50Hz = 20ms intervals)
        for _ in range(30):
            output, state = sl.validate(
                ml_output=0.5,  # Constant target
                heading_error=0,
                rudder_position=0,
                imu_age_ms=50,
                rudder_age_ms=20,
                commanded_position=0
            )
            time_module.sleep(0.02)  # 20ms between calls
        
        # After ~600ms with realistic timing, filtered output should approach target
        # Rate limit is 5 deg/s = 0.167 normalized/s, so in 0.6s max change ~0.1
        # Plus filter needs time to settle (alpha=0.3)
        assert output > 0.03  # Should have built up with realistic timing


class TestSensorTimeouts:
    """Tests for sensor timeout detection."""
    
    def test_imu_timeout_triggers_alarm(self, safety_layer):
        """Stale IMU data should trigger timeout alarm."""
        output, state = safety_layer.validate(
            ml_output=0.5,
            heading_error=0,
            rudder_position=0,
            imu_age_ms=250,  # > 200ms threshold
            rudder_age_ms=20,
            commanded_position=0
        )
        
        assert output == 0.0
        assert state.alarm_code == AlarmCode.SENSOR_TIMEOUT
        assert state.is_safe is False
        assert "IMU" in state.alarm_message
    
    def test_rudder_timeout_triggers_alarm(self, safety_layer):
        """Stale rudder data should trigger timeout alarm."""
        output, state = safety_layer.validate(
            ml_output=0.5,
            heading_error=0,
            rudder_position=0,
            imu_age_ms=50,
            rudder_age_ms=150,  # > 100ms threshold
            commanded_position=0
        )
        
        assert output == 0.0
        assert state.alarm_code == AlarmCode.SENSOR_TIMEOUT
        assert state.is_safe is False
        assert "Rudder" in state.alarm_message
    
    def test_valid_sensor_data_no_alarm(self, safety_layer):
        """Fresh sensor data should not trigger timeout alarm."""
        output, state = safety_layer.validate(
            ml_output=0.5,
            heading_error=0,
            rudder_position=0,
            imu_age_ms=50,
            rudder_age_ms=20,
            commanded_position=0
        )
        
        assert state.alarm_code != AlarmCode.SENSOR_TIMEOUT


class TestHeadingDeviation:
    """Tests for heading deviation monitoring."""
    
    def test_large_heading_error_triggers_alarm(self, safety_layer):
        """Heading error > 45° should trigger alarm."""
        output, state = safety_layer.validate(
            ml_output=0.5,
            heading_error=50.0,  # > 45° threshold
            rudder_position=0,
            imu_age_ms=50,
            rudder_age_ms=20,
            commanded_position=0
        )
        
        assert output == 0.0
        assert state.alarm_code == AlarmCode.HEADING_DEVIATION
        assert state.is_safe is False
    
    def test_negative_heading_error_triggers_alarm(self, safety_layer):
        """Negative heading error magnitude > 45° should trigger alarm."""
        output, state = safety_layer.validate(
            ml_output=0.5,
            heading_error=-50.0,  # > 45° magnitude
            rudder_position=0,
            imu_age_ms=50,
            rudder_age_ms=20,
            commanded_position=0
        )
        
        assert output == 0.0
        assert state.alarm_code == AlarmCode.HEADING_DEVIATION
    
    def test_warning_threshold_sets_alarm_but_continues(self, safety_layer):
        """Heading error > 30° but < 45° should warn but continue."""
        output, state = safety_layer.validate(
            ml_output=0.5,
            heading_error=35.0,  # > 30° warning, < 45° limit
            rudder_position=0,
            imu_age_ms=50,
            rudder_age_ms=20,
            commanded_position=0
        )
        
        # Should warn but still produce output
        assert state.alarm_code == AlarmCode.HEADING_DEVIATION
        assert state.is_safe is True  # Still safe, just warning


class TestManualOverride:
    """Tests for manual override detection."""
    
    def test_override_detected_after_timeout(self):
        """Sustained position error should trigger override alarm."""
        config = SafetyConfig(
            override_threshold_deg=5.0,
            override_time_s=0.1  # Short timeout for testing
        )
        sl = SafetyLayer(config=config)
        
        # Simulate sustained difference between actual and commanded
        for _ in range(5):
            output, state = sl.validate(
                ml_output=0.5,
                heading_error=0,
                rudder_position=20.0,  # Actual position
                imu_age_ms=50,
                rudder_age_ms=20,
                commanded_position=0.0  # Commanded position (far from actual)
            )
            time.sleep(0.05)
        
        assert state.alarm_code == AlarmCode.MANUAL_OVERRIDE
        assert state.is_safe is False
    
    def test_no_override_within_threshold(self, safety_layer):
        """Small position error should not trigger override."""
        output, state = safety_layer.validate(
            ml_output=0.5,
            heading_error=0,
            rudder_position=2.0,  # Actual
            imu_age_ms=50,
            rudder_age_ms=20,
            commanded_position=0.0  # < 5° difference
        )
        
        assert state.alarm_code != AlarmCode.MANUAL_OVERRIDE


class TestOutputFiltering:
    """Tests for output low-pass filtering."""
    
    def test_filter_smooths_output(self, safety_layer):
        """Output should be smoothed by low-pass filter."""
        # Start with zero
        output1, _ = safety_layer.validate(
            ml_output=0.0,
            heading_error=0,
            rudder_position=0,
            imu_age_ms=50,
            rudder_age_ms=20,
            commanded_position=0
        )
        
        # Step to 0.5
        output2, _ = safety_layer.validate(
            ml_output=0.5,
            heading_error=0,
            rudder_position=0,
            imu_age_ms=50,
            rudder_age_ms=20,
            commanded_position=0
        )
        
        # Output should be between 0 and 0.5 due to filtering
        assert 0 < output2 < 0.5


class TestSafetyLayerReset:
    """Tests for safety layer reset functionality."""
    
    def test_reset_clears_state(self, safety_layer):
        """Reset should clear accumulated state."""
        # Build up some state
        for _ in range(5):
            safety_layer.validate(
                ml_output=0.5,
                heading_error=0,
                rudder_position=0,
                imu_age_ms=50,
                rudder_age_ms=20,
                commanded_position=0
            )
        
        safety_layer.reset()
        
        assert safety_layer._last_output == 0.0
        assert safety_layer._filtered_output == 0.0
        assert safety_layer._in_override is False
    
    def test_reset_clears_statistics(self, safety_layer):
        """Reset through get_state should return clean state."""
        safety_layer.reset()
        state = safety_layer.get_state()
        
        assert state.alarm_code == AlarmCode.OK
        assert state.is_safe is True


class TestSafetyLayerStats:
    """Tests for safety layer statistics."""
    
    def test_stats_tracking(self, safety_layer):
        """Stats should track limit and override counts."""
        # Initial stats
        stats = safety_layer.stats
        assert "limit_count" in stats
        assert "override_count" in stats


class TestSystemSafety:
    """Tests for SystemSafety class."""
    
    def test_actuator_timeout_disengages(self, stale_actuator_status):
        """Stale actuator status should disengage."""
        mock_actuator = Mock()
        ss = SystemSafety(mock_actuator)
        
        output, engage, alarm = ss.check(
            ml_output=0.5,
            heading_error=10.0,
            imu_age=0.1,
            actuator_status=stale_actuator_status
        )
        
        assert output == 0.0
        assert engage is False
        assert alarm == AlarmCode.ACTUATOR_TIMEOUT
    
    def test_mcu_fault_disengages(self, faulted_actuator_status):
        """MCU fault should disengage."""
        mock_actuator = Mock()
        ss = SystemSafety(mock_actuator)
        
        output, engage, alarm = ss.check(
            ml_output=0.5,
            heading_error=10.0,
            imu_age=0.1,
            actuator_status=faulted_actuator_status
        )
        
        assert output == 0.0
        assert engage is False
        assert alarm == AlarmCode.MCU_FAULT
    
    def test_imu_timeout_disengages(self, valid_actuator_status):
        """Stale IMU data should disengage."""
        mock_actuator = Mock()
        ss = SystemSafety(mock_actuator)
        
        output, engage, alarm = ss.check(
            ml_output=0.5,
            heading_error=10.0,
            imu_age=1.0,  # > 0.5s threshold
            actuator_status=valid_actuator_status
        )
        
        assert output == 0.0
        assert engage is False
        assert alarm == AlarmCode.SENSOR_TIMEOUT
    
    def test_heading_error_timeout_before_alarm(self, valid_actuator_status):
        """Heading error needs sustained time before alarm."""
        mock_actuator = Mock()
        ss = SystemSafety(mock_actuator)
        
        # First call with large heading error
        output, engage, alarm = ss.check(
            ml_output=0.5,
            heading_error=50.0,  # > 45° threshold
            imu_age=0.1,
            actuator_status=valid_actuator_status
        )
        
        # Should not immediately disengage (needs 3s timeout)
        assert engage is True
        # Alarm might be RATE_LIMIT or RUDDER_LIMIT due to rate limiting on first call
        # The important thing is it's NOT HEADING_DEVIATION (which requires timeout)
        assert alarm != AlarmCode.HEADING_DEVIATION
    
    def test_normal_operation_engages(self, valid_actuator_status):
        """Normal conditions should keep engaged."""
        mock_actuator = Mock()
        ss = SystemSafety(mock_actuator)
        
        output, engage, alarm = ss.check(
            ml_output=0.5,
            heading_error=10.0,
            imu_age=0.1,
            actuator_status=valid_actuator_status
        )
        
        assert engage is True
        assert alarm == AlarmCode.OK or alarm == AlarmCode.RATE_LIMIT
    
    def test_rate_limiting_applied(self, valid_actuator_status):
        """SystemSafety should also apply rate limiting."""
        mock_actuator = Mock()
        ss = SystemSafety(mock_actuator)
        
        # First call with zero
        ss.check(
            ml_output=0.0,
            heading_error=0,
            imu_age=0.1,
            actuator_status=valid_actuator_status
        )
        
        # Large jump should be limited
        output, engage, alarm = ss.check(
            ml_output=1.0,
            heading_error=0,
            imu_age=0.1,
            actuator_status=valid_actuator_status
        )
        
        assert abs(output) < 1.0


class TestSystemSafetyReset:
    """Tests for SystemSafety reset."""
    
    def test_reset_clears_state(self, valid_actuator_status):
        """Reset should clear all tracking state."""
        mock_actuator = Mock()
        ss = SystemSafety(mock_actuator)
        
        # Build some state
        ss.check(
            ml_output=0.5,
            heading_error=50.0,
            imu_age=0.1,
            actuator_status=valid_actuator_status
        )
        
        ss.reset()
        
        assert ss._last_output == 0.0
        assert ss._heading_error_start is None
        assert ss._override_start is None
        assert ss.alarm_code == AlarmCode.OK


class TestEmergencyStop:
    """Tests for EmergencyStop class."""
    
    def test_trigger_sets_state(self):
        """Triggering emergency stop should set state."""
        es = EmergencyStop()
        
        es.trigger("Test reason")
        
        assert es.is_triggered is True
        assert es.trigger_info["reason"] == "Test reason"
        assert es.trigger_info["time"] > 0
    
    def test_reset_clears_state(self):
        """Reset should clear emergency stop state."""
        es = EmergencyStop()
        
        es.trigger("Test reason")
        es.reset()
        
        assert es.is_triggered is False
        assert es.trigger_info["reason"] == ""
    
    def test_initially_not_triggered(self):
        """Emergency stop should not be triggered initially."""
        es = EmergencyStop()
        
        assert es.is_triggered is False
