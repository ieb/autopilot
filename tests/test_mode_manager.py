"""
Unit tests for Mode Manager module.

Tests mode transitions, heading normalization, tack functionality,
and target adjustments.
"""

import time
import pytest
from unittest.mock import Mock, patch

from src.control.mode_manager import ModeManager, ModeState, AutopilotMode
from src.sensors.nmea2000_interface import N2KData
from src.ml.polar import Polar


class TestAutopilotMode:
    """Tests for AutopilotMode enumeration."""
    
    def test_all_modes_defined(self):
        """All expected modes should be defined."""
        assert AutopilotMode.STANDBY is not None
        assert AutopilotMode.COMPASS is not None
        assert AutopilotMode.WIND_AWA is not None
        assert AutopilotMode.WIND_TWA is not None
        assert AutopilotMode.VMG_UP is not None
        assert AutopilotMode.VMG_DOWN is not None
        assert AutopilotMode.TRACK is not None


class TestModeState:
    """Tests for ModeState dataclass."""
    
    def test_default_state(self):
        """Default state should be STANDBY."""
        state = ModeState()
        
        assert state.mode == AutopilotMode.STANDBY
        assert state.target_value == 0.0
        assert state.mode_name == "STANDBY"


class TestModeManagerInit:
    """Tests for ModeManager initialization."""
    
    def test_default_initialization(self):
        """ModeManager should initialize with default polar."""
        mm = ModeManager()
        
        assert mm.polar is not None
        assert mm._state.mode == AutopilotMode.STANDBY
    
    def test_custom_polar(self, polar):
        """ModeManager should accept custom polar."""
        mm = ModeManager(polar=polar)
        
        assert mm.polar == polar


class TestSetMode:
    """Tests for set_mode function."""
    
    def test_set_compass_mode(self):
        """Setting compass mode should store target heading."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.COMPASS, target=180.0)
        
        state = mm.get_state()
        assert state.mode == AutopilotMode.COMPASS
        assert state.target_value == 180.0
        assert state.mode_name == "COMPASS"
        assert "180" in state.mode_description
    
    def test_set_wind_awa_mode(self):
        """Setting wind AWA mode should store target angle."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.WIND_AWA, target=45.0)
        
        state = mm.get_state()
        assert state.mode == AutopilotMode.WIND_AWA
        assert state.target_value == 45.0
        assert "AWA" in state.mode_name
    
    def test_set_wind_awa_port_tack(self):
        """Port tack wind AWA should show 'P' in mode name."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.WIND_AWA, target=-45.0)
        
        state = mm.get_state()
        assert "P" in state.mode_name  # Port
    
    def test_set_wind_awa_starboard_tack(self):
        """Starboard tack wind AWA should show 'S' in mode name."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.WIND_AWA, target=45.0)
        
        state = mm.get_state()
        assert "S" in state.mode_name  # Starboard
    
    def test_set_vmg_up_mode(self):
        """Setting VMG up mode should store tack side."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.VMG_UP, target=42.0, tack_side=1)
        
        state = mm.get_state()
        assert state.mode == AutopilotMode.VMG_UP
        assert state.tack_side == 1
        assert "VMG UP" in state.mode_name
    
    def test_set_vmg_down_mode(self):
        """Setting VMG down mode should store tack side."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.VMG_DOWN, target=145.0, tack_side=-1)
        
        state = mm.get_state()
        assert state.mode == AutopilotMode.VMG_DOWN
        assert state.tack_side == -1
        assert "VMG DN" in state.mode_name
    
    def test_set_standby_mode(self):
        """Setting standby mode should disable steering."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.COMPASS, target=180.0)
        mm.set_mode(AutopilotMode.STANDBY)
        
        state = mm.get_state()
        assert state.mode == AutopilotMode.STANDBY
        assert state.mode_name == "STANDBY"
    
    def test_mode_start_time_recorded(self):
        """Mode change should record start time."""
        mm = ModeManager()
        before = time.time()
        mm.set_mode(AutopilotMode.COMPASS, target=180.0)
        after = time.time()
        
        state = mm.get_state()
        assert before <= state.mode_start_time <= after


class TestUpdate:
    """Tests for update function that calculates target heading."""
    
    def test_standby_returns_current_heading(self, sample_n2k_data):
        """Standby mode should return current heading."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.STANDBY)
        
        heading = 180.0
        target = mm.update(heading, sample_n2k_data)
        
        assert target == heading
    
    def test_compass_returns_target_heading(self, sample_n2k_data):
        """Compass mode should return target heading."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.COMPASS, target=270.0)
        
        heading = 180.0
        target = mm.update(heading, sample_n2k_data)
        
        assert target == 270.0
    
    def test_wind_awa_computes_heading(self, sample_n2k_data):
        """Wind AWA mode should compute heading to achieve target AWA."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.WIND_AWA, target=50.0)
        
        # Current AWA is 45°, target is 50°
        # Need to bear away (increase heading) by 5°
        heading = 180.0
        target = mm.update(heading, sample_n2k_data)
        
        # Target should be heading + (target_awa - current_awa) = 180 + 5 = 185
        assert target == pytest.approx(185.0, rel=0.1)
    
    def test_update_tracks_duration(self, sample_n2k_data):
        """Update should track mode duration."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.COMPASS, target=180.0)
        
        time.sleep(0.1)
        mm.update(180.0, sample_n2k_data)
        
        state = mm.get_state()
        assert state.mode_duration_s >= 0.1


class TestNormalizeHeading:
    """Tests for _normalize_heading function."""
    
    def test_normalize_positive(self):
        """Positive heading within range should be unchanged."""
        mm = ModeManager()
        
        assert mm._normalize_heading(180.0) == 180.0
        assert mm._normalize_heading(0.0) == 0.0
        assert mm._normalize_heading(359.0) == 359.0
    
    def test_normalize_negative(self):
        """Negative heading should be normalized to 0-360."""
        mm = ModeManager()
        
        assert mm._normalize_heading(-10.0) == 350.0
        assert mm._normalize_heading(-90.0) == 270.0
        assert mm._normalize_heading(-180.0) == 180.0
    
    def test_normalize_over_360(self):
        """Heading over 360 should wrap around."""
        mm = ModeManager()
        
        assert mm._normalize_heading(360.0) == 0.0
        assert mm._normalize_heading(370.0) == 10.0
        assert mm._normalize_heading(720.0) == 0.0
    
    def test_normalize_large_negative(self):
        """Large negative heading should normalize correctly."""
        mm = ModeManager()
        
        assert mm._normalize_heading(-370.0) == 350.0


class TestTack:
    """Tests for tack function."""
    
    def test_tack_flips_tack_side(self):
        """Tack should flip tack side."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.VMG_UP, target=42.0, tack_side=1)
        
        mm.tack()
        
        state = mm.get_state()
        assert state.tack_side == -1
    
    def test_tack_flips_target_value(self):
        """Tack should flip target value sign."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.WIND_AWA, target=45.0)
        
        mm.tack()
        
        state = mm.get_state()
        assert state.target_value == -45.0
    
    def test_tack_updates_mode_info(self):
        """Tack should update mode description."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.WIND_AWA, target=45.0, tack_side=1)
        
        mm.tack()
        
        state = mm.get_state()
        # Should now show port indicator
        assert "P" in state.mode_name
    
    def test_tack_ignored_in_compass_mode(self):
        """Tack should be ignored in compass mode."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.COMPASS, target=180.0)
        
        mm.tack()
        
        state = mm.get_state()
        # Target should be unchanged
        assert state.target_value == 180.0
    
    def test_tack_ignored_in_standby(self):
        """Tack should be ignored in standby mode."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.STANDBY)
        
        mm.tack()
        
        state = mm.get_state()
        assert state.mode == AutopilotMode.STANDBY


class TestAdjustTarget:
    """Tests for adjust_target function."""
    
    def test_adjust_compass_heading(self):
        """Adjusting compass target should change heading."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.COMPASS, target=180.0)
        
        mm.adjust_target(10.0)
        
        state = mm.get_state()
        assert state.target_value == 190.0
    
    def test_adjust_compass_wraps(self):
        """Adjusting compass target should wrap at 360."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.COMPASS, target=355.0)
        
        mm.adjust_target(10.0)
        
        state = mm.get_state()
        assert state.target_value == 5.0
    
    def test_adjust_compass_negative(self):
        """Adjusting compass target negative should decrease heading."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.COMPASS, target=180.0)
        
        mm.adjust_target(-20.0)
        
        state = mm.get_state()
        assert state.target_value == 160.0
    
    def test_adjust_wind_angle(self):
        """Adjusting wind target should change angle."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.WIND_AWA, target=45.0)
        
        mm.adjust_target(5.0)
        
        state = mm.get_state()
        assert state.target_value == 50.0
    
    def test_adjust_wind_angle_clamped(self):
        """Wind angle adjustment should be clamped to ±180."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.WIND_AWA, target=175.0)
        
        mm.adjust_target(20.0)
        
        state = mm.get_state()
        assert state.target_value == 180.0  # Clamped
    
    def test_adjust_ignored_in_standby(self):
        """Adjust should be ignored in standby mode."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.STANDBY)
        
        mm.adjust_target(10.0)
        
        state = mm.get_state()
        assert state.target_value == 0.0  # Unchanged


class TestCallbacks:
    """Tests for mode change callbacks."""
    
    def test_callback_called_on_mode_change(self):
        """Callback should be called when mode changes."""
        mm = ModeManager()
        callback = Mock()
        mm.add_callback(callback)
        
        mm.set_mode(AutopilotMode.COMPASS, target=180.0)
        
        callback.assert_called_once()
        # Callback receives ModeState
        call_arg = callback.call_args[0][0]
        assert call_arg.mode == AutopilotMode.COMPASS
    
    def test_multiple_callbacks(self):
        """Multiple callbacks should all be called."""
        mm = ModeManager()
        callback1 = Mock()
        callback2 = Mock()
        mm.add_callback(callback1)
        mm.add_callback(callback2)
        
        mm.set_mode(AutopilotMode.COMPASS, target=180.0)
        
        callback1.assert_called_once()
        callback2.assert_called_once()
    
    def test_callback_exception_handled(self):
        """Exception in callback should not break mode change."""
        mm = ModeManager()
        bad_callback = Mock(side_effect=Exception("Test error"))
        good_callback = Mock()
        mm.add_callback(bad_callback)
        mm.add_callback(good_callback)
        
        # Should not raise
        mm.set_mode(AutopilotMode.COMPASS, target=180.0)
        
        # Good callback should still be called
        good_callback.assert_called_once()


class TestIsActive:
    """Tests for is_active property."""
    
    def test_not_active_in_standby(self):
        """Standby mode should not be active."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.STANDBY)
        
        assert mm.is_active is False
    
    def test_active_in_compass(self):
        """Compass mode should be active."""
        mm = ModeManager()
        mm.set_mode(AutopilotMode.COMPASS, target=180.0)
        
        assert mm.is_active is True
    
    def test_active_in_wind_modes(self):
        """Wind modes should be active."""
        mm = ModeManager()
        
        mm.set_mode(AutopilotMode.WIND_AWA, target=45.0)
        assert mm.is_active is True
        
        mm.set_mode(AutopilotMode.WIND_TWA, target=120.0)
        assert mm.is_active is True
    
    def test_active_in_vmg_modes(self):
        """VMG modes should be active."""
        mm = ModeManager()
        
        mm.set_mode(AutopilotMode.VMG_UP, target=42.0)
        assert mm.is_active is True
        
        mm.set_mode(AutopilotMode.VMG_DOWN, target=145.0)
        assert mm.is_active is True
