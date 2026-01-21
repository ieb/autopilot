"""
Unit tests for Feature Engineering module.

Tests normalization, angle calculations, heading errors,
wave period estimation, and output shapes.
"""

import time
import pytest
import numpy as np

from src.ml.feature_engineering import (
    FeatureEngineering, FeatureConfig, FeatureFrame, TargetState
)
from src.sensors.imu_fusion import IMUData
from src.sensors.nmea2000_interface import N2KData
from src.sensors.adc_reader import RudderData


class TestNormalization:
    """Tests for the _normalize function."""
    
    def test_normalize_within_range(self, feature_engineering):
        """Values within range should normalize proportionally."""
        assert feature_engineering._normalize(15, 30) == pytest.approx(0.5)
        assert feature_engineering._normalize(-15, 30) == pytest.approx(-0.5)
        assert feature_engineering._normalize(0, 30) == pytest.approx(0.0)
    
    def test_normalize_at_limits(self, feature_engineering):
        """Values at limits should normalize to ±1."""
        assert feature_engineering._normalize(30, 30) == pytest.approx(1.0)
        assert feature_engineering._normalize(-30, 30) == pytest.approx(-1.0)
    
    def test_normalize_clipping_high(self, feature_engineering):
        """Values above max should clip to 1.0."""
        assert feature_engineering._normalize(50, 30) == 1.0
        assert feature_engineering._normalize(100, 30) == 1.0
    
    def test_normalize_clipping_low(self, feature_engineering):
        """Values below -max should clip to -1.0."""
        assert feature_engineering._normalize(-50, 30) == -1.0
        assert feature_engineering._normalize(-100, 30) == -1.0


class TestAngleDiff:
    """Tests for the _angle_diff function (signed angle difference)."""
    
    def test_angle_diff_simple(self, feature_engineering):
        """Simple angle differences without wrap-around."""
        assert feature_engineering._angle_diff(90, 45) == pytest.approx(45)
        assert feature_engineering._angle_diff(45, 90) == pytest.approx(-45)
        assert feature_engineering._angle_diff(180, 90) == pytest.approx(90)
    
    def test_angle_diff_wraparound_positive(self, feature_engineering):
        """Wrap-around when crossing 0/360 boundary."""
        # 350 to 10 should be -20 (shortest path is counterclockwise)
        assert feature_engineering._angle_diff(350, 10) == pytest.approx(-20)
        # 10 to 350 should be +20 (shortest path is clockwise)
        assert feature_engineering._angle_diff(10, 350) == pytest.approx(20)
    
    def test_angle_diff_half_circle(self, feature_engineering):
        """180 degree difference should normalize to -180."""
        # Both are equidistant, implementation chooses -180
        assert feature_engineering._angle_diff(90, 270) == pytest.approx(-180)
        assert feature_engineering._angle_diff(0, 180) == pytest.approx(-180)
    
    def test_angle_diff_zero(self, feature_engineering):
        """Same angles should produce zero difference."""
        assert feature_engineering._angle_diff(45, 45) == pytest.approx(0)
        assert feature_engineering._angle_diff(0, 0) == pytest.approx(0)
        assert feature_engineering._angle_diff(360, 0) == pytest.approx(0)


class TestHeadingError:
    """Tests for heading error calculation in different modes."""
    
    def test_heading_error_compass_mode(self, sample_imu_data, sample_n2k_data):
        """Compass mode uses IMU heading vs target heading."""
        fe = FeatureEngineering()
        fe.set_target("compass", 200.0)
        
        # heading=180, target=200 → error = -20
        error = fe._compute_heading_error(sample_imu_data, sample_n2k_data)
        assert error == pytest.approx(-20.0)
    
    def test_heading_error_wind_awa_mode(self, sample_imu_data, sample_n2k_data):
        """Wind AWA mode uses current AWA vs target AWA."""
        fe = FeatureEngineering()
        fe.set_target("wind_awa", 50.0)
        
        # awa=45, target=50 → error = -5
        error = fe._compute_heading_error(sample_imu_data, sample_n2k_data)
        assert error == pytest.approx(-5.0)
    
    def test_heading_error_standby_mode(self, sample_imu_data, sample_n2k_data):
        """Standby mode returns zero error."""
        fe = FeatureEngineering()
        fe.set_target("standby")
        
        error = fe._compute_heading_error(sample_imu_data, sample_n2k_data)
        assert error == pytest.approx(0.0)


class TestSetTarget:
    """Tests for set_target mode switching."""
    
    def test_set_target_compass_mode(self):
        """Setting compass mode stores target heading."""
        fe = FeatureEngineering()
        fe.set_target("compass", 270.0)
        
        assert fe.target.mode == "compass"
        assert fe.target.target_heading == 270.0
    
    def test_set_target_wind_awa_mode(self):
        """Setting wind_awa mode stores target AWA."""
        fe = FeatureEngineering()
        fe.set_target("wind_awa", 35.0)
        
        assert fe.target.mode == "wind_awa"
        assert fe.target.target_awa == 35.0
    
    def test_set_target_wind_twa_mode(self):
        """Setting wind_twa mode stores target TWA."""
        fe = FeatureEngineering()
        fe.set_target("wind_twa", 120.0)
        
        assert fe.target.mode == "wind_twa"
        assert fe.target.target_twa == 120.0
    
    def test_set_target_vmg_modes(self):
        """Setting VMG modes stores target TWA."""
        fe = FeatureEngineering()
        
        fe.set_target("vmg_up", 42.0)
        assert fe.target.mode == "vmg_up"
        assert fe.target.target_twa == 42.0
        
        fe.set_target("vmg_down", 145.0)
        assert fe.target.mode == "vmg_down"
        assert fe.target.target_twa == 145.0
    
    def test_set_target_resets_integral(self):
        """Mode change should reset heading error integral."""
        fe = FeatureEngineering()
        fe._heading_error_integral = 50.0  # Simulate accumulated integral
        
        fe.set_target("compass", 180.0)
        assert fe._heading_error_integral == 0.0


class TestWavePeriodEstimation:
    """Tests for wave period estimation from accelerometer data."""
    
    def test_wave_period_default_with_insufficient_data(self):
        """Should return default 5.0s when insufficient data."""
        fe = FeatureEngineering()
        # Buffer has fewer than 50 samples
        for _ in range(30):
            fe._accel_buffer.append(9.8)
        
        period = fe._estimate_wave_period()
        assert period == pytest.approx(5.0)
    
    def test_wave_period_with_sinusoidal_data(self):
        """Should detect period from sinusoidal acceleration data."""
        fe = FeatureEngineering()
        
        # Simulate 100Hz data with 4 second period (0.25 Hz)
        # 100 samples covers 1 second
        for i in range(100):
            t = i / 100.0  # 0 to 1 second
            # 0.25 Hz wave = period 4s
            accel = 9.8 + 0.5 * np.sin(2 * np.pi * 0.25 * t)
            fe._accel_buffer.append(accel)
        
        period = fe._estimate_wave_period()
        # Should be clamped between 2 and 15 seconds
        assert 2.0 <= period <= 15.0
    
    def test_wave_period_clamping(self):
        """Period should be clamped to valid range [2, 15] seconds."""
        fe = FeatureEngineering()
        
        # Fill with constant data (no zero crossings)
        for _ in range(100):
            fe._accel_buffer.append(9.8)
        
        period = fe._estimate_wave_period()
        # Should return default when detection fails
        assert period == pytest.approx(5.0)


class TestOutputShape:
    """Tests for output shapes and buffer management."""
    
    def test_get_sequence_shape(self, feature_engineering):
        """get_sequence should return correct shape."""
        seq = feature_engineering.get_sequence()
        
        assert seq.shape == (20, 25)  # (sequence_length, feature_dim)
    
    def test_update_returns_correct_shape(
        self, feature_engineering, sample_imu_data, sample_n2k_data, sample_rudder_data
    ):
        """update() should return sequence with correct shape."""
        seq = feature_engineering.update(sample_imu_data, sample_n2k_data, sample_rudder_data)
        
        assert seq.shape == (20, 25)
    
    def test_buffer_maintains_sequence_length(
        self, feature_engineering, sample_imu_data, sample_n2k_data, sample_rudder_data
    ):
        """Buffer should maintain fixed sequence length."""
        # Update many times
        for _ in range(50):
            feature_engineering.update(sample_imu_data, sample_n2k_data, sample_rudder_data)
        
        # Buffer should still be sequence_length
        assert len(feature_engineering._buffer) == 20
    
    def test_features_normalized_range(
        self, feature_engineering, sample_imu_data, sample_n2k_data, sample_rudder_data
    ):
        """Features should be in normalized [-1, 1] range (mostly)."""
        feature_engineering.set_target("compass", 180.0)
        seq = feature_engineering.update(sample_imu_data, sample_n2k_data, sample_rudder_data)
        
        # Most features should be normalized to [-1, 1]
        # Exception: polar performance can be > 1 (up to 1.2)
        latest_frame = seq[-1]
        
        # Check a few key features are in valid range
        assert -1.0 <= latest_frame[0] <= 1.0  # heading error
        assert -1.0 <= latest_frame[3] <= 1.0  # roll
        assert -1.0 <= latest_frame[4] <= 1.0  # pitch


class TestIsValid:
    """Tests for validity checking."""
    
    def test_is_valid_after_updates(
        self, feature_engineering, sample_imu_data, sample_n2k_data, sample_rudder_data
    ):
        """Should be valid after receiving enough valid data."""
        # Need at least sequence_length/2 = 10 valid frames
        for _ in range(15):
            feature_engineering.update(sample_imu_data, sample_n2k_data, sample_rudder_data)
        
        assert feature_engineering.is_valid is True
    
    def test_is_valid_initially_false(self, feature_engineering):
        """Should be invalid initially (buffer has no valid frames)."""
        # Initial buffer is filled with invalid frames
        assert feature_engineering.is_valid is False


class TestFeatureIndices:
    """Tests to verify feature indices are correctly assigned."""
    
    def test_mode_flags_mutually_exclusive(
        self, sample_imu_data, sample_n2k_data, sample_rudder_data
    ):
        """Only one mode flag should be set at a time."""
        fe = FeatureEngineering()
        
        # Test compass mode
        fe.set_target("compass", 180.0)
        seq = fe.update(sample_imu_data, sample_n2k_data, sample_rudder_data)
        latest = seq[-1]
        assert latest[fe.FEAT_MODE_COMPASS] == 1.0
        assert latest[fe.FEAT_MODE_WIND_AWA] == 0.0
        assert latest[fe.FEAT_MODE_WIND_TWA] == 0.0
        
        # Test wind_awa mode
        fe.set_target("wind_awa", 45.0)
        seq = fe.update(sample_imu_data, sample_n2k_data, sample_rudder_data)
        latest = seq[-1]
        assert latest[fe.FEAT_MODE_COMPASS] == 0.0
        assert latest[fe.FEAT_MODE_WIND_AWA] == 1.0
        assert latest[fe.FEAT_MODE_WIND_TWA] == 0.0
    
    def test_feature_count(self, feature_engineering):
        """Should have correct number of feature indices defined."""
        # Last index is FEAT_WAVE_PERIOD = 24, so 25 features total
        assert feature_engineering.config.feature_dim == 25
        assert feature_engineering.FEAT_WAVE_PERIOD == 24
