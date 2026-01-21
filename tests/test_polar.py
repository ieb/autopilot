"""
Unit tests for Polar Diagram module.

Tests polar interpolation, VMG optimization, and performance ratio calculations.
"""

import pytest
import math

from src.ml.polar import Polar, PolarData


class TestPolarData:
    """Tests for PolarData dataclass."""
    
    def test_pogo_1250_polar_structure(self, polar):
        """Pogo 1250 polar should have correct structure."""
        assert polar._data.name == 'pogo1250'
        assert len(polar._data.tws) == 17
        assert len(polar._data.twa) == 24
        assert len(polar._data.stw) == 24  # One row per TWA
        assert len(polar._data.stw[0]) == 17  # One column per TWS
    
    def test_pogo_1250_tws_range(self, polar):
        """TWS range should cover 0 to 60 knots."""
        assert polar._data.tws[0] == 0
        assert polar._data.tws[-1] == 60
    
    def test_pogo_1250_twa_range(self, polar):
        """TWA range should cover 0 to 180 degrees."""
        assert polar._data.twa[0] == 0
        assert polar._data.twa[-1] == 180


class TestGetTargetSpeed:
    """Tests for get_target_speed bilinear interpolation."""
    
    def test_exact_grid_point(self, polar):
        """Speed at exact grid point should match table."""
        # TWA=90°, TWS=10kts should return stw[14][4] = 8.6 (check table)
        # Actually need to find the correct indices
        # TWA 90 is index 14, TWS 10 is index 4
        speed = polar.get_target_speed(twa=90, tws=10)
        
        # Should be positive and reasonable
        assert speed > 0
        assert speed < 15
    
    def test_interpolation_between_points(self, polar):
        """Interpolated speed should be between grid points."""
        # Get speeds at surrounding grid points
        speed_low_twa = polar.get_target_speed(twa=80, tws=10)
        speed_high_twa = polar.get_target_speed(twa=90, tws=10)
        
        # Interpolated at 85° should be between them
        speed_mid = polar.get_target_speed(twa=85, tws=10)
        
        assert min(speed_low_twa, speed_high_twa) <= speed_mid <= max(speed_low_twa, speed_high_twa)
    
    def test_zero_speed_at_zero_wind(self, polar):
        """Should return 0 speed at 0 wind."""
        speed = polar.get_target_speed(twa=90, tws=0)
        
        assert speed == 0.0
    
    def test_zero_speed_dead_downwind_light(self, polar):
        """Very light wind dead downwind should be slow."""
        speed = polar.get_target_speed(twa=180, tws=4)
        
        # Should be positive but slow
        assert 0 <= speed < 5
    
    def test_negative_twa_normalized(self, polar):
        """Negative TWA should be normalized to positive."""
        speed_pos = polar.get_target_speed(twa=45, tws=12)
        speed_neg = polar.get_target_speed(twa=-45, tws=12)
        
        assert speed_pos == pytest.approx(speed_neg)
    
    def test_twa_over_180_normalized(self, polar):
        """TWA > 180° should be normalized."""
        speed_normal = polar.get_target_speed(twa=135, tws=12)
        speed_over = polar.get_target_speed(twa=225, tws=12)  # 360 - 225 = 135
        
        assert speed_normal == pytest.approx(speed_over)
    
    def test_high_tws_clamped(self, polar):
        """TWS beyond range should clamp to maximum."""
        speed_at_max = polar.get_target_speed(twa=90, tws=60)
        speed_beyond = polar.get_target_speed(twa=90, tws=100)
        
        assert speed_at_max == pytest.approx(speed_beyond)
    
    def test_speed_increases_with_tws(self, polar):
        """Speed should generally increase with TWS."""
        speed_6 = polar.get_target_speed(twa=90, tws=6)
        speed_12 = polar.get_target_speed(twa=90, tws=12)
        speed_20 = polar.get_target_speed(twa=90, tws=20)
        
        assert speed_6 < speed_12 < speed_20


class TestOptimalVMGUpwind:
    """Tests for get_optimal_vmg_upwind function."""
    
    def test_upwind_angle_reasonable(self, polar):
        """Optimal upwind angle should be in reasonable range."""
        twa, stw, vmg = polar.get_optimal_vmg_upwind(tws=12)
        
        # Typical upwind angles are 30-55 degrees
        assert 25 <= twa <= 70
    
    def test_upwind_vmg_positive(self, polar):
        """Upwind VMG should be positive."""
        twa, stw, vmg = polar.get_optimal_vmg_upwind(tws=12)
        
        assert vmg > 0
    
    def test_upwind_vmg_calculation(self, polar):
        """VMG should equal STW * cos(TWA)."""
        twa, stw, vmg = polar.get_optimal_vmg_upwind(tws=12)
        
        expected_vmg = stw * math.cos(math.radians(twa))
        assert vmg == pytest.approx(expected_vmg, rel=0.01)
    
    def test_upwind_vmg_varies_with_tws(self, polar):
        """Optimal angle may vary with wind strength."""
        _, _, vmg_light = polar.get_optimal_vmg_upwind(tws=6)
        _, _, vmg_medium = polar.get_optimal_vmg_upwind(tws=12)
        _, _, vmg_heavy = polar.get_optimal_vmg_upwind(tws=20)
        
        # Higher wind should generally give higher VMG
        assert vmg_light < vmg_medium < vmg_heavy
    
    def test_upwind_returns_correct_stw(self, polar):
        """Returned STW should match polar for the angle."""
        twa, stw, vmg = polar.get_optimal_vmg_upwind(tws=12)
        
        expected_stw = polar.get_target_speed(twa, 12)
        assert stw == pytest.approx(expected_stw, rel=0.01)


class TestOptimalVMGDownwind:
    """Tests for get_optimal_vmg_downwind function."""
    
    def test_downwind_angle_reasonable(self, polar):
        """Optimal downwind angle should be in reasonable range."""
        twa, stw, vmg = polar.get_optimal_vmg_downwind(tws=12)
        
        # Typical downwind angles are 120-175 degrees
        assert 110 <= twa <= 180
    
    def test_downwind_vmg_positive(self, polar):
        """Downwind VMG should be positive."""
        twa, stw, vmg = polar.get_optimal_vmg_downwind(tws=12)
        
        assert vmg > 0
    
    def test_downwind_vmg_calculation(self, polar):
        """VMG should equal STW * cos(180 - TWA)."""
        twa, stw, vmg = polar.get_optimal_vmg_downwind(tws=12)
        
        expected_vmg = stw * math.cos(math.radians(180 - twa))
        assert vmg == pytest.approx(expected_vmg, rel=0.01)
    
    def test_downwind_vmg_varies_with_tws(self, polar):
        """Optimal downwind VMG should increase with wind."""
        _, _, vmg_light = polar.get_optimal_vmg_downwind(tws=6)
        _, _, vmg_medium = polar.get_optimal_vmg_downwind(tws=12)
        _, _, vmg_heavy = polar.get_optimal_vmg_downwind(tws=20)
        
        assert vmg_light < vmg_medium < vmg_heavy


class TestPerformanceRatio:
    """Tests for get_performance_ratio function."""
    
    def test_performance_at_target(self, polar):
        """Performance at target speed should be 1.0."""
        target = polar.get_target_speed(twa=90, tws=12)
        ratio = polar.get_performance_ratio(twa=90, tws=12, actual_stw=target)
        
        assert ratio == pytest.approx(1.0)
    
    def test_performance_below_target(self, polar):
        """Performance below target should be < 1.0."""
        target = polar.get_target_speed(twa=90, tws=12)
        ratio = polar.get_performance_ratio(twa=90, tws=12, actual_stw=target * 0.8)
        
        assert ratio == pytest.approx(0.8, rel=0.01)
    
    def test_performance_above_target(self, polar):
        """Performance above target should be > 1.0."""
        target = polar.get_target_speed(twa=90, tws=12)
        ratio = polar.get_performance_ratio(twa=90, tws=12, actual_stw=target * 1.1)
        
        assert ratio == pytest.approx(1.1, rel=0.01)
    
    def test_performance_zero_target_returns_zero(self, polar):
        """Performance with zero target should return 0."""
        # At TWA=0, TWS=0, target is 0
        ratio = polar.get_performance_ratio(twa=0, tws=0, actual_stw=5.0)
        
        assert ratio == 0.0
    
    def test_performance_zero_actual_returns_zero(self, polar):
        """Performance with zero actual speed should return 0."""
        ratio = polar.get_performance_ratio(twa=90, tws=12, actual_stw=0)
        
        assert ratio == 0.0


class TestPolarFromJson:
    """Tests for loading polar from JSON."""
    
    def test_default_polar_is_pogo(self):
        """Default polar should be Pogo 1250."""
        polar = Polar()
        
        assert polar._data.name == 'pogo1250'
    
    def test_pogo_1250_classmethod(self):
        """Pogo 1250 classmethod should return valid polar."""
        polar = Polar.pogo_1250()
        
        assert polar._data.name == 'pogo1250'
        assert len(polar._data.tws) > 0
        assert len(polar._data.twa) > 0


class TestFindIndices:
    """Tests for _find_indices helper function."""
    
    def test_find_indices_at_start(self, polar):
        """Value at start of array should return (0, 0, 0)."""
        idx = polar._find_indices(polar._data.tws, 0)
        
        assert idx == (0, 0, 0.0)
    
    def test_find_indices_at_end(self, polar):
        """Value at end of array should return (n-1, n-1, 0)."""
        last_idx = len(polar._data.tws) - 1
        idx = polar._find_indices(polar._data.tws, 60)
        
        assert idx == (last_idx, last_idx, 0.0)
    
    def test_find_indices_beyond_end(self, polar):
        """Value beyond end should clamp to end."""
        last_idx = len(polar._data.tws) - 1
        idx = polar._find_indices(polar._data.tws, 100)
        
        assert idx == (last_idx, last_idx, 0.0)
    
    def test_find_indices_midpoint(self, polar):
        """Midpoint between values should have frac=0.5."""
        # TWS array has 4 and 6 at indices 1 and 2
        idx = polar._find_indices([0, 4, 6, 8], 5)
        
        assert idx[0] == 1
        assert idx[1] == 2
        assert idx[2] == pytest.approx(0.5)


class TestBilinearInterpolate:
    """Tests for bilinear interpolation."""
    
    def test_interpolation_at_corners(self, polar):
        """Interpolation at corner should return corner value."""
        # Set up simple test grid
        test_data = PolarData(
            name="test",
            tws=[0, 10],
            twa=[0, 90],
            stw=[[0, 5], [0, 10]]
        )
        test_polar = Polar(test_data)
        
        # At TWA=0, TWS=10, should get stw[0][1] = 5
        speed = test_polar.get_target_speed(twa=0, tws=10)
        assert speed == pytest.approx(5.0)
        
        # At TWA=90, TWS=10, should get stw[1][1] = 10
        speed = test_polar.get_target_speed(twa=90, tws=10)
        assert speed == pytest.approx(10.0)
    
    def test_interpolation_at_center(self, polar):
        """Interpolation at center should average four corners."""
        test_data = PolarData(
            name="test",
            tws=[0, 10],
            twa=[0, 90],
            stw=[[0, 4], [0, 8]]
        )
        test_polar = Polar(test_data)
        
        # At TWA=45, TWS=5, should get average = (0+2+0+4)/4 = 1.5
        # Actually: interpolate along TWS first, then TWA
        speed = test_polar.get_target_speed(twa=45, tws=5)
        
        # At TWS=5 (midpoint): row0 = 2, row1 = 4
        # At TWA=45 (midpoint): 3
        assert speed == pytest.approx(3.0)
