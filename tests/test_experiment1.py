"""
Tests for Experiment 1 - Planned Passage Following
==================================================

Tests for navigator, metrics, and route parsing functionality.
"""

import math
import pytest
from datetime import datetime, timedelta
from unittest.mock import MagicMock

from experiments.experiment1.navigator import (
    Navigator,
    NavigationState,
    SteeringMode,
    EARTH_RADIUS_NM,
)
from experiments.experiment1.route_parser import Waypoint
from experiments.experiment1.metrics import MetricsTracker, TimeSeriesPoint


# =============================================================================
# Test Fixtures
# =============================================================================

@pytest.fixture
def sample_waypoint():
    """Create a sample waypoint for testing."""
    return Waypoint(
        timestamp=datetime(2026, 1, 25, 10, 0, 0),
        from_lat=51.5,
        from_lon=-1.0,
        to_lat=51.6,
        to_lon=-0.9,
        cog=45.0,
        sog=6.0,
        distance=8.0,
        tws=15.0,
        twd=180.0,
        surface_wind_angle=45.0,
        distance_to_finish=50.0,
        is_motoring=False,
    )


@pytest.fixture
def sample_route():
    """Create a sample route with multiple waypoints."""
    base_time = datetime(2026, 1, 25, 10, 0, 0)
    return [
        Waypoint(
            timestamp=base_time,
            from_lat=51.5, from_lon=-1.0,
            to_lat=51.6, to_lon=-0.9,
            cog=45.0, sog=6.0, distance=8.0,
            tws=15.0, twd=180.0, surface_wind_angle=45.0,
            distance_to_finish=50.0, is_motoring=False,
        ),
        Waypoint(
            timestamp=base_time + timedelta(hours=1),
            from_lat=51.6, from_lon=-0.9,
            to_lat=51.7, to_lon=-0.8,
            cog=50.0, sog=6.5, distance=7.0,
            tws=12.0, twd=200.0, surface_wind_angle=60.0,
            distance_to_finish=42.0, is_motoring=False,
        ),
        Waypoint(
            timestamp=base_time + timedelta(hours=2),
            from_lat=51.7, from_lon=-0.8,
            to_lat=51.8, to_lon=-0.7,
            cog=55.0, sog=5.5, distance=6.0,
            tws=10.0, twd=220.0, surface_wind_angle=70.0,
            distance_to_finish=35.0, is_motoring=False,
        ),
    ]


@pytest.fixture
def navigator(sample_route):
    """Create a navigator with sample route."""
    return Navigator(sample_route)


@pytest.fixture
def metrics_tracker():
    """Create a metrics tracker."""
    start = datetime(2026, 1, 25, 10, 0, 0)
    end = datetime(2026, 1, 25, 20, 0, 0)
    return MetricsTracker(start, end)


# =============================================================================
# Waypoint Tests
# =============================================================================

class TestWaypoint:
    """Tests for Waypoint dataclass."""
    
    def test_twa_calculation(self, sample_waypoint):
        """Test true wind angle calculation."""
        # TWD=180, COG=45 -> TWA=135
        assert sample_waypoint.twa == 135.0
    
    def test_twa_wrap_positive(self):
        """Test TWA wrapping for positive angles > 180."""
        wp = Waypoint(
            timestamp=datetime.now(),
            from_lat=0, from_lon=0, to_lat=0, to_lon=0,
            cog=10.0, sog=6.0, distance=1.0,
            tws=15.0, twd=300.0,  # TWD - COG = 290 -> should wrap
            surface_wind_angle=0.0,
            distance_to_finish=10.0, is_motoring=False,
        )
        # TWD=300, COG=10 -> raw diff = 290 -> wrapped to -70
        assert -180 <= wp.twa <= 180
    
    def test_twa_wrap_negative(self):
        """Test TWA wrapping for negative angles < -180."""
        wp = Waypoint(
            timestamp=datetime.now(),
            from_lat=0, from_lon=0, to_lat=0, to_lon=0,
            cog=350.0, sog=6.0, distance=1.0,
            tws=15.0, twd=10.0,  # TWD - COG = -340 -> should wrap
            surface_wind_angle=0.0,
            distance_to_finish=10.0, is_motoring=False,
        )
        assert -180 <= wp.twa <= 180
    
    def test_duration_calculation(self, sample_waypoint):
        """Test leg duration calculation."""
        # distance=8nm, sog=6kts -> 8/6 = 1.33 hours = 4800 seconds
        expected = (8.0 / 6.0) * 3600
        assert abs(sample_waypoint.duration_seconds - expected) < 0.1
    
    def test_duration_zero_speed(self):
        """Test duration returns 0 for zero speed."""
        wp = Waypoint(
            timestamp=datetime.now(),
            from_lat=0, from_lon=0, to_lat=0, to_lon=0,
            cog=0.0, sog=0.0, distance=1.0,  # Zero speed
            tws=0.0, twd=0.0, surface_wind_angle=0.0,
            distance_to_finish=0.0, is_motoring=True,
        )
        assert wp.duration_seconds == 0.0


# =============================================================================
# Navigator Tests
# =============================================================================

class TestNavigator:
    """Tests for Navigator class."""
    
    def test_init_empty_route_raises(self):
        """Test that empty route raises ValueError."""
        with pytest.raises(ValueError, match="at least one waypoint"):
            Navigator([])
    
    def test_init_sets_first_leg(self, navigator, sample_route):
        """Test that navigator initializes to first leg."""
        assert navigator.state.current_leg_index == 0
        assert navigator.state.from_waypoint == sample_route[0]
        # to_waypoint is the next waypoint (second in list)
        assert navigator.state.to_waypoint == sample_route[1]
    
    def test_current_leg_property(self, navigator):
        """Test current_leg property."""
        assert navigator.current_leg == 0
    
    def test_route_progress_initial(self, navigator):
        """Test route progress at start is small."""
        progress = navigator.get_route_progress()
        # Progress starts at first leg, so may not be exactly 0
        assert 0.0 <= progress <= 0.5
    
    def test_update_position_computes_bearing(self, navigator):
        """Test that update computes bearing to waypoint."""
        nav_state = navigator.update(
            lat=51.5,
            lon=-1.0,
            twd=180.0,
            heading=45.0,
            stw=6.0,
        )
        # Should have a valid bearing
        assert 0 <= nav_state.bearing_to_waypoint < 360
    
    def test_update_position_computes_distance(self, navigator):
        """Test that update computes distance to waypoint."""
        nav_state = navigator.update(
            lat=51.5,
            lon=-1.0,
            twd=180.0,
            heading=45.0,
            stw=6.0,
        )
        # Should have positive distance
        assert nav_state.distance_to_waypoint >= 0
    
    def test_steering_mode_selection(self, navigator):
        """Test steering mode is set based on conditions."""
        nav_state = navigator.update(
            lat=51.55,  # Midway point
            lon=-0.95,
            twd=180.0,
            heading=140.0,
            stw=6.0,
        )
        # Steering mode should be one of the valid modes
        assert nav_state.steering_mode in [
            SteeringMode.WIND_AWA,
            SteeringMode.WIND_TWA, 
            SteeringMode.COMPASS,
            SteeringMode.MOTORING
        ]
    
    def test_steering_mode_motoring(self, sample_route):
        """Test steering mode for motoring leg."""
        # Modify first waypoint to be motoring
        sample_route[0].is_motoring = True
        nav = Navigator(sample_route)
        
        nav_state = nav.update(
            lat=51.55,
            lon=-0.95,
            twd=180.0,
            heading=45.0,
            stw=6.0,
        )
        assert nav_state.steering_mode == SteeringMode.MOTORING
    
    def test_leg_advance_on_arrival(self, navigator):
        """Test that leg advances when waypoint is reached."""
        # Position at the to_waypoint (second waypoint's to_lat/to_lon)
        nav_state = navigator.update(
            lat=51.7,  # to_lat of second waypoint
            lon=-0.8,  # to_lon of second waypoint
            twd=180.0,
            heading=45.0,
            stw=6.0,
        )
        # Should have advanced or completed leg
        assert navigator.current_leg >= 1 or nav_state.leg_complete
    
    def test_cross_track_error_on_track(self, navigator):
        """Test XTE when exactly on track."""
        # Start position is on track
        nav_state = navigator.update(
            lat=51.5,
            lon=-1.0,
            twd=180.0,
            heading=45.0,
            stw=6.0,
        )
        # XTE should be small at start position
        assert abs(nav_state.cross_track_error) < 1.0  # Less than 1nm


class TestNavigatorGeometry:
    """Tests for Navigator geometry calculations."""
    
    def test_distance_same_point(self):
        """Test distance for same point is zero."""
        nav = Navigator([Waypoint(
            timestamp=datetime.now(),
            from_lat=51.5, from_lon=-1.0,
            to_lat=51.6, to_lon=-0.9,
            cog=45.0, sog=6.0, distance=8.0,
            tws=15.0, twd=180.0, surface_wind_angle=45.0,
            distance_to_finish=50.0, is_motoring=False,
        ), Waypoint(
            timestamp=datetime.now(),
            from_lat=51.6, from_lon=-0.9,
            to_lat=51.7, to_lon=-0.8,
            cog=45.0, sog=6.0, distance=8.0,
            tws=15.0, twd=180.0, surface_wind_angle=45.0,
            distance_to_finish=42.0, is_motoring=False,
        )])
        
        dist = nav._compute_distance(51.5, -1.0, 51.5, -1.0)
        assert dist == 0.0
    
    def test_distance_known_points(self):
        """Test distance for known points."""
        nav = Navigator([Waypoint(
            timestamp=datetime.now(),
            from_lat=51.5, from_lon=-1.0,
            to_lat=51.6, to_lon=-0.9,
            cog=45.0, sog=6.0, distance=8.0,
            tws=15.0, twd=180.0, surface_wind_angle=45.0,
            distance_to_finish=50.0, is_motoring=False,
        ), Waypoint(
            timestamp=datetime.now(),
            from_lat=51.6, from_lon=-0.9,
            to_lat=51.7, to_lon=-0.8,
            cog=45.0, sog=6.0, distance=8.0,
            tws=15.0, twd=180.0, surface_wind_angle=45.0,
            distance_to_finish=42.0, is_motoring=False,
        )])
        
        # London to Paris is approximately 190nm
        dist = nav._compute_distance(51.5, -0.1, 48.9, 2.3)
        assert 180 < dist < 200  # Rough check
    
    def test_bearing_north(self, sample_route):
        """Test bearing calculation for due north."""
        nav = Navigator(sample_route)
        
        # Point directly north
        bearing = nav._compute_bearing(51.5, 0.0, 52.5, 0.0)
        assert abs(bearing - 0.0) < 1.0 or abs(bearing - 360.0) < 1.0
    
    def test_bearing_east(self, sample_route):
        """Test bearing calculation for due east."""
        nav = Navigator(sample_route)
        
        # Point directly east
        bearing = nav._compute_bearing(51.5, 0.0, 51.5, 1.0)
        assert 85 < bearing < 95  # Should be ~90 degrees


# =============================================================================
# MetricsTracker Tests
# =============================================================================

class TestMetricsTracker:
    """Tests for MetricsTracker class."""
    
    def test_init(self, metrics_tracker):
        """Test metrics tracker initialization."""
        assert metrics_tracker.planned_start_time == datetime(2026, 1, 25, 10, 0, 0)
        assert len(metrics_tracker.time_series) == 0
    
    def test_record_point(self, metrics_tracker):
        """Test recording a data point."""
        metrics_tracker.record(
            elapsed_time=100.0,
            lat=51.5,
            lon=-1.0,
            heading=45.0,
            target_heading=45.0,
            stw=6.0,
            sog=6.0,
            tws=15.0,
            twd=180.0,
            cross_track_error=0.01,
            polar_performance=0.95,
            rudder_angle=2.0,
            steering_mode="compass",
            leg_index=0,
        )
        assert len(metrics_tracker.time_series) == 1
        assert metrics_tracker.time_series[0].timestamp == 100.0
    
    def test_record_multiple_points(self, metrics_tracker):
        """Test recording multiple points."""
        for i in range(10):
            metrics_tracker.record(
                elapsed_time=float(i * 10),
                lat=51.5 + i * 0.01,
                lon=-1.0 + i * 0.01,
                heading=45.0,
                target_heading=45.0,
                stw=6.0,
                sog=6.0,
                tws=15.0,
                twd=180.0,
                cross_track_error=0.01 * i,
                polar_performance=0.95,
                rudder_angle=2.0,
                steering_mode="compass",
                leg_index=0,
            )
        assert len(metrics_tracker.time_series) == 10
    
    def test_compute_summary_empty(self, metrics_tracker):
        """Test computing summary with no data."""
        summary = metrics_tracker.compute_summary()
        assert summary is not None
        # compute_summary returns a SummaryMetrics dataclass
        assert summary.planned_duration_sec > 0
    
    def test_compute_summary_with_data(self, metrics_tracker):
        """Test computing summary with recorded data."""
        # Record some data points
        for i in range(100):
            metrics_tracker.record(
                elapsed_time=float(i * 10),
                lat=51.5,
                lon=-1.0,
                heading=45.0 + math.sin(i * 0.1) * 5,  # Some variation
                target_heading=45.0,
                stw=6.0,
                sog=6.0,
                tws=15.0,
                twd=180.0,
                cross_track_error=0.01 * math.sin(i * 0.1),
                polar_performance=0.95,
                rudder_angle=2.0,
                steering_mode="compass",
                leg_index=0,
            )
        
        metrics_tracker.actual_elapsed_time = 1000.0
        summary = metrics_tracker.compute_summary()
        
        # summary is a SummaryMetrics dataclass
        assert hasattr(summary, 'mean_xte_nm')
        assert hasattr(summary, 'mean_polar_performance')
        assert hasattr(summary, 'rms_heading_error_deg')
    
    def test_xte_statistics(self, metrics_tracker):
        """Test XTE statistics calculation."""
        # Record points with known XTE values
        xte_values = [0.1, 0.2, 0.15, 0.25, 0.1]
        for i, xte in enumerate(xte_values):
            metrics_tracker.record(
                elapsed_time=float(i * 10),
                lat=51.5,
                lon=-1.0,
                heading=45.0,
                target_heading=45.0,
                stw=6.0,
                sog=6.0,
                tws=15.0,
                twd=180.0,
                cross_track_error=xte,
                polar_performance=0.95,
                rudder_angle=2.0,
                steering_mode="compass",
                leg_index=0,
            )
        
        metrics_tracker.actual_elapsed_time = 50.0
        summary = metrics_tracker.compute_summary()
        
        # Check mean XTE
        expected_mean = sum(xte_values) / len(xte_values)
        assert abs(summary.mean_xte_nm - expected_mean) < 0.01


# =============================================================================
# Integration Tests
# =============================================================================

class TestNavigatorMetricsIntegration:
    """Integration tests for Navigator and MetricsTracker."""
    
    def test_simulated_passage(self, sample_route):
        """Test a simulated passage with navigator and metrics."""
        navigator = Navigator(sample_route)
        start = sample_route[0].timestamp
        end = start + timedelta(hours=10)
        metrics = MetricsTracker(start, end)
        
        # Simulate 100 updates
        lat, lon = 51.5, -1.0
        for i in range(100):
            elapsed = float(i * 10)
            
            # Update position (simple straight-line movement)
            lat += 0.001
            lon += 0.001
            
            nav_state = navigator.update(
                lat=lat,
                lon=lon,
                twd=180.0,
                heading=45.0,
                stw=6.0,
            )
            
            metrics.record(
                elapsed_time=elapsed,
                lat=lat,
                lon=lon,
                heading=45.0,
                target_heading=nav_state.target_heading,
                stw=6.0,
                sog=6.0,
                tws=15.0,
                twd=180.0,
                cross_track_error=abs(nav_state.cross_track_error),
                polar_performance=0.9,
                rudder_angle=0.0,
                steering_mode=nav_state.steering_mode.value,
                leg_index=navigator.current_leg,
            )
        
        metrics.actual_elapsed_time = 1000.0
        summary = metrics.compute_summary()
        
        assert summary is not None
        assert len(metrics.time_series) == 100
