"""
Unit tests for NMEA2000 Interface module.

Tests PGN decoding for wind, speed, COG/SOG, heading, and rudder data,
as well as true wind calculation.
"""

import time
import struct
import math
import pytest
from unittest.mock import Mock, patch

from src.sensors.nmea2000_interface import (
    NMEA2000Interface, N2KData, N2KConfig, PGN, calculate_true_wind
)


class TestN2KData:
    """Tests for N2KData dataclass."""
    
    def test_default_values(self):
        """Default values should be zero/empty."""
        data = N2KData()
        
        assert data.awa == 0.0
        assert data.aws == 0.0
        assert data.stw == 0.0
        assert data.sog == 0.0
        assert data.cog == 0.0
    
    def test_get_age_ms_with_valid_timestamp(self):
        """get_age_ms should return correct age."""
        data = N2KData()
        data.awa_timestamp = time.time() - 0.5  # 500ms ago
        
        age = data.get_age_ms(data.awa_timestamp)
        
        assert 400 < age < 600  # Allow some tolerance
    
    def test_get_age_ms_with_zero_timestamp(self):
        """get_age_ms should return infinity for zero timestamp."""
        data = N2KData()
        data.awa_timestamp = 0.0
        
        age = data.get_age_ms(data.awa_timestamp)
        
        assert age == float('inf')


class TestPGNConstants:
    """Tests for PGN enumeration values."""
    
    def test_wind_data_pgn(self):
        """Wind data PGN should be 130306."""
        assert PGN.WIND_DATA == 130306
    
    def test_speed_pgn(self):
        """Speed PGN should be 128259."""
        assert PGN.SPEED == 128259
    
    def test_cog_sog_pgn(self):
        """COG/SOG PGN should be 129026."""
        assert PGN.COG_SOG == 129026
    
    def test_vessel_heading_pgn(self):
        """Vessel heading PGN should be 127250."""
        assert PGN.VESSEL_HEADING == 127250
    
    def test_rudder_pgn(self):
        """Rudder PGN should be 127245."""
        assert PGN.RUDDER == 127245


class TestWindDataDecoding:
    """Tests for PGN 130306 (Wind Data) decoding."""
    
    def test_decode_apparent_wind(self):
        """Decode apparent wind speed and angle."""
        interface = NMEA2000Interface()
        
        # Construct wind data packet:
        # Byte 0: SID = 0
        # Bytes 1-2: Speed = 10 m/s = 1000 (0x03E8 little-endian)
        # Bytes 3-4: Angle = 45° = 0.785398 rad = 7854 (0x1EAE)
        # Byte 5: Reference = 2 (Apparent)
        speed_raw = int(10.0 / 0.01)  # 10 m/s
        angle_raw = int(math.radians(45.0) / 0.0001)  # 45 degrees
        
        data = bytes([0x00]) + struct.pack('<H', speed_raw) + struct.pack('<H', angle_raw) + bytes([0x02])
        
        # Call internal decode method
        interface._decode_pgn(PGN.WIND_DATA, data, 0)
        
        # Check decoded values (speed in knots)
        expected_aws = 10.0 * 1.94384  # m/s to knots
        assert interface._data.aws == pytest.approx(expected_aws, rel=0.01)
        assert interface._data.awa == pytest.approx(45.0, rel=0.1)
    
    def test_ignore_non_apparent_wind(self):
        """Non-apparent wind references should be ignored."""
        interface = NMEA2000Interface()
        
        speed_raw = 1000
        angle_raw = 7854
        
        # Reference = 0 (True wind ground)
        data = bytes([0x00]) + struct.pack('<H', speed_raw) + struct.pack('<H', angle_raw) + bytes([0x00])
        
        interface._decode_pgn(PGN.WIND_DATA, data, 0)
        
        # Should not update (still 0)
        assert interface._data.aws == 0.0
    
    def test_handle_invalid_wind_values(self):
        """Invalid values (0xFFFF) should be handled."""
        interface = NMEA2000Interface()
        
        # Set initial value
        interface._data.aws = 10.0
        
        # Invalid speed (0xFFFF)
        data = bytes([0x00, 0xFF, 0xFF, 0xAE, 0x1E, 0x02])
        
        interface._decode_pgn(PGN.WIND_DATA, data, 0)
        
        # AWS should not be updated (remain at 10.0)
        assert interface._data.aws == 10.0


class TestSpeedDecoding:
    """Tests for PGN 128259 (Speed) decoding."""
    
    def test_decode_speed_through_water(self):
        """Decode speed through water."""
        interface = NMEA2000Interface()
        
        # Byte 0: SID
        # Bytes 1-2: STW = 5 m/s = 500 (0x01F4)
        stw_raw = int(5.0 / 0.01)  # 5 m/s
        
        data = bytes([0x00]) + struct.pack('<H', stw_raw) + bytes([0x00, 0x00, 0x00])
        
        interface._decode_pgn(PGN.SPEED, data, 0)
        
        expected_stw = 5.0 * 1.94384  # m/s to knots
        assert interface._data.stw == pytest.approx(expected_stw, rel=0.01)
    
    def test_handle_invalid_speed(self):
        """Invalid speed value (0xFFFF) should not update."""
        interface = NMEA2000Interface()
        interface._data.stw = 6.0
        
        # Invalid STW
        data = bytes([0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00])
        
        interface._decode_pgn(PGN.SPEED, data, 0)
        
        # Should not update
        assert interface._data.stw == 6.0


class TestCOGSOGDecoding:
    """Tests for PGN 129026 (COG/SOG Rapid Update) decoding."""
    
    def test_decode_cog_sog(self):
        """Decode course and speed over ground."""
        interface = NMEA2000Interface()
        
        # Byte 0: SID
        # Byte 1: COG Reference
        # Bytes 2-3: COG = 180° = 3.14159 rad = 31416 (0x7AB8)
        # Bytes 4-5: SOG = 8 m/s = 800 (0x0320)
        cog_raw = int(math.radians(180.0) / 0.0001)
        sog_raw = int(8.0 / 0.01)
        
        data = bytes([0x00, 0x00]) + struct.pack('<H', cog_raw) + struct.pack('<H', sog_raw)
        
        interface._decode_pgn(PGN.COG_SOG, data, 0)
        
        assert interface._data.cog == pytest.approx(180.0, rel=0.1)
        expected_sog = 8.0 * 1.94384  # m/s to knots
        assert interface._data.sog == pytest.approx(expected_sog, rel=0.01)
    
    def test_handle_invalid_cog_sog(self):
        """Invalid COG/SOG values should not update."""
        interface = NMEA2000Interface()
        interface._data.cog = 90.0
        interface._data.sog = 5.0
        
        # Invalid values
        data = bytes([0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF])
        
        interface._decode_pgn(PGN.COG_SOG, data, 0)
        
        # Should not update
        assert interface._data.cog == 90.0
        assert interface._data.sog == 5.0


class TestHeadingDecoding:
    """Tests for PGN 127250 (Vessel Heading) decoding."""
    
    def test_decode_heading(self):
        """Decode vessel heading."""
        interface = NMEA2000Interface()
        
        # Byte 0: SID
        # Bytes 1-2: Heading = 270° = 4.7124 rad = 47124
        heading_raw = int(math.radians(270.0) / 0.0001)
        
        data = bytes([0x00]) + struct.pack('<H', heading_raw) + bytes([0x00, 0x00, 0x00, 0x00, 0x00])
        
        interface._decode_pgn(PGN.VESSEL_HEADING, data, 0)
        
        assert interface._data.heading_n2k == pytest.approx(270.0, rel=0.1)
    
    def test_handle_invalid_heading(self):
        """Invalid heading value should not update."""
        interface = NMEA2000Interface()
        interface._data.heading_n2k = 180.0
        
        # Invalid heading
        data = bytes([0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00])
        
        interface._decode_pgn(PGN.VESSEL_HEADING, data, 0)
        
        # Should not update
        assert interface._data.heading_n2k == 180.0


class TestRudderDecoding:
    """Tests for PGN 127245 (Rudder) decoding."""
    
    def test_decode_rudder_positive(self):
        """Decode positive (starboard) rudder angle."""
        interface = NMEA2000Interface()
        
        # Bytes 4-5: Position = +15° = 0.2618 rad = 2618 (signed)
        position_raw = int(math.radians(15.0) / 0.0001)
        
        data = bytes([0x00, 0x00, 0x00, 0x00]) + struct.pack('<h', position_raw)
        
        interface._decode_pgn(PGN.RUDDER, data, 0)
        
        assert interface._data.rudder_angle_n2k == pytest.approx(15.0, rel=0.1)
    
    def test_decode_rudder_negative(self):
        """Decode negative (port) rudder angle."""
        interface = NMEA2000Interface()
        
        # Position = -20° = -0.3491 rad = -3491 (signed)
        position_raw = int(math.radians(-20.0) / 0.0001)
        
        data = bytes([0x00, 0x00, 0x00, 0x00]) + struct.pack('<h', position_raw)
        
        interface._decode_pgn(PGN.RUDDER, data, 0)
        
        assert interface._data.rudder_angle_n2k == pytest.approx(-20.0, rel=0.1)
    
    def test_handle_invalid_rudder(self):
        """Invalid rudder value (0x7FFF) should not update."""
        interface = NMEA2000Interface()
        interface._data.rudder_angle_n2k = 10.0
        
        # Invalid position
        data = bytes([0x00, 0x00, 0x00, 0x00, 0xFF, 0x7F])
        
        interface._decode_pgn(PGN.RUDDER, data, 0)
        
        # Should not update
        assert interface._data.rudder_angle_n2k == 10.0


class TestTrueWindCalculation:
    """Tests for calculate_true_wind function."""
    
    def test_true_wind_upwind(self):
        """True wind calculation for upwind sailing."""
        # Apparent wind: 45° at 15 knots, boat speed 6 knots
        twa, tws = calculate_true_wind(awa=45, aws=15, stw=6, heading=0)
        
        # TWA should be wider than AWA (apparent wind shifted forward by boat motion)
        assert twa > 45
        assert twa < 90  # Still upwind
        
        # TWS should be less than AWS (closing on true wind)
        assert tws < 15
    
    def test_true_wind_downwind(self):
        """True wind calculation for downwind sailing."""
        # Apparent wind: 150° at 10 knots, boat speed 8 knots
        twa, tws = calculate_true_wind(awa=150, aws=10, stw=8, heading=0)
        
        # TWA should be narrower than AWA (running away from wind)
        # TWS should be greater than AWS
        assert tws > 10
    
    def test_true_wind_stationary(self):
        """True wind equals apparent when stationary."""
        # No boat speed
        twa, tws = calculate_true_wind(awa=90, aws=20, stw=0, heading=0)
        
        assert twa == pytest.approx(90.0)
        assert tws == pytest.approx(20.0)
    
    def test_true_wind_head_to_wind(self):
        """True wind calculation for head to wind."""
        # Wind dead ahead at 20 knots, boat moving 8 knots
        twa, tws = calculate_true_wind(awa=0, aws=20, stw=8, heading=0)
        
        # TWA should still be ~0 (ahead)
        assert abs(twa) < 5
        
        # TWS should be AWS minus boat speed
        assert tws == pytest.approx(12.0, rel=0.01)
    
    def test_true_wind_negative_awa(self):
        """True wind calculation with negative AWA (port tack)."""
        # Port tack: AWA = -45°
        twa, tws = calculate_true_wind(awa=-45, aws=15, stw=6, heading=0)
        
        # TWA should also be negative (port side)
        assert twa < 0
    
    def test_true_wind_with_running_dead_downwind(self):
        """True wind calculation running dead downwind."""
        # Wind from stern at 5 knots, boat speed 10 knots (faster than wind!)
        twa, tws = calculate_true_wind(awa=180, aws=5, stw=10, heading=0)
        
        # TWS should be AWS + boat speed (wind from behind + boat running away)
        assert tws == pytest.approx(15.0, rel=0.1)


class TestNMEA2000InterfaceGetData:
    """Tests for get_data method thread safety."""
    
    def test_get_data_returns_copy(self):
        """get_data should return a copy of the data."""
        interface = NMEA2000Interface()
        
        data1 = interface.get_data()
        data1.awa = 999.0  # Modify the copy
        
        data2 = interface.get_data()
        
        # Original should not be modified
        assert data2.awa != 999.0


class TestNMEA2000InterfaceStats:
    """Tests for interface statistics."""
    
    def test_stats_structure(self):
        """Stats should contain expected fields."""
        interface = NMEA2000Interface()
        stats = interface.stats
        
        assert "message_count" in stats
        assert "pgn_counts" in stats
        assert "connected" in stats
    
    def test_initial_stats(self):
        """Initial stats should be zero/empty."""
        interface = NMEA2000Interface()
        stats = interface.stats
        
        assert stats["message_count"] == 0
        assert stats["pgn_counts"] == {}
        assert stats["connected"] is False


class TestN2KConfig:
    """Tests for N2KConfig dataclass."""
    
    def test_default_config(self):
        """Default config should have expected values."""
        config = N2KConfig()
        
        assert config.channel == "can0"
        assert config.bitrate == 250000
        assert config.max_age_ms == 5000.0
    
    def test_custom_config(self):
        """Custom config should override defaults."""
        config = N2KConfig(channel="can1", bitrate=500000)
        
        assert config.channel == "can1"
        assert config.bitrate == 500000
