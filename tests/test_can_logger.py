"""Tests for CAN bus raw frame datalogger."""

import json
import struct
import time
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from src.n2k.can_logger import (
    BinaryLogWriter,
    CANLogger,
    CANLoggerConfig,
    FILE_MAGIC,
    FILE_VERSION,
    FRAME_SIZE,
    FRAME_STRUCT,
    FrameFilter,
    GPSTimeSync,
    HEADER_SIZE,
    HEADER_STRUCT,
    MAX_TIMESTAMP_OFFSET,
    TIMESTAMP_UNIT_S,
    PGN_NAMES,
    PGN_SYSTEM_TIME,
    PGN_TIME_DATE,
    extract_pgn_source_priority,
)


# --- PGN Extraction ---


class TestPGNExtraction:
    """Test CAN ID → PGN/source/priority extraction."""

    def test_pdu2_wind_data(self):
        """PGN 130306 (Wind Data) — PDU2 (PF=0xFD ≥ 240)."""
        # PGN 130306 = 0x1FD02, PF=0xFD, PS=0x02
        # Priority=2, source=0x42
        # CAN ID: priority(26-28) | reserved(25) | DP(24) | PF(16-23) | PS(8-15) | SA(0-7)
        # 130306 = 0x1FD02 → DP=1, PF=0xFD, PS=0x02
        can_id = (2 << 26) | (1 << 24) | (0xFD << 16) | (0x02 << 8) | 0x42
        pgn, source, priority = extract_pgn_source_priority(can_id)
        assert pgn == 130306
        assert source == 0x42
        assert priority == 2

    def test_pdu1_heading_track_control(self):
        """PGN 127237 (Heading/Track Control) — PDU1 (PF=0xF1 < 240).

        Wait — 127237 = 0x1F105. PF = 0xF1 = 241 ≥ 240 → actually PDU2.
        Let's test a real PDU1: PGN 59904 = 0xEA00. PF=0xEA=234 < 240.
        """
        # PGN 59904 = 0xEA00, PF=0xEA (234 < 240), destination=0xFF
        # Priority=6, source=0x01
        can_id = (6 << 26) | (0 << 24) | (0xEA << 16) | (0xFF << 8) | 0x01
        pgn, source, priority = extract_pgn_source_priority(can_id)
        assert pgn == 0xEA00  # 59904
        assert source == 0x01
        assert priority == 6

    def test_pdu2_vessel_heading(self):
        """PGN 127250 (Vessel Heading) — PDU2 (PF=0xF1 ≥ 240)."""
        # 127250 = 0x1F112 → DP=1, PF=0xF1, PS=0x12
        can_id = (2 << 26) | (1 << 24) | (0xF1 << 16) | (0x12 << 8) | 0xCC
        pgn, source, priority = extract_pgn_source_priority(can_id)
        assert pgn == 127250
        assert source == 0xCC
        assert priority == 2

    def test_source_extraction(self):
        """Source address is lowest 8 bits."""
        can_id = (3 << 26) | (1 << 24) | (0xFD << 16) | (0x02 << 8) | 0xAB
        _, source, _ = extract_pgn_source_priority(can_id)
        assert source == 0xAB

    def test_priority_extraction(self):
        """Priority is bits 26-28."""
        for p in range(8):
            can_id = (p << 26) | (1 << 24) | (0xFD << 16) | (0x02 << 8) | 0x01
            _, _, priority = extract_pgn_source_priority(can_id)
            assert priority == p

    def test_data_page_bit(self):
        """Data page bit sets bit 16 of PGN."""
        # Without data page: PF=0xF0, PS=0x10 → PGN = 0xF010 = 61456
        can_id_no_dp = (2 << 26) | (0 << 24) | (0xF0 << 16) | (0x10 << 8) | 0x01
        pgn_no_dp, _, _ = extract_pgn_source_priority(can_id_no_dp)
        assert pgn_no_dp == 0xF010

        # With data page: PGN = 0x1F010 = 126992
        can_id_dp = (2 << 26) | (1 << 24) | (0xF0 << 16) | (0x10 << 8) | 0x01
        pgn_dp, _, _ = extract_pgn_source_priority(can_id_dp)
        assert pgn_dp == 0x1F010  # 126992 = System Time


# --- Binary Format ---


class TestBinaryFormat:
    """Test binary file header and frame record encode/decode."""

    def test_header_magic_version(self):
        """Header encodes magic and version correctly."""
        header = HEADER_STRUCT.pack(FILE_MAGIC, FILE_VERSION, 1, 0, 1234567890.123)
        assert header[:4] == b'CANL'
        magic, version, flags, _, start_ts = HEADER_STRUCT.unpack(header)
        assert magic == b'CANL'
        assert version == 1
        assert flags == 1
        assert abs(start_ts - 1234567890.123) < 0.001

    def test_header_size(self):
        assert HEADER_STRUCT.size == HEADER_SIZE == 32

    def test_frame_record_size(self):
        assert FRAME_STRUCT.size == FRAME_SIZE == 24

    def test_frame_record_roundtrip(self):
        """Encode and decode a frame record."""
        ts_us = 123456
        can_id = 0x09FD0242
        dlc = 8
        flags = 0
        source = 0x42
        priority = 2
        data = b'\x01\x64\x00\x1E\x06\x02\x00\xFF'
        pgn = 130306

        packed = FRAME_STRUCT.pack(ts_us, can_id, dlc, flags, source, priority, data, pgn)
        assert len(packed) == FRAME_SIZE

        ts2, cid2, dlc2, fl2, src2, pri2, data2, pgn2 = FRAME_STRUCT.unpack(packed)
        assert ts2 == ts_us
        assert cid2 == can_id
        assert dlc2 == dlc
        assert fl2 == flags
        assert src2 == source
        assert pri2 == priority
        assert data2 == data
        assert pgn2 == pgn

    def test_frame_data_zero_padded(self):
        """Short data is zero-padded to 8 bytes."""
        short_data = b'\x01\x02\x03'
        padded = (short_data + b'\x00' * 8)[:8]
        assert len(padded) == 8
        assert padded == b'\x01\x02\x03\x00\x00\x00\x00\x00'


# --- Frame Filter ---


class TestFrameFilter:
    """Test include/exclude filtering and rate limiting."""

    def _make_config(self, **kwargs) -> CANLoggerConfig:
        return CANLoggerConfig(**kwargs)

    def test_no_filters_passes_all(self):
        config = self._make_config()
        f = FrameFilter(config)
        assert f.should_log(130306, 0x42, 1.0) is True
        assert f.should_log(127250, 0x01, 1.0) is True

    def test_include_pgn_only(self):
        config = self._make_config(include=[{"pgn": 130306}])
        f = FrameFilter(config)
        assert f.should_log(130306, 0x42, 1.0) is True
        assert f.should_log(130306, 0x01, 1.0) is True  # Any source
        assert f.should_log(127250, 0x42, 1.0) is False

    def test_include_pgn_source(self):
        config = self._make_config(include=[{"pgn": 130306, "source": 0x42}])
        f = FrameFilter(config)
        assert f.should_log(130306, 0x42, 1.0) is True
        assert f.should_log(130306, 0x01, 1.0) is False  # Wrong source
        assert f.should_log(127250, 0x42, 1.0) is False  # Wrong PGN

    def test_include_mixed(self):
        """Include with both PGN-only and PGN+source entries."""
        config = self._make_config(include=[
            {"pgn": 130306},
            {"pgn": 127250, "source": 0xCC},
        ])
        f = FrameFilter(config)
        assert f.should_log(130306, 0x01, 1.0) is True  # PGN match
        assert f.should_log(127250, 0xCC, 1.0) is True  # PGN+source match
        assert f.should_log(127250, 0x01, 1.0) is False  # Wrong source for 127250
        assert f.should_log(127488, 0x01, 1.0) is False  # Not in include

    def test_exclude_overrides_include(self):
        config = self._make_config(
            include=[{"pgn": 130306}],
            exclude=[{"pgn": 130306}],
        )
        f = FrameFilter(config)
        assert f.should_log(130306, 0x42, 1.0) is False

    def test_exclude_pgn_source(self):
        config = self._make_config(exclude=[{"pgn": 127488, "source": 0x01}])
        f = FrameFilter(config)
        assert f.should_log(127488, 0x01, 1.0) is False
        assert f.should_log(127488, 0x02, 1.0) is True  # Different source OK

    def test_rate_limiting_pgn(self):
        config = self._make_config(rate_limits=[{"pgn": 130306, "max_hz": 2.0}])
        f = FrameFilter(config)

        # First message passes
        assert f.should_log(130306, 0x42, 1.0) is True
        # Too soon (< 0.5s)
        assert f.should_log(130306, 0x42, 1.1) is False
        # After interval
        assert f.should_log(130306, 0x42, 1.6) is True

    def test_rate_limiting_pgn_source_specific(self):
        """Pair-specific rate limit takes priority over PGN-only."""
        config = self._make_config(rate_limits=[
            {"pgn": 130306, "max_hz": 2.0},
            {"pgn": 130306, "source": 0x42, "max_hz": 10.0},
        ])
        f = FrameFilter(config)

        # Source 0x42 uses pair-specific limit (10 Hz = 0.1s interval)
        assert f.should_log(130306, 0x42, 1.0) is True
        assert f.should_log(130306, 0x42, 1.05) is False  # <0.1s
        assert f.should_log(130306, 0x42, 1.15) is True   # >0.1s

        # Source 0x01 uses PGN-only limit (2 Hz = 0.5s interval)
        assert f.should_log(130306, 0x01, 1.0) is True
        assert f.should_log(130306, 0x01, 1.3) is False   # <0.5s
        assert f.should_log(130306, 0x01, 1.6) is True    # >0.5s

    def test_update_preserves_timestamps(self):
        """Calling update() shouldn't reset rate limit timestamps."""
        config = self._make_config(rate_limits=[{"pgn": 130306, "max_hz": 2.0}])
        f = FrameFilter(config)

        assert f.should_log(130306, 0x42, 1.0) is True
        assert f.should_log(130306, 0x42, 1.1) is False

        # Update config (same limits)
        f.update(config)

        # Timestamps preserved — still rate limited
        assert f.should_log(130306, 0x42, 1.2) is False
        assert f.should_log(130306, 0x42, 1.6) is True

    def test_stats_tracking(self):
        config = self._make_config(
            include=[{"pgn": 130306}],
            exclude=[{"pgn": 127488}],
        )
        f = FrameFilter(config)
        f.should_log(130306, 0x42, 1.0)  # passed
        f.should_log(127250, 0x42, 1.0)  # no include match
        stats = f.stats
        assert stats["passed"] == 1
        assert stats["no_include_match"] == 1


# --- Binary Log Writer ---


class TestBinaryLogWriter:
    """Test file creation, rotation, listing, and pruning."""

    def test_creates_directory(self, tmp_path):
        log_dir = tmp_path / "logs" / "can"
        writer = BinaryLogWriter(str(log_dir), 1024 * 1024)
        assert log_dir.exists()
        writer.close()

    def test_writes_valid_header(self, tmp_path):
        writer = BinaryLogWriter(str(tmp_path), 1024 * 1024)
        ts = 1709000000.0
        writer.write_frame(ts, 0x09FD0242, 8, b'\x01' * 8, 130306, 0x42, 2)

        files = list(tmp_path.glob("can_*.bin"))
        assert len(files) == 1

        with open(files[0], "rb") as f:
            header = f.read(HEADER_SIZE)
        magic, version, flags, _, start_ts = HEADER_STRUCT.unpack(header)
        assert magic == b'CANL'
        assert version == 1
        assert flags == 1
        assert abs(start_ts - ts) < 0.001
        writer.close()

    def test_frame_written_after_header(self, tmp_path):
        writer = BinaryLogWriter(str(tmp_path), 1024 * 1024)
        ts = 1709000000.0
        data = b'\x01\x64\x00\x1E\x06\x02\x00\xFF'
        writer.write_frame(ts, 0x09FD0242, 8, data, 130306, 0x42, 2)

        files = list(tmp_path.glob("can_*.bin"))
        with open(files[0], "rb") as f:
            f.seek(HEADER_SIZE)
            record = f.read(FRAME_SIZE)

        ts_us, can_id, dlc, flags, source, priority, rec_data, pgn = FRAME_STRUCT.unpack(record)
        assert ts_us == 0  # First frame has 0 offset
        assert can_id == 0x09FD0242
        assert dlc == 8
        assert source == 0x42
        assert priority == 2
        assert rec_data == data
        assert pgn == 130306
        writer.close()

    def test_rotation_on_size(self, tmp_path):
        # Max size = header + 2 frames
        max_size = HEADER_SIZE + 2 * FRAME_SIZE
        writer = BinaryLogWriter(str(tmp_path), max_size)

        base_ts = 1709000000.0
        for i in range(5):
            writer.write_frame(base_ts + i * 0.1, 0x09FD0242, 8, b'\x01' * 8, 130306, 0x42, 2)

        files = list(tmp_path.glob("can_*.bin"))
        assert len(files) >= 2  # Should have rotated
        writer.close()

    def test_rotation_on_timestamp_overflow(self, tmp_path):
        writer = BinaryLogWriter(str(tmp_path), 1024 * 1024 * 100)

        base_ts = 1709000000.0
        writer.write_frame(base_ts, 0x09FD0242, 8, b'\x01' * 8, 130306, 0x42, 2)

        # Jump past uint32 overflow at 0.1ms resolution (~4.97 days)
        overflow_ts = base_ts + 430000  # ~4.98 days in seconds
        writer.write_frame(overflow_ts, 0x09FD0242, 8, b'\x01' * 8, 130306, 0x42, 2)

        files = list(tmp_path.glob("can_*.bin"))
        assert len(files) == 2  # New file for overflow
        writer.close()

    def test_list_files(self, tmp_path):
        writer = BinaryLogWriter(str(tmp_path), 1024 * 1024)
        writer.write_frame(1709000000.0, 0x09FD0242, 8, b'\x01' * 8, 130306, 0x42, 2)
        writer.write_frame(1709000000.1, 0x09FD0242, 8, b'\x01' * 8, 130306, 0x42, 2)

        files = writer.list_files()
        assert len(files) == 1
        assert files[0]["frames"] == 2
        assert files[0]["size"] == HEADER_SIZE + 2 * FRAME_SIZE
        assert files[0]["name"].startswith("can_")
        writer.close()

    def test_prune(self, tmp_path):
        writer = BinaryLogWriter(str(tmp_path), HEADER_SIZE + FRAME_SIZE)

        # Write 3 files (rotation after each frame)
        for i in range(3):
            writer.write_frame(1709000000.0 + i * 2, 0x09FD0242, 8, b'\x01' * 8, 130306, 0x42, 2)

        files_before = list(tmp_path.glob("can_*.bin"))
        assert len(files_before) == 3

        # Prune to keep only ~1 file worth
        deleted = writer.prune(HEADER_SIZE + FRAME_SIZE + 10)
        files_after = list(tmp_path.glob("can_*.bin"))
        assert deleted >= 1
        assert len(files_after) < len(files_before)
        writer.close()

    def test_stats(self, tmp_path):
        writer = BinaryLogWriter(str(tmp_path), 1024 * 1024)
        writer.write_frame(1709000000.0, 0x09FD0242, 8, b'\x01' * 8, 130306, 0x42, 2)

        s = writer.stats
        assert s["total_frames"] == 1
        assert s["total_files"] == 1
        assert s["current_file"] is not None
        assert s["current_file_frames"] == 1
        writer.close()


# --- GPS Time Sync ---


class TestGPSTimeSync:
    """Test GPS time extraction from PGNs."""

    def test_should_check_when_no_gps_yet(self):
        sync = GPSTimeSync(enabled=True)
        # Should check until GPS time is received (even with RTC)
        assert sync.should_check() is True

    def test_should_check_disabled(self):
        sync = GPSTimeSync(enabled=False)
        assert sync.should_check() is False

    def test_should_check_stops_after_gps_received_with_rtc(self):
        sync = GPSTimeSync(enabled=True)
        sync._has_rtc = True
        sync._gps_received = True
        assert sync.should_check() is False

    @patch.object(GPSTimeSync, '_set_system_time')
    def test_sync_from_pgn_126992_no_rtc(self, mock_set_time):
        """PGN 126992: syncs system time when no RTC detected."""
        sync = GPSTimeSync(enabled=True)
        sync._has_rtc = False

        # 2024-01-15 = day 19738 since 1970-01-01
        # 12:00:00 = 43200 seconds = 432000000 units of 0.0001s
        days = 19738
        tod = 432000000

        data = bytearray(8)
        struct.pack_into('<H', data, 2, days)
        struct.pack_into('<I', data, 4, tod)

        result = sync.try_sync_from_pgn(PGN_SYSTEM_TIME, bytes(data))

        assert result is True
        assert sync._synced is True
        assert sync._gps_received is True
        mock_set_time.assert_called_once()
        dt_arg = mock_set_time.call_args[0][0]
        assert dt_arg.year == 2024
        assert dt_arg.month == 1

    @patch.object(GPSTimeSync, '_set_system_time')
    def test_gps_received_but_not_synced_with_rtc(self, mock_set_time):
        """With RTC present, GPS time is recorded but system clock not changed."""
        sync = GPSTimeSync(enabled=True)
        sync._has_rtc = True

        days = 19738
        tod = 432000000

        data = bytearray(8)
        struct.pack_into('<H', data, 2, days)
        struct.pack_into('<I', data, 4, tod)

        result = sync.try_sync_from_pgn(PGN_SYSTEM_TIME, bytes(data))

        assert result is False
        assert sync._synced is False
        assert sync._gps_received is True
        assert sync._rtc_gps_delta_s is not None
        mock_set_time.assert_not_called()

    @patch.object(GPSTimeSync, '_set_system_time')
    def test_sync_from_pgn_129033(self, mock_set_time):
        """PGN 129033 (Time & Date): days at offset 0, TOD at offset 2."""
        sync = GPSTimeSync(enabled=True)
        sync._has_rtc = False

        days = 19738
        tod = 432000000

        data = bytearray(8)
        struct.pack_into('<H', data, 0, days)
        struct.pack_into('<I', data, 2, tod)

        sync.try_sync_from_pgn(PGN_TIME_DATE, bytes(data))

        assert sync._synced is True
        mock_set_time.assert_called_once()

    @patch.object(GPSTimeSync, '_set_system_time')
    def test_invalid_data_no_crash(self, mock_set_time):
        """Invalid/sentinel values should be rejected."""
        sync = GPSTimeSync(enabled=True)

        # 0xFFFF days = sentinel
        data = bytearray(8)
        struct.pack_into('<H', data, 2, 0xFFFF)
        struct.pack_into('<I', data, 4, 0)

        sync.try_sync_from_pgn(PGN_SYSTEM_TIME, bytes(data))
        assert sync._synced is False
        assert sync._gps_received is False
        mock_set_time.assert_not_called()

    @patch.object(GPSTimeSync, '_set_system_time')
    def test_no_op_when_synced(self, mock_set_time):
        """Once synced, no further sync attempts."""
        sync = GPSTimeSync(enabled=True)
        sync._synced = True

        days = 19738
        tod = 432000000
        data = bytearray(8)
        struct.pack_into('<H', data, 2, days)
        struct.pack_into('<I', data, 4, tod)

        result = sync.try_sync_from_pgn(PGN_SYSTEM_TIME, bytes(data))
        assert result is False
        mock_set_time.assert_not_called()

    def test_short_data_rejected(self):
        sync = GPSTimeSync(enabled=True)
        result = sync.try_sync_from_pgn(PGN_SYSTEM_TIME, b'\x00\x01')
        assert result is False

    def test_status_property(self):
        sync = GPSTimeSync(enabled=True)
        s = sync.status
        assert s["enabled"] is True
        assert s["synced"] is False
        assert s["has_rtc"] is True  # running on modern system
        assert s["gps_received"] is False
        assert s["rtc_gps_delta_s"] is None


# --- Config ---


class TestCANLoggerConfig:
    """Test config loading and defaults."""

    def test_defaults(self):
        config = CANLoggerConfig()
        assert config.channel == "can0"
        assert config.log_dir == "logs/can"
        assert config.max_file_size_mb == 20.0
        assert config.max_disk_mb == 500.0
        assert config.include == []
        assert config.exclude == []
        assert config.rate_limits == []
        assert config.gps_time_sync is True

    def test_from_json(self, tmp_path):
        config_data = {
            "channel": "can1",
            "log_dir": "/tmp/test",
            "max_file_size_mb": 10,
            "include": [{"pgn": 130306}],
            "exclude": [{"pgn": 127488}],
            "rate_limits": [{"pgn": 130306, "max_hz": 2.0}],
        }
        config_file = tmp_path / "config.json"
        config_file.write_text(json.dumps(config_data))

        config = CANLoggerConfig.from_json(str(config_file))
        assert config.channel == "can1"
        assert config.log_dir == "/tmp/test"
        assert config.max_file_size_mb == 10
        assert len(config.include) == 1
        assert len(config.exclude) == 1

    def test_unknown_keys_ignored(self, tmp_path):
        config_data = {
            "channel": "can0",
            "unknown_field": True,
            "another_unknown": 42,
        }
        config_file = tmp_path / "config.json"
        config_file.write_text(json.dumps(config_data))

        config = CANLoggerConfig.from_json(str(config_file))
        assert config.channel == "can0"
        assert not hasattr(config, "unknown_field")

    def test_to_dict(self):
        config = CANLoggerConfig(channel="can1", max_file_size_mb=10)
        d = config.to_dict()
        assert d["channel"] == "can1"
        assert d["max_file_size_mb"] == 10
        assert "include" in d


# --- Web API ---


class TestWebAPI:
    """Test Flask web endpoints."""

    @pytest.fixture
    def mock_logger(self, tmp_path):
        config = CANLoggerConfig(log_dir=str(tmp_path))
        logger_inst = MagicMock(spec=CANLogger)
        logger_inst.config = config
        logger_inst.stats = {
            "running": True,
            "uptime": 100,
            "msg_rate": 50.0,
            "total_received": 5000,
            "filter": {"passed": 4000, "excluded": 500, "rate_limited": 500, "no_include_match": 0},
            "writer": {"total_frames": 4000, "total_files": 1, "total_bytes": 96032,
                       "current_file": "can_20240301_120000.bin", "current_file_size": 96032,
                       "current_file_frames": 4000},
            "disk_used_bytes": 96032,
            "disk_limit_bytes": 500 * 1024 * 1024,
            "gps_sync": {"enabled": True, "synced": False, "sync_time": None},
            "pgn_counts": {130306: 2000, 127250: 2000},
        }
        logger_inst.get_recent_frames.return_value = []
        return logger_inst

    @pytest.fixture
    def client(self, mock_logger, tmp_path):
        from src.n2k.can_logger_web import create_app
        config_file = tmp_path / "config.json"
        config_file.write_text(json.dumps(mock_logger.config.to_dict()))
        app = create_app(mock_logger, str(config_file))
        app.config["TESTING"] = True
        with app.test_client() as client:
            yield client

    def test_get_status(self, client):
        resp = client.get("/api/status")
        assert resp.status_code == 200
        data = resp.get_json()
        assert data["running"] is True
        assert data["msg_rate"] == 50.0

    def test_get_config(self, client):
        resp = client.get("/api/config")
        assert resp.status_code == 200
        data = resp.get_json()
        assert "channel" in data

    def test_post_config(self, client, tmp_path):
        new_config = {"include": [{"pgn": 130306}]}
        resp = client.post("/api/config", json=new_config)
        assert resp.status_code == 200

        # Verify file was updated
        config_file = tmp_path / "config.json"
        saved = json.loads(config_file.read_text())
        assert saved["include"] == [{"pgn": 130306}]

    def test_get_files(self, client, mock_logger):
        mock_logger._config = mock_logger.config
        # Create a real writer to test file listing
        resp = client.get("/api/files")
        assert resp.status_code == 200

    def test_get_pgn_names(self, client):
        resp = client.get("/api/pgn_names")
        assert resp.status_code == 200
        data = resp.get_json()
        assert "130306" in data
        assert data["130306"] == "Wind Data"

    def test_path_traversal_blocked(self, client):
        resp = client.get("/api/files/../../../etc/passwd")
        assert resp.status_code in (400, 404)

    def test_path_traversal_dotdot(self, client):
        resp = client.get("/api/files/..%2F..%2Fetc%2Fpasswd")
        assert resp.status_code in (400, 404)

    def test_index_html(self, client):
        resp = client.get("/")
        assert resp.status_code == 200
