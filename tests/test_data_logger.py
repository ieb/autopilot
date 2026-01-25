"""
Tests for NMEA2000 Data Logger Module
=====================================

Tests log rotation, mode detection, format validation, and compression.
"""

import gzip
import json
import os
import tempfile
import time
from dataclasses import asdict
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock

import pytest

from src.n2k.data_logger import (
    DataLogger,
    DataLoggerConfig,
    OperationMode,
    HelmSource,
    LogRecord,
    PositionSample,
)
from src.sensors.nmea2000_interface import N2KData, PilotMode


class TestLogRecord:
    """Test LogRecord dataclass."""
    
    def test_default_values(self):
        """Test LogRecord has correct defaults."""
        record = LogRecord(timestamp=1234567890.0)
        
        assert record.timestamp == 1234567890.0
        assert record.heading == 0.0
        assert record.awa == 0.0
        assert record.helm_source == "unknown"
        assert record.operation_mode == "unknown"
        assert record.ml_pilot_engaged is False
        
    def test_asdict_serialization(self):
        """Test LogRecord can be serialized to dict."""
        record = LogRecord(
            timestamp=1234567890.0,
            heading=180.0,
            awa=45.0,
            aws=15.0,
            helm_source="human",
            operation_mode="sailing",
        )
        
        d = asdict(record)
        
        assert d["timestamp"] == 1234567890.0
        assert d["heading"] == 180.0
        assert d["awa"] == 45.0
        assert d["helm_source"] == "human"
        
    def test_json_serialization(self):
        """Test LogRecord can be serialized to JSON."""
        record = LogRecord(
            timestamp=1234567890.0,
            heading=180.0,
            latitude=37.7749,
            longitude=-122.4194,
        )
        
        json_str = json.dumps(asdict(record))
        parsed = json.loads(json_str)
        
        assert parsed["timestamp"] == 1234567890.0
        assert parsed["heading"] == 180.0
        assert parsed["latitude"] == 37.7749


class TestOperationMode:
    """Test OperationMode enum."""
    
    def test_all_modes_exist(self):
        """Test all expected operation modes exist."""
        assert OperationMode.ANCHOR
        assert OperationMode.MOTORING
        assert OperationMode.SAILING
        assert OperationMode.UNKNOWN
        
    def test_mode_names(self):
        """Test mode names are lowercase-able."""
        assert OperationMode.ANCHOR.name.lower() == "anchor"
        assert OperationMode.SAILING.name.lower() == "sailing"


class TestHelmSource:
    """Test HelmSource enum."""
    
    def test_all_sources_exist(self):
        """Test all expected helm sources exist."""
        assert HelmSource.HUMAN
        assert HelmSource.RAYMARINE_COMPASS
        assert HelmSource.RAYMARINE_WIND
        assert HelmSource.ML_PILOT
        assert HelmSource.UNKNOWN
        
    def test_source_names(self):
        """Test source names are lowercase-able."""
        assert HelmSource.HUMAN.name.lower() == "human"
        assert HelmSource.RAYMARINE_COMPASS.name.lower() == "raymarine_compass"


class TestDataLoggerConfig:
    """Test DataLoggerConfig dataclass."""
    
    def test_default_config(self):
        """Test default configuration values."""
        config = DataLoggerConfig()
        
        assert config.can_channel == "can0"
        assert config.log_dir == "logs/n2k"
        assert config.max_file_size_mb == 256.0
        assert config.max_file_age_minutes == 60.0
        assert config.sample_rate_hz == 1.0
        assert config.compression is True
        
    def test_custom_config(self):
        """Test custom configuration."""
        config = DataLoggerConfig(
            can_channel="can1",
            log_dir="/tmp/logs",
            max_file_size_mb=128.0,
            compression=False,
        )
        
        assert config.can_channel == "can1"
        assert config.log_dir == "/tmp/logs"
        assert config.max_file_size_mb == 128.0
        assert config.compression is False


class TestDataLoggerAnchorDetection:
    """Test anchor detection logic."""
    
    def test_anchor_detected_low_sog_small_wander(self):
        """Test anchor detected with low SOG and small position wander."""
        config = DataLoggerConfig(
            anchor_sog_threshold=0.5,
            anchor_position_radius=50.0,
            anchor_detection_window_s=60.0,
        )
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            mock_n2k.start.return_value = True
            MockN2K.return_value = mock_n2k
            
            logger = DataLogger(config)
            
            # Simulate position buffer with low SOG and small wander
            now = time.time()
            base_lat, base_lon = 37.7749, -122.4194
            
            for i in range(20):
                logger._position_buffer.append(PositionSample(
                    timestamp=now - (19 - i),
                    latitude=base_lat + (i % 3) * 0.00001,  # ~1m wander
                    longitude=base_lon + (i % 2) * 0.00001,
                    sog=0.2,  # Low SOG
                ))
                
            assert logger._is_at_anchor() is True
            
    def test_anchor_not_detected_high_sog(self):
        """Test anchor not detected with high SOG."""
        config = DataLoggerConfig(
            anchor_sog_threshold=0.5,
            anchor_position_radius=50.0,
        )
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            mock_n2k.start.return_value = True
            MockN2K.return_value = mock_n2k
            
            logger = DataLogger(config)
            
            now = time.time()
            base_lat, base_lon = 37.7749, -122.4194
            
            for i in range(20):
                logger._position_buffer.append(PositionSample(
                    timestamp=now - (19 - i),
                    latitude=base_lat,
                    longitude=base_lon,
                    sog=5.0,  # High SOG
                ))
                
            assert logger._is_at_anchor() is False
            
    def test_anchor_not_detected_large_wander(self):
        """Test anchor not detected with large position wander."""
        config = DataLoggerConfig(
            anchor_sog_threshold=0.5,
            anchor_position_radius=50.0,
        )
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            MockN2K.return_value = mock_n2k
            
            logger = DataLogger(config)
            
            now = time.time()
            base_lat, base_lon = 37.7749, -122.4194
            
            for i in range(20):
                logger._position_buffer.append(PositionSample(
                    timestamp=now - (19 - i),
                    latitude=base_lat + i * 0.001,  # ~100m per step
                    longitude=base_lon,
                    sog=0.2,
                ))
                
            assert logger._is_at_anchor() is False
            
    def test_anchor_not_detected_insufficient_samples(self):
        """Test anchor not detected with insufficient samples."""
        config = DataLoggerConfig()
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            MockN2K.return_value = mock_n2k
            
            logger = DataLogger(config)
            
            # Only 5 samples (< 10 required)
            now = time.time()
            for i in range(5):
                logger._position_buffer.append(PositionSample(
                    timestamp=now - i,
                    latitude=37.7749,
                    longitude=-122.4194,
                    sog=0.1,
                ))
                
            assert logger._is_at_anchor() is False


class TestDataLoggerHelmSourceDetection:
    """Test helm source detection."""
    
    def test_human_helm_when_pilot_standby(self):
        """Test human detected when pilot in standby."""
        config = DataLoggerConfig()
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            MockN2K.return_value = mock_n2k
            
            logger = DataLogger(config)
            
            # Create N2KData with pilot in standby
            now = time.time()
            data = N2KData(
                timestamp=now,
                pilot_mode=PilotMode.STANDBY,
                pilot_mode_timestamp=now,
            )
            
            logger._update_helm_source(data)
            
            assert logger._helm_source == HelmSource.HUMAN
            
    def test_raymarine_compass_when_auto_mode(self):
        """Test Raymarine compass detected when pilot in auto mode."""
        config = DataLoggerConfig()
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            MockN2K.return_value = mock_n2k
            
            logger = DataLogger(config)
            
            now = time.time()
            data = N2KData(
                timestamp=now,
                pilot_mode=PilotMode.AUTO,
                pilot_mode_timestamp=now,
            )
            
            logger._update_helm_source(data)
            
            assert logger._helm_source == HelmSource.RAYMARINE_COMPASS
            
    def test_raymarine_wind_when_wind_mode(self):
        """Test Raymarine wind detected when pilot in wind mode."""
        config = DataLoggerConfig()
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            MockN2K.return_value = mock_n2k
            
            logger = DataLogger(config)
            
            now = time.time()
            data = N2KData(
                timestamp=now,
                pilot_mode=PilotMode.WIND,
                pilot_mode_timestamp=now,
            )
            
            logger._update_helm_source(data)
            
            assert logger._helm_source == HelmSource.RAYMARINE_WIND
            
    def test_unknown_helm_with_stale_data(self):
        """Test unknown helm source with stale pilot data."""
        config = DataLoggerConfig()
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            MockN2K.return_value = mock_n2k
            
            logger = DataLogger(config)
            
            # Data from 10 seconds ago (> 5s threshold)
            now = time.time()
            data = N2KData(
                timestamp=now,
                pilot_mode=PilotMode.AUTO,
                pilot_mode_timestamp=now - 10,  # 10 seconds old
                track_control_timestamp=now - 10,
            )
            
            logger._update_helm_source(data)
            
            assert logger._helm_source == HelmSource.UNKNOWN


class TestDataLoggerLogRotation:
    """Test log rotation logic."""
    
    def test_should_rotate_when_no_file(self):
        """Test rotation needed when no file is open."""
        config = DataLoggerConfig()
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            MockN2K.return_value = mock_n2k
            
            logger = DataLogger(config)
            
            # No file open
            assert logger._should_rotate() is True
            
    def test_should_rotate_when_max_age_exceeded(self):
        """Test rotation when max age exceeded."""
        config = DataLoggerConfig(max_file_age_minutes=60.0)
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            MockN2K.return_value = mock_n2k
            
            logger = DataLogger(config)
            
            with tempfile.TemporaryDirectory() as tmpdir:
                logger.config.log_dir = tmpdir
                
                # Create a file
                logger._open_new_file()
                
                # Simulate file is 61 minutes old
                logger._file_start_time = time.time() - (61 * 60)
                
                assert logger._should_rotate() is True
                
                logger._close_current_file()
                
    def test_should_not_rotate_when_within_limits(self):
        """Test no rotation when within limits."""
        config = DataLoggerConfig(
            max_file_age_minutes=60.0,
            max_file_size_mb=256.0,
        )
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            MockN2K.return_value = mock_n2k
            
            with tempfile.TemporaryDirectory() as tmpdir:
                config.log_dir = tmpdir
                logger = DataLogger(config)
                
                # Create a file
                logger._open_new_file()
                
                # File is new (start time is now)
                assert logger._should_rotate() is False
                
                logger._close_current_file()


class TestDataLoggerCompression:
    """Test log compression."""
    
    def test_compressed_file_created(self):
        """Test gzip compressed file is created."""
        config = DataLoggerConfig(compression=True)
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            MockN2K.return_value = mock_n2k
            
            with tempfile.TemporaryDirectory() as tmpdir:
                config.log_dir = tmpdir
                logger = DataLogger(config)
                
                # Write a record
                record = LogRecord(
                    timestamp=time.time(),
                    heading=180.0,
                    awa=45.0,
                )
                logger._write_record(record)
                logger._close_current_file()
                
                # Check file exists and is gzip
                files = list(Path(tmpdir).glob("*.jsonlog.gz"))
                assert len(files) == 1
                
                # Verify can decompress
                with gzip.open(files[0], 'rt') as f:
                    line = f.readline()
                    data = json.loads(line)
                    assert data["heading"] == 180.0
                    
    def test_uncompressed_file_created(self):
        """Test uncompressed file is created when compression disabled."""
        config = DataLoggerConfig(compression=False)
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            MockN2K.return_value = mock_n2k
            
            with tempfile.TemporaryDirectory() as tmpdir:
                config.log_dir = tmpdir
                logger = DataLogger(config)
                
                # Write a record
                record = LogRecord(
                    timestamp=time.time(),
                    heading=270.0,
                )
                logger._write_record(record)
                logger._close_current_file()
                
                # Check file exists and is not gzip
                files = list(Path(tmpdir).glob("*.jsonlog"))
                gz_files = list(Path(tmpdir).glob("*.jsonlog.gz"))
                
                assert len(files) == 1
                assert len(gz_files) == 0
                
                # Verify can read directly
                with open(files[0], 'r') as f:
                    line = f.readline()
                    data = json.loads(line)
                    assert data["heading"] == 270.0


class TestDataLoggerRecordFormat:
    """Test record format compatibility with training data loader."""
    
    def test_record_has_training_required_fields(self):
        """Test LogRecord has all fields required for training."""
        record = LogRecord(timestamp=time.time())
        record_dict = asdict(record)
        
        # Required fields from LoggedFrame in data_loader.py
        required_fields = [
            "timestamp",
            "heading",
            "pitch",
            "roll",
            "yaw_rate",
            "awa",
            "aws",
            "stw",
            "cog",
            "sog",
            "rudder_angle",
            "target_heading",
            "target_awa",
            "target_twa",
            "mode",
            "latitude",
            "longitude",
        ]
        
        for field in required_fields:
            assert field in record_dict, f"Missing required field: {field}"
            
    def test_record_has_additional_logging_fields(self):
        """Test LogRecord has additional fields for logging context."""
        record = LogRecord(timestamp=time.time())
        record_dict = asdict(record)
        
        additional_fields = [
            "helm_source",
            "operation_mode",
            "pilot_heading",
            "pilot_wind_angle",
            "pilot_mode",
            "pilot_submode",
            "engine_rpm",
            "ml_pilot_engaged",
            "twa",
            "tws",
        ]
        
        for field in additional_fields:
            assert field in record_dict, f"Missing additional field: {field}"


class TestDataLoggerOperationModeDetection:
    """Test operation mode detection."""
    
    def test_motoring_detected_with_engine_rpm(self):
        """Test motoring mode detected with engine RPM."""
        config = DataLoggerConfig(motoring_min_rpm=500.0)
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            MockN2K.return_value = mock_n2k
            
            logger = DataLogger(config)
            
            now = time.time()
            data = N2KData(
                timestamp=now,
                engine_rpm=1500.0,  # Above threshold
                sog=5.0,
            )
            
            logger._update_operation_mode(data)
            
            assert logger._operation_mode == OperationMode.MOTORING
            
    def test_sailing_detected_with_wind_and_speed(self):
        """Test sailing mode detected with adequate wind and speed."""
        config = DataLoggerConfig(
            motoring_min_rpm=500.0,
            motoring_low_wind_threshold=5.0,
        )
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            MockN2K.return_value = mock_n2k
            
            logger = DataLogger(config)
            
            now = time.time()
            data = N2KData(
                timestamp=now,
                engine_rpm=0.0,  # No engine
                awa=90.0,  # Beam reach
                aws=15.0,  # Good wind
                stw=6.0,
                sog=6.0,
                heading_n2k=180.0,
            )
            
            logger._update_operation_mode(data)
            
            assert logger._operation_mode == OperationMode.SAILING


class TestDataLoggerStats:
    """Test statistics tracking."""
    
    def test_stats_initial_values(self):
        """Test initial stats values."""
        config = DataLoggerConfig()
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            mock_n2k.stats = {"message_count": 0}
            MockN2K.return_value = mock_n2k
            
            logger = DataLogger(config)
            stats = logger.stats
            
            assert stats["total_records"] == 0
            assert stats["files_written"] == 0
            assert stats["skipped_anchor"] == 0
            assert stats["operation_mode"] == "UNKNOWN"
            assert stats["helm_source"] == "UNKNOWN"
            
    def test_stats_updated_after_records(self):
        """Test stats updated after writing records."""
        config = DataLoggerConfig(compression=False)
        
        with patch('src.n2k.data_logger.NMEA2000Interface') as MockN2K:
            mock_n2k = MagicMock()
            mock_n2k.stats = {"message_count": 100}
            MockN2K.return_value = mock_n2k
            
            with tempfile.TemporaryDirectory() as tmpdir:
                config.log_dir = tmpdir
                logger = DataLogger(config)
                
                # Write some records
                for i in range(5):
                    record = LogRecord(timestamp=time.time() + i)
                    logger._write_record(record)
                    
                stats = logger.stats
                
                assert stats["total_records"] == 5
                assert stats["files_written"] == 1
                
                logger._close_current_file()
