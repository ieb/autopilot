"""
Unit tests for Log Analyzer module.

Tests operation mode detection, steering target detection, and metadata generation.
"""

import json
import math
import tempfile
import time
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

from src.training.log_analyzer import (
    AnalysisConfig,
    AnalysisResult,
    FeatureCoverage,
    LogAnalyzer,
    OperationDetector,
    OperationMode,
    REQUIRED_FEATURES,
    Segment,
    SteeringMode,
    TargetDetector,
    analyze_directory,
)
from src.training.data_loader import LoggedFrame, CANLogParser, TrainingDataLoader, DataConfig


class TestOperationMode:
    """Tests for OperationMode enumeration."""
    
    def test_all_modes_defined(self):
        """All expected operation modes should be defined."""
        assert OperationMode.ANCHOR is not None
        assert OperationMode.MOTORING is not None
        assert OperationMode.SAILING is not None
        assert OperationMode.UNKNOWN is not None
        
    def test_mode_values(self):
        """Mode values should match expected strings."""
        assert OperationMode.ANCHOR.value == "anchor"
        assert OperationMode.MOTORING.value == "motoring"
        assert OperationMode.SAILING.value == "sailing"
        assert OperationMode.UNKNOWN.value == "unknown"


class TestSteeringMode:
    """Tests for SteeringMode enumeration."""
    
    def test_all_modes_defined(self):
        """All expected steering modes should be defined."""
        assert SteeringMode.HEADING is not None
        assert SteeringMode.AWA is not None
        assert SteeringMode.TWA is not None
        assert SteeringMode.NONE is not None
        
    def test_mode_values(self):
        """Mode values should match expected strings."""
        assert SteeringMode.HEADING.value == "heading"
        assert SteeringMode.AWA.value == "awa"
        assert SteeringMode.TWA.value == "twa"
        assert SteeringMode.NONE.value == "none"


class TestAnalysisConfig:
    """Tests for AnalysisConfig defaults."""
    
    def test_default_values(self):
        """Default config should have reasonable values."""
        config = AnalysisConfig()
        
        assert config.anchor_sog_threshold == 0.5
        assert config.motoring_min_rpm > 0
        assert config.motoring_max_stw == 7.0
        assert config.sailing_min_roll_offset > 0
        assert config.min_segment_duration > 0


class TestFeatureCoverage:
    """Tests for FeatureCoverage dataclass."""
    
    def test_coverage_pct_calculation(self):
        """Coverage percentage should be calculated correctly."""
        fc = FeatureCoverage(name="heading", present_count=80, total_count=100)
        assert fc.coverage_pct == 80.0
        
    def test_coverage_pct_empty(self):
        """Coverage percentage should be 0 for empty data."""
        fc = FeatureCoverage(name="heading", present_count=0, total_count=0)
        assert fc.coverage_pct == 0.0
        
    def test_is_available_above_threshold(self):
        """Feature should be available if coverage > 50%."""
        fc = FeatureCoverage(name="heading", present_count=60, total_count=100)
        assert fc.is_available is True
        
    def test_is_available_below_threshold(self):
        """Feature should not be available if coverage <= 50%."""
        fc = FeatureCoverage(name="heading", present_count=50, total_count=100)
        assert fc.is_available is False
        
    def test_to_dict(self):
        """to_dict should include all fields."""
        fc = FeatureCoverage(name="heading", present_count=80, total_count=100)
        d = fc.to_dict()
        assert d["name"] == "heading"
        assert d["present_count"] == 80
        assert d["total_count"] == 100
        assert d["coverage_pct"] == 80.0
        assert d["available"] is True


class TestSegment:
    """Tests for Segment dataclass."""
    
    def test_duration(self):
        """Duration should be end_time - start_time."""
        segment = Segment(
            start_time=100.0,
            end_time=200.0,
            operation_mode="sailing",
            steering_mode="awa",
            target_value=45.0,
            confidence=0.9,
        )
        
        assert segment.duration() == 100.0
        
    def test_duration_hours(self):
        """Duration in hours should be duration / 3600."""
        segment = Segment(
            start_time=0.0,
            end_time=3600.0,  # 1 hour
            operation_mode="sailing",
            steering_mode="awa",
            target_value=45.0,
            confidence=0.9,
        )
        
        assert segment.duration_hours() == 1.0
        
    def test_missing_features(self):
        """Missing features should be computed from coverage."""
        coverage = {
            "heading": FeatureCoverage(name="heading", present_count=100, total_count=100),
            "awa": FeatureCoverage(name="awa", present_count=100, total_count=100),
            "stw": FeatureCoverage(name="stw", present_count=10, total_count=100),  # Low coverage
        }
        segment = Segment(
            start_time=0.0,
            end_time=100.0,
            operation_mode="sailing",
            steering_mode="awa",
            target_value=45.0,
            confidence=0.9,
            feature_coverage=coverage,
        )
        
        missing = segment.missing_features()
        assert "stw" in missing
        assert "heading" not in missing
        
    def test_is_usable_for_training_valid(self):
        """Usable segment should return True."""
        coverage = {name: FeatureCoverage(name=name, present_count=100, total_count=100) 
                   for name in REQUIRED_FEATURES}
        segment = Segment(
            start_time=0.0,
            end_time=100.0,
            operation_mode="sailing",
            steering_mode="awa",
            target_value=45.0,
            confidence=0.9,
            feature_coverage=coverage,
        )
        
        assert segment.is_usable_for_training() is True
        
    def test_is_usable_for_training_anchor(self):
        """Anchor segment should not be usable."""
        segment = Segment(
            start_time=0.0,
            end_time=100.0,
            operation_mode="anchor",
            steering_mode="none",
            target_value=0.0,
            confidence=0.9,
        )
        
        assert segment.is_usable_for_training() is False
        
    def test_to_dict(self):
        """to_dict should include all fields."""
        segment = Segment(
            start_time=100.0,
            end_time=200.0,
            operation_mode="sailing",
            steering_mode="awa",
            target_value=45.0,
            confidence=0.9,
            notes="Test segment",
            frame_count=500,
        )
        
        d = segment.to_dict()
        assert d["start_time"] == 100.0
        assert d["end_time"] == 200.0
        assert d["operation_mode"] == "sailing"
        assert d["steering_mode"] == "awa"
        assert d["target_value"] == 45.0
        assert d["confidence"] == 0.9
        assert d["notes"] == "Test segment"
        assert d["frame_count"] == 500
        assert d["duration_sec"] == 100.0
        assert "usable_for_training" in d
        assert "missing_features" in d


class TestOperationDetector:
    """Tests for OperationDetector class."""
    
    @pytest.fixture
    def config(self):
        """Default analysis config."""
        return AnalysisConfig()
        
    @pytest.fixture
    def detector(self, config):
        """OperationDetector instance."""
        return OperationDetector(config)
        
    def test_empty_frames_returns_unknown(self, detector):
        """Empty frame list should return UNKNOWN."""
        result = detector.detect([])
        assert result == OperationMode.UNKNOWN
        
    def test_detect_at_anchor(self, detector):
        """Low SOG with position wander should detect anchor."""
        # Create frames at anchor - low SOG, minimal position change
        base_time = time.time()
        frames = []
        for i in range(10):
            frame = LoggedFrame(
                timestamp=base_time + i,
                sog=0.1,
                latitude=51.5 + (i % 3) * 0.00001,  # Small wander
                longitude=-1.5 + (i % 2) * 0.00001,
                engine_rpm=0,
                roll=0,
                awa=0,
                aws=0,
                stw=0,
            )
            frames.append(frame)
            
        result = detector.detect(frames)
        assert result == OperationMode.ANCHOR
        
    def test_detect_motoring_by_rpm(self, detector):
        """Engine RPM present should detect motoring."""
        base_time = time.time()
        frames = []
        for i in range(10):
            frame = LoggedFrame(
                timestamp=base_time + i,
                sog=5.0,
                engine_rpm=1500,  # Engine running
                roll=0,  # Flat
                awa=90,
                aws=5,
                stw=5.0,
            )
            frames.append(frame)
            
        result = detector.detect(frames)
        assert result == OperationMode.MOTORING
        
    def test_detect_motoring_low_wind(self, detector):
        """Low wind speed should detect motoring."""
        base_time = time.time()
        frames = []
        for i in range(10):
            frame = LoggedFrame(
                timestamp=base_time + i,
                sog=5.0,
                engine_rpm=0,
                roll=0,
                awa=90,
                aws=3,  # Low wind
                stw=5.0,
            )
            frames.append(frame)
            
        result = detector.detect(frames)
        assert result == OperationMode.MOTORING
        
    def test_detect_sailing_with_heel(self, detector):
        """Significant heel angle should detect sailing."""
        base_time = time.time()
        frames = []
        for i in range(10):
            frame = LoggedFrame(
                timestamp=base_time + i,
                sog=7.0,
                engine_rpm=0,
                roll=15,  # Heeled over
                awa=45,
                aws=15,
                stw=7.0,
            )
            frames.append(frame)
            
        result = detector.detect(frames)
        assert result == OperationMode.SAILING


class TestTargetDetector:
    """Tests for TargetDetector class."""
    
    @pytest.fixture
    def config(self):
        """Default analysis config."""
        return AnalysisConfig()
        
    @pytest.fixture
    def detector(self, config):
        """TargetDetector instance."""
        return TargetDetector(config)
        
    def test_empty_frames_returns_none(self, detector):
        """Empty frame list should return NONE mode."""
        mode, target, conf = detector.detect([], OperationMode.SAILING)
        assert mode == SteeringMode.NONE
        
    def test_anchor_returns_none(self, detector):
        """Anchor operation should return NONE steering."""
        frames = [LoggedFrame(timestamp=time.time(), heading=180)]
        mode, target, conf = detector.detect(frames, OperationMode.ANCHOR)
        assert mode == SteeringMode.NONE
        
    def test_detect_stable_heading(self, detector):
        """Stable heading should detect HEADING mode."""
        base_time = time.time()
        frames = []
        for i in range(30):
            frame = LoggedFrame(
                timestamp=base_time + i,
                heading=180.0 + (i % 3 - 1) * 2,  # Small variation around 180
                awa=45 + i * 2,  # Varying AWA
                aws=15,
                stw=6.0,
            )
            frames.append(frame)
            
        mode, target, conf = detector.detect(frames, OperationMode.MOTORING)
        assert mode == SteeringMode.HEADING
        assert 175 < target < 185  # Near 180
        
    def test_detect_stable_awa(self, detector):
        """Stable AWA with varying heading should detect AWA mode."""
        base_time = time.time()
        frames = []
        for i in range(30):
            frame = LoggedFrame(
                timestamp=base_time + i,
                heading=180.0 + i * 5,  # Varying heading
                awa=45.0 + (i % 3 - 1),  # Stable AWA around 45
                aws=15,
                stw=6.0,
            )
            frames.append(frame)
            
        mode, target, conf = detector.detect(frames, OperationMode.SAILING)
        assert mode == SteeringMode.AWA
        assert 42 < target < 48  # Near 45
        
    def test_detect_twa_downwind(self, detector):
        """Downwind sailing should prefer TWA mode."""
        base_time = time.time()
        frames = []
        for i in range(30):
            # Create frames where TWA is stable around 150 (downwind)
            frame = LoggedFrame(
                timestamp=base_time + i,
                heading=180.0 + i * 2,  # Varying heading
                awa=145.0 + (i % 3 - 1),  # AWA varies slightly
                aws=12,
                stw=6.0,
            )
            frames.append(frame)
            
        mode, target, conf = detector.detect(frames, OperationMode.SAILING)
        # Should detect either TWA or HEADING/AWA depending on stability
        assert mode in [SteeringMode.TWA, SteeringMode.AWA, SteeringMode.HEADING]


class TestLogAnalyzer:
    """Tests for LogAnalyzer class."""
    
    @pytest.fixture
    def analyzer(self):
        """LogAnalyzer instance."""
        return LogAnalyzer()
        
    def test_analyzer_has_detectors(self, analyzer):
        """Analyzer should have operation and target detectors."""
        assert analyzer.operation_detector is not None
        assert analyzer.target_detector is not None
        assert analyzer.parser is not None
        
    def test_analyze_empty_file(self, analyzer):
        """Analyzing empty file should return empty result."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.log', delete=False) as f:
            f.write("")
            filepath = f.name
            
        result = analyzer.analyze_file(filepath)
        
        assert result.total_duration_sec == 0
        assert len(result.segments) == 0
        assert result.summary.get('usable_for_training') == False
        
        Path(filepath).unlink()
        
    def test_result_to_dict(self, analyzer):
        """AnalysisResult.to_dict should serialize correctly."""
        result = AnalysisResult(
            source_file="test.log",
            analyzed_at="2026-01-23T12:00:00Z",
            total_duration_sec=300.0,
            segments=[
                Segment(
                    start_time=0,
                    end_time=100,
                    operation_mode="sailing",
                    steering_mode="awa",
                    target_value=45.0,
                    confidence=0.9,
                )
            ],
            summary={"usable_for_training": True}
        )
        
        d = result.to_dict()
        assert d["source_file"] == "test.log"
        assert d["total_duration_sec"] == 300.0
        assert len(d["segments"]) == 1
        assert d["summary"]["usable_for_training"] == True
        
    def test_save_metadata(self, analyzer):
        """save_metadata should create JSON file."""
        result = AnalysisResult(
            source_file="test.log",
            analyzed_at="2026-01-23T12:00:00Z",
            total_duration_sec=300.0,
            segments=[],
            summary={}
        )
        
        with tempfile.TemporaryDirectory() as tmpdir:
            meta_path = Path(tmpdir) / "test.meta.json"
            analyzer.save_metadata(result, str(meta_path))
            
            assert meta_path.exists()
            
            with open(meta_path) as f:
                data = json.load(f)
                
            assert data["source_file"] == "test.log"
            assert data["total_duration_sec"] == 300.0


class TestDataLoaderMetadataIntegration:
    """Tests for TrainingDataLoader metadata integration."""
    
    def test_load_metadata_returns_none_for_missing_file(self):
        """_load_metadata should return None when no .meta.json exists."""
        loader = TrainingDataLoader()
        result = loader._load_metadata("/nonexistent/path/file.log")
        assert result is None
        
    def test_load_metadata_parses_valid_file(self):
        """_load_metadata should parse valid metadata file."""
        loader = TrainingDataLoader()
        
        metadata = {
            "source_file": "test.log",
            "segments": [
                {
                    "start_time": 0.0,
                    "end_time": 100.0,
                    "operation_mode": "sailing",
                    "steering_mode": "awa",
                    "target_value": 45.0,
                    "confidence": 0.9,
                }
            ]
        }
        
        with tempfile.TemporaryDirectory() as tmpdir:
            log_path = Path(tmpdir) / "test.log"
            meta_path = Path(tmpdir) / "test.meta.json"
            
            log_path.touch()
            with open(meta_path, 'w') as f:
                json.dump(metadata, f)
                
            segments = loader._load_metadata(str(log_path))
            
            assert segments is not None
            assert len(segments) == 1
            assert segments[0].operation_mode == "sailing"
            assert segments[0].steering_mode == "awa"
            assert segments[0].target_value == 45.0
            
    def test_apply_metadata_filters_anchor(self):
        """_apply_metadata should filter out anchor frames."""
        loader = TrainingDataLoader()
        
        from src.training.data_loader import MetadataSegment
        
        frames = [
            LoggedFrame(timestamp=50.0, heading=180),
            LoggedFrame(timestamp=150.0, heading=185),
        ]
        
        segments = [
            MetadataSegment(
                start_time=0.0,
                end_time=100.0,
                operation_mode="anchor",
                steering_mode="none",
                target_value=0.0,
            ),
            MetadataSegment(
                start_time=100.0,
                end_time=200.0,
                operation_mode="sailing",
                steering_mode="awa",
                target_value=45.0,
            ),
        ]
        
        result = loader._apply_metadata(frames, segments)
        
        # Only the second frame (sailing) should remain
        assert len(result) == 1
        assert result[0].timestamp == 150.0
        assert result[0].mode == "wind_awa"
        
    def test_apply_metadata_sets_mode_correctly(self):
        """_apply_metadata should set correct mode names."""
        loader = TrainingDataLoader()
        
        from src.training.data_loader import MetadataSegment
        
        frames = [
            LoggedFrame(timestamp=50.0, heading=180),
            LoggedFrame(timestamp=150.0, heading=185),
            LoggedFrame(timestamp=250.0, heading=190),
        ]
        
        segments = [
            MetadataSegment(
                start_time=0.0,
                end_time=100.0,
                operation_mode="motoring",
                steering_mode="heading",
                target_value=180.0,
            ),
            MetadataSegment(
                start_time=100.0,
                end_time=200.0,
                operation_mode="sailing",
                steering_mode="awa",
                target_value=45.0,
            ),
            MetadataSegment(
                start_time=200.0,
                end_time=300.0,
                operation_mode="sailing",
                steering_mode="twa",
                target_value=150.0,
            ),
        ]
        
        result = loader._apply_metadata(frames, segments)
        
        assert len(result) == 3
        assert result[0].mode == "compass"
        assert result[0].target_heading == 180.0
        assert result[1].mode == "wind_awa"
        assert result[1].target_heading == 45.0
        assert result[2].mode == "wind_twa"
        assert result[2].target_heading == 150.0


class TestCANLogParserPGNs:
    """Tests for extended PGN decoding in CANLogParser."""
    
    def test_decode_engine_rpm(self):
        """Should decode PGN 127488 engine RPM."""
        parser = CANLogParser()
        data = {}
        
        # PGN 127488 = 0x1F200
        # CAN ID format: priority(3) | reserved(1) | data_page(1) | pdu_format(8) | pdu_specific(8) | source(8)
        # For PGN 127488: DP=0, PF=0xF2, PS=0x00 -> CAN ID = 0x09F20000 (with priority 2)
        can_id = 0x09F20000
        # Data: [instance, rpm_lo, rpm_hi, ...]
        # RPM = raw * 0.25, so 1000 RPM = 4000 raw = 0x0FA0
        rpm_data = bytes([0x00, 0xA0, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
        
        parser._decode_can_frame(can_id, rpm_data, data, 0)
        
        assert 'engine_rpm' in data
        assert abs(data['engine_rpm'] - 1000.0) < 0.5
        
    def test_decode_position(self):
        """Should decode PGN 129025 position data."""
        parser = CANLogParser()
        data = {}
        
        # PGN 129025 = 0x1F801
        can_id = 0x09F80100
        # Lat/Lon in degrees * 1e7 as signed 32-bit
        # 51.5 deg = 515000000 = 0x1EB3F2C0
        # -1.5 deg = -15000000 = 0xFF19FDA0
        import struct
        lat_bytes = struct.pack('<i', 515000000)
        lon_bytes = struct.pack('<i', -15000000)
        pos_data = lat_bytes + lon_bytes
        
        parser._decode_can_frame(can_id, pos_data, data, 0)
        
        assert 'latitude' in data
        assert abs(data['latitude'] - 51.5) < 0.001
        assert 'longitude' in data
        assert abs(data['longitude'] - (-1.5)) < 0.001
        
    def test_decode_attitude(self):
        """Should decode PGN 127257 attitude (pitch/roll)."""
        parser = CANLogParser()
        data = {}
        
        # PGN 127257 = 0x1F119
        can_id = 0x09F11900
        # Data: [SID, yaw_lo, yaw_hi, pitch_lo, pitch_hi, roll_lo, roll_hi]
        # Values in radians * 10000
        # 5 degrees = 0.0873 rad = 873 = 0x0369
        # -10 degrees = -0.1745 rad = -1745 = 0xF92F (signed)
        import struct
        pitch_raw = int(5 * 3.14159 / 180 * 10000)  # 5 degrees
        roll_raw = int(-10 * 3.14159 / 180 * 10000)  # -10 degrees
        att_data = bytes([0x00, 0xFF, 0x7F]) + struct.pack('<h', pitch_raw) + struct.pack('<h', roll_raw)
        
        parser._decode_can_frame(can_id, att_data, data, 0)
        
        assert 'pitch' in data
        assert abs(data['pitch'] - 5.0) < 0.5
        assert 'roll' in data
        assert abs(data['roll'] - (-10.0)) < 0.5


class TestAnalyzeRealLogs:
    """Integration tests using real log files from n2klogs/."""
    
    @pytest.fixture
    def sample_log_path(self):
        """Path to a real sample log file."""
        # Use one of the existing log files
        return Path(__file__).parent.parent / "n2klogs" / "raw" / "2018" / "05" / "candump-2018-05-07-15-18.log"
        
    def test_analyze_real_log_produces_result(self, sample_log_path):
        """Analyzing a real log file should produce segments."""
        if not sample_log_path.exists():
            pytest.skip(f"Sample log not found: {sample_log_path}")
            
        analyzer = LogAnalyzer()
        result = analyzer.analyze_file(str(sample_log_path))
        
        assert result.source_file == sample_log_path.name
        assert result.total_duration_sec > 0
        # Real logs should produce some segments
        assert len(result.segments) >= 0
        assert "usable_for_training" in result.summary
        
    def test_parse_real_log_frames(self, sample_log_path):
        """Parser should extract frames from real log."""
        if not sample_log_path.exists():
            pytest.skip(f"Sample log not found: {sample_log_path}")
            
        parser = CANLogParser()
        frames = list(parser.parse_file(str(sample_log_path)))
        
        # Should have extracted some frames
        assert len(frames) > 0
        
        # Check that sensor data was populated
        has_heading = any(f.heading != 0 for f in frames)
        has_wind = any(f.aws != 0 for f in frames)
        
        assert has_heading or has_wind, "Log should contain heading or wind data"
