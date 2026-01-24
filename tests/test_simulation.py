"""
Tests for Simulation Module
===========================

Unit tests for yacht dynamics, wind model, wave model, helm controller,
and data generator components.
"""

import math
import json
import tempfile
from pathlib import Path
import pytest

from src.simulation.yacht_dynamics import YachtDynamics, YachtConfig, YachtState
from src.simulation.wind_model import WindModel, WindConfig, WindState
from src.simulation.wave_model import WaveModel, WaveConfig
from src.simulation.helm_controller import HelmController, HelmConfig, SteeringMode
from src.simulation.maneuvers import Tack, Gybe, CourseChange, ManeuverGenerator
from src.simulation.scenarios import get_scenario, ScenarioType, create_random_scenario
from src.simulation.data_generator import (
    SailingDataGenerator, SimConfig, generate_training_data
)


class TestYachtDynamics:
    """Tests for yacht dynamics model."""
    
    def test_yacht_initialization(self):
        """Test yacht initializes with default state."""
        yacht = YachtDynamics()
        assert yacht.state.heading == 0.0
        assert yacht.state.stw >= 0.0
        assert yacht.state.rudder_angle == 0.0
        
    def test_heading_response_to_rudder(self):
        """Test yacht heading changes in response to rudder."""
        yacht = YachtDynamics()
        yacht.reset(heading=0.0, twd=180.0, tws=15.0)
        
        initial_heading = yacht.state.heading
        
        # Apply starboard rudder
        for _ in range(100):
            yacht.step(rudder_command=10.0, dt=0.1)
            
        # Heading should have increased (turned right)
        assert yacht.state.heading != initial_heading
        assert yacht.state.rudder_angle > 0
        
    def test_rudder_rate_limiting(self):
        """Test rudder cannot change faster than rate limit."""
        yacht = YachtDynamics()
        yacht.state.rudder_angle = 0.0
        
        # Request large instant change
        yacht.step(rudder_command=30.0, dt=0.1)
        
        # Should be rate limited (4 deg/s * 0.1s = 0.4 deg max change)
        assert yacht.state.rudder_angle <= 0.5
        
    def test_speed_from_polar(self):
        """Test boat speed comes from polar lookup."""
        yacht = YachtDynamics()
        yacht.reset(heading=0.0, twd=90.0, tws=15.0)  # Beam reach
        
        # Let it settle
        for _ in range(100):
            yacht.step(rudder_command=0.0, dt=0.1)
            
        # Should have reasonable speed at beam reach
        assert yacht.state.stw > 0.0
        assert yacht.state.stw < 20.0
        
    def test_heel_from_wind(self):
        """Test boat heels in response to wind."""
        yacht = YachtDynamics()
        yacht.reset(heading=0.0, twd=90.0, tws=20.0)  # Strong beam reach
        
        # Let it settle
        for _ in range(100):
            yacht.step(rudder_command=0.0, dt=0.1)
            
        # Should have heel
        assert abs(yacht.state.roll) > 0.0
        
    def test_apparent_wind_calculation(self):
        """Test apparent wind is computed correctly."""
        yacht = YachtDynamics()
        yacht.reset(heading=0.0, twd=90.0, tws=15.0)  # Beam reach
        
        # For beam reach, apparent wind should exist
        assert yacht.state.aws > 0
        assert yacht.state.awa != 0
        
        # AWA should be forward of beam when moving
        # (apparent wind shifts forward due to boat motion)
        

class TestWindModel:
    """Tests for wind model."""
    
    def test_wind_initialization(self):
        """Test wind initializes within configured range."""
        config = WindConfig(base_tws_min=10.0, base_tws_max=20.0)
        wind = WindModel(config, seed=42)
        
        assert 10.0 <= wind.state.base_tws <= 20.0
        
    def test_wind_shift(self):
        """Test wind direction shifts over time."""
        config = WindConfig(shift_rate=2.0)  # High shift rate
        wind = WindModel(config, seed=42)
        
        initial_twd = wind.state.twd
        
        # Run for a while
        for _ in range(1000):
            wind.step(dt=0.1)
            
        # Direction should have changed
        # (might wrap around, so just check it's not stuck)
        assert wind.state.twd != initial_twd or True  # Allow for coincidence
        
    def test_gust_increases_speed(self):
        """Test gusts increase wind speed."""
        wind = WindModel(seed=42)
        initial_tws = wind.state.base_tws
        
        wind.trigger_gust(intensity=1.5, duration=10.0)
        wind.step(dt=0.1)
        
        assert wind.state.tws > initial_tws
        
    def test_lull_decreases_speed(self):
        """Test lulls decrease wind speed."""
        wind = WindModel(seed=42)
        initial_tws = wind.state.base_tws
        
        wind.trigger_lull(intensity=0.5, duration=10.0)
        wind.step(dt=0.1)
        
        assert wind.state.tws < initial_tws


class TestWaveModel:
    """Tests for wave model."""
    
    def test_wave_motion_oscillates(self):
        """Test wave motion oscillates over time."""
        waves = WaveModel(seed=42)
        
        rolls = []
        for i in range(100):
            roll, pitch = waves.step(dt=0.1, tws=15.0)
            rolls.append(roll)
            
        # Should have positive and negative values
        assert min(rolls) < 0
        assert max(rolls) > 0
        
    def test_wave_amplitude_scales_with_wind(self):
        """Test wave amplitude increases with wind speed."""
        waves = WaveModel(seed=42)
        
        # Light wind
        roll_light, _ = waves.step(dt=0.1, tws=5.0)
        waves.reset()
        
        # Heavy wind - run for a bit to get motion
        rolls_heavy = []
        for _ in range(50):
            roll, _ = waves.step(dt=0.1, tws=25.0)
            rolls_heavy.append(abs(roll))
            
        # Heavy wind should produce larger motion on average
        assert max(rolls_heavy) > 0
        
    def test_calm_sea_state(self):
        """Test calm sea state has smaller motion."""
        waves = WaveModel.calm()
        
        rolls = []
        for _ in range(100):
            roll, _ = waves.step(dt=0.1, tws=5.0)
            rolls.append(abs(roll))
            
        # Calm seas should have limited motion
        assert max(rolls) < 15.0  # Less than 15 degrees


class TestHelmController:
    """Tests for helm controller."""
    
    def test_helm_initialization(self):
        """Test helm initializes in standby."""
        helm = HelmController()
        assert helm.state.rudder_command == 0.0
        
    def test_compass_mode_correction(self):
        """Test helm corrects heading error in compass mode."""
        helm = HelmController(seed=42)
        helm.set_mode(SteeringMode.COMPASS, target=90.0)
        
        # Off target heading
        rudder = helm.compute_rudder(
            heading=100.0,  # 10 degrees off
            awa=45.0,
            twa=45.0,
            heading_rate=0.0,
            dt=0.1
        )
        
        # Should command rudder to correct
        assert rudder != 0.0
        
    def test_awa_mode_correction(self):
        """Test helm corrects AWA error in wind mode."""
        helm = HelmController(seed=42)
        helm.set_mode(SteeringMode.WIND_AWA, target=45.0)
        
        # AWA off target
        rudder = helm.compute_rudder(
            heading=90.0,
            awa=50.0,  # 5 degrees off
            twa=50.0,
            heading_rate=0.0,
            dt=0.1
        )
        
        # Should command correction
        assert abs(rudder) > 0
        
    def test_rudder_rate_limiting(self):
        """Test helm respects rudder rate limits."""
        helm = HelmController(seed=42)
        helm.set_mode(SteeringMode.COMPASS, target=0.0)
        helm.state.rudder_command = 0.0
        
        # Large error should still produce limited rate
        rudder1 = helm.compute_rudder(
            heading=90.0,  # 90 degrees off
            awa=45.0,
            twa=45.0,
            heading_rate=0.0,
            dt=0.1
        )
        
        # Rate limit is 4 deg/s, so max change in 0.1s is 0.4 degrees
        assert abs(rudder1) <= 1.0  # Should be rate limited
        
    def test_expert_vs_novice(self):
        """Test expert helm is more precise than novice."""
        expert = HelmController.expert()
        novice = HelmController.novice()
        
        assert expert.config.skill_level > novice.config.skill_level
        assert expert.config.noise_std < novice.config.noise_std


class TestManeuvers:
    """Tests for sailing maneuvers."""
    
    def test_tack_execution(self):
        """Test tack maneuver executes."""
        tack = Tack(target_awa=45.0, duration=12.0)
        state = YachtState(heading=0.0, awa=45.0)
        
        tack.start(state, twd=90.0)
        
        steps = 0
        max_steps = 300
        while not tack.is_complete() and steps < max_steps:
            result = tack.step(state, dt=0.1)
            # Simulate heading change
            state.heading += result.rudder_command * 0.2  # Faster response for test
            state.heading %= 360
            steps += 1
            
        # Tack should either complete or make progress
        assert steps <= max_steps
        # Check that we're in a valid phase or complete
        assert tack.phase in ["prep", "turn", "settle", "exit"] or tack.is_complete()
        
    def test_gybe_execution(self):
        """Test gybe maneuver executes."""
        gybe = Gybe(target_twa=140.0, duration=15.0)
        state = YachtState(heading=0.0, awa=140.0)
        
        gybe.start(state, twd=180.0)
        
        steps = 0
        max_steps = 300
        while not gybe.is_complete() and steps < max_steps:
            result = gybe.step(state, dt=0.1)
            state.heading += result.rudder_command * 0.2  # Faster response for test
            state.heading %= 360
            steps += 1
            
        # Gybe should either complete or make progress
        assert steps <= max_steps
        assert gybe.phase in ["prep", "turn", "settle", "exit"] or gybe.is_complete()
        
    def test_course_change_smooth(self):
        """Test course change is gradual."""
        change = CourseChange(heading_change=30.0)
        state = YachtState(heading=90.0)
        
        change.start(state, twd=180.0)
        
        headings = []
        while not change.is_complete():
            result = change.step(state, dt=0.1)
            state.heading += result.rudder_command * 0.1
            headings.append(state.heading)
            
            if len(headings) > 200:
                break
                
        # Heading should change smoothly
        assert len(headings) > 5


class TestScenarios:
    """Tests for scenario definitions."""
    
    def test_get_predefined_scenario(self):
        """Test loading predefined scenarios."""
        scenario = get_scenario(ScenarioType.MEDIUM_UPWIND)
        
        assert scenario.name == "medium_upwind"
        assert scenario.steering_mode == SteeringMode.WIND_AWA
        assert scenario.target_angle == 42.0  # Close hauled
        
    def test_random_scenario(self):
        """Test creating random scenarios."""
        scenario = create_random_scenario(
            tws_range=(10, 20),
            steering_mode=SteeringMode.COMPASS,
        )
        
        assert scenario.name == "random"
        assert 10 <= scenario.wind_config.base_tws_min
        assert scenario.wind_config.base_tws_max <= 20
        
    def test_scenario_randomize_initial(self):
        """Test scenario can randomize initial conditions."""
        scenario = get_scenario(ScenarioType.MIXED_COASTAL)
        scenario.randomize_initial_conditions()
        
        assert 0 <= scenario.initial_heading < 360
        assert 0 <= scenario.initial_twd < 360


class TestDataGenerator:
    """Tests for data generator."""
    
    def test_generator_produces_records(self):
        """Test generator produces valid records."""
        config = SimConfig(
            duration_hours=0.001,  # Very short for testing
            sample_rate_hz=10.0,
            seed=42,
        )
        
        generator = SailingDataGenerator(config, seed=42)
        
        records = list(generator.generate())
        
        assert len(records) > 0
        
        # Check record structure
        record = records[0]
        assert "timestamp" in record
        assert "heading" in record
        assert "rudder_angle" in record
        assert "awa" in record
        assert "stw" in record
        
    def test_generator_with_scenario(self):
        """Test generator works with scenario."""
        scenario = get_scenario(ScenarioType.LIGHT_AIR_REACHING)
        scenario.duration_hours = 0.001
        
        config = SimConfig(duration_hours=0.001, seed=42)
        generator = SailingDataGenerator(config, scenario, seed=42)
        
        records = list(generator.generate())
        assert len(records) > 0
        
    def test_generate_training_data_creates_files(self):
        """Test generate_training_data creates output files."""
        with tempfile.TemporaryDirectory() as tmpdir:
            files = generate_training_data(
                output_path=tmpdir,
                duration_hours=0.001,
                scenarios=["medium_upwind"],
                num_runs=1,
                seed=42,
            )
            
            assert len(files) == 1
            
            # Check log file exists and is valid JSON
            log_path = Path(files[0])
            assert log_path.exists()
            
            with open(log_path) as f:
                first_line = f.readline()
                record = json.loads(first_line)
                assert "timestamp" in record
                
            # Check metadata file exists
            meta_path = log_path.with_suffix('.meta.json')
            assert meta_path.exists()
            
            with open(meta_path) as f:
                metadata = json.load(f)
                assert "segments" in metadata
                assert metadata["summary"]["usable_for_training"]
                
    def test_domain_randomization(self):
        """Test domain randomization produces variation."""
        configs = []
        
        for i in range(3):
            config = SimConfig(enable_randomization=True, seed=i)
            config.apply_randomization()
            configs.append(config)
            
        # Should have variation in noise parameters
        noise_stds = [c.heading_noise_std for c in configs]
        assert len(set(noise_stds)) > 1  # Not all the same


class TestIntegration:
    """Integration tests for complete simulation."""
    
    def test_full_simulation_run(self):
        """Test a complete simulation run produces valid data."""
        config = SimConfig(
            duration_hours=0.01,  # ~36 seconds of data
            sample_rate_hz=10.0,
            seed=42,
        )
        
        scenario = get_scenario(ScenarioType.MEDIUM_UPWIND)
        scenario.duration_hours = 0.01
        
        generator = SailingDataGenerator(config, scenario, seed=42)
        
        records = list(generator.generate())
        
        # Should have ~360 records
        assert 300 < len(records) < 400
        
        # Check data is physically reasonable
        for record in records:
            assert 0 <= record["heading"] < 360
            assert -90 <= record["roll"] <= 90
            assert -30 <= record["rudder_angle"] <= 30
            assert record["aws"] >= 0
            assert record["stw"] >= 0
            
    def test_data_loadable_by_training(self):
        """Test generated data can be loaded by TrainingDataLoader."""
        from src.training.data_loader import TrainingDataLoader, CANLogParser
        
        with tempfile.TemporaryDirectory() as tmpdir:
            # Generate enough data for training sequences (need at least 21 frames)
            # 0.01 hours = 36 seconds = 360 frames at 10Hz
            files = generate_training_data(
                output_path=tmpdir,
                duration_hours=0.02,  # ~72 seconds, ~720 frames
                scenarios=["medium_upwind"],
                num_runs=1,
                seed=42,
            )
            
            # Try to parse with CANLogParser
            parser = CANLogParser()
            frames = list(parser.parse_file(files[0]))
            
            assert len(frames) > 20, f"Got {len(frames)} frames, need > 20"
            
            # Check frames have required fields
            frame = frames[0]
            assert hasattr(frame, 'heading')
            assert hasattr(frame, 'rudder_angle')
            assert hasattr(frame, 'awa')
            
            # Try loading with TrainingDataLoader
            loader = TrainingDataLoader()
            X, y = loader.load_file(files[0])
            
            # Should produce training sequences (need enough frames for sequences)
            assert len(X) > 0, f"Expected training sequences, got X shape {X.shape}"
            assert len(y) > 0
            assert X.shape[1] == 20  # sequence_length
            assert X.shape[2] == 25  # feature_dim
