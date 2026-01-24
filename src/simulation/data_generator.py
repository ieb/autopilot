"""
Data Generator
==============

Main class for generating simulated sailing data.
Integrates all simulation components and provides CLI interface.
"""

import argparse
import json
import random
import math
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional, List, Iterator, Dict, Any
import logging

from .yacht_dynamics import YachtDynamics, YachtConfig, YachtState
from .wind_model import WindModel, WindConfig
from .wave_model import WaveModel, WaveConfig
from .helm_controller import HelmController, HelmConfig, SteeringMode
from .maneuvers import ManeuverGenerator, Maneuver
from .scenarios import Scenario, ScenarioType, get_scenario, create_random_scenario
from .calibration import calibrate_from_logs, CalibratedConfig

logger = logging.getLogger(__name__)


@dataclass
class SimConfig:
    """Configuration for data generation."""
    # Duration
    duration_hours: float = 4.0
    sample_rate_hz: float = 10.0
    
    # Domain randomization
    enable_randomization: bool = True
    yacht_variation: float = 0.2       # ±20% on yacht parameters
    polar_variation: float = 0.1       # ±10% on polar performance
    helm_skill_range: tuple = (0.7, 1.0)
    noise_variation: float = 0.3       # ±30% on sensor noise
    
    # Sensor noise
    heading_noise_std: float = 0.5
    awa_noise_std: float = 2.0
    aws_noise_std: float = 0.3
    stw_noise_std: float = 0.1
    rudder_noise_std: float = 0.3
    roll_noise_std: float = 0.5
    pitch_noise_std: float = 0.3
    
    # Random seed
    seed: Optional[int] = None
    
    def apply_randomization(self):
        """Apply random variation to noise parameters."""
        if not self.enable_randomization:
            return
            
        def vary(value: float) -> float:
            factor = 1.0 + random.uniform(-self.noise_variation, self.noise_variation)
            return value * factor
            
        self.heading_noise_std = vary(self.heading_noise_std)
        self.awa_noise_std = vary(self.awa_noise_std)
        self.aws_noise_std = vary(self.aws_noise_std)
        self.stw_noise_std = vary(self.stw_noise_std)
        self.rudder_noise_std = vary(self.rudder_noise_std)


@dataclass
class SimState:
    """Complete simulation state for output."""
    timestamp: float = 0.0
    
    # From yacht dynamics
    heading: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw_rate: float = 0.0
    rudder_angle: float = 0.0
    stw: float = 0.0
    sog: float = 0.0
    cog: float = 0.0
    latitude: float = 60.0
    longitude: float = 22.0
    
    # From wind model
    awa: float = 0.0
    aws: float = 0.0
    twd: float = 0.0
    tws: float = 0.0
    
    # Mode and target
    mode: str = "compass"
    target_heading: float = 0.0


class SailingDataGenerator:
    """
    Main sailing data generator.
    
    Integrates yacht dynamics, wind model, wave model, and helm controller
    to produce realistic sailing data for ML training.
    """
    
    def __init__(self, 
                 config: Optional[SimConfig] = None,
                 scenario: Optional[Scenario] = None,
                 seed: Optional[int] = None):
        """
        Initialize data generator.
        
        Args:
            config: Simulation configuration
            scenario: Sailing scenario (overrides some config)
            seed: Random seed for reproducibility
        """
        self.config = config or SimConfig()
        self.scenario = scenario
        
        # Set seed
        if seed is not None:
            self.config.seed = seed
            random.seed(seed)
        elif self.config.seed is not None:
            random.seed(self.config.seed)
            
        # Apply domain randomization
        if self.config.enable_randomization:
            self.config.apply_randomization()
            
        # Initialize components
        self._init_components()
        
        # Simulation state
        self.elapsed_time = 0.0
        self.frame_count = 0
        
    def _init_components(self):
        """Initialize all simulation components."""
        # Use scenario configs if provided
        if self.scenario:
            wind_config = self.scenario.wind_config
            wave_config = self.scenario.wave_config
            yacht_config = self.scenario.yacht_config
            helm_config = self.scenario.helm_config
        else:
            wind_config = WindConfig()
            wave_config = WaveConfig()
            yacht_config = YachtConfig()
            helm_config = HelmConfig()
            
        # Apply variation to yacht
        variation = self.config.yacht_variation if self.config.enable_randomization else 0.0
        
        self.yacht = YachtDynamics(yacht_config, variation=variation)
        self.wind = WindModel(wind_config, seed=self.config.seed)
        self.waves = WaveModel(wave_config, seed=self.config.seed)
        self.helm = HelmController(helm_config, seed=self.config.seed)
        self.maneuvers = ManeuverGenerator()
        
        # Set initial conditions
        self._set_initial_conditions()
        
    def _set_initial_conditions(self):
        """Set initial yacht and wind conditions."""
        if self.scenario:
            twd = self.scenario.initial_twd
            tws = self.scenario.initial_tws
            
            # Set steering mode
            self.helm.set_mode(self.scenario.steering_mode, self.scenario.target_angle)
            
            # Calculate initial heading based on mode and target
            if self.scenario.steering_mode == SteeringMode.WIND_AWA:
                # Heading = TWD - target AWA
                heading = (twd - self.scenario.target_angle) % 360
            elif self.scenario.steering_mode == SteeringMode.WIND_TWA:
                # Heading = TWD - target TWA
                heading = (twd - self.scenario.target_angle) % 360
            else:
                heading = self.scenario.initial_heading
                self.helm.set_mode(SteeringMode.COMPASS, heading)
        else:
            # Random initial conditions
            twd = random.uniform(0, 360)
            tws = random.uniform(10, 20)
            heading = random.uniform(0, 360)
            self.helm.set_mode(SteeringMode.COMPASS, heading)
            
        self.yacht.reset(heading=heading, twd=twd, tws=tws)
        self.wind.set_wind(twd, tws)
        
    def generate(self) -> Iterator[Dict[str, Any]]:
        """
        Generate simulation data.
        
        Yields:
            Dictionary records compatible with CANLogParser JSON format
        """
        dt = 1.0 / self.config.sample_rate_hz
        total_samples = int(self.config.duration_hours * 3600 * self.config.sample_rate_hz)
        
        for i in range(total_samples):
            self.elapsed_time = i * dt
            self.frame_count = i
            
            # Step wind model
            wind_state = self.wind.step(dt)
            self.yacht.set_wind(wind_state.twd, wind_state.tws)
            
            # Step wave model
            wave_roll, wave_pitch = self.waves.step(
                dt, 
                tws=wind_state.tws,
                heading=self.yacht.state.heading
            )
            
            # Compute current TWA for maneuver decisions
            twa = self._compute_twa()
            
            # Check for maneuvers
            if self.scenario and self.scenario.enable_maneuvers:
                self.maneuvers.tack_probability = self.scenario.tack_probability
                self.maneuvers.gybe_probability = self.scenario.gybe_probability
                
            maneuver = self.maneuvers.check_maneuver(
                self.yacht.state, 
                wind_state.twd,
                self.elapsed_time, 
                dt
            )
            
            # Compute rudder command
            if maneuver and not maneuver.is_complete():
                # Let maneuver control the rudder
                result = maneuver.step(self.yacht.state, dt)
                rudder_command = result.rudder_command
            else:
                # Normal helm control
                rudder_command = self.helm.compute_rudder(
                    heading=self.yacht.state.heading,
                    awa=self.yacht.state.awa,
                    twa=twa,
                    heading_rate=self.yacht.state.heading_rate,
                    dt=dt,
                )
                
            # Step yacht dynamics
            self.yacht.step(rudder_command, dt)
            
            # Combine wave motion with heel
            total_roll = self.yacht.state.roll + wave_roll
            total_pitch = self.yacht.state.pitch + wave_pitch
            
            # Create output record with noise
            record = self._create_record(
                total_roll=total_roll,
                total_pitch=total_pitch,
            )
            
            yield record
            
            # Periodically change conditions for mixed scenarios
            if self.scenario and self.scenario.name == "mixed_coastal":
                self._maybe_change_conditions(i, dt)
                
    def _compute_twa(self) -> float:
        """Compute true wind angle."""
        twa = self.yacht.state.twd - self.yacht.state.heading
        while twa > 180:
            twa -= 360
        while twa < -180:
            twa += 360
        return twa
        
    def _create_record(self, total_roll: float, total_pitch: float) -> Dict[str, Any]:
        """Create output record with sensor noise."""
        cfg = self.config
        state = self.yacht.state
        
        # Get target based on mode
        target = self.helm.get_target()
        mode_str = self.helm.state.mode.value
        
        return {
            "timestamp": self.elapsed_time,
            "heading": state.heading + random.gauss(0, cfg.heading_noise_std),
            "pitch": total_pitch + random.gauss(0, cfg.pitch_noise_std),
            "roll": total_roll + random.gauss(0, cfg.roll_noise_std),
            "yaw_rate": state.heading_rate + random.gauss(0, 0.1),
            "awa": state.awa + random.gauss(0, cfg.awa_noise_std),
            "aws": max(0, state.aws + random.gauss(0, cfg.aws_noise_std)),
            "stw": max(0, state.stw + random.gauss(0, cfg.stw_noise_std)),
            "cog": state.cog + random.gauss(0, 1.0),
            "sog": max(0, state.sog + random.gauss(0, cfg.stw_noise_std)),
            "rudder_angle": state.rudder_angle + random.gauss(0, cfg.rudder_noise_std),
            "target_heading": target,
            "mode": mode_str,
            "latitude": state.latitude,
            "longitude": state.longitude,
        }
        
    def _maybe_change_conditions(self, frame_idx: int, dt: float):
        """Randomly change conditions for mixed scenarios."""
        # Change mode every ~10 minutes on average
        if frame_idx > 0 and random.random() < 0.0001:
            modes = [SteeringMode.COMPASS, SteeringMode.WIND_AWA, SteeringMode.WIND_TWA]
            new_mode = random.choice(modes)
            
            if new_mode == SteeringMode.COMPASS:
                target = self.yacht.state.heading
            elif new_mode == SteeringMode.WIND_AWA:
                target = self.yacht.state.awa
            else:
                target = self._compute_twa()
                
            self.helm.set_mode(new_mode, target)


def generate_training_data(
    output_path: str,
    duration_hours: float = 10.0,
    scenarios: Optional[List[str]] = None,
    num_runs: int = 1,
    randomize: bool = True,
    calibrate_from: Optional[str] = None,
    seed: int = 42,
) -> List[str]:
    """
    Generate training data files.
    
    Args:
        output_path: Directory for output files
        duration_hours: Total hours of data per run
        scenarios: List of scenario names to use
        num_runs: Number of separate runs (for domain randomization)
        randomize: Enable domain randomization
        calibrate_from: Path to real logs for calibration
        seed: Base random seed
        
    Returns:
        List of generated file paths
    """
    output_dir = Path(output_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Calibrate from real data if provided
    calibrated_config = None
    if calibrate_from:
        calibrated_config = calibrate_from_logs(calibrate_from)
        logger.info("Using calibrated parameters from real data")
        
    # Default scenarios
    if scenarios is None:
        scenarios = ["medium_upwind", "downwind_vmg", "mixed_coastal", "light_air_reaching"]
        
    generated_files = []
    
    for run_idx in range(num_runs):
        run_seed = seed + run_idx * 1000
        random.seed(run_seed)
        
        # Distribute time across scenarios
        hours_per_scenario = duration_hours / len(scenarios)
        
        # Create run-specific output file
        if num_runs > 1:
            log_file = output_dir / f"simulated_run{run_idx:02d}.jsonlog"
            meta_file = output_dir / f"simulated_run{run_idx:02d}.meta.json"
        else:
            log_file = output_dir / "simulated_training.jsonlog"
            meta_file = output_dir / "simulated_training.meta.json"
            
        segments = []
        current_time = 0.0
        
        with open(log_file, 'w') as f:
            for scenario_idx, scenario_name in enumerate(scenarios):
                # Get scenario
                try:
                    scenario_type = ScenarioType(scenario_name)
                    scenario = get_scenario(scenario_type)
                except ValueError:
                    # Try as custom/random
                    scenario = create_random_scenario(duration_hours=hours_per_scenario)
                    
                scenario.duration_hours = hours_per_scenario
                scenario.randomize_initial_conditions()
                
                # Apply calibration if available
                if calibrated_config:
                    scenario.wind_config = calibrated_config.wind_config
                    scenario.helm_config = calibrated_config.helm_config
                    
                # Configure generator
                config = SimConfig(
                    duration_hours=hours_per_scenario,
                    enable_randomization=randomize,
                    seed=run_seed + scenario_idx,
                )
                
                generator = SailingDataGenerator(config, scenario, seed=run_seed + scenario_idx)
                
                segment_start = current_time
                
                # Generate data
                for record in generator.generate():
                    record["timestamp"] = current_time + record["timestamp"]
                    f.write(json.dumps(record) + "\n")
                    
                segment_end = current_time + hours_per_scenario * 3600
                
                # Map steering mode to metadata format (awa, twa, heading)
                mode_value = scenario.steering_mode.value
                if mode_value == "wind_awa":
                    mode_value = "awa"
                elif mode_value == "wind_twa":
                    mode_value = "twa"
                
                # Create metadata segment
                segments.append({
                    "start_time": segment_start,
                    "end_time": segment_end,
                    "duration_sec": segment_end - segment_start,
                    "operation_mode": scenario.operation_mode,
                    "steering_mode": mode_value,
                    "target_value": scenario.target_angle,
                    "confidence": 0.95,
                    "notes": f"Simulated {scenario.name}",
                    "usable_for_training": True,
                    "missing_features": [],
                })
                
                current_time = segment_end
                logger.info(f"Generated {hours_per_scenario:.1f}h of {scenario.name}")
                
        # Write metadata
        metadata = {
            "source_file": log_file.name,
            "analyzed_at": "simulated",
            "total_duration_sec": duration_hours * 3600,
            "total_duration_hours": duration_hours,
            "total_frame_count": int(duration_hours * 3600 * 10),
            "usable_duration_sec": duration_hours * 3600,
            "usable_duration_hours": duration_hours,
            "usable_frame_count": int(duration_hours * 3600 * 10),
            "segments": segments,
            "summary": {
                "usable_for_training": True,
                "usable_duration_hours": duration_hours,
                "total_segment_count": len(segments),
                "usable_segment_count": len(segments),
                "missing_required_features": [],
                "required_features_available": True,
            },
            "overall_feature_coverage": {
                "heading": {"coverage_pct": 100.0, "available": True},
                "yaw_rate": {"coverage_pct": 100.0, "available": True},
                "roll": {"coverage_pct": 100.0, "available": True},
                "pitch": {"coverage_pct": 100.0, "available": True},
                "awa": {"coverage_pct": 100.0, "available": True},
                "aws": {"coverage_pct": 100.0, "available": True},
                "stw": {"coverage_pct": 100.0, "available": True},
                "sog": {"coverage_pct": 100.0, "available": True},
                "cog": {"coverage_pct": 100.0, "available": True},
                "rudder_angle": {"coverage_pct": 100.0, "available": True},
            },
        }
        
        with open(meta_file, 'w') as f:
            json.dump(metadata, f, indent=2)
            
        generated_files.append(str(log_file))
        logger.info(f"Generated run {run_idx+1}/{num_runs}: {log_file}")
        
    # Print summary
    total_hours = duration_hours * num_runs
    print(f"\nGenerated {total_hours:.1f} hours of simulated training data:")
    for path in generated_files:
        print(f"  {path}")
    print(f"  Metadata files created alongside log files")
    
    return generated_files


def main():
    """CLI entry point for data generator."""
    parser = argparse.ArgumentParser(
        description="Generate simulated sailing data for autopilot training"
    )
    parser.add_argument(
        "--output", "-o",
        type=str,
        default="data/simulated",
        help="Output directory for generated files"
    )
    parser.add_argument(
        "--hours", "-t",
        type=float,
        default=10.0,
        help="Hours of data to generate per run"
    )
    parser.add_argument(
        "--scenarios", "-s",
        type=str,
        default=None,
        help="Comma-separated list of scenarios (e.g., 'medium_upwind,downwind_vmg')"
    )
    parser.add_argument(
        "--num-runs", "-n",
        type=int,
        default=1,
        help="Number of separate runs (for domain randomization)"
    )
    parser.add_argument(
        "--randomize", "-r",
        action="store_true",
        default=True,
        help="Enable domain randomization"
    )
    parser.add_argument(
        "--no-randomize",
        action="store_true",
        help="Disable domain randomization"
    )
    parser.add_argument(
        "--calibrate-from", "-c",
        type=str,
        default=None,
        help="Path to real log files for calibration"
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for reproducibility"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Verbose output"
    )
    
    args = parser.parse_args()
    
    # Setup logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=log_level, format='%(levelname)s: %(message)s')
    
    # Parse scenarios
    scenarios = None
    if args.scenarios:
        scenarios = [s.strip() for s in args.scenarios.split(',')]
        
    # Handle randomize flags
    randomize = args.randomize and not args.no_randomize
    
    # Generate data
    generate_training_data(
        output_path=args.output,
        duration_hours=args.hours,
        scenarios=scenarios,
        num_runs=args.num_runs,
        randomize=randomize,
        calibrate_from=args.calibrate_from,
        seed=args.seed,
    )


if __name__ == "__main__":
    main()
