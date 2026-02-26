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
import struct
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional, List, Iterator, Dict, Any
import logging
import numpy as np

from .yacht_dynamics import YachtDynamics, YachtConfig, YachtState
from .wind_model import WindModel, WindConfig
from .wave_model import WaveModel, WaveConfig
from .helm_controller import HelmController, HelmConfig, SteeringMode
from .maneuvers import ManeuverGenerator, Maneuver
from .scenarios import Scenario, ScenarioType, get_scenario, create_random_scenario
from .calibration import calibrate_from_logs, CalibratedConfig

logger = logging.getLogger(__name__)


def _sample_multiscale_kick(max_mag: float) -> float:
    """Sample a perturbation from a multi-scale distribution.

    50% small (≤10% of max), 30% medium (10-50% of max), 20% large (50-100%).
    This ensures the model sees plenty of fine-correction examples alongside
    large error recovery, preventing the undershoot bias observed when only
    large perturbations are used.
    """
    r = random.random()
    if r < 0.50:
        mag = max_mag * random.uniform(0.02, 0.10)
    elif r < 0.80:
        mag = max_mag * random.uniform(0.10, 0.50)
    else:
        mag = max_mag * random.uniform(0.50, 1.00)
    return mag * random.choice([-1, 1])


# Fields that wrap at 360° and need angular (sin/cos) averaging
_ANGULAR_FIELDS = {'heading', 'cog', 'target_heading'}


def _average_records(records: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Average a batch of simulation records, using angular means for angles.

    Non-numeric fields (mode) are taken from the last record.
    """
    n = len(records)
    if n == 1:
        return records[0]

    out: Dict[str, Any] = {}
    # Use the last record's timestamp and categorical fields
    out['timestamp'] = records[-1]['timestamp']
    out['mode'] = records[-1]['mode']
    out['latitude'] = records[-1]['latitude']
    out['longitude'] = records[-1]['longitude']

    for key in records[0]:
        if key in out:
            continue
        if key in _ANGULAR_FIELDS:
            # Circular mean via sin/cos
            sin_sum = sum(math.sin(math.radians(r[key])) for r in records)
            cos_sum = sum(math.cos(math.radians(r[key])) for r in records)
            out[key] = math.degrees(math.atan2(sin_sum / n, cos_sum / n)) % 360
        else:
            out[key] = sum(r[key] for r in records) / n

    return out


# Binary training data format
_BIN_MAGIC = b'APFD'   # AutoPilot Frame Data
_BIN_VERSION = 1
_BIN_HEADER_SIZE = 16   # 4 magic + 4 version + 4 feature_dim + 4 reserved
_BIN_COLS = 23           # 22 features + 1 label (rudder)


class BinaryFrameWriter:
    """Writes pre-computed feature frames to a compact binary file.

    File layout:
        [header: 16 bytes]
            4B  magic   b'APFD'
            4B  version uint32 LE
            4B  feature_dim uint32 LE  (22)
            4B  reserved (0)
        [frames: N * 23 * 4 bytes]
            each frame is 23 x float32 LE
            columns 0-21 = normalised features (from compute_features)
            column  22   = normalised rudder label  (rudder / 25, clipped)

    A companion .meta.json is written on close() with frame count,
    mode-transition timestamps, and generation config.
    """

    def __init__(self, path: Path, meta_extra: Optional[Dict[str, Any]] = None):
        from src.training.data_loader import compute_features  # deferred to avoid circular
        self._compute = compute_features
        self._path = Path(path)
        self._file = open(self._path, 'wb')
        self._write_header()
        self._frame_count = 0
        self._last_mode: Optional[str] = None
        self._transition_times: List[float] = []
        self._meta_extra = meta_extra or {}

    def _write_header(self):
        self._file.write(_BIN_MAGIC)
        self._file.write(struct.pack('<III', _BIN_VERSION, _BIN_COLS - 1, 0))

    def write_record(self, record: Dict[str, Any]):
        """Write one simulation record as a binary feature frame."""
        features = self._compute(
            heading=record['heading'],
            pitch=record['pitch'],
            roll=record['roll'],
            yaw_rate=record['yaw_rate'],
            awa=record['awa'],
            aws=record['aws'],
            stw=record['stw'],
            cog=record['cog'],
            sog=record['sog'],
            rudder_angle=record['rudder_angle'],
            target_heading=record['target_heading'],
            target_awa=record['target_awa'],
            target_twa=record['target_twa'],
            mode=record['mode'],
            roll_rate=record.get('roll_rate', 0.0),
            awa_rate=record.get('awa_rate', 0.0),
            rudder_velocity=record.get('rudder_velocity', 0.0),
            wave_period=record.get('wave_period', 0.0),
        )
        label = np.clip(record['rudder_angle'] / 25.0, -1.0, 1.0)
        row = np.empty(_BIN_COLS, dtype=np.float32)
        row[:_BIN_COLS - 1] = features
        row[_BIN_COLS - 1] = label
        self._file.write(row.tobytes())
        # Track mode transitions
        mode = record['mode']
        if self._last_mode is not None and mode != self._last_mode:
            self._transition_times.append(record['timestamp'])
        self._last_mode = mode
        self._frame_count += 1

    def close(self):
        self._file.close()
        meta = {
            'format': 'binary_frame_v1',
            'frame_count': self._frame_count,
            'feature_dim': 22,
            'columns': 23,
            'transition_times': self._transition_times,
        }
        meta.update(self._meta_extra)
        meta_path = self._path.with_suffix('.meta.json')
        with open(meta_path, 'w') as f:
            json.dump(meta, f, indent=2)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


@dataclass
class SimConfig:
    """Configuration for data generation."""
    # Duration
    duration_hours: float = 4.0
    sample_rate_hz: float = 10.0
    
    # Warm-up period: skip initial transient before recording
    # This prevents training on startup data where errors are large but
    # rudder is still ramping up due to rate limiting
    warmup_seconds: float = 90.0  # 60-120 seconds recommended
    
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
    
    # Recording rate (may differ from simulation rate)
    # Physics always runs at sample_rate_hz for stability.
    # If recording_rate_hz < sample_rate_hz, consecutive frames are
    # averaged (angular-aware) before writing to disk.
    recording_rate_hz: float = 2.0

    # Heading perturbations (DAgger-lite)
    # Multi-scale: frequent small perturbations teach fine corrections,
    # occasional large perturbations teach error recovery.
    # Format: list of (weight, magnitude_deg, interval_sec) triples.
    perturb_interval_sec: float = 10.0      # mean time between perturbations
    perturb_magnitude_deg: float = 45.0     # max perturbation magnitude (legacy)
    enable_perturbations: bool = True

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
        
    def _step_simulation(self, dt: float):
        """
        Execute one simulation step without recording data.
        
        Used during warm-up period to let the system reach steady state.
        """
        # Step wind model
        wind_state = self.wind.step(dt)
        self.yacht.set_wind(wind_state.twd, wind_state.tws)
        
        # Compute TWA for wave model and helm controller
        twa = self._compute_twa()
        
        # Step wave model (includes yaw effect)
        _, _, wave_yaw = self.waves.step(
            dt, 
            tws=wind_state.tws, 
            heading=self.yacht.state.heading,
            twa=twa,
            heel=self.yacht.state.roll
        )
        
        # Compute rudder command (normal helm control, no maneuvers during warmup)
        rudder_command = self.helm.compute_rudder(
            heading=self.yacht.state.heading,
            awa=self.yacht.state.awa,
            twa=twa,
            heading_rate=self.yacht.state.heading_rate,
            dt=dt,
            aws=self.yacht.state.aws,
            stw=self.yacht.state.stw,
        )
        
        # Step yacht dynamics (wave_yaw affects heading)
        self.yacht.step(rudder_command, dt, wave_yaw=wave_yaw)
        
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
            
            # Apply initial error for error recovery scenarios
            # This intentionally starts the boat off-target so the helm
            # controller demonstrates recovery behavior
            if self.scenario.initial_error_deg != 0:
                heading = (heading + self.scenario.initial_error_deg) % 360
                logger.debug(f"Applied initial error: {self.scenario.initial_error_deg:.0f}°")
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
        
        Includes a warm-up period where the simulation runs but data is not
        recorded. This allows the system to reach steady state before training
        data is captured, avoiding artifacts from:
        - Rudder rate limiting during initial error correction
        - Large errors with small rudder commands (bad training signal)
        
        Yields:
            Dictionary records compatible with CANLogParser JSON format
        """
        dt = 1.0 / self.config.sample_rate_hz
        total_samples = int(self.config.duration_hours * 3600 * self.config.sample_rate_hz)
        
        # Determine warmup: skip if scenario says so (e.g., error_recovery)
        skip_warmup = self.scenario and getattr(self.scenario, 'skip_warmup', False)
        warmup_samples = 0 if skip_warmup else int(self.config.warmup_seconds * self.config.sample_rate_hz)
        
        # Run warm-up period (no data recorded)
        if warmup_samples > 0:
            logger.debug(f"Running {self.config.warmup_seconds:.0f}s warm-up period...")
            for i in range(warmup_samples):
                self._step_simulation(dt)
            logger.debug("Warm-up complete, starting data recording")
        elif skip_warmup:
            logger.debug("Skipping warm-up for error recovery scenario")
        
        # Perturbation state -- multi-scale kicks
        next_perturb_time = (
            self.config.perturb_interval_sec * random.uniform(0.5, 1.5)
            if self.config.enable_perturbations else float('inf')
        )

        # Rate tracking for derived features
        self._prev_awa = self.yacht.state.awa
        self._prev_rudder = self.yacht.state.rudder_angle
        self._prev_total_roll = self.yacht.state.roll

        for i in range(total_samples):
            self.elapsed_time = i * dt
            self.frame_count = i
            
            # Multi-scale heading perturbation injection
            if self.elapsed_time >= next_perturb_time:
                kick = _sample_multiscale_kick(self.config.perturb_magnitude_deg)
                self.yacht.state.heading = (self.yacht.state.heading + kick) % 360
                next_perturb_time = self.elapsed_time + (
                    self.config.perturb_interval_sec * random.uniform(0.5, 1.5))
            
            # Step wind model
            wind_state = self.wind.step(dt)
            self.yacht.set_wind(wind_state.twd, wind_state.tws)
            
            # Compute current TWA for wave model and maneuver decisions
            twa = self._compute_twa()
            
            # Step wave model (now includes yaw effect based on TWA and heel)
            wave_roll, wave_pitch, wave_yaw = self.waves.step(
                dt, 
                tws=wind_state.tws,
                heading=self.yacht.state.heading,
                twa=twa,
                heel=self.yacht.state.roll
            )
            
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
                    aws=self.yacht.state.aws,
                    stw=self.yacht.state.stw,
                )
                
            # Step yacht dynamics (wave_yaw affects heading)
            self.yacht.step(rudder_command, dt, wave_yaw=wave_yaw)
            
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
        mode_str = self.helm.state.mode.value
        dt = 1.0 / cfg.sample_rate_hz

        # Compute rates from consecutive frames
        awa_rate = (state.awa - self._prev_awa) / dt
        # Handle AWA wrap-around (e.g. -179 to 179)
        if awa_rate > 180 / dt:
            awa_rate -= 360 / dt
        elif awa_rate < -180 / dt:
            awa_rate += 360 / dt

        rudder_velocity = (state.rudder_angle - self._prev_rudder) / dt
        roll_rate = (total_roll - self._prev_total_roll) / dt

        # Update previous-frame state
        self._prev_awa = state.awa
        self._prev_rudder = state.rudder_angle
        self._prev_total_roll = total_roll

        # Always store all target values separately for proper feature computation
        # target_heading is the compass heading target (used in compass mode)
        # target_awa/target_twa are the wind angle targets (used in wind modes)
        return {
            "timestamp": self.elapsed_time,
            "heading": (state.heading + random.gauss(0, cfg.heading_noise_std)) % 360,
            "pitch": total_pitch + random.gauss(0, cfg.pitch_noise_std),
            "roll": total_roll + random.gauss(0, cfg.roll_noise_std),
            "yaw_rate": state.heading_rate + random.gauss(0, 0.1),
            "awa": state.awa + random.gauss(0, cfg.awa_noise_std),
            "aws": max(0, state.aws + random.gauss(0, cfg.aws_noise_std)),
            "stw": max(0, state.stw + random.gauss(0, cfg.stw_noise_std)),
            "cog": (state.cog + random.gauss(0, 1.0)) % 360,
            "sog": max(0, state.sog + random.gauss(0, cfg.stw_noise_std)),
            "rudder_angle": state.rudder_angle + random.gauss(0, cfg.rudder_noise_std),
            "target_heading": self.helm.state.target_heading,
            "target_awa": self.helm.state.target_awa,
            "target_twa": self.helm.state.target_twa,
            "mode": mode_str,
            "latitude": state.latitude,
            "longitude": state.longitude,
            "roll_rate": roll_rate + random.gauss(0, 0.1),
            "awa_rate": awa_rate + random.gauss(0, 0.2),
            "rudder_velocity": rudder_velocity + random.gauss(0, 0.1),
            "wave_period": self.waves.state.swell_period,
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
    warmup_seconds: float = 90.0,
    enable_perturbations: bool = True,
    perturb_interval_sec: float = 30.0,
    perturb_magnitude_deg: float = 20.0,
    output_json: bool = False,
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
        warmup_seconds: Seconds to run simulation before recording (skip transient)
        enable_perturbations: Inject heading perturbations for error recovery
        perturb_interval_sec: Mean seconds between perturbations
        perturb_magnitude_deg: Max perturbation magnitude in degrees
        output_json: Write legacy .jsonlog format instead of binary .bin
        
    Returns:
        List of generated file paths
    """
    output_dir = Path(output_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    ext = '.jsonlog' if output_json else '.bin'

    # Calibrate from real data if provided
    calibrated_config = None
    if calibrate_from:
        calibrated_config = calibrate_from_logs(calibrate_from)
        logger.info("Using calibrated parameters from real data")
        
    # Default scenarios — matches run 26-4 which achieved 2/3 CL pass.
    # 3 wind + 3 compass + 3 error recovery (random mode each).
    if scenarios is None:
        scenarios = [
            "medium_upwind", "downwind_vmg", "light_air_reaching",  # wind modes
            "calm_compass", "motoring", "mixed_coastal",            # compass modes
            "error_recovery", "error_recovery", "error_recovery",   # large-error training
        ]
        
    generated_files = []
    
    for run_idx in range(num_runs):
        run_seed = seed + run_idx * 1000
        random.seed(run_seed)
        
        # Distribute time across scenarios
        hours_per_scenario = duration_hours / len(scenarios)
        
        # Create run-specific output file
        if num_runs > 1:
            log_file = output_dir / f"simulated_run{run_idx:02d}{ext}"
            meta_file = output_dir / f"simulated_run{run_idx:02d}.meta.json"
        else:
            log_file = output_dir / f"simulated_training{ext}"
            meta_file = output_dir / f"simulated_training.meta.json"
            
        segments = []
        current_time = 0.0

        writer = None
        json_fh = None
        if output_json:
            json_fh = open(log_file, 'w')
        else:
            writer = BinaryFrameWriter(log_file)

        try:
            for scenario_idx, scenario_name in enumerate(scenarios):
                # Get scenario
                try:
                    scenario_type = ScenarioType(scenario_name)
                    scenario = get_scenario(scenario_type)
                except ValueError:
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
                    warmup_seconds=warmup_seconds,
                    seed=run_seed + scenario_idx,
                    enable_perturbations=enable_perturbations,
                    perturb_interval_sec=perturb_interval_sec,
                    perturb_magnitude_deg=perturb_magnitude_deg,
                )
                
                generator = SailingDataGenerator(config, scenario, seed=run_seed + scenario_idx)
                
                segment_start = current_time
                
                # Decimation: physics runs at sample_rate_hz, recording at
                # recording_rate_hz.  Buffer N frames and write their average.
                decimate = max(1, round(config.sample_rate_hz
                                        / config.recording_rate_hz))
                buf: List[Dict[str, Any]] = []

                # Generate data
                for record in generator.generate():
                    record["timestamp"] = current_time + record["timestamp"]
                    buf.append(record)
                    if len(buf) < decimate:
                        continue
                    averaged = _average_records(buf)
                    buf.clear()
                    if output_json:
                        json_fh.write(json.dumps(averaged) + "\n")
                    else:
                        writer.write_record(averaged)
                    
                segment_end = current_time + hours_per_scenario * 3600
                
                # Map steering mode to metadata format (awa, twa, heading)
                mode_value = scenario.steering_mode.value
                if mode_value == "wind_awa":
                    mode_value = "awa"
                elif mode_value == "wind_twa":
                    mode_value = "twa"
                
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

        finally:
            if writer:
                writer.close()
            if json_fh:
                json_fh.close()

        # Write metadata (JSON path writes its own; binary meta is in writer.close)
        if output_json:
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


def generate_grib_training_data(
    route_files: List[str],
    output_path: str,
    grib_dir: Optional[str] = None,
    num_runs: int = 1,
    randomize: bool = True,
    seed: int = 42,
    warmup_seconds: float = 90.0,
    sample_rate_hz: float = 10.0,
    sample_window_hours: Optional[float] = None,
    sample_interval_hours: Optional[float] = None,
    enable_perturbations: bool = True,
    perturb_interval_sec: float = 30.0,
    perturb_magnitude_deg: float = 20.0,
    output_json: bool = False,
) -> List[str]:
    """
    Generate training data driven by route files and GRIB weather.
    
    Uses real wind/wave conditions from GRIB files (or route predictions)
    along a planned passage, producing binary .bin (default) or .jsonlog
    training data compatible with TrainingDataLoader.
    
    When sample_window_hours and sample_interval_hours are set, the
    generator produces short simulation windows at regular intervals
    along the route instead of one continuous simulation.  This keeps
    data size manageable while capturing diverse conditions.
    
    Heading perturbations (DAgger-lite) are injected by default.  These
    periodically knock the heading off course so the helm controller
    demonstrates recovery behaviour, teaching the model to correct errors
    rather than only maintain a steady course.
    
    Args:
        route_files: List of route file paths (.csv or .kml)
        output_path: Directory for output files
        grib_dir: Path to GRIB directory (optional, falls back to route wind)
        num_runs: Number of runs per route (varied by domain randomization)
        randomize: Enable domain randomization
        seed: Base random seed
        warmup_seconds: Warm-up period before recording
        sample_rate_hz: Output sample rate
        sample_window_hours: Duration of each sample window (e.g. 0.5 for 30 min)
        sample_interval_hours: Interval between window starts (e.g. 12.0)
        enable_perturbations: Inject heading perturbations for error recovery
        perturb_interval_sec: Mean seconds between perturbations
        perturb_magnitude_deg: Max perturbation magnitude in degrees
        
    Returns:
        List of generated file paths
    """
    from experiments.experiment1.route_parser import RouteParser
    from experiments.experiment1.grib_reader import GribReader
    from experiments.experiment1.wind_provider import WindProvider
    from experiments.experiment1.wave_provider import WaveProvider
    from experiments.experiment1.navigator import Navigator
    from experiments.experiment1.navigator import SteeringMode as NavSteeringMode
    from datetime import timedelta

    output_dir = Path(output_path)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Load GRIB data once (shared across all routes)
    grib_reader = None
    if grib_dir:
        grib_reader = GribReader()
        files_loaded = grib_reader.load_directory(grib_dir)
        logger.info(f"Loaded {files_loaded} GRIB files from {grib_dir}")

    nav_to_helm_mode = {
        NavSteeringMode.COMPASS: SteeringMode.COMPASS,
        NavSteeringMode.WIND_AWA: SteeringMode.WIND_AWA,
        NavSteeringMode.WIND_TWA: SteeringMode.WIND_TWA,
        NavSteeringMode.MOTORING: SteeringMode.COMPASS,
    }
    MOTORING_STW = 6.0

    generated_files = []
    file_idx = 0

    for route_path in route_files:
        route_parser = RouteParser(route_path)
        waypoints = route_parser.parse()
        if not waypoints:
            logger.warning(f"No waypoints in {route_path}, skipping")
            continue

        total_duration_hours = route_parser.planned_duration_seconds / 3600.0
        route_name = Path(route_path).stem
        logger.info(f"Route {route_name}: {len(waypoints)} waypoints, "
                     f"{total_duration_hours:.1f}h planned duration")

        # Determine simulation windows
        if sample_window_hours and sample_interval_hours:
            windows = []
            offset_h = 0.0
            while offset_h + sample_window_hours <= total_duration_hours:
                windows.append((offset_h, sample_window_hours))
                offset_h += sample_interval_hours
            if not windows:
                windows = [(0.0, min(sample_window_hours, total_duration_hours))]
            logger.info(
                f"  Sampling {len(windows)} windows of "
                f"{sample_window_hours:.1f}h every {sample_interval_hours:.1f}h "
                f"({sum(w[1] for w in windows):.1f}h total)")
        else:
            windows = [(0.0, total_duration_hours)]

        for run_idx in range(num_runs):
            for win_idx, (win_offset_h, win_duration_h) in enumerate(windows):
                run_seed = seed + file_idx * 1000 + run_idx * 100 + win_idx
                random.seed(run_seed)

                if len(windows) > 1:
                    tag = f"grib_{route_name}_run{run_idx:02d}_w{win_idx:03d}"
                else:
                    tag = f"grib_{route_name}_run{run_idx:02d}"
                grib_ext = '.jsonlog' if output_json else '.bin'
                log_file = output_dir / f"{tag}{grib_ext}"
                meta_file = output_dir / f"{tag}.meta.json"

                config = SimConfig(
                    duration_hours=win_duration_h,
                    sample_rate_hz=sample_rate_hz,
                    warmup_seconds=warmup_seconds,
                    enable_randomization=randomize,
                    seed=run_seed,
                    enable_perturbations=enable_perturbations,
                    perturb_interval_sec=perturb_interval_sec,
                    perturb_magnitude_deg=perturb_magnitude_deg,
                )
                if randomize:
                    config.apply_randomization()

                variation = config.yacht_variation if randomize else 0.0
                yacht = YachtDynamics(YachtConfig(), variation=variation)
                waves = WaveModel(WaveConfig(), seed=run_seed)

                helm_config = HelmConfig()
                if randomize:
                    helm_config.skill_level = random.uniform(*config.helm_skill_range)
                helm = HelmController(helm_config, seed=run_seed)

                wind_provider = WindProvider(grib_reader, waypoints)
                wave_provider = WaveProvider(grib_reader, waypoints)
                navigator = Navigator(waypoints)

                dt = 1.0 / sample_rate_hz
                warmup_samples = int(warmup_seconds * sample_rate_hz)
                window_samples = int(win_duration_h * 3600 * sample_rate_hz)

                # Fast-forward to window start position along the route
                win_start_time = route_parser.start_time + timedelta(
                    hours=win_offset_h)
                wp, wp_idx = route_parser.get_waypoint_at_time(win_start_time)
                if wp is None:
                    wp = waypoints[-1]
                    wp_idx = len(waypoints) - 1

                # Interpolate position within the leg
                if wp_idx < len(waypoints) - 1:
                    next_wp = waypoints[wp_idx + 1]
                    leg_dur = (next_wp.timestamp - wp.timestamp).total_seconds()
                    if leg_dur > 0:
                        frac = (win_start_time - wp.timestamp).total_seconds() / leg_dur
                        frac = max(0.0, min(1.0, frac))
                    else:
                        frac = 0.0
                    start_lat = wp.from_lat + frac * (next_wp.from_lat - wp.from_lat)
                    start_lon = wp.from_lon + frac * (next_wp.from_lon - wp.from_lon)
                else:
                    start_lat = wp.from_lat
                    start_lon = wp.from_lon

                yacht.reset(heading=wp.cog, twd=wp.twd, tws=wp.tws)
                yacht.state.latitude = start_lat
                yacht.state.longitude = start_lon
                helm.set_mode(SteeringMode.COMPASS, wp.cog)

                # Advance navigator to the correct leg
                if wp_idx > 0:
                    navigator._start_leg(wp_idx)

                current_time = win_start_time

                # Warm-up: settle dynamics without recording
                if warmup_samples > 0:
                    for _ in range(warmup_samples):
                        lat = yacht.state.latitude
                        lon = yacht.state.longitude
                        wind = wind_provider.get_wind(lat, lon, current_time)
                        yacht.set_wind(wind.twd, wind.tws)

                        twa = wind.twd - yacht.state.heading
                        while twa > 180: twa -= 360
                        while twa < -180: twa += 360

                        wave_cond = wave_provider.get_waves(
                            lat, lon, current_time, wind.tws, wind.twd)
                        waves.set_sea_state(wave_cond.mean_period,
                                            wave_cond.amplitude_degrees)
                        _, _, wave_yaw = waves.step(
                            dt, tws=wind.tws, heading=yacht.state.heading,
                            twa=twa, heel=yacht.state.roll)

                        rudder_cmd = helm.compute_rudder(
                            heading=yacht.state.heading,
                            awa=yacht.state.awa,
                            twa=twa,
                            heading_rate=yacht.state.heading_rate,
                            dt=dt,
                            aws=yacht.state.aws,
                            stw=yacht.state.stw,
                        )
                        yacht.step(rudder_cmd, dt, wave_yaw=wave_yaw)
                        current_time += timedelta(seconds=dt)

                # Perturbation state for this window
                next_perturb_time = (
                    config.perturb_interval_sec * random.uniform(0.5, 1.5)
                    if config.enable_perturbations else float('inf')
                )

                # Main recording loop
                segments = []
                current_leg = -1
                segment_start_time = 0.0
                elapsed = 0.0

                grib_writer = None
                grib_json_fh = None
                if output_json:
                    grib_json_fh = open(log_file, 'w')
                else:
                    grib_writer = BinaryFrameWriter(log_file)

                try:
                    for i in range(window_samples):
                        elapsed = i * dt

                        # Multi-scale heading perturbation injection
                        if elapsed >= next_perturb_time:
                            kick = _sample_multiscale_kick(
                                config.perturb_magnitude_deg)
                            yacht.state.heading = (
                                yacht.state.heading + kick) % 360
                            next_perturb_time = elapsed + (
                                config.perturb_interval_sec
                                * random.uniform(0.5, 1.5))

                        lat = yacht.state.latitude
                        lon = yacht.state.longitude
                        heading = yacht.state.heading
                        stw = yacht.state.stw

                        wind = wind_provider.get_wind(lat, lon, current_time)
                        yacht.set_wind(wind.twd, wind.tws)

                        twa = wind.twd - heading
                        while twa > 180: twa -= 360
                        while twa < -180: twa += 360

                        wave_cond = wave_provider.get_waves(
                            lat, lon, current_time, wind.tws, wind.twd)
                        waves.set_sea_state(wave_cond.mean_period,
                                            wave_cond.amplitude_degrees)
                        wave_roll, wave_pitch, wave_yaw = waves.step(
                            dt, tws=wind.tws, heading=heading,
                            twa=twa, heel=yacht.state.roll)

                        nav_state = navigator.update(
                            lat, lon, wind.twd, heading, stw)

                        if nav_state.arrived_at_destination:
                            logger.info(
                                f"Arrived at destination at window offset "
                                f"{elapsed/3600:.1f}h")
                            break

                        if nav_state.current_leg_index != current_leg:
                            if current_leg >= 0:
                                seg_mode = helm.state.mode.value
                                if seg_mode == "wind_awa":
                                    seg_mode = "awa"
                                elif seg_mode == "wind_twa":
                                    seg_mode = "twa"
                                segments.append({
                                    "start_time": segment_start_time,
                                    "end_time": elapsed,
                                    "duration_sec": elapsed - segment_start_time,
                                    "operation_mode": (
                                        "motoring" if nav_state.is_motoring
                                        else "sailing"),
                                    "steering_mode": seg_mode,
                                    "target_value": helm.state.target_heading,
                                    "confidence": 0.95,
                                    "notes": (
                                        f"GRIB route {route_name} "
                                        f"leg {current_leg}"),
                                    "usable_for_training": True,
                                    "missing_features": [],
                                })
                            current_leg = nav_state.current_leg_index
                            segment_start_time = elapsed

                        helm_mode = nav_to_helm_mode.get(
                            nav_state.steering_mode, SteeringMode.COMPASS)

                        if nav_state.is_motoring:
                            helm.set_mode(
                                SteeringMode.COMPASS, nav_state.target_heading)
                            yacht.state.stw = MOTORING_STW
                        elif helm_mode == SteeringMode.WIND_AWA:
                            helm.set_mode(
                                SteeringMode.WIND_AWA, nav_state.target_awa)
                        elif helm_mode == SteeringMode.WIND_TWA:
                            helm.set_mode(
                                SteeringMode.WIND_TWA, nav_state.target_twa)
                        else:
                            helm.set_mode(
                                SteeringMode.COMPASS, nav_state.target_heading)

                        rudder_cmd = helm.compute_rudder(
                            heading=heading,
                            awa=yacht.state.awa,
                            twa=twa,
                            heading_rate=yacht.state.heading_rate,
                            dt=dt,
                            aws=yacht.state.aws,
                            stw=yacht.state.stw,
                        )

                        yacht.step(rudder_cmd, dt, wave_yaw=wave_yaw)

                        total_roll = yacht.state.roll + wave_roll
                        total_pitch = yacht.state.pitch + wave_pitch

                        record = {
                            "timestamp": elapsed,
                            "heading": (yacht.state.heading + random.gauss(
                                0, config.heading_noise_std)) % 360,
                            "pitch": total_pitch + random.gauss(
                                0, config.pitch_noise_std),
                            "roll": total_roll + random.gauss(
                                0, config.roll_noise_std),
                            "yaw_rate": yacht.state.heading_rate + random.gauss(
                                0, 0.1),
                            "awa": yacht.state.awa + random.gauss(
                                0, config.awa_noise_std),
                            "aws": max(0, yacht.state.aws + random.gauss(
                                0, config.aws_noise_std)),
                            "stw": max(0, yacht.state.stw + random.gauss(
                                0, config.stw_noise_std)),
                            "cog": (yacht.state.cog + random.gauss(
                                0, 1.0)) % 360,
                            "sog": max(0, yacht.state.sog + random.gauss(
                                0, config.stw_noise_std)),
                            "rudder_angle": yacht.state.rudder_angle + random.gauss(
                                0, config.rudder_noise_std),
                            "target_heading": helm.state.target_heading,
                            "target_awa": helm.state.target_awa,
                            "target_twa": helm.state.target_twa,
                            "mode": helm.state.mode.value,
                            "latitude": lat,
                            "longitude": lon,
                        }
                        if output_json:
                            grib_json_fh.write(json.dumps(record) + "\n")
                        else:
                            grib_writer.write_record(record)

                        current_time += timedelta(seconds=dt)

                finally:
                    if grib_writer:
                        grib_writer.close()
                    if grib_json_fh:
                        grib_json_fh.close()

                # Close final segment
                if current_leg >= 0:
                        seg_mode = helm.state.mode.value
                        if seg_mode == "wind_awa":
                            seg_mode = "awa"
                        elif seg_mode == "wind_twa":
                            seg_mode = "twa"
                        segments.append({
                            "start_time": segment_start_time,
                            "end_time": elapsed,
                            "duration_sec": elapsed - segment_start_time,
                            "operation_mode": "sailing",
                            "steering_mode": seg_mode,
                            "target_value": helm.state.target_heading,
                            "confidence": 0.95,
                            "notes": f"GRIB route {route_name} leg {current_leg}",
                            "usable_for_training": True,
                            "missing_features": [],
                        })

                actual_duration = elapsed
                actual_hours = actual_duration / 3600.0

                if output_json:
                    metadata = {
                        "source_file": log_file.name,
                        "analyzed_at": "simulated-grib",
                        "total_duration_sec": actual_duration,
                        "total_duration_hours": actual_hours,
                        "total_frame_count": int(actual_duration * sample_rate_hz),
                        "usable_duration_sec": actual_duration,
                        "usable_duration_hours": actual_hours,
                        "usable_frame_count": int(actual_duration * sample_rate_hz),
                        "route_file": route_path,
                        "grib_dir": grib_dir,
                        "window_offset_hours": win_offset_h,
                        "window_duration_hours": win_duration_h,
                        "segments": segments,
                        "summary": {
                            "usable_for_training": True,
                            "usable_duration_hours": actual_hours,
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
                win_label = (f" window {win_idx} (+{win_offset_h:.1f}h)"
                             if len(windows) > 1 else "")
                logger.info(
                    f"Generated {actual_hours:.1f}h from {route_name} "
                    f"run {run_idx+1}/{num_runs}{win_label}: {log_file}")
                file_idx += 1

    total_files = len(generated_files)
    print(f"\nGenerated {total_files} GRIB-driven training data files:")
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
        "--warmup",
        type=float,
        default=90.0,
        help="Warm-up period in seconds (skip initial transient, default 90s)"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Verbose output"
    )
    
    # GRIB-driven mode
    parser.add_argument(
        "--route", "-R",
        type=str,
        nargs='+',
        default=None,
        help="Path(s) to route file(s) (.csv or .kml). Activates GRIB-driven mode"
    )
    parser.add_argument(
        "--gribs", "-g",
        type=str,
        default=None,
        help="Path to GRIB directory (used with --route, falls back to route wind)"
    )
    parser.add_argument(
        "--sample-window",
        type=float,
        default=None,
        help="Duration of each sample window in hours (e.g. 0.5 for 30 min). "
             "Used with --route to generate short representative samples"
    )
    parser.add_argument(
        "--sample-interval",
        type=float,
        default=None,
        help="Interval between sample windows in hours (e.g. 12.0). "
             "Used with --sample-window"
    )

    # Output format
    parser.add_argument(
        "--json",
        action="store_true",
        help="Write legacy .jsonlog output instead of binary .bin (default: binary)"
    )

    # Perturbation settings (DAgger-lite)
    parser.add_argument(
        "--no-perturbations",
        action="store_true",
        help="Disable heading perturbations (default: enabled)"
    )
    parser.add_argument(
        "--perturb-interval",
        type=float,
        default=15.0,
        help="Mean seconds between heading perturbations (default 15)"
    )
    parser.add_argument(
        "--perturb-magnitude",
        type=float,
        default=45.0,
        help="Max perturbation magnitude in degrees (default 45)"
    )
    
    args = parser.parse_args()
    
    # Setup logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=log_level, format='%(levelname)s: %(message)s')
    
    # Handle randomize flags
    randomize = args.randomize and not args.no_randomize
    
    if args.route:
        # GRIB-driven mode
        if args.sample_window and not args.sample_interval:
            parser.error("--sample-window requires --sample-interval")
        if args.sample_interval and not args.sample_window:
            parser.error("--sample-interval requires --sample-window")

        generate_grib_training_data(
            route_files=args.route,
            output_path=args.output,
            grib_dir=args.gribs,
            num_runs=args.num_runs,
            randomize=randomize,
            seed=args.seed,
            warmup_seconds=args.warmup,
            sample_window_hours=args.sample_window,
            sample_interval_hours=args.sample_interval,
            enable_perturbations=not args.no_perturbations,
            perturb_interval_sec=args.perturb_interval,
            perturb_magnitude_deg=args.perturb_magnitude,
            output_json=args.json,
        )
    else:
        # Scenario-based mode
        scenarios = None
        if args.scenarios:
            scenarios = [s.strip() for s in args.scenarios.split(',')]
            
        generate_training_data(
            output_path=args.output,
            duration_hours=args.hours,
            scenarios=scenarios,
            num_runs=args.num_runs,
            randomize=randomize,
            calibrate_from=args.calibrate_from,
            seed=args.seed,
            warmup_seconds=args.warmup,
            enable_perturbations=not args.no_perturbations,
            perturb_interval_sec=args.perturb_interval,
            perturb_magnitude_deg=args.perturb_magnitude,
            output_json=args.json,
        )


if __name__ == "__main__":
    main()
