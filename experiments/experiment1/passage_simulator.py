"""
Passage Simulator
=================

Main simulation orchestrator for the planned passage experiment.
Integrates all components to simulate following a route.
"""

import math
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timedelta
from pathlib import Path
from typing import Optional, Iterator, Dict, Any, List
import logging

import numpy as np

# Import project modules
from src.simulation.yacht_dynamics import YachtDynamics, YachtConfig
from src.simulation.wave_model import WaveModel, WaveConfig
from src.simulation.helm_controller import HelmController, HelmConfig, SteeringMode as HelmSteeringMode
from src.ml.polar import Polar

# Import experiment modules
from .route_parser import RouteParser, Waypoint
from .grib_reader import GribReader
from .wind_provider import WindProvider
from .wave_provider import WaveProvider
from .navigator import Navigator, SteeringMode
from .metrics import MetricsTracker

logger = logging.getLogger(__name__)


@dataclass
class SimulationConfig:
    """Configuration for passage simulation."""
    # Timing
    dt: float = 0.1                    # Time step (seconds)
    sample_rate: float = 10.0          # Samples per second for output
    max_duration_hours: float = 24.0   # Maximum simulation duration
    
    # Physics
    yacht_config: Optional[YachtConfig] = None
    
    # Controller
    use_baseline: bool = False         # Use helm controller instead of ML model
    use_mock: bool = False             # Use mock autopilot (simple P control) for testing
    model_path: Optional[str] = None   # Path to trained model
    
    # Options
    verbose: bool = False
    progress_interval: float = 300.0   # Progress update interval (seconds)


class PassageSimulator:
    """
    Simulates following a planned passage using the autopilot.
    
    Features:
    - Loads route and GRIB data
    - Runs yacht dynamics simulation
    - Uses autopilot model or baseline controller for steering
    - Tracks comprehensive performance metrics
    """
    
    # Motoring speed (knots)
    MOTORING_STW = 6.0
    
    def __init__(self, 
                 route_file: str,
                 grib_dir: Optional[str] = None,
                 config: Optional[SimulationConfig] = None):
        """
        Initialize passage simulator.
        
        Args:
            route_file: Path to route CSV file
            grib_dir: Path to directory containing GRIB files
            config: Simulation configuration
        """
        self.config = config or SimulationConfig()
        
        # Parse route
        logger.info(f"Loading route from {route_file}")
        self.route_parser = RouteParser(route_file)
        self.waypoints = self.route_parser.parse()
        
        if not self.waypoints:
            raise ValueError("No waypoints found in route file")
            
        logger.info(f"Loaded {len(self.waypoints)} waypoints")
        
        # Load GRIB data if available
        self.grib_reader: Optional[GribReader] = None
        if grib_dir:
            logger.info(f"Loading GRIB data from {grib_dir}")
            self.grib_reader = GribReader()
            files_loaded = self.grib_reader.load_directory(grib_dir)
            logger.info(f"Loaded {files_loaded} GRIB files")
            
        # Initialize providers
        self.wind_provider = WindProvider(self.grib_reader, self.waypoints)
        self.wave_provider = WaveProvider(self.grib_reader, self.waypoints)
        
        # Initialize navigator
        self.navigator = Navigator(self.waypoints)
        
        # Initialize yacht dynamics
        yacht_config = self.config.yacht_config or YachtConfig()
        self.polar = Polar.pogo_1250()
        self.yacht = YachtDynamics(yacht_config, self.polar)
        
        # Initialize wave model
        self.wave_model = WaveModel()
        
        # Initialize controller
        self.helm_controller: Optional[HelmController] = None
        self.autopilot = None
        
        if self.config.use_baseline:
            self.helm_controller = HelmController.expert()
            logger.info("Using baseline helm controller")
        elif self.config.use_mock:
            self._load_mock_autopilot()
        else:
            self._load_autopilot_model()
        
        # Inference performance tracking (uses wall-clock time, not sim time)
        self._inference_times: List[float] = []
        self._last_inference_report_walltime = 0.0
        self._inference_report_interval = 60.0  # Report every 60 seconds of wall-clock time
            
        # Initialize metrics
        self.metrics = MetricsTracker(
            self.route_parser.start_time,
            self.route_parser.end_time
        )
        
        # Simulation state
        self.current_time: Optional[datetime] = None
        self.elapsed_time = 0.0
        self.is_running = False
        
    def _load_autopilot_model(self):
        """Load the trained autopilot model."""
        if self.config.model_path:
            try:
                from src.ml.autopilot_model import AutopilotInference
                self.autopilot = AutopilotInference(self.config.model_path)
                logger.info(f"Loaded autopilot model from {self.config.model_path}")
            except Exception as e:
                logger.warning(f"Failed to load model: {e}. Using baseline controller.")
                self.helm_controller = HelmController.expert()
        else:
            # No model specified - use mock or baseline
            logger.info("No model specified, using baseline controller")
            self.helm_controller = HelmController.expert()
    
    def _load_mock_autopilot(self):
        """Load mock autopilot for hypothesis testing."""
        from src.ml.autopilot_model import MockAutopilotInference
        self.autopilot = MockAutopilotInference()
        logger.info("Using MOCK autopilot (simple P control: rudder = +error * 0.5)")
            
    def run(self) -> Dict[str, Any]:
        """
        Run the complete passage simulation.
        
        Returns:
            Dictionary with simulation results
        """
        logger.info("Starting passage simulation...")
        
        # Initialize state
        self._initialize()
        
        # Run simulation loop
        self.is_running = True
        last_progress_time = 0.0
        self._last_inference_report_walltime = time.time()
        
        try:
            while self.is_running:
                # Step simulation
                self._step()
                
                # Check for completion
                if self.navigator.state.arrived_at_destination:
                    logger.info("Arrived at destination!")
                    break
                    
                # Check for timeout
                if self.elapsed_time > self.config.max_duration_hours * 3600:
                    logger.warning("Simulation timeout reached")
                    break
                    
                # Progress update
                if self.config.verbose:
                    if self.elapsed_time - last_progress_time >= self.config.progress_interval:
                        self._log_progress()
                        last_progress_time = self.elapsed_time
                
                # Inference performance report (for ML autopilot, every 60s wall-clock)
                if self.autopilot and self._inference_times:
                    current_walltime = time.time()
                    if current_walltime - self._last_inference_report_walltime >= self._inference_report_interval:
                        self._log_inference_stats()
                        self._last_inference_report_walltime = current_walltime
                        
        except KeyboardInterrupt:
            logger.info("Simulation interrupted by user")
            
        self.is_running = False
        
        # Final inference report
        if self.autopilot and self._inference_times:
            self._log_inference_stats(final=True)
        
        # Compute and return results
        summary = self.metrics.compute_summary()
        self.metrics.print_summary()
        
        return {
            'summary': summary,
            'arrived': self.navigator.state.arrived_at_destination,
            'elapsed_time': self.elapsed_time,
            'waypoints_completed': self.navigator.current_leg,
        }
        
    def _initialize(self):
        """Initialize simulation state."""
        # Set start time
        self.current_time = self.route_parser.start_time
        self.elapsed_time = 0.0
        
        # Set initial position
        first_wp = self.waypoints[0]
        start_lat = first_wp.from_lat
        start_lon = first_wp.from_lon
        
        # Set initial heading and wind
        initial_heading = first_wp.cog
        initial_twd = first_wp.twd
        initial_tws = first_wp.tws
        
        # Initialize yacht state
        self.yacht.reset(heading=initial_heading, twd=initial_twd, tws=initial_tws)
        self.yacht.state.latitude = start_lat
        self.yacht.state.longitude = start_lon
        
        # Initialize feature history for model (if using model)
        self._feature_history = []
        
        logger.info(f"Initialized at {start_lat:.4f}°N, {start_lon:.4f}°E, "
                   f"heading {initial_heading:.0f}°")
        
    def _step(self):
        """Execute one simulation timestep."""
        dt = self.config.dt
        
        # Get current position
        lat = self.yacht.state.latitude
        lon = self.yacht.state.longitude
        heading = self.yacht.state.heading
        stw = self.yacht.state.stw
        
        # Get wind conditions
        wind = self.wind_provider.get_wind(lat, lon, self.current_time)
        twd = wind.twd
        tws = wind.tws
        
        # Update yacht wind state
        self.yacht.set_wind(twd, tws)
        
        # Get wave conditions and update wave model
        waves = self.wave_provider.get_waves(lat, lon, self.current_time, tws, twd)
        self.wave_model.set_sea_state(waves.mean_period, waves.amplitude_degrees)
        
        # Step wave model for motion
        wave_roll, wave_pitch = self.wave_model.step(
            dt, tws=tws, heading=heading
        )
        
        # Update navigator
        nav_state = self.navigator.update(lat, lon, twd, heading, stw)
        
        # Determine control input
        if nav_state.is_motoring:
            # Motoring mode - simple heading hold
            rudder_command = self._compute_motoring_rudder(heading, nav_state.target_heading)
            # Override speed for motoring
            self.yacht.state.stw = self.MOTORING_STW
        else:
            # Sailing mode - use autopilot or helm controller
            rudder_command = self._compute_sailing_rudder(nav_state, twd)
            
        # Step yacht dynamics
        self.yacht.step(rudder_command, dt)
        
        # Apply wave motion to attitude
        total_roll = self.yacht.state.roll + wave_roll
        total_pitch = self.yacht.state.pitch + wave_pitch
        
        # Compute polar performance
        twa = twd - heading
        while twa > 180:
            twa -= 360
        while twa < -180:
            twa += 360
        polar_target = self.polar.get_target_speed(abs(twa), tws)
        polar_perf = stw / polar_target if polar_target > 0 else 0.0
        
        # Record metrics (subsample to reduce data)
        if int(self.elapsed_time * self.config.sample_rate) > int((self.elapsed_time - dt) * self.config.sample_rate):
            self.metrics.record(
                elapsed_time=self.elapsed_time,
                lat=lat, lon=lon,
                heading=heading,
                target_heading=nav_state.target_heading,
                stw=stw, sog=self.yacht.state.sog,
                tws=tws, twd=twd,
                cross_track_error=nav_state.cross_track_error,
                polar_performance=polar_perf,
                rudder_angle=self.yacht.state.rudder_angle,
                steering_mode=nav_state.steering_mode.value,
                leg_index=nav_state.current_leg_index,
            )
            
        # Advance time
        self.elapsed_time += dt
        self.current_time += timedelta(seconds=dt)
        
    def _compute_sailing_rudder(self, nav_state, twd: float) -> float:
        """Compute rudder command for sailing mode."""
        # Get current state
        heading = self.yacht.state.heading
        awa = self.yacht.state.awa
        heading_rate = self.yacht.state.heading_rate
        
        # Compute TWA
        twa = twd - heading
        while twa > 180:
            twa -= 360
        while twa < -180:
            twa += 360
            
        if self.helm_controller:
            # Use simple PD controller for route following
            # Target heading computed by navigator
            target_heading = nav_state.bearing_to_waypoint
            
            # Heading error
            error = target_heading - heading
            while error > 180:
                error -= 360
            while error < -180:
                error += 360
                
            # PD control
            kp = 0.5   # Proportional gain
            kd = 0.3   # Derivative gain
            
            command = kp * error - kd * heading_rate
            
            # Limit rudder angle
            return max(-25, min(25, command))
            
        elif self.autopilot:
            # Use ML autopilot model
            features = self._compute_features(nav_state, twd)
            self._feature_history.append(features)
            
            # Keep last 20 timesteps
            if len(self._feature_history) > 20:
                self._feature_history.pop(0)
                
            # Pad if needed
            while len(self._feature_history) < 20:
                self._feature_history.insert(0, self._feature_history[0] if self._feature_history else features)
                
            # Stack into sequence
            sequence = np.array(self._feature_history)
            
            # Get model prediction with timing
            t_start = time.perf_counter()
            rudder_normalized = self.autopilot.predict(sequence)
            t_elapsed = time.perf_counter() - t_start
            self._inference_times.append(t_elapsed)
            
            # Convert from [-1, 1] to degrees (max 25°)
            return rudder_normalized * 25.0
            
        else:
            # Fallback - simple proportional control
            return self._simple_heading_hold(heading, nav_state.target_heading)
            
    def _compute_motoring_rudder(self, heading: float, target: float) -> float:
        """Simple heading hold for motoring."""
        return self._simple_heading_hold(heading, target)
        
    def _simple_heading_hold(self, heading: float, target: float) -> float:
        """Simple proportional heading hold."""
        error = target - heading
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
            
        # Simple P controller
        # Positive error (target > heading) means turn right = positive rudder
        kp = 0.5
        return max(-25, min(25, kp * error))
        
    def _compute_features(self, nav_state, twd: float) -> np.ndarray:
        """
        Compute feature vector for ML model.
        
        ARCHITECTURE: The model always steers to a computed heading.
        In wind modes, the target heading is computed from the target wind angle.
        This matches ModeManager.update() behavior and simplifies the model task.
        
        Feature indices:
        - FEAT_HEADING_ERROR = 0: (heading - computed_heading) / 180 (ALWAYS heading-based)
        - FEAT_HEADING_ERROR_INTEGRAL = 1
        - FEAT_HEADING_RATE = 2
        - FEAT_ROLL = 3
        - FEAT_PITCH = 4
        - FEAT_ROLL_RATE = 5
        - FEAT_AWA = 6 (context: affects boat dynamics)
        - FEAT_AWA_RATE = 7
        - FEAT_AWS = 8 (context: affects boat dynamics)
        - FEAT_TWA = 9
        - FEAT_TWS = 10
        - FEAT_STW = 11
        - FEAT_SOG = 12
        - FEAT_COG_ERROR = 13
        - FEAT_RUDDER_POSITION = 14
        - FEAT_RUDDER_VELOCITY = 15
        - FEAT_TARGET_ANGLE = 16: The computed target heading (always heading)
        - FEAT_VMG_UP = 17
        - FEAT_VMG_DOWN = 18
        - FEAT_POLAR_TARGET = 19
        - FEAT_POLAR_PERFORMANCE = 20
        - FEAT_MODE_COMPASS = 21
        - FEAT_MODE_WIND_AWA = 22
        - FEAT_MODE_WIND_TWA = 23
        - FEAT_WAVE_PERIOD = 24
        """
        features = np.zeros(25)
        
        heading = self.yacht.state.heading
        awa = self.yacht.state.awa
        
        # Compute TWA
        twa = twd - heading
        while twa > 180:
            twa -= 360
        while twa < -180:
            twa += 360
        
        # ARCHITECTURE: Compute target heading from mode-specific target
        # Wind targets are converted to heading targets so the model always steers to heading.
        # 
        # Sign convention (HelmController aligned):
        # - error = computed_heading - heading (positive = target to starboard)
        # - Positive error → positive rudder → turn starboard
        # 
        # For AWA mode: If AWA > target_awa (wind too far aft, need to head up):
        # - computed_heading = heading + (awa - target_awa) > heading
        # - error = computed_heading - heading > 0 (positive)
        # - Positive rudder → turn starboard → heading increases → AWA decreases ✓
        
        if nav_state.steering_mode == SteeringMode.COMPASS:
            # Direct heading target
            computed_heading = nav_state.target_heading
        elif nav_state.steering_mode == SteeringMode.WIND_AWA:
            # Compute heading that would achieve target AWA
            # If awa > target_awa: need to head up → increase heading → computed > heading
            awa_delta = awa - nav_state.target_awa
            computed_heading = heading + awa_delta
        elif nav_state.steering_mode == SteeringMode.WIND_TWA:
            # Compute heading that would achieve target TWA
            twa_delta = twa - nav_state.target_twa
            computed_heading = heading + twa_delta
        else:
            # Default to compass mode (for MOTORING)
            computed_heading = nav_state.target_heading
        
        # Normalize computed_heading to [0, 360)
        while computed_heading < 0:
            computed_heading += 360
        while computed_heading >= 360:
            computed_heading -= 360
            
        # FEAT_ERROR (0): ALWAYS heading error (HelmController convention)
        # error = computed_heading - heading (target - current)
        # Positive error = target to starboard → positive rudder → turn starboard
        error = computed_heading - heading
        
        # Normalize error to [-180, 180]
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        features[0] = np.clip(error / 180.0, -1.0, 1.0)
        
        # FEAT_HEADING_ERROR_INTEGRAL (1): Skip for now (would need state tracking)
        features[1] = 0.0
        
        # FEAT_HEADING_RATE (2)
        features[2] = np.clip(self.yacht.state.heading_rate / 30.0, -1.0, 1.0)
        
        # FEAT_ROLL (3), FEAT_PITCH (4)
        features[3] = np.clip(self.yacht.state.roll / 45.0, -1.0, 1.0)
        features[4] = np.clip(self.yacht.state.pitch / 30.0, -1.0, 1.0)
        
        # FEAT_ROLL_RATE (5): Skip (would need state tracking)
        features[5] = 0.0
        
        # FEAT_AWA (6)
        features[6] = np.clip(awa / 180.0, -1.0, 1.0)
        
        # FEAT_AWA_RATE (7): Skip
        features[7] = 0.0
        
        # FEAT_AWS (8)
        features[8] = np.clip(self.yacht.state.aws / 60.0, -1.0, 1.0)
        
        # FEAT_TWA (9), FEAT_TWS (10)
        features[9] = np.clip(twa / 180.0, -1.0, 1.0)
        features[10] = np.clip(self.yacht.state.tws / 60.0, 0.0, 1.0)
        
        # FEAT_STW (11), FEAT_SOG (12)
        features[11] = np.clip(self.yacht.state.stw / 25.0, 0.0, 1.0)
        features[12] = np.clip(self.yacht.state.sog / 25.0, 0.0, 1.0)
        
        # FEAT_COG_ERROR (13): cog - target_heading (compass bearing error)
        cog_error = self.yacht.state.cog - nav_state.target_heading
        while cog_error > 180:
            cog_error -= 360
        while cog_error < -180:
            cog_error += 360
        features[13] = np.clip(cog_error / 180.0, -1.0, 1.0)
        
        # FEAT_RUDDER_POSITION (14) - normalized to 25° max
        features[14] = np.clip(self.yacht.state.rudder_angle / 25.0, -1.0, 1.0)
        
        # FEAT_RUDDER_VELOCITY (15): Skip
        features[15] = 0.0
        
        # FEAT_TARGET_ANGLE (16): The computed target heading (always heading-based)
        features[16] = np.clip(computed_heading / 180.0, -1.0, 1.0)
        
        # FEAT_VMG_UP (17), FEAT_VMG_DOWN (18)
        stw = self.yacht.state.stw
        if stw > 0:
            vmg_up = stw * math.cos(math.radians(abs(twa)))
            vmg_down = stw * math.cos(math.radians(180 - abs(twa)))
        else:
            vmg_up = vmg_down = 0.0
        features[17] = np.clip(vmg_up / 15.0, -1.0, 1.0)
        features[18] = np.clip(vmg_down / 20.0, -1.0, 1.0)
        
        # FEAT_POLAR_TARGET (19), FEAT_POLAR_PERFORMANCE (20)
        polar_target = self.polar.get_target_speed(abs(twa), self.yacht.state.tws)
        polar_perf = stw / polar_target if polar_target > 0 else 0.0
        features[19] = np.clip(polar_target / 25.0, 0.0, 1.0)
        features[20] = np.clip(polar_perf, 0.0, 1.2)
        
        # Mode flags (21-23)
        if nav_state.steering_mode == SteeringMode.COMPASS:
            features[21] = 1.0
        elif nav_state.steering_mode == SteeringMode.WIND_AWA:
            features[22] = 1.0
        elif nav_state.steering_mode == SteeringMode.WIND_TWA:
            features[23] = 1.0
            
        # FEAT_WAVE_PERIOD (24): Use default
        features[24] = np.clip(5.0 / 15.0, 0.0, 1.0)
            
        return features
        
    def _log_progress(self):
        """Log simulation progress."""
        progress = self.navigator.get_route_progress() * 100
        leg = self.navigator.current_leg + 1
        total_legs = self.navigator.total_waypoints
        
        hours = self.elapsed_time / 3600
        remaining = self.navigator.state.distance_to_waypoint
        
        logger.info(f"Progress: {progress:.1f}% | Leg {leg}/{total_legs} | "
                   f"Time: {hours:.1f}h | DTW: {remaining:.2f} nm")
    
    def _log_inference_stats(self, final: bool = False):
        """Log ML autopilot inference performance statistics."""
        if not self._inference_times:
            return
        
        times_ms = np.array(self._inference_times) * 1000  # Convert to ms
        
        count = len(times_ms)
        mean_ms = np.mean(times_ms)
        std_ms = np.std(times_ms)
        min_ms = np.min(times_ms)
        max_ms = np.max(times_ms)
        p95_ms = np.percentile(times_ms, 95)
        p99_ms = np.percentile(times_ms, 99)
        
        # Calculate effective rate
        rate_hz = 1000.0 / mean_ms if mean_ms > 0 else 0
        
        # Check if meeting real-time requirements (10Hz = 100ms per inference)
        realtime_ok = mean_ms < 100
        status = "OK" if realtime_ok else "SLOW"
        
        prefix = "FINAL " if final else ""
        logger.info(
            f"{prefix}ML Inference Stats ({count} calls): "
            f"mean={mean_ms:.1f}ms, std={std_ms:.1f}ms, "
            f"min={min_ms:.1f}ms, max={max_ms:.1f}ms, "
            f"p95={p95_ms:.1f}ms, p99={p99_ms:.1f}ms, "
            f"rate={rate_hz:.1f}Hz [{status}]"
        )
        
        # Clear for next reporting period (but keep last 100 for running stats)
        if not final and len(self._inference_times) > 100:
            self._inference_times = self._inference_times[-100:]
                   
    def save_results(self, output_dir: str):
        """Save simulation results to files."""
        self.metrics.save_results(output_dir)
