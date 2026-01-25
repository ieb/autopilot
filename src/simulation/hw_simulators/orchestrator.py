"""
Simulator Orchestrator
======================

Coordinates hardware simulators with shared yacht state.

Manages:
- Shared state between processes via multiprocessing.Manager
- IMU, Actuator, and CAN simulator processes
- Yacht dynamics physics loop
- Clean startup and shutdown
"""

import time
import multiprocessing
from multiprocessing import Process, Manager
from multiprocessing.managers import DictProxy
from dataclasses import dataclass, field
from typing import Optional, Dict, Any
import logging
import signal
import sys

from ..yacht_dynamics import YachtDynamics, YachtConfig
from ..wind_model import WindModel, WindConfig
from ..wave_model import WaveModel, WaveConfig
from ..scenarios import Scenario, get_scenario

from .imu_sim import IMUSimulator, IMUSimulatorConfig, run_imu_sim
from .actuator_sim import ActuatorSimulator, ActuatorSimulatorConfig, run_actuator_sim
from .can_sim import CANSimulator, CANSimulatorConfig, run_can_sim

logger = logging.getLogger(__name__)


@dataclass
class OrchestratorConfig:
    """Configuration for the simulator orchestrator."""
    # Physics update rate
    physics_rate_hz: float = 50.0
    
    # Simulator configs
    imu_config: IMUSimulatorConfig = field(default_factory=IMUSimulatorConfig)
    actuator_config: ActuatorSimulatorConfig = field(default_factory=ActuatorSimulatorConfig)
    can_config: CANSimulatorConfig = field(default_factory=CANSimulatorConfig)
    
    # Yacht configs
    yacht_config: YachtConfig = field(default_factory=YachtConfig)
    wind_config: WindConfig = field(default_factory=WindConfig)
    wave_config: WaveConfig = field(default_factory=WaveConfig)


class SimulatorOrchestrator:
    """
    Launch and coordinate hardware simulators.
    
    Creates a shared state dict that all simulators read from and write to,
    runs the physics simulation, and manages process lifecycle.
    """
    
    def __init__(
        self,
        scenario: Optional[Scenario] = None,
        config: Optional[OrchestratorConfig] = None
    ):
        self.config = config or OrchestratorConfig()
        self.scenario = scenario
        
        # Multiprocessing manager for shared state
        self._manager: Optional[Manager] = None
        self._state: Optional[DictProxy] = None
        
        # Simulator processes
        self._imu_proc: Optional[Process] = None
        self._actuator_proc: Optional[Process] = None
        self._can_proc: Optional[Process] = None
        
        # Physics models
        self._yacht: Optional[YachtDynamics] = None
        self._wind: Optional[WindModel] = None
        self._wave: Optional[WaveModel] = None
        
        # State
        self.running = False
        self._physics_time = 0.0
        
    def start(self) -> bool:
        """
        Start all simulators and physics loop.
        
        Returns:
            True if all components started successfully
        """
        try:
            # Create shared state manager
            self._manager = Manager()
            self._state = self._manager.dict()
            
            # Initialize state with defaults
            self._init_state()
            
            # Initialize physics models
            self._init_physics()
            
            # Start simulator processes
            self._start_simulators()
            
            self.running = True
            logger.info("Simulator orchestrator started")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start orchestrator: {e}")
            self.stop()
            return False
            
    def stop(self):
        """Stop all simulators and cleanup."""
        self.running = False
        
        # Terminate simulator processes
        for proc, name in [
            (self._imu_proc, "IMU"),
            (self._actuator_proc, "Actuator"),
            (self._can_proc, "CAN")
        ]:
            if proc and proc.is_alive():
                proc.terminate()
                proc.join(timeout=2.0)
                if proc.is_alive():
                    proc.kill()
                logger.info(f"{name} simulator stopped")
                
        self._imu_proc = None
        self._actuator_proc = None
        self._can_proc = None
        
        # Shutdown manager
        if self._manager:
            self._manager.shutdown()
            self._manager = None
            
        logger.info("Simulator orchestrator stopped")
        
    def step(self, dt: Optional[float] = None):
        """
        Advance physics simulation by one step.
        
        Args:
            dt: Time step in seconds (defaults to 1/physics_rate_hz)
        """
        if not self.running:
            return
            
        if dt is None:
            dt = 1.0 / self.config.physics_rate_hz
            
        self._physics_time += dt
        
        # Read rudder angle from actuator simulator
        rudder_deg = self._state.get('rudder_angle', 0.0)
        rudder_normalized = rudder_deg / self.config.actuator_config.max_rudder_angle
        
        # Update wind
        self._wind.update(dt)
        
        # Update yacht dynamics
        self._yacht.step(
            dt=dt,
            rudder_command=rudder_normalized,
            wind_speed=self._wind.current_tws,
            wind_direction=self._wind.current_twd
        )
        
        # Update wave model
        self._wave.update(dt)
        pitch, roll = self._wave.get_motion(self._yacht.heading, self._yacht.speed)
        
        # Update shared state
        self._update_state(pitch, roll)
        
    def run(self, duration: float, realtime: bool = True):
        """
        Run the simulation for a specified duration.
        
        Args:
            duration: Simulation duration in seconds
            realtime: If True, pace to real time; if False, run as fast as possible
        """
        dt = 1.0 / self.config.physics_rate_hz
        steps = int(duration / dt)
        
        if realtime:
            next_step = time.time()
            
        for i in range(steps):
            if not self.running:
                break
                
            self.step(dt)
            
            if realtime:
                next_step += dt
                now = time.time()
                if next_step > now:
                    time.sleep(next_step - now)
                elif next_step < now - 0.1:
                    # Fell behind, catch up
                    next_step = now
                    
    def get_state(self) -> Dict[str, Any]:
        """Get a copy of the current state dict."""
        if self._state:
            return dict(self._state)
        return {}
        
    def set_state(self, key: str, value: Any):
        """Set a value in the shared state."""
        if self._state:
            self._state[key] = value
            
    def _init_state(self):
        """Initialize shared state with default values."""
        # Position and orientation
        self._state['heading'] = 0.0
        self._state['pitch'] = 0.0
        self._state['roll'] = 0.0
        self._state['yaw_rate'] = 0.0
        self._state['pitch_rate'] = 0.0
        self._state['roll_rate'] = 0.0
        
        # Accelerations
        self._state['accel_x'] = 0.0
        self._state['accel_y'] = 0.0
        self._state['accel_z'] = 9.81
        
        # Speed
        self._state['stw'] = 0.0
        self._state['sog'] = 0.0
        self._state['cog'] = 0.0
        
        # Wind
        self._state['awa'] = 0.0
        self._state['aws'] = 0.0
        self._state['twa'] = 0.0
        self._state['tws'] = 0.0
        self._state['twd'] = 0.0
        
        # Position
        self._state['latitude'] = 51.0
        self._state['longitude'] = 1.0
        
        # Rudder (written by actuator simulator)
        self._state['rudder_angle'] = 0.0
        
    def _init_physics(self):
        """Initialize physics models from scenario or config."""
        # Wind model
        wind_config = self.config.wind_config
        if self.scenario:
            wind_config = WindConfig(
                base_tws=self.scenario.wind_speed,
                base_twd=self.scenario.wind_direction,
                gust_intensity=self.scenario.gust_intensity,
                shift_rate=self.scenario.shift_rate
            )
        self._wind = WindModel(wind_config)
        
        # Wave model
        self._wave = WaveModel(self.config.wave_config)
        
        # Yacht dynamics
        self._yacht = YachtDynamics(self.config.yacht_config)
        
        # Set initial conditions from scenario
        if self.scenario:
            self._yacht.heading = self.scenario.initial_heading
            self._yacht.speed = self.scenario.initial_speed
            
    def _start_simulators(self):
        """Start simulator processes."""
        # IMU simulator
        self._imu_proc = Process(
            target=run_imu_sim,
            args=(self._state, self.config.imu_config),
            name="IMUSimulator"
        )
        self._imu_proc.start()
        logger.info(f"Started IMU simulator (PID {self._imu_proc.pid})")
        
        # Actuator simulator
        self._actuator_proc = Process(
            target=run_actuator_sim,
            args=(self._state, self.config.actuator_config),
            name="ActuatorSimulator"
        )
        self._actuator_proc.start()
        logger.info(f"Started Actuator simulator (PID {self._actuator_proc.pid})")
        
        # CAN simulator
        self._can_proc = Process(
            target=run_can_sim,
            args=(self._state, self.config.can_config),
            name="CANSimulator"
        )
        self._can_proc.start()
        logger.info(f"Started CAN simulator (PID {self._can_proc.pid})")
        
        # Give simulators time to start
        time.sleep(0.5)
        
    def _update_state(self, pitch: float, roll: float):
        """Update shared state from physics models."""
        # Orientation
        self._state['heading'] = self._yacht.heading
        self._state['pitch'] = pitch
        self._state['roll'] = roll
        
        # Compute rates (simple finite difference)
        # In a real system these come from the IMU gyroscope
        self._state['yaw_rate'] = self._yacht.yaw_rate
        self._state['pitch_rate'] = 0.0  # Would need history
        self._state['roll_rate'] = 0.0   # Would need history
        
        # Speed
        self._state['stw'] = self._yacht.speed
        self._state['sog'] = self._yacht.sog
        self._state['cog'] = self._yacht.cog
        
        # Wind
        self._state['tws'] = self._wind.current_tws
        self._state['twd'] = self._wind.current_twd
        
        # Compute apparent wind
        awa, aws = self._compute_apparent_wind(
            self._yacht.speed,
            self._yacht.heading,
            self._wind.current_tws,
            self._wind.current_twd
        )
        self._state['awa'] = awa
        self._state['aws'] = aws
        self._state['twa'] = self._compute_twa(self._yacht.heading, self._wind.current_twd)
        
        # Position (simplified - would need proper integration)
        # For now just drift slightly
        self._state['latitude'] = 51.0 + self._physics_time * 0.00001
        self._state['longitude'] = 1.0 + self._physics_time * 0.00002
        
    def _compute_apparent_wind(
        self, 
        boat_speed: float, 
        heading: float,
        tws: float, 
        twd: float
    ) -> tuple[float, float]:
        """Compute apparent wind angle and speed."""
        import math
        
        twa = twd - heading
        # Normalize to -180 to 180
        while twa > 180:
            twa -= 360
        while twa < -180:
            twa += 360
            
        twa_rad = math.radians(twa)
        
        # Vector addition
        aw_x = tws * math.cos(twa_rad) + boat_speed
        aw_y = tws * math.sin(twa_rad)
        
        aws = math.sqrt(aw_x**2 + aw_y**2)
        awa = math.degrees(math.atan2(aw_y, aw_x))
        
        return awa, aws
        
    def _compute_twa(self, heading: float, twd: float) -> float:
        """Compute true wind angle relative to boat heading."""
        twa = twd - heading
        while twa > 180:
            twa -= 360
        while twa < -180:
            twa += 360
        return twa


def main():
    """CLI entry point for running the simulator orchestrator."""
    import argparse
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    parser = argparse.ArgumentParser(description="Run hardware simulators")
    parser.add_argument(
        '--scenario', '-s',
        default='upwind',
        help='Scenario name (upwind, reaching, downwind, tacking)'
    )
    parser.add_argument(
        '--duration', '-d',
        type=float,
        default=60.0,
        help='Simulation duration in seconds'
    )
    parser.add_argument(
        '--fast',
        action='store_true',
        help='Run as fast as possible (not realtime)'
    )
    
    args = parser.parse_args()
    
    # Load scenario
    scenario = get_scenario(args.scenario)
    if not scenario:
        logger.warning(f"Unknown scenario '{args.scenario}', using defaults")
        
    # Create and start orchestrator
    orchestrator = SimulatorOrchestrator(scenario=scenario)
    
    # Handle signals
    def shutdown(signum, frame):
        logger.info("Shutting down...")
        orchestrator.stop()
        sys.exit(0)
        
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    
    if orchestrator.start():
        logger.info(f"Running simulation for {args.duration}s...")
        orchestrator.run(args.duration, realtime=not args.fast)
        orchestrator.stop()
    else:
        logger.error("Failed to start orchestrator")
        sys.exit(1)


if __name__ == "__main__":
    main()
