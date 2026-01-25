"""
IMU Simulator
=============

Simulates the ICM-20948 IMU MCU by streaming $IMU messages at 100Hz.

Protocol:
    $IMU,heading,pitch,roll,yaw_rate,pitch_rate,roll_rate,ax,ay,az*XX\r\n
    
    All angles in degrees, rates in deg/s, accelerations in m/s²
"""

import time
import random
import logging
from dataclasses import dataclass
from typing import Optional
from multiprocessing.managers import DictProxy

from .base import HardwareSimulator, SimulatorConfig

logger = logging.getLogger(__name__)


@dataclass
class IMUSimulatorConfig(SimulatorConfig):
    """Configuration for IMU simulator."""
    socket_path: str = "/tmp/autopilot_imu.sock"
    update_rate_hz: float = 100.0
    
    # Noise parameters
    heading_noise_std: float = 0.1      # degrees
    attitude_noise_std: float = 0.05    # degrees
    rate_noise_std: float = 0.02        # deg/s
    accel_noise_std: float = 0.01       # m/s²
    
    # Timing jitter
    timing_jitter_std: float = 0.0005   # seconds (0.5ms)


class IMUSimulator(HardwareSimulator):
    """
    Simulates IMU MCU output.
    
    Reads yacht state from shared memory and streams NMEA-style
    IMU messages at 100Hz with realistic sensor noise.
    """
    
    def __init__(
        self, 
        shared_state: DictProxy,
        config: Optional[IMUSimulatorConfig] = None
    ):
        config = config or IMUSimulatorConfig()
        super().__init__(config, shared_state)
        self.imu_config = config
        
        # Previous heading for rate calculation
        self._prev_heading = 0.0
        self._prev_time = time.time()
        
    def _step(self):
        """Generate and send one IMU message."""
        now = time.time()
        
        # Read state from shared memory
        heading = self.state.get('heading', 0.0)
        pitch = self.state.get('pitch', 0.0)
        roll = self.state.get('roll', 0.0)
        yaw_rate = self.state.get('yaw_rate', 0.0)
        pitch_rate = self.state.get('pitch_rate', 0.0)
        roll_rate = self.state.get('roll_rate', 0.0)
        
        # Accelerations from motion (default to gravity if not set)
        accel_x = self.state.get('accel_x', 0.0)
        accel_y = self.state.get('accel_y', 0.0)
        accel_z = self.state.get('accel_z', 9.81)
        
        # Add sensor noise
        heading += random.gauss(0, self.imu_config.heading_noise_std)
        pitch += random.gauss(0, self.imu_config.attitude_noise_std)
        roll += random.gauss(0, self.imu_config.attitude_noise_std)
        yaw_rate += random.gauss(0, self.imu_config.rate_noise_std)
        pitch_rate += random.gauss(0, self.imu_config.rate_noise_std)
        roll_rate += random.gauss(0, self.imu_config.rate_noise_std)
        accel_x += random.gauss(0, self.imu_config.accel_noise_std)
        accel_y += random.gauss(0, self.imu_config.accel_noise_std)
        accel_z += random.gauss(0, self.imu_config.accel_noise_std)
        
        # Normalize heading to 0-360
        heading = heading % 360.0
        
        # Format message
        payload = (
            f"IMU,{heading:.1f},{pitch:.1f},{roll:.1f},"
            f"{yaw_rate:.2f},{pitch_rate:.2f},{roll_rate:.2f},"
            f"{accel_x:.2f},{accel_y:.2f},{accel_z:.2f}"
        )
        checksum = self.compute_checksum(payload)
        message = f"${payload}*{checksum}\r\n"
        
        # Add timing jitter
        if self.imu_config.timing_jitter_std > 0:
            jitter = abs(random.gauss(0, self.imu_config.timing_jitter_std))
            time.sleep(jitter)
        
        # Send
        self.send(message.encode('ascii'))
        
        # Update state for rate calculations
        self._prev_heading = heading
        self._prev_time = now


def run_imu_sim(shared_state: DictProxy, config: Optional[IMUSimulatorConfig] = None):
    """
    Entry point for running IMU simulator as a separate process.
    
    Args:
        shared_state: Shared state dict from multiprocessing.Manager
        config: Optional configuration
    """
    import signal
    
    sim = IMUSimulator(shared_state, config)
    
    # Handle shutdown signals
    def shutdown(signum, frame):
        sim.stop()
        
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    
    if sim.start():
        logger.info("IMU simulator running")
        # Keep process alive
        try:
            while sim.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            sim.stop()
    else:
        logger.error("Failed to start IMU simulator")
