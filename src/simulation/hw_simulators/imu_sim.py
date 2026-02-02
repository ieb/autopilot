"""
IMU Simulator
=============

Simulates the ICM-20948 IMU MCU with simplex protocol.

Protocol phases:
    1. STARTUP: Send $VER, $RDY on connection
    2. CONFIG: Process commands ($OFF, $MAG, $CAL,START/STOP, $START)
    3. STREAMING: Send $IMU messages at 100Hz (simplex, no command processing)

Streaming output:
    $IMU,heading,pitch,roll,yaw_rate,pitch_rate,roll_rate,ax,ay,az*XX\r\n
    
    All angles in degrees, rates in deg/s, accelerations in m/s²
"""

import time
import random
import logging
import math
import threading
from dataclasses import dataclass
from enum import Enum
from typing import Optional
from multiprocessing.managers import DictProxy

from .base import HardwareSimulator, SimulatorConfig

logger = logging.getLogger(__name__)


class SimState(Enum):
    """IMU simulator state machine states."""
    WAITING_FOR_CONNECTION = 0
    CONFIG = 1
    CALIBRATING = 2
    STREAMING = 3


@dataclass
class IMUSimulatorConfig(SimulatorConfig):
    """Configuration for IMU simulator."""
    socket_path: str = "/tmp/autopilot_imu.sock"
    update_rate_hz: float = 100.0
    
    # Firmware version to report
    firmware_version: str = "sim-1.0.0"
    
    # Noise parameters
    heading_noise_std: float = 0.1      # degrees
    attitude_noise_std: float = 0.05    # degrees
    rate_noise_std: float = 0.02        # deg/s
    accel_noise_std: float = 0.01       # m/s²
    
    # Timing jitter
    timing_jitter_std: float = 0.0005   # seconds (0.5ms)
    
    # IMU mounting offset from center of rotation (meters)
    # When non-zero, simulates centripetal/tangential accelerations
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0


class IMUSimulator(HardwareSimulator):
    """
    Simulates IMU MCU with simplex protocol.
    
    Implements three-phase protocol:
    1. Send startup sequence on client connection
    2. Process configuration commands
    3. Stream IMU data (simplex mode)
    """
    
    def __init__(
        self, 
        shared_state: DictProxy,
        config: Optional[IMUSimulatorConfig] = None
    ):
        config = config or IMUSimulatorConfig()
        super().__init__(config, shared_state)
        self.imu_config = config
        
        # State machine
        self._sim_state = SimState.WAITING_FOR_CONNECTION
        
        # Configuration (received from Pi)
        self._offset_x = config.offset_x
        self._offset_y = config.offset_y
        self._offset_z = config.offset_z
        
        self._mag_offset = (0.0, 0.0, 0.0)
        self._mag_scale = (1.0, 1.0, 1.0)
        
        # Calibration state
        self._calibrating = False
        self._cal_start_time = 0.0
        self._mag_min = [99999.0, 99999.0, 99999.0]
        self._mag_max = [-99999.0, -99999.0, -99999.0]
        
        # Previous readings for calculations
        self._prev_time = time.time()
        self._prev_gx = 0.0
        self._prev_gy = 0.0
        self._prev_gz = 0.0
        
        # Command processing thread
        self._cmd_thread: Optional[threading.Thread] = None
        self._cmd_buffer = ""
    
    def _send_message(self, payload: str):
        """Send an NMEA-style message with checksum."""
        checksum = self.compute_checksum(payload)
        message = f"${payload}*{checksum}\r\n"
        self.send(message.encode('ascii'))
    
    def _step(self):
        """Main simulation step - behavior depends on state."""
        if self._sim_state == SimState.WAITING_FOR_CONNECTION:
            # Check if we have a client - send startup sequence
            if self._client_socket is not None:
                # Send version
                self._send_message(f"VER,{self.imu_config.firmware_version}")
                time.sleep(0.05)
                # Send ready
                self._send_message("RDY")
                self._sim_state = SimState.CONFIG
                logger.info("IMU simulator: client connected, sent startup sequence")
            return
        
        elif self._sim_state == SimState.CONFIG:
            # Process configuration commands
            self._process_commands()
            return
        
        elif self._sim_state == SimState.CALIBRATING:
            # Update calibration and check for commands
            self._update_calibration()
            self._process_commands()
            return
        
        elif self._sim_state == SimState.STREAMING:
            # Generate and send IMU data
            self._output_imu_data()
    
    def _process_commands(self):
        """Process incoming configuration commands."""
        # Read available data using base class method
        data = self.recv(256)
        if data:
            self._cmd_buffer += data.decode('ascii', errors='ignore')
        
        # Process complete commands
        while '\n' in self._cmd_buffer:
            line, self._cmd_buffer = self._cmd_buffer.split('\n', 1)
            line = line.strip()
            if line.startswith('$'):
                self._handle_command(line[1:])  # Remove leading $
    
    def _handle_command(self, cmd: str):
        """Handle a configuration command."""
        # Remove checksum if present
        if '*' in cmd:
            cmd = cmd.split('*')[0]
        
        if cmd == "START":
            self._send_message("ACK")
            self._sim_state = SimState.STREAMING
            logger.info("IMU simulator: entering streaming mode")
            
        elif cmd.startswith("OFF,"):
            # Parse offset: OFF,x,y,z
            try:
                parts = cmd.split(',')
                self._offset_x = float(parts[1])
                self._offset_y = float(parts[2])
                self._offset_z = float(parts[3])
                self._send_message(f"OFS,{self._offset_x:.3f},{self._offset_y:.3f},{self._offset_z:.3f}")
                logger.debug(f"IMU simulator: offset set to {self._offset_x}, {self._offset_y}, {self._offset_z}")
            except (ValueError, IndexError) as e:
                self._send_message("ERR,PARSE_OFF")
                
        elif cmd.startswith("MAG,"):
            # Parse mag calibration: MAG,ox,oy,oz,sx,sy,sz
            try:
                parts = cmd.split(',')
                ox, oy, oz = float(parts[1]), float(parts[2]), float(parts[3])
                sx, sy, sz = float(parts[4]), float(parts[5]), float(parts[6])
                self._mag_offset = (ox, oy, oz)
                self._mag_scale = (sx, sy, sz)
                self._send_message(f"MGC,{ox:.2f},{oy:.2f},{oz:.2f},{sx:.3f},{sy:.3f},{sz:.3f}")
                logger.debug(f"IMU simulator: mag cal set")
            except (ValueError, IndexError) as e:
                self._send_message("ERR,PARSE_MAG")
                
        elif cmd == "CAL,START":
            self._calibrating = True
            self._cal_start_time = time.time()
            self._mag_min = [99999.0, 99999.0, 99999.0]
            self._mag_max = [-99999.0, -99999.0, -99999.0]
            self._sim_state = SimState.CALIBRATING
            self._send_message("ACK")
            logger.info("IMU simulator: calibration started")
            
        elif cmd == "CAL,STOP":
            if self._calibrating:
                self._finish_calibration()
            else:
                self._send_message("ERR,NOT_CALIBRATING")
        else:
            self._send_message("ERR,UNKNOWN_CMD")
    
    def _update_calibration(self):
        """Update calibration min/max during calibration mode."""
        # Simulate mag readings with some variation
        base_mag = [30.0, -15.0, 45.0]  # Simulated base magnetic field
        
        # Add some variation based on simulated orientation
        heading = self.state.get('heading', 0.0)
        roll = self.state.get('roll', 0.0)
        
        mx = base_mag[0] * math.cos(math.radians(heading)) + random.gauss(0, 1)
        my = base_mag[1] * math.sin(math.radians(heading)) + random.gauss(0, 1)
        mz = base_mag[2] + roll * 0.1 + random.gauss(0, 1)
        
        # Update min/max
        self._mag_min[0] = min(self._mag_min[0], mx)
        self._mag_max[0] = max(self._mag_max[0], mx)
        self._mag_min[1] = min(self._mag_min[1], my)
        self._mag_max[1] = max(self._mag_max[1], my)
        self._mag_min[2] = min(self._mag_min[2], mz)
        self._mag_max[2] = max(self._mag_max[2], mz)
        
        # Auto-finish after 30 seconds
        if time.time() - self._cal_start_time > 30.0:
            self._finish_calibration()
    
    def _finish_calibration(self):
        """Finish calibration and calculate values."""
        self._calibrating = False
        
        # Calculate offsets (center of min/max)
        ox = (self._mag_min[0] + self._mag_max[0]) / 2
        oy = (self._mag_min[1] + self._mag_max[1]) / 2
        oz = (self._mag_min[2] + self._mag_max[2]) / 2
        
        # Calculate scales
        rx = max(0.001, self._mag_max[0] - self._mag_min[0])
        ry = max(0.001, self._mag_max[1] - self._mag_min[1])
        rz = max(0.001, self._mag_max[2] - self._mag_min[2])
        avg_range = (rx + ry + rz) / 3
        
        sx = avg_range / rx
        sy = avg_range / ry
        sz = avg_range / rz
        
        self._mag_offset = (ox, oy, oz)
        self._mag_scale = (sx, sy, sz)
        
        # Send calibration values
        self._send_message(f"MGC,{ox:.2f},{oy:.2f},{oz:.2f},{sx:.3f},{sy:.3f},{sz:.3f}")
        
        self._sim_state = SimState.CONFIG
        logger.info("IMU simulator: calibration complete")
    
    def _output_imu_data(self):
        """Generate and send one IMU message."""
        now = time.time()
        dt = now - self._prev_time
        
        # Read state from shared memory
        heading = self.state.get('heading', 0.0)
        pitch = self.state.get('pitch', 0.0)
        roll = self.state.get('roll', 0.0)
        yaw_rate = self.state.get('yaw_rate', 0.0)
        pitch_rate = self.state.get('pitch_rate', 0.0)
        roll_rate = self.state.get('roll_rate', 0.0)
        
        # Gyro readings
        gx = roll_rate
        gy = pitch_rate
        gz = yaw_rate
        
        # Accelerations
        accel_x = self.state.get('accel_x', 0.0)
        accel_y = self.state.get('accel_y', 0.0)
        accel_z = self.state.get('accel_z', 9.81)
        
        # Add off-center mounting effects
        dx, dy, dz = self._offset_x, self._offset_y, self._offset_z
        
        if dx != 0.0 or dy != 0.0 or dz != 0.0:
            if dt > 0 and dt < 0.1:
                alpha_x = (gx - self._prev_gx) / dt
                alpha_y = (gy - self._prev_gy) / dt
                alpha_z = (gz - self._prev_gz) / dt
            else:
                alpha_x = alpha_y = alpha_z = 0.0
            
            deg_to_rad = math.pi / 180.0
            wx, wy, wz = gx * deg_to_rad, gy * deg_to_rad, gz * deg_to_rad
            ax_rad = alpha_x * deg_to_rad
            ay_rad = alpha_y * deg_to_rad
            az_rad = alpha_z * deg_to_rad
            
            # Centripetal acceleration
            cent_x = wy * (wy * dx - wx * dy) + wz * (wz * dx - wx * dz)
            cent_y = wx * (wx * dy - wy * dx) + wz * (wz * dy - wy * dz)
            cent_z = wx * (wx * dz - wz * dx) + wy * (wy * dz - wz * dy)
            
            # Tangential acceleration
            tang_x = ay_rad * dz - az_rad * dy
            tang_y = az_rad * dx - ax_rad * dz
            tang_z = ax_rad * dy - ay_rad * dx
            
            accel_x += cent_x + tang_x
            accel_y += cent_y + tang_y
            accel_z += cent_z + tang_z
        
        # Store for next iteration
        self._prev_gx, self._prev_gy, self._prev_gz = gx, gy, gz
        self._prev_time = now
        
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
        
        # Normalize heading
        heading = heading % 360.0
        
        # Send message
        payload = (
            f"IMU,{heading:.1f},{pitch:.1f},{roll:.1f},"
            f"{yaw_rate:.2f},{pitch_rate:.2f},{roll_rate:.2f},"
            f"{accel_x:.2f},{accel_y:.2f},{accel_z:.2f}"
        )
        
        # Add timing jitter
        if self.imu_config.timing_jitter_std > 0:
            jitter = abs(random.gauss(0, self.imu_config.timing_jitter_std))
            time.sleep(jitter)
        
        self._send_message(payload)


def run_imu_sim(shared_state: DictProxy, config: Optional[IMUSimulatorConfig] = None):
    """
    Entry point for running IMU simulator as a separate process.
    
    Args:
        shared_state: Shared state dict from multiprocessing.Manager
        config: Optional configuration
    """
    import signal
    
    sim = IMUSimulator(shared_state, config)
    
    def shutdown(signum, frame):
        sim.stop()
        
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    
    if sim.start():
        logger.info("IMU simulator running")
        try:
            while sim.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            sim.stop()
    else:
        logger.error("Failed to start IMU simulator")
