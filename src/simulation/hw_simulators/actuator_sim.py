"""
Actuator Simulator
==================

Simulates the Actuator Controller MCU with position control, clutch, and watchdog.

Protocol:
    Input:  $RUD,<target>,<engage>*XX
    Output: $STS,<target>,<actual>,<clutch>,<voltage>,<current>,<fault>*XX @20Hz
"""

import time
import re
import logging
from dataclasses import dataclass
from typing import Optional
from enum import IntEnum
from multiprocessing.managers import DictProxy

from .base import HardwareSimulator, SimulatorConfig

logger = logging.getLogger(__name__)


class FaultCode(IntEnum):
    """Fault codes matching MCU firmware."""
    NONE = 0
    OVERCURRENT = 1
    STALL = 2
    POSITION_LIMIT = 3
    SENSOR = 4
    WATCHDOG = 5


@dataclass
class ActuatorSimulatorConfig(SimulatorConfig):
    """Configuration for Actuator simulator."""
    socket_path: str = "/tmp/autopilot_actuator.sock"
    update_rate_hz: float = 20.0  # Status output rate
    
    # Physical parameters
    max_rudder_rate: float = 4.0       # degrees per second
    max_rudder_angle: float = 30.0     # degrees
    position_limit: float = 28.0       # software limit (degrees)
    
    # Watchdog
    watchdog_timeout: float = 2.0      # seconds
    
    # Current simulation
    base_current: float = 0.5          # Amps at rest
    load_current_factor: float = 0.3   # Additional A per degree of rudder load
    
    # Voltage
    supply_voltage: float = 12.4       # Volts
    
    # Clutch timing
    clutch_engage_delay: float = 0.1   # seconds


class ActuatorSimulator(HardwareSimulator):
    """
    Simulates Actuator Controller MCU.
    
    Receives rudder commands, simulates position control with rate limiting,
    manages clutch state, and enforces watchdog timeout.
    """
    
    def __init__(
        self,
        shared_state: DictProxy,
        config: Optional[ActuatorSimulatorConfig] = None
    ):
        config = config or ActuatorSimulatorConfig()
        super().__init__(config, shared_state)
        self.act_config = config
        
        # State
        self._target_angle = 0.0        # Normalized -1 to 1
        self._actual_angle = 0.0        # Normalized -1 to 1
        self._clutch_engaged = False
        self._clutch_engaging = False
        self._clutch_engage_start = 0.0
        self._fault_code = FaultCode.NONE
        
        # Watchdog
        self._last_command_time = 0.0
        
        # Input buffer
        self._input_buffer = ""
        
        # Timing
        self._last_step_time = time.time()
        
    def _step(self):
        """Process commands and send status."""
        now = time.time()
        dt = now - self._last_step_time
        self._last_step_time = now
        
        # Process any incoming commands
        self._process_commands()
        
        # Check watchdog
        if self._clutch_engaged:
            if now - self._last_command_time > self.act_config.watchdog_timeout:
                logger.warning("Actuator watchdog timeout - disengaging clutch")
                self._clutch_engaged = False
                self._fault_code = FaultCode.WATCHDOG
                
        # Handle clutch engage delay
        if self._clutch_engaging:
            if now - self._clutch_engage_start > self.act_config.clutch_engage_delay:
                self._clutch_engaged = True
                self._clutch_engaging = False
                
        # Simulate position control
        if self._clutch_engaged:
            self._update_position(dt)
            
        # Update shared state with current rudder angle
        self.state['rudder_angle'] = self._actual_angle * self.act_config.max_rudder_angle
        
        # Compute current draw
        current = self._compute_current()
        
        # Send status
        self._send_status(current)
        
    def _process_commands(self):
        """Read and process incoming commands."""
        data = self.recv(1024)
        if data:
            self._input_buffer += data.decode('ascii', errors='ignore')
            
            # Process complete lines
            while '\n' in self._input_buffer:
                line, self._input_buffer = self._input_buffer.split('\n', 1)
                line = line.strip()
                if line:
                    self._handle_command(line)
                    
    def _handle_command(self, line: str):
        """Parse and execute a command."""
        # Verify checksum
        if not self.verify_checksum(line):
            logger.warning(f"Invalid checksum: {line}")
            return
            
        # Parse command
        try:
            # Remove $ prefix and *XX checksum
            content = line[1:line.index('*')]
            parts = content.split(',')
            
            if parts[0] == 'RUD' and len(parts) >= 3:
                target = float(parts[1])
                engage = int(parts[2])
                self._handle_rud_command(target, engage)
                
            elif parts[0] == 'CFG' and len(parts) >= 3:
                if parts[1] == 'TIMEOUT':
                    timeout_ms = int(parts[2])
                    self.act_config.watchdog_timeout = timeout_ms / 1000.0
                    logger.info(f"Watchdog timeout set to {timeout_ms}ms")
                    
            elif parts[0] == 'CAL':
                self._handle_cal_command(parts[1] if len(parts) > 1 else '')
                
        except Exception as e:
            logger.error(f"Command parse error: {e} for {line}")
            
    def _handle_rud_command(self, target: float, engage: int):
        """Handle rudder command."""
        now = time.time()
        self._last_command_time = now
        
        # Clamp target
        self._target_angle = max(-1.0, min(1.0, target))
        
        # Handle engage/disengage
        if engage and not self._clutch_engaged and not self._clutch_engaging:
            self._clutch_engaging = True
            self._clutch_engage_start = now
            self._fault_code = FaultCode.NONE  # Clear watchdog fault on new command
            logger.debug("Clutch engaging")
        elif not engage and (self._clutch_engaged or self._clutch_engaging):
            self._clutch_engaged = False
            self._clutch_engaging = False
            logger.debug("Clutch disengaged")
            
    def _handle_cal_command(self, cal_type: str):
        """Handle calibration command."""
        if cal_type == 'CENTER':
            logger.info("Calibration: CENTER position set")
        elif cal_type == 'PORT':
            logger.info("Calibration: PORT limit set")
        elif cal_type == 'STBD':
            logger.info("Calibration: STBD limit set")
            
    def _update_position(self, dt: float):
        """Update rudder position with rate limiting."""
        error = self._target_angle - self._actual_angle
        
        # Calculate max movement this step
        max_move = (self.act_config.max_rudder_rate / 
                   self.act_config.max_rudder_angle) * dt
        
        # Apply rate limit
        if abs(error) > max_move:
            move = max_move if error > 0 else -max_move
        else:
            move = error
            
        new_angle = self._actual_angle + move
        
        # Check position limits
        limit_normalized = self.act_config.position_limit / self.act_config.max_rudder_angle
        if abs(new_angle) > limit_normalized:
            new_angle = limit_normalized if new_angle > 0 else -limit_normalized
            if self._fault_code == FaultCode.NONE:
                self._fault_code = FaultCode.POSITION_LIMIT
                
        self._actual_angle = new_angle
        
    def _compute_current(self) -> float:
        """Compute simulated motor current."""
        if not self._clutch_engaged:
            return 0.0
            
        # Base current + load proportional to rudder angle and speed
        error = abs(self._target_angle - self._actual_angle)
        load = abs(self._actual_angle) + error * 2
        current = self.act_config.base_current + load * self.act_config.load_current_factor
        
        return min(current, 15.0)  # Cap at 15A
        
    def _send_status(self, current: float):
        """Send status message."""
        clutch_state = 1 if self._clutch_engaged else 0
        
        payload = (
            f"STS,{self._target_angle:.3f},{self._actual_angle:.3f},"
            f"{clutch_state},{self.act_config.supply_voltage:.1f},"
            f"{current:.1f},{int(self._fault_code)}"
        )
        checksum = self.compute_checksum(payload)
        message = f"${payload}*{checksum}\r\n"
        
        self.send(message.encode('ascii'))


def run_actuator_sim(
    shared_state: DictProxy, 
    config: Optional[ActuatorSimulatorConfig] = None
):
    """
    Entry point for running Actuator simulator as a separate process.
    
    Args:
        shared_state: Shared state dict from multiprocessing.Manager
        config: Optional configuration
    """
    import signal
    
    sim = ActuatorSimulator(shared_state, config)
    
    # Handle shutdown signals
    def shutdown(signum, frame):
        sim.stop()
        
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    
    if sim.start():
        logger.info("Actuator simulator running")
        try:
            while sim.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            sim.stop()
    else:
        logger.error("Failed to start Actuator simulator")
