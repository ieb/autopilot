"""
Actuator Controller Interface
=============================

Serial interface to the Actuator Controller MCU (ATtiny3226).
Sends rudder target commands and receives status updates.

The Actuator Controller handles:
- Closed-loop rudder position control
- Electromagnetic clutch management
- Motor current monitoring
- Hardware safety limits (position, current, watchdog)

Serial Protocol:
    Commands (Pi → MCU):
        $RUD,<target>,<engage>*XX    - Set target angle, engage/disengage clutch
        $CFG,TIMEOUT,<ms>*XX         - Set watchdog timeout
        $CAL,CENTER*XX               - Calibrate center position
        $CAL,PORT*XX                 - Calibrate port limit
        $CAL,STBD*XX                 - Calibrate starboard limit

    Status (MCU → Pi at 20Hz):
        $STS,<target>,<actual>,<clutch>,<voltage>,<current>,<fault>*XX
"""

import threading
import time
import re
from dataclasses import dataclass
from typing import Optional, Callable
from enum import IntEnum
import logging

logger = logging.getLogger(__name__)

# Try to import serial
try:
    import serial
    HAS_SERIAL = True
except ImportError:
    serial = None
    HAS_SERIAL = False
    logger.warning("pyserial not available. Install with: pip install pyserial")


class MCUFaultCode(IntEnum):
    """Fault codes reported by Actuator Controller MCU."""
    NONE = 0
    OVERCURRENT = 1      # Motor current exceeded hard limit
    STALL = 2            # Motor stalled (current but no movement)
    POSITION_LIMIT = 3   # At mechanical position limit
    SENSOR = 4           # ADC read failure
    WATCHDOG = 5         # Command timeout triggered


@dataclass
class ActuatorStatus:
    """Status received from Actuator Controller."""
    timestamp: float = 0.0
    target_angle: float = 0.0      # Commanded angle (normalized -1 to +1)
    actual_angle: float = 0.0      # Current angle (normalized -1 to +1)
    clutch_engaged: bool = False
    voltage: float = 0.0           # Supply voltage (V)
    current: float = 0.0           # Motor current (A)
    fault_code: MCUFaultCode = MCUFaultCode.NONE
    valid: bool = False
    
    def get_angle_degrees(self, max_angle: float = 30.0) -> float:
        """Get actual angle in degrees."""
        return self.actual_angle * max_angle
    
    def get_target_degrees(self, max_angle: float = 30.0) -> float:
        """Get target angle in degrees."""
        return self.target_angle * max_angle


@dataclass
class ActuatorConfig:
    """Configuration for actuator interface."""
    port: str = "/dev/ttyAMA1"
    baudrate: int = 115200
    timeout: float = 0.1
    
    # Watchdog timeout on MCU (ms)
    watchdog_timeout_ms: int = 2000
    
    # Status validity
    max_status_age_s: float = 0.2   # Max age before status considered stale
    
    # Command rate limiting
    min_command_interval_s: float = 0.02  # 50Hz max command rate


class ActuatorInterface:
    """
    Serial interface to Actuator Controller MCU.
    
    Replaces direct PWM control - sends target angles over serial,
    receives status updates including rudder position, clutch state,
    and motor current.
    
    The MCU handles closed-loop position control internally.
    """
    
    def __init__(self, config: Optional[ActuatorConfig] = None):
        self.config = config or ActuatorConfig()
        self._serial: Optional[serial.Serial] = None
        self._status = ActuatorStatus()
        self._lock = threading.Lock()
        self._running = False
        self._read_thread: Optional[threading.Thread] = None
        self._last_command_time = 0.0
        
        # Callbacks
        self._status_callback: Optional[Callable[[ActuatorStatus], None]] = None
        self._fault_callback: Optional[Callable[[MCUFaultCode], None]] = None
        
        # Statistics
        self._commands_sent = 0
        self._status_received = 0
        self._parse_errors = 0
        
    def start(self) -> bool:
        """Open serial connection and start status reader thread."""
        if not HAS_SERIAL:
            logger.error("pyserial not available")
            return False
            
        try:
            self._serial = serial.Serial(
                port=self.config.port,
                baudrate=self.config.baudrate,
                timeout=self.config.timeout
            )
            
            # Clear any stale data
            self._serial.reset_input_buffer()
            
            # Start reader thread
            self._running = True
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            
            # Configure watchdog timeout on MCU
            self._send_raw(f"CFG,TIMEOUT,{self.config.watchdog_timeout_ms}")
            
            logger.info(f"Actuator interface started on {self.config.port}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to open serial port: {e}")
            return False
            
    def stop(self):
        """Disengage clutch and stop interface."""
        # Send disengage command
        self.disengage()
        
        self._running = False
        if self._read_thread:
            self._read_thread.join(timeout=1.0)
            
        if self._serial:
            self._serial.close()
            self._serial = None
            
        logger.info("Actuator interface stopped")
        
    def send_command(self, target_normalized: float, engage: bool = True):
        """
        Send rudder target command to Actuator Controller.
        
        Args:
            target_normalized: Target angle [-1, 1] where ±1 = ±30°
            engage: True to engage clutch, False to disengage
        """
        # Rate limit commands
        now = time.time()
        if now - self._last_command_time < self.config.min_command_interval_s:
            return
        self._last_command_time = now
        
        # Clamp target
        target = max(-1.0, min(1.0, target_normalized))
        
        # Send command
        self._send_raw(f"RUD,{target:.3f},{1 if engage else 0}")
        self._commands_sent += 1
        
    def disengage(self):
        """Immediately disengage clutch (emergency or standby)."""
        self._send_raw("RUD,0.000,0")
        logger.info("Clutch disengage commanded")
        
    def calibrate_center(self):
        """Calibrate current position as center (0°)."""
        self._send_raw("CAL,CENTER")
        logger.info("Calibrating center position")
        
    def calibrate_port(self):
        """Calibrate current position as port limit."""
        self._send_raw("CAL,PORT")
        logger.info("Calibrating port limit")
        
    def calibrate_starboard(self):
        """Calibrate current position as starboard limit."""
        self._send_raw("CAL,STBD")
        logger.info("Calibrating starboard limit")
        
    def get_status(self) -> ActuatorStatus:
        """
        Get latest status from Actuator Controller.
        
        Returns:
            ActuatorStatus with valid=False if data is stale
        """
        with self._lock:
            status = ActuatorStatus(
                timestamp=self._status.timestamp,
                target_angle=self._status.target_angle,
                actual_angle=self._status.actual_angle,
                clutch_engaged=self._status.clutch_engaged,
                voltage=self._status.voltage,
                current=self._status.current,
                fault_code=self._status.fault_code,
                valid=self._status.valid
            )
            
        # Check if status is stale
        age = time.time() - status.timestamp
        if age > self.config.max_status_age_s:
            status.valid = False
            
        return status
        
    def set_status_callback(self, callback: Callable[[ActuatorStatus], None]):
        """Set callback for status updates."""
        self._status_callback = callback
        
    def set_fault_callback(self, callback: Callable[[MCUFaultCode], None]):
        """Set callback for fault notifications."""
        self._fault_callback = callback
        
    def _send_raw(self, payload: str):
        """Send raw command with checksum."""
        if not self._serial or not self._serial.is_open:
            return
            
        checksum = self._compute_checksum(payload)
        message = f"${payload}*{checksum:02X}\r\n"
        
        try:
            self._serial.write(message.encode('ascii'))
        except Exception as e:
            logger.warning(f"Failed to send command: {e}")
            
    def _compute_checksum(self, payload: str) -> int:
        """Compute XOR checksum of payload."""
        checksum = 0
        for c in payload:
            checksum ^= ord(c)
        return checksum
        
    def _read_loop(self):
        """Background thread reading status messages."""
        buffer = ""
        
        while self._running:
            try:
                if self._serial and self._serial.in_waiting:
                    data = self._serial.read(self._serial.in_waiting).decode('ascii', errors='ignore')
                    buffer += data
                    
                    # Process complete messages
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line.startswith('$STS,'):
                            self._parse_status(line)
                else:
                    time.sleep(0.01)
                    
            except Exception as e:
                logger.warning(f"Read error: {e}")
                time.sleep(0.1)
                
    def _parse_status(self, message: str):
        """Parse status message from MCU."""
        # Format: $STS,<target>,<actual>,<clutch>,<voltage>,<current>,<fault>*XX
        try:
            # Remove $ and verify checksum
            if not message.startswith('$'):
                return
                
            if '*' not in message:
                self._parse_errors += 1
                return
                
            payload, checksum_str = message[1:].rsplit('*', 1)
            expected_checksum = int(checksum_str, 16)
            actual_checksum = self._compute_checksum(payload)
            
            if expected_checksum != actual_checksum:
                self._parse_errors += 1
                logger.debug(f"Checksum mismatch: expected {expected_checksum:02X}, got {actual_checksum:02X}")
                return
                
            # Parse fields
            parts = payload.split(',')
            if len(parts) != 7 or parts[0] != 'STS':
                self._parse_errors += 1
                return
                
            target = float(parts[1])
            actual = float(parts[2])
            clutch = int(parts[3]) == 1
            voltage = float(parts[4])
            current = float(parts[5])
            fault = MCUFaultCode(int(parts[6]))
            
            # Update status
            with self._lock:
                old_fault = self._status.fault_code
                
                self._status.timestamp = time.time()
                self._status.target_angle = target
                self._status.actual_angle = actual
                self._status.clutch_engaged = clutch
                self._status.voltage = voltage
                self._status.current = current
                self._status.fault_code = fault
                self._status.valid = True
                
            self._status_received += 1
            
            # Notify callbacks
            if self._status_callback:
                self._status_callback(self._status)
                
            if fault != MCUFaultCode.NONE and fault != old_fault:
                logger.warning(f"MCU fault: {fault.name}")
                if self._fault_callback:
                    self._fault_callback(fault)
                    
        except (ValueError, IndexError) as e:
            self._parse_errors += 1
            logger.debug(f"Failed to parse status: {e}")
            
    @property
    def stats(self) -> dict:
        """Get interface statistics."""
        return {
            "commands_sent": self._commands_sent,
            "status_received": self._status_received,
            "parse_errors": self._parse_errors
        }


class MockActuatorInterface(ActuatorInterface):
    """Mock actuator interface for testing without hardware."""
    
    def __init__(self, config: Optional[ActuatorConfig] = None):
        super().__init__(config)
        self._mock_position = 0.0
        self._mock_target = 0.0
        self._mock_clutch = False
        
    def start(self) -> bool:
        """Start mock interface."""
        self._running = True
        self._read_thread = threading.Thread(target=self._mock_loop, daemon=True)
        self._read_thread.start()
        logger.info("Mock actuator interface started")
        return True
        
    def stop(self):
        """Stop mock interface."""
        self._running = False
        if self._read_thread:
            self._read_thread.join(timeout=1.0)
        logger.info("Mock actuator interface stopped")
        
    def send_command(self, target_normalized: float, engage: bool = True):
        """Process mock command."""
        self._mock_target = max(-1.0, min(1.0, target_normalized))
        self._mock_clutch = engage
        self._commands_sent += 1
        
    def _mock_loop(self):
        """Simulate actuator behavior."""
        while self._running:
            # Simulate rudder movement toward target
            if self._mock_clutch:
                error = self._mock_target - self._mock_position
                # Move at ~4°/s = 0.133 normalized/s
                rate = 0.133 * 0.05  # 50Hz update
                if abs(error) > rate:
                    self._mock_position += rate if error > 0 else -rate
                else:
                    self._mock_position = self._mock_target
                    
            # Update status
            with self._lock:
                self._status.timestamp = time.time()
                self._status.target_angle = self._mock_target
                self._status.actual_angle = self._mock_position
                self._status.clutch_engaged = self._mock_clutch
                self._status.voltage = 12.6
                self._status.current = 0.5 if self._mock_clutch and abs(self._mock_target - self._mock_position) > 0.01 else 0.0
                self._status.fault_code = MCUFaultCode.NONE
                self._status.valid = True
                
            self._status_received += 1
            time.sleep(0.05)  # 20Hz
