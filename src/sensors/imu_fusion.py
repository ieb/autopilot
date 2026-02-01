"""
IMU Fusion Module
=================

Serial interface to ICM-20948 running on dedicated microcontroller.
The MCU runs Madgwick AHRS filter at 100Hz and sends fused orientation data.

Uses a three-phase simplex protocol:
1. STARTUP: Wait for $VER and $RDY from IMU
2. CONFIG: Send configuration (mag cal, offset) to IMU
3. STREAMING: Receive IMU data only (simplex mode)

Expected serial protocol from MCU:
    Startup: $VER,<version>*XX, $RDY*XX
    Config responses: $ACK*XX, $OFS,x,y,z*XX, $MGC,ox,oy,oz,sx,sy,sz*XX
    Streaming: $IMU,<heading>,<pitch>,<roll>,<yaw_rate>,<pitch_rate>,<roll_rate>,<ax>,<ay>,<az>*XX
    
All angles in degrees, rates in deg/s, accelerations in m/s²
"""

import json
import os
import threading
import time
import socket
from dataclasses import dataclass, field
from typing import Optional, Callable, Union, Tuple
from pathlib import Path
import logging

logger = logging.getLogger(__name__)

# Try to import serial (optional for socket-only mode)
try:
    import serial
    HAS_SERIAL = True
except ImportError:
    serial = None
    HAS_SERIAL = False


@dataclass
class IMUData:
    """Fused IMU orientation and rate data."""
    timestamp: float = 0.0
    
    # Orientation (degrees)
    heading: float = 0.0      # Magnetic heading, 0-360
    pitch: float = 0.0        # Bow up positive
    roll: float = 0.0         # Starboard down positive
    
    # Angular rates (deg/s)
    yaw_rate: float = 0.0     # Rate of turn
    pitch_rate: float = 0.0
    roll_rate: float = 0.0
    
    # Linear accelerations (m/s²)
    accel_x: float = 0.0      # Forward positive
    accel_y: float = 0.0      # Starboard positive  
    accel_z: float = 0.0      # Down positive
    
    # Status
    valid: bool = False
    age_ms: float = float('inf')
    
    def update_age(self, current_time: float):
        """Update the age of this reading."""
        self.age_ms = (current_time - self.timestamp) * 1000


@dataclass
class IMUCalibrationData:
    """Stored calibration data for IMU."""
    # Magnetometer calibration
    mag_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    mag_scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    
    # IMU mounting position offset from center of rotation (meters)
    position_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return {
            "mag_offset": list(self.mag_offset),
            "mag_scale": list(self.mag_scale),
            "position_offset": list(self.position_offset)
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'IMUCalibrationData':
        """Create from dictionary."""
        return cls(
            mag_offset=tuple(data.get("mag_offset", [0.0, 0.0, 0.0])),
            mag_scale=tuple(data.get("mag_scale", [1.0, 1.0, 1.0])),
            position_offset=tuple(data.get("position_offset", [0.0, 0.0, 0.0]))
        )


@dataclass  
class IMUConfig:
    """Configuration for IMU serial or socket connection."""
    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    timeout: float = 0.1
    max_age_ms: float = 200.0  # Max age before data considered stale
    use_socket: bool = False   # If True, connect via Unix socket instead of serial
    
    # Calibration storage
    calibration_file: Optional[str] = None  # Path to calibration JSON file
    
    # Startup timeout
    startup_timeout: float = 10.0  # Seconds to wait for $RDY


class IMUFusion:
    """
    Interface to external IMU processor over serial or Unix socket.
    
    The ICM-20948 is connected to a dedicated microcontroller that runs
    the sensor fusion algorithm. This class handles:
    1. Startup sequence (wait for $RDY, send configuration)
    2. Streaming mode (receive IMU data)
    
    Uses simplex protocol - after configuration, only receives data.
    """
    
    def __init__(self, config: Optional[IMUConfig] = None, 
                 calibration: Optional[IMUCalibrationData] = None):
        self.config = config or IMUConfig()
        self.calibration = calibration or IMUCalibrationData()
        
        self._conn: Optional[Union['serial.Serial', socket.socket]] = None
        self._conn_file: Optional[object] = None
        self._data = IMUData()
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._callbacks: list[Callable[[IMUData], None]] = []
        
        # Response waiting
        self._response_buffer: list[str] = []
        self._response_lock = threading.Lock()
        self._response_event = threading.Event()
        
        # Firmware version (received at startup)
        self.firmware_version: Optional[str] = None
        
        # Statistics
        self._msg_count = 0
        self._error_count = 0
        self._last_msg_time = 0.0
        
        # Load calibration from file if specified
        if self.config.calibration_file and not calibration:
            self._load_calibration()
    
    def _load_calibration(self):
        """Load calibration from JSON file."""
        if not self.config.calibration_file:
            return
        
        path = Path(self.config.calibration_file)
        if path.exists():
            try:
                with open(path, 'r') as f:
                    data = json.load(f)
                self.calibration = IMUCalibrationData.from_dict(data)
                logger.info(f"Loaded IMU calibration from {path}")
            except Exception as e:
                logger.warning(f"Failed to load IMU calibration: {e}")
    
    def save_calibration(self):
        """Save calibration to JSON file."""
        if not self.config.calibration_file:
            logger.warning("No calibration file configured")
            return False
        
        path = Path(self.config.calibration_file)
        try:
            # Ensure directory exists
            path.parent.mkdir(parents=True, exist_ok=True)
            
            with open(path, 'w') as f:
                json.dump(self.calibration.to_dict(), f, indent=2)
            logger.info(f"Saved IMU calibration to {path}")
            return True
        except Exception as e:
            logger.error(f"Failed to save IMU calibration: {e}")
            return False
        
    def start(self) -> bool:
        """
        Start the IMU interface.
        
        1. Connect to serial/socket
        2. Wait for $RDY from IMU
        3. Send configuration (offset, mag cal)
        4. Send $START to begin streaming
        5. Start background read thread
        
        Returns:
            True if startup sequence completed successfully
        """
        try:
            # Connect
            if not self._connect():
                return False
            
            # Wait for startup sequence
            if not self._wait_for_ready():
                logger.error("IMU did not send ready prompt")
                self._disconnect()
                return False
            
            # Send configuration
            self._send_configuration()
            
            # Start streaming
            self._send_command("START")
            if not self._wait_for_ack(timeout=2.0):
                logger.warning("No ACK received for START command")
            
            # Start read thread for streaming mode
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            
            logger.info("IMU started in streaming mode")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start IMU: {e}")
            self._disconnect()
            return False
    
    def _connect(self) -> bool:
        """Establish connection to IMU."""
        try:
            if self.config.use_socket:
                self._conn = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self._conn.settimeout(self.config.timeout)
                self._conn.connect(self.config.port)
                self._conn_file = self._conn.makefile('r', encoding='ascii', errors='ignore')
                logger.info(f"IMU connected via socket: {self.config.port}")
            else:
                if not HAS_SERIAL:
                    logger.error("pyserial not installed. Run: pip install pyserial")
                    return False
                self._conn = serial.Serial(
                    port=self.config.port,
                    baudrate=self.config.baudrate,
                    timeout=self.config.timeout
                )
                logger.info(f"IMU connected on serial: {self.config.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to IMU: {e}")
            return False
    
    def _disconnect(self):
        """Close connection to IMU."""
        if self._conn_file:
            try:
                self._conn_file.close()
            except Exception:
                pass
            self._conn_file = None
        if self._conn:
            try:
                self._conn.close()
            except Exception:
                pass
            self._conn = None
    
    def _wait_for_ready(self) -> bool:
        """Wait for $VER and $RDY messages from IMU."""
        start_time = time.time()
        got_version = False
        got_ready = False
        buffer = ""
        
        while time.time() - start_time < self.config.startup_timeout:
            try:
                line = self._read_line(timeout=0.5)
                if not line:
                    continue
                
                if line.startswith('$VER,'):
                    # Parse version
                    parts = line[1:].split('*')[0].split(',')
                    if len(parts) >= 2:
                        self.firmware_version = parts[1]
                        logger.info(f"IMU firmware version: {self.firmware_version}")
                    got_version = True
                    
                elif line.startswith('$RDY'):
                    got_ready = True
                    logger.info("IMU ready for configuration")
                    
                if got_ready:
                    return True
                    
            except Exception as e:
                logger.debug(f"Error reading startup message: {e}")
                time.sleep(0.1)
        
        return False
    
    def _send_configuration(self):
        """Send calibration configuration to IMU."""
        # Send position offset
        x, y, z = self.calibration.position_offset
        self._send_command(f"OFF,{x:.3f},{y:.3f},{z:.3f}")
        self._wait_for_response("OFS", timeout=2.0)
        
        # Send magnetometer calibration
        ox, oy, oz = self.calibration.mag_offset
        sx, sy, sz = self.calibration.mag_scale
        self._send_command(f"MAG,{ox:.2f},{oy:.2f},{oz:.2f},{sx:.3f},{sy:.3f},{sz:.3f}")
        self._wait_for_response("MGC", timeout=2.0)
        
        logger.info("IMU configuration sent")
    
    def _wait_for_ack(self, timeout: float = 2.0) -> bool:
        """Wait for $ACK response."""
        return self._wait_for_response("ACK", timeout) is not None
    
    def _wait_for_response(self, prefix: str, timeout: float = 2.0) -> Optional[str]:
        """Wait for a response with given prefix."""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            line = self._read_line(timeout=0.5)
            if line and line.startswith(f'${prefix}'):
                return line
            elif line and line.startswith('$ERR'):
                logger.error(f"IMU error: {line}")
                return None
        
        return None
    
    def _read_line(self, timeout: float = 0.5) -> Optional[str]:
        """Read a single line from the connection."""
        if self._conn is None:
            return None
            
        try:
            if self.config.use_socket:
                old_timeout = self._conn.gettimeout()
                self._conn.settimeout(timeout)
                try:
                    line = self._conn_file.readline()
                    if line:
                        return line.strip()
                except socket.timeout:
                    pass
                finally:
                    self._conn.settimeout(old_timeout)
            else:
                # Serial
                old_timeout = self._conn.timeout
                self._conn.timeout = timeout
                try:
                    line = self._conn.readline().decode('ascii', errors='ignore').strip()
                    if line:
                        return line
                finally:
                    self._conn.timeout = old_timeout
        except Exception as e:
            logger.debug(f"Read error: {e}")
        
        return None
            
    def stop(self):
        """Stop the IMU reader thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        self._disconnect()
        logger.info("IMU stopped")
        
    def get_data(self) -> IMUData:
        """Get the latest IMU data (thread-safe)."""
        with self._lock:
            data = IMUData(
                timestamp=self._data.timestamp,
                heading=self._data.heading,
                pitch=self._data.pitch,
                roll=self._data.roll,
                yaw_rate=self._data.yaw_rate,
                pitch_rate=self._data.pitch_rate,
                roll_rate=self._data.roll_rate,
                accel_x=self._data.accel_x,
                accel_y=self._data.accel_y,
                accel_z=self._data.accel_z,
                valid=self._data.valid
            )
            data.update_age(time.time())
            data.valid = data.age_ms < self.config.max_age_ms
            return data
            
    def add_callback(self, callback: Callable[[IMUData], None]):
        """Register a callback for new IMU data."""
        self._callbacks.append(callback)
        
    def _read_loop(self):
        """Background thread reading serial or socket data (streaming mode)."""
        buffer = ""
        
        while self._running and self._conn:
            try:
                if self.config.use_socket:
                    try:
                        line = self._conn_file.readline()
                        if line:
                            line = line.strip()
                            if line.startswith('$IMU'):
                                self._parse_imu_message(line)
                        else:
                            time.sleep(0.001)
                    except socket.timeout:
                        continue
                else:
                    # Serial
                    if hasattr(self._conn, 'in_waiting') and self._conn.in_waiting:
                        chunk = self._conn.read(self._conn.in_waiting).decode('ascii', errors='ignore')
                        buffer += chunk
                        
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            if line.startswith('$IMU'):
                                self._parse_imu_message(line)
                    else:
                        time.sleep(0.001)
                    
            except Exception as e:
                self._error_count += 1
                logger.warning(f"IMU read error: {e}")
                time.sleep(0.01)
                
    def _parse_imu_message(self, line: str):
        """Parse IMU NMEA-style message."""
        try:
            # Validate checksum if present
            if '*' in line:
                msg, checksum_str = line.rsplit('*', 1)
                expected_checksum = 0
                for char in msg[1:]:  # Skip leading $
                    expected_checksum ^= ord(char)
                if int(checksum_str, 16) != expected_checksum:
                    self._error_count += 1
                    return
            else:
                msg = line
                
            # Parse fields: $IMU,heading,pitch,roll,yaw_rate,pitch_rate,roll_rate,ax,ay,az
            parts = msg.split(',')
            if len(parts) >= 10:
                with self._lock:
                    self._data.timestamp = time.time()
                    self._data.heading = float(parts[1])
                    self._data.pitch = float(parts[2])
                    self._data.roll = float(parts[3])
                    self._data.yaw_rate = float(parts[4])
                    self._data.pitch_rate = float(parts[5])
                    self._data.roll_rate = float(parts[6])
                    self._data.accel_x = float(parts[7])
                    self._data.accel_y = float(parts[8])
                    self._data.accel_z = float(parts[9].split('*')[0])
                    self._data.valid = True
                    
                self._msg_count += 1
                self._last_msg_time = time.time()
                
                # Notify callbacks
                data = self.get_data()
                for callback in self._callbacks:
                    try:
                        callback(data)
                    except Exception as e:
                        logger.warning(f"IMU callback error: {e}")
                        
        except (ValueError, IndexError) as e:
            self._error_count += 1
            logger.debug(f"IMU parse error: {e}, line: {line}")
            
    @property
    def stats(self) -> dict:
        """Get statistics about IMU communication."""
        connected = False
        if self._conn is not None:
            if self.config.use_socket:
                connected = True
            elif hasattr(self._conn, 'is_open'):
                connected = self._conn.is_open
        return {
            "message_count": self._msg_count,
            "error_count": self._error_count,
            "last_message_age_ms": (time.time() - self._last_msg_time) * 1000 if self._last_msg_time else float('inf'),
            "connected": connected,
            "firmware_version": self.firmware_version
        }
    
    def _compute_checksum(self, payload: str) -> int:
        """Compute NMEA-style XOR checksum for a payload string."""
        checksum = 0
        for char in payload:
            checksum ^= ord(char)
        return checksum
    
    def _send_command(self, payload: str) -> bool:
        """Send a command to the MCU with proper checksum."""
        if self._conn is None:
            logger.warning("Cannot send command: not connected")
            return False
        
        checksum = self._compute_checksum(payload)
        cmd = f"${payload}*{checksum:02X}\r\n"
        
        try:
            if self.config.use_socket:
                self._conn.send(cmd.encode('ascii'))
            elif hasattr(self._conn, 'write'):
                self._conn.write(cmd.encode('ascii'))
            return True
        except Exception as e:
            logger.error(f"Failed to send command: {e}")
            return False


class IMUCalibration:
    """
    Magnetometer calibration helper.
    
    Performs interactive calibration during the IMU configuration phase.
    Slowly rotate the sensor through all orientations for 30 seconds.
    """
    
    @staticmethod
    def run_calibration(port: str, duration: float = 30.0, 
                        use_socket: bool = False) -> Optional[IMUCalibrationData]:
        """
        Run interactive magnetometer calibration.
        
        This connects to the IMU, starts calibration, waits for the specified
        duration, then retrieves the calibration values.
        
        Args:
            port: Serial port or socket path
            duration: Calibration duration in seconds
            use_socket: If True, connect via Unix socket
            
        Returns:
            IMUCalibrationData with mag calibration values, or None on failure
        """
        config = IMUConfig(port=port, use_socket=use_socket)
        
        try:
            # Connect
            if use_socket:
                conn = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                conn.settimeout(1.0)
                conn.connect(port)
                conn_file = conn.makefile('r', encoding='ascii', errors='ignore')
            else:
                if not HAS_SERIAL:
                    logger.error("pyserial not installed")
                    return None
                conn = serial.Serial(port=port, baudrate=config.baudrate, timeout=1.0)
                conn_file = None
            
            def send_cmd(payload: str):
                checksum = 0
                for char in payload:
                    checksum ^= ord(char)
                cmd = f"${payload}*{checksum:02X}\r\n"
                if use_socket:
                    conn.send(cmd.encode('ascii'))
                else:
                    conn.write(cmd.encode('ascii'))
            
            def read_line() -> Optional[str]:
                try:
                    if use_socket:
                        return conn_file.readline().strip()
                    else:
                        return conn.readline().decode('ascii', errors='ignore').strip()
                except Exception:
                    return None
            
            # Wait for RDY
            logger.info("Waiting for IMU ready...")
            start = time.time()
            while time.time() - start < 10.0:
                line = read_line()
                if line and line.startswith('$RDY'):
                    break
            else:
                logger.error("IMU did not send ready prompt")
                return None
            
            # Start calibration
            logger.info(f"Starting calibration - rotate sensor slowly for {duration} seconds")
            send_cmd("CAL,START")
            
            # Wait for ACK
            line = read_line()
            if not line or not line.startswith('$ACK'):
                logger.error("No ACK for calibration start")
                return None
            
            # Wait for calibration duration
            time.sleep(duration)
            
            # Stop calibration
            logger.info("Stopping calibration...")
            send_cmd("CAL,STOP")
            
            # Wait for MGC response
            start = time.time()
            while time.time() - start < 5.0:
                line = read_line()
                if line and line.startswith('$MGC,'):
                    # Parse: $MGC,ox,oy,oz,sx,sy,sz*XX
                    parts = line[1:].split('*')[0].split(',')
                    if len(parts) >= 7:
                        cal_data = IMUCalibrationData(
                            mag_offset=(float(parts[1]), float(parts[2]), float(parts[3])),
                            mag_scale=(float(parts[4]), float(parts[5]), float(parts[6]))
                        )
                        logger.info(f"Calibration complete: offset={cal_data.mag_offset}, scale={cal_data.mag_scale}")
                        conn.close()
                        return cal_data
            
            logger.error("No calibration response received")
            conn.close()
            return None
            
        except Exception as e:
            logger.error(f"Calibration failed: {e}")
            return None
