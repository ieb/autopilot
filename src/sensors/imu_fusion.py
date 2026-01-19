"""
IMU Fusion Module
=================

Serial interface to ICM-20948 running on dedicated microcontroller.
The MCU runs Madgwick AHRS filter at 100Hz and sends fused orientation data.

Expected serial protocol from MCU:
    $IMU,<heading>,<pitch>,<roll>,<yaw_rate>,<pitch_rate>,<roll_rate>,<accel_x>,<accel_y>,<accel_z>*<checksum>\r\n
    
    All angles in degrees, rates in deg/s, accelerations in m/s²
"""

import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Callable
import serial
import logging

logger = logging.getLogger(__name__)


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
class IMUConfig:
    """Configuration for IMU serial connection."""
    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    timeout: float = 0.1
    max_age_ms: float = 200.0  # Max age before data considered stale


class IMUFusion:
    """
    Interface to external IMU processor over serial.
    
    The ICM-20948 is connected to a dedicated microcontroller (e.g., RP2040)
    that runs the sensor fusion algorithm. This class receives the fused
    orientation data over serial.
    """
    
    def __init__(self, config: Optional[IMUConfig] = None):
        self.config = config or IMUConfig()
        self._serial: Optional[serial.Serial] = None
        self._data = IMUData()
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._callbacks: list[Callable[[IMUData], None]] = []
        
        # Statistics
        self._msg_count = 0
        self._error_count = 0
        self._last_msg_time = 0.0
        
    def start(self) -> bool:
        """Start the IMU reader thread."""
        try:
            self._serial = serial.Serial(
                port=self.config.port,
                baudrate=self.config.baudrate,
                timeout=self.config.timeout
            )
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            logger.info(f"IMU started on {self.config.port}")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to open IMU serial port: {e}")
            return False
            
    def stop(self):
        """Stop the IMU reader thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._serial:
            self._serial.close()
            self._serial = None
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
        """Background thread reading serial data."""
        buffer = ""
        
        while self._running and self._serial:
            try:
                # Read available data
                if self._serial.in_waiting:
                    chunk = self._serial.read(self._serial.in_waiting).decode('ascii', errors='ignore')
                    buffer += chunk
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line.startswith('$IMU'):
                            self._parse_imu_message(line)
                else:
                    time.sleep(0.001)  # Avoid busy-waiting
                    
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
                    self._data.accel_z = float(parts[9].split('*')[0])  # Handle checksum
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
        return {
            "message_count": self._msg_count,
            "error_count": self._error_count,
            "last_message_age_ms": (time.time() - self._last_msg_time) * 1000 if self._last_msg_time else float('inf'),
            "connected": self._serial is not None and self._serial.is_open
        }


# Calibration command support
class IMUCalibration:
    """
    Magnetometer calibration helper.
    
    To calibrate, slowly rotate the boat through 360 degrees
    (or rotate the IMU through all orientations).
    """
    
    @staticmethod
    def send_calibration_start(imu: IMUFusion):
        """Send command to MCU to start magnetometer calibration."""
        if imu._serial and imu._serial.is_open:
            imu._serial.write(b"$CAL,START*XX\r\n")
            logger.info("Sent calibration start command")
            
    @staticmethod  
    def send_calibration_save(imu: IMUFusion):
        """Send command to MCU to save calibration to EEPROM."""
        if imu._serial and imu._serial.is_open:
            imu._serial.write(b"$CAL,SAVE*XX\r\n")
            logger.info("Sent calibration save command")
