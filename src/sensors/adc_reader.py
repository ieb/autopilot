"""
ADC Reader Module
=================

.. deprecated::
    This module is deprecated as of the Actuator Controller architecture update.
    Rudder position is now read by the Actuator Controller MCU (ATtiny3226) and
    reported via serial to the Pi through ActuatorInterface.
    
    See: src/control/actuator_interface.py
    
    This module is retained for reference and potential use with other ADC
    sensors, but is no longer used for rudder position sensing.

Original description:
    Reads rudder position potentiometer via ADS1115 ADC over I2C.
    Provides calibrated rudder angle in degrees.
"""

import warnings
warnings.warn(
    "adc_reader is deprecated. Rudder position is now read by the Actuator Controller MCU. "
    "Use actuator_interface.ActuatorInterface.get_status().actual_angle instead.",
    DeprecationWarning,
    stacklevel=2
)

import threading
import time
from dataclasses import dataclass
from typing import Optional
import logging

logger = logging.getLogger(__name__)

# Try to import ADS1115 library
try:
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    HAS_ADS1115 = True
except ImportError:
    HAS_ADS1115 = False
    logger.warning("ADS1115 library not available. Install with: pip install adafruit-circuitpython-ads1x15")


@dataclass
class RudderConfig:
    """Configuration for rudder position sensor."""
    # ADC channel (0-3)
    adc_channel: int = 0
    
    # Calibration: ADC values at known positions
    adc_center: int = 16384      # ADC value at rudder center (midpoint of 16-bit)
    adc_port_limit: int = 6000   # ADC value at max port (-30°)
    adc_starboard_limit: int = 26000  # ADC value at max starboard (+30°)
    
    # Physical limits
    max_angle_deg: float = 25.0  # Max rudder angle each side
    
    # Filtering
    filter_alpha: float = 0.3    # Low-pass filter coefficient (0-1, lower = more smoothing)
    
    # Update rate
    sample_rate_hz: float = 50.0  # Target sample rate
    
    # Validity
    max_age_ms: float = 100.0    # Max age before considered stale


@dataclass
class RudderData:
    """Rudder position data."""
    timestamp: float = 0.0
    angle_deg: float = 0.0       # Positive = starboard, negative = port
    raw_adc: int = 0
    valid: bool = False
    
    def get_angle_normalized(self, max_angle: float = 30.0) -> float:
        """Get angle normalized to [-1, 1]."""
        return self.angle_deg / max_angle


class ADCReader:
    """
    Reads rudder potentiometer position via ADS1115 ADC.
    
    The potentiometer is connected to the rudder stock or tiller,
    providing position feedback for the control loop.
    """
    
    def __init__(self, config: Optional[RudderConfig] = None):
        self.config = config or RudderConfig()
        self._adc = None
        self._channel = None
        self._data = RudderData()
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._filtered_value: float = 0.0
        
        # Pre-calculate calibration slope
        self._update_calibration()
        
    def _update_calibration(self):
        """Calculate calibration coefficients."""
        # Port side slope (negative angles)
        self._port_slope = -self.config.max_angle_deg / (self.config.adc_center - self.config.adc_port_limit)
        # Starboard side slope (positive angles)  
        self._starboard_slope = self.config.max_angle_deg / (self.config.adc_starboard_limit - self.config.adc_center)
        
    def start(self) -> bool:
        """Start the ADC reader thread."""
        if not HAS_ADS1115:
            logger.error("ADS1115 library not available")
            return False
            
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self._adc = ADS.ADS1115(i2c)
            self._adc.gain = 1  # ±4.096V range
            self._adc.data_rate = 128  # 128 SPS
            
            # Select channel
            channel_map = [ADS.P0, ADS.P1, ADS.P2, ADS.P3]
            self._channel = AnalogIn(self._adc, channel_map[self.config.adc_channel])
            
            # Initialize filter
            self._filtered_value = self._channel.value
            
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            logger.info("ADC reader started")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize ADC: {e}")
            return False
            
    def stop(self):
        """Stop the ADC reader thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        logger.info("ADC reader stopped")
        
    def get_data(self) -> RudderData:
        """Get current rudder position (thread-safe)."""
        with self._lock:
            data = RudderData(
                timestamp=self._data.timestamp,
                angle_deg=self._data.angle_deg,
                raw_adc=self._data.raw_adc,
                valid=self._data.valid
            )
            # Check age
            age_ms = (time.time() - data.timestamp) * 1000
            data.valid = age_ms < self.config.max_age_ms
            return data
            
    def _read_loop(self):
        """Background thread reading ADC."""
        interval = 1.0 / self.config.sample_rate_hz
        
        while self._running:
            try:
                start_time = time.time()
                
                # Read raw ADC value
                raw_value = self._channel.value
                
                # Apply low-pass filter
                self._filtered_value = (
                    self.config.filter_alpha * raw_value +
                    (1 - self.config.filter_alpha) * self._filtered_value
                )
                
                # Convert to angle
                angle = self._raw_to_angle(int(self._filtered_value))
                
                # Update data
                with self._lock:
                    self._data.timestamp = time.time()
                    self._data.raw_adc = raw_value
                    self._data.angle_deg = angle
                    self._data.valid = True
                    
                # Maintain sample rate
                elapsed = time.time() - start_time
                if elapsed < interval:
                    time.sleep(interval - elapsed)
                    
            except Exception as e:
                logger.warning(f"ADC read error: {e}")
                time.sleep(0.1)
                
    def _raw_to_angle(self, raw: int) -> float:
        """Convert raw ADC value to angle in degrees."""
        if raw < self.config.adc_center:
            # Port side
            offset = self.config.adc_center - raw
            angle = -offset * (-self._port_slope)
        else:
            # Starboard side
            offset = raw - self.config.adc_center
            angle = offset * self._starboard_slope
            
        # Clamp to valid range
        return max(-self.config.max_angle_deg, min(self.config.max_angle_deg, angle))
        
    def calibrate_center(self):
        """Set current position as center (0 degrees)."""
        if self._channel:
            self.config.adc_center = self._channel.value
            self._update_calibration()
            logger.info(f"Rudder center calibrated to ADC value: {self.config.adc_center}")
            
    def calibrate_port_limit(self):
        """Set current position as port limit."""
        if self._channel:
            self.config.adc_port_limit = self._channel.value
            self._update_calibration()
            logger.info(f"Rudder port limit calibrated to ADC value: {self.config.adc_port_limit}")
            
    def calibrate_starboard_limit(self):
        """Set current position as starboard limit."""
        if self._channel:
            self.config.adc_starboard_limit = self._channel.value
            self._update_calibration()
            logger.info(f"Rudder starboard limit calibrated to ADC value: {self.config.adc_starboard_limit}")


class MockADCReader(ADCReader):
    """Mock ADC reader for testing without hardware."""
    
    def __init__(self, config: Optional[RudderConfig] = None):
        super().__init__(config)
        self._mock_angle = 0.0
        
    def start(self) -> bool:
        """Start mock reader."""
        self._running = True
        self._thread = threading.Thread(target=self._mock_loop, daemon=True)
        self._thread.start()
        logger.info("Mock ADC reader started")
        return True
        
    def _mock_loop(self):
        """Generate mock rudder data."""
        while self._running:
            with self._lock:
                self._data.timestamp = time.time()
                self._data.angle_deg = self._mock_angle
                self._data.raw_adc = int(self.config.adc_center + self._mock_angle * 333)  # ~333 counts per degree
                self._data.valid = True
            time.sleep(1.0 / self.config.sample_rate_hz)
            
    def set_mock_angle(self, angle: float):
        """Set the mock rudder angle for testing."""
        self._mock_angle = max(-30.0, min(30.0, angle))
