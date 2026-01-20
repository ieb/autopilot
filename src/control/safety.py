"""
Safety Layer Module
===================

Two-layer safety architecture:

Layer 1 - Hardware Protection (Actuator Controller MCU):
    - Position limits (±28° software, ±30° mechanical)
    - Current limiting (12A soft, 15A hard)
    - Stall detection
    - Watchdog timeout (2s)
    
    These are enforced by the MCU regardless of Pi commands.

Layer 2 - System Safety (this module, Raspberry Pi):
    - Heading deviation monitoring
    - Sensor validity checking
    - Actuator communication monitoring
    - Manual override detection
    - Mode management
    
    These require the full sensor context that only the Pi has.
"""

import time
from dataclasses import dataclass
from enum import IntEnum, auto
from typing import Optional, Tuple, TYPE_CHECKING
import logging

if TYPE_CHECKING:
    from .actuator_interface import ActuatorInterface, ActuatorStatus

logger = logging.getLogger(__name__)


class AlarmCode(IntEnum):
    """System safety alarm codes (Pi-side, Layer 2)."""
    OK = 0
    HEADING_DEVIATION = auto()      # Off course > 45° for > 3s
    SENSOR_TIMEOUT = auto()         # IMU/NMEA data stale
    ACTUATOR_TIMEOUT = auto()       # No status from Actuator Controller
    MCU_FAULT = auto()              # MCU reported hardware fault
    MANUAL_OVERRIDE = auto()        # Human grabbed the helm
    EMERGENCY_STOP = auto()         # Manual emergency stop triggered
    RUDDER_LIMIT = auto()           # Approaching rudder limit (warning)
    RATE_LIMIT = auto()             # Rate limiting active (warning)


@dataclass
class SafetyConfig:
    """Safety layer configuration."""
    # Rudder limits
    rudder_limit_deg: float = 28.0       # Software limit (hardware is 30°)
    rudder_warning_deg: float = 25.0     # Warning when approaching limit
    
    # Rate limits  
    max_rudder_rate: float = 5.0         # deg/s max change rate
    
    # Heading deviation
    max_heading_error: float = 45.0      # Disengage if exceeded
    heading_warning: float = 30.0        # Warning threshold
    
    # Sensor validity
    imu_max_age_ms: float = 200.0        # IMU data max age
    rudder_max_age_ms: float = 100.0     # Rudder position max age
    
    # Override detection
    override_threshold_deg: float = 5.0  # Difference to detect human override
    override_time_s: float = 1.0         # Duration to confirm override
    
    # Output filtering
    output_filter_alpha: float = 0.3     # Low-pass filter for smoothness


@dataclass  
class SafetyState:
    """Current safety system state."""
    alarm_code: AlarmCode = AlarmCode.OK
    alarm_message: str = ""
    is_safe: bool = True
    output_limited: bool = False
    original_output: float = 0.0
    safe_output: float = 0.0


class SafetyLayer:
    """
    Safety layer between ML model and rudder actuator.
    
    Enforces hard limits and sanity checks:
    - Rudder position limits
    - Rudder rate limits  
    - Heading deviation monitoring
    - Sensor validity checking
    - Manual override detection
    - Output smoothing
    """
    
    def __init__(self, config: Optional[SafetyConfig] = None):
        self.config = config or SafetyConfig()
        self._state = SafetyState()
        
        # State tracking
        self._last_output = 0.0
        self._last_time = 0.0
        self._filtered_output = 0.0
        
        # Override detection
        self._override_start = 0.0
        self._in_override = False
        
        # Statistics
        self._limit_count = 0
        self._override_count = 0
        
    def validate(self, ml_output: float, 
                 heading_error: float,
                 rudder_position: float,
                 imu_age_ms: float,
                 rudder_age_ms: float,
                 commanded_position: float) -> Tuple[float, SafetyState]:
        """
        Validate and potentially modify ML output.
        
        Args:
            ml_output: Raw ML model output [-1, 1]
            heading_error: Current heading error (degrees)
            rudder_position: Current rudder position (degrees)
            imu_age_ms: Age of IMU data (milliseconds)
            rudder_age_ms: Age of rudder position data (milliseconds)
            commanded_position: Last commanded rudder position (degrees)
            
        Returns:
            (safe_output, state): Validated output and safety state
        """
        now = time.time()
        dt = now - self._last_time if self._last_time > 0 else 0.1
        self._last_time = now
        
        # Reset state
        self._state.original_output = ml_output
        self._state.alarm_code = AlarmCode.OK
        self._state.alarm_message = ""
        self._state.output_limited = False
        
        output = ml_output
        
        # 1. Check sensor validity
        if imu_age_ms > self.config.imu_max_age_ms:
            self._state.alarm_code = AlarmCode.SENSOR_TIMEOUT
            self._state.alarm_message = f"IMU data stale ({imu_age_ms:.0f}ms)"
            self._state.is_safe = False
            self._state.safe_output = 0.0
            return (0.0, self._state)
            
        if rudder_age_ms > self.config.rudder_max_age_ms:
            self._state.alarm_code = AlarmCode.SENSOR_TIMEOUT
            self._state.alarm_message = f"Rudder data stale ({rudder_age_ms:.0f}ms)"
            self._state.is_safe = False
            self._state.safe_output = 0.0
            return (0.0, self._state)
            
        # 2. Check heading deviation
        if abs(heading_error) > self.config.max_heading_error:
            self._state.alarm_code = AlarmCode.HEADING_DEVIATION
            self._state.alarm_message = f"Heading deviation {heading_error:.1f}°"
            self._state.is_safe = False
            self._state.safe_output = 0.0
            return (0.0, self._state)
            
        if abs(heading_error) > self.config.heading_warning:
            self._state.alarm_code = AlarmCode.HEADING_DEVIATION
            self._state.alarm_message = f"Heading warning {heading_error:.1f}°"
            # Don't return, just warn
            
        # 3. Clamp rudder position
        max_normalized = self.config.rudder_limit_deg / 30.0  # Convert to normalized
        if abs(output) > max_normalized:
            output = max(-max_normalized, min(max_normalized, output))
            self._state.output_limited = True
            self._limit_count += 1
            if self._state.alarm_code == AlarmCode.OK:
                self._state.alarm_code = AlarmCode.RUDDER_LIMIT
                self._state.alarm_message = "Rudder limit"
                
        # 4. Rate limit
        max_rate_normalized = (self.config.max_rudder_rate / 30.0) * dt
        rate = output - self._last_output
        if abs(rate) > max_rate_normalized:
            output = self._last_output + max_rate_normalized * (1 if rate > 0 else -1)
            self._state.output_limited = True
            if self._state.alarm_code == AlarmCode.OK:
                self._state.alarm_code = AlarmCode.RATE_LIMIT
                self._state.alarm_message = "Rate limited"
                
        # 5. Check for manual override
        # If actual rudder position differs significantly from commanded
        position_error = abs(rudder_position - commanded_position)
        if position_error > self.config.override_threshold_deg:
            if not self._in_override:
                self._override_start = now
                self._in_override = True
            elif now - self._override_start > self.config.override_time_s:
                self._state.alarm_code = AlarmCode.MANUAL_OVERRIDE
                self._state.alarm_message = "Manual override detected"
                self._state.is_safe = False
                self._override_count += 1
                self._state.safe_output = 0.0
                return (0.0, self._state)
        else:
            self._in_override = False
            self._override_start = 0
            
        # 6. Low-pass filter for smoothness
        self._filtered_output = (
            self.config.output_filter_alpha * output +
            (1 - self.config.output_filter_alpha) * self._filtered_output
        )
        output = self._filtered_output
        
        # 7. Update state
        self._last_output = output
        self._state.safe_output = output
        self._state.is_safe = True
        
        return (output, self._state)
        
    def reset(self):
        """Reset safety layer state."""
        self._last_output = 0.0
        self._filtered_output = 0.0
        self._in_override = False
        self._override_start = 0.0
        self._state = SafetyState()
        logger.info("Safety layer reset")
        
    def get_state(self) -> SafetyState:
        """Get current safety state."""
        return self._state
        
    @property
    def stats(self) -> dict:
        """Get safety statistics."""
        return {
            "limit_count": self._limit_count,
            "override_count": self._override_count
        }


class SystemSafety:
    """
    System-level safety for autopilot operation (Layer 2).
    
    This class handles autopilot-level safety decisions that require
    the full sensor context. It works with the ActuatorInterface to
    monitor system health and command clutch disengagement when needed.
    
    The MCU (Layer 1) handles hardware protection independently.
    """
    
    # Thresholds
    MAX_HEADING_ERROR = 45.0        # degrees
    HEADING_ERROR_TIMEOUT = 3.0     # seconds before alarm
    SENSOR_TIMEOUT = 0.5            # seconds - IMU/NMEA data max age
    ACTUATOR_TIMEOUT = 0.2          # seconds - actuator status max age
    OVERRIDE_THRESHOLD = 5.0        # degrees - position error to detect override
    OVERRIDE_TIMEOUT = 1.0          # seconds - duration to confirm override
    
    # Rate limiting (applied before sending to MCU)
    MAX_RUDDER_RATE = 5.0           # deg/s
    RUDDER_LIMIT_DEG = 28.0         # software limit
    
    def __init__(self, actuator: 'ActuatorInterface'):
        """
        Initialize system safety.
        
        Args:
            actuator: ActuatorInterface for sending disengage commands
        """
        self.actuator = actuator
        
        # State tracking
        self._last_output = 0.0
        self._last_time = time.time()
        self._heading_error_start: Optional[float] = None
        self._override_start: Optional[float] = None
        
        # Current alarm state
        self._alarm_code = AlarmCode.OK
        self._alarm_message = ""
        
    def check(self, 
              ml_output: float,
              heading_error: float,
              imu_age: float,
              actuator_status: 'ActuatorStatus') -> Tuple[float, bool, AlarmCode]:
        """
        Check system-level safety conditions and apply rate limiting.
        
        Args:
            ml_output: Raw ML model output [-1, 1]
            heading_error: Current heading error in degrees
            imu_age: Age of IMU data in seconds
            actuator_status: Latest status from Actuator Controller
            
        Returns:
            (safe_output, engage, alarm_code):
                safe_output: Rate-limited output [-1, 1]
                engage: True if clutch should remain engaged
                alarm_code: Current alarm code
        """
        now = time.time()
        dt = now - self._last_time
        self._last_time = now
        
        # Start with OK
        self._alarm_code = AlarmCode.OK
        self._alarm_message = ""
        
        # 1. Check actuator communication
        if not actuator_status.valid:
            self._alarm_code = AlarmCode.ACTUATOR_TIMEOUT
            self._alarm_message = "Actuator status invalid"
            return (0.0, False, self._alarm_code)
            
        status_age = now - actuator_status.timestamp
        if status_age > self.ACTUATOR_TIMEOUT:
            self._alarm_code = AlarmCode.ACTUATOR_TIMEOUT
            self._alarm_message = f"Actuator status stale ({status_age:.2f}s)"
            return (0.0, False, self._alarm_code)
            
        # 2. Check for MCU-reported faults
        if actuator_status.fault_code != 0:
            self._alarm_code = AlarmCode.MCU_FAULT
            self._alarm_message = f"MCU fault: {actuator_status.fault_code}"
            return (0.0, False, self._alarm_code)
            
        # 3. Check sensor validity (IMU)
        if imu_age > self.SENSOR_TIMEOUT:
            self._alarm_code = AlarmCode.SENSOR_TIMEOUT
            self._alarm_message = f"IMU data stale ({imu_age:.2f}s)"
            return (0.0, False, self._alarm_code)
            
        # 4. Heading error monitoring (with timeout before alarm)
        if abs(heading_error) > self.MAX_HEADING_ERROR:
            if self._heading_error_start is None:
                self._heading_error_start = now
            elif now - self._heading_error_start > self.HEADING_ERROR_TIMEOUT:
                self._alarm_code = AlarmCode.HEADING_DEVIATION
                self._alarm_message = f"Off course {heading_error:.1f}° for {self.HEADING_ERROR_TIMEOUT}s"
                return (0.0, False, self._alarm_code)
        else:
            self._heading_error_start = None
            
        # 5. Manual override detection
        position_error_deg = abs(actuator_status.actual_angle - actuator_status.target_angle) * 30.0
        if position_error_deg > self.OVERRIDE_THRESHOLD:
            if self._override_start is None:
                self._override_start = now
            elif now - self._override_start > self.OVERRIDE_TIMEOUT:
                self._alarm_code = AlarmCode.MANUAL_OVERRIDE
                self._alarm_message = "Manual override detected"
                return (0.0, False, self._alarm_code)
        else:
            self._override_start = None
            
        # 6. Apply rate limiting to ML output
        output = ml_output
        
        # Clamp to position limits
        max_normalized = self.RUDDER_LIMIT_DEG / 30.0
        if abs(output) > max_normalized:
            output = max(-max_normalized, min(max_normalized, output))
            if self._alarm_code == AlarmCode.OK:
                self._alarm_code = AlarmCode.RUDDER_LIMIT
                self._alarm_message = "Rudder limit applied"
                
        # Rate limit
        max_rate_normalized = (self.MAX_RUDDER_RATE / 30.0) * dt if dt > 0 else 0.1
        rate = output - self._last_output
        if abs(rate) > max_rate_normalized:
            output = self._last_output + max_rate_normalized * (1 if rate > 0 else -1)
            if self._alarm_code == AlarmCode.OK:
                self._alarm_code = AlarmCode.RATE_LIMIT
                self._alarm_message = "Rate limit applied"
                
        self._last_output = output
        
        # All checks passed - keep clutch engaged
        return (output, True, self._alarm_code)
        
    def disengage(self):
        """Send disengage command to actuator."""
        self.actuator.disengage()
        logger.warning("System safety: clutch disengaged")
        
    def reset(self):
        """Reset safety state."""
        self._last_output = 0.0
        self._heading_error_start = None
        self._override_start = None
        self._alarm_code = AlarmCode.OK
        self._alarm_message = ""
        logger.info("System safety reset")
        
    @property
    def alarm_code(self) -> AlarmCode:
        """Get current alarm code."""
        return self._alarm_code
        
    @property
    def alarm_message(self) -> str:
        """Get current alarm message."""
        return self._alarm_message


class EmergencyStop:
    """
    Emergency stop handler.
    
    Can be triggered by external conditions or user input.
    Immediately stops rudder movement and disengages autopilot.
    """
    
    def __init__(self):
        self._triggered = False
        self._trigger_time = 0.0
        self._trigger_reason = ""
        
    def trigger(self, reason: str = "Manual"):
        """Trigger emergency stop."""
        self._triggered = True
        self._trigger_time = time.time()
        self._trigger_reason = reason
        logger.critical(f"EMERGENCY STOP: {reason}")
        
    def reset(self):
        """Reset emergency stop (requires manual confirmation)."""
        self._triggered = False
        self._trigger_time = 0.0
        self._trigger_reason = ""
        logger.info("Emergency stop reset")
        
    @property
    def is_triggered(self) -> bool:
        """Check if emergency stop is active."""
        return self._triggered
        
    @property
    def trigger_info(self) -> dict:
        """Get emergency stop information."""
        return {
            "triggered": self._triggered,
            "time": self._trigger_time,
            "reason": self._trigger_reason
        }
