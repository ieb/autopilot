"""
Safety Layer Module
===================

Hard limits and sanity checks between ML model and rudder actuator.
Ensures safe operation even if ML model produces unexpected outputs.
"""

import time
from dataclasses import dataclass
from enum import IntEnum, auto
from typing import Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class AlarmCode(IntEnum):
    """Safety alarm codes."""
    OK = 0
    RUDDER_LIMIT = auto()           # Approaching rudder limit
    HEADING_DEVIATION = auto()      # Large heading error
    SENSOR_TIMEOUT = auto()         # Sensor data stale
    RATE_LIMIT = auto()             # Rate limiting active
    MANUAL_OVERRIDE = auto()        # Human override detected
    MODEL_ERROR = auto()            # ML model produced invalid output
    EMERGENCY_STOP = auto()         # Emergency stop triggered


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
