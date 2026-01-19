"""
Rudder Controller Module
========================

Inner control loop that drives the rudder actuator to achieve
the target position commanded by the ML model.
"""

import time
from dataclasses import dataclass
from typing import Optional, Tuple
from enum import Enum, auto
import logging

logger = logging.getLogger(__name__)

# GPIO library with fallback
try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    GPIO = None
    HAS_GPIO = False
    logger.warning("RPi.GPIO not available. Running in simulation mode.")


class RudderDirection(Enum):
    """Rudder movement direction."""
    STOPPED = auto()
    PORT = auto()
    STARBOARD = auto()


@dataclass
class RudderControlConfig:
    """Configuration for rudder controller."""
    # GPIO pins for H-Bridge PWM
    pwm_port_pin: int = 18          # BCM pin for port drive
    pwm_starboard_pin: int = 19     # BCM pin for starboard drive
    pwm_frequency: int = 10000      # 10kHz for silent operation
    
    # Physical limits
    max_rudder_angle: float = 30.0  # degrees
    software_limit: float = 28.0    # degrees (inside hardware limit)
    
    # Rate limits (based on 15s lock-to-lock over 60 degrees)
    max_rate: float = 4.0           # deg/s
    
    # Control gains
    kp: float = 0.8                 # Proportional gain
    deadband: float = 0.5           # degrees - don't move if error smaller
    
    # PWM limits
    min_pwm: float = 0.1            # Minimum PWM to overcome friction
    max_pwm: float = 1.0            # Maximum PWM (full speed)
    
    # Timing
    update_rate_hz: float = 50.0    # Control loop rate


@dataclass
class RudderState:
    """Current state of rudder control."""
    target_deg: float = 0.0         # Target position (degrees)
    current_deg: float = 0.0        # Current position (degrees)
    error_deg: float = 0.0          # Position error
    rate_demand: float = 0.0        # Demanded rate (deg/s)
    direction: RudderDirection = RudderDirection.STOPPED
    pwm_output: float = 0.0         # Current PWM duty cycle
    at_target: bool = True          # Within deadband of target


class RudderController:
    """
    Inner control loop: ML target → PWM output.
    
    Takes target rudder position from ML model (normalized -1 to +1)
    and drives H-Bridge PWM to move rudder to that position.
    
    Runs at 50Hz (faster than 10Hz ML inference rate).
    """
    
    def __init__(self, config: Optional[RudderControlConfig] = None):
        self.config = config or RudderControlConfig()
        self._state = RudderState()
        self._pwm_port = None
        self._pwm_starboard = None
        self._initialized = False
        self._last_update = 0.0
        
    def start(self) -> bool:
        """Initialize GPIO and PWM."""
        if not HAS_GPIO:
            logger.warning("GPIO not available, running in simulation mode")
            self._initialized = True
            return True
            
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Setup PWM pins
            GPIO.setup(self.config.pwm_port_pin, GPIO.OUT)
            GPIO.setup(self.config.pwm_starboard_pin, GPIO.OUT)
            
            self._pwm_port = GPIO.PWM(
                self.config.pwm_port_pin, 
                self.config.pwm_frequency
            )
            self._pwm_starboard = GPIO.PWM(
                self.config.pwm_starboard_pin,
                self.config.pwm_frequency
            )
            
            # Start with 0% duty cycle (stopped)
            self._pwm_port.start(0)
            self._pwm_starboard.start(0)
            
            self._initialized = True
            logger.info("Rudder controller initialized")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize GPIO: {e}")
            return False
            
    def stop(self):
        """Stop PWM and cleanup GPIO."""
        self._set_pwm(0, 0)
        
        if self._pwm_port:
            self._pwm_port.stop()
        if self._pwm_starboard:
            self._pwm_starboard.stop()
            
        if HAS_GPIO:
            GPIO.cleanup([self.config.pwm_port_pin, self.config.pwm_starboard_pin])
            
        self._initialized = False
        logger.info("Rudder controller stopped")
        
    def update(self, target_normalized: float, current_position_deg: float) -> RudderState:
        """
        Update control loop.
        
        Args:
            target_normalized: ML output [-1, 1] → [-30°, +30°]
            current_position_deg: Current rudder position from ADC
            
        Returns:
            Current rudder state
        """
        now = time.time()
        dt = now - self._last_update if self._last_update > 0 else 0.02
        self._last_update = now
        
        # Convert normalized target to degrees
        target_deg = target_normalized * self.config.max_rudder_angle
        
        # Apply software limits
        target_deg = max(-self.config.software_limit, 
                        min(self.config.software_limit, target_deg))
        
        # Calculate error
        error = target_deg - current_position_deg
        
        # Update state
        self._state.target_deg = target_deg
        self._state.current_deg = current_position_deg
        self._state.error_deg = error
        
        # Check deadband
        if abs(error) < self.config.deadband:
            self._state.at_target = True
            self._state.direction = RudderDirection.STOPPED
            self._state.rate_demand = 0.0
            self._state.pwm_output = 0.0
            self._set_pwm(0, 0)
            return self._state
            
        self._state.at_target = False
        
        # Proportional control with rate limiting
        rate_demand = self.config.kp * error
        rate_demand = max(-self.config.max_rate, 
                         min(self.config.max_rate, rate_demand))
        self._state.rate_demand = rate_demand
        
        # Convert rate to PWM
        pwm = abs(rate_demand) / self.config.max_rate
        
        # Apply min/max PWM
        if pwm > 0:
            pwm = max(self.config.min_pwm, min(self.config.max_pwm, pwm))
            
        self._state.pwm_output = pwm
        
        # Set direction and output
        if rate_demand > 0:
            # Move starboard (positive)
            self._state.direction = RudderDirection.STARBOARD
            self._set_pwm(0, pwm)
        else:
            # Move port (negative)
            self._state.direction = RudderDirection.PORT
            self._set_pwm(pwm, 0)
            
        return self._state
        
    def emergency_stop(self):
        """Immediately stop rudder movement."""
        self._set_pwm(0, 0)
        self._state.direction = RudderDirection.STOPPED
        self._state.rate_demand = 0.0
        self._state.pwm_output = 0.0
        logger.warning("Rudder emergency stop")
        
    def _set_pwm(self, port: float, starboard: float):
        """Set PWM duty cycles (0-1 range input, converted to 0-100%)."""
        if not self._initialized:
            return
            
        # Convert 0-1 to 0-100 for RPi.GPIO
        port_duty = max(0, min(100, port * 100))
        starboard_duty = max(0, min(100, starboard * 100))
        
        if self._pwm_port and self._pwm_starboard:
            self._pwm_port.ChangeDutyCycle(port_duty)
            self._pwm_starboard.ChangeDutyCycle(starboard_duty)
            
    def get_state(self) -> RudderState:
        """Get current rudder control state."""
        return self._state


class MockRudderController(RudderController):
    """Mock controller for testing without hardware."""
    
    def __init__(self, config: Optional[RudderControlConfig] = None):
        super().__init__(config)
        self._simulated_position = 0.0
        
    def start(self) -> bool:
        """Start mock controller."""
        self._initialized = True
        logger.info("Mock rudder controller started")
        return True
        
    def update(self, target_normalized: float, current_position_deg: float) -> RudderState:
        """Update with simulated rudder movement."""
        # Calculate what would happen
        state = super().update(target_normalized, current_position_deg)
        
        # Simulate rudder movement
        dt = 0.02  # Assume 50Hz
        if state.direction == RudderDirection.STARBOARD:
            self._simulated_position += state.rate_demand * dt
        elif state.direction == RudderDirection.PORT:
            self._simulated_position += state.rate_demand * dt
            
        # Clamp
        self._simulated_position = max(-30, min(30, self._simulated_position))
        
        return state
        
    @property
    def simulated_position(self) -> float:
        """Get simulated rudder position."""
        return self._simulated_position
