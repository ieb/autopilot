"""
Maneuvers Module
================

Defines discrete sailing maneuvers: tacks, gybes, and course changes.
Each maneuver modifies yacht state over a defined duration.
"""

import math
import random
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Generator, Tuple
from enum import Enum
import logging

from .yacht_dynamics import YachtState
from .helm_controller import SteeringMode

logger = logging.getLogger(__name__)


class ManeuverType(Enum):
    """Types of sailing maneuvers."""
    TACK = "tack"
    GYBE = "gybe"
    COURSE_CHANGE = "course_change"
    BEAR_AWAY = "bear_away"
    HEAD_UP = "head_up"


@dataclass
class ManeuverResult:
    """Result of a maneuver step."""
    rudder_command: float       # Rudder command for this step
    heading_target: float       # Target heading during maneuver
    in_progress: bool           # Whether maneuver is still in progress
    phase: str                  # Current phase of maneuver
    speed_factor: float = 1.0   # Speed reduction factor during maneuver


class Maneuver(ABC):
    """
    Base class for sailing maneuvers.
    
    Maneuvers override normal helm control to execute a specific sequence.
    """
    
    def __init__(self, duration: float = 10.0):
        """
        Initialize maneuver.
        
        Args:
            duration: Expected duration of maneuver (seconds)
        """
        self.duration = duration
        self.elapsed = 0.0
        self.completed = False
        self.initial_state: Optional[YachtState] = None
        
    @abstractmethod
    def start(self, state: YachtState, twd: float) -> None:
        """
        Start the maneuver from current state.
        
        Args:
            state: Current yacht state
            twd: True wind direction
        """
        pass
        
    @abstractmethod
    def step(self, state: YachtState, dt: float) -> ManeuverResult:
        """
        Compute next step of maneuver.
        
        Args:
            state: Current yacht state
            dt: Time step (seconds)
            
        Returns:
            ManeuverResult with rudder command and status
        """
        pass
        
    def is_complete(self) -> bool:
        """Check if maneuver is complete."""
        return self.completed
        
    def reset(self):
        """Reset maneuver state."""
        self.elapsed = 0.0
        self.completed = False
        self.initial_state = None


class Tack(Maneuver):
    """
    Tack maneuver - turn through the wind.
    
    Phases:
    1. Preparation: Build speed, helm to weather
    2. Turn: Hard over through the wind
    3. Exit: Settle on new course
    """
    
    def __init__(self, target_awa: float = 45.0, duration: float = 12.0):
        """
        Initialize tack.
        
        Args:
            target_awa: Target AWA on new tack (degrees, will flip sign)
            duration: Expected duration (seconds)
        """
        super().__init__(duration)
        self.target_awa = abs(target_awa)
        self.target_heading = 0.0
        self.start_heading = 0.0
        self.turn_direction = 1  # 1 = starboard tack to port, -1 = reverse
        self.phase = "prep"
        
    def start(self, state: YachtState, twd: float) -> None:
        """Start tack from current state."""
        self.initial_state = state
        self.start_heading = state.heading
        self.elapsed = 0.0
        self.completed = False
        self.phase = "prep"
        
        # Determine turn direction based on current tack
        # If AWA is positive (wind from starboard), tack to port (turn left)
        if state.awa > 0:
            self.turn_direction = -1  # Turn left (port)
            new_awa = -self.target_awa
        else:
            self.turn_direction = 1   # Turn right (starboard)
            new_awa = self.target_awa
            
        # Calculate target heading
        # New heading = TWD - new_AWA (approximately)
        self.target_heading = (twd - new_awa) % 360
        
    def step(self, state: YachtState, dt: float) -> ManeuverResult:
        """Execute tack step."""
        self.elapsed += dt
        
        # Calculate progress through maneuver
        progress = min(1.0, self.elapsed / self.duration)
        
        # Determine phase
        if progress < 0.15:
            self.phase = "prep"
            # Slight rudder to build momentum
            rudder = 5.0 * self.turn_direction
            speed_factor = 1.0
        elif progress < 0.6:
            self.phase = "turn"
            # Hard over
            rudder = 25.0 * self.turn_direction
            speed_factor = 0.6  # Speed loss during turn
        elif progress < 0.85:
            self.phase = "settle"
            # Reduce rudder, let boat settle
            settle_progress = (progress - 0.6) / 0.25
            rudder = 25.0 * (1.0 - settle_progress) * self.turn_direction
            speed_factor = 0.7 + 0.3 * settle_progress
        else:
            self.phase = "exit"
            # Fine-tune to target
            heading_error = self._angle_diff(state.heading, self.target_heading)
            rudder = heading_error * 0.3
            speed_factor = 0.9
            
            # Check if complete
            if abs(heading_error) < 5.0 and progress >= 0.95:
                self.completed = True
                
        # Add some human variation
        rudder += random.gauss(0, 1.0)
        
        return ManeuverResult(
            rudder_command=max(-25, min(25, rudder)),
            heading_target=self.target_heading,
            in_progress=not self.completed,
            phase=self.phase,
            speed_factor=speed_factor,
        )
        
    def _angle_diff(self, a: float, b: float) -> float:
        """Compute signed angle difference."""
        diff = a - b
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff


class Gybe(Maneuver):
    """
    Gybe maneuver - turn away from the wind.
    
    Phases:
    1. Preparation: Bear away slightly
    2. Turn: Controlled turn through dead downwind
    3. Exit: Settle on new course
    """
    
    def __init__(self, target_twa: float = 140.0, duration: float = 15.0):
        """
        Initialize gybe.
        
        Args:
            target_twa: Target TWA on new gybe (degrees, will flip sign)
            duration: Expected duration (seconds)
        """
        super().__init__(duration)
        self.target_twa = abs(target_twa)
        self.target_heading = 0.0
        self.start_heading = 0.0
        self.turn_direction = 1
        self.phase = "prep"
        
    def start(self, state: YachtState, twd: float) -> None:
        """Start gybe from current state."""
        self.initial_state = state
        self.start_heading = state.heading
        self.elapsed = 0.0
        self.completed = False
        self.phase = "prep"
        
        # Compute current TWA
        current_twa = twd - state.heading
        while current_twa > 180:
            current_twa -= 360
        while current_twa < -180:
            current_twa += 360
            
        # Determine turn direction
        if current_twa > 0:
            self.turn_direction = 1   # Turn right (bear away to gybe)
            new_twa = -self.target_twa
        else:
            self.turn_direction = -1  # Turn left
            new_twa = self.target_twa
            
        # Calculate target heading
        self.target_heading = (twd - new_twa) % 360
        
    def step(self, state: YachtState, dt: float) -> ManeuverResult:
        """Execute gybe step."""
        self.elapsed += dt
        progress = min(1.0, self.elapsed / self.duration)
        
        if progress < 0.2:
            self.phase = "prep"
            # Bear away gently
            rudder = 8.0 * self.turn_direction
            speed_factor = 1.0
        elif progress < 0.5:
            self.phase = "turn"
            # Moderate rudder through dead downwind
            rudder = 15.0 * self.turn_direction
            speed_factor = 0.85
        elif progress < 0.8:
            self.phase = "settle"
            # Ease out of turn
            settle_progress = (progress - 0.5) / 0.3
            rudder = 15.0 * (1.0 - settle_progress) * self.turn_direction
            speed_factor = 0.9
        else:
            self.phase = "exit"
            # Fine-tune
            heading_error = self._angle_diff(state.heading, self.target_heading)
            rudder = heading_error * 0.25
            speed_factor = 0.95
            
            if abs(heading_error) < 5.0 and progress >= 0.95:
                self.completed = True
                
        rudder += random.gauss(0, 0.8)
        
        return ManeuverResult(
            rudder_command=max(-20, min(20, rudder)),
            heading_target=self.target_heading,
            in_progress=not self.completed,
            phase=self.phase,
            speed_factor=speed_factor,
        )
        
    def _angle_diff(self, a: float, b: float) -> float:
        diff = a - b
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff


class CourseChange(Maneuver):
    """
    Gradual course change to a new heading.
    
    Used for tactical adjustments without tacking or gybing.
    """
    
    def __init__(self, heading_change: float, duration: Optional[float] = None):
        """
        Initialize course change.
        
        Args:
            heading_change: Change in heading (degrees, positive = starboard)
            duration: Duration of maneuver (auto-calculated if not specified)
        """
        # Calculate duration based on magnitude of change
        if duration is None:
            duration = max(5.0, abs(heading_change) / 5.0)  # ~5 deg/s turn rate
            
        super().__init__(duration)
        self.heading_change = heading_change
        self.target_heading = 0.0
        self.start_heading = 0.0
        self.phase = "turn"
        
    def start(self, state: YachtState, twd: float) -> None:
        """Start course change from current state."""
        self.initial_state = state
        self.start_heading = state.heading
        self.target_heading = (state.heading + self.heading_change) % 360
        self.elapsed = 0.0
        self.completed = False
        self.phase = "turn"
        
    def step(self, state: YachtState, dt: float) -> ManeuverResult:
        """Execute course change step."""
        self.elapsed += dt
        progress = min(1.0, self.elapsed / self.duration)
        
        # Smooth S-curve profile
        if progress < 0.3:
            self.phase = "accelerate"
            turn_rate = progress / 0.3
        elif progress < 0.7:
            self.phase = "turn"
            turn_rate = 1.0
        else:
            self.phase = "decelerate"
            turn_rate = (1.0 - progress) / 0.3
            
        # Calculate intermediate target
        target_progress = self._smooth_step(progress)
        intermediate_heading = (
            self.start_heading + self.heading_change * target_progress
        ) % 360
        
        # PD control to intermediate target
        heading_error = self._angle_diff(state.heading, intermediate_heading)
        rudder = heading_error * 0.4 * turn_rate
        
        # Check completion
        if progress >= 0.98:
            final_error = self._angle_diff(state.heading, self.target_heading)
            if abs(final_error) < 3.0:
                self.completed = True
                
        rudder += random.gauss(0, 0.5)
        
        return ManeuverResult(
            rudder_command=max(-20, min(20, rudder)),
            heading_target=intermediate_heading,
            in_progress=not self.completed,
            phase=self.phase,
            speed_factor=0.98,
        )
        
    def _smooth_step(self, x: float) -> float:
        """Smooth step function for gradual transitions."""
        return x * x * (3 - 2 * x)
        
    def _angle_diff(self, a: float, b: float) -> float:
        diff = a - b
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff


class ManeuverGenerator:
    """
    Generates random maneuvers during simulation.
    
    Decides when to initiate maneuvers based on conditions.
    """
    
    def __init__(self, 
                 tack_probability: float = 0.001,
                 gybe_probability: float = 0.0008,
                 course_change_probability: float = 0.002):
        """
        Initialize maneuver generator.
        
        Args:
            tack_probability: Probability per second of initiating a tack
            gybe_probability: Probability per second of initiating a gybe
            course_change_probability: Probability per second of course change
        """
        self.tack_probability = tack_probability
        self.gybe_probability = gybe_probability
        self.course_change_probability = course_change_probability
        
        self.current_maneuver: Optional[Maneuver] = None
        self.min_interval = 60.0  # Minimum seconds between maneuvers
        self.last_maneuver_time = -self.min_interval
        
    def check_maneuver(self, state: YachtState, twd: float, 
                       elapsed_time: float, dt: float) -> Optional[Maneuver]:
        """
        Check if a maneuver should be initiated.
        
        Args:
            state: Current yacht state
            twd: True wind direction
            elapsed_time: Total elapsed simulation time
            dt: Time step
            
        Returns:
            New Maneuver if one should start, None otherwise
        """
        # Don't start new maneuver if one is in progress
        if self.current_maneuver and not self.current_maneuver.is_complete():
            return None
            
        # Enforce minimum interval
        if elapsed_time - self.last_maneuver_time < self.min_interval:
            return None
            
        # Compute current TWA
        twa = twd - state.heading
        while twa > 180:
            twa -= 360
        while twa < -180:
            twa += 360
            
        # Check for tack (only when upwind)
        if abs(twa) < 70:
            if random.random() < self.tack_probability * dt:
                self.last_maneuver_time = elapsed_time
                maneuver = Tack(target_awa=abs(state.awa))
                maneuver.start(state, twd)
                self.current_maneuver = maneuver
                return maneuver
                
        # Check for gybe (only when downwind)
        if abs(twa) > 120:
            if random.random() < self.gybe_probability * dt:
                self.last_maneuver_time = elapsed_time
                maneuver = Gybe(target_twa=abs(twa))
                maneuver.start(state, twd)
                self.current_maneuver = maneuver
                return maneuver
                
        # Check for course change
        if random.random() < self.course_change_probability * dt:
            self.last_maneuver_time = elapsed_time
            # Random change within ±30 degrees
            change = random.uniform(-30, 30)
            maneuver = CourseChange(heading_change=change)
            maneuver.start(state, twd)
            self.current_maneuver = maneuver
            return maneuver
            
        return None
        
    def get_current_maneuver(self) -> Optional[Maneuver]:
        """Get the currently executing maneuver."""
        if self.current_maneuver and not self.current_maneuver.is_complete():
            return self.current_maneuver
        return None
        
    def clear(self):
        """Clear current maneuver."""
        self.current_maneuver = None
