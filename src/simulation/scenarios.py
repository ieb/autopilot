"""
Scenarios Module
================

Predefined sailing scenarios for data generation.
Each scenario defines wind, wave, and steering conditions.
"""

import random
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any
from enum import Enum
import logging

from .wind_model import WindConfig
from .wave_model import WaveConfig
from .helm_controller import HelmConfig, SteeringMode
from .yacht_dynamics import YachtConfig

logger = logging.getLogger(__name__)


class ScenarioType(Enum):
    """Types of sailing scenarios."""
    LIGHT_AIR_REACHING = "light_air_reaching"
    MEDIUM_UPWIND = "medium_upwind"
    HEAVY_WEATHER = "heavy_weather"
    DOWNWIND_VMG = "downwind_vmg"
    MIXED_COASTAL = "mixed_coastal"
    MOTORING = "motoring"
    RACE_UPWIND = "race_upwind"
    RACE_DOWNWIND = "race_downwind"
    DELIVERY = "delivery"
    ERROR_RECOVERY = "error_recovery"
    CUSTOM = "custom"


@dataclass
class Scenario:
    """
    Complete sailing scenario definition.
    
    Combines all configuration parameters for a realistic sailing session.
    """
    name: str
    description: str
    
    # Wind conditions
    wind_config: WindConfig = field(default_factory=WindConfig)
    
    # Wave conditions
    wave_config: WaveConfig = field(default_factory=WaveConfig)
    
    # Yacht configuration (for domain randomization)
    yacht_config: YachtConfig = field(default_factory=YachtConfig)
    
    # Helm configuration
    helm_config: HelmConfig = field(default_factory=HelmConfig)
    
    # Steering mode
    steering_mode: SteeringMode = SteeringMode.COMPASS
    
    # Initial conditions
    initial_heading: float = 0.0
    initial_twd: float = 180.0
    initial_tws: float = 15.0
    
    # Target angle (meaning depends on steering mode)
    target_angle: float = 0.0
    
    # Maneuver settings
    enable_maneuvers: bool = True
    tack_probability: float = 0.001
    gybe_probability: float = 0.0008
    
    # Duration
    duration_hours: float = 1.0
    
    # Initial error for error recovery scenarios (degrees)
    # If non-zero, yacht starts this many degrees off target
    initial_error_deg: float = 0.0
    
    # Skip warmup for this scenario (e.g., error_recovery needs transient)
    skip_warmup: bool = False
    
    # Metadata for training labels
    operation_mode: str = "sailing"  # sailing, motoring
    
    def randomize_initial_conditions(self):
        """Randomize initial heading and wind direction."""
        self.initial_heading = random.uniform(0, 360)
        self.initial_twd = random.uniform(0, 360)
        
        # Set target based on mode
        if self.steering_mode == SteeringMode.COMPASS:
            self.target_angle = self.initial_heading
        elif self.steering_mode == SteeringMode.WIND_AWA:
            # Keep roughly the same AWA
            pass
        elif self.steering_mode == SteeringMode.WIND_TWA:
            # Keep roughly the same TWA
            pass


def get_scenario(scenario_type: ScenarioType) -> Scenario:
    """
    Get a predefined scenario by type.
    
    Args:
        scenario_type: Type of scenario to create
        
    Returns:
        Configured Scenario object
    """
    if scenario_type == ScenarioType.LIGHT_AIR_REACHING:
        return _light_air_reaching()
    elif scenario_type == ScenarioType.MEDIUM_UPWIND:
        return _medium_upwind()
    elif scenario_type == ScenarioType.HEAVY_WEATHER:
        return _heavy_weather()
    elif scenario_type == ScenarioType.DOWNWIND_VMG:
        return _downwind_vmg()
    elif scenario_type == ScenarioType.MIXED_COASTAL:
        return _mixed_coastal()
    elif scenario_type == ScenarioType.MOTORING:
        return _motoring()
    elif scenario_type == ScenarioType.RACE_UPWIND:
        return _race_upwind()
    elif scenario_type == ScenarioType.RACE_DOWNWIND:
        return _race_downwind()
    elif scenario_type == ScenarioType.DELIVERY:
        return _delivery()
    elif scenario_type == ScenarioType.ERROR_RECOVERY:
        return _error_recovery()
    else:
        return _mixed_coastal()


def get_all_scenarios() -> List[Scenario]:
    """Get all predefined scenarios."""
    return [
        _light_air_reaching(),
        _medium_upwind(),
        _heavy_weather(),
        _downwind_vmg(),
        _mixed_coastal(),
        _motoring(),
        _race_upwind(),
        _race_downwind(),
        _delivery(),
        _error_recovery(),
    ]


def _light_air_reaching() -> Scenario:
    """Light air reaching conditions (6-10 knots)."""
    wind_config = WindConfig(
        base_tws_min=6.0,
        base_tws_max=10.0,
        shift_rate=1.0,  # More shifty in light air
        gust_probability=0.03,
        gust_intensity_min=1.3,
        gust_intensity_max=1.6,
    )
    
    wave_config = WaveConfig(
        swell_amplitude_min=1.0,
        swell_amplitude_max=3.0,
        chop_enabled=False,
    )
    
    helm_config = HelmConfig(
        skill_level=0.9,
        noise_std=0.4,  # More concentration needed
    )
    
    return Scenario(
        name="light_air_reaching",
        description="Light air reaching in 6-10 knots, AWA 70-110 degrees",
        wind_config=wind_config,
        wave_config=wave_config,
        helm_config=helm_config,
        steering_mode=SteeringMode.WIND_AWA,
        initial_tws=8.0,
        target_angle=85.0,  # Beam reach
        enable_maneuvers=False,  # Fewer maneuvers in light air
        operation_mode="sailing",
    )


def _medium_upwind() -> Scenario:
    """Medium wind upwind sailing (12-18 knots)."""
    wind_config = WindConfig(
        base_tws_min=12.0,
        base_tws_max=18.0,
        shift_rate=0.3,
        oscillation_amplitude=8.0,
        gust_probability=0.02,
    )
    
    wave_config = WaveConfig(
        swell_amplitude_min=3.0,
        swell_amplitude_max=6.0,
        chop_enabled=True,
    )
    
    helm_config = HelmConfig(
        skill_level=0.85,
        awa_kp=0.28,
        awa_kd=0.65,
    )
    
    return Scenario(
        name="medium_upwind",
        description="Medium wind upwind beating in 12-18 knots",
        wind_config=wind_config,
        wave_config=wave_config,
        helm_config=helm_config,
        steering_mode=SteeringMode.WIND_AWA,
        initial_tws=15.0,
        target_angle=42.0,  # Close hauled
        enable_maneuvers=True,
        tack_probability=0.002,  # More tacks upwind
        gybe_probability=0.0,
        operation_mode="sailing",
    )


def _heavy_weather() -> Scenario:
    """Heavy weather sailing (20-30 knots)."""
    wind_config = WindConfig(
        base_tws_min=20.0,
        base_tws_max=30.0,
        shift_rate=0.2,
        gust_probability=0.04,
        gust_intensity_min=1.2,
        gust_intensity_max=1.4,
    )
    
    wave_config = WaveConfig(
        swell_amplitude_min=6.0,
        swell_amplitude_max=12.0,
        swell_period_min=8.0,
        swell_period_max=14.0,
        chop_amplitude_factor=0.5,
    )
    
    yacht_config = YachtConfig(
        max_heel=35.0,
        heel_stiffness=0.025,
    )
    
    helm_config = HelmConfig(
        skill_level=0.95,  # Need good helm
        noise_std=0.5,
        fatigue_time_constant=1800.0,  # Fatigue faster
    )
    
    return Scenario(
        name="heavy_weather",
        description="Heavy weather sailing in 20-30 knots, reefed",
        wind_config=wind_config,
        wave_config=wave_config,
        yacht_config=yacht_config,
        helm_config=helm_config,
        steering_mode=SteeringMode.WIND_AWA,
        initial_tws=25.0,
        target_angle=50.0,  # Slightly cracked off
        enable_maneuvers=False,  # Minimize maneuvers
        operation_mode="sailing",
    )


def _downwind_vmg() -> Scenario:
    """Downwind VMG sailing (15-20 knots)."""
    wind_config = WindConfig(
        base_tws_min=15.0,
        base_tws_max=20.0,
        shift_rate=0.4,
        oscillation_amplitude=12.0,  # Bigger shifts matter more downwind
    )
    
    wave_config = WaveConfig(
        swell_amplitude_min=4.0,
        swell_amplitude_max=8.0,
        swell_period_min=8.0,
        swell_period_max=12.0,
    )
    
    helm_config = HelmConfig(
        skill_level=0.9,
        twa_kp=0.18,
        twa_kd=0.35,
    )
    
    return Scenario(
        name="downwind_vmg",
        description="Downwind VMG sailing in 15-20 knots, TWA 140-160",
        wind_config=wind_config,
        wave_config=wave_config,
        helm_config=helm_config,
        steering_mode=SteeringMode.WIND_TWA,
        initial_tws=17.0,
        target_angle=145.0,  # VMG angle
        enable_maneuvers=True,
        tack_probability=0.0,
        gybe_probability=0.0015,
        operation_mode="sailing",
    )


def _mixed_coastal() -> Scenario:
    """Mixed coastal sailing with changing conditions."""
    wind_config = WindConfig(
        base_tws_min=8.0,
        base_tws_max=20.0,
        shift_rate=0.8,  # Shifty near coast
        oscillation_period=180.0,  # Shorter oscillations
        oscillation_amplitude=15.0,
        gust_probability=0.025,
    )
    
    wave_config = WaveConfig(
        swell_amplitude_min=2.0,
        swell_amplitude_max=6.0,
        secondary_swell_enabled=True,  # Reflected waves
    )
    
    helm_config = HelmConfig(
        skill_level=0.8,
    )
    
    return Scenario(
        name="mixed_coastal",
        description="Mixed coastal sailing with variable conditions",
        wind_config=wind_config,
        wave_config=wave_config,
        helm_config=helm_config,
        steering_mode=SteeringMode.COMPASS,  # Will change during sim
        initial_tws=14.0,
        target_angle=0.0,  # Will be set based on conditions
        enable_maneuvers=True,
        tack_probability=0.001,
        gybe_probability=0.0008,
        operation_mode="sailing",
    )


def _motoring() -> Scenario:
    """Motoring in light wind."""
    wind_config = WindConfig(
        base_tws_min=0.0,
        base_tws_max=8.0,
        gust_probability=0.01,
    )
    
    wave_config = WaveConfig(
        swell_amplitude_min=0.5,
        swell_amplitude_max=2.0,
        chop_enabled=False,
    )
    
    helm_config = HelmConfig(
        compass_kp=0.25,
        compass_kd=0.4,
        noise_std=0.2,
    )
    
    return Scenario(
        name="motoring",
        description="Motoring in light wind, compass steering",
        wind_config=wind_config,
        wave_config=wave_config,
        helm_config=helm_config,
        steering_mode=SteeringMode.COMPASS,
        initial_tws=4.0,
        target_angle=0.0,  # Heading-based
        enable_maneuvers=False,
        operation_mode="motoring",
    )


def _race_upwind() -> Scenario:
    """Race-mode upwind sailing (aggressive)."""
    wind_config = WindConfig(
        base_tws_min=10.0,
        base_tws_max=16.0,
        shift_rate=0.5,
        oscillation_amplitude=10.0,
    )
    
    wave_config = WaveConfig(
        swell_amplitude_min=2.0,
        swell_amplitude_max=5.0,
    )
    
    helm_config = HelmConfig(
        skill_level=1.0,  # Expert helm
        awa_kp=0.3,
        awa_kd=0.7,
        noise_std=0.15,
        anticipation_factor=0.15,
    )
    
    return Scenario(
        name="race_upwind",
        description="Race upwind with expert helm, aggressive settings",
        wind_config=wind_config,
        wave_config=wave_config,
        helm_config=helm_config,
        steering_mode=SteeringMode.WIND_AWA,
        initial_tws=13.0,
        target_angle=40.0,  # Tight upwind
        enable_maneuvers=True,
        tack_probability=0.003,  # Frequent tacks
        operation_mode="sailing",
    )


def _race_downwind() -> Scenario:
    """Race-mode downwind sailing (aggressive)."""
    wind_config = WindConfig(
        base_tws_min=12.0,
        base_tws_max=18.0,
        shift_rate=0.4,
        oscillation_amplitude=12.0,
    )
    
    wave_config = WaveConfig(
        swell_amplitude_min=3.0,
        swell_amplitude_max=7.0,
    )
    
    helm_config = HelmConfig(
        skill_level=1.0,
        twa_kp=0.22,
        twa_kd=0.45,
        noise_std=0.15,
    )
    
    return Scenario(
        name="race_downwind",
        description="Race downwind with expert helm",
        wind_config=wind_config,
        wave_config=wave_config,
        helm_config=helm_config,
        steering_mode=SteeringMode.WIND_TWA,
        initial_tws=15.0,
        target_angle=140.0,
        enable_maneuvers=True,
        gybe_probability=0.002,
        operation_mode="sailing",
    )


def _delivery() -> Scenario:
    """Long-distance delivery sailing (conservative)."""
    wind_config = WindConfig(
        base_tws_min=10.0,
        base_tws_max=22.0,
        shift_rate=0.3,
        oscillation_period=600.0,  # Longer periods
    )
    
    wave_config = WaveConfig(
        swell_period_min=8.0,
        swell_period_max=14.0,
        swell_amplitude_min=3.0,
        swell_amplitude_max=8.0,
    )
    
    helm_config = HelmConfig(
        skill_level=0.8,
        fatigue_enabled=True,
        fatigue_time_constant=7200.0,  # 2 hour watches
    )
    
    return Scenario(
        name="delivery",
        description="Long-distance delivery, conservative sailing",
        wind_config=wind_config,
        wave_config=wave_config,
        helm_config=helm_config,
        steering_mode=SteeringMode.COMPASS,
        initial_tws=16.0,
        target_angle=0.0,
        enable_maneuvers=False,
        duration_hours=4.0,
        operation_mode="sailing",
    )


def _error_recovery() -> Scenario:
    """
    Error recovery training scenario.
    
    This scenario trains the model to recover from large heading errors.
    The initial conditions intentionally place the boat far from target,
    simulating disturbances like:
    - Wave knockdowns
    - Sudden wind shifts
    - Crew distractions
    - Autopilot mode changes
    
    The helm controller demonstrates proper recovery: large sustained rudder
    input until the error is corrected, then smooth settling to target.
    
    NOTE: This scenario should NOT use warmup period (set warmup_seconds=0)
    because we specifically want to capture the error recovery behavior.
    """
    wind_config = WindConfig(
        base_tws_min=12.0,
        base_tws_max=20.0,
        shift_rate=0.5,
        gust_probability=0.03,
        gust_intensity_min=1.2,
        gust_intensity_max=1.5,
    )
    
    wave_config = WaveConfig(
        swell_amplitude_min=3.0,
        swell_amplitude_max=7.0,
        chop_enabled=True,
    )
    
    # Expert helm for clean recovery demonstration
    helm_config = HelmConfig(
        skill_level=1.0,
        noise_std=0.15,
        reaction_delay=0.1,
        max_rudder_rate=4.0,  # Standard rate limiting
    )
    
    # Randomly choose a steering mode
    mode = random.choice([
        SteeringMode.COMPASS,
        SteeringMode.WIND_AWA,
        SteeringMode.WIND_TWA,
    ])
    
    # Set target based on mode
    if mode == SteeringMode.COMPASS:
        target = random.uniform(0, 360)
    elif mode == SteeringMode.WIND_AWA:
        target = random.choice([-1, 1]) * random.uniform(40, 90)
    else:  # TWA
        target = random.choice([-1, 1]) * random.uniform(120, 160)
    
    # Random initial error between 30-90 degrees (either direction)
    initial_error = random.choice([-1, 1]) * random.uniform(30, 90)
    
    return Scenario(
        name="error_recovery",
        description="Error recovery training with large initial disturbances",
        wind_config=wind_config,
        wave_config=wave_config,
        helm_config=helm_config,
        steering_mode=mode,
        initial_tws=16.0,
        target_angle=target,
        initial_error_deg=initial_error,  # Start 30-90 degrees off target
        skip_warmup=True,  # Capture the recovery transient
        enable_maneuvers=False,  # Focus on recovery, not maneuvers
        duration_hours=0.5,  # Shorter runs, more variety
        operation_mode="sailing",
    )


def create_random_scenario(
    tws_range: tuple = (8, 25),
    steering_mode: Optional[SteeringMode] = None,
    duration_hours: float = 1.0,
) -> Scenario:
    """
    Create a randomized scenario.
    
    Args:
        tws_range: Range of true wind speeds (min, max)
        steering_mode: Specific mode or None for random
        duration_hours: Duration of scenario
        
    Returns:
        Randomized Scenario
    """
    if steering_mode is None:
        steering_mode = random.choice([
            SteeringMode.COMPASS,
            SteeringMode.WIND_AWA,
            SteeringMode.WIND_TWA,
        ])
        
    # Randomize wind
    wind_config = WindConfig(
        base_tws_min=tws_range[0],
        base_tws_max=tws_range[1],
        shift_rate=random.uniform(0.2, 1.0),
        oscillation_amplitude=random.uniform(5, 15),
        gust_probability=random.uniform(0.01, 0.04),
    )
    
    # Randomize waves based on wind
    avg_tws = (tws_range[0] + tws_range[1]) / 2
    wave_amp = avg_tws * 0.3
    wave_config = WaveConfig(
        swell_amplitude_min=max(1, wave_amp - 2),
        swell_amplitude_max=wave_amp + 3,
    )
    
    # Randomize helm skill
    helm_config = HelmConfig(
        skill_level=random.uniform(0.7, 1.0),
        noise_std=random.uniform(0.2, 0.5),
    )
    
    # Set target based on mode
    if steering_mode == SteeringMode.COMPASS:
        target = random.uniform(0, 360)
    elif steering_mode == SteeringMode.WIND_AWA:
        target = random.choice([-1, 1]) * random.uniform(35, 110)
    else:  # TWA
        target = random.choice([-1, 1]) * random.uniform(120, 170)
        
    return Scenario(
        name="random",
        description="Randomly generated scenario",
        wind_config=wind_config,
        wave_config=wave_config,
        helm_config=helm_config,
        steering_mode=steering_mode,
        initial_twd=random.uniform(0, 360),
        initial_tws=random.uniform(*tws_range),
        target_angle=target,
        duration_hours=duration_hours,
    )
