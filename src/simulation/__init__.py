"""
Simulation Module
=================

Generates realistic sailing data for ML training.
Provides physics-based yacht dynamics, wind modeling, and human-like steering.
"""

from .yacht_dynamics import YachtDynamics, YachtConfig, YachtState
from .wind_model import WindModel, WindConfig, WindState
from .wave_model import WaveModel, WaveConfig
from .helm_controller import HelmController, HelmConfig
from .maneuvers import Maneuver, Tack, Gybe, CourseChange
from .scenarios import Scenario, get_scenario
from .data_generator import SailingDataGenerator, SimConfig, generate_training_data

__all__ = [
    'YachtDynamics', 'YachtConfig', 'YachtState',
    'WindModel', 'WindConfig', 'WindState',
    'WaveModel', 'WaveConfig',
    'HelmController', 'HelmConfig',
    'Maneuver', 'Tack', 'Gybe', 'CourseChange',
    'Scenario', 'get_scenario',
    'SailingDataGenerator', 'SimConfig', 'generate_training_data',
]
