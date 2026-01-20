"""
Control System Modules
======================

This package contains the control system components for the autopilot.

Primary Components (current architecture):
    - ActuatorInterface: Serial interface to Actuator Controller MCU
    - SystemSafety: System-level safety monitoring (Layer 2)
    - ModeManager: Operating mode management

Deprecated Components (retained for reference):
    - RudderController: Replaced by Actuator Controller MCU
    - SafetyLayer: Replaced by SystemSafety + MCU hardware protection
"""

from .actuator_interface import (
    ActuatorInterface,
    ActuatorStatus,
    ActuatorConfig,
    MCUFaultCode,
    MockActuatorInterface,
)

from .safety import (
    SystemSafety,
    AlarmCode,
    EmergencyStop,
    # Legacy - retained for compatibility
    SafetyLayer,
    SafetyConfig,
    SafetyState,
)

from .mode_manager import (
    ModeManager,
    AutopilotMode,
)

__all__ = [
    # Primary components
    'ActuatorInterface',
    'ActuatorStatus',
    'ActuatorConfig',
    'MCUFaultCode',
    'MockActuatorInterface',
    'SystemSafety',
    'AlarmCode',
    'EmergencyStop',
    'ModeManager',
    'AutopilotMode',
    # Legacy (deprecated)
    'SafetyLayer',
    'SafetyConfig',
    'SafetyState',
]
