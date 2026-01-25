"""
Hardware Simulators
===================

Software simulators that expose the same interfaces as physical hardware,
enabling full-system testing without physical devices.

Each simulator runs as a separate process to avoid Python GIL synchronization:

- IMU Simulator: Unix socket, $IMU messages at 100Hz
- Actuator Simulator: Unix socket, $RUD commands / $STS status at 20Hz  
- CAN Simulator: vcan0 interface, NMEA2000 PGNs at ~1Hz

Usage:
    from src.simulation.hw_simulators import SimulatorOrchestrator
    
    orchestrator = SimulatorOrchestrator(scenario)
    orchestrator.start()
    # ... run autopilot against simulators
    orchestrator.stop()
"""

from .base import HardwareSimulator
from .imu_sim import IMUSimulator, run_imu_sim
from .actuator_sim import ActuatorSimulator, run_actuator_sim
from .can_sim import CANSimulator, run_can_sim
from .orchestrator import SimulatorOrchestrator

__all__ = [
    'HardwareSimulator',
    'IMUSimulator',
    'ActuatorSimulator', 
    'CANSimulator',
    'SimulatorOrchestrator',
    'run_imu_sim',
    'run_actuator_sim',
    'run_can_sim',
]
