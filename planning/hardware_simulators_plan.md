# Hardware Simulators Implementation Plan

**Status: Complete**

This plan documents the hardware simulator architecture implemented for full-system testing without physical devices.

## Overview

Build software simulators that expose the same interfaces as physical hardware (IMU MCU, Actuator Controller, NMEA2000 bus), enabling full-system testing without physical devices. Each simulator runs as a separate process to avoid Python GIL synchronization.

## Completed Milestones

| Task | Status |
|------|--------|
| Create base simulator class with Unix socket server | ✅ Complete |
| Build IMU simulator (100Hz $IMU messages) | ✅ Complete |
| Build Actuator simulator ($RUD/$STS protocol) | ✅ Complete |
| Build CAN simulator (vcan0 / stub mode) | ✅ Complete |
| Create orchestrator for shared yacht state | ✅ Complete |
| Add socket support to IMU and Actuator interfaces | ✅ Complete |
| Write unit tests (33 tests) | ✅ Complete |

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Simulator Orchestrator                       │
│  (Manages shared YachtState, launches simulator processes)       │
└─────────────────────────────────────────────────────────────────┘
         │                    │                    │
         ▼                    ▼                    ▼
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│  IMU Simulator  │  │ Actuator Sim    │  │  CAN Simulator  │
│  (Process 1)    │  │  (Process 2)    │  │   (Process 3)   │
├─────────────────┤  ├─────────────────┤  ├─────────────────┤
│ Unix Socket     │  │ Unix Socket     │  │ vcan0 / stub    │
│ $IMU,... @100Hz │  │ $RUD ↔ $STS     │  │ PGNs @1Hz       │
└────────┬────────┘  └────────┬────────┘  └────────┬────────┘
         │                    │                    │
         ▼                    ▼                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Autopilot Main Process                      │
│  IMUFusion ─────► FeatureEng ─────► MLModel ─────► ActuatorIF   │
│  N2KInterface ──►                                                │
└─────────────────────────────────────────────────────────────────┘
```

## Components

### Shared State

All simulators read from a shared yacht state (via `multiprocessing.Manager`) that evolves based on:
- Wind conditions (from scenario or external)
- Rudder commands (from actuator simulator feedback)
- Physics model (heading, speed, heel, position)

### IMU Simulator

**Location**: `src/simulation/hw_simulators/imu_sim.py`

- **Protocol**: `$IMU,heading,pitch,roll,yaw_rate,pitch_rate,roll_rate,ax,ay,az*XX\r\n`
- **Rate**: 100Hz
- **Socket**: `/tmp/autopilot_imu.sock`
- **Features**: Configurable sensor noise, timing jitter

### Actuator Simulator

**Location**: `src/simulation/hw_simulators/actuator_sim.py`

- **Input**: `$RUD,<target>,<engage>*XX` commands
- **Output**: `$STS,<target>,<actual>,<clutch>,<voltage>,<current>,<fault>*XX` @20Hz
- **Socket**: `/tmp/autopilot_actuator.sock`
- **Features**:
  - Position control with rate limit (~4°/s)
  - Clutch engage/disengage with soft-start
  - Watchdog timeout (2s no command → disengage)
  - Current draw proportional to rudder load
  - Fault conditions (overcurrent, stall, position limit)

### CAN Simulator

**Location**: `src/simulation/hw_simulators/can_sim.py`

- **Interface**: `vcan0` on Linux, stub mode on macOS
- **Rate**: ~1Hz per PGN
- **PGNs**: 130306 (Wind), 128259 (Speed), 129026 (COG/SOG), 127250 (Heading), 129029 (Position)

### Orchestrator

**Location**: `src/simulation/hw_simulators/orchestrator.py`

Coordinates all simulators with shared physics:
- Launches simulator processes
- Runs yacht dynamics at 50Hz
- Updates shared state
- Handles clean startup/shutdown

## Files Created

| File | Description |
|------|-------------|
| `src/simulation/hw_simulators/__init__.py` | Package exports |
| `src/simulation/hw_simulators/base.py` | Base class with Unix socket server |
| `src/simulation/hw_simulators/imu_sim.py` | IMU simulator |
| `src/simulation/hw_simulators/actuator_sim.py` | Actuator simulator |
| `src/simulation/hw_simulators/can_sim.py` | CAN/NMEA2000 simulator |
| `src/simulation/hw_simulators/orchestrator.py` | Process coordinator |
| `tests/test_hw_simulators.py` | 33 unit tests |

## Files Modified

| File | Change |
|------|--------|
| `src/sensors/imu_fusion.py` | Added `use_socket` option |
| `src/control/actuator_interface.py` | Added `use_socket` option |
| `planning/implementation_plan.md` | Added current status section |
| `docs/model_architecture.md` | Updated parameter counts |
| `docs/README.md` | Added new documentation links |

## Platform Notes

### macOS

- vcan is not available (Linux kernel module)
- CAN simulator runs in **stub mode** automatically
- Frames are generated but not sent to a real bus
- Use `get_frame_buffer()` to inspect generated frames

### Linux

Set up virtual CAN before running:
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

## Test Results

All 33 hardware simulator tests pass:
- Base simulator: 4 tests
- IMU simulator: 6 tests
- Actuator simulator: 9 tests
- CAN simulator: 9 tests
- Integration: 2 tests
- Orchestrator: 3 tests
