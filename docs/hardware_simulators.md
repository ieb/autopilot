# Hardware Simulators

Software simulators that replicate the behavior of physical hardware components, enabling full-system testing of the autopilot without requiring actual hardware.

## Overview

The hardware simulators provide the same interfaces as the physical devices:

| Simulator | Real Hardware | Interface | Rate |
|-----------|---------------|-----------|------|
| IMU Simulator | ICM-20948 MCU | Unix socket, `$IMU` messages | 100 Hz |
| Actuator Simulator | ATtiny3226 MCU | Unix socket, `$RUD`/`$STS` | 20 Hz |
| CAN Simulator | NMEA2000 bus | vcan0 / stub mode | 1 Hz |

## Quick Start

### Running the Simulators

```bash
# Run with default scenario (upwind sailing)
uv run python -m src.simulation.hw_simulators.orchestrator

# Run with specific scenario for 60 seconds
uv run python -m src.simulation.hw_simulators.orchestrator -s reaching -d 60

# Run as fast as possible (not realtime)
uv run python -m src.simulation.hw_simulators.orchestrator --fast
```

### Connecting the Autopilot

```python
from src.sensors.imu_fusion import IMUFusion, IMUConfig
from src.control.actuator_interface import ActuatorInterface, ActuatorConfig
from src.sensors.nmea2000_interface import NMEA2000Interface, N2KConfig

# Connect to simulated IMU
imu = IMUFusion(IMUConfig(
    port="/tmp/autopilot_imu.sock",
    use_socket=True
))
imu.start()

# Connect to simulated actuator
actuator = ActuatorInterface(ActuatorConfig(
    port="/tmp/autopilot_actuator.sock",
    use_socket=True
))
actuator.start()

# Connect to simulated CAN bus (Linux only)
n2k = NMEA2000Interface(N2KConfig(channel="vcan0"))
n2k.start()
```

## Programmatic Usage

### Basic Example

```python
from src.simulation.hw_simulators import SimulatorOrchestrator
from src.simulation.scenarios import get_scenario

# Create orchestrator with scenario
scenario = get_scenario("upwind")
orchestrator = SimulatorOrchestrator(scenario=scenario)

# Start all simulators
orchestrator.start()

# Run simulation for 60 seconds in realtime
orchestrator.run(duration=60.0, realtime=True)

# Or step manually
for _ in range(1000):
    orchestrator.step(dt=0.02)  # 50 Hz physics

# Clean shutdown
orchestrator.stop()
```

### Accessing State

```python
# Get current yacht state
state = orchestrator.get_state()
print(f"Heading: {state['heading']:.1f}°")
print(f"Speed: {state['stw']:.1f} kts")
print(f"AWA: {state['awa']:.1f}°")
print(f"Rudder: {state['rudder_angle']:.1f}°")

# Set external values
orchestrator.set_state('tws', 15.0)  # True wind speed
orchestrator.set_state('twd', 270.0)  # True wind direction
```

## IMU Simulator

Simulates the ICM-20948 IMU connected to a fusion MCU.

### Message Format

```
$IMU,heading,pitch,roll,yaw_rate,pitch_rate,roll_rate,ax,ay,az*XX\r\n
```

| Field | Unit | Description |
|-------|------|-------------|
| heading | degrees | Magnetic heading (0-360) |
| pitch | degrees | Bow up/down (+ = bow up) |
| roll | degrees | Heel angle (+ = starboard) |
| yaw_rate | deg/s | Rate of turn (+ = starboard) |
| pitch_rate | deg/s | Pitching rate |
| roll_rate | deg/s | Rolling rate |
| ax, ay, az | m/s² | Linear accelerations |
| XX | hex | XOR checksum |

### Configuration

```python
from src.simulation.hw_simulators.imu_sim import IMUSimulator, IMUSimulatorConfig

config = IMUSimulatorConfig(
    socket_path="/tmp/autopilot_imu.sock",
    update_rate_hz=100.0,
    heading_noise_std=0.1,    # degrees
    attitude_noise_std=0.05,  # degrees
    rate_noise_std=0.02,      # deg/s
    accel_noise_std=0.01,     # m/s²
    timing_jitter_std=0.0005  # seconds
)
```

## Actuator Simulator

Simulates the ATtiny3226 actuator controller with motor, clutch, and watchdog.

### Command Format (Input)

```
$RUD,<target>,<engage>*XX\r\n
```

| Field | Range | Description |
|-------|-------|-------------|
| target | -1.0 to 1.0 | Normalized rudder position (±1 = ±30°) |
| engage | 0 or 1 | Clutch engage (1) or disengage (0) |

### Status Format (Output @ 20Hz)

```
$STS,<target>,<actual>,<clutch>,<voltage>,<current>,<fault>*XX\r\n
```

| Field | Description |
|-------|-------------|
| target | Commanded position (-1 to 1) |
| actual | Current position (-1 to 1) |
| clutch | 0 = disengaged, 1 = engaged |
| voltage | Supply voltage (V) |
| current | Motor current (A) |
| fault | Fault code (0 = none) |

### Fault Codes

| Code | Name | Description |
|------|------|-------------|
| 0 | NONE | No fault |
| 1 | OVERCURRENT | Motor current exceeded limit |
| 2 | STALL | Motor stalled |
| 3 | POSITION_LIMIT | Hit position limit |
| 4 | SENSOR | Position sensor fault |
| 5 | WATCHDOG | No command for 2 seconds |

### Configuration

```python
from src.simulation.hw_simulators.actuator_sim import ActuatorSimulator, ActuatorSimulatorConfig

config = ActuatorSimulatorConfig(
    socket_path="/tmp/autopilot_actuator.sock",
    update_rate_hz=20.0,
    max_rudder_rate=4.0,      # degrees/second
    max_rudder_angle=30.0,    # degrees
    watchdog_timeout=2.0,     # seconds
    clutch_engage_delay=0.1,  # seconds
)
```

## CAN Simulator

Simulates the NMEA2000 bus by emitting PGN frames.

### Platform Support

| Platform | Mode | Description |
|----------|------|-------------|
| Linux | vcan0 | Full virtual CAN interface |
| macOS | stub | Frames generated but not sent |

### Setup (Linux)

```bash
# Load virtual CAN module
sudo modprobe vcan

# Create virtual interface
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Verify
candump vcan0
```

### PGNs Emitted

| PGN | Name | Rate | Fields |
|-----|------|------|--------|
| 130306 | Wind Data | 1 Hz | AWA, AWS |
| 128259 | Speed | 1 Hz | STW, SOG |
| 129026 | COG/SOG | 1 Hz | COG, SOG |
| 127250 | Vessel Heading | 1 Hz | Heading |
| 129029 | GNSS Position | 1 Hz | Lat, Lon |

### Stub Mode (macOS)

On macOS, the CAN simulator runs in stub mode automatically:

```python
from src.simulation.hw_simulators.can_sim import CANSimulator, CANSimulatorConfig
from multiprocessing import Manager

manager = Manager()
state = manager.dict()
state['heading'] = 45.0
state['awa'] = 30.0
state['aws'] = 15.0

sim = CANSimulator(state, CANSimulatorConfig(stub_mode=True))
sim.start()
sim.step()

# Inspect generated frames
for pgn, data in sim.get_frame_buffer():
    print(f"PGN {pgn}: {data.hex()}")

sim.stop()
```

## Testing

Run the hardware simulator tests:

```bash
# Run all simulator tests
uv run pytest tests/test_hw_simulators.py -v

# Run specific test class
uv run pytest tests/test_hw_simulators.py::TestIMUSimulator -v

# Run integration tests
uv run pytest tests/test_hw_simulators.py::TestSimulatorIntegration -v
```

### Test Coverage

| Test Class | Tests | Description |
|------------|-------|-------------|
| TestBaseSimulator | 4 | Checksum, config |
| TestIMUSimulator | 6 | Socket, messages, state |
| TestActuatorSimulator | 9 | Commands, clutch, watchdog |
| TestCANSimulator | 9 | Stub mode, PGN encoding |
| TestSimulatorIntegration | 2 | Full interface connections |
| TestOrchestratorUnit | 3 | Wind calculations |

## Troubleshooting

### Socket Connection Refused

```
ConnectionRefusedError: [Errno 111] Connection refused
```

**Cause**: Simulator not running or socket not created yet.

**Fix**: Ensure orchestrator is started before connecting:
```python
orchestrator.start()
time.sleep(0.5)  # Wait for sockets to be created
# Now connect
```

### CAN Interface Not Found

```
OSError: [Errno 19] No such device
```

**Cause**: vcan0 interface not created (Linux) or vcan not supported (macOS).

**Fix (Linux)**:
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

**Fix (macOS)**: CAN simulator automatically uses stub mode. No real CAN bus is available.

### Watchdog Timeout

```
WARNING - Actuator watchdog timeout - disengaging clutch
```

**Cause**: No `$RUD` command received for 2 seconds.

**Fix**: Send commands at least every 1 second to maintain clutch engagement:
```python
while running:
    actuator.send_command(rudder, engage=True)
    time.sleep(0.1)
```

## See Also

- [Implementation Plan](../planning/hardware_simulators_plan.md) - Technical implementation details
- [Simulated Training Data](simulated_training_data.md) - Generating training data
- [Model Architecture](model_architecture.md) - Neural network details
