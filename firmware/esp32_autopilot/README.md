# ESP32-S3 Autopilot Firmware

Autopilot controller for a Pogo 1250 yacht. PD, PID, Smooth, and Adaptive pilots with BNO055 IMU fusion, NMEA2000 CAN bus integration, and BTS7960 H-bridge rudder actuator control.

## Hardware

| Component | Interface | Pins |
|-----------|-----------|------|
| BNO055 IMU | I2C 400 kHz | SDA=GPIO8, SCL=GPIO9 |
| NMEA2000 CAN | TWAI | TX=GPIO4, RX=GPIO5 |
| Rudder pot (ADC) | ADC1 | GPIO1 |
| BTS7960 H-bridge | PWM 10 kHz | RPWM=GPIO15, LPWM=GPIO16, EN=GPIO17 |
| Clutch (N-FET) | Digital | GPIO18 |
| Status LED (NeoPixel) | Digital | GPIO48 |

**Rudder ADC calibration:** center=1650 mV, port (−25°)=1000 mV, starboard (+25°)=2300 mV. Calibration is saved to NVS via the web UI.

**Actuator rate:** 1.7 deg/s (Jefa LD-100, ~30 s full travel over 50°). Safety rate limit: 2.0 deg/s.

## Architecture

```
                  ┌──────────────┐
   BNO055 ──I2C──┤  imu.cpp     │ 20 Hz
                  │  (heading,   │
                  │   roll, yaw) │
                  ├──────────────┤
   N2K bus ──CAN──┤  n2k.cpp     │ parse every loop, send 5 Hz
                  │  (AWA, STW,  │
                  │   COG, hdg)  │
                  ├──────────────┤
                  │ pilot_manager│ 5 Hz
                  │  selects:    │
                  │  PD/PID/     │
                  │  Smooth/     │
                  │  Adaptive    │
                  ├──────────────┤
   BTS7960 ──PWM──┤ actuator.cpp │ 50 Hz position P-loop
   Rudder pot─ADC─┤              │ Kp=0.8, deadband=0.5°
                  └──────────────┘
```

### Control rates

| Subsystem | Rate | Interval |
|-----------|------|----------|
| IMU read | 20 Hz | 50 ms |
| Pilot update | 5 Hz | 200 ms |
| Actuator loop | 50 Hz | 20 ms |
| N2K transmit | 5 Hz | 200 ms |

### Steering modes

| Mode | Description |
|------|-------------|
| `MODE_STANDBY` | No steering output |
| `MODE_COMPASS` | Hold magnetic heading |
| `MODE_WIND_AWA` | Hold apparent wind angle |
| `MODE_WIND_TWA` | Hold true wind angle |
| `MODE_VMG_UP` | Optimise VMG upwind |
| `MODE_VMG_DOWN` | Optimise VMG downwind |

All wind modes compute a target heading via the wind triangle, then the heading error drives the same PD controller.

### Pilot types

| Type | Description |
|------|-------------|
| `PILOT_PD` | Proportional-derivative (kp=1.0, kd=1.5) |
| `PILOT_PID` | Adds integral term (ki=0.1) |
| `PILOT_SMOOTH` | Wraps PD with speed-dependent gain scaling and asymmetric rate limiting |
| `PILOT_ADAPTIVE` | Wraps PD with condition-based gain adjustment |

### NMEA2000 PGNs

**Received (instruments → autopilot):**

| PGN | Name | Data |
|-----|------|------|
| 130306 | Wind Speed | AWA, AWS (apparent only on real bus) |
| 128259 | Boat Speed | STW |
| 129026 | COG/SOG Rapid | COG, SOG |
| 127250 | Vessel Heading | Magnetic heading from fluxgate |

**Transmitted (autopilot → bus):**

| PGN | Name | Data |
|-----|------|------|
| 127245 | Rudder | Actual position, commanded position |
| 127237 | Heading/Track Control | Steering mode, target heading |

**True wind computation:** The firmware computes TWA/TWS from apparent wind + STW using the wind triangle in `n2k.cpp:parse_wind()`. Real N2K buses typically only carry apparent wind from instruments.

### Source layout

```
src/
├── actuator.cpp        Rudder motor control (position P-loop, current limits)
├── imu.cpp/h           BNO055 IMU read (heading, roll, pitch, yaw_rate)
├── n2k.cpp/h           NMEA2000 parse/send, true wind computation
├── pilot_manager.cpp/h Pilot selection, mode management
├── web.cpp/h           ESP32 web UI (ESPAsyncWebServer)
├── polar.cpp/h         Polar diagram lookup
├── pilots/
│   ├── pilot_base.h    Abstract base: steer(features) → rudder_command
│   ├── pd_pilot.h      PD controller
│   ├── pid_pilot.h     PID controller
│   ├── smooth_pilot.h  Rate-limited wrapper
│   └── adaptive_pilot.h  Adaptive gains wrapper
├── hal/                HAL-level simulator (macOS native)
│   ├── hal_sim_main.cpp   Main loop (same cooperative structure as ESP32)
│   ├── sim_environment.*  Internal sailing dynamics
│   ├── sim_socket.*       TCP socket for external simulator
│   ├── NMEA2000_sim.*     In-memory CAN queues
│   ├── Wire.*             Mock I2C / BNO055 register map
│   ├── Arduino.h          Arduino API stubs
│   └── arduino_stubs.cpp  GPIO, ADC, PWM, Serial mocks
└── sim/                Application-level simulator
    ├── sim_main.cpp
    ├── sim_n2k.cpp     Direct state injection (no CAN)
    ├── sim_imu.cpp
    ├── sim_actuator.cpp
    └── sim_web.cpp     HTTP server (shared with hal_sim)
```

## Build

Requires [PlatformIO](https://platformio.org/). All builds from the `firmware/esp32_autopilot/` directory.

```bash
# ESP32-S3 firmware (flash to board)
pio run -e esp32s3
pio run -e esp32s3 -t upload

# Upload web UI to SPIFFS
pio run -e esp32s3 -t uploadfs

# Native tests (pilots only, runs on host)
pio test -e native

# Application-level simulator (macOS)
pio run -e sim

# HAL-level simulator (macOS, compiles real firmware code)
pio run -e hal_sim
```

### Build environments

| Environment | What compiles | Output |
|-------------|--------------|--------|
| `esp32s3` | Full firmware for ESP32-S3 | `.pio/build/esp32s3/firmware.bin` |
| `native` | Pilot tests only | test runner |
| `sim` | Firmware with app-level fakes | `.pio/build/sim/program` |
| `hal_sim` | Real `n2k.cpp`, `imu.cpp`, `actuator.cpp` with HAL mocks | `.pio/build/hal_sim/program` |

### Local libraries

The `lib/` directory contains local copies of dependencies used by the native builds (sim, hal_sim):

- `NMEA2000/` — NMEA2000 protocol library
- `Adafruit_BNO055/` — BNO055 IMU driver
- `Adafruit_BusIO/` — I2C/SPI abstraction
- `Adafruit_Sensor/` — Sensor base classes
- `httplib/` — Single-header HTTP server (cpp-httplib)

ESP32 builds pull their own copies via `lib_deps` in `platformio.ini`.

## Running the simulators

### Sim (application-level)

Self-contained simulator with web UI. Uses replacement implementations for N2K, IMU, and actuator — validates pilot logic but not hardware interfaces.

```bash
.pio/build/sim/program [--port 8080] [--tws 12] [--twd 225]
```

Open `http://localhost:8080/` to see the dashboard. Engage compass or wind mode, adjust target, and watch the pilot steer.

### HAL Sim (hardware-level)

Compiles and runs the real `n2k.cpp`, `imu.cpp`, and `actuator.cpp` unchanged against mock hardware (in-memory CAN queues, BNO055 register map, simulated ADC/PWM). Validates the full firmware stack including CAN frame encoding, I2C register reads, and ADC-based rudder feedback.

```bash
.pio/build/hal_sim/program [--port 8080] [--tws 12] [--twd 225] [--socket-port 9876]
```

When no external client is connected, an internal physics model drives the simulation. When a TCP client connects to the socket port, it switches to external mode — the client provides all sensor data and the internal model is bypassed.

### External simulator (Python → HAL Sim)

Connects to the HAL sim's socket port and runs the full Python `YachtDynamics` physics model in a closed loop with the firmware. The Python side encodes instrument data as real N2K CAN frames (PGNs 130306, 128259, 129026, 127250) plus BNO055 IMU registers, and reads back PGN 127245 (rudder) to feed into the next physics step.

```bash
# From the repo root:
uv run python scripts/external_sim.py [--tws 12] [--twd 225] [--host localhost] [--port 9876]
```

**Wire protocol** (see `src/hal/sim_socket.h` for details):

```
[type:1][length:2 LE][payload:length]

Type 0x01  Python→ESP  CAN frame  (4B CAN ID LE + 1B DLC + up to 8B data)
Type 0x02  ESP→Python  CAN frame  (same format)
Type 0x03  Python→ESP  IMU data   (6 × int16 LE: heading, roll, pitch, gyro_x, gyro_y, gyro_z)
                                   BNO055 raw units: 1 LSB = 1/16 degree or 1/16 dps
```

When the Python client disconnects, the HAL sim automatically reverts to its internal physics model.

## Web UI

The web dashboard (`data/index.html`) provides:

- Live sensor telemetry (heading, wind angle/speed, boat speed)
- Mode selector (standby, compass, AWA, TWA, VMG)
- Pilot type selector (PD, PID, Smooth, Adaptive)
- Rudder command vs actual position display
- Gain adjustment (kp, ki, kd)
- Rudder ADC calibration (center, port, starboard)
- Fault/status indicators

Served by ESPAsyncWebServer on the ESP32, or cpp-httplib in the native simulators.

## Sign conventions

These are critical — see the top-level `CLAUDE.md` for the authoritative reference.

| Signal | Positive | Negative |
|--------|----------|----------|
| Rudder angle | Starboard (right) | Port (left) |
| AWA / TWA | Wind from starboard | Wind from port |
| Heading | 0–360° clockwise from North | — |

Physical effects:
- **Positive rudder** → boat turns starboard → heading increases → AWA decreases
- **Negative rudder** → boat turns port → heading decreases → AWA increases


