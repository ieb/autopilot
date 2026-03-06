# ESP32-S3 Consolidated Autopilot Module

## Context

The current autopilot hardware uses three separate processors:
- **Raspberry Pi 4**: Runs Python pilots (PD, PID, Smooth, Adaptive), ML model, NMEA2000 via socketCAN
- **ATtiny3224 (IMU MCU)**: ICM-20948 + Madgwick AHRS at 100Hz, serial output to Pi (`firmware/mcu/`)
- **ATtiny3226 (Actuator MCU)**: Rudder position control, BTS7960 H-bridge, EM clutch, serial commands from Pi (`firmware/actuator/`)

The PD/PID/Smooth/Adaptive pilots are computationally trivial (<200 FLOPs per update). The ML model is NOT required on the ESP32 — only classical control. Consolidating into a single ESP32-S3 module eliminates the Pi, two serial links, and reduces cost/complexity/power.

**Goal:** Implement a single ESP32-S3 PlatformIO project that combines IMU fusion, NMEA2000 CAN interface, pilot algorithms, actuator control, and a web configuration UI.

## Feasibility Assessment

| Aspect | Requirement | ESP32-S3 Capability | Verdict |
|--------|-------------|---------------------|---------|
| Compute | PD: 5 FLOPs, EKF: ~100 FLOPs, Smooth: ~60 FLOPs | 240MHz dual-core FPU, 1 core used for pilot | Massively overpowered |
| RAM   | Pilots: <1KB state, IMU+buffers: ~4KB, N2K: ~8KB, Web: ~32KB | 512KB SRAM + 8MB PSRAM | >10x headroom |
| Flash | Firmware ~200KB, web assets ~100KB | 16MB (typical dev board) | Fine |
| I2C   | BNO055 IMU (onboard fusion, no Madgwick needed) | Native I2C controller | Yes |
| CAN   | NMEA2000 @ 250kbps | TWAI controller (built-in), needs TJA1051 transceiver | Yes |
| ADC   | Rudder potentiometer | 12-bit SAR ADC | Yes |
| PWM   | BTS7960 H-bridge (2ch) | LEDC/MCPWM peripheral | Yes |
| GPIO  | Clutch, bridge enable, LED, status | 45 GPIOs available | Yes |
| WiFi  | Web config UI | Built-in 802.11 b/g/n | Yes |
| FPU   | EKF matrix ops (3x3), polar interpolation | Single-precision FPU | Yes |

**Verdict: Fully viable.**

## Architecture

### Single-Core Arduino Model

All application code runs on **Core 1 (APP_CPU)** via the standard Arduino `loop()`. Core 0 is reserved for WiFi/LWIP/system tasks managed by ESP-IDF. This avoids cross-core race conditions and keeps the programming model simple.

```
Core 0 (PRO_CPU - system):            Core 1 (APP_CPU - Arduino loop):
├─ WiFi/LWIP stack                    ├─ NMEA2000.ParseMessages()  [every loop]
├─ ESP-IDF system tasks               │    - ttlappalainen library handles CAN RX/TX
└─ (do not touch)                     │    - Message handlers update shared state
                                      │
                                      ├─ IMU read (20Hz)
                                      │    - BNO055 I2C read (fused heading/pitch/roll/rates)
                                      │    - No Madgwick — BNO055 does onboard fusion
                                      │
                                      ├─ Pilot update (5Hz)
                                      │    - Build features from current state
                                      │    - Run active pilot (PD/PID/Smooth/Adaptive)
                                      │    - Write rudder target
                                      │
                                      ├─ Actuator control (50Hz)
                                      │    - Read rudder ADC
                                      │    - Position P-controller → H-bridge PWM
                                      │    - Clutch management
                                      │
                                      └─ Web server (on-demand, async)
                                           - AsyncWebServer handles HTTP
                                           - Config/status/gains API
```

All timing via `millis()` checks in `loop()` (cooperative scheduling). No FreeRTOS task creation needed — the Arduino framework already runs on FreeRTOS under the hood.

### NMEA2000 PGNs

| Direction | PGN | Content | Rate |
|-----------|-----|---------|------|
| Receive | 130306 | Wind Data (AWA, AWS) | 1 Hz |
| Receive | 128259 | Speed (STW) | 1 Hz |
| Receive | 129026 | COG/SOG | 1 Hz |
| Receive | 127250 | Vessel Heading | 10 Hz |
| Send | 127245 | Rudder (target + actual) | 5 Hz |
| Send | 127237 | Heading/Track Control | 5 Hz |

Using `ttlappalainen/NMEA2000` library with `NMEA2000_esp32` driver. The library handles all CAN bus management (TWAI init, frame encoding/decoding, device address claiming, heartbeat) — application code only registers message handlers and calls `NMEA2000.ParseMessages()` in `loop()`. User has prior experience in `ieb/N2KNMEA0183Wifi`.

### Application State

All state lives in a single struct, updated and read within the same `loop()` on Core 1. No mutex needed (single-threaded cooperative scheduling). Only the AsyncWebServer callbacks need `volatile` for fields they read.

```cpp
struct AppState {
    // From BNO055 (updated at 20Hz)
    float heading;        // deg, 0-360
    float pitch, roll;    // deg
    float yaw_rate;       // deg/s (from gyro)
    float roll_rate;      // deg/s

    // From NMEA2000 message handlers
    float awa, aws;       // deg, kts
    float twa, tws;       // deg, kts (computed via wind triangle)
    float stw, sog, cog;  // kts, kts, deg
    uint32_t n2k_last_ms; // timestamp of last N2K message

    // From actuator control (updated at 50Hz)
    float rudder_actual;  // normalized -1..+1
    float rudder_velocity;// deg/s
    float motor_current;  // Amps
    float supply_voltage; // Volts

    // Pilot output
    float rudder_target;  // normalized -1..+1
    uint8_t pilot_mode;   // 0=standby, 1=compass, 2=wind_awa, 3=wind_twa, 4=vmg_up, 5=vmg_down
    float target_value;   // target heading/AWA/TWA (or computed from polar for VMG modes)
};
```

### Feature Vector Construction

The pilot needs a subset of the 22-feature vector. On ESP32, we construct a minimal struct directly (no numpy):

```cpp
struct PilotFeatures {
    float heading_error;    // feat[0]: error / 90.0
    float mode_flag;        // feat[1]: compass=0.0, awa=0.5, twa=1.0
    float heading_rate;     // feat[2]: rate / 30.0
    float roll;             // feat[3]: roll / 45.0 (for future use)
    float stw;              // feat[11]: stw / 25.0 (for SmoothPilot speed scaling)
    float rudder_position;  // feat[14]: zeroed (not used by PD/PID)
    float pd_suggestion;    // feat[19]: computed inline
};
```

The PD/PID/Smooth/Adaptive pilots only read features[0], [2], [11], and [19]. No need for the full 22-element vector.

## Key Implementation Details

### Pilot Port Strategy

Direct 1:1 port from Python. The pilots use only basic math (multiply, add, clip, abs) — no numpy dependencies.

**PDPilot** (from `src/pilots/pd_pilot.py`):
```cpp
class PDPilot {
    float kp = 1.0f, kd = 1.5f, max_rudder = 1.0f;
    float steer(const PilotFeatures& f) {
        float h_err = f.heading_error * 90.0f;
        float h_rate = f.heading_rate * 30.0f;
        float cmd = kp * h_err + kd * (-h_rate);
        return constrain(cmd / 25.0f, -max_rudder, max_rudder);
    }
};
```

**PIDPilot** (from `src/pilots/pid_pilot.py`): Same + `_integrator` with anti-windup.

**SmoothPilot** (from `src/pilots/smooth_pilot.py`): Speed scaling, rate limit with urgency/error scaling, turn-rate awareness, asymmetric rate limit. All uses of `np.clip` become `constrain()`.

**AdaptivePilot** (from `src/pilots/adaptive_pilot.py`): 3x3 EKF. Matrix ops done inline (no library needed for 3x3). The EKF update is ~100 FLOPs — trivial for ESP32 FPU.

### Heading Error Computation

All modes reduce to compass heading error (same as Python `helm_controller.py`):
- **Compass**: `error = target_heading - current_heading` (wrapped to +/-180)
- **Wind AWA**: Convert target AWA to target heading via wind triangle, then compass error
- **Wind TWA**: `heading_delta = current_TWA - target_TWA`, apply to heading
- **VMG Up**: Use polar to find optimal upwind TWA for current TWS, then steer as wind_twa mode
- **VMG Down**: Use polar to find optimal downwind TWA for current TWS, then steer as wind_twa mode

### Polar Diagram

Port `src/ml/polar.py:Polar` class to C++. The Pogo 1250 polar data (17 TWS x 24 TWA = 408 floats, ~1.6KB) is stored as `const` arrays in flash. Bilinear interpolation for `get_target_speed()` and brute-force search (45 iterations) for `get_optimal_vmg_upwind()`/`get_optimal_vmg_downwind()`.

The polar is used by:
1. **VMG modes**: Compute optimal TWA target from current TWS
2. **Performance display**: Show % of polar target on web dashboard
3. **Target TWA is recomputed every pilot cycle (5Hz)** to track changing wind conditions

### Actuator Control (from `firmware/actuator/src/main.cpp`)

Direct port of existing P-controller with:
- Position deadband (0.5 deg)
- Current limiting (soft 12A, hard 15A)
- Stall detection (high current + low velocity)
- Clutch soft-start (200ms ramp)
- Watchdog timeout (2s)

### NMEA2000 Integration

The `ttlappalainen/NMEA2000` library handles all CAN bus details. Application code is minimal:

```cpp
#include <NMEA2000_esp32.h>
#include <N2kMessages.h>

// Library manages TWAI driver, address claiming, heartbeat
tNMEA2000_esp32 NMEA2000(CAN_TX_PIN, CAN_RX_PIN);

void setup() {
    NMEA2000.SetProductInformation("00001", 100, "Autopilot", "1.0", "1.0");
    NMEA2000.SetDeviceInformation(1, 150, 40);  // Steering
    NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode);
    NMEA2000.SetMsgHandler(HandleN2kMsg);
    NMEA2000.Open();
}

void loop() {
    NMEA2000.ParseMessages();  // RX + TX scheduling
    // ... other periodic tasks
}

void HandleN2kMsg(const tN2kMsg &msg) {
    switch (msg.PGN) {
        case 130306: ParseWind(msg); break;
        case 128259: ParseSTW(msg); break;
        case 129026: ParseCOGSOG(msg); break;
        case 127250: ParseHeading(msg); break;
    }
}
```

Sending rudder/track PGNs uses the library's `SetN2kRudder()` and `SetN2kHeadingTrackControl()` helpers, called from the pilot update at 5Hz.

### Web Configuration UI

Minimal AsyncWebServer (ESP32 Arduino) serving SPIFFS:
- **GET /api/status** — JSON: heading, AWA, rudder, pilot mode, gains, faults
- **GET /api/gains** — JSON: current kp, ki, kd values
- **POST /api/gains** — Set gains (kp, kd, ki)
- **POST /api/mode** — Set pilot mode + target
- **POST /api/pilot** — Select pilot type (pd/pid/smooth/adaptive)
- **GET /** — Static HTML dashboard

### Persistence

Gains, pilot config, mag calibration, and rudder calibration stored in ESP32 NVS (Preferences library).

## File Structure

```
firmware/esp32_autopilot/
├── platformio.ini
├── include/
│   ├── config.h              # Pin definitions, timing constants
│   ├── app_state.h           # AppState struct, PilotFeatures, enums
│   └── polar_data.h          # Pogo 1250 polar as const arrays
├── src/
│   ├── main.cpp              # setup(), loop(), timing dispatch
│   ├── imu.h/.cpp            # BNO055 I2C read (fused output, no Madgwick)
│   ├── n2k.h/.cpp            # NMEA2000 setup + message handlers
│   ├── actuator.h/.cpp       # Rudder position control, clutch, H-bridge
│   ├── pilot_manager.h/.cpp  # Feature build, mode logic, pilot dispatch
│   ├── polar.h/.cpp          # Polar interpolation + VMG optimisation
│   ├── web.h/.cpp            # AsyncWebServer + API endpoints
│   └── pilots/
│       ├── pilot_base.h      # BasePilot interface
│       ├── pd_pilot.h/.cpp
│       ├── pid_pilot.h/.cpp
│       ├── smooth_pilot.h/.cpp
│       └── adaptive_pilot.h/.cpp
├── data/                     # SPIFFS web assets
│   └── index.html
└── test/
    └── test_pilots.cpp       # PlatformIO native unit tests
```

## Existing Code to Reuse

| Source | Destination | What |
|--------|-------------|------|
| `firmware/actuator/src/main.cpp` | `actuator.cpp` | Position control, clutch, current limits, stall detect |
| `firmware/actuator/src/config.h` | `include/config.h` | Control params, fault codes, limits |
| `src/pilots/pd_pilot.py` | `pilots/pd_pilot.cpp` | PD control law (kp=1.0, kd=1.5) |
| `src/pilots/pid_pilot.py` | `pilots/pid_pilot.cpp` | PID + anti-windup |
| `src/pilots/smooth_pilot.py` | `pilots/smooth_pilot.cpp` | Rate limiter, speed scaling, asymmetric limits |
| `src/pilots/adaptive_pilot.py` | `pilots/adaptive_pilot.cpp` | EKF gain tuner (3x3 inline matrix ops) |
| `src/ml/polar.py` | `polar.cpp` + `polar_data.h` | Pogo 1250 polar data, bilinear interp, VMG optimisation |

**Not reused:** `firmware/mcu/` (ICM-20948 + Madgwick). BNO055 does onboard fusion — just I2C reads for heading/pitch/roll/rates. Use Adafruit BNO055 library or direct register access.

## Implementation Order

1. **Scaffolding**: `platformio.ini`, `config.h`, `app_state.h`, `main.cpp` (empty loop with timing)
2. **Pilots**: Port PD -> PID -> Smooth -> Adaptive (each testable standalone)
3. **Polar**: Port `polar.py` to C++ with const data arrays, bilinear interp, VMG search
4. **Unit tests**: `test/test_pilots.cpp` — verify numerical equivalence with Python
5. **IMU**: BNO055 I2C read (Adafruit library or direct registers) — heading, pitch, roll, rates
6. **Actuator**: Port from `firmware/actuator/`, adapt ADC/PWM for ESP32 LEDC
7. **N2K**: NMEA2000 setup with `ttlappalainen/NMEA2000`, message handlers
8. **Pilot manager**: Feature construction, mode management (including VMG with polar), pilot dispatch
9. **Web**: AsyncWebServer with status/config/gains API
10. **Integration**: Full system test on bench with CAN bus + BNO055 + motor

## Hardware Connections

```
ESP32-S3 DevKitC
├── I2C (GPIO 8/9): BNO055 IMU (onboard AHRS fusion)
├── TWAI CAN (GPIO 4/5): TJA1051 transceiver -> NMEA2000 bus
├── ADC (GPIO 1): Rudder potentiometer
├── PWM (GPIO 15/16): BTS7960 H-bridge (RPWM/LPWM)
├── GPIO 17: BTS7960 R_EN/L_EN (bridge enable)
├── GPIO 18: EM clutch (via N-FET)
├── GPIO 48: Status LED (onboard NeoPixel on most dev boards)
└── USB: Programming + serial debug
```

Pin assignments are configurable in `config.h`. Final PCB layout determines actual pins.

## Verification

```bash
# Build (no hardware needed)
cd firmware/esp32_autopilot
pio run -e esp32s3

# Run native unit tests (pilots only, on host)
pio test -e native

# Upload to board
pio run -e esp32s3 -t upload

# Monitor serial output
pio device monitor -b 115200
```

### Numerical Equivalence Tests

C++ pilots must produce identical output to Python pilots for the same inputs:
```cpp
void test_pd_matches_python() {
    PDPilot pd;
    PilotFeatures f = {.heading_error = 5.0f/90.0f, .heading_rate = 0.0f};
    float cmd = pd.steer(f);
    // Python: kp=1.0, 5.0 * 1.0 / 25.0 = 0.2
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, cmd);
}
```

### On-Boat Integration

1. Connect BNO055 via I2C, verify heading/pitch/roll via serial debug
2. Connect ESP32 to NMEA2000 bus via TJA1051 transceiver
3. Verify PGN reception (wind, speed, COG/SOG) via serial debug
4. Connect rudder pot + H-bridge + clutch
5. Run in compass mode, compare heading hold to Pi-based system
6. Test VMG upwind mode — verify polar target TWA updates with wind changes

## Not In Scope

- ML model inference (stays on Pi if needed)
- OTA firmware updates (add later)
- Multi-device N2K autopilot coordination
- Custom PCB design (use dev board + breakout for prototype)
