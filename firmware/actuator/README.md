# Actuator Controller Firmware

ATtiny3226 firmware for closed-loop rudder control with electromagnetic clutch management.

## Overview

This MCU handles the "Layer 1" hardware protection for the autopilot system:

- **Position Control**: Closed-loop P-controller drives rudder to target angle
- **Clutch Management**: Engages/disengages electromagnetic clutch with soft-start
- **Current Monitoring**: Protects motor and H-bridge from overload (INA219 or ADC)
- **Watchdog Timeout**: Auto-disengages clutch if no commands received
- **Serial Interface**: Communicates with Raspberry Pi at 115200 baud

## Current Sensing Options

The firmware supports two current sensing methods, selectable at compile time:

### Option A: INA219 I2C Module (Recommended, Default)

```bash
pio run -e attiny3226          # Build with INA219 support
```

- Uses INA219 I2C current/voltage monitor
- Factory calibrated, temperature compensated
- High-side sensing (no ground offset issues)
- Also provides bus voltage reading
- Requires external 0.01Ω shunt for 20A range

### Option B: Direct ADC with Shunt + Op-Amp

```bash
pio run -e attiny3226_adc      # Build with ADC support
```

- Uses discrete shunt resistor + differential amplifier
- Lower cost but requires careful layout
- Subject to temperature drift and calibration issues
- Low-side sensing with potential ground offset

## Hardware

### Pin Allocation (ATtiny3226)

**With INA219 (Option A - Default):**

| Pin | Function | Description |
|-----|----------|-------------|
| PA0 | UPDI | Programming interface |
| PA1 | ADC_RUDDER | Rudder potentiometer input |
| PA2 | SDA | I2C data to INA219 |
| PA3 | SCL | I2C clock to INA219 |
| PA4 | PWM_PORT | TCA0 WO4 - Port drive to H-Bridge |
| PA5 | PWM_STBD | TCA0 WO5 - Starboard drive to H-Bridge |
| PA6 | CLUTCH | N-FET gate for electromagnetic clutch |
| PA7 | LED | Status indicator |
| PB2 | TX | Serial to Pi RX |
| PB3 | RX | Serial from Pi TX |

**With Direct ADC (Option B):**

| Pin | Function | Description |
|-----|----------|-------------|
| PA0 | UPDI | Programming interface |
| PA1 | ADC_RUDDER | Rudder potentiometer input |
| PA2 | ADC_CURRENT | Motor current sense (shunt + op-amp) |
| PA3 | ADC_VOLTAGE | Supply voltage sense (divider) |
| PA4 | PWM_PORT | TCA0 WO4 - Port drive to H-Bridge |
| PA5 | PWM_STBD | TCA0 WO5 - Starboard drive to H-Bridge |
| PA6 | CLUTCH | N-FET gate for electromagnetic clutch |
| PA7 | LED | Status indicator |
| PB2 | TX | Serial to Pi RX |
| PB3 | RX | Serial from Pi TX |

### Current Sensing

Using a 0.01Ω shunt resistor with 50x gain op-amp:
- 1A = 0.5V at ADC
- 15A = 7.5V (clamped to VDD)

### Voltage Sensing

Using 4:1 voltage divider:
- 12V system = 3V at ADC

## Serial Protocol

### Commands (Pi → MCU)

```
$RUD,<target>,<engage>*XX    Set target angle and clutch state
  target: -1.0 to +1.0 (normalized, ±1 = ±30°)
  engage: 0 = disengage clutch, 1 = engage clutch
  
$CFG,TIMEOUT,<ms>*XX         Set watchdog timeout (default 2000ms)

$CAL,CENTER*XX               Calibrate current position as center (0°)
$CAL,PORT*XX                 Calibrate current position as port limit
$CAL,STBD*XX                 Calibrate current position as starboard limit
```

### Status (MCU → Pi at 20Hz)

```
$STS,<target>,<actual>,<clutch>,<voltage>,<current>,<fault>*XX
  target:  Commanded angle (-1.0 to +1.0)
  actual:  Current angle (-1.0 to +1.0)
  clutch:  0 = disengaged, 1 = engaged
  voltage: Supply voltage in Volts
  current: Motor current in Amps
  fault:   Fault code (see below)
```

### Fault Codes

| Code | Name | Description |
|------|------|-------------|
| 0 | NONE | No fault |
| 1 | OVERCURRENT | Motor current exceeded 15A |
| 2 | STALL | High current but no movement for 500ms |
| 3 | POSITION_LIMIT | At software position limit (±28°) |
| 4 | SENSOR | ADC read failure |
| 5 | WATCHDOG | No command received within timeout |

### Checksum

XOR of all characters between `$` and `*` (exclusive), as 2-digit hex.

## Safety Limits

These are enforced by the MCU regardless of Pi commands:

| Protection | Soft Limit | Hard Limit | Action |
|------------|------------|------------|--------|
| Position | - | ±28° | Stop motor |
| Current | 12A | 15A | Reduce PWM / Stop |
| Stall | - | 10A + <0.5°/s for 500ms | Stop motor |
| Watchdog | - | 2000ms (configurable) | Disengage clutch |

## Building

```bash
cd firmware/actuator
pio run -e attiny3226
```

## Uploading

Requires UPDI programmer (e.g., SerialUPDI adapter):

```bash
pio run -e attiny3226 -t upload
```

## Calibration

1. Move rudder to center position
2. Send `$CAL,CENTER*XX` (calculate checksum)
3. Move rudder to full port
4. Send `$CAL,PORT*XX`
5. Move rudder to full starboard
6. Send `$CAL,STBD*XX`

Calibration is stored in EEPROM and persists across power cycles.
