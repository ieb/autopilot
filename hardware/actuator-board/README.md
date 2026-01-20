# Actuator Controller Board

PCB design for the ATtiny3226-based rudder actuator controller.

## Overview

This board provides:
- Closed-loop rudder position control
- Electromagnetic clutch management
- Motor current monitoring
- Voltage monitoring
- Serial interface to Raspberry Pi

## Schematic

Open `actuator-board.kicad_sch` in KiCad 9 for the full schematic with design notes.

---

## Current Sensing Analysis

### The Problem

We need to monitor motor current (0-15A typical, 20A peak) for:
- Overload protection
- Stall detection
- Load monitoring

### Option A: Discrete Shunt + Op-Amp

```
    Motor-  ────┬──────────────────────────── GND
                │
             ┌──┴──┐
             │R_sh │  0.002Ω 1% 5W
             └──┬──┘
                │
         V_shunt (30mV at 15A)
                │
    ┌───────────┴───────────┐
    │                       │
    │    Differential Amp   │
    │    (MCP6002, G=50)    │
    │                       │
    └───────────┬───────────┘
                │
           V_out = 1.5V at 15A
                │
           To PA2 (ADC)
```

**Circuit Details:**
```
                     R3 = 10k
                ┌────/\/\/────┐
                │             │
    Shunt+  ────┼───┤-   U3A  ├───┬──── V_out
                │   │  MCP6002│   │
                │ ┌─┤+        │   R2 = 500k
    Shunt-  ────┼─┘ └─────────┘   │
                │                 │
                └───/\/\/────┬────┴──── GND
                    R4=10k   R1=10k
```

**Gain Calculation:**
- G = R2/R1 = 500k/10k = 50
- V_shunt = I × R_sh = 15A × 0.002Ω = 30mV
- V_out = 30mV × 50 = 1.5V ✓

**Pros:**
- Low cost (~$0.50 total)
- No I2C overhead
- Fast response (direct ADC)

**Cons:**
- ⚠️ Layout critical - noise pickup
- ⚠️ Temperature drift affects accuracy
- ⚠️ Gain calibration required
- ⚠️ Ground offset errors (low-side sensing)
- ⚠️ Shunt power dissipation (0.45W at 15A)

### Option B: INA219 I2C Module (Recommended)

```
    12V ────┬───────────────────────┬──── Motor+
            │                       │
            │      ┌───────────┐    │
            └──────┤VIN+       ├────┘
                   │           │
            ┌──────┤VIN-       │
            │      │  INA219   │
    Motor-  ┴      │           │
                   │ SDA ──────┼──── PA2 (I2C)
                   │ SCL ──────┼──── PA3 (I2C)
                   │ VCC ──────┼──── 3.3V
                   │ GND ──────┼──── GND
                   └───────────┘
```

For >10A applications, use external shunt:
- Internal shunt: 0.1Ω (max ~3.2A)
- External shunt: 0.01Ω for 15A, 0.005Ω for 30A

**Pros:**
- ✓ Factory calibrated
- ✓ Temperature compensated
- ✓ High-side sensing (no ground issues)
- ✓ Reports current, voltage, and power
- ✓ 12-bit resolution
- ✓ No analog layout concerns
- ✓ Breakout modules readily available

**Cons:**
- Higher cost ($3-5 for module)
- Requires I2C (but ATtiny3226 has I2C available)
- ~1ms per reading (vs ~20µs for ADC)

### Recommendation

**Use INA219 for v1 prototype.**

The discrete approach requires:
1. Precision resistor matching
2. Careful PCB layout with short, thick traces
3. Kelvin connections to shunt
4. Temperature compensation algorithm
5. Gain calibration procedure

In a marine environment with vibration, temperature swings, and EMI from the motor, the INA219's integrated solution is more reliable.

### INA219 Configuration

For 15A max with 0.01Ω external shunt:

```cpp
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

void setup() {
    ina219.begin();
    // Calibrate for 0.01 ohm shunt, 32V max, 3.2A range
    // With 0.01 ohm shunt, actual max is 32A
    ina219.setCalibration_32V_2A();  // Adjust as needed
}

float readCurrent() {
    return ina219.getCurrent_mA() / 1000.0;  // Convert to Amps
}
```

---

## Voltage Sensing

Simple resistor divider with overvoltage protection:

```
    12V ───┬──── R5 = 30kΩ ────┬──── PA3 (ADC)
           │                   │
           │                  ─┴─ C5 = 100nF
           │                  ─┬─
           │                   │
           │    D1 = 3.3V ────┼──── Zener clamp
           │    Zener         │
           │                   │
           └──── R6 = 10kΩ ────┴──── GND
```

V_out = V_in × R6/(R5+R6) = 12V × 10k/40k = 3.0V

For 14.4V charging: V_out = 3.6V (Zener clamps to 3.3V)

---

## Clutch Driver

N-FET low-side switch with flyback protection:

```
    12V ─────┬──────────────────────┬──── (to clutch coil)
             │                      │
             │      ┌────────┐      │
             │      │ Clutch │      │
             │      │ Coil   │──────┘
             │      │ ~1A    │
             │      └────┬───┘
             │           │
         D2 ─┴─ ◄────────┤  Schottky flyback
             │           │
             │       ┌───┴───┐
             │       │   D   │
             │       │ Q1    │  IRLML6344
    PA6 ─────┼── R7 ─┤ G     │  N-FET
             │ 100Ω  │       │
             │       │   S   │
             │       └───┬───┘
             │           │
    GND ─────┴───────────┴─────────
```

- Q1: IRLML6344 - 30V, 5A, Rds(on) = 29mΩ
- D2: SS34 Schottky - catches inductive spike
- R7: 100Ω limits gate current

---

## Pin Usage Summary

| ATtiny3226 Pin | Function | Notes |
|----------------|----------|-------|
| PA0 | UPDI | Programming |
| PA1 | ADC1 | Rudder potentiometer |
| PA2 | ADC2 / SDA | Current sense (Option A) or I2C (Option B) |
| PA3 | ADC3 / SCL | Voltage sense (Option A) or I2C (Option B) |
| PA4 | PWM | TCA0 WO4 - Port drive |
| PA5 | PWM | TCA0 WO5 - Starboard drive |
| PA6 | GPIO | Clutch FET gate |
| PA7 | GPIO | Status LED |
| PB2 | TX | Serial to Pi |
| PB3 | RX | Serial from Pi |

**Note:** If using INA219 (Option B), voltage sensing moves to I2C as well (INA219 reads bus voltage), freeing PA3 for other use.

---

## PCB Considerations

- 2-layer board, 35µm copper minimum
- Power traces: 1mm+ for motor current path
- Keep shunt traces short and thick (if using Option A)
- Ground plane on bottom layer
- Decoupling caps close to MCU and op-amp
- JST connectors for reliable marine connections
- Conformal coating recommended for marine environment

---

## Bill of Materials

| Ref | Value | Package | Description |
|-----|-------|---------|-------------|
| U1 | ATtiny3226 | SOIC-20 | MCU |
| U2 | AP2112K-3.3 | SOT-23-5 | 3.3V LDO |
| U3 | INA219 | Module | Current/voltage monitor (Option B) |
| Q1 | IRLML6344 | SOT-23 | Clutch N-FET |
| D1 | BZT52C3V3 | SOD-123 | 3.3V Zener (voltage clamp) |
| D2 | SS34 | SMA | Flyback Schottky |
| D3 | LED Green | 0603 | Status LED |
| R5 | 30kΩ | 0603 | Voltage divider |
| R6 | 10kΩ | 0603 | Voltage divider |
| R7 | 100Ω | 0603 | Gate resistor |
| R8 | 330Ω | 0603 | LED resistor |
| C1,C2 | 10µF | 0805 | LDO caps |
| C3 | 100nF | 0603 | MCU decoupling |
| C5 | 100nF | 0603 | Voltage sense filter |
| J1 | JST-SH 4-pin | SMD | Serial to Pi |
| J2 | JST-XH 6-pin | THT | H-Bridge |
| J3 | JST-XH 2-pin | THT | Clutch |
| J4 | JST-XH 3-pin | THT | Rudder pot |
| J5 | 5mm terminal | THT | Power input |
| J6 | 1x3 header | 2.54mm | UPDI programming |

---

## Next Steps

1. Review current sensing decision (INA219 recommended)
2. Open schematic in KiCad 9
3. Add symbols from KiCad libraries
4. Create PCB layout
5. Generate Gerbers for fabrication
