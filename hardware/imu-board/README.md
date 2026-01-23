# IMU Sensor Fusion Board

ATtiny3224-based IMU sensor fusion board with ICM-20948 9-DoF IMU and Madgwick AHRS filter.

## Overview

This board provides real-time attitude estimation (heading, pitch, roll) at 100Hz, offloading the timing-critical sensor fusion from the Raspberry Pi.

**Key Features:**
- 100Hz Madgwick AHRS sensor fusion
- 9-axis IMU (accelerometer, gyroscope, magnetometer)
- NMEA-style serial output at 115200 baud
- Magnetometer calibration support
- Compact 20x20mm form factor
- 3.3V operation from 5V input

## Schematic

The schematic (`imu-board.kicad_sch`) includes detailed design notes and calculations. Open in KiCad 9 for full details.

### Block Diagram

```
                    +5V from Pi
                         │
                    ┌────┴────┐
                    │ AP2112K │
                    │  3.3V   │
                    └────┬────┘
                         │
         ┌───────────────┼───────────────┐
         │               │               │
    ┌────┴────┐    ┌─────┴─────┐    ┌────┴────┐
    │ ATtiny  │    │ ICM-20948 │    │   LED   │
    │  3224   │◄──►│   IMU     │    │  (PA3)  │
    └────┬────┘    └───────────┘    └─────────┘
         │
    ┌────┴────┐
    │ Serial  │
    │  to Pi  │
    └─────────┘
```

### Pin Mapping

| ATtiny3224 Pin | Function | Connection |
|----------------|----------|------------|
| PA0 | UPDI | Programming header J2 |
| PA3 | LED | Status LED (via 330Ω) |
| PB0 | SCL | ICM-20948 SCL (with 4.7k pull-up) |
| PB1 | SDA | ICM-20948 SDA (with 4.7k pull-up) |
| PB2 | TX | Serial to Pi RX |
| PB3 | RX | Serial from Pi TX |
| VCC | Power | 3.3V from LDO |
| GND | Ground | Common ground |

## Components

### Bill of Materials

| Ref | Value | Package | Description | LCSC |
|-----|-------|---------|-------------|------|
| U1 | ATtiny3224-SU | SOIC-14 | MCU, 32KB Flash, 3KB RAM | C2682858 |
| U2 | AP2112K-3.3 | SOT-23-5 | 600mA LDO Regulator | C51118 |
| C1 | 10µF | 0805 | Input capacitor, X5R, 10V | C15850 |
| C2 | 10µF | 0805 | Output capacitor, X5R, 6.3V | C15850 |
| C3 | 100nF | 0603 | MCU decoupling, X7R | C14663 |
| R1 | 4.7kΩ | 0603 | I2C SDA pull-up | C23162 |
| R2 | 4.7kΩ | 0603 | I2C SCL pull-up | C23162 |
| R3 | 330Ω | 0603 | LED current limiter | C23138 |
| D1 | Green | 0603 | Status LED | C72043 |
| J1 | JST-SH 4P | SMD | Serial connector to Pi | C160404 |
| J2 | Header 1x3 | 2.54mm | UPDI programming | C492405 |
| J3 | JST-SH 4P | SMD | IMU module connector | C160404 |

**External Module (not on PCB):**
- ICM-20948 breakout module (SparkFun, Adafruit, or generic)

### Power Supply

The AP2112K-3.3 LDO provides:
- Input: 4.5V - 6V (5V nominal from Pi)
- Output: 3.3V @ 600mA max
- Dropout: 250mV @ 600mA
- Quiescent: 55µA typical

**Power Budget:**
| Component | Current |
|-----------|---------|
| ATtiny3224 @ 16MHz | ~5mA |
| ICM-20948 (all sensors) | ~3mA |
| Status LED | ~4mA |
| **Total** | **~12mA** |

### I2C Bus

- Speed: 400kHz (Fast Mode)
- Pull-ups: 4.7kΩ to 3.3V
- Address: 0x68 (ICM-20948 with AD0 to GND)

Keep I2C traces short (<50mm) and route as a differential pair.

## Connectors

### J1 - Serial to Raspberry Pi (JST-SH 4-pin)

| Pin | Signal | Pi GPIO |
|-----|--------|---------|
| 1 | GND | GND |
| 2 | VCC (5V in) | 5V (Pin 2 or 4) |
| 3 | TX (MCU out) | RX (GPIO15) |
| 4 | RX (MCU in) | TX (GPIO14) |

### J2 - UPDI Programming (2.54mm header)

| Pin | Signal |
|-----|--------|
| 1 | UPDI (PA0) |
| 2 | VCC (3.3V) |
| 3 | GND |

### J3 - IMU Module (JST-SH 4-pin)

| Pin | Signal |
|-----|--------|
| 1 | VCC (3.3V) |
| 2 | GND |
| 3 | SDA |
| 4 | SCL |

## PCB Layout

**Specifications:**
- Dimensions: 20mm × 20mm
- Layers: 2
- Thickness: 1.6mm
- Copper: 1oz (35µm)
- Finish: HASL or ENIG

**Mounting:**
- 4× M2.5 holes at corners (17mm spacing)

**Layout Guidelines:**
1. Place MCU in center
2. LDO near power input
3. I2C pull-ups close to MCU
4. Decoupling caps adjacent to IC power pins
5. Ground pour on bottom layer

## Firmware Alignment

This hardware design is aligned with:
- `firmware/mcu/src/config.h` - Pin definitions
- `firmware/mcu/src/main.cpp` - Main firmware

**Verification Checklist:**

| Parameter | Hardware | Firmware | Match |
|-----------|----------|----------|-------|
| LED Pin | PA3 | LED_PIN = PIN_PA3 | ✓ |
| I2C SDA | PA1 | Wire library default | ✓ |
| I2C SCL | PA2 | Wire library default | ✓ |
| UART TX | PB2 | Serial default | ✓ |
| UART RX | PB3 | Serial default | ✓ |
| I2C Address | 0x68 (AD0=GND) | ICM20948_I2C_ADDR | ✓ |
| Baud Rate | 115200 | SERIAL_BAUD_RATE | ✓ |

## Assembly

### SMD Components

1. Apply solder paste to pads (stencil recommended)
2. Place components with tweezers
3. Reflow solder (hot plate or reflow oven)
4. Inspect joints under magnification

### Through-Hole Components

1. Solder J2 programming header after SMD reflow
2. Use flux for clean joints

### IMU Module

1. Connect ICM-20948 module to J3 with JST-SH cable
2. Secure module with double-sided tape or mounting hardware
3. Orient module with axes aligned to boat reference frame

## Programming

### Using SerialUPDI

```bash
# Connect USB-Serial adapter with 4.7k resistor on TX line
# Install PlatformIO
cd firmware/mcu

# Build and upload
pio run -e attiny3224 -t upload

# Monitor output
pio device monitor -b 115200
```

### Using ATMEL-ICE

```bash
pio run -e attiny3224 -t upload --upload-port usb
```

## Testing

### Power-On Test

1. Apply 5V to J1 (pins 1, 2)
2. Verify 3.3V on U2 output
3. LED D1 should light solid (or blink if IMU not connected)

### Serial Output Test

1. Connect to Pi serial port
2. Run: `screen /dev/ttyAMA0 115200`
3. Verify `$IMU,...*XX` messages at 100Hz

### IMU Test

```bash
# Check for valid heading, pitch, roll values
# Rotate board and verify angles change correctly
```

### Magnetometer Calibration

1. Send: `$CAL,START*XX`
2. Slowly rotate board through all orientations (30 seconds)
3. Send: `$CAL,SAVE*XX`
4. Verify heading accuracy against known reference

## Files

```
hardware/imu-board/
├── imu-board.kicad_pro     # KiCad project
├── imu-board.kicad_sch     # Schematic (with design notes)
├── imu-board.kicad_pcb     # PCB layout (to be created)
├── README.md               # This file
└── fabrication/
    ├── BOM.csv             # Bill of materials
    ├── CPL.csv             # Component placement
    └── README.md           # Fabrication instructions
```

## Revision History

| Rev | Date | Changes |
|-----|------|---------|
| 1.0 | 2026-01-20 | Initial design, aligned with firmware |
