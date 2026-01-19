# IMU Sensor Fusion Board

A compact PCB for the autopilot IMU sensor fusion subsystem, featuring an ATtiny3224 microcontroller and ICM-20948 9-DoF IMU.

## Overview

This board runs the Madgwick AHRS filter at 100Hz, outputting fused orientation data (heading, pitch, roll, rates) via serial to the Raspberry Pi. By offloading sensor fusion to a dedicated MCU, the autopilot achieves:

- Consistent 100Hz timing regardless of Pi CPU load
- Galvanic isolation from Pi electrical noise
- Compact, mountable form factor

## Specifications

| Parameter | Value |
|-----------|-------|
| MCU | ATtiny3224 @ 16MHz |
| IMU | ICM-20948 (external module) |
| Power Input | 5V (from Pi) |
| Power Consumption | ~15mA typical |
| Board Size | 20mm x 20mm |
| Serial Output | 115200 baud, NMEA-style |
| Update Rate | 100Hz |

## Schematic

```
                        +5V
                         │
                    ┌────┴────┐
        ┌───────────┤ AP2112K ├───────────┐
        │           │  3.3V   │           │
       ─┴─          └────┬────┘          ─┴─
       ─┬─ C1            │               ─┬─ C2
        │  10uF          │                │  10uF
       ───              ─┴─              ───
                        GND

                      +3V3
                        │
            ┌───────────┼───────────┐
            │           │           │
           ─┴─     ┌────┴────┐     ─┴─
           ─┬─ C3  │ATtiny   │     ─┬─ R1,R2
            │      │ 3224    │      │  4.7k
           ───     │         │      │
                   │ PA0─────┼──────┼──── UPDI
                   │ PA1─────┼──────┼──── SDA ────┐
                   │ PA2─────┼──────┴──── SCL ────┤
                   │ PA3─────┼── R3 ── LED ── GND │
                   │         │                    │
                   │ PB2─────┼──── TX ──── Pi RX  │
                   │ PB3─────┼──── RX ──── Pi TX  │
                   │         │                    │
                   └────┬────┘              ┌─────┘
                        │                   │
                       ───            ┌─────┴─────┐
                       GND            │ ICM-20948 │
                                      │  Module   │
                                      └───────────┘
```

## Connectors

### J1 - Serial (JST-SH 4-pin, 1mm pitch)

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | TX | Serial data out (to Pi RX) |
| 2 | RX | Serial data in (from Pi TX) |
| 3 | +5V | Power input |
| 4 | GND | Ground |

### J2 - UPDI Programming (2.54mm header)

| Pin | Signal |
|-----|--------|
| 1 | UPDI |
| 2 | VCC |
| 3 | GND |

### J3 - IMU (JST-SH 4-pin, 1mm pitch)

| Pin | Signal |
|-----|--------|
| 1 | VCC |
| 2 | GND |
| 3 | SDA |
| 4 | SCL |

## Bill of Materials

| Ref | Value | Package | Description |
|-----|-------|---------|-------------|
| U1 | ATtiny3224-SU | SOIC-14 | MCU |
| U2 | AP2112K-3.3 | SOT-23-5 | 3.3V LDO |
| C1, C2 | 10µF | 0805 | Power capacitors |
| C3, C4 | 100nF | 0603 | Decoupling capacitors |
| R1, R2 | 4.7kΩ | 0603 | I2C pull-ups |
| R3 | 330Ω | 0603 | LED current limit |
| D1 | Green | 0603 | Status LED |
| J1 | SM04B-SRSS-TB | JST-SH | Serial connector |
| J2 | 1x3 header | 2.54mm | UPDI programming |
| J3 | SM04B-SRSS-TB | JST-SH | IMU connector |

## Assembly Notes

1. **Solder SMD components first** - Use solder paste and hot air or reflow
2. **Install through-hole header J2 last**
3. **ICM-20948 module** - Connect via the JST cable to J3
4. **Orientation** - Arrow on silkscreen points forward (bow direction)

## Programming

Connect a UPDI programmer (e.g., jtag2updi) to J2:

```bash
cd firmware/mcu
pio run -t upload
```

## Testing

1. Connect serial to Pi (J1)
2. Power up (5V via J1)
3. LED should blink briefly on startup
4. Monitor serial output:
   ```bash
   picocom -b 115200 /dev/ttyUSB0
   ```
5. Expect NMEA-style messages:
   ```
   $IMU,127.3,-2.5,8.1,0.32,-0.15,0.45,0.12,-0.08,9.78*4A
   ```

## Magnetometer Calibration

With the board installed on the boat:

1. Send `$CAL,START*XX` via serial
2. Slowly rotate the boat through all headings (30 seconds)
3. Send `$CAL,SAVE*XX` to store calibration in EEPROM

## Files

```
hardware/imu-board/
├── imu-board.kicad_pro     # KiCad project
├── imu-board.kicad_sch     # Schematic
├── imu-board.kicad_pcb     # PCB layout
├── fabrication/
│   ├── BOM.csv             # Bill of materials
│   ├── CPL.csv             # Component placement
│   └── README.md           # Fabrication instructions
├── symbols/                # Custom schematic symbols
├── footprints/             # Custom footprints
└── README.md               # This file
```

## Design Files

- **KiCad version**: 7.0+
- **Reference design**: Based on [N2KEngine caninterfaceboard](https://github.com/ieb/N2KEngine/tree/main/pcb/caninterfaceboard)
