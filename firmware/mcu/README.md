# IMU MCU Firmware

ATtiny3224-based IMU processor with ICM-20948 and Madgwick AHRS filter.

## Purpose

This firmware reads the ICM-20948 9-DoF IMU at 100Hz, runs a Madgwick sensor fusion algorithm, and outputs fused orientation data over serial. This offloads the timing-critical sensor fusion from the Raspberry Pi and provides galvanic isolation.

## Hardware

- **MCU**: ATtiny3224 @ 16MHz (14-pin SOIC)
- **IMU**: ICM-20948 (accelerometer, gyroscope, magnetometer)
- **Interface**: Serial (115200 baud) to Raspberry Pi

The ATtiny3224 (14-pin) is used instead of ATtiny3226 (20-pin) for a smaller footprint. Both have identical flash (32KB), RAM (3KB), and peripherals.

### Connections

| ATtiny3224 Pin | Function | Connection |
|----------------|----------|------------|
| PA0 | UPDI | Programming interface |
| PA1 (SDA) | I2C Data | ICM-20948 SDA |
| PA2 (SCL) | I2C Clock | ICM-20948 SCL |
| PA3 | LED | Status indicator |
| PB2 (TX) | Serial Out | Pi RX |
| PB3 (RX) | Serial In | Pi TX |
| VCC | Power | 3.3V |
| GND | Ground | GND |

## Building

Requires PlatformIO:

```bash
cd firmware/mcu

# Build
pio run

# Upload (adjust upload_port in platformio.ini)
pio run -t upload

# Monitor serial output
pio device monitor
```

## Serial Protocol

### Output (MCU → Pi)

NMEA-style messages at 100Hz:

```
$IMU,heading,pitch,roll,yaw_rate,pitch_rate,roll_rate,ax,ay,az*checksum\r\n
```

| Field | Unit | Range | Description |
|-------|------|-------|-------------|
| heading | degrees | 0-360 | Magnetic heading |
| pitch | degrees | -90 to +90 | Bow up positive |
| roll | degrees | -180 to +180 | Starboard down positive |
| yaw_rate | deg/s | | Rate of heading change |
| pitch_rate | deg/s | | Rate of pitch change |
| roll_rate | deg/s | | Rate of roll change |
| ax | m/s² | | Forward acceleration |
| ay | m/s² | | Starboard acceleration |
| az | m/s² | | Down acceleration |
| checksum | hex | | XOR of chars between $ and * |

Example:
```
$IMU,127.3,-2.5,8.1,0.32,-0.15,0.45,0.12,-0.08,9.78*4A
```

### Commands (Pi → MCU)

```
$CAL,START*XX   - Start magnetometer calibration (rotate slowly for 30s)
$CAL,SAVE*XX    - Save calibration to EEPROM
$CAL,STOP*XX    - Stop calibration early
```

## Calibration

The magnetometer requires calibration to compensate for hard-iron and soft-iron distortions from the boat.

1. Send `$CAL,START*XX`
2. Slowly rotate the sensor through all orientations (30 seconds)
3. Send `$CAL,SAVE*XX` to store calibration in EEPROM

The calibration calculates:
- **Hard-iron offsets**: Center of the magnetic sphere
- **Soft-iron scales**: Correction for sphere distortion

## Configuration

Edit `src/config.h` to adjust:

- IMU update rate (default 100Hz)
- Serial output rate (default 100Hz)
- Madgwick filter gain (beta)
- Sensor ranges

## Resource Usage (ATtiny3224)

- Flash: ~20KB of 32KB
- RAM: ~1.5KB of 3KB
- CPU: ~50% at 100Hz update rate

## Alternative: ATtiny3226

The firmware also supports the ATtiny3226 (20-pin). Build with:

```bash
pio run -e attiny3226
```

Note: On ATtiny3226, the LED can optionally use PA7 instead of PA3.

## Files

```
firmware/mcu/
├── platformio.ini      # Build configuration
├── src/
│   ├── main.cpp        # Main application
│   ├── config.h        # Configuration
│   ├── ICM20948.h      # IMU driver
│   ├── MadgwickAHRS.h  # AHRS filter
│   └── version.h       # (generated)
└── README.md           # This file
```
