# IMU MCU Firmware

ATtiny3224-based IMU processor with ICM-20948 and Madgwick AHRS filter.

## Purpose

This firmware reads the ICM-20948 9-DoF IMU at 100Hz, runs a Madgwick sensor fusion algorithm, and outputs fused orientation data over serial. This offloads the timing-critical sensor fusion from the Raspberry Pi and provides galvanic isolation.

## Simplex Protocol

The firmware uses a three-phase communication protocol that enables simplex RS-485:

1. **Startup**: IMU initializes and sends version/ready prompt
2. **Configuration**: IMU receives configuration from Pi (bidirectional)
3. **Streaming**: IMU sends data only (simplex, no command processing)

This design allows:
- Hot-swappable IMU (configuration comes from Pi, not stored in EEPROM)
- Single twisted pair RS-485 after configuration phase
- Centralized configuration management on Pi

To reconfigure, power cycle the IMU.

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

### Phase 1: Startup Sequence (IMU → Pi)

On power-up or reset, the IMU sends:

```
$VER,<git_sha>*XX    - Firmware version
$RDY*XX              - Ready for configuration
```

The Pi must configure the IMU within 30 seconds or it enters error state.

### Phase 2: Configuration Commands (Pi → IMU)

| Command | Description |
|---------|-------------|
| `$OFF,x,y,z*XX` | Set mounting offset (meters) |
| `$MAG,ox,oy,oz,sx,sy,sz*XX` | Set magnetometer calibration |
| `$CAL,START*XX` | Start interactive mag calibration |
| `$CAL,STOP*XX` | Stop calibration, return values |
| `$START*XX` | Begin streaming (enter simplex mode) |

### Configuration Responses (IMU → Pi)

| Response | Description |
|----------|-------------|
| `$ACK*XX` | Command accepted |
| `$OFS,x,y,z*XX` | Offset confirmation |
| `$MGC,ox,oy,oz,sx,sy,sz*XX` | Mag calibration values |
| `$ERR,msg*XX` | Error message |

### Phase 3: Streaming (IMU → Pi only)

After `$START`, the IMU streams data at 100Hz:

```
$IMU,heading,pitch,roll,yaw_rate,pitch_rate,roll_rate,ax,ay,az*XX
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

**No commands are processed in streaming mode.** To reconfigure, power cycle the IMU.

## Typical Startup Sequence

```
# IMU powers on, sends:
$VER,abc1234*XX
$RDY*XX

# Pi sends configuration:
$OFF,3.0,0.5,0.0*XX     # Mount offset
$MAG,12.5,-3.2,8.1,1.02,0.98,1.01*XX  # Mag calibration
$START*XX               # Begin streaming

# IMU responds and starts streaming:
$OFS,3.000,0.500,0.000*XX
$MGC,12.50,-3.20,8.10,1.020,0.980,1.010*XX
$ACK*XX
$IMU,127.3,-2.5,8.1,0.32,-0.15,0.45,0.12,-0.08,9.78*XX
$IMU,...
```

## Magnetometer Calibration

The magnetometer requires calibration to compensate for hard-iron and soft-iron distortions.

### Option 1: Send Pre-Calibrated Values

If calibration values are already known:

```
$MAG,12.5,-3.2,8.1,1.02,0.98,1.01*XX
```

Format: `$MAG,offsetX,offsetY,offsetZ,scaleX,scaleY,scaleZ*XX`

### Option 2: Interactive Calibration

1. Send `$CAL,START*XX`
2. Slowly rotate the sensor through all orientations (30 seconds)
3. Send `$CAL,STOP*XX`
4. IMU responds with calculated values: `$MGC,ox,oy,oz,sx,sy,sz*XX`
5. Store these values on the Pi for future use

## IMU Mounting Offset

When the IMU is mounted away from the boat's center of rotation, rotational motion creates spurious accelerations:

- **Centripetal acceleration**: `a = ω² × r`
- **Tangential acceleration**: `a = α × r`

### Configuration

Send the offset at startup:

```
$OFF,3.0,0.5,0.0*XX
```

Where:
- `x`: Forward offset in meters (positive = toward bow)
- `y`: Starboard offset in meters (positive = toward starboard)
- `z`: Down offset in meters (positive = below waterline)

## Error Handling

### Configuration Timeout

If no `$START` command is received within 30 seconds of power-up, the IMU enters error state:
- LED blinks rapidly (5Hz)
- Sends `$ERR,CONFIG_TIMEOUT*XX`
- Power cycle required to recover

### Parse Errors

Invalid commands receive an error response:
- `$ERR,PARSE_OFF*XX` - Invalid offset format
- `$ERR,PARSE_MAG*XX` - Invalid mag calibration format
- `$ERR,UNKNOWN_CMD*XX` - Unrecognized command

## LED Status

| Pattern | State |
|---------|-------|
| Off | Initializing |
| Solid on | Ready/streaming |
| Fast blink (5Hz) | Error - power cycle required |

## Configuration

Edit `src/config.h` to adjust:

- IMU update rate (default 100Hz)
- Serial output rate (default 100Hz)
- Configuration timeout (default 30s)
- Madgwick filter gain (beta)
- Sensor ranges

## Resource Usage (ATtiny3224)

- Flash: ~18KB of 32KB
- RAM: ~1.2KB of 3KB
- CPU: ~50% at 100Hz update rate

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
