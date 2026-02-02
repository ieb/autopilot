# IMU I2C Direct Connection Setup

This guide covers connecting the ICM-20948 IMU directly to the Raspberry Pi via I2C, eliminating the need for a separate MCU.

## Overview

The `imu_fusion_i2c.py` module provides a drop-in replacement for the serial/MCU-based `imu_fusion.py`. Instead of communicating with an external ATtiny3224 running the Madgwick filter, it:

1. Reads the ICM-20948 sensor directly via I2C
2. Runs the Madgwick AHRS filter in Python
3. Provides the same API as `IMUFusion`

## Hardware Requirements

- Raspberry Pi (any model with I2C)
- ICM-20948 9-DoF IMU breakout board
- 4 jumper wires

## Wiring

Connect the ICM-20948 to the Raspberry Pi GPIO header:

| ICM-20948 | Raspberry Pi | Pin |
|-----------|--------------|-----|
| VCC | 3.3V | 1 |
| GND | GND | 6 |
| SDA | SDA (GPIO 2) | 3 |
| SCL | SCL (GPIO 3) | 5 |

```
ICM-20948                Raspberry Pi
┌─────────┐             ┌─────────────────┐
│   VCC   │─────────────│ Pin 1 (3.3V)    │
│   GND   │─────────────│ Pin 6 (GND)     │
│   SDA   │─────────────│ Pin 3 (GPIO 2)  │
│   SCL   │─────────────│ Pin 5 (GPIO 3)  │
│   AD0   │─(NC or GND)─│                 │
│   INT   │─(optional)──│ Any GPIO        │
└─────────┘             └─────────────────┘
```

**I2C Address Selection:**
- AD0 = GND or floating: Address 0x68 (default)
- AD0 = VCC: Address 0x69

## Raspberry Pi Setup

### 1. Enable I2C

```bash
sudo raspi-config
```

Navigate to: Interface Options → I2C → Enable

Or add to `/boot/config.txt`:
```
dtparam=i2c_arm=on
```

Reboot after enabling.

### 2. Install Dependencies

```bash
# System packages
sudo apt-get install -y i2c-tools python3-smbus

# Python packages
pip install smbus2 numpy
```

Or with uv in the project:
```bash
uv add smbus2
```

### 3. Verify Connection

```bash
sudo i2cdetect -y 1
```

Expected output:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- 0c -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

- `68`: ICM-20948 main sensor
- `0c`: AK09916 magnetometer (via I2C bypass)

## Usage

### Basic Usage

```python
from src.sensors.imu_fusion_i2c import IMUFusionI2C, IMUConfigI2C

# Create configuration
config = IMUConfigI2C(
    i2c_bus=1,           # I2C bus number
    i2c_address=0x68,    # ICM-20948 address
    update_rate_hz=100,  # Target update rate
    madgwick_beta=0.1    # Filter gain
)

# Start IMU
imu = IMUFusionI2C(config)
if imu.start():
    # Read data
    data = imu.get_data()
    print(f"Heading: {data.heading:.1f}°")
    print(f"Pitch: {data.pitch:.1f}°")
    print(f"Roll: {data.roll:.1f}°")
    
    imu.stop()
```

### With Calibration

```python
from src.sensors.imu_fusion_i2c import (
    IMUFusionI2C, IMUConfigI2C, IMUCalibrationData
)

# Load or create calibration
calibration = IMUCalibrationData(
    mag_offset=(12.5, -3.2, 8.1),
    mag_scale=(1.02, 0.98, 1.01),
    position_offset=(3.0, 0.5, 0.0)  # IMU offset from center
)

config = IMUConfigI2C(
    calibration_file="/etc/autopilot/imu_cal.json"
)

imu = IMUFusionI2C(config, calibration)
imu.start()

# Save calibration for next time
imu.save_calibration()
```

### Interactive Magnetometer Calibration

```python
from src.sensors.imu_fusion_i2c import IMUCalibrationI2C, IMUConfigI2C

config = IMUConfigI2C()

# Rotate sensor slowly through all orientations
cal_data = IMUCalibrationI2C.run_calibration(config, duration=30.0)

if cal_data:
    print(f"Mag offset: {cal_data.mag_offset}")
    print(f"Mag scale: {cal_data.mag_scale}")
    
    # Save to file
    import json
    with open("/etc/autopilot/imu_cal.json", "w") as f:
        json.dump(cal_data.to_dict(), f, indent=2)
```

### Using Callbacks

```python
def on_imu_data(data):
    print(f"Heading: {data.heading:.1f}°, Rate: {data.yaw_rate:.2f}°/s")

imu = IMUFusionI2C()
imu.add_callback(on_imu_data)
imu.start()
```

## Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| `i2c_bus` | 1 | I2C bus number |
| `i2c_address` | 0x68 | ICM-20948 I2C address |
| `update_rate_hz` | 100 | Target update rate (Hz) |
| `max_age_ms` | 200 | Max age before data is stale |
| `madgwick_beta` | 0.1 | Filter gain (0.01-0.5) |
| `accel_range` | 4 | Accelerometer range (g) |
| `gyro_range` | 500 | Gyroscope range (deg/s) |
| `calibration_file` | None | Path to calibration JSON |

## Calibration File Format

```json
{
  "mag_offset": [12.5, -3.2, 8.1],
  "mag_scale": [1.02, 0.98, 1.01],
  "position_offset": [3.0, 0.5, 0.0]
}
```

## Comparison: I2C vs MCU

### When to Use I2C Direct

- Development and testing
- Short cable runs (< 1m)
- Cost-sensitive installations
- Rapid iteration on filter tuning

### When to Use MCU (Serial)

- Production installations
- Long cable runs (use RS-485)
- Electrically noisy environments
- Safety-critical applications
- Need for galvanic isolation

### Detailed Comparison

| Aspect | I2C Direct | MCU (Serial) |
|--------|------------|--------------|
| **Timing** | Subject to Linux jitter | Deterministic (MCU loop) |
| **Cable length** | < 1m typical | Up to 100m (RS-485) |
| **Isolation** | None | Galvanic isolation possible |
| **CPU usage** | Higher (filter on Pi) | Minimal (data parsing only) |
| **Cost** | Lower (no MCU) | Higher (MCU + board) |
| **Updates** | Instant (Python) | Requires firmware flash |
| **Filter accuracy** | Good (timing jitter) | Better (consistent rate) |

## Troubleshooting

### "ICM-20948 not found"

1. Check wiring connections
2. Verify I2C is enabled: `ls /dev/i2c-*`
3. Check address: `sudo i2cdetect -y 1`
4. Try alternate address (0x69) if AD0 is high

### "smbus2 not installed"

```bash
pip install smbus2
# or
uv add smbus2
```

### Poor heading accuracy

1. Calibrate magnetometer (rotate through all orientations)
2. Keep away from magnetic sources (motors, magnets, iron)
3. Increase `madgwick_beta` for faster convergence
4. Check for timing issues (high CPU load)

### Inconsistent update rate

The Pi is not a real-time system. For best results:
- Minimize other CPU load
- Use a Pi 4 or newer
- Consider the MCU solution for production

### I2C errors under load

The Pi's I2C can be unreliable with long cables or noise:
- Keep cables short (< 30cm ideal)
- Use shielded cable
- Add 2.2kΩ pull-ups if using long cables
- Consider I2C buffer/extender ICs for longer runs

## Performance Notes

- Update rate: 100Hz target, typically 95-105Hz actual
- Filter latency: ~10ms (one update cycle)
- CPU usage: ~5-10% on Pi 4 at 100Hz
- Memory: ~50MB (Python + NumPy overhead)

## API Compatibility

`IMUFusionI2C` is a drop-in replacement for `IMUFusion`:

```python
# Original (serial/MCU)
from src.sensors.imu_fusion import IMUFusion, IMUConfig

# Direct I2C
from src.sensors.imu_fusion_i2c import IMUFusion, IMUConfig  # Aliases

# Both have identical APIs:
# - start() / stop()
# - get_data() -> IMUData
# - add_callback()
# - save_calibration()
# - stats property
# - calibration attribute
```
