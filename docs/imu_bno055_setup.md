# BNO055 IMU Setup Guide

This guide covers connecting the BNO055 Absolute Orientation IMU directly to the Raspberry Pi via I2C.

## Overview

The BNO055 is a 9-DOF IMU with an integrated ARM Cortex-M0 processor that runs sensor fusion internally. This means:

- **No software fusion needed** - Euler angles output directly at 100Hz
- **Consistent timing** - Fusion runs on sensor, not affected by Linux scheduling
- **Built-in calibration** - Automatic calibration with status reporting
- **Lower CPU usage** - Pi only reads fused data, no Madgwick filter

## Hardware Requirements

- Raspberry Pi (any model with I2C)
- BNO055 breakout board (Adafruit, SparkFun, or generic)
- 4 jumper wires

## Wiring

| BNO055 | Raspberry Pi | Pin |
|--------|--------------|-----|
| VIN | 3.3V (or 5V*) | 1 |
| GND | GND | 6 |
| SDA | SDA (GPIO 2) | 3 |
| SCL | SCL (GPIO 3) | 5 |

```
BNO055                   Raspberry Pi
┌─────────┐             ┌─────────────────┐
│   VIN   │─────────────│ Pin 1 (3.3V)    │
│   GND   │─────────────│ Pin 6 (GND)     │
│   SDA   │─────────────│ Pin 3 (GPIO 2)  │
│   SCL   │─────────────│ Pin 5 (GPIO 3)  │
│  COM3   │─(NC or GND)─│                 │
│   RST   │─(optional)──│ Any GPIO        │
└─────────┘             └─────────────────┘
```

*Some breakouts (Adafruit) have onboard regulators and can use 5V on VIN.

**I2C Address Selection:**
- COM3 = GND or floating: Address 0x28 (default)
- COM3 = VCC: Address 0x29

## Raspberry Pi Setup

### 1. Enable I2C

```bash
sudo raspi-config
```

Navigate to: Interface Options → I2C → Enable

### 2. Install Dependencies

```bash
# With uv (recommended)
uv add smbus2

# Or with pip
pip install smbus2 numpy
```

### 3. Verify Connection

```bash
sudo i2cdetect -y 1
```

Expected output:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

## Usage

### Basic Usage

```python
from src.sensors.imu_fusion_bno055 import IMUFusionBNO055, IMUConfigBNO055

config = IMUConfigBNO055(
    i2c_bus=1,
    i2c_address=0x28,
    update_rate_hz=100
)

imu = IMUFusionBNO055(config)
if imu.start():
    data = imu.get_data()
    print(f"Heading: {data.heading:.1f}°")
    print(f"Pitch: {data.pitch:.1f}°")
    print(f"Roll: {data.roll:.1f}°")
    print(f"Calibration: sys={data.cal_sys}, mag={data.cal_mag}")
    
    imu.stop()
```

### Checking Calibration Status

The BNO055 calibrates automatically. Each sensor reports 0-3 (3 = fully calibrated):

```python
imu = IMUFusionBNO055()
imu.start()

# Check calibration
status = imu.get_calibration_status()
print(f"System: {status['sys']}/3")
print(f"Gyro: {status['gyro']}/3")
print(f"Accel: {status['accel']}/3")
print(f"Mag: {status['mag']}/3")
print(f"Fully calibrated: {status['fully_calibrated']}")

# Check via IMUData
data = imu.get_data()
if data.is_calibrated:
    print("Sensor is fully calibrated!")
```

### Saving and Restoring Calibration

The BNO055 loses calibration on power cycle. Save and restore it:

```python
from src.sensors.imu_fusion_bno055 import IMUFusionBNO055, IMUConfigBNO055

config = IMUConfigBNO055(
    calibration_file="/etc/autopilot/bno055_cal.json"
)

imu = IMUFusionBNO055(config)
imu.start()

# After calibration is complete (all sensors at 3)
if imu.get_calibration_status()['fully_calibrated']:
    imu.save_calibration()  # Saves to calibration_file
```

On next startup, calibration is automatically restored from the file.

### Interactive Calibration

```python
from src.sensors.imu_fusion_bno055 import IMUCalibrationBNO055, IMUConfigBNO055

config = IMUConfigBNO055()
imu = IMUFusionBNO055(config)
imu.start()

# Monitor calibration progress
def on_status(status):
    print(f"Cal: sys={status['sys']}, gyro={status['gyro']}, "
          f"accel={status['accel']}, mag={status['mag']}")

# Wait up to 60 seconds for full calibration
if IMUCalibrationBNO055.wait_for_calibration(imu, timeout=60, callback=on_status):
    print("Calibration complete!")
    imu.save_calibration()
else:
    print("Calibration timed out")

imu.stop()
```

### Calibration Procedure

During calibration:

1. **Gyroscope** (fastest): Keep sensor still for 2-3 seconds
2. **Accelerometer**: Place sensor in 6 orientations (each axis pointing up and down), hold still
3. **Magnetometer**: Wave sensor in figure-8 pattern through all orientations
4. **System**: Automatically calibrated when other sensors are done

## Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| `i2c_bus` | 1 | I2C bus number |
| `i2c_address` | 0x28 | BNO055 I2C address |
| `update_rate_hz` | 100 | Target update rate (Hz) |
| `max_age_ms` | 200 | Max age before data is stale |
| `operation_mode` | NDOF | Fusion mode (9-DOF) |
| `use_linear_accel` | True | Use gravity-removed acceleration |
| `calibration_file` | None | Path to calibration JSON |

## Operating Modes

The BNO055 supports multiple operating modes:

| Mode | Description | Use Case |
|------|-------------|----------|
| `OPR_MODE_NDOF` | 9-DOF fusion (default) | Full orientation with compass |
| `OPR_MODE_IMU` | 6-DOF fusion | No magnetometer (magnetic interference) |
| `OPR_MODE_COMPASS` | Compass mode | Heading only |
| `OPR_MODE_AMG` | Raw sensors | Custom fusion |

## BNO055 vs ICM-20948 Comparison

| Aspect | BNO055 | ICM-20948 |
|--------|--------|-----------|
| **Fusion** | On-sensor (M0 processor) | Software (Madgwick on Pi) |
| **CPU usage** | Very low | Higher (filter running) |
| **Timing** | Deterministic | Subject to Linux jitter |
| **Calibration** | Automatic with status | Manual magnetometer cal |
| **Cost** | ~$35 | ~$15 |
| **Accuracy** | Good | Good |
| **Heading drift** | Low (internal correction) | Depends on filter tuning |

**Choose BNO055 when:**
- You want simplest integration
- CPU usage matters
- You prefer automatic calibration
- Budget allows

**Choose ICM-20948 when:**
- Lower cost is priority
- You want custom fusion algorithms
- You need raw sensor access for debugging

## Linear Acceleration

By default, `use_linear_accel=True` uses the BNO055's gravity-compensated acceleration output. This is useful for detecting actual motion without gravity bias.

```python
# With linear acceleration (gravity removed)
config = IMUConfigBNO055(use_linear_accel=True)

# With raw acceleration (includes gravity)
config = IMUConfigBNO055(use_linear_accel=False)
```

## Off-Center Mounting

Like the ICM-20948 version, the BNO055 module supports off-center mounting correction:

```python
from src.sensors.imu_fusion_bno055 import IMUFusionBNO055, IMUCalibrationData

cal = IMUCalibrationData(
    position_offset=(3.0, 0.5, 0.0)  # 3m forward, 0.5m starboard
)

imu = IMUFusionBNO055(calibration=cal)
```

## Troubleshooting

### "BNO055 not found"

1. Check wiring connections
2. Verify I2C is enabled: `ls /dev/i2c-*`
3. Check address: `sudo i2cdetect -y 1`
4. Try alternate address (0x29) if COM3 is high

### Calibration Won't Complete

- **Gyro stuck**: Keep sensor completely still
- **Accel stuck**: Try more orientations, hold still in each
- **Mag stuck**: Wave in larger figure-8, away from metal/magnets

### Poor Heading Accuracy

1. Ensure magnetometer is calibrated (cal_mag = 3)
2. Mount away from motors, speakers, iron/steel
3. Avoid dynamic magnetic fields during operation

### I2C Errors

- BNO055 is sensitive to I2C bus speed
- Try reducing I2C speed if unstable:
  ```bash
  # /boot/config.txt
  dtparam=i2c_arm_baudrate=50000
  ```

## API Compatibility

`IMUFusionBNO055` is a drop-in replacement for `IMUFusion`:

```python
# Original (serial/MCU)
from src.sensors.imu_fusion import IMUFusion

# BNO055 direct I2C
from src.sensors.imu_fusion_bno055 import IMUFusion  # Alias

# Both have identical APIs:
# - start() / stop()
# - get_data() -> IMUData
# - add_callback()
# - save_calibration()
# - stats property
```

The BNO055 version adds:
- `get_calibration_status()` - Detailed calibration info
- `IMUData.cal_sys/gyro/accel/mag` - Per-sensor calibration status
- `IMUData.is_calibrated` - Quick check for full calibration
