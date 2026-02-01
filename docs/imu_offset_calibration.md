# IMU Configuration Guide

This guide explains how to configure the IMU, including mounting offset and magnetometer calibration.

## Simplex Protocol Overview

The IMU firmware uses a three-phase simplex protocol:

1. **Startup**: IMU powers on and sends `$VER` and `$RDY` prompts
2. **Configuration**: Pi sends configuration (offset, mag cal) to IMU
3. **Streaming**: IMU streams data at 100Hz (simplex, no command processing)

This design enables:
- **Hot-swappable IMU**: Configuration is stored on Pi, not in IMU EEPROM
- **Simplex RS-485**: Single twisted pair sufficient after configuration
- **Centralized management**: All calibration data on Pi

To reconfigure, power cycle the IMU.

## Configuration Storage

All IMU configuration is stored in a JSON file on the Pi, typically at `/etc/autopilot/imu_cal.json`:

```json
{
  "mag_offset": [12.5, -3.2, 8.1],
  "mag_scale": [1.02, 0.98, 1.01],
  "position_offset": [3.0, 0.5, 0.0]
}
```

## Mounting Offset Calibration

### Why Offset Calibration?

When the IMU sensor is mounted away from the boat's center of rotation, rotational motion creates spurious accelerations:

- **Centripetal acceleration**: When the boat turns, an off-center IMU experiences acceleration toward the center of rotation
- **Tangential acceleration**: When the turn rate changes, the IMU experiences acceleration perpendicular to the radius

These spurious accelerations can cause the Madgwick AHRS filter to produce incorrect heading estimates, especially during maneuvers.

### Coordinate System

| Axis | Direction | Positive Value |
|------|-----------|----------------|
| X | Forward/Aft | Toward bow |
| Y | Port/Starboard | Toward starboard |
| Z | Up/Down | Below waterline |

### Finding the Center of Rotation

The center of rotation is approximately:
- **Fore/aft**: Near the center of the keel (longitudinally)
- **Athwartships**: On the centerline
- **Vertically**: Near the waterline

For most sailboats, this is roughly at the main companionway hatch, on centerline.

### Example Measurements

**IMU mounted in the nav station (3m forward, 0.5m to starboard):**
- X offset: 3.0 meters
- Y offset: 0.5 meters
- Z offset: 0.0 meters

### Configuring via Python

```python
from src.sensors.imu_fusion import IMUFusion, IMUConfig, IMUCalibrationData

# Create calibration data with offset
calibration = IMUCalibrationData(
    position_offset=(3.0, 0.5, 0.0),  # 3m forward, 0.5m starboard
    mag_offset=(12.5, -3.2, 8.1),     # From previous calibration
    mag_scale=(1.02, 0.98, 1.01)
)

# Create config with calibration file path
config = IMUConfig(
    port="/dev/ttyUSB0",
    calibration_file="/etc/autopilot/imu_cal.json"
)

# Start IMU (sends configuration automatically)
imu = IMUFusion(config, calibration)
imu.start()

# Save calibration to file
imu.save_calibration()
```

## Magnetometer Calibration

The magnetometer requires calibration to compensate for hard-iron and soft-iron distortions from the boat's metal components.

### Option 1: Interactive Calibration

Perform calibration during the configuration phase:

```python
from src.sensors.imu_fusion import IMUCalibration

# Run interactive calibration (30 seconds)
# Slowly rotate the sensor through all orientations
cal_data = IMUCalibration.run_calibration(
    port="/dev/ttyUSB0",
    duration=30.0
)

if cal_data:
    print(f"Mag offset: {cal_data.mag_offset}")
    print(f"Mag scale: {cal_data.mag_scale}")
    
    # Save to file for future use
    import json
    with open("/etc/autopilot/imu_cal.json", "w") as f:
        json.dump(cal_data.to_dict(), f, indent=2)
```

### Option 2: Send Known Values

If calibration values are already known:

```python
calibration = IMUCalibrationData(
    mag_offset=(12.5, -3.2, 8.1),
    mag_scale=(1.02, 0.98, 1.01),
    position_offset=(3.0, 0.5, 0.0)
)
```

### Manual Calibration Commands

During the configuration phase, you can send commands manually:

```
# Set mag calibration
$MAG,12.5,-3.2,8.1,1.02,0.98,1.01*XX

# Start interactive calibration
$CAL,START*XX

# Stop calibration (returns calculated values)
$CAL,STOP*XX
```

## Typical Startup Sequence

1. Power on IMU
2. IMU sends: `$VER,abc1234*XX` and `$RDY*XX`
3. Pi sends offset: `$OFF,3.0,0.5,0.0*XX`
4. IMU responds: `$OFS,3.000,0.500,0.000*XX`
5. Pi sends mag cal: `$MAG,12.5,-3.2,8.1,1.02,0.98,1.01*XX`
6. IMU responds: `$MGC,12.50,-3.20,8.10,1.020,0.980,1.010*XX`
7. Pi sends: `$START*XX`
8. IMU responds: `$ACK*XX`
9. IMU streams: `$IMU,...*XX` at 100Hz

## Effect of Offset Correction

### Without Correction

During a tack with 10°/s yaw rate and IMU 3m from center:
- Centripetal acceleration: ~0.09 m/s² (~1% of gravity)
- This causes the Madgwick filter to incorrectly estimate attitude
- Over time, heading drift accumulates

### With Correction

The firmware subtracts the calculated centripetal and tangential accelerations from the raw readings, providing clean acceleration data to the AHRS filter.

## Verification

After configuring:

1. **Static test**: With the boat stationary, verify heading remains stable
2. **Dynamic test**: During maneuvers, verify heading tracks correctly
3. **Compare**: If you have a GPS compass, compare heading during turns

## Troubleshooting

### IMU times out during startup
- Check serial connection
- Verify Pi sends configuration within 30 seconds
- Check for `$ERR,CONFIG_TIMEOUT*XX` message

### Heading drifts during turns
- Offset values may be incorrect
- Re-measure the distance from center of rotation

### Heading oscillates
- Offset may be too large (over-correction)
- Verify measurements are in meters, not centimeters

### Magnetometer heading incorrect
- Run interactive calibration
- Ensure you rotate through all orientations during calibration

## Technical Details

The correction uses the full 3D rotational dynamics:

**Centripetal acceleration**:
```
a_c = ω × (ω × r)
```

**Tangential acceleration**:
```
a_t = α × r
```

Where:
- ω = angular velocity vector (from gyroscope)
- α = angular acceleration (derivative of ω)
- r = offset vector (x, y, z)

The firmware calculates angular acceleration by differentiating the gyroscope readings between samples.
