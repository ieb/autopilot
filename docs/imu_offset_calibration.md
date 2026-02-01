# IMU Offset Calibration Guide

This guide explains how to configure the IMU mounting offset to compensate for off-center installation.

## Why Offset Calibration?

When the IMU sensor is mounted away from the boat's center of rotation (typically near the mast or keel), rotational motion creates spurious accelerations:

- **Centripetal acceleration**: When the boat turns, an off-center IMU experiences acceleration toward the center of rotation
- **Tangential acceleration**: When the turn rate changes, the IMU experiences acceleration perpendicular to the radius

These spurious accelerations can cause the Madgwick AHRS filter to produce incorrect heading estimates, especially during maneuvers like tacking or gybing.

## Measuring the Offset

The offset is measured in meters from the boat's center of rotation to the IMU mounting location.

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

**IMU mounted at the mast base (5m forward, on centerline):**
- X offset: 5.0 meters
- Y offset: 0.0 meters
- Z offset: 0.0 meters

## Configuring the Offset

### Using Serial Commands

Connect to the IMU MCU via serial (115200 baud) and send:

```
$OFF,3.0,0.5,0.0*XX
```

Where:
- First value = X offset (forward, meters)
- Second value = Y offset (starboard, meters)  
- Third value = Z offset (down, meters)
- `*XX` = checksum (optional, firmware accepts any value)

### Query Current Settings

```
$OFF,GET*XX
```

The MCU responds with:
```
$OFS,3.000,0.500,0.000*XX
```

### Save to EEPROM

To persist the offset across power cycles:

```
$OFF,SAVE*XX
```

### Using Python

```python
from src.sensors.imu_fusion import IMUFusion, IMUConfig

# Connect to IMU
config = IMUConfig(port="/dev/ttyUSB0")
imu = IMUFusion(config)
imu.start()

# Set offset (3m forward, 0.5m starboard)
imu.set_offset(3.0, 0.5, 0.0)

# Save to EEPROM
imu.save_offset()
```

## Effect of Offset Correction

### Without Correction

During a tack with 10°/s yaw rate and IMU 3m from center:
- Centripetal acceleration: ~0.09 m/s² (~1% of gravity)
- This causes the Madgwick filter to incorrectly estimate attitude
- Over time, heading drift accumulates

### With Correction

The firmware subtracts the calculated centripetal and tangential accelerations from the raw readings, providing clean acceleration data to the AHRS filter.

## Verification

After configuring the offset:

1. **Static test**: With the boat stationary, verify heading remains stable
2. **Dynamic test**: During maneuvers, verify heading tracks correctly
3. **Compare**: If you have a GPS compass, compare heading during turns

## Troubleshooting

### Heading drifts during turns
- Offset values may be incorrect
- Re-measure the distance from center of rotation

### Heading oscillates
- Offset may be too large (over-correction)
- Verify measurements are in meters, not centimeters

### No change after setting offset
- Verify the command was acknowledged (look for `$OFS` response)
- Run `$OFF,GET*XX` to confirm values were set
- Make sure to run `$OFF,SAVE*XX` before power cycling

## Default Values

If no offset calibration is stored in EEPROM, the firmware uses:
- X: 0.0 m
- Y: 0.0 m
- Z: 0.0 m

This means no correction is applied, maintaining backwards compatibility.

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
