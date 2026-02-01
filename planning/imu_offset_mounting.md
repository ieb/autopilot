# IMU Off-Center Mounting Support

## Overview

Add configuration for IMU mounting offset from boat center, and implement acceleration corrections to compensate for centripetal and tangential accelerations caused by off-center mounting during rotational motion.

## Background

When an IMU is mounted away from the boat's center of rotation, rotational motion creates additional accelerations that contaminate the accelerometer readings:

**Centripetal acceleration** (toward center of rotation):
```
a_centripetal = omega^2 * r
```

**Tangential acceleration** (perpendicular to radius):
```
a_tangential = alpha * r  (where alpha = d(omega)/dt)
```

For a boat rotating in yaw, pitch, and roll, an IMU offset by (x, y, z) meters from the center experiences these spurious accelerations that must be subtracted from the raw readings.

## Coordinate System

Following the existing convention in `firmware/mcu/README.md`:
- X: Forward (positive = bow)
- Y: Starboard (positive = starboard)
- Z: Down (positive = down)

Offset configuration:
- `x_offset`: Forward/aft offset in meters (positive = forward of center)
- `y_offset`: Port/starboard offset in meters (positive = starboard of center)
- `z_offset`: Up/down offset in meters (positive = below center, rarely needed)

## Implementation

### 1. Firmware: Configuration Header

Modify `firmware/mcu/src/config.h`:

- Add default IMU offset values (0,0,0 for backwards compatibility)
- Add EEPROM address for offset storage (after magnetometer calibration data)

```cpp
// IMU mounting offset from center of rotation (meters)
// Positive X = forward, positive Y = starboard, positive Z = down
#define DEFAULT_IMU_OFFSET_X 0.0f
#define DEFAULT_IMU_OFFSET_Y 0.0f
#define DEFAULT_IMU_OFFSET_Z 0.0f

// EEPROM address for offset calibration (after mag cal)
#define EEPROM_OFFSET_ADDR (sizeof(MagCalibration))
```

### 2. Firmware: Main Implementation

Modify `firmware/mcu/src/main.cpp`:

**Add offset calibration struct**:
```cpp
struct OffsetCalibration {
  float x, y, z;  // meters
  uint16_t magic; // 0xBEEF if valid
};
```

**Track previous gyro readings** for computing angular acceleration (alpha):
```cpp
float prev_gx = 0, prev_gy = 0, prev_gz = 0;
uint32_t prev_gyro_time = 0;
```

**Add correction function**:
```cpp
void correctAccelForOffset(float& ax, float& ay, float& az,
                           float gx, float gy, float gz,
                           float alpha_x, float alpha_y, float alpha_z) {
  // Convert rates to rad/s
  float wx = gx * DEG_TO_RAD;
  float wy = gy * DEG_TO_RAD;
  float wz = gz * DEG_TO_RAD;
  float ax_rad = alpha_x * DEG_TO_RAD;
  float ay_rad = alpha_y * DEG_TO_RAD;
  float az_rad = alpha_z * DEG_TO_RAD;
  
  // Centripetal: a = omega x (omega x r)
  // Tangential: a = alpha x r
  // Combined correction to subtract:
  float dx = offsetCal.x, dy = offsetCal.y, dz = offsetCal.z;
  
  // Centripetal components (cross product expansion)
  float cent_x = wy*(wy*dx - wx*dy) + wz*(wz*dx - wx*dz);
  float cent_y = wx*(wx*dy - wy*dx) + wz*(wz*dy - wy*dz);
  float cent_z = wx*(wx*dz - wz*dx) + wy*(wy*dz - wz*dy);
  
  // Tangential components (alpha x r)
  float tang_x = ay_rad*dz - az_rad*dy;
  float tang_y = az_rad*dx - ax_rad*dz;
  float tang_z = ax_rad*dy - ay_rad*dx;
  
  // Subtract corrections
  ax -= (cent_x + tang_x);
  ay -= (cent_y + tang_y);
  az -= (cent_z + tang_z);
}
```

**Add serial commands**:
- `$OFF,x,y,z*XX` - Set offset values (meters)
- `$OFF,GET*XX` - Query current offsets
- `$OFF,SAVE*XX` - Save to EEPROM

### 3. Firmware: EEPROM Storage

Add functions to load/save offset calibration alongside magnetometer calibration:
- `loadOffsetCalibration()` - Called at startup
- `saveOffsetCalibration()` - Called on `$OFF,SAVE` command

### 4. Update Serial Protocol Documentation

Update `firmware/mcu/README.md`:

Add documentation for new commands:
```
### Offset Commands (Pi -> MCU)

$OFF,x,y,z*XX   - Set IMU mounting offset (meters from center of rotation)
                  x = forward (positive = bow)
                  y = starboard (positive = right)
                  z = down (positive = below waterline)
$OFF,GET*XX     - Query current offset values
$OFF,SAVE*XX    - Save offsets to EEPROM

### Offset Response (MCU -> Pi)

$OFS,x,y,z*XX   - Current offset values in response to GET
```

### 5. Python Simulator (Optional Enhancement)

Update `src/simulation/hw_simulators/imu_sim.py` to optionally simulate off-center effects:

- Add offset configuration to `IMUSimulatorConfig`
- When offset is non-zero, add rotational accelerations to simulated readings

This enables testing the firmware correction logic in simulation.

### 6. IMUConfig Update

Update `src/sensors/imu_fusion.py`:

Add method to send offset configuration to the MCU:
```python
def set_offset(self, x: float, y: float, z: float):
    """Set IMU mounting offset (meters from center of rotation)."""
    payload = f"OFF,{x:.3f},{y:.3f},{z:.3f}"
    checksum = self._compute_checksum(payload)
    cmd = f"${payload}*{checksum:02X}\r\n"
    self._conn.write(cmd.encode('ascii'))
```

## Practical Example

For a Pogo 1250 with IMU mounted 3m forward and 0.5m to starboard:
- x_offset = 3.0
- y_offset = 0.5
- z_offset = 0.0

At a yaw rate of 10 deg/s (typical for a tack):
- omega = 0.175 rad/s
- Centripetal acceleration at 3m forward = 0.175^2 * 3 = 0.092 m/s^2

This is about 1% of gravity - small but measurable, and cumulative errors in the Madgwick filter can cause heading drift if not corrected.

## Testing

- Add unit test for acceleration correction math
- Test that default offsets (0,0,0) produce identical behavior to current firmware
- Test with simulator using known offsets and rotation rates

## Implementation Status

All tasks completed:
- [x] Add offset configuration defines and EEPROM address to config.h
- [x] Add OffsetCalibration struct and global variables in main.cpp
- [x] Implement correctAccelForOffset() function with centripetal/tangential math
- [x] Apply acceleration correction in main loop before outputting data
- [x] Add serial command parsing for $OFF,x,y,z and $OFF,SAVE
- [x] Add loadOffsetCalibration() and saveOffsetCalibration() functions
- [x] Update firmware/mcu/README.md with new offset commands documentation
- [x] Update imu_sim.py to simulate off-center effects
- [x] Add set_offset() method to IMUFusion class
