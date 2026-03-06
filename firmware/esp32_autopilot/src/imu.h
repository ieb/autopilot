#ifndef IMU_H
#define IMU_H

#include "app_state.h"

// Initialize BNO055 on I2C. Returns true if sensor found.
bool imu_init();

// Read fused orientation + gyro rates. Updates state in-place.
void imu_read(AppState& state);

#endif // IMU_H
