#if !defined(NATIVE_BUILD) || defined(HAL_SIM)

#include "imu.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <math.h>

static Adafruit_BNO055 bno(55, 0x28, &Wire);
static float prev_roll = 0.0f;

// Wrap angle difference to [-180, 180]
static float wrap_180(float deg) {
    deg = fmodf(deg, 360.0f);
    if (deg > 180.0f) deg -= 360.0f;
    if (deg < -180.0f) deg += 360.0f;
    return deg;
}

// Wrap angle to [0, 360)
static float wrap_360(float deg) {
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg;
}

bool imu_init() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);

    if (!bno.begin(OPERATION_MODE_NDOF)) {
        return false;
    }

    bno.setExtCrystalUse(true);
    return true;
}

void imu_read(AppState& state) {
    // Fused Euler angles from BNO055 onboard AHRS
    sensors_event_t orient;
    bno.getEvent(&orient, Adafruit_BNO055::VECTOR_EULER);

    float imu_heading = orient.orientation.x;  // 0-360 clockwise from North
    state.roll    = orient.orientation.z;       // positive = starboard heel
    state.pitch   = orient.orientation.y;       // positive = bow up
    state.imu_raw_heading = imu_heading;        // store raw for N2K fusion snapshot

    // Fuse N2K heading (primary) with IMU delta (interpolation between updates)
    if (state.n2k_heading_valid) {
        float imu_delta = wrap_180(imu_heading - state.imu_heading_at_n2k);
        state.heading = wrap_360(state.n2k_heading + imu_delta);
    } else {
        // No N2K heading yet — fall back to raw IMU heading
        state.heading = imu_heading;
    }

    // Gyro rates (deg/s)
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    state.yaw_rate = gyro.z();  // deg/s, positive = turning starboard

    // Compute roll rate from successive readings
    float dt = IMU_INTERVAL_MS / 1000.0f;
    if (dt > 0.0f) {
        state.roll_rate = (state.roll - prev_roll) / dt;
    }
    prev_roll = state.roll;

    state.imu_last_ms = millis();
}

#endif // !NATIVE_BUILD || HAL_SIM
