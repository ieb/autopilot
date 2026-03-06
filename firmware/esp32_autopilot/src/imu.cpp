#ifndef NATIVE_BUILD

#include "imu.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_BNO055.h>

static Adafruit_BNO055 bno(55, 0x28, &Wire);
static float prev_roll = 0.0f;

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

    state.heading = orient.orientation.x;  // 0-360 clockwise from North
    state.roll    = orient.orientation.z;   // positive = starboard heel
    state.pitch   = orient.orientation.y;   // positive = bow up

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

#endif // NATIVE_BUILD
