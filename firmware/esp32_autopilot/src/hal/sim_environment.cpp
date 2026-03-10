#ifdef HAL_SIM

#include "sim_environment.h"
#include "NMEA2000_sim.h"
#include "Wire.h"
#include "config.h"
#include "polar.h"
#include <math.h>
#include <stdio.h>

// From arduino_stubs.cpp
extern void hal_sim_set_rudder_adc_mv(uint32_t mv);
extern int hal_sim_get_pwm_duty(int channel);
extern int hal_sim_get_pin(int pin);

extern "C" uint32_t millis();

SimEnvironment::SimEnvironment(tNMEA2000_sim& nmea, TwoWire& wire)
    : nmea(nmea), wire(wire) {}

void SimEnvironment::set_wind(float tws_kts, float twd_deg) {
    tws = tws_kts;
    twd = twd_deg;
}

void SimEnvironment::set_external_active(bool active) {
    if (active != external_active) {
        external_active = active;
        printf("\n[SIM] %s mode\n", active ? "EXTERNAL" : "INTERNAL");
    }
}

static float wrap_360(float deg) {
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg;
}

static float wrap_180(float deg) {
    deg = fmodf(deg, 360.0f);
    if (deg > 180.0f) deg -= 360.0f;
    if (deg < -180.0f) deg += 360.0f;
    return deg;
}

void SimEnvironment::update(float dt) {
    // When external simulator is connected, it handles all sensor injection
    // (CAN frames and IMU registers come over the socket). We still read
    // the actuator ADC so the firmware's position feedback loop works.
    if (external_active) {
        // Still read actuator PWM and update rudder ADC so the
        // actuator position P-controller works correctly
        read_actuator_pwm();
        update_rudder_dynamics(dt);
        update_rudder_adc();
        return;
    }

    // 1. Read actuator PWM output → determine motor command
    read_actuator_pwm();

    // 2. Update rudder position (rate-limited)
    update_rudder_dynamics(dt);

    // 3. Update heading from rudder
    update_heading_dynamics(dt);

    // 4. Compute wind triangle
    compute_wind();

    // 5. Push BNO055 registers
    update_imu_registers();

    // 6. Periodically inject N2K messages
    uint32_t now = millis();
    if (now - last_n2k_inject_ms >= N2K_INJECT_INTERVAL_MS) {
        last_n2k_inject_ms = now;
        inject_n2k_messages();
    }

    // 7. Update simulated ADC reading
    update_rudder_adc();
}

void SimEnvironment::read_actuator_pwm() {
    // Check clutch pin (PIN_CLUTCH = 18)
    clutch_engaged = hal_sim_get_pin(PIN_CLUTCH) != 0;

    if (!clutch_engaged) {
        motor_command = 0.0f;
        return;
    }

    // Read PWM duty from LEDC channels
    // LEDC_CH_RPWM (0) = starboard drive, LEDC_CH_LPWM (1) = port drive
    int rpwm = hal_sim_get_pwm_duty(LEDC_CH_RPWM);
    int lpwm = hal_sim_get_pwm_duty(LEDC_CH_LPWM);

    // Convert to normalized motor command
    if (rpwm > 0 && lpwm == 0) {
        motor_command = rpwm / 255.0f;   // positive = starboard
    } else if (lpwm > 0 && rpwm == 0) {
        motor_command = -lpwm / 255.0f;  // negative = port
    } else {
        motor_command = 0.0f;
    }
}

void SimEnvironment::update_rudder_dynamics(float dt) {
    if (!clutch_engaged) return;

    // The real actuator uses a position P-controller which drives the motor
    // toward rudder_target. In our sim, the motor_command from PWM tells us
    // which direction the motor is driving. We simulate the physical rudder
    // moving at the actuator rate limit in that direction.
    if (fabsf(motor_command) > 0.01f) {
        float direction = (motor_command > 0) ? 1.0f : -1.0f;
        float max_move = ACTUATOR_RATE * dt;
        rudder_pos += direction * max_move;

        // Clamp to position limits
        float limit = POSITION_LIMIT_DEG / 25.0f;
        if (rudder_pos > limit) rudder_pos = limit;
        if (rudder_pos < -limit) rudder_pos = -limit;
    }
}

void SimEnvironment::update_heading_dynamics(float dt) {
    // Turn rate: ~3 deg/s per 10 deg rudder
    float rudder_deg = rudder_pos * 25.0f;
    yaw_rate = rudder_deg * 0.3f;
    heading = wrap_360(heading + yaw_rate * dt);
}

void SimEnvironment::compute_wind() {
    // TWA: signed, positive = starboard
    twa = wrap_180(twd - heading);

    // STW from polar
    float abs_twa = fabsf(twa);
    if (abs_twa < 1.0f) abs_twa = 1.0f;
    stw = polar_get_target_speed(abs_twa, tws) * 0.95f;
    if (stw < 0.5f) stw = 0.5f;

    // AWA/AWS from wind triangle
    float twa_rad = twa * (M_PI / 180.0f);
    float wx = tws * cosf(twa_rad) + stw;
    float wy = tws * sinf(twa_rad);
    aws = sqrtf(wx * wx + wy * wy);
    awa = atan2f(wy, wx) * (180.0f / M_PI);

    // SOG/COG
    sog = stw * 0.98f;
    cog = heading;
}

void SimEnvironment::update_imu_registers() {
    // BNO055 Euler angles: 1 degree = 16 LSB
    int16_t heading_raw = (int16_t)(heading * 16.0f);
    wire.set_register_16(0x1A, heading_raw);  // EULER_H

    // Roll: simplified heel model
    float target_roll = 0.0f;
    if (stw > 0.5f) {
        float awa_rad = awa * (M_PI / 180.0f);
        target_roll = sinf(awa_rad) * stw * 0.15f;
    }
    roll += (target_roll - roll) * 0.1f;
    int16_t roll_raw = (int16_t)(roll * 16.0f);
    wire.set_register_16(0x1C, roll_raw);     // EULER_R

    // Pitch: wave oscillation
    pitch_phase++;
    pitch = 2.0f * sinf(pitch_phase * 0.05f);
    int16_t pitch_raw = (int16_t)(pitch * 16.0f);
    wire.set_register_16(0x1E, pitch_raw);    // EULER_P

    // Gyro: 1 dps = 16 LSB
    int16_t gyro_z_raw = (int16_t)(yaw_rate * 16.0f);
    wire.set_register_16(0x18, gyro_z_raw);   // GYRO_Z
    wire.set_register_16(0x14, 0);            // GYRO_X
    wire.set_register_16(0x16, 0);            // GYRO_Y
}

void SimEnvironment::inject_n2k_messages() {
    // Wind (PGN 130306) — apparent
    {
        tN2kMsg msg;
        // AWA: convert signed degrees to 0-360 radians for N2K
        float awa_360 = awa;
        if (awa_360 < 0) awa_360 += 360.0f;
        float aws_ms = aws * 0.514444f;
        SetN2kWindSpeed(msg, 0, aws_ms, DegToRad(awa_360), N2kWind_Apparent);
        msg.Source = 10;  // simulated instrument source
        msg.Priority = 2;
        nmea.inject_message(msg);
    }

    // True wind is NOT injected — real N2K buses only carry apparent wind.
    // The firmware computes TWA/TWS from apparent wind + STW in n2k.cpp.

    // STW (PGN 128259)
    {
        tN2kMsg msg;
        double stw_ms = stw * 0.514444;
        SetN2kBoatSpeed(msg, 0, stw_ms);
        msg.Source = 10;
        msg.Priority = 2;
        nmea.inject_message(msg);
    }

    // COG/SOG (PGN 129026)
    {
        tN2kMsg msg;
        SetN2kCOGSOGRapid(msg, 0, N2khr_magnetic, DegToRad(cog), sog * 0.514444);
        msg.Source = 10;
        msg.Priority = 2;
        nmea.inject_message(msg);
    }

    // Heading (PGN 127250)
    {
        tN2kMsg msg;
        SetN2kMagneticHeading(msg, 0, DegToRad(heading));
        msg.Source = 10;
        msg.Priority = 2;
        nmea.inject_message(msg);
    }
}

void SimEnvironment::update_rudder_adc() {
    // Convert rudder position to millivolts matching calibration defaults
    uint32_t mv;
    if (rudder_pos < 0) {
        float frac = -rudder_pos;
        mv = (uint32_t)(ADC_RUDDER_CENTER_MV - frac * (ADC_RUDDER_CENTER_MV - ADC_RUDDER_PORT_MV));
    } else {
        float frac = rudder_pos;
        mv = (uint32_t)(ADC_RUDDER_CENTER_MV + frac * (ADC_RUDDER_STBD_MV - ADC_RUDDER_CENTER_MV));
    }
    hal_sim_set_rudder_adc_mv(mv);
}

#endif // HAL_SIM
