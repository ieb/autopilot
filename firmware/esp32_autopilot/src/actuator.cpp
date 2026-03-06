#ifndef NATIVE_BUILD

#include "actuator.h"
#include "config.h"
#include <Arduino.h>
#include <Preferences.h>

// Calibration data
static uint16_t adc_center = ADC_RUDDER_CENTER;
static uint16_t adc_port_limit = ADC_RUDDER_PORT_LIMIT;
static uint16_t adc_stbd_limit = ADC_RUDDER_STBD_LIMIT;

static float last_angle = 0.0f;
static uint16_t stall_timer = 0;

static Preferences prefs;

static float adc_to_angle(uint16_t adc) {
    float angle;
    if (adc < adc_center) {
        float range = (float)(adc_center - adc_port_limit);
        angle = -((float)(adc_center - adc) / range);
    } else {
        float range = (float)(adc_stbd_limit - adc_center);
        angle = (float)(adc - adc_center) / range;
    }
    return constrain(angle, -1.0f, 1.0f);
}

void actuator_init() {
    // Configure PWM via LEDC (ESP32 Arduino 2.x API)
    ledcSetup(LEDC_CH_RPWM, PWM_FREQUENCY, LEDC_RESOLUTION);
    ledcAttachPin(PIN_PWM_RPWM, LEDC_CH_RPWM);
    ledcSetup(LEDC_CH_LPWM, PWM_FREQUENCY, LEDC_RESOLUTION);
    ledcAttachPin(PIN_PWM_LPWM, LEDC_CH_LPWM);

    // GPIO
    pinMode(PIN_BRIDGE_EN, OUTPUT);
    pinMode(PIN_CLUTCH, OUTPUT);
    pinMode(PIN_LED, OUTPUT);

    // Start with everything off
    ledcWrite(LEDC_CH_RPWM, 0);
    ledcWrite(LEDC_CH_LPWM, 0);
    digitalWrite(PIN_BRIDGE_EN, LOW);
    digitalWrite(PIN_CLUTCH, LOW);

    // Load calibration from NVS
    prefs.begin(NVS_NAMESPACE, true);  // read-only
    adc_center = prefs.getUShort("adc_center", ADC_RUDDER_CENTER);
    adc_port_limit = prefs.getUShort("adc_port", ADC_RUDDER_PORT_LIMIT);
    adc_stbd_limit = prefs.getUShort("adc_stbd", ADC_RUDDER_STBD_LIMIT);
    prefs.end();
}

void actuator_read_sensors(AppState& state) {
    uint16_t adc_rudder = analogRead(PIN_ADC_RUDDER);
    float new_angle = adc_to_angle(adc_rudder);

    float dt = ACTUATOR_INTERVAL_MS / 1000.0f;
    state.rudder_velocity = (new_angle - last_angle) / dt;
    last_angle = state.rudder_actual;
    state.rudder_actual = new_angle;

    // Current sense via BTS7960 IS pin (if wired)
    // For now use ADC — can be expanded to INA219 I2C
    // state.motor_current = analogRead(PIN_ADC_CURRENT) * BTS7960_CURRENT_SCALE;
    state.motor_current = 0.0f;  // placeholder until wired
    state.supply_voltage = 0.0f;
}

static void enforce_limits(float& pwm_output, AppState& state) {
    // Position limits
    if (state.rudder_actual >= POSITION_LIMIT_NORM && pwm_output > 0) {
        pwm_output = 0;
        state.fault_code = FAULT_POSITION_LIMIT;
    }
    if (state.rudder_actual <= -POSITION_LIMIT_NORM && pwm_output < 0) {
        pwm_output = 0;
        state.fault_code = FAULT_POSITION_LIMIT;
    }

    // Current limiting
    if (state.motor_current > CURRENT_LIMIT_HARD) {
        pwm_output = 0;
        state.fault_code = FAULT_OVERCURRENT;
    } else if (state.motor_current > CURRENT_LIMIT_SOFT) {
        pwm_output *= 0.5f;
    }

    // Stall detection
    if (state.motor_current > STALL_CURRENT_THRESHOLD &&
        fabsf(state.rudder_velocity) < STALL_VELOCITY_THRESHOLD) {
        stall_timer += ACTUATOR_INTERVAL_MS;
        if (stall_timer > STALL_TIMEOUT_MS) {
            pwm_output = 0;
            state.fault_code = FAULT_STALL;
        }
    } else {
        stall_timer = 0;
    }

    // Clear position limit fault if moved away
    if (state.fault_code == FAULT_POSITION_LIMIT) {
        if (fabsf(state.rudder_actual) < POSITION_LIMIT_NORM - 0.05f) {
            state.fault_code = FAULT_NONE;
        }
    }
}

static void update_clutch(AppState& state) {
    if (state.clutch_requested && !state.clutch_engaged) {
        if (state.clutch_engage_start == 0) {
            state.clutch_engage_start = millis();
            digitalWrite(PIN_CLUTCH, HIGH);
            digitalWrite(PIN_BRIDGE_EN, HIGH);
        }
        if (millis() - state.clutch_engage_start >= CLUTCH_ENGAGE_RAMP_MS) {
            state.clutch_engaged = true;
            state.fault_code = FAULT_NONE;
        }
    } else if (!state.clutch_requested && state.clutch_engaged) {
        state.clutch_engaged = false;
        state.clutch_engage_start = 0;
        digitalWrite(PIN_CLUTCH, LOW);
        digitalWrite(PIN_BRIDGE_EN, LOW);
        ledcWrite(LEDC_CH_RPWM, 0);
        ledcWrite(LEDC_CH_LPWM, 0);
    } else if (!state.clutch_requested) {
        state.clutch_engage_start = 0;
        digitalWrite(PIN_CLUTCH, LOW);
        digitalWrite(PIN_BRIDGE_EN, LOW);
    }
}

void actuator_update(AppState& state) {
    // Watchdog: disengage if no pilot update for too long
    if (millis() - state.last_pilot_ms > WATCHDOG_TIMEOUT_MS) {
        if (state.clutch_engaged) {
            state.clutch_requested = false;
            state.fault_code = FAULT_WATCHDOG;
        }
    }

    update_clutch(state);

    if (!state.clutch_engaged) {
        ledcWrite(LEDC_CH_RPWM, 0);
        ledcWrite(LEDC_CH_LPWM, 0);
        return;
    }

    // Position P-controller
    float error = state.rudder_target - state.rudder_actual;

    if (fabsf(error) < POSITION_DEADBAND_NORM) {
        ledcWrite(LEDC_CH_RPWM, 0);
        ledcWrite(LEDC_CH_LPWM, 0);
        return;
    }

    float pwm_output = POSITION_KP * error;
    enforce_limits(pwm_output, state);

    if (state.fault_code == FAULT_NONE || state.fault_code == FAULT_WATCHDOG) {
        if (pwm_output > 0) {
            uint8_t pwm = constrain((int)(pwm_output * 255), PWM_MIN, PWM_MAX);
            ledcWrite(LEDC_CH_RPWM, pwm);
            ledcWrite(LEDC_CH_LPWM, 0);
        } else if (pwm_output < 0) {
            uint8_t pwm = constrain((int)(-pwm_output * 255), PWM_MIN, PWM_MAX);
            ledcWrite(LEDC_CH_LPWM, pwm);
            ledcWrite(LEDC_CH_RPWM, 0);
        } else {
            ledcWrite(LEDC_CH_RPWM, 0);
            ledcWrite(LEDC_CH_LPWM, 0);
        }
    } else {
        ledcWrite(LEDC_CH_RPWM, 0);
        ledcWrite(LEDC_CH_LPWM, 0);
    }
}

void actuator_save_calibration() {
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putUShort("adc_center", adc_center);
    prefs.putUShort("adc_port", adc_port_limit);
    prefs.putUShort("adc_stbd", adc_stbd_limit);
    prefs.end();
}

void actuator_calibrate_center() {
    adc_center = analogRead(PIN_ADC_RUDDER);
    actuator_save_calibration();
}

void actuator_calibrate_port() {
    adc_port_limit = analogRead(PIN_ADC_RUDDER);
    actuator_save_calibration();
}

void actuator_calibrate_stbd() {
    adc_stbd_limit = analogRead(PIN_ADC_RUDDER);
    actuator_save_calibration();
}

#endif // NATIVE_BUILD
