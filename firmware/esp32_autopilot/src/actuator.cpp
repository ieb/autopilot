#ifndef NATIVE_BUILD

#include "actuator.h"
#include "config.h"
#include <Arduino.h>
#include <Preferences.h>
#include <esp_adc_cal.h>

// Calibration data (stored as millivolts for linearity)
static uint32_t mv_center = ADC_RUDDER_CENTER_MV;
static uint32_t mv_port_limit = ADC_RUDDER_PORT_MV;
static uint32_t mv_stbd_limit = ADC_RUDDER_STBD_MV;

static float last_angle = 0.0f;
static uint16_t stall_timer = 0;

static Preferences prefs;

// ESP32 ADC calibration characteristics
static esp_adc_cal_characteristics_t adc_chars;
static bool adc_cal_valid = false;

// Read ADC as calibrated millivolts
static uint32_t read_rudder_mv() {
    uint32_t raw = analogRead(PIN_ADC_RUDDER);
    if (adc_cal_valid) {
        return esp_adc_cal_raw_to_voltage(raw, &adc_chars);
    }
    // Fallback: linear approximation (3.3V over 12-bit range)
    return (raw * 3300) / 4095;
}

static float mv_to_angle(uint32_t mv) {
    float angle;
    if (mv < mv_center) {
        float range = (float)(mv_center - mv_port_limit);
        if (range < 1.0f) return 0.0f;
        angle = -((float)(mv_center - mv) / range);
    } else {
        float range = (float)(mv_stbd_limit - mv_center);
        if (range < 1.0f) return 0.0f;
        angle = (float)(mv - mv_center) / range;
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

    // Initialize ESP32 ADC calibration (uses eFuse factory cal if available)
    esp_adc_cal_value_t cal_type = esp_adc_cal_characterize(
        ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    adc_cal_valid = (cal_type != ESP_ADC_CAL_VAL_NOT_SUPPORTED);

    // Load calibration from NVS (millivolts)
    prefs.begin(NVS_NAMESPACE, true);  // read-only
    mv_center = prefs.getULong("adc_center_mv", ADC_RUDDER_CENTER_MV);
    mv_port_limit = prefs.getULong("adc_port_mv", ADC_RUDDER_PORT_MV);
    mv_stbd_limit = prefs.getULong("adc_stbd_mv", ADC_RUDDER_STBD_MV);
    prefs.end();
}

void actuator_read_sensors(AppState& state) {
    uint32_t mv = read_rudder_mv();
    float new_angle = mv_to_angle(mv);

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
    prefs.putULong("adc_center_mv", mv_center);
    prefs.putULong("adc_port_mv", mv_port_limit);
    prefs.putULong("adc_stbd_mv", mv_stbd_limit);
    prefs.end();
}

void actuator_calibrate_center() {
    mv_center = read_rudder_mv();
    actuator_save_calibration();
}

void actuator_calibrate_port() {
    mv_port_limit = read_rudder_mv();
    actuator_save_calibration();
}

void actuator_calibrate_stbd() {
    mv_stbd_limit = read_rudder_mv();
    actuator_save_calibration();
}

void actuator_calibration_linearise() {
    // Make port and starboard ranges symmetric by clamping the wider side.
    // This ensures ±1.0 maps to equal physical deflection on both sides.
    uint32_t port_range = mv_center - mv_port_limit;
    uint32_t stbd_range = mv_stbd_limit - mv_center;
    uint32_t min_range = (port_range < stbd_range) ? port_range : stbd_range;
    mv_port_limit = mv_center - min_range;
    mv_stbd_limit = mv_center + min_range;
    actuator_save_calibration();
}

void actuator_get_calibration(uint32_t& center, uint32_t& port, uint32_t& stbd) {
    center = mv_center;
    port = mv_port_limit;
    stbd = mv_stbd_limit;
}

uint32_t actuator_read_raw_mv() {
    return read_rudder_mv();
}

#endif // NATIVE_BUILD
