// Arduino function stubs for HAL-level native simulation.
// GPIO, ADC, PWM functions that actuator.cpp and other firmware code call.

#ifdef HAL_SIM

#include "Arduino.h"
#include <thread>
#include <chrono>
#include <atomic>

// Global Serial instance
HardwareSerial Serial;

// --- Simulated hardware state (read/written by SimEnvironment) ---

// Rudder potentiometer reading in millivolts (set by sim_environment)
static std::atomic<uint32_t> sim_adc_rudder_mv{1650};

// PWM duty values (read by sim_environment to determine motor drive)
static std::atomic<int> sim_pwm_duty[4] = {};  // indexed by LEDC channel

// Digital pin state
static std::atomic<int> sim_pin_state[64] = {};

// --- Public sim interface ---

void hal_sim_set_rudder_adc_mv(uint32_t mv) {
    sim_adc_rudder_mv.store(mv);
}

int hal_sim_get_pwm_duty(int channel) {
    if (channel < 0 || channel >= 4) return 0;
    return sim_pwm_duty[channel].load();
}

int hal_sim_get_pin(int pin) {
    if (pin < 0 || pin >= 64) return 0;
    return sim_pin_state[pin].load();
}

// --- Arduino API stubs ---

void delay(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void pinMode(int pin, int mode) {
    (void)pin; (void)mode;
}

void digitalWrite(int pin, int val) {
    if (pin >= 0 && pin < 64) {
        sim_pin_state[pin].store(val);
    }
}

int digitalRead(int pin) {
    if (pin >= 0 && pin < 64) return sim_pin_state[pin].load();
    return 0;
}

uint32_t analogRead(int pin) {
    (void)pin;
    // Return simulated rudder ADC value (in millivolts for esp_adc_cal_raw_to_voltage)
    return sim_adc_rudder_mv.load();
}

void ledcSetup(int channel, int freq, int resolution) {
    (void)channel; (void)freq; (void)resolution;
}

void ledcAttachPin(int pin, int channel) {
    (void)pin; (void)channel;
}

void ledcWrite(int channel, int duty) {
    if (channel >= 0 && channel < 4) {
        sim_pwm_duty[channel].store(duty);
    }
}

#endif // HAL_SIM
