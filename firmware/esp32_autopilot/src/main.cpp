/**
 * ESP32-S3 Consolidated Autopilot
 * ================================
 *
 * Single-board autopilot for Pogo 1250.
 * Combines IMU fusion, NMEA2000, pilot algorithms,
 * actuator control, and web configuration.
 *
 * All application code runs on Core 1 (APP_CPU) via Arduino loop().
 * Core 0 is reserved for WiFi/LWIP/system tasks.
 */

#ifndef NATIVE_BUILD

#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "app_state.h"
#include "imu.h"
#include "n2k.h"
#include "actuator.h"
#include "pilot_manager.h"
#include "seatalk.h"
#include "web.h"

static AppState state;

// Timing
static uint32_t last_imu_ms = 0;
static uint32_t last_pilot_ms = 0;
static uint32_t last_actuator_ms = 0;
static uint32_t last_n2k_send_ms = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("Pogo Autopilot v1.0 starting...");

    // Zero state
    memset(&state, 0, sizeof(state));
    state.pilot_mode = MODE_STANDBY;
    state.pilot_type = PILOT_PD;

    // Initialize subsystems
    if (!imu_init()) {
        Serial.println("ERROR: BNO055 not found!");
        state.fault_code = FAULT_IMU;
    } else {
        Serial.println("BNO055 OK");
    }

    n2k_init();
    Serial.println("NMEA2000 OK");

    actuator_init();
    Serial.println("Actuator OK");

    pilot_manager_init();
    Serial.println("Pilot manager OK");

    web_init();
    Serial.printf("Web UI at http://%s/\n", WiFi.softAPIP().toString().c_str());

    digitalWrite(PIN_LED, HIGH);
    Serial.println("Ready.");
}

void loop() {
    uint32_t now = millis();

    // NMEA2000 parse — every loop iteration
    n2k_update(state);

    // Apply pending web and p70 commands
    web_apply_commands(state);
    seatalk_apply_commands(state);

    // IMU read at 20Hz
    if (now - last_imu_ms >= IMU_INTERVAL_MS) {
        last_imu_ms = now;
        if (state.fault_code != FAULT_IMU) {
            imu_read(state);
        }
    }

    // Pilot update at 5Hz
    if (now - last_pilot_ms >= PILOT_INTERVAL_MS) {
        last_pilot_ms = now;
        pilot_manager_update(state);
    }

    // Actuator control at 50Hz
    if (now - last_actuator_ms >= ACTUATOR_INTERVAL_MS) {
        last_actuator_ms = now;
        actuator_read_sensors(state);
        actuator_update(state);
    }

    // N2K transmit at 5Hz
    if (now - last_n2k_send_ms >= N2K_SEND_INTERVAL_MS) {
        last_n2k_send_ms = now;
        n2k_send(state);
    }
}

#endif // NATIVE_BUILD
