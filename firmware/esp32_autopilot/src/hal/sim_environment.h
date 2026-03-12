// SimEnvironment — centralizes sailing dynamics for HAL-level simulation.
// Owns the physical simulation state (heading, wind, rudder) and pushes
// data into the HAL layer: BNO055 registers via Wire mock, N2K messages
// via NMEA2000_sim, and ADC values via arduino_stubs.

#ifndef SIM_ENVIRONMENT_H
#define SIM_ENVIRONMENT_H

#include <stdint.h>
#include <cmath>

// Forward declarations
class tNMEA2000_sim;
class TwoWire;

class SimEnvironment {
public:
    SimEnvironment(tNMEA2000_sim& nmea, TwoWire& wire);

    // Set true wind conditions
    void set_wind(float tws_kts, float twd_deg);

    // Main update — call each loop iteration
    void update(float dt);

    // External simulator bypass — when active, update() skips all internal
    // dynamics (CAN frames and IMU data come from the socket instead)
    void set_external_active(bool active);
    bool is_external_active() const { return external_active; }

    // Read-only access to sim state (for console display)
    float get_heading() const { return heading; }
    float get_tws() const { return tws; }
    float get_twd() const { return twd; }

    // Simulated motor current (A) based on PWM duty: ~3A at full duty
    float get_motor_current() const { return fabsf(motor_command) * 3.0f; }
    bool get_clutch_engaged() const { return clutch_engaged; }

private:
    // Update sub-systems
    void read_actuator_pwm();
    void update_rudder_dynamics(float dt);
    void update_heading_dynamics(float dt);
    void compute_wind();
    void update_imu_registers();
    void inject_n2k_messages();
    void update_rudder_adc();

    // HAL references
    tNMEA2000_sim& nmea;
    TwoWire& wire;

    // Sailing state
    float heading = 180.0f;   // degrees, 0-360
    float yaw_rate = 0.0f;    // deg/s
    float roll = 0.0f;        // degrees
    float pitch = 0.0f;       // degrees
    float rudder_pos = 0.0f;  // normalized -1..+1

    // Environment
    float tws = 12.0f;   // true wind speed (kts)
    float twd = 225.0f;  // true wind direction (deg)

    // Derived values (computed each update)
    float twa = 0.0f;    // true wind angle (signed, deg)
    float stw = 0.0f;    // speed through water (kts)
    float awa = 0.0f;    // apparent wind angle (signed, deg)
    float aws = 0.0f;    // apparent wind speed (kts)
    float sog = 0.0f;
    float cog = 0.0f;

    // PWM state read from actuator
    float motor_command = 0.0f;  // normalized -1..+1
    bool clutch_engaged = false;

    // Actuator rate limit: 1.7 deg/s = 0.068 normalized/s
    static constexpr float ACTUATOR_RATE = 1.7f / 25.0f;

    // N2K injection timing
    uint32_t last_n2k_inject_ms = 0;
    static constexpr uint32_t N2K_INJECT_INTERVAL_MS = 100;  // 10 Hz

    // Pitch phase counter
    uint32_t pitch_phase = 0;

    // External simulator mode
    bool external_active = false;
};

#endif // SIM_ENVIRONMENT_H
