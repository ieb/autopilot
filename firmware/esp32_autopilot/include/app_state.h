#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>

// ============================================================================
// Enums
// ============================================================================

enum PilotMode : uint8_t {
    MODE_STANDBY   = 0,
    MODE_COMPASS   = 1,
    MODE_WIND_AWA  = 2,
    MODE_WIND_TWA  = 3,
    MODE_VMG_UP    = 4,
    MODE_VMG_DOWN  = 5,
};

enum PilotType : uint8_t {
    PILOT_PD       = 0,
    PILOT_PID      = 1,
    PILOT_SMOOTH   = 2,
    PILOT_ADAPTIVE = 3,
};

// ============================================================================
// Pilot Features (subset of the 22-element Python feature vector)
// ============================================================================

struct PilotFeatures {
    float heading_error;    // feat[0]: error / 90.0
    float mode_flag;        // feat[1]: compass=0.0, awa=0.5, twa=1.0
    float heading_rate;     // feat[2]: rate / 30.0
    float roll;             // feat[3]: roll / 45.0
    float stw;              // feat[11]: stw / 25.0
    float rudder_position;  // feat[14]: normalized -1..+1
    float pd_suggestion;    // feat[19]: computed inline
};

// ============================================================================
// Application State
// ============================================================================

struct AppState {
    // From BNO055 (updated at 20Hz)
    float heading;           // deg, 0-360
    float pitch;             // deg
    float roll;              // deg
    float yaw_rate;          // deg/s (from gyro)
    float roll_rate;         // deg/s
    uint32_t imu_last_ms;

    // From NMEA2000 message handlers
    float awa;               // deg, signed (+ starboard)
    float aws;               // kts
    float twa;               // deg, signed
    float tws;               // kts
    float stw;               // kts
    float sog;               // kts
    float cog;               // deg, 0-360
    uint32_t n2k_wind_ms;    // last wind message time
    uint32_t n2k_speed_ms;
    uint32_t n2k_cog_ms;
    uint32_t n2k_heading_ms;

    // Actuator (updated at 50Hz)
    float rudder_actual;     // normalized -1..+1
    float rudder_velocity;   // normalized/s
    float motor_current;     // Amps
    float supply_voltage;    // Volts

    // Pilot output
    float rudder_target;     // normalized -1..+1
    PilotMode pilot_mode;
    PilotType pilot_type;
    float target_value;      // target heading/AWA/TWA (degrees)

    // Faults
    uint8_t fault_code;

    // Clutch
    bool clutch_engaged;
    bool clutch_requested;
    uint32_t clutch_engage_start;

    // Watchdog
    uint32_t last_pilot_ms;  // last pilot update time for actuator watchdog
};

#endif // APP_STATE_H
