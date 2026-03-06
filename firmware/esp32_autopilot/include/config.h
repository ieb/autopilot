#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// Pin Definitions (ESP32-S3 DevKitC)
// ============================================================================

// I2C — BNO055 IMU
#define PIN_I2C_SDA          8
#define PIN_I2C_SCL          9

// TWAI CAN — NMEA2000 via TJA1051 transceiver
#define PIN_CAN_TX           4
#define PIN_CAN_RX           5

// ADC — Rudder potentiometer
#define PIN_ADC_RUDDER       1

// PWM — BTS7960 H-bridge
#define PIN_PWM_RPWM         15   // Starboard drive
#define PIN_PWM_LPWM         16   // Port drive

// Digital outputs
#define PIN_BRIDGE_EN        17   // BTS7960 R_EN/L_EN
#define PIN_CLUTCH           18   // EM clutch (N-FET gate)
#define PIN_LED              48   // Status LED (onboard NeoPixel on most boards)

// ============================================================================
// Timing (Hz)
// ============================================================================

#define IMU_RATE_HZ          20
#define PILOT_RATE_HZ        5
#define ACTUATOR_RATE_HZ     50
#define N2K_SEND_RATE_HZ     5

// Derived intervals (ms)
#define IMU_INTERVAL_MS      (1000 / IMU_RATE_HZ)
#define PILOT_INTERVAL_MS    (1000 / PILOT_RATE_HZ)
#define ACTUATOR_INTERVAL_MS (1000 / ACTUATOR_RATE_HZ)
#define N2K_SEND_INTERVAL_MS (1000 / N2K_SEND_RATE_HZ)

// ============================================================================
// Actuator Control
// ============================================================================

#define POSITION_KP              0.8f
#define POSITION_DEADBAND_DEG    0.5f
#define POSITION_DEADBAND_NORM   (POSITION_DEADBAND_DEG / 25.0f)
#define POSITION_LIMIT_DEG       28.0f
#define POSITION_LIMIT_NORM      (POSITION_LIMIT_DEG / 25.0f)

#define CURRENT_LIMIT_SOFT       12.0f   // Amps — reduce PWM
#define CURRENT_LIMIT_HARD       15.0f   // Amps — stop motor

#define STALL_CURRENT_THRESHOLD  10.0f   // Amps
#define STALL_VELOCITY_THRESHOLD 0.5f    // normalized/s
#define STALL_TIMEOUT_MS         500

#define CLUTCH_ENGAGE_RAMP_MS    200
#define WATCHDOG_TIMEOUT_MS      2000

#define PWM_MIN                  26      // ~10% minimum
#define PWM_MAX                  255
#define PWM_FREQUENCY            10000   // 10kHz silent operation

// LEDC channels for PWM
#define LEDC_CH_RPWM             0
#define LEDC_CH_LPWM             1
#define LEDC_RESOLUTION          8       // 8-bit (0-255)

// ============================================================================
// ADC Calibration Defaults
// ============================================================================

#define ADC_RUDDER_CENTER        2048    // 12-bit ADC mid-range
#define ADC_RUDDER_PORT_LIMIT    410     // ADC at -25 deg
#define ADC_RUDDER_STBD_LIMIT    3686    // ADC at +25 deg

// BTS7960 IS pin current scale
#define BTS7960_CURRENT_SCALE    0.00806f  // Amps per ADC count (12-bit, 3.3V)

// ============================================================================
// Pilot Defaults
// ============================================================================

#define DEFAULT_KP               1.0f
#define DEFAULT_KI               0.1f
#define DEFAULT_KD               1.5f
#define MAX_RUDDER_DEG           25.0f

// ============================================================================
// Fault Codes
// ============================================================================

#define FAULT_NONE               0
#define FAULT_OVERCURRENT        1
#define FAULT_STALL              2
#define FAULT_POSITION_LIMIT     3
#define FAULT_SENSOR             4
#define FAULT_WATCHDOG           5
#define FAULT_IMU                6
#define FAULT_N2K                7

// ============================================================================
// NVS Keys
// ============================================================================

#define NVS_NAMESPACE            "autopilot"

#endif // CONFIG_H
