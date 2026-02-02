/**
 * Configuration Header
 * ====================
 *
 * Hardware configuration for IMU MCU
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// Pin Definitions for ATtiny3224 (14-pin package)
// ============================================================================
//
// Available pins: PA0-PA3, PB0-PB3
//
// PA0 = UPDI (programming)
// PA1 = SDA (I2C data)
// PA2 = SCL (I2C clock)
// PA3 = LED (status)
// PB0 = Available
// PB1 = Available
// PB2 = TX (serial out)
// PB3 = RX (serial in)

// I2C pins (hardware I2C on ATtiny3224)
// SDA = PA1 (pin 4)
// SCL = PA2 (pin 5)

// Serial TX = PB2 (default UART)
// Serial RX = PB3 (default UART)

// Status LED for indication (PA3 on 14-pin package, PA7 not available)
#define LED_PIN PIN_PA3

// ============================================================================
// I2C Addresses
// ============================================================================

// ICM-20948 I2C address (AD0 pin low = 0x68, high = 0x69)
#define ICM20948_I2C_ADDR 0x68

// AK09916 magnetometer (inside ICM-20948) address
#define AK09916_I2C_ADDR 0x0C

// ============================================================================
// IMU Configuration
// ============================================================================

// Accelerometer range options:
// ICM20948_ACCEL_RANGE_2G, _4G, _8G, _16G
#define DEFAULT_ACCEL_RANGE ICM20948_ACCEL_RANGE_4G

// Gyroscope range options:
// ICM20948_GYRO_RANGE_250DPS, _500DPS, _1000DPS, _2000DPS
#define DEFAULT_GYRO_RANGE ICM20948_GYRO_RANGE_500DPS

// ============================================================================
// Filter Configuration
// ============================================================================

// Madgwick filter gain (beta)
// Higher = faster convergence but more noise sensitivity
// Lower = smoother but slower to track changes
// Typical values: 0.01 to 0.5
#define MADGWICK_BETA 0.1f

// ============================================================================
// Timing Configuration
// ============================================================================

// Main loop timing (can be overridden by build flags)
#ifndef IMU_UPDATE_RATE_HZ
#define IMU_UPDATE_RATE_HZ 100
#endif

#ifndef SERIAL_OUTPUT_RATE_HZ
#define SERIAL_OUTPUT_RATE_HZ 100
#endif

// ============================================================================
// Serial Configuration
// ============================================================================

#define SERIAL_BAUD_RATE 115200

// ============================================================================
// Protocol Configuration
// ============================================================================

// Configuration timeout in milliseconds
// If no configuration is received within this time, IMU enters error state
// Power cycle required to recover
#ifndef CONFIG_TIMEOUT_MS
#define CONFIG_TIMEOUT_MS 30000
#endif

// Interactive magnetometer calibration duration in milliseconds
#define CALIBRATION_DURATION_MS 30000

// ============================================================================
// IMU Mounting Offset Configuration
// ============================================================================
// 
// When the IMU is mounted away from the boat's center of rotation,
// rotational motion creates centripetal and tangential accelerations
// that must be compensated. The offset is configured at startup by the Pi.
//
// Coordinate system (NED-like, boat-relative):
//   X: Forward (positive = toward bow)
//   Y: Starboard (positive = toward starboard)
//   Z: Down (positive = below waterline)

#endif // CONFIG_H
