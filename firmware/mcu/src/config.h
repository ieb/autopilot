/**
 * Configuration Header
 * ====================
 *
 * Hardware configuration for IMU MCU
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// Pin Definitions for ATtiny3226
// ============================================================================

// I2C pins (hardware I2C on ATtiny3226)
// SDA = PA1 (pin 2)
// SCL = PA2 (pin 3)

// Serial TX = PA1 (alternate) or PB2 (default)
// Using default UART on PB2/PB3

// Optional LED for status indication
#define LED_PIN PIN_PA7

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
// Calibration
// ============================================================================

// EEPROM address for calibration data
#define EEPROM_CAL_ADDR 0

// Calibration duration in milliseconds
#define CALIBRATION_DURATION_MS 30000

#endif // CONFIG_H
