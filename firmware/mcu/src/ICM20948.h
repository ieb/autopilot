/**
 * ICM-20948 9-DoF IMU Driver
 * ==========================
 *
 * Lightweight driver for ATtiny series MCUs with limited resources.
 * Supports accelerometer, gyroscope, and magnetometer.
 *
 * Compatible with ATtiny3224, ATtiny3226, and similar megaAVR devices.
 */

#ifndef ICM20948_H
#define ICM20948_H

#include <Arduino.h>
#include <Wire.h>

// ============================================================================
// Register Addresses
// ============================================================================

// User Bank 0
#define ICM20948_REG_WHO_AM_I 0x00
#define ICM20948_REG_USER_CTRL 0x03
#define ICM20948_REG_LP_CONFIG 0x05
#define ICM20948_REG_PWR_MGMT_1 0x06
#define ICM20948_REG_PWR_MGMT_2 0x07
#define ICM20948_REG_INT_PIN_CFG 0x0F
#define ICM20948_REG_INT_ENABLE 0x10
#define ICM20948_REG_INT_STATUS 0x19
#define ICM20948_REG_ACCEL_XOUT_H 0x2D
#define ICM20948_REG_GYRO_XOUT_H 0x33
#define ICM20948_REG_TEMP_OUT_H 0x39
#define ICM20948_REG_EXT_SLV_SENS_DATA_00 0x3B
#define ICM20948_REG_BANK_SEL 0x7F

// User Bank 2
#define ICM20948_REG_GYRO_SMPLRT_DIV 0x00
#define ICM20948_REG_GYRO_CONFIG_1 0x01
#define ICM20948_REG_ACCEL_SMPLRT_DIV_1 0x10
#define ICM20948_REG_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20948_REG_ACCEL_CONFIG 0x14

// User Bank 3 (I2C master)
#define ICM20948_REG_I2C_MST_CTRL 0x01
#define ICM20948_REG_I2C_SLV0_ADDR 0x03
#define ICM20948_REG_I2C_SLV0_REG 0x04
#define ICM20948_REG_I2C_SLV0_CTRL 0x05
#define ICM20948_REG_I2C_SLV0_DO 0x06

// AK09916 Magnetometer registers
#define AK09916_I2C_ADDR 0x0C
#define AK09916_REG_WIA2 0x01
#define AK09916_REG_ST1 0x10
#define AK09916_REG_HXL 0x11
#define AK09916_REG_CNTL2 0x31
#define AK09916_REG_CNTL3 0x32

// Device ID
#define ICM20948_WHO_AM_I_ID 0xEA
#define AK09916_WHO_AM_I_ID 0x09

// ============================================================================
// Configuration Enums
// ============================================================================

enum ICM20948AccelRange {
  ICM20948_ACCEL_RANGE_2G = 0,
  ICM20948_ACCEL_RANGE_4G = 1,
  ICM20948_ACCEL_RANGE_8G = 2,
  ICM20948_ACCEL_RANGE_16G = 3
};

enum ICM20948GyroRange {
  ICM20948_GYRO_RANGE_250DPS = 0,
  ICM20948_GYRO_RANGE_500DPS = 1,
  ICM20948_GYRO_RANGE_1000DPS = 2,
  ICM20948_GYRO_RANGE_2000DPS = 3
};

// ============================================================================
// ICM20948 Class
// ============================================================================

class ICM20948 {
public:
  ICM20948(uint8_t addr = 0x68) : _addr(addr), _currentBank(0) {}

  /**
   * Initialize the sensor
   * @return true if successful
   */
  bool begin() {
    // Reset device
    selectBank(0);
    writeRegister(ICM20948_REG_PWR_MGMT_1, 0x80); // Reset
    delay(100);

    // Wake up
    writeRegister(ICM20948_REG_PWR_MGMT_1, 0x01); // Auto select best clock
    delay(50);

    // Check WHO_AM_I
    uint8_t whoami = readRegister(ICM20948_REG_WHO_AM_I);
    if (whoami != ICM20948_WHO_AM_I_ID) {
      return false;
    }

    // Enable all sensors
    writeRegister(ICM20948_REG_PWR_MGMT_2, 0x00);

    // Default configuration
    setAccelRange(ICM20948_ACCEL_RANGE_4G);
    setGyroRange(ICM20948_GYRO_RANGE_500DPS);

    return true;
  }

  /**
   * Enable the magnetometer via I2C master
   */
  bool enableMagnetometer() {
    selectBank(0);

    // Enable I2C master
    writeRegister(ICM20948_REG_USER_CTRL, 0x20);

    // Configure I2C master
    selectBank(3);
    writeRegister(ICM20948_REG_I2C_MST_CTRL, 0x07); // 400kHz

    // Reset magnetometer
    writeMagRegister(AK09916_REG_CNTL3, 0x01);
    delay(100);

    // Check magnetometer WHO_AM_I
    uint8_t magId = readMagRegister(AK09916_REG_WIA2);
    if (magId != AK09916_WHO_AM_I_ID) {
      return false;
    }

    // Set continuous measurement mode 4 (100Hz)
    writeMagRegister(AK09916_REG_CNTL2, 0x08);

    // Configure SLV0 to read magnetometer data
    selectBank(3);
    writeRegister(ICM20948_REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80); // Read
    writeRegister(ICM20948_REG_I2C_SLV0_REG, AK09916_REG_ST1);
    writeRegister(ICM20948_REG_I2C_SLV0_CTRL, 0x89); // Enable, 9 bytes

    selectBank(0);
    return true;
  }

  /**
   * Set accelerometer range
   */
  void setAccelRange(ICM20948AccelRange range) {
    _accelRange = range;
    selectBank(2);
    uint8_t config = readRegister(ICM20948_REG_ACCEL_CONFIG);
    config = (config & 0xF9) | (range << 1);
    writeRegister(ICM20948_REG_ACCEL_CONFIG, config);
    selectBank(0);

    // Update scale factor
    switch (range) {
    case ICM20948_ACCEL_RANGE_2G:
      _accelScale = 2.0f / 32768.0f;
      break;
    case ICM20948_ACCEL_RANGE_4G:
      _accelScale = 4.0f / 32768.0f;
      break;
    case ICM20948_ACCEL_RANGE_8G:
      _accelScale = 8.0f / 32768.0f;
      break;
    case ICM20948_ACCEL_RANGE_16G:
      _accelScale = 16.0f / 32768.0f;
      break;
    }
  }

  /**
   * Set gyroscope range
   */
  void setGyroRange(ICM20948GyroRange range) {
    _gyroRange = range;
    selectBank(2);
    uint8_t config = readRegister(ICM20948_REG_GYRO_CONFIG_1);
    config = (config & 0xF9) | (range << 1);
    writeRegister(ICM20948_REG_GYRO_CONFIG_1, config);
    selectBank(0);

    // Update scale factor
    switch (range) {
    case ICM20948_GYRO_RANGE_250DPS:
      _gyroScale = 250.0f / 32768.0f;
      break;
    case ICM20948_GYRO_RANGE_500DPS:
      _gyroScale = 500.0f / 32768.0f;
      break;
    case ICM20948_GYRO_RANGE_1000DPS:
      _gyroScale = 1000.0f / 32768.0f;
      break;
    case ICM20948_GYRO_RANGE_2000DPS:
      _gyroScale = 2000.0f / 32768.0f;
      break;
    }
  }

  /**
   * Set sample rate divider
   */
  void setSampleRate(uint16_t rate) {
    // Sample rate = 1.125kHz / (1 + divider)
    uint16_t divider = (1125 / rate) - 1;

    selectBank(2);
    writeRegister(ICM20948_REG_GYRO_SMPLRT_DIV, divider & 0xFF);

    // Accelerometer uses 16-bit divider
    writeRegister(ICM20948_REG_ACCEL_SMPLRT_DIV_1, (divider >> 8) & 0xFF);
    writeRegister(ICM20948_REG_ACCEL_SMPLRT_DIV_2, divider & 0xFF);

    selectBank(0);
  }

  /**
   * Check if new data is available
   */
  bool dataReady() {
    selectBank(0);
    uint8_t status = readRegister(ICM20948_REG_INT_STATUS);
    return (status & 0x01) != 0;
  }

  /**
   * Read all sensor data
   */
  void read() {
    selectBank(0);

    // Read accel, gyro, temp in one burst (14 bytes)
    uint8_t buffer[20];
    readRegisters(ICM20948_REG_ACCEL_XOUT_H, buffer, 14);

    // Parse accelerometer (big-endian)
    _rawAccel[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
    _rawAccel[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
    _rawAccel[2] = (int16_t)((buffer[4] << 8) | buffer[5]);

    // Parse gyroscope
    _rawGyro[0] = (int16_t)((buffer[6] << 8) | buffer[7]);
    _rawGyro[1] = (int16_t)((buffer[8] << 8) | buffer[9]);
    _rawGyro[2] = (int16_t)((buffer[10] << 8) | buffer[11]);

    // Read magnetometer from external sensor data (via I2C master)
    readRegisters(ICM20948_REG_EXT_SLV_SENS_DATA_00, buffer, 9);

    // Check if mag data is ready (ST1 bit 0)
    if (buffer[0] & 0x01) {
      // Parse magnetometer (little-endian!)
      _rawMag[0] = (int16_t)(buffer[1] | (buffer[2] << 8));
      _rawMag[1] = (int16_t)(buffer[3] | (buffer[4] << 8));
      _rawMag[2] = (int16_t)(buffer[5] | (buffer[6] << 8));
    }
  }

  // Scaled getters (in physical units)
  float accelX() { return _rawAccel[0] * _accelScale; } // g
  float accelY() { return _rawAccel[1] * _accelScale; }
  float accelZ() { return _rawAccel[2] * _accelScale; }

  float gyroX() { return _rawGyro[0] * _gyroScale; } // deg/s
  float gyroY() { return _rawGyro[1] * _gyroScale; }
  float gyroZ() { return _rawGyro[2] * _gyroScale; }

  float magX() { return _rawMag[0] * 0.15f; } // uT (0.15 uT/LSB)
  float magY() { return _rawMag[1] * 0.15f; }
  float magZ() { return _rawMag[2] * 0.15f; }

private:
  uint8_t _addr;
  uint8_t _currentBank;

  ICM20948AccelRange _accelRange;
  ICM20948GyroRange _gyroRange;

  float _accelScale = 4.0f / 32768.0f;  // Default ±4g
  float _gyroScale = 500.0f / 32768.0f; // Default ±500 dps

  int16_t _rawAccel[3];
  int16_t _rawGyro[3];
  int16_t _rawMag[3];

  void selectBank(uint8_t bank) {
    if (bank != _currentBank) {
      Wire.beginTransmission(_addr);
      Wire.write(ICM20948_REG_BANK_SEL);
      Wire.write(bank << 4);
      Wire.endTransmission();
      _currentBank = bank;
    }
  }

  void writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }

  uint8_t readRegister(uint8_t reg) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, (uint8_t)1);
    return Wire.read();
  }

  void readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
      buffer[i] = Wire.read();
    }
  }

  void writeMagRegister(uint8_t reg, uint8_t value) {
    selectBank(3);
    writeRegister(ICM20948_REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR); // Write
    writeRegister(ICM20948_REG_I2C_SLV0_REG, reg);
    writeRegister(ICM20948_REG_I2C_SLV0_DO, value);
    writeRegister(ICM20948_REG_I2C_SLV0_CTRL, 0x81); // Enable, 1 byte
    delay(10);
    selectBank(0);
  }

  uint8_t readMagRegister(uint8_t reg) {
    selectBank(3);
    writeRegister(ICM20948_REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80); // Read
    writeRegister(ICM20948_REG_I2C_SLV0_REG, reg);
    writeRegister(ICM20948_REG_I2C_SLV0_CTRL, 0x81); // Enable, 1 byte
    delay(10);
    selectBank(0);
    return readRegister(ICM20948_REG_EXT_SLV_SENS_DATA_00);
  }
};

#endif // ICM20948_H
