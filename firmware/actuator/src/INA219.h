/**
 * INA219 I2C Current/Voltage Monitor Driver
 * ==========================================
 *
 * Minimal driver for INA219 current sensing IC.
 * Designed for ATtiny3226 with limited flash.
 *
 * Features:
 * - Current measurement via external shunt
 * - Bus voltage measurement
 * - Power calculation
 * - Configurable for various shunt values
 */

#ifndef INA219_H
#define INA219_H

#include <Arduino.h>
#include <Wire.h>

// INA219 Register Addresses
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNT_VOLTAGE 0x01
#define INA219_REG_BUS_VOLTAGE  0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

// Configuration register bits
#define INA219_CONFIG_RESET                 0x8000
#define INA219_CONFIG_BVOLTAGERANGE_32V     0x2000
#define INA219_CONFIG_BVOLTAGERANGE_16V     0x0000

#define INA219_CONFIG_GAIN_1_40MV           0x0000  // ±40mV
#define INA219_CONFIG_GAIN_2_80MV           0x0800  // ±80mV
#define INA219_CONFIG_GAIN_4_160MV          0x1000  // ±160mV
#define INA219_CONFIG_GAIN_8_320MV          0x1800  // ±320mV (default)

#define INA219_CONFIG_BADCRES_12BIT         0x0180  // 12-bit, 532us
#define INA219_CONFIG_SADCRES_12BIT_1S      0x0018  // 12-bit, 532us

#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS 0x0007


class INA219 {
public:
    /**
     * Constructor
     * @param addr I2C address (default 0x40)
     */
    INA219(uint8_t addr = 0x40) : _addr(addr), _calValue(0), _currentLSB(0) {}
    
    /**
     * Initialize the INA219
     * @param shuntOhms Shunt resistor value in ohms
     * @param maxCurrent Maximum expected current in amps
     * @return true if successful
     */
    bool begin(float shuntOhms = 0.01f, float maxCurrent = 20.0f) {
        Wire.begin();
        
        // Check if device is present
        Wire.beginTransmission(_addr);
        if (Wire.endTransmission() != 0) {
            return false;
        }
        
        // Reset
        writeRegister(INA219_REG_CONFIG, INA219_CONFIG_RESET);
        delay(1);
        
        // Calculate calibration
        // Current_LSB = Max_Current / 2^15 = maxCurrent / 32768
        _currentLSB = maxCurrent / 32768.0f;
        
        // Cal = trunc(0.04096 / (Current_LSB * R_shunt))
        _calValue = (uint16_t)(0.04096f / (_currentLSB * shuntOhms));
        
        // Write calibration
        writeRegister(INA219_REG_CALIBRATION, _calValue);
        
        // Configure: 32V bus, ±320mV shunt (for 0.01Ω this is ±32A), 12-bit, continuous
        uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                          INA219_CONFIG_GAIN_8_320MV |
                          INA219_CONFIG_BADCRES_12BIT |
                          INA219_CONFIG_SADCRES_12BIT_1S |
                          INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
        writeRegister(INA219_REG_CONFIG, config);
        
        return true;
    }
    
    /**
     * Read shunt voltage in millivolts
     */
    float getShuntVoltage_mV() {
        int16_t value = readRegister(INA219_REG_SHUNT_VOLTAGE);
        return value * 0.01f;  // 10uV per bit
    }
    
    /**
     * Read bus voltage in volts
     */
    float getBusVoltage_V() {
        uint16_t value = readRegister(INA219_REG_BUS_VOLTAGE);
        // Shift right 3 bits (bits 0-2 are status), multiply by 4mV
        return ((value >> 3) * 4) * 0.001f;
    }
    
    /**
     * Read current in amps
     */
    float getCurrent_A() {
        // Re-write calibration in case of power glitch
        writeRegister(INA219_REG_CALIBRATION, _calValue);
        
        int16_t value = readRegister(INA219_REG_CURRENT);
        return value * _currentLSB;
    }
    
    /**
     * Read power in watts
     */
    float getPower_W() {
        int16_t value = readRegister(INA219_REG_POWER);
        // Power LSB = 20 * Current_LSB
        return value * (_currentLSB * 20);
    }
    
    /**
     * Check if conversion is ready
     */
    bool conversionReady() {
        uint16_t value = readRegister(INA219_REG_BUS_VOLTAGE);
        return (value & 0x0002) != 0;  // CNVR bit
    }
    
    /**
     * Check for math overflow
     */
    bool overflow() {
        uint16_t value = readRegister(INA219_REG_BUS_VOLTAGE);
        return (value & 0x0001) != 0;  // OVF bit
    }

private:
    uint8_t _addr;
    uint16_t _calValue;
    float _currentLSB;
    
    void writeRegister(uint8_t reg, uint16_t value) {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        Wire.write((value >> 8) & 0xFF);
        Wire.write(value & 0xFF);
        Wire.endTransmission();
    }
    
    uint16_t readRegister(uint8_t reg) {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        Wire.endTransmission();
        
        Wire.requestFrom(_addr, (uint8_t)2);
        uint16_t value = Wire.read() << 8;
        value |= Wire.read();
        return value;
    }
};

#endif // INA219_H
