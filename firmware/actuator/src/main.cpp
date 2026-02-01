/**
 * Actuator Controller Firmware
 * ============================
 *
 * ATtiny3224 firmware for closed-loop rudder control.
 *
 * Features:
 * - Rudder position sensing via ADC
 * - PWM motor control via H-Bridge
 * - Electromagnetic clutch control with soft-start
 * - H-Bridge enable linked to clutch state
 * - Current/voltage monitoring (ADC or INA219 I2C)
 * - Serial communication with Raspberry Pi
 * - Watchdog timeout for fail-safe operation
 *
 * Current Sensing Options (compile-time, see config.h):
 *   CURRENT_SENSE_ADC    - Direct ADC with shunt + op-amp
 *   CURRENT_SENSE_INA219 - INA219 I2C module (recommended)
 *
 * Serial Protocol:
 *   Commands (from Pi):
 *     $RUD,<target>,<engage>*XX  - Set target angle, engage clutch
 *     $CFG,TIMEOUT,<ms>*XX       - Set watchdog timeout
 *     $CAL,CENTER*XX             - Calibrate center position
 *     $CAL,PORT*XX               - Calibrate port limit
 *     $CAL,STBD*XX               - Calibrate starboard limit
 *
 *   Status (to Pi at 20Hz):
 *     $STS,<target>,<actual>,<clutch>,<voltage>,<current>,<fault>*XX
 */

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"

#ifdef CURRENT_SENSE_INA219
#include <Wire.h>
#include "INA219.h"
#endif

// ============================================================================
// Calibration Data (stored in EEPROM)
// ============================================================================

struct CalibrationData {
    uint16_t adc_center;
    uint16_t adc_port_limit;
    uint16_t adc_stbd_limit;
    float current_scale;
    float voltage_scale;
    uint16_t watchdog_timeout;
    uint16_t magic;
};

CalibrationData calibration;

// ============================================================================
// Current Sensor (compile-time selected)
// ============================================================================

#ifdef CURRENT_SENSE_INA219
INA219 currentSensor(INA219_I2C_ADDRESS);
bool ina219Available = false;
#endif

// ============================================================================
// State Variables
// ============================================================================

// Target and actual positions (normalized -1 to +1)
float targetAngle = 0.0f;
float actualAngle = 0.0f;
float rudderVelocity = 0.0f;
float lastAngle = 0.0f;

// Sensor readings
float motorCurrent = 0.0f;
float supplyVoltage = 0.0f;

// Clutch state
bool clutchEngaged = false;
bool clutchRequested = false;
uint32_t clutchEngageStart = 0;

// Fault state
uint8_t faultCode = FAULT_NONE;

// Timing
uint32_t lastControlLoop = 0;
uint32_t lastSerialOutput = 0;
uint32_t lastCommandTime = 0;
uint16_t watchdogTimeout = WATCHDOG_TIMEOUT_MS;

// Stall detection
uint16_t stallTimer = 0;

// ============================================================================
// Function Declarations
// ============================================================================

void loadCalibration();
void saveCalibration();
void readSensors();
void runPositionControl();
void enforceHardwareLimits(float &pwmOutput);
void updateClutch();
void outputStatus();
void processCommands();
uint8_t computeChecksum(const char *str);
float adcToAngle(uint16_t adc);

// ============================================================================
// Setup
// ============================================================================

void setup() {
    // Initialize serial
    Serial.begin(115200);
    
    // Configure pins
    pinMode(PIN_PWM_PORT, OUTPUT);
    pinMode(PIN_PWM_STBD, OUTPUT);
    pinMode(PIN_CLUTCH, OUTPUT);
    pinMode(PIN_BRIDGE_EN, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    
    // Start with everything off
    analogWrite(PIN_PWM_PORT, 0);
    analogWrite(PIN_PWM_STBD, 0);
    digitalWrite(PIN_CLUTCH, LOW);
    digitalWrite(PIN_BRIDGE_EN, LOW);
    digitalWrite(PIN_LED, LOW);
    
    // Configure ADC (always needed for rudder position)
    analogReference(VDD);  // Use VDD as reference
    
#ifdef CURRENT_SENSE_INA219
    // Initialize INA219 I2C current sensor
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C
    ina219Available = currentSensor.begin(INA219_SHUNT_OHMS, INA219_MAX_CURRENT);
    if (!ina219Available) {
        // Flash LED rapidly to indicate sensor error
        for (int i = 0; i < 10; i++) {
            digitalWrite(PIN_LED, HIGH);
            delay(50);
            digitalWrite(PIN_LED, LOW);
            delay(50);
        }
    }
#endif
    
    // Load calibration from EEPROM
    loadCalibration();
    
    // Initialize timing
    lastCommandTime = millis();
    
    // LED on to indicate ready
    digitalWrite(PIN_LED, HIGH);
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
    uint32_t now = micros();
    
    // Process incoming commands
    processCommands();
    
    // Control loop at 50Hz
    if (now - lastControlLoop >= CONTROL_LOOP_INTERVAL_US) {
        lastControlLoop = now;
        
        // Read sensors
        readSensors();
        
        // Check watchdog timeout
        if (millis() - lastCommandTime > watchdogTimeout) {
            if (clutchEngaged) {
                clutchRequested = false;
                faultCode = FAULT_WATCHDOG;
            }
        }
        
        // Update clutch state
        updateClutch();
        
        // Run position control if clutch engaged
        if (clutchEngaged) {
            runPositionControl();
        } else {
            // Stop motor
            analogWrite(PIN_PWM_PORT, 0);
            analogWrite(PIN_PWM_STBD, 0);
        }
    }
    
    // Output status at 20Hz
    if (now - lastSerialOutput >= SERIAL_OUTPUT_INTERVAL_US) {
        lastSerialOutput = now;
        outputStatus();
    }
}

// ============================================================================
// Sensor Reading
// ============================================================================

void readSensors() {
    // Read rudder position (always via ADC)
    uint16_t adcRudder = analogRead(PIN_ADC_RUDDER);
    float newAngle = adcToAngle(adcRudder);
    
    // Calculate velocity (deg/s, then normalize)
    float dt = CONTROL_LOOP_INTERVAL_US / 1000000.0f;
    rudderVelocity = (newAngle - lastAngle) / dt * 30.0f;  // Convert to deg/s
    lastAngle = actualAngle;
    actualAngle = newAngle;
    
#ifdef CURRENT_SENSE_INA219
    // Read current and voltage via INA219 I2C
    if (ina219Available) {
        motorCurrent = currentSensor.getCurrent_A();
        supplyVoltage = currentSensor.getBusVoltage_V();
        
        // Check for sensor overflow/error
        if (currentSensor.overflow()) {
            faultCode = FAULT_SENSOR;
        }
    } else {
        // Sensor not available - set to zero and flag
        motorCurrent = 0.0f;
        supplyVoltage = 0.0f;
        // Don't set fault continuously, just note it's unavailable
    }
#elif defined(CURRENT_SENSE_BTS7960)
    // Read current via BTS7960 IS pins with software averaging
    uint32_t currentSum = 0;
    for (int i = 0; i < 8; i++) {
        currentSum += analogRead(PIN_ADC_CURRENT);
    }
    motorCurrent = (currentSum / 8.0f) * calibration.current_scale;
    
    // Read voltage via ADC with resistor divider
    uint16_t adcVoltage = analogRead(PIN_ADC_VOLTAGE);
    supplyVoltage = adcVoltage * calibration.voltage_scale;
#else
    // Read current via direct ADC (Option A: shunt + op-amp)
    uint16_t adcCurrent = analogRead(PIN_ADC_CURRENT);
    motorCurrent = adcCurrent * calibration.current_scale;
    
    // Read voltage via ADC with resistor divider
    uint16_t adcVoltage = analogRead(PIN_ADC_VOLTAGE);
    supplyVoltage = adcVoltage * calibration.voltage_scale;
#endif
}

float adcToAngle(uint16_t adc) {
    // Convert ADC reading to normalized angle (-1 to +1)
    float angle;
    
    if (adc < calibration.adc_center) {
        // Port side (negative)
        float range = calibration.adc_center - calibration.adc_port_limit;
        angle = -((float)(calibration.adc_center - adc) / range);
    } else {
        // Starboard side (positive)
        float range = calibration.adc_stbd_limit - calibration.adc_center;
        angle = (float)(adc - calibration.adc_center) / range;
    }
    
    // Clamp to valid range
    return constrain(angle, -1.0f, 1.0f);
}

// ============================================================================
// Position Control
// ============================================================================

void runPositionControl() {
    // Calculate error
    float error = targetAngle - actualAngle;
    
    // Deadband
    if (abs(error) < POSITION_DEADBAND_NORMALIZED) {
        analogWrite(PIN_PWM_PORT, 0);
        analogWrite(PIN_PWM_STBD, 0);
        return;
    }
    
    // P-controller
    float pwmOutput = POSITION_KP * error;
    
    // Apply hardware limits (current, position, stall)
    enforceHardwareLimits(pwmOutput);
    
    // Output to H-Bridge
    if (faultCode == FAULT_NONE || faultCode == FAULT_WATCHDOG) {
        if (pwmOutput > 0) {
            // Move starboard
            uint8_t pwm = constrain((int)(pwmOutput * 255), PWM_MIN, PWM_MAX);
            analogWrite(PIN_PWM_STBD, pwm);
            analogWrite(PIN_PWM_PORT, 0);
        } else if (pwmOutput < 0) {
            // Move port
            uint8_t pwm = constrain((int)(-pwmOutput * 255), PWM_MIN, PWM_MAX);
            analogWrite(PIN_PWM_PORT, pwm);
            analogWrite(PIN_PWM_STBD, 0);
        } else {
            analogWrite(PIN_PWM_PORT, 0);
            analogWrite(PIN_PWM_STBD, 0);
        }
    } else {
        // Fault condition - stop motor
        analogWrite(PIN_PWM_PORT, 0);
        analogWrite(PIN_PWM_STBD, 0);
    }
}

void enforceHardwareLimits(float &pwmOutput) {
    // 1. Position limits - ALWAYS enforced
    if (actualAngle >= POSITION_LIMIT_NORMALIZED && pwmOutput > 0) {
        pwmOutput = 0;
        faultCode = FAULT_POSITION_LIMIT;
    }
    if (actualAngle <= -POSITION_LIMIT_NORMALIZED && pwmOutput < 0) {
        pwmOutput = 0;
        faultCode = FAULT_POSITION_LIMIT;
    }
    
    // 2. Current limiting
    if (motorCurrent > CURRENT_LIMIT_HARD) {
        pwmOutput = 0;
        faultCode = FAULT_OVERCURRENT;
    } else if (motorCurrent > CURRENT_LIMIT_SOFT) {
        pwmOutput *= 0.5f;  // Reduce power
    }
    
    // 3. Stall detection
    if (motorCurrent > STALL_CURRENT_THRESHOLD && 
        abs(rudderVelocity) < STALL_VELOCITY_THRESHOLD) {
        stallTimer += CONTROL_LOOP_INTERVAL_US / 1000;  // Convert to ms
        if (stallTimer > STALL_TIMEOUT_MS) {
            pwmOutput = 0;
            faultCode = FAULT_STALL;
        }
    } else {
        stallTimer = 0;
    }
    
    // Clear position limit fault if moved away from limit
    if (faultCode == FAULT_POSITION_LIMIT) {
        if (abs(actualAngle) < POSITION_LIMIT_NORMALIZED - 0.05f) {
            faultCode = FAULT_NONE;
        }
    }
}

// ============================================================================
// Clutch Control
// ============================================================================

void updateClutch() {
    if (clutchRequested && !clutchEngaged) {
        // Engaging - soft start
        if (clutchEngageStart == 0) {
            clutchEngageStart = millis();
            digitalWrite(PIN_CLUTCH, HIGH);
            digitalWrite(PIN_BRIDGE_EN, HIGH);
        }
        
        // Check if ramp complete
        if (millis() - clutchEngageStart >= CLUTCH_ENGAGE_RAMP_MS) {
            clutchEngaged = true;
            faultCode = FAULT_NONE;  // Clear watchdog fault on new engage
        }
    } else if (!clutchRequested && clutchEngaged) {
        // Disengaging - immediate
        clutchEngaged = false;
        clutchEngageStart = 0;
        digitalWrite(PIN_CLUTCH, LOW);
        digitalWrite(PIN_BRIDGE_EN, LOW);
        
        // Stop motor
        analogWrite(PIN_PWM_PORT, 0);
        analogWrite(PIN_PWM_STBD, 0);
    } else if (!clutchRequested) {
        clutchEngageStart = 0;
        digitalWrite(PIN_CLUTCH, LOW);
        digitalWrite(PIN_BRIDGE_EN, LOW);
    }
}

// ============================================================================
// Serial Communication
// ============================================================================

void outputStatus() {
    // Format: $STS,<target>,<actual>,<clutch>,<voltage>,<current>,<fault>*XX
    char payload[64];
    snprintf(payload, sizeof(payload), 
             "STS,%.3f,%.3f,%d,%.1f,%.1f,%d",
             targetAngle, actualAngle, clutchEngaged ? 1 : 0,
             supplyVoltage, motorCurrent, faultCode);
    
    uint8_t checksum = computeChecksum(payload);
    
    Serial.print('$');
    Serial.print(payload);
    Serial.print('*');
    if (checksum < 16) Serial.print('0');
    Serial.println(checksum, HEX);
}

void processCommands() {
    static char cmdBuffer[64];
    static uint8_t cmdIndex = 0;
    
    while (Serial.available()) {
        char c = Serial.read();
        
        if (c == '$') {
            cmdIndex = 0;
        } else if (c == '\n' || c == '\r') {
            cmdBuffer[cmdIndex] = '\0';
            
            // Verify checksum
            char *asterisk = strchr(cmdBuffer, '*');
            if (asterisk) {
                *asterisk = '\0';
                uint8_t expectedChecksum = strtol(asterisk + 1, NULL, 16);
                uint8_t actualChecksum = computeChecksum(cmdBuffer);
                
                if (expectedChecksum == actualChecksum) {
                    // Parse command
                    if (strncmp(cmdBuffer, "RUD,", 4) == 0) {
                        // $RUD,<target>,<engage>*XX
                        float target;
                        int engage;
                        if (sscanf(cmdBuffer + 4, "%f,%d", &target, &engage) == 2) {
                            targetAngle = constrain(target, -1.0f, 1.0f);
                            clutchRequested = (engage == 1);
                            lastCommandTime = millis();
                            if (faultCode == FAULT_WATCHDOG) {
                                faultCode = FAULT_NONE;
                            }
                        }
                    } else if (strncmp(cmdBuffer, "CFG,TIMEOUT,", 12) == 0) {
                        // $CFG,TIMEOUT,<ms>*XX
                        int timeout;
                        if (sscanf(cmdBuffer + 12, "%d", &timeout) == 1) {
                            watchdogTimeout = constrain(timeout, 500, 10000);
                            calibration.watchdog_timeout = watchdogTimeout;
                            saveCalibration();
                        }
                    } else if (strcmp(cmdBuffer, "CAL,CENTER") == 0) {
                        calibration.adc_center = analogRead(PIN_ADC_RUDDER);
                        saveCalibration();
                    } else if (strcmp(cmdBuffer, "CAL,PORT") == 0) {
                        calibration.adc_port_limit = analogRead(PIN_ADC_RUDDER);
                        saveCalibration();
                    } else if (strcmp(cmdBuffer, "CAL,STBD") == 0) {
                        calibration.adc_stbd_limit = analogRead(PIN_ADC_RUDDER);
                        saveCalibration();
                    }
                }
            }
            
            cmdIndex = 0;
        } else if (cmdIndex < sizeof(cmdBuffer) - 1) {
            cmdBuffer[cmdIndex++] = c;
        }
    }
}

uint8_t computeChecksum(const char *str) {
    uint8_t checksum = 0;
    while (*str) {
        checksum ^= *str++;
    }
    return checksum;
}

// ============================================================================
// Calibration
// ============================================================================

void loadCalibration() {
    EEPROM.get(EEPROM_ADDR_CALIBRATION, calibration);
    
    if (calibration.magic != EEPROM_MAGIC) {
        // No valid calibration - use defaults
        calibration.adc_center = ADC_RUDDER_CENTER;
        calibration.adc_port_limit = ADC_RUDDER_PORT_LIMIT;
        calibration.adc_stbd_limit = ADC_RUDDER_STBD_LIMIT;
        
#if defined(CURRENT_SENSE_BTS7960)
        calibration.current_scale = BTS7960_CURRENT_SCALE;
#else
        calibration.current_scale = ADC_CURRENT_SCALE;
#endif
        calibration.voltage_scale = ADC_VOLTAGE_SCALE;
        calibration.watchdog_timeout = WATCHDOG_TIMEOUT_MS;
        calibration.magic = EEPROM_MAGIC;
    }
    
    watchdogTimeout = calibration.watchdog_timeout;
}

void saveCalibration() {
    calibration.magic = EEPROM_MAGIC;
    EEPROM.put(EEPROM_ADDR_CALIBRATION, calibration);
}
