/**
 * Actuator Controller Configuration
 * ==================================
 *
 * Pin definitions and configuration for ATtiny3224 actuator controller.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// Current Sensing Configuration (compile-time option)
// ============================================================================

// Select ONE via build flags or uncomment here:
//   -DCURRENT_SENSE_ADC       Option A: Direct ADC with shunt + op-amp
//   -DCURRENT_SENSE_INA219    Option B: INA219 I2C module (recommended)
//   -DCURRENT_SENSE_BTS7960   Option C: BTS7960 IS pins (RC filter on ADC)
//
// If neither is defined, default to BTS7960
#if !defined(CURRENT_SENSE_ADC) && !defined(CURRENT_SENSE_INA219) && !defined(CURRENT_SENSE_BTS7960)
#define CURRENT_SENSE_BTS7960
#endif

// INA219 Configuration (defaults, can be overridden via build flags)
#ifndef INA219_I2C_ADDRESS
#define INA219_I2C_ADDRESS    0x40    // Default address (A0=GND, A1=GND)
#endif

#ifndef INA219_SHUNT_OHMS
#define INA219_SHUNT_OHMS     0.01f   // External shunt resistance
#endif

#ifndef INA219_MAX_CURRENT
#define INA219_MAX_CURRENT    20.0f   // Max expected current (Amps)
#endif

// ============================================================================
// Pin Definitions (ATtiny3224)
// ============================================================================

// Analog inputs
#define PIN_ADC_RUDDER    PIN_PA2   // Rudder potentiometer

#ifdef CURRENT_SENSE_ADC
// Option A: Direct ADC sensing
#define PIN_ADC_CURRENT   PIN_PB1   // Motor current sense (shunt resistor)
#define PIN_ADC_VOLTAGE   PIN_PB0   // Supply voltage (divider)
#else
// Option B: I2C for INA219 (PA2/PA3 become I2C) - NOT RECOMMENDED for 3224 with this layout
#define PIN_I2C_SDA       PIN_PA2   // I2C data (conflicts with Rudder ADC on 3224 netlist)
#define PIN_I2C_SCL       PIN_PA3   // I2C clock
// Note: INA219 also reads bus voltage, so no separate ADC needed
#elif defined(CURRENT_SENSE_BTS7960)
// Option C: BTS7960 IS pins
#define PIN_ADC_CURRENT   PIN_PB1   // Motor current sense (BTS7960 IS)
#define PIN_ADC_VOLTAGE   PIN_PB0   // Supply voltage (divider)
#endif

// PWM outputs (TCA0)
#define PIN_PWM_PORT      PIN_PA5   // Port drive (TCA0 WO5)
#define PIN_PWM_STBD      PIN_PA4   // Starboard drive (TCA0 WO4)

// Digital outputs
#define PIN_CLUTCH        PIN_PA1   // Electromagnetic clutch (N-FET gate)
#define PIN_BRIDGE_EN     PIN_PA6   // H-Bridge enable (BEnable)
#define PIN_LED           PIN_PA3   // Status LED

// Serial (hardware UART)
#define PIN_TX            PIN_PB2   // Serial TX to Pi RX
#define PIN_RX            PIN_PB3   // Serial RX from Pi TX

// ============================================================================
// Timing Configuration
// ============================================================================

#ifndef CONTROL_LOOP_RATE_HZ
#define CONTROL_LOOP_RATE_HZ     50   // Position control loop rate
#endif

#ifndef SERIAL_OUTPUT_RATE_HZ
#define SERIAL_OUTPUT_RATE_HZ    20   // Status output rate
#endif

#ifndef WATCHDOG_TIMEOUT_MS
#define WATCHDOG_TIMEOUT_MS      2000 // Clutch disengage if no command
#endif

// Derived timing
#define CONTROL_LOOP_INTERVAL_US (1000000UL / CONTROL_LOOP_RATE_HZ)
#define SERIAL_OUTPUT_INTERVAL_US (1000000UL / SERIAL_OUTPUT_RATE_HZ)

// ============================================================================
// Position Limits
// ============================================================================

#ifndef POSITION_LIMIT_DEG
#define POSITION_LIMIT_DEG       28.0f  // Software limit (mechanical is 30°)
#endif

#define POSITION_LIMIT_NORMALIZED (POSITION_LIMIT_DEG / 30.0f)

// ============================================================================
// Current Limits
// ============================================================================

#ifndef CURRENT_LIMIT_SOFT
#define CURRENT_LIMIT_SOFT       12.0f  // Reduce PWM above this (Amps)
#endif

#ifndef CURRENT_LIMIT_HARD
#define CURRENT_LIMIT_HARD       15.0f  // Stop motor above this (Amps)
#endif

// Stall detection
#define STALL_CURRENT_THRESHOLD  10.0f  // Amps - high current
#define STALL_VELOCITY_THRESHOLD 0.5f   // deg/s - low movement
#define STALL_TIMEOUT_MS         500    // Duration before fault

// ============================================================================
// Control Parameters
// ============================================================================

// P-controller gain
#define POSITION_KP              0.8f

// Deadband (don't move if error smaller)
#define POSITION_DEADBAND_DEG    0.5f
#define POSITION_DEADBAND_NORMALIZED (POSITION_DEADBAND_DEG / 30.0f)

// PWM limits
#define PWM_MIN                  26    // ~10% - minimum to overcome friction
#define PWM_MAX                  255   // 100%
#define PWM_FREQUENCY            10000 // 10kHz for silent operation

// ============================================================================
// ADC Calibration Defaults
// ============================================================================

// Rudder potentiometer (10-bit ADC, 0-1023)
#define ADC_RUDDER_CENTER        512   // ADC value at 0°
#define ADC_RUDDER_PORT_LIMIT    102   // ADC value at -30°
#define ADC_RUDDER_STBD_LIMIT    922   // ADC value at +30°

// Current sense (assuming 0.01 ohm shunt, 50x gain op-amp)
// V = I * 0.01 * 50 = I * 0.5, so 1A = 0.5V = 102 ADC counts @ 5V ref
#define ADC_CURRENT_SCALE        0.0098f  // Amps per ADC count

// BTS7960 Current Sense Scale
// I_is = I_load / 8500. With 1k resistor, V_is = I_load / 8.5.
// 1A -> 0.1176V -> (0.1176 / 5.0) * 1023 = 24 counts.
// Scale = 1 / 24 = 0.0416 Amps per ADC count
#define BTS7960_CURRENT_SCALE    0.0416f

// Voltage sense (assuming 10k/2.2k divider -> 1/5.54)
// 12V -> 2.16V at ADC, 2.16V = 442 counts @ 5V ref
// Scale = 12 / 442 = 0.0271 Volts per ADC count
#define ADC_VOLTAGE_SCALE        0.0271f

// ============================================================================
// EEPROM Layout
// ============================================================================

#define EEPROM_ADDR_CALIBRATION  0
#define EEPROM_MAGIC             0xACDC

// ============================================================================
// Fault Codes
// ============================================================================

#define FAULT_NONE               0
#define FAULT_OVERCURRENT        1
#define FAULT_STALL              2
#define FAULT_POSITION_LIMIT     3
#define FAULT_SENSOR             4
#define FAULT_WATCHDOG           5

// ============================================================================
// Clutch Configuration
// ============================================================================

// Soft-start ramp time (ms)
#define CLUTCH_ENGAGE_RAMP_MS    200

#endif // CONFIG_H
