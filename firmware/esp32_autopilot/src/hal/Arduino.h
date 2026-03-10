// Mock Arduino.h for HAL-level native simulation
// Provides Arduino-compatible types and functions without real hardware.
// Does NOT define ARDUINO macro — NMEA2000 library uses its non-Arduino paths.

#ifndef HAL_ARDUINO_H
#define HAL_ARDUINO_H

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

// Arduino type aliases
typedef uint8_t byte;
typedef bool boolean;

#ifndef PI
#define PI M_PI
#endif
#define DEG_TO_RAD (PI / 180.0)
#define RAD_TO_DEG (180.0 / PI)

// Pin modes
#define INPUT     0
#define OUTPUT    1
#define INPUT_PULLUP 2

// Digital levels
#define LOW  0
#define HIGH 1

// ADC constants
#define ADC_UNIT_1    0
#define ADC_ATTEN_DB_11 3
#define ADC_WIDTH_BIT_12 3

// Timing — implemented in sim_main.cpp
extern "C" uint32_t millis();
void delay(uint32_t ms);

// GPIO stubs — implemented in arduino_stubs.cpp
void pinMode(int pin, int mode);
void digitalWrite(int pin, int val);
int digitalRead(int pin);
uint32_t analogRead(int pin);

// LEDC PWM stubs (ESP32 Arduino 2.x API used by actuator.cpp)
void ledcSetup(int channel, int freq, int resolution);
void ledcAttachPin(int pin, int channel);
void ledcWrite(int channel, int duty);

// constrain macro
#ifndef constrain
#define constrain(val, lo, hi) ((val) < (lo) ? (lo) : ((val) > (hi) ? (hi) : (val)))
#endif

// min/max
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

// F() macro for string literals (no-op on native)
#ifndef F
#define F(str) str
#endif

// HEX for print formatting
#define HEX 16
#define DEC 10

// Serial mock — printf-based
class HardwareSerial {
public:
    void begin(int) {}
    void print(const char* s) { printf("%s", s); }
    void print(int v, int base = DEC) {
        if (base == HEX) printf("%x", v);
        else printf("%d", v);
    }
    void print(unsigned int v, int base = DEC) {
        if (base == HEX) printf("%x", v);
        else printf("%u", v);
    }
    void print(float v) { printf("%.2f", v); }
    void print(double v) { printf("%.2f", v); }
    void println(const char* s = "") { printf("%s\n", s); }
    void println(int v, int base = DEC) { print(v, base); printf("\n"); }
    void println(unsigned int v, int base = DEC) { print(v, base); printf("\n"); }
    void println(float v) { printf("%.2f\n", v); }
    void println(double v) { printf("%.2f\n", v); }
    void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
    }
};

extern HardwareSerial Serial;

#endif // HAL_ARDUINO_H
