// Mock Wire.h — TwoWire class backed by a simulated BNO055 register map.
// The sim_environment writes sensor values into the register map;
// the real Adafruit_BNO055 library reads them via this mock I2C bus.

#ifndef HAL_WIRE_H
#define HAL_WIRE_H

#include <stdint.h>
#include <string.h>

class TwoWire {
public:
    TwoWire() {
        memset(registers, 0, sizeof(registers));
        // BNO055 chip ID
        registers[0x00] = 0xA0;
        // Fully calibrated
        registers[0x35] = 0xFF;
        // System status: fusion running
        registers[0x39] = 0x05;
        // Self-test passed
        registers[0x36] = 0x0F;
        // SW revision
        registers[0x04] = 0x11;
        registers[0x05] = 0x03;
    }

    // Arduino ESP32 Wire.begin(sda, scl)
    void begin(int sda, int scl) { (void)sda; (void)scl; }
    // Standard Wire.begin()
    void begin() {}
    void end() {}
    void setClock(uint32_t) {}

    void beginTransmission(uint8_t addr) {
        (void)addr;
        tx_len = 0;
    }

    size_t write(uint8_t val) {
        if (tx_len < sizeof(tx_buf)) {
            tx_buf[tx_len++] = val;
        }
        return 1;
    }

    size_t write(const uint8_t* buf, size_t len) {
        for (size_t i = 0; i < len; i++) write(buf[i]);
        return len;
    }

    uint8_t endTransmission(bool stop = true) {
        (void)stop;
        if (tx_len >= 1) {
            current_reg = tx_buf[0];
            // If more than 1 byte written, it's a register write
            for (size_t i = 1; i < tx_len; i++) {
                registers[current_reg] = tx_buf[i];
                current_reg++;
            }
        }
        return 0; // success
    }

    uint8_t requestFrom(uint8_t addr, uint8_t qty, uint8_t stop = 1) {
        (void)addr; (void)stop;
        rx_len = qty;
        rx_pos = 0;
        return qty;
    }

    int available() {
        return rx_len - rx_pos;
    }

    int read() {
        if (rx_pos < rx_len) {
            uint8_t val = registers[current_reg];
            current_reg++;
            rx_pos++;
            return val;
        }
        return 0;
    }

    // --- Sim interface: called by SimEnvironment to update sensor data ---

    void set_register(uint8_t addr, uint8_t val) {
        registers[addr] = val;
    }

    void set_register_16(uint8_t addr, int16_t val) {
        registers[addr] = (uint8_t)(val & 0xFF);         // LSB
        registers[addr + 1] = (uint8_t)((val >> 8) & 0xFF); // MSB
    }

    uint8_t get_register(uint8_t addr) const {
        return registers[addr];
    }

private:
    uint8_t registers[256];
    uint8_t current_reg = 0;

    // TX buffer (write phase)
    uint8_t tx_buf[32];
    size_t tx_len = 0;

    // RX state (read phase)
    uint8_t rx_len = 0;
    uint8_t rx_pos = 0;
};

extern TwoWire Wire;

#endif // HAL_WIRE_H
