#ifndef SIM_HAL_H
#define SIM_HAL_H

/**
 * Simulation HAL — portable replacements for Arduino primitives.
 * Only compiled when both NATIVE_BUILD and SIM_BUILD are defined.
 */

#include <stdint.h>
#include <stdio.h>
#include <chrono>

// ============================================================================
// millis() — real elapsed time via std::chrono
// ============================================================================

namespace sim_hal {
    inline auto& start_time() {
        static auto t0 = std::chrono::steady_clock::now();
        return t0;
    }
}

// Override the stub millis() from pilot_base.h at link time
// pilot_base.h defines millis() as inline returning 0 under NATIVE_BUILD,
// but we provide a real implementation here for the sim.
// Since sim_hal.h is included first in sim files, this takes precedence.
// For pilot_manager.cpp which includes pilot_base.h, we need millis()
// to be defined before it — handled via include order in sim_main.cpp.

inline uint32_t sim_millis() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - sim_hal::start_time());
    return static_cast<uint32_t>(elapsed.count());
}

// ============================================================================
// Serial — printf-based stub
// ============================================================================

struct SimSerial {
    void begin(int) {}
    void println(const char* s) { printf("%s\n", s); }
    void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
    }
};

#endif // SIM_HAL_H
