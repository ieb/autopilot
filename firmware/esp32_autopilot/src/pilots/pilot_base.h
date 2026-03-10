#ifndef PILOT_BASE_H
#define PILOT_BASE_H

#include "app_state.h"
#include <math.h>

#ifdef NATIVE_BUILD
// Under SIM_BUILD, millis() is provided by sim_main.cpp / hal_sim_main.cpp
// with real timing. Under plain NATIVE_BUILD (unit tests), return 0.
// Use extern "C" linkage to match N2kDef.h's declaration.
#ifndef SIM_BUILD
inline uint32_t millis() { return 0; }
#else
extern "C" uint32_t millis();
#endif
#else
#include <Arduino.h>
#endif

// Portable clamp — avoids Arduino constrain macro ordering issues
inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

class BasePilot {
public:
    virtual ~BasePilot() = default;

    // Compute rudder command from features. Returns normalized -1..+1 (+-25 deg).
    virtual float steer(const PilotFeatures& f) = 0;

    // Reset internal state (integrator, rate limiter, etc.)
    virtual void reset() = 0;

    // Override gains at runtime
    virtual void configure(float kp, float ki, float kd) {
        (void)kp; (void)ki; (void)kd;
    }
};

#endif // PILOT_BASE_H
