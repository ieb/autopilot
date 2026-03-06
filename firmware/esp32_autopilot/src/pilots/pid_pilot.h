#ifndef PID_PILOT_H
#define PID_PILOT_H

#include "pilot_base.h"

class PIDPilot : public BasePilot {
public:
    float kp;
    float ki;
    float kd;
    float max_rudder;
    float integrator_limit;  // degrees
    float dt;                // seconds between calls

    PIDPilot(float kp = 1.0f, float ki = 0.1f, float kd = 1.5f,
             float max_rudder = 1.0f, float integrator_limit = 10.0f,
             float dt = 0.2f)
        : kp(kp), ki(ki), kd(kd), max_rudder(max_rudder),
          integrator_limit(integrator_limit), dt(dt),
          _integrator(0.0f), _prev_command(0.0f) {}

    float steer(const PilotFeatures& f) override {
        float heading_error = f.heading_error * 90.0f;
        float heading_rate = f.heading_rate * 30.0f;

        // Accumulate integral with clamping
        float prev_integrator = _integrator;
        _integrator += heading_error * dt;
        _integrator = clampf(_integrator, -integrator_limit, integrator_limit);
        float actual_delta = _integrator - prev_integrator;

        float command_deg = kp * heading_error
                          + ki * _integrator
                          + kd * (-heading_rate);
        float command_norm = clampf(command_deg / 25.0f,
                                       -max_rudder, max_rudder);

        // Anti-windup: if output saturated, undo accumulation
        if (fabsf(command_norm) >= max_rudder) {
            // Check if error and integrator have same sign
            if ((heading_error > 0 && _integrator > 0) ||
                (heading_error < 0 && _integrator < 0)) {
                _integrator -= actual_delta;
            }
        }

        _prev_command = command_norm;
        return command_norm;
    }

    void reset() override {
        _integrator = 0.0f;
        _prev_command = 0.0f;
    }

    void configure(float kp_, float ki_, float kd_) override {
        kp = kp_;
        ki = ki_;
        kd = kd_;
    }

private:
    float _integrator;
    float _prev_command;
};

#endif // PID_PILOT_H
