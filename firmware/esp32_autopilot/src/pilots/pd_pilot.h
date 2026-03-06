#ifndef PD_PILOT_H
#define PD_PILOT_H

#include "pilot_base.h"

class PDPilot : public BasePilot {
public:
    float kp;
    float kd;
    float max_rudder;

    PDPilot(float kp = 1.0f, float kd = 1.5f, float max_rudder = 1.0f)
        : kp(kp), kd(kd), max_rudder(max_rudder), _prev_command(0.0f) {}

    float steer(const PilotFeatures& f) override {
        float heading_error = f.heading_error * 90.0f;  // denormalize to degrees
        float heading_rate = f.heading_rate * 30.0f;    // denormalize to deg/s

        // PD control: derivative = -heading_rate (damping opposes rotation)
        float command_deg = kp * heading_error + kd * (-heading_rate);
        float command_norm = clampf(command_deg / 25.0f, -max_rudder, max_rudder);
        _prev_command = command_norm;
        return command_norm;
    }

    void reset() override {
        _prev_command = 0.0f;
    }

    void configure(float kp_, float ki_, float kd_) override {
        (void)ki_;
        kp = kp_;
        kd = kd_;
    }

private:
    float _prev_command;
};

#endif // PD_PILOT_H
