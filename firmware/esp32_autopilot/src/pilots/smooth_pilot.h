#ifndef SMOOTH_PILOT_H
#define SMOOTH_PILOT_H

#include "pilot_base.h"
#include <math.h>

class SmoothPilot : public BasePilot {
public:
    SmoothPilot(BasePilot* inner, float dt = 0.2f,
                float base_rate_limit = 3.0f, float speed_ref = 6.0f,
                float speed_gain_min = 0.4f, float speed_gain_max = 1.0f,
                float turn_rate_threshold = 3.0f, float turn_rate_gain = 0.5f)
        : _inner(inner), _dt(dt),
          _base_rate_limit(base_rate_limit), _speed_ref(speed_ref),
          _speed_gain_min(speed_gain_min), _speed_gain_max(speed_gain_max),
          _turn_rate_threshold(turn_rate_threshold), _turn_rate_gain(turn_rate_gain),
          _prev_output(0.0f), _time_at_error(0.0f) {}

    float steer(const PilotFeatures& f) override {
        float heading_error_deg = fabsf(f.heading_error * 90.0f);

        // Track time at error for urgency ramp
        if (heading_error_deg > 1.0f) {
            _time_at_error += _dt;
        } else {
            _time_at_error = fmaxf(0.0f, _time_at_error - _dt);
        }

        // 1. Speed-dependent gain scaling
        float stw = f.stw * 25.0f;  // boat speed (knots)
        float speed_factor = _speed_ref / fmaxf(stw, 1.0f);
        speed_factor = clampf(speed_factor, _speed_gain_min, _speed_gain_max);
        float target = _inner->steer(f) * speed_factor;

        // 2. Rate limiting with urgency ramp and error scaling
        float urgency = fminf(1.0f, _time_at_error / 30.0f);
        float error_scale = fminf(3.0f, 1.0f + heading_error_deg / 15.0f);
        float effective_rate = _base_rate_limit * error_scale * (1.0f + 2.0f * urgency);
        float max_change = (effective_rate / 25.0f) * _dt;

        // 3. Turn-rate awareness (only when increasing rudder magnitude)
        float turn_rate = fabsf(f.heading_rate * 30.0f);
        if (turn_rate > _turn_rate_threshold) {
            float excess = (turn_rate - _turn_rate_threshold) / 3.0f;
            max_change *= 1.0f / (1.0f + _turn_rate_gain * excess);
        }

        // Asymmetric rate limit: full when building rudder,
        // 3x faster when reducing or reversing
        float delta = target - _prev_output;
        bool reducing = fabsf(target) < fabsf(_prev_output);
        bool reversing = (target * _prev_output < 0) && fabsf(_prev_output) > 0.02f;
        float effective_max = (reducing || reversing) ? max_change * 3.0f : max_change;
        delta = clampf(delta, -effective_max, effective_max);
        _prev_output += delta;

        return clampf(_prev_output, -1.0f, 1.0f);
    }

    void reset() override {
        _prev_output = 0.0f;
        _time_at_error = 0.0f;
        _inner->reset();
    }

    void configure(float kp, float ki, float kd) override {
        _inner->configure(kp, ki, kd);
    }

private:
    BasePilot* _inner;
    float _dt;
    float _base_rate_limit;
    float _speed_ref;
    float _speed_gain_min;
    float _speed_gain_max;
    float _turn_rate_threshold;
    float _turn_rate_gain;
    float _prev_output;
    float _time_at_error;
};

#endif // SMOOTH_PILOT_H
