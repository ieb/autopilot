#ifndef ADAPTIVE_PILOT_H
#define ADAPTIVE_PILOT_H

#include "pilot_base.h"
#include <math.h>
#include <string.h>

// Inline 3x3 matrix ops — no external library needed for EKF this small.

class AdaptivePilot : public BasePilot {
public:
    AdaptivePilot(BasePilot* inner, bool has_ki = false,
                  float dt = 0.2f, float q_diag = 1e-6f,
                  float r_scalar = 1.0f, float rudder_eff = 0.5f,
                  float kp_min = 0.5f, float kp_max = 4.0f,
                  float kd_min = 0.5f, float kd_max = 4.0f,
                  float ki_min = 0.0f, float ki_max = 0.5f,
                  float max_p_trace = 10.0f)
        : _inner(inner), _has_ki(has_ki), _dt(dt),
          _q_diag(q_diag), _r_scalar(r_scalar), _rudder_eff(rudder_eff),
          _max_p_trace(max_p_trace), _step(0),
          _prev_error(0.0f), _integrator_est(0.0f)
    {
        _n = _has_ki ? 3 : 2;
        _bounds[0][0] = kp_min; _bounds[0][1] = kp_max;
        if (_has_ki) {
            _bounds[1][0] = ki_min; _bounds[1][1] = ki_max;
            _bounds[2][0] = kd_min; _bounds[2][1] = kd_max;
        } else {
            _bounds[1][0] = kd_min; _bounds[1][1] = kd_max;
        }
        _init_ekf();
    }

    float steer(const PilotFeatures& f) override {
        float e = f.heading_error * 90.0f;
        float rate = f.heading_rate * 30.0f;

        // Build regressor phi
        float phi[3] = {0};
        if (_has_ki) {
            _integrator_est += e * _dt;
            phi[0] = e;
            phi[1] = _integrator_est;
            phi[2] = -rate;
        } else {
            phi[0] = e;
            phi[1] = -rate;
        }

        // EKF update (skip during large manoeuvres)
        if (_step > 0 && fabsf(e) < 45.0f && fabsf(rate) < 15.0f) {
            float innovation = e - _prev_error + rate * _dt;

            // H = -rudder_eff * dt * phi  (1 x n)
            float H[3];
            for (int i = 0; i < _n; i++)
                H[i] = -_rudder_eff * _dt * phi[i];

            // S = H @ P @ H^T + R  (scalar)
            float S = _r_scalar;
            for (int i = 0; i < _n; i++)
                for (int j = 0; j < _n; j++)
                    S += H[i] * _cov[i][j] * H[j];

            // K = P @ H^T / S  (n x 1)
            float K[3];
            for (int i = 0; i < _n; i++) {
                K[i] = 0;
                for (int j = 0; j < _n; j++)
                    K[i] += _cov[i][j] * H[j];
                K[i] /= S;
            }

            // x = x + K * innovation
            for (int i = 0; i < _n; i++)
                _x[i] += K[i] * innovation;

            // P = P - K @ H @ P + Q
            float KH_cov[3][3] = {{0}};
            for (int i = 0; i < _n; i++)
                for (int j = 0; j < _n; j++)
                    for (int k = 0; k < _n; k++)
                        KH_cov[i][j] += K[i] * H[k] * _cov[k][j];

            for (int i = 0; i < _n; i++)
                for (int j = 0; j < _n; j++)
                    _cov[i][j] = _cov[i][j] - KH_cov[i][j] + ((i == j) ? _q_diag : 0.0f);

            _clamp_gains();

            // Check for divergence
            float trace = 0;
            for (int i = 0; i < _n; i++) trace += _cov[i][i];
            if (trace > _max_p_trace)
                _reset_to_defaults();
        }

        _prev_error = e;
        _step++;

        // Apply adapted gains to inner pilot
        _apply_gains();

        return _inner->steer(f);
    }

    void reset() override {
        _init_ekf();
        _inner->reset();
    }

    void configure(float kp, float ki, float kd) override {
        _inner->configure(kp, ki, kd);
    }

    // Accessors for web API
    float get_kp() const { return _x[0]; }
    float get_kd() const { return _has_ki ? _x[2] : _x[1]; }
    float get_ki() const { return _has_ki ? _x[1] : 0.0f; }
    float get_confidence() const {
        float trace = 0;
        for (int i = 0; i < _n; i++) trace += _cov[i][i];
        return 1.0f / (1.0f + trace);
    }

private:
    BasePilot* _inner;
    bool _has_ki;
    int _n;
    float _dt;
    float _q_diag;
    float _r_scalar;
    float _rudder_eff;
    float _max_p_trace;
    uint32_t _step;
    float _prev_error;
    float _integrator_est;

    float _x[3];                  // state: [kp, kd] or [kp, ki, kd]
    float _cov[3][3];               // covariance
    float _bounds[3][2];          // [min, max] per gain
    float _default_x[3];          // for reset

    void _init_ekf() {
        // Initialize state from inner pilot's current gains
        // We store defaults for reset
        memset(_cov, 0, sizeof(_cov));
        for (int i = 0; i < _n; i++) {
            _cov[i][i] = 0.1f;
            _x[i] = (_bounds[i][0] + _bounds[i][1]) / 2.0f;
        }
        // Set to inner pilot's actual gains via default
        // (caller should set _x after construction if needed)
        memcpy(_default_x, _x, sizeof(_x));
        _prev_error = 0.0f;
        _integrator_est = 0.0f;
        _step = 0;
    }

    void _clamp_gains() {
        for (int i = 0; i < _n; i++) {
            if (_x[i] < _bounds[i][0]) _x[i] = _bounds[i][0];
            if (_x[i] > _bounds[i][1]) _x[i] = _bounds[i][1];
        }
    }

    void _apply_gains() {
        if (_has_ki)
            _inner->configure(_x[0], _x[1], _x[2]);
        else
            _inner->configure(_x[0], 0.0f, _x[1]);
    }

    void _reset_to_defaults() {
        memcpy(_x, _default_x, sizeof(_x));
        memset(_cov, 0, sizeof(_cov));
        for (int i = 0; i < _n; i++)
            _cov[i][i] = 0.1f;
    }
};

#endif // ADAPTIVE_PILOT_H
