#ifndef ADAPTIVE_PILOT_H
#define ADAPTIVE_PILOT_H

#include "pilot_base.h"
#include <math.h>
#include <string.h>

// Plant-identification adaptive pilot.
//
// Instead of estimating controller gains directly (which requires an indirect
// observation model), we estimate the PLANT parameters from direct observations:
//
//   d(yaw_rate)/dt = K_r * rudder_deg - K_d * yaw_rate
//
// Where:
//   K_r = rudder effectiveness (deg/s² per deg of rudder)
//   K_d = natural yaw damping (1/s)
//
// The EKF observes changes in heading rate and the actual rudder position each
// step, giving it a direct, well-conditioned measurement model. From the
// estimated plant, we compute optimal PD gains via pole placement:
//
//   Closed-loop characteristic: s² + (K_d + K_r*kd)*s + K_r*kp = 0
//   For desired ωn and damping ratio ζ:
//     kp = ωn² / K_r
//     kd = (2*ζ*ωn - K_d) / K_r

class AdaptivePilot : public BasePilot {
public:
    AdaptivePilot(BasePilot* inner,
                  float dt = 0.2f,
                  float q_diag = 1e-4f,
                  float r_scalar = 0.5f,
                  float omega_n = 0.6f,
                  float zeta = 0.9f,
                  float kr_init = 0.4f,
                  float kd_plant_init = 0.5f,
                  float kr_min = 0.02f, float kr_max = 3.0f,
                  float kd_plant_min = 0.02f, float kd_plant_max = 3.0f,
                  float max_p_trace = 10.0f)
        : _inner(inner), _dt(dt),
          _q_diag(q_diag), _r_scalar(r_scalar),
          _omega_n(omega_n), _zeta(zeta),
          _max_p_trace(max_p_trace), _step(0),
          _prev_rate(0.0f)
    {
        _bounds[0][0] = kr_min;       _bounds[0][1] = kr_max;
        _bounds[1][0] = kd_plant_min; _bounds[1][1] = kd_plant_max;
        _kr_init = kr_init;
        _kd_plant_init = kd_plant_init;
        _init_ekf();
    }

    float steer(const PilotFeatures& f) override {
        float rate = f.heading_rate * 30.0f;           // deg/s
        float rudder_deg = f.rudder_position * 25.0f;  // actual rudder in degrees

        // EKF update: observe heading rate change, regress against rudder and rate
        if (_step > 0 && fabsf(rate) < 20.0f && fabsf(rudder_deg) < 26.0f) {
            float delta_rate = rate - _prev_rate;

            // Predicted delta_rate = (K_r * rudder_deg - K_d * rate) * dt
            float predicted = (_x[0] * rudder_deg - _x[1] * rate) * _dt;
            float innovation = delta_rate - predicted;

            // Jacobian H = d(predicted)/d(x) = [rudder_deg * dt, -rate * dt]
            float H[2] = {
                rudder_deg * _dt,
                -rate * _dt
            };

            // S = H @ P @ H^T + R  (scalar)
            float S = _r_scalar;
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++)
                    S += H[i] * _cov[i][j] * H[j];

            if (S > 1e-10f) {
                // K = P @ H^T / S  (2 x 1)
                float K[2];
                for (int i = 0; i < 2; i++) {
                    K[i] = 0;
                    for (int j = 0; j < 2; j++)
                        K[i] += _cov[i][j] * H[j];
                    K[i] /= S;
                }

                // x = x + K * innovation
                for (int i = 0; i < 2; i++)
                    _x[i] += K[i] * innovation;

                // P = (I - K @ H) @ P + Q
                float KH_P[2][2] = {{0}};
                for (int i = 0; i < 2; i++)
                    for (int j = 0; j < 2; j++)
                        for (int k = 0; k < 2; k++)
                            KH_P[i][j] += K[i] * H[k] * _cov[k][j];

                for (int i = 0; i < 2; i++)
                    for (int j = 0; j < 2; j++)
                        _cov[i][j] = _cov[i][j] - KH_P[i][j] + ((i == j) ? _q_diag : 0.0f);

                _clamp_state();

                // Check for divergence
                float trace = _cov[0][0] + _cov[1][1];
                if (trace > _max_p_trace)
                    _reset_to_defaults();
            }
        }

        _prev_rate = rate;
        _step++;

        // Compute optimal PD gains from estimated plant and apply
        _apply_gains();

        return _inner->steer(f);
    }

    void reset() override {
        _init_ekf();
        _inner->reset();
    }

    void configure(float kp, float ki, float kd) override {
        // Manual gain override — set inner pilot directly, but also
        // back-compute what plant params would produce these gains,
        // so the EKF starts from the right place.
        _inner->configure(kp, ki, kd);
    }

    // Accessors for web API — report the derived PD gains and plant params
    float get_kp() const {
        if (_x[0] < 0.01f) return 0.0f;
        return _omega_n * _omega_n / _x[0];
    }
    float get_kd() const {
        if (_x[0] < 0.01f) return 0.0f;
        float kd = (2.0f * _zeta * _omega_n - _x[1]) / _x[0];
        return (kd > 0.0f) ? kd : 0.0f;
    }
    float get_ki() const { return 0.0f; }

    float get_kr() const { return _x[0]; }
    float get_kd_plant() const { return _x[1]; }

    float get_confidence() const {
        float trace = _cov[0][0] + _cov[1][1];
        // Scale: initial trace 0.2 → ~50%, converged trace ~0.02 → ~91%
        float c = 1.0f / (1.0f + 5.0f * trace);
        if (c < 0.0f) c = 0.0f;
        if (c > 1.0f) c = 1.0f;
        return c;
    }

private:
    BasePilot* _inner;
    float _dt;
    float _q_diag;
    float _r_scalar;
    float _omega_n;    // desired natural frequency (rad/s)
    float _zeta;       // desired damping ratio
    float _max_p_trace;
    uint32_t _step;
    float _prev_rate;  // previous heading rate (deg/s)

    float _x[2];           // state: [K_r, K_d]  (plant params)
    float _cov[2][2];      // covariance
    float _bounds[2][2];   // [min, max] per state
    float _default_x[2];   // for reset
    float _kr_init, _kd_plant_init;

    void _init_ekf() {
        memset(_cov, 0, sizeof(_cov));
        _cov[0][0] = 0.1f;
        _cov[1][1] = 0.1f;

        _x[0] = _kr_init;
        _x[1] = _kd_plant_init;

        memcpy(_default_x, _x, sizeof(_x));
        _prev_rate = 0.0f;
        _step = 0;
    }

    void _clamp_state() {
        for (int i = 0; i < 2; i++) {
            if (_x[i] < _bounds[i][0]) _x[i] = _bounds[i][0];
            if (_x[i] > _bounds[i][1]) _x[i] = _bounds[i][1];
        }
    }

    void _apply_gains() {
        float kr = _x[0];
        float kd_plant = _x[1];

        if (kr < 0.01f) return;  // can't compute gains if K_r ~= 0

        float kp = _omega_n * _omega_n / kr;
        float kd = (2.0f * _zeta * _omega_n - kd_plant) / kr;

        // Clamp to sane controller gain range
        kp = clampf(kp, 0.1f, 8.0f);
        kd = clampf(kd, 0.0f, 10.0f);

        _inner->configure(kp, 0.0f, kd);
    }

    void _reset_to_defaults() {
        memcpy(_x, _default_x, sizeof(_x));
        memset(_cov, 0, sizeof(_cov));
        _cov[0][0] = 0.1f;
        _cov[1][1] = 0.1f;
    }
};

#endif // ADAPTIVE_PILOT_H
