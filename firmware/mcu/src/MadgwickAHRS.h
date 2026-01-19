/**
 * Madgwick AHRS Filter
 * ====================
 *
 * Implementation of Sebastian Madgwick's AHRS algorithm.
 * Fuses accelerometer, gyroscope, and magnetometer data
 * to produce stable orientation estimates.
 *
 * Optimized for ATtiny3226 with limited resources.
 *
 * Based on: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 */

#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

#include <Arduino.h>
#include <math.h>

class Madgwick {
public:
  Madgwick()
      : beta(0.1f), sampleFreq(100.0f), q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
        invSampleFreq(1.0f / 100.0f) {}

  /**
   * Initialize filter with sample frequency
   * @param freq Sample frequency in Hz
   */
  void begin(float freq) {
    sampleFreq = freq;
    invSampleFreq = 1.0f / freq;
  }

  /**
   * Set filter gain (beta)
   * Higher values = faster convergence but more noise
   * Lower values = smoother but slower response
   * @param b Beta value (typically 0.01 to 0.5)
   */
  void setBeta(float b) { beta = b; }

  /**
   * Update with 9-DoF data (accelerometer, gyroscope, magnetometer)
   *
   * @param gx, gy, gz Gyroscope in degrees/second
   * @param ax, ay, az Accelerometer (any unit, will be normalized)
   * @param mx, my, mz Magnetometer (any unit, will be normalized)
   */
  void update(float gx, float gy, float gz, float ax, float ay, float az,
              float mx, float my, float mz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz;
    float _4bx, _4bz, _2q0, _2q1, _2q2, _2q3;
    float _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3;
    float q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Convert gyro to radians/sec
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

      // Normalise accelerometer measurement
      recipNorm = invSqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      // Normalise magnetometer measurement
      recipNorm = invSqrt(mx * mx + my * my + mz * mz);
      mx *= recipNorm;
      my *= recipNorm;
      mz *= recipNorm;

      // Auxiliary variables to avoid repeated arithmetic
      _2q0mx = 2.0f * q0 * mx;
      _2q0my = 2.0f * q0 * my;
      _2q0mz = 2.0f * q0 * mz;
      _2q1mx = 2.0f * q1 * mx;
      _2q0 = 2.0f * q0;
      _2q1 = 2.0f * q1;
      _2q2 = 2.0f * q2;
      _2q3 = 2.0f * q3;
      _2q0q2 = 2.0f * q0 * q2;
      _2q2q3 = 2.0f * q2 * q3;
      q0q0 = q0 * q0;
      q0q1 = q0 * q1;
      q0q2 = q0 * q2;
      q0q3 = q0 * q3;
      q1q1 = q1 * q1;
      q1q2 = q1 * q2;
      q1q3 = q1 * q3;
      q2q2 = q2 * q2;
      q2q3 = q2 * q3;
      q3q3 = q3 * q3;

      // Reference direction of Earth's magnetic field
      hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 +
           _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
      hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 +
           my * q2q2 + _2q2 * mz * q3 - my * q3q3;
      _2bx = sqrtf(hx * hx + hy * hy);
      _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 +
             _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
      _4bx = 2.0f * _2bx;
      _4bz = 2.0f * _2bz;

      // Gradient decent algorithm corrective step
      s0 =
          -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) +
          _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
          _2bz * q2 *
              (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
          (-_2bx * q3 + _2bz * q1) *
              (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
          _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

      s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) +
           _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
           4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
           _2bz * q3 *
               (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
           (_2bx * q2 + _2bz * q0) *
               (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
           (_2bx * q3 - _4bz * q1) *
               (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

      s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) +
           _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
           4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
           (-_4bx * q2 - _2bz * q0) *
               (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
           (_2bx * q1 + _2bz * q3) *
               (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
           (_2bx * q0 - _4bz * q2) *
               (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

      s3 =
          _2q1 * (2.0f * q1q3 - _2q0q2 - ax) +
          _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
          (-_4bx * q3 + _2bz * q1) *
              (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
          (-_2bx * q0 + _2bz * q2) *
              (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
          _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

      // Normalise step magnitude
      recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      // Apply feedback step
      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Compute angles
    computeAngles();
  }

  /**
   * Update with 6-DoF data (accelerometer and gyroscope only)
   * Use this if magnetometer is not available
   */
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    // Convert gyro to radians/sec
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

      // Normalise accelerometer measurement
      recipNorm = invSqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      // Auxiliary variables
      _2q0 = 2.0f * q0;
      _2q1 = 2.0f * q1;
      _2q2 = 2.0f * q2;
      _2q3 = 2.0f * q3;
      _4q0 = 4.0f * q0;
      _4q1 = 4.0f * q1;
      _4q2 = 4.0f * q2;
      _8q1 = 8.0f * q1;
      _8q2 = 8.0f * q2;
      q0q0 = q0 * q0;
      q1q1 = q1 * q1;
      q2q2 = q2 * q2;
      q3q3 = q3 * q3;

      // Gradient decent algorithm corrective step
      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 +
           _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
           _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

      // Normalise step magnitude
      recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      // Apply feedback step
      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    computeAngles();
  }

  // Get Euler angles (in degrees)
  float getRoll() { return roll; }
  float getPitch() { return pitch; }
  float getYaw() { return yaw; }

  // Get quaternion
  float getQ0() { return q0; }
  float getQ1() { return q1; }
  float getQ2() { return q2; }
  float getQ3() { return q3; }

private:
  // Use Arduino's DEG_TO_RAD and RAD_TO_DEG macros if available
  // Otherwise define our own conversion constants
  #ifndef DEG_TO_RAD
  static constexpr float _DEG_TO_RAD = 0.0174532925f;
  #define DEG_TO_RAD _DEG_TO_RAD
  #endif
  #ifndef RAD_TO_DEG
  static constexpr float _RAD_TO_DEG = 57.2957795f;
  #define RAD_TO_DEG _RAD_TO_DEG
  #endif

  float beta;          // Algorithm gain
  float sampleFreq;    // Sample frequency in Hz
  float invSampleFreq; // Inverse of sample frequency

  // Quaternion of sensor frame relative to auxiliary frame
  float q0, q1, q2, q3;

  // Euler angles in degrees
  float roll, pitch, yaw;

  /**
   * Fast inverse square root approximation
   */
  float invSqrt(float x) {
    // Use standard library for reliability on ATtiny
    return 1.0f / sqrtf(x);
  }

  /**
   * Compute Euler angles from quaternion
   */
  void computeAngles() {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
      pitch = copysignf(90.0f, sinp); // Use 90 degrees if out of range
    } else {
      pitch = asinf(sinp) * RAD_TO_DEG;
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;

    // Normalize yaw to 0-360
    if (yaw < 0) {
      yaw += 360.0f;
    }
  }
};

#endif // MADGWICK_AHRS_H
