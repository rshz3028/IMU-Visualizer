#pragma once

#include "filter.h"

// Joint error-state Kalman filter for two rigidly co-mounted IMUs.
//
// References:
//   J. Sola, "Quaternion kinematics for the error-state Kalman filter",
//   arXiv:1711.02508 (local / right-hand attitude error, reset Jacobian).
//   Libero & Klein, unified multi-IMU EKF (joint bias states, gyro
//   difference as a pseudo-measurement).
//
// Conventions (same as MEKFFilter):
//   q          body -> world, world z up
//   gravity    specific force at rest measures +z in world, i.e. [0,0,1]
//   true q     q_t = q (x) Exp(dtheta), dtheta in BODY frame
//   biases     body frame, rad/s; bias error db = b_true - b_est
//
// Nominal state : q, b1, b2
// Error state   : x = [ dtheta(3), db1(3), db2(3) ]          (9)
//
// Inputs are body-frame samples, i.e. the mount rotation has already been
// applied by the pipeline (gyro rad/s, accel m/s^2, mag uT).
//
// Measurements (processed as sequential scalar updates on one
// linearisation point, then a single injection + reset per step):
//   1. gyro difference  z = g1 - g2  = b1 - b2 + n1 - n2       H = [0, I, -I]
//   2. gravity direction from averaged accelerometers           (gated)
//   3. zero-rate (ZARU) z = g_i = b_i + n_i when stationary     H = [0, I, 0] / [0, 0, I]
//   4. magnetometer yaw against a horizontal reference captured at init
//      (gated by field norm, disabled by default)
//
// Covariance is held in double internally for numerical robustness: the
// difference measurement makes b1 and b2 almost perfectly correlated
// (diff known well, sum poorly), which loses PSD quickly in float.
class DualESKFFilter : public Filter
{
public:
    static constexpr int N = 9;

    struct Diagnostics
    {
        bool diffUsed  = false;
        bool accelUsed = false;
        bool magUsed   = false;
        bool zaruUsed  = false;

        float accelNormError = 0.0f;   // | |a| - g |, m/s^2
        float rate           = 0.0f;   // |fused omega|, rad/s
    };

public:
    DualESKFFilter();

    void reset() override;

    // Dual-IMU step. s1/s2 are body-frame (mount-rotated) samples taken at the
    // same instant; dt in seconds.
    void update(
        const IMUSample& s1,
        const IMUSample& s2,
        float dt
    );

    // Single-input Filter interface: runs the same filter as a single-IMU
    // ESKF. The sample feeds both slots, the gyro-difference update is
    // skipped (it would be identically zero) and ZARU constrains only the
    // fused bias w1*b1 + w2*b2. Do not mix calls to both update() overloads
    // without reset().
    void update(
        const IMUSample& sample,
        float dt
    ) override;

    const Quaternion& orientation() const override;

    // ZARU. While stationary, every update() applies the zero-rate update
    // (and skips the gyro-difference update, which would double-count the
    // same gyro samples).
    void setStationary(bool stationary);
    bool isStationary() const;

    // One-shot ZARU using the samples from the last update() call.
    void updateZeroRate();

    // ---- Noise parameters (continuous-time densities) ----
    // Gyro white noise (angle random walk), rad/s/sqrt(Hz) == rad/sqrt(s).
    // 0.30 deg/sqrt(h) = 8.73e-5 rad/sqrt(s).
    void setGyroNoise(float imu1, float imu2);
    // Gyro bias random walk, rad/s/sqrt(s).
    void setGyroBiasNoise(float imu1, float imu2);
    // Extra white noise floor on the gyro-difference residual, rad/s (1 sigma,
    // per sample). Covers scale-factor / misalignment / timing mismatch.
    void setGyroDiffNoiseFloor(float stdRadPerSec);
    // Additional gyro-difference std proportional to |omega| (e.g. 0.005 for
    // 0.5 % relative scale-factor / misalignment mismatch).
    void setGyroDiffRateInflation(float relative);
    // Gravity direction noise, 1 sigma, on the unit vector (~rad).
    void setAccelNoise(float stdDir);
    // Magnetometer yaw noise, 1 sigma, rad.
    void setMagNoise(float stdRad);
    // Extra ZARU noise floor (vibration, imperfect stop detection), rad/s 1 sigma.
    void setZaruNoise(float stdRadPerSec);
    // Initial standard deviations used by reset().
    void setInitialBiasStd(float stdRadPerSec);
    void setInitialTiltStd(float stdRad);

    // ---- Fusion weights / gating / enable flags ----
    // Fused rate omega = w1 (g1 - b1) + w2 (g2 - b2); normalised to sum 1.
    void setWeights(float w1, float w2);
    // Gravity update is skipped when | |a| - g | > normTol (m/s^2) or
    // |omega| > rateTol (rad/s) (centripetal acceleration does not change
    // |a| much, so the rate gate matters for vehicles).
    void setAccelGate(float normTol, float rateTol);
    // Magnetometer update skipped when | |m| - |m0| | > relTol * |m0|.
    void setMagGate(float relTol);
    void setGravity(float g);
    // When enabled (default), the gravity update uses a projected gain that
    // leaves heading and the vertical components of b1/b2 untouched. With a
    // nearly level vehicle those states are only weakly observable through
    // gravity, and unmodelled (time-correlated) vehicle accelerations then
    // drive them confidently wrong. Heading / vertical bias are then informed
    // only by ZARU, the gyro difference and the magnetometer.
    void setAccelHeadingDecoupling(bool enabled);

    void setGyroDiffEnabled(bool enabled);
    void setAccelEnabled(bool enabled);
    void setMagEnabled(bool enabled);

    // ---- Outputs ----
    bool initialized() const;

    Vec3 bias1() const;
    Vec3 bias2() const;
    // Fused bias w1*b1 + w2*b2 (what actually corrupts the attitude).
    Vec3 fusedBias() const;

    // Attitude error covariance (body-frame dtheta), rad^2.
    void attitudeCovariance(float out[3][3]) const;
    // Full 9x9 error covariance [dtheta, db1, db2].
    void errorCovariance(float out[N][N]) const;

    const Diagnostics& diagnostics() const;

private:
    // Nominal state
    Quaternion m_orientation;
    double     m_b1[3] = { 0.0, 0.0, 0.0 };
    double     m_b2[3] = { 0.0, 0.0, 0.0 };

    // Error covariance and per-step accumulated error mean
    double m_P[N][N];
    double m_dx[N];

    // Tunables
    float m_gyroNoise1     = 2.6e-4f;   // ~0.9 deg/sqrt(h), ICM-20948 class
    float m_gyroNoise2     = 2.6e-4f;
    float m_biasNoise1     = 1.0e-5f;
    float m_biasNoise2     = 1.0e-5f;
    float m_diffFloor      = 2.0e-4f;
    float m_diffRateInfl   = 5.0e-3f;
    float m_accelNoise     = 5.0e-2f;
    float m_magNoise       = 5.0e-2f;
    float m_zaruNoise      = 1.0e-3f;
    float m_initBiasStd    = 1.0e-2f;
    float m_initTiltStd    = 3.5e-2f;

    float m_w1 = 0.5f;
    float m_w2 = 0.5f;

    float m_accelNormGate  = 0.5f;
    float m_accelRateGate  = 0.35f;
    float m_magRelGate     = 0.15f;
    float m_gravity        = 9.80665f;

    bool m_diffEnabled  = true;
    bool m_accelEnabled = true;
    bool m_magEnabled   = false;
    bool m_accelDecouple = true;

    bool m_stationary   = false;
    bool m_initialized  = false;
    bool m_singleInput  = false;

    // Magnetic reference (world frame, horizontal unit vector) and norm
    bool  m_magRefValid = false;
    float m_magRef[2]   = { 1.0f, 0.0f };
    float m_magRefNorm  = 0.0f;

    // Last inputs (for updateZeroRate)
    IMUSample m_last1 {};
    IMUSample m_last2 {};
    float     m_lastDt  = 0.0f;
    bool      m_hasLast = false;

    Diagnostics m_diag;

private:
    void resetCovariance();

    void initialize(
        const IMUSample& s1,
        const IMUSample& s2
    );

    void predict(
        const IMUSample& s1,
        const IMUSample& s2,
        float dt,
        double omega[3]
    );

    // Sequential scalar update on the accumulated error mean m_dx.
    // r is the residual z - h(nominal); H is the 1x9 Jacobian row.
    // upBody (optional): body-frame vertical; if given, the gain is projected
    // so that the update does not change heading or vertical bias components.
    bool scalarUpdate(
        const double H[N],
        double r,
        double R,
        const double* upBody = nullptr
    );

    void updateGyroDifference(
        const IMUSample& s1,
        const IMUSample& s2,
        double dt,
        const double omega[3]
    );

    void updateZeroRateInternal(
        const IMUSample& s1,
        const IMUSample& s2,
        double dt
    );

    void updateGravity(
        const IMUSample& s1,
        const IMUSample& s2,
        const double omega[3]
    );

    void updateMagnetometer(
        const IMUSample& s1,
        const IMUSample& s2
    );

    void injectAndReset();

    void conditionCovariance();

    void step(
        const IMUSample& s1,
        const IMUSample& s2,
        float dt
    );
};
