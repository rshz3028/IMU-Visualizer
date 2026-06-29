#pragma once

#include "filter.h"

class MEKFFilter : public Filter
{
public:
    MEKFFilter();

    void reset() override;

    void update(
        const IMUSample& sample,
        float dt
    ) override;

    const Quaternion& orientation() const override;

    void setGyroNoise(float value);
    void setGyroBiasNoise(float value);
    void setAccelNoise(float value);
    void setMagNoise(float value);

private:
    Quaternion m_orientation;

    // Gyro bias estimate, rad/s
    float m_biasX = 0.0f;
    float m_biasY = 0.0f;
    float m_biasZ = 0.0f;

    // 6x6 error covariance:
    // [ attitude error xyz, gyro bias error xyz ]
    float m_P[6][6];

    // Tunables
    float m_gyroNoise     = 1.0e-4f;
    float m_biasNoise     = 1.0e-6f;
    float m_accelNoise    = 2.5e-2f;
    float m_magNoise      = 8.0e-2f;

    bool m_initialized = false;

private:
    void resetCovariance();

    void initializeFromAccel(
        float ax,
        float ay,
        float az
    );

    void predict(
        float gx,
        float gy,
        float gz,
        float dt
    );

    void updateDirection(
        float zx,
        float zy,
        float zz,
        float refX,
        float refY,
        float refZ,
        float noise
    );

    void applyErrorState(
        const float dx[6]
    );

    void normalizeQuaternion();
};
