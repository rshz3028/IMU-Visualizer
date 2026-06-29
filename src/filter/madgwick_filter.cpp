#include "madgwick_filter.h"

#include <cmath>

static bool validVector(float x, float y, float z)
{
    float n2 = x*x + y*y + z*z;
    return n2 > 1e-12f;
}

static void normalizeVector(float& x, float& y, float& z)
{
    float n = std::sqrt(x*x + y*y + z*z);

    if (n <= 0.0f)
        return;

    x /= n;
    y /= n;
    z /= n;
}

void MadgwickFilter::reset()
{
    m_orientation = Quaternion();
    m_initialized = false;
}

void MadgwickFilter::update(
    const IMUSample& sample,
    float dt
)
{
    if (dt <= 0.0f)
        return;

    // Your pipeline already provides gyro in rad/s.
    float gxRaw = sample.gx;
    float gyRaw = sample.gy;
    float gzRaw = sample.gz;

    float gx = gxRaw;
    float gy = gyRaw;
    float gz = gzRaw;
    
    float ax = sample.ax;
    float ay = sample.ay;
    float az = sample.az;

    float mx = sample.mx;
    float my = sample.my;
    float mz = sample.mz;

    if (!validVector(ax, ay, az))
        return;

    normalizeVector(ax, ay, az);

    bool useMag = validVector(mx, my, mz);

    if (useMag)
        normalizeVector(mx, my, mz);

    float q0 = m_orientation.w;
    float q1 = m_orientation.x;
    float q2 = m_orientation.y;
    float q3 = m_orientation.z;

    float qDot0 =
        0.5f * (-q1 * gx - q2 * gy - q3 * gz);

    float qDot1 =
        0.5f * ( q0 * gx + q2 * gz - q3 * gy);

    float qDot2 =
        0.5f * ( q0 * gy - q1 * gz + q3 * gx);

    float qDot3 =
        0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    float s0 = 0.0f;
    float s1 = 0.0f;
    float s2 = 0.0f;
    float s3 = 0.0f;

    if (useMag)
    {
        float _2q0mx = 2.0f * q0 * mx;
        float _2q0my = 2.0f * q0 * my;
        float _2q0mz = 2.0f * q0 * mz;
        float _2q1mx = 2.0f * q1 * mx;

        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;

        float q0q0 = q0 * q0;
        float q0q1 = q0 * q1;
        float q0q2 = q0 * q2;
        float q0q3 = q0 * q3;
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q3q3 = q3 * q3;

        float hx =
            mx * q0q0
            - _2q0my * q3
            + _2q0mz * q2
            + mx * q1q1
            + _2q1 * my * q2
            + _2q1 * mz * q3
            - mx * q2q2
            - mx * q3q3;

        float hy =
            _2q0mx * q3
            + my * q0q0
            - _2q0mz * q1
            + _2q1mx * q2
            - my * q1q1
            + my * q2q2
            + _2q2 * mz * q3
            - my * q3q3;

        float _2bx = std::sqrt(hx * hx + hy * hy);

        float _2bz =
            -_2q0mx * q2
            + _2q0my * q1
            + mz * q0q0
            + _2q1mx * q3
            - mz * q1q1
            + _2q2 * my * q3
            - mz * q2q2
            + mz * q3q3;

        float _4bx = 2.0f * _2bx;
        float _4bz = 2.0f * _2bz;

        s0 =
            -_2q2 * (2.0f * q1q3 - 2.0f * q0q2 - ax)
            + _2q1 * (2.0f * q0q1 + 2.0f * q2q3 - ay)
            - _2bz * q2 *
                (_2bx * (0.5f - q2q2 - q3q3)
                + _2bz * (q1q3 - q0q2) - mx)
            + (-_2bx * q3 + _2bz * q1) *
                (_2bx * (q1q2 - q0q3)
                + _2bz * (q0q1 + q2q3) - my)
            + _2bx * q2 *
                (_2bx * (q0q2 + q1q3)
                + _2bz * (0.5f - q1q1 - q2q2) - mz);

        s1 =
            _2q3 * (2.0f * q1q3 - 2.0f * q0q2 - ax)
            + _2q0 * (2.0f * q0q1 + 2.0f * q2q3 - ay)
            - 4.0f * q1 *
                (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
            + _2bz * q3 *
                (_2bx * (0.5f - q2q2 - q3q3)
                + _2bz * (q1q3 - q0q2) - mx)
            + (_2bx * q2 + _2bz * q0) *
                (_2bx * (q1q2 - q0q3)
                + _2bz * (q0q1 + q2q3) - my)
            + (_2bx * q3 - _4bz * q1) *
                (_2bx * (q0q2 + q1q3)
                + _2bz * (0.5f - q1q1 - q2q2) - mz);

        s2 =
            -_2q0 * (2.0f * q1q3 - 2.0f * q0q2 - ax)
            + _2q3 * (2.0f * q0q1 + 2.0f * q2q3 - ay)
            - 4.0f * q2 *
                (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
            + (-_4bx * q2 - _2bz * q0) *
                (_2bx * (0.5f - q2q2 - q3q3)
                + _2bz * (q1q3 - q0q2) - mx)
            + (_2bx * q1 + _2bz * q3) *
                (_2bx * (q1q2 - q0q3)
                + _2bz * (q0q1 + q2q3) - my)
            + (_2bx * q0 - _4bz * q2) *
                (_2bx * (q0q2 + q1q3)
                + _2bz * (0.5f - q1q1 - q2q2) - mz);

        s3 =
            _2q1 * (2.0f * q1q3 - 2.0f * q0q2 - ax)
            + _2q2 * (2.0f * q0q1 + 2.0f * q2q3 - ay)
            + (-_4bx * q3 + _2bz * q1) *
                (_2bx * (0.5f - q2q2 - q3q3)
                + _2bz * (q1q3 - q0q2) - mx)
            + (-_2bx * q0 + _2bz * q2) *
                (_2bx * (q1q2 - q0q3)
                + _2bz * (q0q1 + q2q3) - my)
            + _2bx * q1 *
                (_2bx * (q0q2 + q1q3)
                + _2bz * (0.5f - q1q1 - q2q2) - mz);
    }
    else
    {
        // 6-DoF fallback: gyro + accelerometer only.
        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1;
        float _8q2 = 8.0f * q2;

        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;

        s0 =
            _4q0 * q2q2
            + _2q2 * ax
            + _4q0 * q1q1
            - _2q1 * ay;

        s1 =
            _4q1 * q3q3
            - _2q3 * ax
            + 4.0f * q0q0 * q1
            - _2q0 * ay
            - _4q1
            + _8q1 * q1q1
            + _8q1 * q2q2
            + _4q1 * az;

        s2 =
            4.0f * q0q0 * q2
            + _2q0 * ax
            + _4q2 * q3q3
            - _2q3 * ay
            - _4q2
            + _8q2 * q1q1
            + _8q2 * q2q2
            + _4q2 * az;

        s3 =
            4.0f * q1q1 * q3
            - _2q1 * ax
            + 4.0f * q2q2 * q3
            - _2q2 * ay;
    }

    float norm =
        std::sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);

    if (norm > 1e-12f)
    {
        s0 /= norm;
        s1 /= norm;
        s2 /= norm;
        s3 /= norm;

        qDot0 -= m_beta * s0;
        qDot1 -= m_beta * s1;
        qDot2 -= m_beta * s2;
        qDot3 -= m_beta * s3;
    }

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    norm =
        std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    if (norm <= 0.0f)
        return;

    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;

    m_orientation.w = q0;
    m_orientation.x = q1;
    m_orientation.y = q2;
    m_orientation.z = q3;

    m_initialized = true;
}

const Quaternion& MadgwickFilter::orientation() const
{
    return m_orientation;
}

void MadgwickFilter::setBeta(float beta)
{
    m_beta = beta;
}