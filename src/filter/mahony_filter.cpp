#include "filter/mahony_filter.h"

#include <cmath>

static bool validVector(float x, float y, float z)
{
    return (x*x + y*y + z*z) > 1e-12f;
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

void MahonyFilter::reset()
{
    m_orientation = Quaternion();

    m_integralX = 0.0f;
    m_integralY = 0.0f;
    m_integralZ = 0.0f;
}

void MahonyFilter::update(
    const IMUSample& sample,
    float dt
)
{
    if (dt <= 0.0f)
        return;

    // Your current pipeline already provides gyro in rad/s.
    float gx = sample.gx;
    float gy = sample.gy;
    float gz = sample.gz;

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

    float halfVx;
    float halfVy;
    float halfVz;

    float halfWx = 0.0f;
    float halfWy = 0.0f;
    float halfWz = 0.0f;

    // Estimated gravity direction.
    halfVx = q1*q3 - q0*q2;
    halfVy = q0*q1 + q2*q3;
    halfVz = q0*q0 - 0.5f + q3*q3;

    if (useMag)
    {
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
            2.0f * mx * (0.5f - q2q2 - q3q3)
            + 2.0f * my * (q1q2 - q0q3)
            + 2.0f * mz * (q1q3 + q0q2);

        float hy =
            2.0f * mx * (q1q2 + q0q3)
            + 2.0f * my * (0.5f - q1q1 - q3q3)
            + 2.0f * mz * (q2q3 - q0q1);

        float bx = std::sqrt(hx*hx + hy*hy);

        float bz =
            2.0f * mx * (q1q3 - q0q2)
            + 2.0f * my * (q2q3 + q0q1)
            + 2.0f * mz * (0.5f - q1q1 - q2q2);

        // Estimated magnetic field direction.
        halfWx =
            bx * (0.5f - q2q2 - q3q3)
            + bz * (q1q3 - q0q2);

        halfWy =
            bx * (q1q2 - q0q3)
            + bz * (q0q1 + q2q3);

        halfWz =
            bx * (q0q2 + q1q3)
            + bz * (0.5f - q1q1 - q2q2);
    }

    // Error = measured direction × estimated direction.
    float halfEx =
        (ay * halfVz - az * halfVy);

    float halfEy =
        (az * halfVx - ax * halfVz);

    float halfEz =
        (ax * halfVy - ay * halfVx);

    if (useMag)
    {
        halfEx +=
            (my * halfWz - mz * halfWy);

        halfEy +=
            (mz * halfWx - mx * halfWz);

        halfEz +=
            (mx * halfWy - my * halfWx);
    }

    if (m_ki > 0.0f)
    {
        m_integralX += m_ki * halfEx * dt;
        m_integralY += m_ki * halfEy * dt;
        m_integralZ += m_ki * halfEz * dt;

        gx += m_integralX;
        gy += m_integralY;
        gz += m_integralZ;
    }
    else
    {
        m_integralX = 0.0f;
        m_integralY = 0.0f;
        m_integralZ = 0.0f;
    }

    gx += m_kp * halfEx;
    gy += m_kp * halfEy;
    gz += m_kp * halfEz;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = q0;
    float qb = q1;
    float qc = q2;

    q0 += -qb * gx - qc * gy - q3 * gz;
    q1 +=  qa * gx + qc * gz - q3 * gy;
    q2 +=  qa * gy - qb * gz + q3 * gx;
    q3 +=  qa * gz + qb * gy - qc * gx;

    float norm =
        std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    if (norm <= 0.0f)
        return;

    m_orientation.w = q0 / norm;
    m_orientation.x = q1 / norm;
    m_orientation.y = q2 / norm;
    m_orientation.z = q3 / norm;
}

const Quaternion& MahonyFilter::orientation() const
{
    return m_orientation;
}

void MahonyFilter::setKp(float kp)
{
    m_kp = kp;
}

void MahonyFilter::setKi(float ki)
{
    m_ki = ki;
}