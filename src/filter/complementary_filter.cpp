#include "complementary_filter.h"

#include <cmath>

static Quaternion multiply(
    const Quaternion& a,
    const Quaternion& b
)
{
    Quaternion q;

    q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    q.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    q.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    q.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;

    return q;
}

static Quaternion accelTiltQuaternion(
    const IMUSample& s
)
{
    float roll =
        std::atan2(s.ay, s.az);

    float pitch =
        std::atan2(
            -s.ax,
            std::sqrt(s.ay * s.ay + s.az * s.az)
        );

    Quaternion qRoll;
    qRoll.w = std::cos(roll * 0.5f);
    qRoll.x = std::sin(roll * 0.5f);

    Quaternion qPitch;
    qPitch.w = std::cos(pitch * 0.5f);
    qPitch.y = std::sin(pitch * 0.5f);

    Quaternion q =
        multiply(qPitch, qRoll);

    q.normalize();

    return q;
}

static Quaternion lerpQuat(
    const Quaternion& a,
    const Quaternion& b,
    float t
)
{
    Quaternion q;

    q.w = a.w * (1.0f - t) + b.w * t;
    q.x = a.x * (1.0f - t) + b.x * t;
    q.y = a.y * (1.0f - t) + b.y * t;
    q.z = a.z * (1.0f - t) + b.z * t;

    q.normalize();

    return q;
}

void ComplementaryFilter::reset()
{
    m_orientation = Quaternion();
    m_initialized = false;
}

void ComplementaryFilter::update(
    const IMUSample& sample,
    float dt
)
{
    if (dt <= 0.0f)
        return;

    Quaternion accelQ =
        accelTiltQuaternion(sample);

    if (!m_initialized)
    {
        m_orientation = accelQ;
        m_initialized = true;
        return;
    }

    float gx = sample.gx * 3.14159265f / 180.0f;
    float gy = sample.gy * 3.14159265f / 180.0f;
    float gz = sample.gz * 3.14159265f / 180.0f;

    Quaternion omega;
    omega.w = 0.0f;
    omega.x = gx;
    omega.y = gy;
    omega.z = gz;

    Quaternion qDot =
        multiply(m_orientation, omega);

    m_orientation.w += 0.5f * qDot.w * dt;
    m_orientation.x += 0.5f * qDot.x * dt;
    m_orientation.y += 0.5f * qDot.y * dt;
    m_orientation.z += 0.5f * qDot.z * dt;

    m_orientation.normalize();

    m_orientation =
        lerpQuat(
            m_orientation,
            accelQ,
            m_accelBlend
        );
}

const Quaternion& ComplementaryFilter::orientation() const
{
    return m_orientation;
}