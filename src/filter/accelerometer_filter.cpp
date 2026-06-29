#include "filter/accelerometer_filter.h"

#include <cmath>

static Quaternion accelToQuaternion(const IMUSample& s)
{
    float ax = s.ax;
    float ay = s.ay;
    float az = s.az;

    float roll  = std::atan2(ay, az);
    float pitch = std::atan2(-ax,
                             std::sqrt(ay * ay + az * az));

    Quaternion qRoll;
    qRoll.w = std::cos(roll * 0.5f);
    qRoll.x = std::sin(roll * 0.5f);
    qRoll.y = 0.0f;
    qRoll.z = 0.0f;

    Quaternion qPitch;
    qPitch.w = std::cos(pitch * 0.5f);
    qPitch.x = 0.0f;
    qPitch.y = std::sin(pitch * 0.5f);
    qPitch.z = 0.0f;

    Quaternion q = qPitch * qRoll;

    q.normalize();

    return q;
}

void AccelerometerFilter::reset()
{
    m_orientation = Quaternion();
}

void AccelerometerFilter::update(
    const IMUSample& sample,
    float)
{
    m_orientation =
        accelToQuaternion(sample);
}

const Quaternion&
AccelerometerFilter::orientation() const
{
    return m_orientation;
}