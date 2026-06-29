#pragma once

#include "serial_imu.h"
#include "quaternion.h"

class ComplementaryFilter
{
public:
    void reset();

    void update(
        const IMUSample& sample,
        float dt
    );

    const Quaternion& orientation() const;

private:
    Quaternion m_orientation;
    bool m_initialized = false;

    float m_accelBlend = 0.02f;
};