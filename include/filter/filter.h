#pragma once

#include "serial_imu.h"
#include "quaternion.h"

class Filter
{
public:

    virtual ~Filter() = default;

    virtual void reset() = 0;

    virtual void update(
        const IMUSample& sample,
        float dt
    ) = 0;

    virtual const Quaternion&
    orientation() const = 0;
};