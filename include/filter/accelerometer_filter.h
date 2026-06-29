#pragma once

#include "filter.h"

class AccelerometerFilter : public Filter
{
public:

    void reset() override;

    void update(
        const IMUSample& sample,
        float dt
    ) override;

    const Quaternion&
    orientation() const override;

private:

    Quaternion m_orientation;
};