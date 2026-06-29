#pragma once

#include "filter.h"

class MadgwickFilter : public Filter
{
public:

    void reset() override;

    void update(
        const IMUSample& sample,
        float dt
    ) override;

    const Quaternion&
    orientation() const override;

    void setBeta(float beta);

private:

    Quaternion m_orientation;

    float m_beta = 0.03f;
    bool m_initialized = false;

};