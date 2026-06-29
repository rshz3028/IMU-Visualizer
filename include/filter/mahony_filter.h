#pragma once

#include "filter.h"

class MahonyFilter : public Filter
{
public:
    void reset() override;

    void update(
        const IMUSample& sample,
        float dt
    ) override;

    const Quaternion& orientation() const override;

    void setKp(float kp);
    void setKi(float ki);

private:
    Quaternion m_orientation;

    float m_kp = 2.0f;
    float m_ki = 0.05f;

    float m_integralX = 0.0f;
    float m_integralY = 0.0f;
    float m_integralZ = 0.0f;
};