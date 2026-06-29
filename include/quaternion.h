#pragma once

struct Quaternion
{
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quaternion operator*(
        const Quaternion& other
    ) const;

    void normalize();
};