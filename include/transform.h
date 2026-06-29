#pragma once

#include "math3d.h"
#include "quaternion.h"

struct Transform
{
    Vec3 position = {0.0f, 0.0f, 0.0f};
    Quaternion rotation;
    Vec3 scale    = {1.0f, 1.0f, 1.0f};

    Mat4 matrix() const;
};