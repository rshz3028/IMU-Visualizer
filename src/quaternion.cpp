#include "quaternion.h"

#include <cmath>

Quaternion Quaternion::operator*(
    const Quaternion& other
) const
{
    Quaternion q;

    q.w =
        w * other.w
        - x * other.x
        - y * other.y
        - z * other.z;

    q.x =
        w * other.x
        + x * other.w
        + y * other.z
        - z * other.y;

    q.y =
        w * other.y
        - x * other.z
        + y * other.w
        + z * other.x;

    q.z =
        w * other.z
        + x * other.y
        - y * other.x
        + z * other.w;

    return q;
}

void Quaternion::normalize()
{
    float len =
        std::sqrt(
            w * w +
            x * x +
            y * y +
            z * z
        );

    if (len == 0.0f)
        return;

    w /= len;
    x /= len;
    y /= len;
    z /= len;
}