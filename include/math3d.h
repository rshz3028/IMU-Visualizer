/*
    Matrix Convention

    Column-major (OpenGL)

    Memory layout:

    | 0  4  8 12 |
    | 1  5  9 13 |
    | 2  6 10 14 |
    | 3  7 11 15 |

    Column vectors

    gl_Position = Projection * View * Model * vec4(position, 1.0)
*/

#pragma once

#include <cmath>
#include "quaternion.h"

struct Vec3
{
    float x;
    float y;
    float z;

    Vec3();

    Vec3(float x, float y, float z);

    Vec3 operator+(const Vec3& rhs) const;
    Vec3 operator-(const Vec3& rhs) const;
    Vec3 operator*(float scalar) const;
    Vec3 operator/(float scalar) const;

    Vec3& operator+=(const Vec3& rhs);
    Vec3& operator-=(const Vec3& rhs);

    float length() const;
    Vec3 normalized() const;

    static float dot(const Vec3& a, const Vec3& b);
    static Vec3 cross(const Vec3& a, const Vec3& b);
};


struct Mat4
{
    float m[16];

    Mat4();

    Mat4 operator*(const Mat4& rhs) const;

    static Mat4 identity();

    static Mat4 translation(const Vec3& position);

    static Mat4 scale(const Vec3& scale);

    static Mat4 rotationX(float radians);

    static Mat4 rotationY(float radians);

    static Mat4 rotationZ(float radians);

    static Mat4 rotation(const Quaternion& q);

    static Mat4 perspective(
        float fov,
        float aspect,
        float nearPlane,
        float farPlane
    );

    static Mat4 lookAt(
        const Vec3& eye,
        const Vec3& target,
        const Vec3& up
    );
};