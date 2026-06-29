#include "math3d.h"

Vec3::Vec3()
    : x(0.0f),
      y(0.0f),
      z(0.0f)
{
}

Vec3::Vec3(float x, float y, float z)
    : x(x),
      y(y),
      z(z)
{
}

Vec3 Vec3::operator+(const Vec3& rhs) const
{
    return Vec3(
        x + rhs.x,
        y + rhs.y,
        z + rhs.z
    );
}

Vec3 Vec3::operator-(const Vec3& rhs) const
{
    return Vec3(
        x - rhs.x,
        y - rhs.y,
        z - rhs.z
    );
}

Vec3 Vec3::operator*(float scalar) const
{
    return Vec3(
        x * scalar,
        y * scalar,
        z * scalar
    );
}

Vec3 Vec3::operator/(float scalar) const
{
    return Vec3(
        x / scalar,
        y / scalar,
        z / scalar
    );
}

Vec3& Vec3::operator+=(const Vec3& rhs)
{
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;

    return *this;
}

Vec3& Vec3::operator-=(const Vec3& rhs)
{
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;

    return *this;
}

float Vec3::length() const
{
    return std::sqrt(
        x * x +
        y * y +
        z * z
    );
}

Vec3 Vec3::normalized() const
{
    float len = length();

    if (len == 0.0f)
        return Vec3();

    return (*this) / len;
}

float Vec3::dot(
    const Vec3& a,
    const Vec3& b
)
{
    return
        a.x * b.x +
        a.y * b.y +
        a.z * b.z;
}

Vec3 Vec3::cross(
    const Vec3& a,
    const Vec3& b
)
{
    return Vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

Mat4::Mat4()
{
    for (int i = 0; i < 16; i++)
        m[i] = 0.0f;
}

Mat4 Mat4::identity()
{
    Mat4 mat;

    mat.m[0] = 1.0f;
    mat.m[5] = 1.0f;
    mat.m[10] = 1.0f;
    mat.m[15] = 1.0f;

    return mat;
}

Mat4 Mat4::translation(const Vec3& position)
{
    Mat4 result = identity();

    result.m[12] = position.x;
    result.m[13] = position.y;
    result.m[14] = position.z;

    return result;
}  

Mat4 Mat4::scale(const Vec3& scale)
{
    Mat4 result = identity();

    result.m[0] = scale.x;
    result.m[5] = scale.y;
    result.m[10] = scale.z;

    return result;
}

Mat4 Mat4::rotationX(float radians)
{
    Mat4 result = identity();

    float c = std::cos(radians);
    float s = std::sin(radians);

    result.m[5] = c;
    result.m[6] = -s;
    result.m[9] = s;
    result.m[10] = c;

    return result;
}

Mat4 Mat4::rotationY(float radians)
{
    Mat4 result = identity();

    float c = std::cos(radians);
    float s = std::sin(radians);

    result.m[0] = c;
    result.m[2] = s;
    result.m[8] = -s;
    result.m[10] = c;

    return result;
}

Mat4 Mat4::rotationZ(float radians)
{
    Mat4 result = identity();

    float c = std::cos(radians);
    float s = std::sin(radians);

    result.m[0] = c;
    result.m[1] = -s;
    result.m[4] = s;
    result.m[5] = c;

    return result;
}

Mat4 Mat4::rotation(
    const Quaternion& q
)
{
    Mat4 m = identity();

    float xx = q.x * q.x;
    float yy = q.y * q.y;
    float zz = q.z * q.z;

    float xy = q.x * q.y;
    float xz = q.x * q.z;
    float yz = q.y * q.z;

    float wx = q.w * q.x;
    float wy = q.w * q.y;
    float wz = q.w * q.z;

    m.m[0] = 1.0f - 2.0f * (yy + zz);
    m.m[1] = 2.0f * (xy + wz);
    m.m[2] = 2.0f * (xz - wy);

    m.m[4] = 2.0f * (xy - wz);
    m.m[5] = 1.0f - 2.0f * (xx + zz);
    m.m[6] = 2.0f * (yz + wx);

    m.m[8] = 2.0f * (xz + wy);
    m.m[9] = 2.0f * (yz - wx);
    m.m[10] = 1.0f - 2.0f * (xx + yy);

    return m;
}

Mat4 Mat4::operator*(const Mat4& rhs) const
{
    Mat4 result;

    for (int col = 0; col < 4; ++col)
    {
        for (int row = 0; row < 4; ++row)
        {
            result.m[col * 4 + row] =
                m[0 * 4 + row] * rhs.m[col * 4 + 0] +
                m[1 * 4 + row] * rhs.m[col * 4 + 1] +
                m[2 * 4 + row] * rhs.m[col * 4 + 2] +
                m[3 * 4 + row] * rhs.m[col * 4 + 3];
        }
    }

    return result;
}

Mat4 Mat4::perspective(
    float fovRadians,
    float aspect,
    float nearPlane,
    float farPlane
)
{
    Mat4 result;

    float tanHalfFov = std::tan(fovRadians / 2.0f);

    result.m[0] = 1.0f / (aspect * tanHalfFov);
    result.m[5] = 1.0f / tanHalfFov;
    result.m[10] = -(farPlane + nearPlane) / (farPlane - nearPlane);
    result.m[11] = -1.0f;
    result.m[14] = -(2.0f * farPlane * nearPlane) / (farPlane - nearPlane);

    return result;
}

Mat4 Mat4::lookAt(
    const Vec3& eye,
    const Vec3& target,
    const Vec3& up)
{
    Vec3 f = (target - eye).normalized();
    Vec3 s = Vec3::cross(f, up).normalized();
    Vec3 u = Vec3::cross(s, f);

    Mat4 result = Mat4::identity();

    result.m[0] = s.x;
    result.m[1] = u.x;
    result.m[2] = -f.x;

    result.m[4] = s.y;
    result.m[5] = u.y;
    result.m[6] = -f.y;

    result.m[8] = s.z;
    result.m[9] = u.z;
    result.m[10] = -f.z;

    result.m[12] = -Vec3::dot(s, eye);
    result.m[13] = -Vec3::dot(u, eye);
    result.m[14] = Vec3::dot(f, eye);

    return result;
}