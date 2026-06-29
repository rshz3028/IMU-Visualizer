#include "transform.h"

Mat4 Transform::matrix() const
{
    return
        Mat4::translation(position)
        *
        Mat4::rotation(rotation)
        *
        Mat4::scale(scale);
}