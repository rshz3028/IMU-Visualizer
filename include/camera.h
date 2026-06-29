#pragma once

#include "math3d.h"
#include "input.h"

class Camera
{
public:
    Camera();

    void update(Input& input, float dt);

    Mat4 getViewMatrix() const;
    Mat4 getProjectionMatrix(float aspect) const;

    Vec3 getPosition() const;

    void frontView();
    void backView();

    void rightView();
    void leftView();

    void topView();
    void bottomView();

private:
    Vec3 m_position;

    float m_yaw;
    float m_pitch;

    float m_moveSpeed;
    float m_mouseSensitivity;
};