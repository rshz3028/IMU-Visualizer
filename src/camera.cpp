#include "camera.h"

#include <cmath>

Camera::Camera()
{
    m_position = Vec3(0.0f, 2.0f, 5.0f);

    m_yaw = -90.0f;
    m_pitch = 0.0f;

    m_moveSpeed = 1.8f;
    m_mouseSensitivity = 0.15f;
}

void Camera::update(Input& input, float deltaTime)
{
    if (input.rightMouseDown())
    {
        input.lockCursor(true);

        m_yaw += input.mouseDeltaX() * m_mouseSensitivity;
        m_pitch -= input.mouseDeltaY() * m_mouseSensitivity;

        if (m_pitch > 89.0f)
            m_pitch = 89.0f;

        if (m_pitch < -89.0f)
            m_pitch = -89.0f;
    }
    else
    {
        input.lockCursor(false);
    }

    float yawRad = m_yaw * 3.14159265f / 180.0f;
    float pitchRad = m_pitch * 3.14159265f / 180.0f;

    Vec3 forward;

    forward.x = std::cos(pitchRad) * std::cos(yawRad);
    forward.y = std::sin(pitchRad);
    forward.z = std::cos(pitchRad) * std::sin(yawRad);

    forward = forward.normalized();

    Vec3 right = Vec3::cross(forward, Vec3(0.0f, 1.0f, 0.0f)).normalized();

    if (input.keyDown(SDL_SCANCODE_KP_1))
        frontView();

    if (input.keyDown(SDL_SCANCODE_KP_3))
        rightView();

    if (input.keyDown(SDL_SCANCODE_KP_7))
        topView();

    if (input.keyDown(SDL_SCANCODE_W))
        m_position += forward * (m_moveSpeed * deltaTime);

    if (input.keyDown(SDL_SCANCODE_S))
        m_position -= forward * (m_moveSpeed * deltaTime);

    if (input.keyDown(SDL_SCANCODE_D))
        m_position += right * (m_moveSpeed * deltaTime);

    if (input.keyDown(SDL_SCANCODE_A))
        m_position -= right * (m_moveSpeed * deltaTime);

    if (input.keyDown(SDL_SCANCODE_SPACE))
        m_position.y += m_moveSpeed * deltaTime;

    if (input.keyDown(SDL_SCANCODE_LCTRL) ||
        input.keyDown(SDL_SCANCODE_RCTRL))
    {
        m_position.y -= m_moveSpeed * deltaTime;
    }
}

Mat4 Camera::getViewMatrix() const
{
    float yawRad = m_yaw * 3.14159265f / 180.0f;
    float pitchRad = m_pitch * 3.14159265f / 180.0f;

    Vec3 forward;

    forward.x = std::cos(pitchRad) * std::cos(yawRad);
    forward.y = std::sin(pitchRad);
    forward.z = std::cos(pitchRad) * std::sin(yawRad);

    forward = forward.normalized();

    return Mat4::lookAt(
        m_position,
        m_position + forward.normalized(),
        Vec3(0.0f,1.0f,0.0f)
    );
}

Mat4 Camera::getProjectionMatrix(float aspect) const
{
    return Mat4::perspective(
        45.0f * 3.14159265f / 180.0f,
        aspect,
        0.1f,
        1000.0f
    );
}

void Camera::frontView()
{
    m_yaw = -90.0f;
    m_pitch = 0.0f;
}

void Camera::backView()
{
    m_yaw = 90.0f;
    m_pitch = 0.0f;
}

void Camera::rightView()
{
    m_yaw = 0.0f;
    m_pitch = 0.0f;
}

void Camera::leftView()
{
    m_yaw = 180.0f;
    m_pitch = 0.0f;
}

void Camera::topView()
{
    m_yaw = -90.0f;
    m_pitch = 89.9f;
}

void Camera::bottomView()
{
    m_yaw = -90.0f;
    m_pitch = -89.9f;
}

Vec3 Camera::getPosition() const
{
    return m_position;
}