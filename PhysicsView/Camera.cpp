#include "pch.h"
#include "Camera.h"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace Phantom {

void Camera::handleMouseButton(bool pressed, float x, float y)
{
    mouseDown_ = pressed;
    lastMouse_ = glm::vec2(x, y);
}

void Camera::handleMouseMove(float x, float y)
{
    if (mouseDown_) {
        const float dx = (x - lastMouse_.x) * 0.005f;
        const float dy = (y - lastMouse_.y) * 0.005f;
        yaw_ += dx;
        pitch_ = std::max(0.05f, std::min(3.09f, pitch_ + dy));
    }
    lastMouse_ = glm::vec2(x, y);
}

void Camera::handleScroll(float dy)
{
    distance_ = std::max(5.0f, distance_ - dy * 2.0f);
}

void Camera::viewXY()
{
    yaw_ = glm::half_pi<float>();
    pitch_ = glm::half_pi<float>();
    up_ = glm::vec3(0.f, 1.f, 0.f);
}

void Camera::viewYZ()
{
    yaw_ = 0.f;
    pitch_ = glm::half_pi<float>();
    up_ = glm::vec3(0.f, 0.f, 1.f);
}

void Camera::viewZX()
{
    yaw_ = 0.f;
    pitch_ = 0.f;
    up_ = glm::vec3(0.f, 0.f, 1.f);
}

void Camera::fit()
{
    yaw_ = 0.6f;
    pitch_ = 0.6f;
    distance_ = 120.f;
    up_ = glm::vec3(0.f, 1.f, 0.f);
}

glm::mat4 Camera::getViewMatrix() const
{
    const float x = distance_ * std::sin(pitch_) * std::cos(yaw_);
    const float y = distance_ * std::cos(pitch_);
    const float z = distance_ * std::sin(pitch_) * std::sin(yaw_);
    return glm::lookAt(center_ + glm::vec3(x, y, z), center_, up_);
}

} // namespace Phantom
