#pragma once

#include <glm/glm.hpp>

#include <algorithm>

namespace Phantom {

class Camera {
public:
    glm::mat4 getViewMatrix() const;
    float yaw() const { return yaw_; }
    float pitch() const { return pitch_; }
    float distance() const { return distance_; }
    void setYaw(float value) { yaw_ = value; }
    void setPitch(float value) { pitch_ = value; }
    void setDistance(float value) { distance_ = std::max(5.0f, value); }

    void viewXY();
    void viewYZ();
    void viewZX();
    void fit();

    void handleMouseButton(bool pressed, float x, float y);
    void handleMouseMove(float x, float y);
    void handleScroll(float dy);

private:
    float yaw_ = 0.6f;
    float pitch_ = 0.6f;
    float distance_ = 120.f;
    glm::vec3 center_{ 20.f, 20.f, 20.f };
    glm::vec3 up_{ 0.f, 1.f, 0.f };
    glm::vec2 lastMouse_{ 0.f, 0.f };
    bool mouseDown_ = false;
};

} // namespace Phantom
