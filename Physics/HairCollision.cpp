#include "HairCollision.h"
#include "CGLib/Space/Space/DistanceCalculator.h"
#include <algorithm>
#include <cmath>

namespace Phantom::Physics {
namespace {
bool finite(const Math::Vector3df& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
}
bool validHairCollider(const HairCollider& c) {
    return finite(c.a) && finite(c.b) && glm::length(c.a) <= 10000.f && glm::length(c.b) <= 10000.f &&
        std::isfinite(c.radius) && c.radius > 0.f && c.radius <= 100.f &&
        std::isfinite(c.friction) && c.friction >= 0.f && c.friction <= 1.f;
}
HairContact sampleHairContact(const Math::Vector3df& p, const Math::Vector3df& q,
                              const HairCollider& c, float thickness) {
    const auto closest = Space::DistanceCalculator<float>::closestSegments(p, q, c.a, c.b);
    HairContact out;
    out.strandFraction = closest.firstFraction;
    out.colliderFraction = closest.secondFraction;
    out.penetration = std::max(0.f, c.radius+thickness-closest.distance);
    out.normal = closest.normal;
    return out;
}
Math::Vector3df hairContactVelocity(const Math::Vector3df& velocity,
    const Math::Vector3df& normal, const Math::Vector3df& boundaryVelocity,
    float correction, float friction, float dt) {
    const auto relative = velocity-boundaryVelocity;
    const float vn = glm::dot(relative, normal);
    const auto tangent = relative-vn*normal;
    const float speed = glm::length(tangent);
    const float impulse = std::max(0.f, -vn) + correction/dt;
    const float scale = speed > 1.e-8f ? std::max(0.f, 1.f-friction*impulse/speed) : 0.f;
    return boundaryVelocity + std::max(0.f, vn)*normal + scale*tangent;
}
} // namespace Phantom::Physics
