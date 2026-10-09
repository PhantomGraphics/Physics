#include "SphereCollider.h"

namespace Phantom {
namespace Physics {

void SphereCollider::resolve(SoftParticleSoA& particles, size_t i) const {
    Math::Vector3df diff = particles.predicted[i] - center;
    float dist = glm::length(diff);
    if (dist < radius) {
        // At the center the normal is undefined; choose a deterministic direction.
        const Math::Vector3df normal = dist > 1e-9f
            ? diff / dist : Math::Vector3df(0.f, 1.f, 0.f);
        particles.predicted[i] = center + normal * radius;
    }
}

} // namespace Physics
} // namespace Phantom
