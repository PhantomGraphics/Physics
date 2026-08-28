#include "SphereCollider.h"

namespace Phantom {
namespace Physics {

void SphereCollider::resolve(SoftParticleSoA& particles, size_t i) const {
    Math::Vector3df diff = particles.predicted[i] - center;
    float dist = glm::length(diff);
    if (dist < radius && dist > 1e-9f)
        particles.predicted[i] = center + diff * (radius / dist);
}

} // namespace Physics
} // namespace Phantom
