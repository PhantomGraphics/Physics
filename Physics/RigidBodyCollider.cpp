#include "RigidBodyCollider.h"
#include "Physics/Physics/RigidBody.h"
#include "Physics/Physics/ICollisionShape.h"

namespace Phantom {
namespace Physics {

void RigidBodyCollider::resolve(SoftParticleSoA& particles, size_t i) const {
    if (!body_ || !body_->shape) return;

    Math::Vector3df& predicted = particles.predicted[i];
    float dist = body_->shape->getSignedDistance(predicted, body_->position, body_->orientation);
    if (dist < 0.f) {
        Math::Vector3df n = body_->shape->getSurfaceNormal(predicted, body_->position, body_->orientation);
        predicted -= dist * n;
    }
}

} // namespace Physics
} // namespace Phantom
