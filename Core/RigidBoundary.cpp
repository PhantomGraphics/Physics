#include "pch.h"
#include "RigidBoundary.h"

namespace Phantom {
namespace Physics {

void RigidBoundary::sync(const RigidBody& body) {
    shape_    = body.shape;
    pos_      = body.position;
    orient_   = body.orientation;
    velocity_ = body.linearVelocity;
}

void RigidBoundary::syncKinematic(const Math::Vector3df& pos, const Math::Quaternion& orient,
                                   const Math::Vector3df& linearVel) {
    pos_      = pos;
    orient_   = orient;
    velocity_ = linearVel;
}

Math::Vector3df RigidBoundary::getBoundaryForce(const Math::Vector3df& particlePos) const {
    if (shape_ == nullptr) return Math::Vector3df(0.f, 0.f, 0.f);

    const float d = shape_->getSignedDistance(particlePos, pos_, orient_);
    if (d >= 0.f) return Math::Vector3df(0.f, 0.f, 0.f);

    const Math::Vector3df n = shape_->getSurfaceNormal(particlePos, pos_, orient_);
    return n * (-d * penaltyStiffness_);
}

} // namespace Physics
} // namespace Phantom
