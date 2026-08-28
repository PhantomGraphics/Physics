#include "pch.h"
#include "RigidBody.h"

#include "CGLib/Math/Quaternion.h"
#include "CGLib/ThirdParty/glm-0.9.9.8/glm/gtc/quaternion.hpp"

namespace Phantom {
namespace Physics {

void RigidBody::setMass(float m) {
    if (m <= 0.f) {
        mass        = 0.f;
        inverseMass = 0.f;
        invI_local_ = Math::Matrix3df(0.f);
    } else {
        mass        = m;
        inverseMass = 1.f / m;
        computeLocalInertia();
    }
}

void RigidBody::computeLocalInertia() {
    if (!shape || inverseMass == 0.f) {
        invI_local_ = Math::Matrix3df(0.f);
        return;
    }

    if (shape->getType() == ShapeType::Sphere) {
        auto* s = static_cast<SphereShape*>(shape);
        float r2 = s->radius * s->radius;
        float Iv = (2.f / 5.f) * mass * r2;
        invI_local_ = Math::Matrix3df(1.f / Iv);
    } else if (shape->getType() == ShapeType::Box) {
        auto* b = static_cast<BoxShape*>(shape);
        float w = b->halfExtents.x, h = b->halfExtents.y, d = b->halfExtents.z;
        float Ixx = (mass / 3.f) * (h * h + d * d);
        float Iyy = (mass / 3.f) * (w * w + d * d);
        float Izz = (mass / 3.f) * (w * w + h * h);
        invI_local_ = Math::Matrix3df(
            1.f / Ixx, 0, 0,
            0, 1.f / Iyy, 0,
            0, 0, 1.f / Izz
        );
    } else {
        invI_local_ = Math::Matrix3df(0.f);
    }
}

void RigidBody::updateInertiaTensor() {
    R_         = glm::mat3_cast(orientation);
    invI_world_ = R_ * invI_local_ * glm::transpose(R_);
}

void RigidBody::applyImpulse(const Math::Vector3df& impulse,
                              const Math::Vector3df& worldPoint) {
    if (isStatic()) return;
    linearVelocity  += inverseMass * impulse;
    Math::Vector3df r = worldPoint - position;
    angularVelocity += invI_world_ * glm::cross(r, impulse);
}

void RigidBody::integrate(float dt) {
    if (isStatic()) return;

    linearVelocity  += dt * inverseMass * forceAccum;
    angularVelocity += dt * (invI_world_ * torqueAccum);

    position += dt * linearVelocity;

    Math::Quaternion omega(0.f, angularVelocity.x, angularVelocity.y, angularVelocity.z);
    orientation = glm::normalize(orientation + (dt * 0.5f) * omega * orientation);

    forceAccum  = Math::Vector3df(0.f);
    torqueAccum = Math::Vector3df(0.f);
}

Math::Box3df RigidBody::getAABB() const {
    if (!shape) return Math::Box3df(position, position);
    return shape->getAABB(position, orientation);
}

} // namespace Physics
} // namespace Phantom
