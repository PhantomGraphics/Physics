#pragma once

#include "CGLib/Math/Vector3d.h"
#include <vector>

namespace Phantom {
namespace Physics {

class RigidBody;

struct ContactPoint {
    Math::Vector3df position;
    Math::Vector3df normal;        // push-out direction (from B toward A)
    float           penetration    = 0.f;
    float           accImpulse     = 0.f;  // accumulated normal impulse
    float           accFriction    = 0.f;  // accumulated friction impulse
    float           initialVelocity = 0.f; // pre-solver closing velocity for restitution
};

struct ContactManifold {
    RigidBody*              bodyA = nullptr;
    RigidBody*              bodyB = nullptr;
    std::vector<ContactPoint> contacts;
};

} // namespace Physics
} // namespace Phantom
