#pragma once

#include "CollisionPair.h"
#include "RigidBody.h"

namespace Phantom {
namespace Physics {

class NarrowPhase {
public:
    static bool detect(RigidBody& a, RigidBody& b, ContactManifold& out);

private:
    static bool sphereSphere(RigidBody& a, RigidBody& b, ContactManifold& out);
    static bool spherePlane (RigidBody& a, RigidBody& b, ContactManifold& out);
    static bool sphereBox   (RigidBody& a, RigidBody& b, ContactManifold& out);
    static bool boxBox      (RigidBody& a, RigidBody& b, ContactManifold& out);
    static bool boxPlane    (RigidBody& a, RigidBody& b, ContactManifold& out);
    // The capsule is always body A; detect() flips the normals when it is body B.
    static bool capsulePlane  (RigidBody& a, RigidBody& b, ContactManifold& out);
    static bool capsuleSphere (RigidBody& a, RigidBody& b, ContactManifold& out);
    static bool capsuleBox    (RigidBody& a, RigidBody& b, ContactManifold& out);
    static bool capsuleCapsule(RigidBody& a, RigidBody& b, ContactManifold& out);
};

} // namespace Physics
} // namespace Phantom
