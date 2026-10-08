#pragma once
#include "CGLib/Math/Vector3d.h"

namespace Phantom::Physics {

// A sphere is the degenerate capsule a == b. Coordinates are in world metres.
struct HairCollider {
    Math::Vector3df a{0.f};
    Math::Vector3df b{0.f};
    float radius = 0.1f;
    float friction = 0.3f;
};

struct HairContact {
    float strandFraction = 0.f;
    float colliderFraction = 0.f;
    float penetration = 0.f;
    Math::Vector3df normal{1.f, 0.f, 0.f};
};

bool validHairCollider(const HairCollider& collider);
// Closest segment/segment distance, including points, parallel axes and centres.
HairContact sampleHairContact(const Math::Vector3df& p, const Math::Vector3df& q,
                              const HairCollider& collider, float thickness);
// Non-rebounding normal response and Coulomb tangent friction in boundary space.
Math::Vector3df hairContactVelocity(const Math::Vector3df& velocity,
    const Math::Vector3df& normal, const Math::Vector3df& boundaryVelocity,
    float correction, float friction, float dt);

} // namespace Phantom::Physics
