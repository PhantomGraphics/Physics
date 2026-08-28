#pragma once

#include "ISoftCollider.h"

namespace Phantom {
namespace Physics {

struct SphereCollider : ISoftCollider {
    Math::Vector3df center = { 0.f, 0.f, 0.f };
    float           radius = 0.3f;

    void resolve(SoftParticleSoA& particles, size_t i) const override;
};

} // namespace Physics
} // namespace Phantom
