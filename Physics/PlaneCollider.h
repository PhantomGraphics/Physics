#pragma once

#include "ISoftCollider.h"

namespace Phantom {
namespace Physics {

struct PlaneCollider : ISoftCollider {
    Math::Vector3df normal = { 0.f, 1.f, 0.f };
    float           offset = 0.f;

    void resolve(SoftParticleSoA& particles, size_t i) const override;
};

} // namespace Physics
} // namespace Phantom
