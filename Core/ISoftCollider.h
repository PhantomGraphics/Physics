#pragma once

#include <cstddef>
#include "SoftParticle.h"

namespace Phantom {
namespace Physics {

struct ISoftCollider {
    virtual void resolve(SoftParticleSoA& particles, size_t i) const = 0;
    virtual ~ISoftCollider() = default;
};

} // namespace Physics
} // namespace Phantom
