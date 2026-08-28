#pragma once

#include "IConstraint.h"

namespace Phantom {
namespace Physics {

struct DistanceConstraint : IConstraint {
    int   a, b;
    float restLength;

    void project(SoftParticleSoA& particles, float alphaHat) override;
};

} // namespace Physics
} // namespace Phantom
