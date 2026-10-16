#pragma once

#include "IConstraint.h"

namespace Phantom {
namespace Physics {

// 四面体 (a,b,c,d) の体積保存制約
// C = V - restVolume  where V = dot(b-a, cross(c-a, d-a)) / 6
struct VolumeConstraint : IConstraint {
    int   a, b, c, d;
    float restVolume;

    void project(SoftParticleSoA& particles, float alphaHat) override;
};

} // namespace Physics
} // namespace Phantom
