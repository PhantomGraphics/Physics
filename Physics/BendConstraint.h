#pragma once

#include "IConstraint.h"

namespace Phantom {
namespace Physics {

// i0,i1 が共有エッジ、i2,i3 が各三角形の反対頂点
// C = n̂0·n̂1 - restCos  (二面角の cos 差分を制約化)
struct BendConstraint : IConstraint {
    int   i0, i1, i2, i3;
    float restCos;    // cos(rest dihedral angle)

    void project(SoftParticleSoA& particles, float alphaHat) override;
};

} // namespace Physics
} // namespace Phantom
