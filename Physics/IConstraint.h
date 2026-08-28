#pragma once

#include "SoftParticle.h"

namespace Phantom {
namespace Physics {

struct IConstraint {
    float alpha  = 0.f;   // compliance [m/N]; 0 = 剛
    float lambda = 0.f;   // Lagrange 乗数（サブステップ冒頭で 0 リセット）

    // predicted 位置を修正し lambda を累積する
    // alphaHat = alpha / dt_sub²
    virtual void project(SoftParticleSoA& particles, float alphaHat) = 0;

    void resetLambda() { lambda = 0.f; }

    virtual ~IConstraint() = default;
};

} // namespace Physics
} // namespace Phantom
