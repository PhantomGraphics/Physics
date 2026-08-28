#pragma once

#include <vector>
#include "SoftParticle.h"

namespace Phantom {
namespace Physics {

// ピン固定ユーティリティ: 指定粒子の inverseMass を 0 に設定する
// （ソルバー内では inverseMass=0 の粒子は自動的に固定される）
struct PinConstraint {
    std::vector<int> indices;

    void apply(SoftParticleSoA& particles) const {
        for (int i : indices)
            if (i >= 0 && i < static_cast<int>(particles.size()))
                particles.inverseMasses[i] = 0.f;
    }
};

} // namespace Physics
} // namespace Phantom
