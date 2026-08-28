#pragma once

#include <vector>
#include "CGLib/Math/Vector3d.h"

namespace Phantom {
namespace Physics {

// 1 粒子分の初期値をまとめて構築するための値型。
// 粒子の実体は SoftParticleSoA が保持するため、この型はコンテナへの
// push_back() の引数（および単体テストでの単一粒子テストダブル）としてのみ使う。
struct SoftParticle {
    Math::Vector3df position    = { 0.f, 0.f, 0.f };
    Math::Vector3df predicted   = { 0.f, 0.f, 0.f };
    Math::Vector3df velocity    = { 0.f, 0.f, 0.f };
    Math::Vector3df force       = { 0.f, 0.f, 0.f };
    float           inverseMass = 1.0f;              // 0.0f = ピン固定

    bool isPinned() const { return inverseMass == 0.f; }
};

// SoA (Structure of Arrays) 粒子コンテナ。SoftMesh::particles の実体。
// 属性ごとに連続配置することで、属性単位のバッチアクセス（例: position のみ走査）の
// キャッシュ効率を上げる。要素アクセスは particles.positions[i] のように配列を経由する。
struct SoftParticleSoA {
    std::vector<Math::Vector3df> positions;
    std::vector<Math::Vector3df> predicted;
    std::vector<Math::Vector3df> velocities;
    std::vector<Math::Vector3df> forces;
    std::vector<float>           inverseMasses;

    size_t size()  const { return positions.size(); }
    bool   empty() const { return positions.empty(); }
    bool   isPinned(size_t i) const { return inverseMasses[i] == 0.f; }

    void clear() {
        positions.clear();
        predicted.clear();
        velocities.clear();
        forces.clear();
        inverseMasses.clear();
    }

    void resize(size_t n) {
        positions.resize(n);
        predicted.resize(n);
        velocities.resize(n);
        forces.resize(n);
        inverseMasses.resize(n, 1.0f);
    }

    void push_back(const SoftParticle& p) {
        positions.push_back(p.position);
        predicted.push_back(p.predicted);
        velocities.push_back(p.velocity);
        forces.push_back(p.force);
        inverseMasses.push_back(p.inverseMass);
    }
};

} // namespace Physics
} // namespace Phantom
