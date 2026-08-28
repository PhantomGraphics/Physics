#include "SelfCollision.h"
#include "CGLib/Space/Space/CompactSpaceHash.h"

#include <cmath>
#include <set>
#include <utility>

namespace Phantom {
namespace Physics {

void SelfCollision::update(SoftMesh& mesh, Params p) {
    if (p.cellSize <= 1e-8f) return;
    auto& particles = mesh.particles;
    const int n = static_cast<int>(particles.size());
    if (n < 2) return;

    // 構造エッジで直接繋がっている粒子ペアは自己衝突の対象から除外
    std::set<std::pair<int, int>> connected;
    for (const auto& e : mesh.edges)
        connected.insert(e.a < e.b ? std::make_pair(e.a, e.b) : std::make_pair(e.b, e.a));

    Space::CompactSpaceHash grid(p.cellSize, n * 2);
    for (int i = 0; i < n; ++i)
        grid.add(particles.predicted[i]);

    for (int i = 0; i < n; ++i) {
        float wi = particles.inverseMasses[i];

        for (int j : grid.findNeighborIndices(i)) {
            if (j <= i) continue;  // 各ペアを一度だけ処理

            auto key = std::make_pair(i, j);
            if (connected.count(key)) continue;

            float wj = particles.inverseMasses[j];
            float wSum = wi + wj;
            if (wSum < 1e-8f) continue;

            Math::Vector3df diff = particles.predicted[j] - particles.predicted[i];
            float dist = glm::length(diff);
            if (dist >= p.thickness || dist < 1e-8f) continue;

            Math::Vector3df n = diff / dist;
            float penetration = p.thickness - dist;
            Math::Vector3df correction = n * penetration;

            particles.predicted[i] -= correction * (wi / wSum);
            particles.predicted[j] += correction * (wj / wSum);
        }
    }
}

} // namespace Physics
} // namespace Phantom
