#include "CrossBodyCollision.h"
#include "CGLib/Space/Space/CompactSpaceHash.h"

#include <cmath>

namespace Phantom {
namespace Physics {

namespace {

struct ParticleRef {
    SoftMesh* mesh;
    int       particleIndex;
    int       meshIndex;
};

} // namespace

void CrossBodyCollision::resolve(const std::vector<SoftMesh*>& meshes, Params p, float dt) {
    if (p.cellSize <= 1e-8f || dt <= 1e-8f) return;
    if (meshes.size() < 2) return;

    std::vector<ParticleRef> refs;
    for (int m = 0; m < static_cast<int>(meshes.size()); ++m) {
        for (size_t i = 0; i < meshes[m]->particles.size(); ++i)
            refs.push_back({ meshes[m], static_cast<int>(i), m });
    }

    const int total = static_cast<int>(refs.size());
    if (total < 2) return;

    Space::CompactSpaceHash grid(p.cellSize, total * 2);
    for (const auto& r : refs)
        grid.add(r.mesh->particles.positions[r.particleIndex]);

    for (int i = 0; i < total; ++i) {
        const auto& ri = refs[i];
        SoftParticleSoA& pi = ri.mesh->particles;

        for (int j : grid.findNeighborIndices(i)) {
            if (j <= i) continue;                          // 各ペアを一度だけ処理
            const auto& rj = refs[j];
            if (rj.meshIndex == ri.meshIndex) continue;     // 同一ボディは対象外（SelfCollision が担当）

            SoftParticleSoA& pj = rj.mesh->particles;
            float wSum = pi.inverseMasses[ri.particleIndex] + pj.inverseMasses[rj.particleIndex];
            if (wSum < 1e-8f) continue;

            Math::Vector3df diff = pj.positions[rj.particleIndex] - pi.positions[ri.particleIndex];
            float dist = glm::length(diff);
            if (dist >= p.thickness || dist < 1e-8f) continue;

            Math::Vector3df n = diff / dist;
            float penetration = p.thickness - dist;
            Math::Vector3df correction = n * penetration;

            Math::Vector3df correctionI = -correction * (pi.inverseMasses[ri.particleIndex] / wSum);
            Math::Vector3df correctionJ =  correction * (pj.inverseMasses[rj.particleIndex] / wSum);

            pi.positions[ri.particleIndex] += correctionI;
            pj.positions[rj.particleIndex] += correctionJ;

            pi.velocities[ri.particleIndex] += correctionI / dt;
            pj.velocities[rj.particleIndex] += correctionJ / dt;
        }
    }
}

} // namespace Physics
} // namespace Phantom
