#pragma once

#include <vector>
#include "SoftMesh.h"

namespace Phantom {
namespace Physics {

// 粒子ベースの自己衝突検出・応答（Space::CompactSpaceHash によるブロードフェーズ）。
// mesh.edges で直接繋がっている粒子ペアは構造制約と競合するため対象外。
// cellSize は thickness 以上を推奨（3x3x3 近傍探索で取りこぼしを防ぐため）。
class SelfCollision {
public:
    struct Params {
        float thickness = 0.02f;
        float cellSize  = 0.1f;
    };

    void update(SoftMesh& mesh, Params p);
};

} // namespace Physics
} // namespace Phantom
