#pragma once

#include <vector>
#include "CGLib/Util/UnCopyable.h"
#include "SoftMesh.h"

namespace Phantom {
namespace Physics {

// 複数ソフトボディ（別々の SoftMesh）を跨いだ粒子衝突の検出・応答
// （Space::CompactSpaceHash によるブロードフェーズ）。
// 同一 mesh 内のペアは SelfCollision が別途担当するため対象外。
// meshes 内の各ポインタは非所有。SoftBodySolver::step() の各ボディ
// XPBDSolver::step() 完了後、確定位置 (position) に対して 1 回だけ適用する想定。
class CrossBodyCollision : private UnCopyable {
public:
    struct Params {
        float thickness = 0.02f;
        float cellSize  = 0.1f;
    };

    // dt はフルステップ幅（SoftBodySolver::Params.timeStep 相当）。
    // 位置補正量に応じて速度も補正し、次ステップとの整合を保つ。
    void resolve(const std::vector<SoftMesh*>& meshes, Params p, float dt);
};

} // namespace Physics
} // namespace Phantom
