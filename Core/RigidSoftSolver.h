#pragma once

#include "RigidBody.h"
#include "RigidBoundary.h"
#include "RigidFluidSolver.h"  // CouplingMode を再利用
#include "SoftBodySolver.h"
#include "CGLib/Util/UnCopyable.h"

#include <deque>

namespace Phantom {
namespace Physics {

/**
 * @brief 剛体1個をSoftBodyワールドに結合する。RigidFluidBinding/SoftFluidBindingと違い、
 * boundaryは「粒子の反力を測るための道具」でしかなく、柔体側の位置拘束応答は
 * SoftBodySolver::addRigidBodyCollider()（bind()内で呼ぶ）が別途常に効く。
 */
struct RigidSoftBinding {
    RigidBody*    rigidBody = nullptr;
    RigidBoundary boundary;
    CouplingMode  mode = CouplingMode::OneWay;
};

/**
 * @brief RigidBodySolverとSoftBodySolverを"どちらも所有せず"橋渡しする軽量アダプタ。
 * PhysicsSolverが既に持つ rigidSolver()/softSolver()（RigidFluidSolver/SoftFluidSolverが
 * 実体を所有）をそのまま参照させる想定 -- 3つ目の独立したワールドを作らない。
 */
class RigidSoftSolver : private UnCopyable {
public:
    /**
     * @brief 剛体をsoftWorldへ結合する。SoftBodySolver::addRigidBodyCollider()を即座に呼ぶため
     * (RigidFluidSolver::bind()の"register immediately"と同じ規約)、softWorld上の全SoftBody
     * (bind()以降に追加されるものも含む、syncColliders()が毎ステップ再登録するため)がこの剛体の
     * SDFに対して位置拘束されるようになる。mode==OneWayでもTwoWayでもこの位置拘束は常に有効
     * (方向で変わるのは反力の有無だけ)。
     * @return 作成したバインディングへの参照。clearBindings()されるまで有効
     *         (bindings_はstd::dequeなので、後続のbind()で再確保が起きても
     *         既存の参照/ポインタは無効化されない)。
     */
    RigidSoftBinding& bind(RigidBody* rigidBody, SoftBodySolver& softWorld, CouplingMode mode);

    /** @brief 全バインディングと softWorld 側の剛体コライダー登録を両方消す。 */
    void clearBindings(SoftBodySolver& softWorld);

    const std::deque<RigidSoftBinding>& getBindings() const { return bindings_; }

    /**
     * @brief Two-Wayバインディングそれぞれについて、boundaryを再syncしてから
     * softWorld.computeRigidReactionForce()で反力を求め、rigidBody->applyForce/applyTorque()で
     * 剛体へ加える。RigidBodySolverが積分する前（PhysicsSolver::stepUnconditional()なら
     * rigidFluid_.stepForced()より前）に1回呼ぶ。
     */
    void applyTwoWayReactions(const SoftBodySolver& softWorld);

private:
    // std::vectorではなくstd::deque: RigidFluidSolver::bindings_と同じ理由
    // (登録済み参照/ポインタをbind()の再確保で無効化させないため)。
    std::deque<RigidSoftBinding> bindings_;
};

} // namespace Physics
} // namespace Phantom
