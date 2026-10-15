#pragma once

#include <memory>
#include <vector>
#include "CGLib/Util/UnCopyable.h"
#include "CGLib/Math/Math/Vector3d.h"
#include "ISoftBody.h"
#include "XPBDSolver.h"
#include "SphereCollider.h"
#include "PlaneCollider.h"
#include "RigidBodyCollider.h"
#include "RigidBoundary.h"
#include "CrossBodyCollision.h"

namespace Phantom {
namespace Physics {

class SoftBodySolver : private UnCopyable {
public:
    struct Params {
        bool            sphereEnabled = false;
        Math::Vector3df sphereCenter  = { 0.f, 0.5f, 0.f };
        float           sphereRadius  = 0.3f;
        bool            running       = false;

        bool            crossBodyCollisionEnabled   = false;
        float           crossBodyCollisionThickness = 0.02f;
        float           crossBodyCollisionCellSize  = 0.1f;
    };

    /** @brief Registers a soft body. Non-owning; caller must keep it alive. */
    void addBody(ISoftBody* body) {
        bodies_.push_back(body);
        solvers_.push_back(std::make_unique<XPBDSolver>());
        rebuildEntry(bodies_.size() - 1);
    }

    void clearBodies();

    /** @brief Advances one step if isRunning(), no-op otherwise (interactive Play/Pause). */
    void step();

    /**
     * @brief Advances exactly one step regardless of isRunning() -- for a
     * scenario/manual-driven "Step" command (mirrors RigidBodySolver::stepUnconditional()).
     */
    void stepUnconditional();

    void reset();

    // 外部（RigidBodyWorld 等）が生存期間を管理する RigidBody への
    // 衝突コライダーを登録する。非所有ポインタ。
    void addRigidBodyCollider(RigidBody* rb);
    void clearRigidBodyColliders();

    /**
     * @brief Sums the Newton's-third-law reaction to boundary.getBoundaryForce(particle.position)
     * (i.e. -getBoundaryForce(), since that method itself returns the penalty force pushing the
     * *particle* away from the rigid shape) over every particle of every SoftBody (read-only --
     * does not move any particle), plus the resulting torque about pivot. Independent of
     * RigidBodyCollider's position-based collision response, which still applies separately via
     * syncColliders(); this only *measures* the reaction (mass-independent, geometry only,
     * mirroring RigidBoundary's own penalty-force convention).
     * @param boundary Must already be sync()'d for this step.
     * @param pivot    Point the torque is measured about (typically the
     *                 rigid body's center of mass).
     * @param outTorque Receives the accumulated torque.
     */
    Math::Vector3df computeRigidReactionForce(const RigidBoundary& boundary,
                                               const Math::Vector3df& pivot,
                                               Math::Vector3df& outTorque) const;

    bool isRunning()        const { return params_.running; }
    void setRunning(bool v)       { params_.running = v; }

    Params&              params()       { return params_; }
    const Params&        params() const { return params_; }
    XPBDSolver::Params&  solverParams() { return solverParams_; }

    size_t getBodyCount()     const { return bodies_.size(); }
    size_t getParticleCount() const;
    float  getMaxSpeed()      const;  // 全粒子中の最大速度（デバッグ・回帰検証用）

    /** @brief The registered soft bodies (non-owning), for read-only consumers such as the view. */
    const std::vector<ISoftBody*>& bodies() const { return bodies_; }

    /** @brief The rigid bodies registered through addRigidBodyCollider() (non-owning). */
    std::vector<RigidBody*> rigidColliderBodies() const {
        std::vector<RigidBody*> out;
        out.reserve(rigidColliders_.size());
        for (const auto& rc : rigidColliders_) out.push_back(rc->getBody());
        return out;
    }

private:
    Params                                   params_;
    XPBDSolver::Params                       solverParams_;
    std::vector<ISoftBody*>                  bodies_;
    std::vector<std::unique_ptr<XPBDSolver>> solvers_;
    SphereCollider                           sphere_;
    PlaneCollider                            floor_;
    std::vector<std::unique_ptr<RigidBodyCollider>> rigidColliders_;
    CrossBodyCollision                       crossBodyCollision_;

    void rebuildEntry(size_t idx);
    void syncColliders();
    void stepInternal();
};

} // namespace Physics
} // namespace Phantom
