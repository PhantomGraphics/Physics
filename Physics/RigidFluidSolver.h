#pragma once

#include "RigidBody.h"
#include "RigidBodySolver.h"
#include "RigidBoundary.h"
#include "RigidBoundaryParticles.h"
#include "ICollisionShape.h"
#include "CGLib/Util/UnCopyable.h"

#include <deque>

namespace Phantom {
namespace Physics {

/** @brief Direction of the rigid-fluid interaction for a single binding. */
enum class CouplingMode { OneWay, TwoWay };

/**
 * @brief Couples one rigid body to the fluid solvers through an SDF penalty
 * boundary (One-Way, `boundary`) and/or Akinci-style boundary particles
 * (Two-Way, `particles`).
 *
 * For TwoWay bindings, the caller is responsible for sampling and wiring
 * `particles` right after bind() (mirroring how `boundary` itself is
 * registered with the fluid solver(s) -- see this class's own doc above):
 *
 * @code
 * auto& binding = world.bind(body, shape, CouplingMode::TwoWay);
 * binding.particles.sample(*shape, spacing);
 * binding.particles.computePsi(*dfsphFluid->getKernel(), dfsphFluid->getDensity());
 * dfsphSolver.addRigidBoundaryParticles(&binding.particles);
 * @endcode
 *
 * For OneWay bindings, `particles` is simply left empty and unused.
 */
struct RigidFluidBinding {
    RigidBody*             body = nullptr;
    RigidBoundary          boundary;
    RigidBoundaryParticles particles;
    CouplingMode           mode = CouplingMode::OneWay;
};

/**
 * @brief Orchestrates a RigidBodySolver together with one or more fluid
 * solvers coupled through RigidBoundary penalty forces.
 *
 * DFSPHSolver/PBSPHSolver/CSPHSolver all implement the common
 * ISPHSolver interface (unified simulate(dt, maxIter)), but this class still
 * does not own or call any fluid solver itself -- the caller runs its own
 * solver(s) sandwiched between syncBoundaries() and step():
 *
 * @code
 * // one-time setup
 * auto& binding = world.bind(body, body->shape, CouplingMode::OneWay);
 * dfsphSolver.addRigidBoundary(&binding.boundary);
 * world.rigidWorld().setRunning(true);
 *
 * // per-frame
 * world.syncBoundaries();        // 1: refresh boundary pose, clear Two-Way accumForce
 * dfsphSolver.simulate(dt, maxIter);  // 2: caller-owned fluid solver(s)
 * world.step(dt);                // 3: apply Two-Way reaction, then advance rigid bodies
 * @endcode
 *
 * Large dt values can cause non-physical penalty-force oscillation;
 * splitting a frame's dt into several substeps (repeating the per-frame
 * sequence above with a smaller dt) is the caller's responsibility.
 */
class RigidFluidSolver : private UnCopyable {
public:
    RigidBodySolver&       rigidWorld()       { return world_; }
    const RigidBodySolver& rigidWorld() const { return world_; }

    /**
     * @brief Registers a rigid body as a fluid boundary.
     * @param body  Rigid body already added to rigidWorld() (non-owning).
     * @param shape Collision shape used for the SDF query (non-owning; must
     *              outlive this binding). Typically the same shape as
     *              body->shape.
     * @param mode  One-Way (no reaction) or Two-Way (reaction; Phase 5).
     * @return Reference to the created binding. Valid until clearBindings()
     *         is called (bindings_ is a std::deque, so later bind() calls
     *         never invalidate earlier references/pointers); register the
     *         returned boundary with the fluid solver(s) immediately.
     */
    RigidFluidBinding& bind(RigidBody* body, ICollisionShape* shape, CouplingMode mode);

    /** @brief Removes all bindings. */
    void clearBindings() { bindings_.clear(); }

    /** @brief Returns all registered bindings. */
    const std::deque<RigidFluidBinding>& getBindings() const { return bindings_; }

    /**
     * @brief Refreshes every binding's RigidBoundary/particles pose from its
     * RigidBody and clears Two-Way accumForce. Call once per step, before
     * running the fluid solver(s).
     */
    void syncBoundaries();

    /**
     * @brief Applies each Two-Way binding's accumulated boundary-particle
     * reaction to its rigid body (force at the sample point, plus the
     * resulting torque about the body's center of mass), then advances the
     * rigid-body world by dt if rigidWorld().isRunning(). Call once per
     * step, after the fluid solver(s) have finished.
     * @param dt Time step (seconds); assigned to rigidWorld().timeStep.
     */
    void step(float dt);

    /**
     * @brief Same as step(), but advances the rigid-body world unconditionally
     * (ignoring rigidWorld().isRunning()) -- for a scenario/manual-driven
     * "Step" command (see RigidBodyWorld::stepUnconditional()).
     */
    void stepForced(float dt);

private:
    void applyTwoWayReactions();

    RigidBodySolver world_;
    // std::deque, not std::vector: callers register &binding.boundary /
    // &binding.particles with the fluid solver(s) immediately after bind()
    // returns (see doc above), so the container must never invalidate
    // previously-returned addresses on a later push_back().
    std::deque<RigidFluidBinding> bindings_;
};

} // namespace Physics
} // namespace Phantom
