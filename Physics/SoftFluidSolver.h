#pragma once

#include "ISoftBody.h"
#include "SoftBodySolver.h"
#include "SoftBoundaryParticles.h"
#include "SPHKernel.h"
#include "CGLib/Util/UnCopyable.h"

#include <deque>

namespace Phantom {
namespace Physics {

/**
 * @brief Couples one SoftBody to the fluid solvers through Akinci-style
 * boundary particles (mirrors RigidFluidBinding, but SoftBody has no
 * analytic collision shape, so there is no One-Way SDF penalty equivalent --
 * every binding is inherently Two-Way).
 */
struct SoftFluidBinding {
    ISoftBody*            body = nullptr;
    SoftBoundaryParticles particles;
};

/**
 * @brief Orchestrates a SoftBodySolver together with one or more fluid
 * solvers coupled through SoftBoundaryParticles (mirrors RigidFluidSolver).
 *
 * Unlike RigidFluidSolver's reaction (force + torque about the rigid body's
 * center of mass), SoftBody reactions are purely per-particle -- there is no
 * torque concept for an individual SoftParticle.
 *
 * Documented per-frame sequence (mirrors RigidFluidSolver's):
 * @code
 * world.syncBoundaries(kernel, restDensity); // 1: refresh worldPos, recompute psi, clear accumForce
 * fluidSolver.simulate(dt, maxIter);         // 2: caller-owned fluid solver(s)
 * world.step(dt);                            // 3: apply reaction, then advance soft bodies
 * @endcode
 *
 * Unlike a rigid shape, a SoftMesh deforms every frame, so psi cannot be
 * computed once at bind() time -- syncBoundaries() recomputes it every step
 * (see SoftBoundaryParticles::computePsi()'s doc comment).
 */
class SoftFluidSolver : private UnCopyable {
public:
    SoftBodySolver&       softWorld()       { return world_; }
    const SoftBodySolver& softWorld() const { return world_; }

    /**
     * @brief Binds a SoftBody as a fluid boundary.
     * @param body SoftBody already added to softWorld() (non-owning).
     * @return Reference to the created binding. Valid until clearBindings()
     *         is called (bindings_ is a std::deque, so later bind() calls
     *         never invalidate earlier references/pointers); register the
     *         returned boundary particles with the fluid solver(s)
     *         immediately.
     */
    SoftFluidBinding& bind(ISoftBody* body);

    /** @brief Removes all bindings. */
    void clearBindings() { bindings_.clear(); }

    /** @brief Returns all registered bindings. */
    const std::deque<SoftFluidBinding>& getBindings() const { return bindings_; }

    /**
     * @brief Refreshes every binding's boundary-particle world positions and
     * recomputes psi, and clears accumForce. Call once per step, before
     * running the fluid solver(s).
     * @param kernel      Active fluid's SPH kernel, or nullptr if no fluid
     *                    solver is set yet (psi recomputation is skipped).
     * @param restDensity Active fluid's rest density; ignored if kernel is null.
     */
    void syncBoundaries(const SPHKernel* kernel, float restDensity);

    /**
     * @brief Applies each binding's accumulated boundary-particle reaction
     * to its SoftBody particles, then advances the soft-body world by dt if
     * softWorld().isRunning(). Call once per step, after the fluid solver(s)
     * have finished.
     * @param dt Time step (seconds); assigned to softWorld().solverParams().timeStep.
     */
    void step(float dt);

    /**
     * @brief Same as step(), but advances the soft-body world unconditionally
     * (ignoring softWorld().isRunning()) -- for a scenario/manual-driven
     * "Step" command (see SoftBodySolver::stepUnconditional()).
     */
    void stepForced(float dt);

private:
    void applyTwoWayReactions();

    SoftBodySolver world_;
    // std::deque, not std::vector: callers register &binding.particles with
    // the fluid solver(s) immediately after bind() returns (see doc above),
    // so the container must never invalidate previously-returned addresses
    // on a later push_back().
    std::deque<SoftFluidBinding> bindings_;
};

} // namespace Physics
} // namespace Phantom
