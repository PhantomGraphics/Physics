#pragma once

#include "RigidBody.h"
#include "RigidBodySolver.h"
#include "RigidFluidSolver.h"
#include "RigidSoftSolver.h"
#include "SoftFluidSolver.h"
#include "ISPHSolver.h"
#include "CGLib/Util/UnCopyable.h"

#include <memory>
#include <vector>

namespace Phantom {
namespace Physics {

/**
 * @brief Top-level orchestrator that steps a caller-supplied ISPHSolver
 * together with a RigidBodySolver and RigidFluidSolver behind a single
 * step().
 *
 * PhysicsSolver does not know how to build a fluid or seed particles --
 * that responsibility belongs to whoever constructs the ISPHSolver (e.g.
 * PhysicsFluidFactory) and hands it over via setFluidSolver(). PhysicsSolver
 * only orchestrates stepping: RigidBodySolver/RigidFluidSolver are owned
 * unconditionally (rigidSolver()/rigidFluidSolver() are always valid), while
 * fluidSolver() may be null until setFluidSolver() is called.
 *
 * step() follows RigidFluidSolver's documented per-frame sequence
 * (syncBoundaries() -> fluid simulate() -> rigidFluidSolver().step()), so
 * bindRigidBody() bindings registered as fluid boundaries are applied
 * automatically every step.
 *
 * SoftBody classes (ClothBody/JellyBody/RopeBody/XPBDSolver/SoftBodySolver)
 * live in this same library since the SoftBody -> Physics integration (see
 * internal design notes). This class also drives a
 * three-way (fluid/rigid/soft) coupling: bindSoftBody() registers a SoftBody
 * as an Akinci-style boundary-particle set (SoftBoundaryParticles), mirroring
 * RigidFluidSolver's Two-Way ("Track B") mechanism -- SoftBody has no
 * analytic collision shape, so unlike rigid bodies there is no One-Way SDF
 * path, and coupling only works with fluid solvers that implement boundary
 * particles (DFSPHSolver/PBSPHSolver; WCSPH/MVC remain unsupported, same as
 * supportsTwoWayCoupling()). Every SoftBody mesh particle is treated as a
 * boundary particle: exactly correct for ClothBody/RopeBody (every particle
 * is on the surface), an accepted approximation for JellyBody (whose
 * SoftMesh::faces is empty -- no surface/interior distinction exists yet),
 * which reads as slightly *more* repulsive than the visual mesh rather than
 * leaky. Since setFluidSolver() may be called with a fluid PhysicsSolver
 * itself doesn't own the kernel/rest-density for, the caller must supply
 * both via setSoftCouplingFluidInfo() right after setFluidSolver() (mirrors
 * bindRigidBody()'s Track B doc comment below).
 *
 * A third bridge, RigidSoftSolver (rigidSoft_), couples the same
 * rigidSolver()/softSolver() to each other directly (Rigid<->SoftBody,
 * independent of any fluid): bindRigidSoftBody() registers a rigid body as
 * both a position-constraint collider (always-on, via
 * SoftBodySolver::addRigidBodyCollider()) and, for CouplingMode::TwoWay, an
 * SDF-penalty reaction source that pushes back on the rigid body every
 * stepUnconditional() (see its call site below for the resulting one-frame
 * Gauss-Seidel-style lag between the two directions).
 */
class PhysicsSolver : private UnCopyable {
public:
    /**
     * @brief Installs the fluid solver to step every frame, discarding any
     * previous registration.
     * @param solver Non-owning; the caller creates and destroys it (same
     *               convention as bindRigidBody()/bindSoftBody() and every
     *               other add()/bind() API in this library -- see
     *               Physics/CLAUDE.md's "非所有ポインタの寿命" section). Must
     *               outlive this PhysicsSolver, or be replaced/cleared first.
     */
    void setFluidSolver(ISPHSolver* solver) { fluidSolver_ = solver; }

    ISPHSolver*       fluidSolver()       { return fluidSolver_; }
    const ISPHSolver* fluidSolver() const { return fluidSolver_; }

    /** @brief Time step (seconds) passed to fluidSolver()->simulate() and RigidFluidSolver::stepForced(). */
    void  setTimeStep(float dt) { timeStep_ = dt; }
    float getTimeStep() const   { return timeStep_; }

    /** @brief Constraint-projection iteration count passed to fluidSolver()->simulate(). */
    void setMaxIter(int maxIter) { maxIter_ = maxIter; }
    int  getMaxIter() const      { return maxIter_; }

    RigidBodySolver&       rigidSolver()       { return rigidFluid_.rigidWorld(); }
    const RigidBodySolver& rigidSolver() const { return rigidFluid_.rigidWorld(); }

    RigidFluidSolver&       rigidFluidSolver()       { return rigidFluid_; }
    const RigidFluidSolver& rigidFluidSolver() const { return rigidFluid_; }

    SoftBodySolver&       softSolver()       { return softFluid_.softWorld(); }
    const SoftBodySolver& softSolver() const { return softFluid_.softWorld(); }

    SoftFluidSolver&       softFluidSolver()       { return softFluid_; }
    const SoftFluidSolver& softFluidSolver() const { return softFluid_; }

    RigidSoftSolver&       rigidSoftSolver()       { return rigidSoft_; }
    const RigidSoftSolver& rigidSoftSolver() const { return rigidSoft_; }

    /**
     * @brief rigidBody を softSolver() へ結合する（bindRigidBody()/bindSoftBody()と同じ
     * "register immediately" 規約）。
     */
    RigidSoftBinding& bindRigidSoftBody(RigidBody* rigidBody, CouplingMode mode) {
        return rigidSoft_.bind(rigidBody, softSolver(), mode);
    }
    void clearRigidSoftBindings() { rigidSoft_.clearBindings(softSolver()); }

    /**
     * @brief Binds a rigid body to the active fluid solver as a One-Way SDF
     * penalty boundary (Track A) and registers it immediately, per
     * RigidFluidSolver::bind()'s "register immediately" caveat.
     *
     * For Two-Way (Track B) coupling, the caller still samples
     * binding.particles and calls addRigidBoundaryParticles() manually
     * (mirrors RigidFluidSolver's own documented usage): psi computation
     * needs the fluid's kernel/rest density, which the caller's own
     * fluid-building code (e.g. PhysicsFluidFactory::getActiveKernel()/
     * getActiveRestDensity()) already has -- PhysicsSolver doesn't hold the
     * fluid, so it can't provide them.
     */
    RigidFluidBinding& bindRigidBody(RigidBody* body, ICollisionShape* shape, CouplingMode mode);

    /** @brief Clears all bindings and the active fluid solver's boundary registrations. */
    void clearRigidBodyBindings();

    void addRigidBoundaryParticles(RigidBoundaryParticles* p);
    void clearRigidBoundaryParticles();

    /**
     * @brief Binds a SoftBody to the active fluid solver as an Akinci-style
     * boundary-particle set and registers it immediately (unlike
     * bindRigidBody()'s Track B path, no separate manual sampling/psi step is
     * needed here -- SoftBody coupling is inherently the boundary-particle
     * method, and psi is recomputed automatically every step by
     * stepUnconditional() using setSoftCouplingFluidInfo()'s kernel/rest
     * density). No-op on the fluid side if fluidSolver() is null or doesn't
     * support boundary particles (WCSPH/MVC) -- the binding is still created
     * and the SoftBody still simulates, it just won't feel/exert fluid forces.
     */
    SoftFluidBinding& bindSoftBody(ISoftBody* body);

    /** @brief Clears all SoftBody bindings and the active fluid solver's SoftBody boundary registrations. */
    void clearSoftBodyBindings();

    /**
     * @brief Supplies the active fluid's SPH kernel and rest density, used by
     * stepUnconditional() to recompute each bound SoftBody's boundary-particle
     * psi every step (SoftBody meshes deform, unlike rigid shapes, so psi
     * can't be computed once at bind time -- see SoftBoundaryParticles's doc
     * comment). Call once, right after setFluidSolver().
     * @param kernel      Non-owning; must outlive this PhysicsSolver (or be
     *                    cleared via another call before it's destroyed).
     * @param restDensity Rest density of the fluid being coupled to.
     */
    void setSoftCouplingFluidInfo(const SPHKernel* kernel, float restDensity) {
        softFluidKernel_      = kernel;
        softFluidRestDensity_ = restDensity;
    }

    /** @brief True only when fluidSolver() is set and implements Two-Way (Track B) coupling. */
    bool supportsTwoWayCoupling() const {
        return fluidSolver_ && fluidSolver_->supportsTwoWayCoupling();
    }

    /** @brief Governs both the fluid step and (via RigidBodySolver::running_) the rigid step. */
    void setRunning(bool running);
    bool isRunning() const { return running_; }

    /** @brief Advances one step if isRunning(), no-op otherwise (interactive Play/Pause). */
    void step();

    /**
     * @brief Advances exactly one step regardless of isRunning() -- for a
     * scenario/manual-driven "Step" command (mirrors RigidBodySolver::stepUnconditional()).
     */
    void stepUnconditional();

private:
    ISPHSolver*      fluidSolver_ = nullptr;
    RigidFluidSolver            rigidFluid_;
    SoftFluidSolver             softFluid_;
    RigidSoftSolver             rigidSoft_;

    const SPHKernel* softFluidKernel_      = nullptr;
    float            softFluidRestDensity_ = 0.f;

    bool  running_  = false;
    float timeStep_ = 0.01f;
    int   maxIter_  = 3;

    void stepFluid();
    void addRigidBoundary(RigidBoundary* b);
    void clearRigidBoundaries();
};

} // namespace Physics
} // namespace Phantom
