#pragma once

#include "BoundaryParticle.h"

#include <vector>

namespace Phantom {
namespace Physics {

/**
 * @brief Common Two-Way (Track B) boundary-particle-set interface,
 * implemented by RigidBoundaryParticles and SoftBoundaryParticles.
 *
 * Before this interface existed, DFSPHSolver/PBSPHSolver each carried two
 * near-identical copies of their density/constraint-gradient/pressure
 * coupling loops -- one iterating `std::vector<RigidBoundaryParticles*>`,
 * the other `std::vector<SoftBoundaryParticles*>`, differing only in the
 * element type's name (docs/todo/PLAN_physics_ownership_and_coupling_unification.md,
 * 2.2 節). Both element types already exposed the same worldPos/psi/accumForce
 * shape (BoundaryParticle), so routing both through `particles()` here lets
 * those loops be written once against `IBoundaryParticles*`.
 *
 * Deliberately does *not* declare sync()/computePsi(): their call sites
 * (RigidFluidSolver::syncBoundaries(), SoftFluidSolver::syncBoundaries())
 * already dispatch through the concrete rigid/soft binding types, and
 * RigidBoundaryParticles::sync() takes a pose (position + orientation) that
 * SoftBoundaryParticles::sync() has no equivalent for -- forcing a common
 * zero-argument signature would mean caching that pose internally for no
 * benefit to the one thing this interface actually exists for (the
 * density/pressure coupling loops).
 */
class IBoundaryParticles {
public:
    virtual ~IBoundaryParticles() = default;

    /** @brief Returns the mutable particle list. */
    virtual std::vector<BoundaryParticle>& particles() = 0;
    /** @brief Returns the read-only particle list. */
    virtual const std::vector<BoundaryParticle>& particles() const = 0;

    /** @brief Zeroes accumForce on every particle; call once per step before the fluid solver runs. */
    void clearAccumForce() {
        for (auto& bp : particles()) bp.accumForce = Math::Vector3df(0.f, 0.f, 0.f);
    }

    /**
     * @brief True if psi must be recomputed every step (SoftBody: the mesh
     * deforms, so pairwise distances change every frame) vs. once at bind
     * time (RigidBody: samples move rigidly with the shape, so pairwise
     * distances -- and therefore psi -- never change after the first
     * computePsi()).
     */
    virtual bool needsPsiEveryStep() const = 0;
};

} // namespace Physics
} // namespace Phantom
