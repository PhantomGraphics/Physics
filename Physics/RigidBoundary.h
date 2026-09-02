#pragma once

#include "ICollisionShape.h"
#include "RigidBody.h"
#include "CGLib/Math/Vector3d.h"
#include "CGLib/Math/Quaternion.h"
#include <algorithm>

namespace Phantom {
namespace Physics {

/**
 * @brief Adapts a rigid body's pose into an SDF penalty boundary that fluid
 * solvers can query per particle (One-Way coupling: rigid body -> fluid only).
 *
 * Does not accumulate any reaction force back onto the rigid body; see
 * RigidBoundaryParticles for the Two-Way (Track B) counterpart.
 */
class RigidBoundary {
public:
    /**
     * @brief Copies pose/velocity/shape from a simulated rigid body.
     * @param body Rigid body to read the current pose from.
     */
    void sync(const RigidBody& body);

    /**
     * @brief Sets an arbitrary kinematic pose not driven by a RigidBody
     * (e.g. a scripted/animated obstacle).
     * @param pos       World-space position.
     * @param orient    World-space orientation.
     * @param linearVel Linear velocity (used only by callers that need it; not
     *                  applied internally by getBoundaryForce()).
     */
    void syncKinematic(const Math::Vector3df& pos, const Math::Quaternion& orient,
                       const Math::Vector3df& linearVel = Math::Vector3df(0.f, 0.f, 0.f));

    /**
     * @brief Computes the One-Way penalty force pushing a fluid particle out
     * of this boundary's collision shape.
     * @param particlePos World-space particle position.
     * @return Zero if the particle is outside the shape; otherwise a force
     *         along the surface normal proportional to the penetration depth.
     */
    Math::Vector3df getBoundaryForce(const Math::Vector3df& particlePos) const;

    /**
     * @brief Sets the collision shape used for the SDF query (non-owning).
     * @param shape Collision shape; must outlive this RigidBoundary.
     */
    void setShape(const ICollisionShape* shape) { shape_ = shape; }

    /**
     * @brief Sets the penalty spring stiffness.
     * @param stiffness Acceleration per unit penetration depth (see
     *        getBoundaryForce() -- despite the name, every solver treats its
     *        return value as an acceleration: DFSPH/WCSPH multiply it by
     *        mass/density to get a force, PBSPH multiplies by dt^2 for a
     *        position correction, MVC integrates it into velocity directly).
     *        Units are 1/time^2.
     */
    void setPenaltyStiffness(const float stiffness) { penaltyStiffness_ = stiffness; }

    /**
     * @brief Returns the penalty spring stiffness.
     * @return Acceleration per unit penetration depth (1/time^2); see
     *         setPenaltyStiffness().
     */
    float getPenaltyStiffness() const { return penaltyStiffness_; }

    /**
     * @brief Estimates a scale-appropriate penalty stiffness from the
     * simulation time step alone (see internal design notes
     * Phase 2: penaltyStiffness_ has units 1/time^2 -- getBoundaryForce()
     * returns stiffness * penetrationDepth as an acceleration, not a length-
     * scaled force -- so, unlike #9's pressureCoe/stiffness, it does not need
     * effectLength/restDensity to become scale-invariant; it only needs to
     * track whatever dt the caller actually integrates with).
     *
     * Every caller (DFSPH/WCSPH/MVC via explicit or symplectic Euler, PBSPH
     * via an unconditional position correction) is a first-order integration
     * of the Hooke's-law spring `accel = -stiffness * penetrationDepth`,
     * which is only non-divergent for `stiffness * dt^2 < 4`. stabilityMargin
     * is the fraction of that bound to target; the default 0.5 reproduces
     * the historical fixed default (5000.f) exactly at dt=0.01f (this
     * codebase's default maxTimeStep/boundaryTimeStep), and scales smaller
     * automatically as dt shrinks with the scene (e.g. via a CFL-derived dt
     * that tracks the particle radius).
     * @param dt Characteristic simulation time step this boundary will be
     *        integrated with (e.g. the fluid solver's maxTimeStep/
     *        boundaryTimeStep); must be positive.
     * @param stabilityMargin Fraction of the explicit-integration stability
     *        bound (4/dt^2) to target. Defaults to 0.5.
     * @return Penalty stiffness (1/time^2) for setPenaltyStiffness().
     */
    static float estimateStiffness(const float dt, const float stabilityMargin = 0.5f) {
        const float safeDt = std::max(dt, 1.0e-8f);
        return std::max(stabilityMargin, 0.0f) / (safeDt * safeDt);
    }

    /**
     * @brief Sets penaltyStiffness_ from estimateStiffness(dt, stabilityMargin).
     * Does nothing unless called explicitly -- existing callers of
     * setPenaltyStiffness() and the fixed default (5000.f) are unaffected.
     * @param dt See estimateStiffness().
     * @param stabilityMargin See estimateStiffness().
     */
    void setStiffnessFromScale(const float dt, const float stabilityMargin = 0.5f) {
        penaltyStiffness_ = estimateStiffness(dt, stabilityMargin);
    }

    /** @brief Returns the last synced world-space position. */
    const Math::Vector3df&  getPosition()    const { return pos_; }
    /** @brief Returns the last synced world-space orientation. */
    const Math::Quaternion& getOrientation() const { return orient_; }
    /** @brief Returns the last synced linear velocity. */
    const Math::Vector3df&  getVelocity()    const { return velocity_; }

private:
    const ICollisionShape* shape_ = nullptr;
    Math::Vector3df  pos_ = { 0.f, 0.f, 0.f };
    Math::Quaternion orient_ = { 1.f, 0.f, 0.f, 0.f };
    Math::Vector3df  velocity_ = { 0.f, 0.f, 0.f };
    float penaltyStiffness_ = 5000.f;
};

} // namespace Physics
} // namespace Phantom
