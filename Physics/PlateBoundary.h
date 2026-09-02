#pragma once

#include "BoundaryPenalty.h"
#include "IShapeBoundary.h"
#include "CGLib/Math/Vector3d.h"

namespace Phantom {
	namespace Physics {

/**
 * @brief A single finite plate boundary: a thin oriented bounding box (OBB)
 * whose *outside* is the valid/interior region (getSignedDistance() >= 0),
 * the same convention as PlaneBoundary/SphereBoundary.
 *
 * Where PlaneBoundary is an infinite half-space (so a box container can only
 * be a convex interior), a plate is finite in every direction: water runs off
 * its edge instead of being held by an invisible wall that extends forever.
 * See docs/todo/PLAN_sph_showcase_waterfall.md section 3 for the design
 * rationale (the "waterfall" showcase's channel floor, weir, cliff face and
 * rock shelves are all finite plates).
 *
 * The plate is given a half-thickness rather than being a zero-thickness
 * rectangle so that (a) both sides repel equally -- water that wraps behind a
 * weir is not pushed back through it -- (b) penetration depth is automatically
 * bounded by halfThickness (no maxPenetration safety valve is needed, unlike
 * SphereBoundary) and (c) the visible rock geometry can be generated at
 * (2*halfU, 2*halfV, 2*halfThickness) so the water surface and the rock agree.
 * Choose halfThickness comfortably larger than v_max * dt so a particle cannot
 * tunnel through the plate in one step.
 */
class PlateBoundary : public IShapeBoundary
{
public:
	PlateBoundary() = default;

	/**
	 * @brief Constructs a finite plate boundary.
	 * @param center        World-space center of the plate.
	 * @param normal        Plate normal (normalized internally).
	 * @param uAxis         First in-plane axis; its component along the normal
	 *                      is removed and the result normalized. The second
	 *                      in-plane axis is cross(normal, u).
	 * @param halfU         In-plane half-size along u.
	 * @param halfV         In-plane half-size along v.
	 * @param halfThickness Half-size along the normal. Take it well above
	 *                      v_max * dt (see the class doc comment).
	 */
	PlateBoundary(const Math::Vector3df& center,
	              const Math::Vector3df& normal,
	              const Math::Vector3df& uAxis,
	              const float halfU, const float halfV,
	              const float halfThickness);

	const Math::Vector3df& getCenter() const { return center_; }
	const Math::Vector3df& getNormal() const { return n_; }
	const Math::Vector3df& getUAxis() const { return u_; }
	const Math::Vector3df& getVAxis() const { return v_; }
	float getHalfU() const { return halfU_; }
	float getHalfV() const { return halfV_; }
	float getHalfThickness() const { return halfThickness_; }

	/**
	 * @brief Standard box signed distance (negative inside the OBB); >= 0 is
	 * the valid region, matching PlaneBoundary/SphereBoundary.
	 * @param pos World-space position.
	 */
	float getSignedDistance(const Math::Vector3df& pos) const override;

	/**
	 * @brief Whether pos lies within the plate's world AABB inflated by
	 * effectLength -- a cheap early-out for the particle x plate double loop
	 * in WCSPHSolver::addBoundaryForce()/addBoundaryDensity().
	 * @param pos          World-space position.
	 * @param effectLength Kernel support radius the AABB is inflated by.
	 */
	bool isActiveAt(const Math::Vector3df& pos, const float effectLength) const override;

	/**
	 * @brief Penalty force pushing a penetrating position out through the
	 * nearest face. Same -d/dt^2 spring formulation as
	 * PlaneBoundary::getBoundaryForce(); zero when getSignedDistance() >= 0
	 * (so a particle past the plate's edge feels nothing and falls freely --
	 * the behavior the whole finite-plate design exists for).
	 * @param pos      World-space position.
	 * @param timeStep Time step used to scale the repulsion force.
	 */
	Math::Vector3df getBoundaryForce(const Math::Vector3df& pos, const float timeStep) const override;

	/**
	 * @brief Same penalty force, plus a damper on the face-normal velocity
	 * component (see boundaryPenaltyAcceleration()). dampingRatio == 0
	 * reproduces the two-argument overload exactly.
	 * @param pos          World-space position.
	 * @param velocity     World-space velocity of the same particle.
	 * @param timeStep     Time step used to scale the repulsion force.
	 * @param dampingRatio Damping ratio, clamped to [0, 0.5].
	 */
	Math::Vector3df getBoundaryForce(const Math::Vector3df& pos, const Math::Vector3df& velocity,
	                                 const float timeStep, const float dampingRatio) const override;

	/**
	 * @brief Hard position correction: projects a position inside the plate
	 * onto its nearest face. pos unchanged if already outside.
	 * @param pos World-space position.
	 */
	Math::Vector3df clampPosition(const Math::Vector3df& pos) const override;

	/**
	 * @brief Density contribution helper: outward distance from the large
	 * (normal-facing) face, |dot(pos - center, n)| - halfThickness. Negative
	 * inside the slab.
	 * @param pos World-space position.
	 */
	float getFaceDistance(const Math::Vector3df& pos) const;
	float getDensityDistance(const Math::Vector3df& pos) const override { return getFaceDistance(pos); }

	/**
	 * @brief Density contribution helper: a [0, 1] taper that falls off near
	 * the plate's rim, so an edge particle is not handed the density of a
	 * full solid half-space that mostly isn't there (see
	 * docs/todo/PLAN_sph_showcase_waterfall.md section 3.4). 1 more than
	 * effectLength inside the rim, 0.5 right over the rim, 0 past it.
	 * @param pos          World-space position.
	 * @param effectLength Kernel support radius.
	 */
	float getRimFraction(const Math::Vector3df& pos, const float effectLength) const;
	float getDensityWeight(const Math::Vector3df& pos, const float effectLength) const override {
		return getRimFraction(pos, effectLength);
	}

private:
	Math::Vector3df center_{ 0.f, 0.f, 0.f };
	Math::Vector3df n_{ 0.f, 0.f, 1.f };
	Math::Vector3df u_{ 1.f, 0.f, 0.f };
	Math::Vector3df v_{ 0.f, 1.f, 0.f };
	float halfU_{ 1.f };
	float halfV_{ 1.f };
	float halfThickness_{ 0.01f };
	// World AABB of the OBB, precomputed for isActiveAt().
	Math::Vector3df aabbMin_{ 0.f, 0.f, 0.f };
	Math::Vector3df aabbMax_{ 0.f, 0.f, 0.f };

	// (localU, localV, localN) = coordinates of pos in the plate's frame.
	Math::Vector3df toLocal(const Math::Vector3df& pos) const;
};

	}
}
