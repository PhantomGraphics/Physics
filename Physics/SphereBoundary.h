#pragma once

#include "BoundaryPenalty.h"
#include "CGLib/Math/Vector3d.h"

#include <limits>

namespace Phantom {
	namespace Physics {

/**
 * @brief A single solid-sphere boundary shared by Fluid (and future Rigid/SoftBody use).
 *
 * The valid/interior region is where getSignedDistance() >= 0 (inside the
 * sphere), the same convention as PlaneBoundary. See
 * docs/todo/PLAN_sph_showcase_water_sphere.md section 8-A for the design
 * rationale (closed container for the "water sphere" showcase scene).
 */
class SphereBoundary
{
public:
	SphereBoundary() = default;

	/**
	 * @brief Constructs a sphere boundary.
	 * @param center World-space center of the sphere.
	 * @param radius Sphere radius; valid points satisfy |pos - center| <= radius.
	 * @param maxPenetration Particles whose penetration depth (distance outside
	 *        the sphere) exceeds this are considered lost and left alone by
	 *        both isActiveAt() and getBoundaryForce() -- a safety valve against
	 *        the -d/dt^2 spring force exploding for a particle that somehow
	 *        ended up far outside. Defaults to unbounded (no cutoff).
	 */
	SphereBoundary(const Math::Vector3df& center, const float radius,
	                const float maxPenetration = std::numeric_limits<float>::max());

	const Math::Vector3df& getCenter() const { return center_; }

	float getRadius() const { return radius_; }

	float getMaxPenetration() const { return maxPenetration_; }

	/**
	 * @brief Signed distance to the sphere surface; >= 0 inside the sphere.
	 * @param pos World-space position.
	 * @return radius - |pos - center|.
	 */
	float getSignedDistance(const Math::Vector3df& pos) const;

	/**
	 * @brief Whether pos is within maxPenetration of the sphere (inside, or
	 * only shallowly outside). False for a particle that has escaped too far
	 * to be worth correcting.
	 * @param pos World-space position.
	 */
	bool isActiveAt(const Math::Vector3df& pos) const;

	/**
	 * @brief Penalty force pushing a penetrating position back toward the center.
	 * Same -d/dt^2 spring formulation as PlaneBoundary::getBoundaryForce().
	 * Zero if pos is inside the sphere, or beyond maxPenetration outside it.
	 * @param pos World-space position.
	 * @param timeStep Time step used to scale the repulsion force.
	 * @return Zero, or a force directed toward the center.
	 */
	Math::Vector3df getBoundaryForce(const Math::Vector3df& pos, const float timeStep) const;

	/**
	 * @brief Same penalty force, plus a damper on the wall-normal velocity
	 * component -- see boundaryPenaltyAcceleration() for why the undamped form
	 * above is a restitution-1 trampoline, and
	 * docs/issue/water_sphere_showcase_emitter_instability.md section 3 for the
	 * showcase scene that made it visible (a jet landing in an *empty* sphere
	 * has no water above it to absorb the rebound).
	 * @param pos          World-space position.
	 * @param velocity     World-space velocity of the same particle.
	 * @param timeStep     Time step used to scale the repulsion force.
	 * @param dampingRatio Damping ratio, clamped to [0, 0.5]; 0 reproduces the
	 *        two-argument overload exactly.
	 * @return Zero, or a force directed toward the center.
	 */
	Math::Vector3df getBoundaryForce(const Math::Vector3df& pos, const Math::Vector3df& velocity,
	                                 const float timeStep, const float dampingRatio) const;

	/**
	 * @brief Hard position correction: projects a penetrating position back onto the sphere surface.
	 * @param pos World-space position.
	 * @return pos unchanged if inside the sphere; otherwise pos moved toward the center onto the surface.
	 */
	Math::Vector3df clampPosition(const Math::Vector3df& pos) const;

private:
	Math::Vector3df center_{ 0.f, 0.f, 0.f };
	float radius_{ 1.f };
	float maxPenetration_{ std::numeric_limits<float>::max() };
};

	}
}
