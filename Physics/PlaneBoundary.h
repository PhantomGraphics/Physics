#pragma once

#include "BoundaryPenalty.h"
#include "CGLib/Math/Vector3d.h"
#include "CGLib/Math/Box3d.h"

#include <vector>

namespace Phantom {
	namespace Physics {

/**
 * @brief A single infinite half-space boundary shared by Fluid/RigidBody/SoftBody.
 *
 * The valid/interior region is where getSignedDistance() >= 0 (the side the
 * normal points to). This is the same normal/offset convention already used
 * by PlaneShape (RigidBody) and PlaneCollider (SoftBody); this class exists
 * so all three physics types share one implementation of the underlying math
 * instead of each keeping its own copy.
 */
class PlaneBoundary
{
public:
	PlaneBoundary() = default;

	/**
	 * @brief Constructs a plane boundary.
	 * @param normal Unit normal pointing into the valid/interior region.
	 * @param offset Plane offset such that valid points satisfy dot(normal, p) >= offset.
	 */
	PlaneBoundary(const Math::Vector3df& normal, const float offset);

	const Math::Vector3df& getNormal() const { return normal; }

	float getOffset() const { return offset; }

	/**
	 * @brief Signed distance to the plane; >= 0 on the valid/interior side.
	 * @param pos World-space position.
	 * @return dot(normal, pos) - offset.
	 */
	float getSignedDistance(const Math::Vector3df& pos) const;

	/**
	 * @brief Penalty force pushing a penetrating position back to the valid side.
	 * Same -over/dt^2 spring formulation as the former BoxBoundary::getForceX/Y/Z.
	 * @param pos World-space position.
	 * @param timeStep Time step used to scale the repulsion force.
	 * @return Zero if pos is on the valid side; otherwise a force along +normal.
	 */
	Math::Vector3df getBoundaryForce(const Math::Vector3df& pos, const float timeStep) const;

	/**
	 * @brief Same penalty force, plus a damper on the wall-normal velocity
	 * component -- see boundaryPenaltyAcceleration() for why the undamped form
	 * above is a restitution-1 trampoline and when that matters.
	 * @param pos          World-space position.
	 * @param velocity     World-space velocity of the same particle.
	 * @param timeStep     Time step used to scale the repulsion force.
	 * @param dampingRatio Damping ratio, clamped to [0, 0.5]; 0 reproduces the
	 *        two-argument overload exactly.
	 * @return Zero if pos is on the valid side; otherwise a force along +normal.
	 */
	Math::Vector3df getBoundaryForce(const Math::Vector3df& pos, const Math::Vector3df& velocity,
	                                 const float timeStep, const float dampingRatio) const;

	/**
	 * @brief Hard position correction: projects a penetrating position back onto the plane.
	 * @param pos World-space position.
	 * @return pos unchanged if valid; otherwise pos - signedDistance*normal.
	 */
	Math::Vector3df clampPosition(const Math::Vector3df& pos) const;

private:
	Math::Vector3df normal{ 0.f, 1.f, 0.f };
	float offset{ 0.f };
};

/**
 * @brief Converts an axis-aligned box into 6 inward-facing PlaneBoundary instances.
 * This is the direct replacement for the former BoxBoundary: a box container is
 * just 6 planes whose normals point toward the box interior.
 * @param box Axis-aligned bounding box defining the interior region.
 * @return Six PlaneBoundary instances, one per box face.
 */
std::vector<PlaneBoundary> makeBoxPlaneBoundaries(const Math::Box3df& box);

	}
}
