#pragma once

#include "BoundaryPenalty.h"
#include "IShapeBoundary.h"

#include <limits>

namespace Phantom::Physics {

/** A finite, capped cylindrical container. The valid region is its interior. */
class CylinderBoundary : public IShapeBoundary
{
public:
	CylinderBoundary() = default;
	CylinderBoundary(const Math::Vector3df& center, const Math::Vector3df& axis,
	                 float radius, float halfHeight,
	                 float maxPenetration = std::numeric_limits<float>::max());

	const Math::Vector3df& getCenter() const { return center_; }
	const Math::Vector3df& getAxis() const { return axis_; }
	float getRadius() const { return radius_; }
	float getHalfHeight() const { return halfHeight_; }
	float getMaxPenetration() const { return maxPenetration_; }

	/** Exact signed distance to the capped cylinder; positive inside. */
	float getSignedDistance(const Math::Vector3df& pos) const override;
	bool isActiveAt(const Math::Vector3df& pos) const;
	bool isActiveAt(const Math::Vector3df& pos, float) const override { return isActiveAt(pos); }
	Math::Vector3df getBoundaryForce(const Math::Vector3df& pos, float timeStep) const override;
	Math::Vector3df getBoundaryForce(const Math::Vector3df& pos,
	                                 const Math::Vector3df& velocity,
	                                 float timeStep, float dampingRatio) const override;
	Math::Vector3df clampPosition(const Math::Vector3df& pos) const override;

private:
	Math::Vector3df closestPoint(const Math::Vector3df& pos) const;

	Math::Vector3df center_{ 0.f, 0.f, 0.f };
	Math::Vector3df axis_{ 0.f, 1.f, 0.f };
	float radius_{ 1.f };
	float halfHeight_{ 1.f };
	float maxPenetration_{ std::numeric_limits<float>::max() };
};

}
