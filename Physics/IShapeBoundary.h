#pragma once

#include "CGLib/Math/Vector3d.h"

namespace Phantom::Physics {

/** Common contract for analytic simulation boundaries. */
class IShapeBoundary {
public:
	virtual ~IShapeBoundary() = default;

	/** >= 0 denotes the valid side of the boundary. */
	virtual float getSignedDistance(const Math::Vector3df& pos) const = 0;
	virtual Math::Vector3df getBoundaryForce(const Math::Vector3df& pos,
		const float timeStep) const = 0;
	virtual Math::Vector3df getBoundaryForce(const Math::Vector3df& pos,
		const Math::Vector3df& velocity, const float timeStep,
		const float dampingRatio) const = 0;
	virtual Math::Vector3df clampPosition(const Math::Vector3df& pos) const = 0;

	/** Cheap optional rejection used before evaluating a boundary. */
	virtual bool isActiveAt(const Math::Vector3df&, const float) const { return true; }

	/** Distance and coverage used by the analytic wall-density approximation. */
	virtual float getDensityDistance(const Math::Vector3df& pos) const {
		return getSignedDistance(pos);
	}
	virtual float getDensityWeight(const Math::Vector3df&, const float) const {
		return 1.0f;
	}
};

}
