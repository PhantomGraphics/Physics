#pragma once

#include "CGLib/Math/Vector3d.h"

#include <memory>
#include <utility>
#include <vector>

namespace Phantom::Physics {

/** Values that a solver commonly needs from one analytic-boundary query. */
struct ShapeBoundarySample {
	bool active = false;
	float signedDistance = 0.0f;
	float densityDistance = 0.0f;
	float densityWeight = 0.0f;
};

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

	/**
	 * Samples the mutually related distance/coverage values as one contract.
	 * Custom boundaries normally only need to implement the primitive virtual
	 * methods above; this default keeps them source-compatible.
	 */
	virtual ShapeBoundarySample sample(const Math::Vector3df& pos,
	                                  const float effectLength) const {
		ShapeBoundarySample result;
		result.active = isActiveAt(pos, effectLength);
		if (!result.active) return result;
		result.signedDistance = getSignedDistance(pos);
		result.densityDistance = getDensityDistance(pos);
		result.densityWeight = getDensityWeight(pos, effectLength);
		return result;
	}
};

/** Takes value-based boundary input and stores owned polymorphic copies. */
template <class Boundary>
std::vector<std::shared_ptr<IShapeBoundary>> ownShapeBoundaries(
	std::vector<Boundary> boundaries) {
	std::vector<std::shared_ptr<IShapeBoundary>> result;
	result.reserve(boundaries.size());
	for (auto& boundary : boundaries)
		result.push_back(std::make_shared<Boundary>(std::move(boundary)));
	return result;
}

}
