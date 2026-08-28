#pragma once

#include "CGLib/Math/Vector3d.h"
#include "CGLib/Math/Box3d.h"

namespace Phantom {
	namespace Physics {
		class SPHKernel;

/**
 * @brief Density map-based boundary representation.
 *
 * Computes boundary density contributions using a signed-distance
 * or positional query against an axis-aligned box boundary.
 */
class DMBoundary
{
public:
	/**
	 * @brief Sets the axis-aligned box used as the boundary region.
	 * @param box The bounding box that defines the boundary.
	 */
	void addBox(const Math::Box3df& box) { this->boundary = box; }

	/**
	 * @brief Calculates the boundary weight at a given position.
	 * @param position World-space position of the query point.
	 * @param r        Effect radius of the SPH kernel.
	 * @param restDensity Rest density of the fluid.
	 * @param kernel   SPH kernel used for weighting.
	 * @return Computed boundary weight.
	 */
	float calculateWeight(const Math::Vector3df& position, const float r, const float restDensity, const SPHKernel& kernel);

	/**
	 * @brief Calculates the boundary weight from a precomputed signed distance.
	 * @param signedDistance Signed distance from the boundary surface.
	 * @param r             Effect radius of the SPH kernel.
	 * @param restDensity   Rest density of the fluid.
	 * @param kernel        SPH kernel used for weighting.
	 * @return Computed boundary weight.
	 */
	float calculateWeight(const float signedDistance, const float r, const float restDensity, const SPHKernel& kernel);

	/**
	 * @brief Calculates the boundary density contribution along one axis.
	 * @param x           Distance along the axis from the boundary.
	 * @param r           Effect radius of the SPH kernel.
	 * @param restDensity Rest density of the fluid.
	 * @return Density contribution from the boundary.
	 */
	float calculateDensity(const float x, const float r, const float restDensity);

private:
	Math::Box3df boundary;
};
	}
}
