#pragma once

#include "CGLib/Util/UnCopyable.h"
#include "CGLib/Math/Vector3d.h"
#include "CGLib/Math/Matrix3d.h"

#include "SPHKernel.h"
#include <vector>

namespace Phantom {
	namespace Physics {

/**
 * @brief A surface particle used for anisotropic surface reconstruction.
 *
 * Stores per-particle position, density, and an anisotropy matrix
 * computed from the local neighborhood via weighted PCA.
 */
class SPHSurfaceParticle : private UnCopyable
{
public:
	/**
	 * @brief Constructs a surface particle.
	 * @param p      Initial world-space position.
	 * @param radius Particle radius.
	 */
	SPHSurfaceParticle(const Math::Vector3df& p, const float radius);

	/**
	 * @brief Returns the world-space position of this particle.
	 * @return Position vector.
	 */
	Math::Vector3df getPosition() const { return position; }

	/**
	 * @brief Applies a position correction toward the weighted mean of neighbors.
	 * @param lamda       Blending weight for the correction.
	 * @param weightedMean Weighted mean position of the neighborhood.
	 */
	void correctedPosition(const float lamda, const Math::Vector3df& weightedMean);

	/**
	 * @brief Computes the anisotropy matrix from the neighbor positions.
	 * @param neighbors    World-space positions of neighboring particles.
	 * @param searchRadius Radius used to weight neighbors.
	 */
	void calculateAnisotoropicMatrix(const std::vector<Math::Vector3df>& neighbors, const float searchRadius);

	/**
	 * @brief Computes the particle density from neighbor positions.
	 * @param neighbors    World-space positions of neighboring particles.
	 * @param searchRadius SPH support radius.
	 * @param kernel       SPH kernel used for density estimation.
	 */
	void calculateDensity(const std::vector<Math::Vector3df>& neighbors, const float searchRadius, const SPHKernel& kernel);

	/**
	 * @brief Returns the anisotropy matrix of this particle.
	 * @return 3ﾃ・ anisotropy matrix (double precision).
	 */
	Math::Matrix3dd getMatrix() const { return matrix; }

	/**
	 * @brief Returns the computed density of this particle.
	 * @return Density value.
	 */
	float getDensity() const { return density; }

	void setDensity(const float d) { density = d; }

	/**
	 * @brief Returns the mass of this particle derived from its radius.
	 * @return Mass value.
	 */
	float getMass() const;

private:
	Math::Vector3df position;
	Math::Matrix3dd matrix;
	float density;
	float radius;
};

	}
}
