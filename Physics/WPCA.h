#pragma once

#include "CGLib/Math/Vector3d.h"
#include "CGLib/Math/Matrix3d.h"

#include <vector>

namespace Phantom {
	namespace Physics {

/**
 * @brief Weighted Principal Component Analysis (WPCA) utility.
 *
 * Computes weighted means and covariance matrices from a set of positions
 * using a radially-decaying weight function. Used for anisotropic
 * surface reconstruction in SPH simulations.
 */
class WPCA
{
public:
	/**
	 * @brief Computes the weighted covariance matrix of the given positions.
	 * @param center    Reference point used for weighting.
	 * @param positions Neighboring positions to include in the computation.
	 * @param radius    Influence radius for the weight function.
	 * @return Weighted covariance matrix (3ﾃ・, single precision).
	 */
	Math::Matrix3df calculateCovarianceMatrix(const Math::Vector3df& center, const std::vector<Math::Vector3df>& positions, const float radius);

	/**
	 * @brief Computes the weighted mean position of the given positions.
	 * @param center    Reference point used for weighting.
	 * @param positions Neighboring positions to average.
	 * @param radius    Influence radius for the weight function.
	 * @return Weighted mean position.
	 */
	Math::Vector3df calculateWeightedMean(const Math::Vector3df& center, const std::vector<Math::Vector3df>& positions, const float radius);

	/**
	 * @brief Computes the scalar weight between two points.
	 * @param lhs    First point.
	 * @param rhs    Second point.
	 * @param radius Influence radius; weight falls to zero at this distance.
	 * @return Weight value in [0, 1].
	 */
	float calculateWeight(const Math::Vector3df& lhs, const Math::Vector3df& rhs, const float radius);

	/**
	 * @brief Precomputes per-neighbor weights and stores them internally.
	 * @param center    Reference point used for weighting.
	 * @param positions Neighboring positions.
	 * @param radius    Influence radius for the weight function.
	 */
	void setup(const Math::Vector3df& center, const std::vector<Math::Vector3df>& positions, const float radius);

private:
	class PositionWeight
	{
	public:
		Math::Vector3df position;
		float weight;
	};

	std::vector<PositionWeight> pws;
	float totalWeight;
};

	}
}
