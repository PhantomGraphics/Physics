#pragma once

#include "CGLib/Math/Vector3d.h"
#include "CGLib/Math/Matrix3d.h"

#include <cstddef>
#include <vector>

namespace Phantom {
	namespace Physics {

/**
 * @brief Parameters of the Yu & Turk (2013) anisotropic particle kernel.
 *
 * Shared by the volume conversion (SPHSurfaceParticle / SPHVolumeConverter)
 * and the screen-space fluid renderer (ellipsoid splats), so both use one
 * definition of a particle's ellipsoid.
 */
struct AnisotropyParams
{
	/// Neighbourhood radius used for the weighted PCA (world units).
	float searchRadius = 1.0f;
	/// k_r: largest principal stretch / smallest allowed stretch.
	float maxRatio = 4.0f;
	/// k_n: isotropic stretch of particles with too few neighbours.
	float isolatedScale = 0.5f;
	/// N_eps: minimum neighbourhood size (including the particle itself)
	/// for the PCA to be trusted.
	int minNeighbors = 25;
	/// lambda: blend of each particle toward its weighted neighbourhood mean
	/// (Laplacian smoothing of the kernel centre). 0 keeps the positions.
	float smoothing = 0.95f;
};

/**
 * @brief Principal frame of one anisotropic kernel.
 *
 * The ellipsoid is { v : |diag(1/sigma) R^T v| <= h } -- i.e. it extends
 * h * sigma[k] along the k-th column of `rotation`. sigma is dimensionless
 * (unit product for a trusted neighbourhood, isolatedScale otherwise), so a
 * uniform neighbourhood gives sigma = (1,1,1).
 */
struct AnisotropyFrame
{
	Math::Matrix3dd rotation;  ///< Columns = principal axes (orthonormal).
	Math::Vector3dd sigma;     ///< Stretch along each column of `rotation`.
};

/**
 * @brief Derives the kernel frame from a weighted covariance matrix.
 * @param covariance    Weighted covariance of the neighbourhood.
 * @param neighborCount Neighbourhood size including the particle itself.
 */
AnisotropyFrame anisotropyFromCovariance(const Math::Matrix3dd& covariance, std::size_t neighborCount, const AnisotropyParams& params);

/// G = R diag(1/sigma) R^T: maps a world offset v to the distance |Gv| fed to
/// an isotropic SPH kernel (what SPHVolumeConverter uses).
Math::Matrix3dd toKernelMatrix(const AnisotropyFrame& frame);

/// T = R diag(sigma): maps the unit sphere onto the (dimensionless) ellipsoid.
/// Columns are the scaled principal axes (what the ellipsoid splats use).
Math::Matrix3df toEllipsoidAxes(const AnisotropyFrame& frame);

/// Weight of a neighbour at `distance` (WPCA's 1 - (d/r)^3, 0 beyond r).
float anisotropyWeight(float distance, float radius);

struct AnisotropyResult
{
	/// Smoothed kernel centres (particle positions blended toward their mean).
	std::vector<Math::Vector3df> centers;
	/// Per-particle ellipsoid axes T = R diag(sigma) (dimensionless).
	std::vector<Math::Matrix3df> axes;
	/// Mean over particles of sigma_max / sigma_min (1 = all spheres).
	float meanStretchRatio = 1.0f;
	/// Number of particles whose neighbourhood was large enough for PCA.
	std::size_t anisotropicCount = 0;
};

/**
 * @brief Computes the anisotropic kernel of every particle.
 *
 * Neighbours come from Space::CSRNeighborList (strictly inside
 * params.searchRadius); the particle itself is part of its own neighbourhood,
 * matching SPHVolumeConverter's CompactSpaceHash query. The PCA and weighted
 * mean read only the input positions and the smoothed centres go to a
 * separate array, so the result does not depend on thread scheduling.
 */
void computeAnisotropy(const std::vector<Math::Vector3df>& positions, const AnisotropyParams& params, AnisotropyResult& out);

	}
}
