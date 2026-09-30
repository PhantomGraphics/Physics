#pragma once

#include "CloudParticle.h"
#include "CGLib/Volume/Volume/ScalarGrid3D.h"

namespace Phantom {
	namespace Physics {

/** @brief Result of a density reconstruction and its mass check. */
struct CloudDensityStats {
	double particleMass = 0.0;   ///< sum(mDry*qc) over the particles [kg]
	double gridMass = 0.0;       ///< sum(rho*cell^3) over the grid [kg]
	double relativeError = 0.0;  ///< |gridMass - particleMass| / particleMass (0 if no cloud water)
	size_t cloudyParticles = 0;  ///< particles with qc > 0
};

/**
 * @brief Cloud-water density field for rendering (docs/todo/PLAN_cloud_sph_pbvr.md section 5):
 *   rhoCloud(x) = sum_i mDry_i * qc_i * W(|x - x_i|, h)   [kg/m^3]
 * with the normalized cubic spline W (integral 1), so the grid integral equals the cloud-water
 * mass except where the kernel is cut by the grid boundary or by cell-sampling error (reported
 * by CloudDensityStats, not hidden). The grid covers [domainMin, domainMax] with cubic cells of
 * size max(extent)/resolution and is filled at cell centres.
 */
namespace CloudDensity {

/** @brief Grid geometry for the domain at the given resolution along the longest axis. */
Volume::ScalarGridDesc makeGridDesc(const Math::Vector3dd& domainMin, const Math::Vector3dd& domainMax,
                                    uint32_t resolution);

/**
 * @brief Splats every particle with qc > 0 into `out` (which is cleared and given the desc).
 * `supportRadius` is the kernel support [m]; values below 2 cells are raised to 2 cells so the
 * cell-centre sampling stays accurate. Returns the mass check.
 */
CloudDensityStats reconstruct(const CloudParticleSoA& soa, const Volume::ScalarGridDesc& desc,
                              double supportRadius, Volume::ScalarGrid3D& out);

}  // namespace CloudDensity

	}
}
