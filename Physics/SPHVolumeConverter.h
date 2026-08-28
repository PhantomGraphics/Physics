#pragma once

#include <vector>
#include <memory>

#include "SPHSurfaceParticle.h"

#include "CGLib/Volume/Volume/SparseVolumeTree/SparseVolume.h"

namespace Phantom {
	namespace Physics {

/**
 * @brief Converts an SPH particle set into a sparse scalar volume.
 *
 * Supports both isotropic and anisotropic kernel-based volume construction
 * for surface reconstruction from SPH simulation data.
 * Both float and double position inputs are accepted; internally all
 * particle state is stored at float precision.
 */
class SPHVolumeConverter
{
public:
	// ---- float overloads (canonical implementation) -------------------------

	/**
	 * @brief Builds an isotropic sparse volume from float particle positions.
	 * @param positions      World-space particle positions (float precision).
	 * @param particleRadius Radius of each particle.
	 * @param cellLength     Side length of each voxel cell.
	 * @return Pointer to the constructed sparse volume, or nullptr if empty.
	 */
	std::unique_ptr<Volume::SparseVolumef> buildIsotoropic(const std::vector<Math::Vector3df>& positions, float particleRadius, float cellLength);

	/**
	 * @brief Builds an anisotropic sparse volume from float particle positions.
	 * @param positions      World-space particle positions (float precision).
	 * @param particleRadius Radius of each particle.
	 * @param cellLength     Side length of each voxel cell.
	 * @return Pointer to the constructed sparse volume, or nullptr if empty.
	 */
	std::unique_ptr<Volume::SparseVolumef> buildAnisotoropic(const std::vector<Math::Vector3df>& positions, float particleRadius, float cellLength);

	// ---- float overloads with pre-computed per-particle density -------------

	std::unique_ptr<Volume::SparseVolumef> buildIsotoropic(const std::vector<Math::Vector3df>& positions, const std::vector<float>& densities, float particleRadius, float cellLength);

	std::unique_ptr<Volume::SparseVolumef> buildAnisotoropic(const std::vector<Math::Vector3df>& positions, const std::vector<float>& densities, float particleRadius, float cellLength);

	// ---- double overloads (backward-compatible forwarding wrappers) ----------

	/**
	 * @brief Builds an isotropic sparse volume from double particle positions.
	 *        Converts to float and delegates to the float overload.
	 */
	std::unique_ptr<Volume::SparseVolumef> buildIsotoropic(const std::vector<Math::Vector3dd>& positions, float particleRadius, float cellLength);

	/**
	 * @brief Builds an anisotropic sparse volume from double particle positions.
	 *        Converts to float and delegates to the float overload.
	 */
	std::unique_ptr<Volume::SparseVolumef> buildAnisotoropic(const std::vector<Math::Vector3dd>& positions, float particleRadius, float cellLength);

	/**
	 * @brief Returns the internal surface particle list.
	 * @return Read-only reference to the vector of surface particles.
	 */
	const std::vector<std::unique_ptr<SPHSurfaceParticle>>& getParticles() const { return particles; }

private:
	std::unique_ptr<Volume::SparseVolumef> createSparseVolume(float cellLength);

	void calculateDensity(float searchRadius);

	void calculateAnisotropy(float searchRadius);

	std::vector<std::unique_ptr<SPHSurfaceParticle>> particles;
};

	}
}
