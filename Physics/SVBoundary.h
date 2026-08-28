#pragma once

#include "../../CGLib/Math/Box3d.h"
#include "../../CGLib/Volume/Volume/SparseVolumeTree/SparseVolume.h"
#include <memory>

namespace Phantom {
	namespace Physics {

/**
 * @brief Signed-volume boundary representation backed by a sparse volume.
 *
 * Builds a signed-distance field inside a sparse volume for use as a
 * collision boundary in PBSPH simulations.
 */
class SVBoundary
{
public:
	SVBoundary() = default;

	/**
	 * @brief Builds the signed-volume boundary from an axis-aligned box.
	 * @param boundary   Box defining the interior boundary region.
	 * @param cellLength Side length of each voxel cell.
	 * @param tableSize  Hash table size for the sparse volume.
	 */
	void build(const Math::Box3df& boundary, const Math::Vector3df& cellLength, const int tableSize);

	/**
	 * @brief Returns a read-only pointer to the underlying sparse volume.
	 * @return Const pointer to the signed-distance sparse volume.
	 */
	const Volume::SparseVolumef* getVolume() const { return volume.get(); }

private:
	std::unique_ptr<Volume::SparseVolumef> volume;

};
	}
}
