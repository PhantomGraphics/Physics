#pragma once

#include "../Physics/SPHVolumeConverter.h"

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

namespace Phantom {

/**
 * @brief Converts the fluid simulation's live particle set into a
 * Volume::SparseVolumef via Physics::SPHVolumeConverter for live rendering
 * and subsequent mesh conversion.
 */
class FluidVolumeConverter {
public:
    enum class KernelType { Isotropic, Anisotropic };

    struct Params {
        float       particleRadius = 0.025f;
        float       cellLength     = 0.05f;
        KernelType  kernelType     = KernelType::Isotropic;
    };

    Params&       params()       { return params_; }
    const Params& params() const { return params_; }

    /**
     * @brief Builds a SparseVolume from the given world-space particle
     * positions using the current params(). Returns false (and sets
     * lastError()) if positions is empty or the resulting volume has no
     * active voxels; getVolume() is cleared to nullptr in that case.
     */
    bool convert(const std::vector<glm::vec3>& positions);

    const Phantom::Volume::SparseVolumef* getVolume() const { return volume_.get(); }
    int getVoxelCount() const { return volume_ ? volume_->getActiveVoxelCount() : 0; }

    /**
     * @brief World-space positions of every active voxel in the last
     * converted volume, for point-cloud rendering (FluidVolumeRenderer).
     * Empty if convert() has not yet succeeded.
     */
    std::vector<glm::vec3> getVoxelPositions() const;

    const std::string& lastError() const { return lastError_; }

private:
    Params params_;
    Phantom::Physics::SPHVolumeConverter converter_;
    std::unique_ptr<Phantom::Volume::SparseVolumef> volume_;
    std::string lastError_;
};

} // namespace Phantom
