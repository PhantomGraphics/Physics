#pragma once

#include "../Physics/SPHVolumeConverter.h"
#include "../../CGLib/Volume/Volume/SparseVolumeTree/VdbWriter.h"

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

namespace Phantom {

/**
 * @brief Converts the fluid simulation's live particle set into a
 * Volume::SparseVolumef via Physics::SPHVolumeConverter, and optionally
 * writes the result out as a .vdb file.
 *
 * Uses Volume::SparseVolumeVdbWriter (the dependency-free writer that lives
 * alongside SparseVolume itself) rather than CGApp/VDBIO's compressed
 * VdbWriter -- PhysicsView sits below CGApp in the module dependency graph
 * (see ../CLAUDE.md) and must not depend back up into it. FluidStudio's
 * offline PLY->VDB conversion (CGApp/FluidStudio/FSPLYtoVDBConverter) uses
 * the CGApp/VDBIO writer instead since it already lives above both.
 */
class FluidVolumeConverter {
public:
    enum class KernelType { Isotropic, Anisotropic };

    struct Params {
        float       particleRadius = 0.025f;
        float       cellLength     = 0.05f;
        KernelType  kernelType     = KernelType::Isotropic;
        std::string gridName       = "density";
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

    /**
     * @brief Writes the last converted volume to filePath as a .vdb file.
     * Returns false (and sets lastError()) if convert() has not yet
     * succeeded or the write itself fails.
     */
    bool saveToVdb(const std::string& filePath);

    const std::string& lastError() const { return lastError_; }

private:
    Params params_;
    Phantom::Physics::SPHVolumeConverter converter_;
    std::unique_ptr<Phantom::Volume::SparseVolumef> volume_;
    std::string lastError_;
};

} // namespace Phantom
