#pragma once

#include "../../CGLib/Volume/Volume/SparseVolumeTree/SparseVolume.h"
#include "../../CGLib/Math/Triangle3d.h"

#include <cstdint>
#include <string>
#include <vector>

namespace Phantom {

/**
 * @brief Converts a Volume::SparseVolumef (as produced by FluidVolumeConverter)
 * into a triangle mesh via Volume::MCSurfaceBuilder (Marching Cubes), and
 * optionally writes the result out as an .obj file -- the live-simulation
 * counterpart to FluidStudio's offline VDB->Mesh conversion
 * (CGApp/FluidStudio/FSVDBMeshConverter).
 */
class FluidMeshConverter {
public:
    struct Params {
        float isoLevel = 0.5f;
    };

    Params&       params()       { return params_; }
    const Params& params() const { return params_; }

    /**
     * @brief Runs Marching Cubes over volume at the current params().isoLevel.
     * Returns false (and sets lastError()) if volume has no active voxels or
     * the surface extraction produces no triangles; the mesh buffers are
     * cleared to empty in that case.
     */
    bool convert(const Phantom::Volume::SparseVolumef& volume);

    size_t getTriangleCount() const { return triangles_.size(); }

    // Flattened triangle-soup buffers ready for FluidMeshRenderer
    // (Phantom::VKG::VkTriangleRenderer::Buffer layout: positions xyz*3,
    // colors rgba*3, one index per vertex per triangle).
    const std::vector<float>&    getPositions() const { return positions_; }
    const std::vector<float>&    getColors()    const { return colors_; }
    const std::vector<uint32_t>& getIndices()   const { return indices_; }

    /**
     * @brief Writes the last converted mesh to filePath as an .obj file.
     * Returns false (and sets lastError()) if convert() has not yet
     * succeeded or the write itself fails.
     */
    bool saveToObj(const std::string& filePath);

    const std::string& lastError() const { return lastError_; }

private:
    Params params_;
    std::vector<Phantom::Math::Triangle3df> triangles_;

    std::vector<float>    positions_;
    std::vector<float>    colors_;
    std::vector<uint32_t> indices_;

    std::string lastError_;
};

} // namespace Phantom
