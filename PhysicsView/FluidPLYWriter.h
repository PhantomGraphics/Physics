#pragma once

#include <glm/vec3.hpp>

#include <filesystem>
#include <vector>

namespace Phantom {

/**
 * @brief Writes fluid particle positions to a binary-little-endian PLY point
 * cloud (x,y,z float32 only -- no color/normal/velocity/density). Creates
 * the parent directory if it doesn't exist yet.
 *
 * Deliberately minimal compared to CGApp/blender/PyFluid/PyFluid.cpp's own
 * writeFluidsToPLY() (which also carries velocity/density/fluid_id so a bake
 * can resume from it): PhysicsView's own PLY export exists to give the
 * native showcase scenarios (scenarios/showcase/*.json) an inspectable
 * per-frame point cloud, not a resumable bake checkpoint. Position-only is
 * enough for that and keeps this independent of any *Fluid's internal SoA
 * layout.
 *
 * @return False if the file couldn't be opened/written.
 */
bool writeFluidParticlesToPLY(const std::filesystem::path& path,
                               const std::vector<glm::vec3>& positions);

} // namespace Phantom
