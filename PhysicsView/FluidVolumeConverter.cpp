#include "pch.h"
#include "FluidVolumeConverter.h"

namespace Phantom {

bool FluidVolumeConverter::convert(const std::vector<glm::vec3>& positions)
{
    lastError_.clear();
    volume_.reset();

    if (positions.empty()) {
        lastError_ = "no particles to convert";
        return false;
    }

    std::vector<Phantom::Math::Vector3df> pts;
    pts.reserve(positions.size());
    for (const auto& p : positions)
        pts.emplace_back(p.x, p.y, p.z);

    volume_ = (params_.kernelType == KernelType::Anisotropic)
        ? converter_.buildAnisotoropic(pts, params_.particleRadius, params_.cellLength)
        : converter_.buildIsotoropic(pts, params_.particleRadius, params_.cellLength);

    if (!volume_) {
        lastError_ = "volume construction produced no active voxels";
        return false;
    }
    return true;
}

std::vector<glm::vec3> FluidVolumeConverter::getVoxelPositions() const
{
    std::vector<glm::vec3> out;
    if (!volume_) return out;

    out.reserve(static_cast<size_t>(volume_->getActiveVoxelCount()));
    volume_->forEachActive([&](const Phantom::Volume::Coord&,
                               const Phantom::Math::Vector3df& worldPos,
                               const float&) {
        out.emplace_back(worldPos.x, worldPos.y, worldPos.z);
    });
    return out;
}

bool FluidVolumeConverter::saveToVdb(const std::string& filePath)
{
    if (!volume_) {
        lastError_ = "no volume to save -- call convert() first";
        return false;
    }

    std::error_code ec;
    std::filesystem::create_directories(
        std::filesystem::path(filePath).parent_path(), ec);

    Phantom::Volume::SparseVolumeVdbWriter writer;
    if (!writer.write(filePath, *volume_, params_.gridName)) {
        lastError_ = "failed to write '" + filePath + "'";
        return false;
    }
    return true;
}

} // namespace Phantom
