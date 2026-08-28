#include "pch.h"
#include "FluidMeshConverter.h"

#include "../../CGLib/Volume/Volume/MCSurfaceBuilder.h"
#include "../../CGLib/File/File/OBJFile.h"
#include "../../CGLib/File/File/OBJFileWriter.h"

#include <filesystem>

namespace Phantom {

bool FluidMeshConverter::convert(const Phantom::Volume::SparseVolumef& volume)
{
    lastError_.clear();
    triangles_.clear();
    positions_.clear();
    colors_.clear();
    indices_.clear();

    if (volume.getActiveVoxelCount() == 0) {
        lastError_ = "volume has no active voxels";
        return false;
    }

    Phantom::Volume::MCSurfaceBuilder builder;
    builder.build(volume, params_.isoLevel);
    triangles_ = builder.getTriangles();

    if (triangles_.empty()) {
        lastError_ = "marching cubes produced no triangles";
        return false;
    }

    positions_.reserve(triangles_.size() * 9);
    colors_.reserve(triangles_.size() * 12);
    indices_.reserve(triangles_.size() * 3);

    uint32_t idx = 0;
    for (const auto& tri : triangles_) {
        const auto verts = tri.getVertices();
        for (int vi = 0; vi < 3; ++vi) {
            positions_.push_back(verts[vi].x);
            positions_.push_back(verts[vi].y);
            positions_.push_back(verts[vi].z);
            colors_.push_back(0.8f);
            colors_.push_back(0.8f);
            colors_.push_back(0.9f);
            colors_.push_back(0.85f);
            indices_.push_back(idx++);
        }
    }
    return true;
}

bool FluidMeshConverter::saveToObj(const std::string& filePath)
{
    if (triangles_.empty()) {
        lastError_ = "no mesh to save -- call convert() first";
        return false;
    }

    Phantom::File::OBJFile obj;
    Phantom::File::OBJGroup group;
    group.name = "mesh";

    for (const auto& tri : triangles_) {
        const auto verts = tri.getVertices();
        const auto norm  = tri.getNormal();

        const auto base = static_cast<unsigned int>(obj.positions.size());
        const int  ni   = static_cast<int>(obj.normals.size());

        for (const auto& v : verts)
            obj.positions.push_back(v);
        obj.normals.push_back(norm);

        Phantom::File::OBJFace face;
        face.positionIndices = { base, base + 1, base + 2 };
        face.normalIndices   = { ni, ni, ni };
        face.texCoordIndices = { 0, 0, 0 };
        group.faces.push_back(std::move(face));
    }

    obj.groups.push_back(std::move(group));

    std::error_code ec;
    std::filesystem::create_directories(
        std::filesystem::path(filePath).parent_path(), ec);

    Phantom::File::OBJFileWriter writer;
    if (!writer.write(filePath, obj)) {
        lastError_ = "failed to write '" + filePath + "'";
        return false;
    }
    return true;
}

} // namespace Phantom
