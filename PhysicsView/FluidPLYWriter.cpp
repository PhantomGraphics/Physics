#include "pch.h"
#include "FluidPLYWriter.h"

#include <cstring>
#include <fstream>

namespace Phantom {

bool writeFluidParticlesToPLY(const std::filesystem::path& path,
                               const std::vector<glm::vec3>& positions)
{
    if (path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
    }

    std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
    if (!ofs) return false;

    ofs << "ply\n"
        << "format binary_little_endian 1.0\n"
        << "element vertex " << positions.size() << "\n"
        << "property float x\n"
        << "property float y\n"
        << "property float z\n"
        << "end_header\n";

    // One buffered pass rather than a stream call per particle -- matters at
    // the several-hundred-thousand-particle scale the showcase scenarios use
    // (mirrors CGApp/blender/PyFluid/PyFluid.cpp's writeFluidsToPLY()).
    std::vector<char> blob(positions.size() * sizeof(float) * 3);
    char* out = blob.data();
    for (const auto& p : positions) {
        const float record[3] = { p.x, p.y, p.z };
        std::memcpy(out, record, sizeof(record));
        out += sizeof(record);
    }
    ofs.write(blob.data(), static_cast<std::streamsize>(blob.size()));

    return static_cast<bool>(ofs);
}

} // namespace Phantom
