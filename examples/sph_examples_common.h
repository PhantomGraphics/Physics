// sph_examples_common.h -- tiny helpers shared by the Phantom::Physics SPH
// example programs. Header-only, no GUI, no Vulkan: just the standard library
// plus Phantom's math types.
//
// Nothing here is part of the Physics library's public API -- it exists only
// to keep the example .cpp files short. Copy what you need into your own code.
#pragma once

#include "CGLib/Math/Vector3d.h"
#include "CGLib/Math/Box3d.h"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace sph_examples {

using Phantom::Math::Vector3df;
using Phantom::Math::Box3df;

// ---------------------------------------------------------------------------
// Particle seeding
// ---------------------------------------------------------------------------

// Fills an axis-aligned box with a regular cubic lattice of points, one point
// every `spacing` units. This is the layout SPH's rest density is calibrated
// against, so it is what every "block of fluid" initial condition should use.
// `spacing` is normally the particle diameter (2 * radius).
inline std::vector<Vector3df> seedBox(const Box3df& region, float spacing)
{
    std::vector<Vector3df> out;
    if (spacing <= 0.0f) return out;

    const Vector3df lo = region.getMin();
    const Vector3df hi = region.getMax();
    // + a small epsilon so a box whose extent is an exact multiple of spacing
    // still includes its far face.
    const float eps = spacing * 1e-3f;
    for (float x = lo.x; x <= hi.x + eps; x += spacing)
        for (float y = lo.y; y <= hi.y + eps; y += spacing)
            for (float z = lo.z; z <= hi.z + eps; z += spacing)
                out.emplace_back(x, y, z);
    return out;
}

// ---------------------------------------------------------------------------
// PLY output (binary little-endian, positions only)
// ---------------------------------------------------------------------------
//
// A frame of particle positions written as a point-cloud PLY. Blender,
// MeshLab, Houdini, CloudCompare, Open3D, etc. all read this directly, so it
// is the simplest way to get an SPH bake out of a headless run and look at it.

inline bool writePLY(const std::filesystem::path& path,
                     const std::vector<Vector3df>& positions)
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
        << "property float x\nproperty float y\nproperty float z\n"
        << "end_header\n";

    std::vector<char> blob(positions.size() * sizeof(float) * 3);
    char* out = blob.data();
    for (const auto& p : positions) {
        const float rec[3] = { p.x, p.y, p.z };
        std::memcpy(out, rec, sizeof(rec));
        out += sizeof(rec);
    }
    ofs.write(blob.data(), static_cast<std::streamsize>(blob.size()));
    return static_cast<bool>(ofs);
}

// Writes <dir>/<prefix>_0000.ply, _0001.ply, ... -- a numbered sequence a
// DCC tool can import as an animation.
inline bool writePLYFrame(const std::filesystem::path& dir,
                          const std::string& prefix,
                          int frame,
                          const std::vector<Vector3df>& positions)
{
    char name[64];
    std::snprintf(name, sizeof(name), "%s_%04d.ply", prefix.c_str(), frame);
    return writePLY(dir / name, positions);
}

// ---------------------------------------------------------------------------
// Misc
// ---------------------------------------------------------------------------

struct Stopwatch {
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    double seconds() const
    {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - t0).count();
    }
};

// max |v| over a velocity list -- a cheap "is the solver still stable?" probe.
inline float maxSpeed(const std::vector<Vector3df>& velocities)
{
    float m = 0.0f;
    for (const auto& v : velocities)
        m = std::max(m, Phantom::Math::getLength(v));
    return m;
}

// min / max of one component over a position list.
inline float minY(const std::vector<Vector3df>& p)
{
    float m = 1e30f;
    for (const auto& v : p) m = std::min(m, v.y);
    return p.empty() ? 0.0f : m;
}
inline float maxY(const std::vector<Vector3df>& p)
{
    float m = -1e30f;
    for (const auto& v : p) m = std::max(m, v.y);
    return p.empty() ? 0.0f : m;
}

} // namespace sph_examples
