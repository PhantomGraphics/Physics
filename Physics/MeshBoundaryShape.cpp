#include "pch.h"
#include "MeshBoundaryShape.h"

#include "CGLib/Volume/Volume/LevelSet.h"
#include "CGLib/Volume/Volume/SparseVolumeTree/Interpolator.h"
#include "CGLib/ThirdParty/glm-0.9.9.8/glm/gtc/quaternion.hpp"

#include <array>

namespace Phantom {
namespace Physics {

using namespace Phantom::Math;
using namespace Phantom::Volume;

namespace {
// Sentinel signed distance for voxels never touched by LevelSet::setSignedDistance()
// (i.e. outside the mesh's padded AABB): a large-but-finite "far outside" value, so
// RigidBoundary's `d >= 0 -> no force` check trivially holds and trilinear blending
// near the AABB edge never mixes finite values with +/-infinity (which would produce
// NaN via 0 * inf when a blend weight is exactly zero).
constexpr float kFarOutside = 1.0e6f;
}

MeshBoundaryShape::MeshBoundaryShape(const std::vector<Triangle3df>& triangles, float voxelSize)
    : triangleCount_(triangles.size())
{
    volume_ = std::make_unique<SparseVolumef>(kFarOutside);
    volume_->setVoxelSize(voxelSize > 0.f ? voxelSize : 1.f);

    LevelSet builder;
    builder.setSignedDistance(triangles, *volume_);

    if (triangles.empty()) {
        localAABB_ = Box3df(Vector3df(0.f, 0.f, 0.f));
        return;
    }

    Box3df box(triangles.front().getVertices()[0]);
    for (const auto& tri : triangles) {
        for (const auto& v : tri.getVertices()) box.add(v);
    }
    localAABB_ = box;
}

float MeshBoundaryShape::getSignedDistance(const Vector3df& worldPoint,
                                            const Vector3df& pos,
                                            const Quaternion& orient) const
{
    const Vector3df local = glm::inverse(orient) * (worldPoint - pos);
    const TrilinearInterpolator<float> interp(*volume_);
    return interp.getValue(local);
}

Vector3df MeshBoundaryShape::getSurfaceNormal(const Vector3df& worldPoint,
                                               const Vector3df& pos,
                                               const Quaternion& orient) const
{
    const Vector3df local = glm::inverse(orient) * (worldPoint - pos);
    const TrilinearInterpolator<float> interp(*volume_);
    const Vector3df grad = interp.getGradient(local);

    const float len = Math::getLength(grad);
    const Vector3df localNormal = (len > 1e-8f) ? (grad / len) : Vector3df(0.f, 1.f, 0.f);
    return orient * localNormal;
}

Math::Box3df MeshBoundaryShape::getAABB(const Vector3df& pos, const Quaternion& orient) const
{
    const Vector3df mn = localAABB_.getMin();
    const Vector3df mx = localAABB_.getMax();
    const std::array<Vector3df, 8> corners = {
        Vector3df(mn.x, mn.y, mn.z), Vector3df(mx.x, mn.y, mn.z),
        Vector3df(mn.x, mx.y, mn.z), Vector3df(mx.x, mx.y, mn.z),
        Vector3df(mn.x, mn.y, mx.z), Vector3df(mx.x, mn.y, mx.z),
        Vector3df(mn.x, mx.y, mx.z), Vector3df(mx.x, mx.y, mx.z),
    };

    Box3df box(pos + orient * corners[0]);
    for (size_t i = 1; i < corners.size(); ++i) box.add(pos + orient * corners[i]);
    return box;
}

} // namespace Physics
} // namespace Phantom
