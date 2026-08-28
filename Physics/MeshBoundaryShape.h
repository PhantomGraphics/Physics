#pragma once

#include "ICollisionShape.h"
#include "CGLib/Math/Triangle3d.h"
#include "CGLib/Volume/Volume/SparseVolumeTree/SparseVolume.h"

#include <memory>
#include <vector>

namespace Phantom {
namespace Physics {

/**
 * @brief A collision shape built from an arbitrary closed triangle mesh,
 * backed by a voxelized signed-distance field (Phantom::Volume::LevelSet +
 * SparseVolume, sampled with trilinear interpolation).
 *
 * Intended for static/kinematic fluid boundaries only: plug into
 * RigidBoundary for One-Way SDF penalty coupling
 * (ISPHSolver::addRigidBoundary()), which every CPU fluid solver
 * (DFSPH/PBSPH/WCSPH/MVC) already implements. There is no mesh-vs-mesh or
 * mesh-vs-sphere/box narrow-phase routine (see NarrowPhase.cpp), so this
 * shape must not be attached to a simulated RigidBody for rigid-rigid
 * collision.
 */
class MeshBoundaryShape : public ICollisionShape {
public:
    /**
     * @param triangles Triangles forming the mesh surface, in the shape's own
     *                  local (rest) frame -- getSignedDistance()'s pos/orient
     *                  parameters place this frame in the world, exactly like
     *                  SphereShape/BoxShape. Does not need to be a closed
     *                  manifold in the strict sense, but LevelSet's ray-parity
     *                  inside/outside test (see LevelSet::setSignedDistance())
     *                  assumes a closed, non-self-intersecting surface.
     * @param voxelSize Voxel edge length used to build the signed-distance
     *                  field. Smaller values increase accuracy at higher
     *                  memory/build cost. Values <= 0 are clamped to 1.f.
     */
    MeshBoundaryShape(const std::vector<Math::Triangle3df>& triangles, float voxelSize);

    ShapeType getType() const override { return ShapeType::Mesh; }

    Math::Box3df getAABB(const Math::Vector3df& pos,
                         const Math::Quaternion& orient) const override;

    float getSignedDistance(const Math::Vector3df& worldPoint,
                            const Math::Vector3df& pos,
                            const Math::Quaternion& orient) const override;

    Math::Vector3df getSurfaceNormal(const Math::Vector3df& worldPoint,
                                     const Math::Vector3df& pos,
                                     const Math::Quaternion& orient) const override;

    /** @brief Number of triangles the signed-distance field was built from (diagnostic). */
    size_t getTriangleCount() const { return triangleCount_; }

private:
    std::unique_ptr<Volume::SparseVolumef> volume_;
    Math::Box3df localAABB_;
    size_t triangleCount_ = 0;
};

} // namespace Physics
} // namespace Phantom
