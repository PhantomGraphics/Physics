#pragma once

#include "ICollisionShape.h"

#include <vector>

namespace Phantom {
namespace Physics {

/**
 * @brief Static triangle-mesh collider for rigid bodies (terrain, ramps,
 * arbitrary level geometry).
 *
 * Static only: RigidBody::setMass() keeps a body with this shape at mass 0,
 * and NarrowPhase collides it with sphere / capsule / box / convex hull but
 * not with another mesh or a plane (static-static pairs never reach the
 * narrow phase anyway). Triangles are two-sided, so an open surface works.
 *
 * Unlike MeshBoundaryShape (ShapeType::Mesh, an SDF-sampled fluid boundary)
 * this keeps the exact triangles in shape-local space plus an AABB tree for
 * the per-pair triangle query.
 */
class TriangleMeshShape : public ICollisionShape {
public:
    // `indices` is a triangle list into `vertices`; out-of-range and
    // degenerate triangles are dropped. Returns false if nothing remains.
    bool build(const std::vector<Math::Vector3df>& vertices,
               const std::vector<uint32_t>& indices);

    size_t getTriangleCount() const { return tris_.size(); }
    const std::vector<Math::Vector3df>& getVertices() const { return verts_; }
    const std::array<uint32_t, 3>& getTriangle(size_t i) const { return tris_[i]; }
    const Math::Box3df& getLocalBounds() const { return localBounds_; }

    // Indices of the triangles whose local AABB overlaps `localBox`.
    void queryTriangles(const Math::Box3df& localBox, std::vector<int>& out) const;

    ShapeType getType() const override { return ShapeType::TriangleMesh; }

    Math::Box3df getAABB(const Math::Vector3df& pos,
                         const Math::Quaternion& orient) const override;

    // Unsigned distance to the nearest triangle (the mesh need not be closed,
    // so there is no inside).
    float getSignedDistance(const Math::Vector3df& worldPoint,
                            const Math::Vector3df& pos,
                            const Math::Quaternion& orient) const override;

    Math::Vector3df getSurfaceNormal(const Math::Vector3df& worldPoint,
                                     const Math::Vector3df& pos,
                                     const Math::Quaternion& orient) const override;

private:
    struct Node {
        Math::Box3df box;
        int left = -1, right = -1; // children, -1 for a leaf
        int first = 0, count = 0;  // leaf: range in order_
    };

    int buildNode(int first, int count, int depth);
    int nearestTriangle(const Math::Vector3df& localPoint, Math::Vector3df& closest) const;

    std::vector<Math::Vector3df>          verts_;
    std::vector<std::array<uint32_t, 3>>  tris_;
    std::vector<Math::Box3df>             triBoxes_;
    std::vector<int>                      order_;
    std::vector<Node>                     nodes_;
    Math::Box3df                          localBounds_;
};

} // namespace Physics
} // namespace Phantom
