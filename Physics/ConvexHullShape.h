#pragma once

#include "ICollisionShape.h"
#include "CGLib/Math/Matrix3d.h"

#include <vector>

namespace Phantom {
namespace Physics {

/**
 * @brief Convex polyhedron collider built from a point cloud (typically a
 * render mesh's vertices -- the same thing Blender's "Convex Hull" rigid-body
 * shape uses).
 *
 * build() computes the hull, merges coplanar triangles into polygon faces
 * (needed for stable multi-point face contacts), and recenters everything on
 * the hull's center of mass: the shape's local origin IS the center of mass,
 * so RigidBody::position is the body's COM like every other shape. The offset
 * that was removed is reported by getCenterOffset() so a caller that wants the
 * hull to stay where its points were places the body at
 * `origin + orient * getCenterOffset()`.
 *
 * Hulls are capped at `maxVertices` hull vertices (SAT cost grows with the
 * square of the edge count): a larger hull is reduced to its support points
 * along evenly spread directions, which keeps its silhouette.
 */
struct ConvexHullShape : ICollisionShape {
    struct Face {
        Math::Vector3df  normal;   // outward, unit
        float            d = 0.f;  // plane: dot(normal, x) == d
        std::vector<int> loop;     // vertex indices, counter-clockwise around `normal`
    };

    static constexpr int kDefaultMaxVertices = 64;

    // Returns false (and leaves the shape empty) when the points do not span a
    // volume: fewer than 4 points, or all (nearly) coplanar.
    bool build(const std::vector<Math::Vector3df>& points,
               int maxVertices = kDefaultMaxVertices);

    bool isValid() const { return !faces.empty(); }

    const std::vector<Math::Vector3df>& getVertices()  const { return vertices; }
    const std::vector<Face>&            getFaces()     const { return faces; }
    const std::vector<Math::Vector3df>& getEdgeDirections() const { return edgeDirs; }
    const std::vector<std::pair<int, int>>& getEdges() const { return edges; }
    const std::vector<std::array<int, 3>>& getTriangles() const { return triangles; }
    const Math::Vector3df& getCenterOffset() const { return centerOffset; }
    float getVolume() const { return volume; }
    // Inertia tensor about the center of mass for unit mass (scale by mass).
    const Math::Matrix3df& getUnitInertia() const { return unitInertia; }

    ShapeType getType() const override { return ShapeType::ConvexHull; }

    Math::Box3df getAABB(const Math::Vector3df& pos,
                         const Math::Quaternion& orient) const override;

    // Exact inside (distance to the nearest face plane); exact outside
    // (distance to the nearest surface triangle).
    float getSignedDistance(const Math::Vector3df& worldPoint,
                            const Math::Vector3df& pos,
                            const Math::Quaternion& orient) const override;

    Math::Vector3df getSurfaceNormal(const Math::Vector3df& worldPoint,
                                     const Math::Vector3df& pos,
                                     const Math::Quaternion& orient) const override;

    // Local-space closest surface point to `localPoint`. `normal` is the
    // outward direction from the surface toward the point (the nearest face's
    // normal when the point is inside); returns the signed distance.
    float closestLocal(const Math::Vector3df& localPoint,
                       Math::Vector3df& surfacePoint, Math::Vector3df& normal) const;

private:
    std::vector<Math::Vector3df>     vertices;
    std::vector<Face>                faces;
    std::vector<std::array<int, 3>>  triangles; // hull surface, for closest-point queries
    std::vector<std::pair<int, int>> edges;     // unique polygon edges
    std::vector<Math::Vector3df>     edgeDirs;  // unique (non-parallel) edge directions
    Math::Vector3df centerOffset = { 0.f, 0.f, 0.f };
    float           volume = 0.f;
    Math::Matrix3df unitInertia = Math::Matrix3df(0.f);
};

} // namespace Physics
} // namespace Phantom
