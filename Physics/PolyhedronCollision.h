#pragma once

#include "CollisionPair.h"
#include "CGLib/Math/Quaternion.h"

#include <vector>

namespace Phantom {
namespace Physics {

struct BoxShape;
struct ConvexHullShape;

// A convex polyhedron in world space, the common input of the SAT +
// face-clipping contact routine used by box/convex-hull/triangle pairs
// (NarrowPhase). A triangle is a flat polyhedron with two opposite faces.
struct WorldPolyhedron {
    struct Face {
        Math::Vector3df  normal;  // outward, unit
        float            d = 0.f; // dot(normal, x) == d on the face
        std::vector<int> loop;    // counter-clockwise around normal
    };
    std::vector<Math::Vector3df>     verts;
    std::vector<Face>                faces;
    std::vector<std::pair<int, int>> edges;
    std::vector<Math::Vector3df>     edgeDirs; // unit, pairwise non-parallel
    Math::Vector3df                  center = { 0.f, 0.f, 0.f };

    static WorldPolyhedron fromBox(const BoxShape& box, const Math::Vector3df& pos,
                                   const Math::Quaternion& orient);
    static WorldPolyhedron fromHull(const ConvexHullShape& hull, const Math::Vector3df& pos,
                                    const Math::Quaternion& orient);
    static WorldPolyhedron fromTriangle(const Math::Vector3df& a, const Math::Vector3df& b,
                                        const Math::Vector3df& c);
};

// Separating-axis test over A's faces, B's faces and every edge-direction
// pair. On overlap appends up to 4 contacts to `out` (face contacts come from
// clipping the incident face against the reference face) with the
// ContactPoint convention: normal points from B toward A. Returns whether any
// contact was added.
bool collidePolyhedra(const WorldPolyhedron& a, const WorldPolyhedron& b,
                      std::vector<ContactPoint>& out);

// Keeps at most 4 of `contacts` (deepest, then the ones spanning the largest
// area in the plane perpendicular to `normal`).
void reduceContacts(std::vector<ContactPoint>& contacts, const Math::Vector3df& normal);

} // namespace Physics
} // namespace Phantom
