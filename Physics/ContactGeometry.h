#pragma once

#include "CGLib/Math/Vector3d.h"

namespace Phantom {
namespace Physics {

// Closest-point queries shared by the convex-hull and triangle-mesh narrow
// phase (Ericson, Real-Time Collision Detection 5.1).

// Closest point to `p` on triangle (a, b, c). Degenerate triangles are fine.
Math::Vector3df closestPointOnTriangle(const Math::Vector3df& p,
                                       const Math::Vector3df& a,
                                       const Math::Vector3df& b,
                                       const Math::Vector3df& c);

// Closest points c1 on segment [p1, q1] and c2 on segment [p2, q2].
void closestPointsSegmentSegment(const Math::Vector3df& p1, const Math::Vector3df& q1,
                                 const Math::Vector3df& p2, const Math::Vector3df& q2,
                                 Math::Vector3df& c1, Math::Vector3df& c2);

// Closest points onSeg on segment [p, q] and onTri on triangle (a, b, c).
// When the segment pierces the triangle both are the piercing point.
void closestPointsSegmentTriangle(const Math::Vector3df& p, const Math::Vector3df& q,
                                  const Math::Vector3df& a, const Math::Vector3df& b,
                                  const Math::Vector3df& c,
                                  Math::Vector3df& onSeg, Math::Vector3df& onTri);

} // namespace Physics
} // namespace Phantom
