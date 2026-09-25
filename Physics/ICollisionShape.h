#pragma once

#include "CGLib/Math/Vector3d.h"
#include "CGLib/Math/Quaternion.h"
#include "CGLib/Math/Box3d.h"
#include "PlaneBoundary.h"

#include <array>

namespace Phantom {
namespace Physics {

// Mesh: static fluid-boundary shape only (MeshBoundaryShape). No mesh-vs-X
// narrow-phase routine exists, so it must not be attached to a simulated
// RigidBody for rigid-rigid collision (see NarrowPhase.cpp).
// Capsule is appended after Mesh so existing enumerator values stay unchanged.
enum class ShapeType { Sphere, Box, Plane, Mesh, Capsule };

struct ICollisionShape {
    virtual ShapeType    getType()                                            const = 0;
    virtual Math::Box3df getAABB(const Math::Vector3df& pos,
                                 const Math::Quaternion& orient)              const = 0;

    virtual float getSignedDistance(const Math::Vector3df& worldPoint,
                                    const Math::Vector3df& pos,
                                    const Math::Quaternion& orient)           const = 0;
    virtual Math::Vector3df getSurfaceNormal(const Math::Vector3df& worldPoint,
                                             const Math::Vector3df& pos,
                                             const Math::Quaternion& orient)  const = 0;

    virtual ~ICollisionShape() = default;
};

struct SphereShape : ICollisionShape {
    float radius = 0.5f;

    ShapeType getType() const override { return ShapeType::Sphere; }

    Math::Box3df getAABB(const Math::Vector3df& pos,
                         const Math::Quaternion&) const override {
        Math::Vector3df r(radius, radius, radius);
        return Math::Box3df(pos - r, pos + r);
    }

    float getSignedDistance(const Math::Vector3df& worldPoint,
                            const Math::Vector3df& pos,
                            const Math::Quaternion&) const override {
        return Math::getDistance(worldPoint, pos) - radius;
    }

    Math::Vector3df getSurfaceNormal(const Math::Vector3df& worldPoint,
                                     const Math::Vector3df& pos,
                                     const Math::Quaternion&) const override {
        Math::Vector3df d = worldPoint - pos;
        float len = Math::getLength(d);
        return (len > 1e-8f) ? (d / len) : Math::Vector3df(0.f, 1.f, 0.f);
    }
};

struct BoxShape : ICollisionShape {
    Math::Vector3df halfExtents = { 0.5f, 0.5f, 0.5f };

    ShapeType getType() const override { return ShapeType::Box; }

    Math::Box3df getAABB(const Math::Vector3df& pos,
                         const Math::Quaternion& orient) const override;

    std::array<Math::Vector3df, 8> getWorldCorners(
        const Math::Vector3df& pos,
        const Math::Quaternion& orient) const;

    float getSignedDistance(const Math::Vector3df& worldPoint,
                            const Math::Vector3df& pos,
                            const Math::Quaternion& orient) const override;

    Math::Vector3df getSurfaceNormal(const Math::Vector3df& worldPoint,
                                     const Math::Vector3df& pos,
                                     const Math::Quaternion& orient) const override;
};

struct PlaneShape : ICollisionShape {
    Math::Vector3df normal = { 0.f, 1.f, 0.f };
    float           offset = 0.f;

    ShapeType getType() const override { return ShapeType::Plane; }

    Math::Box3df getAABB(const Math::Vector3df&,
                         const Math::Quaternion&) const override {
        constexpr float INF = 1e30f;
        return Math::Box3df({ -INF, -INF, -INF }, { INF, INF, INF });
    }

    float getSignedDistance(const Math::Vector3df& worldPoint,
                            const Math::Vector3df&,
                            const Math::Quaternion&) const override {
        return PlaneBoundary(normal, offset).getSignedDistance(worldPoint);
    }

    Math::Vector3df getSurfaceNormal(const Math::Vector3df&,
                                     const Math::Vector3df&,
                                     const Math::Quaternion&) const override {
        return normal;
    }
};

// Capsule whose core segment runs along the shape's local Y axis from
// -halfHeight to +halfHeight; the surface is every point at `radius` from that
// segment. Local Y matches a Blender capsule's local Z after the glTF Y-up
// conversion, so an authored capsule needs no extra axis remap.
struct CapsuleShape : ICollisionShape {
    float radius     = 0.25f;
    float halfHeight = 0.5f; // half the length of the cylindrical part (excludes the caps)

    ShapeType getType() const override { return ShapeType::Capsule; }

    // World-space endpoints of the core segment.
    void getSegment(const Math::Vector3df& pos, const Math::Quaternion& orient,
                    Math::Vector3df& a, Math::Vector3df& b) const;

    Math::Box3df getAABB(const Math::Vector3df& pos,
                         const Math::Quaternion& orient) const override;

    float getSignedDistance(const Math::Vector3df& worldPoint,
                            const Math::Vector3df& pos,
                            const Math::Quaternion& orient) const override;

    Math::Vector3df getSurfaceNormal(const Math::Vector3df& worldPoint,
                                     const Math::Vector3df& pos,
                                     const Math::Quaternion& orient) const override;
};

// Closest point to `p` on segment [a, b] (a when the segment is degenerate).
Math::Vector3df closestPointOnSegment(const Math::Vector3df& p,
                                      const Math::Vector3df& a,
                                      const Math::Vector3df& b);

} // namespace Physics
} // namespace Phantom
