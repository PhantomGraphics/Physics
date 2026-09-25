#include "pch.h"
#include "NarrowPhase.h"

#include "CGLib/ThirdParty/glm-0.9.9.8/glm/gtc/quaternion.hpp"

namespace Phantom {
namespace Physics {

// ------------------------------------------------------------------ helpers --

static bool flipAndMerge(bool hit, ContactManifold& tmp, ContactManifold& out) {
    if (!hit) return false;
    for (auto& cp : tmp.contacts) cp.normal = -cp.normal;
    out.contacts = std::move(tmp.contacts);
    return true;
}

// Sphere (center ca, radius ra) of body A against sphere (cb, rb) of body B.
// Normal points from B to A; contact lies on B's surface (sphereSphere convention).
static bool pointSpheres(const Math::Vector3df& ca, float ra,
                         const Math::Vector3df& cb, float rb, ContactManifold& out) {
    Math::Vector3df delta = ca - cb;
    float dist = glm::length(delta);
    float sumR = ra + rb;
    if (dist >= sumR) return false;

    ContactPoint cp;
    cp.normal      = (dist > 1e-7f) ? delta / dist : Math::Vector3df(0.f, 1.f, 0.f);
    cp.penetration = sumR - dist;
    cp.position    = cb + cp.normal * rb;
    out.contacts.push_back(cp);
    return true;
}

// Sphere (center c, radius r) of body A against the OBB of body B.
static bool sphereVsBox(const Math::Vector3df& c, float r, const RigidBody& b,
                        ContactManifold& out) {
    auto* sb = static_cast<BoxShape*>(b.shape);

    Math::Matrix3df Rb = glm::mat3_cast(b.orientation);
    Math::Vector3df pLocal = glm::transpose(Rb) * (c - b.position);

    Math::Vector3df closest = glm::clamp(pLocal, -sb->halfExtents, sb->halfExtents);
    Math::Vector3df delta   = pLocal - closest;
    float dist = glm::length(delta);

    if (dist >= r) return false;

    ContactPoint cp;

    if (dist > 1e-6f) {
        cp.normal = glm::normalize(Rb * delta);
    } else {
        // Sphere center is inside the box -- push through nearest face.
        Math::Vector3df absP(std::abs(pLocal.x), std::abs(pLocal.y), std::abs(pLocal.z));
        Math::Vector3df pen = sb->halfExtents - absP;

        if (pen.x <= pen.y && pen.x <= pen.z) {
            float sign   = (pLocal.x >= 0.f) ? 1.f : -1.f;
            cp.normal    = Rb[0] * sign;
            cp.penetration = r + pen.x;
        } else if (pen.y <= pen.z) {
            float sign   = (pLocal.y >= 0.f) ? 1.f : -1.f;
            cp.normal    = Rb[1] * sign;
            cp.penetration = r + pen.y;
        } else {
            float sign   = (pLocal.z >= 0.f) ? 1.f : -1.f;
            cp.normal    = Rb[2] * sign;
            cp.penetration = r + pen.z;
        }
        cp.position = b.position + Rb * closest;
        out.contacts.push_back(cp);
        return true;
    }

    cp.penetration = r - dist;
    cp.position    = b.position + Rb * closest;

    out.contacts.push_back(cp);
    return true;
}

// Closest points between segments [p1,q1] and [p2,q2] (Ericson, Real-Time
// Collision Detection 5.1.9).
static void closestSegmentSegment(const Math::Vector3df& p1, const Math::Vector3df& q1,
                                  const Math::Vector3df& p2, const Math::Vector3df& q2,
                                  Math::Vector3df& c1, Math::Vector3df& c2) {
    const Math::Vector3df d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const float a = glm::dot(d1, d1), e = glm::dot(d2, d2), f = glm::dot(d2, r);
    constexpr float eps = 1e-12f;
    float s = 0.f, t = 0.f;
    if (a <= eps && e <= eps) {
        s = t = 0.f;
    } else if (a <= eps) {
        t = std::clamp(f / e, 0.f, 1.f);
    } else {
        const float c = glm::dot(d1, r);
        if (e <= eps) {
            s = std::clamp(-c / a, 0.f, 1.f);
        } else {
            const float b = glm::dot(d1, d2);
            const float denom = a * e - b * b;
            s = (denom > eps) ? std::clamp((b * f - c * e) / denom, 0.f, 1.f) : 0.f;
            t = (b * s + f) / e;
            if (t < 0.f)      { t = 0.f; s = std::clamp(-c / a, 0.f, 1.f); }
            else if (t > 1.f) { t = 1.f; s = std::clamp((b - c) / a, 0.f, 1.f); }
        }
    }
    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
}

// ------------------------------------------------------------------- detect --

bool NarrowPhase::detect(RigidBody& a, RigidBody& b, ContactManifold& out) {
    out.bodyA = &a;
    out.bodyB = &b;
    out.contacts.clear();

    auto typeA = a.shape->getType();
    auto typeB = b.shape->getType();

    if (typeA == ShapeType::Sphere && typeB == ShapeType::Sphere)
        return sphereSphere(a, b, out);

    if (typeA == ShapeType::Sphere && typeB == ShapeType::Plane)
        return spherePlane(a, b, out);

    if (typeA == ShapeType::Plane && typeB == ShapeType::Sphere) {
        ContactManifold tmp;
        tmp.bodyA = &b; tmp.bodyB = &a;
        return flipAndMerge(spherePlane(b, a, tmp), tmp, out);
    }

    if (typeA == ShapeType::Sphere && typeB == ShapeType::Box)
        return sphereBox(a, b, out);

    if (typeA == ShapeType::Box && typeB == ShapeType::Sphere) {
        ContactManifold tmp;
        tmp.bodyA = &b; tmp.bodyB = &a;
        return flipAndMerge(sphereBox(b, a, tmp), tmp, out);
    }

    if (typeA == ShapeType::Box && typeB == ShapeType::Box)
        return boxBox(a, b, out);

    if (typeA == ShapeType::Box && typeB == ShapeType::Plane)
        return boxPlane(a, b, out);

    if (typeA == ShapeType::Plane && typeB == ShapeType::Box) {
        ContactManifold tmp;
        tmp.bodyA = &b; tmp.bodyB = &a;
        return flipAndMerge(boxPlane(b, a, tmp), tmp, out);
    }

    if (typeA == ShapeType::Capsule || typeB == ShapeType::Capsule) {
        // Put the capsule in slot A; flip the normals back if we swapped.
        const bool swap = (typeA != ShapeType::Capsule);
        RigidBody& cap   = swap ? b : a;
        RigidBody& other = swap ? a : b;
        ContactManifold tmp;
        tmp.bodyA = &cap; tmp.bodyB = &other;
        bool hit = false;
        switch (other.shape->getType()) {
            case ShapeType::Plane:   hit = capsulePlane(cap, other, tmp);   break;
            case ShapeType::Sphere:  hit = capsuleSphere(cap, other, tmp);  break;
            case ShapeType::Box:     hit = capsuleBox(cap, other, tmp);     break;
            case ShapeType::Capsule: hit = capsuleCapsule(cap, other, tmp); break;
            default: return false; // no capsule-vs-mesh routine (see ShapeType)
        }
        if (swap) return flipAndMerge(hit, tmp, out);
        if (!hit) return false;
        out.contacts = std::move(tmp.contacts);
        return true;
    }

    return false;
}

// ---------------------------------------------------------- Sphere-Sphere ---

bool NarrowPhase::sphereSphere(RigidBody& a, RigidBody& b, ContactManifold& out) {
    auto* sa = static_cast<SphereShape*>(a.shape);
    auto* sb = static_cast<SphereShape*>(b.shape);
    return pointSpheres(a.position, sa->radius, b.position, sb->radius, out);
}

// ----------------------------------------------------------- Sphere-Plane ---

bool NarrowPhase::spherePlane(RigidBody& a, RigidBody& b, ContactManifold& out) {
    auto* ss = static_cast<SphereShape*>(a.shape);
    auto* sp = static_cast<PlaneShape*>(b.shape);

    float d = glm::dot(a.position, sp->normal) - sp->offset;
    if (d >= ss->radius) return false;

    ContactPoint cp;
    cp.normal      = sp->normal;
    cp.penetration = ss->radius - d;
    cp.position    = a.position - sp->normal * d;

    out.contacts.push_back(cp);
    return true;
}

// ------------------------------------------------------------- Sphere-OBB ---

bool NarrowPhase::sphereBox(RigidBody& a, RigidBody& b, ContactManifold& out) {
    auto* ss = static_cast<SphereShape*>(a.shape);
    return sphereVsBox(a.position, ss->radius, b, out);
}

// --------------------------------------------------------------- OBB-OBB ---

bool NarrowPhase::boxBox(RigidBody& a, RigidBody& b, ContactManifold& out) {
    auto* sa = static_cast<BoxShape*>(a.shape);
    auto* sb = static_cast<BoxShape*>(b.shape);

    Math::Matrix3df Ra = glm::mat3_cast(a.orientation);
    Math::Matrix3df Rb = glm::mat3_cast(b.orientation);
    Math::Vector3df hA = sa->halfExtents;
    Math::Vector3df hB = sb->halfExtents;
    Math::Vector3df T  = b.position - a.position;

    float minPen = std::numeric_limits<float>::max();
    Math::Vector3df bestAxis(0.f);

    auto testAxis = [&](Math::Vector3df axis) -> bool {
        float len = glm::length(axis);
        if (len < 1e-6f) return false;
        axis /= len;

        float projT = std::abs(glm::dot(T, axis));

        float rA = std::abs(hA.x * glm::dot(Ra[0], axis))
                 + std::abs(hA.y * glm::dot(Ra[1], axis))
                 + std::abs(hA.z * glm::dot(Ra[2], axis));

        float rB = std::abs(hB.x * glm::dot(Rb[0], axis))
                 + std::abs(hB.y * glm::dot(Rb[1], axis))
                 + std::abs(hB.z * glm::dot(Rb[2], axis));

        float pen = rA + rB - projT;
        if (pen < 0.f) return true;  // separating axis

        if (pen < minPen) {
            minPen   = pen;
            bestAxis = (glm::dot(T, axis) >= 0.f) ? -axis : axis;
        }
        return false;
    };

    // 3 face normals of A
    for (int i = 0; i < 3; ++i)
        if (testAxis(Ra[i])) return false;

    // 3 face normals of B
    for (int i = 0; i < 3; ++i)
        if (testAxis(Rb[i])) return false;

    // 9 edge-cross axes
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (testAxis(glm::cross(Ra[i], Rb[j]))) return false;

    // Contact point: surface of B facing A
    float rBn = std::abs(hB.x * glm::dot(Rb[0], bestAxis))
              + std::abs(hB.y * glm::dot(Rb[1], bestAxis))
              + std::abs(hB.z * glm::dot(Rb[2], bestAxis));

    ContactPoint cp;
    cp.normal      = bestAxis;
    cp.penetration = minPen;
    cp.position    = b.position + bestAxis * rBn;

    out.contacts.push_back(cp);
    return true;
}

// --------------------------------------------------------------- OBB-Plane --

bool NarrowPhase::boxPlane(RigidBody& a, RigidBody& b, ContactManifold& out) {
    auto* sb = static_cast<BoxShape*>(a.shape);
    auto* sp = static_cast<PlaneShape*>(b.shape);

    auto corners = sb->getWorldCorners(a.position, a.orientation);

    bool anyContact = false;
    for (const auto& corner : corners) {
        float d = glm::dot(corner, sp->normal) - sp->offset;
        if (d < 0.f) {
            ContactPoint cp;
            cp.normal      = sp->normal;
            cp.penetration = -d;
            cp.position    = corner - sp->normal * d;
            out.contacts.push_back(cp);
            anyContact = true;
        }
    }
    return anyContact;
}

// ----------------------------------------------------------- Capsule-Plane --

bool NarrowPhase::capsulePlane(RigidBody& a, RigidBody& b, ContactManifold& out) {
    auto* sc = static_cast<CapsuleShape*>(a.shape);
    auto* sp = static_cast<PlaneShape*>(b.shape);

    Math::Vector3df ends[2];
    sc->getSegment(a.position, a.orientation, ends[0], ends[1]);

    // Both cap centers can touch, which keeps a lying capsule from rocking.
    bool anyContact = false;
    for (const auto& e : ends) {
        float d = glm::dot(e, sp->normal) - sp->offset;
        if (d >= sc->radius) continue;
        ContactPoint cp;
        cp.normal      = sp->normal;
        cp.penetration = sc->radius - d;
        cp.position    = e - sp->normal * d;
        out.contacts.push_back(cp);
        anyContact = true;
    }
    return anyContact;
}

// ---------------------------------------------------------- Capsule-Sphere --

bool NarrowPhase::capsuleSphere(RigidBody& a, RigidBody& b, ContactManifold& out) {
    auto* sc = static_cast<CapsuleShape*>(a.shape);
    auto* ss = static_cast<SphereShape*>(b.shape);
    Math::Vector3df p, q;
    sc->getSegment(a.position, a.orientation, p, q);
    return pointSpheres(closestPointOnSegment(b.position, p, q), sc->radius,
                        b.position, ss->radius, out);
}

// ------------------------------------------------------------- Capsule-OBB --

bool NarrowPhase::capsuleBox(RigidBody& a, RigidBody& b, ContactManifold& out) {
    auto* sc = static_cast<CapsuleShape*>(a.shape);
    auto* sb = static_cast<BoxShape*>(b.shape);
    Math::Vector3df p, q;
    sc->getSegment(a.position, a.orientation, p, q);

    // Segment point closest to the box, found by alternating projections
    // (segment and box are both convex, so this converges quickly).
    const Math::Matrix3df Rb = glm::mat3_cast(b.orientation);
    Math::Vector3df onSeg = closestPointOnSegment(b.position, p, q);
    for (int i = 0; i < 4; ++i) {
        Math::Vector3df local = glm::transpose(Rb) * (onSeg - b.position);
        Math::Vector3df onBox = b.position + Rb * glm::clamp(local, -sb->halfExtents, sb->halfExtents);
        onSeg = closestPointOnSegment(onBox, p, q);
    }

    // Test the cap centers too so a capsule lying on a box gets two support
    // points; skip the interior point when it coincides with a cap.
    const float segLen = glm::length(q - p);
    const float mergeDist = std::max(1e-4f, segLen * 1e-3f);
    bool hit = sphereVsBox(p, sc->radius, b, out);
    hit = sphereVsBox(q, sc->radius, b, out) || hit;
    if (glm::length(onSeg - p) > mergeDist && glm::length(onSeg - q) > mergeDist)
        hit = sphereVsBox(onSeg, sc->radius, b, out) || hit;
    return hit;
}

// --------------------------------------------------------- Capsule-Capsule --

bool NarrowPhase::capsuleCapsule(RigidBody& a, RigidBody& b, ContactManifold& out) {
    auto* ca = static_cast<CapsuleShape*>(a.shape);
    auto* cb = static_cast<CapsuleShape*>(b.shape);
    Math::Vector3df p1, q1, p2, q2, c1, c2;
    ca->getSegment(a.position, a.orientation, p1, q1);
    cb->getSegment(b.position, b.orientation, p2, q2);
    closestSegmentSegment(p1, q1, p2, q2, c1, c2);
    return pointSpheres(c1, ca->radius, c2, cb->radius, out);
}

} // namespace Physics
} // namespace Phantom
