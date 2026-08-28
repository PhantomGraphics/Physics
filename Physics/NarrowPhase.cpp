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

    return false;
}

// ---------------------------------------------------------- Sphere-Sphere ---

bool NarrowPhase::sphereSphere(RigidBody& a, RigidBody& b, ContactManifold& out) {
    auto* sa = static_cast<SphereShape*>(a.shape);
    auto* sb = static_cast<SphereShape*>(b.shape);

    Math::Vector3df delta = a.position - b.position;
    float dist = glm::length(delta);
    float sumR  = sa->radius + sb->radius;

    if (dist >= sumR) return false;

    ContactPoint cp;
    if (dist > 1e-7f)
        cp.normal = delta / dist;
    else
        cp.normal = Math::Vector3df(0.f, 1.f, 0.f);

    cp.penetration = sumR - dist;
    cp.position    = b.position + cp.normal * sb->radius;

    out.contacts.push_back(cp);
    return true;
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
    auto* sb = static_cast<BoxShape*>(b.shape);

    Math::Matrix3df Rb = glm::mat3_cast(b.orientation);
    Math::Vector3df pLocal = glm::transpose(Rb) * (a.position - b.position);

    Math::Vector3df closest = glm::clamp(pLocal, -sb->halfExtents, sb->halfExtents);
    Math::Vector3df delta   = pLocal - closest;
    float dist = glm::length(delta);

    if (dist >= ss->radius) return false;

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
            cp.penetration = ss->radius + pen.x;
        } else if (pen.y <= pen.z) {
            float sign   = (pLocal.y >= 0.f) ? 1.f : -1.f;
            cp.normal    = Rb[1] * sign;
            cp.penetration = ss->radius + pen.y;
        } else {
            float sign   = (pLocal.z >= 0.f) ? 1.f : -1.f;
            cp.normal    = Rb[2] * sign;
            cp.penetration = ss->radius + pen.z;
        }
        cp.position = b.position + Rb * closest;
        out.contacts.push_back(cp);
        return true;
    }

    cp.penetration = ss->radius - dist;
    cp.position    = b.position + Rb * closest;

    out.contacts.push_back(cp);
    return true;
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

} // namespace Physics
} // namespace Phantom
