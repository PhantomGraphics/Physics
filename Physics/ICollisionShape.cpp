#include "pch.h"
#include "ICollisionShape.h"

#include "CGLib/Math/Matrix3d.h"
#include "CGLib/ThirdParty/glm-0.9.9.8/glm/gtc/quaternion.hpp"

namespace Phantom {
namespace Physics {

std::array<Math::Vector3df, 8> BoxShape::getWorldCorners(
    const Math::Vector3df& pos,
    const Math::Quaternion& orient) const
{
    Math::Matrix3df R = glm::mat3_cast(orient);
    std::array<Math::Vector3df, 8> corners;
    int idx = 0;
    for (int sx : { -1, 1 })
    for (int sy : { -1, 1 })
    for (int sz : { -1, 1 }) {
        corners[idx++] = pos
            + R[0] * (halfExtents.x * static_cast<float>(sx))
            + R[1] * (halfExtents.y * static_cast<float>(sy))
            + R[2] * (halfExtents.z * static_cast<float>(sz));
    }
    return corners;
}

Math::Box3df BoxShape::getAABB(const Math::Vector3df& pos,
                                const Math::Quaternion& orient) const {
    auto corners = getWorldCorners(pos, orient);
    Math::Box3df box(corners[0]);
    for (int i = 1; i < 8; ++i)
        box.add(corners[i]);
    return box;
}

float BoxShape::getSignedDistance(const Math::Vector3df& worldPoint,
                                   const Math::Vector3df& pos,
                                   const Math::Quaternion& orient) const {
    Math::Vector3df local = glm::inverse(orient) * (worldPoint - pos);
    Math::Vector3df absLocal(std::abs(local.x), std::abs(local.y), std::abs(local.z));
    Math::Vector3df q = absLocal - halfExtents;

    Math::Vector3df qMax(std::max(q.x, 0.f), std::max(q.y, 0.f), std::max(q.z, 0.f));
    float outside = Math::getLength(qMax);
    float inside  = std::min(std::max({ q.x, q.y, q.z }), 0.f);
    return outside + inside;
}

Math::Vector3df BoxShape::getSurfaceNormal(const Math::Vector3df& worldPoint,
                                            const Math::Vector3df& pos,
                                            const Math::Quaternion& orient) const {
    Math::Vector3df local = glm::inverse(orient) * (worldPoint - pos);
    Math::Vector3df absLocal(std::abs(local.x), std::abs(local.y), std::abs(local.z));
    Math::Vector3df q = absLocal - halfExtents;

    Math::Vector3df localNormal;
    if (q.x > 0.f || q.y > 0.f || q.z > 0.f) {
        // Outside the box: gradient of length(max(q, 0)).
        localNormal = Math::Vector3df(
            (q.x > 0.f) ? (local.x >= 0.f ? q.x : -q.x) : 0.f,
            (q.y > 0.f) ? (local.y >= 0.f ? q.y : -q.y) : 0.f,
            (q.z > 0.f) ? (local.z >= 0.f ? q.z : -q.z) : 0.f);
        float len = Math::getLength(localNormal);
        localNormal = (len > 1e-8f) ? localNormal / len : Math::Vector3df(0.f, 1.f, 0.f);
    } else {
        // Inside the box: push out through the nearest face.
        if (q.x >= q.y && q.x >= q.z)
            localNormal = Math::Vector3df(local.x >= 0.f ? 1.f : -1.f, 0.f, 0.f);
        else if (q.y >= q.z)
            localNormal = Math::Vector3df(0.f, local.y >= 0.f ? 1.f : -1.f, 0.f);
        else
            localNormal = Math::Vector3df(0.f, 0.f, local.z >= 0.f ? 1.f : -1.f);
    }
    return orient * localNormal;
}

} // namespace Physics
} // namespace Phantom
