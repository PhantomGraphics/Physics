#include "pch.h"
#include "RigidBoundaryParticles.h"
#include "BoundaryPsi.h"

#include "CGLib/Math/pi.h"

namespace Phantom {
namespace Physics {

void RigidBoundaryParticles::sample(const ICollisionShape& shape, float spacing) {
    particles_.clear();
    localPositions_.clear();
    if (spacing <= 0.f) return;

    switch (shape.getType()) {
    case ShapeType::Sphere:
        sampleSphere(static_cast<const SphereShape&>(shape).radius, spacing);
        break;
    case ShapeType::Box:
        sampleBox(static_cast<const BoxShape&>(shape).halfExtents, spacing);
        break;
    case ShapeType::Plane:
    default:
        // Infinite plane: no finite surface to sample; Two-Way coupling is
        // unsupported here (see class doc).
        break;
    }
}

void RigidBoundaryParticles::sampleSphere(float radius, float spacing) {
    const int numRings = std::max(1, static_cast<int>(std::round(Math::PIf * radius / spacing)));

    for (int i = 0; i <= numRings; ++i) {
        const float theta       = Math::PIf * static_cast<float>(i) / static_cast<float>(numRings); // 0..PI
        const float ringRadius  = radius * std::sin(theta);
        const float y           = radius * std::cos(theta);

        const int numOnRing = (ringRadius > 1.0e-5f)
            ? std::max(1, static_cast<int>(std::round(2.f * Math::PIf * ringRadius / spacing)))
            : 1;

        for (int j = 0; j < numOnRing; ++j) {
            const float phi = 2.f * Math::PIf * static_cast<float>(j) / static_cast<float>(numOnRing);
            particles_.push_back(BoundaryParticle{});
            localPositions_.push_back(Math::Vector3df(ringRadius * std::cos(phi), y, ringRadius * std::sin(phi)));
        }
    }
}

void RigidBoundaryParticles::sampleBox(const Math::Vector3df& halfExtents, float spacing) {
    const float extents[3] = { halfExtents.x, halfExtents.y, halfExtents.z };

    auto addFace = [&](int axis, float sign) {
        const int u = (axis + 1) % 3;
        const int v = (axis + 2) % 3;
        const float hu = extents[u];
        const float hv = extents[v];

        const int nu = std::max(1, static_cast<int>(std::round(2.f * hu / spacing)) + 1);
        const int nv = std::max(1, static_cast<int>(std::round(2.f * hv / spacing)) + 1);

        for (int i = 0; i < nu; ++i) {
            const float uCoord = (nu > 1) ? (-hu + 2.f * hu * static_cast<float>(i) / static_cast<float>(nu - 1)) : 0.f;
            for (int j = 0; j < nv; ++j) {
                const float vCoord = (nv > 1) ? (-hv + 2.f * hv * static_cast<float>(j) / static_cast<float>(nv - 1)) : 0.f;

                float coords[3];
                coords[axis] = sign * extents[axis];
                coords[u]    = uCoord;
                coords[v]    = vCoord;

                particles_.push_back(BoundaryParticle{});
                localPositions_.push_back(Math::Vector3df(coords[0], coords[1], coords[2]));
            }
        }
    };

    addFace(0,  1.f); addFace(0, -1.f);
    addFace(1,  1.f); addFace(1, -1.f);
    addFace(2,  1.f); addFace(2, -1.f);
}

void RigidBoundaryParticles::computePsi(const SPHKernel& kernel, float restDensity) {
    std::vector<float> psi;
    computeBoundaryPsi(localPositions_, kernel, restDensity, psi);

    for (size_t k = 0; k < particles_.size(); ++k) particles_[k].psi = psi[k];
}

void RigidBoundaryParticles::sync(const Math::Vector3df& pos, const Math::Quaternion& orient) {
    for (size_t i = 0; i < particles_.size(); ++i) {
        particles_[i].worldPos = pos + orient * localPositions_[i];
    }
}

} // namespace Physics
} // namespace Phantom
