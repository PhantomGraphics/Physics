#include "JellyBody.h"
#include "DistanceConstraint.h"
#include "VolumeConstraint.h"

namespace Phantom {
namespace Physics {

JellyBody::JellyBody(const JellyBodyParams& p) : params_(p) {
    mesh_.generateTetBox(p.nx, p.ny, p.nz, p.w, p.h, p.d);
    if (p.offset != Math::Vector3df{ 0.f, 0.f, 0.f }) {
        for (size_t i = 0; i < mesh_.particles.size(); ++i) {
            mesh_.particles.positions[i]  += p.offset;
            mesh_.particles.predicted[i]  += p.offset;
        }
    }
}

void JellyBody::build(XPBDSolver& solver) {
    // Distance constraints over all tetra edges (shape preservation)
    for (const auto& e : mesh_.edges) {
        auto dc = std::make_unique<DistanceConstraint>();
        dc->alpha      = params_.distAlpha;
        dc->a          = e.a;
        dc->b          = e.b;
        dc->restLength = e.restLength;
        solver.addConstraint(std::move(dc));
    }

    // Volume constraints for all tetrahedra (incompressibility)
    for (const auto& t : mesh_.tetrahedra) {
        auto vc = std::make_unique<VolumeConstraint>();
        vc->alpha      = params_.volumeAlpha;
        vc->a          = t.a;
        vc->b          = t.b;
        vc->c          = t.c;
        vc->d          = t.d;
        vc->restVolume = t.restVolume;
        solver.addConstraint(std::move(vc));
    }
}

float JellyBody::getTotalVolume() const {
    float total = 0.f;
    for (const auto& t : mesh_.tetrahedra) {
        const auto& pa = mesh_.particles.positions[t.a];
        const auto& pb = mesh_.particles.positions[t.b];
        const auto& pc = mesh_.particles.positions[t.c];
        const auto& pd = mesh_.particles.positions[t.d];
        total += glm::dot(pb - pa, glm::cross(pc - pa, pd - pa)) / 6.f;
    }
    return total;
}

} // namespace Physics
} // namespace Phantom
