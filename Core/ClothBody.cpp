#include "ClothBody.h"
#include "BendConstraint.h"
#include "DistanceConstraint.h"
#include "PinConstraint.h"

namespace Phantom {
namespace Physics {

ClothBody::ClothBody(const ClothBodyParams& p) : params_(p) {
    mesh_.generateGrid(p.rows, p.cols, p.width, p.height);
    if (p.origin != Math::Vector3df{ 0.f, 0.f, 0.f }) {
        for (size_t i = 0; i < mesh_.particles.size(); ++i) {
            mesh_.particles.positions[i]  += p.origin;
            mesh_.particles.predicted[i]  += p.origin;
        }
    }
}

float ClothBody::computeRestCos(int i0, int i1, int i2, int i3) const {
    const auto& pa = mesh_.particles.positions[i0];
    const auto& pb = mesh_.particles.positions[i1];
    const auto& pc = mesh_.particles.positions[i2];
    const auto& pd = mesh_.particles.positions[i3];
    auto e   = pb - pa;
    auto n0  = glm::cross(e, pc - pa);
    auto n1  = glm::cross(e, pd - pa);
    float l0 = glm::length(n0), l1 = glm::length(n1);
    if (l0 < 1e-8f || l1 < 1e-8f) return -1.f;
    return glm::clamp(glm::dot(n0 / l0, n1 / l1), -1.f, 1.f);
}

void ClothBody::build(XPBDSolver& solver) {
    int rows = params_.rows, cols = params_.cols;

    // Pin constraints
    PinConstraint pins;
    if (params_.pinTopEdge) {
        for (int c = 0; c < cols; ++c)
            pins.indices.push_back(c);
    } else {
        if (params_.pinTopLeft)  pins.indices.push_back(0);
        if (params_.pinTopRight) pins.indices.push_back(cols - 1);
    }
    pins.apply(mesh_.particles);

    // Distance constraints for structural edges
    int se = mesh_.numStructuralEdges > 0
           ? mesh_.numStructuralEdges
           : static_cast<int>(mesh_.edges.size());
    for (int i = 0; i < se; ++i) {
        const auto& e = mesh_.edges[i];
        auto dc = std::make_unique<DistanceConstraint>();
        dc->alpha      = params_.distAlpha;
        dc->a          = e.a;
        dc->b          = e.b;
        dc->restLength = e.restLength;
        solver.addConstraint(std::move(dc));
    }

    // Bend constraints from dihedral angle between adjacent triangles
    auto idx = [cols](int r, int c) { return r * cols + c; };

    // Type 1: within-quad (shared diagonal edge tr-bl)
    for (int r = 0; r < rows - 1; ++r)
        for (int c = 0; c < cols - 1; ++c) {
            int i0 = idx(r, c + 1), i1 = idx(r + 1, c);
            int i2 = idx(r, c),     i3 = idx(r + 1, c + 1);
            auto bc = std::make_unique<BendConstraint>();
            bc->alpha   = params_.bendAlpha;
            bc->i0 = i0; bc->i1 = i1; bc->i2 = i2; bc->i3 = i3;
            bc->restCos = computeRestCos(i0, i1, i2, i3);
            solver.addConstraint(std::move(bc));
        }

    // Type 2: between vertically adjacent quads (shared horizontal edge)
    for (int r = 0; r < rows - 2; ++r)
        for (int c = 0; c < cols - 1; ++c) {
            int i0 = idx(r + 1, c),     i1 = idx(r + 1, c + 1);
            int i2 = idx(r,     c + 1), i3 = idx(r + 2, c);
            auto bc = std::make_unique<BendConstraint>();
            bc->alpha   = params_.bendAlpha;
            bc->i0 = i0; bc->i1 = i1; bc->i2 = i2; bc->i3 = i3;
            bc->restCos = computeRestCos(i0, i1, i2, i3);
            solver.addConstraint(std::move(bc));
        }

    // Type 3: between horizontally adjacent quads (shared vertical edge)
    for (int r = 0; r < rows - 1; ++r)
        for (int c = 0; c < cols - 2; ++c) {
            int i0 = idx(r,     c + 1), i1 = idx(r + 1, c + 1);
            int i2 = idx(r + 1, c),     i3 = idx(r,     c + 2);
            auto bc = std::make_unique<BendConstraint>();
            bc->alpha   = params_.bendAlpha;
            bc->i0 = i0; bc->i1 = i1; bc->i2 = i2; bc->i3 = i3;
            bc->restCos = computeRestCos(i0, i1, i2, i3);
            solver.addConstraint(std::move(bc));
        }
}

float ClothBody::getStretch(int edgeIdx) const {
    const auto& e  = mesh_.edges[edgeIdx];
    const auto& pa = mesh_.particles.positions[e.a];
    const auto& pb = mesh_.particles.positions[e.b];
    float cur = glm::length(pb - pa);
    return (e.restLength > 1e-8f) ? cur / e.restLength : 1.f;
}

} // namespace Physics
} // namespace Phantom
