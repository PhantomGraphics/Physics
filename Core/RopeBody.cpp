#include "RopeBody.h"
#include "DistanceConstraint.h"
#include "PinConstraint.h"

namespace Phantom {
namespace Physics {

RopeBody::RopeBody(const RopeBodyParams& p) : params_(p) {
    mesh_.generateChain(p.n, p.length);
}

void RopeBody::build(XPBDSolver& solver) {
    int   n      = params_.n;
    float segLen = (n > 1) ? params_.length / (n - 1) : 0.f;

    // Pin constraints
    PinConstraint pins;
    if (params_.pinStart) pins.indices.push_back(0);
    if (params_.pinEnd)   pins.indices.push_back(n - 1);
    pins.apply(mesh_.particles);

    // Structural distance constraints (consecutive pairs)
    for (const auto& e : mesh_.edges) {
        auto dc = std::make_unique<DistanceConstraint>();
        dc->alpha      = params_.distAlpha;
        dc->a          = e.a;
        dc->b          = e.b;
        dc->restLength = e.restLength;
        solver.addConstraint(std::move(dc));
    }

    // Bending: distance constraint between every other particle (i, i+2)
    float bendLen = 2.f * segLen;
    for (int i = 0; i < n - 2; ++i) {
        auto dc = std::make_unique<DistanceConstraint>();
        dc->alpha      = params_.bendAlpha;
        dc->a          = i;
        dc->b          = i + 2;
        dc->restLength = bendLen;
        solver.addConstraint(std::move(dc));
    }
}

} // namespace Physics
} // namespace Phantom
