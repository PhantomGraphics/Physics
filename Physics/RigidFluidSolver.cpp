#include "pch.h"
#include "RigidFluidSolver.h"

namespace Phantom {
namespace Physics {

RigidFluidBinding& RigidFluidSolver::bind(RigidBody* body, ICollisionShape* shape, CouplingMode mode) {
    RigidFluidBinding binding;
    binding.body = body;
    binding.mode = mode;
    binding.boundary.setShape(shape);
    if (body != nullptr) {
        binding.boundary.sync(*body);
    }
    bindings_.push_back(binding);
    return bindings_.back();
}

void RigidFluidSolver::syncBoundaries() {
    for (auto& binding : bindings_) {
        if (binding.body == nullptr) continue;

        binding.boundary.sync(*binding.body);
        if (binding.mode == CouplingMode::TwoWay) {
            binding.particles.sync(binding.body->position, binding.body->orientation);
            binding.particles.clearAccumForce();
        }
    }
}

void RigidFluidSolver::applyTwoWayReactions() {
    for (auto& binding : bindings_) {
        if (binding.mode != CouplingMode::TwoWay || binding.body == nullptr) continue;

        for (const auto& bp : binding.particles.particles()) {
            binding.body->applyForce(bp.accumForce);
            const Math::Vector3df r = bp.worldPos - binding.body->position;
            binding.body->applyTorque(glm::cross(r, bp.accumForce));
        }
    }
}

void RigidFluidSolver::step(float dt) {
    applyTwoWayReactions();
    world_.timeStep = dt;
    world_.step();
}

void RigidFluidSolver::stepForced(float dt) {
    applyTwoWayReactions();
    world_.timeStep = dt;
    world_.stepUnconditional();
}

} // namespace Physics
} // namespace Phantom
