#include "pch.h"
#include "SoftFluidSolver.h"

namespace Phantom {
namespace Physics {

SoftFluidBinding& SoftFluidSolver::bind(ISoftBody* body) {
    SoftFluidBinding binding;
    binding.body = body;
    if (body != nullptr) {
        binding.particles.bind(&body->getMesh());
    }
    bindings_.push_back(binding);
    return bindings_.back();
}

void SoftFluidSolver::syncBoundaries(const SPHKernel* kernel, float restDensity) {
    for (auto& binding : bindings_) {
        if (binding.body == nullptr) continue;

        binding.particles.sync();
        if (kernel != nullptr) {
            binding.particles.computePsi(*kernel, restDensity);
        }
        binding.particles.clearAccumForce();
    }
}

void SoftFluidSolver::applyTwoWayReactions() {
    for (auto& binding : bindings_) {
        if (binding.body == nullptr) continue;

        SoftMesh& mesh = binding.body->getMesh();
        const auto& bps = binding.particles.particles();
        for (size_t i = 0; i < bps.size() && i < mesh.particles.size(); ++i) {
            mesh.particles.forces[i] = bps[i].accumForce;
        }
    }
}

void SoftFluidSolver::step(float dt) {
    applyTwoWayReactions();
    world_.solverParams().timeStep = dt;
    world_.step();
}

void SoftFluidSolver::stepForced(float dt) {
    applyTwoWayReactions();
    world_.solverParams().timeStep = dt;
    world_.stepUnconditional();
}

} // namespace Physics
} // namespace Phantom
