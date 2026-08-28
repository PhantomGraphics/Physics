#include "pch.h"
#include "RigidSoftSolver.h"

namespace Phantom {
namespace Physics {

RigidSoftBinding& RigidSoftSolver::bind(RigidBody* rigidBody, SoftBodySolver& softWorld, CouplingMode mode) {
    RigidSoftBinding binding;
    binding.rigidBody = rigidBody;
    binding.mode      = mode;
    if (rigidBody != nullptr) {
        binding.boundary.setShape(rigidBody->shape);
        binding.boundary.sync(*rigidBody);
    }
    bindings_.push_back(binding);

    softWorld.addRigidBodyCollider(rigidBody);
    return bindings_.back();
}

void RigidSoftSolver::clearBindings(SoftBodySolver& softWorld) {
    bindings_.clear();
    softWorld.clearRigidBodyColliders();
}

void RigidSoftSolver::applyTwoWayReactions(const SoftBodySolver& softWorld) {
    for (auto& binding : bindings_) {
        if (binding.mode != CouplingMode::TwoWay || binding.rigidBody == nullptr) continue;

        binding.boundary.sync(*binding.rigidBody);

        Math::Vector3df torque{ 0.f, 0.f, 0.f };
        const Math::Vector3df force =
            softWorld.computeRigidReactionForce(binding.boundary, binding.rigidBody->position, torque);

        binding.rigidBody->applyForce(force);
        binding.rigidBody->applyTorque(torque);
    }
}

} // namespace Physics
} // namespace Phantom
