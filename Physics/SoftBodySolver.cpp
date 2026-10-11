#include "SoftBodySolver.h"
#include "Physics/Physics/RigidBody.h"
#include "Physics/Physics/ICollisionShape.h"

#include <array>
#include <cmath>

namespace Phantom {
namespace Physics {

void SoftBodySolver::clearBodies() {
    bodies_.clear();
    solvers_.clear();
}

void SoftBodySolver::addRigidBodyCollider(RigidBody* rb) {
    rigidColliders_.push_back(std::make_unique<RigidBodyCollider>(rb));
    syncColliders();
}

void SoftBodySolver::clearRigidBodyColliders() {
    rigidColliders_.clear();
    syncColliders();
}

Math::Vector3df SoftBodySolver::computeRigidReactionForce(const RigidBoundary& boundary,
                                                            const Math::Vector3df& pivot,
                                                            Math::Vector3df& outTorque) const {
    Math::Vector3df totalForce{ 0.f, 0.f, 0.f };
    outTorque = { 0.f, 0.f, 0.f };

    for (const auto& body : bodies_) {
        const auto& particles = body->getMesh().particles;
        for (size_t i = 0; i < particles.size(); ++i) {
            // getBoundaryForce() returns the penalty spring's force on the *particle* (pushing
            // it away from the rigid shape's center, along the surface normal at its position).
            // Newton's third law: the rigid body feels the equal-and-opposite reaction.
            const Math::Vector3df& pos = particles.positions[i];
            const Math::Vector3df reaction = -boundary.getBoundaryForce(pos);
            totalForce += reaction;
            outTorque  += glm::cross(pos - pivot, reaction);
        }
    }
    return totalForce;
}

void SoftBodySolver::step() {
    if (!params_.running) return;
    stepInternal();
}

void SoftBodySolver::stepUnconditional() {
    stepInternal();
}

void SoftBodySolver::stepInternal() {
    syncColliders();
    for (size_t i = 0; i < solvers_.size(); ++i) {
        solvers_[i]->params() = solverParams_;
        solvers_[i]->step();
    }

    if (params_.crossBodyCollisionEnabled && bodies_.size() > 1) {
        std::vector<SoftMesh*> meshes;
        meshes.reserve(bodies_.size());
        for (auto& b : bodies_)
            meshes.push_back(&b->getMesh());
        crossBodyCollision_.resolve(meshes,
            { params_.crossBodyCollisionThickness, params_.crossBodyCollisionCellSize },
            solverParams_.timeStep);
    }
}

void SoftBodySolver::reset() {
    for (auto& s : solvers_)
        s->reset();
}

size_t SoftBodySolver::getParticleCount() const {
    size_t n = 0;
    for (const auto& b : bodies_)
        n += b->getMesh().particles.size();
    return n;
}

float SoftBodySolver::getMaxSpeed() const {
    float m = 0.f;
    for (const auto& b : bodies_)
        for (const auto& v : b->getMesh().particles.velocities)
            m = std::max(m, glm::length(v));
    return m;
}

void SoftBodySolver::rebuildEntry(size_t idx) {
    auto& solver = *solvers_[idx];
    auto& body   = *bodies_[idx];
    solver.params() = solverParams_;
    solver.clearConstraints();
    solver.setMesh(&body.getMesh());
    body.build(solver);
    syncColliders();
}

void SoftBodySolver::syncColliders() {
    sphere_.center = params_.sphereCenter;
    sphere_.radius = params_.sphereRadius;
    floor_.normal  = { 0.f, 1.f, 0.f };
    floor_.offset  = 0.f;

    for (auto& s : solvers_) {
        s->clearColliders();
        s->addCollider(&floor_);
        if (params_.sphereEnabled)
            s->addCollider(&sphere_);
        for (auto& rc : rigidColliders_)
            s->addCollider(rc.get());
    }
}

} // namespace Physics
} // namespace Phantom
