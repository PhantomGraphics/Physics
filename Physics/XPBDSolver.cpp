#include "XPBDSolver.h"

namespace Phantom {
namespace Physics {

void XPBDSolver::setMesh(SoftMesh* mesh) {
    mesh_ = mesh;
    restPositions_.clear();
    if (mesh_) {
        restPositions_ = mesh_->particles.positions;
    }
}

void XPBDSolver::addConstraint(std::unique_ptr<IConstraint> c) {
    constraints_.push_back(std::move(c));
}

void XPBDSolver::clearConstraints() {
    constraints_.clear();
}

void XPBDSolver::addCollider(ISoftCollider* c) {
    colliders_.push_back(c);
}

void XPBDSolver::reset() {
    if (!mesh_) return;
    auto& particles = mesh_->particles;
    for (size_t i = 0; i < particles.size() && i < restPositions_.size(); ++i) {
        particles.positions[i]  = restPositions_[i];
        particles.predicted[i]  = restPositions_[i];
        particles.velocities[i] = { 0.f, 0.f, 0.f };
        particles.forces[i]     = { 0.f, 0.f, 0.f };
    }
}

void XPBDSolver::step() {
    if (!mesh_) return;

    float dt = params_.timeStep / static_cast<float>(params_.numSubsteps);
    float alphaHat = 0.f;  // 各制約で個別に計算（dt² で割る）

    for (int s = 0; s < params_.numSubsteps; ++s) {
        applyExternalForces(dt);
        predictPositions(dt);

        // サブステップ冒頭で λ を 0 リセット
        for (auto& c : constraints_)
            c->resetLambda();

        projectConstraints(dt);
        resolveCollisions();
        resolveSelfCollision();
        updateVelocities(dt);
    }
    (void)alphaHat;
}

void XPBDSolver::applyExternalForces(float dt) {
    Math::Vector3df ext = params_.gravity + params_.wind;
    auto& particles = mesh_->particles;
    for (size_t i = 0; i < particles.size(); ++i) {
        if (!particles.isPinned(i))
            particles.velocities[i] += dt * (ext + particles.forces[i] * particles.inverseMasses[i]);
    }
}

void XPBDSolver::predictPositions(float dt) {
    auto& particles = mesh_->particles;
    for (size_t i = 0; i < particles.size(); ++i) {
        if (!particles.isPinned(i))
            particles.predicted[i] = particles.positions[i] + dt * particles.velocities[i];
        else
            particles.predicted[i] = particles.positions[i];
    }
}

void XPBDSolver::projectConstraints(float dt) {
    float dt2 = dt * dt;
    for (int iter = 0; iter < params_.numIterations; ++iter) {
        for (auto& c : constraints_) {
            float aHat = (dt2 > 1e-12f) ? c->alpha / dt2 : 0.f;
            c->project(mesh_->particles, aHat);
        }
    }
}

void XPBDSolver::resolveCollisions() {
    auto& particles = mesh_->particles;
    for (size_t i = 0; i < particles.size(); ++i) {
        if (particles.isPinned(i)) continue;
        for (auto* col : colliders_)
            col->resolve(particles, i);
    }
}

void XPBDSolver::resolveSelfCollision() {
    if (!params_.selfCollisionEnabled) return;
    selfCollision_.update(*mesh_, { params_.selfCollisionThickness, params_.selfCollisionCellSize });
}

void XPBDSolver::updateVelocities(float dt) {
    auto& particles = mesh_->particles;
    for (size_t i = 0; i < particles.size(); ++i) {
        if (!particles.isPinned(i)) {
            particles.velocities[i] = (particles.predicted[i] - particles.positions[i]) / dt;
            particles.velocities[i] *= params_.damping;
        }
        particles.positions[i] = particles.predicted[i];
    }
}

} // namespace Physics
} // namespace Phantom
