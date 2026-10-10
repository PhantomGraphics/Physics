#include "pch.h"
#include "PhysicsSolver.h"

#include <algorithm>

namespace Phantom {
namespace Physics {

namespace {
template <class T>
bool contains(const std::vector<T*>& list, const T* p)
{
    return std::find(list.begin(), list.end(), p) != list.end();
}
template <class T>
bool erase(std::vector<T*>& list, const T* p)
{
    const auto it = std::find(list.begin(), list.end(), p);
    if (it == list.end()) return false;
    list.erase(it);
    return true;
}
} // namespace

void PhysicsSolver::setFluidSolver(ISPHSolver* solver)
{
    if (solver == fluidSolver_) return;
    detachFromFluid();
    fluidSolver_ = solver;
    attachToFluid();
}

void PhysicsSolver::resyncFluidRegistrations()
{
    attachToFluid();
}

// Removes only the entries this object registered; entries registered on the
// solver by other owners stay.
void PhysicsSolver::detachFromFluid()
{
    if (!fluidSolver_) return;
    for (auto* boundary : rigidBoundaries_) fluidSolver_->removeRigidBoundary(boundary);
    for (auto* particles : rigidParticles_) fluidSolver_->removeRigidBoundaryParticles(particles);
    for (auto* particles : softParticles_) fluidSolver_->removeSoftBoundaryParticles(particles);
}

// Single path used for the first attach, for a fluid switch and for a resync:
// refresh what depends on the fluid (kernel / rest density, psi, stale reaction
// forces), then register. Registration is idempotent.
void PhysicsSolver::attachToFluid()
{
    softFluidKernel_      = fluidSolver_ ? fluidSolver_->getKernel() : nullptr;
    softFluidRestDensity_ = fluidSolver_ ? fluidSolver_->getRestDensity() : 0.f;
    if (!fluidSolver_) return;
    for (auto* boundary : rigidBoundaries_) fluidSolver_->addRigidBoundary(boundary);
    for (auto* particles : rigidParticles_) {
        if (softFluidKernel_) particles->computePsi(*softFluidKernel_, softFluidRestDensity_);
        particles->clearAccumForce();
        fluidSolver_->addRigidBoundaryParticles(particles);
    }
    for (auto* particles : softParticles_) {
        particles->clearAccumForce();
        fluidSolver_->addSoftBoundaryParticles(particles);
    }
}

RigidFluidBinding& PhysicsSolver::bindRigidBody(RigidBody* body, ICollisionShape* shape, CouplingMode mode)
{
    auto& binding = rigidFluid_.bind(body, shape, mode);
    addRigidBoundary(&binding.boundary);
    return binding;
}

void PhysicsSolver::clearRigidBodyBindings()
{
    // Unregister before the bindings (which own the registered objects) die.
    if (fluidSolver_) {
        for (auto* boundary : rigidBoundaries_) fluidSolver_->removeRigidBoundary(boundary);
        for (auto* particles : rigidParticles_) fluidSolver_->removeRigidBoundaryParticles(particles);
    }
    rigidBoundaries_.clear();
    rigidParticles_.clear();
    rigidFluid_.clearBindings();
}

bool PhysicsSolver::addRigidBoundary(RigidBoundary* b)
{
    if (b == nullptr || contains(rigidBoundaries_, b)) return false;
    rigidBoundaries_.push_back(b);
    if (fluidSolver_) fluidSolver_->addRigidBoundary(b);
    return true;
}

bool PhysicsSolver::removeRigidBoundary(RigidBoundary* b)
{
    if (!erase(rigidBoundaries_, b)) return false;
    if (fluidSolver_) fluidSolver_->removeRigidBoundary(b);
    return true;
}

bool PhysicsSolver::addRigidBoundaryParticles(RigidBoundaryParticles* p)
{
    if (p == nullptr || contains(rigidParticles_, p)) return false;
    rigidParticles_.push_back(p);
    if (fluidSolver_) {
        if (softFluidKernel_) p->computePsi(*softFluidKernel_, softFluidRestDensity_);
        p->clearAccumForce();
        fluidSolver_->addRigidBoundaryParticles(p);
    }
    return true;
}

bool PhysicsSolver::removeRigidBoundaryParticles(RigidBoundaryParticles* p)
{
    if (!erase(rigidParticles_, p)) return false;
    if (fluidSolver_) fluidSolver_->removeRigidBoundaryParticles(p);
    return true;
}

void PhysicsSolver::clearRigidBoundaryParticles()
{
    if (fluidSolver_) {
        for (auto* particles : rigidParticles_) fluidSolver_->removeRigidBoundaryParticles(particles);
    }
    rigidParticles_.clear();
}

SoftFluidBinding& PhysicsSolver::bindSoftBody(ISoftBody* body)
{
    auto& binding = softFluid_.bind(body);
    softParticles_.push_back(&binding.particles);
    if (fluidSolver_) fluidSolver_->addSoftBoundaryParticles(&binding.particles);
    return binding;
}

void PhysicsSolver::clearSoftBodyBindings()
{
    if (fluidSolver_) {
        for (auto* particles : softParticles_) fluidSolver_->removeSoftBoundaryParticles(particles);
    }
    softParticles_.clear();
    softFluid_.clearBindings();
}

void PhysicsSolver::setRunning(bool running)
{
    running_ = running;
    rigidFluid_.rigidWorld().setRunning(running);
    softFluid_.softWorld().setRunning(running);
}

void PhysicsSolver::step()
{
    if (!running_) return;
    stepUnconditional();
}

// NOTE on the Rigid<->Soft ordering below: applyTwoWayReactions() measures
// the soft-body reaction against *last frame's* (already-integrated) soft
// particle positions, then the rigid body integrates this frame including
// that reaction, and only after that does softFluid_.stepForced() run XPBD
// (which pushes soft particles out against *this frame's* rigid pose via
// RigidBodyCollider). So the two directions are one frame out of phase with
// each other (Gauss-Seidel-style), unlike RigidFluidSolver's Two-Way, where
// syncBoundaries() snapshots the boundary before the fluid step so both
// directions see the same frame's pose. Acceptable for the SDF-penalty
// approximation this bridges uses (see internal design notes).
void PhysicsSolver::stepUnconditional()
{
    rigidFluid_.syncBoundaries();
    softFluid_.syncBoundaries(softFluidKernel_, softFluidRestDensity_);
    stepFluid();
    rigidSoft_.applyTwoWayReactions(softSolver());  // reaction added before rigid bodies integrate
    rigidFluid_.stepForced(timeStep_);              // rigid bodies integrate here (incl. the reaction above)
    softFluid_.stepForced(timeStep_);                // XPBD runs; RigidBodyCollider pushes against the now-integrated rigid pose
}

void PhysicsSolver::stepFluid()
{
    if (fluidSolver_) fluidSolver_->simulate(timeStep_, maxIter_);
}

} // namespace Physics
} // namespace Phantom
