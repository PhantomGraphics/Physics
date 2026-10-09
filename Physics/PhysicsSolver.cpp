#include "pch.h"
#include "PhysicsSolver.h"

namespace Phantom {
namespace Physics {

void PhysicsSolver::setFluidSolver(ISPHSolver* solver)
{
    if (solver == fluidSolver_) return;
    if (fluidSolver_) {
        fluidSolver_->clearRigidBoundaries();
        fluidSolver_->clearRigidBoundaryParticles();
        fluidSolver_->clearSoftBoundaryParticles();
    }
    fluidSolver_ = solver;
    softFluidKernel_ = solver ? solver->getKernel() : nullptr;
    softFluidRestDensity_ = solver ? solver->getRestDensity() : 0.f;
    if (!solver) return;
    for (auto* boundary : rigidBoundaries_) solver->addRigidBoundary(boundary);
    for (auto* particles : rigidParticles_) {
        if (softFluidKernel_) particles->computePsi(*softFluidKernel_, softFluidRestDensity_);
        particles->clearAccumForce();
        solver->addRigidBoundaryParticles(particles);
    }
    for (auto* particles : softParticles_) solver->addSoftBoundaryParticles(particles);
}

RigidFluidBinding& PhysicsSolver::bindRigidBody(RigidBody* body, ICollisionShape* shape, CouplingMode mode)
{
    auto& binding = rigidFluid_.bind(body, shape, mode);
    addRigidBoundary(&binding.boundary);
    return binding;
}

void PhysicsSolver::clearRigidBodyBindings()
{
    clearRigidBoundaries();
    clearRigidBoundaryParticles();
    rigidFluid_.clearBindings();
}

void PhysicsSolver::addRigidBoundary(RigidBoundary* b)
{
    rigidBoundaries_.push_back(b);
    if (fluidSolver_) fluidSolver_->addRigidBoundary(b);
}

void PhysicsSolver::clearRigidBoundaries()
{
    rigidBoundaries_.clear();
    if (fluidSolver_) fluidSolver_->clearRigidBoundaries();
}

void PhysicsSolver::addRigidBoundaryParticles(RigidBoundaryParticles* p)
{
    rigidParticles_.push_back(p);
    if (fluidSolver_) fluidSolver_->addRigidBoundaryParticles(p);
}

void PhysicsSolver::clearRigidBoundaryParticles()
{
    rigidParticles_.clear();
    if (fluidSolver_) fluidSolver_->clearRigidBoundaryParticles();
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
    if (fluidSolver_) fluidSolver_->clearSoftBoundaryParticles();
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
