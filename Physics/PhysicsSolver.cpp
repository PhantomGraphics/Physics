#include "pch.h"
#include "PhysicsSolver.h"

namespace Phantom {
namespace Physics {

RigidFluidBinding& PhysicsSolver::bindRigidBody(RigidBody* body, ICollisionShape* shape, CouplingMode mode)
{
    auto& binding = rigidFluid_.bind(body, shape, mode);
    addRigidBoundary(&binding.boundary);
    return binding;
}

void PhysicsSolver::clearRigidBodyBindings()
{
    rigidFluid_.clearBindings();
    clearRigidBoundaries();
    clearRigidBoundaryParticles();
}

void PhysicsSolver::addRigidBoundary(RigidBoundary* b)
{
    if (fluidSolver_) fluidSolver_->addRigidBoundary(b);
}

void PhysicsSolver::clearRigidBoundaries()
{
    if (fluidSolver_) fluidSolver_->clearRigidBoundaries();
}

void PhysicsSolver::addRigidBoundaryParticles(RigidBoundaryParticles* p)
{
    if (fluidSolver_) fluidSolver_->addRigidBoundaryParticles(p);
}

void PhysicsSolver::clearRigidBoundaryParticles()
{
    if (fluidSolver_) fluidSolver_->clearRigidBoundaryParticles();
}

SoftFluidBinding& PhysicsSolver::bindSoftBody(ISoftBody* body)
{
    auto& binding = softFluid_.bind(body);
    if (fluidSolver_) fluidSolver_->addSoftBoundaryParticles(&binding.particles);
    return binding;
}

void PhysicsSolver::clearSoftBodyBindings()
{
    softFluid_.clearBindings();
    if (fluidSolver_) fluidSolver_->clearSoftBoundaryParticles();
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
