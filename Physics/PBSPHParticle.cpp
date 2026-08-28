#include "pch.h"

#include "PBSPHParticle.h"

#include "SPHKernel.h"

#include "PBSPHFluid.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

void PBSPHParticle::init()
{
	soa_->densities[index_] = 0.0f;
	dxRef() = Math::Vector3df(0.0f, 0.0f, 0.0f);
	Math::Vector3df& normal = soa_->normals[index_];
	normal = Math::Vector3df(0.0f, 0.0f, 0.0f);
	forceRef() = Math::Vector3df(0.0f, 0.0f, 0.0f);
	predictPositionRef() = getPosition();
}

float PBSPHParticle::getDensityRatio() const
{
	return getDensity() / fluid_->getRestDensity();
}

float PBSPHParticle::getMass() const
{
	const auto diameter = soa_->radii[index_] * 2.0f;
	return fluid_->getRestDensity() * diameter * diameter * diameter;
}

float PBSPHParticle::getVolume() const
{
	return getMass() / getDensity();
}

float PBSPHParticle::getRestVolume() const
{
	return getMass() / fluid_->getRestDensity();
}

void PBSPHParticle::setDefaultDensity()
{
	setDensity(fluid_->getRestDensity());
}

void PBSPHParticle::forwardTime(const float timeStep)
{
	const auto& acc = getAccelaration();
	velocityRef() += (acc * timeStep);
	positionRef() += (getVelocity() * timeStep);
}

void PBSPHParticle::addExternalForce(const Vector3df& externalForce)
{
	if (fluid_->isBoundary()) {
		return;
	}
	forceRef() += externalForce * fluid_->getRestDensity();
}

void PBSPHParticle::addSelfDensity()
{
	addDensity(getKernel()->getPoly6Kernel(0.0) * getMass());
}

void PBSPHParticle::addDensity(const PBSPHParticle& rhs)
{
	const float distance = glm::distance(getPredictPosition(), rhs.getPredictPosition());
	addDensity(fluid_->getKernel()->getPoly6Kernel(distance) * rhs.getMass());
}

void PBSPHParticle::addDensity(const float distance, const float mass)
{
	addDensity(getKernel()->getPoly6Kernel(distance) * mass);
}

void PBSPHParticle::predictPosition_(const float dt)
{
	if (fluid_->isBoundary()) {
		return;
	}
	positionRef() = getPredictPosition();
	velocityRef() += dt * getForce();
	predictPositionRef() += dt * getVelocity();
}

void PBSPHParticle::updatePredictPosition()
{
	if (fluid_->isBoundary()) {
		return;
	}
	predictPositionRef() += getDx();
}

void PBSPHParticle::updateVelocity(const float dt)
{
	if (fluid_->isBoundary()) {
		return;
	}
	velocityRef() = (getPredictPosition() - getPosition()) / dt;
}

void PBSPHParticle::updatePosition()
{
	if (fluid_->isBoundary()) {
		return;
	}
	positionRef() = getPredictPosition();
}

void PBSPHParticle::addPositionCorrection(const Vector3df& distanceVector)
{
	dxRef() += distanceVector;
}

float PBSPHParticle::getEffectLength() const
{
	return fluid_->getKernel()->getEffectLength();
}

void PBSPHParticle::accumulateConstraintGradient(const PBSPHParticle& rhs)
{
	const auto v = getPredictPosition() - rhs.getPredictPosition();
	// Weighted by the neighbor's mass, consistent with addDensity(rhs) above
	// and with how boundary particles already weight their contribution by
	// psi (PBSPHSolver::addRigidBoundaryParticleConstraintGradient()).
	// Without this, the density constraint C_i (mass-weighted) and its own
	// gradient (previously unweighted) used inconsistent units, making
	// lambda's natural magnitude scale as effectLength^8 instead of
	// effectLength^2 -- see docs/todo/PLAN_sph_scale_invariance.md section 4.
	const auto weight = getKernel()->getPoly6KernelGradient(v) * rhs.getMass();
	addConstraintGradient(weight);
}

void PBSPHParticle::calculateLambda()
{
	// Macklin & Muller 2013, Eq. 8-11: the constraint gradient wrt this
	// particle's own position is (1/rho0) * gradientSum (the self term),
	// and wrt each neighbor's position is -(1/rho0) * gradW_ij (same
	// magnitude as the per-neighbor term already summed into gradientSqSum).
	//
	// The textbook formula is lambda = -C / denom, which relies on the SPH
	// gradient pointing from this particle toward its neighbor. This
	// codebase's SPHKernel::getPoly6KernelGradient(v) does the opposite --
	// it returns v (this particle's position minus its neighbor's) scaled by
	// an unconditionally positive factor, i.e. it points *away* from the
	// neighbor. calculatePressure() and the Track B rigid-boundary coupling
	// both multiply this same gradient by lambda, so lambda's sign has to be
	// flipped here to match: a positive (not negative) lambda under
	// compression (C > 0), combined with the away-from-neighbor gradient,
	// is what actually pushes overlapping particles apart.
	const float rho0Sq = fluid_->getRestDensity() * fluid_->getRestDensity();
	const float denom = (Math::getLengthSquared(soa_->gradientSums[index_]) + soa_->gradientSqSums[index_]) / rho0Sq;
	float lambda = getConstraint() / std::max(denom, 1.0e-6f);

	// Clamp: a particle with few/no neighbors (e.g. isolated near a domain
	// wall, where neighbors beyond the wall don't exist) has a tiny
	// gradientSum/gradientSqSum, so denom collapses toward the 1.0e-6f floor
	// above and lambda can blow up to an enormous value even for a modest
	// density excess. That oversized lambda then feeds calculatePressure()'s
	// dx correction, which combined with a boundary correction acting the
	// same step produces a runaway feedback (confirmed empirically: a dam
	// break collapsing against a wall diverges to +-1000s without this
	// clamp, but settles into an ordinary, bounded puddle with it -- see
	// docs/todo/PLAN_rigid_fluid_coupling_phase8_status.md, "problem B").
	constexpr float kLambdaCap = 1.0f;
	lambda = std::max(-kLambdaCap, std::min(kLambdaCap, lambda));
	setLambda(lambda);
}

void PBSPHParticle::calculatePressure(const PBSPHParticle& rhs)
{
	const auto v = getPredictPosition() - rhs.getPredictPosition();
	// Mass-weighted for the same reason as accumulateConstraintGradient()
	// above -- matches the boundary-particle counterpart
	// (PBSPHSolver::addRigidBoundaryParticlePressure(), which already
	// weights its position correction by bp.psi).
	const auto weight = getKernel()->getPoly6KernelGradient(v) * rhs.getMass();
	const auto lambdaSum = getLambda() + rhs.getLambda();
	dxRef() += lambdaSum * weight / fluid_->getRestDensity() * fluid_->getStiffness();
}

void PBSPHParticle::calculateViscosity(const PBSPHParticle& rhs)
{
	const auto v = getPredictPosition() - rhs.getPredictPosition();
	const auto vel = rhs.getVelocity() - getVelocity();
	const auto weight = getKernel()->getViscosityKernelLaplacian(glm::length(v));
	xviscRef() += vel * weight * fluid_->getViscosity();
}

float PBSPHParticle::getConstraint() const
{
	return std::max(getDensityRatio() - 1.0f, 0.0f);
}

SPHKernel* PBSPHParticle::getKernel()
{
	return fluid_->getKernel();
}
