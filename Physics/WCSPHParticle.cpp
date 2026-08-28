#include "pch.h"

#include "WCSPHParticle.h"
#include "WCSPHFluid.h"
#include "SPHKernel.h"

#include "CGLib/Math/Vector3d.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

void WCSPHParticle::init()
{
	densityRef() = 0.0f;
	normalRef() = Math::Vector3df(0.0f, 0.0f, 0.0f);
	forceRef() = Math::Vector3df(0.0f, 0.0f, 0.0f);
}

float WCSPHParticle::getDensityRatio() const
{
	return getDensity() / fluid_->getDensity();
}

float WCSPHParticle::getPressure() const
{
	return fluid_->getPressureCoe() * std::max(0.0f, getDensity() - fluid_->getDensity());
}

float WCSPHParticle::getMass() const
{
	return fluid_->getDensity() * std::pow(getDiameter(), 3.0f);
}

float WCSPHParticle::getVolume() const
{
	return getMass() / getDensity();
}

float WCSPHParticle::getRestVolume() const
{
	return getMass() / fluid_->getDensity();
}

void WCSPHParticle::forwardTime(const float timeStep)
{
	const auto acc = getForce() / getDensity(); //getAccelaration() / getDensity();
	velocityRef() += (acc * timeStep);
	positionRef() += (getVelocity() * timeStep);
}

void WCSPHParticle::addExternalForce(const Vector3df& externalForce)
{
	forceRef() += externalForce;
}

void WCSPHParticle::solveNormal(const WCSPHParticle& rhs)
{
	const auto& distanceVector = this->getPosition() - rhs.getPosition();
	normalRef() += kernel_->getPoly6KernelGradient(distanceVector) * rhs.getVolume();
	//pairs[i].getParticle1()->addForce(viscosityCoe * velocityDiff * kernel.getViscosityKernelLaplacian(distance, effectLength) * pairs[i].getParticle2()->getVolume());
}

void WCSPHParticle::solveSurfaceTension(const WCSPHParticle& rhs)
{
	const auto distance = Math::getDistance(this->getPosition(), rhs.getPosition());
	const auto tensionCoe = (this->fluid_->getTensionCoe() + rhs.getFluid()->getTensionCoe()) * 0.5f;
	forceRef() -= tensionCoe * kernel_->getPoly6KernelLaplacian(distance) * rhs.getMass() * surfaceNormalHat();
}

Vector3df WCSPHParticle::surfaceNormalHat() const
{
	const auto normal = getNormal();
	const auto effectLength = kernel_->getEffectLength();
	// solveNormal() accumulates Sum (m_j/rho_j) * gradient(Poly6), which has
	// units of 1/length (Poly6 gradient ~ 1/length^4, volume ~ length^3), so
	// a raw squared-magnitude epsilon like the pre-SoA code's "< 0.1f" would
	// mean a different relative sensitivity at every scene scale. Multiplying
	// by effectLength^2 nondimensionalizes the comparison instead.
	constexpr float kSurfaceThresholdSq = 1.0e-4f;
	if (Math::getLengthSquared(normal) * effectLength * effectLength < kSurfaceThresholdSq) {
		return Vector3df(0.0f, 0.0f, 0.0f);
	}
	return glm::normalize(normal);
}

// Deliberately weight by rhs.getMass() (constant rest mass), not
// rhs.getVolume() (= mass/density, as solveNormal() above uses). Muller et
// al. 2003's textbook formula for both of these forces does divide by the
// neighbor's *current* density (f_pressure_i = -Sum_j m_j (p_i+p_j)/(2*rho_j)
// * gradW, f_viscosity_i analogous with the viscosity Laplacian), i.e. uses
// getVolume() -- but empirically that 1/rho_j blows up for low-density
// (sparse/free-surface) neighbors and explodes real scenes (verified
// 2026-08-17: swapping to getVolume() here made
// FluidPoolStabilityTest.WCSPH_PoolSettlesInBoxWithoutExploding and
// WCSPHSolverTest.SurfaceTensionBlockStaysFiniteWhileSettling diverge to
// hundreds of units in under a second). Mass-weighting is the safer,
// intentional choice here even though it disagrees with solveNormal()'s
// convention.
void WCSPHParticle::solvePressureForce(const WCSPHParticle& rhs)
{
	const auto pressure = (this->getPressure() + rhs.getPressure()) * 0.5f;
	const auto& distanceVector = (this->getPosition() - rhs.getPosition());
	const auto& f = kernel_->getSpikyKernelGradient(distanceVector) * pressure * rhs.getMass();
	forceRef() += f;
}

void WCSPHParticle::solveViscosityForce(const WCSPHParticle& rhs)
{
	const auto viscosityCoe = (this->fluid_->getViscosityCoe() + rhs.getFluid()->getViscosityCoe()) * 0.5f;
	const auto& velocityDiff = (rhs.getVelocity() - this->getVelocity());
	const auto distance = Math::getDistance(this->getPosition(), rhs.getPosition());
	this->addForce(viscosityCoe * velocityDiff * kernel_->getViscosityKernelLaplacian(distance) * rhs.getMass());
}

void WCSPHParticle::addSelfDensity()
{
	this->addDensity(kernel_->getPoly6Kernel(0.0) * this->getMass());
}

void WCSPHParticle::addDensity(const WCSPHParticle& rhs)
{
	const float distance = Math::getDistance(this->getPosition(), rhs.getPosition());
	this->addDensity(kernel_->getPoly6Kernel(distance) * rhs.getMass());
}

void WCSPHParticle::move(const Vector3df& v)
{
	positionRef() += v;
}
