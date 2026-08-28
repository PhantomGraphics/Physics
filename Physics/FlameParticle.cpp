#include "pch.h"

#include "FlameParticle.h"
#include "FlameFluid.h"
#include "SPHKernel.h"

#include "CGLib/Math/Vector3d.h"

#include <algorithm>

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace {
// Thresholds for the "burned out and cooled" half of isDead(); not exposed as
// fluid parameters since the plan only calls for a loose lifetime heuristic.
constexpr float kFuelDeadEps = 0.02f;
constexpr float kTempAmbientEps = 5.0f;

// Boussinesq buoyancy is only valid for small relative density perturbations
// (deltaRho/rho0 << 1). Clamping it keeps applyBuoyancy() from exploding when
// UI sliders push thermalExpansion/ignitionTemperature far apart -- see
// FlameParticle::applyBuoyancy().
constexpr float kMaxRelativeDensityDelta = 0.5f;
}

float FlameParticle::getDensityRatio() const
{
	return getDensity() / fluid_->getDensity();
}

float FlameParticle::getPressure() const
{
	return fluid_->getPressureCoe() * std::max(0.0f, getDensity() - fluid_->getDensity());
}

float FlameParticle::getMass() const
{
	return fluid_->getDensity() * std::pow(getDiameter(), 3.0f);
}

float FlameParticle::getVolume() const
{
	return getMass() / getDensity();
}

float FlameParticle::getRestVolume() const
{
	return getMass() / fluid_->getDensity();
}

void FlameParticle::init()
{
	soa_->densities[index_] = 0.0f;
	forceRef() = Vector3df(0.0f, 0.0f, 0.0f);
	vorticityRef() = Vector3df(0.0f, 0.0f, 0.0f);
	vorticityGradAccumRef() = Vector3df(0.0f, 0.0f, 0.0f);
}

void FlameParticle::forwardTime(const float timeStep)
{
	const auto& acc = getForce() / getDensity();
	velocityRef() += acc * timeStep;

	// Hard safety net: bounds a single unstable step (e.g. a sparse-neighbor
	// pressure spike or an aggressive buoyancy/UI setting) from launching the
	// particle far outside the domain in one frame. See FlameFluid::maxSpeed.
	const float maxSpeed = fluid_->getMaxSpeed();
	const float speed = Math::getLength(getVelocity());
	if (speed > maxSpeed) {
		velocityRef() *= maxSpeed / speed;
	}

	positionRef() += getVelocity() * timeStep;
}

void FlameParticle::addExternalForce(const Vector3df& externalForce)
{
	forceRef() += externalForce;
}

void FlameParticle::solvePressureForce(const FlameParticle& rhs)
{
	const auto pressure = (getPressure() + rhs.getPressure()) * 0.5f;
	const auto& distanceVector = getPosition() - rhs.getPosition();
	forceRef() += kernel_->getSpikyKernelGradient(distanceVector) * pressure * rhs.getMass();
}

void FlameParticle::solveViscosityForce(const FlameParticle& rhs)
{
	const auto viscosityCoe = (fluid_->getViscosityCoe() + rhs.fluid_->getViscosityCoe()) * 0.5f;
	const auto& velocityDiff = rhs.getVelocity() - getVelocity();
	const auto distance = Math::getDistance(getPosition(), rhs.getPosition());
	forceRef() += viscosityCoe * velocityDiff * kernel_->getViscosityKernelLaplacian(distance) * rhs.getMass();
}

void FlameParticle::addSelfDensity()
{
	addDensity(kernel_->getPoly6Kernel(0.0f) * getMass());
}

void FlameParticle::addDensity(const FlameParticle& rhs)
{
	const float distance = Math::getDistance(getPosition(), rhs.getPosition());
	addDensity(kernel_->getPoly6Kernel(distance) * rhs.getMass());
}

void FlameParticle::move(const Vector3df& v)
{
	positionRef() += v;
}

void FlameParticle::react(const float dt)
{
	const float kBurn = fluid_->getBurnRate();
	const float ambient = fluid_->getAmbientTemperature();

	const float fuel = getFuel();
	const float temperature = getTemperature();

	const float dFuel = -kBurn * fuel;
	const float dTemp = kBurn * fuel * fluid_->getHeatRelease() - fluid_->getCoolRate() * (temperature - ambient);
	const float dSoot = kBurn * fuel * fluid_->getSootYield();

	setFuel(std::max(0.0f, fuel + dFuel * dt));
	setTemperature(temperature + dTemp * dt);
	setSoot(std::max(0.0f, getSoot() + dSoot * dt));
	setAge(getAge() + dt);
}

bool FlameParticle::isDead() const
{
	if (getAge() > fluid_->getLifeMax()) {
		return true;
	}
	if (isAir()) {
		// Air carrier particles start at fuel=0/temperature=ambient by design
		// (see FlameFluid::updateEmitters()); the cooling heuristic below would
		// otherwise mark them dead on the very step they spawn.
		return false;
	}
	const float ambient = fluid_->getAmbientTemperature();
	return getFuel() < kFuelDeadEps && std::abs(getTemperature() - ambient) < kTempAmbientEps;
}

void FlameParticle::applyBuoyancy(const Vector3df& gravity)
{
	const float rho0 = fluid_->getDensity();
	const float relDensityDelta = std::clamp(
		fluid_->getThermalExpansion() * (getTemperature() - fluid_->getAmbientTemperature()),
		-kMaxRelativeDensityDelta, kMaxRelativeDensityDelta);
	const float rhoEff = rho0 * (1.0f - relDensityDelta);
	// rhoEff < rho0 when hot (temperature > ambient); multiplying that negative
	// difference by a downward gravity vector yields a net *upward* force here.
	const Vector3df buoyancy = fluid_->getBuoyancyCoe() * (rhoEff - rho0) * gravity;
	forceRef() += buoyancy * getDensity();
}

void FlameParticle::addVorticity(const FlameParticle& rhs)
{
	const auto& distanceVector = getPosition() - rhs.getPosition();
	const auto& velocityDiff = rhs.getVelocity() - getVelocity();
	vorticityRef() += glm::cross(velocityDiff, kernel_->getSpikyKernelGradient(distanceVector)) * rhs.getVolume();
}

void FlameParticle::addVorticityGradient(const FlameParticle& rhs)
{
	const auto& distanceVector = getPosition() - rhs.getPosition();
	const float mag = Math::getLength(getVorticity());
	const float rhsMag = Math::getLength(rhs.getVorticity());
	vorticityGradAccumRef() += kernel_->getSpikyKernelGradient(distanceVector) * (rhsMag - mag) * rhs.getVolume();
}

void FlameParticle::applyVorticityConfinement(const float eps, const float h)
{
	const float len = Math::getLength(soa_->vorticityGradAccums[index_]);
	if (len < 1.0e-6f) {
		return;
	}
	const auto n = soa_->vorticityGradAccums[index_] / len;
	forceRef() += eps * glm::cross(n, getVorticity()) * h;
}
