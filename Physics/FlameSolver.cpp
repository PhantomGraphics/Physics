#include "pch.h"

#include "FlameSolver.h"
#include "FlameParticle.h"
#include "FlameFluid.h"
#include "FlameSolidCoupler.h"
#ifdef _OPENMP
#include <omp.h>
#endif

#include "CGLib/Space/Space/NeighborList.h"
#include "CGLib/ThirdParty/glm-0.9.9.8/glm/gtc/noise.hpp"

using namespace Phantom::Math;
using namespace Phantom::Space;
using namespace Phantom::Physics;

namespace {
// Small scenes pay more for OpenMP team synchronization than they gain from
// parallel work. Include neighbor search in this policy, and restore the
// calling thread's configuration when the substep ends.
class FlameThreadBudget {
public:
    explicit FlameThreadBudget(size_t count) {
#ifdef _OPENMP
        previous_ = omp_get_max_threads();
        if (count < 4096) omp_set_num_threads(1);
#endif
    }
    ~FlameThreadBudget() {
#ifdef _OPENMP
        omp_set_num_threads(previous_);
#endif
    }
private:
#ifdef _OPENMP
    int previous_ = 1;
#endif
};

// Upper bound on kappa*dt/h^2 for the explicit scalar diffusion pass.
constexpr float kMaxDiffusionNumber = 0.1f;

// Standard curl-noise construction: evaluate a 3-component vector potential
// field (three decorrelated Perlin fields) and take its curl via central
// finite differences. Purely decorative (idea doc section 2: "実質的なコスト
// なしで見た目を稼げる"), not meant to be physically accurate.
//
// The potential is 4D noise -- (x, y, z) * frequency plus time * timeScale on
// the 4th axis -- so the flow pattern evolves instead of being frozen in space
// (plan A2). timeScale == 0 reproduces the old static 3D-pattern behaviour
// (a fixed 4th coordinate still gives a valid, divergence-free field).
Vector3df samplePotential(const Vector3df& p, const float frequency, const float w)
{
	const Vector3df sp = p * frequency;
	return Vector3df(
		glm::perlin(glm::vec4(sp + Vector3df(37.2f, 17.1f, 91.3f), w)),
		glm::perlin(glm::vec4(sp + Vector3df(3.7f, 61.9f, 5.5f), w + 13.7f)),
		glm::perlin(glm::vec4(sp + Vector3df(71.1f, 2.4f, 43.8f), w + 29.3f)));
}

Vector3df curlNoise(const Vector3df& p, const float frequency, const float w)
{
	constexpr float eps = 0.01f;

	const auto dx = (samplePotential(p + Vector3df(eps, 0.0f, 0.0f), frequency, w) -
		samplePotential(p - Vector3df(eps, 0.0f, 0.0f), frequency, w)) / (2.0f * eps);
	const auto dy = (samplePotential(p + Vector3df(0.0f, eps, 0.0f), frequency, w) -
		samplePotential(p - Vector3df(0.0f, eps, 0.0f), frequency, w)) / (2.0f * eps);
	const auto dz = (samplePotential(p + Vector3df(0.0f, 0.0f, eps), frequency, w) -
		samplePotential(p - Vector3df(0.0f, 0.0f, eps), frequency, w)) / (2.0f * eps);

	return Vector3df(
		dy.z - dz.y,
		dz.x - dx.z,
		dx.y - dy.x);
}

}

void FlameSolver::simulate(const float dt)
{
	if (!std::isfinite(dt) || dt <= 0) return;
	// A solid coupler currently adds carriers. Reject the incompatible combination.
	if (solidCoupler_) for (auto* fluid : fluids) if (fluid->fixedCarriers) return;
	// Fixed mode substeps rather than changing the requested diffusion coefficient.
	float stableDt = dt;
	for (auto* fluid : fluids) if (fluid->fixedCarriers) {
		const float k = std::max({fluid->getThermalDiffusivity(), fluid->getFuelDiffusivity(),
			fluid->getOxygenDiffusivity(), fluid->getSootDiffusivity()});
		if (k > 0) stableDt = std::min(stableDt, kMaxDiffusionNumber * effectLength * effectLength / k);
		stableDt = std::min(stableDt, 0.25f * effectLength /
			(std::max(0.01f, fluid->getMaxSpeed()) + std::sqrt(std::max(0.0f, fluid->getPressureCoe()))));
	}
	if (dt > stableDt * 1.001f) {
		const int steps = static_cast<int>(std::ceil(dt / stableDt));
		for (int i = 0; i < steps; ++i) simulate(dt / steps);
		return;
	}
	if (solidCoupler_) for (auto* fluid : fluids) {
		fluid->setCombustionModel(FlameFluid::CombustionModel::Physical);
		solidCoupler_->update(*fluid,dt,simTime_);
	}
	std::vector<FlameParticle> particles;
	for (auto fluid : fluids) {
		auto& soa = fluid->getParticles();
		for (size_t i = 0; i < soa.size(); ++i) {
			particles.emplace_back(soa, i, fluid);
			if (sphere_) {
				auto& p = particles.back();
				const float allowed = std::max(0.0f, sphere_->getRadius()-soa.radii[i]);
				const auto offset = p.getPosition()-sphere_->getCenter(); const float distance = Math::getLength(offset);
				if (distance > allowed && distance > 0) {
					const auto normal=offset/distance;
					fluid->maxAttemptedWallPenetration=std::max(fluid->maxAttemptedWallPenetration,distance-allowed);
					const float speed2=Math::getLengthSquared(p.getVelocity());
					p.move(sphere_->getCenter()+normal*allowed-p.getPosition());
					p.setVelocity(p.getVelocity()-normal*std::max(0.0f,glm::dot(p.getVelocity(),normal)));
					fluid->boundaryEnergyLoss+=0.5*p.getMass()*(speed2-Math::getLengthSquared(p.getVelocity()));
				}
			}
		}
	}

	SPHKernel kernel(effectLength);
	for (auto& p : particles) {
		p.setKernel(&kernel);
	}

	for (auto& p : particles) {
		p.init();
	}

	std::vector<Vector3df> positions;
	positions.reserve(particles.size());
	for (const auto& p : particles) {
		positions.push_back(p.getPosition());
	}

	// Per-particle neighbor rows, not the raw pair list. The passes below used
	// to be parallelized over pairs, with each iteration writing to *both*
	// endpoints -- so two pairs sharing a particle could land on different
	// threads and race on that particle's density/force/vorticity accumulators
	// (non-atomic +=). Gathering per-particle means each thread only ever
	// writes to particles[i]; this is the same fix WCSPHSolver/PBSPHSolver
	// already carry, which FlameSolver had not picked up.
	const int particleCount = static_cast<int>(particles.size());
	const FlameThreadBudget threadBudget(particles.size());
	CSRNeighborList neighbors;
	neighbors.build(positions, effectLength);
	// Mirror samples across each near-wall particle's local tangent plane.
	// These supply missing kernel support and pressure reaction without adding carriers.
	const auto mirror = [&](int i, int j) {
		const auto offset = positions[i]-sphere_->getCenter();
		const float distance = Math::getLength(offset);
		const auto normal = distance > 0 ? offset/distance : Vector3df(0,1,0);
		const auto wall = sphere_->getCenter()+normal*sphere_->getRadius();
		return positions[j]+normal*(2*glm::dot(wall-positions[j],normal));
	};
	const auto nearWall = [&](int i) {
		return sphere_ && particles[i].getFluid()->fixedCarriers && sphere_->getSignedDistance(positions[i]) < effectLength;
	};

	// ---- Density pass -------------------------------------------------------
#pragma omp parallel for
	for (int i = 0; i < particleCount; ++i) {
		for (const int neighbor : neighbors[i]) {
			particles[i].addDensity(particles[neighbor]);
			if (nearWall(i)) particles[i].addDensity(particles[neighbor].getMass()*
				kernel.getPoly6Kernel(Math::getDistance(positions[i],mirror(i,neighbor))));
		}
		if (nearWall(i)) particles[i].addDensity(particles[i].getMass()*
			kernel.getPoly6Kernel(Math::getDistance(positions[i],mirror(i,i))));
	}
	for (auto& p : particles) {
		p.addSelfDensity();
	}

	// ---- Scalar diffusion pass (plan Phase 2 item 2) --------------------------
	// Cleary-Monaghan SPH diffusion of temperature / fuel / oxygen / soot:
	//   dA_i/dt = sum_j (m_j / rho_ij) * 2 kappa * (A_j - A_i) * F_ij,
	//   F_ij = -(r_ij . gradW_ij) / (|r_ij|^2 + eta^2) = |r|^2 w(r) / (|r|^2 + eta^2)
	// with w(r) = |gradW|/r (SPHKernel::getSpikyKernelGradientWeight(), >= 0)
	// and rho_ij the pair's mean density. Every factor is symmetric in (i, j)
	// and (A_j - A_i) is antisymmetric, so sum_i m_i A_i is conserved exactly
	// by the pass. Gathered per particle (writes only rates[i]), per the
	// neighbor-search convention in Physics/CLAUDE.md.
	std::vector<FlameScalarRates> rates(particleCount);
	const float eta2 = 0.01f * effectLength * effectLength;
	// Explicit diffusion is stable for kappa*dt/h^2 below ~0.1 (plan section 6);
	// clamp instead of letting a UI slider blow the step up.
	const float kappaMax = (dt > 0.0f) ? kMaxDiffusionNumber * effectLength * effectLength / dt : 0.0f;
	const auto clampKappa = [kappaMax](const float k) { return std::clamp(k, 0.0f, kappaMax); };
#pragma omp parallel for
	for (int i = 0; i < particleCount; ++i) {
		auto& pi = particles[i];
		const auto* fluid = pi.getFluid();
		const float kT = clampKappa(fluid->getThermalDiffusivity());
		const float kF = clampKappa(fluid->getFuelDiffusivity());
		const float kO = clampKappa(fluid->getOxygenDiffusivity());
		const float kS = clampKappa(fluid->getSootDiffusivity());
		if (kT == 0.0f && kF == 0.0f && kO == 0.0f && kS == 0.0f) {
			continue;
		}
		FlameScalarRates r;
		const auto posI = pi.getPosition();
		for (const int j : neighbors[i]) {
			const auto& pj = particles[j];
			if (solidCoupler_ && solidCoupler_->occluded(posI,pj.getPosition())) continue;
			const float dist = Math::getDistance(posI, pj.getPosition());
			const float d2 = dist * dist;
			const float F = d2 * kernel.getSpikyKernelGradientWeight(dist) / (d2 + eta2);
			const float coef = 2.0f * pj.getMass() / (0.5f * (pi.getDensity() + pj.getDensity())) * F;
			r.temperature += kT * coef * (pj.getTemperature() - pi.getTemperature());
			r.fuel += kF * coef * (pj.getFuel() - pi.getFuel());
			r.oxygen += kO * coef * (pj.getOxygen() - pi.getOxygen());
			r.soot += kS * coef * (pj.getSoot() - pi.getSoot());
		}
		rates[i] = r;
	}

	bool conservative = solidCoupler_ != nullptr;
	for (auto* fluid : fluids) conservative |= fluid->fixedCarriers;
	if (conservative) {
		// A common limiter preserves the antisymmetric mass-weighted diffusion
		// flux. Per-particle clamping in react() otherwise silently loses fuel.
		float fuelScale=1,oxygenScale=1,temperatureScale=1,sootScale=1;
		const auto limit=[dt](float value,float rate) { const float delta=rate*dt;
			return delta>0?std::min(1.0f,(1-value)/delta):delta<0?std::min(1.0f,-value/delta):1.0f; };
		for(int i=0;i<particleCount;++i) {
			fuelScale=std::min(fuelScale,limit(particles[i].getFuel(),rates[i].fuel));
			oxygenScale=std::min(oxygenScale,limit(particles[i].getOxygen(),rates[i].oxygen));
			if (rates[i].temperature < 0) temperatureScale = std::min(temperatureScale,
				particles[i].getTemperature() / (-dt * rates[i].temperature));
			if (rates[i].soot < 0) sootScale = std::min(sootScale,
				particles[i].getSoot() / (-dt * rates[i].soot));
		}
		for(auto& r:rates) { r.fuel*=fuelScale; r.oxygen*=oxygenScale;
			r.temperature*=temperatureScale; r.soot*=sootScale; }
	}
	// ---- Pressure + viscosity pass -------------------------------------------
#pragma omp parallel for
	for (int i = 0; i < particleCount; ++i) {
		for (const int neighbor : neighbors[i]) {
			particles[i].solvePressureForce(particles[neighbor]);
			particles[i].solveViscosityForce(particles[neighbor]);
			if (nearWall(i)) particles[i].addForce(kernel.getSpikyKernelGradient(positions[i]-mirror(i,neighbor))*
				(0.5f*(particles[i].getPressure()+particles[neighbor].getPressure())*particles[neighbor].getMass()));
		}
		if (nearWall(i)) particles[i].addForce(kernel.getSpikyKernelGradient(positions[i]-mirror(i,i))*
			(particles[i].getPressure()*particles[i].getMass()));
	}

	// ---- Vorticity confinement: pass A (omega), pass B (grad |omega|) -------
	// Two passes, not one: pass B reads every neighbor's omega, so pass A must
	// have finished accumulating it for all particles first.
#pragma omp parallel for
	for (int i = 0; i < particleCount; ++i) {
		for (const int neighbor : neighbors[i]) {
			particles[i].addVorticity(particles[neighbor]);
		}
	}
#pragma omp parallel for
	for (int i = 0; i < particleCount; ++i) {
		for (const int neighbor : neighbors[i]) {
			particles[i].addVorticityGradient(particles[neighbor]);
		}
	}

	// ---- Per-particle forces: vorticity confinement + Boussinesq buoyancy
	// + curl noise ------------------------------------------------------------
	// Curl noise is an *acceleration* (strength in length/s^2) added as a force
	// before integration, so it scales with dt and goes through forwardTime()'s
	// maxSpeed cap like every other force (plan A1). It used to be added to the
	// velocity after integration with no dt -- i.e. strength*60 m/s^2 at the
	// default 1/60 step, bypassing the speed cap (curlNoiseStrength defaults
	// were multiplied by 60 when this changed, so the look is preserved).
	const float noiseW = simTime_;
#pragma omp parallel for
	for (int i = 0; i < particleCount; ++i) {
		auto& p = particles[i];
		const auto* fluid = p.getFluid();
		p.applyVorticityConfinement(fluid->getVorticityEps(), effectLength);
		p.applyBuoyancy(gravity);
		const float strength = fluid->getCurlNoiseStrength();
		if (strength > 0.0f) {
			const auto acc = curlNoise(p.getPosition(), fluid->getCurlNoiseFrequency(),
				noiseW * fluid->getCurlNoiseTimeScale()) * strength;
			p.addForce(acc * p.getDensity());
		}
	}

	this->addBoundaryForce(particles, dt);

	// ---- Integrate and react -------------------------------------------------
	std::vector<float> oldTemperature(particleCount), oldOxygen(particleCount), oldFuel(particleCount);
	for (int i = 0; i < particleCount; ++i) {
		oldTemperature[i] = particles[i].getTemperature();
		oldOxygen[i] = particles[i].getOxygen();
		oldFuel[i] = particles[i].getFuel();
	}
#pragma omp parallel for
	for (int i = 0; i < particleCount; ++i) {
		particles[i].forwardTime(dt);
		particles[i].react(dt, &rates[i]);
	}
	// Serial accounting avoids writing shared fluid counters from OpenMP workers.
	for (int i = 0; i < particleCount; ++i) {
		auto& p = particles[i]; auto* fluid = p.getFluid();
		const float mass = p.getMass();
		const float before = oldTemperature[i] + rates[i].temperature * dt;
		const double burned = (oldOxygen[i] + rates[i].oxygen * dt - p.getOxygen()) * mass;
		const double reaction = (oldFuel[i] + rates[i].fuel * dt - p.getFuel()) * mass * fluid->getHeatRelease();
		const double cooling = mass * fluid->getCoolRate() * (before - fluid->getAmbientTemperature()) * dt;
		if (fluid->fixedCarriers) {
			fluid->consumedOxygen += burned;
			fluid->reactionHeat += reaction; fluid->coolingHeat += cooling;
			fluid->clampHeat += mass * (before - p.getTemperature()) + reaction - cooling;
		}
		if (sphere_) {
			const float radius = std::max(0.0f, sphere_->getRadius() - 0.5f * p.getDiameter());
			const auto offset = p.getPosition() - sphere_->getCenter(); const float distance = Math::getLength(offset);
			if (distance > radius && distance > 0) {
				const auto normal = offset / distance;
				fluid->maxAttemptedWallPenetration=std::max(fluid->maxAttemptedWallPenetration,distance-radius);
				const float speed2=Math::getLengthSquared(p.getVelocity());
				p.move(sphere_->getCenter() + normal * radius - p.getPosition());
				p.setVelocity(p.getVelocity() - normal * std::max(0.0f, glm::dot(p.getVelocity(), normal)));
				fluid->boundaryEnergyLoss+=0.5*mass*(speed2-Math::getLengthSquared(p.getVelocity()));
			}
		}
	}
	applyThermalBoundary(dt);
	for (auto* fluid : fluids) {
		for (float amount : fluid->getParticles().lastBurnedMass) fluid->burnedFuelMass += amount;
		if (solidCoupler_) {
			std::vector<Vector3df> old;
			for (size_t i=0;i<particles.size();++i) if (particles[i].getFluid() == fluid) old.push_back(positions[i]);
			solidCoupler_->constrain(*fluid,old);
		}
	}
	simTime_ += dt;

	// ---- Pilot (wick) heating: keeps the Physical model's flame anchored ------
	for (auto fluid : fluids) {
		if (!fluid->fixedCarriers) fluid->applyPilots();
	}

	// ---- Emitters and lifetime ------------------------------------------------
	for (auto fluid : fluids) {
		if (!fluid->fixedCarriers) fluid->updateEmitters(dt);
		if (solidCoupler_) solidCoupler_->constrain(*fluid,{});
		fluid->removeDead();
	}

	// ---- Secondary particles: sparks/smoke (cosmetic, non-SPH) --------------
	for (auto fluid : fluids) {
		fluid->updateSecondaryParticles(dt, gravity);
	}
}

void FlameSolver::addBoundaryForce(std::vector<FlameParticle>& particles, const float dt)
{
	if (boundaryPlanes_.empty() && !sphere_) {
		return;
	}
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		Vector3df force(0.0f, 0.0f, 0.0f);
		for (const auto& plane : boundaryPlanes_) {
			force += plane.getBoundaryForce(particles[i].getPosition(), dt);
		}
		if (sphere_) {
			const SphereBoundary wall(sphere_->getCenter(), std::max(0.001f,
				sphere_->getRadius() - particles[i].getDiameter() * 0.5f));
			force += wall.getBoundaryForce(particles[i].getPosition(), particles[i].getVelocity(), dt, sphereDamping_);
		}
		particles[i].addForce(force * particles[i].getDensity());
	}
}

bool FlameSolver::setBoundarySphere(const Vector3df& center, float radius, float damping)
{
	if (!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(center.z) ||
		!std::isfinite(radius) || radius <= 0 || !std::isfinite(damping) || damping < 0 || damping > 0.5f) return false;
	sphere_.emplace(center, radius); sphereDamping_ = damping; boundaryPlanes_.clear(); return true;
}

bool FlameSolver::setThermalBoundary(const ThermalBoundary& t)
{
	if (!std::isfinite(t.sourceCenter.x) || !std::isfinite(t.sourceCenter.y) || !std::isfinite(t.sourceCenter.z)) return false;
	for (float v : {t.sourceRadius, t.sourcePower, t.sourceDuration, t.wallTemperature, t.wallRate, t.wallThickness})
		if (!std::isfinite(v) || v < 0) return false;
	if (t.sourceRadius <= 0 || t.wallThickness <= 0) return false;
	thermal_ = t; return true;
}

void FlameSolver::applyThermalBoundary(float dt)
{
	if (!sphere_) return;
	const float activeDt = thermal_.sourceDuration > 0 ?
		std::clamp(thermal_.sourceDuration - simTime_, 0.0f, dt) : dt;
	for (auto* fluid : fluids) {
		if (!fluid->fixedCarriers) continue;
		auto& gas = fluid->getParticles(); double sourceMass = 0;
		for (size_t i = 0; i < gas.size(); ++i) if (Math::getDistance(gas.positions[i], thermal_.sourceCenter) <= thermal_.sourceRadius)
			sourceMass += FlameParticle(gas, i, fluid).getMass();
		for (size_t i = 0; i < gas.size(); ++i) {
			const float mass = FlameParticle(gas, i, fluid).getMass();
			if (sourceMass > 0 && Math::getDistance(gas.positions[i], thermal_.sourceCenter) <= thermal_.sourceRadius) {
				const float delta = static_cast<float>(thermal_.sourcePower * activeDt / sourceMass);
				const float before = gas.temperatures[i];
				gas.temperatures[i] = std::min(fluid->getMaxTemperature(), before + delta);
				fluid->sourceHeat += mass * delta;
				fluid->clampHeat += mass * (before + delta - gas.temperatures[i]);
			}
			const float wallDistance = sphere_->getSignedDistance(gas.positions[i]) - gas.radii[i];
			const float weight = std::clamp(1 - wallDistance / thermal_.wallThickness, 0.0f, 1.0f);
			const float before = gas.temperatures[i];
			gas.temperatures[i] = thermal_.wallTemperature + (before - thermal_.wallTemperature) *
				std::exp(-thermal_.wallRate * weight * dt);
			fluid->wallHeat += mass * (before - gas.temperatures[i]);
		}
	}
}
