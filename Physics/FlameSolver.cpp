#include "pch.h"

#include "FlameSolver.h"
#include "FlameParticle.h"
#include "FlameFluid.h"

#include "CGLib/Space/Space/NeighborList.h"
#include "CGLib/ThirdParty/glm-0.9.9.8/glm/gtc/noise.hpp"

using namespace Phantom::Math;
using namespace Phantom::Space;
using namespace Phantom::Physics;

namespace {

// Standard curl-noise construction: evaluate a 3-component vector potential
// field (three decorrelated Perlin fields) and take its curl via central
// finite differences. Purely decorative (idea doc section 2: "実質的なコスト
// なしで見た目を稼げる"), not meant to be physically accurate.
Vector3df samplePotential(const Vector3df& p, const float frequency)
{
	const Vector3df sp = p * frequency;
	return Vector3df(
		glm::perlin(sp + Vector3df(37.2f, 17.1f, 91.3f)),
		glm::perlin(sp + Vector3df(3.7f, 61.9f, 5.5f)),
		glm::perlin(sp + Vector3df(71.1f, 2.4f, 43.8f)));
}

Vector3df curlNoise(const Vector3df& p, const float frequency)
{
	constexpr float eps = 0.01f;

	const auto dx = (samplePotential(p + Vector3df(eps, 0.0f, 0.0f), frequency) -
		samplePotential(p - Vector3df(eps, 0.0f, 0.0f), frequency)) / (2.0f * eps);
	const auto dy = (samplePotential(p + Vector3df(0.0f, eps, 0.0f), frequency) -
		samplePotential(p - Vector3df(0.0f, eps, 0.0f), frequency)) / (2.0f * eps);
	const auto dz = (samplePotential(p + Vector3df(0.0f, 0.0f, eps), frequency) -
		samplePotential(p - Vector3df(0.0f, 0.0f, eps), frequency)) / (2.0f * eps);

	return Vector3df(
		dy.z - dz.y,
		dz.x - dx.z,
		dx.y - dy.x);
}

}

void FlameSolver::simulate(const float dt)
{
	std::vector<FlameParticle> particles;
	for (auto fluid : fluids) {
		auto& soa = fluid->getParticles();
		for (size_t i = 0; i < soa.size(); ++i) {
			particles.emplace_back(soa, i, fluid);
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
	CSRNeighborList neighbors;
	neighbors.build(positions, effectLength);

	// ---- Density pass -------------------------------------------------------
#pragma omp parallel for
	for (int i = 0; i < particleCount; ++i) {
		for (const int neighbor : neighbors[i]) {
			particles[i].addDensity(particles[neighbor]);
		}
	}
	for (auto& p : particles) {
		p.addSelfDensity();
	}

	// ---- Pressure + viscosity pass -------------------------------------------
#pragma omp parallel for
	for (int i = 0; i < particleCount; ++i) {
		for (const int neighbor : neighbors[i]) {
			particles[i].solvePressureForce(particles[neighbor]);
			particles[i].solveViscosityForce(particles[neighbor]);
		}
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

	// ---- Per-particle forces: vorticity confinement + Boussinesq buoyancy ---
	for (auto& p : particles) {
		p.applyVorticityConfinement(p.getFluid()->getVorticityEps(), effectLength);
		p.applyBuoyancy(gravity);
	}

	this->addBoundaryForce(particles, dt);

	// ---- Integrate, react, and decorate with curl noise ----------------------
	for (auto& p : particles) {
		const auto* fluid = p.getFluid();
		p.forwardTime(dt);
		p.react(dt);
		p.addVelocity(curlNoise(p.getPosition(), fluid->getCurlNoiseFrequency()) * fluid->getCurlNoiseStrength());
	}

	// ---- Emitters and lifetime ------------------------------------------------
	for (auto fluid : fluids) {
		fluid->updateEmitters(dt);
		fluid->removeDead();
	}

	// ---- Secondary particles: sparks/smoke (cosmetic, non-SPH) --------------
	for (auto fluid : fluids) {
		fluid->updateSecondaryParticles(dt, gravity);
	}
}

void FlameSolver::addBoundaryForce(std::vector<FlameParticle>& particles, const float dt)
{
	if (boundaryPlanes_.empty()) {
		return;
	}
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		Vector3df force(0.0f, 0.0f, 0.0f);
		for (const auto& plane : boundaryPlanes_) {
			force += plane.getBoundaryForce(particles[i].getPosition(), dt);
		}
		particles[i].addForce(force * particles[i].getDensity());
	}
}
