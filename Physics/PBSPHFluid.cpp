#include "pch.h"

#include "PBSPHFluid.h"

#include "PBSPHParticle.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

PBSPHFluid::PBSPHFluid() :
	// Default-constructed kernel: effectLength stays 0.f until setEffectLength()
	// is called, matching WCSPHFluid/DFSPHFluid so that PBSPHSolver::simulate()
	// treats a fluid whose support radius was never set as an inert no-op rather
	// than silently running with h == 1 (see docs/guide/conventions.md and
	// PBSPHSolver::simulate()'s configuration check).
	restDensity(1.0f),
	stiffness(0.05f),
	viscosity(0.1f),
	_isBoundary(false)
{
}

PBSPHFluid::~PBSPHFluid()
{
}

Box3df PBSPHFluid::getBoundingBox() const
{
	Box3df bb = Box3df::createDegeneratedBox();
	for (const auto& pos : particles.positions) {
		bb.add(pos);
	}
	return bb;
}

void PBSPHFluid::setEffectLength(const float effectLength)
{
	this->kernel = SPHKernel(effectLength);
}

void PBSPHFluid::setRestDensity(const float restDensity)
{
	this->restDensity = restDensity;
}

void PBSPHFluid::updateEmitters(const float dt)
{
	std::uniform_real_distribution<float> jitterDist(-1.0f, 1.0f);

	for (auto& e : emitters_) {
		const int remaining = maxParticles_ - static_cast<int>(particles.size());
		if (remaining <= 0) {
			continue;
		}

		const int n = accumulateEmission(e, dt, remaining);
		const float dirLenSq = Math::getLengthSquared(e.direction);
		const Vector3df dir = dirLenSq > 1.0e-12f
			? e.direction / std::sqrt(dirLenSq)
			: Vector3df(0.0f, 1.0f, 0.0f);

		for (int i = 0; i < n; ++i) {
			const Vector3df pos = e.center + nextDiskLatticeOffset(e, dir);
			createParticle(pos, e.particleRadius);
			particles.velocities.back() = dir * (e.speed + e.speedJitter * jitterDist(rng_));
		}
	}
}

void PBSPHFluid::removeOutflowParticles()
{
	if (outflowRegions_.empty()) {
		return;
	}

	for (size_t i = 0; i < particles.size();) {
		const Vector3df& pos = particles.positions[i];
		bool remove = false;
		for (const auto& r : outflowRegions_) {
			if (r.bounds.contains(pos, 0.0f)) {
				remove = true;
				break;
			}
		}

		if (remove) {
			particles.swapAndPop(i);
			continue;
		}
		++i;
	}
}