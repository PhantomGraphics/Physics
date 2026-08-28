#include "pch.h"

#include "DFSPHFluid.h"

#include "DFSPHParticle.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

DFSPHFluid::DFSPHFluid()
{

}

Box3df DFSPHFluid::getBoundingBox() const
{
	auto bb = Box3df::createDegeneratedBox();
	for (const auto& pos : particles.positions) {
		bb.add(pos);
	}
	return bb;
}

void DFSPHFluid::setEffectLength(const float effectLength)
{
	this->kernel = SPHKernel(effectLength);
}

void DFSPHFluid::updateEmitters(const float dt)
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

		const float diameter = e.particleRadius * 2.0f;
		const float mass = diameter * diameter * diameter;

		for (int i = 0; i < n; ++i) {
			const Vector3df pos = e.center + nextDiskLatticeOffset(e, dir);
			createParticle(pos, e.particleRadius, mass);
			particles.velocities.back() = dir * (e.speed + e.speedJitter * jitterDist(rng_));
		}
	}
}

void DFSPHFluid::removeOutflowParticles()
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

