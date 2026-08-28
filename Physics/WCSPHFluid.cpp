#include "pch.h"

#include "WCSPHFluid.h"

#include "WCSPHParticle.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

WCSPHFluid::WCSPHFluid() :
	tensionCoe(0.0f),
	viscosityCoe(0.0f),
	pressureCoe(0.0f),
	density(0.0f),
	rng_(kDefaultRandomSeed)
{

}

Box3df WCSPHFluid::getBoundingBox() const
{
	auto bb = Box3df::createDegeneratedBox();
	for (const auto& pos : particles.positions) {
		bb.add(pos);
	}
	return bb;
}

float WCSPHFluid::estimatePressureCoe(const float effectLength, const float pressureCoeScale)
{
	return pressureCoeScale * effectLength;
}

void WCSPHFluid::updateEmitters(const float dt)
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

void WCSPHFluid::removeOutflowParticles()
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
