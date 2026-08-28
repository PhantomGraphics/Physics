#include "pch.h"

#include "MVCFluid.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

Box3df MVCFluid::getBoundingBox() const
{
	Box3df bb = Box3df::createDegeneratedBox();
	for (size_t i = 0; i < particles.size(); ++i) {
		bb.add(particles.positions[i]);
	}
	return bb;
}

void MVCFluid::updateEmitters(const float dt)
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
			MVCParticle p;
			p.position = e.center + sampleDiskOffset(e.radius, rng_);
			p.velocity = dir * (e.speed + e.speedJitter * jitterDist(rng_));
			p.mass = 1.0f;
			p.volume = 1.0f;
			p.density = density_;
			p.delta = Vector3df(0.0f, 0.0f, 0.0f);
			addParticle(p);
		}
	}
}
