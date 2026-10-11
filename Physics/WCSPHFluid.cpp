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
	emitFromEmitters(emitters_, dt, maxParticles_, rng_,
		[this] { return particles.size(); },
		[this](const Vector3df& pos, const Emitter& e) { createParticle(pos, e.particleRadius); },
		[this](const Vector3df& velocity) { particles.velocities.back() = velocity; });
}

void WCSPHFluid::removeOutflowParticles()
{
	removeParticlesInRegions(outflowRegions_, particles);
}
