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
	emitFromEmitters(emitters_, dt, maxParticles_, rng_,
		[this] { return particles.size(); },
		[this](const Vector3df& pos, const Emitter& e) {
			const float diameter = e.particleRadius * 2.0f;
			createParticle(pos, e.particleRadius, diameter * diameter * diameter);
		},
		[this](const Vector3df& velocity) { particles.velocities.back() = velocity; });
}

void DFSPHFluid::removeOutflowParticles()
{
	removeParticlesInRegions(outflowRegions_, particles);
}

