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
	emitFromEmitters(emitters_, dt, maxParticles_, rng_,
		[this] { return particles.size(); },
		[this](const Vector3df& pos, const Emitter& e) { createParticle(pos, e.particleRadius); },
		[this](const Vector3df& velocity) { particles.velocities.back() = velocity; });
}

void PBSPHFluid::removeOutflowParticles()
{
	removeParticlesInRegions(outflowRegions_, particles);
}