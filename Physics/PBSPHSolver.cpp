#include "pch.h"

#include "PBSPHSolver.h"
#include "PBSPHFluid.h"
#include "PBSPHParticle.h"
#include "PlaneBoundary.h"
#include "OpenMPRuntimeTuning.h"

#include "CGLib/Space/Space/NeighborList.h"


#include <iostream>

using namespace Phantom::Math;
using namespace Phantom::Space;
using namespace Phantom::Physics;

PBSPHSolver::PBSPHSolver() :
	maxTimeStep(0.01f)
{}

SPHKernel* PBSPHSolver::getKernel()
{
	return fluids.empty() ? nullptr : fluids.front()->getKernel();
}

float PBSPHSolver::getRestDensity() const
{
	return fluids.empty() ? 0.f : fluids.front()->getRestDensity();
}

int PBSPHSolver::getParticleCount() const
{
	int count = 0;
	for (const auto* fluid : fluids) {
		count += static_cast<int>(fluid->getParticles().size());
	}
	return count;
}

std::vector<Vector3df> PBSPHSolver::getParticlePositions() const
{
	std::vector<Vector3df> positions;
	for (const auto* fluid : fluids) {
		const auto& soa = fluid->getParticles();
		positions.insert(positions.end(), soa.positions.begin(), soa.positions.end());
	}
	return positions;
}

/*
void PBSPHSolver::step()
{
	simulate(maxTimeStep, 3);
}
*/

void PBSPHSolver::simulate(const float dt, const int maxIter)
{
	ensurePassiveOpenMPWaitPolicy();

	std::vector<PBSPHParticle> particles;
	std::vector<bool> isStaticMask;
	for (auto fluid : fluids) {
		const bool staticFluid = fluid->isStatic();
		auto& soa = fluid->getParticles();
		for (size_t i = 0; i < soa.size(); ++i) {
			particles.emplace_back(soa, i, fluid);
			isStaticMask.push_back(staticFluid);
		}
	}

	for (auto& p : particles) {
		p.init();
	}

	//PBSPHBoundarySolver boundarySolver(boundary);
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		if (isStaticMask[i]) continue;
		particles[i].addExternalForce(externalForce);
		particles[i].predictPosition_(dt);
	}

	std::vector<Vector3df> positions;
	positions.reserve(particles.size());
	for (const auto& p : particles) {
		positions.push_back(p.getPosition());
	}

	// Per-particle neighbor list, built once -- the search runs only before
	// the maxIter loop, so this list is reused unchanged across iterations.
	// This lets the three density/constraint-gradient/pressure passes below be
	// parallelized over particles instead of over pairs. Parallelizing over
	// pairs (the previous form) let two pairs that share a particle land on
	// different threads, racing on that particle's
	// density/constraint-gradient/pressure accumulators (non-atomic +=) and
	// making results non-deterministic across runs. Gathering per-particle
	// means each thread only ever writes to its own particle (same as
	// WCSPHSolver.cpp).
	const auto searchLength = fluids.front()->getKernel()->getEffectLength() * 1.1f;
	CSRNeighborList neighbors;
	neighbors.build(positions, searchLength);

	for (int iter = 0; iter < maxIter; ++iter) {
		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			particles[i].setDensity(0.0f);
			particles[i].setDx(Math::Vector3df(0, 0, 0));
		}

		//boundarySolver.addDX(particles, dt);

		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			particles[i].addSelfDensity();
		}

#pragma omp parallel for
		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			for (const int neighbor : neighbors[i]) {
				particles[i].addDensity(particles[neighbor]);
			}
		}

		this->addBoundaryParticleDensity(particles, rigidBoundaryParticles_);
		this->addBoundaryParticleDensity(particles, softBoundaryParticles_);

		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			particles[i].resetConstraintGradient();
		}

#pragma omp parallel for
		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			for (const int neighbor : neighbors[i]) {
				particles[i].accumulateConstraintGradient(particles[neighbor]);
			}
		}

		this->addBoundaryParticleConstraintGradient(particles, rigidBoundaryParticles_);
		this->addBoundaryParticleConstraintGradient(particles, softBoundaryParticles_);

		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			particles[i].calculateLambda();
		}

#pragma omp parallel for
		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			for (const int neighbor : neighbors[i]) {
				particles[i].calculatePressure(particles[neighbor]);
			}
		}

		this->addBoundadryPressure(particles);
		this->addRigidBoundaryPressure(particles);
		this->addBoundaryParticlePressure(particles, rigidBoundaryParticles_, dt);
		this->addBoundaryParticlePressure(particles, softBoundaryParticles_, dt);

		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			if (isStaticMask[i]) continue;
			particles[i].updatePredictPosition();
		}

		// Safety-net clamp, applied *after* the density-driven predictPosition
		// update above -- not a competing force, just a floor/ceiling on how
		// far a single iteration's motion can go, so it doesn't override the
		// soft correction the way the old full-snap addBoundadryPressure() did.
		this->clampToBoundary(particles);
	}

	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		particles[i].setXvisc(Vector3df(0, 0, 0));
	}
//	boundarySolver->calculateViscosity(particles);
	// Gathered per-particle like the passes above (this one was still walking
	// the raw pair list, writing to both endpoints -- which is why it was the
	// one pass here that could not be parallelized).
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		for (const int neighbor : neighbors[i]) {
			particles[i].calculateViscosity(particles[neighbor]);
		}
	}

	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		if (isStaticMask[i]) continue;
		particles[i].updateVelocity(dt);
		particles[i].addVelocity(particles[i].getXvisc());
		particles[i].updatePosition();
	}

	auto densityError = 0.0;
	for (auto& particle : particles) {
		densityError += particle.getDensity() / (double)particles.size();
	}
	//std::cout << densityError << std::endl;

	/*
	for (auto fluid : fluids) {
		fluid->getPresenter()->updateView();
	}
	*/
}

float PBSPHSolver::calculateTimeStep(const std::vector<PBSPHParticle>& particles)
{
	float maxVelocity = 0.0f;
	for (auto& p : particles) {
		maxVelocity = std::max<float>(maxVelocity, Math::getLengthSquared(p.getVelocity()));
	}
	if (maxVelocity < 1.0e-3) {
		return maxTimeStep;
	}
	maxVelocity = std::sqrt(maxVelocity);
	const auto dt = 0.4f * particles.front().getRadius() * 2.0f / maxVelocity;
	return maxTimeStep;
	//return std::min(dt, maxTimeStep);
}


void PBSPHSolver::addBoundadryPressure(std::vector<PBSPHParticle>& particles)
{
	if (boundaryPlanes_.empty()) {
		return;
	}
	// getBoundaryForce returns -over / dt^2 per axis. Multiplying by dt^2
	// recovers -over, a full, unconditional position correction each
	// iteration (not the softened/relaxed version tried earlier during this
	// investigation -- see setBoundary()'s doc comment). This full
	// correction is fine as long as calculateLambda()'s lambda is clamped:
	// an UNCLAMPED lambda blowing up near a sparse/boundary neighborhood
	// (few or no neighbors past the wall -> tiny gradient sums -> huge
	// lambda) is what actually caused runaway divergence when combined with
	// this correction, not the correction's strength by itself (confirmed
	// empirically by reproducing the pre-Two-Way-coupling algorithm, which
	// used this same full correction with a raw, non-lambda constraint and
	// was stable -- see docs/todo/PLAN_rigid_fluid_coupling_phase8_status.md,
	// "problem B").
	// Uses the predicted (in-flight) position, not getPosition() (the
	// position committed at the *previous* simulate() call and left
	// unchanged for the whole maxIter loop) -- otherwise "over" never
	// shrinks between iterations and the same full correction gets re-added
	// every iteration, amplifying the correction by ~maxIter.
	const float dt2 = maxTimeStep * maxTimeStep;
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		Vector3df force(0.0f, 0.0f, 0.0f);
		for (const auto& plane : boundaryPlanes_) {
			force += plane.getBoundaryForce(particles[i].getPredictPosition(), maxTimeStep);
		}
		particles[i].addPositionCorrection(force * dt2);
	}
}

void PBSPHSolver::clampToBoundary(std::vector<PBSPHParticle>& particles)
{
	if (boundaryPlanes_.empty()) {
		return;
	}
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto pos = particles[i].getPredictPosition();
		for (const auto& plane : boundaryPlanes_) {
			pos = plane.clampPosition(pos);
		}
		particles[i].setPredictPosition(pos);
	}
}

void PBSPHSolver::addRigidBoundaryPressure(std::vector<PBSPHParticle>& particles)
{
	if (rigidBoundaries_.empty()) return;
	// Same acceleration -> position-correction conversion and same
	// predicted-position requirement as addBoundadryPressure().
	const float dt2 = maxTimeStep * maxTimeStep;
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto& p = particles[i];
		for (auto* rb : rigidBoundaries_) {
			const auto force = rb->getBoundaryForce(p.getPredictPosition());
			p.addPositionCorrection(force * dt2);
		}
	}
}

void PBSPHSolver::addBoundaryParticleDensity(std::vector<PBSPHParticle>& particles,
                                              const std::vector<IBoundaryParticles*>& boundaries)
{
	if (boundaries.empty() || particles.empty()) return;

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto& p = particles[i];
		PBSPHFluid* fluid = p.getFluid();
		if (fluid == nullptr) continue;

		SPHKernel* kernel = fluid->getKernel();
		if (kernel == nullptr) continue;
		const float effectLength = kernel->getEffectLength();

		for (auto* boundary : boundaries) {
			for (auto& bp : boundary->particles()) {
				const float dist = Math::getDistance(p.getPredictPosition(), bp.worldPos);
				if (dist >= effectLength) continue;
				p.addDensity(kernel->getPoly6Kernel(dist) * bp.psi);
			}
		}
	}
}

void PBSPHSolver::addBoundaryParticleConstraintGradient(std::vector<PBSPHParticle>& particles,
                                                         const std::vector<IBoundaryParticles*>& boundaries)
{
	if (boundaries.empty() || particles.empty()) return;

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto& p = particles[i];
		PBSPHFluid* fluid = p.getFluid();
		if (fluid == nullptr) continue;

		SPHKernel* kernel = fluid->getKernel();
		if (kernel == nullptr) continue;
		const float effectLength = kernel->getEffectLength();

		for (auto* boundary : boundaries) {
			for (auto& bp : boundary->particles()) {
				const float dist = Math::getDistance(p.getPredictPosition(), bp.worldPos);
				if (dist >= effectLength) continue;
				const auto v = p.getPredictPosition() - bp.worldPos;
				const auto grad = kernel->getPoly6KernelGradient(v) * bp.psi;
				p.addConstraintGradient(grad);
			}
		}
	}
}

// Mirrors PBSPHParticle::calculatePressure(): reuses the same
// lambda/weight formula, dropping the neighbor's own lambda term (boundary
// particles have no constraint/lambda of their own) and substituting the
// neighbor mass with psi. Uses the normalized PBF Lagrange multiplier
// (getLambda(), populated by calculateLambda() earlier this iteration), not
// the raw density constraint -- the raw constraint multiplied directly by
// the Poly6 kernel gradient would reintroduce the same small-effectLength
// blowup fixed in calculatePressure() (the gradient constant scales as
// 1/effectLength^9). Shared by rigid and SoftBody boundaries alike -- see
// addBoundaryParticleDensity().
void PBSPHSolver::addBoundaryParticlePressure(std::vector<PBSPHParticle>& particles,
                                               const std::vector<IBoundaryParticles*>& boundaries,
                                               const float dt)
{
	if (boundaries.empty() || particles.empty() || dt <= 0.0f) return;
	const float dt2 = dt * dt;

	// Sequential: bp.accumForce is shared across fluid particles, so this loop
	// cannot be parallelized without a data race on the boundary particles.
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto& p = particles[i];
		PBSPHFluid* fluid = p.getFluid();
		if (fluid == nullptr) continue;

		SPHKernel* kernel = fluid->getKernel();
		if (kernel == nullptr) continue;

		const float restDensity  = fluid->getRestDensity();
		const float effectLength = kernel->getEffectLength();
		const float lambda_i     = p.getLambda();

		for (auto* boundary : boundaries) {
			for (auto& bp : boundary->particles()) {
				const float dist = Math::getDistance(p.getPredictPosition(), bp.worldPos);
				if (dist >= effectLength) continue;

				const auto v      = p.getPredictPosition() - bp.worldPos;
				const auto weight = kernel->getPoly6KernelGradient(v);
				const auto dx     = lambda_i * bp.psi * weight / restDensity * fluid->getStiffness();
				p.addPositionCorrection(dx);

				// Convert the PBD position correction back into an equivalent
				// force for accumForce bookkeeping (same dt^2 conversion used
				// for domain-boundary/RigidBoundary above).
				const auto f = p.getMass() * dx / dt2;
				bp.accumForce -= f;
			}
		}
	}
}