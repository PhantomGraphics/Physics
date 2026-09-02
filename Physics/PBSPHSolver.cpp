#include "pch.h"

#include "PBSPHSolver.h"
#include "PBSPHFluid.h"
#include "PBSPHParticle.h"
#include "PlaneBoundary.h"
#include "OpenMPRuntimeTuning.h"

#include "CGLib/Space/Space/NeighborList.h"


#include <algorithm>
#include <cmath>
#include <iostream>

using namespace Phantom::Math;
using namespace Phantom::Space;
using namespace Phantom::Physics;

PBSPHSolver::PBSPHSolver() :
	maxTimeStep(0.01f)
{}

void PBSPHSolver::setEffectLength(const float length)
{
	for (auto* fluid : fluids) fluid->setEffectLength(length);
}

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

std::vector<Vector3df> PBSPHSolver::getParticleVelocities() const
{
	std::vector<Vector3df> velocities;
	for (const auto* fluid : fluids) {
		const auto& soa = fluid->getParticles();
		velocities.insert(velocities.end(), soa.velocities.begin(), soa.velocities.end());
	}
	return velocities;
}

std::vector<float> PBSPHSolver::getParticleDensities() const
{
	std::vector<float> densities;
	for (const auto* fluid : fluids) {
		const auto& soa = fluid->getParticles();
		densities.insert(densities.end(), soa.densities.begin(), soa.densities.end());
	}
	return densities;
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
	lastSolveStats_ = {};

	if (fluids.empty()) return;
	if (!std::isfinite(dt) || dt <= 0.0f || maxIter <= 0) {
		lastSolveStats_.validConfiguration = false;
		return;
	}

	// Reject an unset/garbage kernel instead of feeding it to the neighbor
	// search: effectLength stays 0.f until setEffectLength() is called, so a
	// caller that forgets it gets an inert (no-op) solver rather than UB
	// (mirrors WCSPHSolver/DFSPHSolver; internal design notes Phase 5).
	SPHKernel* frontKernel = fluids.front()->getKernel();
	const float commonEffectLength = (frontKernel != nullptr) ? frontKernel->getEffectLength() : 0.0f;
	const float commonRestDensity  = fluids.front()->getRestDensity();
	if (frontKernel == nullptr || !std::isfinite(commonEffectLength) || commonEffectLength <= 0.0f) {
		lastSolveStats_.validConfiguration = false;
		return;
	}
	// Every registered fluid must share the same kernel support radius and rest
	// density (mirrors DFSPHSolver): the single neighbor search and the shared
	// lambda calibration below assume one consistent SPH scale.
	for (auto* fluid : fluids) {
		if (!fluid || !fluid->getKernel() ||
		    std::abs(fluid->getKernel()->getEffectLength() - commonEffectLength) >
		        std::max(1.0e-6f, std::abs(commonEffectLength) * 1.0e-5f) ||
		    std::abs(fluid->getRestDensity() - commonRestDensity) >
		        std::max(1.0e-6f, std::abs(commonRestDensity) * 1.0e-5f)) {
			lastSolveStats_.validConfiguration = false;
			return;
		}
	}

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

	if (particles.empty()) return;

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

		// Analytic non-plane shapes (spheres): adds the paired density +
		// constraint-gradient contribution in one pass, before calculateLambda().
		this->addShapeBoundaryConstraint(particles);

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

	// Single-step position-based solve: exactly one "substep" advancing the
	// full requested dt. The maxIter incompressibility-constraint iterations
	// are the PBSPH counterpart of DFSPH's density-projection sweeps, so they
	// go in densityIterations; divergenceIterations stays 0 (no separate
	// divergence solve) and converged stays true (no residual tolerance check).
	lastSolveStats_.substeps = 1;
	lastSolveStats_.advancedTime = dt;
	lastSolveStats_.densityIterations = maxIter;

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
	if (!hasShapeBoundaries()) {
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
	// was stable -- see internal design notes,
	// "problem B").
	// Uses the predicted (in-flight) position, not getPosition() (the
	// position committed at the *previous* simulate() call and left
	// unchanged for the whole maxIter loop) -- otherwise "over" never
	// shrinks between iterations and the same full correction gets re-added
	// every iteration, amplifying the correction by ~maxIter.
	const float dt2 = maxTimeStep * maxTimeStep;
	const std::vector<std::shared_ptr<IShapeBoundary>>* lists[] = {
		&boundaryPlanes_, &boundarySpheres_, &boundaryPlates_, &boundaryShapes_
	};
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		const auto pos = particles[i].getPredictPosition();
		Vector3df force(0.0f, 0.0f, 0.0f);
		for (const auto* list : lists) {
			for (const auto& shape : *list) {
				if (!shape) continue;
				force += shape->getBoundaryForce(pos, maxTimeStep);
			}
		}
		particles[i].addPositionCorrection(force * dt2);
	}
}

void PBSPHSolver::clampToBoundary(std::vector<PBSPHParticle>& particles)
{
	if (!hasShapeBoundaries()) {
		return;
	}
	const std::vector<std::shared_ptr<IShapeBoundary>>* lists[] = {
		&boundaryPlanes_, &boundarySpheres_, &boundaryPlates_, &boundaryShapes_
	};
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto pos = particles[i].getPredictPosition();
		for (const auto* list : lists) {
			for (const auto& shape : *list) {
				if (!shape) continue;
				pos = shape->clampPosition(pos);
			}
		}
		particles[i].setPredictPosition(pos);
	}
}

// See the header doc comment. Mirrors DFSPHSolver::addBoundaryDensity()'s
// analytic-shape branch (numerator + denominator from one sample()), translated
// to PBSPH's Poly6 kernel and constraint-gradient accumulators. The pseudo
// boundary "particle" is weighted by the fluid particle's own mass -- the same
// convention the discrete boundary-particle path uses via bp.psi -- so the
// contribution is on the scale of one fluid neighbour rather than a raw
// restDensity spike.
void PBSPHSolver::addShapeBoundaryConstraint(std::vector<PBSPHParticle>& particles)
{
	if (!hasShapeBoundaries() || particles.empty()) return;

	const std::vector<std::shared_ptr<IShapeBoundary>>* lists[] = {
		&boundaryPlanes_, &boundarySpheres_, &boundaryPlates_, &boundaryShapes_
	};
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto& p = particles[i];
		PBSPHFluid* fluid = p.getFluid();
		if (fluid == nullptr) continue;
		SPHKernel* kernel = fluid->getKernel();
		if (kernel == nullptr) continue;

		const float r = kernel->getEffectLength();
		if (r <= 0.0f) continue;
		const float pseudoMass = p.getMass();
		const auto pos = p.getPredictPosition();

		auto addShape = [&](const IShapeBoundary& boundary) {
			const auto sample = boundary.sample(pos, r);
			if (!sample.active) return;
			const float dist = -sample.signedDistance;   // penetration depth past the wall
			if (dist <= 0.0f || dist >= r) return;
			const float weight = sample.densityWeight;
			if (weight <= 0.0f) return;

			// numerator: the wall stands in for missing fluid on its far side.
			p.addDensity(pseudoMass * weight * (1.0f - dist / r) * kernel->getPoly6Kernel(dist));

			// denominator: the matching constraint-gradient term, along the
			// shortest correction back onto the valid side of the shape.
			const auto correction = boundary.clampPosition(pos) - pos;
			const float correctionLength = glm::length(correction);
			if (correctionLength > 1.0e-8f) {
				const auto direction = correction / correctionLength;
				p.addConstraintGradient(
					kernel->getPoly6KernelGradient(direction * dist) * pseudoMass * weight);
			}
		};
		for (const auto* list : lists) {
			for (const auto& shape : *list) {
				if (shape) addShape(*shape);
			}
		}
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
