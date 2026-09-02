#include "pch.h"

#include "DFSPHSolver.h"
#include "DFSPHParticle.h"
#include "DFSPHFluid.h"
#include "OpenMPRuntimeTuning.h"

#include "CGLib/Space/Space/NeighborList.h"

#include <iostream>

using namespace Phantom::Math;
using namespace Phantom::Space;
using namespace Phantom::Physics;

DFSPHSolver::DFSPHSolver() :
	maxTimeStep(0.01f)
{}

void DFSPHSolver::setEffectLength(const float length)
{
	for (auto* fluid : fluids) fluid->setEffectLength(length);
}

SPHKernel* DFSPHSolver::getKernel()
{
	return fluids.empty() ? nullptr : fluids.front()->getKernel();
}

float DFSPHSolver::getRestDensity() const
{
	return fluids.empty() ? 0.f : fluids.front()->getDensity();
}

int DFSPHSolver::getParticleCount() const
{
	int count = 0;
	for (const auto* fluid : fluids) {
		count += static_cast<int>(fluid->getParticles().size());
	}
	return count;
}

std::vector<Vector3df> DFSPHSolver::getParticlePositions() const
{
	std::vector<Vector3df> positions;
	for (const auto* fluid : fluids) {
		const auto& soa = fluid->getParticles();
		positions.insert(positions.end(), soa.positions.begin(), soa.positions.end());
	}
	return positions;
}

std::vector<Vector3df> DFSPHSolver::getParticleVelocities() const
{
	std::vector<Vector3df> velocities;
	for (const auto* fluid : fluids) {
		const auto& soa = fluid->getParticles();
		velocities.insert(velocities.end(), soa.velocities.begin(), soa.velocities.end());
	}
	return velocities;
}

std::vector<float> DFSPHSolver::getParticleDensities() const
{
	std::vector<float> densities;
	for (const auto* fluid : fluids) {
		const auto& soa = fluid->getParticles();
		densities.insert(densities.end(), soa.densities.begin(), soa.densities.end());
	}
	return densities;
}

/*
void DFFluidSolver::step()
{
	simulate(maxTimeStep, 2.25, 2.5, 3);
}
*/

void DFSPHSolver::simulate(const float dt, const int maxIter)
{
	ensurePassiveOpenMPWaitPolicy();
	lastSolveStats_ = {};
	if (fluids.empty()) return;
	if (!std::isfinite(dt) || dt <= 0.0f || maxIter <= 0) {
		lastSolveStats_.validConfiguration = false;
		return;
	}
	frameTimeStep_ = dt;
	if (fluids.front()->getKernel() == nullptr ||
	    fluids.front()->getKernel()->getEffectLength() <= 0.0f) {
		lastSolveStats_.validConfiguration = false;
		return;
	}
	const float commonEffectLength = fluids.front()->getKernel()->getEffectLength();
	const float commonDensity = fluids.front()->getDensity();
	for (auto* fluid : fluids) {
		if (!fluid || !fluid->getKernel() ||
		    std::abs(fluid->getKernel()->getEffectLength() - commonEffectLength) >
		        std::max(1.0e-6f, std::abs(commonEffectLength) * 1.0e-5f) ||
		    std::abs(fluid->getDensity() - commonDensity) >
		        std::max(1.0e-6f, std::abs(commonDensity) * 1.0e-5f)) {
			lastSolveStats_.validConfiguration = false;
			return;
		}
	}

	std::vector<DFSPHParticle> particles;
	for (auto fluid : fluids) {
		auto& soa = fluid->getParticles();
		for (size_t i = 0; i < soa.size(); ++i) {
			particles.emplace_back(soa, i, fluid);
		}
	}

	if (particles.empty()) return;

	// Must match the kernel actually used to weight density/alpha/viscosity
	// contributions below (DFSPHParticle::getKernel() -> fluid->getKernel()),
	// not an independently hardcoded ratio of particle radius -- otherwise a
	// fluid whose effectLength was set to something other than the default
	// 2.25x radius (via setEffectLength()) silently misses real neighbors
	// within the kernel's support (docs/todo/PLAN_sph_scale_invariance.md
	// Phase 4, item #8).
	const auto searchRadius = particles.front().getParent()->getKernel()->getEffectLength();

	// Neighbor indices into the combined `particles` working set above --
	// unlike the pre-SoA design, these are not stored on the particles
	// themselves (see DFSPHParticle's class doc). Flattened (CSR) rather than
	// one std::vector<int> per particle: this list is rebuilt on every
	// sub-step of the loop below, so the per-particle heap allocations the old
	// CompactSpaceHash::findNeighborIndices() form paid were the dominant cost
	// (docs/issue/wcsph_parallel_scaling_profile.md section 4). The neighbor
	// *set* is unchanged -- both forms exclude the particle itself and keep
	// only particles strictly closer than searchRadius; only the order within
	// a row (hence the floating-point summation order) differs.
	std::vector<Vector3df> positions(particles.size());
	CSRNeighborList neighborIndices;

	// Rebuild neighbor list using the current particle positions.
	auto buildNeighbors = [&]() {
		for (size_t i = 0; i < particles.size(); ++i) {
			positions[i] = particles[i].getPosition();
		}
		neighborIndices.build(positions, searchRadius);
	};

	// Initial neighbor list and density/alpha before the time loop.
	buildNeighbors();

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		particles[i].calculateDensity(particles, neighborIndices[i]);
		particles[i].calculateAlpha(particles, neighborIndices[i]);
	}
	this->addBoundaryDensity(particles);
	this->addBoundaryParticleConstraintTerms(particles, rigidBoundaryParticles_);
	this->addBoundaryParticleConstraintTerms(particles, softBoundaryParticles_);

	auto time = 0.0f;
	while (time < dt) {
		// Apply external force (e.g. gravity) to every particle.
		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			particles[i].setForce(externalForce * particles[i].getMass());
		}

		// Surface normals must be fully accumulated over all neighbors before
		// calculateSurfaceTension() (below) can use them, the same reason
		// density is accumulated in its own pass before pressure/viscosity
		// (mirrors WCSPHSolver::simulate()).
		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			particles[i].calculateNormal(particles, neighborIndices[i]);
		}

		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			particles[i].calculateViscosity(particles, neighborIndices[i]);
			particles[i].calculateSurfaceTension(particles, neighborIndices[i]);
		}

		auto dt = calculateTimeStep(particles);
        if (!std::isfinite(dt) || dt <= 0.0f) {
			lastSolveStats_.converged = false;
			break;
		}
		// Land exactly on the caller's frame dt instead of overshooting it. The CFL
		// substep does not divide the frame evenly in general, so without this
		// the loop kept taking whole substeps until `time` passed the frame end
		// and the frame advanced further than the caller asked for -- by up to
		// one substep, i.e. as much as 50% (calculateTimeStep() caps a substep
		// at maxTimeStep/2). Measured on a block in free fall with no wall in
		// reach: 0.6 s of frames moved it 2.02 m where gravity alone gives
		// 1.76 m, and peak speed came out 6.27 m/s against an analytic 5.88.
		// That is simulated time appearing out of nowhere, and it made every
		// DFSPH scene run fast by a data-dependent amount.
		//
		// The remainder is only ever *shortened*, never lengthened, so it stays
		// within the CFL bound the substep was chosen for. A remainder too
		// small to be worth a full solve (below calculateTimeStep()'s own
		// minTimeStep floor) ends the frame instead, so the density/divergence
		// solves are never handed a degenerate dt.
		const float remaining = frameTimeStep_ - time;
		if (remaining <= std::max(dt, maxTimeStep) * 1.0e-6f) {
			break;
		}
		if (dt > remaining) {
			dt = remaining;
		}
		//std::cout << "timestep " << dt << std::endl;
		this->addBoundaryPressure(particles, dt);
		this->addRigidBoundaryPressure(particles);
		this->addBoundaryParticlePressure(particles, rigidBoundaryParticles_, dt);
		this->addBoundaryParticlePressure(particles, softBoundaryParticles_, dt);

		// Integrate velocity with non-pressure forces (skip static fluids).
		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			if (particles[i].getParent()->isStatic()) continue;
			particles[i].addVelocity(dt * particles[i].getForce() / particles[i].getMass());
		}
		correctDensityError(particles, neighborIndices, dt, maxIter);

		// Integrate positions (skip static fluids).
		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			if (particles[i].getParent()->isStatic()) continue;
			particles[i].addPosition(dt * particles[i].getVelocity());
		}

		// Rebuild neighbors with updated positions before recomputing density/alpha.
		buildNeighbors();

#pragma omp parallel for
		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			particles[i].calculateDensity(particles, neighborIndices[i]);
			particles[i].calculateAlpha(particles, neighborIndices[i]);
		}
		this->addBoundaryDensity(particles);
		this->addBoundaryParticleConstraintTerms(particles, rigidBoundaryParticles_);
		this->addBoundaryParticleConstraintTerms(particles, softBoundaryParticles_);

		correctDivergenceError(particles, neighborIndices, dt, maxIter);
		time += dt;
		lastSolveStats_.substeps++;
	}
	lastSolveStats_.advancedTime = time;

	auto densityError = 0.0;
	for (auto& particle : particles) {
		densityError += particle.getDensity() / (double)particles.size();
	}
	//std::cout << densityError << std::endl;
}

void DFSPHSolver::correctDivergenceError(std::vector<DFSPHParticle>& particles, const CSRNeighborList& neighbors,
	                                     const float dt, const int maxIter)
{
	bool isErrorOk = false;
	int iter = 0;
	const auto restDensity = particles.front().getParent()->getDensity();
	while ((!isErrorOk || iter < 1) && iter < maxIter) {
		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			particles[i].calculateDpDt(particles, neighbors[i]);
		}
		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			particles[i].calculateVelocityInDivergenceError(dt, particles, neighbors[i]);
		}
		const auto averageDpDt = calculateAverageDpDt(particles);
		isErrorOk = isDivergenceErrorAcceptable(averageDpDt, restDensity, dt);
		iter++;
	}
	lastSolveStats_.divergenceIterations += iter;
	if (!isErrorOk) lastSolveStats_.converged = false;
}

bool DFSPHSolver::isDivergenceErrorAcceptable(float averageDpDt, float restDensity, float dt, float maxDivergenceErrorRatio)
{
	if (restDensity <= 0.0f) return true;
	return ::fabs(averageDpDt) * dt / restDensity < maxDivergenceErrorRatio;
}

void DFSPHSolver::correctDensityError(std::vector<DFSPHParticle>& particles, const CSRNeighborList& neighbors,
	                                  const float dt, const int maxIter)
{
	bool isErrorOk = false;
	auto iter = 0;
	// DFSPH requires two startup sweeps for a usable density projection. Keep
	// that algorithmic minimum even when the common API's advisory maxIter is
	// 1 (the value WCSPH-oriented callers traditionally pass).
	const int iterationLimit = std::max(2, maxIter);
	while ((!isErrorOk || iter < 2) && iter < iterationLimit) {
		const auto restDensity = particles.front().getParent()->getDensity();

		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			particles[i].predictDensity(dt, particles, neighbors[i]);
		}
		for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
			particles[i].calculateVelocityInDensityError(dt, particles, neighbors[i]);
		}

		const auto averageDensity = calculateAverageDensity(particles);
		const auto densityErrorRatio = ::fabs(averageDensity - restDensity) / restDensity;
		isErrorOk = densityErrorRatio < 0.05;

		iter++;
	}
	lastSolveStats_.densityIterations += iter;
	if (!isErrorOk) lastSolveStats_.converged = false;
}

void DFSPHSolver::addBoundaryParticleConstraintTerms(
	std::vector<DFSPHParticle>& particles,
	const std::vector<IBoundaryParticles*>& boundaries)
{
	// These are the numerator and denominator of one DFSPH constraint. Keep
	// them behind one internal call so simulate() cannot accidentally apply
	// boundary density without the matching alpha gradient.
	addBoundaryParticleDensity(particles, boundaries);
	addBoundaryParticleAlpha(particles, boundaries);
}

float DFSPHSolver::calculateTimeStep(const std::vector<DFSPHParticle>& particles)
{
	// Safety floor for the CFL-derived dt below, expressed relative to
	// maxTimeStep instead of an absolute magic number (docs/todo/PLAN_sph_scale_invariance.md
	// Phase 6) so it scales along with maxTimeStep instead of becoming
	// disproportionately coarse/fine if a caller's maxTimeStep departs from
	// the historical default of 0.01f. Reproduces the historical constant
	// (1.0e-6f) exactly at that default: 0.01f * 1.0e-4f == 1.0e-6f.
	const float minTimeStep = maxTimeStep * 1.0e-4f;

	float maxVelocity = 0.0;
	for (const auto& p : particles) {
		maxVelocity = std::max<float>(maxVelocity, Math::getLengthSquared(p.getVelocity()));
	}

	if (!std::isfinite(maxVelocity)) {
		return maxTimeStep / 2.0f;
	}

	if (maxVelocity < 1.0e-3) {
     return std::max(maxTimeStep / 2.0f, minTimeStep);
	}
	maxVelocity = std::sqrt(maxVelocity);
   if (!std::isfinite(maxVelocity) || maxVelocity <= 0.0f) {
		return maxTimeStep / 2.0f;
	}
	//std::cout << maxVelocity << std::endl;
	const auto dt = 0.4f * particles.front().getRadius() * 2.0f / maxVelocity;
 return std::max(std::min(dt, maxTimeStep / 2.0f), minTimeStep);
}

float DFSPHSolver::calculateAverageDensity(const std::vector<DFSPHParticle>& particles)
{
	float totalDensity = 0.0f;
	for (const auto& p : particles) {
		totalDensity += p.getPredictedDensity();
	}
	return totalDensity / (float)particles.size();
}

float DFSPHSolver::calculateAverageDpDt(const std::vector<DFSPHParticle>& particles)
{
	float totalDensity = 0.0f;
	for (const auto& p : particles) {
		totalDensity += p.getDpDt();
	}
	return totalDensity / (float)particles.size();
}

void DFSPHSolver::addBoundaryDensity(std::vector<DFSPHParticle>& particles)
{
	if (boundaryPlanes_.empty() && boundarySpheres_.empty() &&
	    boundaryPlates_.empty() && boundaryShapes_.empty()) {
		return;
	}

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto& p = particles[i];

		const auto pos = p.getPosition();

		SPHKernel* kernel = p.getParent()->getKernel();
		if (kernel == nullptr) continue;

		const float restDensity = p.getParent()->getDensity();

		// influence radius: the fluid's actual kernel support (see simulate()'s
		// searchRadius comment -- must track setEffectLength(), not an
		// independently hardcoded ratio of particle radius).
		const float r = kernel->getEffectLength();

		// Each shape contributes independently when the particle has already
		// penetrated past it (dist > 0, within influence radius r). Near a box
		// corner/edge this sums more than one plane's contribution, unlike the
		// former nearest-point-on-box distance -- an accepted approximation of
		// decomposing the box into 6 independent half-spaces.
		auto addShapeConstraint = [&](const IShapeBoundary& boundary) {
			const auto sample = boundary.sample(pos, r);
			if (!sample.active) return;
			const float dist = -sample.signedDistance;
			if (dist <= 0.0f || dist >= r) return;

			// boundary density contribution: rest * (1 - d/r) * kernel(d)
			const float weight = sample.densityWeight;
			if (weight <= 0.0f) return;
			const float contribution = restDensity * weight * (1.0f - dist / r) * kernel->getCubicSpline(dist);
			p.addDensity(contribution);

			// DFSPH's constraint numerator (density) and denominator (alpha)
			// must be extended together. The projection direction is the shortest
			// correction back to the valid side of the analytic shape.
			const auto correction = boundary.clampPosition(pos) - pos;
			const float correctionLength = glm::length(correction);
			if (correctionLength > 1.0e-8f) {
				const auto direction = correction / correctionLength;
				p.addBoundaryAlphaGradient(
					kernel->getCubicSplineGradient(direction * dist) * restDensity * weight);
			}
		};
		for (const auto& plane : boundaryPlanes_) if (plane) addShapeConstraint(*plane);
		for (const auto& sphere : boundarySpheres_) if (sphere) addShapeConstraint(*sphere);
		for (const auto& plate : boundaryPlates_) if (plate) addShapeConstraint(*plate);
		for (const auto& shape : boundaryShapes_) if (shape) addShapeConstraint(*shape);
	}
}

namespace {

// Caps a wall's penalty acceleration at what a *passive* wall can deliver in
// one substep. See addBoundaryPressure() below for why DFSPH specifically
// needs this.
//
// The spring PlaneBoundary applies is -d/dt^2, i.e. it returns the whole
// penetration as velocity in a single step (dv = |d|/dt). That is the right
// amount while d is the penetration this step just accrued -- d ~ |v_n|*dt
// makes dv ~ |v_n| -- but it grows without bound as dt shrinks for a d that
// fails to clear. Bound it by the two things a wall may physically do: stop
// the particle's inward motion (|v_n|), and push a residual penetration back
// out at the frame-level recovery rate (|d|/frameStep) rather than the
// substep one. Neither term depends on how far the CFL substep has collapsed,
// and neither binds during an ordinary single-step impact.
Vector3df clampToPassiveWall(const Vector3df& penaltyAcceleration, const IShapeBoundary& boundary,
                             const Vector3df& position, const Vector3df& velocity,
	                         const float signedDistance, const float dt, const float frameStep)
{
	if (signedDistance >= 0.0f) return penaltyAcceleration;

	const auto correction = boundary.clampPosition(position) - position;
	const float correctionLength = glm::length(correction);
	if (correctionLength <= 1.0e-8f) return penaltyAcceleration;
	const auto normal = correction / correctionLength;
	const float normalVelocity = glm::dot(normal, velocity);
	const float stopInward = (normalVelocity < 0.0f) ? -normalVelocity : 0.0f;
	const float recovery   = (frameStep > 0.0f) ? (-signedDistance / frameStep) : 0.0f;
	const float maxAcceleration = (stopInward + recovery) / dt;

	const float magnitude = glm::length(penaltyAcceleration);
	if (!(magnitude > maxAcceleration)) return penaltyAcceleration;
	return penaltyAcceleration * (maxAcceleration / magnitude);
}

}

// The dt here must be the *substep* actually about to be integrated, not the
// timeStep that used to be stashed by setBoundary()/setBoundaryPlanes(). The
// penalty spring is -d/dt^2 (PlaneBoundary::getBoundaryForce()), calibrated to
// undo the penetration d in exactly one step: acceleration -d/dt^2 gives
// dv = -d/dt and dx = -d. Feed it a T larger than the step being taken and the
// correction collapses to -d*(dt/T)^2.
//
// This solver substeps adaptively (calculateTimeStep(), 0.4*2r/v_max clamped
// to [maxTimeStep*1e-4, maxTimeStep/2]) while the old boundaryTimeStep held a
// separately registered frame-level value. That made T >= 2*dt always,
// so the walls removed at most a quarter of each penetration -- and for fast
// particles, where the CFL substep runs down toward maxTimeStep*1e-4, the
// walls effectively stopped existing exactly when they were needed most.
// Measured before the fix on a single particle fired at a floor: it sank
// 0.037 / 0.089 / 0.171 at 5 / 10 / 20 m/s, i.e. |v|*frameStep, instead of the
// |v|*substep the spring is calibrated for. WCSPH never hit this because it
// takes the caller's dt verbatim and its callers pass the same value to
// setBoundary(); DFSPH's substepping is what pulled the two apart.
//
// Handing the spring the substep is necessary but not sufficient: a particle
// held against a wall by the ones stacked on top of it keeps a residual d that
// the position update cannot clear, so the spring re-fires every substep with
// a larger |d|/dt, each kick raises v_max, that shrinks the next substep, and
// the next kick is larger still. clampToPassiveWall() above breaks that loop.
// Measured A/B on a 4x12x4 column dropped 8 particle spacings onto a floor:
// peak speed reached 109x the column's own landing speed with the raw substep
// and 11.5x with the cap, and only the capped run settled on the floor instead
// of being launched back up.
void DFSPHSolver::addBoundaryPressure(std::vector<DFSPHParticle>& particles, const float dt)
{
	if (boundaryPlanes_.empty() && boundarySpheres_.empty() &&
	    boundaryPlates_.empty() && boundaryShapes_.empty()) return;
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto& p = particles[i];
		Vector3df force(0.0f, 0.0f, 0.0f);
		auto addShapeForce = [&](const IShapeBoundary& boundary) {
			const auto sample = boundary.sample(p.getPosition(), 0.0f);
			if (!sample.active) return;
			const auto penalty = boundary.getBoundaryForce(p.getPosition(), p.getVelocity(),
			                                                   dt, boundaryDampingRatio_);
			force += clampToPassiveWall(penalty, boundary, p.getPosition(), p.getVelocity(),
			                            sample.signedDistance, dt, frameTimeStep_);
		};
		for (const auto& plane : boundaryPlanes_) if (plane) addShapeForce(*plane);
		for (const auto& sphere : boundarySpheres_) if (sphere) addShapeForce(*sphere);
		for (const auto& plate : boundaryPlates_) if (plate) addShapeForce(*plate);
		for (const auto& shape : boundaryShapes_) if (shape) addShapeForce(*shape);
		p.addForce(force * p.getMass());
	}
}

void DFSPHSolver::addRigidBoundaryPressure(std::vector<DFSPHParticle>& particles)
{
	if (this->rigidBoundaries_.empty()) return;
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto& p = particles[i];
		for (auto* rb : this->rigidBoundaries_) {
			const auto force = rb->getBoundaryForce(p.getPosition());
			p.addForce(force * p.getMass());
		}
	}
}

void DFSPHSolver::addBoundaryParticleDensity(std::vector<DFSPHParticle>& particles,
                                              const std::vector<IBoundaryParticles*>& boundaries)
{
	if (boundaries.empty() || particles.empty()) return;

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto& p = particles[i];
		SPHKernel* kernel = p.getParent()->getKernel();
		if (kernel == nullptr) continue;
		const float effectLength = kernel->getEffectLength();

		for (auto* boundary : boundaries) {
			for (auto& bp : boundary->particles()) {
				const float dist = Math::getDistance(p.getPosition(), bp.worldPos);
				if (dist >= effectLength) continue;
				p.addDensity(kernel->getCubicSpline(dist) * bp.psi);
			}
		}
	}
}

// Denominator counterpart of addBoundaryParticleDensity() -- see the header doc.
void DFSPHSolver::addBoundaryParticleAlpha(std::vector<DFSPHParticle>& particles,
                                            const std::vector<IBoundaryParticles*>& boundaries)
{
	if (boundaries.empty() || particles.empty()) return;

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto& p = particles[i];
		SPHKernel* kernel = p.getParent()->getKernel();
		if (kernel == nullptr) continue;
		const float effectLength = kernel->getEffectLength();

		for (auto* boundary : boundaries) {
			for (auto& bp : boundary->particles()) {
				const float dist = Math::getDistance(p.getPosition(), bp.worldPos);
				if (dist >= effectLength) continue;
				const auto v = p.getPosition() - bp.worldPos;
				p.addBoundaryAlphaGradient(kernel->getCubicSplineGradient(v) * bp.psi);
			}
		}
	}
}

// Akinci et al. 2012: reuses the standard SPH pressure-force formula
// f_i = -m_i * sum_j m_j * (p_i/rho_i^2 + p_j/rho_j^2) * gradW_ij, dropping the
// p_j/rho_j^2 term for boundary neighbors (they have no pressure of their own)
// and substituting the neighbor mass m_j with the boundary particle's psi.
// Shared by rigid and SoftBody boundaries alike -- see addBoundaryParticleDensity().
void DFSPHSolver::addBoundaryParticlePressure(std::vector<DFSPHParticle>& particles,
                                               const std::vector<IBoundaryParticles*>& boundaries,
                                               const float dt)
{
	if (boundaries.empty() || particles.empty()) return;

	// simulate() calls this once per *substep*, but the caller consumes
	// accumForce once per *frame* and treats it as a force acting over the
	// whole frame (RigidFluidSolver::step()/SoftFluidSolver::step()). Weighting
	// each substep's contribution by its share of the frame turns the running
	// sum into the frame-average force instead of a raw sum whose magnitude
	// tracks the substep count -- which is CFL-derived and therefore data
	// dependent, so an unweighted sum made the reaction on the rigid body swing
	// by orders of magnitude between frames (docs/issue/CODEBASE_ISSUES.md 1.6).
	// The fluid-side force below is per-substep and stays unweighted.
	const float frameShare = (frameTimeStep_ > 0.0f) ? (dt / frameTimeStep_) : 1.0f;

	// Sequential: bp.accumForce is shared across fluid particles, so this loop
	// cannot be parallelized the way the sibling boundary method above is
	// without introducing a data race on the boundary particles' accumForce.
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto& p = particles[i];
		SPHKernel* kernel = p.getParent()->getKernel();
		if (kernel == nullptr) continue;

		const float restDensity = p.getParent()->getDensity();
		const float rho_i = std::max(p.getDensity(), 1.0e-6f);
		const float pressure_i = std::max(p.getParent()->pressureCoe * (rho_i - restDensity), 0.0f);
		const float effectLength = kernel->getEffectLength();

		for (auto* boundary : boundaries) {
			for (auto& bp : boundary->particles()) {
				const float dist = Math::getDistance(p.getPosition(), bp.worldPos);
				if (dist >= effectLength) continue;

				const auto v = p.getPosition() - bp.worldPos;
				const auto gradW = kernel->getCubicSplineGradient(v);
				const auto f = -p.getMass() * bp.psi * (pressure_i / (rho_i * rho_i)) * gradW;

				p.addForce(f);
				bp.accumForce -= f * frameShare;
			}
		}
	}
}

void DFSPHSolver::addBoundaryViscosity(std::vector<DFSPHParticle>& particles)
{
	(void)particles;
}

float DFSPHSolver::calculateRestDensity(const float searchLength, const float particleRadius, const float mass, DFSPHFluid* fluid)
{
	(void)searchLength;

	DFSPHParticleSoA soa;
	size_t refIndex = 0;
	for (int i = 0; i < 5; ++i) {
		for (int j = 0; j < 5; ++j) {
			for (int k = 0; k < 5; ++k) {
				soa.push_back(Vector3df(i * particleRadius * 2.0f, j * particleRadius * 2.0f, k * particleRadius * 2.0f), particleRadius, mass);
				if (i == 2 && j == 2 && k == 2) {
					refIndex = soa.size() - 1;
				}
			}
		}
	}

	std::vector<DFSPHParticle> particles;
	particles.reserve(soa.size());
	for (size_t i = 0; i < soa.size(); ++i) {
		particles.emplace_back(soa, i, fluid);
	}

	// Must match the kernel used by calculateDensity() below -- see
	// simulate()'s searchRadius comment.
	const auto searchRadius = fluid->getKernel()->getEffectLength();

	std::vector<Vector3df> positions;
	positions.reserve(particles.size());
	for (const auto& particle : particles) {
		positions.push_back(particle.getPosition());
	}

	CSRNeighborList neighborIndices;
	neighborIndices.build(positions, searchRadius);

	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		particles[i].calculateDensity(particles, neighborIndices[i]);
	}
	const float d = particles[refIndex].getDensity();
	return d;
}
