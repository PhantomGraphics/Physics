#include "pch.h"

#include "WCSPHSolver.h"
#include "WCSPHParticle.h"
#include "WCSPHFluid.h"
#include "OpenMPRuntimeTuning.h"

#include "CGLib/Space/Space/NeighborList.h"


using namespace Phantom::Math;
using namespace Phantom::Space;
using namespace Phantom::Physics;

namespace {

// Fraction of the Poly6 kernel's total mass lying beyond a plane whose
// (unsigned) distance from the particle is d. Used by
// WCSPHSolver::addBoundaryDensity() to treat a domain wall as a solid
// half-space filled with fluid at rest density.
//
// With W(r) = c (h^2 - r^2)^3, c = 315 / (64 pi h^9), integrating over the
// half-space {z >= d} in cylindrical coordinates collapses the radial integral
// in closed form:
//
//   I(d) = int_d^h int_0^sqrt(h^2-z^2) c (h^2 - z^2 - r^2)^3 2 pi r dr dz
//        = int_d^h (pi c / 4) (h^2 - z^2)^4 dz
//        = 315 / (256 h^9) * [ 128 h^9 / 315 - F(d) ]
//   F(z) = h^8 z - (4/3) h^6 z^3 + (6/5) h^4 z^5 - (4/7) h^2 z^7 + z^9 / 9
//
// Sanity checks: I(h) == 0 (wall out of range) and I(0) == 0.5 (particle
// exactly on the wall sees half a full neighbourhood), so a particle lying on
// the floor with fluid above reaches ~rest density instead of ~half of it.
float poly6HalfSpaceFraction(const float d, const float h)
{
	if (d >= h) return 0.0f;
	// A particle that has been pushed past the wall (d < 0) is capped at the
	// on-wall value rather than extrapolated: the integral keeps growing below
	// zero, and feeding that back as density during a penetration spike is
	// exactly when the solver can least afford extra pressure.
	const float z = (d > 0.0f) ? d : 0.0f;
	const float h2 = h * h;
	const float h4 = h2 * h2;
	const float h6 = h4 * h2;
	const float h8 = h4 * h4;
	const float h9 = h8 * h;
	const float z2 = z * z;
	const float z3 = z2 * z;
	const float z5 = z3 * z2;
	const float z7 = z5 * z2;
	const float z9 = z7 * z2;
	const float F = h8 * z
		- (4.0f / 3.0f) * h6 * z3
		+ (6.0f / 5.0f) * h4 * z5
		- (4.0f / 7.0f) * h2 * z7
		+ z9 / 9.0f;
	const float total = 128.0f * h9 / 315.0f;
	return (315.0f / (256.0f * h9)) * (total - F);
}

}

void WCSPHSolver::setEffectLength(const float length)
{
	for (auto* fluid : fluids) fluid->setEffectLength(length);
}

SPHKernel* WCSPHSolver::getKernel()
{
	return fluids.empty() ? nullptr : fluids.front()->getKernel();
}

float WCSPHSolver::getRestDensity() const
{
	return fluids.empty() ? 0.f : fluids.front()->getDensity();
}

int WCSPHSolver::getParticleCount() const
{
	int count = 0;
	for (const auto* fluid : fluids) {
		count += fluid->getNumParticles();
	}
	return count;
}

std::vector<Vector3df> WCSPHSolver::getParticlePositions() const
{
	std::vector<Vector3df> positions;
	for (const auto* fluid : fluids) {
		const auto& soa = fluid->getParticles();
		positions.insert(positions.end(), soa.positions.begin(), soa.positions.end());
	}
	return positions;
}

void WCSPHSolver::simulate(const float dt, const int maxIter)
{
	ensurePassiveOpenMPWaitPolicy();
	lastSolveStats_ = {};

	(void)maxIter;
	this->timeStep = dt;

	if (fluids.empty()) return;
	if (!std::isfinite(dt) || dt <= 0.0f) {
		lastSolveStats_.validConfiguration = false;
		return;
	}
	const float commonEffectLength = fluids.front()->getEffectLength();
	for (const auto* fluid : fluids) {
		if (!fluid || std::abs(fluid->getEffectLength() - commonEffectLength) >
		              std::max(1.0e-6f, std::abs(commonEffectLength) * 1.0e-5f)) {
			lastSolveStats_.validConfiguration = false;
			return;
		}
	}

	// Fail safe instead of feeding a garbage search radius to the neighbor
	// search below: effectLength defaults to 0.f
	// (SPHKernel/WCSPHFluid) until setEffectLength() is called, so a caller
	// that forgets to call it gets an inert (no-op) solver rather than an
	// undefined/UB neighbor-search radius (see simulate()'s doc comment and
	// docs/todo/PLAN_sph_scale_invariance.md Phase 5).
	if (commonEffectLength <= 0.0f) {
		lastSolveStats_.validConfiguration = false;
		return;
	}

	std::vector<WCSPHParticle> particles;

	for (auto fluid : fluids) {
		auto& soa = fluid->getParticles();
		for (size_t i = 0; i < soa.size(); ++i) {
			particles.emplace_back(soa, i, fluid);
		}
	}

	// Kernel now lives on the fluid (mirrors DFSPHFluid/PBSPHFluid), not a
	// throwaway solver-local SPHKernel -- see setEffectLength()'s doc comment.
	SPHKernel* kernel = fluids.front()->getKernel();
	for (auto& p : particles) {
		p.setKernel(kernel);
	}

	for (auto& particle : particles) {
		particle.init();
	}

	std::vector<Vector3df> positions;
	positions.reserve(particles.size());
	for (const auto& p : particles) {
		positions.push_back(p.getPosition());
	}

	//boundarySolver.setTimestep(timeStep);
	//boundarySolver.addDensity(particles);

	// Per-particle neighbor list, so the density/force passes below can be
	// parallelized over particles instead of over pairs. Parallelizing over
	// pairs let two pairs that share a particle land on different threads,
	// racing on that particle's density/force accumulators (non-atomic +=) and
	// making results non-deterministic across runs. Gathering per-particle
	// means each thread only ever writes to its own particle. See
	// CSRNeighborList's class doc for why the rows are flattened rather than
	// one std::vector<int> per particle.
	const float searchLength = fluids.front()->getEffectLength();
	CSRNeighborList neighbors;
	neighbors.build(positions, searchLength);

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		for (const int neighbor : neighbors[i]) {
			particles[i].addDensity(particles[neighbor]);
		}
	}

	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		particles[i].addSelfDensity();
	}

	this->addBoundaryDensity(particles);
	this->addBoundaryParticleDensity(particles, rigidBoundaryParticles_);
	this->addBoundaryParticleDensity(particles, softBoundaryParticles_);

	// Surface normals must be fully accumulated over all neighbors before
	// solveSurfaceTension() (below) can use them, the same reason density is
	// accumulated in its own pass before the pressure/viscosity pass.
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		for (const int neighbor : neighbors[i]) {
			particles[i].solveNormal(particles[neighbor]);
		}
	}

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		for (const int neighbor : neighbors[i]) {
			particles[i].solvePressureForce(particles[neighbor]);
			particles[i].solveViscosityForce(particles[neighbor]);
			particles[i].solveSurfaceTension(particles[neighbor]);
		}
	}

	for (auto& p : particles) {
		p.addExternalForce(externalForce * p.getDensity());
	}

	this->addBoundaryForce(particles);
	this->addRigidBoundaryPressure(particles);
	this->addBoundaryParticlePressure(particles, rigidBoundaryParticles_);
	this->addBoundaryParticlePressure(particles, softBoundaryParticles_);

	for (auto f : fluids) {
		if (f->isStatic()) {
			continue;
		}
		auto& soa = f->getParticles();
		for (size_t i = 0; i < soa.size(); ++i) {
			WCSPHParticle p(soa, i, f);
			p.forwardTime(timeStep);
		}
	}
	lastSolveStats_.substeps = 1;
	lastSolveStats_.advancedTime = dt;

	/*
	for (auto fluid : fluids) {
		fluid->getPresenter()->updateView();
	}
	*/
}

/*
void CSPHSolver::addBoundaryDensity(const std::vector<CSPHParticle*>& particles)
{
	const auto& grid = this->boundary->getVolume();
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		const auto v = particles[i]->getPosition();
		const auto vv = grid->getValueAt(v);

		if (vv > 0.0f) {
			auto k = particles[i]->getKernel();
			particles[i]->addDensity(k->getPoly6Kernel(vv) * particles[i]->getMass());
		}
	}
}
*/

// See the doc comment on the declaration (WCSPHSolver.h) for why the walls
// need to contribute density at all, and why this is done analytically rather
// than by sampling the walls with Akinci boundary particles.
void WCSPHSolver::addBoundaryDensity(std::vector<WCSPHParticle>& particles)
{
	if ((boundaryPlanes_.empty() && boundarySpheres_.empty() && boundaryPlates_.empty() && boundaryShapes_.empty()) || particles.empty()) return;

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto& p = particles[i];
		const SPHKernel* kernel = p.getKernel();
		if (kernel == nullptr) continue;
		const float effectLength = kernel->getEffectLength();
		if (effectLength <= 0.0f) continue;
		const float restDensity = p.getFluid()->getDensity();

		float contribution = 0.0f;
		auto addShapeDensity = [&](const IShapeBoundary& boundary) {
			if (!boundary.isActiveAt(p.getPosition(), effectLength)) return;
			const float d = boundary.getDensityDistance(p.getPosition());
			if (d >= effectLength) return;
			const float weight = boundary.getDensityWeight(p.getPosition(), effectLength);
			if (weight > 0.0f)
				contribution += restDensity * poly6HalfSpaceFraction(d, effectLength) * weight;
		};
		for (const auto& plane : boundaryPlanes_) addShapeDensity(plane);
		for (const auto& sphere : boundarySpheres_) addShapeDensity(sphere);
		// A finite plate contributes like a plane's half-space integral off its
		// large (normal-facing) face, but tapered toward the rim: near an edge
		// most of that half-space is air, not solid, so handing the particle a
		// full plane's worth would lift the water unnaturally at the weir lip
		// (docs/todo/PLAN_sph_showcase_waterfall.md section 3.4). Added into the
		// same accumulator and subject to the one headroom clamp below, exactly
		// like planes and spheres -- the clamp is what keeps overlapping plates
		// at a 9-plate seam from summing past rest density.
		for (const auto& plate : boundaryPlates_) addShapeDensity(plate);
		for (const auto& shape : boundaryShapes_) if (shape) addShapeDensity(*shape);
		// The wall stands in for the neighbors a particle is *missing* because
		// the wall is where they would have been -- so it may only ever fill
		// the deficit, never push the particle past rest density. Clamping to
		// the remaining headroom (rather than to restDensity, as this did
		// before) is what makes that true:
		//
		//  - Near a corner (two planes) or a plane/sphere seam the half-spaces
		//    overlap, so summing them double counts the wedge they share. The
		//    headroom clamp subsumes the old restDensity clamp that guarded
		//    this, since headroom <= restDensity always.
		//  - A particle whose *fluid* neighbors alone already reach rest
		//    density is not missing anything: the fluid has compressed into
		//    the very region the wall was standing in for, so adding the
		//    wall's share on top counts that space twice. That is what turned
		//    a jet landing in the empty water-sphere container into spray --
		//    the first layer against the floor was handed rho ~ 1500 (990
		//    from the column ramming into it, plus the wall's unconditional
		//    ~500) and the resulting pressure spike blew it back up at 2.4x
		//    its own impact speed. See docs/issue/
		//    water_sphere_showcase_emitter_instability.md.
		//
		// A settled pool is unaffected: its floor layer really is missing its
		// lower half (fluid-only density ~0.5*rho0), so the headroom is ~0.5*
		// rho0 and the wall still contributes all of it, exactly as before.
		const float headroom = restDensity - p.getDensity();
		if (contribution > headroom) contribution = headroom;
		if (contribution > 0.0f) p.addDensity(contribution);
	}
}

void WCSPHSolver::addBoundaryForce(std::vector<WCSPHParticle>& particles) {
	if (boundaryPlanes_.empty() && boundarySpheres_.empty() && boundaryPlates_.empty() && boundaryShapes_.empty()) return;
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		Vector3df force(0.0f, 0.0f, 0.0f);
		const Vector3df& pos = particles[i].getPosition();
		const Vector3df& vel = particles[i].getVelocity();
		// timeStep, not a separately stored boundary time step: the penalty
		// spring -d/dt^2 is calibrated against the step being integrated, and
		// simulate() has already set this to its own dt. See
		// setBoundaryPlanes()'s doc comment in the header.
		auto addShapeForce = [&](const IShapeBoundary& boundary) {
			if (boundary.isActiveAt(pos, 0.0f))
				force += boundary.getBoundaryForce(pos, vel, timeStep, boundaryDampingRatio_);
		};
		for (const auto& plane : boundaryPlanes_) addShapeForce(plane);
		for (const auto& sphere : boundarySpheres_) addShapeForce(sphere);
		// isActiveAt(pos, 0) exactly bounds the plate's OBB; getBoundaryForce()
		// still does the precise inside test, so this is only an early-out.
		for (const auto& plate : boundaryPlates_) addShapeForce(plate);
		for (const auto& shape : boundaryShapes_) if (shape) addShapeForce(*shape);
		particles[i].addForce(force * particles[i].getDensity());
	}
}

void WCSPHSolver::addRigidBoundaryPressure(std::vector<WCSPHParticle>& particles) {
	if (rigidBoundaries_.empty()) return;
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto& p = particles[i];
		for (auto* rb : rigidBoundaries_) {
			// Same force/density convention as addBoundaryForce() above.
			const auto force = rb->getBoundaryForce(p.getPosition());
			p.addForce(force * p.getDensity());
		}
	}
}

// Akinci et al. 2012, substituting the boundary sample's psi for a neighbor
// particle's mass -- same substitution DFSPHSolver/PBSPHSolver make, but
// using WCSPHParticle::addDensity(rhs)'s own Poly6 kernel/formula (WCSPH_.
// addDensity(const WCSPHParticle&) above) rather than DFSPH's cubic spline,
// since it must stay consistent with how this solver already accumulates
// fluid-fluid density.
void WCSPHSolver::addBoundaryParticleDensity(std::vector<WCSPHParticle>& particles,
                                              const std::vector<IBoundaryParticles*>& boundaries)
{
	if (boundaries.empty() || particles.empty()) return;

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto& p = particles[i];
		SPHKernel* kernel = p.getKernel();
		if (kernel == nullptr) continue;
		const float effectLength = kernel->getEffectLength();

		for (auto* boundary : boundaries) {
			for (auto& bp : boundary->particles()) {
				const float dist = Math::getDistance(p.getPosition(), bp.worldPos);
				if (dist >= effectLength) continue;
				p.addDensity(kernel->getPoly6Kernel(dist) * bp.psi);
			}
		}
	}
}

// Mirrors WCSPHParticle::solvePressureForce() (same Spiky-gradient, symmetric-
// pressure formula this solver already uses for fluid-fluid pairs), dropping
// the boundary's own pressure term (it has none) and substituting the
// neighbor mass with psi -- same Akinci et al. 2012 substitution
// addBoundaryParticleDensity() makes above.
void WCSPHSolver::addBoundaryParticlePressure(std::vector<WCSPHParticle>& particles,
                                               const std::vector<IBoundaryParticles*>& boundaries)
{
	if (boundaries.empty() || particles.empty()) return;

	// Sequential: bp.accumForce is shared across fluid particles, so this loop
	// cannot be parallelized without a data race on the boundary particles.
	for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
		auto& p = particles[i];
		SPHKernel* kernel = p.getKernel();
		if (kernel == nullptr) continue;
		const float effectLength = kernel->getEffectLength();
		const float pressure = p.getPressure() * 0.5f;

		for (auto* boundary : boundaries) {
			for (auto& bp : boundary->particles()) {
				const float dist = Math::getDistance(p.getPosition(), bp.worldPos);
				if (dist >= effectLength) continue;

				const auto distanceVector = p.getPosition() - bp.worldPos;
				const auto f = kernel->getSpikyKernelGradient(distanceVector) * pressure * bp.psi;

				p.addForce(f);
				bp.accumForce -= f;
			}
		}
	}
}
