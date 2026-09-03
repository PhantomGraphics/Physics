#pragma once

#include <vector>

#include "CGLib/Math/Vector3d.h"
#include "CGLib/Math/Box3d.h"
#include "PlaneBoundary.h"
#include "SphereBoundary.h"
#include "PlateBoundary.h"
#include "CylinderBoundary.h"
#include "RigidBoundary.h"
#include "RigidBoundaryParticles.h"
#include "SoftBoundaryParticles.h"
#include "IBoundaryParticles.h"
#include "ISPHSolver.h"
#include "DFSPHParticle.h"
#include "CGLib/Space/Space/NeighborList.h"
#include <memory>

namespace Phantom {
	namespace Physics {

class DFSPHFluid;

/**
 * @brief Solver for Divergence-Free SPH (DFSPH) fluid simulation.
 *
 * Advances DFSPH fluid objects through time by enforcing both
 * divergence-free velocity fields and constant density conditions.
 */
class DFSPHSolver : public ISPHSolver
{
public:
	DFSPHSolver();

	/**
	 * @brief Registers a fluid object with the solver.
	 * @param fluid Pointer to the fluid to simulate.
	 */
	void add(DFSPHFluid* fluid) { this->fluids.push_back(fluid); }

	/** @brief Removes all registered fluids (does not delete them; caller owns them). */
	void clear() override { this->fluids.clear(); }

	/**
	 * @brief Advances the simulation by one step.
	 * @param dt      Frame duration to advance. Adaptive substeps land exactly
	 *                on this duration; setTimeStep() only limits their size.
	 * @param maxIter Maximum correction sweeps per adaptive substep. Density
	 *                correction retains its required minimum of two sweeps.
	 */
	void simulate(const float dt, const int maxIter) override;

	/**
	 * @brief Sets the domain container's walls as an axis-aligned box, internally
	 * represented as 6 inward-facing PlaneBoundary instances.
	 * @param box Axis-aligned bounding box defining the interior region.
	 * @param timeStep Ignored by this solver -- see the note on
	 *        setBoundaryPlanes() below.
	 */
	void setBoundary(const Math::Box3df& box, const float timeStep) override {
		(void)timeStep;
		this->boundaryPlanes_ = ownShapeBoundaries(makeBoxPlaneBoundaries(box));
	}

	/**
	 * @brief Sets the domain container's walls as an arbitrary set of planes.
	 * @param planes Plane boundaries; valid region is the intersection of all of them.
	 * @param timeStep Ignored by this solver. Unlike WCSPH/PBSPH, DFSPH does
	 *        integrates the caller's frame duration using adaptive substeps
	 *        bounded by setTimeStep(). The wall penalty spring is calibrated as -d/dt^2
	 *        against the step actually being integrated, so
	 *        addBoundaryPressure() uses that substep and storing a separate
	 *        boundary time step here would only reintroduce the mismatch it
	 *        used to have (see addBoundaryPressure()'s definition).
	 */
	void setBoundaryPlanes(std::vector<PlaneBoundary> planes, const float timeStep) override {
		(void)timeStep;
		this->boundaryPlanes_ = ownShapeBoundaries(std::move(planes));
	}

	void setBoundarySpheres(std::vector<SphereBoundary> spheres, const float timeStep) override {
		(void)timeStep;
		boundarySpheres_ = ownShapeBoundaries(std::move(spheres));
	}

	void setBoundaryPlates(std::vector<PlateBoundary> plates, const float timeStep) override {
		(void)timeStep;
		boundaryPlates_ = ownShapeBoundaries(std::move(plates));
	}
	void setBoundaryCylinders(std::vector<CylinderBoundary> cylinders, const float timeStep) override {
		(void)timeStep;
		boundaryCylinders_ = ownShapeBoundaries(std::move(cylinders));
	}

	void setShapeBoundaries(std::vector<std::shared_ptr<IShapeBoundary>> boundaries,
	                        const float timeStep) override {
		(void)timeStep;
		boundaryShapes_ = std::move(boundaries);
	}

	/**
	 * @brief Sets how much of a particle's wall-normal velocity the domain
	 * walls absorb on contact (see ISPHSolver::setBoundaryDampingRatio()).
	 * @param ratio Damping ratio in [0, 0.5]; 0 (default) = historical
	 *        undamped penalty spring.
	 */
	void setBoundaryDampingRatio(const float ratio) override { this->boundaryDampingRatio_ = ratio; }

	float getBoundaryDampingRatio() const override { return this->boundaryDampingRatio_; }

	/**
	 * @brief Registers a rigid-body boundary for One-Way SDF penalty coupling.
	 * @param b Non-owning pointer to the boundary; must outlive the solver.
	 */
	void addRigidBoundary(RigidBoundary* b) override { rigidBoundaries_.push_back(b); }

	/** @brief Removes all registered rigid-body boundaries. */
	void clearRigidBoundaries() override { rigidBoundaries_.clear(); }

	/**
	 * @brief Registers a rigid-body boundary particle set for Two-Way (Track B) coupling.
	 * @param r Non-owning pointer; must outlive the solver.
	 */
	void addRigidBoundaryParticles(RigidBoundaryParticles* r) override { rigidBoundaryParticles_.push_back(r); }

	/** @brief Removes all registered Two-Way rigid-body boundary particle sets. */
	void clearRigidBoundaryParticles() override { rigidBoundaryParticles_.clear(); }

	/** @brief DFSPH implements Two-Way (Track B) coupling. */
	bool supportsTwoWayCoupling() const override { return true; }

	/**
	 * @brief Registers a SoftBody boundary particle set for Two-Way coupling.
	 * @param s Non-owning pointer; must outlive the solver.
	 */
	void addSoftBoundaryParticles(SoftBoundaryParticles* s) override { softBoundaryParticles_.push_back(s); }

	/** @brief Removes all registered SoftBody boundary particle sets. */
	void clearSoftBoundaryParticles() override { softBoundaryParticles_.clear(); }

	/**
	 * @brief Adds each boundary particle's psi-weighted kernel contribution to
	 * nearby fluid particles' density (Akinci et al. 2012, Eq. 2-3). Without
	 * this, a submerged boundary never registers as "compression" and
	 * addBoundaryParticlePressure()'s pressure term stays zero regardless of
	 * proximity. Call once per density recompute (mirrors addBoundaryDensity()
	 * above), after calculateDensity(). Shared by rigid and SoftBody boundary
	 * particle sets alike (internal design notes,
	 * Phase 3) -- the two only differ in how their worldPos/psi get produced,
	 * not in how this loop consumes them.
	 * @param particles  Fluid particles to accumulate density onto.
	 * @param boundaries Boundary particle sets to couple against (rigid or SoftBody).
	 */
	void addBoundaryParticleDensity(std::vector<DFSPHParticle>& particles,
	                                 const std::vector<IBoundaryParticles*>& boundaries);

	/**
	 * @brief Adds each boundary particle's psi-weighted kernel gradient to
	 * nearby fluid particles' alpha (DFSPHParticle::addBoundaryAlphaGradient()).
	 *
	 * The denominator counterpart of addBoundaryParticleDensity()'s numerator:
	 * both must be applied together, or the density-error/divergence solve
	 * sees a boundary-induced density excess with no matching stiffness term
	 * and diverges (internal design notes 1.6). Mirrors
	 * PBSPHSolver::addBoundaryParticleConstraintGradient(). Call right after
	 * each calculateAlpha() pass, for the same boundary lists that
	 * addBoundaryParticleDensity() is called with.
	 * @param particles  Fluid particles whose alpha to extend.
	 * @param boundaries Boundary particle sets to couple against (rigid or SoftBody).
	 */
	void addBoundaryParticleAlpha(std::vector<DFSPHParticle>& particles,
	                               const std::vector<IBoundaryParticles*>& boundaries);

	/**
	 * @brief Computes the Two-Way (Track B) pressure-force coupling between
	 * fluid particles and boundary particles: pushes each fluid particle out
	 * (added to its force accumulator) and accumulates the equal-and-opposite
	 * reaction on the boundary particles' accumForce (consumed once per
	 * PhysicsSolver frame by RigidFluidSolver::applyTwoWayReactions() /
	 * SoftFluidSolver::applyTwoWayReactions()). Called automatically from
	 * simulate() using the boundaries registered via addRigidBoundaryParticles()/
	 * addSoftBoundaryParticles(), but exposed publicly since (unlike the other
	 * addXxxBoundaryYyy() helpers) it takes its inputs as explicit parameters
	 * rather than solver-internal state.
	 * @param particles  Fluid particles to couple.
	 * @param boundaries Boundary particle sets to couple against (rigid or SoftBody).
	 * @param dt         This substep's time step (seconds). The fluid-side force
	 *                   is per-substep and does not use it; the boundary-side
	 *                   reaction is weighted by dt/frameDt so accumForce
	 *                   ends the frame holding the frame-average force rather
	 *                   than a substep-count-dependent sum (see the .cpp).
	 */
	void addBoundaryParticlePressure(std::vector<DFSPHParticle>& particles,
	                                  const std::vector<IBoundaryParticles*>& boundaries,
	                                  const float dt);

	/**
	 * @brief Sets the external body force applied to all particles (e.g., gravity).
	 * @param force External force vector.
	 */
	void setExternalForce(const Math::Vector3df& force) override { this->externalForce = force; }

	/**
	 * @brief Sets the maximum allowed time step.
	 * @param dt Maximum time step (seconds).
	 */
	void setTimeStep(const float dt) override { this->maxTimeStep = dt; }
	void setMaxSubstep(const float dt) override { setTimeStep(dt); }

	/** Applies the kernel support radius to every fluid currently registered. */
	void setEffectLength(const float length) override;

	void addShapeBoundary(std::shared_ptr<IShapeBoundary> boundary) override {
		if (boundary) boundaryShapes_.push_back(std::move(boundary));
	}
	void clearShapeBoundaries() override {
		boundaryPlanes_.clear(); boundarySpheres_.clear(); boundaryPlates_.clear(); boundaryCylinders_.clear(); boundaryShapes_.clear();
	}
	SPHSolveStats getLastSolveStats() const override { return lastSolveStats_; }

	/**
	 * @brief Returns the SPH kernel of the first registered fluid.
	 * @return Pointer to the SPHKernel, or nullptr if no fluid is registered.
	 */
	SPHKernel* getKernel() override;

	/**
	 * @brief Returns the rest density of the first registered fluid.
	 * @return Rest density, or 0.f if no fluid is registered.
	 */
	float getRestDensity() const override;

	/** @brief Returns the total particle count across all registered fluids. */
	int getParticleCount() const override;

	/** @brief Returns the world-space positions of all particles across all registered fluids. */
	std::vector<Math::Vector3df> getParticlePositions() const override;
	std::vector<Math::Vector3df> getParticleVelocities() const override;
	std::vector<float> getParticleDensities() const override;

	/**
	 * @brief Computes the rest density for a DFSPH fluid configuration.
	 * @param searchLength   Unused; the neighbor search radius is taken from
	 *                       fluid->getKernel()->getEffectLength() instead, to
	 *                       stay consistent with the kernel actually used to
	 *                       weight the density sum below (caller must have
	 *                       already called fluid->setEffectLength()).
	 * @param particleRadius Particle radius.
	 * @param mass           Particle mass.
	 * @param fluid          Fluid whose kernel is used for the computation.
	 * @return Computed rest density.
	 */
	static float calculateRestDensity(const float searchLength, const float particleRadius, const float mass, DFSPHFluid* fluid);

	/**
	 * @brief Default relative tolerance for correctDivergenceError()'s
	 * convergence check (internal design notes Phase 3):
	 * the loop stops once one substep's predicted density drift from
	 * residual divergence (|averageDpDt| * dt) is below this fraction of
	 * restDensity, instead of the pre-Phase-3 hardcoded absolute threshold
	 * on the raw (non-restDensity-normalized) averageDpDt. Calibrated to
	 * reproduce that historical threshold (averageDpDt < 2.0f) exactly at
	 * the historical default maxTimeStep=0.01f (dt capped to
	 * maxTimeStep/2=0.005f by the CFL rule in calculateTimeStep()) and
	 * restDensity=1: 0.01f == 2.0f * 0.005f.
	 */
	static constexpr float kDefaultMaxDivergenceErrorRatio = 0.01f;

	/**
	 * @brief Whether one Jacobi sweep of the divergence-free velocity
	 * correction has converged, as a dimensionless ratio independent of
	 * restDensity and the length/time scale (see kDefaultMaxDivergenceErrorRatio).
	 * @param averageDpDt Mean per-particle density time-derivative from calculateAverageDpDt().
	 * @param restDensity Rest density of the fluid being corrected.
	 * @param dt Current substep time step.
	 * @param maxDivergenceErrorRatio Convergence tolerance as a fraction of restDensity's one-step drift.
	 */
	static bool isDivergenceErrorAcceptable(float averageDpDt, float restDensity, float dt,
	                                         float maxDivergenceErrorRatio = kDefaultMaxDivergenceErrorRatio);

private:
	std::vector<DFSPHFluid*> fluids;
	Math::Vector3df externalForce;
	// Upper bound on the adaptive substep dt (see calculateTimeStep()'s CFL
	// rule below). Not itself derived from any fluid's radius/effectLength --
	// callers normally overwrite this via setTimeStep() (all production
	// scene builders do). The 0.01f constructor default (DFSPHSolver.cpp) is
	// only a fallback for a solver used without an explicit setTimeStep()
	// call, and was chosen as roughly the CFL limit
	// (0.4*2*radius/velocity) for the codebase's conventional default scene
	// (radius=1, typical fall velocities of a few units/s) -- rescale it
	// proportionally to the scene's length scale if radius moves away from
	// that default (internal design notes Phase 6).
	float maxTimeStep;
	std::vector<std::shared_ptr<IShapeBoundary>> boundaryPlanes_;
	std::vector<std::shared_ptr<IShapeBoundary>> boundarySpheres_;
	std::vector<std::shared_ptr<IShapeBoundary>> boundaryPlates_;
	std::vector<std::shared_ptr<IShapeBoundary>> boundaryCylinders_;
	std::vector<std::shared_ptr<IShapeBoundary>> boundaryShapes_;
	// Duration requested by the current simulate() call. Used to average
	// two-way reactions and to cap passive-wall recovery across substeps.
	float frameTimeStep_ = 0.0f;
	SPHSolveStats lastSolveStats_;
	// No boundaryTimeStep member on purpose: addBoundaryPressure() takes the
	// adaptive substep it is about to integrate. See setBoundaryPlanes().
	// 0 == the historical undamped penalty spring; see
	// ISPHSolver::setBoundaryDampingRatio().
	float boundaryDampingRatio_ = 0.0f;
	std::vector<RigidBoundary*> rigidBoundaries_;
	// Kept as two separate lists (rather than one combined
	// std::vector<IBoundaryParticles*>) so clearRigidBoundaryParticles() and
	// clearSoftBoundaryParticles() can each drop only their own kind, as
	// FluidWorld::teardownCoupling()/teardownSoftCoupling() require (they are
	// toggled independently). addBoundaryParticleDensity()/
	// addBoundaryParticlePressure() above are still called once per list
	// (rigid first, then soft -- the pre-Phase-3 order), so the duplicated
	// *loop bodies* are gone even though the two lists remain.
	std::vector<IBoundaryParticles*> rigidBoundaryParticles_;
	std::vector<IBoundaryParticles*> softBoundaryParticles_;

	void correctDivergenceError(std::vector<DFSPHParticle>& particles, const Space::CSRNeighborList& neighbors,
	                            const float dt, const int maxIter);

	void correctDensityError(std::vector<DFSPHParticle>& particles, const Space::CSRNeighborList& neighbors,
	                         const float dt, const int maxIter);

	float calculateTimeStep(const std::vector<DFSPHParticle>& particles);

	float calculateAverageDensity(const std::vector<DFSPHParticle>& particles);

	float calculateAverageDpDt(const std::vector<DFSPHParticle>& particles);

	void addBoundaryDensity(std::vector<DFSPHParticle>& particles);

	void addBoundaryPressure(std::vector<DFSPHParticle>& particles, const float dt);

	void addRigidBoundaryPressure(std::vector<DFSPHParticle>& particles);

	void addBoundaryViscosity(std::vector<DFSPHParticle>& particles);
	void addBoundaryParticleConstraintTerms(std::vector<DFSPHParticle>& particles,
	                                        const std::vector<IBoundaryParticles*>& boundaries);

};

	}
}
