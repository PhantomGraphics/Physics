#pragma once

#include <memory>
#include <vector>

#include "CGLib/Math/Vector3d.h"
#include "CGLib/Math/Box3d.h"
#include "CGLib/Util/UnCopyable.h"
#include "PlaneBoundary.h"
#include "SphereBoundary.h"
#include "PlateBoundary.h"
#include "IShapeBoundary.h"
#include "RigidBoundary.h"
#include "RigidBoundaryParticles.h"
#include "SoftBoundaryParticles.h"
#include "IBoundaryParticles.h"
#include "ISPHSolver.h"

namespace Phantom {
	namespace Physics {

class PBSPHParticle;
class PBSPHFluid;

/**
 * @brief Solver for Position-Based SPH (PBSPH) fluid simulation.
 *
 * Manages one or more PBSPHFluid objects and advances the simulation
 * by iteratively projecting incompressibility constraints.
 */
class PBSPHSolver : public ISPHSolver, private UnCopyable
{
public:
	PBSPHSolver();

	/** @brief Removes all registered fluid objects from the solver. */
	void clear() override { this->fluids.clear(); }

	/**
	 * @brief Registers a fluid object with the solver.
	 * @param fluid Pointer to the fluid to simulate.
	 */
	void add(PBSPHFluid* fluid) { this->fluids.push_back(fluid); }

	/**
	 * @brief Advances all registered fluids by the requested frame duration.
	 * @param dt      Frame duration to advance (seconds). PBSPH integrates this
	 *                in a single position-based step -- it does not split dt into
	 *                adaptive substeps, so setMaxSubstep()/setTimeStep() do not
	 *                bound it (see setTimeStep()).
	 * @param maxIter Number of incompressibility-constraint projection iterations.
	 */
	void simulate(const float dt, const int maxIter) override;

	/**
	 * @brief Sets the box boundary used for the domain container's walls.
	 * Cheap, O(1)-per-particle mechanism (no boundary-particle sampling,
	 * unlike addRigidBoundaryParticles() -- a domain box is often sized much
	 * larger than the fluid itself, so sampling its surface would be an
	 * O(surface area) cost, not just O(1) per fluid particle):
	 *   1. addBoundadryPressure(): a full, unconditional penalty (cancels
	 *      the whole penetration depth), called once per iteration alongside
	 *      the fluid-fluid pressure pass.
	 *   2. clampToBoundary(): a hard clamp, called once per iteration after
	 *      the density-driven predictPosition update. Cheap extra insurance
	 *      against tunneling through a wall in one step; not load-bearing
	 *      for ordinary contact once (3) below is in place.
	 *   3. addShapeBoundaryConstraint(): the paired boundary density /
	 *      constraint-gradient contribution -- the box planes go through the
	 *      same analytic-shape path as spheres and plates, so lambda near a
	 *      wall is calibrated for the fluid the wall stands in for.
	 *   4. PBSPHParticle::calculateLambda()'s lambda clamp is what actually
	 *      makes (1) safe to apply at full strength. This was investigated
	 *      at length (internal design notes,
	 *      "problem B"): a dam break settling against a wall diverged to the
	 *      thousands with the *unclamped* PBF-normalized lambda, even though
	 *      the pre-Two-Way-coupling algorithm (a raw, non-lambda density
	 *      constraint, from before this codebase added Track B) used this
	 *      exact same full boundary correction and was stable. Reproducing
	 *      both algorithms confirmed the unclamped lambda -- which blows up
	 *      for a particle with few/no neighbors (e.g. isolated near a wall,
	 *      where neighbors past the wall don't exist, so the constraint
	 *      gradient's denominator collapses toward zero) -- was the actual
	 *      source of the divergence, not the boundary correction's strength.
	 * @param box Axis-aligned bounding box defining the interior region.
	 * @param timeStep Time step used to scale the repulsion force.
	 */
	void setBoundary(const Math::Box3df& box, const float timeStep) override {
		this->boundaryTimeStep = timeStep;
		this->boundaryPlanes_ = ownShapeBoundaries(makeBoxPlaneBoundaries(box));
	}

	/**
	 * @brief Sets the domain container's walls as an arbitrary set of planes.
	 * @param planes Plane boundaries; valid region is the intersection of all of them.
	 * @param timeStep Time step used to scale the repulsion force.
	 */
	void setBoundaryPlanes(std::vector<PlaneBoundary> planes, const float timeStep) override {
		this->boundaryTimeStep = timeStep;
		this->boundaryPlanes_ = ownShapeBoundaries(std::move(planes));
	}

	/**
	 * @brief Sets additional solid-sphere container walls, on top of (not
	 * instead of) the box/plane boundary.
	 * @param spheres  Sphere boundaries; valid region is the union interior of all of them.
	 * @param timeStep Time step used to scale the repulsion force.
	 */
	void setBoundarySpheres(std::vector<SphereBoundary> spheres, const float timeStep) override {
		this->boundaryTimeStep = timeStep;
		this->boundarySpheres_ = ownShapeBoundaries(std::move(spheres));
	}

	/**
	 * @brief Sets additional finite-plate container walls, on top of (not
	 * instead of) the box/plane boundary.
	 * @param plates   Finite plate boundaries; each plate's valid region is its exterior.
	 * @param timeStep Time step used to scale the repulsion force.
	 */
	void setBoundaryPlates(std::vector<PlateBoundary> plates, const float timeStep) override {
		this->boundaryTimeStep = timeStep;
		this->boundaryPlates_ = ownShapeBoundaries(std::move(plates));
	}

	/**
	 * @brief Registers arbitrary analytic boundaries through their common
	 * interface. Evaluated through the exact same penalty / hard-clamp /
	 * paired-density-and-constraint-gradient path as the typed
	 * setBoundaryPlanes()/setBoundarySpheres()/setBoundaryPlates() lists, so a
	 * given shape behaves identically however it was registered.
	 * @param boundaries Analytic boundaries to own.
	 * @param timeStep   Time step used to scale the repulsion force.
	 */
	void setShapeBoundaries(std::vector<std::shared_ptr<IShapeBoundary>> boundaries,
	                        const float timeStep) override {
		this->boundaryTimeStep = timeStep;
		this->boundaryShapes_ = std::move(boundaries);
	}

	/** Adds one analytic boundary without replacing those already registered. */
	void addShapeBoundary(std::shared_ptr<IShapeBoundary> boundary) override {
		if (boundary) boundaryShapes_.push_back(std::move(boundary));
	}

	/** Clears every analytic boundary registered through typed or generic APIs. */
	void clearShapeBoundaries() override {
		boundaryPlanes_.clear();
		boundarySpheres_.clear();
		boundaryPlates_.clear();
		boundaryShapes_.clear();
	}

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

	/** @brief PBSPH implements Two-Way (Track B) coupling. */
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
	 * addBoundaryParticlePressure()'s constraint stays zero regardless of
	 * proximity. Call once per iteration, after the fluid-fluid density
	 * accumulation and before calculatePressure(). Shared by rigid and
	 * SoftBody boundary particle sets alike
	 * (internal design notes, Phase 3).
	 * @param particles  Fluid particles to accumulate density onto.
	 * @param boundaries Boundary particle sets to couple against (rigid or SoftBody).
	 */
	void addBoundaryParticleDensity(std::vector<PBSPHParticle>& particles,
	                                 const std::vector<IBoundaryParticles*>& boundaries);

	/**
	 * @brief Adds each boundary particle's psi-weighted kernel gradient to
	 * nearby fluid particles' constraint-gradient accumulators, so that
	 * calculateLambda()'s denominator reflects boundary proximity consistently
	 * with addBoundaryParticleDensity()'s contribution to the numerator (the
	 * density constraint itself). Without this, a fluid particle whose density
	 * comes mostly from a nearby boundary (few fluid neighbors) would divide
	 * by a near-zero denominator and produce a wildly miscalibrated lambda.
	 * Call once per iteration, after the fluid-fluid
	 * accumulateConstraintGradient() pass and before calculateLambda().
	 * @param particles  Fluid particles to accumulate the gradient onto.
	 * @param boundaries Boundary particle sets to couple against (rigid or SoftBody).
	 */
	void addBoundaryParticleConstraintGradient(std::vector<PBSPHParticle>& particles,
	                                            const std::vector<IBoundaryParticles*>& boundaries);

	/**
	 * @brief Computes the Two-Way (Track B) constraint coupling between fluid
	 * particles and boundary particles: applies a position correction to each
	 * fluid particle (via addPositionCorrection()) and accumulates the
	 * equal-and-opposite reaction (converted back to a force) on the boundary
	 * particles' accumForce. Called automatically from simulate() using the
	 * boundaries registered via addRigidBoundaryParticles()/
	 * addSoftBoundaryParticles(), but exposed publicly since it takes its
	 * inputs as explicit parameters rather than solver-internal state.
	 * @param particles  Fluid particles to couple.
	 * @param boundaries Boundary particle sets to couple against (rigid or SoftBody).
	 * @param dt         Time step (seconds); used to convert the PBD position
	 *                  correction back into a force for accumForce.
	 */
	void addBoundaryParticlePressure(std::vector<PBSPHParticle>& particles,
	                                  const std::vector<IBoundaryParticles*>& boundaries,
	                                  const float dt);

	/**
	 * @brief Sets the external body force applied to all particles (e.g., gravity).
	 * @param force External force vector.
	 */
	void setExternalForce(const Math::Vector3df& force) override { this->externalForce = force; }

	/**
	 * @brief Sets the time step used to convert boundary penalty accelerations
	 * into position corrections.
	 *
	 * Unlike WCSPH/DFSPH this is NOT an adaptive-substep ceiling: PBSPH advances
	 * simulate()'s dt in a single position-based step. maxTimeStep only feeds the
	 * dt^2 factor in addBoundadryPressure()/addRigidBoundaryPressure(), so with
	 * no analytic/rigid boundary registered it has no effect on the result.
	 * @param dt Time step (seconds).
	 */
	void setTimeStep(const float dt) override { this->maxTimeStep = dt; }
	/** Explicit alias for setTimeStep(); see the note there on why PBSPH has no
	 *  actual adaptive substep. */
	void setMaxSubstep(const float dt) override { setTimeStep(dt); }

	/** Applies the kernel support radius to every fluid currently registered. */
	void setEffectLength(const float length) override;

	/** @brief Returns diagnostics for the most recent simulate() call
	 *  (mirrors DFSPHSolver/WCSPHSolver). */
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

private:
	std::vector<PBSPHFluid*> fluids;
	Math::Vector3df externalForce;
	// See DFSPHSolver.h's identical field for the CFL/default-scene
	// calibration note (internal design notes Phase 6) --
	// same role here (upper bound on calculateTimeStep()'s adaptive dt,
	// normally overwritten via setTimeStep()).
	float maxTimeStep;

	// Diagnostics for the most recent simulate() call, reset at its start.
	// PBSPH is a single-step position-based solver, so on a valid solve
	// substeps == 1 and advancedTime == the requested dt; the maxIter
	// incompressibility-constraint iterations are counted into
	// densityIterations, divergenceIterations stays 0 (no separate divergence
	// solve), and converged stays true (PBSPH has no residual tolerance check).
	SPHSolveStats lastSolveStats_;

	float calculateTimeStep(const std::vector<PBSPHParticle>& particles);

	/**
	 * @brief Full, unconditional penalty against `boundary`: cancels the
	 * whole penetration depth per iteration (see setBoundary()'s doc
	 * comment -- safe because calculateLambda() clamps lambda). Called
	 * alongside the fluid-fluid pressure pass, before updatePredictPosition().
	 */
	void addBoundadryPressure(std::vector<PBSPHParticle>& particles);

	/**
	 * @brief Hard-clamps every particle's predicted position to stay inside
	 * `boundary`, after the density/pressure correction for this iteration
	 * has already been applied. A safety net only -- see setBoundary().
	 */
	void clampToBoundary(std::vector<PBSPHParticle>& particles);

	/**
	 * @brief For every registered analytic shape boundary (planes, spheres,
	 * plates and generic shapes alike), adds -- from a single
	 * IShapeBoundary::sample() -- both the boundary's contribution to each
	 * penetrating fluid particle's density (the PBF constraint numerator) and
	 * the matching constraint-gradient contribution (its denominator). The two
	 * must be added together: a density excess with no matching gradient term
	 * drives calculateLambda() unstable. Call once per iteration, after the
	 * fluid-fluid and boundary-particle gradient passes and before calculateLambda().
	 */
	void addShapeBoundaryConstraint(std::vector<PBSPHParticle>& particles);

	/** True while at least one analytic shape boundary of any kind is registered. */
	bool hasShapeBoundaries() const {
		return !boundaryPlanes_.empty() || !boundarySpheres_.empty() ||
		       !boundaryPlates_.empty() || !boundaryShapes_.empty();
	}

	void addRigidBoundaryPressure(std::vector<PBSPHParticle>& particles);

	// Owned polymorphic copies of the domain container's walls (currently only
	// PlaneBoundary is registered here -- Sphere/Plate/generic shape support is
	// a later change unit). std::shared_ptr<IShapeBoundary> to match
	// WCSPHSolver/DFSPHSolver's ownership model so the same registration and
	// iteration structure can be reused.
	// Owned polymorphic copies of the domain container's analytic walls, kept in
	// four lists so each typed setter (and clearShapeBoundaries()) can replace
	// exactly its own kind -- mirrors WCSPHSolver/DFSPHSolver. All four are
	// evaluated through the identical penalty / hard-clamp / paired
	// density-and-constraint-gradient path (addBoundadryPressure(),
	// clampToBoundary(), addShapeBoundaryConstraint()).
	std::vector<std::shared_ptr<IShapeBoundary>> boundaryPlanes_;
	std::vector<std::shared_ptr<IShapeBoundary>> boundarySpheres_;
	std::vector<std::shared_ptr<IShapeBoundary>> boundaryPlates_;
	std::vector<std::shared_ptr<IShapeBoundary>> boundaryShapes_;
	// Fallback used only before setBoundary()/setBoundaryPlanes() is called
	// (both overwrite it); see DFSPHSolver.h's identical field for the
	// default-scene calibration note.
	float boundaryTimeStep = 0.01f;
	std::vector<RigidBoundary*> rigidBoundaries_;
	// Two separate lists, not one combined std::vector<IBoundaryParticles*> --
	// see DFSPHSolver.h's identical fields for why (independent
	// clearRigidBoundaryParticles()/clearSoftBoundaryParticles() semantics).
	std::vector<IBoundaryParticles*> rigidBoundaryParticles_;
	std::vector<IBoundaryParticles*> softBoundaryParticles_;

};

	}
}
