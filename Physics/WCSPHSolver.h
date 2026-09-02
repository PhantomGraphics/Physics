#pragma once

#include "SPHKernel.h"
#include "CGLib/Math/Box3d.h"
#include "CGLib/Util/UnCopyable.h"
#include "PlaneBoundary.h"
#include "SphereBoundary.h"
#include "PlateBoundary.h"
#include "RigidBoundary.h"
#include "RigidBoundaryParticles.h"
#include "SoftBoundaryParticles.h"
#include "IBoundaryParticles.h"
#include "ISPHSolver.h"
#include "WCSPHParticle.h"
#include <vector>

namespace Phantom {
	namespace Physics {
		class WCSPHFluid;

/**
 * @brief Solver for Classical SPH (CSPH) fluid simulation.
 *
 * Advances registered CSPHFluid objects by computing SPH forces
 * (pressure, viscosity, surface tension) and integrating particle positions.
 */
class WCSPHSolver : public ISPHSolver, private UnCopyable
{
public:
	WCSPHSolver()
	{}

	/**
	 * @brief Registers a fluid object with the solver.
	 * @param particle Pointer to the fluid to simulate.
	 */
	void add(WCSPHFluid* particle) { this->fluids.push_back(particle); }

	/** @brief Removes all registered fluids (does not delete them; caller owns them). */
	void clear() override { this->fluids.clear(); }

	/**
	 * @brief Advances all registered fluids by one time step.
	 *
	 * Precondition: the first registered fluid's effectLength must already
	 * be set (via this solver's setEffectLength() or the fluid's own,
	 * before add()/simulate()). It defaults to 0.f otherwise, in which case
	 * this is a no-op (docs/todo/PLAN_sph_scale_invariance.md Phase 5) rather
	 * than feeding a garbage/UB search radius to the neighbor search.
	 * @param dt      Time step (seconds), also stored via setTimeStep().
	 * @param maxIter Unused by CSPH (single-pass solver, no constraint
	 *                projection loop; kept for ISPHSolver signature parity).
	 */
	void simulate(const float dt, const int maxIter) override;

	/**
	 * @brief Sets the external body force applied to all particles (e.g., gravity).
	 * @param force External force vector.
	 */
	void setExternalForce(const Math::Vector3df& force) override { this->externalForce = force; }

	/**
	 * @brief Sets the simulation time step.
	 * @param timeStep Time step (seconds).
	 */
	void setTimeStep(const float timeStep) override { this->timeStep = timeStep; }
	void setMaxSubstep(const float timeStep) override { setTimeStep(timeStep); }

	/**
	 * @brief Sets the domain container's walls as an axis-aligned box, internally
	 * represented as 6 inward-facing PlaneBoundary instances.
	 * @param box Axis-aligned bounding box defining the interior region.
	 * @param timeStep Ignored -- see setBoundaryPlanes() below.
	 */
	void setBoundary(const Math::Box3df& box, const float timeStep) override {
		(void)timeStep;
		this->boundaryPlanes_ = makeBoxPlaneBoundaries(box);
	}

	/**
	 * @brief Sets the domain container's walls as an arbitrary set of planes.
	 * @param planes Plane boundaries; valid region is the intersection of all of them.
	 * @param timeStep Ignored. The penalty spring is -d/dt^2, calibrated to undo
	 *        a penetration in exactly one step, so the only time step it can
	 *        correctly use is the one being integrated -- which is the dt
	 *        simulate() was called with, not whatever was current when the
	 *        boundary happened to be registered. Keeping a second copy here
	 *        only let the two drift apart: FluidPoolStabilityTest.WCSPH_
	 *        PoolSettlesInBoxWithoutExploding registered its walls with 0.01
	 *        and then stepped at 0.005, quietly running with walls four times
	 *        softer than the formula intends. DFSPH had the same split and it
	 *        was a real bug there (see DFSPHSolver::addBoundaryPressure()).
	 */
	void setBoundaryPlanes(std::vector<PlaneBoundary> planes, const float timeStep) override {
		(void)timeStep;
		this->boundaryPlanes_ = std::move(planes);
	}

	/**
	 * @brief Sets additional solid-sphere container walls (e.g. the "water
	 * sphere" showcase's closed spherical container). See ISPHSolver's doc
	 * comment for why this is on top of, not instead of, the box/planes.
	 * DFSPH/PBSPH intentionally do not implement this -- see
	 * docs/todo/PLAN_sph_showcase_water_sphere.md section 8-B: those solvers
	 * must keep a boundary's density and its solver-specific alpha/constraint-
	 * gradient contribution in lockstep, and a sphere-only density term would
	 * violate that pairing (same reasoning as the DFSPH boundary-particle
	 * density/alpha pairing documented in ../CLAUDE.md).
	 * @param spheres Sphere boundaries; valid region is the union interior of all of them.
	 * @param timeStep Ignored -- see setBoundaryPlanes() above.
	 */
	void setBoundarySpheres(std::vector<SphereBoundary> spheres, const float timeStep) override {
		(void)timeStep;
		this->boundarySpheres_ = std::move(spheres);
	}

	/**
	 * @brief Sets additional finite-plate container walls (see ISPHSolver's
	 * doc comment). Held by value -- like boundarySpheres_, the Python side
	 * does not have to keep the list alive. Same reason as boundarySpheres_
	 * for why DFSPH/PBSPH do not implement this.
	 * @param plates Finite plate boundaries; each plate's valid region is its exterior.
	 * @param timeStep Ignored -- see setBoundaryPlanes() above.
	 */
	void setBoundaryPlates(std::vector<PlateBoundary> plates, const float timeStep) override {
		(void)timeStep;
		this->boundaryPlates_ = std::move(plates);
	}

	void setShapeBoundaries(std::vector<std::shared_ptr<IShapeBoundary>> boundaries,
	                        const float timeStep) override {
		(void)timeStep;
		this->boundaryShapes_ = std::move(boundaries);
	}
	void addShapeBoundary(std::shared_ptr<IShapeBoundary> boundary) override {
		if (boundary) boundaryShapes_.push_back(std::move(boundary));
	}
	void clearShapeBoundaries() override {
		boundaryPlanes_.clear(); boundarySpheres_.clear(); boundaryPlates_.clear(); boundaryShapes_.clear();
	}
	SPHSolveStats getLastSolveStats() const override { return lastSolveStats_; }

	/**
	 * @brief Sets how much of a particle's wall-normal velocity the domain
	 * walls absorb on contact (see ISPHSolver::setBoundaryDampingRatio()).
	 * Applies to boundaryPlanes_, boundarySpheres_ and boundaryPlates_.
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
	 * @brief Sets the kernel support radius. Kept for compatibility with the
	 * 4 call sites that historically set both the fluid's and the solver's
	 * effectLength to the same value; forwards to every registered fluid's
	 * own setEffectLength() (docs/todo/PLAN_physics_ownership_and_coupling_unification.md,
	 * Phase 4) rather than keeping a second, solver-local copy that simulate()
	 * used to build its own throwaway SPHKernel from. Fluids added *after*
	 * this call are unaffected -- call add() first, or call this again.
	 * @param length Effect length value.
	 */
	void setEffectLength(const float length) override;

	/**
	 * @brief Registers a rigid-body boundary particle set for Two-Way (Track B) coupling.
	 * @param r Non-owning pointer; must outlive the solver.
	 */
	void addRigidBoundaryParticles(RigidBoundaryParticles* r) override { rigidBoundaryParticles_.push_back(r); }

	/** @brief Removes all registered Two-Way rigid-body boundary particle sets. */
	void clearRigidBoundaryParticles() override { rigidBoundaryParticles_.clear(); }

	/**
	 * @brief Registers a SoftBody boundary particle set for Two-Way coupling.
	 * @param s Non-owning pointer; must outlive the solver.
	 */
	void addSoftBoundaryParticles(SoftBoundaryParticles* s) override { softBoundaryParticles_.push_back(s); }

	/** @brief Removes all registered SoftBody boundary particle sets. */
	void clearSoftBoundaryParticles() override { softBoundaryParticles_.clear(); }

	/**
	 * @brief WCSPH is force-based like DFSPH (as opposed to PBSPH's
	 * position-based projection), so it implements Two-Way (Track B) coupling
	 * the same way DFSPH does (docs/todo/PLAN_physics_ownership_and_coupling_unification.md,
	 * Phase 4).
	 */
	bool supportsTwoWayCoupling() const override { return true; }

	/**
	 * @brief Returns the list of registered fluid objects.
	 * @return Vector of fluid pointers.
	 */
	std::vector<WCSPHFluid*> getFluids() const { return this->fluids; }

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

	/**
	 * @brief Adds each boundary particle's psi-weighted kernel contribution to
	 * nearby fluid particles' density (Akinci et al. 2012, Eq. 2-3). Mirrors
	 * DFSPHSolver::addBoundaryParticleDensity() -- same force-based model,
	 * same formula, only the fluid particle type differs. Called automatically
	 * from simulate(), but exposed publicly like DFSPH's/PBSPH's equivalents
	 * since it takes its inputs as explicit parameters rather than
	 * solver-internal state.
	 * @param particles  Fluid particles to accumulate density onto.
	 * @param boundaries Boundary particle sets to couple against (rigid or SoftBody).
	 */
	void addBoundaryParticleDensity(std::vector<WCSPHParticle>& particles,
	                                 const std::vector<IBoundaryParticles*>& boundaries);

	/**
	 * @brief Computes the Two-Way (Track B) pressure-force coupling between
	 * fluid particles and boundary particles. Mirrors
	 * DFSPHSolver::addBoundaryParticlePressure(): pushes each fluid particle
	 * out (added to its force accumulator) and accumulates the
	 * equal-and-opposite reaction on the boundary particles' accumForce.
	 * @param particles  Fluid particles to couple.
	 * @param boundaries Boundary particle sets to couple against (rigid or SoftBody).
	 */
	void addBoundaryParticlePressure(std::vector<WCSPHParticle>& particles,
	                                  const std::vector<IBoundaryParticles*>& boundaries);

private:
	std::vector<WCSPHFluid*> fluids;
	Math::Vector3df externalForce;
	std::vector<PlaneBoundary> boundaryPlanes_;
	std::vector<SphereBoundary> boundarySpheres_;
	std::vector<PlateBoundary> boundaryPlates_;
	std::vector<std::shared_ptr<IShapeBoundary>> boundaryShapes_;
	// No boundaryTimeStep member on purpose: addBoundaryForce() uses timeStep,
	// the step simulate() is integrating. See setBoundaryPlanes().
	// 0 == the historical undamped penalty spring, so every existing scene
	// behaves bit-for-bit as before until it opts in.
	float boundaryDampingRatio_ = 0.0f;
	std::vector<RigidBoundary*> rigidBoundaries_;
	// Two separate lists, not one combined std::vector<IBoundaryParticles*> --
	// see DFSPHSolver.h's identical fields for why (independent
	// clearRigidBoundaryParticles()/clearSoftBoundaryParticles() semantics).
	std::vector<IBoundaryParticles*> rigidBoundaryParticles_;
	std::vector<IBoundaryParticles*> softBoundaryParticles_;
	float timeStep;
	SPHSolveStats lastSolveStats_;

	/**
	 * @brief Adds the density the domain walls (boundaryPlanes_ and
	 * boundarySpheres_) contribute to fluid particles within one kernel
	 * support of them.
	 *
	 * Without this, a particle resting on a wall only ever sums the fluid
	 * sitting above it, so its density saturates near half the bulk value and
	 * WCSPHParticle::getPressure()'s max(0, rho - rho0) clamp makes it exactly
	 * zero -- i.e. particles against a wall generate no repulsion at all and
	 * are free to collapse onto each other under load. Instead of sampling the
	 * wall with Akinci boundary *particles* (which would be O(N_fluid *
	 * N_boundary) here, since addBoundaryParticleDensity() scans the boundary
	 * set linearly for every fluid particle), the wall is treated as a solid
	 * half-space filled with fluid at rest density and the kernel is integrated
	 * over it analytically -- O(1) per particle per plane/sphere. A sphere
	 * wall reuses the same poly6HalfSpaceFraction(d, h) as a plane, approximating
	 * the curved wall by its tangent plane at distance d = R - |x - C|; this is
	 * exact for planes and O(h/R) accurate for spheres, which is why
	 * SphereBoundary's design requires R >= 10h (docs/todo/
	 * PLAN_sph_showcase_water_sphere.md section 8-B).
	 *
	 * Planes and spheres are combined into a single contribution and capped at
	 * restDensity once, at the end, rather than each capping its own subtotal
	 * -- summing two already-capped subtotals near a wall intersection could
	 * exceed rest density and hand the particle a spurious pressure spike.
	 */
	void addBoundaryDensity(std::vector<WCSPHParticle>& particles);

	void addBoundaryForce(std::vector<WCSPHParticle>& particles);

	void addRigidBoundaryPressure(std::vector<WCSPHParticle>& particles);

};

	}
}
