#pragma once

#include "SPHKernel.h"
#include "PlaneBoundary.h"
#include "CGLib/Util/UnCopyable.h"
#include "CGLib/Math/Box3d.h"

#include <vector>

namespace Phantom {
	namespace Physics {
		class FlameFluid;
		class FlameParticle;

/**
 * @brief Solver for the Flame (reacting hot gas) SPH simulation.
 *
 * Same skeleton as WCSPHSolver::simulate() (kernel -> neighbor search via
 * IndexedSortBasedSearcher -> density pass -> force pass -> integrate), with
 * two extra neighbor passes for vorticity confinement, a per-particle
 * Boussinesq buoyancy term, curl-noise velocity decoration, and emitter/
 * lifetime bookkeeping at the end of each step.
 *
 * Deliberately does NOT implement ISPHSolver: Rigid/SoftBody coupling is out
 * of scope for this solver (see internal design notes), so simulate()
 * takes only dt (no maxIter, no boundary-particle registration API).
 */
class FlameSolver : private UnCopyable
{
public:
	FlameSolver() = default;

	/** @brief Registers a fluid object with the solver. */
	void add(FlameFluid* fluid) { fluids.push_back(fluid); }

	/**
	 * @brief Advances all registered fluids by one time step: SPH density/pressure/
	 * viscosity, vorticity confinement, Boussinesq buoyancy, curl-noise decoration,
	 * integration, then emitter spawning and dead-particle removal.
	 * @param dt Time step (seconds).
	 */
	void simulate(const float dt);

	/**
	 * @brief Sets the gravity vector used by the Boussinesq buoyancy term
	 * (FlameParticle::applyBuoyancy()); this solver does not apply plain
	 * gravity separately (see applyBuoyancy() doc comment).
	 */
	void setGravity(const Math::Vector3df& g) { gravity = g; }
	Math::Vector3df getGravity() const { return gravity; }

	/** @brief Sets the kernel support radius used during neighbor searches. */
	void setEffectLength(const float length) { effectLength = length; }
	float getEffectLength() const { return effectLength; }

	/** @brief Sets the domain container's walls as an axis-aligned box (6 inward planes). */
	void setBoundary(const Math::Box3df& box, const float timeStep)
	{
		(void)timeStep;   // ignored -- see setBoundaryPlanes() below
		boundaryPlanes_ = makeBoxPlaneBoundaries(box);
	}

	/**
	 * @brief Sets the domain container's walls as an arbitrary set of planes.
	 * @param timeStep Ignored. The penalty spring is -d/dt^2, calibrated to undo
	 *        a penetration in exactly one step, so the only time step it can
	 *        correctly use is the one simulate() is integrating -- which is why
	 *        addBoundaryForce() takes that dt rather than a copy stored here.
	 *        Mirrors WCSPHSolver/DFSPHSolver.
	 */
	void setBoundaryPlanes(std::vector<PlaneBoundary> planes, const float timeStep)
	{
		(void)timeStep;
		boundaryPlanes_ = std::move(planes);
	}

	/** @brief Returns the list of registered fluid objects. */
	std::vector<FlameFluid*> getFluids() const { return fluids; }

private:
	float effectLength = 0.15f;
	Math::Vector3df gravity{ 0.0f, -9.8f, 0.0f };
	std::vector<FlameFluid*> fluids;
	std::vector<PlaneBoundary> boundaryPlanes_;

	void addBoundaryForce(std::vector<FlameParticle>& particles, const float dt);
};

	}
}
