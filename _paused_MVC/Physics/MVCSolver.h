#pragma once

#include "../../CGLib/Math/Vector3d.h"
#include "MVCParticle.h"
#include "RigidBoundary.h"
#include "ISPHSolver.h"
#include <vector>

namespace Phantom {
	namespace Physics {
		class MVCFluid;

/**
 * @brief Solver for MVC (Moving Voronoi Cell) fluid simulation.
 *
 * Advances registered MVCFluid objects through time by applying
 * external forces and integrating particle positions.
 */
class MVCSolver : public ISPHSolver
{
public:
	/**
	 * @brief Advances all registered fluids by one time step.
	 * @param dt      Time step (seconds).
	 * @param maxIter Volume-constraint projection iteration count.
	 */
	void simulate(const float dt, const int maxIter) override;

	/**
	 * @brief Registers a fluid object with the solver.
	 * @param fluid Pointer to the fluid to simulate.
	 */
	void add(MVCFluid* fluid) {
		fluids.push_back(fluid);
	}

	/** @brief Removes all registered fluids (does not delete them; caller owns them). */
	void clear() override { fluids.clear(); }

	/**
	 * @brief Sets the external body force applied to all particles (e.g., gravity).
	 * @param force External force vector.
	 */
	void setExternalForce(const Math::Vector3df& force) override {
		this->externalForce = force;
	}

	/**
	 * @brief Registers a rigid-body boundary for One-Way SDF penalty coupling.
	 * @param b Non-owning pointer to the boundary; must outlive the solver.
	 */
	void addRigidBoundary(RigidBoundary* b) override { rigidBoundaries_.push_back(b); }

	/** @brief Removes all registered rigid-body boundaries. */
	void clearRigidBoundaries() override { rigidBoundaries_.clear(); }

	/**
	 * @brief Returns the total particle count across all registered fluids.
	 * MVC has no SPHKernel/rest-density concept (getKernel()/getRestDensity()
	 * inherit ISPHSolver's no-op defaults), but its fluids do hold particles,
	 * so particle count/positions are implemented like the other solvers.
	 */
	int getParticleCount() const override;

	/** @brief Returns the world-space positions of all particles across all registered fluids. */
	std::vector<Math::Vector3df> getParticlePositions() const override;

private:
	std::vector<MVCFluid*> fluids;
	Math::Vector3df externalForce{ 0.0f, -9.8f, 0.0f };
	std::vector<RigidBoundary*> rigidBoundaries_;
};
	}
}
