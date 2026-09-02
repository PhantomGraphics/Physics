#pragma once

#include <vector>
#include <memory>

#include "CGLib/Math/Vector3d.h"
#include "CGLib/Math/Box3d.h"
#include "PlaneBoundary.h"
#include "SphereBoundary.h"
#include "PlateBoundary.h"
#include "IShapeBoundary.h"

namespace Phantom {
	namespace Physics {

class RigidBoundary;
class RigidBoundaryParticles;
class SoftBoundaryParticles;
class SPHKernel;

struct SPHSolveStats {
	/** Number of integration substeps completed by the last simulate() call. */
	int substeps = 0;
	/** Total DFSPH density-projection iterations; zero for non-iterative solvers. */
	int densityIterations = 0;
	/** Total DFSPH divergence-projection iterations; zero where unsupported. */
	int divergenceIterations = 0;
	/** Simulation time actually advanced by the last simulate() call. */
	float advancedTime = 0.0f;
	/** False when an iterative solve exhausted maxIter before meeting its tolerance. */
	bool converged = true;
	/** False when simulate() rejected invalid or incompatible solver/fluid settings. */
	bool validConfiguration = true;
};

/**
 * @brief Common interface implemented by the CPU SPH fluid solvers
 * (DFSPHSolver, PBSPHSolver, CSPHSolver). (MVCSolver formerly implemented
 * this interface too; its development is paused and the sources have been
 * moved out of the build -- see Physics/_paused_MVC/.)
 *
 * Before this interface existed, each solver had its own simulate() arity
 * (DFSPHSolver::simulate(searchRadius, maxIter), PBSPHSolver::simulate(dt,
 * maxIter), CSPHSolver::simulate()), which forced callers
 * that support more than one fluid type (PhysicsSolver, PhysicsView's
 * FluidWorld, FluidStudio's FluidWorld) to dispatch every operation through a switch on
 * the active fluid type. ISPHSolver unifies simulate()'s signature and the
 * RigidBoundary(Particles) coupling calls so those callers can hold a single
 * `std::unique_ptr<ISPHSolver>` instead.
 *
 * Two-Way (Track B, RigidBoundaryParticles) coupling is only implemented by
 * DFSPH/PBSPH, so those two methods default to a no-op rather than being
 * pure virtual -- CSPH simply inherits the no-op.
 */
class ISPHSolver
{
public:
	virtual ~ISPHSolver() = default;

	/**
	 * @brief Advances all fluids registered with this solver by one frame.
	 * @param dt      Requested frame duration (seconds). DFSPH may split it
	 *                 into adaptive substeps; all solvers advance this duration.
	 * @param maxIter Constraint-projection iteration count. Ignored by
	 *                 solvers without an iterative projection step (CSPH).
	 */
	virtual void simulate(const float dt, const int maxIter) = 0;

	/**
	 * @brief Sets the external body force applied to all particles (e.g., gravity).
	 * @param force External force vector.
	 */
	virtual void setExternalForce(const Math::Vector3df& force) = 0;

	/**
	 * @brief Sets the solver's time-step limit/state. For DFSPH this is the
	 * maximum adaptive substep; simulate()'s dt remains the frame duration.
	 * @param dt Time step (seconds).
	 */
	virtual void setTimeStep(const float dt) { (void)dt; }
	/** Explicit name for DFSPH's adaptive-substep ceiling. */
	virtual void setMaxSubstep(const float dt) { setTimeStep(dt); }

	/** Sets the kernel support radius on fluids already registered with the solver. */
	virtual void setEffectLength(const float length) { (void)length; }

	/**
	 * @brief Sets the domain container's walls as an arbitrary set of planes.
	 * No-op where unsupported.
	 * @param planes Plane boundaries; valid region is the intersection of all of them.
	 * @param timeStep Time step used to scale the repulsion force.
	 */
	virtual void setBoundaryPlanes(std::vector<PlaneBoundary> planes, const float timeStep) { (void)planes; (void)timeStep; }

	/**
	 * @brief Sets the domain container's walls as an axis-aligned box, internally
	 * represented as 6 inward-facing PlaneBoundary instances. No-op where
	 * unsupported (see setBoundaryPlanes()).
	 * @param box Axis-aligned bounding box defining the interior region.
	 * @param timeStep Time step used to scale the repulsion force.
	 */
	virtual void setBoundary(const Math::Box3df& box, const float timeStep) { (void)box; (void)timeStep; }

	/**
	 * @brief Sets additional solid-sphere container walls (e.g. the
	 * "water sphere" showcase's closed spherical container). These are on top
	 * of, not instead of, the box/plane boundary from setBoundary()/
	 * setBoundaryPlanes() -- a fluid solver still requires a domain box.
	 * No-op where unsupported. WCSPH and DFSPH implement this; DFSPH adds the
	 * matching alpha-gradient term whenever it adds boundary density.
	 * @param spheres Sphere boundaries; valid region is the union interior of all of them.
	 * @param timeStep Time step used to scale the repulsion force.
	 */
	virtual void setBoundarySpheres(std::vector<SphereBoundary> spheres, const float timeStep) { (void)spheres; (void)timeStep; }

	/**
	 * @brief Sets additional finite-plate container walls (the "waterfall"
	 * showcase's channel floor, weir, cliff face and rock shelves). Like
	 * setBoundarySpheres(), these are on top of -- not instead of -- the
	 * box/plane boundary. WCSPH and DFSPH implement this; PBSPH retains the
	 * default no-op until it has a matching constraint-gradient formulation.
	 * @param plates   Finite plate boundaries; each plate's valid region is its exterior.
	 * @param timeStep Ignored -- see setBoundaryPlanes().
	 */
	virtual void setBoundaryPlates(std::vector<PlateBoundary> plates, const float timeStep) { (void)plates; (void)timeStep; }

	/** Sets arbitrary analytic boundaries through their common interface. */
	virtual void setShapeBoundaries(std::vector<std::shared_ptr<IShapeBoundary>> boundaries,
	                                const float timeStep) {
		(void)boundaries; (void)timeStep;
	}

	/** Adds one analytic boundary without replacing those already registered. */
	virtual void addShapeBoundary(std::shared_ptr<IShapeBoundary> boundary) { (void)boundary; }
	/** Clears every analytic boundary registered through typed or generic APIs. */
	virtual void clearShapeBoundaries() {}

	/** Returns diagnostics for the most recent simulate() call. */
	virtual SPHSolveStats getLastSolveStats() const { return {}; }

	/**
	 * @brief Sets how much of a particle's wall-normal velocity the domain
	 * walls (planes and spheres) absorb on contact.
	 *
	 * 0 (the default everywhere) is the historical pure-spring penalty, whose
	 * restitution is ~1: the wall gives back every bit of the impact. That is
	 * harmless under a settled pool but turns a jet landing in an *empty*
	 * container into upward spray -- see boundaryPenaltyAcceleration() and
	 * docs/issue/water_sphere_showcase_emitter_instability.md section 3.
	 * Values are clamped to [0, 0.5] (clampBoundaryDampingRatio()).
	 *
	 * No-op where unsupported: PBSPHSolver has no use for it (it hard-clamps
	 * predicted positions onto the wall rather than integrating a penalty
	 * spring, so it never stores rebound energy in the first place).
	 * @param ratio Damping ratio in [0, 0.5].
	 */
	virtual void setBoundaryDampingRatio(const float ratio) { (void)ratio; }

	/** @brief Returns the wall damping ratio set by setBoundaryDampingRatio(). */
	virtual float getBoundaryDampingRatio() const { return 0.f; }

	/** @brief Removes all registered fluids (does not delete them; caller owns them). */
	virtual void clear() = 0;

	/**
	 * @brief Returns the SPH kernel shared by this solver's registered fluids.
	 * @return Pointer to a registered fluid's SPHKernel, or nullptr where
	 *         unsupported or no fluid is registered.
	 */
	virtual SPHKernel* getKernel() { return nullptr; }

	/**
	 * @brief Returns the rest density of this solver's registered fluids.
	 * @return Rest density, or 0.f where unsupported or no fluid is registered.
	 */
	virtual float getRestDensity() const { return 0.f; }

	/** @brief Returns the total particle count across all registered fluids. */
	virtual int getParticleCount() const = 0;

	/** @brief Returns the world-space positions of all particles across all registered fluids. */
	virtual std::vector<Math::Vector3df> getParticlePositions() const = 0;

	/** @brief Returns the world-space velocities in the same order as positions. */
	virtual std::vector<Math::Vector3df> getParticleVelocities() const = 0;

	/** @brief Returns particle densities in the same order as positions. */
	virtual std::vector<float> getParticleDensities() const = 0;

	/**
	 * @brief Registers a rigid-body boundary for One-Way SDF penalty coupling.
	 * @param b Non-owning pointer to the boundary; must outlive the solver.
	 */
	virtual void addRigidBoundary(RigidBoundary* b) = 0;

	/** @brief Removes all registered rigid-body boundaries. */
	virtual void clearRigidBoundaries() = 0;

	/**
	 * @brief Registers a rigid-body boundary particle set for Two-Way (Track B)
	 * coupling. No-op where unsupported (CSPH).
	 * @param r Non-owning pointer; must outlive the solver.
	 */
	virtual void addRigidBoundaryParticles(RigidBoundaryParticles* r) { (void)r; }

	/** @brief Removes all registered Two-Way rigid-body boundary particle sets. No-op where unsupported. */
	virtual void clearRigidBoundaryParticles() {}

	/** @brief True only for solvers that implement Two-Way (Track B) coupling. */
	virtual bool supportsTwoWayCoupling() const { return false; }

	/**
	 * @brief Registers a SoftBody boundary particle set for Two-Way
	 * SoftBody-fluid coupling (mirrors the rigid-body Track B mechanism --
	 * SoftBody has no analytic collision shape, so there is no One-Way SDF
	 * equivalent; boundary particles are the only coupling path). No-op
	 * where unsupported (CSPH).
	 * @param s Non-owning pointer; must outlive the solver.
	 */
	virtual void addSoftBoundaryParticles(SoftBoundaryParticles* s) { (void)s; }

	/** @brief Removes all registered SoftBody boundary particle sets. No-op where unsupported. */
	virtual void clearSoftBoundaryParticles() {}
};

	}
}
