#pragma once

#include "CGLib/Math/Vector3d.h"

#include "SPHKernel.h"

#include <vector>

namespace Phantom {
	namespace Physics {
		class PBSPHFluid;

/**
 * @brief Structure-of-Arrays storage for PBSPH particle data.
 *
 * Owned by PBSPHFluid. PBSPHParticle is a non-owning (soa, index) view over
 * this storage -- see the class comment on PBSPHParticle below.
 */
struct PBSPHParticleSoA {
	std::vector<Math::Vector3df> predictPositions;
	std::vector<Math::Vector3df> forces;
	std::vector<Math::Vector3df> velocities;
	std::vector<Math::Vector3df> normals;
	std::vector<Math::Vector3df> positions;
	std::vector<float> radii;
	std::vector<float> densities;

	std::vector<Math::Vector3df> gradientSums;
	std::vector<float> gradientSqSums;
	std::vector<float> lambdas;

	std::vector<Math::Vector3df> dxs;
	std::vector<Math::Vector3df> xviscs;

	size_t size() const { return positions.size(); }
	bool empty() const { return positions.empty(); }

	void clear()
	{
		predictPositions.clear();
		forces.clear();
		velocities.clear();
		normals.clear();
		positions.clear();
		radii.clear();
		densities.clear();
		gradientSums.clear();
		gradientSqSums.clear();
		lambdas.clear();
		dxs.clear();
		xviscs.clear();
	}

	/** @brief Appends a new particle; density seeds from the owning fluid's rest density. */
	void push_back(const Math::Vector3df& position, const float radius, const float density)
	{
		predictPositions.push_back(position);
		forces.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
		velocities.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
		normals.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
		positions.push_back(position);
		radii.push_back(radius);
		densities.push_back(density);
		gradientSums.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
		gradientSqSums.push_back(0.0f);
		lambdas.push_back(0.0f);
		dxs.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
		xviscs.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
	}

	/** @brief Removes the particle at index via swap-with-last across all arrays (order not preserved). */
	void swapAndPop(const size_t index)
	{
		predictPositions[index] = predictPositions.back();
		predictPositions.pop_back();
		forces[index] = forces.back();
		forces.pop_back();
		velocities[index] = velocities.back();
		velocities.pop_back();
		normals[index] = normals.back();
		normals.pop_back();
		positions[index] = positions.back();
		positions.pop_back();
		radii[index] = radii.back();
		radii.pop_back();
		densities[index] = densities.back();
		densities.pop_back();
		gradientSums[index] = gradientSums.back();
		gradientSums.pop_back();
		gradientSqSums[index] = gradientSqSums.back();
		gradientSqSums.pop_back();
		lambdas[index] = lambdas.back();
		lambdas.pop_back();
		dxs[index] = dxs.back();
		dxs.pop_back();
		xviscs[index] = xviscs.back();
		xviscs.pop_back();
	}
};

/**
 * @brief A non-owning (PBSPHParticleSoA*, index) handle for a single particle
 * in a Position-Based SPH (PBSPH) simulation.
 *
 * All actual particle data lives in PBSPHParticleSoA (owned by PBSPHFluid);
 * this class is a lightweight, copyable view -- constructed fresh wherever
 * per-particle behavior is needed (PBSPHSolver's per-step working set),
 * rather than persisted. Implements the position-based constraint projection
 * steps.
 */
class PBSPHParticle
{
public:
	/**
	 * @brief Constructs a view over one particle's slot in a PBSPHParticleSoA.
	 * @param soa   SoA storage backing this particle; must outlive the view.
	 * @param index Index of this particle within soa.
	 * @param fluid Owning fluid object supplying shared parameters.
	 */
	PBSPHParticle(PBSPHParticleSoA& soa, const size_t index, PBSPHFluid* fluid) :
		soa_(&soa), index_(index), fluid_(fluid)
	{}

	/**
	 * @brief Returns the ratio of the current density to the rest density.
	 * @return Density ratio (current / rest).
	 */
	float getDensityRatio() const;

	/**
	 * @brief Returns the particle mass derived from its radius and rest density.
	 * @return Mass value.
	 */
	float getMass() const;

	/**
	 * @brief Returns the current particle volume.
	 * @return Volume value.
	 */
	float getVolume() const;

	/**
	 * @brief Returns the rest volume of the particle.
	 * @return Rest volume value.
	 */
	float getRestVolume() const;

	/**
	 * @brief Adds a force contribution to this particle.
	 * @param force Force vector to accumulate.
	 */
	void addForce(const Math::Vector3df& force) { forceRef() += force; }

	/**
	 * @brief Replaces the current force with the given value.
	 * @param force New force vector.
	 */
	void setForce(const Math::Vector3df& force) { forceRef() = force; }

	/**
	 * @brief Returns the accumulated force on this particle.
	 * @return Force vector.
	 */
	Math::Vector3df getForce() const { return soa_->forces[index_]; }

	/** @brief Resets density to the fluid rest density. */
	void setDefaultDensity();

	/**
	 * @brief Returns the current density of this particle.
	 * @return Density value.
	 */
	float getDensity() const { return soa_->densities[index_]; }

	/**
	 * @brief Adds a density contribution to this particle.
	 * @param density Density value to accumulate.
	 */
	void addDensity(const float density) { soa_->densities[index_] += density; }

	/** @brief Resets per-step state (forces, density) to initial values. */
	void init();

	/**
	 * @brief Returns the acceleration derived from force and density.
	 * @return Acceleration vector (force / density).
	 */
	Math::Vector3df getAccelaration() { return getForce() / getDensity(); }

	/**
	 * @brief Returns the current velocity of this particle.
	 * @return Velocity vector.
	 */
	Math::Vector3df getVelocity() const { return soa_->velocities[index_]; }

	/**
	 * @brief Sets the velocity of this particle.
	 * @param velocity New velocity vector.
	 */
	void setVelocity(const Math::Vector3df& velocity) { velocityRef() = velocity; }

	/**
	 * @brief Adds a velocity increment to this particle.
	 * @param velocity Velocity increment to accumulate.
	 */
	void addVelocity(const Math::Vector3df& velocity) { velocityRef() += velocity; }

	/**
	 * @brief Advances the particle position by one time step using its velocity.
	 * @param timeStep Simulation time step (seconds).
	 */
	void forwardTime(const float timeStep);

	/**
	 * @brief Applies an external body force (e.g., gravity) to this particle.
	 * @param force External force vector.
	 */
	void addExternalForce(const Math::Vector3df& force);

	/** @brief Adds the particle's self-density contribution (W(0) term). */
	void addSelfDensity();

	/**
	 * @brief Accumulates density contribution from a neighboring particle.
	 * @param rhs Neighboring PBSPH particle.
	 */
	void addDensity(const PBSPHParticle& rhs);

	/**
	 * @brief Accumulates density from a neighbor given distance and mass.
	 * @param distance Distance to the neighbor.
	 * @param mass     Mass of the neighbor.
	 */
	void addDensity(const float distance, const float mass);

	/**
	 * @brief Computes the predicted position for this time step.
	 * @param dt Time step (seconds).
	 */
	void predictPosition_(const float dt);

	/** @brief Commits the predicted position as the current position. */
	void updatePredictPosition();

	/**
	 * @brief Updates the velocity from the position change over a time step.
	 * @param dt Time step (seconds).
	 */
	void updateVelocity(const float dt);

	/** @brief Applies the accumulated position corrections to the position. */
	void updatePosition();

	/**
	 * @brief Accumulates a position correction vector.
	 * @param pc Position correction to add.
	 */
	void addPositionCorrection(const Math::Vector3df& pc);

	/**
	 * @brief Returns the SPH kernel support radius of this particle.
	 * @return Effect length value.
	 */
	float getEffectLength() const;

	/**
	 * @brief Returns the predicted position for the current iteration.
	 * @return Predicted position vector.
	 */
	Math::Vector3df getPredictPosition() const { return soa_->predictPositions[index_]; }

	/**
	 * @brief Directly overwrites the predicted (in-flight) position -- used by
	 * PBSPHSolver::clampToBoundary() to hard-clamp a particle back inside the
	 * domain box after the density/pressure correction has been applied, as a
	 * safety net against tunneling through a wall in one step (e.g. a
	 * high-speed impact) that the softer boundary-particle density response
	 * alone would not catch in time.
	 * @param p New predicted position.
	 */
	void setPredictPosition(const Math::Vector3df& p) { predictPositionRef() = p; }

	/**
	 * @brief Returns the current world-space position.
	 * @return Position vector.
	 */
	Math::Vector3df getPosition() const { return soa_->positions[index_]; }

	/**
	 * @brief Accumulates pressure contribution from a neighboring particle.
	 *
	 * Uses the normalized PBF Lagrange multiplier (see calculateLambda()),
	 * not the raw density constraint, so the resulting position correction
	 * stays well-scaled regardless of the kernel's effect length (the raw
	 * constraint * kernel-gradient product used before this normalization
	 * would blow up for small effect lengths, since the Poly6 gradient
	 * constant scales as 1/effectLength^9).
	 * @param rhs Neighboring PBSPH particle.
	 */
	void calculatePressure(const PBSPHParticle& rhs);

	/** @brief Zeroes the constraint-gradient accumulators used by calculateLambda(). Call once per iteration before accumulateConstraintGradient(). */
	void resetConstraintGradient()
	{
		gradientSumRef() = Math::Vector3df(0.f, 0.f, 0.f);
		soa_->gradientSqSums[index_] = 0.f;
	}

	/**
	 * @brief Accumulates this neighbor's contribution to the density
	 * constraint's gradient (Macklin & Muller 2013, PBF Eq. 8-9).
	 * @param rhs Neighboring PBSPH particle.
	 */
	void accumulateConstraintGradient(const PBSPHParticle& rhs);

	/**
	 * @brief Accumulates an already-weighted constraint-gradient contribution
	 * (same accumulators as accumulateConstraintGradient(), for callers that
	 * are not themselves a PBSPHParticle -- e.g. a rigid-body boundary sample).
	 * @param grad Gradient contribution to add.
	 */
	void addConstraintGradient(const Math::Vector3df& grad)
	{
		gradientSumRef() += grad;
		soa_->gradientSqSums[index_] += Math::getLengthSquared(grad);
	}

	/**
	 * @brief Computes this particle's PBF Lagrange multiplier from its
	 * density constraint and the gradient sums accumulated via
	 * accumulateConstraintGradient() (Macklin & Muller 2013, Eq. 11).
	 * Call once per iteration, after all neighbor pairs have called
	 * accumulateConstraintGradient() and before calculatePressure().
	 */
	void calculateLambda();

	/**
	 * @brief Returns the PBF Lagrange multiplier computed by calculateLambda().
	 * @return Lambda value.
	 */
	float getLambda() const { return soa_->lambdas[index_]; }

	/**
	 * @brief Overrides the Lagrange multiplier directly, bypassing
	 * calculateLambda(). Mainly for unit tests that exercise a single
	 * particle in isolation, without a full neighbor pass to derive it from.
	 * @param l New lambda value.
	 */
	void setLambda(const float l) { soa_->lambdas[index_] = l; }

	/**
	 * @brief Accumulates viscosity force from a neighboring particle.
	 * @param rhs Neighboring PBSPH particle.
	 */
	void calculateViscosity(const PBSPHParticle& rhs);

	/**
	 * @brief Returns the incompressibility constraint value for this particle.
	 * @return Constraint value (density ratio - 1).
	 */
	float getConstraint() const;

	/** @brief Returns the position correction delta accumulated during constraint solving. */
	Math::Vector3df getDx() const { return soa_->dxs[index_]; }

	/** @brief Overwrites the position correction delta (e.g. to reset it to zero at the start of an iteration). */
	void setDx(const Math::Vector3df& v) { dxRef() = v; }

	/** @brief Returns the viscosity correction delta accumulated during constraint solving. */
	Math::Vector3df getXvisc() const { return soa_->xviscs[index_]; }

	/** @brief Overwrites the viscosity correction delta (e.g. to reset it to zero at the start of a step). */
	void setXvisc(const Math::Vector3df& v) { xviscRef() = v; }

	/**
	 * @brief Overrides the density with the given value.
	 * @param d New density value.
	 */
	void setDensity(const float d) { soa_->densities[index_] = d; }

	/**
	 * @brief Returns the particle radius.
	 * @return Radius value (double precision).
	 */
	double getRadius() const { return soa_->radii[index_]; }

	/**
	 * @brief Returns the owning fluid object.
	 * @return Pointer to the parent PBSPHFluid.
	 */
	PBSPHFluid* getFluid() const { return fluid_; }

private:
	Math::Vector3df& predictPositionRef() { return soa_->predictPositions[index_]; }
	Math::Vector3df& forceRef()           { return soa_->forces[index_]; }
	Math::Vector3df& velocityRef()        { return soa_->velocities[index_]; }
	Math::Vector3df& positionRef()        { return soa_->positions[index_]; }
	Math::Vector3df& gradientSumRef()     { return soa_->gradientSums[index_]; }
	Math::Vector3df& dxRef()              { return soa_->dxs[index_]; }
	Math::Vector3df& xviscRef()           { return soa_->xviscs[index_]; }

	PBSPHParticleSoA* soa_;
	size_t index_;
	PBSPHFluid* fluid_;

	SPHKernel* getKernel();
};

	}
}
