#pragma once

#include <vector>

#include "CGLib/Math/Vector3d.h"

namespace Phantom {
	namespace Physics {
		class WCSPHFluid;
		class SPHKernel;

/**
 * @brief Structure-of-Arrays storage for WCSPH particle data.
 *
 * Owned by WCSPHFluid. WCSPHParticle is a non-owning (soa, index) view over
 * this storage -- see the class comment on WCSPHParticle below.
 */
struct WCSPHParticleSoA {
	std::vector<Math::Vector3df> positions;
	std::vector<Math::Vector3df> velocities;
	std::vector<Math::Vector3df> forces;
	std::vector<Math::Vector3df> normals;
	std::vector<float> densities;
	std::vector<float> radii;

	size_t size() const { return positions.size(); }
	bool empty() const { return positions.empty(); }

	void clear()
	{
		positions.clear();
		velocities.clear();
		forces.clear();
		normals.clear();
		densities.clear();
		radii.clear();
	}

	void push_back(const Math::Vector3df& position, const float radius, const float initialDensity)
	{
		positions.push_back(position);
		velocities.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
		forces.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
		normals.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
		densities.push_back(initialDensity);
		radii.push_back(radius);
	}

	/** @brief Removes the particle at index via swap-with-last across all arrays (order not preserved). */
	void swapAndPop(const size_t index)
	{
		positions[index] = positions.back();
		positions.pop_back();
		velocities[index] = velocities.back();
		velocities.pop_back();
		forces[index] = forces.back();
		forces.pop_back();
		normals[index] = normals.back();
		normals.pop_back();
		densities[index] = densities.back();
		densities.pop_back();
		radii[index] = radii.back();
		radii.pop_back();
	}
};

/**
 * @brief A non-owning (WCSPHParticleSoA*, index) handle for a single particle
 * in a Weakly Compressible SPH (WCSPH) simulation.
 *
 * All actual particle data lives in WCSPHParticleSoA (owned by WCSPHFluid);
 * this class is a lightweight, copyable view -- constructed fresh wherever
 * per-particle behavior is needed (WCSPHSolver's per-step working set,
 * pybind11 get_particles() results) rather than persisted. Implements
 * standard SPH force accumulation including pressure, viscosity, surface
 * tension, and normal computation.
 */
class WCSPHParticle
{
public:
	/**
	 * @brief Constructs a view over one particle's slot in a WCSPHParticleSoA.
	 * @param soa   SoA storage backing this particle; must outlive the view.
	 * @param index Index of this particle within soa.
	 * @param fluid Owning fluid object supplying shared parameters.
	 */
	WCSPHParticle(WCSPHParticleSoA& soa, const size_t index, WCSPHFluid* fluid) :
		soa_(&soa), index_(index), fluid_(fluid), kernel_(nullptr)
	{}

	/**
	 * @brief Returns the ratio of the current density to the rest density.
	 * @return Density ratio (current / rest).
	 */
	float getDensityRatio() const;

	/**
	 * @brief Returns the pressure computed from the current density ratio.
	 * @return Pressure value.
	 */
	float getPressure() const;

	/**
	 * @brief Returns the particle mass derived from radius and rest density.
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

	/** @brief Resets per-step state (forces, density, normals) to initial values. */
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

	/**
	 * @brief Accumulates the surface normal contribution from a neighbor.
	 * @param rhs Neighboring WCSPH particle.
	 */
	void solveNormal(const WCSPHParticle& rhs);

	/**
	 * @brief Accumulates the surface tension force from a neighbor.
	 * @param rhs Neighboring WCSPH particle.
	 */
	void solveSurfaceTension(const WCSPHParticle& rhs);

	/**
	 * @brief Accumulates the pressure force from a neighbor.
	 * @param rhs Neighboring WCSPH particle.
	 */
	void solvePressureForce(const WCSPHParticle& rhs);

	/**
	 * @brief Accumulates the viscosity force from a neighbor.
	 * @param rhs Neighboring WCSPH particle.
	 */
	void solveViscosityForce(const WCSPHParticle& rhs);

	/** @brief Adds the particle's self-density contribution (W(0) term). */
	void addSelfDensity();

	/**
	 * @brief Accumulates density contribution from a neighboring particle.
	 * @param rhs Neighboring WCSPH particle.
	 */
	void addDensity(const WCSPHParticle& rhs);

	/**
	 * @brief Returns the surface normal at this particle.
	 * @return Normal vector.
	 */
	Math::Vector3df getNormal() const { return soa_->normals[index_]; }

	/**
	 * @brief Returns the current world-space position.
	 * @return Position vector.
	 */
	Math::Vector3df getPosition() const { return soa_->positions[index_]; }

	/**
	 * @brief Returns the diameter of this particle (2 * radius).
	 * @return Diameter value.
	 */
	float getDiameter() const { return soa_->radii[index_] * 2.0f; }

	/**
	 * @brief Translates the particle by the given displacement.
	 * @param v Displacement vector.
	 */
	void move(const Math::Vector3df& v);

	/**
	 * @brief Sets the SPH kernel used by this particle.
	 * @param kernel Pointer to the kernel object.
	 */
	void setKernel(SPHKernel* kernel) { this->kernel_ = kernel; }

	/**
	 * @brief Returns the SPH kernel used by this particle.
	 * @return Pointer to the kernel object.
	 */
	SPHKernel* getKernel() { return kernel_; }

	/**
	 * @brief Returns the owning fluid object.
	 * @return Pointer to the parent WCSPHFluid.
	 */
	WCSPHFluid* getFluid() const { return fluid_; }

private:
	Math::Vector3df& positionRef() { return soa_->positions[index_]; }
	Math::Vector3df& forceRef()    { return soa_->forces[index_]; }
	Math::Vector3df& velocityRef() { return soa_->velocities[index_]; }
	Math::Vector3df& normalRef()   { return soa_->normals[index_]; }
	float&           densityRef()  { return soa_->densities[index_]; }

	/**
	 * @brief Returns the unit surface normal, or a zero vector if the raw
	 * normal's magnitude -- nondimensionalized by the kernel's effect length,
	 * so the same threshold applies regardless of scene scale (see
	 * docs/todo/PLAN_sph_scale_invariance.md) -- is below a noise threshold,
	 * i.e. this particle is judged interior rather than on the free surface.
	 * @return Unit surface normal, or (0,0,0) for interior particles.
	 */
	Math::Vector3df surfaceNormalHat() const;

	WCSPHParticleSoA* soa_;
	size_t index_;
	WCSPHFluid* fluid_;

protected:
	SPHKernel* kernel_;
};

	}
}
