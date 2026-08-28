#pragma once

#include "CGLib/Math/Vector3d.h"

#include <vector>

namespace Phantom {
	namespace Physics {
		class FlameFluid;
		class SPHKernel;

/**
 * @brief Structure-of-Arrays storage for Flame (reacting hot gas) particle data.
 *
 * Owned by FlameFluid. FlameParticle is a non-owning (soa, index) view over
 * this storage -- see the class comment on FlameParticle below.
 */
struct FlameParticleSoA {
	std::vector<Math::Vector3df> positions;
	std::vector<Math::Vector3df> velocities;
	std::vector<Math::Vector3df> forces;
	std::vector<float> radii;
	std::vector<float> densities;

	std::vector<float> temperatures;
	std::vector<float> fuels;
	std::vector<float> soots;
	std::vector<float> ages;
	std::vector<bool> airs;

	std::vector<Math::Vector3df> vorticities;
	std::vector<Math::Vector3df> vorticityGradAccums;

	size_t size() const { return positions.size(); }
	bool empty() const { return positions.empty(); }

	void clear()
	{
		positions.clear();
		velocities.clear();
		forces.clear();
		radii.clear();
		densities.clear();
		temperatures.clear();
		fuels.clear();
		soots.clear();
		ages.clear();
		airs.clear();
		vorticities.clear();
		vorticityGradAccums.clear();
	}

	/** @brief Appends a new (initially unignited) particle; density/temperature seed from the owning fluid. */
	void push_back(const Math::Vector3df& position, const float radius, const float density, const float temperature)
	{
		positions.push_back(position);
		velocities.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
		forces.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
		radii.push_back(radius);
		densities.push_back(density);
		temperatures.push_back(temperature);
		fuels.push_back(0.0f);
		soots.push_back(0.0f);
		ages.push_back(0.0f);
		airs.push_back(false);
		vorticities.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
		vorticityGradAccums.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
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
		radii[index] = radii.back();
		radii.pop_back();
		densities[index] = densities.back();
		densities.pop_back();
		temperatures[index] = temperatures.back();
		temperatures.pop_back();
		fuels[index] = fuels.back();
		fuels.pop_back();
		soots[index] = soots.back();
		soots.pop_back();
		ages[index] = ages.back();
		ages.pop_back();
		airs[index] = airs.back();
		airs.pop_back();
		vorticities[index] = vorticities.back();
		vorticities.pop_back();
		vorticityGradAccums[index] = vorticityGradAccums.back();
		vorticityGradAccums.pop_back();
	}
};

/**
 * @brief A non-owning (FlameParticleSoA*, index) handle for a single particle
 * in the Flame (reacting hot gas) SPH simulation.
 *
 * All actual particle data lives in FlameParticleSoA (owned by FlameFluid);
 * this class is a lightweight, copyable view -- constructed fresh wherever
 * per-particle behavior is needed (FlameSolver's per-step working set),
 * rather than persisted. Mirrors WCSPHParticle's SPH state (position/
 * velocity/force/density) and adds combustion state (temperature/fuel/soot/
 * age) plus vorticity-confinement bookkeeping. Deliberately independent of
 * WCSPHParticle: no surface tension or normal (flames need not cohere, see
 * idea doc section 2), and no Rigid/SoftBody boundary hooks (out of scope,
 * see docs/todo/sph_flame_plan.md).
 */
class FlameParticle
{
public:
	/**
	 * @brief Constructs a view over one particle's slot in a FlameParticleSoA.
	 * @param soa   SoA storage backing this particle; must outlive the view.
	 * @param index Index of this particle within soa.
	 * @param fluid Owning fluid object supplying shared parameters.
	 */
	FlameParticle(FlameParticleSoA& soa, const size_t index, FlameFluid* fluid) :
		soa_(&soa), index_(index), fluid_(fluid)
	{}

	/** @brief Returns the ratio of the current density to the rest density. */
	float getDensityRatio() const;

	/** @brief Returns the pressure computed from the current density ratio. */
	float getPressure() const;

	/** @brief Returns the particle mass derived from radius and rest density. */
	float getMass() const;

	/** @brief Returns the current particle volume. */
	float getVolume() const;

	/** @brief Returns the rest volume of the particle. */
	float getRestVolume() const;

	/** @brief Adds a force contribution to this particle. */
	void addForce(const Math::Vector3df& f) { forceRef() += f; }

	/** @brief Replaces the current force with the given value. */
	void setForce(const Math::Vector3df& f) { forceRef() = f; }

	/** @brief Returns the accumulated force on this particle. */
	Math::Vector3df getForce() const { return soa_->forces[index_]; }

	/** @brief Returns the current density of this particle. */
	float getDensity() const { return soa_->densities[index_]; }

	/** @brief Adds a density contribution to this particle. */
	void addDensity(const float d) { soa_->densities[index_] += d; }

	/** @brief Accumulates density contribution from a neighboring particle. */
	void addDensity(const FlameParticle& rhs);

	/** @brief Adds the particle's self-density contribution (W(0) term). */
	void addSelfDensity();

	/** @brief Resets per-step state (force, density, vorticity accumulators). */
	void init();

	/** @brief Returns the current velocity of this particle. */
	Math::Vector3df getVelocity() const { return soa_->velocities[index_]; }

	/** @brief Sets the velocity of this particle. */
	void setVelocity(const Math::Vector3df& v) { velocityRef() = v; }

	/** @brief Adds a velocity increment to this particle (used for curl-noise decoration). */
	void addVelocity(const Math::Vector3df& v) { velocityRef() += v; }

	/** @brief Advances velocity (via force/density) and position by one time step. */
	void forwardTime(const float timeStep);

	/** @brief Applies an external force (already scaled by density, see WCSPH convention). */
	void addExternalForce(const Math::Vector3df& force);

	/** @brief Accumulates the pressure force from a neighbor. */
	void solvePressureForce(const FlameParticle& rhs);

	/** @brief Accumulates the viscosity force from a neighbor. */
	void solveViscosityForce(const FlameParticle& rhs);

	/** @brief Returns the current world-space position. */
	Math::Vector3df getPosition() const { return soa_->positions[index_]; }

	/** @brief Returns the diameter of this particle (2 * radius). */
	float getDiameter() const { return soa_->radii[index_] * 2.0f; }

	/** @brief Translates the particle by the given displacement. */
	void move(const Math::Vector3df& v);

	/** @brief Sets the SPH kernel used by this particle. */
	void setKernel(SPHKernel* k) { kernel_ = k; }

	/** @brief Returns the SPH kernel used by this particle. */
	SPHKernel* getKernel() { return kernel_; }

	/** @brief Returns the owning fluid object. */
	FlameFluid* getFluid() const { return fluid_; }

	// ---- Combustion (idea doc section 1) -----------------------------------

	/**
	 * @brief Advances the per-particle combustion reaction (no neighbor access).
	 *
	 * df/dt = -k_burn * f
	 * dT/dt = k_burn * f * heat_release - k_cool * (T - T_ambient)
	 * ds/dt = k_burn * f * soot_yield
	 *
	 * O2_available is simplified to a constant 1 in Phase 1; a neighbor-soot
	 * driven decay is left as a Phase 4 extension (see plan).
	 */
	void react(const float dt);

	/** @brief Returns the current temperature. */
	float getTemperature() const { return soa_->temperatures[index_]; }

	/** @brief Sets the temperature. */
	void setTemperature(const float t) { soa_->temperatures[index_] = t; }

	/** @brief Returns the current fuel concentration (0..1). */
	float getFuel() const { return soa_->fuels[index_]; }

	/** @brief Sets the fuel concentration. */
	void setFuel(const float f) { soa_->fuels[index_] = f; }

	/** @brief Returns the current soot/smoke concentration. */
	float getSoot() const { return soa_->soots[index_]; }

	/** @brief Sets the soot/smoke concentration. */
	void setSoot(const float s) { soa_->soots[index_] = s; }

	/** @brief Returns the particle's age in seconds. */
	float getAge() const { return soa_->ages[index_]; }

	/** @brief Sets the particle's age in seconds. */
	void setAge(const float a) { soa_->ages[index_] = a; }

	/** @brief True once past the fluid's lifeMax, or burned out and cooled back near ambient. */
	bool isDead() const;

	/** @brief True for non-igniting ambient "air" carrier particles (see FlameFluid::updateEmitters()). */
	bool isAir() const { return soa_->airs[index_]; }

	/**
	 * @brief Marks/unmarks this particle as an ambient "air" carrier: it never
	 * ignites and isDead() only culls it via age > lifeMax (the burned-out/cooled
	 * heuristic would otherwise mark it dead the instant it spawns, since it
	 * starts at fuel=0 / temperature=ambient).
	 */
	void setAir(const bool a) { soa_->airs[index_] = a; }

	// ---- Buoyancy: Boussinesq approximation (idea doc section 2) -----------

	/**
	 * @brief Adds the Boussinesq buoyancy force derived from the temperature-based
	 * effective density, in place of plain gravity (see FlameSolver::simulate()).
	 * @param gravity Gravity vector (e.g. (0,-9.8,0)); used as the force direction.
	 */
	void applyBuoyancy(const Math::Vector3df& gravity);

	// ---- Vorticity confinement (idea doc section 2) ------------------------

	/** @brief Returns the vorticity accumulated by the most recent addVorticity() pass. */
	Math::Vector3df getVorticity() const { return soa_->vorticities[index_]; }

	/** @brief Pass A: accumulates omega_i = sum_j (v_j - v_i) x gradW_ij * volume_j. */
	void addVorticity(const FlameParticle& rhs);

	/** @brief Pass B: accumulates the (unnormalized) gradient of |omega| toward rhs. */
	void addVorticityGradient(const FlameParticle& rhs);

	/** @brief Applies F_vc = eps * (normalize(N) x omega) * h using the Pass B accumulator. */
	void applyVorticityConfinement(const float eps, const float h);

private:
	Math::Vector3df& positionRef() { return soa_->positions[index_]; }
	Math::Vector3df& forceRef() { return soa_->forces[index_]; }
	Math::Vector3df& velocityRef() { return soa_->velocities[index_]; }
	Math::Vector3df& vorticityRef() { return soa_->vorticities[index_]; }
	Math::Vector3df& vorticityGradAccumRef() { return soa_->vorticityGradAccums[index_]; }

	FlameParticleSoA* soa_;
	size_t index_;
	FlameFluid* fluid_;

protected:
	SPHKernel* kernel_ = nullptr;
};

	}
}
