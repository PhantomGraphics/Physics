#pragma once

#include "DFSPHParticle.h"
#include "SPHKernel.h"
#include "Emitter.h"
#include "OutflowRegion.h"
#include "RandomSeed.h"
#include "CGLib/Util/UnCopyable.h"
#include "CGLib/Math/Box3d.h"

#include <random>
#include <vector>

namespace Phantom {
	namespace Physics {

/**
 * @brief A fluid object composed of Divergence-Free SPH (DFSPH) particles.
 *
 * Owns the SoA particle storage (DFSPHParticleSoA) and stores fluid
 * material parameters (density, viscosity, pressure coefficient, effect
 * length) used by DFSPHSolver.
 */
class DFSPHFluid : private UnCopyable
{
public:
	DFSPHFluid();

	~DFSPHFluid() {}

	/**
	 * @brief Creates and adds a new particle at the given position.
	 * @param pos    World-space position for the new particle.
	 * @param radius Radius of the new particle.
	 * @param mass   Mass of the new particle.
	 */
	void createParticle(const Math::Vector3df& pos, const float radius, const float mass)
	{
		particles.push_back(pos, radius, mass);
	}

	/**
	 * @brief Returns a read-only view of the particle storage.
	 * @return Const reference to the SoA particle storage.
	 */
	const DFSPHParticleSoA& getParticles() const { return particles; }

	/**
	 * @brief Returns a mutable view of the particle storage.
	 * @return Reference to the SoA particle storage.
	 */
	DFSPHParticleSoA& getParticles() { return particles; }

	/**
	 * @brief Computes the axis-aligned bounding box enclosing all particles.
	 * @return Bounding box.
	 */
	Math::Box3df getBoundingBox() const;

	/**
	 * @brief Returns the rest density of this fluid.
	 * @return Density value.
	 */
	float getDensity() const { return density; }

	/**
	 * @brief Returns the viscosity coefficient.
	 * @return Viscosity coefficient value.
	 */
	float getViscosityCoe() const { return viscosityCoe; }

	/**
	 * @brief Returns the surface tension coefficient.
	 * @return Surface tension coefficient value.
	 */
	float getTensionCoe() const { return tensionCoe; }

	/**
	 * @brief Sets the kernel support radius and updates the SPH kernel.
	 * @param effectLength New kernel support radius.
	 */
	void setEffectLength(const float effectLength);

	/**
	 * @brief Returns a pointer to the SPH kernel used by this fluid.
	 * @return Pointer to the SPHKernel.
	 */
	SPHKernel* getKernel() { return &kernel; }

	void setStatic(const bool b) { isStatic_ = b; }
	bool isStatic() const { return isStatic_; }

	// ---- Emitter (continuous particle generation, docs/todo/PLAN_physics_fluid_emitter.md) ----

	/** @brief Registers a new emission region. */
	void addEmitter(const Emitter& e) { emitters_.push_back(e); }

	/** @brief Returns the registered emission regions. */
	const std::vector<Emitter>& getEmitters() const { return emitters_; }

	/** @brief Mutable access to the registered emission regions (e.g. for live UI tweaks). */
	std::vector<Emitter>& getEmittersMutable() { return emitters_; }

	/** @brief Removes all registered emission regions. */
	void clearEmitters() { emitters_.clear(); }

	/**
	 * @brief Reseeds the RNG that updateEmitters() draws its speed jitter from.
	 * @param seed New seed. The default (kDefaultRandomSeed) makes an
	 *        emitter-driven scene reproducible run to run; pass
	 *        std::random_device{}() to get a different draw each run. See
	 *        RandomSeed.h for why deterministic is the default.
	 */
	void setRandomSeed(const unsigned int seed) { rng_.seed(seed); }

	/** @brief Upper bound on particle count enforced by updateEmitters(). */
	int getMaxParticles() const { return maxParticles_; }
	void setMaxParticles(const int n) { maxParticles_ = n; }

	/**
	 * @brief Spawns new particles from registered emitters (rate*dt
	 * accumulation per emitter, see accumulateEmission()), seeding each with
	 * a jittered position within the emitter's disk and an initial velocity
	 * of direction*speed (+/- speedJitter). Mass is derived from
	 * e.particleRadius the same way createDFSPH() derives it from
	 * params_.radius (mass = diameter^3), keeping emitted particles
	 * consistent with the rest-density calibration done at scene setup.
	 * Stops once getMaxParticles() is reached. No-op if no emitters are
	 * registered.
	 * @param dt Time step (seconds).
	 */
	void updateEmitters(const float dt);

	// ---- Outflow region (optional particle removal) ----

	/** @brief Registers a new outflow (deletion) region. */
	void addOutflowRegion(const OutflowRegion& r) { outflowRegions_.push_back(r); }

	/** @brief Returns the registered outflow regions. */
	const std::vector<OutflowRegion>& getOutflowRegions() const { return outflowRegions_; }

	/** @brief Mutable access to the registered outflow regions (e.g. for live UI tweaks). */
	std::vector<OutflowRegion>& getOutflowRegionsMutable() { return outflowRegions_; }

	/** @brief Removes all registered outflow regions. */
	void clearOutflowRegions() { outflowRegions_.clear(); }

	/**
	 * @brief Removes every particle whose position currently lies inside any
	 * registered outflow region (swap-and-pop, order not preserved). No-op
	 * if no outflow regions are registered -- purely opt-in.
	 */
	void removeOutflowParticles();

	/** @brief Viscosity coefficient. */
	float viscosityCoe;
	/** @brief Kernel support radius. */
	float effectLength;
	/** @brief Pressure stiffness coefficient. */
	float pressureCoe;
	/** @brief Rest density of the fluid. */
	float density;
	/**
	 * @brief Surface tension coefficient. Defaults to 0.f (disabled) --
	 * unlike viscosityCoe/effectLength/pressureCoe/density above, this field
	 * is new and has no existing caller that always sets it, so it is
	 * default-initialized rather than left indeterminate (see
	 * docs/todo/PLAN_sph_surface_tension.md Phase 0's WCSPHFluid::tensionCoe
	 * bug, which this avoids repeating).
	 */
	float tensionCoe = 0.0f;

private:
	bool isStatic_ = false;
	DFSPHParticleSoA particles;
	SPHKernel kernel;

	std::vector<Emitter> emitters_;
	std::mt19937 rng_{ kDefaultRandomSeed };
	int maxParticles_ = 50000;

	std::vector<OutflowRegion> outflowRegions_;
};

	}
}
