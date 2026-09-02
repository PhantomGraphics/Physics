#pragma once

#include "CGLib/Util/UnCopyable.h"
#include "CGLib/Math/Box3d.h"
#include "SPHKernel.h"
#include "PBSPHParticle.h"
#include "Emitter.h"
#include "OutflowRegion.h"
#include "RandomSeed.h"

#include <random>
#include <vector>

namespace Phantom {
	namespace Physics {

/**
 * @brief A fluid object composed of Position-Based SPH particles.
 *
 * Owns the SoA particle storage (PBSPHParticleSoA) and stores
 * fluid material parameters such as rest density, stiffness, and viscosity.
 */
class PBSPHFluid : private UnCopyable
{
public:
	PBSPHFluid();

	~PBSPHFluid();

	/** @brief Creates and adds a new particle at the given position. */
	void createParticle(const Math::Vector3df& pos, const float radius)
	{
		particles.push_back(pos, radius, restDensity);
	}

	/**
	 * @brief Returns a read-only view of the particle storage.
	 * @return Const reference to the SoA particle storage.
	 */
	const PBSPHParticleSoA& getParticles() const { return particles; }

	/**
	 * @brief Returns a mutable view of the particle storage.
	 * @return Reference to the SoA particle storage.
	 */
	PBSPHParticleSoA& getParticles() { return particles; }

	/**
	 * @brief Computes the axis-aligned bounding box enclosing all particles.
	 * @return Bounding box.
	 */
	Math::Box3df getBoundingBox() const;

	/**
	 * @brief Returns a pointer to the SPH kernel used by this fluid.
	 * @return Pointer to the SPHKernel.
	 */
	SPHKernel* getKernel() { return &kernel; }

	/**
	 * @brief Returns the rest density of this fluid.
	 * @return Rest density value.
	 */
	float getRestDensity() const { return restDensity; }

	/**
	 * @brief Sets the kernel support radius and updates the kernel accordingly.
	 * @param effectLength New kernel support radius.
	 */
	void setEffectLength(const float effectLength);

	/**
	 * @brief Sets the rest density for this fluid.
	 * @param restDensity New rest density value.
	 */
	void setRestDensity(const float restDensity);

	/**
	 * @brief Sets the pressure stiffness coefficient.
	 * @param s Stiffness value.
	 */
	void setStiffness(const float s) { this->stiffness = s; }

	/**
	 * @brief Returns the pressure stiffness coefficient.
	 * @return Stiffness value.
	 */
	float getStiffness() const { return stiffness; }

	/**
	 * @brief Sets the viscosity coefficient.
	 * @param v Viscosity value.
	 */
	void setVicsosity(const float v) { this->viscosity = v; }

	/**
	 * @brief Returns the viscosity coefficient.
	 * @return Viscosity value.
	 */
	float getViscosity() const { return viscosity; }

	/**
	 * @brief Returns whether this fluid acts as a static boundary.
	 * @return True if this fluid is a boundary object.
	 */
	bool isBoundary() const { return _isBoundary; }

	/**
	 * @brief Sets whether this fluid acts as a static boundary.
	 * @param b True to treat this fluid as a boundary.
	 */
	void setIsBoundary(const bool b) { _isBoundary = b; }

	void setStatic(const bool b) { _isBoundary = b; }
	bool isStatic() const { return _isBoundary; }

	// ---- Emitter (continuous particle generation, internal design notes) ----

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
	 * of direction*speed (+/- speedJitter). Stops once getMaxParticles() is
	 * reached. No-op if no emitters are registered.
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

private:
	PBSPHParticleSoA particles;
	SPHKernel kernel;
	float restDensity;
	float stiffness;
	float viscosity;
	bool _isBoundary;

	std::vector<Emitter> emitters_;
	std::mt19937 rng_{ kDefaultRandomSeed };
	int maxParticles_ = 50000;

	std::vector<OutflowRegion> outflowRegions_;
};

	}
}
