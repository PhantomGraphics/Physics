#pragma once

#include "MVCParticle.h"
#include "Emitter.h"
#include "CGLib/Util/UnCopyable.h"
#include "CGLib/Math/Box3d.h"

#include <random>
#include <vector>

namespace Phantom {
	namespace Physics {

/**
 * @brief A fluid object composed of MVC (Moving Voronoi Cell) particles.
 *
 * Manages a Structure-of-Arrays collection of MVC particle state and
 * exposes accessors for use by MVCSolver.
 */
class MVCFluid : private UnCopyable
{
public:
	MVCFluid() = default;

	~MVCFluid() {}

	/**
	 * @brief Adds a particle to this fluid.
	 * @param p Initial particle values, copied into the SoA storage.
	 */
	void addParticle(const MVCParticle& p) { particles.push_back(p); }

	/**
	 * @brief Returns a read-only view of the particle storage.
	 * @return Const reference to the particle SoA.
	 */
	const MVCParticleSoA& getParticles() const { return particles; }

	/**
	 * @brief Returns a mutable view of the particle storage (used by MVCSolver).
	 * @return Mutable reference to the particle SoA.
	 */
	MVCParticleSoA& getParticles() { return particles; }

	/**
	 * @brief Returns the number of particles in this fluid.
	 * @return Particle count.
	 */
	int getNumParticles() const { return static_cast<int>(particles.size()); }

	/**
	 * @brief Computes the axis-aligned bounding box enclosing all particles.
	 * @return Bounding box.
	 */
	Math::Box3df getBoundingBox() const;

	void setStatic(const bool b) { isStatic_ = b; }
	bool isStatic() const { return isStatic_; }

	/**
	 * @brief Density used to seed particles spawned by updateEmitters()
	 * (mirrors WCSPHFluid/DFSPHFluid/PBSPHFluid's density field -- MVCFluid
	 * itself has no other use for a rest-density concept, since
	 * FluidWorld::createMVC() only ever set it per call-site on each
	 * MVCParticle directly before this got added).
	 */
	float getDensity() const { return density_; }
	void setDensity(const float d) { density_ = d; }

	// ---- Emitter (continuous particle generation, docs/todo/PLAN_physics_fluid_emitter.md) ----

	/** @brief Registers a new emission region. */
	void addEmitter(const Emitter& e) { emitters_.push_back(e); }

	/** @brief Returns the registered emission regions. */
	const std::vector<Emitter>& getEmitters() const { return emitters_; }

	/** @brief Mutable access to the registered emission regions (e.g. for live UI tweaks). */
	std::vector<Emitter>& getEmittersMutable() { return emitters_; }

	/** @brief Removes all registered emission regions. */
	void clearEmitters() { emitters_.clear(); }

	/** @brief Upper bound on particle count enforced by updateEmitters(). */
	int getMaxParticles() const { return maxParticles_; }
	void setMaxParticles(const int n) { maxParticles_ = n; }

	/**
	 * @brief Spawns new particles from registered emitters (rate*dt
	 * accumulation per emitter, see accumulateEmission()), seeding each with
	 * a jittered position within the emitter's disk, mass/volume of 1.0 and
	 * density() (matching FluidWorld::createMVC()'s own seeding convention),
	 * and an initial velocity of direction*speed (+/- speedJitter). Stops
	 * once getMaxParticles() is reached. No-op if no emitters are registered.
	 * @param dt Time step (seconds).
	 */
	void updateEmitters(const float dt);

private:
	bool isStatic_ = false;
	MVCParticleSoA particles;
	float density_ = 1.0f;

	std::vector<Emitter> emitters_;
	std::mt19937 rng_{ std::random_device{}() };
	int maxParticles_ = 50000;
};

	}
}
