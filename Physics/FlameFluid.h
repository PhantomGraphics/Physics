#pragma once

#include "FlameParticle.h"
#include "CGLib/Util/UnCopyable.h"
#include "CGLib/Math/Box3d.h"
#include "RandomSeed.h"

#include <random>
#include <vector>

namespace Phantom {
	namespace Physics {

/**
 * @brief A fluid object composed of Flame (reacting hot gas) SPH particles.
 *
 * Mirrors WCSPHFluid's material parameters (density/pressureCoe/viscosityCoe/
 * effectLength) and adds combustion/buoyancy/vorticity/curl-noise parameters
 * plus a simple particle-emitter and lifetime management (idea doc sections
 * 1-3). See internal design notes for the design rationale.
 */
class FlameFluid : private UnCopyable
{
public:
	/** @brief A circular emission region that spawns new ignited particles over time. */
	struct Emitter {
		Math::Vector3df center{ 0.0f, 0.0f, 0.0f };
		float radius = 0.1f;
		float rate = 50.0f; // particles/sec

		// Fractional-particle carry between steps (rate*dt is rarely integral).
		// Not part of the plan's minimal struct, but required for updateEmitters()
		// to emit at the correct average rate rather than rounding every step.
		float accumulator = 0.0f;

		// Non-igniting ambient "air" carrier particles/sec, co-emitted in a
		// slightly wider halo around the fuel core (see updateEmitters()). These
		// give newly-ignited particles SPH neighbors from frame one instead of
		// spawning into a near-vacuum, which otherwise makes pressure/vorticity
		// forces divide by a tiny density and spike. Zero by default (opt-in;
		// see FlameApp for a tuned example).
		float airRate = 0.0f;
		float airAccumulator = 0.0f;
	};

	/** @brief What a SecondaryParticle represents, for rendering purposes. */
	enum class SecondaryKind {
		Spark, // A bright ember flung from a hot primary particle; fades/cools out.
		Smoke, // A soft, slow-rising puff spawned from a sooty, cooling primary particle.
	};

	/**
	 * @brief A cosmetic, non-SPH particle (spark/ember or smoke puff) meant to add
	 * visual richness around the physically-simulated flame core.
	 *
	 * Deliberately NOT part of the SPH simulation: no neighbor search, no
	 * density/pressure, no influence on -- or from -- FlameParticle. Purely
	 * kinematic (see FlameFluid::updateSecondaryParticles()), since realism
	 * here is a rendering concern, not a physics one.
	 */
	struct SecondaryParticle {
		Math::Vector3df position{ 0.0f, 0.0f, 0.0f };
		Math::Vector3df velocity{ 0.0f, 0.0f, 0.0f };
			// Vorticity inherited from the source primary particle at spawn time
			// (see FlameParticle::getVorticity()), decayed over the secondary's
			// life in updateSecondaryParticles() to drive a purely-kinematic
			// swirl -- lets sparks/smoke keep curling after they decouple from
			// the SPH simulation instead of flying in a straight line.
			Math::Vector3df vorticity{ 0.0f, 0.0f, 0.0f };
		float age = 0.0f;
		float lifeMax = 1.0f;
		float size = 1.0f;
		float temperature = 300.0f; // Spark/smoke color (reuses the flame's temperature gradient).
		float opacity = 0.0f;       // Smoke alpha.
		SecondaryKind kind = SecondaryKind::Spark;
	};

	FlameFluid();

	~FlameFluid() {}

	/** @brief Creates and adds a new (initially unignited, ambient-temperature) particle. */
	void createParticle(const Math::Vector3df& pos, const float radius)
	{
		particles.push_back(pos, radius, density, ambientTemperature);
	}

	/** @brief Returns a read-only view of the particle storage. */
	const FlameParticleSoA& getParticles() const { return particles; }

	/** @brief Returns a mutable view of the particle storage. */
	FlameParticleSoA& getParticles() { return particles; }

	/** @brief Returns the number of particles in this fluid. */
	int getNumParticles() const { return static_cast<int>(particles.size()); }

	/** @brief Returns the world-space position of the particle at the given index. */
	Math::Vector3df getPosition(const int index) const { return particles.positions[index]; }

	/** @brief Computes the axis-aligned bounding box enclosing all particles. */
	Math::Box3df getBoundingBox() const;

	// ---- WCSPH-style material parameters ------------------------------------

	float getDensity() const { return density; }
	void setDensity(const float d) { density = d; }

	float getPressureCoe() const { return pressureCoe; }
	void setPressureCoe(const float c) { pressureCoe = c; }

	float getViscosityCoe() const { return viscosityCoe; }
	void setVicosityCoe(const float c) { viscosityCoe = c; }

	float getEffectLength() const { return effectLength; }
	void setEffectLength(const float l) { effectLength = l; }

	// ---- Combustion parameters (idea doc section 1) -------------------------

	float getAmbientTemperature() const { return ambientTemperature; }
	void setAmbientTemperature(const float t) { ambientTemperature = t; }

	float getIgnitionTemperature() const { return ignitionTemperature; }
	void setIgnitionTemperature(const float t) { ignitionTemperature = t; }

	float getBurnRate() const { return burnRate; }
	void setBurnRate(const float v) { burnRate = v; }

	float getHeatRelease() const { return heatRelease; }
	void setHeatRelease(const float v) { heatRelease = v; }

	float getCoolRate() const { return coolRate; }
	void setCoolRate(const float v) { coolRate = v; }

	float getSootYield() const { return sootYield; }
	void setSootYield(const float v) { sootYield = v; }

	// ---- Buoyancy / vorticity / curl-noise parameters (idea doc section 2) --

	float getBuoyancyCoe() const { return buoyancyCoe; }
	void setBuoyancyCoe(const float v) { buoyancyCoe = v; }

	float getThermalExpansion() const { return thermalExpansion; }
	void setThermalExpansion(const float v) { thermalExpansion = v; }

	float getVorticityEps() const { return vorticityEps; }
	void setVorticityEps(const float v) { vorticityEps = v; }

	float getCurlNoiseStrength() const { return curlNoiseStrength; }
	void setCurlNoiseStrength(const float v) { curlNoiseStrength = v; }

	float getCurlNoiseFrequency() const { return curlNoiseFrequency; }
	void setCurlNoiseFrequency(const float v) { curlNoiseFrequency = v; }

	float getLifeMax() const { return lifeMax; }
	void setLifeMax(const float v) { lifeMax = v; }

	/** @brief Returns the per-particle speed cap applied in FlameParticle::forwardTime(). */
	float getMaxSpeed() const { return maxSpeed; }
	/** @brief Sets the per-particle speed cap (stability safety net, see forwardTime()). */
	void setMaxSpeed(const float v) { maxSpeed = v; }

	// ---- Emitter / lifetime management (idea doc section 3) -----------------

	int getMaxParticles() const { return maxParticles; }
	void setMaxParticles(const int n) { maxParticles = n; }

	/** @brief Registers a new emission region. */
	void addEmitter(const Emitter& e) { emitters.push_back(e); }

	/** @brief Returns the registered emission regions. */
	const std::vector<Emitter>& getEmitters() const { return emitters; }

	/** @brief Mutable access to the registered emission regions (e.g. for live UI tweaks). */
	std::vector<Emitter>& getEmittersMutable() { return emitters; }

	/** @brief Removes all registered emission regions. */
	void clearEmitters() { emitters.clear(); }

	/**
	 * @brief Reseeds the RNG this fluid draws its spawn-disk, spark and smoke
	 * sampling from.
	 * @param seed New seed. The default (kDefaultRandomSeed) makes a flame
	 *        scene reproducible run to run; pass std::random_device{}() to get
	 *        a different draw each run. See RandomSeed.h.
	 */
	void setRandomSeed(const unsigned int seed) { rng.seed(seed); }


	/**
	 * @brief Generates new particles from registered emitters (rate*dt accumulation
	 * per emitter), seeding them with T=ignitionTemperature, f=1, s=0, age=0, and an
	 * upward jittered velocity. Stops once getMaxParticles() is reached.
	 * @param dt Time step (seconds).
	 */
	void updateEmitters(const float dt);

	/**
	 * @brief Removes particles that are dead (see FlameParticle::isDead()), via
	 * swap-and-pop (same pattern as WhiteWaterSystem::removeDead()).
	 */
	void removeDead();

	// ---- Secondary particles: sparks / smoke (cosmetic, non-SPH) ------------

	/**
	 * @brief Target spark population, expressed as a multiple of the current
	 * primary particle count (see updateSecondaryParticles()). Together with
	 * getSmokeCountPerPrimary(), this is what lets secondary particles alone
	 * carry the rendering: a value of e.g. 6-20 keeps the spark cloud an order
	 * of magnitude denser than the SPH core it is sampled from.
	 */
	float getSparkCountPerPrimary() const { return sparkCountPerPrimary; }
	void setSparkCountPerPrimary(const float v) { sparkCountPerPrimary = v; }

	float getSparkLifeMax() const { return sparkLifeMax; }
	void setSparkLifeMax(const float v) { sparkLifeMax = v; }

	float getSparkSpeed() const { return sparkSpeed; }
	void setSparkSpeed(const float v) { sparkSpeed = v; }

	float getSparkSize() const { return sparkSize; }
	void setSparkSize(const float v) { sparkSize = v; }

	/** @brief Target smoke population, as a multiple of the primary particle count (see getSparkCountPerPrimary()). */
	float getSmokeCountPerPrimary() const { return smokeCountPerPrimary; }
	void setSmokeCountPerPrimary(const float v) { smokeCountPerPrimary = v; }

	float getSmokeLifeMax() const { return smokeLifeMax; }
	void setSmokeLifeMax(const float v) { smokeLifeMax = v; }

	float getSmokeRiseSpeed() const { return smokeRiseSpeed; }
	void setSmokeRiseSpeed(const float v) { smokeRiseSpeed = v; }

	float getSmokeStartSize() const { return smokeStartSize; }
	void setSmokeStartSize(const float v) { smokeStartSize = v; }

	float getSmokeEndSize() const { return smokeEndSize; }
	void setSmokeEndSize(const float v) { smokeEndSize = v; }

	int getMaxSecondaryParticles() const { return maxSecondaryParticles; }
	void setMaxSecondaryParticles(const int n) { maxSecondaryParticles = n; }

	/**
	 * @brief Strength of the kinematic swirl applied from each secondary
	 * particle's inherited vorticity (see SecondaryParticle::vorticity and
	 * updateSecondaryParticles()). 0 disables the effect.
	 */
	float getSecondarySwirlStrength() const { return secondarySwirlStrength; }
	void setSecondarySwirlStrength(const float v) { secondarySwirlStrength = v; }

	/** @brief Returns a read-only view of the current secondary (spark/smoke) particles. */
	const std::vector<SecondaryParticle>& getSecondaryParticles() const { return secondaryParticles; }

	/**
	 * @brief Tops up sparks/smoke from randomly sampled primary particles until
	 * each population reaches sparkCountPerPrimary/smokeCountPerPrimary times
	 * the current primary particle count (so the secondary cloud stays dense
	 * enough to render on its own, independent of any flat rate), inheriting
	 * position/velocity/vorticity/temperature from the sampled source. Advects
	 * existing secondary particles with simple kinematics -- including a
	 * vorticity-driven swirl (see secondarySwirlStrength) -- (no SPH forces),
	 * and culls those past their lifeMax. No-op if there are no primary
	 * particles yet.
	 * @param dt Time step (seconds).
	 * @param gravity Direction used for the sparks' light "ember fall" drift.
	 */
	void updateSecondaryParticles(const float dt, const Math::Vector3df& gravity);

private:
	float density = 1.0f;
	float pressureCoe = 50.0f;
	float viscosityCoe = 0.001f;
	float effectLength = 0.15f;

	float ambientTemperature = 300.0f;
	float ignitionTemperature = 1200.0f;
	float burnRate = 1.0f;
	float heatRelease = 2000.0f;
	float coolRate = 3.0f;
	float sootYield = 0.3f;
	// buoyancyCoe/thermalExpansion are tuned so a freshly ignited particle
	// (temperature == ignitionTemperature, deltaT == 900K against the defaults
	// below) produces a lively but bounded ~3g rise. The previous defaults
	// (4.0 / 0.02) produced a relative density perturbation of 18 -- i.e. 18x
	// the rest density -- which is far outside Boussinesq's small-perturbation
	// validity and caused particles to be launched at extreme speed; see also
	// the clamp in FlameParticle::applyBuoyancy() for a hard bound regardless
	// of parameter tuning.
	float buoyancyCoe = 2.0f;
	float thermalExpansion = 0.0015f;
	float vorticityEps = 2.0f;
	float curlNoiseStrength = 0.5f;
	float curlNoiseFrequency = 0.3f;
	float lifeMax = 4.0f;
	float maxSpeed = 3.0f;

	int maxParticles = 5000;

	std::vector<Emitter> emitters;
	std::mt19937 rng;

	// Sparks/embers: short-lived, fast, bright -- rendered via the same
	// temperature-gradient point sprite as primary flame particles (see
	// FlameApp), so only count/life/speed/size need tuning here. Population is
	// maintained at sparkCountPerPrimary * (primary particle count) each step
	// (see updateSecondaryParticles()), not a flat rate, so the secondary
	// cloud scales with however many primaries the SPH core currently has.
	float sparkCountPerPrimary = 0.0f; // 0 disables spawning (opt-in, see FlameApp).
	float sparkLifeMax = 0.6f;
	float sparkSpeed = 1.5f;
	float sparkSize = 0.5f; // relative to the primary particles' point size.

	// Smoke: long-lived, slow, soft, dark -- rendered via a separate
	// alpha-blended pipeline (FlameView's FlameSmokePipeline).
	float smokeCountPerPrimary = 0.0f; // 0 disables spawning (opt-in, see FlameApp).
	float smokeLifeMax = 3.0f;
	float smokeRiseSpeed = 0.3f;
	float smokeStartSize = 1.0f;
	float smokeEndSize = 2.5f;

	float secondarySwirlStrength = 1.5f;

	// Large enough to hold sparkCountPerPrimary/smokeCountPerPrimary at ~50x
	// against FlameApp's default maxParticles (see setupInitialScene()).
	int maxSecondaryParticles = 200000;
	std::vector<SecondaryParticle> secondaryParticles;

private:
	FlameParticleSoA particles;
};

	}
}
