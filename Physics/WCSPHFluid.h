#pragma once

#include "WCSPHParticle.h"
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
 * @brief A fluid object composed of Weakly Compressible SPH (WCSPH) particles.
 *
 * Owns the SoA particle storage (WCSPHParticleSoA) and stores fluid
 * material parameters such as density, pressure, viscosity, and surface tension.
 */
class WCSPHFluid : private UnCopyable
{
public:
	WCSPHFluid();

	~WCSPHFluid() {}

	/**
	 * @brief Creates and adds a new particle at the given position.
	 * @param pos  World-space position for the new particle.
	 * @param mass Radius of the new particle (misnomer kept for API compatibility).
	 */
	void createParticle(const Math::Vector3df& pos, const float mass)
	{
		particles.push_back(pos, mass, this->getDensity());
	}

	/**
	 * @brief Returns a read-only view of the particle storage.
	 * @return Const reference to the SoA particle storage.
	 */
	const WCSPHParticleSoA& getParticles() const { return particles; }

	/**
	 * @brief Returns a mutable view of the particle storage.
	 * @return Reference to the SoA particle storage.
	 */
	WCSPHParticleSoA& getParticles() { return particles; }

	/**
	 * @brief Returns the number of particles in this fluid.
	 * @return Particle count.
	 */
	int getNumParticles() const { return static_cast<int>(particles.size()); }

	/**
	 * @brief Returns the world-space position of the particle at the given index.
	 * @param index Zero-based particle index.
	 * @return Position vector.
	 */
	Math::Vector3df getPosition(const int index) const { return particles.positions[index]; }

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
	 * @brief Returns the pressure stiffness coefficient.
	 * @return Pressure coefficient value.
	 */
	float getPressureCoe() const { return pressureCoe; }

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
	 * @brief Returns the kernel support radius.
	 * @return Effect length value.
	 */
	float getEffectLength() const { return effectLength; }

	/**
	 * @brief Sets the rest density.
	 * @param d New density value.
	 */
	void setDensity(const float d) { this->density = d; }

	/**
	 * @brief Sets the pressure stiffness coefficient.
	 * @param c New pressure coefficient.
	 */
	void setPressureCoe(const float c) { this->pressureCoe = c; }

	/**
	 * @brief Estimates a scale-appropriate pressure stiffness coefficient
	 * (see internal design notes section 4: with pressureCoe
	 * held fixed, the pressure acceleration for a fixed relative compression
	 * scales as 1/effectLength, so a raw numeric pressureCoe means something
	 * different at every scene scale). Simply linear in effectLength
	 * (pressureCoe = pressureCoeScale * effectLength) rather than derived from
	 * gravity/target density error/rest density -- see internal design notes for the retired physics-based derivation.
	 * @param effectLength Kernel support radius (the scene's length scale).
	 * @param pressureCoeScale Proportionality constant. Defaults to 1960.0f,
	 *        matching the historical value at effectLength=1 under the retired
	 *        derivation's own defaults (gravity=9.8, maxDensityErrorRatio=0.01).
	 * @return Recommended value for setPressureCoe().
	 */
	static float estimatePressureCoe(const float effectLength, const float pressureCoeScale = 1960.0f);

	/**
	 * @brief Sets the pressure stiffness coefficient from estimatePressureCoe(),
	 * using this fluid's own effectLength (must already be set via
	 * setEffectLength() before calling this).
	 * @param pressureCoeScale See estimatePressureCoe().
	 */
	void setPressureCoeFromScale(const float pressureCoeScale = 1960.0f)
	{
		this->pressureCoe = estimatePressureCoe(this->effectLength, pressureCoeScale);
	}

	/**
	 * @brief Sets the viscosity coefficient.
	 * @param c New viscosity coefficient.
	 */
	void setVicosityCoe(const float c) { this->viscosityCoe = c; }

	// No estimateViscosityCoe()/setViscosityCoeFromScale() on purpose.
	//
	// There used to be a pair that derived viscosityCoe ~ effectLength^1.5 from
	// a target damping ratio. The derivation was wrong, and worse, it advertised
	// "scale viscosity with resolution" as this codebase's rule -- which is how
	// the water-sphere showcase came to run its production tier at half the
	// preview tier's viscosity and spray all over the container
	// (internal design notes 11.1-11.3).
	//
	// solveViscosityForce() adds viscosityCoe * dv * laplacian(W_visc) * m_j and
	// forwardTime() divides by rho, so next to Muller et al. 2003's
	// mu * sum_j m_j dv/rho_j * laplacian(W) the neighbor's 1/rho_j is missing:
	// viscosityCoe == mu/rho, the kinematic viscosity (m^2/s) itself. Through
	// the SPH Laplacian the acceleration is viscosityCoe * laplacian(v), which
	// contains no h -- so the same value damps the same physical flow by the
	// same amount at any resolution (measured: within 5% across a 2x change;
	// pinned by WCSPHFluidTest.
	// ViscousDampingIsResolutionIndependentForAFixedViscosityCoe).
	//
	// It cannot be estimated from effectLength either, and that is not an
	// oversight: the length scale a viscosity has to be judged against belongs
	// to the flow being simulated, not to the discretization. Pick it from the
	// fluid being modelled and pass it to setVicosityCoe() unchanged.

	/**
	 * @brief Sets the surface tension coefficient.
	 * @param c New surface tension coefficient.
	 */
	void setTensionCoe(const float c) { this->tensionCoe = c; }

	/**
	 * @brief Sets the kernel support radius and updates the SPH kernel
	 * (mirrors DFSPHFluid::setEffectLength()/PBSPHFluid::setEffectLength()).
	 * @param l New effect length value.
	 */
	void setEffectLength(const float l) { this->effectLength = l; this->kernel = SPHKernel(l); }

	/**
	 * @brief Returns a pointer to the SPH kernel used by this fluid.
	 * @return Pointer to the SPHKernel.
	 */
	SPHKernel* getKernel() { return &kernel; }

	/**
	 * @brief Marks this fluid as a static boundary object.
	 * @param b True to treat this fluid as a static boundary.
	 */
	void setStatic(const bool b) { this->isBoundary = b; }

	/**
	 * @brief Returns whether this fluid is a static boundary object.
	 * @return True if this fluid acts as a boundary.
	 */
	bool isStatic() const { return isBoundary; }

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
	float tensionCoe;
	float viscosityCoe;
	// Zero-initialized so WCSPHSolver::simulate() can detect "setEffectLength()
	// was never called" as a checkable getEffectLength() == 0.f rather than
	// reading indeterminate garbage into its neighbor search radius (see
	// internal design notes Phase 5).
	float effectLength = 0.0f;
	float pressureCoe;
	float density;
	bool isBoundary = false;

private:
	WCSPHParticleSoA particles;
	SPHKernel kernel;

	std::vector<Emitter> emitters_;
	std::mt19937 rng_;
	int maxParticles_ = 50000;

	std::vector<OutflowRegion> outflowRegions_;
};

	}
}
