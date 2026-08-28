#pragma once

#include <vector>

#include "CGLib/Math/Vector3d.h"
#include "CGLib/Space/Space/NeighborIndexView.h"

namespace Phantom {
	namespace Physics {
		class DFSPHFluid;
		class SPHKernel;

/**
 * @brief Structure-of-Arrays storage for DFSPH particle data.
 *
 * Owned by DFSPHFluid. DFSPHParticle is a non-owning (soa, index) view over
 * this storage -- see the class comment on DFSPHParticle below. Unlike
 * WCSPH, DFSPHSolver's neighbor lists are not stored here: they are
 * built per-step as index arrays local to the solver's combined (possibly
 * cross-fluid) working set, since a DFSPH neighbor can belong to a
 * different DFSPHFluid than the particle itself.
 */
struct DFSPHParticleSoA {
	std::vector<Math::Vector3df> positions;
	std::vector<Math::Vector3df> velocities;
	std::vector<Math::Vector3df> forces;
	std::vector<Math::Vector3df> normals;
	std::vector<float> radii;
	std::vector<float> masses;
	std::vector<float> densities;
	std::vector<float> alphas;
	std::vector<float> dpdts;
	std::vector<float> predictedDensities;
	// The two halves alpha is assembled from, kept so a boundary pass can add
	// its own gradient contribution *after* the fluid-neighbor pass has run
	// (DFSPHParticle::addBoundaryAlphaGradient()) -- alpha = |gradSum|^2 +
	// gradSqSum, and the boundary only contributes to the first term.
	std::vector<Math::Vector3df> alphaGradSums;
	std::vector<float>           alphaGradSqSums;

	size_t size() const { return positions.size(); }
	bool empty() const { return positions.empty(); }

	void clear()
	{
		positions.clear();
		velocities.clear();
		forces.clear();
		normals.clear();
		radii.clear();
		masses.clear();
		densities.clear();
		alphas.clear();
		dpdts.clear();
		predictedDensities.clear();
		alphaGradSums.clear();
		alphaGradSqSums.clear();
	}

	void push_back(const Math::Vector3df& position, const float radius, const float mass)
	{
		positions.push_back(position);
		velocities.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
		forces.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
		normals.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
		radii.push_back(radius);
		masses.push_back(mass);
		densities.push_back(0.0f);
		alphas.push_back(0.0f);
		dpdts.push_back(0.0f);
		predictedDensities.push_back(0.0f);
		alphaGradSums.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
		alphaGradSqSums.push_back(0.0f);
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
		radii[index] = radii.back();
		radii.pop_back();
		masses[index] = masses.back();
		masses.pop_back();
		densities[index] = densities.back();
		densities.pop_back();
		alphas[index] = alphas.back();
		alphas.pop_back();
		dpdts[index] = dpdts.back();
		dpdts.pop_back();
		predictedDensities[index] = predictedDensities.back();
		predictedDensities.pop_back();
		alphaGradSums[index] = alphaGradSums.back();
		alphaGradSums.pop_back();
		alphaGradSqSums[index] = alphaGradSqSums.back();
		alphaGradSqSums.pop_back();
	}
};

/**
 * @brief A non-owning (DFSPHParticleSoA*, index) handle for a single particle
 * in a Divergence-Free SPH (DFSPH) simulation.
 *
 * All actual particle data lives in DFSPHParticleSoA (owned by DFSPHFluid);
 * this class is a lightweight, copyable view -- constructed fresh wherever
 * per-particle behavior is needed (DFSPHSolver's per-step working set,
 * pybind11 get_particles() results) rather than persisted. Implements the
 * two-stage DFSPH correction (divergence-free velocity and constant-density
 * enforcement). Unlike the pre-SoA design, this class does not store a
 * neighbor list itself -- callers pass the current neighbor index list (into
 * the same working set vector this particle came from) explicitly to each
 * method that needs it, since DFSPHSolver::simulate() builds one combined
 * working set spanning every registered DFSPHFluid and a neighbor may
 * belong to a different fluid than "this".
 */
class DFSPHParticle
{
public:
	/**
	 * @brief Constructs a view over one particle's slot in a DFSPHParticleSoA.
	 * @param soa   SoA storage backing this particle; must outlive the view.
	 * @param index Index of this particle within soa.
	 * @param fluid Owning fluid object supplying shared parameters.
	 */
	DFSPHParticle(DFSPHParticleSoA& soa, const size_t index, DFSPHFluid* fluid) :
		soa_(&soa), index_(index), fluid_(fluid)
	{}

	/**
	 * @brief Returns the current world-space position.
	 * @return Position vector.
	 */
	Math::Vector3df getPosition() const { return soa_->positions[index_]; }

	/**
	 * @brief Sets the world-space position.
	 * @param p New position vector.
	 */
	void setPosition(const Math::Vector3df& p) { positionRef() = p; }

	/**
	 * @brief Translates the particle by the given displacement.
	 * @param d Displacement vector.
	 */
	void addPosition(const Math::Vector3df& d) { positionRef() += d; }

	/**
	 * @brief Returns the current velocity.
	 * @return Velocity vector.
	 */
	Math::Vector3df getVelocity() const { return soa_->velocities[index_]; }

	/**
	 * @brief Sets the velocity.
	 * @param v New velocity vector.
	 */
	void setVelocity(const Math::Vector3df& v) { velocityRef() = v; }

	/**
	 * @brief Adds a velocity increment to this particle.
	 * @param v Velocity increment to accumulate.
	 */
	void addVelocity(const Math::Vector3df& v) { velocityRef() += v; }

	/**
	 * @brief Returns the accumulated force on this particle.
	 * @return Force vector.
	 */
	Math::Vector3df getForce() const { return soa_->forces[index_]; }

	/**
	 * @brief Replaces the current force with the given value.
	 * @param f New force vector.
	 */
	void setForce(const Math::Vector3df& f) { forceRef() = f; }

	/**
	 * @brief Adds a force contribution to this particle.
	 * @param f Force vector to accumulate.
	 */
	void addForce(const Math::Vector3df& f) { forceRef() += f; }

	/**
	 * @brief Returns the particle radius.
	 * @return Radius value.
	 */
	float getRadius() const { return soa_->radii[index_]; }

	/**
	 * @brief Returns the particle mass.
	 * @return Mass value.
	 */
	float getMass() const { return soa_->masses[index_]; }

	/**
	 * @brief Returns the current density.
	 * @return Density value.
	 */
	float getDensity() const { return soa_->densities[index_]; }

	/**
	 * @brief Adds a density contribution to this particle.
	 * @param d Density increment to accumulate.
	 */
	void addDensity(const float d) { soa_->densities[index_] += d; }

	/**
	 * @brief Returns the density predicted for the next time step.
	 * @return Predicted density value.
	 */
	float getPredictedDensity() const { return soa_->predictedDensities[index_]; }

	/**
	 * @brief Returns the time derivative of density (dp/dt).
	 * @return dp/dt value.
	 */
	float getDpDt() const { return soa_->dpdts[index_]; }

	/**
	 * @brief Computes and stores the SPH density from the given neighbor list.
	 * @param all       Combined particle working set that neighbor indices index into.
	 * @param neighbors Indices (into all) of this particle's neighbors.
	 */
	void calculateDensity(const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors);

	/** @brief Computes the DFSPH alpha (stiffness) coefficient from neighbors. */
	void calculateAlpha(const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors);

	/**
	 * @brief Folds a Two-Way boundary particle's psi-weighted kernel gradient
	 * into this particle's alpha, and returns the gradient sum alpha is now
	 * built from.
	 *
	 * alpha is DFSPH's denominator: how much the density constraint can
	 * actually be changed by the pressure solve. addBoundaryParticleDensity()
	 * raises the *numerator* (density) whenever a boundary is nearby, so
	 * without the matching denominator term the solve computes an unbounded
	 * stiffness for a density excess it cannot remove -- and slams the fluid
	 * (docs/issue/CODEBASE_ISSUES.md 1.6). This is DFSPH's counterpart of
	 * PBSPHSolver::addBoundaryParticleConstraintGradient(), which has always
	 * fed the boundary into PBSPH's own denominator.
	 *
	 * Call after calculateAlpha() (which resets the accumulators) and before
	 * the pressure solve, once per boundary particle in range.
	 * @param gradient psi-weighted kernel gradient (psi_b * gradW_ib).
	 */
	void addBoundaryAlphaGradient(const Math::Vector3df& gradient);

	/** @brief Returns the DFSPH alpha (stiffness) coefficient. */
	float getAlpha() const { return soa_->alphas[index_]; }

	/** @brief Computes the time derivative of density (dp/dt) from neighbors. */
	void calculateDpDt(const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors);

	/**
	 * @brief Predicts the density after one time step using current velocities.
	 * @param dt Time step (seconds).
	 */
	void predictDensity(const float dt, const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors);

	/**
	 * @brief Applies a velocity correction to eliminate divergence error.
	 * @param dt Time step (seconds).
	 */
	void calculateVelocityInDivergenceError(const float dt, const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors);

	/**
	 * @brief Applies a velocity correction to eliminate density error.
	 * @param dt Time step (seconds).
	 */
	void calculateVelocityInDensityError(const float dt, const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors);

	/** @brief Computes and accumulates the viscosity force from neighbors. */
	void calculateViscosity(const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors);

	/**
	 * @brief Returns the surface normal at this particle.
	 * @return Normal vector.
	 */
	Math::Vector3df getNormal() const { return soa_->normals[index_]; }

	/**
	 * @brief Computes and stores the surface normal from the given neighbor
	 * list (mass/density-weighted sum of Poly6 kernel gradients, mirroring
	 * WCSPHParticle::solveNormal()). Must be called, for every particle, after
	 * calculateDensity() and before calculateSurfaceTension() -- the normal
	 * needs every neighbor's density already accumulated, the same reason
	 * DFSPHSolver::simulate() computes density in its own pass before pressure/
	 * viscosity.
	 * @param all       Combined particle working set that neighbor indices index into.
	 * @param neighbors Indices (into all) of this particle's neighbors.
	 */
	void calculateNormal(const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors);

	/**
	 * @brief Computes and accumulates the surface tension force from
	 * neighbors (Poly6 Laplacian pulling this particle along its surface
	 * normal, mirroring WCSPHParticle::solveSurfaceTension()). Requires
	 * calculateNormal() to have already been called for this particle.
	 * @param all       Combined particle working set that neighbor indices index into.
	 * @param neighbors Indices (into all) of this particle's neighbors.
	 */
	void calculateSurfaceTension(const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors);

	/**
	 * @brief Returns the owning fluid object.
	 * @return Pointer to the parent DFSPHFluid.
	 */
	DFSPHFluid* getParent() const { return fluid_; }

private:
	SPHKernel* getKernel();

	/** @brief Recomputes alpha from the stored gradient accumulators (clamped away from zero). */
	void finalizeAlpha();

	/**
	 * @brief Returns the unit surface normal, or a zero vector if the raw
	 * normal's magnitude -- nondimensionalized by the kernel's effect length,
	 * so the same threshold applies regardless of scene scale (see
	 * docs/todo/PLAN_sph_scale_invariance.md) -- is below a noise threshold,
	 * i.e. this particle is judged interior rather than on the free surface.
	 * @return Unit surface normal, or (0,0,0) for interior particles.
	 */
	Math::Vector3df surfaceNormalHat() const;

	Math::Vector3df& positionRef() { return soa_->positions[index_]; }
	Math::Vector3df& velocityRef() { return soa_->velocities[index_]; }
	Math::Vector3df& forceRef()    { return soa_->forces[index_]; }
	Math::Vector3df& normalRef()   { return soa_->normals[index_]; }
	float& densityRef()            { return soa_->densities[index_]; }
	float& alphaRef()              { return soa_->alphas[index_]; }
	Math::Vector3df& alphaGradSumRef() { return soa_->alphaGradSums[index_]; }
	float& alphaGradSqSumRef()         { return soa_->alphaGradSqSums[index_]; }
	float& dpdtRef()                { return soa_->dpdts[index_]; }
	float& predictedDensityRef()   { return soa_->predictedDensities[index_]; }

	DFSPHParticleSoA* soa_;
	size_t index_;
	DFSPHFluid* fluid_;
};

	}
}
