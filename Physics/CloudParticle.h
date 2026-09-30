#pragma once

#include "CloudParams.h"
#include "CloudStats.h"
#include "CGLib/Math/Vector3d.h"

#include <cstdint>
#include <vector>

namespace Phantom {
	namespace Physics {

/**
 * @brief Structure-of-Arrays state of the moist-air particles (plan section 4).
 * Double precision: this is the CPU reference model the GPU port is checked
 * against. Every particle is an air parcel (transparent air is simulated too).
 */
struct CloudParticleSoA {
	std::vector<uint32_t> ids;               ///< Stable IDs (never reused within a run).
	std::vector<Math::Vector3dd> positions;  ///< [m]
	std::vector<Math::Vector3dd> velocities; ///< [m/s]
	std::vector<double> mDry;                ///< Dry-air mass [kg]
	std::vector<double> densities;           ///< SPH density [kg/m^3] (filled by the solver)
	std::vector<double> temperatures;        ///< [K]
	std::vector<double> qv;                  ///< Vapour mixing ratio [kg/kg dry air]
	std::vector<double> qc;                  ///< Cloud-water mixing ratio [kg/kg dry air]

	uint32_t nextId = 0;

	size_t size() const { return positions.size(); }
	bool empty() const { return positions.empty(); }
	void clear();

	/** @brief Appends one particle with a fresh stable ID; returns its index. */
	size_t add(const Math::Vector3dd& position, const Math::Vector3dd& velocity, double massDry,
	           double temperature, double vapor, double cloudWater);
};

namespace CloudOps {

/**
 * @brief Fills the axis-aligned box [lo, hi] with a cubic lattice (spacing in
 * metres) of air at the environment state, at rest relative to the wind.
 * mDry = rho_dry * spacing^3 with rho_dry from the ideal gas law at the local
 * environment T and p. Adds the initial water to `ledger.initialWater`.
 */
void fillEnvironmentLattice(CloudParticleSoA& soa, const CloudThermoParams& th,
                            const CloudEnvironmentParams& env, const Math::Vector3dd& lo,
                            const Math::Vector3dd& hi, double spacing, CloudWaterLedger& ledger);

/**
 * @brief Applies the source terms over [time, time+dt] (clipped to the source's
 * active window) to existing particles. Weights are a normalized smooth ball
 * falloff; vapour added is recorded in `ledger.suppliedWater`. A source whose
 * ball contains no particle supplies nothing (and records nothing).
 */
void applySources(CloudParticleSoA& soa, const std::vector<CloudSourceParams>& sources,
                  double time, double dt, CloudWaterLedger& ledger);

/**
 * @brief Conservative pairwise exchange of T, qv and qc between particles i and j.
 * `a` in [0,1] is the mixing fraction (1 = full equalization). Uses mass-weighted
 * convex updates, so mDry*qv, mDry*qc, mDry*T are conserved exactly and values stay
 * inside their pre-exchange range (no new negatives). Ignores a<=0 or i==j.
 */
void mixPair(CloudParticleSoA& soa, size_t i, size_t j, double a);

/**
 * @brief mixPair() for a stratified atmosphere: T and qv are exchanged as deviations from the
 * environment profile at each particle's height (mixing must not diffuse the background
 * lapse rate / humidity gradient); qc is exchanged directly. Mass-weighted, so
 * sum(mDry*qv), sum(mDry*qc) are conserved exactly (positions are fixed during mixing).
 * If the vapour exchange would make either qv negative, the vapour part is skipped.
 */
void mixPairRelative(CloudParticleSoA& soa, size_t i, size_t j, double a, const CloudThermoParams& th,
                     const CloudEnvironmentParams& env);

/**
 * @brief Runs the saturation adjustment for every particle using the hydrostatic
 * environment pressure at its height. Returns the number of particles whose
 * adjustment failed to converge or had invalid input (state left unchanged).
 */
size_t saturationAdjustAll(CloudParticleSoA& soa, const CloudThermoParams& th,
                           const CloudEnvironmentParams& env, CloudStats* stats = nullptr);

/** @brief Aggregates conservation / validity statistics. `qcThreshold` defines cloud base/top. */
CloudStats computeStats(const CloudParticleSoA& soa, const CloudWaterLedger& ledger,
                        double qcThreshold = 1.0e-5);

}  // namespace CloudOps

	}
}
