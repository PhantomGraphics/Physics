#pragma once

#include "CloudParams.h"
#include "CloudParticle.h"
#include "CloudStats.h"
#include "CGLib/Space/Space/NeighborList.h"

#include <vector>

namespace Phantom {
	namespace Physics {

/**
 * @brief Solver settings for the CPU reference cloud model (plan section 4).
 * The flow is a weakly compressible, constant-reference-density CG
 * approximation: pressure is driven by deviation of the *number density* from
 * each particle's rest value, with an artificial sound speed. It is not a full
 * anelastic atmosphere model.
 */
struct CloudSolverParams {
	Math::Vector3dd domainMin = Math::Vector3dd(0.0);
	Math::Vector3dd domainMax = Math::Vector3dd(1000.0);
	double spacing = 31.25;             ///< Rest lattice spacing [m].
	double effectLengthRatio = 2.0;     ///< Kernel support radius = ratio * spacing.
	double soundSpeed = 50.0;           ///< Artificial sound speed c [m/s]; p = c^2 m (n - n0).
	double viscosity = 5.0;             ///< Kinematic (artificial) viscosity [m^2/s].
	double windRelaxTime = 60.0;        ///< Relaxation time toward the background wind [s].
	double mixDiffusivity = 10.0;       ///< Heat/moisture eddy diffusivity kappa [m^2/s] (resolution independent).
	double wallRestitution = 0.0;       ///< Normal-velocity restitution at closed walls (0..1).
	double cfl = 0.25;                  ///< Safety factor of stableTimeStep().
	bool enableBuoyancy = true;
	bool enableAdiabatic = true;        ///< Dry-adiabatic cooling/heating with height change.
	bool enableCondensation = true;
	bool enableMixing = true;
	bool enablePressure = true;

	double effectLength() const { return effectLengthRatio * spacing; }
};

/** @brief Per-step diagnostics of the flow solve. */
struct CloudSolverDiagnostics {
	double maxSpeed = 0.0;         ///< [m/s]
	double maxDensityRatio = 1.0;  ///< max n_i / n0_i
	double minDensityRatio = 1.0;  ///< min n_i / n0_i
	double maxAcceleration = 0.0;  ///< [m/s^2]
	size_t adjustFailures = 0;     ///< Cumulative failed saturation adjustments.
	size_t wallClamps = 0;         ///< Cumulative positions pushed back into the domain.
};

/**
 * @brief CPU reference solver for the moist-air particles. Owns nothing but
 * scratch: the particle state lives in a CloudParticleSoA owned by the caller
 * (CloudWorld). Vulkan / ImGui independent.
 *
 * One step() = the plan's substep: neighbors+density -> pressure, viscosity,
 * buoyancy, wind -> integrate -> walls -> adiabatic p-work -> sources+mixing ->
 * saturation adjustment.
 */
class CloudSolver
{
public:
	CloudSolver() = default;

	CloudSolverParams params;
	CloudThermoParams thermo;
	CloudEnvironmentParams environment;

	/** @brief Records each particle's rest number density from the current configuration. Call after (re)filling the SoA. */
	void initialize(const CloudParticleSoA& soa);

	/** @brief Largest stable step from sound speed, advection, viscosity, mixing and buoyancy [s]. */
	double stableTimeStep(const CloudParticleSoA& soa) const;

	/**
	 * @brief Advances by dt. `time` is the simulation time at the start of the step
	 * (used by the source windows). Returns false and leaves the state untouched if
	 * dt <= 0 or the particle set is empty.
	 */
	bool step(CloudParticleSoA& soa, const std::vector<CloudSourceParams>& sources, double time,
	          double dt, CloudWaterLedger& ledger);

	const CloudSolverDiagnostics& diagnostics() const { return diag_; }
	void resetDiagnostics() { diag_ = CloudSolverDiagnostics(); }

private:
	double kernel(double r, double h) const;
	double kernelGradMag(double r, double h) const;  ///< dW/dr

	std::vector<double> restNumber_;   ///< n0_i
	std::vector<double> number_;       ///< n_i (scratch)
	std::vector<double> pressure_;     ///< p_i (scratch)
	std::vector<Math::Vector3dd> acc_; ///< scratch
	Space::CSRNeighborList neighbors_;
	CloudSolverDiagnostics diag_;
};

	}
}
