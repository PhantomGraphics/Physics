#pragma once

#include "CloudParams.h"

namespace Phantom {
	namespace Physics {

/** @brief Result of one saturation adjustment. */
struct CloudSaturationResult {
	double temperature = 0.0;
	double qv = 0.0;
	double qc = 0.0;
	double condensed = 0.0;  ///< Signed dq: >0 condensation, <0 evaporation [kg/kg].
	int iterations = 0;
	bool converged = true;
};

/**
 * @brief Moist-air thermodynamics for the Cloud subsystem. Pure functions in
 * double precision; no Vulkan / ImGui dependency.
 */
namespace CloudThermodynamics {

/** @brief Saturation vapour pressure over liquid water (Bolton 1980) [Pa]. */
double saturationVaporPressure(double temperature);
/** @brief d e_s / dT [Pa/K]. */
double saturationVaporPressureDerivative(double temperature);

/** @brief Saturation mixing ratio q_vs(T, p) [kg/kg dry air]. */
double saturationMixingRatio(const CloudThermoParams& th, double temperature, double pressure);

/** @brief Environment temperature at domain height z [m]. */
double environmentTemperature(const CloudEnvironmentParams& env, double z);
/** @brief Hydrostatic environment pressure at domain height z [m] [Pa]. */
double environmentPressure(const CloudThermoParams& th, const CloudEnvironmentParams& env, double z);
/** @brief Environment vapour mixing ratio at domain height z: RH * q_vs(Tenv, penv). */
double environmentVaporMixingRatio(const CloudThermoParams& th, const CloudEnvironmentParams& env, double z);

/** @brief Dry-adiabatic temperature change for a pressure change p0 -> p1 (Poisson). */
double adiabaticTemperature(const CloudThermoParams& th, double temperature, double p0, double p1);

/**
 * @brief Saturation adjustment at fixed pressure. Finds dq such that
 * qv - dq = q_vs(T + L/cp * dq), clamped to [-qc, qv] so cloud water never goes
 * negative and evaporation never exceeds the existing cloud water. qv + qc is
 * conserved by construction and cp*T + L*qv up to the iteration tolerance.
 * Non-finite or negative input returns converged=false and leaves the state unchanged.
 */
CloudSaturationResult saturationAdjust(const CloudThermoParams& th, double temperature,
                                       double qv, double qc, double pressure);

/** @brief Buoyancy acceleration [m/s^2] (+z up):
 * g * ((T - Tenv)/Tenv + (1/eps - 1)*(qv - qvEnv) - qc). */
double buoyancy(const CloudThermoParams& th, double temperature, double qv, double qc,
                double tEnv, double qvEnv);

}  // namespace CloudThermodynamics

	}
}
