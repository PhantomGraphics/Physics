#pragma once

#include "CGLib/Math/Vector3d.h"

#include <cstdint>

namespace Phantom {
	namespace Physics {

/**
 * @brief Physical constants and limits for the moist-air thermodynamics of the
 * Cloud subsystem (docs/todo/PLAN_cloud_sph_pbvr.md section 4). SI units.
 * The CPU reference and the GPU port share this exact layout and meaning.
 */
struct CloudThermoParams {
	double gravity = 9.80665;         ///< [m/s^2]
	double gasConstantDry = 287.04;   ///< R_d [J/(kg K)]
	double cpDry = 1005.0;            ///< c_p [J/(kg K)]
	double latentHeat = 2.5e6;        ///< L_v [J/kg] (constant; adequate for warm clouds)
	double epsilon = 0.622;           ///< R_d / R_v
	int maxIterations = 32;           ///< Saturation adjustment Newton iteration cap.
	double relTolerance = 1.0e-12;    ///< Convergence on dq relative to max(qv, qc, 1e-12).
};

/**
 * @brief Background atmosphere: linear temperature profile with hydrostatic
 * pressure and a constant relative humidity. z is metres above the domain
 * floor; `altitudeOrigin` is the altitude above the reference level of z=0.
 */
struct CloudEnvironmentParams {
	double altitudeOrigin = 0.0;        ///< Altitude [m] of domain height 0.
	double surfaceTemperature = 300.0;  ///< T at altitude 0 [K].
	double surfacePressure = 1.0e5;     ///< p at altitude 0 [Pa].
	double lapseRate = 0.0065;          ///< -dT/dz [K/m]; must be > 0.
	double relativeHumidity = 0.6;      ///< Environment RH (0..1) w.r.t. saturation.
	Math::Vector3dd wind = Math::Vector3dd(0.0);  ///< Background wind [m/s].
};

/**
 * @brief Localized heat / vapour supply. Weights are a smooth ball falloff
 * over the *existing* air particles (no particles are created).
 */
struct CloudSourceParams {
	Math::Vector3dd center = Math::Vector3dd(0.0);
	double radius = 50.0;         ///< [m]
	double startTime = 0.0;       ///< simulation time [s]
	double endTime = 1.0e30;      ///< simulation time [s]
	double heatingRate = 0.0;     ///< Peak dT/dt at the centre [K/s].
	double vaporRate = 0.0;       ///< Total supplied vapour [kg water / s].
	bool enabled = true;
};

	}
}
