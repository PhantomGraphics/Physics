#pragma once

#include <cstddef>

namespace Phantom {
	namespace Physics {

/**
 * @brief Running water budget: initial + supplied + inflow - outflow should
 * equal the current total (qv + qc summed over dry mass). Evaporation moves
 * water between qc and qv and never counts as a loss.
 */
struct CloudWaterLedger {
	double initialWater = 0.0;   ///< [kg]
	double suppliedWater = 0.0;  ///< [kg] vapour added by sources
	double inflowWater = 0.0;    ///< [kg] entering through open boundaries
	double outflowWater = 0.0;   ///< [kg] leaving through open boundaries
};

struct CloudStats {
	size_t particleCount = 0;
	double totalWater = 0.0;       ///< sum(mDry*(qv+qc)) [kg]
	double totalVapor = 0.0;       ///< [kg]
	double totalCloudWater = 0.0;  ///< [kg]
	double expectedWater = 0.0;    ///< initial + supplied + inflow - outflow [kg]
	double balanceError = 0.0;     ///< |total - expected| / max(expected, tiny)
	double cloudBase = 0.0;        ///< min z of particles with qc >= threshold (0 if none)
	double cloudTop = 0.0;         ///< max z of particles with qc >= threshold (0 if none)
	size_t cloudyParticles = 0;    ///< particles with qc >= threshold
	size_t nonFiniteCount = 0;     ///< particles with any non-finite state
	size_t negativeCount = 0;      ///< particles with qv<0, qc<0, mDry<=0 or T<=0
	size_t adjustFailures = 0;     ///< saturation adjustments that failed (accumulated by caller)
};

	}
}
