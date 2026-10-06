#pragma once

#include <array>
#include <string>
#include "CombustibleBody.h"

namespace Phantom {
	namespace Physics {
		class FlameFluid;
		class FlameSolidCoupler;
		class FlameSolver;

/**
 * @brief Scalar summary of a FlameFluid's current particle state.
 *
 * Used by PhysicsView's Flame scenario commands (GetFlameStats /
 * GetFlameStat:<name>) and by PhysicsTest, so a regression in the flame's
 * overall shape (how high it rises, where it is hottest, whether it blew up)
 * can be asserted on numbers instead of only by eye
 * (docs/todo/PLAN_flame_sph_pbvr_improvement.md Phase 0).
 *
 * "Fuel" statistics cover non-air particles only (air carriers never ignite
 * and would otherwise dilute every temperature figure with ambient samples).
 */
struct FlameStats {
	double carrierMass = 0, totalHeat = 0, oxygenMass = 0, kineticEnergy = 0;
	double boundaryEnergyLoss = 0;
	float maxAttemptedWallPenetration = 0;
	double heatBalanceError = 0, oxygenBalanceError = 0, sourceHeat = 0, wallHeat = 0, clampHeat = 0;
	float allAvgT = 0, coreVelocityY = 0, outerVelocityY = 0, thermalVelocityCovariance = 0;
	float maxWallPenetration = 0, densityError = 0, speedCapFraction = 0;
	int coreCount = 0, outerCount = 0;
	double gasFuel = 0, burnedFuel = 0, outflowFuel = 0, sourceFuel = 0;
	double solidFuel = 0, pendingFuel = 0, residueMass = 0, pyrolyzedMass = 0, solidHeatExchange = 0;
	double initialSolidFuel = 0, removedSolidMass = 0, fuelBalanceError = 0;
	std::vector<CombustibleBodyStats> bodies;
	static constexpr int kHistogramBins = 8;

	int count = 0;          ///< All primary (SPH) particles.
	int airCount = 0;       ///< Of which ambient "air" carriers.
	int secondaryCount = 0; ///< Cosmetic spark/smoke particles.
	int nanCount = 0;       ///< Particles with a non-finite position/velocity/temperature.

	float avgY = 0.0f;      ///< Mean height of all primaries.
	float maxY = 0.0f;      ///< Highest primary.
	float airAvgY = 0.0f;   ///< Mean height of air carriers (0 if none).
	float avgSpeed = 0.0f;
	float maxSpeed = 0.0f;

	float avgT = 0.0f;      ///< Mean temperature of non-air particles.
	float maxT = 0.0f;      ///< Max temperature over all primaries.
	/**
	 * Mean height of the hottest 10% of non-air particles -- a robust "where is
	 * the flame hottest" probe (a single max-T particle is too noisy to assert on).
	 */
	float hotY = 0.0f;
	float avgFuel = 0.0f;   ///< Mean fuel over non-air particles.
	float avgSoot = 0.0f;   ///< Mean soot over all primaries.
	float avgOxygen = 0.0f; ///< Mean oxygen over all primaries.
	float burningFraction = 0.0f; ///< Fraction of primaries currently reacting (reaction rate > 0).

	/** Temperature histogram over [ambient, histMaxT] (non-air), kHistogramBins bins. */
	std::array<int, kHistogramBins> tempHistogram{};
	float histMinT = 0.0f;
	float histMaxT = 0.0f;

	/** @brief Returns one named scalar (see names()), or false for an unknown name. */
	bool get(const std::string& name, float& out) const;

	/** @brief Comma-separated list of the names get() accepts. */
	static const char* names();

	/** @brief One-line "key=value;..." summary (histogram as h=a/b/c/...). */
	std::string toString() const;
};

/** @brief Computes FlameStats over fluid's current primary + secondary particles. */
FlameStats computeFlameStats(const FlameFluid& fluid, const FlameSolidCoupler* coupler = nullptr,
	const FlameSolver* solver = nullptr);

	}
}
