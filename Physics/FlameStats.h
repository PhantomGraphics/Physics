#pragma once

#include <array>
#include <string>

namespace Phantom {
	namespace Physics {
		class FlameFluid;

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
FlameStats computeFlameStats(const FlameFluid& fluid);

	}
}
