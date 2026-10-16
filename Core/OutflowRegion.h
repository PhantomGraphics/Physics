#pragma once

#include "CGLib/Math/Core/Box3d.h"

#include <vector>

namespace Phantom {
	namespace Physics {

/**
 * @brief An axis-aligned region that deletes fluid particles entering it.
 *
 * Shared data model used by WCSPHFluid/DFSPHFluid/PBSPHFluid's
 * removeOutflowParticles() -- the deletion counterpart to Emitter.h's
 * spawning regions. Purely opt-in: a fluid with no registered outflow
 * regions is completely unaffected (removeOutflowParticles() is a no-op),
 * so this is safe to leave unused for scenes that don't want particle
 * removal (e.g. a closed tank).
 */
struct OutflowRegion {
	Math::Box3df bounds;
};

/**
 * @brief Deletes every particle whose position lies inside any region (swap-and-pop, so the
 * order of the survivors is not preserved) -- the loop shared by the three fluids'
 * removeOutflowParticles(). A no-op for an empty region list.
 *
 * @tparam SoA Particle storage exposing `positions`, `size()` and `swapAndPop(i)`.
 */
template <class SoA>
void removeParticlesInRegions(const std::vector<OutflowRegion>& regions, SoA& particles)
{
	if (regions.empty()) {
		return;
	}

	for (size_t i = 0; i < particles.size();) {
		const Math::Vector3df& pos = particles.positions[i];
		bool remove = false;
		for (const auto& r : regions) {
			if (r.bounds.contains(pos, 0.0f)) {
				remove = true;
				break;
			}
		}

		if (remove) {
			particles.swapAndPop(i);
			continue;
		}
		++i;
	}
}
	}
}
