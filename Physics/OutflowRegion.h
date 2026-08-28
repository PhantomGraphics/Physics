#pragma once

#include "CGLib/Math/Box3d.h"

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

	}
}
