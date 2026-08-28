#pragma once

#include "RigidBody.h"

#include <utility>
#include <vector>

namespace Phantom {
namespace Physics {

/**
 * @brief Broad-phase overlap detection for rigid bodies, backed by Space::BVH.
 *
 * Mirrors NarrowPhase's static-utility convention. Builds a fresh Space::BVH
 * every call and returns index pairs into `bodies` (i < j), excluding
 * shape-less bodies and static-static pairs -- the same contract
 * RigidBodySolver's previous brute-force broadPhase() implemented.
 */
class BroadPhase {
public:
    /**
     * @param bodies   Non-owning pointers, one per body, index-aligned with
     *                 the caller's body storage (entries may be nullptr or
     *                 have shape == nullptr; both are skipped in place, so
     *                 the remaining indices are never remapped).
     * @param outPairs Cleared and filled with (i, j), i < j, overlapping,
     *                 not-both-static pairs. Indices refer to `bodies`.
     */
    static void detect(const std::vector<RigidBody*>& bodies,
                        std::vector<std::pair<int, int>>& outPairs);
};

} // namespace Physics
} // namespace Phantom
