#include "pch.h"
#include "BroadPhase.h"

#include "CGLib/Space/Core/BVH.h"

namespace Phantom {
namespace Physics {

void BroadPhase::detect(const std::vector<RigidBody*>& bodies,
                         std::vector<std::pair<int, int>>& outPairs) {
    outPairs.clear();

    // storage must be reserved before any emplace_back/back(): Space::BVH keeps
    // raw BVHObject* pointers without copying, so a mid-loop reallocation here
    // would silently dangle every pointer already handed to objPtrs.
    std::vector<Space::BVHObject> storage;
    storage.reserve(bodies.size());
    std::vector<Space::BVHObject*> objPtrs;
    objPtrs.reserve(bodies.size());

    for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
        RigidBody* b = bodies[i];
        if (!b || !b->shape) continue;
        storage.emplace_back(i, b->getAABB());
        objPtrs.push_back(&storage.back());
    }

    if (objPtrs.size() < 2) return;

    // Per-thread work buffers: detect() is called every step, so the pair list and the traversal
    // stack keep their capacity instead of being reallocated.
    thread_local std::vector<std::pair<int, int>> bvhPairs;
    thread_local std::vector<std::pair<int, int>> bvhStack;

    Space::BVH bvh(objPtrs);
    bvh.findAllPairs(bvhPairs, bvhStack);
    for (auto& [ia, ib] : bvhPairs) {
        if (bodies[ia]->isStatic() && bodies[ib]->isStatic()) continue;
        outPairs.emplace_back(ia, ib);
    }
}

} // namespace Physics
} // namespace Phantom
