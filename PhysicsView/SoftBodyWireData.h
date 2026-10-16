#pragma once

#include "../Core/SoftBodySolver.h"

#include <cstdint>
#include <vector>

namespace Phantom {

/**
 * @brief Line-list wireframe of a soft-body scene, in the flat float layout the wire renderer
 * uploads: xyz per vertex, rgba per vertex, and a trivial 0..n-1 index list.
 */
struct SoftBodyWireData {
    std::vector<float>    positions;
    std::vector<float>    colors;
    std::vector<uint32_t> indices;
};

/**
 * @brief Builds the display wireframe for `solver`: every body edge (green at rest length, red
 * stretched, blue compressed), the optional sphere collider (three great circles) and the box
 * outline of every rigid-body box collider.
 *
 * This is a view concern. It used to be SoftBodySolver::buildWireData() inside PhysicsCore, which
 * made the simulation library own colors and circle tessellation; the solver now only exposes the
 * data this reads (bodies(), params(), rigidColliderBodies()).
 */
SoftBodyWireData buildSoftBodyWireData(const Physics::SoftBodySolver& solver);

} // namespace Phantom
