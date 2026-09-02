#pragma once

#include "CGLib/Math/Vector3d.h"

namespace Phantom {
namespace Physics {

/**
 * @brief A single Akinci et al. 2012-style boundary sample point used for
 * Two-Way (Track B) fluid coupling.
 *
 * Shared by RigidBoundaryParticles and SoftBoundaryParticles (see
 * IBoundaryParticles) so DFSPHSolver/PBSPHSolver can couple against either
 * through one code path instead of duplicating it per boundary kind
 * (internal design notes, Phase 3).
 * RigidBoundaryParticles additionally keeps each sample's shape-local rest
 * position in a parallel array (RigidBoundaryParticles::localPositions()),
 * since that is rigid-specific (a SoftBody particle has no separate
 * local/world frame -- its world position *is* its state).
 */
struct BoundaryParticle {
    /** @brief Current world-space position, updated by sync(). */
    Math::Vector3df worldPos;
    /** @brief Pseudo-mass (Akinci et al. 2012) enforcing the rest density near the boundary. */
    float psi = 0.f;
    /** @brief Reaction force accumulated this step from all coupled fluid particles. */
    Math::Vector3df accumForce;
};

} // namespace Physics
} // namespace Phantom
