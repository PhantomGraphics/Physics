#pragma once

#include "CGLib/Math/Vector3d.h"
#include "SPHKernel.h"

#include <vector>

namespace Phantom {
namespace Physics {

/**
 * @brief Computes the Akinci et al. 2012 (Eq. 3) pseudo-mass ("psi") for a
 * set of boundary-sample positions, enforcing the given rest density near
 * the boundary.
 *
 * psi[k] = restDensity / (kernel(0) + sum_{l != k} kernel(dist(k, l)))
 *
 * Shared by RigidBoundaryParticles (computed once, since rigid samples move
 * rigidly with their shape) and SoftBoundaryParticles (recomputed every
 * step, since soft-body meshes deform).
 *
 * @param positions   Boundary-sample positions (any consistent frame --
 *                     local or world -- since only pairwise distances matter).
 * @param kernel      SPH kernel used to weight neighboring samples.
 * @param restDensity Rest density of the fluid being coupled to.
 * @param psiOut      Resized to positions.size() and filled with each sample's psi.
 */
void computeBoundaryPsi(const std::vector<Math::Vector3df>& positions,
                         const SPHKernel& kernel,
                         float restDensity,
                         std::vector<float>& psiOut);

} // namespace Physics
} // namespace Phantom
