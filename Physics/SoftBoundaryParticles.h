#pragma once

#include "SPHKernel.h"
#include "SoftMesh.h"
#include "BoundaryParticle.h"
#include "IBoundaryParticles.h"
#include "CGLib/Math/Vector3d.h"

#include <vector>

namespace Phantom {
namespace Physics {

/**
 * @brief Exposes every particle of a SoftMesh as an Akinci-style boundary
 * particle set for Two-Way SoftBody-fluid coupling.
 *
 * SoftBody has no analytic collision shape (unlike RigidBody's
 * ICollisionShape), so there is no One-Way SDF-penalty equivalent
 * (RigidBoundary) here -- Two-Way boundary particles are the only coupling
 * path, mirroring RigidBoundaryParticles' "Track B".
 *
 * Every mesh particle is treated as a boundary particle (1:1, no sparse
 * sampling): correct for ClothBody/RopeBody, where every particle is on the
 * body's surface; an accepted approximation for JellyBody, whose
 * SoftMesh::faces is empty (no surface/interior distinction exists yet) --
 * interior particles couple with the fluid too, which reads as a slightly
 * *more* repulsive obstacle than the visual mesh, not a leaky one.
 *
 * Unlike a rigid shape, a SoftMesh deforms, so psi must be recomputed every
 * step (computePsi() is not a one-time bind-time cost like
 * RigidBoundaryParticles::computePsi()).
 */
class SoftBoundaryParticles : public IBoundaryParticles {
public:
    /** @brief Binds to a mesh, (re)sizing the particle list to mesh->particles.size(). Non-owning. */
    void bind(SoftMesh* mesh);

    /** @brief Copies worldPos from the bound mesh's current particle positions. */
    void sync();

    /**
     * @brief Recomputes every particle's psi (Akinci et al. 2012, Eq. 3)
     * from the current world positions. Call every step (after sync()),
     * since the mesh's pairwise distances change as it deforms.
     * @param kernel      SPH kernel used to weight neighboring samples.
     * @param restDensity Rest density of the fluid being coupled to.
     */
    void computePsi(const SPHKernel& kernel, float restDensity);

    /** @brief Returns the mutable particle list. */
    std::vector<BoundaryParticle>& particles() override { return particles_; }
    /** @brief Returns the read-only particle list. */
    const std::vector<BoundaryParticle>& particles() const override { return particles_; }

    /** @brief Returns the bound mesh (non-owning), or nullptr if bind() has not been called. */
    SoftMesh* mesh() const { return mesh_; }

    /** @brief A SoftMesh deforms every frame, so psi must be recomputed every step. */
    bool needsPsiEveryStep() const override { return true; }

private:
    SoftMesh* mesh_ = nullptr;
    std::vector<BoundaryParticle> particles_;
};

} // namespace Physics
} // namespace Phantom
