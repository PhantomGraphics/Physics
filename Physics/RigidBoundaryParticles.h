#pragma once

#include "ICollisionShape.h"
#include "SPHKernel.h"
#include "BoundaryParticle.h"
#include "IBoundaryParticles.h"
#include "CGLib/Math/Vector3d.h"
#include "CGLib/Math/Quaternion.h"

#include <vector>

namespace Phantom {
namespace Physics {

/**
 * @brief Samples a collision shape's surface into boundary particles for
 * Two-Way (Track B) rigid-fluid coupling.
 *
 * PlaneShape is unsupported: an infinite plane has no finite surface to
 * sample, so sample() leaves the particle list empty for it (Two-Way
 * coupling with planes is out of scope; use RigidBoundary's One-Way penalty
 * instead).
 */
class RigidBoundaryParticles : public IBoundaryParticles {
public:
    /**
     * @brief (Re)samples the shape's surface in its local rest frame.
     * @param shape   Collision shape to sample (Sphere: latitude/longitude
     *                rings; Box: a grid on each of the 6 faces; Plane: no-op).
     * @param spacing Target distance between neighboring sample points.
     */
    void sample(const ICollisionShape& shape, float spacing);

    /**
     * @brief Computes each particle's psi (Akinci et al. 2012, Eq. 3) from
     * the current (local-frame) sample positions. Since particles move
     * rigidly with the shape, pairwise distances -- and therefore psi -- do
     * not change after sync() and only need to be computed once.
     * @param kernel      SPH kernel used to weight neighboring samples.
     * @param restDensity Rest density of the fluid being coupled to.
     */
    void computePsi(const SPHKernel& kernel, float restDensity);

    /**
     * @brief Updates every particle's world-space position from the shape's
     * current pose.
     * @param pos    World-space position of the shape's origin.
     * @param orient World-space orientation of the shape.
     */
    void sync(const Math::Vector3df& pos, const Math::Quaternion& orient);

    /** @brief Returns the mutable particle list. */
    std::vector<BoundaryParticle>& particles() override { return particles_; }
    /** @brief Returns the read-only particle list. */
    const std::vector<BoundaryParticle>& particles() const override { return particles_; }

    /**
     * @brief Sample positions in the shape's local (rest-pose) frame, parallel
     * to particles() (localPositions()[i] is particles()[i]'s local position).
     * Rigid-specific: a SoftBody particle's world position already is its
     * state, so SoftBoundaryParticles has no local-frame equivalent.
     */
    const std::vector<Math::Vector3df>& localPositions() const { return localPositions_; }

    /** @brief Rigid samples are attached to the shape once at bind time; psi never changes afterward. */
    bool needsPsiEveryStep() const override { return false; }

private:
    std::vector<BoundaryParticle>  particles_;
    std::vector<Math::Vector3df>   localPositions_;

    void sampleSphere(float radius, float spacing);
    void sampleBox(const Math::Vector3df& halfExtents, float spacing);
};

} // namespace Physics
} // namespace Phantom
