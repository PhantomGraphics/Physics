#pragma once
#include "HairStrands.h"
#include <array>

namespace Phantom::Physics {

struct HairFollowerBinding {
    HairRootPose root; // initial world-space surface root
    std::array<size_t,3> guides{};
    std::array<float,3> weights{};
};

// Render-only strands. No mass, forces, constraints or simulation clock.
class HairFollowers {
public:
    // Choose up to three nearest guide roots once, using inverse-distance weights.
    // Roots and reference poses are copied. Invalid input preserves existing data.
    bool initialize(const HairStrands& guides, const std::vector<HairRootPose>& roots,
                    size_t particlesPerStrand);
    // Pass the same live guide object used by initialize. Reinitialize after any
    // topology change. The identity pointer is only compared, never dereferenced.
    // Positions are resampled at normalized strand coordinates, aligned to each
    // follower's root frame, and blended. Failure preserves the previous output.
    bool update(const HairStrands& guides);
    const std::vector<Math::Vector3df>& positions() const { return positions_; }
    const std::vector<HairFollowerBinding>& bindings() const { return bindings_; }
    size_t strandCount() const { return bindings_.size(); }
    size_t particleCount() const { return positions_.size(); }
    size_t particlesPerStrand() const { return particlesPerStrand_; }

private:
    const HairStrands* source_ = nullptr;
    uint64_t revision_ = 0;
    size_t sourceParticles_ = 0;
    size_t particlesPerStrand_ = 0;
    std::vector<HairRootPose> referenceRoots_;
    std::vector<HairFollowerBinding> bindings_;
    std::vector<Math::Vector3df> positions_, scratch_;
};
} // namespace Phantom::Physics
