#pragma once

#include "SoftParticle.h"
#include "CGLib/Math/Quaternion.h"
#include <cstdint>

namespace Phantom::Physics {

struct HairRootPose {
    Math::Vector3df position{0.f};
    Math::Quaternion rotation{1.f, 0.f, 0.f, 0.f};
};

struct HairStrandInput {
    HairRootPose root;
    // Root-local positions. The first point must be the local origin.
    std::vector<Math::Vector3df> restPositions;
    float inverseMass = 1.f;
};

struct HairStrandRange {
    size_t offset = 0;
    size_t count = 0;
    HairRootPose root;
};

// Caller-owned simulation data. Topology is immutable between initialize calls.
// Detach from HairSolver before reinitializing or destroying this object.
class HairStrands {
public:
    bool initialize(const std::vector<HairStrandInput>& inputs);
    // Restore resampled dynamic state before registering with a solver.
    // Invalid input preserves state; pinned roots must match their pose.
    bool setDynamicState(const std::vector<Math::Vector3df>& positions,
                         const std::vector<Math::Vector3df>& velocities);
    const SoftParticleSoA& particles() const { return particles_; }
    const std::vector<HairStrandRange>& ranges() const { return ranges_; }
    const std::vector<Math::Vector3df>& restPositions() const { return restPositions_; }
    size_t strandCount() const { return ranges_.size(); }
    size_t particleCount() const { return particles_.size(); }
    uint64_t revision() const { return revision_; }

private:
    friend class HairSolver;
    SoftParticleSoA particles_;
    std::vector<HairStrandRange> ranges_;
    std::vector<Math::Vector3df> restPositions_;
    uint64_t revision_ = 0;
};

} // namespace Phantom::Physics
