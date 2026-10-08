#pragma once
#include "HairStrands.h"
#include "DistanceConstraint.h"
#include "CGLib/Util/UnCopyable.h"

namespace Phantom::Physics {

class HairSolver : private UnCopyable {
public:
    struct Params {
        float timeStep = 1.f / 60.f;
        int numSubsteps = 8;
        int numIterations = 8;
        float stretchCompliance = 0.f;
        float bendCompliance = 1.e-4f;
        float shapeCompliance = 0.02f;
        float dampingRate = 2.f; // per second; substep multiplier = exp(-rate * dt)
        bool shapeEnabled = true;
        Math::Vector3df gravity{0.f, -9.8f, 0.f};
    };
    struct Stats {
        double simulatedTime = 0.;
        uint64_t steps = 0;
        size_t particleCount = 0;
        size_t nonFiniteCount = 0;
        float maxSpeed = 0.f;
        float maxRelativeLengthError = 0.f;
    };

    // Non-owning. Caller must detach before changing topology or destroying data.
    bool setStrands(HairStrands* strands);
    bool setParams(const Params& params);
    const Params& params() const { return params_; }
    const Stats& stats() const { return stats_; }
    bool step();
    void reset();

private:
    HairStrands* strands_ = nullptr;
    uint64_t revision_ = 0;
    Params params_;
    Stats stats_;
    std::vector<DistanceConstraint> stretch_;
    std::vector<DistanceConstraint> bend_;
    std::vector<Math::Vector3df> shapeLambda_;
    std::vector<Math::Vector3df> targets_;
    void measure();
};

} // namespace Phantom::Physics
