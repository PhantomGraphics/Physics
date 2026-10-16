#pragma once
#include "HairStrands.h"
#include "DistanceConstraint.h"
#include "HairCollision.h"
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
        Math::Vector3df windVelocity{0.f}; // m/s
        float windDrag = 0.f; // velocity relaxation rate, per second
        float collisionRadius = 0.003f;
        float teleportDistance = 0.5f;
        float teleportAngle = 2.1f; // radians, shortest quaternion arc
    };
    struct Stats {
        double simulatedTime = 0.;
        uint64_t steps = 0;
        size_t particleCount = 0;
        size_t nonFiniteCount = 0;
        float maxSpeed = 0.f;
        float maxRelativeLengthError = 0.f;
        float maxPenetration = 0.f; // includes strand segments, excludes fixed-root point
        float pinnedPenetration = 0.f;
        uint64_t rootResets = 0;
    };

    // Non-owning. Caller must detach before changing topology or destroying data.
    // preserveClock retains simulated time, step count and root-reset count for LOD rebinding.
    bool setStrands(HairStrands* strands, bool preserveClock = false);
    bool setParams(const Params& params);
    // Target pose for the next step. Large jumps or teleport=true reset this
    // strand immediately, without resetting the simulation clock.
    bool setRootPose(size_t strand, const HairRootPose& pose, bool teleport = false);
    // Values are copied; same-sized updates interpolate endpoints per substep.
    // teleport=true discards boundary motion history (e.g. character teleport).
    bool setColliders(const std::vector<HairCollider>& colliders, bool teleport = false);
    const std::vector<HairCollider>& colliders() const { return colliders_; }
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
    std::vector<HairRootPose> previousRoots_;
    std::vector<HairCollider> colliders_, previousColliders_, stepColliders_, beforeColliders_;
    struct ContactVelocity {
        Math::Vector3df normal{0.f}, velocity{0.f};
        float correction = 0.f, friction = 0.f;
        bool active = false;
    };
    std::vector<ContactVelocity> contacts_;
    void updateTargets(float fraction);
    void resetStrand(size_t strand);
    void resolveCollisions(float dt);
    void measure();
};

} // namespace Phantom::Physics
