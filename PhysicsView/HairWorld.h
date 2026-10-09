#pragma once
#include "../Physics/HairSolver.h"
#include "../Physics/HairGenerator.h"
#include "../Physics/HairFollowers.h"
#include "SceneComponent.h"

namespace Phantom {

enum class HairPreset { Single, Bundle, Body, HeadShake, StrongWind, LongHair, ShortFur };

class HairWorld {
public:
    struct LodParams {
        bool enabled = false;
        float mediumDistance = 6.f;
        float farDistance = 12.f;
        float hysteresis = 0.5f;
        float transitionSeconds = 0.15f;
    };
    bool setLodParams(const LodParams& params);
    const LodParams& lodParams() const { return lodParams_; }
    bool setCameraDistance(float distance);
    int lodLevel() const { return lodLevel_; }
    float cameraDistance() const { return cameraDistance_; }
    int lodUpdateScale() const;
    size_t drawnFollowerCount() const;
    float lodTransitionProgress() const;
    struct CountParams {
        int longHairGuides = 48;
        int longHairFollowers = 384;
        int shortFurGuides = 96;
        int shortFurFollowers = 768;
    };
    // Staged for the next style preset. Guides: 1..10000, followers: 0..10000.
    bool setCountParams(const CountParams& params);
    const CountParams& countParams() const { return countParams_; }
    struct WireData {
        std::vector<float> positions;
        std::vector<float> colors;
        std::vector<uint32_t> indices;
    };
    ~HairWorld();
    void setComponentRegistry(SceneComponentRegistry* registry);
    bool setPreset(HairPreset preset);
    bool setGenerationParams(const Physics::HairVariationParams& params);
    const Physics::HairVariationParams& generationParams() const { return generationParams_; }
    void clear();
    void reset();
    bool stepOnce();
    bool update(double elapsedSeconds);
    void setRunning(bool running) { running_ = running; accumulator_ = 0.; }
    bool isRunning() const { return running_; }
    bool setParams(const Physics::HairSolver::Params& p);
    bool setRigPose(const Physics::HairRootPose& pose, bool teleport = false);
    const Physics::HairRootPose& rigPose() const { return rigPose_; }
    void setMotion(bool enabled) { motion_ = enabled; }
    bool motionEnabled() const { return motion_; }
    bool setFriction(float friction);
    float friction() const { return baseColliders_.empty() ? 0.f : baseColliders_[0].friction; }
    size_t colliderCount() const { return solver_.colliders().size(); }
    const Physics::HairSolver::Params& params() const { return solver_.params(); }
    const Physics::HairSolver::Stats& stats() const { return solver_.stats(); }
    const Physics::HairStrands& strands() const { return strands_; }
    const Physics::HairFollowers& followers() const { return followers_; }
    double droppedTime() const { return droppedTime_; }
    float tipY() const;
    float tipX() const;
    float maxRootError() const;
    WireData buildWireData() const;

private:
    LodParams lodParams_;
    int lodLevel_ = 0;
    float cameraDistance_ = 0.f;
    std::vector<Physics::HairStrandInput> fullRest_;
    std::vector<Physics::HairRootPose> followerRoots_;
    size_t followerVertexCount_ = 0;
    bool changeLod(int level);
    WireData canonicalWireData() const;
    bool advanceLodTransition(double seconds);
    WireData transitionFrom_;
    Physics::HairRootPose transitionRig_;
    double transitionElapsed_ = 0.;
    CountParams countParams_;
    Physics::HairVariationParams generationParams_; // applies on next generated preset
    Physics::HairStrands strands_; // must outlive solver_
    Physics::HairSolver solver_;
    Physics::HairFollowers followers_;
    bool running_ = false;
    double accumulator_ = 0.;
    double droppedTime_ = 0.;
    SceneComponentRegistry* registry_ = nullptr;
    int componentId_ = 0;
    std::vector<Physics::HairRootPose> baseRoots_;
    std::vector<Physics::HairCollider> baseColliders_;
    Physics::HairRootPose rigPose_, completedRig_;
    bool motion_ = false;
    bool advanceOnce(int timeScale = 1);
    bool applyRig(const Physics::HairRootPose& pose, bool teleport);
    Physics::HairRootPose animatedRig(double time) const;
    void removeComponent();
};
} // namespace Phantom
