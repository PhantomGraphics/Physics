#pragma once
#include "../Physics/HairSolver.h"
#include "../Physics/HairGenerator.h"
#include "SceneComponent.h"

namespace Phantom {

enum class HairPreset { Single, Bundle };

class HairWorld {
public:
    struct WireData {
        std::vector<float> positions;
        std::vector<float> colors;
        std::vector<uint32_t> indices;
    };
    ~HairWorld();
    void setComponentRegistry(SceneComponentRegistry* registry);
    bool setPreset(HairPreset preset);
    void clear();
    void reset();
    bool stepOnce();
    bool update(double elapsedSeconds);
    void setRunning(bool running) { running_ = running; accumulator_ = 0.; }
    bool isRunning() const { return running_; }
    bool setParams(const Physics::HairSolver::Params& p);
    const Physics::HairSolver::Params& params() const { return solver_.params(); }
    const Physics::HairSolver::Stats& stats() const { return solver_.stats(); }
    const Physics::HairStrands& strands() const { return strands_; }
    double droppedTime() const { return droppedTime_; }
    float tipY() const;
    float maxRootError() const;
    WireData buildWireData() const;

private:
    Physics::HairStrands strands_; // must outlive solver_
    Physics::HairSolver solver_;
    bool running_ = false;
    double accumulator_ = 0.;
    double droppedTime_ = 0.;
    SceneComponentRegistry* registry_ = nullptr;
    int componentId_ = 0;
    void removeComponent();
};
} // namespace Phantom
