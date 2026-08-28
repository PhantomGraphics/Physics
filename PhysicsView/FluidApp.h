#pragma once

#include "../../CGLib/VkAppBase/VkAppBase.h"
#include "../../CGLib/VkAppBase/ScenarioRunner/ScenarioRunner.h"
#include "../../CGLib/VkAppBase/ScenarioRunner/IScenarioHost.h"
#include "../../CGLib/VkAppBase/ScenarioRunner/ScenarioBrowserPanel.h"

#include "FluidRenderer.h"
#include "FluidWorld.h"
#include "ControlPanel.h"
#include "SSFRPanel.h"
#include "SSFRTestPanel.h"
#include "FluidVolumeConverter.h"
#include "FluidMeshConverter.h"
#include "VolumeRenderer.h"
#include "FluidMeshRenderer.h"
#include "FluidVolumeConvertPanel.h"
#include "CommandDispatcher.h"
#include "../FluidRenderer/SSFluidRenderer.h"

#include "RigidBodyWireRenderer.h"
#include "RigidBodyControlPanel.h"

#include "SoftBodyWorld.h"
#include "SoftBodyWireRenderer.h"
#include "SoftBodyControlPanel.h"

#include <filesystem>
#include <optional>

namespace Phantom {

class FluidApp : public ::VKG::VkAppBase, public ::IScenarioHost {
public:
    FluidApp(int width, int height, const std::string& title);

    FluidWorld& getWorld() { return world_; }
    FluidVolumeConverter& getVolumeConverter() { return volumeConverter_; }
    FluidMeshConverter& getMeshConverter() { return meshConverter_; }

    // Scenario runner control (call before run()).
    bool loadScenario(const std::string& jsonPath) override;
    void setExitOnScenarioComplete(bool v) override { exitOnComplete_ = v; }
    int  getExitCode() const               { return exitCode_; }

    // IScenarioHost (drives ScenarioBrowserPanel)
    bool   isScenarioActive()   const override { return runner_.isActive();   }
    bool   scenarioHasFailed()  const override { return runner_.hasFailed();  }
    const std::string& scenarioFailMessage() const override { return runner_.failMessage(); }
    size_t scenarioStepCount()  const override { return runner_.stepCount();  }

protected:
    void onInit() override;
    void onSwapChainCreated() override;
    void onUpdate(uint32_t frameIndex) override;
    void onPreRender(VkCommandBuffer cmd, uint32_t frameIndex) override;
    void onImGui() override;
    void onCleanup() override;

private:
    // Owns both the fluid and (via world_.rigid()) the rigid-body world,
    // plus their optional coupling (see docs/todo/PLAN_rigid_fluid_coupling.md
    // Phase 7/8).
    FluidWorld world_;

    FluidRenderer fluidRenderer_;
    SSFluidRenderer ssfrRenderer_;
    ControlPanel controlPanel_;
    SSFRPanel ssfrPanel_;
    SSFRTestPanel ssfrTestPanel_;
    bool prevTestActive_ = false;

    FluidVolumeConverter volumeConverter_;
    FluidMeshConverter meshConverter_;
    VolumeRenderer volumeRenderer_;
    FluidMeshRenderer meshRenderer_;
    FluidVolumeConvertPanel volumeConvertPanel_;

    RigidBodyWireRenderer rigidRenderer_;
    RigidBodyControlPanel rigidControlPanel_;

    // Independent SoftBody (cloth/rope/jelly) scene, added alongside fluid/
    // rigid without any physical coupling between the three (see
    // docs/todo/PLAN_softbody_physics_integration.md) -- purely code
    // placement + minimal scenario-command wiring.
    SoftBodyWorld    softWorld_;
    SoftBodyWireRenderer  softRenderer_;
    SoftBodyControlPanel  softControlPanel_;

    CommandDispatcher dispatcher_;
    ScenarioRunner           runner_;
    ScenarioBrowserPanel     scenarioBrowser_;
    bool exitOnComplete_ = true;
    int  exitCode_       = 0;

    bool        screenshotPending_ = false;
    std::string screenshotPendingPath_;

    void setupCallbacks();
    void syncParticlesToRenderer();
    void syncGpuCsphBufferToRenderer();
    void syncRigidRenderer();
    void syncSoftRenderer();
    void syncVolumeRenderer();
    void syncMeshRenderer();
};

} // namespace Phantom
