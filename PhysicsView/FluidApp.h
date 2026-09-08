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
#include "ControlPanelHost.h"
#include "IEmbeddedPanel.h"
#include "FluidStatusView.h"
#include "SceneComponent.h"
#include "ObjectListPanel.h"
#include "CommandDispatcher.h"
#include "../FluidRenderer/SSFluidRenderer.h"

#include "../../CGLib/UIWidgets/MainMenuBar.h"
#include "../../CGLib/UIWidgets/Menu.h"
#include "../../CGLib/UIWidgets/MenuItem.h"
#include "../../CGLib/UIWidgets/Separator.h"

#include "RigidBodyWireRenderer.h"
#include "RigidBodyControlPanel.h"

#include "SoftBodyWorld.h"
#include "SoftBodyWireRenderer.h"
#include "SoftBodyControlPanel.h"

#include "FlameWorld.h"
#include "FlameRenderer.h"
#include "FlameControlPanel.h"

#include <filesystem>
#include <list>
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
    void disableInteractiveLayoutPersistence() { controlHost_.setLayoutFile({}); }
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
    // ID-keyed list of every scene object; each world registers into it.
    // Declared first so it outlives the worlds that hold ids into it.
    SceneComponentRegistry sceneComponents_;

    // Owns both the fluid and (via world_.rigid()) the rigid-body world,
    // plus their optional coupling (see internal design notes
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
    // internal design notes) -- purely code
    // placement + minimal scenario-command wiring.
    SoftBodyWorld    softWorld_;
    SoftBodyWireRenderer  softRenderer_;
    SoftBodyControlPanel  softControlPanel_;

    // Flame (reacting hot-gas SPH), folded in from the former standalone
    // FlameView. A peer domain alongside fluid/rigid/soft with its own
    // Play/Pause/Step and control page, deliberately uncoupled from all three
    // (FlameSolver does not implement ISPHSolver -- see Physics/CLAUDE.md).
    FlameWorld        flameWorld_;
    FlameRenderer     flameRenderer_;
    FlameControlPanel flameControlPanel_;

    CommandDispatcher dispatcher_;
    ScenarioRunner           runner_;
    ScenarioBrowserPanel     scenarioBrowser_;
    // ScenarioBrowserPanel lives in CGLib and cannot derive from the
    // PhysicsView-local IEmbeddedPanel, so bridge it through a callable.
    FnEmbeddedPanel          scenarioBrowserEmbed_{
        [this]() { scenarioBrowser_.drawEmbedded(); } };

    // The single shared "Control" window: the Physics menu picks its page,
    // this host renders the selected embedded panel plus a common status area.
    ControlPanelHost         controlHost_;
    FluidStatusView          statusView_;

    // Standalone "Scene Objects" window (View menu toggle).
    ObjectListPanel          objectListPanel_;

    // Main menu bar, assembled once in buildMenuBar(). File / Physics / View.
    UI::MainMenuBar          menuBar_;
    UI::Menu                 fileMenu_    {"File"};
    UI::Menu                 physicsMenu_ {"Physics"};
    UI::Menu                 viewMenu_    {"View"};
    UI::Separator            physicsMenuSeparator_;
    // Quit + one item per ControlPage + "Control Window" -- non-copyable
    // widgets, so held in a node-stable list rather than an array.
    std::list<UI::MenuItem>  menuItems_;

    bool exitOnComplete_ = true;
    int  exitCode_       = 0;

    bool        screenshotPending_ = false;
    std::string screenshotPendingPath_;

    void setupCallbacks();
    void registerControlPages();
    void buildMenuBar();
    void syncParticlesToRenderer();
    void syncGpuCsphBufferToRenderer();
    void syncRigidRenderer();
    void syncSoftRenderer();
    void syncFlameRenderer();
    void syncVolumeRenderer();
    void syncMeshRenderer();
};

} // namespace Phantom
