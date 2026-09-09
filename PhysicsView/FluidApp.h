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

#include "../../CGLib/GltfRenderer/Gltf/GltfDocument.h"
#include "../../CGLib/GltfRenderer/Renderer/GltfSceneRenderer.h"
#include "../../CGLib/GltfRenderer/Renderer/ShadowMapPass.h"
#include "../../CGLib/VulkanGraphics/VulkanOffscreen.h"
#include "../../CGLib/VulkanGraphics/VulkanSampler.h"
#include "RenderBackground.h"
#include "RenderingPanel.h"
#include "GltfBodyRenderer.h"
#include "GltfSoftRenderer.h"

#include "../../CGLib/UIWidgets/MainMenuBar.h"
#include "FileMenu.h"
#include "PhysicsMenu.h"
#include "RenderingMenu.h"
#include "ToolsMenu.h"
#include "WindowMenu.h"
#include "ViewMenu.h"

#include "RigidBodyWireRenderer.h"
#include "RigidBodyControlPanel.h"

#include "SoftBodyWorld.h"
#include "SoftBodyWireRenderer.h"
#include "SoftBodyControlPanel.h"

#include "FlameWorld.h"
#include "FlameRenderer.h"
#include "FlameControlPanel.h"

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
    void onSwapChainDestroying() override;
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

    // glTF background/set pass (docs/todo/PLAN_physicsview_gltf_rendering.md):
    // one GltfSceneRenderer drawn on the shared FluidRenderer camera. Its
    // document + environment + shared directional light are owned by
    // renderBackground_ (which also holds the GltfDocument alive across
    // in-flight frames -- loadDocument() stores a pointer into it), driven by
    // CommandDispatcher and the "glTF Rendering" Control page.
    Phantom::Gltf::GltfSceneRenderer bgGltfRenderer_;
    RenderBackground                 renderBackground_;
    RenderingPanel                   renderingPanel_;

    // Phase 4: one depth-only shadow map for the shared directional light,
    // sampled by the background / rigid / soft glTF PBR passes. FluidApp owns it
    // (the sub-renderers only take its render-pass/view/sampler handles).
    Phantom::Gltf::ShadowMapPass     shadowPass_;

    // Phase 5: linear-HDR offscreen the opaque scene renders into; SSFR's
    // composite samples it, tonemaps (ACES) once, and writes the swapchain.
    Phantom::VKG::VulkanOffscreen    hdrScene_;
    Phantom::VKG::VulkanSampler      hdrSampler_;
    bool                             hdrValid_ = false;
    // Opaque "scene" sub-renderers, driven manually against hdrScene_'s render
    // pass (NOT via add()). Populated in the constructor.
    std::vector<::VKG::IVkSubRenderer*> hdrRenderers_;
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
    // Rigid-body PBR ("shaded") pass, an alternative to rigidRenderer_'s
    // wireframe (docs/todo/PLAN_physicsview_gltf_rendering.md Phase 2). One
    // GltfSceneRenderer + synthesized unit primitive per body; SetRigidRenderMode
    // / the "glTF Rendering" panel pick wire / shaded / both.
    GltfBodyRenderer      rigidGltfRenderer_;
    RigidBodyControlPanel rigidControlPanel_;

    // Independent SoftBody (cloth/rope/jelly) scene, added alongside fluid/
    // rigid without any physical coupling between the three (see
    // internal design notes) -- purely code
    // placement + minimal scenario-command wiring.
    SoftBodyWorld    softWorld_;
    SoftBodyWireRenderer  softRenderer_;
    // SoftBody PBR ("shaded") pass -- sibling of rigidGltfRenderer_
    // (docs/todo/PLAN_physicsview_gltf_rendering.md Phase 3). Cloth/Jelly get a
    // shaded surface (double-wound, per-frame CPU normals); Rope has no faces
    // and stays wireframe. SetSoftRenderMode / the "glTF Rendering" panel.
    GltfSoftRenderer      softGltfRenderer_;
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
    // The single shared "Control" window: the Physics menu picks its page,
    // this host renders the selected embedded panel plus a common status area.
    ControlPanelHost         controlHost_;
    FluidStatusView          statusView_;

    // Standalone windows (Window menu toggles).
    ObjectListPanel          objectListPanel_;

    // Main menu bar, assembled once in buildMenuBar().
    UI::MainMenuBar          menuBar_;
    FileMenu                fileMenu_;
    PhysicsMenu             physicsMenu_;
    RenderingMenu           renderingMenu_;
    ToolsMenu               toolsMenu_;
    WindowMenu              windowMenu_;
    ViewMenu                viewMenu_;

    bool exitOnComplete_ = true;
    int  exitCode_       = 0;

    bool        screenshotPending_ = false;
    std::string screenshotPendingPath_;

    void setupCallbacks();
    void registerControlPages();
    void buildMenuBar();
    // Tears the shared 3D scene (fluid + rigid + soft + glTF background) down to
    // nothing. Run at startup and from File > New.
    void newScene();
    void syncParticlesToRenderer();
    void syncGpuCsphBufferToRenderer();
    void syncBackgroundCamera();
    void refreshShadowLightVP();
    bool createHdrTargets();
    void destroyHdrTargets();
    void syncRigidRenderer();
    void syncSoftRenderer();
    void syncFlameRenderer();
    void syncVolumeRenderer();
    void syncMeshRenderer();
};

} // namespace Phantom
