#include "pch.h"
#include "FluidApp.h"

#include <chrono>
#include "../Physics/CloudDensity.h"

#include "../../CGLib/VulkanGraphics/VulkanSPVResolver.h"

#include <algorithm>
#include <cmath>
#include "imgui.h"

namespace Phantom {

FluidApp::FluidApp(int width, int height, const std::string& title)
    : VkAppBase(width, height, title)
    , controlPanel_(&world_)
    , rigidControlPanel_(&world_.rigid())
    , softWorld_(world_.physicsSolver())
    , softControlPanel_(&softWorld_)
    , flameControlPanel_(&flameWorld_)
    , cloudControlPanel_(&cloudWorld_)
    , hairControlPanel_(&hairWorld_)
{
    // Every scene object registers itself into the shared registry: the fluid
    // + mesh boundary + emitters/outflow via FluidWorld, its rigid bodies via
    // the forwarded RigidBodyWorld, and the soft bodies via SoftBodyWorld.
    world_.setComponentRegistry(&sceneComponents_);
    softWorld_.setComponentRegistry(&sceneComponents_);
    hairWorld_.setComponentRegistry(&sceneComponents_);
    objectListPanel_.bind(&sceneComponents_);

    dispatcher_.setWorld(&world_);
    dispatcher_.setOnNewScene([this]() { newScene(); });
    dispatcher_.setOnWorldChanged([this]() {
        if (world_.getSimulationType() == FluidWorld::SimulationType::GPU_CSPH)
            syncGpuCsphBufferToRenderer();
        else
            syncParticlesToRenderer();
    });
    dispatcher_.setRigidWorld(&world_.rigid());
    dispatcher_.setOnRigidWorldChanged([this]() {
        // The rigid body list may have just changed (preset switch, AddSphere/
        // AddBox/AddFloor) -- re-bind if Rigid-Fluid coupling is active.
        world_.refreshCoupling();
        syncRigidRenderer();
    });
    world_.setSoftBodyWorld(&softWorld_);
    dispatcher_.setSoftWorld(&softWorld_);
    dispatcher_.setOnSoftWorldChanged([this]() {
        // The soft-body list may have just changed (preset switch) -- re-bind
        // if SoftBody-Fluid coupling is active.
        world_.refreshSoftCoupling();
        syncSoftRenderer();
    });
    scenarioBrowser_.setHost(this);
    scenarioBrowser_.setDefaultFolder("scenarios");

    controlPanel_.setOnWorldChanged([this]() { syncParticlesToRenderer(); });
    controlPanel_.setCommandSink([this](const std::string& cmd) { dispatcher_.submitUi(cmd); });
    rigidControlPanel_.setCommandSink([this](const std::string& cmd) { dispatcher_.submitUi(cmd); });
    softControlPanel_.setCommandSink([this](const std::string& cmd) { dispatcher_.submitUi(cmd); });
    rigidControlPanel_.setOnWorldChanged([this]() {
        world_.refreshCoupling();
        syncRigidRenderer();
    });
    softControlPanel_.setOnWorldChanged([this]() {
        world_.refreshSoftCoupling();
        syncSoftRenderer();
    });
    flameControlPanel_.setOnWorldChanged([this]() { syncFlameRenderer(); syncRigidRenderer(); });
    flameControlPanel_.setPBVRStatsSource([this] { return flameRenderer_.pbvrStats(); });
    ssfrTestPanel_.bindSSFRRenderer(&ssfrRenderer_);
    ssfrTestPanel_.init();
    ssfrPanel_.bindRenderer(&ssfrRenderer_);
    ssfrPanel_.bindWorld(&world_);
    // The former "SSFR Test" page is now a collapsible section of the SSFR page.
    ssfrPanel_.bindDebugPanel(&ssfrTestPanel_);
    ssfrPanel_.init();

    // glTF background / environment / shared light (PLAN_physicsview_gltf_rendering.md).
    renderBackground_.bind(&bgGltfRenderer_, &ssfrRenderer_);
    dispatcher_.setRenderBackground(&renderBackground_);
    // Rigid-/soft-body PBR ("shaded") rendering, Phase 2/3.
    rigidGltfRenderer_.bindWorld(&world_.rigid());
    dispatcher_.setRigidBodyRenderer(&rigidGltfRenderer_);
    softGltfRenderer_.bindWorld(&softWorld_);
    dispatcher_.setSoftBodyRenderer(&softGltfRenderer_);
    dispatcher_.setSsfrPanel(&ssfrPanel_);
    dispatcher_.setFluidRenderer(&fluidRenderer_);
    dispatcher_.setUIVisibilityHooks([this](bool v) { uiVisible_ = v; },
                                     [this] { return uiVisible_; });
    flameWorld_.setRigidWorld(&world_.rigid());
    flameWorld_.setCanBindRigid([this]{ return !world_.isCouplingEnabled(); });
    dispatcher_.flame().setWorld(&flameWorld_);
    dispatcher_.flame().setPageHooks(
        [this](bool on) {
            controlHost_.setPage(on ? ControlPage::Flame : ControlPage::Fluid);
            controlHost_.setVisible(true);
        },
        [this] { return controlHost_.getPage() == ControlPage::Flame; });
    dispatcher_.flame().setPBVRStatsHook([this] { return flameRenderer_.pbvrStats(); });
    dispatcher_.flame().setOnFlameChanged([this]() { syncFlameRenderer(); syncRigidRenderer(); });
    cloudControlPanel_.setOnWorldChanged([this]() { cloudDirty_ = true; });
    hairControlPanel_.setOnWorldChanged([this]() { syncHairRenderer(); });
    hairControlPanel_.setOnFrameGuides([this]() {
        fluidRenderer_.setCameraTarget({0.f, 0.9f, 0.f});
        fluidRenderer_.setCameraOrbit(5.f, 0.25f, 0.1f);
    });
    dispatcher_.hair().setWorld(&hairWorld_);
    dispatcher_.hair().setOnChanged([this]() { syncHairRenderer(); });
    dispatcher_.hair().setPageHooks(
        [this](bool on) {
            controlHost_.setPage(on ? ControlPage::Hair : ControlPage::Fluid);
            controlHost_.setVisible(true);
        },
        [this] { return controlHost_.getPage() == ControlPage::Hair; });
    dispatcher_.cloud().setWorld(&cloudWorld_);
    dispatcher_.cloud().setPageHooks(
        [this](bool on) {
            controlHost_.setPage(on ? ControlPage::Cloud : ControlPage::Fluid);
            controlHost_.setVisible(true);
        },
        [this] { return controlHost_.getPage() == ControlPage::Cloud; });
    dispatcher_.cloud().setOnCloudChanged([this]() { cloudDirty_ = true; });
    dispatcher_.cloud().setPbvrStatsHook([this]() -> std::optional<std::array<double, 3>> {
        if (!cloudVolumeRenderer_.isReady()) return std::nullopt;
        const auto s = cloudVolumeRenderer_.queryStats();
        return std::array<double, 3>{ static_cast<double>(s.accumulated), static_cast<double>(s.overflowed),
                                      static_cast<double>(s.generated) };
    });
    renderingPanel_.setCommandSink([this](const std::string& cmd) { dispatcher_.submitUi(cmd); });
    renderingPanel_.bind(&renderBackground_);
    renderingPanel_.bindRigidBodyRenderer(&rigidGltfRenderer_);
    renderingPanel_.bindSoftBodyRenderer(&softGltfRenderer_);
    renderingPanel_.init();

    volumeConvertPanel_.bindWorld(&world_);
    volumeConvertPanel_.bindConverter(&volumeConverter_);
    volumeConvertPanel_.bindMeshConverter(&meshConverter_);
    volumeConvertPanel_.bindVolumeRenderer(&volumeRenderer_);
    volumeConvertPanel_.bindMeshRenderer(&meshRenderer_);
    volumeConvertPanel_.setOnVolumeChanged([this]() { syncVolumeRenderer(); });
    volumeConvertPanel_.setOnMeshChanged([this]() { syncMeshRenderer(); });
    volumeConvertPanel_.init();
    dispatcher_.setVolumeConverter(&volumeConverter_);
    dispatcher_.setMeshConverter(&meshConverter_);
    dispatcher_.setVolumeRenderer(&volumeRenderer_);
    dispatcher_.setMeshRenderer(&meshRenderer_);
    dispatcher_.setOnVolumeChanged([this]() { syncVolumeRenderer(); });
    dispatcher_.setOnMeshChanged([this]() { syncMeshRenderer(); });

    // Background glTF first so its opaque geometry writes depth before the
    // fluid/particle passes draw over it (PLAN_physicsview_gltf_rendering.md
    // §3.4). SSFR still composites last, as before.
    // Phase 5 (PLAN_physicsview_gltf_rendering.md): the opaque scene renders into
    // a linear-HDR offscreen (hdrScene_) in onPreRender(), and SSFluidRenderer's
    // composite is the single final pass -- it samples that HDR color/depth,
    // composites the fluid surface (or just passes the scene through when SSFR is
    // off), applies ACES + exposure once, and writes the swapchain. So only the
    // SSFR composite stays a VkAppBase sub-renderer (its pipeline is built
    // against the swapchain render pass); every "scene" renderer is driven
    // manually against hdrScene_'s render pass.
    hdrRenderers_ = { &bgGltfRenderer_, &fluidRenderer_,
                      &rigidGltfRenderer_, &rigidRenderer_,
                      &softGltfRenderer_, &softRenderer_, &hairRenderer_,
                      &volumeRenderer_, &meshRenderer_, &combustibleRenderer_, &flameRenderer_, &flameDebugRenderer_,
                      &cloudVolumeRenderer_ };
    add(&ssfrRenderer_);

    // The per-domain control panels are no longer registered as standalone
    // UI panels -- they are embedded into controlHost_ (one shared "Control"
    // window, selected from the Physics menu), which is the only control-side
    // IVkUIPanel added here. This also prevents the double-draw described in
    // GUI_RESTRUCTURING_PLAN.md section 11.
    registerControlPages();
    statusView_.bind(&world_, &softWorld_, &runner_);
    controlHost_.setStatusView(&statusView_);
    // Remembers the last active page + window-visible flag across runs (the
    // window geometry and section fold state are handled by imgui.ini). The
    // layout file is loaded once in controlHost_.init() (from onInit(), after
    // main.cpp may have cleared it for a scenario run).
    controlHost_.setLayoutFile("physicsview_control_layout.ini");
    add(&controlHost_);
    add(&objectListPanel_);
    add(&scenarioBrowser_);

    // Standard screen: render area + menu + Command + Outliner. The existing windows keep
    // drawing themselves; the shell owns their visibility (hidden by default, imgui.ini)
    // and first-use placement.
    shell_.setDispatcher(&dispatcher_);
    shell_.bindPanel("Control", {0.70f, 0.00f, 0.30f, 0.66f},
                     [this] { return controlHost_.isVisible(); }, [this](bool v) { controlHost_.setVisible(v); });
    shell_.bindPanel("Scene Objects", {0.45f, 0.00f, 0.25f, 0.50f},
                     [this] { return objectListPanel_.isVisible(); }, [this](bool v) { objectListPanel_.setVisible(v); });
    shell_.bindPanel("Scenario Browser", {0.30f, 0.05f, 0.40f, 0.55f},
                     [this] { return scenarioBrowser_.isVisible(); }, [this](bool v) { scenarioBrowser_.setVisible(v); });
    shell_.setOutlinerProvider([this] {
        std::vector<ViewShell::OutlinerItem> items;
        for (const auto& c : sceneComponents_.components()) {
            const char* page = "Fluid";
            if (c.kind == SceneComponentKind::RigidBody) page = "RigidBody";
            else if (c.kind == SceneComponentKind::SoftBody) page = "SoftBody";
            else if (c.kind == SceneComponentKind::Hair) page = "Hair";
            items.push_back({static_cast<uint64_t>(c.id),
                             "#" + std::to_string(c.id) + " " + c.label + ": " + (c.describe ? c.describe() : std::string{}),
                             page});
        }
        return items;
    });
    // An outliner entry opens the page of its domain in the Control window.
    shell_.setOpenHandler([this](const std::string& page) {
        ControlPage p = ControlPage::Fluid;
        if (page == "RigidBody") p = ControlPage::RigidBody;
        else if (page == "SoftBody") p = ControlPage::SoftBody;
        else if (page == "Hair") p = ControlPage::Hair;
        controlHost_.setPage(p);
        controlHost_.setVisible(true);
    });

    buildMenuBar();
}

void FluidApp::onImGuiReady()
{
    // Context exists, imgui.ini is not read until the first frame.
    shell_.installSettings();
}

void FluidApp::buildMenuBar()
{
    // File > New: tear the 3D scene down to nothing (no fluid particles, no
    // rigid/soft bodies, no glTF background). This is the same routine onInit()
    // runs at startup -- presets, scenarios and dispatcher commands populate the
    // scene from an empty state.
    fileMenu_.build([this] { newScene(); }, [this] {
        glfwSetWindowShouldClose(getWindow().get(), GLFW_TRUE);
    });

    // One declarative entry per ControlPage. Rendering and tool pages are
    // routed to their dedicated top-level menus.
    for (int i = 0; i < static_cast<int>(kControlPageCount); ++i) {
        const auto page = static_cast<ControlPage>(i);
        const bool isRenderingPage =
            page == ControlPage::FluidRendering ||
            page == ControlPage::SSFR ||
            page == ControlPage::Rendering;
        const bool isToolsPage = page == ControlPage::VolumeConversion;
        DeclarativeMenu* targetMenu = isRenderingPage ? static_cast<DeclarativeMenu*>(&renderingMenu_)
                                    : isToolsPage ? static_cast<DeclarativeMenu*>(&toolsMenu_)
                                                  : static_cast<DeclarativeMenu*>(&physicsMenu_);

        targetMenu->build({{toString(page), [this, page] {
            controlHost_.setPage(page);
            controlHost_.setVisible(true);
        }, [this, page] {
            return controlHost_.getPage() == page && controlHost_.isVisible();
        }, [this, page] { return controlHost_.isPageEnabled(page); },
        [this, page]() -> std::string {
            return controlHost_.isPageEnabled(page)
                ? std::string{}
                : controlHost_.pageDisabledReason(page);
        }}});
    }

    windowMenu_.build({{"Outliner",
                [this] { shell_.setPanelVisible("Outliner", !shell_.isPanelVisible("Outliner")); },
                [this] { return shell_.isPanelVisible("Outliner"); }},

        {"Command",
                [this] { shell_.setPanelVisible("Command", !shell_.isPanelVisible("Command")); },
                [this] { return shell_.isPanelVisible("Command"); }},

        {"Control Window",
                [this] { controlHost_.setVisible(!controlHost_.isVisible()); },
                [this] { return controlHost_.isVisible(); }},

        {"Scene Objects",
                [this] { objectListPanel_.setVisible(!objectListPanel_.isVisible()); },
                [this] { return objectListPanel_.isVisible(); }},

        {"Scenario Browser", [this] {
        scenarioBrowser_.setVisible(!scenarioBrowser_.isVisible());
    }, [this] { return scenarioBrowser_.isVisible(); }},

        {"Reset Layout", [this] {
        shell_.resetLayout();
        controlHost_.setVisible(false);
    }} });

    viewMenu_.build({
        {"Camera XY", [this] { fluidRenderer_.viewXY(); }},
        {"Camera YZ", [this] { fluidRenderer_.viewYZ(); }},
        {"Camera ZX", [this] { fluidRenderer_.viewZX(); }},
        {"Camera Fit", [this] { fluidRenderer_.fitCamera(); }}
    });

    menuBar_.add(&fileMenu_);
    menuBar_.add(&viewMenu_);
    menuBar_.add(&physicsMenu_);
    menuBar_.add(&renderingMenu_);
    menuBar_.add(&toolsMenu_);
    menuBar_.add(&windowMenu_);
}

void FluidApp::registerControlPages()
{
    controlHost_.registerPage(ControlPage::Fluid,            &controlPanel_);
    controlHost_.registerPage(ControlPage::RigidBody,        &rigidControlPanel_);
    controlHost_.registerPage(ControlPage::SoftBody,         &softControlPanel_);
    controlHost_.registerPage(ControlPage::Flame,            &flameControlPanel_);
    controlHost_.registerPage(ControlPage::Cloud,            &cloudControlPanel_);
    controlHost_.registerPage(ControlPage::Hair,             &hairControlPanel_);
    controlHost_.registerPage(ControlPage::FluidRendering,   &fluidRenderer_);
    controlHost_.registerPage(ControlPage::SSFR,             &ssfrPanel_);
    controlHost_.registerPage(ControlPage::Rendering,        &renderingPanel_);
    controlHost_.registerPage(ControlPage::VolumeConversion, &volumeConvertPanel_);
}

bool FluidApp::loadScenario(const std::string& jsonPath) {
    return runner_.load(jsonPath);
}

void FluidApp::onInit()
{
    world_.setVulkanContext(getContext(), getCommandPool());

    fluidRenderer_.setExtent(getExtent());
    ssfrRenderer_.setExtent(getExtent());
    ssfrRenderer_.setParticleRadius(world_.params().radius);
    rigidRenderer_.setExtent(getExtent());
    softRenderer_.setExtent(getExtent());
    hairRenderer_.setExtent(getExtent());

    {
        FluidRenderer::Shaders s;
        s.vertSpv = ::VKG::loadSPVRepo("shaders/fluid_point.vert.spv");
        s.fragSpv = ::VKG::loadSPVRepo("shaders/fluid_point.frag.spv");
        fluidRenderer_.setShaders(std::move(s));
    }
    {
        RigidBodyWireRenderer::Shaders s;
        s.vertSpv = ::VKG::loadSPVRepo("shaders/line.vert.spv");
        s.fragSpv = ::VKG::loadSPVRepo("shaders/line.frag.spv");
        rigidRenderer_.setShaders(std::move(s));
        RigidBodyWireRenderer::Shaders debug;
        debug.vertSpv = ::VKG::loadSPVRepo("shaders/line.vert.spv");
        debug.fragSpv = ::VKG::loadSPVRepo("shaders/line.frag.spv");
        flameDebugRenderer_.setShaders(std::move(debug));
        flameDebugRenderer_.setExtent(getExtent());
    }
    {
        SoftBodyWireRenderer::Shaders s;
        s.vertSpv = ::VKG::loadSPVRepo("shaders/line.vert.spv");
        s.fragSpv = ::VKG::loadSPVRepo("shaders/line.frag.spv");
        softRenderer_.setShaders(std::move(s));
        SoftBodyWireRenderer::Shaders hairShaders;
        hairShaders.vertSpv = ::VKG::loadSPVRepo("shaders/line.vert.spv");
        hairShaders.fragSpv = ::VKG::loadSPVRepo("shaders/line.frag.spv");
        hairRenderer_.setShaders(std::move(hairShaders));
    }
    {
        VolumeRenderer::Shaders s;
        s.vertSpv = ::VKG::loadSPVRepo("shaders/point.vert.spv");
        s.fragSpv = ::VKG::loadSPVRepo("shaders/point.frag.spv");
        volumeRenderer_.setShaders(std::move(s));
    }
    {
        FluidMeshRenderer::Shaders s;
        s.vertSpv = ::VKG::loadSPVRepo("shaders/triangle.vert.spv");
        s.fragSpv = ::VKG::loadSPVRepo("shaders/triangle.frag.spv");
        meshRenderer_.setShaders(std::move(s));
    }
    {
        // Flame's three self-contained point-sprite pipelines (additive flame/
        // spark, alpha-blended smoke, unified opaque PBVR) -- see FlameRenderer.
        FlameRenderer::Shaders fs;
        fs.flameVert = ::VKG::loadSPVRepo("shaders/flame_point.vert.spv");
        fs.flameFrag = ::VKG::loadSPVRepo("shaders/flame_point.frag.spv");
        fs.smokeVert = ::VKG::loadSPVRepo("shaders/flame_smoke.vert.spv");
        fs.smokeFrag = ::VKG::loadSPVRepo("shaders/flame_smoke.frag.spv");
        fs.pbvr.pointVert      = ::VKG::loadSPVRepo("shaders/flame_pbvr_point.vert.spv");
        fs.pbvr.pointFrag      = ::VKG::loadSPVRepo("shaders/flame_pbvr_point.frag.spv");
        fs.pbvr.emissiveVert   = fs.flameVert;
        fs.pbvr.emissiveFrag   = fs.flameFrag;
        fs.pbvr.fullscreenVert = ::VKG::loadSPVRepo("shaders/flame_fullscreen.vert.spv");
        fs.pbvr.blendFrag      = ::VKG::loadSPVRepo("shaders/flame_pbvr_blend.frag.spv");
        fs.pbvr.compositeFrag  = ::VKG::loadSPVRepo("shaders/flame_pbvr_composite.frag.spv");
        fs.pbvr.generateComp   = ::VKG::loadSPVRepo("shaders/flame_pbvr_generate.comp.spv");
        fs.pbvr.finalizeComp   = ::VKG::loadSPVRepo("shaders/flame_pbvr_finalize.comp.spv");
        fs.pbvr.shadowVert     = ::VKG::loadSPVRepo("shaders/flame_pbvr_shadow.vert.spv");
        fs.pbvr.shadowFrag     = ::VKG::loadSPVRepo("shaders/flame_pbvr_shadow.frag.spv");
        fs.haze.fieldVert      = ::VKG::loadSPVRepo("shaders/flame_haze_field.vert.spv");
        fs.haze.fieldFrag      = ::VKG::loadSPVRepo("shaders/flame_haze_field.frag.spv");
        fs.haze.fullscreenVert = fs.pbvr.fullscreenVert;
        fs.haze.applyFrag      = ::VKG::loadSPVRepo("shaders/flame_haze_apply.frag.spv");
        flameRenderer_.setShaders(std::move(fs));
        flameRenderer_.setDepthFormat(getSwapChain().findDepthFormat().value_or(VK_FORMAT_D32_SFLOAT));

        CloudVolumeRenderer::Shaders cs;
        cs.fullscreenVert = ::VKG::loadSPVRepo("shaders/volume_fullscreen.vert.spv");
        cs.raymarchFrag = ::VKG::loadSPVRepo("shaders/volume_raymarch.frag.spv");
        cs.sunTransmittanceComp = ::VKG::loadSPVRepo("shaders/volume_sun_transmittance.comp.spv");
        cs.pbvrGenerateComp = ::VKG::loadSPVRepo("shaders/volume_pbvr_generate.comp.spv");
        cs.pbvrPointVert = ::VKG::loadSPVRepo("shaders/volume_pbvr_point.vert.spv");
        cs.pbvrPointFrag = ::VKG::loadSPVRepo("shaders/volume_pbvr_point.frag.spv");
        cs.pbvrAccumulateComp = ::VKG::loadSPVRepo("shaders/volume_pbvr_accumulate.comp.spv");
        cs.pbvrCompositeFrag = ::VKG::loadSPVRepo("shaders/volume_pbvr_composite.frag.spv");
        cloudVolumeRenderer_.setDepthFormat(getSwapChain().findDepthFormat().value_or(VK_FORMAT_D32_SFLOAT));
        cloudVolumeRenderer_.setShaders(std::move(cs));
    }
    {
        static constexpr auto kSS = "shaders/";
        Phantom::SSFluidRenderer::Shaders s;
        s.depthVert      = ::VKG::loadSPVRepo(std::string(kSS) + "ssfr_depth.vert.spv");
        s.depthFrag      = ::VKG::loadSPVRepo(std::string(kSS) + "ssfr_depth.frag.spv");
        s.thicknessVert  = ::VKG::loadSPVRepo(std::string(kSS) + "ssfr_thickness.vert.spv");
        s.thicknessFrag  = ::VKG::loadSPVRepo(std::string(kSS) + "ssfr_thickness.frag.spv");
        s.bilateralVert  = ::VKG::loadSPVRepo(std::string(kSS) + "ssfr_bilateral.vert.spv");
        s.bilateralFrag  = ::VKG::loadSPVRepo(std::string(kSS) + "ssfr_bilateral.frag.spv");
        s.reflectionVert = ::VKG::loadSPVRepo(std::string(kSS) + "ssfr_reflection.vert.spv");
        s.reflectionFrag = ::VKG::loadSPVRepo(std::string(kSS) + "ssfr_reflection.frag.spv");
        s.refractionVert = ::VKG::loadSPVRepo(std::string(kSS) + "ssfr_refraction.vert.spv");
        s.refractionFrag = ::VKG::loadSPVRepo(std::string(kSS) + "ssfr_refraction.frag.spv");
        s.compositeVert  = ::VKG::loadSPVRepo(std::string(kSS) + "ssfr_composite.vert.spv");
        s.compositeFrag  = ::VKG::loadSPVRepo(std::string(kSS) + "ssfr_composite.frag.spv");
        s.skyboxVert     = ::VKG::loadSPVRepo(std::string(kSS) + "skybox.vert.spv");
        s.skyboxFrag     = ::VKG::loadSPVRepo(std::string(kSS) + "skybox.frag.spv");
        s.anisoVert          = ::VKG::loadSPVRepo(std::string(kSS) + "ssfr_aniso.vert.spv");
        s.depthAnisoFrag     = ::VKG::loadSPVRepo(std::string(kSS) + "ssfr_depth_aniso.frag.spv");
        s.thicknessAnisoFrag = ::VKG::loadSPVRepo(std::string(kSS) + "ssfr_thickness_aniso.frag.spv");
        ssfrRenderer_.setShaders(std::move(s));
    }

    // glTF background pass (PLAN_physicsview_gltf_rendering.md). Only extent +
    // shaders are needed before the base onInit() runs each sub-renderer's
    // onInit() (VkAppBase.cpp:73); the app opens with no background loaded
    // (newScene() below), and LoadRenderBackground installs one later.
    bgGltfRenderer_.setExtent(getExtent());
    {
        Phantom::Gltf::GltfSceneRenderer::Shaders s;
        s.vertSpv = ::VKG::loadSPVRepo("shaders/gltf.vert.spv");
        s.fragSpv = ::VKG::loadSPVRepo("shaders/gltf.frag.spv");
        s.sampledLightFragSpv = ::VKG::loadSPVRepo("shaders/gltf_flame.frag.spv");
        s.shadowVertSpv = ::VKG::loadSPVRepo("shaders/shadow.vert.spv");
        s.shadowFragSpv = ::VKG::loadSPVRepo("shaders/shadow.frag.spv");
        bgGltfRenderer_.setShaders(std::move(s));
    }
    {
        Gltf::GltfSceneRenderer::Shaders s;
        s.vertSpv=::VKG::loadSPVRepo("shaders/combustible.vert.spv");
        s.fragSpv=::VKG::loadSPVRepo("shaders/combustible.frag.spv");
        s.sampledLightFragSpv=::VKG::loadSPVRepo("shaders/combustible_flame.frag.spv");
        combustibleRenderer_.setShaders(std::move(s));
        combustibleRenderer_.bind(&flameWorld_);
        combustibleRenderer_.setOnChanged([this] { if(!flameWorld_.isRunning()) flameRenderer_.notifySimulationAdvanced(true); });
        flameRenderer_.setOpaqueDraw([this](VkCommandBuffer cmd,uint32_t frame,VkDescriptorSet shadows,const Gltf::GltfSampledLight& light) {
            bgGltfRenderer_.renderSampledLight(cmd,frame,shadows,light);
            rigidGltfRenderer_.renderSampledLight(cmd,frame,shadows,light);
            softGltfRenderer_.renderSampledLight(cmd,frame,shadows,light);
            combustibleRenderer_.renderSampledLight(cmd,frame,shadows,light);
        });
    }
    // Rigid-/soft-body shaded pass: same gltf.{vert,frag} + shadow.{vert,frag};
    // each per-body GltfSceneRenderer instance gets its own copy (see
    // GltfBodyRenderer / GltfSoftRenderer).
    rigidGltfRenderer_.setShaders(::VKG::loadSPVRepo("shaders/gltf.vert.spv"),
                                  ::VKG::loadSPVRepo("shaders/gltf.frag.spv"));
    rigidGltfRenderer_.setSampledLightShader(::VKG::loadSPVRepo("shaders/gltf_flame.frag.spv"));
    rigidGltfRenderer_.setShadowShaders(::VKG::loadSPVRepo("shaders/shadow.vert.spv"),
                                        ::VKG::loadSPVRepo("shaders/shadow.frag.spv"));
    softGltfRenderer_.setShaders(::VKG::loadSPVRepo("shaders/gltf.vert.spv"),
                                  ::VKG::loadSPVRepo("shaders/gltf.frag.spv"));
    softGltfRenderer_.setSampledLightShader(::VKG::loadSPVRepo("shaders/gltf_flame.frag.spv"));
    softGltfRenderer_.setShadowShaders(::VKG::loadSPVRepo("shaders/shadow.vert.spv"),
                                       ::VKG::loadSPVRepo("shaders/shadow.frag.spv"));

    ::VKG::VkAppBase::onInit();   // onInits ssfrRenderer_ (composite -> swapchain) + UI panels
    setupCallbacks();

    // Phase 5: linear-HDR opaque-scene target. Every "scene" renderer is onInit'd
    // against hdrScene_'s render pass (NOT the swapchain one) and driven manually
    // from onPreRender(); SSFR's composite then samples it and writes the swapchain.
    if (createHdrTargets()) {
        for (auto* r : hdrRenderers_)
            r->onInit(getContext(), getCommandPool(), hdrScene_.getRenderPass(), MAX_FRAMES_IN_FLIGHT);
        flameRenderer_.resize(hdrScene_.getExtent().width, hdrScene_.getExtent().height);
        cloudVolumeRenderer_.resize(hdrScene_.getExtent().width, hdrScene_.getExtent().height);
        ssfrRenderer_.setSceneInput(hdrScene_.getColorImageView(), hdrScene_.getDepthImageView(),
                                    hdrSampler_.get(), 0.1f, 1000.f);
    } else {
        std::fprintf(stderr, "[FluidApp] HDR scene target creation failed; the viewport will be blank\n");
    }

    syncRigidRenderer();
    syncSoftRenderer();
    syncFlameRenderer();

    // Shadow map (Phase 4): one depth-only pass for the shared directional light,
    // sampled by the background / rigid / soft glTF PBR passes. Wired here (device
    // idle just after onInit); the light VP is refreshed per frame in onUpdate()
    // and the caster geometry recorded in onPreRender().
    shadowPass_.create(getContext(), 2048);
    if (shadowPass_.isValid()) {
        refreshShadowLightVP();
        bgGltfRenderer_.createShadowPipeline(shadowPass_.getRenderPass());
        bgGltfRenderer_.setShadowMap(shadowPass_.getDepthView(),
                                     shadowPass_.getShadowSampler(), shadowPass_.getLightVP());
        rigidGltfRenderer_.enableShadows(shadowPass_.getRenderPass(),
                                         shadowPass_.getDepthView(), shadowPass_.getShadowSampler());
        softGltfRenderer_.enableShadows(shadowPass_.getRenderPass(),
                                        shadowPass_.getDepthView(), shadowPass_.getShadowSampler());
    }

    // Load the Control-window layout (page + visible flag) and assemble its
    // widget tree now -- after main.cpp had its chance to clear the layout file
    // for a non-interactive scenario run (docs/todo/PLAN_physicsview_declarative_ui.md
    // Phase 2: load at init, save on change).
    controlHost_.init();

    // Load environment map after Vulkan is initialized
    static const std::array<std::string, 6> kFaceNames = {
        "right.png", "left.png", "top.png", "bottom.png", "front.png", "back.png"
    };
    // Shipped default environment: a compact studio with large light panels
    // and a cool window, giving water readable highlights without visual noise.
    static const std::string kEnvMapDir =
        (::VKG::detail::detectModuleDir() / "envmap").string();
    {
        std::array<std::string, 6> paths;
        bool ok = true;
        for (int i = 0; i < 6; ++i) {
            paths[i] = kEnvMapDir + "/" + kFaceNames[i];
            if (!std::filesystem::exists(paths[i])) { ok = false; break; }
        }
        if (ok) {
            ssfrRenderer_.loadEnvMap(paths);
        }
    }

    // Push the shared light now that every sub-renderer is onInit'd.
    // LoadRenderBackground / SetEnvironment / SetLight (CommandDispatcher) and the
    // "glTF Rendering" Control page drive the background/environment from here on.
    renderBackground_.setDefaultEnvDir(kEnvMapDir);
    renderBackground_.applyLight();

    // Start from an empty 3D scene -- no fluid particles, no rigid/soft bodies,
    // no glTF background. File > New runs this same routine; presets, scenarios
    // and dispatcher commands populate the scene from here.
    newScene();
}

void FluidApp::newScene()
{
    world_.newScene();          // drop the fluid solver -> 0 particles; clear
                                // emitters / outflow / sources / boundaries / coupling
    world_.rigid().clear();     // 0 rigid bodies (no preset, not even a floor)
    softWorld_.clear();         // 0 soft bodies
    hairWorld_.clear();
    hairLastTime_ = -1.;
    syncHairRenderer();
    flameWorld_.clear();        // fresh flame scene, stopped and out of the shared scene
    syncFlameRenderer();
    renderBackground_.clearBackground();  // release the glTF background document

    syncParticlesToRenderer();
    syncGpuCsphBufferToRenderer();
    syncRigidRenderer();
    syncSoftRenderer();
}

bool FluidApp::createHdrTargets()
{
    // Idempotent: run() calls onInit() then onSwapChainCreated() back to back
    // (and recreateSwapChain() calls onSwapChainDestroying() -> onSwapChainCreated()),
    // so this can be entered with a live target set. A recreated VulkanOffscreen
    // render pass stays render-pass-compatible with the one the scene-renderer
    // pipelines were built against (same attachment formats / sample counts), so
    // they need no rebuild.
    destroyHdrTargets();

    const VkExtent2D ext = getExtent();
    const VkFormat depthFmt = getSwapChain().findDepthFormat().value_or(VK_FORMAT_D32_SFLOAT);
    if (!hdrScene_.create(getContext(), ext.width, ext.height,
                          VK_FORMAT_R16G16B16A16_SFLOAT, depthFmt))
        return false;
    if (!hdrSampler_.create(getContext().getDevice(), VK_FILTER_NEAREST,
                            VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE)) {
        hdrScene_.destroy(getContext());
        return false;
    }
    hdrValid_ = true;
    return true;
}

void FluidApp::destroyHdrTargets()
{
    if (!hdrValid_) return;
    hdrSampler_.destroy(getContext().getDevice());
    hdrScene_.destroy(getContext());
    hdrValid_ = false;
}

void FluidApp::onSwapChainDestroying()
{
    // Device is idle here (VkAppBase::recreateSwapChain).
    destroyHdrTargets();
}

void FluidApp::onSwapChainCreated()
{
    const VkExtent2D ext = getExtent();
    fluidRenderer_.setExtent(ext);
    ssfrRenderer_.setExtent(ext);
    rigidRenderer_.setExtent(ext);
    flameDebugRenderer_.setExtent(ext);
    softRenderer_.setExtent(ext);
    hairRenderer_.setExtent(ext);
    bgGltfRenderer_.setExtent(ext);

    // Phase 5. run() calls onInit() (which builds the HDR target) then this,
    // back to back at the same size -- skip the redundant rebuild so the
    // scene-renderer pipelines onInit() just built are not left pointing at a
    // freed render pass. On a genuine resize, rebuild the HDR target + SSFR
    // offscreen targets, re-point SSFR at the fresh views, and hand the new
    // render-pass handle to the renderers that create pipelines lazily
    // (GltfBodyRenderer / GltfSoftRenderer make one per body on preset switch);
    // pipelines already built stay valid via render-pass compatibility.
    if (hdrValid_ && hdrScene_.getExtent().width == ext.width &&
        hdrScene_.getExtent().height == ext.height) {
        return;
    }
    if (createHdrTargets()) {
        rigidGltfRenderer_.setMainRenderPass(hdrScene_.getRenderPass());
        softGltfRenderer_.setMainRenderPass(hdrScene_.getRenderPass());
        ssfrRenderer_.resize(ext.width, ext.height);
        flameRenderer_.resize(ext.width, ext.height);
        cloudVolumeRenderer_.resize(ext.width, ext.height);
        ssfrRenderer_.setSceneInput(hdrScene_.getColorImageView(), hdrScene_.getDepthImageView(),
                                    hdrSampler_.get(), 0.1f, 1000.f);
    }
}

void FluidApp::onUpdate(uint32_t frameIndex)
{
    // Initial ImGuiCond_Once hints have run by frame 2. Verification can then
    // restore the edited ini without changing the shipped window defaults.
    if (!verificationLayout_.empty() && ImGui::GetFrameCount()>=2) {
        ImGui::LoadIniSettingsFromMemory(verificationLayout_.data(),verificationLayout_.size());
        verificationLayout_.clear();
    }
    dispatcher_.processQueue();
    const double hairNow = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const double hairDt = hairLastTime_ < 0. ? 0. : hairNow - hairLastTime_;
    hairLastTime_ = hairNow;
    if (hairWorld_.update(hairDt)) syncHairRenderer();

    // Keep the Scenario Browser's GUI run-queue advancing every frame, even
    // when its page is not the one currently shown in the Control window.
    scenarioBrowser_.pumpQueue();
    ssfrRenderer_.setParticleRadius(world_.params().radius);

    // Flame is a normal scene domain: it is drawn together with the fluid /
    // rigid / soft domains and the glTF background, whichever page is shown.
    // Opening its page (or Play / Step / Reset) puts it into the scene; File >
    // New takes it out again.
    const bool flameActive = (controlHost_.getPage() == ControlPage::Flame);
    if (flameActive) flameWorld_.setPopulated(true);
    const bool cloudActive = (controlHost_.getPage() == ControlPage::Cloud);

    const bool testActive = ssfrTestPanel_.isActive();

    if (testActive) {
        fluidRenderer_.clearDirectGpuBuffer();
        if (ssfrTestPanel_.consumeDirty()) {
            const auto& pts = ssfrTestPanel_.getPositions();
            fluidRenderer_.setParticles(pts);
            ssfrRenderer_.setParticles(pts);
            ssfrRenderer_.setSprayParticles({});
            ssfrRenderer_.setFoamParticles({});
            updateAnisotropicKernel(pts);
        } else if (ssfrPanel_.kernelGeneration() != appliedKernelGeneration_) {
            updateAnisotropicKernel(ssfrTestPanel_.getPositions());
        }
    } else {
        if (prevTestActive_)
            syncParticlesToRenderer();
        if (world_.getSimulationType() != FluidWorld::SimulationType::GPU_CSPH) {
            ssfrRenderer_.clearParticleBuffer();
        }

        const bool fluidRunning = world_.isRunning();
        const bool rigidRunning = world_.rigid().isRunning();
        const bool softRunning  = softWorld_.isRunning();

        // Steps whichever of fluid/rigid are running; if both are running and
        // Rigid-Fluid coupling is enabled, they step together in lock-step
        // (see FluidWorld::step()) instead of independently.
        flameWorld_.syncRigidBindings();
        world_.step();

        // softWorld_ is independent of world_ unless SoftBody-Fluid coupling
        // is enabled (see FluidWorld::setSoftCouplingEnabled()), in which
        // case world_.step() above already advanced it -- stepping it again
        // here would double-step it.
        if (softRunning && !world_.isSoftCouplingEnabled()) softWorld_.step();

        if (fluidRunning) {
            if (world_.getSimulationType() == FluidWorld::SimulationType::GPU_CSPH) {
                syncGpuCsphBufferToRenderer();
            } else {
                syncParticlesToRenderer();
            }
        } else if (ssfrPanel_.kernelGeneration() != appliedKernelGeneration_ &&
                   world_.getSimulationType() != FluidWorld::SimulationType::GPU_CSPH) {
            // Kernel toggled / retuned while paused: rebuild the ellipsoids
            // from the current (unchanged) particle positions.
            updateAnisotropicKernel(world_.getParticlePositions());
        }
        if (rigidRunning) {
            syncRigidRenderer();
        }
        if (softRunning) {
            syncSoftRenderer();
        }
    }
    prevTestActive_ = testActive;

    const bool useSSFR = testActive || ssfrPanel_.isEnabled();
    // Phase 5: SSFR's composite is the mandatory final pass whenever the HDR
    // path is active -- it samples the linear-HDR scene target, applies ACES +
    // exposure once, and writes the swapchain. That composite runs even with
    // SSFR "disabled" because setSceneInput() makes hasScene true (see
    // SSFluidRenderer::onRender): mode -1 = pure scene passthrough.
    //
    // enabled_ therefore gates only the *fluid-surface* work: the depth /
    // thickness / bilateral / reflection / refraction pre-passes (onPreRender)
    // and the surface composite mode. Turn it on only when there is actually a
    // fluid surface to reconstruct -- forcing it on with an empty particle set
    // still runs the whole pre-pass chain, and the bilateral pass reuses one
    // per-frame descriptor set across its ping-pong iterations, which trips
    // "descriptor set updated while bound" and invalidates the command buffer.
    // The pre-pass targets are transitioned to SHADER_READ_ONLY_OPTIMAL once at
    // init (initializeSSFRTargetLayouts), so the composite can sample them in
    // passthrough mode without the pre-pass having run this frame.
    ssfrRenderer_.setEnabled(useSSFR);
    fluidRenderer_.setEnabled(!useSSFR);
    ssfrRenderer_.setMode(static_cast<SSFluidRenderer::Mode>(ssfrPanel_.getModeIndex()));
    {
        // White-water spray/foam visibility follows the SSFRPanel checkboxes
        // (whiteWaterParams()) but is force-off for GPU_CSPH (no CPU white
        // water). Was a per-frame push inside SSFRPanel::drawContents();
        // moved here so it stays in sync regardless of the active page.
        const bool gpu = world_.getSimulationType() == FluidWorld::SimulationType::GPU_CSPH;
        const auto& ww = world_.whiteWaterParams();
        ssfrRenderer_.setShowSpray(gpu ? false : ww.enableSpray);
        ssfrRenderer_.setShowFoam(gpu ? false : ww.enableFoam);
    }
    // Flame steps and renders on every page (its own Play/Pause); the fluid
    // renderers above keep whatever mode they chose -- the flame is composited
    // into the same linear-HDR scene the SSFR composite tonemaps.
    const bool hadBindings=flameWorld_.hasRigidBindings();
    flameWorld_.syncRigidBindings();
    if(hadBindings && !flameWorld_.hasRigidBindings()) syncRigidRenderer();
    const bool flameInScene = flameWorld_.isPopulated();
    if (flameInScene) {
        if (flameWorld_.isRunning()) flameWorld_.step();
        syncFlameRenderer();
        if(flameWorld_.hasRigidBindings()) syncRigidRenderer();
    }
    flameRenderer_.setEnabled(flameInScene && !cloudActive);
    flameDebugRenderer_.setEnabled(flameInScene && !cloudActive && flameWorld_.fluid().fixedCarriers);
    // Cloud page: the air particles replace the fluid display in the shared
    // FluidRenderer (Phase 2 diagnostic view); every other scene renderer is off.
    if (cloudActive) {
        const double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        const double realDt = cloudLastTime_ < 0.0 ? 0.0 : std::min(now - cloudLastTime_, 0.1);
        cloudLastTime_ = now;
        cloudWorld_.ensureBuilt();
        cloudWorld_.update(realDt);
        if (world_.isRunning()) cloudDirty_ = true;   // the fluid sync above may have overwritten the particles
        syncCloudRenderer();
        ssfrRenderer_.setEnabled(false);
        fluidRenderer_.clearDirectGpuBuffer();
        fluidRenderer_.setEnabled(!cloudWorld_.render().volumeMode);
    } else {
        cloudLastTime_ = -1.0;
        if (prevCloudActive_) {
            // Leaving the page: put the fluid domain's own particles back.
            if (world_.getSimulationType() != FluidWorld::SimulationType::GPU_CSPH) {
                syncParticlesToRenderer();
            } else {
                syncGpuCsphBufferToRenderer();
            }
            cloudDirty_ = true;
        }
    }
    prevCloudActive_ = cloudActive;
    cloudVolumeRenderer_.setEnabled(cloudActive && cloudWorld_.render().volumeMode);
    const bool ownsViewport = cloudActive;
    hairRenderer_.setEnabled(!ownsViewport);
    // Rigid / soft body: wire (RigidBodyWireRenderer / SoftBodyWireRenderer) vs
    // shaded (GltfBodyRenderer / GltfSoftRenderer) vs both, per SetRigidRenderMode
    // / SetSoftRenderMode / the "glTF Rendering" panel. Flame still takes the
    // whole viewport.
    {
        const auto rm = rigidGltfRenderer_.mode();
        rigidRenderer_.setEnabled(!ownsViewport && bodyRenderModeWantsWire(rm));
        rigidGltfRenderer_.setEnabled(!ownsViewport && bodyRenderModeWantsShaded(rm));
        const auto sm = softGltfRenderer_.mode();
        softRenderer_.setEnabled(!ownsViewport && bodyRenderModeWantsWire(sm));
        softGltfRenderer_.setEnabled(!ownsViewport && bodyRenderModeWantsShaded(sm));
    }
    // Only the Cloud page owns the viewport; hide the glTF background there too,
    // matching rigid/soft/fluid.
    bgGltfRenderer_.setVisible(!ownsViewport);

    ssfrRenderer_.setCamera(fluidRenderer_.getProjMatrix(), fluidRenderer_.getViewMatrix());
    rigidRenderer_.setMVP(fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
    flameDebugRenderer_.setMVP(fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
    softRenderer_.setMVP(fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
    hairRenderer_.setMVP(fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
    volumeRenderer_.setMVP(fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
    meshRenderer_.setMVP(fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());

    // FluidRenderer is the single source of truth for the camera; feed the
    // same view/proj to the glTF background + shaded rigid bodies so they
    // compose with the fluid (plan §3.5). Runs before the base onUpdate() below
    // drives their sub-renderer onUpdate().
    syncBackgroundCamera();
    {
        const glm::mat4 view = fluidRenderer_.getViewMatrix();
        const glm::mat4 proj = fluidRenderer_.getProjMatrix();
        const glm::vec3 eye  = glm::vec3(glm::inverse(view)[3]);
        const glm::vec3 ld   = renderBackground_.lightDirection();
        const glm::vec4 lightDir(glm::normalize(ld), 0.0f);
        const glm::vec4 lightCol(renderBackground_.lightColor(), renderBackground_.lightIntensity());
        rigidGltfRenderer_.setCamera(view, proj, eye);
        combustibleRenderer_.setCamera(view,proj,eye);
        combustibleRenderer_.setLight(lightDir,lightCol);
        rigidGltfRenderer_.setLight(lightDir, lightCol);
        softGltfRenderer_.setCamera(view, proj, eye);
        softGltfRenderer_.setLight(lightDir, lightCol);
    }
    // Shadow light VP tracks the (possibly changed) light direction each frame
    // (Phase 4). The caster geometry is recorded in onPreRender().
    if (shadowPass_.isValid()) refreshShadowLightVP();

    if (auto path = dispatcher_.takePendingScreenshot()) {
        screenshotPendingPath_ = path->string();
        screenshotPending_     = true;
        requestScreenshot(screenshotPendingPath_);
    }
    if (screenshotPending_ && isScreenshotDone()) {
        dispatcher_.signalScreenshotDone(true, screenshotPendingPath_);
        screenshotPending_ = false;
    }

    // A command-line scenario run (exit-on-complete) hides the Scenario Browser: its default
    // position overlaps the Control window and ends up in --screenshot captures of every page.
    if (exitOnComplete_ && runner_.isActive()) scenarioBrowser_.setVisible(false);

    // Single place that collects responses: first the ones for commands typed
    // into the Command window (scenario commands included -- they run through
    // it too), the rest go to the running scenario. Deferred answers
    // (SaveScreenshot) arrive in a later frame and are matched by order.
    auto responses = dispatcher_.collectResponses();
    shell_.consumeResponses(responses);
    shell_.setScenarioActive(runner_.isActive());

    if (runner_.isActive()) {
        if (runner_.tick(shell_.scenarioDispatcher(), responses)) {
            if (runner_.hasFailed()) {
                fprintf(stderr, "[Scenario] FAILED: %s\n", runner_.failMessage().c_str());
                exitCode_ = 1;
            } else {
                fprintf(stdout, "[Scenario] PASSED (%zu steps)\n", runner_.stepCount());
                exitCode_ = 0;
            }
            if (exitOnComplete_) getWindow().close();
        }
    } else {
    }

    ::VKG::VkAppBase::onUpdate(frameIndex);   // ssfrRenderer_ + UI panels
    // Scene surfaces are part of the PBVR average too. Restart a paused mean
    // after edits; use temporal averaging while rigid/soft geometry is moving.
    const std::string sceneState=renderBackground_.sceneStateJson()+":"+
        std::to_string(renderBackground_.revision())+":"+
        std::to_string(static_cast<int>(rigidGltfRenderer_.mode()))+":"+
        std::to_string(static_cast<int>(softGltfRenderer_.mode()))+":"+
        std::to_string(rigidGltfRenderer_.isEnabled())+":"+
        std::to_string(softGltfRenderer_.isEnabled());
    if (sceneState!=pbvrSceneState_) {
        pbvrSceneState_=sceneState;
        flameRenderer_.notifySimulationAdvanced(true);
    } else if ((rigidGltfRenderer_.isEnabled() && world_.rigid().isRunning()) ||
               (softGltfRenderer_.isEnabled() && softWorld_.isRunning())) {
        flameRenderer_.notifySimulationAdvanced(false);
    }
    // Phase 5: the opaque scene renderers are no longer VkAppBase sub-renderers.
    for (auto* r : hdrRenderers_) r->onUpdate(frameIndex);
}

void FluidApp::onPreRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
    // GPU_CSPH + SSFR: barrier for compute-written posBuf used as vertex input
    if ((ssfrTestPanel_.isActive() || ssfrPanel_.isEnabled()) &&
        world_.getSimulationType() == FluidWorld::SimulationType::GPU_CSPH)
    {
        VkMemoryBarrier mb{};
        mb.sType         = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        mb.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        mb.dstAccessMask = VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT;
        vkCmdPipelineBarrier(cmd,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_VERTEX_INPUT_BIT,
            0, 1, &mb, 0, nullptr, 0, nullptr);
    }

    // Shadow-caster pass (Phase 4): depth-only, before the main render pass. The
    // pass always runs so the depth target is defined; when shadows are toggled
    // off the casters are simply not drawn (cleared depth = far = nothing
    // occluded). Cloud owns the viewport, so skip casters there.
    if (shadowPass_.isValid()) {
        const glm::mat4 vp = shadowPass_.getLightVP();
        shadowPass_.begin(cmd);
        if (renderBackground_.castShadows() && controlHost_.getPage() != ControlPage::Cloud) {
            bgGltfRenderer_.renderShadowCasters(cmd, vp);
            rigidGltfRenderer_.renderShadowCasters(cmd, vp);
            softGltfRenderer_.renderShadowCasters(cmd, vp);
        }
        shadowPass_.end(cmd);
    }

    // Phase 5: render the whole opaque scene into the linear-HDR target. SSFR's
    // pre-passes (below) sample its color + depth for refraction / occlusion,
    // and its composite (in the swapchain pass) tonemaps it once.
    // Flame PBVR (plan Phase 4): GPU particle generation + ensemble passes
    // run in their own offscreen targets before the HDR pass composites them.
    flameRenderer_.recordPreRender(cmd, frameIndex);   // no-op unless the flame is enabled in PBVR mode
    cloudVolumeRenderer_.recordPreRender(cmd, frameIndex);   // no-op unless the Cloud volume view is enabled

    if (hdrValid_) {
        const std::array<float, 4> clear{ 0.02f, 0.02f, 0.03f, 1.0f };
        // Heat haze: the opaque scene (everything but the flame) is drawn into the
        // haze pass's background target first; the HDR scene pass then starts with
        // a displaced copy of it and the flame is drawn on top.
        const bool haze = flameRenderer_.hazeActive();
        auto* const flameSub = static_cast<::VKG::IVkSubRenderer*>(&flameRenderer_);
        if (haze) {
            flameRenderer_.recordHazeField(cmd, frameIndex);
            flameRenderer_.beginHazeBackground(cmd, clear);
            for (auto* r : hdrRenderers_) if (r != flameSub) r->onRender(cmd, frameIndex);
            flameRenderer_.endHazeBackground(cmd);
        }
        hdrScene_.beginRenderPass(cmd, clear, 1.0f);
        if (haze) {
            flameRenderer_.applyHaze(cmd);
            flameSub->onRender(cmd, frameIndex);
        } else {
            for (auto* r : hdrRenderers_) r->onRender(cmd, frameIndex);
        }
        hdrScene_.endRenderPass(cmd);
    }

    ssfrRenderer_.onPreRender(cmd, frameIndex);
}

void FluidApp::onImGui()
{
    // Menu bar (File / Physics / Rendering / Tools / Window / View) is a widget tree assembled once in
    // buildMenuBar(); the common status area is FluidStatusView, embedded into
    // controlHost_. Nothing here re-assembles UI per frame.
    // SetUIVisible:false (scenario screenshots) hides every window.
    if (!uiVisible_) return;
    menuBar_.show();
    shell_.drawWindows();         // Command + Outliner, and syncs bound panel visibility
    ::VKG::VkAppBase::onImGui(); // the other windows draw themselves
    shell_.placeBoundPanels();
}

void FluidApp::onCleanup()
{
    // Must run before VkAppBase::onCleanup()'s caller (cleanup()) destroys
    // the VulkanContext -- see FluidWorld::releaseGpuResources()'s doc comment.
    world_.releaseGpuResources();
    ::VKG::VkAppBase::onCleanup();   // cleans up ssfrRenderer_ (the only sub-renderer left)

    // Phase 5: the opaque scene renderers are driven manually, so clean them up
    // manually too (device is idle -- cleanup() waited).
    VkDevice device = getContext().getDevice();
    for (auto* r : hdrRenderers_) r->onCleanup(device);

    // After the sub-renderers tore down their shadow pipelines (which reference
    // shadowPass_'s render pass). The VulkanContext is still alive here -- the
    // caller (cleanup()) destroys it afterwards.
    if (shadowPass_.isValid()) shadowPass_.destroy(getContext());
    destroyHdrTargets();
}

void FluidApp::setupCallbacks()
{
    auto& win = getWindow();

    // Camera input is ignored while ImGui owns the mouse and while a scenario
    // runs; a release is always forwarded so a drag can end.
    win.onMouseButton = [this](int button, int action, int) {
        if (button != GLFW_MOUSE_BUTTON_LEFT) {
            return;
        }
        if (action == GLFW_PRESS && (ImGui::GetIO().WantCaptureMouse || runner_.isActive()))
            return;

        double x = 0.0, y = 0.0;
        glfwGetCursorPos(getWindow().get(), &x, &y);
        fluidRenderer_.handleMouseButton(action == GLFW_PRESS,
                                         static_cast<float>(x),
                                         static_cast<float>(y));
        ssfrRenderer_.setCamera(fluidRenderer_.getProjMatrix(), fluidRenderer_.getViewMatrix());
        syncBackgroundCamera();
    };

    win.onCursorPos = [this](double x, double y) {
        fluidRenderer_.handleMouseMove(static_cast<float>(x), static_cast<float>(y));
        ssfrRenderer_.setCamera(fluidRenderer_.getProjMatrix(), fluidRenderer_.getViewMatrix());
        syncBackgroundCamera();
    };

    win.onScroll = [this](double, double dy) {
        if (ImGui::GetIO().WantCaptureMouse || runner_.isActive()) return;
        fluidRenderer_.handleScroll(static_cast<float>(dy));
        ssfrRenderer_.setCamera(fluidRenderer_.getProjMatrix(), fluidRenderer_.getViewMatrix());
        syncBackgroundCamera();
    };
}

void FluidApp::refreshShadowLightVP()
{
    // Frame the shared directional light on a point between the origin (where
    // every physics preset sits) and the shared camera target (20,20,20) with a
    // large ortho extent so both are covered. Kept consistent between the depth
    // render (renderShadowCasters) and the PBR sampling by routing both through
    // ShadowMapPass::getLightVP().
    const glm::vec3 dir = glm::normalize(renderBackground_.lightDirection());
    const glm::vec3 up  = (std::abs(dir.y) > 0.95f) ? glm::vec3(0.f, 0.f, 1.f)
                                                    : glm::vec3(0.f, 1.f, 0.f);
    const glm::vec3 center(12.f, 10.f, 12.f);
    const glm::mat4 view = glm::lookAt(center - dir * 100.f, center, up);
    glm::mat4 proj = glm::ortho(-55.f, 55.f, -55.f, 55.f, 0.1f, 260.f);
    proj[1][1] *= -1.f; // Vulkan Y flip (GLM_FORCE_DEPTH_ZERO_TO_ONE)
    shadowPass_.setLightViewProj(view, proj);

    const glm::mat4 vp = shadowPass_.getLightVP();
    bgGltfRenderer_.setShadowLightVP(vp);
    rigidGltfRenderer_.setShadowLightVP(vp);
    softGltfRenderer_.setShadowLightVP(vp);
}

void FluidApp::syncBackgroundCamera()
{
    // Eye is the translation column of inverse(view) (FluidStudio Phase 1's
    // technique) -- FluidRenderer exposes no eye getter.
    const glm::mat4 view = fluidRenderer_.getViewMatrix();
    const glm::mat4 proj = fluidRenderer_.getProjMatrix();
    const glm::vec3 eye  = glm::vec3(glm::inverse(view)[3]);
    bgGltfRenderer_.setCamera(view, proj, eye);
}

void FluidApp::syncRigidRenderer()
{
    flameRenderer_.notifySimulationAdvanced(!world_.rigid().isRunning() && !flameWorld_.isRunning());
    auto wd = world_.rigid().buildWireData();
    rigidRenderer_.update(wd.positions, wd.colors, wd.indices,
                           fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
    // Reconcile the shaded (glTF) instances with the current body set -- cheap
    // no-op when nothing changed (preset switch / AddSphere/AddBox/AddFloor all
    // funnel through onRigidWorldChanged -> here).
    rigidGltfRenderer_.syncFromWorld();
}

void FluidApp::syncSoftRenderer()
{
    flameRenderer_.notifySimulationAdvanced(!softWorld_.isRunning());
    auto wd = softWorld_.buildWireData();
    softRenderer_.update(wd.positions, wd.colors, wd.indices,
                          fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
    softGltfRenderer_.syncFromWorld();
}

void FluidApp::syncHairRenderer()
{
    // Invalidate flame PBVR accumulation when another visible domain moves.
    flameRenderer_.notifySimulationAdvanced(!hairWorld_.isRunning());
    const auto wd = hairWorld_.buildWireData();
    hairRenderer_.update(wd.positions, wd.colors, wd.indices,
        fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
}

void FluidApp::syncCloudRenderer()
{
    // Maps the cloud's z-up metres onto the shared FluidRenderer camera space
    // (y-up, ~40 units across): (x, y, z) m -> (x, z, y) * 0.04. Colour value =
    // cloud water in g/kg (0 = clear air stays neutral).
    const auto& soa = cloudWorld_.particles();
    const auto& dom = cloudWorld_.config().solver;
    const auto& rp = cloudWorld_.render();
    if (rp.volumeMode) {
        // Density grid (rebuilt when the sim advanced or a command changed the scene) + per-frame camera.
        if (cloudDirty_ || cloudSyncedStep_ != cloudWorld_.stepCount()) {
            cloudDirty_ = false;
            cloudSyncedStep_ = cloudWorld_.stepCount();
            const auto desc = Physics::CloudDensity::makeGridDesc(dom.domainMin, dom.domainMax, rp.gridResolution);
            Physics::CloudDensity::reconstruct(soa, desc, rp.supportScale * dom.spacing, cloudDensity_);
            cloudVolumeRenderer_.setDensity(cloudDensity_);
        }
        Volume::ScatteringParams sp;
        sp.extinction = static_cast<float>(rp.extinction);
        sp.albedo = static_cast<float>(rp.albedo);
        sp.phaseG = static_cast<float>(rp.phaseG);
        sp.sunDirection = rp.sunDirection();
        sp.sunIrradiance = static_cast<float>(rp.sunIrradiance);
        sp.ambient = static_cast<float>(rp.ambient);
        sp.stepLength = static_cast<float>(rp.stepScale) * cloudDensity_.desc().cellSize;
        sp.maxSteps = rp.maxSteps;
        cloudVolumeRenderer_.setScattering(sp);
        // cloud (x, y, z) m -> scene (x, z, y) * k, k = 40 / longest domain axis.
        const float k = 40.0f / static_cast<float>(std::max({ dom.domainMax.x, dom.domainMax.y, dom.domainMax.z }));
        const glm::mat4 cloudToScene(glm::vec4(k, 0, 0, 0), glm::vec4(0, 0, k, 0), glm::vec4(0, k, 0, 0), glm::vec4(0, 0, 0, 1));
        cloudVolumeRenderer_.setMode(rp.renderer == 1 ? CloudVolumeRenderer::Mode::Pbvr : CloudVolumeRenderer::Mode::Raymarch);
        {
            auto ps = cloudVolumeRenderer_.pbvrSettings();
            ps.ensemblesPerFrame = static_cast<uint32_t>(std::max(1, rp.pbvrEnsemblesPerFrame));
            ps.targetEnsembles = static_cast<uint32_t>(std::max(1, rp.pbvrTargetEnsembles));
            cloudVolumeRenderer_.setPbvrSettings(ps);
        }
        cloudVolumeRenderer_.setCamera(fluidRenderer_.getProjMatrix(), fluidRenderer_.getViewMatrix(), cloudToScene,
                                       hdrScene_.getExtent().height);
        return;
    }
    if (!cloudDirty_ && cloudSyncedStep_ == cloudWorld_.stepCount()) return;
    cloudDirty_ = false;
    cloudSyncedStep_ = cloudWorld_.stepCount();
    const float scale = 40.0f / static_cast<float>(std::max({ dom.domainMax.x, dom.domainMax.y, dom.domainMax.z }));
    std::vector<glm::vec3> pts(soa.size());
    std::vector<float> qc(soa.size());
    for (size_t i = 0; i < soa.size(); ++i) {
        const auto& p = soa.positions[i];
        pts[i] = glm::vec3(static_cast<float>(p.x), static_cast<float>(p.z), static_cast<float>(p.y)) * scale;
        qc[i] = static_cast<float>(soa.qc[i] * 1000.0);
    }
    fluidRenderer_.setParticles(pts, qc);
}

void FluidApp::syncFlameRenderer()
{
    // Maps the flame's particles onto FlameRenderer's two streams (plan
    // Phases 3-4), through the display transform (render().renderScale /
    // renderOffset) that puts the native ~3-unit plume into PhysicsView's
    // shared FluidRenderer camera space. The SPH state itself is untouched.
    //   emitters  = every SPH particle + sparks (blackbody radiance by temperature;
    //               cold ones contribute ~nothing, see FlameBlackbody::relativeRadiance)
    //   absorbers = smoke puffs (optical density = fade envelope * inherited soot)
    // Both render modes consume the same streams.
    const auto& fluid       = flameWorld_.fluid();
    const auto& render      = flameWorld_.render();
    const auto& particles   = fluid.getParticles();
    const auto& secondaries = fluid.getSecondaryParticles();

    const auto& sphere = flameWorld_.solver().getBoundarySphere();
    const auto& cylinder = flameWorld_.solver().getBoundaryCylinder();
    if ((sphere || cylinder) && fluid.fixedCarriers && !flameWasSpherical_) {
        const auto center=sphere?sphere->getCenter():cylinder->getCenter();
        const float extent=sphere?sphere->getRadius():std::max(cylinder->getRadius(),cylinder->getHalfHeight());
        fluidRenderer_.setCameraTarget(glm::vec3(center)*render.renderScale+render.renderOffset);
        fluidRenderer_.setCameraOrbit(extent*render.renderScale*3.3f,1.1f,1.35f);
    }
    flameWasSpherical_=(sphere.has_value() || cylinder.has_value()) && fluid.fixedCarriers;

    const bool coupled=!flameWorld_.bodies().empty();
    if(coupled && !flameWasCoupled_) {
        glm::vec3 center(0);
        for(const auto& b:flameWorld_.bodies()) center+=flameWorld_.physicalTransform().toScene(b->center());
        center/=static_cast<float>(flameWorld_.bodies().size());
        fluidRenderer_.setCameraTarget(center);
        fluidRenderer_.setCameraOrbit(18,1.1f,1.35f);
    }
    flameWasCoupled_=coupled;
    const float     scale  = coupled?flameWorld_.physicalTransform().scale:render.renderScale;
    const glm::vec3 offset = coupled?flameWorld_.physicalTransform().offset:render.renderOffset;
    const auto xf = [&](const Phantom::Math::Vector3df& p) {
        return glm::vec3(p.x, p.y, p.z) * scale + offset;
    };
    const float flameSize = render.particleSize * scale;
    const float smokeSize = render.smokeParticleSize * scale;

    if (fluid.fixedCarriers && (sphere || cylinder)) {
        std::vector<float> points, colors; std::vector<uint32_t> indices;
        const auto line = [&](glm::vec3 a,glm::vec3 b,glm::vec3 color) {
            const uint32_t index=static_cast<uint32_t>(points.size()/3);
            for (auto p : {a,b}) { points.insert(points.end(),{p.x,p.y,p.z}); colors.insert(colors.end(),{color.x,color.y,color.z,1.0f}); }
            indices.insert(indices.end(),{index,index+1});
        };
        if (render.sphereWire && sphere) for(int axis=0;axis<3;++axis) for(int j=0;j<96;++j) {
            const float a=j*6.2831853f/96, b=(j+1)*6.2831853f/96;
            glm::vec3 p(0),q(0); p[(axis+1)%3]=std::cos(a); p[(axis+2)%3]=std::sin(a);
            q[(axis+1)%3]=std::cos(b); q[(axis+2)%3]=std::sin(b);
            line(xf(sphere->getCenter()+p*sphere->getRadius()),xf(sphere->getCenter()+q*sphere->getRadius()),{0.2f,0.5f,0.7f});
        }
        if (render.sphereWire && cylinder) {
            const auto center=cylinder->getCenter(); const float r=cylinder->getRadius(),h=cylinder->getHalfHeight();
            for(float y:{-h,h}) for(int j=0;j<96;++j) {
                const float a=j*6.2831853f/96,b=(j+1)*6.2831853f/96;
                line(xf(center+glm::vec3(r*std::cos(a),y,r*std::sin(a))),
                     xf(center+glm::vec3(r*std::cos(b),y,r*std::sin(b))),{0.2f,0.5f,0.7f});
            }
            for(int j=0;j<4;++j) {
                const float a=j*6.2831853f/4;
                line(xf(center+glm::vec3(r*std::cos(a),-h,r*std::sin(a))),
                     xf(center+glm::vec3(r*std::cos(a),h,r*std::sin(a))),{0.2f,0.5f,0.7f});
            }
        }
        if(render.carrierDebug) for(size_t i=0;i<particles.size();i+=render.carrierDebug==2?4:1) {
            const auto p=xf(particles.positions[i]);
            const float t=std::clamp((particles.temperatures[i]-fluid.getAmbientTemperature())/1000,0.0f,1.0f);
            const glm::vec3 color(t,0.2f,1-t);
            const auto delta=render.carrierDebug==2?glm::vec3(particles.velocities[i])*scale*0.1f:glm::vec3(0,scale*0.01f,0);
            line(p,p+delta,color);
        }
        flameDebugRenderer_.update(points,colors,indices,fluidRenderer_.getProjMatrix()*fluidRenderer_.getViewMatrix());
    }

    std::vector<float> positions, temperatures, sizes;
    positions.reserve((particles.size() + secondaries.size()) * 3);
    temperatures.reserve(particles.size() + secondaries.size());
    sizes.reserve(particles.size() + secondaries.size());
    std::vector<float> smokePositions, smokeDensities, smokeSizes, smokeTemperatures;

    const auto pushEmitter = [&](const glm::vec3& p, float t, float s) {
        positions.push_back(p.x);
        positions.push_back(p.y);
        positions.push_back(p.z);
        temperatures.push_back(t);
        sizes.push_back(s);
    };

    float maxT = fluid.getAmbientTemperature();
    for (size_t i = 0; i < particles.size(); ++i) {
        const float t = particles.temperatures[i];
        if (!std::isfinite(t)) continue;
        maxT = std::max(maxT, t);
        if (!fluid.fixedCarriers || render.carrierDebug==0) pushEmitter(xf(particles.positions[i]), t, flameSize);
    }
    size_t emissionSamples=1;
    const auto& history=flameWorld_.emissionHistory();
    if (fluid.fixedCarriers && render.carrierDebug==0 && !render.pbvrMode && render.temporalBlurFrames>1 && !history.empty()) {
        emissionSamples=std::min(history.size(),static_cast<size_t>(render.temporalBlurFrames));
        // Current primaries were emitted above. Exclude the newest snapshot
        // so the latest state is counted exactly once.
        for(size_t frame=history.size()-emissionSamples;frame+1<history.size();++frame)
            for(size_t i=0;i<history[frame].positions.size();++i) {
                const float t=history[frame].temperatures[i];
                if(std::isfinite(t)) pushEmitter(xf(history[frame].positions[i]),t,flameSize);
            }
    }
    for (const auto& sp : secondaries) {
        if (sp.kind == Phantom::Physics::FlameFluid::SecondaryKind::Spark) {
            // Sparks have no SPH history. Replication cancels the common
            // shutter normalization without changing their appearance.
            for(size_t sample=0;sample<emissionSamples;++sample)
                pushEmitter(xf(sp.position), sp.temperature, flameSize * sp.size);
        } else {
            const glm::vec3 p = xf(sp.position);
            smokePositions.push_back(p.x);
            smokePositions.push_back(p.y);
            smokePositions.push_back(p.z);
            // sp.opacity is the 0..0.5 fade envelope (see updateSecondaryParticles()).
            smokeDensities.push_back(std::max(0.0f, 2.0f * sp.opacity * sp.soot * render.smokeOpacityScale));
            smokeSizes.push_back(smokeSize * sp.size);
            smokeTemperatures.push_back(sp.temperature);
        }
    }

    // Radiance reference temperature. Auto mode follows the hottest particle
    // with a frame EMA -- updated only while the sim advances, so a paused
    // frame keeps identical shading (the PBVR history would otherwise reset
    // every frame and never converge).
    const float simTime = flameWorld_.getSimTime();
    const bool animating = (simTime != flameLastSimTime_);
    // More than ~one step at once (FlameStep:N, FlameReset) is a jump, not motion.
    const bool jumped = std::abs(simTime - flameLastSimTime_) > 1.5f * flameWorld_.getTimeStep();
    if (simTime < flameLastSimTime_) {
        flameAutoRefT_ = render.referenceTemperature; // FlameReset
    }
    if (animating) {
        // Smooth frame-to-frame flicker; after a multi-step jump there is no
        // history worth keeping, so snap straight to the new values.
        const float k = jumped ? 1.0f : 0.1f;
        flameAutoRefT_ += k * (maxT - flameAutoRefT_);
    }
    flameLastSimTime_ = simTime;

    FlameRenderer::Shading shading;
    shading.ambientTemperature = fluid.getAmbientTemperature();
    shading.referenceTemperature = std::max(
        render.autoReferenceTemperature ? flameAutoRefT_ : render.referenceTemperature,
        shading.ambientTemperature + 200.0f);
    shading.exposure = render.exposure;
    shading.emissionWeight = 1.0f/static_cast<float>(emissionSamples);
    shading.whiteBalanceTemperature =
        // Auto adapts to the smoothed hottest temperature (the radiance
        // reference). Tried: the radiance-weighted mean temperature ("gray
        // world" on the emitted light) -- at ~1000-1200 K it is so far from
        // D65 that Bradford extrapolation + gamut clipping turned the halo
        // magenta. The same happens, milder, for any white much below
        // ~2000 K: von Kries/Bradford adaptation is validated down to about
        // incandescent illuminants, and extrapolating along the curved
        // Planckian locus beyond that tints the halo pink -- so clamp.
        render.whiteBalance == 1 ? std::max(shading.referenceTemperature, 2000.0f) :
        render.whiteBalance == 2 ? std::max(render.whiteBalanceTemperature, 500.0f) : 0.0f;
    shading.whiteBalanceDegree = std::clamp(render.whiteBalanceDegree, 0.0f, 1.0f);
    shading.smokeExtinction = render.smokeExtinction;
    shading.smokeDensityProfile = render.smokeDensityProfile;
    shading.smokeGlow = render.smokeGlow;
    shading.smokeAlbedo = glm::vec3(render.smokeAlbedo);
    shading.smokeShadowStrength = render.smokeShadowStrength;
    shading.smokeShadowAmbient = render.smokeShadowAmbient;
    shading.smokeFlameLight = render.smokeFlameLight;
    shading.objectFlameLight = render.objectFlameLight;
    shading.smokeLightDirection = renderBackground_.lightDirection();
    shading.smokeLightRadiance = renderBackground_.lightColor() * renderBackground_.lightIntensity();
    shading.pbvrSubdivision = render.pbvrSubdivision;
    shading.pbvrMinSubPixels = render.pbvrMinSubPixels;
    shading.pbvrDensityScale = render.pbvrDensityScale;

    auto& pbvr = flameRenderer_.pbvrSettings();
    pbvr.lodMode = render.pbvrAdaptive ? FlamePBVRPass::LodMode::Adaptive : FlamePBVRPass::LodMode::Manual;
    pbvr.ensemblesPerFrame = static_cast<uint32_t>(std::clamp(render.pbvrEnsembles, 1, 8));
    pbvr.targetEnsembles = static_cast<uint32_t>(std::max(1, render.pbvrTargetEnsembles));
    pbvr.temporalFrames = render.pbvrTemporalFrames;
    pbvr.budgetMs = render.pbvrBudgetMs;

    flameRenderer_.setEmitters(std::move(positions), std::move(temperatures), std::move(sizes));
    flameRenderer_.setAbsorbers(std::move(smokePositions), std::move(smokeDensities), std::move(smokeSizes),
                                std::move(smokeTemperatures));
    const auto ext = getExtent();
    flameRenderer_.setViewportHeight(ext.height > 0 ? static_cast<float>(ext.height) : 720.0f);
    flameRenderer_.setShading(shading);
    FlameHazePass::Settings haze;
    haze.enabled   = render.hazeEnabled;
    haze.strength  = render.hazeStrength;
    haze.extent    = render.hazeExtent;
    haze.frequency = render.hazeFrequency;
    haze.riseSpeed = render.hazeRiseSpeed;
    flameRenderer_.setHaze(haze, simTime);
    if (animating) flameRenderer_.notifySimulationAdvanced(jumped);
    flameRenderer_.setRenderMode(render.pbvrMode ? FlameRenderer::RenderMode::PBVR
                                                 : FlameRenderer::RenderMode::Normal);
    flameRenderer_.setCamera(fluidRenderer_.getProjMatrix(), fluidRenderer_.getViewMatrix());
}

void FluidApp::syncVolumeRenderer()
{
    const auto voxels = volumeConverter_.getVoxelPositions();

    std::vector<float> positions, colors, sizes;
    positions.reserve(voxels.size() * 3);
    colors.reserve(voxels.size() * 4);
    sizes.reserve(voxels.size());
    for (const auto& p : voxels) {
        positions.push_back(p.x);
        positions.push_back(p.y);
        positions.push_back(p.z);
        colors.push_back(0.3f);
        colors.push_back(0.6f);
        colors.push_back(1.0f);
        colors.push_back(0.8f);
        sizes.push_back(4.0f);
    }

    volumeRenderer_.update(positions, colors, sizes,
                            fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
}

void FluidApp::syncMeshRenderer()
{
    meshRenderer_.update(meshConverter_.getPositions(), meshConverter_.getColors(),
                          meshConverter_.getIndices(),
                          fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
}

void FluidApp::syncParticlesToRenderer()
{
    if (world_.getSimulationType() == FluidWorld::SimulationType::GPU_CSPH) {
        syncGpuCsphBufferToRenderer();
        return;
    }

    const auto fluidPositions = world_.getParticlePositions();
    const auto fluidDensities = world_.getParticleDensities();
    const auto spray = world_.getSprayPositions();
    const auto foam  = world_.getFoamPositions();

    ssfrRenderer_.setParticles(fluidPositions);
    ssfrRenderer_.setSprayParticles(spray);
    ssfrRenderer_.setFoamParticles(foam);
    updateAnisotropicKernel(fluidPositions);

    auto merged = fluidPositions;
    merged.insert(merged.end(), spray.begin(), spray.end());
    merged.insert(merged.end(), foam.begin(), foam.end());
    std::vector<float> densityDeviations;
    densityDeviations.reserve(merged.size());
    const float restDensity = world_.getActiveRestDensity();
    const float invRestDensity = restDensity > 0.0f ? 1.0f / restDensity : 0.0f;
    for (float density : fluidDensities) {
        densityDeviations.push_back((density - restDensity) * invRestDensity);
    }
    // White-water particles do not carry SPH density; use the neutral value.
    densityDeviations.resize(merged.size(), 0.0f);
    fluidRenderer_.setParticles(merged, densityDeviations);
}

void FluidApp::updateAnisotropicKernel(const std::vector<glm::vec3>& positions)
{
    appliedKernelGeneration_ = ssfrPanel_.kernelGeneration();
    ssfrPanel_.setKernelUnavailableReason({});
    // Keyed on the kernel flag alone (not on SSFR being enabled) so turning
    // SSFR on while paused already finds up-to-date ellipsoids.
    const bool wanted = ssfrRenderer_.getAnisotropicKernel() &&
                        ssfrRenderer_.supportsAnisotropicKernel();
    if (!wanted) {
        ssfrRenderer_.clearParticleEllipsoids();
        return;
    }
    const float radius = ssfrTestPanel_.isActive() ? ssfrRenderer_.getParticleRadius()
                                                   : world_.params().radius;
    ssfrPanel_.setKernelStats(anisotropyBuilder_.build(positions, radius, ssfrPanel_.kernelSettings(),
                                                       ellipsoidCenters_, ellipsoidAxes_));
    ssfrRenderer_.setParticleEllipsoids(ellipsoidCenters_, ellipsoidAxes_);
}

void FluidApp::syncGpuCsphBufferToRenderer()
{
    if (world_.getSimulationType() != FluidWorld::SimulationType::GPU_CSPH) {
        fluidRenderer_.clearDirectGpuBuffer();
        ssfrRenderer_.clearParticleBuffer();
        return;
    }

    auto* solver = world_.getGpuSolver();
    if (!solver) {
        fluidRenderer_.clearDirectGpuBuffer();
        ssfrRenderer_.clearParticleBuffer();
        return;
    }

    const VkBuffer   posBuf = solver->getPositionBuffer();
    const uint32_t   count  = static_cast<uint32_t>(solver->getNumParticles());

    fluidRenderer_.setDirectGpuBuffer(posBuf, count);

    // GPU_CSPH positions never reach the CPU, so the (CPU) anisotropic kernel
    // cannot run here yet; SSFR falls back to sphere sprites.
    ssfrRenderer_.clearParticleEllipsoids();
    ssfrPanel_.setKernelUnavailableReason("Unavailable for GPU_CSPH (sphere sprites)");
    appliedKernelGeneration_ = ssfrPanel_.kernelGeneration();

    if (ssfrPanel_.isEnabled() || ssfrTestPanel_.isActive()) {
        ssfrRenderer_.setParticleBuffer(posBuf, count);
    }

    ssfrRenderer_.setSprayParticles({});
    ssfrRenderer_.setFoamParticles({});
}

} // namespace Phantom
