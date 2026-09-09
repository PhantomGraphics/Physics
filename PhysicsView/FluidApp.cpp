#include "pch.h"
#include "FluidApp.h"

#include "../../CGLib/VulkanGraphics/VulkanSPVResolver.h"

#include <random>

namespace Phantom {

namespace {

// Mirrors flame_point.frag's blackbody-ish gradient -- kept in sync with that
// shader since the Flame PBVR path pre-computes flame/spark colour on the CPU
// (see FlamePBVRPipeline's class doc comment). Ported verbatim from the former
// FlameView/FlameApp.cpp.
glm::vec3 flameColor(float temperature, float tMin, float tMax) {
    const float t = glm::clamp((temperature - tMin) / std::max(tMax - tMin, 1.0e-4f), 0.0f, 1.0f);
    const glm::vec3 cold(0.35f, 0.02f, 0.0f);
    const glm::vec3 mid(1.0f, 0.55f, 0.05f);
    const glm::vec3 hot(1.0f, 0.95f, 0.75f);
    glm::vec3 color = glm::mix(cold, mid, glm::clamp(t * 2.0f, 0.0f, 1.0f));
    color = glm::mix(color, hot, glm::clamp(t * 2.0f - 1.0f, 0.0f, 1.0f));
    return color;
}

// Mirrors flame_smoke.frag's ember-to-soot tint.
glm::vec3 smokeColorTint(float temperature, float tMin, float tMax) {
    const glm::vec3 smokeColor(0.22f, 0.20f, 0.18f);
    const float t = glm::clamp((temperature - tMin) / std::max(tMax - tMin, 1.0e-4f), 0.0f, 1.0f);
    const glm::vec3 emberGlow(1.0f, 0.45f, 0.12f);
    return glm::mix(smokeColor, emberGlow, t * t);
}

} // namespace

FluidApp::FluidApp(int width, int height, const std::string& title)
    : VkAppBase(width, height, title)
    , controlPanel_(&world_)
    , rigidControlPanel_(&world_.rigid())
    , softWorld_(world_.physicsSolver())
    , softControlPanel_(&softWorld_)
    , flameControlPanel_(&flameWorld_)
{
    // Every scene object registers itself into the shared registry: the fluid
    // + mesh boundary + emitters/outflow via FluidWorld, its rigid bodies via
    // the forwarded RigidBodyWorld, and the soft bodies via SoftBodyWorld.
    world_.setComponentRegistry(&sceneComponents_);
    softWorld_.setComponentRegistry(&sceneComponents_);
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
    rigidControlPanel_.setOnWorldChanged([this]() {
        world_.refreshCoupling();
        syncRigidRenderer();
    });
    softControlPanel_.setOnWorldChanged([this]() {
        world_.refreshSoftCoupling();
        syncSoftRenderer();
    });
    flameControlPanel_.setOnWorldChanged([this]() { syncFlameRenderer(); });
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
                      &softGltfRenderer_, &softRenderer_,
                      &volumeRenderer_, &meshRenderer_, &flameRenderer_ };
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

    buildMenuBar();
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

    windowMenu_.build({{"Control Window",
                [this] { controlHost_.setVisible(!controlHost_.isVisible()); },
                [this] { return controlHost_.isVisible(); }},

        {"Scene Objects",
                [this] { objectListPanel_.setVisible(!objectListPanel_.isVisible()); },
                [this] { return objectListPanel_.isVisible(); }},

        {"Scenario Browser", [this] {
        scenarioBrowser_.setVisible(!scenarioBrowser_.isVisible());
    }, [this] { return scenarioBrowser_.isVisible(); }} });

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
    }
    {
        SoftBodyWireRenderer::Shaders s;
        s.vertSpv = ::VKG::loadSPVRepo("shaders/line.vert.spv");
        s.fragSpv = ::VKG::loadSPVRepo("shaders/line.frag.spv");
        softRenderer_.setShaders(std::move(s));
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
        flameRenderer_.setShaders(
            ::VKG::loadSPVRepo("shaders/flame_point.vert.spv"),
            ::VKG::loadSPVRepo("shaders/flame_point.frag.spv"));
        flameRenderer_.setSmokeShaders(
            ::VKG::loadSPVRepo("shaders/flame_smoke.vert.spv"),
            ::VKG::loadSPVRepo("shaders/flame_smoke.frag.spv"));
        flameRenderer_.setPBVRShaders(
            ::VKG::loadSPVRepo("shaders/flame_pbvr.vert.spv"),
            ::VKG::loadSPVRepo("shaders/flame_pbvr.frag.spv"));
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
        s.shadowVertSpv = ::VKG::loadSPVRepo("shaders/shadow.vert.spv");
        s.shadowFragSpv = ::VKG::loadSPVRepo("shaders/shadow.frag.spv");
        bgGltfRenderer_.setShaders(std::move(s));
    }
    // Rigid-/soft-body shaded pass: same gltf.{vert,frag} + shadow.{vert,frag};
    // each per-body GltfSceneRenderer instance gets its own copy (see
    // GltfBodyRenderer / GltfSoftRenderer).
    rigidGltfRenderer_.setShaders(::VKG::loadSPVRepo("shaders/gltf.vert.spv"),
                                  ::VKG::loadSPVRepo("shaders/gltf.frag.spv"));
    rigidGltfRenderer_.setShadowShaders(::VKG::loadSPVRepo("shaders/shadow.vert.spv"),
                                        ::VKG::loadSPVRepo("shaders/shadow.frag.spv"));
    softGltfRenderer_.setShaders(::VKG::loadSPVRepo("shaders/gltf.vert.spv"),
                                 ::VKG::loadSPVRepo("shaders/gltf.frag.spv"));
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
    softRenderer_.setExtent(ext);
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
        ssfrRenderer_.setSceneInput(hdrScene_.getColorImageView(), hdrScene_.getDepthImageView(),
                                    hdrSampler_.get(), 0.1f, 1000.f);
    }
}

void FluidApp::onUpdate(uint32_t frameIndex)
{
    dispatcher_.processQueue();

    // Keep the Scenario Browser's GUI run-queue advancing every frame, even
    // when its page is not the one currently shown in the Control window.
    scenarioBrowser_.pumpQueue();
    ssfrRenderer_.setParticleRadius(world_.params().radius);

    // The Flame page takes over the viewport: its own point-sprite renderer is
    // the only one drawn, and the fluid/rigid/soft domains keep simulating in
    // the background exactly as they do while any other page is shown.
    const bool flameActive = (controlHost_.getPage() == ControlPage::Flame);

    const bool testActive = ssfrTestPanel_.isActive();

    if (testActive) {
        fluidRenderer_.clearDirectGpuBuffer();
        if (ssfrTestPanel_.consumeDirty()) {
            const auto& pts = ssfrTestPanel_.getPositions();
            fluidRenderer_.setParticles(pts);
            ssfrRenderer_.setParticles(pts);
            ssfrRenderer_.setSprayParticles({});
            ssfrRenderer_.setFoamParticles({});
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
    if (flameActive) {
        if (flameWorld_.isRunning()) flameWorld_.step();
        syncFlameRenderer();
        fluidRenderer_.setEnabled(false);
        // Flame owns the viewport. SSFR's fluid-surface work stays off; when the
        // HDR path is active its composite still runs (hasScene) and passes the
        // HDR scene (= flame) through with the shared ACES tonemap.
        ssfrRenderer_.setEnabled(false);
    }
    flameRenderer_.setEnabled(flameActive);
    // Rigid / soft body: wire (RigidBodyWireRenderer / SoftBodyWireRenderer) vs
    // shaded (GltfBodyRenderer / GltfSoftRenderer) vs both, per SetRigidRenderMode
    // / SetSoftRenderMode / the "glTF Rendering" panel. Flame still takes the
    // whole viewport.
    {
        const auto rm = rigidGltfRenderer_.mode();
        rigidRenderer_.setEnabled(!flameActive && bodyRenderModeWantsWire(rm));
        rigidGltfRenderer_.setEnabled(!flameActive && bodyRenderModeWantsShaded(rm));
        const auto sm = softGltfRenderer_.mode();
        softRenderer_.setEnabled(!flameActive && bodyRenderModeWantsWire(sm));
        softGltfRenderer_.setEnabled(!flameActive && bodyRenderModeWantsShaded(sm));
    }
    // The Flame page owns the viewport (see the flameActive comment above);
    // hide the glTF background there too, matching rigid/soft/fluid.
    bgGltfRenderer_.setVisible(!flameActive);

    ssfrRenderer_.setCamera(fluidRenderer_.getProjMatrix(), fluidRenderer_.getViewMatrix());
    rigidRenderer_.setMVP(fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
    softRenderer_.setMVP(fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
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

    if (runner_.isActive()) {
        auto responses = dispatcher_.collectResponses();
        if (runner_.tick(dispatcher_, responses)) {
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
    // occluded). Flame owns the viewport, so skip casters there too.
    const bool flameActive = (controlHost_.getPage() == ControlPage::Flame);
    if (shadowPass_.isValid()) {
        const glm::mat4 vp = shadowPass_.getLightVP();
        shadowPass_.begin(cmd);
        if (renderBackground_.castShadows() && !flameActive) {
            bgGltfRenderer_.renderShadowCasters(cmd, vp);
            rigidGltfRenderer_.renderShadowCasters(cmd, vp);
            softGltfRenderer_.renderShadowCasters(cmd, vp);
        }
        shadowPass_.end(cmd);
    }

    // Phase 5: render the whole opaque scene into the linear-HDR target. SSFR's
    // pre-passes (below) sample its color + depth for refraction / occlusion,
    // and its composite (in the swapchain pass) tonemaps it once.
    if (hdrValid_) {
        const std::array<float, 4> clear{ 0.02f, 0.02f, 0.03f, 1.0f };
        hdrScene_.beginRenderPass(cmd, clear, 1.0f);
        for (auto* r : hdrRenderers_) r->onRender(cmd, frameIndex);
        hdrScene_.endRenderPass(cmd);
    }

    ssfrRenderer_.onPreRender(cmd, frameIndex);
}

void FluidApp::onImGui()
{
    // Menu bar (File / Physics / Rendering / Tools / Window / View) is a widget tree assembled once in
    // buildMenuBar(); the common status area is FluidStatusView, embedded into
    // controlHost_. Nothing here re-assembles UI per frame.
    menuBar_.show();
    ::VKG::VkAppBase::onImGui();
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

    win.onMouseButton = [this](int button, int action, int) {
        if (button != GLFW_MOUSE_BUTTON_LEFT) {
            return;
        }

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
    auto wd = softWorld_.buildWireData();
    softRenderer_.update(wd.positions, wd.colors, wd.indices,
                          fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
    softGltfRenderer_.syncFromWorld();
}

void FluidApp::syncFlameRenderer()
{
    // Direct port of the former FlameView::FlameApp::uploadParticlesToRenderer(),
    // with one addition: every particle position is mapped through the display
    // transform (render().renderScale / renderOffset) so the native ~3-unit
    // plume composes with PhysicsView's shared FluidRenderer camera. The SPH
    // state itself is untouched.
    const auto& fluid       = flameWorld_.fluid();
    const auto& render      = flameWorld_.render();
    const auto& particles   = fluid.getParticles();
    const auto& secondaries = fluid.getSecondaryParticles();
    const float tMin = fluid.getAmbientTemperature();
    const float tMax = fluid.getIgnitionTemperature();

    const float     scale  = render.renderScale;
    const glm::vec3 offset = render.renderOffset;
    const auto xf = [&](const Phantom::Math::Vector3df& p) {
        return glm::vec3(p.x, p.y, p.z) * scale + offset;
    };

    if (render.pbvrMode) {
        std::vector<float> pbvrPositions;
        std::vector<float> pbvrColors;
        std::vector<float> pbvrSizes;
        std::uniform_real_distribution<float> keepDist(0.0f, 1.0f);
        std::uniform_real_distribution<float> jitterDist(-1.0f, 1.0f);
        auto& rng = flameWorld_.pbvrRng();

        const auto ext = getExtent();
        const float viewportHeight = (ext.height > 0) ? static_cast<float>(ext.height) : 720.0f;
        const float camDistance = fluidRenderer_.getCameraDistance();
        const float worldPerPixel =
            (2.0f * camDistance * std::tan(glm::radians(45.0f) * 0.5f)) / viewportHeight;
        const float subSizeScale = 1.0f / std::sqrt(static_cast<float>(render.pbvrRepeatCount));

        const auto pushPBVR = [&](const glm::vec3& worldPos, float opacity,
                                  const glm::vec3& color, float sizePixels) {
            const float footprintRadius = 0.5f * sizePixels * worldPerPixel;
            const float subSize = sizePixels * subSizeScale;
            for (int r = 0; r < render.pbvrRepeatCount; ++r) {
                if (keepDist(rng) >= opacity) {
                    continue;
                }
                glm::vec3 jittered = worldPos;
                if (render.pbvrRepeatCount > 1) {
                    jittered.x += jitterDist(rng) * footprintRadius;
                    jittered.y += jitterDist(rng) * footprintRadius;
                    jittered.z += jitterDist(rng) * footprintRadius;
                }
                pbvrPositions.push_back(jittered.x);
                pbvrPositions.push_back(jittered.y);
                pbvrPositions.push_back(jittered.z);
                pbvrColors.push_back(color.r);
                pbvrColors.push_back(color.g);
                pbvrColors.push_back(color.b);
                pbvrColors.push_back(1.0f);
                pbvrSizes.push_back(subSize);
            }
        };

        const float flameOpacity = std::clamp(render.flameOpacityScale, 0.0f, 1.0f);
        for (size_t i = 0; i < particles.size(); ++i) {
            pushPBVR(xf(particles.positions[i]), flameOpacity,
                     flameColor(particles.temperatures[i], tMin, tMax), render.pointSize);
        }
        for (const auto& sp : secondaries) {
            if (sp.kind == Phantom::Physics::FlameFluid::SecondaryKind::Spark) {
                pushPBVR(xf(sp.position), flameOpacity,
                         flameColor(sp.temperature, tMin, tMax), render.pointSize * sp.size);
            } else {
                const float smokeOpacity = std::clamp(sp.opacity * render.smokeOpacityScale, 0.0f, 1.0f);
                pushPBVR(xf(sp.position), smokeOpacity,
                         smokeColorTint(sp.temperature, tMin, tMax), render.smokePointSize * sp.size);
            }
        }

        flameRenderer_.setPBVRParticles(std::move(pbvrPositions), std::move(pbvrColors), std::move(pbvrSizes));
    } else {
        std::vector<float> positions;
        std::vector<float> temperatures;
        std::vector<float> sizes;
        positions.reserve(particles.size() * 3);
        temperatures.reserve(particles.size());
        sizes.reserve(particles.size());

        for (size_t i = 0; i < particles.size(); ++i) {
            const glm::vec3 p = xf(particles.positions[i]);
            positions.push_back(p.x);
            positions.push_back(p.y);
            positions.push_back(p.z);
            temperatures.push_back(particles.temperatures[i]);
            sizes.push_back(1.0f);
        }

        std::vector<float> smokePositions;
        std::vector<float> smokeOpacities;
        std::vector<float> smokeSizes;
        std::vector<float> smokeTemperatures;

        for (const auto& sp : secondaries) {
            if (sp.kind == Phantom::Physics::FlameFluid::SecondaryKind::Spark) {
                const glm::vec3 p = xf(sp.position);
                positions.push_back(p.x);
                positions.push_back(p.y);
                positions.push_back(p.z);
                temperatures.push_back(sp.temperature);
                sizes.push_back(sp.size);
            } else {
                const glm::vec3 p = xf(sp.position);
                smokePositions.push_back(p.x);
                smokePositions.push_back(p.y);
                smokePositions.push_back(p.z);
                smokeOpacities.push_back(std::clamp(sp.opacity * render.smokeOpacityScale, 0.0f, 1.0f));
                smokeSizes.push_back(sp.size);
                smokeTemperatures.push_back(sp.temperature);
            }
        }

        flameRenderer_.setParticles(std::move(positions), std::move(temperatures), std::move(sizes));
        flameRenderer_.setSmokeParticles(std::move(smokePositions), std::move(smokeOpacities),
                                         std::move(smokeSizes), std::move(smokeTemperatures));
    }

    flameRenderer_.setTemperatureRange(tMin, tMax);
    flameRenderer_.setPointSize(render.pointSize);
    flameRenderer_.setSmokePointSize(render.smokePointSize);
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

    if (ssfrPanel_.isEnabled() || ssfrTestPanel_.isActive()) {
        ssfrRenderer_.setParticleBuffer(posBuf, count);
    }

    ssfrRenderer_.setSprayParticles({});
    ssfrRenderer_.setFoamParticles({});
}

} // namespace Phantom
