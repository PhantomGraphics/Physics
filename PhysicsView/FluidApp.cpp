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
    ssfrPanel_.bindRenderer(&ssfrRenderer_);
    ssfrPanel_.bindWorld(&world_);
    ssfrPanel_.init();
    ssfrTestPanel_.bindSSFRRenderer(&ssfrRenderer_);
    ssfrTestPanel_.init();

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

    add(&fluidRenderer_);
    add(&ssfrRenderer_);
    add(&rigidRenderer_);
    add(&softRenderer_);
    add(&volumeRenderer_);
    add(&meshRenderer_);
    add(&flameRenderer_);

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

    buildMenuBar();
}

void FluidApp::buildMenuBar()
{
    menuItems_.emplace_back("Quit");
    UI::MenuItem& quit = menuItems_.back();
    quit.setFunction([this] {
        glfwSetWindowShouldClose(getWindow().get(), GLFW_TRUE);
    });
    fileMenu_.add(&quit);

    // One entry per ControlPage: selecting it makes that page active in the
    // shared Control window and shows the window if it was hidden. The order
    // matches the ControlPage enum; SSFRTest sits last, after a separator, so
    // it reads as test-only (GUI_RESTRUCTURING_PLAN.md 5.4/6.8).
    for (int i = 0; i < static_cast<int>(kControlPageCount); ++i) {
        const auto page = static_cast<ControlPage>(i);
        if (page == ControlPage::SSFRTest)
            physicsMenu_.add(&physicsMenuSeparator_);

        menuItems_.emplace_back(toString(page));
        UI::MenuItem& item = menuItems_.back();
        item.setFunction([this, page] {
            controlHost_.setPage(page);
            controlHost_.setVisible(true);
        });
        item.setSelected([this, page] {
            return controlHost_.getPage() == page && controlHost_.isVisible();
        });
        item.setEnabled([this, page] { return controlHost_.isPageEnabled(page); });
        item.setTooltip([this, page]() -> std::string {
            return controlHost_.isPageEnabled(page)
                ? std::string{}
                : controlHost_.pageDisabledReason(page);
        });
        physicsMenu_.add(&item);
    }

    menuItems_.emplace_back("Control Window");
    UI::MenuItem& ctrlWin = menuItems_.back();
    ctrlWin.setFunction([this] { controlHost_.setVisible(!controlHost_.isVisible()); });
    ctrlWin.setSelected([this] { return controlHost_.isVisible(); });
    viewMenu_.add(&ctrlWin);

    menuItems_.emplace_back("Scene Objects");
    UI::MenuItem& objList = menuItems_.back();
    objList.setFunction([this] { objectListPanel_.setVisible(!objectListPanel_.isVisible()); });
    objList.setSelected([this] { return objectListPanel_.isVisible(); });
    viewMenu_.add(&objList);

    menuBar_.add(&fileMenu_);
    menuBar_.add(&physicsMenu_);
    menuBar_.add(&viewMenu_);
}

void FluidApp::registerControlPages()
{
    controlHost_.registerPage(ControlPage::Fluid,            &controlPanel_);
    controlHost_.registerPage(ControlPage::RigidBody,        &rigidControlPanel_);
    controlHost_.registerPage(ControlPage::SoftBody,         &softControlPanel_);
    controlHost_.registerPage(ControlPage::Flame,            &flameControlPanel_);
    controlHost_.registerPage(ControlPage::FluidRendering,   &fluidRenderer_);
    controlHost_.registerPage(ControlPage::SSFR,             &ssfrPanel_);
    controlHost_.registerPage(ControlPage::VolumeConversion, &volumeConvertPanel_);
    controlHost_.registerPage(ControlPage::ScenarioBrowser,  &scenarioBrowserEmbed_);
    controlHost_.registerPage(ControlPage::SSFRTest,         &ssfrTestPanel_);
}

bool FluidApp::loadScenario(const std::string& jsonPath) {
    return runner_.load(jsonPath);
}

void FluidApp::onInit()
{
    world_.setVulkanContext(getContext(), getCommandPool());
    world_.reset();
    syncParticlesToRenderer();
    syncGpuCsphBufferToRenderer();

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

    ::VKG::VkAppBase::onInit();
    setupCallbacks();
    syncRigidRenderer();
    syncSoftRenderer();
    syncFlameRenderer();

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
}

void FluidApp::onSwapChainCreated()
{
    fluidRenderer_.setExtent(getExtent());
    ssfrRenderer_.setExtent(getExtent());
    rigidRenderer_.setExtent(getExtent());
    softRenderer_.setExtent(getExtent());
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
        ssfrRenderer_.setEnabled(false);
    }
    flameRenderer_.setEnabled(flameActive);
    rigidRenderer_.setEnabled(!flameActive);
    softRenderer_.setEnabled(!flameActive);

    ssfrRenderer_.setCamera(fluidRenderer_.getProjMatrix(), fluidRenderer_.getViewMatrix());
    rigidRenderer_.setMVP(fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
    softRenderer_.setMVP(fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
    volumeRenderer_.setMVP(fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
    meshRenderer_.setMVP(fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());

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

    ::VKG::VkAppBase::onUpdate(frameIndex);
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

    ssfrRenderer_.onPreRender(cmd, frameIndex);
}

void FluidApp::onImGui()
{
    // Menu bar (File / Physics / View) is a widget tree assembled once in
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
    ::VKG::VkAppBase::onCleanup();
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
    };

    win.onCursorPos = [this](double x, double y) {
        fluidRenderer_.handleMouseMove(static_cast<float>(x), static_cast<float>(y));
        ssfrRenderer_.setCamera(fluidRenderer_.getProjMatrix(), fluidRenderer_.getViewMatrix());
    };

    win.onScroll = [this](double, double dy) {
        fluidRenderer_.handleScroll(static_cast<float>(dy));
        ssfrRenderer_.setCamera(fluidRenderer_.getProjMatrix(), fluidRenderer_.getViewMatrix());
    };
}

void FluidApp::syncRigidRenderer()
{
    auto wd = world_.rigid().buildWireData();
    rigidRenderer_.update(wd.positions, wd.colors, wd.indices,
                           fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
}

void FluidApp::syncSoftRenderer()
{
    auto wd = softWorld_.buildWireData();
    softRenderer_.update(wd.positions, wd.colors, wd.indices,
                          fluidRenderer_.getProjMatrix() * fluidRenderer_.getViewMatrix());
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
