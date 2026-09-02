#include "pch.h"
#include "FluidApp.h"

#include "../../CGLib/VulkanGraphics/VulkanSPVResolver.h"

namespace Phantom {

FluidApp::FluidApp(int width, int height, const std::string& title)
    : VkAppBase(width, height, title)
    , controlPanel_(&world_)
    , rigidControlPanel_(&world_.rigid())
    , softWorld_(world_.physicsSolver())
    , softControlPanel_(&softWorld_)
{
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
    ssfrPanel_.bindRenderer(&ssfrRenderer_);
    ssfrPanel_.bindWorld(&world_);
    ssfrTestPanel_.bindSSFRRenderer(&ssfrRenderer_);

    volumeConvertPanel_.bindWorld(&world_);
    volumeConvertPanel_.bindConverter(&volumeConverter_);
    volumeConvertPanel_.bindMeshConverter(&meshConverter_);
    volumeConvertPanel_.bindVolumeRenderer(&volumeRenderer_);
    volumeConvertPanel_.bindMeshRenderer(&meshRenderer_);
    volumeConvertPanel_.setOnVolumeChanged([this]() { syncVolumeRenderer(); });
    volumeConvertPanel_.setOnMeshChanged([this]() { syncMeshRenderer(); });
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
    add(&controlPanel_);
    add(&rigidControlPanel_);
    add(&softControlPanel_);
    add(&ssfrPanel_);
    add(&ssfrTestPanel_);
    add(&volumeConvertPanel_);
    add(&scenarioBrowser_);
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
    ssfrRenderer_.setParticleRadius(world_.params().radius);

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
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Quit")) {
                glfwSetWindowShouldClose(getWindow().get(), GLFW_TRUE);
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            if (ImGui::MenuItem("Fluid Control", nullptr, controlPanel_.isVisible()))
                controlPanel_.setVisible(!controlPanel_.isVisible());
            if (ImGui::MenuItem("Fluid Renderer", nullptr, fluidRenderer_.isSettingsVisible()))
                fluidRenderer_.setSettingsVisible(!fluidRenderer_.isSettingsVisible());
            if (ImGui::MenuItem("SSFR Control", nullptr, ssfrPanel_.isVisible()))
                ssfrPanel_.setVisible(!ssfrPanel_.isVisible());
            if (ImGui::MenuItem("SSFR Test", nullptr, ssfrTestPanel_.isVisible()))
                ssfrTestPanel_.setVisible(!ssfrTestPanel_.isVisible());
            ImGui::Separator();
            if (ImGui::MenuItem("Rigid Body Control", nullptr, rigidControlPanel_.isVisible()))
                rigidControlPanel_.setVisible(!rigidControlPanel_.isVisible());
            if (ImGui::MenuItem("Soft Body Control", nullptr, softControlPanel_.isVisible()))
                softControlPanel_.setVisible(!softControlPanel_.isVisible());
            if (ImGui::MenuItem("Volume Conversion", nullptr, volumeConvertPanel_.isVisible()))
                volumeConvertPanel_.setVisible(!volumeConvertPanel_.isVisible());
            if (ImGui::MenuItem("Scenario Browser", nullptr, scenarioBrowser_.isVisible()))
                scenarioBrowser_.setVisible(!scenarioBrowser_.isVisible());
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }

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
    std::vector<float> densityRatios;
    densityRatios.reserve(merged.size());
    const float restDensity = world_.getActiveRestDensity();
    const float invRestDensity = restDensity > 0.0f ? 1.0f / restDensity : 0.0f;
    for (float density : fluidDensities) {
        densityRatios.push_back(density * invRestDensity);
    }
    // White-water particles do not carry SPH density; use the neutral value.
    densityRatios.resize(merged.size(), 1.0f);
    fluidRenderer_.setParticles(merged, densityRatios);
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
