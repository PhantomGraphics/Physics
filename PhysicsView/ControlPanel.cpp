#include "pch.h"
#include "ControlPanel.h"

namespace Phantom {

ControlPanel::ControlPanel(FluidWorld* world) : world_(world)
{
    buildUi();
}

namespace {
std::string fmtF(float v)
{
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.9g", v);
    return buf;
}
}

void ControlPanel::issueFloat(const char* name, float v, const std::function<void()>& direct)
{
    issue(std::string(name) + ":" + fmtF(v), direct);
}

void ControlPanel::issueBox(const char* name, const Math::Box3df& b, const std::function<void()>& direct)
{
    const auto mn = b.getMin();
    const auto mx = b.getMax();
    issue(std::string(name) + ":" + fmtF(mn.x) + ":" + fmtF(mn.y) + ":" + fmtF(mn.z) + ":"
              + fmtF(mx.x) + ":" + fmtF(mx.y) + ":" + fmtF(mx.z), direct);
}

void ControlPanel::buildUi()
{
    if (uiBuilt_) return;
    uiBuilt_ = true;

    // ---- Simulation -------------------------------------------------
    methodCombo_.addItem("DFSPH");
    methodCombo_.addItem("PBSPH");
    methodCombo_.addItem("CSPH");
    methodCombo_.addItem("GPU CSPH");
    methodCombo_.bind(
        [this] { return static_cast<int>(world_->getSimulationType()); },
        [this](int i) {
            const auto newType = static_cast<FluidWorld::SimulationType>(i);
            if (newType != world_->getSimulationType()) {
                world_->setSimulationType(newType);
                world_->reset();
                notifyWorldChanged();
            }
        });

    timeStepView_.bind([this] { return p().timeStep; },
                       [this](float v) { issueFloat("SetFluidTimeStep", v, [&] { p().timeStep = v; }); });

    runButton_.setFunction([this] { world_->setRunning(!world_->isRunning()); });
    stepButton_.setFunction([this] { world_->step(); notifyWorldChanged(); });
    resetButton_.setFunction([this] { world_->reset(); notifyWorldChanged(); });
    actionsRow_.add(&runButton_);
    actionsRow_.add(&stepButton_);
    actionsRow_.add(&resetButton_);

    gpuProfileLabel_.setVisibleWhen([this] {
        return isGPU() && world_->getGpuSolver() != nullptr;
    });

    simSection_.add(&methodCombo_);
    simSection_.add(&timeStepView_);
    simSection_.add(&actionsRow_);
    simSection_.add(&stepTimeLabel_);
    simSection_.add(&gpuProfileLabel_);

    // ---- Material -------------------------------------------------
    densityView_.bind([this] { return p().density; },
                      [this](float v) { issueFloat("SetFluidDensity", v, [&] { p().density = v; }); });
    viscosityView_.bind([this] { return p().viscosity; },
                        [this](float v) { issueFloat("SetFluidViscosity", v, [&] { p().viscosity = v; }); });
    pressureCoeScaleView_.bind([this] { return p().pressureCoeScale; },
                               [this](float v) { issueFloat("SetFluidPressureCoeScale", v, [&] { p().pressureCoeScale = v; }); });
    stiffnessView_.bind([this] { return p().stiffness; },
                        [this](float v) { issueFloat("SetFluidStiffness", v, [&] { p().stiffness = v; }); });
    pressureCoeScaleView_.setVisibleWhen([this] { return isWCSPH() || isDFSPH(); });
    stiffnessView_.setVisibleWhen([this] { return !(isWCSPH() || isDFSPH()); });

    tensionSlider_.bind([this] { return p().tension; },
                        [this](float v) { issueFloat("SetFluidTension", v, [&] { p().tension = v; }); });
    tensionScope_.setDisabledWhen([this] { return !isWCSPH(); });
    tensionScope_.setTooltip([this]() -> std::string {
        return isWCSPH() ? std::string{}
            : "Surface tension is only available with the CSPH (WCSPH) method.";
    });
    tensionScope_.add(&tensionSlider_);

    materialSection_.add(&densityView_);
    materialSection_.add(&viscosityView_);
    materialSection_.add(&pressureCoeScaleView_);
    materialSection_.add(&stiffnessView_);
    materialSection_.add(&tensionScope_);

    // ---- Initial Region ----------------------------------------
    radiusView_.bind([this] { return p().radius; },
                     [this](float v) { issueFloat("SetFluidRadius", v, [&] { p().radius = v; }); });
    effectLenView_.bind([this] { return p().effectLength; },
                        [this](float v) { issueFloat("SetFluidEffectLength", v, [&] { p().effectLength = v; }); });
    fluidBoundsView_.bind([this] { return p().fluidBounds; },
                          [this](const Math::Box3df& b) { issueBox("SetFluidBounds", b, [&] { p().fluidBounds = b; }); });
    fluidBoundsScope_.add(&fluidBoundsView_);

    initialRegionSection_.add(&appliedOnResetLabel_);
    initialRegionSection_.add(&radiusView_);
    initialRegionSection_.add(&effectLenView_);
    initialRegionSection_.add(&fluidBoundsScope_);

    // ---- Boundaries -------------------------------------------
    boundaryView_.bind([this] { return p().boundary; },
                       [this](const Math::Box3df& b) { issueBox("SetFluidBoundary", b, [&] { p().boundary = b; }); });
    boundaryScope_.add(&boundaryView_);

    boundaryDampingSlider_.bind([this] { return p().boundaryDampingRatio; },
                                [this](float v) { issueFloat("SetFluidBoundaryDamping", v, [&] { p().boundaryDampingRatio = v; }); });
    boundaryDampingScope_.setDisabledWhen([this] { return !(isWCSPH() || isDFSPH()); });
    boundaryDampingScope_.setTooltip([this]() -> std::string {
        return (isWCSPH() || isDFSPH())
            ? "Fraction of wall-normal velocity absorbed on contact. Applied on Reset. ~0.35 is typical."
            : "Boundary damping only applies to the DFSPH / CSPH methods.";
    });
    boundaryDampingScope_.add(&boundaryDampingSlider_);

    loadMeshBoundaryButton_.setFunction([this] {
        world_->loadMeshBoundary(meshBoundaryPathView_.getValue());
    });
    clearMeshBoundaryButton_.setFunction([this] { world_->clearMeshBoundary(); });
    meshBoundaryRow_.add(&loadMeshBoundaryButton_);
    meshBoundaryRow_.add(&clearMeshBoundaryButton_);

    boundariesSection_.add(&boundaryScope_);
    boundariesSection_.add(&boundaryDampingScope_);
    boundariesSection_.add(&meshBoundarySeparator_);
    boundariesSection_.add(&meshBoundaryHeaderLabel_);
    boundariesSection_.add(&meshBoundaryPathView_);
    boundariesSection_.add(&meshBoundaryRow_);
    boundariesSection_.add(&triangleCountLabel_);

    // ---- Emitters --------------------------------------------
    emitterRadiusDrag_.setValue(0.1f);
    emitterRateDrag_.setValue(50.0f);
    emitterSpeedDrag_.setValue(1.0f);

    addEmitterButton_.setFunction([this] {
        Phantom::Physics::Emitter e;
        e.center    = emitterCenterView_.getValue();
        e.radius    = emitterRadiusDrag_.getValue();
        e.rate      = emitterRateDrag_.getValue();
        e.direction = emitterDirectionView_.getValue();
        e.speed     = emitterSpeedDrag_.getValue();
        world_->addEmitter(e);
    });
    clearEmittersButton_.setFunction([this] { world_->clearEmitters(); });
    emitterButtonsRow_.add(&addEmitterButton_);
    emitterButtonsRow_.add(&clearEmittersButton_);

    emittersSection_.add(&emitterCenterView_);
    emittersSection_.add(&emitterRadiusDrag_);
    emittersSection_.add(&emitterRateDrag_);
    emittersSection_.add(&emitterDirectionView_);
    emittersSection_.add(&emitterSpeedDrag_);
    emittersSection_.add(&emitterButtonsRow_);
    emittersSection_.add(&emitterListLabel_);

    // ---- Outflow Regions ----------------------------------
    newOutflowScope_.add(&newOutflowRegionView_);
    addOutflowButton_.setFunction([this] {
        Phantom::Physics::OutflowRegion r;
        r.bounds = newOutflowRegionView_.getValue();
        world_->addOutflowRegion(r);
    });
    clearOutflowButton_.setFunction([this] { world_->clearOutflowRegions(); });
    outflowButtonsRow_.add(&addOutflowButton_);
    outflowButtonsRow_.add(&clearOutflowButton_);

    outflowSection_.add(&newOutflowScope_);
    outflowSection_.add(&outflowButtonsRow_);
    outflowSection_.add(&outflowListLabel_);

    // ---- White Water ------------------------------------
    sprayVelThresholdSlider_.bind([this] { return ww().sprayVelThreshold; },
                                  [this](float v) { issue("SetWhiteWaterParam:sprayVelThreshold," + fmtF(v), [&] { ww().sprayVelThreshold = v; }); });
    sprayDensityRatioSlider_.bind([this] { return ww().sprayDensityRatio; },
                                  [this](float v) { issue("SetWhiteWaterParam:sprayDensityRatio," + fmtF(v), [&] { ww().sprayDensityRatio = v; }); });
    foamCurvThresholdSlider_.bind([this] { return ww().foamCurvThreshold; },
                                  [this](float v) { issue("SetWhiteWaterParam:foamCurvThreshold," + fmtF(v), [&] { ww().foamCurvThreshold = v; }); });
    foamNeighborCountSlider_.bind([this] { return ww().foamNeighborCount; },
                                  [this](float v) { issue("SetWhiteWaterParam:foamNeighborCount," + fmtF(v), [&] { ww().foamNeighborCount = v; }); });
    maxSprayParticlesSlider_.bind([this] { return ww().maxSprayParticles; },
                                  [this](int v) { issue("SetWhiteWaterParam:maxSprayParticles," + std::to_string(v), [&] { ww().maxSprayParticles = v; }); });
    maxFoamParticlesSlider_.bind([this] { return ww().maxFoamParticles; },
                                 [this](int v) { issue("SetWhiteWaterParam:maxFoamParticles," + std::to_string(v), [&] { ww().maxFoamParticles = v; }); });
    foamBuoyancySlider_.bind([this] { return ww().foamBuoyancy; },
                             [this](float v) { issue("SetWhiteWaterParam:foamBuoyancy," + fmtF(v), [&] { ww().foamBuoyancy = v; }); });
    foamDragSlider_.bind([this] { return ww().foamDrag; },
                         [this](float v) { issue("SetWhiteWaterParam:foamDrag," + fmtF(v), [&] { ww().foamDrag = v; }); });

    whiteWaterSection_.add(&sprayVelThresholdSlider_);
    whiteWaterSection_.add(&sprayDensityRatioSlider_);
    whiteWaterSection_.add(&foamCurvThresholdSlider_);
    whiteWaterSection_.add(&foamNeighborCountSlider_);
    whiteWaterSection_.add(&maxSprayParticlesSlider_);
    whiteWaterSection_.add(&maxFoamParticlesSlider_);
    whiteWaterSection_.add(&foamBuoyancySlider_);
    whiteWaterSection_.add(&foamDragSlider_);

    // ---- Assemble ---------------------------------------
    contents_.add(&simSection_);
    contents_.add(&materialSection_);
    contents_.add(&initialRegionSection_);
    contents_.add(&boundariesSection_);
    contents_.add(&emittersSection_);
    contents_.add(&outflowSeparator_);
    contents_.add(&outflowSection_);
    contents_.add(&whiteWaterSeparator_);
    contents_.add(&whiteWaterSection_);
}

std::string ControlPanel::stepTimeText() const
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Step time: %.3f ms", world_->getLastStepTimeMs());
    return buf;
}

std::string ControlPanel::gpuProfileText() const
{
    const auto* solver = world_->getGpuSolver();
    if (!solver) return {};
    const auto& ps = solver->getLastProfileStats();
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "GPU_CSPH waitPrev2: %.3f ms\n"
        "GPU_CSPH rec1: %.3f ms\n"
        "GPU_CSPH sub1: %.3f ms\n"
        "GPU_CSPH wait1: %.3f ms\n"
        "GPU_CSPH prefix: %.3f ms\n"
        "GPU_CSPH rec2: %.3f ms\n"
        "GPU_CSPH sub2: %.3f ms\n"
        "GPU_CSPH wait2: %.3f ms\n"
        "GPU_CSPH total(sim): %.3f ms",
        ps.waitPrevSecondMs, ps.recordClearCountMs, ps.submitClearCountMs,
        ps.waitClearCountMs, ps.prefixSumMs, ps.recordRestMs, ps.submitRestMs,
        ps.waitRestMs, ps.totalSimulateMs);
    return buf;
}

std::string ControlPanel::emitterListText() const
{
    const auto& emitters = world_->getEmitters();
    std::string s = "Registered: " + std::to_string(emitters.size());
    for (size_t i = 0; i < emitters.size(); ++i) {
        const auto& e = emitters[i];
        char line[128];
        std::snprintf(line, sizeof(line),
            "\n#%llu center=(%.2f,%.2f,%.2f) rate=%.1f/s",
            static_cast<unsigned long long>(i),
            e.center.x, e.center.y, e.center.z, e.rate);
        s += line;
    }
    return s;
}

std::string ControlPanel::outflowListText() const
{
    const auto& regions = world_->getOutflowRegions();
    std::string s = "Registered: " + std::to_string(regions.size());
    for (size_t i = 0; i < regions.size(); ++i) {
        const auto mn = regions[i].bounds.getMin();
        const auto mx = regions[i].bounds.getMax();
        char line[160];
        std::snprintf(line, sizeof(line),
            "\n#%llu min=(%.2f,%.2f,%.2f) max=(%.2f,%.2f,%.2f)",
            static_cast<unsigned long long>(i),
            mn.x, mn.y, mn.z, mx.x, mx.y, mx.z);
        s += line;
    }
    return s;
}

std::string ControlPanel::triangleCountText() const
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Triangles: %llu",
        static_cast<unsigned long long>(world_->getMeshBoundaryTriangleCount()));
    return buf;
}

void ControlPanel::onImGui()
{
    if (!world_ || !visible_) return;

    UI::Immediate::setNextWindowPosition(10.f, 35.f);
    UI::Immediate::setNextWindowSize(360.f, 460.f);
    if (!UI::Immediate::beginWindow("Fluid Control", &visible_)) {
        UI::Immediate::endWindow();
        return;
    }
    drawContents();
    UI::Immediate::endWindow();
}

void ControlPanel::drawContents()
{
    if (!world_) return;
    contents_.show();
}

} // namespace Phantom
