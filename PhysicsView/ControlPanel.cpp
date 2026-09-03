#include "pch.h"
#include "ControlPanel.h"

namespace Phantom {

void ControlPanel::initWidgets()
{
    if (widgetsInitialized_) return;
    widgetsInitialized_ = true;

    methodCombo_.addItem("DFSPH");
    methodCombo_.addItem("PBSPH");
    methodCombo_.addItem("CSPH");
    methodCombo_.addItem("GPU CSPH");
    methodCombo_.setSelected(0);

    runButton_.setFunction([this]() {
        world_->setRunning(!world_->isRunning());
    });
    stepButton_.setFunction([this]() {
        world_->step();
        if (onWorldChanged_) onWorldChanged_();
    });
    resetButton_.setFunction([this]() {
        world_->reset();
        if (onWorldChanged_) onWorldChanged_();
    });
}

void ControlPanel::onImGui()
{
    if (!world_ || !visible_) return;

    initWidgets();

    auto& params = world_->params();

    UI::Immediate::setNextWindowPosition(10.f, 35.f);
    UI::Immediate::setNextWindowSize(360.f, 460.f);
    if (!UI::Immediate::beginWindow("Control", &visible_)) {
        UI::Immediate::endWindow();
        return;
    }

    static const char* kMethodLabels[] = { "DFSPH", "PBSPH", "CSPH", "GPU CSPH" };
    methodCombo_.setSelected(static_cast<int>(world_->getSimulationType()));
    methodCombo_.show();
    {
        const std::string sel = methodCombo_.getSelectedItem();
        for (int i = 0; i < 4; ++i) {
            if (sel == kMethodLabels[i]) {
                const auto newType = static_cast<FluidWorld::SimulationType>(i);
                if (newType != world_->getSimulationType()) {
                    world_->setSimulationType(newType);
                    world_->reset();
                    if (onWorldChanged_) onWorldChanged_();
                }
                break;
            }
        }
    }

    UI::Immediate::separator();

    runButton_.show();
    UI::Immediate::sameLine();
    stepButton_.show();
    UI::Immediate::sameLine();
    resetButton_.show();

    UI::Immediate::text("Particles: %llu",
                static_cast<unsigned long long>(world_->getParticleCount()));
    UI::Immediate::text("Step time: %.3f ms", world_->getLastStepTimeMs());

    if (world_->getSimulationType() == FluidWorld::SimulationType::GPU_CSPH) {
        const auto* solver = world_->getGpuSolver();
        if (solver) {
            const auto& ps = solver->getLastProfileStats();
            UI::Immediate::text("GPU_CSPH waitPrev2: %.3f ms", ps.waitPrevSecondMs);
            UI::Immediate::text("GPU_CSPH rec1: %.3f ms",      ps.recordClearCountMs);
            UI::Immediate::text("GPU_CSPH sub1: %.3f ms",      ps.submitClearCountMs);
            UI::Immediate::text("GPU_CSPH wait1: %.3f ms",     ps.waitClearCountMs);
            UI::Immediate::text("GPU_CSPH prefix: %.3f ms",    ps.prefixSumMs);
            UI::Immediate::text("GPU_CSPH rec2: %.3f ms",      ps.recordRestMs);
            UI::Immediate::text("GPU_CSPH sub2: %.3f ms",      ps.submitRestMs);
            UI::Immediate::text("GPU_CSPH wait2: %.3f ms",     ps.waitRestMs);
            UI::Immediate::text("GPU_CSPH total(sim): %.3f ms",ps.totalSimulateMs);
        }
    }

    UI::Immediate::text("Spray: %llu",
                static_cast<unsigned long long>(world_->getSprayPositions().size()));
    UI::Immediate::text("Foam: %llu",
                static_cast<unsigned long long>(world_->getFoamPositions().size()));

    UI::Immediate::separator();

    timeStepView_.setValue(params.timeStep);
    timeStepView_.show();
    params.timeStep = timeStepView_.getValue();

    radiusView_.setValue(params.radius);
    radiusView_.show();
    params.radius = radiusView_.getValue();

    effectLenView_.setValue(params.effectLength);
    effectLenView_.show();
    params.effectLength = effectLenView_.getValue();

    densityView_.setValue(params.density);
    densityView_.show();
    params.density = densityView_.getValue();

    // WCSPH's own pressure solve and DFSPH's Two-Way boundary coupling force
    // both derive pressureCoe as pressureCoeScale * effectLength instead of a
    // raw stiffness number (see FluidWorld::createWCSPH()/createDFSPH() and
    // internal design notes); PBSPH/GPU_CSPH still take the
    // raw value directly.
    const auto simType = world_->getSimulationType();
    if (simType == FluidWorld::SimulationType::WCSPH ||
        simType == FluidWorld::SimulationType::DFSPH) {
        pressureCoeScaleView_.setValue(params.pressureCoeScale);
        pressureCoeScaleView_.show();
        params.pressureCoeScale = pressureCoeScaleView_.getValue();
    } else {
        stiffnessView_.setValue(params.stiffness);
        stiffnessView_.show();
        params.stiffness = stiffnessView_.getValue();
    }

    viscosityView_.setValue(params.viscosity);
    viscosityView_.show();
    params.viscosity = viscosityView_.getValue();

    UI::Immediate::pushId("FluidBounds");
    fluidBoundsView_.setValue(params.fluidBounds);
    fluidBoundsView_.show();
    params.fluidBounds = fluidBoundsView_.getValue();
    UI::Immediate::popId();

    UI::Immediate::pushId("Boundary");
    boundaryView_.setValue(params.boundary);
    boundaryView_.show();
    params.boundary = boundaryView_.getValue();
    UI::Immediate::popId();

    UI::Immediate::separator();
    if (UI::Immediate::collapsingHeader("Mesh Boundary")) {
        UI::Immediate::inputText("STL Path", meshBoundaryPathBuf_, sizeof(meshBoundaryPathBuf_));
        if (UI::Immediate::button("Load##MeshBoundary")) {
            world_->loadMeshBoundary(meshBoundaryPathBuf_);
        }
        UI::Immediate::sameLine();
        if (UI::Immediate::button("Clear##MeshBoundary")) {
            world_->clearMeshBoundary();
        }
        UI::Immediate::text("Triangles: %llu",
                    static_cast<unsigned long long>(world_->getMeshBoundaryTriangleCount()));
    }

    UI::Immediate::separator();
    if (UI::Immediate::collapsingHeader("Emitters")) {
        UI::Immediate::dragFloat3("Center", newEmitterCenter_, 0.1f);
        UI::Immediate::dragFloat("Disk Radius", newEmitterRadius_, 0.01f, 0.0f, 100.0f);
        UI::Immediate::dragFloat("Rate (particles/sec)", newEmitterRate_, 1.0f, 0.0f, 100000.0f);
        UI::Immediate::dragFloat3("Direction", newEmitterDirection_, 0.01f);
        UI::Immediate::dragFloat("Speed", newEmitterSpeed_, 0.01f, 0.0f, 1000.0f);
        if (UI::Immediate::button("Add Emitter")) {
            Phantom::Physics::Emitter e;
            e.center = Phantom::Math::Vector3df(
                newEmitterCenter_[0], newEmitterCenter_[1], newEmitterCenter_[2]);
            e.radius = newEmitterRadius_;
            e.rate = newEmitterRate_;
            e.direction = Phantom::Math::Vector3df(
                newEmitterDirection_[0], newEmitterDirection_[1], newEmitterDirection_[2]);
            e.speed = newEmitterSpeed_;
            world_->addEmitter(e);
        }
        UI::Immediate::sameLine();
        if (UI::Immediate::button("Clear All##Emitters")) {
            world_->clearEmitters();
        }

        const auto& emitters = world_->getEmitters();
        UI::Immediate::text("Registered: %llu", static_cast<unsigned long long>(emitters.size()));
        for (size_t i = 0; i < emitters.size(); ++i) {
            const auto& e = emitters[i];
            UI::Immediate::text("#%llu center=(%.2f,%.2f,%.2f) rate=%.1f/s",
                        static_cast<unsigned long long>(i),
                        e.center.x, e.center.y, e.center.z, e.rate);
        }
    }

    UI::Immediate::separator();
    if (UI::Immediate::collapsingHeader("Outflow Regions")) {
        UI::Immediate::pushId("NewOutflowRegion");
        newOutflowRegionView_.show();
        UI::Immediate::popId();
        if (UI::Immediate::button("Add Outflow Region")) {
            Phantom::Physics::OutflowRegion r;
            r.bounds = newOutflowRegionView_.getValue();
            world_->addOutflowRegion(r);
        }
        UI::Immediate::sameLine();
        if (UI::Immediate::button("Clear All##OutflowRegions")) {
            world_->clearOutflowRegions();
        }

        const auto& regions = world_->getOutflowRegions();
        UI::Immediate::text("Registered: %llu", static_cast<unsigned long long>(regions.size()));
        for (size_t i = 0; i < regions.size(); ++i) {
            const auto& r = regions[i];
            const auto mn = r.bounds.getMin();
            const auto mx = r.bounds.getMax();
            UI::Immediate::text("#%llu min=(%.2f,%.2f,%.2f) max=(%.2f,%.2f,%.2f)",
                        static_cast<unsigned long long>(i),
                        mn.x, mn.y, mn.z, mx.x, mx.y, mx.z);
        }
    }

    UI::Immediate::separator();
    if (UI::Immediate::collapsingHeader("White Water")) {
        auto& ww = world_->whiteWaterParams();

        UI::Immediate::sliderFloat("Spray Vel Threshold", ww.sprayVelThreshold, 0.5f, 10.0f);
        UI::Immediate::sliderFloat("Spray Density Ratio", ww.sprayDensityRatio, 0.1f, 1.0f);
        UI::Immediate::sliderFloat("Foam Curv Threshold", ww.foamCurvThreshold, 0.1f, 2.0f);
        UI::Immediate::sliderFloat("Foam Neighbor Count", ww.foamNeighborCount, 5.0f, 60.0f);
        UI::Immediate::sliderInt("Max Spray Particles", ww.maxSprayParticles, 0, 100000);
        UI::Immediate::sliderInt("Max Foam Particles", ww.maxFoamParticles, 0, 100000);
        UI::Immediate::sliderFloat("Foam Buoyancy", ww.foamBuoyancy, 0.0f, 5.0f);
        UI::Immediate::sliderFloat("Foam Drag", ww.foamDrag, 0.70f, 1.0f);
    }

    UI::Immediate::endWindow();
}

} // namespace Phantom
