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

    ImGui::SetNextWindowPos(ImVec2(10.f, 35.f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(360.f, 460.f), ImGuiCond_Once);
    if (!ImGui::Begin("Control", &visible_)) {
        ImGui::End();
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

    ImGui::Separator();

    runButton_.show();
    ImGui::SameLine();
    stepButton_.show();
    ImGui::SameLine();
    resetButton_.show();

    ImGui::Text("Particles: %llu",
                static_cast<unsigned long long>(world_->getParticleCount()));
    ImGui::Text("Step time: %.3f ms", world_->getLastStepTimeMs());

    if (world_->getSimulationType() == FluidWorld::SimulationType::GPU_CSPH) {
        const auto* solver = world_->getGpuSolver();
        if (solver) {
            const auto& ps = solver->getLastProfileStats();
            ImGui::Text("GPU_CSPH waitPrev2: %.3f ms", ps.waitPrevSecondMs);
            ImGui::Text("GPU_CSPH rec1: %.3f ms",      ps.recordClearCountMs);
            ImGui::Text("GPU_CSPH sub1: %.3f ms",      ps.submitClearCountMs);
            ImGui::Text("GPU_CSPH wait1: %.3f ms",     ps.waitClearCountMs);
            ImGui::Text("GPU_CSPH prefix: %.3f ms",    ps.prefixSumMs);
            ImGui::Text("GPU_CSPH rec2: %.3f ms",      ps.recordRestMs);
            ImGui::Text("GPU_CSPH sub2: %.3f ms",      ps.submitRestMs);
            ImGui::Text("GPU_CSPH wait2: %.3f ms",     ps.waitRestMs);
            ImGui::Text("GPU_CSPH total(sim): %.3f ms",ps.totalSimulateMs);
        }
    }

    ImGui::Text("Spray: %llu",
                static_cast<unsigned long long>(world_->getSprayPositions().size()));
    ImGui::Text("Foam: %llu",
                static_cast<unsigned long long>(world_->getFoamPositions().size()));

    ImGui::Separator();

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

    ImGui::PushID("FluidBounds");
    fluidBoundsView_.setValue(params.fluidBounds);
    fluidBoundsView_.show();
    params.fluidBounds = fluidBoundsView_.getValue();
    ImGui::PopID();

    ImGui::PushID("Boundary");
    boundaryView_.setValue(params.boundary);
    boundaryView_.show();
    params.boundary = boundaryView_.getValue();
    ImGui::PopID();

    ImGui::Separator();
    if (ImGui::CollapsingHeader("Mesh Boundary")) {
        ImGui::InputText("STL Path", meshBoundaryPathBuf_, sizeof(meshBoundaryPathBuf_));
        if (ImGui::Button("Load##MeshBoundary")) {
            world_->loadMeshBoundary(meshBoundaryPathBuf_);
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear##MeshBoundary")) {
            world_->clearMeshBoundary();
        }
        ImGui::Text("Triangles: %llu",
                    static_cast<unsigned long long>(world_->getMeshBoundaryTriangleCount()));
    }

    ImGui::Separator();
    if (ImGui::CollapsingHeader("Emitters")) {
        ImGui::DragFloat3("Center", newEmitterCenter_, 0.1f);
        ImGui::DragFloat("Disk Radius", &newEmitterRadius_, 0.01f, 0.0f, 100.0f);
        ImGui::DragFloat("Rate (particles/sec)", &newEmitterRate_, 1.0f, 0.0f, 100000.0f);
        ImGui::DragFloat3("Direction", newEmitterDirection_, 0.01f);
        ImGui::DragFloat("Speed", &newEmitterSpeed_, 0.01f, 0.0f, 1000.0f);
        if (ImGui::Button("Add Emitter")) {
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
        ImGui::SameLine();
        if (ImGui::Button("Clear All##Emitters")) {
            world_->clearEmitters();
        }

        const auto& emitters = world_->getEmitters();
        ImGui::Text("Registered: %llu", static_cast<unsigned long long>(emitters.size()));
        for (size_t i = 0; i < emitters.size(); ++i) {
            const auto& e = emitters[i];
            ImGui::Text("#%llu center=(%.2f,%.2f,%.2f) rate=%.1f/s",
                        static_cast<unsigned long long>(i),
                        e.center.x, e.center.y, e.center.z, e.rate);
        }
    }

    ImGui::Separator();
    if (ImGui::CollapsingHeader("Outflow Regions")) {
        ImGui::PushID("NewOutflowRegion");
        newOutflowRegionView_.show();
        ImGui::PopID();
        if (ImGui::Button("Add Outflow Region")) {
            Phantom::Physics::OutflowRegion r;
            r.bounds = newOutflowRegionView_.getValue();
            world_->addOutflowRegion(r);
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear All##OutflowRegions")) {
            world_->clearOutflowRegions();
        }

        const auto& regions = world_->getOutflowRegions();
        ImGui::Text("Registered: %llu", static_cast<unsigned long long>(regions.size()));
        for (size_t i = 0; i < regions.size(); ++i) {
            const auto& r = regions[i];
            const auto mn = r.bounds.getMin();
            const auto mx = r.bounds.getMax();
            ImGui::Text("#%llu min=(%.2f,%.2f,%.2f) max=(%.2f,%.2f,%.2f)",
                        static_cast<unsigned long long>(i),
                        mn.x, mn.y, mn.z, mx.x, mx.y, mx.z);
        }
    }

    ImGui::Separator();
    if (ImGui::CollapsingHeader("White Water")) {
        auto& ww = world_->whiteWaterParams();

        ImGui::SliderFloat("Spray Vel Threshold", &ww.sprayVelThreshold, 0.5f, 10.0f);
        ImGui::SliderFloat("Spray Density Ratio", &ww.sprayDensityRatio, 0.1f, 1.0f);
        ImGui::SliderFloat("Foam Curv Threshold", &ww.foamCurvThreshold, 0.1f, 2.0f);
        ImGui::SliderFloat("Foam Neighbor Count", &ww.foamNeighborCount, 5.0f, 60.0f);
        ImGui::SliderInt("Max Spray Particles",   &ww.maxSprayParticles, 0, 100000);
        ImGui::SliderInt("Max Foam Particles",    &ww.maxFoamParticles,  0, 100000);
        ImGui::SliderFloat("Foam Buoyancy",       &ww.foamBuoyancy, 0.0f, 5.0f);
        ImGui::SliderFloat("Foam Drag",           &ww.foamDrag, 0.70f, 1.0f);
    }

    ImGui::End();
}

} // namespace Phantom
