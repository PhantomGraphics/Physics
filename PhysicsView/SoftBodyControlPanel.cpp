#include "pch.h"
#include "SoftBodyControlPanel.h"

namespace Phantom {

static const char* kPresetNames[] = {
    "ClothTwoPin", "ClothTopEdge", "ClothWithSphere",
    "RopeHanging", "RopePendulum",
    "JellyDrop", "JellyOnBox", "Mixed",
};
static const SoftBodyPreset kPresetValues[] = {
    SoftBodyPreset::ClothTwoPin,
    SoftBodyPreset::ClothTopEdge,
    SoftBodyPreset::ClothWithSphere,
    SoftBodyPreset::RopeHanging,
    SoftBodyPreset::RopePendulum,
    SoftBodyPreset::JellyDrop,
    SoftBodyPreset::JellyOnBox,
    SoftBodyPreset::Mixed,
};
static constexpr int kNumPresets = 8;

void SoftBodyControlPanel::initWidgets() {
    if (widgetsInitialized_) return;
    widgetsInitialized_ = true;

    runButton_.setFunction([this]() {
        world_->setRunning(!world_->isRunning());
    });
    stepButton_.setFunction([this]() {
        world_->getWorld().setRunning(true);
        world_->step();
        world_->getWorld().setRunning(false);
        if (onWorldChanged_) onWorldChanged_();
    });
    resetButton_.setFunction([this]() {
        world_->reset();
        if (onWorldChanged_) onWorldChanged_();
    });
}

void SoftBodyControlPanel::onImGui() {
    if (!world_) return;
    initWidgets();

    ImGui::SetNextWindowPos(ImVec2(10.f, 35.f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(300.f, 560.f), ImGuiCond_Once);
    if (!ImGui::Begin("Soft Body Control")) { ImGui::End(); return; }

    {
        int cur = static_cast<int>(world_->currentPreset());
        if (ImGui::Combo("Preset", &cur, kPresetNames, kNumPresets)) {
            world_->setPreset(kPresetValues[cur]);
            if (onWorldChanged_) onWorldChanged_();
        }
    }

    runButton_.show();
    ImGui::SameLine();
    stepButton_.show();
    ImGui::SameLine();
    resetButton_.show();

    {
        auto& wp = world_->getWorld();
        ImGui::Text("Bodies:%zu  Particles:%zu",
                    wp.getBodyCount(), wp.getParticleCount());
        ImGui::Text("Running: %s", world_->isRunning() ? "Yes" : "No");
    }

    ImGui::Separator();
    ImGui::Text("Solver");
    {
        auto& sp = world_->getWorld().solverParams();

        timeStepView_.setValue(sp.timeStep);
        timeStepView_.show();
        sp.timeStep = timeStepView_.getValue();

        subStepsView_.setValue(sp.numSubsteps);
        subStepsView_.show();
        sp.numSubsteps = subStepsView_.getValue();

        iterView_.setValue(sp.numIterations);
        iterView_.show();
        sp.numIterations = iterView_.getValue();

        gravYView_.setValue(sp.gravity.y);
        gravYView_.show();
        sp.gravity.y = gravYView_.getValue();
    }

    ImGui::Separator();
    ImGui::Text("Sphere Collider");
    {
        auto& wp = world_->getWorld().params();
        bool sphereEnabled = wp.sphereEnabled;
        if (ImGui::Checkbox("Enabled##sphere", &sphereEnabled))
            wp.sphereEnabled = sphereEnabled;

        sphereXView_.setValue(wp.sphereCenter.x);
        sphereXView_.show();
        wp.sphereCenter.x = sphereXView_.getValue();

        sphereYView_.setValue(wp.sphereCenter.y);
        sphereYView_.show();
        wp.sphereCenter.y = sphereYView_.getValue();

        sphereZView_.setValue(wp.sphereCenter.z);
        sphereZView_.show();
        wp.sphereCenter.z = sphereZView_.getValue();

        sphereRView_.setValue(wp.sphereRadius);
        sphereRView_.show();
        wp.sphereRadius = sphereRView_.getValue();
    }

    ImGui::Separator();
    ImGui::Text("Self-Collision");
    {
        auto& sp = world_->getWorld().solverParams();
        bool enabled = sp.selfCollisionEnabled;
        if (ImGui::Checkbox("Enabled##selfcol", &enabled))
            sp.selfCollisionEnabled = enabled;

        selfColThicknessView_.setValue(sp.selfCollisionThickness);
        selfColThicknessView_.show();
        sp.selfCollisionThickness = selfColThicknessView_.getValue();
    }

    ImGui::End();
}

} // namespace Phantom
