#include "pch.h"
#include "RigidBodyControlPanel.h"

namespace Phantom {

static const char* kPresetNames[] = {
    "SphereDrop", "BoxDrop", "Stacking",
    "NewtonsCradle", "Billiards", "SphereBoxCollision", "Custom"
};
static const ScenePreset kPresetValues[] = {
    ScenePreset::SphereDrop, ScenePreset::BoxDrop, ScenePreset::Stacking,
    ScenePreset::NewtonsCradle, ScenePreset::Billiards,
    ScenePreset::SphereBoxCollision, ScenePreset::Custom
};

void RigidBodyControlPanel::initWidgets() {
    if (widgetsInitialized_) return;
    widgetsInitialized_ = true;

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
    addSphereBtn_.setFunction([this]() {
        world_->addSphere(
            {0.f, 3.f, 0.f},
            sphereRadView_.getValue(),
            sphereMassView_.getValue(),
            sphereRestView_.getValue());
        if (onWorldChanged_) onWorldChanged_();
    });
    addBoxBtn_.setFunction([this]() {
        world_->addBox(
            {0.f, 3.f, 0.f},
            {boxHxView_.getValue(), boxHyView_.getValue(), boxHzView_.getValue()},
            boxMassView_.getValue());
        if (onWorldChanged_) onWorldChanged_();
    });
}

void RigidBodyControlPanel::onImGui() {
    if (!world_) return;
    initWidgets();

    ImGui::SetNextWindowPos(ImVec2(700.f, 35.f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(300.f, 640.f), ImGuiCond_Once);
    if (!ImGui::Begin("Rigid Body Control")) { ImGui::End(); return; }

    {
        int cur = static_cast<int>(world_->currentPreset());
        if (ImGui::Combo("Preset", &cur, kPresetNames, 7)) {
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
        int n = static_cast<int>(world_->getWorld().getBodies().size());
        int c = static_cast<int>(world_->getWorld().getContacts().size());
        ImGui::Text("Bodies:%d  Contacts:%d", n, c);
        ImGui::Text("Running: %s", world_->isRunning() ? "Yes" : "No");
    }

    ImGui::Separator();
    ImGui::Text("Simulation");
    {
        auto& wp = world_->getWorld();
        timeStepView_.setValue(wp.timeStep);
        timeStepView_.show();
        wp.timeStep = timeStepView_.getValue();

        iterView_.setValue(wp.params().solverIterations);
        iterView_.show();
        wp.params().solverIterations = iterView_.getValue();

        betaView_.setValue(wp.params().baumgarteBeta);
        betaView_.show();
        wp.params().baumgarteBeta = betaView_.getValue();

        gravYView_.setValue(wp.params().gravity.y);
        gravYView_.show();
        wp.params().gravity.y = gravYView_.getValue();
    }

    ImGui::Separator();
    ImGui::Text("Add Sphere (drops from y=3)");
    sphereRadView_.show();
    sphereMassView_.show();
    sphereRestView_.show();
    addSphereBtn_.show();

    ImGui::Separator();
    ImGui::Text("Add Box (drops from y=3)");
    boxHxView_.show();
    boxHyView_.show();
    boxHzView_.show();
    boxMassView_.show();
    addBoxBtn_.show();

    ImGui::End();
}

} // namespace Phantom
