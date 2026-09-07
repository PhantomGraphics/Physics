#include "pch.h"
#include "RigidBodyControlPanel.h"

namespace Phantom {

static const char* const kPresetNames[] = {
    "SphereDrop", "BoxDrop", "Stacking",
    "NewtonsCradle", "Billiards", "SphereBoxCollision", "Custom"
};
static const ScenePreset kPresetValues[] = {
    ScenePreset::SphereDrop, ScenePreset::BoxDrop, ScenePreset::Stacking,
    ScenePreset::NewtonsCradle, ScenePreset::Billiards,
    ScenePreset::SphereBoxCollision, ScenePreset::Custom
};
static constexpr int kPresetCount = 7;

RigidBodyControlPanel::RigidBodyControlPanel(RigidBodyWorld* w) : world_(w) {
    buildUi();
}

void RigidBodyControlPanel::buildUi() {
    if (uiBuilt_) return;
    uiBuilt_ = true;

    // --- Preset ---------------------------------------------------------
    for (int i = 0; i < kPresetCount; ++i) presetCombo_.addItem(kPresetNames[i]);
    presetCombo_.bind(
        [this] { return static_cast<int>(world_->currentPreset()); },
        [this](int i) {
            if (i < 0 || i >= kPresetCount) return;
            world_->setPreset(kPresetValues[i]);
            notifyWorldChanged();
        });

    // --- Run / Step / Reset -------------------------------------------
    runButton_.setFunction([this] {
        world_->setRunning(!world_->isRunning());
    });
    stepButton_.setFunction([this] {
        world_->step();
        notifyWorldChanged();
    });
    resetButton_.setFunction([this] {
        world_->reset();
        notifyWorldChanged();
    });
    actionsRow_.add(&runButton_);
    actionsRow_.add(&stepButton_);
    actionsRow_.add(&resetButton_);

    // --- Simulation parameters --------------------------------------
    // Fetched through world_ every frame: reset()/setPreset() keep the same
    // RigidBodySolver and Params, so these stay valid across a scene switch.
    timeStepView_.bind(
        [this] { return world_->getWorld().timeStep; },
        [this](float v) { world_->getWorld().timeStep = v; });
    iterView_.bind(
        [this] { return world_->getWorld().params().solverIterations; },
        [this](int v) { world_->getWorld().params().solverIterations = v; });
    betaView_.bind(
        [this] { return world_->getWorld().params().baumgarteBeta; },
        [this](float v) { world_->getWorld().params().baumgarteBeta = v; });
    gravYView_.bind(
        [this] { return world_->getWorld().params().gravity.y; },
        [this](float v) { world_->getWorld().params().gravity.y = v; });
    simSection_.add(&timeStepView_);
    simSection_.add(&iterView_);
    simSection_.add(&betaView_);
    simSection_.add(&gravYView_);

    // --- Add Sphere (draft values held in the sphere*View_ members) ----
    addSphereBtn_.setFunction([this] {
        world_->addSphere(
            {0.f, 3.f, 0.f},
            sphereRadView_.getValue(),
            sphereMassView_.getValue(),
            sphereRestView_.getValue());
        notifyWorldChanged();
    });
    sphereSection_.add(&sphereRadView_);
    sphereSection_.add(&sphereMassView_);
    sphereSection_.add(&sphereRestView_);
    sphereSection_.add(&addSphereBtn_);

    // --- Add Box ------------------------------------------------------
    addBoxBtn_.setFunction([this] {
        world_->addBox(
            {0.f, 3.f, 0.f},
            {boxHxView_.getValue(), boxHyView_.getValue(), boxHzView_.getValue()},
            boxMassView_.getValue());
        notifyWorldChanged();
    });
    boxSection_.add(&boxHxView_);
    boxSection_.add(&boxHyView_);
    boxSection_.add(&boxHzView_);
    boxSection_.add(&boxMassView_);
    boxSection_.add(&addBoxBtn_);

    // --- Assemble the content root ----------------------------------
    contents_.add(&presetCombo_);
    contents_.add(&actionsRow_);
    contents_.add(&statusLabel_);
    contents_.add(&simSection_);
    contents_.add(&sphereSection_);
    contents_.add(&boxSection_);
}

std::string RigidBodyControlPanel::statusText() const {
    const auto& w = world_->getWorld();
    char buf[128];
    std::snprintf(buf, sizeof(buf),
        "Bodies:%d  Contacts:%d\nRunning: %s",
        static_cast<int>(w.getBodies().size()),
        static_cast<int>(w.getContacts().size()),
        w.isRunning() ? "Yes" : "No");
    return buf;
}

void RigidBodyControlPanel::onImGui() {
    if (!world_ || !visible_) return;

    UI::Immediate::setNextWindowPosition(700.f, 35.f);
    UI::Immediate::setNextWindowSize(300.f, 640.f);
    if (!UI::Immediate::beginWindow("Rigid Body Control", &visible_)) {
        UI::Immediate::endWindow();
        return;
    }
    drawContents();
    UI::Immediate::endWindow();
}

void RigidBodyControlPanel::drawContents() {
    if (!world_) return;
    contents_.show();
}

} // namespace Phantom
