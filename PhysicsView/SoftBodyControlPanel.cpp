#include "pch.h"
#include "SoftBodyControlPanel.h"

namespace Phantom {

static const char* const kPresetNames[] = {
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

SoftBodyControlPanel::SoftBodyControlPanel(SoftBodyWorld* w) : world_(w)
{
    buildUi();
}

void SoftBodyControlPanel::buildUi()
{
    if (uiBuilt_) return;
    uiBuilt_ = true;

    for (int i = 0; i < kNumPresets; ++i) presetCombo_.addItem(kPresetNames[i]);
    presetCombo_.bind(
        [this] { return static_cast<int>(world_->currentPreset()); },
        [this](int i) {
            if (i < 0 || i >= kNumPresets) return;
            world_->setPreset(kPresetValues[i]);
            notifyWorldChanged();
        });

    runButton_.setFunction([this] { world_->setRunning(!world_->isRunning()); });
    stepButton_.setFunction([this] {
        world_->getWorld().setRunning(true);
        world_->step();
        world_->getWorld().setRunning(false);
        notifyWorldChanged();
    });
    resetButton_.setFunction([this] { world_->reset(); notifyWorldChanged(); });
    actionsRow_.add(&runButton_);
    actionsRow_.add(&stepButton_);
    actionsRow_.add(&resetButton_);

    // --- Solver (XPBDSolver::Params, applied live per step) ---------
    timeStepView_.bind([this] { return world_->getWorld().solverParams().timeStep; },
                       [this](float v) { world_->getWorld().solverParams().timeStep = v; });
    subStepsView_.bind([this] { return world_->getWorld().solverParams().numSubsteps; },
                       [this](int v) { world_->getWorld().solverParams().numSubsteps = v; });
    iterView_.bind([this] { return world_->getWorld().solverParams().numIterations; },
                   [this](int v) { world_->getWorld().solverParams().numIterations = v; });
    gravYView_.bind([this] { return world_->getWorld().solverParams().gravity.y; },
                    [this](float v) { world_->getWorld().solverParams().gravity.y = v; });
    solverSection_.add(&timeStepView_);
    solverSection_.add(&subStepsView_);
    solverSection_.add(&iterView_);
    solverSection_.add(&gravYView_);

    // --- Sphere Collider (SoftBodySolver::Params) ------------------
    sphereEnabledView_.bind([this] { return world_->getWorld().params().sphereEnabled; },
                            [this](bool v) { world_->getWorld().params().sphereEnabled = v; });
    sphereXView_.bind([this] { return world_->getWorld().params().sphereCenter.x; },
                      [this](float v) { world_->getWorld().params().sphereCenter.x = v; });
    sphereYView_.bind([this] { return world_->getWorld().params().sphereCenter.y; },
                      [this](float v) { world_->getWorld().params().sphereCenter.y = v; });
    sphereZView_.bind([this] { return world_->getWorld().params().sphereCenter.z; },
                      [this](float v) { world_->getWorld().params().sphereCenter.z = v; });
    sphereRView_.bind([this] { return world_->getWorld().params().sphereRadius; },
                      [this](float v) { world_->getWorld().params().sphereRadius = v; });
    sphereSection_.add(&sphereEnabledView_);
    sphereSection_.add(&sphereXView_);
    sphereSection_.add(&sphereYView_);
    sphereSection_.add(&sphereZView_);
    sphereSection_.add(&sphereRView_);

    // --- Self-Collision (XPBDSolver::Params) ---------------------
    selfColEnabledView_.bind([this] { return world_->getWorld().solverParams().selfCollisionEnabled; },
                             [this](bool v) { world_->getWorld().solverParams().selfCollisionEnabled = v; });
    selfColThicknessView_.bind([this] { return world_->getWorld().solverParams().selfCollisionThickness; },
                               [this](float v) { world_->getWorld().solverParams().selfCollisionThickness = v; });
    selfCollisionSection_.add(&selfColEnabledView_);
    selfCollisionSection_.add(&selfColThicknessView_);

    contents_.add(&presetCombo_);
    contents_.add(&actionsRow_);
    contents_.add(&statusLabel_);
    contents_.add(&solverSection_);
    contents_.add(&sphereSection_);
    contents_.add(&selfCollisionSection_);
}

std::string SoftBodyControlPanel::statusText() const
{
    const auto& w = world_->getWorld();
    char buf[128];
    std::snprintf(buf, sizeof(buf),
        "Bodies:%llu  Particles:%llu\nRunning: %s",
        static_cast<unsigned long long>(w.getBodyCount()),
        static_cast<unsigned long long>(w.getParticleCount()),
        world_->isRunning() ? "Yes" : "No");
    return buf;
}

void SoftBodyControlPanel::onImGui()
{
    if (!world_ || !visible_) return;

    UI::Immediate::setNextWindowPosition(10.f, 35.f);
    UI::Immediate::setNextWindowSize(300.f, 560.f);
    if (!UI::Immediate::beginWindow("Soft Body Control", &visible_)) {
        UI::Immediate::endWindow();
        return;
    }
    drawContents();
    UI::Immediate::endWindow();
}

void SoftBodyControlPanel::drawContents()
{
    if (!world_) return;
    contents_.show();
}

} // namespace Phantom
