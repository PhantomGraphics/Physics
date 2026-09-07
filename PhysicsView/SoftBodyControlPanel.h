#pragma once

#include "CGLib/VkAppBase/IVkSubRenderer.h"
#include "CGLib/UIWidgets/IView.h"
#include "CGLib/UIWidgets/Button.h"
#include "CGLib/UIWidgets/BoolView.h"
#include "CGLib/UIWidgets/ComboBox.h"
#include "CGLib/UIWidgets/FloatView.h"
#include "CGLib/UIWidgets/IntView.h"
#include "CGLib/UIWidgets/Label.h"
#include "CGLib/UIWidgets/Row.h"
#include "CGLib/UIWidgets/Section.h"

#include "SoftBodyWorld.h"
#include "IEmbeddedPanel.h"

#include <functional>
#include <string>

namespace Phantom {

/**
 * @brief Soft-body scene controls, assembled declaratively (Phase 3).
 *
 * buildUi() (from the constructor) wires the preset combo, Run/Step/Reset row,
 * live status line and the Solver / Sphere Collider / Self-Collision sections.
 * drawContents() is just contents_.show(). Bindings reach
 * world_->getWorld().solverParams() / params() on each call, so a preset
 * switch (which keeps the same solver/Params objects) stays valid.
 */
class SoftBodyControlPanel : public ::VKG::IVkUIPanel, public IEmbeddedPanel {
public:
    explicit SoftBodyControlPanel(SoftBodyWorld* w);

    void setOnWorldChanged(std::function<void()> fn) { onWorldChanged_ = std::move(fn); }
    void setVisible(bool visible) { visible_ = visible; }
    bool isVisible() const { return visible_; }

    void onImGui() override;
    void drawContents() override;

private:
    SoftBodyWorld*    world_ = nullptr;
    std::function<void()>  onWorldChanged_;
    bool visible_ = true;

    bool uiBuilt_ = false;
    void buildUi();
    void notifyWorldChanged() { if (onWorldChanged_) onWorldChanged_(); }
    std::string statusText() const;

    UI::IView contents_ {"SoftBodyControl"};

    UI::ComboBox presetCombo_ {"Preset"};

    UI::Row    actionsRow_   {"Actions"};
    UI::Button runButton_    {"Run/Pause"};
    UI::Button stepButton_   {"Step"};
    UI::Button resetButton_  {"Reset"};
    UI::Label  statusLabel_  {[this] { return statusText(); }};

    UI::Section   solverSection_ {"Solver"};
    UI::FloatView timeStepView_  {"TimeStep",   0.016f};
    UI::IntView   subStepsView_  {"SubSteps",   10};
    UI::IntView   iterView_      {"Iterations", 5};
    UI::FloatView gravYView_     {"GravY",  -9.8f};

    UI::Section   sphereSection_ {"Sphere Collider"};
    UI::BoolView  sphereEnabledView_ {"Enabled##sphere"};
    UI::FloatView sphereXView_   {"SphereX", 0.f};
    UI::FloatView sphereYView_   {"SphereY", 0.f};
    UI::FloatView sphereZView_   {"SphereZ", 0.f};
    UI::FloatView sphereRView_   {"SphereR", 0.4f};

    UI::Section   selfCollisionSection_ {"Self-Collision"};
    UI::BoolView  selfColEnabledView_ {"Enabled##selfcol"};
    UI::FloatView selfColThicknessView_ {"Thickness", 0.02f};
};

} // namespace Phantom
