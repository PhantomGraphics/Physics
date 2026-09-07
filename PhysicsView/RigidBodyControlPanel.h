#pragma once

#include "CGLib/VkAppBase/IVkSubRenderer.h"
#include "CGLib/UIWidgets/IView.h"
#include "CGLib/UIWidgets/Button.h"
#include "CGLib/UIWidgets/ComboBox.h"
#include "CGLib/UIWidgets/FloatView.h"
#include "CGLib/UIWidgets/IntView.h"
#include "CGLib/UIWidgets/Label.h"
#include "CGLib/UIWidgets/Row.h"
#include "CGLib/UIWidgets/Section.h"

#include "RigidBodyWorld.h"
#include "IEmbeddedPanel.h"

#include <functional>
#include <string>

namespace Phantom {

/**
 * @brief Rigid-body scene controls, assembled declaratively.
 *
 * The whole widget tree (preset combo, Run/Step/Reset row, live status line,
 * simulation-parameter binding, sphere/box spawners) is built once in
 * buildUi() from the constructor. drawContents() only calls contents_.show();
 * every per-frame setValue/getValue round-trip, SameLine and separator that
 * the old immediate-mode version did is now owned by the widgets themselves
 * (see docs/todo/PLAN_physicsview_declarative_ui.md Phase 1).
 *
 * Bindings capture `this` and reach the RigidBodySolver through world_ on
 * every call, so a preset switch or reset (which clears bodies but keeps the
 * same solver/params object) never leaves a binding pointing at freed state.
 */
class RigidBodyControlPanel : public ::VKG::IVkUIPanel, public IEmbeddedPanel {
public:
    explicit RigidBodyControlPanel(RigidBodyWorld* w);

    void setOnWorldChanged(std::function<void()> fn) { onWorldChanged_ = std::move(fn); }
    void setVisible(bool visible) { visible_ = visible; }
    bool isVisible() const { return visible_; }

    void onImGui() override;
    void drawContents() override;

private:
    RigidBodyWorld*  world_ = nullptr;
    std::function<void()> onWorldChanged_;
    bool visible_ = true;

    bool uiBuilt_ = false;
    void buildUi();
    void notifyWorldChanged() { if (onWorldChanged_) onWorldChanged_(); }
    std::string statusText() const;

    // Content root -- the single entry point drawContents() shows.
    UI::IView contents_ {"RigidBodyControl"};

    UI::ComboBox presetCombo_ {"Preset"};

    UI::Row    actionsRow_   {"Actions"};
    UI::Button runButton_    {"Run/Pause"};
    UI::Button stepButton_   {"Step"};
    UI::Button resetButton_  {"Reset"};
    UI::Label  statusLabel_  {[this] { return statusText(); }};

    UI::Section   simSection_    {"Simulation"};
    UI::FloatView timeStepView_  {"TimeStep",   0.016f};
    UI::IntView   iterView_      {"SolverIter", 10};
    UI::FloatView betaView_      {"Baumgarte",  0.2f};
    UI::FloatView gravYView_     {"GravY",      -9.8f};

    UI::Section   sphereSection_  {"Add Sphere (drops from y=3)"};
    UI::FloatView sphereRadView_  {"SphereR",    0.5f};
    UI::FloatView sphereMassView_ {"SphereMass", 1.0f};
    UI::FloatView sphereRestView_ {"SphereRest", 0.3f};
    UI::Button    addSphereBtn_   {"Add Sphere"};

    UI::Section   boxSection_   {"Add Box (drops from y=3)"};
    UI::FloatView boxHxView_    {"BoxHx",   0.5f};
    UI::FloatView boxHyView_    {"BoxHy",   0.5f};
    UI::FloatView boxHzView_    {"BoxHz",   0.5f};
    UI::FloatView boxMassView_  {"BoxMass", 1.0f};
    UI::Button    addBoxBtn_    {"Add Box"};
};

} // namespace Phantom
