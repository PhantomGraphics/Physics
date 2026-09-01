#pragma once

#include "CGLib/VkAppBase/IVkSubRenderer.h"
#include "CGLib/UIWidgets/Button.h"
#include "CGLib/UIWidgets/FloatView.h"
#include "CGLib/UIWidgets/IntView.h"

#include "RigidBodyWorld.h"

#include <functional>

namespace Phantom {

class RigidBodyControlPanel : public ::VKG::IVkUIPanel {
public:
    explicit RigidBodyControlPanel(RigidBodyWorld* w) : world_(w) {}

    void setOnWorldChanged(std::function<void()> fn) { onWorldChanged_ = std::move(fn); }
    void setVisible(bool visible) { visible_ = visible; }
    bool isVisible() const { return visible_; }

    void onImGui() override;

private:
    RigidBodyWorld*  world_ = nullptr;
    std::function<void()> onWorldChanged_;
    bool visible_ = true;

    bool widgetsInitialized_ = false;
    void initWidgets();

    UI::Button runButton_    {"Run/Pause"};
    UI::Button stepButton_   {"Step"};
    UI::Button resetButton_  {"Reset"};

    UI::FloatView timeStepView_  {"TimeStep",   0.016f};
    UI::IntView   iterView_      {"SolverIter", 10};
    UI::FloatView betaView_      {"Baumgarte",  0.2f};
    UI::FloatView gravYView_     {"GravY",      -9.8f};

    UI::FloatView sphereRadView_  {"SphereR",    0.5f};
    UI::FloatView sphereMassView_ {"SphereMass", 1.0f};
    UI::FloatView sphereRestView_ {"SphereRest", 0.3f};
    UI::Button    addSphereBtn_   {"Add Sphere"};

    UI::FloatView boxHxView_   {"BoxHx",   0.5f};
    UI::FloatView boxHyView_   {"BoxHy",   0.5f};
    UI::FloatView boxHzView_   {"BoxHz",   0.5f};
    UI::FloatView boxMassView_ {"BoxMass", 1.0f};
    UI::Button    addBoxBtn_   {"Add Box"};
};

} // namespace Phantom
