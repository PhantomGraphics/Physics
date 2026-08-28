#pragma once

#include "CGLib/VkAppBase/IVkSubRenderer.h"
#include "CGLib/UIWidgets/Button.h"
#include "CGLib/UIWidgets/FloatView.h"
#include "CGLib/UIWidgets/IntView.h"

#include "SoftBodyWorld.h"

#include <functional>

namespace Phantom {

class SoftBodyControlPanel : public ::VKG::IVkUIPanel {
public:
    explicit SoftBodyControlPanel(SoftBodyWorld* w) : world_(w) {}

    void setOnWorldChanged(std::function<void()> fn) { onWorldChanged_ = std::move(fn); }

    void onImGui() override;

private:
    SoftBodyWorld*    world_ = nullptr;
    std::function<void()>  onWorldChanged_;

    bool widgetsInitialized_ = false;
    void initWidgets();

    UI::Button runButton_   {"Run/Pause"};
    UI::Button stepButton_  {"Step"};
    UI::Button resetButton_ {"Reset"};

    UI::FloatView timeStepView_  {"TimeStep",   0.016f};
    UI::IntView   subStepsView_  {"SubSteps",   10};
    UI::IntView   iterView_      {"Iterations", 5};

    UI::FloatView gravYView_     {"GravY",  -9.8f};
    UI::FloatView sphereXView_   {"SphereX", 0.f};
    UI::FloatView sphereYView_   {"SphereY", 0.f};
    UI::FloatView sphereZView_   {"SphereZ", 0.f};
    UI::FloatView sphereRView_   {"SphereR", 0.4f};

    UI::FloatView selfColThicknessView_ {"Thickness", 0.02f};
};

} // namespace Phantom
