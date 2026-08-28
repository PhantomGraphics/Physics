#pragma once

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"
#include "../../CGLib/UIWidgets/BoolView.h"
#include "../../CGLib/UIWidgets/ComboBox.h"

#include "FluidWorld.h"

namespace Phantom {
    class SSFluidRenderer;

class SSFRPanel : public ::VKG::IVkUIPanel {
public:
    void bindRenderer(SSFluidRenderer* r) { renderer_ = r; }
    void bindWorld(FluidWorld* w) { world_ = w; }

    bool isEnabled() const { return enabled_; }
    int  getModeIndex() const { return modeIndex_; }

    void onImGui() override;

private:
    SSFluidRenderer* renderer_ = nullptr;
    FluidWorld* world_ = nullptr;

    bool enabled_ = false;
    int modeIndex_ = 5;

    Phantom::UI::BoolView  enableCheck_      { "SSFR" };
    Phantom::UI::ComboBox  modeCombo_        { "SSFR Mode" };
    Phantom::UI::BoolView  sprayCheck_       { "Use Spray" };
    Phantom::UI::BoolView  foamCheck_        { "Use Foam" };
    Phantom::UI::BoolView  anisoCheck_       { "Anisotropic Smoothing" };
    Phantom::UI::BoolView  depthSmoothCheck_ { "Depth Smoothing" };

    bool widgetsInitialized_ = false;
    void initWidgets();
};

} // namespace Phantom
