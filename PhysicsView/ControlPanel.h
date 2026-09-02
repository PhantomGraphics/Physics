#pragma once

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"
#include "../../CGLib/UIWidgets/Button.h"
#include "../../CGLib/UIWidgets/BoolView.h"
#include "../../CGLib/UIWidgets/FloatView.h"
#include "../../CGLib/UIWidgets/ComboBox.h"
#include "../../CGLib/UIWidgets/Box3dView.h"

#include "FluidWorld.h"

namespace Phantom {

class ControlPanel : public ::VKG::IVkUIPanel {
public:
    explicit ControlPanel(FluidWorld* world) : world_(world) {}

    void setOnWorldChanged(std::function<void()> fn) { onWorldChanged_ = std::move(fn); }
    void setVisible(bool visible) { visible_ = visible; }
    bool isVisible() const { return visible_; }

    void onImGui() override;

private:
    FluidWorld* world_ = nullptr;
    std::function<void()> onWorldChanged_;
    bool visible_ = true;

    Phantom::UI::ComboBox methodCombo_   { "Method"   };
    Phantom::UI::Button   runButton_     { "Run/Pause" };
    Phantom::UI::Button   stepButton_    { "Step"     };
    Phantom::UI::Button   resetButton_   { "Reset"    };

    Phantom::UI::FloatView timeStepView_  { "TimeStep",     0.01f  };
    Phantom::UI::FloatView radiusView_    { "Radius",       1.0f   };
    Phantom::UI::FloatView effectLenView_ { "EffectLength", 2.25f  };
    Phantom::UI::FloatView densityView_   { "Density",      1.0f   };
    // Raw pressure/relaxation coefficient (PBSPH/GPU_CSPH). Not shown for
    // WCSPH/DFSPH -- see pressureCoeScaleView_ below.
    Phantom::UI::FloatView stiffnessView_ { "Stiffness",    20.0f  };
    // Scale-invariant replacement for the raw Stiffness slider, WCSPH and
    // DFSPH only (internal design notes section 4/Phase 1):
    // a raw pressureCoe means something different at every scene scale, so
    // these two instead derive it as pressureCoeScale * effectLength via
    // WCSPHFluid::estimatePressureCoe()/setPressureCoeFromScale() (see
    // FluidWorld::createWCSPH()/createDFSPH()).
    Phantom::UI::FloatView pressureCoeScaleView_ { "Pressure Coe Scale", 1960.0f };
    Phantom::UI::FloatView viscosityView_ { "Viscosity",    5.0f   };

    Phantom::UI::Box3dView fluidBoundsView_ { "Fluid Bounds" };
    Phantom::UI::Box3dView boundaryView_    { "Boundary"     };

    // Mesh Boundary section (LoadMeshBoundary/ClearMeshBoundary demo wiring).
    char meshBoundaryPathBuf_[260] = "";

    // Emitter section (internal design notes): scratch
    // fields for the "new emitter" form, cleared to a plain upward jet on
    // "Add" (mirrors Emitter's own struct defaults).
    float newEmitterCenter_[3] = { 0.0f, 0.0f, 0.0f };
    float newEmitterRadius_ = 0.1f;
    float newEmitterRate_ = 50.0f;
    float newEmitterDirection_[3] = { 0.0f, 1.0f, 0.0f };
    float newEmitterSpeed_ = 1.0f;

    // Outflow region section: scratch AABB for the "new outflow region" form.
    Phantom::UI::Box3dView newOutflowRegionView_ { "New Outflow Region" };

    bool widgetsInitialized_ = false;
    void initWidgets();
};

} // namespace Phantom
