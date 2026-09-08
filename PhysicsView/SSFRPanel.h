#pragma once

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"
#include "../../CGLib/UIWidgets/IView.h"
#include "../../CGLib/UIWidgets/BoolView.h"
#include "../../CGLib/UIWidgets/ComboBox.h"
#include "../../CGLib/UIWidgets/FloatSlider.h"
#include "../../CGLib/UIWidgets/Label.h"
#include "../../CGLib/UIWidgets/Section.h"
#include "../../CGLib/UIWidgets/Separator.h"

#include "FluidWorld.h"
#include "IEmbeddedPanel.h"

namespace Phantom {
    class SSFluidRenderer;
    class SSFRTestPanel;

/**
 * @brief SSFR (screen-space fluid rendering) controls, assembled declaratively.
 *
 * buildUi() is run once from init() (after bindRenderer()/bindWorld()).
 * drawContents() is just contents_.show(). The SSFR-enabled flag and mode
 * index are panel fields that FluidApp reads via isEnabled()/getModeIndex();
 * everything else binds straight to the SSFluidRenderer's getters/setters or
 * whiteWaterParams(). The old per-frame renderer_->setShowSpray()/setShowFoam()
 * push moved to FluidApp::onUpdate (Phase 4).
 */
class SSFRPanel : public ::VKG::IVkUIPanel, public IEmbeddedPanel {
public:
    void bindRenderer(SSFluidRenderer* r) { renderer_ = r; }
    void bindWorld(FluidWorld* w) { world_ = w; }
    // Folds the former "SSFR Test" page in as a collapsible "SSFR Debug"
    // section at the bottom of this panel. Must be called before init();
    // the debug panel itself must be init()'d too (order-independent).
    void bindDebugPanel(SSFRTestPanel* p) { debugPanel_ = p; }
    void init();

    bool isEnabled() const { return enabled_; }
    int  getModeIndex() const { return modeIndex_; }
    // Scenario / command-driven (the bound enableCheck_ / modeCombo_ widgets read
    // these same fields, so the GUI stays in sync).
    void setEnabled(bool v) { enabled_ = v; }
    void setModeIndex(int i) { modeIndex_ = i; }
    void setVisible(bool visible) { visible_ = visible; }
    bool isVisible() const { return visible_; }

    void onImGui() override;
    void drawContents() override;

private:
    SSFluidRenderer* renderer_ = nullptr;
    FluidWorld* world_ = nullptr;
    SSFRTestPanel* debugPanel_ = nullptr;

    bool enabled_ = false;
    bool visible_ = true;
    int modeIndex_ = 5;

    bool uiBuilt_ = false;
    void buildUi();

    UI::IView    contents_    {"SSFRControl"};
    UI::BoolView enableCheck_ {"SSFR"};

    // Wraps debugPanel_->contentsView() (collapsed by default). Only added to
    // contents_ when bindDebugPanel() was called.
    UI::Section  debugSection_ {"SSFR Debug (synthetic particles)", false};

    UI::IView    enabledGroup_ {"SSFREnabled"};
    UI::ComboBox modeCombo_    {"SSFR Mode"};
    UI::BoolView sprayCheck_   {"Use Spray"};
    UI::BoolView foamCheck_    {"Use Foam"};

    UI::IView    rendererGroup_ {"SSFRRenderer"};
    UI::Label       thickHeader_          {std::string("--- Thickness Bilateral ---"), UI::Label::Style::Disabled};
    UI::FloatSlider thicknessSigmaSSlider_ {"Thickness SigmaS", 0.5f, 6.0f};
    UI::FloatSlider thicknessSigmaRSlider_ {"Thickness SigmaR", 0.01f, 0.25f};
    UI::Separator   sep1_;
    UI::Label       anisoHeader_          {std::string("--- Anisotropic (Thickness + Depth) ---"), UI::Label::Style::Disabled};
    UI::BoolView    anisoCheck_           {"Anisotropic Smoothing"};
    UI::FloatSlider anisotropySlider_     {"Anisotropy", 0.0f, 3.0f};
    UI::FloatSlider anisoGradScaleSlider_ {"Aniso Grad Scale", 0.0f, 20.0f};
    UI::Separator   sep2_;
    UI::Label       depthHeader_          {std::string("--- Depth Bilateral (Surface Normals) ---"), UI::Label::Style::Disabled};
    UI::BoolView    depthSmoothCheck_     {"Depth Smoothing"};
    UI::FloatSlider depthSigmaSSlider_    {"Depth SigmaS", 0.5f, 6.0f};
    UI::FloatSlider depthSigmaRSlider_    {"Depth SigmaR", 0.005f, 0.2f};
    UI::Separator   sep3_;
    UI::FloatSlider sprayOpacitySlider_   {"Spray Opacity", 0.0f, 1.0f};
    UI::FloatSlider foamOpacitySlider_    {"Foam Opacity", 0.0f, 1.0f};
};

} // namespace Phantom
