#pragma once

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"
#include "../../CGLib/UIWidgets/IView.h"
#include "../../CGLib/UIWidgets/BoolView.h"
#include "../../CGLib/UIWidgets/ComboBox.h"
#include "../../CGLib/UIWidgets/FloatSlider.h"
#include "../../CGLib/UIWidgets/IntSlider.h"
#include "../../CGLib/UIWidgets/Label.h"
#include "../../CGLib/UIWidgets/Section.h"
#include "../../CGLib/UIWidgets/Separator.h"

#include "FluidWorld.h"
#include "../FluidRenderer/SSFRAnisotropy.h"
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
    // Anisotropic kernel (Yu & Turk ellipsoid splats). The on/off flag lives on
    // the renderer (SSFluidRenderer::setAnisotropicKernel); the PCA settings
    // live here and FluidApp feeds them to SSFRAnisotropyBuilder. The kernel
    // generation bumps on every change so FluidApp can recompute while paused.
    SSFRKernelSettings& kernelSettings() { return kernelSettings_; }
    const SSFRKernelSettings& kernelSettings() const { return kernelSettings_; }
    void markKernelChanged() { ++kernelGeneration_; }
    // Kernel on/off (forwards to the bound renderer; command-driven).
    void setAnisotropicKernel(bool v);
    bool getAnisotropicKernel() const;
    /// Whether the last SSFR pre-pass actually drew ellipsoids.
    bool isAnisotropicKernelActive() const;
    uint64_t kernelGeneration() const { return kernelGeneration_; }
    void setKernelStats(const SSFRKernelStats& s) { kernelStats_ = s; }
    const SSFRKernelStats& kernelStats() const { return kernelStats_; }
    // Set by FluidApp: the kernel is unavailable in the current mode (GPU_CSPH).
    void setKernelUnavailableReason(std::string r) { kernelUnavailable_ = std::move(r); }

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

    SSFRKernelSettings kernelSettings_;
    SSFRKernelStats    kernelStats_;
    uint64_t           kernelGeneration_ = 0;
    std::string        kernelUnavailable_;

    bool uiBuilt_ = false;
    std::string kernelStatsText() const;
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
    UI::Separator   sepKernel_;
    UI::Label       kernelHeader_         {std::string("--- Anisotropic Kernel (Yu & Turk ellipsoids) ---"), UI::Label::Style::Disabled};
    UI::BoolView    kernelCheck_          {"Anisotropic Kernel"};
    UI::IView       kernelGroup_          {"SSFRKernel"};
    UI::FloatSlider kernelSearchSlider_   {"Kernel Search (x radius)", 2.0f, 8.0f};
    UI::FloatSlider kernelMaxRatioSlider_ {"Kernel Max Ratio", 1.0f, 8.0f};
    UI::FloatSlider kernelIsolatedSlider_ {"Kernel Isolated Scale", 0.1f, 1.0f};
    UI::IntSlider   kernelMinNbrSlider_   {"Kernel Min Neighbors", 1, 64};
    UI::FloatSlider kernelSmoothSlider_   {"Kernel Center Smoothing", 0.0f, 1.0f};
    UI::Label       kernelStatsLabel_     {std::function<std::string()>([this] { return kernelStatsText(); }), UI::Label::Style::Disabled};
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
