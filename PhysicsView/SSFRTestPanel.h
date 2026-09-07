#pragma once

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"
#include "../../CGLib/UIWidgets/IView.h"
#include "../../CGLib/UIWidgets/Button.h"
#include "../../CGLib/UIWidgets/BoolView.h"
#include "../../CGLib/UIWidgets/ComboBox.h"
#include "../../CGLib/UIWidgets/FloatSlider.h"
#include "../../CGLib/UIWidgets/IntSlider.h"
#include "../../CGLib/UIWidgets/Label.h"
#include "../../CGLib/UIWidgets/Row.h"
#include "../../CGLib/UIWidgets/Separator.h"
#include "../../CGLib/UIWidgets/Spacing.h"

#include "IEmbeddedPanel.h"

#include <string>
#include <vector>

namespace Phantom {
    class SSFluidRenderer;

/**
 * @brief Test-only page: synthetic particle sets for SSFR debugging.
 *
 * buildUi() runs once from init() (after bindSSFRRenderer()). drawContents()
 * is contents_.show(). show_/active_/preset_/count_/radius_/positions_ stay
 * panel fields that FluidApp drives the render loop from (isActive(),
 * consumeDirty(), getPositions()).
 */
class SSFRTestPanel : public ::VKG::IVkUIPanel, public IEmbeddedPanel {
public:
    enum class Preset { Sphere = 0, DamBreak = 1, Wave = 2 };

    void setVisible(bool v) { show_ = v; }
    bool isVisible()  const { return show_; }
    bool isActive()   const { return active_; }

    void bindSSFRRenderer(SSFluidRenderer* r) { ssfrRenderer_ = r; }
    void init();

    bool consumeDirty();

    const std::vector<glm::vec3>& getPositions() const { return positions_; }

    void onImGui() override;
    void drawContents() override;

private:
    bool   show_   = false;
    bool   active_ = false;
    bool   dirty_  = false;
    Preset preset_ = Preset::Sphere;
    int    count_  = 3000;
    float  radius_ = 12.f;

    SSFluidRenderer* ssfrRenderer_ = nullptr;

    std::vector<glm::vec3> positions_;

    bool uiBuilt_ = false;
    void buildUi();

    void generate();
    void genSphere();
    void genDamBreak();
    void genWave();

    std::string particleCountText() const;

    UI::IView contents_ {"SSFRTest"};
    UI::Label testOnlyLabel_ {std::string("Test-only page -- synthetic particle sets for SSFR debugging."),
                              UI::Label::Style::Disabled};
    UI::BoolView activeView_ {"Test Mode"};

    UI::IView    activeGroup_ {"SSFRTestActive"};
    UI::ComboBox presetCombo_ {"Preset"};
    UI::FloatSlider radiusSlider_ {"szradius", 4.f, 25.f};
    UI::IntSlider   countSlider_  {"Target", 500, 8000};
    UI::Row      generateRow_ {"GenerateRow"};
    UI::Button   generateButton_ {"Generate"};
    UI::Label    particleCountLabel_ {[this] { return particleCountText(); }};

    UI::IView    comparisonGroup_ {"SSFRTestComparison"};
    UI::Separator comparisonSeparator_;
    UI::Label    anisoComparisonHeader_ {std::string("--- Anisotropic Comparison ---"), UI::Label::Style::Disabled};
    UI::BoolView anisoView_       {"Anisotropic ON"};
    UI::BoolView depthSmoothView_ {"Depth Smoothing ON"};
    UI::Spacing  spacing1_;
    UI::Label    compareModesLabel_ {std::string("Compare modes:"), UI::Label::Style::Disabled};
    UI::Row      depthCompareRow_ {"DepthCompareRow"};
    UI::Button   rawDepthButton_    {"Raw Depth"};
    UI::Button   smoothDepthButton_ {"Smooth Depth"};
    UI::Row      thickCompareRow_ {"ThickCompareRow"};
    UI::Button   rawThickButton_    {"Raw Thick"};
    UI::Button   smoothThickButton_ {"Smooth Thick"};
    UI::Row      reflCompareRow_ {"ReflCompareRow"};
    UI::Button   reflectionButton_  {"Reflection"};
    UI::Button   fullSSFRButton_    {"Full SSFR"};
    UI::Spacing  spacing2_;
    UI::Label    instr1_ {std::string("1. Enable Test Mode with Sphere preset"), UI::Label::Style::Wrapped};
    UI::Label    instr2_ {std::string("2. Click 'Raw Depth' then 'Smooth Depth' to see depth filter"), UI::Label::Style::Wrapped};
    UI::Label    instr3_ {std::string("3. In 'Reflection' mode, toggle 'Anisotropic ON' to see normal change"), UI::Label::Style::Wrapped};
};

} // namespace Phantom
