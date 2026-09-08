#pragma once

#include "../../CGLib/UIWidgets/IView.h"
#include "../../CGLib/UIWidgets/BoolView.h"
#include "../../CGLib/UIWidgets/Button.h"
#include "../../CGLib/UIWidgets/ComboBox.h"
#include "../../CGLib/UIWidgets/FloatSlider.h"
#include "../../CGLib/UIWidgets/Label.h"
#include "../../CGLib/UIWidgets/Separator.h"
#include "../../CGLib/UIWidgets/StringView.h"
#include "../../CGLib/UIWidgets/Vector3dView.h"

#include "IEmbeddedPanel.h"

namespace Phantom {

class RenderBackground;
class GltfBodyRenderer;
class GltfSoftRenderer;

/**
 * @brief "glTF Rendering" Control page: background set, environment, and the
 * shared directional light (docs/todo/PLAN_physicsview_gltf_rendering.md Phase 1).
 *
 * A thin declarative panel over RenderBackground. Path/environment I/O is
 * button-driven (file access); the light + transform + IBL toggle are pushed
 * from the widget values every frame (all cheap value stores, no Vulkan work).
 * Widget values are seeded once from RenderBackground's current state.
 */
class RenderingPanel : public IEmbeddedPanel {
public:
    void bind(RenderBackground* bg) { bg_ = bg; }
    void bindRigidBodyRenderer(GltfBodyRenderer* r) { rigidBody_ = r; }
    void bindSoftBodyRenderer(GltfSoftRenderer* r) { softBody_ = r; }
    void init();

    void drawContents() override;

private:
    void buildUi();
    std::string backgroundStatusText() const;
    std::string environmentStatusText() const;
    std::string rigidStatusText() const;
    std::string softStatusText() const;

    RenderBackground* bg_ = nullptr;
    GltfBodyRenderer* rigidBody_ = nullptr;
    GltfSoftRenderer* softBody_ = nullptr;
    bool uiBuilt_ = false;
    bool seeded_  = false;

    UI::IView contents_{"RenderingControl"};

    // Background
    UI::Label       bgHeader_    {std::string("--- Background ---"), UI::Label::Style::Disabled};
    UI::StringView  bgPathInput_ {"Path (.glb/.gltf/.obj/.stl)"};
    UI::Button      loadBtn_     {"Load Background"};
    UI::Button      clearBtn_    {"Clear Background"};
    UI::Label       bgStatus_    {[this] { return backgroundStatusText(); }, UI::Label::Style::Disabled};

    // Transform
    UI::Separator   sep1_;
    UI::Label       xfHeader_    {std::string("--- Transform ---"), UI::Label::Style::Disabled};
    UI::Vector3dView xfPos_      {"Position"};
    UI::Vector3dView xfRot_      {"Rotation (deg XYZ)"};
    UI::FloatSlider  xfScale_    {"Scale", 0.01f, 20.0f};

    // Environment
    UI::Separator   sep2_;
    UI::Label       envHeader_   {std::string("--- Environment ---"), UI::Label::Style::Disabled};
    UI::StringView  envDirInput_ {"Cubemap dir (6x *.png)"};
    UI::Button      setEnvBtn_   {"Load Environment"};
    UI::Button      clearEnvBtn_ {"Clear Environment"};
    UI::BoolView    useIblCheck_ {"Use IBL"};
    UI::BoolView    shadowCheck_ {"Cast Shadows"};
    UI::Label       envStatus_   {[this] { return environmentStatusText(); }, UI::Label::Style::Disabled};

    // Light
    UI::Separator   sep3_;
    UI::Label       lightHeader_ {std::string("--- Directional Light ---"), UI::Label::Style::Disabled};
    UI::Vector3dView lightDir_   {"Direction"};
    UI::Vector3dView lightColor_ {"Color (RGB)"};
    UI::FloatSlider  lightInt_   {"Intensity", 0.0f, 10.0f};

    // Rigid / soft body display mode (wire / shaded / both)
    UI::Separator   sep4_;
    UI::Label       rigidHeader_    {std::string("--- Rigid / Soft Bodies ---"), UI::Label::Style::Disabled};
    UI::ComboBox    rigidModeCombo_ {"Rigid Mode"};
    UI::Label       rigidStatus_    {[this] { return rigidStatusText(); }, UI::Label::Style::Disabled};
    UI::ComboBox    softModeCombo_  {"Soft Mode"};
    UI::Label       softStatus_     {[this] { return softStatusText(); }, UI::Label::Style::Disabled};
};

} // namespace Phantom
