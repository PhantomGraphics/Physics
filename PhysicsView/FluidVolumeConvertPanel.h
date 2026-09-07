#pragma once

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"
#include "../../CGLib/UIWidgets/IView.h"
#include "../../CGLib/UIWidgets/Button.h"
#include "../../CGLib/UIWidgets/BoolView.h"
#include "../../CGLib/UIWidgets/FloatView.h"
#include "../../CGLib/UIWidgets/ComboBox.h"
#include "../../CGLib/UIWidgets/FileSaveView.h"
#include "../../CGLib/UIWidgets/Label.h"
#include "../../CGLib/UIWidgets/Separator.h"

#include "FluidWorld.h"
#include "FluidVolumeConverter.h"
#include "FluidMeshConverter.h"
#include "VolumeRenderer.h"
#include "FluidMeshRenderer.h"
#include "IEmbeddedPanel.h"

#include <functional>
#include <string>

namespace Phantom {

/**
 * @brief UI panel for the full Particle -> SparseVolume -> Mesh pipeline.
 *
 * buildUi() runs once from init() (after all the bind*() calls); drawContents()
 * is contents_.show(). The radius / cell-length / iso-level inputs and the
 * kernel combo are plain draft fields read on the Convert buttons; the
 * volume/mesh renderer "show" checkboxes bind straight to the renderers'
 * isEnabled()/setEnabled(); result strings are provider Labels gated with
 * setVisibleWhen (Phase 4).
 */
class FluidVolumeConvertPanel : public ::VKG::IVkUIPanel, public IEmbeddedPanel {
public:
    void bindWorld(FluidWorld* w) { world_ = w; }
    void bindConverter(FluidVolumeConverter* c) { converter_ = c; }
    void bindMeshConverter(FluidMeshConverter* c) { meshConverter_ = c; }
    void bindVolumeRenderer(VolumeRenderer* r) { volumeRenderer_ = r; }
    void bindMeshRenderer(FluidMeshRenderer* r) { meshRenderer_ = r; }
    void init();

    // Called after a successful particle->volume / volume->mesh conversion
    // so the app can rebuild the corresponding renderer's GPU buffer.
    void setOnVolumeChanged(std::function<void()> fn) { onVolumeChanged_ = std::move(fn); }
    void setOnMeshChanged(std::function<void()> fn) { onMeshChanged_ = std::move(fn); }
    void setVisible(bool visible) { visible_ = visible; }
    bool isVisible() const { return visible_; }

    void onImGui() override;
    void drawContents() override;

private:
    FluidWorld* world_ = nullptr;
    FluidVolumeConverter* converter_ = nullptr;
    FluidMeshConverter* meshConverter_ = nullptr;
    VolumeRenderer* volumeRenderer_ = nullptr;
    FluidMeshRenderer* meshRenderer_ = nullptr;
    std::function<void()> onVolumeChanged_;
    std::function<void()> onMeshChanged_;
    bool visible_ = true;

    bool uiBuilt_ = false;
    void buildUi();

    std::string particlesText() const;
    std::string activeVoxelsText() const;
    std::string triangleCountText() const;

    UI::IView contents_ {"VolumeConvertControl"};

    UI::Label     particleToVolumeHeader_ {std::string("Particle -> SparseVolume")};
    UI::Separator headerSeparator_;
    UI::FloatView particleRadiusView_ {"Particle Radius", 0.025f};
    UI::FloatView cellLengthView_     {"Cell Length",     0.05f};
    UI::ComboBox  kernelCombo_        {"Kernel"};
    UI::Button    convertButton_      {"Convert to Volume"};
    UI::Label     particlesLabel_     {[this] { return particlesText(); }};
    UI::Label     activeVoxelsLabel_  {[this] { return activeVoxelsText(); }};
    UI::FileSaveView saveFileView_    {"Output VDB File"};
    UI::Button    saveButton_         {"Save"};
    UI::BoolView  showVolumeCheck_    {"Show Volume Points"};
    UI::Label     statusLabel_        {[this] { return statusMessage_; }, UI::Label::Style::Wrapped};

    UI::IView     meshGroup_ {"VolumeToMesh"};
    UI::Separator meshSeparator1_;
    UI::Label     volumeToMeshHeader_ {std::string("SparseVolume -> Mesh")};
    UI::Separator meshSeparator2_;
    UI::FloatView isoLevelView_       {"Iso Level", 0.5f};
    UI::Button    convertMeshButton_  {"Convert to Mesh"};
    UI::Label     triangleCountLabel_ {[this] { return triangleCountText(); }};
    UI::FileSaveView saveMeshFileView_ {"Output OBJ File"};
    UI::Button    saveMeshButton_     {"Save Mesh"};
    UI::BoolView  showMeshCheck_      {"Show Mesh"};
    UI::Label     meshStatusLabel_    {[this] { return meshStatusMessage_; }, UI::Label::Style::Wrapped};

    std::string statusMessage_;
    std::string meshStatusMessage_;
};

} // namespace Phantom
