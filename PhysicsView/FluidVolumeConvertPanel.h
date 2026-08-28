#pragma once

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"
#include "../../CGLib/UIWidgets/Button.h"
#include "../../CGLib/UIWidgets/FloatView.h"
#include "../../CGLib/UIWidgets/ComboBox.h"
#include "../../CGLib/UIWidgets/FileSaveView.h"

#include "FluidWorld.h"
#include "FluidVolumeConverter.h"
#include "FluidMeshConverter.h"
#include "VolumeRenderer.h"
#include "FluidMeshRenderer.h"

#include <functional>

namespace Phantom {

/**
 * @brief UI panel for the full Particle -> SparseVolume -> Mesh pipeline:
 * pulls the fluid's current particle set into a Volume::SparseVolumef (via
 * FluidVolumeConverter), then optionally extracts a triangle mesh from that
 * volume via Marching Cubes (FluidMeshConverter). Either stage can be saved
 * to disk (.vdb / .obj) and/or shown live via FluidVolumeRenderer /
 * FluidMeshRenderer -- the live-simulation counterpart to FluidStudio's
 * offline PLY->VDB and VDB->Mesh conversion panels.
 */
class FluidVolumeConvertPanel : public ::VKG::IVkUIPanel {
public:
    void bindWorld(FluidWorld* w) { world_ = w; }
    void bindConverter(FluidVolumeConverter* c) { converter_ = c; }
    void bindMeshConverter(FluidMeshConverter* c) { meshConverter_ = c; }
    void bindVolumeRenderer(VolumeRenderer* r) { volumeRenderer_ = r; }
    void bindMeshRenderer(FluidMeshRenderer* r) { meshRenderer_ = r; }

    // Called after a successful particle->volume / volume->mesh conversion
    // so the app can rebuild the corresponding renderer's GPU buffer (it
    // owns the Vulkan context/camera, which this panel does not).
    void setOnVolumeChanged(std::function<void()> fn) { onVolumeChanged_ = std::move(fn); }
    void setOnMeshChanged(std::function<void()> fn) { onMeshChanged_ = std::move(fn); }

    void onImGui() override;

private:
    FluidWorld* world_ = nullptr;
    FluidVolumeConverter* converter_ = nullptr;
    FluidMeshConverter* meshConverter_ = nullptr;
    VolumeRenderer* volumeRenderer_ = nullptr;
    FluidMeshRenderer* meshRenderer_ = nullptr;
    std::function<void()> onVolumeChanged_;
    std::function<void()> onMeshChanged_;

    Phantom::UI::FloatView    particleRadiusView_ { "Particle Radius", 0.025f };
    Phantom::UI::FloatView    cellLengthView_     { "Cell Length",     0.05f  };
    Phantom::UI::ComboBox     kernelCombo_        { "Kernel" };
    Phantom::UI::Button       convertButton_      { "Convert to Volume" };
    Phantom::UI::FileSaveView saveFileView_       { "Output VDB File" };
    Phantom::UI::Button       saveButton_         { "Save" };

    Phantom::UI::FloatView    isoLevelView_       { "Iso Level", 0.5f };
    Phantom::UI::Button       convertMeshButton_  { "Convert to Mesh" };
    Phantom::UI::FileSaveView saveMeshFileView_   { "Output OBJ File" };
    Phantom::UI::Button       saveMeshButton_     { "Save Mesh" };

    std::string statusMessage_;
    std::string meshStatusMessage_;

    bool widgetsInitialized_ = false;
    void initWidgets();
};

} // namespace Phantom
