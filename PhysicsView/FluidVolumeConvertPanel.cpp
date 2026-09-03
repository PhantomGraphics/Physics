#include "pch.h"
#include "FluidVolumeConvertPanel.h"

namespace Phantom {

void FluidVolumeConvertPanel::initWidgets()
{
    if (widgetsInitialized_) return;
    widgetsInitialized_ = true;

    kernelCombo_.addItem("Isotropic");
    kernelCombo_.addItem("Anisotropic");
    kernelCombo_.setSelected(0);

    saveFileView_.addFilter("*.vdb");
    saveMeshFileView_.addFilter("*.obj");

    convertButton_.setFunction([this]() {
        if (!world_ || !converter_) return;

        auto& p = converter_->params();
        p.particleRadius = particleRadiusView_.getValue();
        p.cellLength     = cellLengthView_.getValue();
        p.kernelType = (kernelCombo_.getSelectedItem() == "Anisotropic")
            ? FluidVolumeConverter::KernelType::Anisotropic
            : FluidVolumeConverter::KernelType::Isotropic;

        const bool ok = converter_->convert(world_->getParticlePositions());
        statusMessage_ = ok
            ? ("Converted: " + std::to_string(converter_->getVoxelCount()) + " active voxels")
            : ("Error: " + converter_->lastError());

        if (ok && onVolumeChanged_) onVolumeChanged_();
    });

    saveButton_.setFunction([this]() {
        if (!converter_) return;

        const std::string path = saveFileView_.getFileName();
        if (path.empty()) {
            statusMessage_ = "Error: no output file selected";
            return;
        }

        const bool ok = converter_->saveToVdb(path);
        statusMessage_ = ok ? ("Saved: " + path) : ("Error: " + converter_->lastError());
    });

    convertMeshButton_.setFunction([this]() {
        if (!converter_ || !meshConverter_) return;

        const auto* volume = converter_->getVolume();
        if (!volume) {
            meshStatusMessage_ = "Error: no volume to convert -- run Convert to Volume first";
            return;
        }

        meshConverter_->params().isoLevel = isoLevelView_.getValue();

        const bool ok = meshConverter_->convert(*volume);
        meshStatusMessage_ = ok
            ? ("Converted: " + std::to_string(meshConverter_->getTriangleCount()) + " triangles")
            : ("Error: " + meshConverter_->lastError());

        if (ok && onMeshChanged_) onMeshChanged_();
    });

    saveMeshButton_.setFunction([this]() {
        if (!meshConverter_) return;

        const std::string path = saveMeshFileView_.getFileName();
        if (path.empty()) {
            meshStatusMessage_ = "Error: no output file selected";
            return;
        }

        const bool ok = meshConverter_->saveToObj(path);
        meshStatusMessage_ = ok ? ("Saved: " + path) : ("Error: " + meshConverter_->lastError());
    });
}

void FluidVolumeConvertPanel::onImGui()
{
    if (!world_ || !converter_ || !visible_) return;

    initWidgets();

    UI::Immediate::setNextWindowPosition(1010.f, 35.f);
    UI::Immediate::setNextWindowSize(280.f, 520.f);
    if (!UI::Immediate::beginWindow("Volume Conversion", &visible_)) {
        UI::Immediate::endWindow();
        return;
    }

    UI::Immediate::textUnformatted("Particle -> SparseVolume");
    UI::Immediate::separator();
    particleRadiusView_.show();
    cellLengthView_.show();
    kernelCombo_.show();
    convertButton_.show();

    UI::Immediate::text("Particles: %llu",
                static_cast<unsigned long long>(world_->getParticleCount()));
    if (converter_->getVolume())
        UI::Immediate::text("Active voxels: %d", converter_->getVoxelCount());

    saveFileView_.show();
    saveButton_.show();
    if (volumeRenderer_) {
        // Reads the renderer's live enabled state (which a scenario command
        // -- SetVolumeRenderEnabled: -- may also drive) rather than owning a
        // separate BoolView, so the checkbox and scenario command can't
        // fight each other by both writing every frame.
        bool show = volumeRenderer_->isEnabled();
        if (UI::Immediate::checkbox("Show Volume Points", show))
            volumeRenderer_->setEnabled(show);
    }

    if (!statusMessage_.empty())
        UI::Immediate::textWrapped("%s", statusMessage_.c_str());

    if (meshConverter_) {
        UI::Immediate::separator();
        UI::Immediate::textUnformatted("SparseVolume -> Mesh");
        UI::Immediate::separator();
        isoLevelView_.show();
        convertMeshButton_.show();

        if (meshConverter_->getTriangleCount() > 0)
            UI::Immediate::text("Triangles: %llu",
                        static_cast<unsigned long long>(meshConverter_->getTriangleCount()));

        saveMeshFileView_.show();
        saveMeshButton_.show();
        if (meshRenderer_) {
            bool show = meshRenderer_->isEnabled();
            if (UI::Immediate::checkbox("Show Mesh", show))
                meshRenderer_->setEnabled(show);
        }

        if (!meshStatusMessage_.empty())
            UI::Immediate::textWrapped("%s", meshStatusMessage_.c_str());
    }

    UI::Immediate::endWindow();
}

} // namespace Phantom
