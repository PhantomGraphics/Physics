#include "pch.h"
#include "FluidVolumeConvertPanel.h"

namespace Phantom {

void FluidVolumeConvertPanel::init()
{
    buildUi();
}

void FluidVolumeConvertPanel::buildUi()
{
    if (uiBuilt_) return;
    uiBuilt_ = true;

    kernelCombo_.addItem("Isotropic");
    kernelCombo_.addItem("Anisotropic");
    kernelCombo_.setSelected(0);

    saveFileView_.addFilter("*.vdb");
    saveMeshFileView_.addFilter("*.obj");

    convertButton_.setFunction([this] {
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

    saveButton_.setFunction([this] {
        if (!converter_) return;

        const std::string path = saveFileView_.getFileName();
        if (path.empty()) {
            statusMessage_ = "Error: no output file selected";
            return;
        }
        const bool ok = converter_->saveToVdb(path);
        statusMessage_ = ok ? ("Saved: " + path) : ("Error: " + converter_->lastError());
    });

    convertMeshButton_.setFunction([this] {
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

    saveMeshButton_.setFunction([this] {
        if (!meshConverter_) return;

        const std::string path = saveMeshFileView_.getFileName();
        if (path.empty()) {
            meshStatusMessage_ = "Error: no output file selected";
            return;
        }
        const bool ok = meshConverter_->saveToObj(path);
        meshStatusMessage_ = ok ? ("Saved: " + path) : ("Error: " + meshConverter_->lastError());
    });

    // Reads the renderer's live enabled state (a scenario command may also
    // drive it) rather than owning a separate flag, so checkbox and command
    // can't fight each other.
    showVolumeCheck_.bind([this] { return volumeRenderer_ && volumeRenderer_->isEnabled(); },
                          [this](bool v) { if (volumeRenderer_) volumeRenderer_->setEnabled(v); });
    showVolumeCheck_.setVisibleWhen([this] { return volumeRenderer_ != nullptr; });
    showMeshCheck_.bind([this] { return meshRenderer_ && meshRenderer_->isEnabled(); },
                        [this](bool v) { if (meshRenderer_) meshRenderer_->setEnabled(v); });
    showMeshCheck_.setVisibleWhen([this] { return meshRenderer_ != nullptr; });

    activeVoxelsLabel_.setVisibleWhen([this] { return converter_ && converter_->getVolume() != nullptr; });
    statusLabel_.setVisibleWhen([this] { return !statusMessage_.empty(); });
    triangleCountLabel_.setVisibleWhen([this] { return meshConverter_ && meshConverter_->getTriangleCount() > 0; });
    meshStatusLabel_.setVisibleWhen([this] { return !meshStatusMessage_.empty(); });

    meshGroup_.add(&meshSeparator1_);
    meshGroup_.add(&volumeToMeshHeader_);
    meshGroup_.add(&meshSeparator2_);
    meshGroup_.add(&isoLevelView_);
    meshGroup_.add(&convertMeshButton_);
    meshGroup_.add(&triangleCountLabel_);
    meshGroup_.add(&saveMeshFileView_);
    meshGroup_.add(&saveMeshButton_);
    meshGroup_.add(&showMeshCheck_);
    meshGroup_.add(&meshStatusLabel_);
    meshGroup_.setVisibleWhen([this] { return meshConverter_ != nullptr; });

    contents_.add(&particleToVolumeHeader_);
    contents_.add(&headerSeparator_);
    contents_.add(&particleRadiusView_);
    contents_.add(&cellLengthView_);
    contents_.add(&kernelCombo_);
    contents_.add(&convertButton_);
    contents_.add(&particlesLabel_);
    contents_.add(&activeVoxelsLabel_);
    contents_.add(&saveFileView_);
    contents_.add(&saveButton_);
    contents_.add(&showVolumeCheck_);
    contents_.add(&statusLabel_);
    contents_.add(&meshGroup_);
}

std::string FluidVolumeConvertPanel::particlesText() const
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Particles: %llu",
        static_cast<unsigned long long>(world_ ? world_->getParticleCount() : 0));
    return buf;
}

std::string FluidVolumeConvertPanel::activeVoxelsText() const
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Active voxels: %d",
        converter_ ? converter_->getVoxelCount() : 0);
    return buf;
}

std::string FluidVolumeConvertPanel::triangleCountText() const
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Triangles: %llu",
        static_cast<unsigned long long>(meshConverter_ ? meshConverter_->getTriangleCount() : 0));
    return buf;
}

void FluidVolumeConvertPanel::onImGui()
{
    if (!world_ || !converter_ || !visible_) return;

    UI::Immediate::setNextWindowPosition(1010.f, 35.f);
    UI::Immediate::setNextWindowSize(280.f, 520.f);
    if (!UI::Immediate::beginWindow("Volume Conversion", &visible_)) {
        UI::Immediate::endWindow();
        return;
    }
    drawContents();
    UI::Immediate::endWindow();
}

void FluidVolumeConvertPanel::drawContents()
{
    if (!world_ || !converter_) return;
    if (!uiBuilt_) buildUi();
    contents_.show();
}

} // namespace Phantom
