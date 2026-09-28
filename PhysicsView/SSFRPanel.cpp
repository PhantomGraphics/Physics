#include "pch.h"
#include "SSFRPanel.h"

#include "SSFRTestPanel.h"
#include "../FluidRenderer/SSFluidRenderer.h"

namespace Phantom {

void SSFRPanel::init()
{
    buildUi();
}

void SSFRPanel::setAnisotropicKernel(bool v)
{
    if (!renderer_) return;
    renderer_->setAnisotropicKernel(v);
    markKernelChanged();
}

bool SSFRPanel::getAnisotropicKernel() const
{
    return renderer_ && renderer_->getAnisotropicKernel();
}

bool SSFRPanel::isAnisotropicKernelActive() const
{
    return renderer_ && renderer_->isAnisotropicKernelActive();
}

std::string SSFRPanel::kernelStatsText() const
{
    if (!kernelUnavailable_.empty()) return kernelUnavailable_;
    if (renderer_ && !renderer_->supportsAnisotropicKernel())
        return std::string("Unavailable: ellipsoid shaders not loaded");
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%u particles, %u anisotropic, mean stretch %.2f, %.1f ms",
                  kernelStats_.particleCount, kernelStats_.anisotropicCount,
                  kernelStats_.meanStretchRatio, kernelStats_.computeMs);
    return std::string(buf);
}

void SSFRPanel::buildUi()
{
    if (uiBuilt_) return;
    uiBuilt_ = true;

    enableCheck_.bind([this] { return enabled_; },
                      [this](bool v) { enabled_ = v; });

    static const char* const kModeLabels[] = {
        "DepthOnly", "ThicknessRaw", "ThicknessBilateral",
        "Reflection", "Refraction", "SSFRMain", "SmoothedDepth"
    };
    for (auto* m : kModeLabels) modeCombo_.addItem(m);
    modeCombo_.bind([this] { return modeIndex_; },
                    [this](int v) { modeIndex_ = v; });

    sprayCheck_.bind([this] { return world_->whiteWaterParams().enableSpray; },
                     [this](bool v) { world_->whiteWaterParams().enableSpray = v; });
    foamCheck_.bind([this] { return world_->whiteWaterParams().enableFoam; },
                    [this](bool v) { world_->whiteWaterParams().enableFoam = v; });
    sprayCheck_.setVisibleWhen([this] { return world_ != nullptr; });
    foamCheck_.setVisibleWhen([this] { return world_ != nullptr; });

    thicknessSigmaSSlider_.bind([this] { return renderer_->getThicknessSmoothingSigmaS(); },
                                [this](float v) { renderer_->setThicknessSmoothingSigmaS(v); });
    thicknessSigmaRSlider_.bind([this] { return renderer_->getThicknessSmoothingSigmaR(); },
                                [this](float v) { renderer_->setThicknessSmoothingSigmaR(v); });
    anisoCheck_.bind([this] { return renderer_->getAnisotropicSmoothing(); },
                     [this](bool v) { renderer_->setAnisotropicSmoothing(v); });
    anisotropySlider_.bind([this] { return renderer_->getAnisotropy(); },
                           [this](float v) { renderer_->setAnisotropy(v); });
    anisoGradScaleSlider_.bind([this] { return renderer_->getAnisotropicGradientScale(); },
                               [this](float v) { renderer_->setAnisotropicGradientScale(v); });
    depthSmoothCheck_.bind([this] { return renderer_->getDepthSmoothing(); },
                           [this](bool v) { renderer_->setDepthSmoothing(v); });
    depthSigmaSSlider_.bind([this] { return renderer_->getDepthSmoothingSigmaS(); },
                            [this](float v) { renderer_->setDepthSmoothingSigmaS(v); });
    depthSigmaRSlider_.bind([this] { return renderer_->getDepthSmoothingSigmaR(); },
                            [this](float v) { renderer_->setDepthSmoothingSigmaR(v); });
    kernelCheck_.bind([this] { return renderer_->getAnisotropicKernel(); },
                      [this](bool v) { renderer_->setAnisotropicKernel(v); markKernelChanged(); });
    const auto bindKernelFloat = [this](UI::FloatSlider& slider, float SSFRKernelSettings::* field) {
        slider.bind([this, field] { return kernelSettings_.*field; },
                    [this, field](float v) { kernelSettings_.*field = v; markKernelChanged(); });
    };
    bindKernelFloat(kernelSearchSlider_, &SSFRKernelSettings::searchScale);
    bindKernelFloat(kernelMaxRatioSlider_, &SSFRKernelSettings::maxRatio);
    bindKernelFloat(kernelIsolatedSlider_, &SSFRKernelSettings::isolatedScale);
    bindKernelFloat(kernelSmoothSlider_, &SSFRKernelSettings::smoothing);
    kernelMinNbrSlider_.bind([this] { return kernelSettings_.minNeighbors; },
                             [this](int v) { kernelSettings_.minNeighbors = v; markKernelChanged(); });
    kernelGroup_.add(&kernelSearchSlider_);
    kernelGroup_.add(&kernelMaxRatioSlider_);
    kernelGroup_.add(&kernelIsolatedSlider_);
    kernelGroup_.add(&kernelMinNbrSlider_);
    kernelGroup_.add(&kernelSmoothSlider_);
    kernelGroup_.add(&kernelStatsLabel_);
    kernelGroup_.setVisibleWhen([this] { return renderer_->getAnisotropicKernel(); });

    sprayOpacitySlider_.bind([this] { return renderer_->getSprayOpacity(); },
                             [this](float v) { renderer_->setSprayOpacity(v); });
    foamOpacitySlider_.bind([this] { return renderer_->getFoamOpacity(); },
                            [this](float v) { renderer_->setFoamOpacity(v); });

    rendererGroup_.add(&thickHeader_);
    rendererGroup_.add(&thicknessSigmaSSlider_);
    rendererGroup_.add(&thicknessSigmaRSlider_);
    rendererGroup_.add(&sep1_);
    rendererGroup_.add(&anisoHeader_);
    rendererGroup_.add(&anisoCheck_);
    rendererGroup_.add(&anisotropySlider_);
    rendererGroup_.add(&anisoGradScaleSlider_);
    rendererGroup_.add(&sepKernel_);
    rendererGroup_.add(&kernelHeader_);
    rendererGroup_.add(&kernelCheck_);
    rendererGroup_.add(&kernelGroup_);
    rendererGroup_.add(&sep2_);
    rendererGroup_.add(&depthHeader_);
    rendererGroup_.add(&depthSmoothCheck_);
    rendererGroup_.add(&depthSigmaSSlider_);
    rendererGroup_.add(&depthSigmaRSlider_);
    rendererGroup_.add(&sep3_);
    rendererGroup_.add(&sprayOpacitySlider_);
    rendererGroup_.add(&foamOpacitySlider_);
    rendererGroup_.setVisibleWhen([this] { return renderer_ != nullptr; });

    enabledGroup_.add(&modeCombo_);
    enabledGroup_.add(&sprayCheck_);
    enabledGroup_.add(&foamCheck_);
    enabledGroup_.add(&rendererGroup_);
    enabledGroup_.setVisibleWhen([this] { return enabled_; });

    contents_.add(&enableCheck_);
    contents_.add(&enabledGroup_);

    // Former "SSFR Test" page, folded in as a collapsed section (2026-09-09).
    if (debugPanel_) {
        debugSection_.add(&debugPanel_->contentsView());
        contents_.add(&debugSection_);
    }
}

void SSFRPanel::onImGui()
{
    if (!visible_) return;

    UI::Immediate::setNextWindowPosition(10.f, 500.f);
    UI::Immediate::setNextWindowSize(360.f, 480.f);
    if (!UI::Immediate::beginWindow("SSFR Control", &visible_)) {
        UI::Immediate::endWindow();
        return;
    }
    drawContents();
    UI::Immediate::endWindow();
}

void SSFRPanel::drawContents()
{
    if (!uiBuilt_) buildUi();
    contents_.show();
}

} // namespace Phantom
