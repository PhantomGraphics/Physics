#include "pch.h"
#include "SSFRTestPanel.h"

#include "../FluidRenderer/SSFluidRenderer.h"

static constexpr float kPI = 3.14159265358979f;

namespace Phantom {

static constexpr glm::vec3 kCenter{ 20.f, 20.f, 20.f };

bool SSFRTestPanel::consumeDirty()
{
    bool d = dirty_;
    dirty_ = false;
    return d;
}

void SSFRTestPanel::init()
{
    buildUi();
}

void SSFRTestPanel::buildUi()
{
    if (uiBuilt_) return;
    uiBuilt_ = true;

    activeView_.bind([this] { return active_; },
        [this](bool v) {
            const bool was = active_;
            active_ = v;
            if (active_ && !was) generate();
        });

    static const char* const kPresetLabels[] = { "Sphere", "Dam Break", "Wave" };
    for (auto* label : kPresetLabels) presetCombo_.addItem(label);
    presetCombo_.bind([this] { return static_cast<int>(preset_); },
                      [this](int i) { preset_ = static_cast<Preset>(i); });

    radiusSlider_.setLabelProvider([this] {
        return std::string(preset_ == Preset::DamBreak ? "Half-Width" : "Radius");
    });
    radiusSlider_.bind([this] { return radius_; },
                       [this](float v) { radius_ = v; });
    countSlider_.bind([this] { return count_; },
                      [this](int v) { count_ = v; });

    generateButton_.setFunction([this] { generate(); });
    generateRow_.add(&generateButton_);
    generateRow_.add(&particleCountLabel_);

    activeGroup_.add(&presetCombo_);
    activeGroup_.add(&radiusSlider_);
    activeGroup_.add(&countSlider_);
    activeGroup_.add(&generateRow_);
    activeGroup_.setVisibleWhen([this] { return active_; });

    // --- comparison / debug ------------------------------------------
    anisoView_.bind([this] { return ssfrRenderer_->getAnisotropicSmoothing(); },
                    [this](bool v) { ssfrRenderer_->setAnisotropicSmoothing(v); });
    depthSmoothView_.bind([this] { return ssfrRenderer_->getDepthSmoothing(); },
                          [this](bool v) { ssfrRenderer_->setDepthSmoothing(v); });

    using Mode = SSFluidRenderer::Mode;
    rawDepthButton_.setFunction([this]    { if (ssfrRenderer_) ssfrRenderer_->setMode(Mode::DepthOnly); });
    smoothDepthButton_.setFunction([this] { if (ssfrRenderer_) ssfrRenderer_->setMode(Mode::SmoothedDepth); });
    rawThickButton_.setFunction([this]    { if (ssfrRenderer_) ssfrRenderer_->setMode(Mode::ThicknessRaw); });
    smoothThickButton_.setFunction([this] { if (ssfrRenderer_) ssfrRenderer_->setMode(Mode::ThicknessBilateral); });
    reflectionButton_.setFunction([this]  { if (ssfrRenderer_) ssfrRenderer_->setMode(Mode::Reflection); });
    fullSSFRButton_.setFunction([this]    { if (ssfrRenderer_) ssfrRenderer_->setMode(Mode::SSFRMain); });

    depthCompareRow_.add(&rawDepthButton_);
    depthCompareRow_.add(&smoothDepthButton_);
    thickCompareRow_.add(&rawThickButton_);
    thickCompareRow_.add(&smoothThickButton_);
    reflCompareRow_.add(&reflectionButton_);
    reflCompareRow_.add(&fullSSFRButton_);

    comparisonGroup_.add(&comparisonSeparator_);
    comparisonGroup_.add(&anisoComparisonHeader_);
    comparisonGroup_.add(&anisoView_);
    comparisonGroup_.add(&depthSmoothView_);
    comparisonGroup_.add(&spacing1_);
    comparisonGroup_.add(&compareModesLabel_);
    comparisonGroup_.add(&depthCompareRow_);
    comparisonGroup_.add(&thickCompareRow_);
    comparisonGroup_.add(&reflCompareRow_);
    comparisonGroup_.add(&spacing2_);
    comparisonGroup_.add(&instr1_);
    comparisonGroup_.add(&instr2_);
    comparisonGroup_.add(&instr3_);
    comparisonGroup_.setVisibleWhen([this] { return ssfrRenderer_ != nullptr; });

    contents_.add(&testOnlyLabel_);
    contents_.add(&activeView_);
    contents_.add(&activeGroup_);
    contents_.add(&comparisonGroup_);
}

std::string SSFRTestPanel::particleCountText() const
{
    char buf[48];
    std::snprintf(buf, sizeof(buf), "(%d particles)", static_cast<int>(positions_.size()));
    return buf;
}

void SSFRTestPanel::generate()
{
    switch (preset_) {
    case Preset::Sphere:   genSphere();   break;
    case Preset::DamBreak: genDamBreak(); break;
    case Preset::Wave:     genWave();     break;
    }
    dirty_ = true;
}

void SSFRTestPanel::genSphere()
{
    positions_.clear();
    const float vol  = (4.f / 3.f) * static_cast<float>(kPI) * radius_ * radius_ * radius_;
    const float step = std::cbrtf(vol / static_cast<float>(count_));

    for (float x = -radius_; x <= radius_; x += step)
        for (float y = -radius_; y <= radius_; y += step)
            for (float z = -radius_; z <= radius_; z += step)
                if (x*x + y*y + z*z <= radius_ * radius_)
                    positions_.push_back(kCenter + glm::vec3(x, y, z));
}

void SSFRTestPanel::genDamBreak()
{
    positions_.clear();
    const float w    = radius_;
    const float h    = radius_ * 2.f;
    const float vol  = w * h * w;
    const float step = std::cbrtf(vol / static_cast<float>(count_));

    for (float x = 0; x <= w; x += step)
        for (float y = 0; y <= h; y += step)
            for (float z = 0; z <= w; z += step)
                positions_.push_back(kCenter + glm::vec3(x - w * 0.5f,
                                                          y - h * 0.5f,
                                                          z - w * 0.5f));
}

void SSFRTestPanel::genWave()
{
    positions_.clear();
    const float area = (2.f * radius_) * (2.f * radius_);
    const float step = std::sqrtf(area / static_cast<float>(count_) * 4.f);
    const float amp  = radius_ * 0.3f;
    const float freq = static_cast<float>(kPI) / radius_;

    for (float x = -radius_; x <= radius_; x += step)
        for (float z = -radius_; z <= radius_; z += step) {
            const float top = amp * std::sinf(x * freq) * std::cosf(z * freq);
            for (float y = -amp * 2.f; y <= top; y += step)
                positions_.push_back(kCenter + glm::vec3(x, y, z));
        }
}

void SSFRTestPanel::onImGui()
{
    if (!show_) return;

    UI::Immediate::setNextWindowPosition(380.f, 35.f);
    UI::Immediate::setNextWindowSize(300.f, 420.f);
    if (!UI::Immediate::beginWindow("SSFR Test", &show_)) {
        UI::Immediate::endWindow();
        return;
    }
    drawContents();
    UI::Immediate::endWindow();
}

void SSFRTestPanel::drawContents()
{
    if (!uiBuilt_) buildUi();
    contents_.show();
}

} // namespace Phantom
