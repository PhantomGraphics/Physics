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

void SSFRTestPanel::initWidgets()
{
    if (widgetsInitialized_) return;
    widgetsInitialized_ = true;

    presetCombo_.addItem("Sphere");
    presetCombo_.addItem("Dam Break");
    presetCombo_.addItem("Wave");
    presetCombo_.setSelected(0);

    generateButton_.setFunction([this]() { generate(); });

    using Mode = SSFluidRenderer::Mode;
    rawDepthButton_.setFunction([this]()    { if (ssfrRenderer_) ssfrRenderer_->setMode(Mode::DepthOnly);         });
    smoothDepthButton_.setFunction([this]() { if (ssfrRenderer_) ssfrRenderer_->setMode(Mode::SmoothedDepth);     });
    rawThickButton_.setFunction([this]()    { if (ssfrRenderer_) ssfrRenderer_->setMode(Mode::ThicknessRaw);      });
    smoothThickButton_.setFunction([this]() { if (ssfrRenderer_) ssfrRenderer_->setMode(Mode::ThicknessBilateral);});
    reflectionButton_.setFunction([this]()  { if (ssfrRenderer_) ssfrRenderer_->setMode(Mode::Reflection);        });
    fullSSFRButton_.setFunction([this]()    { if (ssfrRenderer_) ssfrRenderer_->setMode(Mode::SSFRMain);          });
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

void SSFRTestPanel::onImGuiComparison()
{
    if (!ssfrRenderer_) return;

    ImGui::Separator();
    ImGui::TextDisabled("--- Anisotropic Comparison ---");

    anisoView_.setValue(ssfrRenderer_->getAnisotropicSmoothing());
    anisoView_.show();
    ssfrRenderer_->setAnisotropicSmoothing(anisoView_.getValue());

    depthSmoothView_.setValue(ssfrRenderer_->getDepthSmoothing());
    depthSmoothView_.show();
    ssfrRenderer_->setDepthSmoothing(depthSmoothView_.getValue());

    ImGui::Spacing();
    ImGui::TextDisabled("Compare modes:");

    rawDepthButton_.show();
    ImGui::SameLine();
    smoothDepthButton_.show();

    rawThickButton_.show();
    ImGui::SameLine();
    smoothThickButton_.show();

    reflectionButton_.show();
    ImGui::SameLine();
    fullSSFRButton_.show();

    ImGui::Spacing();
    ImGui::TextWrapped("1. Enable Test Mode with Sphere preset");
    ImGui::TextWrapped("2. Click 'Raw Depth' then 'Smooth Depth' to see depth filter");
    ImGui::TextWrapped("3. In 'Reflection' mode, toggle 'Anisotropic ON' to see normal change");
}

void SSFRTestPanel::onImGui()
{
    if (!show_) return;

    initWidgets();

    ImGui::SetNextWindowPos(ImVec2(380.f, 35.f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(300.f, 420.f), ImGuiCond_Once);
    if (!ImGui::Begin("SSFR Test", &show_)) {
        ImGui::End();
        return;
    }

    const bool wasActive = active_;
    activeView_.setValue(active_);
    activeView_.show();
    active_ = activeView_.getValue();
    if (active_ && !wasActive)
        generate();

    if (active_) {
        static const char* kPresetLabels[] = { "Sphere", "Dam Break", "Wave" };
        presetCombo_.setSelected(static_cast<int>(preset_));
        presetCombo_.show();
        {
            const std::string sel = presetCombo_.getSelectedItem();
            for (int i = 0; i < 3; ++i) {
                if (sel == kPresetLabels[i]) {
                    preset_ = static_cast<Preset>(i);
                    break;
                }
            }
        }

        const char* szLabel = (preset_ == Preset::DamBreak) ? "Half-Width" : "Radius";
        ImGui::SliderFloat(szLabel, &radius_, 4.f, 25.f);
        ImGui::SliderInt("Target", &count_, 500, 8000);

        generateButton_.show();
        ImGui::SameLine();
        ImGui::Text("(%d particles)", static_cast<int>(positions_.size()));
    }

    onImGuiComparison();

    ImGui::End();
}

} // namespace Phantom
