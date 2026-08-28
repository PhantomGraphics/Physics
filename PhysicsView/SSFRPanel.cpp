#include "pch.h"
#include "SSFRPanel.h"

#include "../FluidRenderer/SSFluidRenderer.h"

namespace Phantom {

void SSFRPanel::initWidgets()
{
    if (widgetsInitialized_) return;
    widgetsInitialized_ = true;

    modeCombo_.addItem("DepthOnly");
    modeCombo_.addItem("ThicknessRaw");
    modeCombo_.addItem("ThicknessBilateral");
    modeCombo_.addItem("Reflection");
    modeCombo_.addItem("Refraction");
    modeCombo_.addItem("SSFRMain");
    modeCombo_.addItem("SmoothedDepth");
    modeCombo_.setSelected(modeIndex_);
}

void SSFRPanel::onImGui()
{
    initWidgets();

    ImGui::SetNextWindowPos(ImVec2(10.f, 500.f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(360.f, 480.f), ImGuiCond_Once);
    if (!ImGui::Begin("SSFR Control")) {
        ImGui::End();
        return;
    }

    enableCheck_.setValue(enabled_);
    enableCheck_.show();
    enabled_ = enableCheck_.getValue();

    if (enabled_) {
        static const char* kModeLabels[] = {
            "DepthOnly", "ThicknessRaw", "ThicknessBilateral",
            "Reflection", "Refraction", "SSFRMain", "SmoothedDepth"
        };

        modeCombo_.setSelected(modeIndex_);
        modeCombo_.show();
        const std::string selMode = modeCombo_.getSelectedItem();
        for (int i = 0; i < 7; ++i) {
            if (selMode == kModeLabels[i]) {
                modeIndex_ = i;
                break;
            }
        }

        if (world_) {
            auto& ww = world_->whiteWaterParams();

            sprayCheck_.setValue(ww.enableSpray);
            sprayCheck_.show();
            if (sprayCheck_.getValue() != ww.enableSpray) {
                ww.enableSpray = sprayCheck_.getValue();
            }

            foamCheck_.setValue(ww.enableFoam);
            foamCheck_.show();
            if (foamCheck_.getValue() != ww.enableFoam) {
                ww.enableFoam = foamCheck_.getValue();
            }

            if (renderer_) {
                const bool gpuCsph =
                    (world_->getSimulationType() == FluidWorld::SimulationType::GPU_CSPH);
                renderer_->setShowSpray(gpuCsph ? false : ww.enableSpray);
                renderer_->setShowFoam(gpuCsph ? false : ww.enableFoam);
            }
        }

        if (renderer_) {
            ImGui::TextDisabled("--- Thickness Bilateral ---");
            float thickSigmaS = renderer_->getThicknessSmoothingSigmaS();
            if (ImGui::SliderFloat("Thickness SigmaS", &thickSigmaS, 0.5f, 6.0f))
                renderer_->setThicknessSmoothingSigmaS(thickSigmaS);

            float thickSigmaR = renderer_->getThicknessSmoothingSigmaR();
            if (ImGui::SliderFloat("Thickness SigmaR", &thickSigmaR, 0.01f, 0.25f))
                renderer_->setThicknessSmoothingSigmaR(thickSigmaR);

            ImGui::Separator();
            ImGui::TextDisabled("--- Anisotropic (Thickness + Depth) ---");
            anisoCheck_.setValue(renderer_->getAnisotropicSmoothing());
            anisoCheck_.show();
            renderer_->setAnisotropicSmoothing(anisoCheck_.getValue());

            float aniso = renderer_->getAnisotropy();
            if (ImGui::SliderFloat("Anisotropy", &aniso, 0.0f, 3.0f))
                renderer_->setAnisotropy(aniso);

            float gradScale = renderer_->getAnisotropicGradientScale();
            if (ImGui::SliderFloat("Aniso Grad Scale", &gradScale, 0.0f, 20.0f))
                renderer_->setAnisotropicGradientScale(gradScale);

            ImGui::Separator();
            ImGui::TextDisabled("--- Depth Bilateral (Surface Normals) ---");
            depthSmoothCheck_.setValue(renderer_->getDepthSmoothing());
            depthSmoothCheck_.show();
            renderer_->setDepthSmoothing(depthSmoothCheck_.getValue());

            float depthSigmaS = renderer_->getDepthSmoothingSigmaS();
            if (ImGui::SliderFloat("Depth SigmaS", &depthSigmaS, 0.5f, 6.0f))
                renderer_->setDepthSmoothingSigmaS(depthSigmaS);

            float depthSigmaR = renderer_->getDepthSmoothingSigmaR();
            if (ImGui::SliderFloat("Depth SigmaR", &depthSigmaR, 0.005f, 0.2f))
                renderer_->setDepthSmoothingSigmaR(depthSigmaR);

            ImGui::Separator();
            float sprayOp = renderer_->getSprayOpacity();
            if (ImGui::SliderFloat("Spray Opacity", &sprayOp, 0.0f, 1.0f))
                renderer_->setSprayOpacity(sprayOp);

            float foamOp = renderer_->getFoamOpacity();
            if (ImGui::SliderFloat("Foam Opacity", &foamOp, 0.0f, 1.0f))
                renderer_->setFoamOpacity(foamOp);
        }
    }

    ImGui::End();
}

} // namespace Phantom
