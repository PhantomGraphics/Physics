#pragma once

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"
#include "../../CGLib/VulkanGraphics/VulkanBuffer.h"
#include "../../CGLib/UIWidgets/IView.h"
#include "../../CGLib/UIWidgets/Section.h"
#include "../../CGLib/UIWidgets/FloatSlider.h"
#include "../../CGLib/UIWidgets/BoolView.h"
#include "../../CGLib/UIWidgets/Label.h"

#include "FluidPipeline.h"
#include "Camera.h"
#include "IEmbeddedPanel.h"

#include <string>

namespace Phantom {

struct VkFluidVertex {
    // xyz = position, w = (density - rest density) / rest density.
    // Keeping a vec4 also
    // preserves the GPU_CSPH direct-buffer layout.
    glm::vec4 positionDensity;

    static VkVertexInputBindingDescription getBindingDescription();
    static std::array<VkVertexInputAttributeDescription, 1> getAttributeDescriptions();
};

class FluidRenderer : public ::VKG::IVkSubRenderer, public IEmbeddedPanel {
public:
    FluidRenderer();

    struct Shaders {
        std::vector<uint32_t> vertSpv;
        std::vector<uint32_t> fragSpv;
    };

    void setShaders(Shaders shaders) { shaders_ = std::move(shaders); }
    void setParticles(const std::vector<glm::vec3>& positions);
    void setParticles(const std::vector<glm::vec3>& positions,
                      const std::vector<float>& densityDeviations);
    void setDirectGpuBuffer(VkBuffer buf, uint32_t count);
    void clearDirectGpuBuffer();
    void setExtent(VkExtent2D ext) { extent_ = ext; }
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }

    glm::mat4 getViewMatrix() const;
    glm::mat4 getProjMatrix() const;
    float     getCameraDistance() const { return camera_.distance(); }

    void viewXY();
    void viewYZ();
    void viewZX();
    void fitCamera();

    void handleMouseButton(bool pressed, float x, float y);
    void handleMouseMove(float x, float y);
    void handleScroll(float dy);

    void onInit(Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
                VkRenderPass renderPass, uint32_t framesInFlight) override;
    void onUpdate(uint32_t frameIndex) override;
    void onRender(VkCommandBuffer cmd, uint32_t frameIndex) override;
    void onCleanup(VkDevice device) override;
    void drawContents() override;

private:
    const Phantom::VKG::VulkanContext* ctx_ = nullptr;
    const Phantom::VKG::VulkanCommandPool* pool_ = nullptr;

    Shaders shaders_;
    FluidPipeline pipeline_;
    Phantom::VKG::VulkanBuffer vertexBuffer_;
    VkBuffer directVertexBuffer_ = VK_NULL_HANDLE;
    uint32_t directPointCount_ = 0;

    std::vector<glm::vec3> pendingPositions_;
    uint32_t pointCount_ = 0;
    std::mutex mutex_;
    bool dirty_ = false;
    bool pendingHasDensity_ = false;
    bool hasDensity_ = false;

    float densityRangeMin_ = -0.05f;
    float densityRangeMax_ = 0.05f;
    bool autoDensityRange_ = true;
    float observedDensityRange_ = 0.0f;

    Camera camera_;
    bool enabled_ = true;
    VkExtent2D extent_ = { 1280, 720 };

    glm::mat4 computeMVP() const;
    std::vector<float> pendingDensityDeviations_;
    void uploadVertices(const std::vector<glm::vec3>& pts,
                        const std::vector<float>& densityDeviations,
                        bool hasDensity);
    void updateDensityColorRange(const std::vector<float>& densityDeviations);

    // --- UI (owned content root; drawContents() == contents_.show()) -----
    void buildUi();
    std::string quantityText() const;
    std::string observedRangeText() const;

    UI::IView    contents_ {"FluidRendererControl"};
    UI::Section  cameraSection_ {"Camera", false};
    UI::FloatSlider yawSlider_      {"Yaw", -3.14159f, 3.14159f};
    UI::FloatSlider pitchSlider_    {"Pitch", 0.05f, 3.09f};
    UI::FloatSlider distanceSlider_ {"Distance", 5.0f, 300.0f};
    UI::Section  colorMapSection_ {"Fluid Color Map", false};
    UI::Label    quantityLabel_ {[this] { return quantityText(); }};
    UI::BoolView autoContrastCheck_ {"Auto Contrast"};
    UI::Label    observedRangeLabel_ {[this] { return observedRangeText(); }};
    UI::FloatSlider densityMinSlider_ {"Density Difference Min", -0.5f, 0.0f, "%.3f"};
    UI::FloatSlider densityMaxSlider_ {"Density Difference Max", 0.0f, 0.5f, "%.3f"};
};

} // namespace Phantom  
