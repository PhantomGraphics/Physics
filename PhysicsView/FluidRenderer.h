#pragma once

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"
#include "../../CGLib/VulkanGraphics/VulkanBuffer.h"

#include "FluidPipeline.h"

namespace Phantom {

struct VkFluidVertex {
    // xyz = position, w = (density - rest density) / rest density.
    // Keeping a vec4 also
    // preserves the GPU_CSPH direct-buffer layout.
    glm::vec4 positionDensity;

    static VkVertexInputBindingDescription getBindingDescription();
    static std::array<VkVertexInputAttributeDescription, 1> getAttributeDescriptions();
};

class FluidRenderer : public ::VKG::IVkSubRenderer {
public:
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
    void setSettingsVisible(bool visible) { settingsVisible_ = visible; }
    bool isSettingsVisible() const { return settingsVisible_; }

    glm::mat4 getViewMatrix() const;
    glm::mat4 getProjMatrix() const;

    void handleMouseButton(bool pressed, float x, float y);
    void handleMouseMove(float x, float y);
    void handleScroll(float dy);

    void onInit(Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
                VkRenderPass renderPass, uint32_t framesInFlight) override;
    void onUpdate(uint32_t frameIndex) override;
    void onRender(VkCommandBuffer cmd, uint32_t frameIndex) override;
    void onCleanup(VkDevice device) override;
    void onImGui() override;

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

    float yaw_ = 0.6f;
    float pitch_ = 0.6f;
    float distance_ = 120.f;
    glm::vec2 lastMouse_{ 0.f, 0.f };
    bool mouseDown_ = false;
    bool enabled_ = true;
    bool settingsVisible_ = true;
    VkExtent2D extent_ = { 1280, 720 };

    glm::mat4 computeMVP() const;
    std::vector<float> pendingDensityDeviations_;
    void uploadVertices(const std::vector<glm::vec3>& pts,
                        const std::vector<float>& densityDeviations,
                        bool hasDensity);
};

} // namespace Phantom  
