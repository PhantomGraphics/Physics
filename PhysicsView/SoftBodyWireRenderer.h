#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "CGLib/VkAppBase/IVkSubRenderer.h"
#include "CGLib/Renderer/VkRenderer/VkLineRenderer.h"

#include <optional>
#include <vector>

namespace Phantom {

class SoftBodyWireRenderer : public ::VKG::IVkSubRenderer {
public:
    struct Shaders {
        std::vector<uint32_t> vertSpv;
        std::vector<uint32_t> fragSpv;
    };

    void setShaders(Shaders s)   { shaders_ = std::move(s); }
    void setExtent(VkExtent2D e) { extent_ = e; }

    void update(const std::vector<float>&    positions,
                const std::vector<float>&    colors,
                const std::vector<uint32_t>& indices,
                const glm::mat4&             mvp);

    // Refreshes the camera only, without re-uploading geometry. Call every
    // frame the camera moves so the wireframe tracks it even between
    // world-changed events (which is when update() above gets called).
    void setMVP(const glm::mat4& mvp) { mvp_ = mvp; }

    void onInit(Phantom::VKG::VulkanContext& ctx,
                const Phantom::VKG::VulkanCommandPool& pool,
                VkRenderPass renderPass,
                uint32_t framesInFlight) override;

    void onUpdate(uint32_t frameIndex) override;
    void onRender(VkCommandBuffer cmd, uint32_t frameIndex) override;
    void onCleanup(VkDevice device) override;

private:
    Shaders    shaders_;
    VkExtent2D extent_ = {1280, 720};
    bool       dirty_  = false;

    const Phantom::VKG::VulkanContext*     ctx_  = nullptr;
    const Phantom::VKG::VulkanCommandPool* pool_ = nullptr;

    std::optional<VKG::VkLineRenderer> lineRenderer_;

    std::vector<float>    positions_;
    std::vector<float>    colors_;
    std::vector<uint32_t> indices_;
    glm::mat4             mvp_{1.f};
};

} // namespace Phantom
