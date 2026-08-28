#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "SSFRPipeline.h"
#include "SSFROffscreenSet.h"
#include "../../CGLib/VulkanGraphics/VulkanBuffer.h"
#include "../../CGLib/VulkanGraphics/VulkanCommandPool.h"

#include <vector>

namespace Phantom::VKG { class VulkanContext; }

namespace Phantom {

class SSThicknessRenderer {
public:
    struct UBO {
        glm::mat4 proj;
        glm::mat4 modelView;
        float pointSize;
        float thicknessScale;
        float _pad[2];
    };
    static_assert(sizeof(UBO) == 144, "UBO layout mismatch");

    void create(const Phantom::VKG::VulkanContext& ctx,
                uint32_t framesInFlight,
                VkRenderPass renderPass,
                std::vector<uint32_t> vertSpv,
                std::vector<uint32_t> fragSpv);

    void destroy(VkDevice device);

    void setParticles(const Phantom::VKG::VulkanContext& ctx,
                      const Phantom::VKG::VulkanCommandPool& pool,
                      const std::vector<glm::vec3>& positions);

    void setExternalBuffer(VkBuffer buf, uint32_t count);
    void clearExternalBuffer();

    void render(VkCommandBuffer cmd,
                uint32_t frameIndex,
                SSFROffscreenSet& targets,
                const glm::mat4& proj,
                const glm::mat4& modelView,
                float pointSize = 10.0f,
                float thicknessScale = 0.6f);

    bool isValid() const { return pipeline_.isValid(); }

private:
    SSFRPipeline    pipeline_;       // vec3 stride
    SSFRPipeline    pipelineVec4_;   // vec4 stride (GPU_CSPH external buffer)
    Phantom::VKG::VulkanBuffer vertexBuffer_;
    uint32_t          particleCount_ = 0;

    VkBuffer  externalBuf_   = VK_NULL_HANDLE;
    uint32_t  externalCount_ = 0;
    bool      useExternal_   = false;

};

} // namespace VKSSFR
