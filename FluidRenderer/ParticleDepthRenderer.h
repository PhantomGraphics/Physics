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

// Pass 1: Renders particles as smooth point sprites to the depth texture.
// Output: R32_SFLOAT color attachment of VkSSFROffscreenSet::depth().
// The fragment shader writes corrected depth to gl_FragCoord.z, so this is not a linear buffer.
// Smooth depth boundaries are obtained in later passes.
class ParticleDepthRenderer {
public:
    // UBO layout (matches GLSL std140)
    struct UBO {
        glm::mat4 proj;
        glm::mat4 modelView;
        float     pointSize;
        float     _pad[3];
    };
    static_assert(sizeof(UBO) == 144, "UBO layout mismatch");

    // renderPass: pass VkSSFROffscreenSet::depth().getRenderPass()
    // shaderDir:  directory containing .spv files (trailing slash not required)
    //             An empty string searches multiple fallback paths.
    void create(const Phantom::VKG::VulkanContext& ctx,
                uint32_t framesInFlight,
                VkRenderPass renderPass,
                std::vector<uint32_t> vertSpv,
                std::vector<uint32_t> fragSpv);

    void destroy(VkDevice device);

    // Upload particle positions to the device-local vertex buffer.
    void setParticles(const Phantom::VKG::VulkanContext& ctx,
                      const Phantom::VKG::VulkanCommandPool& pool,
                      const std::vector<glm::vec3>& positions);

    // Call this when using a GPU buffer directly (for GPU_CSPH mode).
    // buf must have VK_BUFFER_USAGE_VERTEX_BUFFER_BIT set.
    // Calling this disables the internal CPU upload path.
    void setExternalBuffer(VkBuffer buf, uint32_t count);

    // Disable external buffer mode and revert to the internal CPU upload mode.
    void clearExternalBuffer();

    // Record Pass 1 into the command buffer.
    // Opens and closes the render pass for targets.depth().
    void render(VkCommandBuffer cmd,
                uint32_t frameIndex,
                SSFROffscreenSet& targets,
                const glm::mat4& proj,
                const glm::mat4& modelView,
                float particleRadius,
                float viewportHeight);

    bool isValid() const { return pipeline_.isValid(); }

private:
    SSFRPipeline    pipeline_;       // vec3 stride (CPU upload path)
    SSFRPipeline    pipelineVec4_;   // vec4 stride (GPU_CSPH external buffer)
    Phantom::VKG::VulkanBuffer vertexBuffer_;
    uint32_t          particleCount_ = 0;

    VkBuffer  externalBuf_   = VK_NULL_HANDLE;
    uint32_t  externalCount_ = 0;
    bool      useExternal_   = false;

};

} // namespace VKSSFR
