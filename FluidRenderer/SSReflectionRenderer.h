#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "SSFRPipeline.h"
#include "SSFROffscreenSet.h"

#include <vector>

namespace Phantom::VKG { class VulkanContext; class VulkanCommandPool; }

namespace Phantom {

class SSReflectionRenderer {
public:
    struct UBO {
        glm::vec4 tint     = glm::vec4(0.18f, 0.36f, 0.85f, 1.0f);
        float     strength = 1.0f;
        int       hasEnvMap = 0;
        float     _pad[2]{};
        glm::mat4 invProj{1.0f};
        glm::mat4 invViewRot{1.0f};
    };
    static_assert(sizeof(UBO) == 160, "UBO layout mismatch");

    void create(const Phantom::VKG::VulkanContext& ctx,
                const Phantom::VKG::VulkanCommandPool& pool,
                uint32_t framesInFlight,
                VkRenderPass renderPass,
                std::vector<uint32_t> vertSpv,
                std::vector<uint32_t> fragSpv);

    void destroy(VkDevice device);

    void render(const Phantom::VKG::VulkanContext& ctx,
                VkCommandBuffer cmd,
                uint32_t frameIndex,
                SSFROffscreenSet& targets,
                VkImageView depthView,
                const glm::mat4& invProj,
                const glm::mat4& invViewRot,
                VkImageView envMapView,
                VkSampler   envMapSampler,
                bool        hasEnvMap);

    bool isValid() const { return pipeline_.isValid(); }

private:
    SSFRPipeline pipeline_;
    UBO ubo_{};

    // Dummy cube map filled at binding=3 when no environment map is set.
    VkImage        dummyImage_   = VK_NULL_HANDLE;
    VkDeviceMemory dummyMemory_  = VK_NULL_HANDLE;
    VkImageView    dummyView_    = VK_NULL_HANDLE;
    VkSampler      dummySampler_ = VK_NULL_HANDLE;

    void createDummyCubeMap(const Phantom::VKG::VulkanContext& ctx,
                            const Phantom::VKG::VulkanCommandPool& pool);
    void destroyDummyCubeMap(VkDevice device);
};

} // namespace VKSSFR
