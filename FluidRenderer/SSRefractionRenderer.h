#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "SSFRPipeline.h"
#include "SSFROffscreenSet.h"

#include <vector>

namespace Phantom::VKG { class VulkanContext; }

namespace Phantom {

class SSRefractionRenderer {
public:
    struct UBO {
        glm::vec4 tint = glm::vec4(0.12f, 0.62f, 0.72f, 1.0f);
        float strength = 1.0f;
        int hasScene = 0;
        int hasEnvMap = 0;
        float ior = 1.333f;
        glm::mat4 invProj{1.0f};
        glm::mat4 invViewRot{1.0f};
        glm::vec4 viewportNearFar{1.0f, 1.0f, 0.1f, 1000.0f};
        glm::vec4 absorptionColor{0.12f, 0.62f, 0.72f, 1.0f};
    };
    static_assert(sizeof(UBO) == 192, "UBO layout mismatch");

    void create(const Phantom::VKG::VulkanContext& ctx,
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
                VkImageView sceneColor, VkImageView sceneDepth, VkSampler sceneSampler,
                VkImageView envMap, VkSampler envSampler,
                const glm::mat4& invProj, const glm::mat4& invViewRot,
                VkExtent2D extent, float nearPlane, float farPlane,
                bool hasScene, bool hasEnvMap, float ior,
                const glm::vec3& absorptionColor);

    bool isValid() const { return pipeline_.isValid(); }

private:
    SSFRPipeline pipeline_;
    UBO ubo_{};

};

} // namespace VKSSFR
