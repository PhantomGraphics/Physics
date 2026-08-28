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
        float _pad[3]{};
    };
    static_assert(sizeof(UBO) == 32, "UBO layout mismatch");

    void create(const Phantom::VKG::VulkanContext& ctx,
                uint32_t framesInFlight,
                VkRenderPass renderPass,
                std::vector<uint32_t> vertSpv,
                std::vector<uint32_t> fragSpv);

    void destroy(VkDevice device);

    void render(const Phantom::VKG::VulkanContext& ctx,
                VkCommandBuffer cmd,
                uint32_t frameIndex,
                SSFROffscreenSet& targets);

    bool isValid() const { return pipeline_.isValid(); }

private:
    SSFRPipeline pipeline_;
    UBO ubo_{};

};

} // namespace VKSSFR
