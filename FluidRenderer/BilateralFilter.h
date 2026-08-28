#pragma once


#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "SSFRPipeline.h"
#include "SSFROffscreenSet.h"
#include "../../CGLib/VulkanGraphics/VulkanOffscreen.h"

#include <vector>

namespace Phantom::VKG { class VulkanContext; }

namespace Phantom {

class BilateralFilter {
public:
    struct UBO {
        glm::vec2 texelSize = glm::vec2(1.0f, 1.0f);
        float sigmaS = 2.0f;
        float sigmaR = 0.08f;
        int useAnisotropic = 1;
        float anisotropy = 1.25f;
        float gradientScale = 8.0f;
        float _pad = 0.0f;
    };
    static_assert(sizeof(UBO) == 32, "UBO layout mismatch");

    void create(const Phantom::VKG::VulkanContext& ctx,
                uint32_t framesInFlight,
                VkRenderPass renderPass,
                std::vector<uint32_t> vertSpv,
                std::vector<uint32_t> fragSpv);

    void destroy(VkDevice device);

    void setParams(float sigmaS, float sigmaR,
                   bool useAnisotropic, float anisotropy, float gradientScale);

    void render(const Phantom::VKG::VulkanContext& ctx,
                VkCommandBuffer cmd,
                uint32_t frameIndex,
                SSFROffscreenSet& targets);

    void render(const Phantom::VKG::VulkanContext& ctx,
                VkCommandBuffer cmd,
                uint32_t frameIndex,
                VkImageView srcView,
                VkSampler sampler,
                Phantom::VKG::VulkanOffscreen& dst);

    bool isValid() const { return pipeline_.isValid(); }

private:
    SSFRPipeline pipeline_;
    UBO ubo_{};

};

} // namespace VKSSFR
