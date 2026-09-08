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
        int passAxis = 0; // 0 = 2-D fallback, 1 = horizontal, 2 = vertical
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
    void setPassAxis(int axis) { ubo_.passAxis = glm::clamp(axis, 0, 2); }

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

    // Max render() calls per frame across every BilateralFilter instance --
    // SSFluidRenderer::onPreRender runs the depth bilateral as 3 separable
    // ping-pong iterations (6 render() calls). Each call needs its own
    // descriptor set + UBO buffer (see SSFRPassConfig::setsPerFrame): the shared
    // set was updated between draws while still bound (invalidating the command
    // buffer) and the shared UBO was overwritten before the earlier draw ran.
    static constexpr uint32_t kMaxCallsPerFrame = 8;

private:
    SSFRPipeline pipeline_;
    UBO ubo_{};
    // Monotonic slot cursor, wrapped to kMaxCallsPerFrame. SSFRPipeline keys the
    // set/UBO by (frame, slot), so same-slot reuse is always >= 2 frames apart
    // (the fence for that frame parity has been waited) -- no reset needed.
    uint32_t slotCursor_ = 0;
};

} // namespace VKSSFR
