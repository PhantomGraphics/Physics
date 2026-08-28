#include "SSFROffscreenSet.h"
#include "../../CGLib/VulkanGraphics/VulkanContext.h"

namespace Phantom {

void SSFROffscreenSet::create(const Phantom::VKG::VulkanContext& ctx, uint32_t width, uint32_t height)
{
    extent_ = { width, height };

    // All targets use D32_SFLOAT for the depth attachment.
    // Used only for the render pass depth test, so a single format is shared.
    constexpr VkFormat kDepthFmt = VK_FORMAT_D32_SFLOAT;

    depth_.create        (ctx, width, height, VK_FORMAT_R32_SFLOAT,     kDepthFmt);
    thickness_.create    (ctx, width, height, VK_FORMAT_R32_SFLOAT,     kDepthFmt);
    smoothed_.create     (ctx, width, height, VK_FORMAT_R32_SFLOAT,     kDepthFmt);
    smoothedDepth_.create(ctx, width, height, VK_FORMAT_R32_SFLOAT,     kDepthFmt);
    reflection_.create   (ctx, width, height, VK_FORMAT_R8G8B8A8_UNORM, kDepthFmt);
    refraction_.create   (ctx, width, height, VK_FORMAT_R8G8B8A8_UNORM, kDepthFmt);
    spray_.create        (ctx, width, height, VK_FORMAT_R32_SFLOAT,     kDepthFmt);
    foam_.create         (ctx, width, height, VK_FORMAT_R32_SFLOAT,     kDepthFmt);

    sampler_.create(ctx.getDevice(),
                    VK_FILTER_LINEAR,
                    VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE);
}

void SSFROffscreenSet::destroy(const Phantom::VKG::VulkanContext& ctx)
{
    sampler_.destroy(ctx.getDevice());

    // Reverse creation order.
    foam_.destroy(ctx);
    spray_.destroy(ctx);
    refraction_.destroy(ctx);
    reflection_.destroy(ctx);
    smoothedDepth_.destroy(ctx);
    smoothed_.destroy(ctx);
    thickness_.destroy(ctx);
    depth_.destroy(ctx);

    extent_ = {};
}

} // namespace VKSSFR
