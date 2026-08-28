#include "pch.h"
#include "VolumeRenderer.h"

#include "CGLib/VulkanGraphics/VulkanContext.h"
#include "CGLib/VulkanGraphics/VulkanCommandPool.h"

namespace Phantom {

void VolumeRenderer::update(const std::vector<float>& positions,
                                  const std::vector<float>& colors,
                                  const std::vector<float>& sizes,
                                  const glm::mat4&          mvp)
{
    positions_ = positions;
    colors_    = colors;
    sizes_     = sizes;
    mvp_       = mvp;
}

void VolumeRenderer::onInit(Phantom::VKG::VulkanContext& ctx,
                                  const Phantom::VKG::VulkanCommandPool& pool,
                                  VkRenderPass renderPass,
                                  uint32_t framesInFlight)
{
    ctx_  = &ctx;
    pool_ = &pool;

    VKG::VkPointRenderer::Config cfg;
    cfg.vertSpv = std::move(shaders_.vertSpv);
    cfg.fragSpv = std::move(shaders_.fragSpv);
    pointRenderer_.emplace(std::move(cfg));
    pointRenderer_->create(ctx, pool, renderPass, framesInFlight);
}

void VolumeRenderer::onUpdate(uint32_t frameIndex)
{
    if (!pointRenderer_ || !enabled_) return;
    if (positions_.empty()) return;

    // VkPointRenderer bakes projection*modelView into its uniform buffer at
    // upload() time (it has no separate per-frame updateMVP() like
    // VkLineRenderer/VkTriangleRenderer) -- re-upload every frame so the
    // points track the shared orbit camera, mirroring VkSpaceRenderer's
    // point-renderer usage (CGLib/Space/SpaceView/VkSpaceRenderer.cpp).
    VKG::VkPointRenderer::Buffer buf;
    buf.positions        = positions_;
    buf.colors           = colors_;
    buf.sizes            = sizes_;
    buf.projectionMatrix = mvp_;
    buf.modelViewMatrix  = glm::mat4{1.f};
    pointRenderer_->upload(*ctx_, *pool_, buf);
}

void VolumeRenderer::onRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
    if (!enabled_) return;
    if (!pointRenderer_ || !pointRenderer_->isValid()) return;
    if (positions_.empty()) return;
    pointRenderer_->render(cmd, frameIndex);
}

void VolumeRenderer::onCleanup(VkDevice device)
{
    if (pointRenderer_) pointRenderer_->destroy(device);
}

} // namespace Phantom
