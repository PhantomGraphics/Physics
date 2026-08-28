#include "pch.h"
#include "RigidBodyWireRenderer.h"

#include "CGLib/VulkanGraphics/VulkanContext.h"
#include "CGLib/VulkanGraphics/VulkanCommandPool.h"

namespace Phantom {

void RigidBodyWireRenderer::update(const std::vector<float>&    positions,
                                    const std::vector<float>&    colors,
                                    const std::vector<uint32_t>& indices,
                                    const glm::mat4&             mvp)
{
    positions_ = positions;
    colors_    = colors;
    indices_   = indices;
    mvp_       = mvp;
    dirty_     = true;
}

void RigidBodyWireRenderer::onInit(Phantom::VKG::VulkanContext& ctx,
                                    const Phantom::VKG::VulkanCommandPool& pool,
                                    VkRenderPass renderPass,
                                    uint32_t framesInFlight)
{
    ctx_  = &ctx;
    pool_ = &pool;

    VKG::VkLineRenderer::Config cfg;
    cfg.vertSpv = std::move(shaders_.vertSpv);
    cfg.fragSpv = std::move(shaders_.fragSpv);
    lineRenderer_.emplace(std::move(cfg));
    lineRenderer_->create(ctx, pool, renderPass, framesInFlight);
}

void RigidBodyWireRenderer::onUpdate(uint32_t frameIndex)
{
    if (!lineRenderer_) return;

    if (!dirty_) {
        lineRenderer_->updateMVP(frameIndex, mvp_);
        return;
    }

    vkDeviceWaitIdle(ctx_->getDevice());

    VKG::VkLineRenderer::Buffer buf;
    buf.positions        = positions_;
    buf.colors           = colors_;
    buf.indices          = indices_;
    buf.projectionMatrix = mvp_;
    buf.modelViewMatrix  = glm::mat4{1.f};
    lineRenderer_->upload(*ctx_, *pool_, buf);
    lineRenderer_->updateMVP(frameIndex, mvp_);

    dirty_ = false;
}

void RigidBodyWireRenderer::onRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
    if (!lineRenderer_ || !lineRenderer_->isValid()) return;
    if (indices_.empty()) return;
    lineRenderer_->render(cmd, frameIndex);
}

void RigidBodyWireRenderer::onCleanup(VkDevice device)
{
    if (lineRenderer_) lineRenderer_->destroy(device);
}

} // namespace Phantom
