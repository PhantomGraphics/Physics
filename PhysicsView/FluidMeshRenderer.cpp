#include "pch.h"
#include "FluidMeshRenderer.h"

#include "CGLib/VulkanGraphics/VulkanContext.h"
#include "CGLib/VulkanGraphics/VulkanCommandPool.h"

namespace Phantom {

void FluidMeshRenderer::update(const std::vector<float>&    positions,
                                const std::vector<float>&    colors,
                                const std::vector<uint32_t>& indices,
                                const glm::mat4&              mvp)
{
    positions_ = positions;
    colors_    = colors;
    indices_   = indices;
    mvp_       = mvp;
    dirty_     = true;
}

void FluidMeshRenderer::onInit(Phantom::VKG::VulkanContext& ctx,
                                const Phantom::VKG::VulkanCommandPool& pool,
                                VkRenderPass renderPass,
                                uint32_t framesInFlight)
{
    ctx_  = &ctx;
    pool_ = &pool;

    VKG::VkTriangleRenderer::Config cfg;
    cfg.vertSpv     = std::move(shaders_.vertSpv);
    cfg.fragSpv     = std::move(shaders_.fragSpv);
    cfg.blendEnable = false;
    cfg.cullBack    = false;
    triangleRenderer_.emplace(std::move(cfg));
    triangleRenderer_->create(ctx, pool, renderPass, framesInFlight);
}

void FluidMeshRenderer::onUpdate(uint32_t frameIndex)
{
    if (!triangleRenderer_) return;

    if (!dirty_) {
        triangleRenderer_->updateMVP(frameIndex, mvp_);
        return;
    }

    vkDeviceWaitIdle(ctx_->getDevice());

    VKG::VkTriangleRenderer::Buffer buf;
    buf.positions        = positions_;
    buf.colors           = colors_;
    buf.indices          = indices_;
    buf.projectionMatrix = mvp_;
    buf.modelViewMatrix  = glm::mat4{1.f};
    triangleRenderer_->upload(*ctx_, *pool_, buf);
    triangleRenderer_->updateMVP(frameIndex, mvp_);

    dirty_ = false;
}

void FluidMeshRenderer::onRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
    if (!enabled_) return;
    if (!triangleRenderer_ || !triangleRenderer_->isValid()) return;
    if (indices_.empty()) return;
    triangleRenderer_->render(cmd, frameIndex);
}

void FluidMeshRenderer::onCleanup(VkDevice device)
{
    if (triangleRenderer_) triangleRenderer_->destroy(device);
}

} // namespace Phantom
