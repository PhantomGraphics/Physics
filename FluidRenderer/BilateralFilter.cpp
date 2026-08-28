#include "BilateralFilter.h"
#include "../../CGLib/VulkanGraphics/VulkanContext.h"

namespace Phantom {

void BilateralFilter::create(const Phantom::VKG::VulkanContext& ctx,
                               uint32_t framesInFlight,
                               VkRenderPass renderPass,
                               std::vector<uint32_t> vertSpv,
                               std::vector<uint32_t> fragSpv)
{
    VkDescriptorSetLayoutBinding texBinding{};
    texBinding.binding         = 0;
    texBinding.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    texBinding.descriptorCount = 1;
    texBinding.stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutBinding uboBinding{};
    uboBinding.binding         = 1;
    uboBinding.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    uboBinding.descriptorCount = 1;
    uboBinding.stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;

    SSFRPassConfig cfg;
    cfg.vertSpv            = std::move(vertSpv);
    cfg.fragSpv            = std::move(fragSpv);
    cfg.topology           = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    cfg.depthTest          = false;
    cfg.depthWrite         = false;
    cfg.additiveBlend      = false;
    cfg.framesInFlight     = framesInFlight;
    cfg.uboSize            = sizeof(UBO);
    cfg.descriptorBindings = { texBinding, uboBinding };

    pipeline_.create(ctx, renderPass, cfg);
}

void BilateralFilter::destroy(VkDevice device)
{
    pipeline_.destroy(device);
}

void BilateralFilter::setParams(float sigmaS, float sigmaR,
                                  bool useAnisotropic, float anisotropy, float gradientScale)
{
    ubo_.sigmaS = sigmaS;
    ubo_.sigmaR = sigmaR;
    ubo_.useAnisotropic = useAnisotropic ? 1 : 0;
    ubo_.anisotropy = anisotropy;
    ubo_.gradientScale = gradientScale;
}

void BilateralFilter::render(const Phantom::VKG::VulkanContext& ctx,
                               VkCommandBuffer cmd,
                               uint32_t frameIndex,
                               SSFROffscreenSet& targets)
{
    render(ctx, cmd, frameIndex,
           targets.thickness().getColorImageView(),
           targets.getSampler(),
           targets.smoothed());
}

void BilateralFilter::render(const Phantom::VKG::VulkanContext& ctx,
                               VkCommandBuffer cmd,
                               uint32_t frameIndex,
                               VkImageView srcView,
                               VkSampler sampler,
                                Phantom::VKG::VulkanOffscreen& dst)
{
    const auto extent = dst.getExtent();
    ubo_.texelSize = glm::vec2(1.0f / static_cast<float>(extent.width),
                               1.0f / static_cast<float>(extent.height));
    pipeline_.updateUBO(frameIndex, &ubo_, sizeof(ubo_));

    VkDescriptorImageInfo ii{};
    ii.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    ii.imageView   = srcView;
    ii.sampler     = sampler;

    VkWriteDescriptorSet w{};
    w.sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    w.dstBinding      = 0;
    w.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    w.descriptorCount = 1;
    w.pImageInfo      = &ii;

    pipeline_.writeDescriptors(ctx.getDevice(), frameIndex, { w });

    dst.beginRenderPass(cmd, {0.f, 0.f, 0.f, 0.f}, 1.0f);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.getPipeline());
    VkDescriptorSet ds = pipeline_.getDescriptorSet(frameIndex);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            pipeline_.getLayout(), 0, 1, &ds, 0, nullptr);

    vkCmdDraw(cmd, 3, 1, 0, 0);

    dst.endRenderPass(cmd);
}

} // namespace VKSSFR
