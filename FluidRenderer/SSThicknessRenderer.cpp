#include "SSThicknessRenderer.h"
#include "../../CGLib/VulkanGraphics/VulkanContext.h"

namespace Phantom {

void SSThicknessRenderer::create(const Phantom::VKG::VulkanContext& ctx,
                                   uint32_t framesInFlight,
                                   VkRenderPass renderPass,
                                   std::vector<uint32_t> vertSpv,
                                   std::vector<uint32_t> fragSpv)
{
    VkVertexInputBindingDescription binding{};
    binding.binding   = 0;
    binding.stride    = sizeof(glm::vec3);
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    VkVertexInputAttributeDescription attr{};
    attr.location = 0;
    attr.binding  = 0;
    attr.format   = VK_FORMAT_R32G32B32_SFLOAT;
    attr.offset   = 0;

    VkDescriptorSetLayoutBinding uboBinding{};
    uboBinding.binding         = 0;
    uboBinding.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    uboBinding.descriptorCount = 1;
    uboBinding.stageFlags      = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

    SSFRPassConfig cfg;
    cfg.vertSpv            = std::move(vertSpv);
    cfg.fragSpv            = std::move(fragSpv);
    cfg.bindingDescs       = { binding };
    cfg.attrDescs          = { attr };
    cfg.descriptorBindings = { uboBinding };
    cfg.topology           = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
    cfg.depthTest          = false;
    cfg.depthWrite         = false;
    cfg.additiveBlend      = true;
    cfg.framesInFlight     = framesInFlight;
    cfg.uboSize            = sizeof(UBO);

    pipeline_.create(ctx, renderPass, cfg);

    VkVertexInputBindingDescription bindingVec4{};
    bindingVec4.binding   = 0;
    bindingVec4.stride    = sizeof(glm::vec4); // 16 bytes
    bindingVec4.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    SSFRPassConfig cfgV4 = cfg;
    cfgV4.bindingDescs = { bindingVec4 };
    pipelineVec4_.create(ctx, renderPass, cfgV4);
}

void SSThicknessRenderer::destroy(VkDevice device)
{
    vertexBuffer_.destroy(device);
    pipeline_.destroy(device);
    pipelineVec4_.destroy(device);
    particleCount_ = 0;
}

void SSThicknessRenderer::setParticles(
    const Phantom::VKG::VulkanContext& ctx,
    const Phantom::VKG::VulkanCommandPool& pool,
    const std::vector<glm::vec3>& positions)
{
    vertexBuffer_.destroy(ctx.getDevice());
    particleCount_ = static_cast<uint32_t>(positions.size());

    if (particleCount_ == 0) return;

    vertexBuffer_.create(ctx, pool,
                         sizeof(glm::vec3) * positions.size(),
                         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                         positions.data());
}

void SSThicknessRenderer::render(
    VkCommandBuffer cmd,
    uint32_t frameIndex,
    SSFROffscreenSet& targets,
    const glm::mat4& proj,
    const glm::mat4& modelView,
    float pointSize,
    float thicknessScale)
{
    UBO ubo{};
    ubo.proj           = proj;
    ubo.modelView      = modelView;
    ubo.pointSize      = pointSize;
    ubo.thicknessScale = thicknessScale;

    SSFRPipeline& activePipeline = useExternal_ ? pipelineVec4_ : pipeline_;
    activePipeline.updateUBO(frameIndex, &ubo, sizeof(ubo));

    auto& offscreen = targets.thickness();
    offscreen.beginRenderPass(cmd, {0.f, 0.f, 0.f, 0.f}, 1.0f);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, activePipeline.getPipeline());

    VkDescriptorSet ds = activePipeline.getDescriptorSet(frameIndex);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            activePipeline.getLayout(), 0, 1, &ds, 0, nullptr);

    VkBuffer  drawBuf   = useExternal_ ? externalBuf_   : vertexBuffer_.get();
    uint32_t  drawCount = useExternal_ ? externalCount_ : particleCount_;

    if (drawCount > 0 && drawBuf != VK_NULL_HANDLE) {
        VkDeviceSize offset = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &drawBuf, &offset);
        vkCmdDraw(cmd, drawCount, 1, 0, 0);
    }

    offscreen.endRenderPass(cmd);
}

void SSThicknessRenderer::setExternalBuffer(VkBuffer buf, uint32_t count)
{
    externalBuf_   = buf;
    externalCount_ = count;
    useExternal_   = true;
}

void SSThicknessRenderer::clearExternalBuffer()
{
    externalBuf_   = VK_NULL_HANDLE;
    externalCount_ = 0;
    useExternal_   = false;
}

} // namespace VKSSFR
