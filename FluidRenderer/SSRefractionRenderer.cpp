#include "SSRefractionRenderer.h"
#include "../../CGLib/VulkanGraphics/VulkanContext.h"

namespace Phantom {

void SSRefractionRenderer::create(const Phantom::VKG::VulkanContext& ctx,
                                    uint32_t framesInFlight,
                                    VkRenderPass renderPass,
                                    std::vector<uint32_t> vertSpv,
                                    std::vector<uint32_t> fragSpv)
{
    VkDescriptorSetLayoutBinding depthBinding{};
    depthBinding.binding         = 0;
    depthBinding.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    depthBinding.descriptorCount = 1;
    depthBinding.stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutBinding thickBinding{};
    thickBinding.binding         = 1;
    thickBinding.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    thickBinding.descriptorCount = 1;
    thickBinding.stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutBinding uboBinding{};
    uboBinding.binding         = 2;
    uboBinding.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    uboBinding.descriptorCount = 1;
    uboBinding.stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;

    std::vector<VkDescriptorSetLayoutBinding> bindings{ depthBinding, thickBinding, uboBinding };
    for (uint32_t binding = 3; binding <= 5; ++binding) {
        VkDescriptorSetLayoutBinding imageBinding{};
        imageBinding.binding = binding;
        imageBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        imageBinding.descriptorCount = 1;
        imageBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        bindings.push_back(imageBinding);
    }

    SSFRPassConfig cfg;
    cfg.vertSpv            = std::move(vertSpv);
    cfg.fragSpv            = std::move(fragSpv);
    cfg.topology           = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    cfg.depthTest          = false;
    cfg.depthWrite         = false;
    cfg.additiveBlend      = false;
    cfg.framesInFlight     = framesInFlight;
    cfg.uboSize            = sizeof(UBO);
    cfg.descriptorBindings = std::move(bindings);

    pipeline_.create(ctx, renderPass, cfg);
}

void SSRefractionRenderer::destroy(VkDevice device)
{
    pipeline_.destroy(device);
}

void SSRefractionRenderer::render(const Phantom::VKG::VulkanContext& ctx,
                                    VkCommandBuffer cmd,
                                    uint32_t frameIndex,
                                    SSFROffscreenSet& targets,
                                    VkImageView depthView,
                                    VkImageView sceneColor, VkImageView sceneDepth, VkSampler sceneSampler,
                                    VkImageView envMap, VkSampler envSampler,
                                    const glm::mat4& invProj, const glm::mat4& invViewRot,
                                    VkExtent2D extent, float nearPlane, float farPlane,
                                    bool hasScene, bool hasEnvMap, float ior,
                                    const glm::vec3& absorptionColor)
{
    ubo_.hasScene = hasScene ? 1 : 0;
    ubo_.hasEnvMap = hasEnvMap ? 1 : 0;
    ubo_.invProj = invProj;
    ubo_.invViewRot = invViewRot;
    ubo_.ior = ior;
    ubo_.absorptionColor = glm::vec4(absorptionColor, 1.0f);
    ubo_.viewportNearFar = glm::vec4(static_cast<float>(extent.width), static_cast<float>(extent.height),
                                     nearPlane, farPlane);
    pipeline_.updateUBO(frameIndex, &ubo_, sizeof(ubo_));

    VkDescriptorImageInfo depthInfo{};
    depthInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    depthInfo.imageView   = depthView;
    depthInfo.sampler     = targets.getSampler();

    VkDescriptorImageInfo thickInfo{};
    thickInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    thickInfo.imageView   = targets.smoothed().getColorImageView();
    thickInfo.sampler     = targets.getSampler();

    VkDescriptorImageInfo sceneColorInfo{ targets.getSampler(), targets.reflection().getColorImageView(),
                                          VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
    VkDescriptorImageInfo sceneDepthInfo{ targets.getSampler(), depthView,
                                          VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
    VkDescriptorImageInfo envInfo{ envSampler, envMap,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
    if (hasScene) {
        sceneColorInfo.sampler = sceneSampler;
        sceneColorInfo.imageView = sceneColor;
        sceneDepthInfo.sampler = sceneSampler;
        sceneDepthInfo.imageView = sceneDepth;
        sceneDepthInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
    }
    VkWriteDescriptorSet writes[5]{};
    writes[0].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[0].dstBinding      = 0;
    writes[0].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[0].descriptorCount = 1;
    writes[0].pImageInfo      = &depthInfo;

    writes[1].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[1].dstBinding      = 1;
    writes[1].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[1].descriptorCount = 1;
    writes[1].pImageInfo      = &thickInfo;

    const VkDescriptorImageInfo* extraInfos[] = { &sceneColorInfo, &sceneDepthInfo, &envInfo };
    for (uint32_t i = 0; i < 3; ++i) {
        writes[i + 2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[i + 2].dstBinding = i + 3;
        writes[i + 2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[i + 2].descriptorCount = 1;
        writes[i + 2].pImageInfo = extraInfos[i];
    }

    pipeline_.writeDescriptors(ctx.getDevice(), frameIndex,
                               { writes[0], writes[1], writes[2], writes[3], writes[4] });

    auto& offscreen = targets.refraction();
    offscreen.beginRenderPass(cmd, {0.f, 0.f, 0.f, 0.f}, 1.0f);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.getPipeline());
    VkDescriptorSet ds = pipeline_.getDescriptorSet(frameIndex);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            pipeline_.getLayout(), 0, 1, &ds, 0, nullptr);

    vkCmdDraw(cmd, 3, 1, 0, 0);

    offscreen.endRenderPass(cmd);
}

} // namespace VKSSFR
