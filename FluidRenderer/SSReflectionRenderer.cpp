#include "SSReflectionRenderer.h"
#include "../../CGLib/VulkanGraphics/VulkanContext.h"
#include "../../CGLib/VulkanGraphics/VulkanCommandPool.h"

#include <cassert>
#include <cstring>
#include <stdexcept>

namespace Phantom {

// ---------------------------------------------------------------------------
// Dummy cube map (1x1 pixels, placeholder for binding=3)
// ---------------------------------------------------------------------------

static void transitionCubeLayout(VkCommandBuffer cmd, VkImage image,
                                  VkImageLayout oldLayout, VkImageLayout newLayout,
                                  VkAccessFlags src, VkAccessFlags dst,
                                  VkPipelineStageFlags srcStage, VkPipelineStageFlags dstStage)
{
    VkImageMemoryBarrier b{};
    b.sType               = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    b.oldLayout           = oldLayout;
    b.newLayout           = newLayout;
    b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b.image               = image;
    b.subresourceRange    = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 6 };
    b.srcAccessMask       = src;
    b.dstAccessMask       = dst;
    vkCmdPipelineBarrier(cmd, srcStage, dstStage, 0, 0, nullptr, 0, nullptr, 1, &b);
}

void SSReflectionRenderer::createDummyCubeMap(const Phantom::VKG::VulkanContext& ctx,
                                                 const Phantom::VKG::VulkanCommandPool& pool)
{
    VkDevice device = ctx.getDevice();

    // 1x1 RGBA pixel * 6 faces
    const uint8_t kBlack[4] = { 0, 0, 0, 255 };
    constexpr VkDeviceSize faceBytes  = 4;
    constexpr VkDeviceSize totalBytes = 4 * 6;

    // Staging buffer
    VkBufferCreateInfo bci{};
    bci.sType       = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bci.size        = totalBytes;
    bci.usage       = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    bci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VkBuffer stagingBuf = VK_NULL_HANDLE;
    VkDeviceMemory stagingMem = VK_NULL_HANDLE;
    vkCreateBuffer(device, &bci, nullptr, &stagingBuf);

    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(device, stagingBuf, &req);

    auto stagingMemType = ctx.findMemoryType(req.memoryTypeBits,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    assert(stagingMemType.has_value());

    VkMemoryAllocateInfo mai{};
    mai.sType           = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    mai.allocationSize  = req.size;
    mai.memoryTypeIndex = stagingMemType.value_or(0);
    vkAllocateMemory(device, &mai, nullptr, &stagingMem);
    vkBindBufferMemory(device, stagingBuf, stagingMem, 0);

    void* mapped = nullptr;
    vkMapMemory(device, stagingMem, 0, totalBytes, 0, &mapped);
    for (int i = 0; i < 6; ++i)
        std::memcpy(static_cast<char*>(mapped) + faceBytes * i, kBlack, faceBytes);
    vkUnmapMemory(device, stagingMem);

    // 1x1 cube image
    VkImageCreateInfo ici{};
    ici.sType         = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    ici.flags         = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
    ici.imageType     = VK_IMAGE_TYPE_2D;
    ici.format        = VK_FORMAT_R8G8B8A8_UNORM;
    ici.extent        = { 1, 1, 1 };
    ici.mipLevels     = 1;
    ici.arrayLayers   = 6;
    ici.samples       = VK_SAMPLE_COUNT_1_BIT;
    ici.tiling        = VK_IMAGE_TILING_OPTIMAL;
    ici.usage         = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    ici.sharingMode   = VK_SHARING_MODE_EXCLUSIVE;
    ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    vkCreateImage(device, &ici, nullptr, &dummyImage_);

    VkMemoryRequirements imgReq;
    vkGetImageMemoryRequirements(device, dummyImage_, &imgReq);
    auto imgMemType = ctx.findMemoryType(imgReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    assert(imgMemType.has_value());

    VkMemoryAllocateInfo imgMAI{};
    imgMAI.sType           = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    imgMAI.allocationSize  = imgReq.size;
    imgMAI.memoryTypeIndex = imgMemType.value_or(0);
    vkAllocateMemory(device, &imgMAI, nullptr, &dummyMemory_);
    vkBindImageMemory(device, dummyImage_, dummyMemory_, 0);

    VkCommandBuffer cmd = pool.beginSingleTimeCommands();

    transitionCubeLayout(cmd, dummyImage_,
        VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        0, VK_ACCESS_TRANSFER_WRITE_BIT,
        VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);

    for (uint32_t i = 0; i < 6; ++i) {
        VkBufferImageCopy region{};
        region.bufferOffset     = faceBytes * i;
        region.imageSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, i, 1 };
        region.imageExtent      = { 1, 1, 1 };
        vkCmdCopyBufferToImage(cmd, stagingBuf, dummyImage_,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
    }

    transitionCubeLayout(cmd, dummyImage_,
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
        VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);

    pool.endSingleTimeCommands(cmd);

    vkDestroyBuffer(device, stagingBuf, nullptr);
    vkFreeMemory(device, stagingMem, nullptr);

    VkImageViewCreateInfo vci{};
    vci.sType    = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    vci.image    = dummyImage_;
    vci.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
    vci.format   = VK_FORMAT_R8G8B8A8_UNORM;
    vci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 6 };
    vkCreateImageView(device, &vci, nullptr, &dummyView_);

    VkSamplerCreateInfo sci{};
    sci.sType        = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sci.magFilter    = VK_FILTER_NEAREST;
    sci.minFilter    = VK_FILTER_NEAREST;
    sci.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sci.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sci.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    vkCreateSampler(device, &sci, nullptr, &dummySampler_);
}

void SSReflectionRenderer::destroyDummyCubeMap(VkDevice device)
{
    if (dummySampler_) { vkDestroySampler(device, dummySampler_, nullptr);     dummySampler_ = VK_NULL_HANDLE; }
    if (dummyView_)    { vkDestroyImageView(device, dummyView_, nullptr);       dummyView_    = VK_NULL_HANDLE; }
    if (dummyImage_)   { vkDestroyImage(device, dummyImage_, nullptr);          dummyImage_   = VK_NULL_HANDLE; }
    if (dummyMemory_)  { vkFreeMemory(device, dummyMemory_, nullptr);           dummyMemory_  = VK_NULL_HANDLE; }
}

// ---------------------------------------------------------------------------
// create / destroy
// ---------------------------------------------------------------------------

void SSReflectionRenderer::create(const Phantom::VKG::VulkanContext& ctx,
                                     const Phantom::VKG::VulkanCommandPool& pool,
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

    VkDescriptorSetLayoutBinding envMapBinding{};
    envMapBinding.binding         = 3;
    envMapBinding.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    envMapBinding.descriptorCount = 1;
    envMapBinding.stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;

    SSFRPassConfig cfg;
    cfg.vertSpv            = std::move(vertSpv);
    cfg.fragSpv            = std::move(fragSpv);
    cfg.topology           = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    cfg.depthTest          = false;
    cfg.depthWrite         = false;
    cfg.additiveBlend      = false;
    cfg.framesInFlight     = framesInFlight;
    cfg.uboSize            = sizeof(UBO);
    cfg.descriptorBindings = { depthBinding, thickBinding, uboBinding, envMapBinding };

    pipeline_.create(ctx, renderPass, cfg);

    createDummyCubeMap(ctx, pool);

    // Pre-fill binding=3 (dummy) for all frames' descriptor sets.
    VkDevice device = ctx.getDevice();
    for (uint32_t i = 0; i < framesInFlight; ++i) {
        VkDescriptorImageInfo envInfo{};
        envInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        envInfo.imageView   = dummyView_;
        envInfo.sampler     = dummySampler_;

        VkWriteDescriptorSet w{};
        w.sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w.dstSet          = pipeline_.getDescriptorSet(i);
        w.dstBinding      = 3;
        w.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w.descriptorCount = 1;
        w.pImageInfo      = &envInfo;
        vkUpdateDescriptorSets(device, 1, &w, 0, nullptr);
    }
}

void SSReflectionRenderer::destroy(VkDevice device)
{
    destroyDummyCubeMap(device);
    pipeline_.destroy(device);
}

// ---------------------------------------------------------------------------
// render
// ---------------------------------------------------------------------------

void SSReflectionRenderer::render(const Phantom::VKG::VulkanContext& ctx,
                                     VkCommandBuffer cmd,
                                     uint32_t frameIndex,
                                     SSFROffscreenSet& targets,
                                     VkImageView depthView,
                                     const glm::mat4& invProj,
                                     const glm::mat4& invViewRot,
                                     VkImageView envMapView,
                                     VkSampler   envMapSampler,
                                     bool        hasEnvMap)
{
    ubo_.invProj    = invProj;
    ubo_.invViewRot = invViewRot;
    ubo_.hasEnvMap  = hasEnvMap ? 1 : 0;
    pipeline_.updateUBO(frameIndex, &ubo_, sizeof(ubo_));

    VkDescriptorImageInfo depthInfo{};
    depthInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    depthInfo.imageView   = depthView;
    depthInfo.sampler     = targets.getSampler();

    VkDescriptorImageInfo thickInfo{};
    thickInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    thickInfo.imageView   = targets.smoothed().getColorImageView();
    thickInfo.sampler     = targets.getSampler();

    VkDescriptorImageInfo envInfo{};
    envInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    envInfo.imageView   = (envMapView != VK_NULL_HANDLE) ? envMapView   : dummyView_;
    envInfo.sampler     = (envMapView != VK_NULL_HANDLE) ? envMapSampler : dummySampler_;

    VkWriteDescriptorSet writes[3]{};
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

    writes[2].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[2].dstBinding      = 3;
    writes[2].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[2].descriptorCount = 1;
    writes[2].pImageInfo      = &envInfo;

    pipeline_.writeDescriptors(ctx.getDevice(), frameIndex,
                               { writes[0], writes[1], writes[2] });

    auto& offscreen = targets.reflection();
    offscreen.beginRenderPass(cmd, {0.f, 0.f, 0.f, 0.f}, 1.0f);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.getPipeline());
    VkDescriptorSet ds = pipeline_.getDescriptorSet(frameIndex);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            pipeline_.getLayout(), 0, 1, &ds, 0, nullptr);

    vkCmdDraw(cmd, 3, 1, 0, 0);

    offscreen.endRenderPass(cmd);
}

} // namespace VKSSFR
