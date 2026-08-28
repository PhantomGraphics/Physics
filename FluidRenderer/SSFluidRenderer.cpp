#include "SSFluidRenderer.h"

#include "../../CGLib/VulkanGraphics/VulkanContext.h"

#include <iostream>

namespace Phantom {

namespace {

void transitionOffscreenColorToShaderReadOnly(const GlobalVulkanCommandPool& pool,
                                              VkImage image)
{
    VkCommandBuffer cmd = pool.beginSingleTimeCommands();

    VkImageMemoryBarrier barrier{};
    barrier.sType               = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrier.oldLayout           = VK_IMAGE_LAYOUT_UNDEFINED;
    barrier.newLayout           = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image               = image;
    barrier.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.baseMipLevel   = 0;
    barrier.subresourceRange.levelCount     = 1;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount     = 1;
    barrier.srcAccessMask       = 0;
    barrier.dstAccessMask       = VK_ACCESS_SHADER_READ_BIT;

    vkCmdPipelineBarrier(cmd,
                         VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                         VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                         0,
                         0, nullptr,
                         0, nullptr,
                         1, &barrier);

    pool.endSingleTimeCommands(cmd);
}

void initializeSSFRTargetLayouts(const GlobalVulkanCommandPool& pool,
                                 SSFROffscreenSet& targets)
{
    const std::array<VkImage, 8> images = {
        targets.depth().getColorImage(),
        targets.thickness().getColorImage(),
        targets.smoothed().getColorImage(),
        targets.smoothedDepth().getColorImage(),
        targets.reflection().getColorImage(),
        targets.refraction().getColorImage(),
        targets.spray().getColorImage(),
        targets.foam().getColorImage(),
    };

    for (VkImage image : images) {
        if (image != VK_NULL_HANDLE) {
            transitionOffscreenColorToShaderReadOnly(pool, image);
        }
    }
}

} // namespace

void SSFluidRenderer::setParticles(const std::vector<glm::vec3>& positions)
{
    std::lock_guard<std::mutex> lock(mutex_);
    pendingPositions_ = positions;
    dirty_ = true;
}

void SSFluidRenderer::setSprayParticles(const std::vector<glm::vec3>& positions)
{
    std::lock_guard<std::mutex> lock(mutex_);
    pendingSprayPositions_ = positions;
    sprayDirty_ = true;
}

void SSFluidRenderer::setFoamParticles(const std::vector<glm::vec3>& positions)
{
    std::lock_guard<std::mutex> lock(mutex_);
    pendingFoamPositions_ = positions;
    foamDirty_ = true;
}

void SSFluidRenderer::setParticleBuffer(VkBuffer buf, uint32_t count)
{
    depthPass_.setExternalBuffer(buf, count);
    thicknessPass_.setExternalBuffer(buf, count);
    // Clear CPU-side pending data to prevent stale upload.
    std::lock_guard<std::mutex> lock(mutex_);
    pendingPositions_.clear();
    dirty_ = false;
}

void SSFluidRenderer::clearParticleBuffer()
{
    depthPass_.clearExternalBuffer();
    thicknessPass_.clearExternalBuffer();
}

void SSFluidRenderer::setCamera(const glm::mat4& proj, const glm::mat4& view)
{
    proj_ = proj;
    view_ = view;
}

void SSFluidRenderer::onInit(GlobalVulkanContext& ctx,
                             const GlobalVulkanCommandPool& pool,
                             VkRenderPass renderPass,
                             uint32_t framesInFlight)
{
    ctx_ = &ctx;
    pool_ = &pool;
    framesInFlight_ = framesInFlight;

    if (!createPassResources(renderPass)) {
        destroyPassResources(ctx.getDevice());
        ctx_ = nullptr;
        pool_ = nullptr;
        framesInFlight_ = 0;
        enabled_ = false;
        std::cerr << "[VKSSFR] Disabled: failed to create pass resources" << std::endl;
    }
}

bool SSFluidRenderer::createPassResources(VkRenderPass mainRenderPass)
{
    targets_.create(*ctx_, extent_.width, extent_.height);
    if (!targets_.isValid()) {
        std::cerr << "[VKSSFR] Failed to create offscreen targets" << std::endl;
        return false;
    }
    initializeSSFRTargetLayouts(*pool_, targets_);

    depthPass_.create(*ctx_, framesInFlight_, targets_.depth().getRenderPass(),
                      std::move(shaders_.depthVert), std::move(shaders_.depthFrag));
    if (!depthPass_.isValid()) {
        std::cerr << "[VKSSFR] Failed to create depth pass" << std::endl;
        return false;
    }

    // thickness shaders are reused for spray and foam passes 窶・copy for the first two
    thicknessPass_.create(*ctx_, framesInFlight_, targets_.thickness().getRenderPass(),
                          shaders_.thicknessVert, shaders_.thicknessFrag);
    if (!thicknessPass_.isValid()) {
        std::cerr << "[VKSSFR] Failed to create thickness pass" << std::endl;
        return false;
    }
    sprayPass_.create(*ctx_, framesInFlight_, targets_.spray().getRenderPass(),
                      shaders_.thicknessVert, shaders_.thicknessFrag);
    if (!sprayPass_.isValid()) {
        std::cerr << "[VKSSFR] Failed to create spray pass" << std::endl;
        return false;
    }
    foamPass_.create(*ctx_, framesInFlight_, targets_.foam().getRenderPass(),
                     std::move(shaders_.thicknessVert), std::move(shaders_.thicknessFrag));
    if (!foamPass_.isValid()) {
        std::cerr << "[VKSSFR] Failed to create foam pass" << std::endl;
        return false;
    }

    // bilateral shaders are reused for the depth bilateral pass 窶・copy for the first
    bilateralPass_.create(*ctx_, framesInFlight_, targets_.smoothed().getRenderPass(),
                          shaders_.bilateralVert, shaders_.bilateralFrag);
    if (!bilateralPass_.isValid()) {
        std::cerr << "[VKSSFR] Failed to create bilateral pass" << std::endl;
        return false;
    }
    bilateralDepthPass_.create(*ctx_, framesInFlight_, targets_.smoothedDepth().getRenderPass(),
                               std::move(shaders_.bilateralVert), std::move(shaders_.bilateralFrag));
    if (!bilateralDepthPass_.isValid()) {
        std::cerr << "[VKSSFR] Failed to create bilateral depth pass" << std::endl;
        return false;
    }

    reflectionPass_.create(*ctx_, *pool_, framesInFlight_, targets_.reflection().getRenderPass(),
                           std::move(shaders_.reflectionVert), std::move(shaders_.reflectionFrag));
    if (!reflectionPass_.isValid()) {
        std::cerr << "[VKSSFR] Failed to create reflection pass" << std::endl;
        return false;
    }
    refractionPass_.create(*ctx_, framesInFlight_, targets_.refraction().getRenderPass(),
                           std::move(shaders_.refractionVert), std::move(shaders_.refractionFrag));
    if (!refractionPass_.isValid()) {
        std::cerr << "[VKSSFR] Failed to create refraction pass" << std::endl;
        return false;
    }

    if (!dummyCubeMap_.createDummy(*ctx_, *pool_)) {
        std::cerr << "[VKSSFR] Failed to create dummy cube map" << std::endl;
        return false;
    }

    if (!shaders_.skyboxVert.empty() && !shaders_.skyboxFrag.empty()) {
        Phantom::VKG::VkSkyBoxRenderer::Config sbCfg;
        sbCfg.vertSpv = std::move(shaders_.skyboxVert);
        sbCfg.fragSpv = std::move(shaders_.skyboxFrag);
        skyBoxRenderer_ = std::make_unique<Phantom::VKG::VkSkyBoxRenderer>(std::move(sbCfg));
        skyBoxRenderer_->create(*ctx_, *pool_, mainRenderPass, framesInFlight_);
        if (!skyBoxRenderer_->isValid()) {
            // Skybox is cosmetic; degrade gracefully instead of disabling SSFR entirely.
            std::cerr << "[VKSSFR] Failed to create skybox renderer; continuing without it" << std::endl;
            skyBoxRenderer_.reset();
        }
    }

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

    VkDescriptorSetLayoutBinding smoothBinding{};
    smoothBinding.binding         = 2;
    smoothBinding.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    smoothBinding.descriptorCount = 1;
    smoothBinding.stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutBinding reflBinding{};
    reflBinding.binding         = 3;
    reflBinding.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    reflBinding.descriptorCount = 1;
    reflBinding.stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutBinding refrBinding{};
    refrBinding.binding         = 4;
    refrBinding.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    refrBinding.descriptorCount = 1;
    refrBinding.stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutBinding sprayBinding{};
    sprayBinding.binding         = 5;
    sprayBinding.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    sprayBinding.descriptorCount = 1;
    sprayBinding.stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutBinding foamBinding{};
    foamBinding.binding         = 6;
    foamBinding.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    foamBinding.descriptorCount = 1;
    foamBinding.stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutBinding uboBinding{};
    uboBinding.binding         = 7;
    uboBinding.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    uboBinding.descriptorCount = 1;
    uboBinding.stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;

    SSFRPassConfig cfg;
    cfg.vertSpv            = std::move(shaders_.compositeVert);
    cfg.fragSpv            = std::move(shaders_.compositeFrag);
    cfg.topology           = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    cfg.depthTest          = false;
    cfg.depthWrite         = false;
    cfg.additiveBlend      = false;
    cfg.framesInFlight     = framesInFlight_;
    cfg.uboSize            = sizeof(CompositeUBO);
    cfg.descriptorBindings = {
        depthBinding, thickBinding, smoothBinding,
        reflBinding, refrBinding, sprayBinding, foamBinding, uboBinding
    };

    if (!compositePipeline_.create(*ctx_, mainRenderPass, cfg))
        return false;
    return true;
}

void SSFluidRenderer::destroyPassResources(VkDevice device)
{
    compositePipeline_.destroy(device);

    refractionPass_.destroy(device);
    reflectionPass_.destroy(device);
    bilateralDepthPass_.destroy(device);
    bilateralPass_.destroy(device);
    foamPass_.destroy(device);
    sprayPass_.destroy(device);
    thicknessPass_.destroy(device);
    depthPass_.destroy(device);

    if (skyBoxRenderer_) {
        skyBoxRenderer_->destroy(device);
        skyBoxRenderer_.reset();
    }

    if (envMap_.isValid())       envMap_.destroy(device);
    if (dummyCubeMap_.isValid()) dummyCubeMap_.destroy(device);
    hasEnvMap_ = false;

    if (ctx_) {
        targets_.destroy(*ctx_);
    }
}

void SSFluidRenderer::loadEnvMap(const std::array<std::string, 6>& facePaths)
{
    if (!ctx_ || !pool_) {
        std::cerr << "[VKSSFR] loadEnvMap: renderer not initialized" << std::endl;
        return;
    }

    if (envMap_.isValid()) envMap_.destroy(ctx_->getDevice());
    if (!envMap_.create(*ctx_, *pool_, facePaths)) {
        std::cerr << "[VKSSFR] loadEnvMap failed" << std::endl;
        return;
    }
    if (skyBoxRenderer_) {
        skyBoxRenderer_->setCubeMap(ctx_->getDevice(),
                                    envMap_.getImageView(),
                                    envMap_.getSampler());
    }
    hasEnvMap_ = true;
}

void SSFluidRenderer::onUpdate(uint32_t)
{
    if (!ctx_) return;

    std::vector<glm::vec3> uploadData;
    std::vector<glm::vec3> sprayUploadData;
    std::vector<glm::vec3> foamUploadData;
    bool hasNewData = false;
    bool hasNewSprayData = false;
    bool hasNewFoamData = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (dirty_) {
            uploadData = pendingPositions_;
            dirty_ = false;
            hasNewData = true;
        }
        if (sprayDirty_) {
            sprayUploadData = pendingSprayPositions_;
            sprayDirty_ = false;
            hasNewSprayData = true;
        }
        if (foamDirty_) {
            foamUploadData = pendingFoamPositions_;
            foamDirty_ = false;
            hasNewFoamData = true;
        }
    }

    if (hasNewData) {
        depthPass_.setParticles(*ctx_, *pool_, uploadData);
        thicknessPass_.setParticles(*ctx_, *pool_, uploadData);
    }
    if (hasNewSprayData) {
        sprayPass_.setParticles(*ctx_, *pool_, sprayUploadData);
    }
    if (hasNewFoamData) {
        foamPass_.setParticles(*ctx_, *pool_, foamUploadData);
    }
}

void SSFluidRenderer::onPreRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
    if (!enabled_) {
        return;
    }

    depthPass_.render(cmd, frameIndex, targets_, proj_, view_);
    thicknessPass_.render(cmd, frameIndex, targets_, proj_, view_);

    bilateralPass_.setParams(bilateralSigmaS_, bilateralSigmaR_,
                             bilateralUseAnisotropic_,
                             bilateralAnisotropy_,
                             bilateralGradientScale_);
    bilateralPass_.render(*ctx_, cmd, frameIndex, targets_);

    bilateralDepthPass_.setParams(bilateralDepthSigmaS_, bilateralDepthSigmaR_,
                                  bilateralUseAnisotropic_,
                                  bilateralAnisotropy_,
                                  bilateralGradientScale_);
    bilateralDepthPass_.render(*ctx_, cmd, frameIndex,
                               targets_.depth().getColorImageView(),
                               targets_.getSampler(),
                               targets_.smoothedDepth());

    VkImageView envView    = hasEnvMap_ ? envMap_.getImageView()   : dummyCubeMap_.getImageView();
    VkSampler   envSampler = hasEnvMap_ ? envMap_.getSampler()     : dummyCubeMap_.getSampler();

    VkImageView depthForNormals = useDepthSmoothing_
        ? targets_.smoothedDepth().getColorImageView()
        : targets_.depth().getColorImageView();

    reflectionPass_.render(*ctx_, cmd, frameIndex, targets_,
                           depthForNormals,
                           glm::inverse(proj_),
                           glm::mat4(glm::transpose(glm::mat3(view_))),
                           envView, envSampler, hasEnvMap_);

    refractionPass_.render(*ctx_, cmd, frameIndex, targets_);
    sprayPass_.render(cmd, frameIndex, targets_, proj_, view_, 5.0f, 0.5f);
    foamPass_.render(cmd, frameIndex, targets_, proj_, view_, 9.0f, 0.35f);
}

void SSFluidRenderer::onRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
    if (!enabled_) {
        return;
    }

    // Skybox: render before composite pass, displayed in areas without fluid.
    if (hasEnvMap_ && skyBoxRenderer_) {
        Phantom::VKG::VkSkyBoxRenderer::Buffer buf;
        buf.projectionMatrix = proj_;
        buf.viewMatrix       = glm::mat4(glm::mat3(view_));  // Strip translation
        skyBoxRenderer_->upload(buf, frameIndex);
        skyBoxRenderer_->render(cmd, frameIndex);
    }

    CompositeUBO ubo{};
    ubo.mode = static_cast<int>(mode_);
    ubo.foamOpacity = foamOpacity_;
    ubo.sprayOpacity = sprayOpacity_;
    ubo.showSpray = showSpray_ ? 1 : 0;
    ubo.showFoam = showFoam_ ? 1 : 0;
    compositePipeline_.updateUBO(frameIndex, &ubo, sizeof(ubo));

    const VkSampler sampler = targets_.getSampler();

    VkDescriptorImageInfo depthInfo{};
    depthInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    depthInfo.imageView   = (mode_ == Mode::SmoothedDepth)
        ? targets_.smoothedDepth().getColorImageView()
        : targets_.depth().getColorImageView();
    depthInfo.sampler     = sampler;

    VkDescriptorImageInfo thickInfo{};
    thickInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    thickInfo.imageView   = targets_.thickness().getColorImageView();
    thickInfo.sampler     = sampler;

    VkDescriptorImageInfo smoothInfo{};
    smoothInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    smoothInfo.imageView   = targets_.smoothed().getColorImageView();
    smoothInfo.sampler     = sampler;

    VkDescriptorImageInfo reflInfo{};
    reflInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    reflInfo.imageView   = targets_.reflection().getColorImageView();
    reflInfo.sampler     = sampler;

    VkDescriptorImageInfo refrInfo{};
    refrInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    refrInfo.imageView   = targets_.refraction().getColorImageView();
    refrInfo.sampler     = sampler;

    VkDescriptorImageInfo sprayInfo{};
    sprayInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    sprayInfo.imageView   = targets_.spray().getColorImageView();
    sprayInfo.sampler     = sampler;

    VkDescriptorImageInfo foamInfo{};
    foamInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    foamInfo.imageView   = targets_.foam().getColorImageView();
    foamInfo.sampler     = sampler;

    VkWriteDescriptorSet writes[7]{};

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
    writes[2].dstBinding      = 2;
    writes[2].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[2].descriptorCount = 1;
    writes[2].pImageInfo      = &smoothInfo;

    writes[3].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[3].dstBinding      = 3;
    writes[3].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[3].descriptorCount = 1;
    writes[3].pImageInfo      = &reflInfo;

    writes[4].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[4].dstBinding      = 4;
    writes[4].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[4].descriptorCount = 1;
    writes[4].pImageInfo      = &refrInfo;

    writes[5].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[5].dstBinding      = 5;
    writes[5].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[5].descriptorCount = 1;
    writes[5].pImageInfo      = &sprayInfo;

    writes[6].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[6].dstBinding      = 6;
    writes[6].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[6].descriptorCount = 1;
    writes[6].pImageInfo      = &foamInfo;

    compositePipeline_.writeDescriptors(ctx_->getDevice(), frameIndex,
                                        { writes[0], writes[1], writes[2], writes[3], writes[4], writes[5], writes[6] });

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, compositePipeline_.getPipeline());
    VkDescriptorSet ds = compositePipeline_.getDescriptorSet(frameIndex);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            compositePipeline_.getLayout(), 0, 1, &ds, 0, nullptr);

    vkCmdDraw(cmd, 3, 1, 0, 0);
}

void SSFluidRenderer::onCleanup(VkDevice device)
{
    destroyPassResources(device);
    ctx_ = nullptr;
    pool_ = nullptr;
}

} // namespace VKSSFR
