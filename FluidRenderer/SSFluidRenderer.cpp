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
    const std::array<VkImage, 9> images = {
        targets.depth().getColorImage(),
        targets.thickness().getColorImage(),
        targets.smoothed().getColorImage(),
        targets.smoothedDepth().getColorImage(),
        targets.filterTemp().getColorImage(),
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

void SSFluidRenderer::setParticleEllipsoids(std::vector<glm::vec4> centers,
                                            std::vector<SSFREllipsoidAxes> axes)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (axes.size() != centers.size()) axes.resize(centers.size());
    ellipsoidCenters_ = std::move(centers);
    ellipsoidAxes_ = std::move(axes);
    ++ellipsoidGeneration_;
}

void SSFluidRenderer::clearParticleEllipsoids()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (ellipsoidCenters_.empty()) return;
    ellipsoidCenters_.clear();
    ellipsoidAxes_.clear();
    ++ellipsoidGeneration_;
}

void SSFluidRenderer::setEllipsoidBuffers(VkBuffer centers, VkBuffer axes, uint32_t count)
{
    extEllipsoidCenters_ = centers;
    extEllipsoidAxes_ = axes;
    extEllipsoidCount_ = count;
    useExternalEllipsoids_ = true;
}

void SSFluidRenderer::clearEllipsoidBuffers()
{
    extEllipsoidCenters_ = VK_NULL_HANDLE;
    extEllipsoidAxes_ = VK_NULL_HANDLE;
    extEllipsoidCount_ = 0;
    useExternalEllipsoids_ = false;
}

void SSFluidRenderer::uploadEllipsoids(uint32_t frameIndex)
{
    if (!ctx_ || frameIndex >= ellipsoidSlots_.size()) return;
    EllipsoidSlot& slot = *ellipsoidSlots_[frameIndex];

    // onUpdate(frameIndex) runs after this frame slot's fence, so its buffers
    // are no longer read by the GPU and may be rewritten or reallocated.
    std::lock_guard<std::mutex> lock(mutex_);
    if (slot.generation == ellipsoidGeneration_) return;
    const size_t n = ellipsoidCenters_.size();
    if (n > slot.capacity) {
        const size_t capacity = n + n / 2;
        slot.centers.destroy(ctx_->getDevice());
        slot.axes.destroy(ctx_->getDevice());
        slot.capacity = 0;
        slot.count = 0;
        if (!slot.centers.createMapped(*ctx_, sizeof(glm::vec4) * capacity, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT) ||
            !slot.axes.createMapped(*ctx_, sizeof(SSFREllipsoidAxes) * capacity, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT)) {
            std::cerr << "[VKSSFR] Failed to allocate ellipsoid buffers (" << n << " particles)" << std::endl;
            slot.centers.destroy(ctx_->getDevice());
            slot.axes.destroy(ctx_->getDevice());
            return;
        }
        slot.capacity = capacity;
    }
    if (n > 0) {
        slot.centers.write(ellipsoidCenters_.data(), sizeof(glm::vec4) * n);
        slot.axes.write(ellipsoidAxes_.data(), sizeof(SSFREllipsoidAxes) * n);
    }
    slot.count = static_cast<uint32_t>(n);
    slot.generation = ellipsoidGeneration_;
}

void SSFluidRenderer::destroyEllipsoidSlots(VkDevice device)
{
    for (auto& slot : ellipsoidSlots_) {
        slot->centers.destroy(device);
        slot->axes.destroy(device);
    }
    ellipsoidSlots_.clear();
}

void SSFluidRenderer::setCamera(const glm::mat4& proj, const glm::mat4& view)
{
    proj_ = proj;
    view_ = view;
}

void SSFluidRenderer::setSceneInput(VkImageView color, VkImageView depth, VkSampler sampler,
                                    float nearPlane, float farPlane)
{
    sceneColor_ = color;
    sceneDepth_ = depth;
    sceneSampler_ = sampler;
    nearPlane_ = nearPlane;
    farPlane_ = farPlane;
}

void SSFluidRenderer::clearSceneInput()
{
    sceneColor_ = VK_NULL_HANDLE;
    sceneDepth_ = VK_NULL_HANDLE;
    sceneSampler_ = VK_NULL_HANDLE;
}

void SSFluidRenderer::setFluidMaterial(const glm::vec3& absorptionColor, float absorptionDistance,
                                       float ior, float roughness, float thicknessScale)
{
    absorptionColor_ = glm::clamp(absorptionColor, glm::vec3(0.0f), glm::vec3(1.0f));
    absorptionDistance_ = glm::max(absorptionDistance, 0.001f);
    ior_ = glm::clamp(ior, 1.0f, 2.5f);
    roughness_ = glm::clamp(roughness, 0.0f, 1.0f);
    thicknessScale_ = glm::max(thicknessScale, 0.0f);
}

void SSFluidRenderer::onInit(GlobalVulkanContext& ctx,
                             const GlobalVulkanCommandPool& pool,
                             VkRenderPass renderPass,
                             uint32_t framesInFlight)
{
    ctx_ = &ctx;
    pool_ = &pool;
    framesInFlight_ = framesInFlight;
    mainRenderPass_ = renderPass;

    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(ctx.getPhysicalDevice(), &props);
    maxPointSize_ = std::max(1.0f, props.limits.pointSizeRange[1]);

    ellipsoidSlots_.clear();
    for (uint32_t i = 0; i < framesInFlight; ++i)
        ellipsoidSlots_.push_back(std::make_unique<EllipsoidSlot>());

    if (!createPassResources(renderPass)) {
        destroyPassResources(ctx.getDevice());
        ctx_ = nullptr;
        pool_ = nullptr;
        framesInFlight_ = 0;
        enabled_ = false;
        std::cerr << "[VKSSFR] Disabled: failed to create pass resources" << std::endl;
    }
}

void SSFluidRenderer::resize(uint32_t width, uint32_t height)
{
    width = width ? width : 1;
    height = height ? height : 1;
    if (extent_.width == width && extent_.height == height) return;
    if (!ctx_) { extent_ = { width, height }; return; }

    vkDeviceWaitIdle(ctx_->getDevice());
    destroyPassResources(ctx_->getDevice());
    extent_ = { width, height };
    if (!createPassResources(mainRenderPass_)) {
        destroyPassResources(ctx_->getDevice());
        enabled_ = false;
        std::cerr << "[VKSSFR] resize: failed to recreate pass resources; disabled" << std::endl;
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
                      shaders_.depthVert, shaders_.depthFrag);
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
    // Anisotropic kernel pipelines are optional: a missing shader or a failed
    // pipeline only disables the ellipsoid splats, never SSFR itself.
    if (!shaders_.anisoVert.empty() && !shaders_.depthAnisoFrag.empty() &&
        !shaders_.thicknessAnisoFrag.empty()) {
        const bool ok =
            depthPass_.createEllipsoid(*ctx_, framesInFlight_, targets_.depth().getRenderPass(),
                                       shaders_.anisoVert, shaders_.depthAnisoFrag) &&
            thicknessPass_.createEllipsoid(*ctx_, framesInFlight_, targets_.thickness().getRenderPass(),
                                           shaders_.anisoVert, shaders_.thicknessAnisoFrag);
        if (!ok)
            std::cerr << "[VKSSFR] Failed to create anisotropic kernel pipelines; ellipsoid splats disabled" << std::endl;
    }

    sprayPass_.create(*ctx_, framesInFlight_, targets_.spray().getRenderPass(),
                      shaders_.thicknessVert, shaders_.thicknessFrag);
    if (!sprayPass_.isValid()) {
        std::cerr << "[VKSSFR] Failed to create spray pass" << std::endl;
        return false;
    }
    foamPass_.create(*ctx_, framesInFlight_, targets_.foam().getRenderPass(),
                     shaders_.thicknessVert, shaders_.thicknessFrag);
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
                               shaders_.bilateralVert, shaders_.bilateralFrag);
    if (!bilateralDepthPass_.isValid()) {
        std::cerr << "[VKSSFR] Failed to create bilateral depth pass" << std::endl;
        return false;
    }

    reflectionPass_.create(*ctx_, *pool_, framesInFlight_, targets_.reflection().getRenderPass(),
                           shaders_.reflectionVert, shaders_.reflectionFrag);
    if (!reflectionPass_.isValid()) {
        std::cerr << "[VKSSFR] Failed to create reflection pass" << std::endl;
        return false;
    }
    refractionPass_.create(*ctx_, framesInFlight_, targets_.refraction().getRenderPass(),
                           shaders_.refractionVert, shaders_.refractionFrag);
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
        sbCfg.vertSpv = shaders_.skyboxVert;
        sbCfg.fragSpv = shaders_.skyboxFrag;
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

    VkDescriptorSetLayoutBinding sceneColorBinding{};
    sceneColorBinding.binding = 8;
    sceneColorBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    sceneColorBinding.descriptorCount = 1;
    sceneColorBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    VkDescriptorSetLayoutBinding sceneDepthBinding = sceneColorBinding;
    sceneDepthBinding.binding = 9;

    SSFRPassConfig cfg;
    cfg.vertSpv            = shaders_.compositeVert;
    cfg.fragSpv            = shaders_.compositeFrag;
    cfg.topology           = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    cfg.depthTest          = linearOutput_;
    cfg.depthWrite         = linearOutput_;
    cfg.depthCompareOp     = VK_COMPARE_OP_ALWAYS;
    cfg.additiveBlend      = false;
    cfg.framesInFlight     = framesInFlight_;
    cfg.uboSize            = sizeof(CompositeUBO);
    cfg.descriptorBindings = {
        depthBinding, thickBinding, smoothBinding,
        reflBinding, refrBinding, sprayBinding, foamBinding, uboBinding,
        sceneColorBinding, sceneDepthBinding
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

    Phantom::VKG::VulkanCubeMap replacement;
    if (!replacement.create(*ctx_, *pool_, facePaths)) {
        std::cerr << "[VKSSFR] loadEnvMap failed" << std::endl;
        return;
    }
    if (skyBoxRenderer_) {
        skyBoxRenderer_->setCubeMap(ctx_->getDevice(),
                                    replacement.getImageView(),
                                    replacement.getSampler());
    }
    envMap_.swap(replacement);
    if (replacement.isValid()) replacement.destroy(ctx_->getDevice());
    hasEnvMap_ = true;
}

void SSFluidRenderer::onUpdate(uint32_t frameIndex)
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
    uploadEllipsoids(frameIndex);
}

void SSFluidRenderer::onPreRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
    if (!isValid() || !enabled_) {
        return;
    }

    const auto gpuMark = [this, cmd](const char* label) {
        if (gpuProfiler_) gpuProfiler_->gpuMark(cmd, label);
    };

    // The reconstruction kernel must overlap neighbouring particles at their
    // rest spacing (normally 2*r).  1.35*r left a scalloped silhouette and
    // visible vertical particle columns at close camera distances.
    constexpr float kSurfaceRadiusScale = 1.5f;
    const float surfaceRadius = particleRadiiInBuffer_ ? -kSurfaceRadiusScale
                                                       : particleRadius_ * kSurfaceRadiusScale;
    const float viewportHeight = static_cast<float>(extent_.height);

    // Anisotropic kernel: the ellipsoid centres carry their own world radius
    // (w), scaled by the same surface factor as the sphere sprites.
    SSFREllipsoidDraw ellipsoids;
    if (anisotropicKernel_ && supportsAnisotropicKernel()) {
        if (useExternalEllipsoids_) {
            ellipsoids.centers = extEllipsoidCenters_;
            ellipsoids.axes = extEllipsoidAxes_;
            ellipsoids.count = extEllipsoidCount_;
        } else if (frameIndex < ellipsoidSlots_.size()) {
            const EllipsoidSlot& slot = *ellipsoidSlots_[frameIndex];
            ellipsoids.centers = slot.centers.get();
            ellipsoids.axes = slot.axes.get();
            ellipsoids.count = slot.count;
        }
        ellipsoids.radiusScale = kSurfaceRadiusScale;
        ellipsoids.maxPointSize = maxPointSize_;
        // The ray cast maps gl_FragCoord back to NDC, so it needs the size the
        // targets were actually created at -- extent_ can run ahead of them
        // (setExtent() without a resize() that rebuilds, e.g. FluidStudio).
        ellipsoids.extent = targets_.depth().getExtent();
    }
    anisotropicKernelActive_ = ellipsoids.valid();

    if (anisotropicKernelActive_) {
        depthPass_.renderEllipsoids(cmd, frameIndex, targets_, proj_, view_, ellipsoids);
    } else {
        depthPass_.render(cmd, frameIndex, targets_, proj_, view_,
                          surfaceRadius, viewportHeight);
    }
    gpuMark("ssfr.depth");
    if (anisotropicKernelActive_) {
        thicknessPass_.renderEllipsoids(cmd, frameIndex, targets_.thickness(), proj_, view_, ellipsoids);
    } else {
        thicknessPass_.render(cmd, frameIndex, targets_, proj_, view_,
                              surfaceRadius, viewportHeight);
    }
    gpuMark("ssfr.thickness");

    // A single 5x5 pass truncated sigmaS=2.5 to less than one standard
    // deviation and left the projected particle lattice in the normals.  Two
    // separable passes cover 3*sigma while remaining much cheaper than a wide
    // 2-D kernel.
    bilateralPass_.setParams(bilateralSigmaS_, bilateralSigmaR_,
                             bilateralUseAnisotropic_,
                             bilateralAnisotropy_,
                             bilateralGradientScale_);
    bilateralPass_.setPassAxis(1);
    bilateralPass_.render(*ctx_, cmd, frameIndex,
                          targets_.thickness().getColorImageView(),
                          targets_.getSampler(), targets_.filterTemp());
    bilateralPass_.setPassAxis(2);
    bilateralPass_.render(*ctx_, cmd, frameIndex,
                          targets_.filterTemp().getColorImageView(),
                          targets_.getSampler(), targets_.smoothed());

    // The depth field is the reconstructed liquid surface.  Applying the
    // thickness-oriented anisotropic edge preservation here mistakes the
    // periodic ridges of sphere splats for real surface edges and locks the
    // particle columns into the reflected normal field.  The range term still
    // preserves true depth discontinuities, so use isotropic spatial smoothing
    // for depth while retaining anisotropy for thickness above.
    bilateralDepthPass_.setParams(bilateralDepthSigmaS_, bilateralDepthSigmaR_,
                                  false, bilateralAnisotropy_,
                                  bilateralGradientScale_);
    bilateralDepthPass_.setPassAxis(1);
    bilateralDepthPass_.render(*ctx_, cmd, frameIndex,
                               targets_.depth().getColorImageView(),
                               targets_.getSampler(),
                               targets_.filterTemp());
    bilateralDepthPass_.setPassAxis(2);
    bilateralDepthPass_.render(*ctx_, cmd, frameIndex,
                               targets_.filterTemp().getColorImageView(),
                               targets_.getSampler(),
                               targets_.smoothedDepth());
    // A second separable iteration removes the residual frequency at the
    // projected particle spacing.  It reuses the same ping-pong target and is
    // applied only to depth, where small ripples become large normal errors.
    bilateralDepthPass_.setPassAxis(1);
    bilateralDepthPass_.render(*ctx_, cmd, frameIndex,
                               targets_.smoothedDepth().getColorImageView(),
                               targets_.getSampler(),
                               targets_.filterTemp());
    bilateralDepthPass_.setPassAxis(2);
    bilateralDepthPass_.render(*ctx_, cmd, frameIndex,
                               targets_.filterTemp().getColorImageView(),
                               targets_.getSampler(),
                               targets_.smoothedDepth());
    gpuMark("ssfr.bilateral");

    const bool externalEnv = externalEnvView_ && externalEnvSampler_;
    const bool useEnv = externalEnv || hasEnvMap_;
    VkImageView envView    = externalEnv ? externalEnvView_ : (hasEnvMap_ ? envMap_.getImageView() : dummyCubeMap_.getImageView());
    VkSampler   envSampler = externalEnv ? externalEnvSampler_ : (hasEnvMap_ ? envMap_.getSampler() : dummyCubeMap_.getSampler());

    VkImageView depthForNormals = useDepthSmoothing_
        ? targets_.smoothedDepth().getColorImageView()
        : targets_.depth().getColorImageView();

    reflectionPass_.render(*ctx_, cmd, frameIndex, targets_,
                           depthForNormals,
                           glm::inverse(proj_),
                           glm::mat4(glm::transpose(glm::mat3(view_))),
                           envView, envSampler, useEnv,
                           lightDirection_, lightColor_, lightIntensity_, roughness_);
    gpuMark("ssfr.reflection");

    const bool hasScene = sceneColor_ != VK_NULL_HANDLE && sceneDepth_ != VK_NULL_HANDLE &&
                          sceneSampler_ != VK_NULL_HANDLE;
    refractionPass_.render(*ctx_, cmd, frameIndex, targets_, depthForNormals,
                           sceneColor_, sceneDepth_, sceneSampler_, envView, envSampler,
                           glm::inverse(proj_), glm::mat4(glm::transpose(glm::mat3(view_))),
                           extent_, nearPlane_, farPlane_, hasScene, useEnv, ior_, absorptionColor_);
    gpuMark("ssfr.refraction");
    sprayPass_.render(cmd, frameIndex, targets_.spray(), proj_, view_,
                      particleRadius_ * 0.55f, viewportHeight, 0.5f);
    foamPass_.render(cmd, frameIndex, targets_.foam(), proj_, view_,
                     particleRadius_ * 0.9f, viewportHeight, 0.35f);
    gpuMark("ssfr.spray_foam");
}

void SSFluidRenderer::onRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
    if (!isValid()) return;
    const bool hasScene = sceneColor_ != VK_NULL_HANDLE && sceneDepth_ != VK_NULL_HANDLE &&
                          sceneSampler_ != VK_NULL_HANDLE;
    if (!enabled_ && !hasScene) {
        return;
    }

    // Skybox: render before composite pass, displayed in areas without fluid.
    if (!hasScene && hasEnvMap_ && skyBoxRenderer_) {
        Phantom::VKG::VkSkyBoxRenderer::Buffer buf;
        buf.projectionMatrix = proj_;
        buf.viewMatrix       = glm::mat4(glm::mat3(view_));  // Strip translation
        skyBoxRenderer_->upload(buf, frameIndex);
        skyBoxRenderer_->render(cmd, frameIndex);
    }

    CompositeUBO ubo{};
    ubo.mode = enabled_ ? static_cast<int>(mode_) : -1;
    ubo.foamOpacity = foamOpacity_;
    ubo.sprayOpacity = sprayOpacity_;
    ubo.showSpray = showSpray_ ? 1 : 0;
    ubo.showFoam = showFoam_ ? 1 : 0;
    ubo.hasScene = hasScene ? 1 : 0;
    ubo.exposure = exposure_;
    ubo.transparent = (transparentBackground_ ? 1 : 0) | (linearOutput_ ? 2 : 0);
    ubo.absorptionColor = glm::vec4(absorptionColor_, 1.0f);
    ubo.absorptionDistance = absorptionDistance_;
    ubo.thicknessScale = thicknessScale_;
    ubo.ior = ior_;
    ubo.roughness = roughness_;
    compositePipeline_.updateUBO(frameIndex, &ubo, sizeof(ubo));

    const VkSampler sampler = targets_.getSampler();

    VkDescriptorImageInfo depthInfo{};
    depthInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    depthInfo.imageView   = (useDepthSmoothing_ && mode_ != Mode::DepthOnly)
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

    VkDescriptorImageInfo sceneColorInfo{};
    sceneColorInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    sceneColorInfo.imageView = hasScene ? sceneColor_ : targets_.reflection().getColorImageView();
    sceneColorInfo.sampler = hasScene ? sceneSampler_ : sampler;

    VkDescriptorImageInfo sceneDepthInfo{};
    sceneDepthInfo.imageLayout = hasScene ? VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL
                                          : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    sceneDepthInfo.imageView = hasScene ? sceneDepth_ : targets_.depth().getColorImageView();
    sceneDepthInfo.sampler = hasScene ? sceneSampler_ : sampler;

    VkWriteDescriptorSet writes[9]{};

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

    writes[7].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[7].dstBinding = 8;
    writes[7].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[7].descriptorCount = 1;
    writes[7].pImageInfo = &sceneColorInfo;

    writes[8].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[8].dstBinding = 9;
    writes[8].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[8].descriptorCount = 1;
    writes[8].pImageInfo = &sceneDepthInfo;

    compositePipeline_.writeDescriptors(ctx_->getDevice(), frameIndex,
                                        { writes[0], writes[1], writes[2], writes[3], writes[4], writes[5], writes[6],
                                          writes[7], writes[8] });

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, compositePipeline_.getPipeline());
    VkDescriptorSet ds = compositePipeline_.getDescriptorSet(frameIndex);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            compositePipeline_.getLayout(), 0, 1, &ds, 0, nullptr);

    vkCmdDraw(cmd, 3, 1, 0, 0);
}

void SSFluidRenderer::onCleanup(VkDevice device)
{
    destroyPassResources(device);
    destroyEllipsoidSlots(device);
    ctx_ = nullptr;
    pool_ = nullptr;
}

} // namespace VKSSFR
