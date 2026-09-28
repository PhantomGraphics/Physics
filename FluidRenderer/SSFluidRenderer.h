#pragma once


#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"

#include "SSFROffscreenSet.h"
#include "ParticleDepthRenderer.h"
#include "SSThicknessRenderer.h"
#include "BilateralFilter.h"
#include "SSReflectionRenderer.h"
#include "SSRefractionRenderer.h"
#include "SSFRPipeline.h"

#include "../../CGLib/VulkanGraphics/VulkanCubeMap.h"
#include "../../CGLib/VulkanGraphics/IGpuProfiler.h"
#include "../../CGLib/Renderer/VkRenderer/VkSkyBoxRenderer.h"

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace Phantom::VKG { class VulkanContext; class VulkanCommandPool; }

using GlobalVkSubRenderer = ::VKG::IVkSubRenderer;
using GlobalVulkanContext = Phantom::VKG::VulkanContext;
using GlobalVulkanCommandPool = Phantom::VKG::VulkanCommandPool;
using GlobalVulkanCubeMap = Phantom::VKG::VulkanCubeMap;

namespace Phantom {

class SSFluidRenderer : public GlobalVkSubRenderer {
public:
    struct Shaders {
        std::vector<uint32_t> depthVert, depthFrag;
        std::vector<uint32_t> thicknessVert, thicknessFrag;
        std::vector<uint32_t> bilateralVert, bilateralFrag;
        std::vector<uint32_t> reflectionVert, reflectionFrag;
        std::vector<uint32_t> refractionVert, refractionFrag;
        std::vector<uint32_t> compositeVert, compositeFrag;
        std::vector<uint32_t> skyboxVert, skyboxFrag;
        // Optional: anisotropic-kernel ellipsoid splats. Empty => the kernel
        // is unavailable and the sphere-sprite passes are always used.
        std::vector<uint32_t> anisoVert, depthAnisoFrag, thicknessAnisoFrag;
    };

    void setShaders(Shaders shaders) { shaders_ = std::move(shaders); }

    enum class Mode {
        DepthOnly = 0,
        ThicknessRaw = 1,
        ThicknessBilateral = 2,
        Reflection = 3,
        Refraction = 4,
        SSFRMain = 5,
        SmoothedDepth = 6,
    };

    struct CompositeUBO {
        int mode = static_cast<int>(Mode::SSFRMain);
        float foamOpacity = 0.8f;
        float sprayOpacity = 0.6f;
        int showSpray = 1;
        int showFoam = 1;
        int hasScene = 0;
        float exposure = 1.0f;
        int transparent = 0; // 1 => emit an RGBA coverage matte (FluidStudio Phase 6b)
        glm::vec4 absorptionColor = glm::vec4(0.12f, 0.62f, 0.72f, 1.0f);
        float absorptionDistance = 6.0f;
        float thicknessScale = 1.0f;
        float ior = 1.333f;
        float roughness = 0.08f;
    };
    static_assert(sizeof(CompositeUBO) == 64, "UBO layout mismatch");

    void setParticles(const std::vector<glm::vec3>& positions);
    void setSprayParticles(const std::vector<glm::vec3>& positions);
    void setFoamParticles(const std::vector<glm::vec3>& positions);

    /// For GPU_CSPH mode: pass a GPU buffer directly to the depth/thickness passes.
    /// buf must have VK_BUFFER_USAGE_VERTEX_BUFFER_BIT set.
    void setParticleBuffer(VkBuffer buf, uint32_t count);

    /// Disable GPU buffer mode and revert to CPU upload mode.
    void clearParticleBuffer();
    void setExtent(VkExtent2D ext) { extent_ = ext; }

    /// Recreates the offscreen targets + sub-passes at a new resolution. Unlike
    /// setExtent() (which only records the size for the next projection), this
    /// actually rebuilds targets_ so screen-space passes render at `width` x
    /// `height`. Call after onInit; no-op if unchanged or not yet initialized.
    /// Used by FluidStudio's offscreen sequence renderer (independent output
    /// resolution) and safe to call on a normal window resize.
    void resize(uint32_t width, uint32_t height);
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }
    bool isValid() const { return compositePipeline_.isValid(); }
    // External vec4 buffers may carry a world-space particle radius in w.
    // Default remains false for GPU_CSPH's position/density layout.
    void setParticleRadiiInBuffer(bool v) { particleRadiiInBuffer_ = v; }
    // Host supplies an already exposed HDR scene and performs the final display transform.
    void setLinearOutput(bool v) { linearOutput_ = v; }
    // Borrowed environment: caller keeps it alive until all submitted draws complete.
    void setEnvironment(VkImageView view, VkSampler sampler) {
        externalEnvView_ = view; externalEnvSampler_ = sampler;
    }

    void setMode(Mode mode) { mode_ = mode; }
    Mode getMode() const { return mode_; }

    void setShowSpray(bool v) { showSpray_ = v; }
    void setShowFoam(bool v) { showFoam_ = v; }
    bool getShowSpray() const { return showSpray_; }
    bool getShowFoam() const { return showFoam_; }

    void setAnisotropicSmoothing(bool v) { bilateralUseAnisotropic_ = v; }
    bool getAnisotropicSmoothing() const { return bilateralUseAnisotropic_; }

    void setAnisotropy(float v) { bilateralAnisotropy_ = v; }
    float getAnisotropy() const { return bilateralAnisotropy_; }

    void setAnisotropicGradientScale(float v) { bilateralGradientScale_ = v; }
    float getAnisotropicGradientScale() const { return bilateralGradientScale_; }

    // Anisotropic kernel (Yu & Turk 2013 ellipsoid splats,
    // docs/todo/PLAN_ssfr_anisotropic_kernel.md). Unrelated to the bilateral
    // "anisotropic smoothing" above: this changes the particle shape itself.
    // The caller computes the ellipsoids (Phantom::Physics::computeAnisotropy)
    // and hands them over each time the positions change; while enabled and
    // ellipsoid data is present, the depth + thickness passes ray cast the
    // ellipsoids instead of drawing sphere sprites. Default off.
    void setAnisotropicKernel(bool v) { anisotropicKernel_ = v; }
    bool getAnisotropicKernel() const { return anisotropicKernel_; }
    /// True when the ellipsoid pipelines were created (shaders supplied).
    bool supportsAnisotropicKernel() const {
        return depthPass_.hasEllipsoidPipeline() && thicknessPass_.hasEllipsoidPipeline();
    }
    /// True when the last onPreRender() actually drew ellipsoids.
    bool isAnisotropicKernelActive() const { return anisotropicKernelActive_; }
    /// CPU path: centers[i] = (smoothed centre, world radius), axes[i] = the
    /// columns of T = R diag(sigma). Uploaded per frame-in-flight in onUpdate().
    void setParticleEllipsoids(std::vector<glm::vec4> centers, std::vector<SSFREllipsoidAxes> axes);
    void clearParticleEllipsoids();
    /// GPU path: caller-owned vertex buffers in the same layout. Takes
    /// precedence over the CPU path until clearEllipsoidBuffers().
    void setEllipsoidBuffers(VkBuffer centers, VkBuffer axes, uint32_t count);
    void clearEllipsoidBuffers();

    void setThicknessSmoothingSigmaS(float v) { bilateralSigmaS_ = v; }
    float getThicknessSmoothingSigmaS() const { return bilateralSigmaS_; }
    void setThicknessSmoothingSigmaR(float v) { bilateralSigmaR_ = v; }
    float getThicknessSmoothingSigmaR() const { return bilateralSigmaR_; }

    void setDepthSmoothing(bool v) { useDepthSmoothing_ = v; }
    bool getDepthSmoothing() const { return useDepthSmoothing_; }
    void setDepthSmoothingSigmaS(float v) { bilateralDepthSigmaS_ = v; }
    float getDepthSmoothingSigmaS() const { return bilateralDepthSigmaS_; }
    void setDepthSmoothingSigmaR(float v) { bilateralDepthSigmaR_ = v; }
    float getDepthSmoothingSigmaR() const { return bilateralDepthSigmaR_; }

    void setSprayOpacity(float v) { sprayOpacity_ = v; }
    float getSprayOpacity() const { return sprayOpacity_; }
    void setFoamOpacity(float v) { foamOpacity_ = v; }
    float getFoamOpacity() const { return foamOpacity_; }

    // Optional GPU-timing hook. When set, onPreRender() calls gpuMark() at
    // each sub-pass boundary (depth / thickness / bilateral / reflection /
    // refraction / spray / foam). Null by default -- no behaviour change.
    // See docs/todo/PLAN_fluidstudio_fast_rendering.md Phase 0.
    void setGpuProfiler(Phantom::VKG::IGpuProfiler* p) { gpuProfiler_ = p; }

    void setCamera(const glm::mat4& proj, const glm::mat4& view);
    void setSceneInput(VkImageView color, VkImageView depth, VkSampler sampler,
                       float nearPlane = 0.1f, float farPlane = 1000.0f);
    void clearSceneInput();
    void setFluidMaterial(const glm::vec3& absorptionColor, float absorptionDistance,
                          float ior, float roughness, float thicknessScale);
    void setLight(const glm::vec3& direction, const glm::vec3& color, float intensity)
    {
        lightDirection_ = direction;
        lightColor_ = color;
        lightIntensity_ = glm::max(intensity, 0.0f);
    }
    void setExposure(float exposure) { exposure_ = glm::max(exposure, 0.001f); }
    /// When true, the composite emits alpha = coverage (transparent background)
    /// instead of a solid alpha of 1. Used by FluidStudio's offscreen sequence
    /// renderer for RGBA PNG output; the interactive path leaves it false.
    void setTransparentBackground(bool v) { transparentBackground_ = v; }
    bool getTransparentBackground() const { return transparentBackground_; }
    void setParticleRadius(float radius) { particleRadius_ = glm::max(radius, 0.001f); }
    float getParticleRadius() const { return particleRadius_; }

    /// @brief Load environment map. Call after onInit.
    /// @param facePaths {right, left, top, bottom, front, back}
    void loadEnvMap(const std::array<std::string, 6>& facePaths);

    void onPreRender(VkCommandBuffer cmd, uint32_t frameIndex);

    void onInit(GlobalVulkanContext& ctx, const GlobalVulkanCommandPool& pool,
                VkRenderPass renderPass, uint32_t framesInFlight) override;
    void onUpdate(uint32_t frameIndex) override;
    void onRender(VkCommandBuffer cmd, uint32_t frameIndex) override;
    void onCleanup(VkDevice device) override;

private:
    Shaders shaders_;
    const GlobalVulkanContext* ctx_ = nullptr;
    const GlobalVulkanCommandPool* pool_ = nullptr;

    uint32_t framesInFlight_ = 0;
    VkExtent2D extent_ = { 1280, 720 };
    VkRenderPass mainRenderPass_ = VK_NULL_HANDLE; // for resize()'s recreate

    bool enabled_ = false;
    bool particleRadiiInBuffer_ = false;
    bool linearOutput_ = false;
    VkImageView externalEnvView_ = VK_NULL_HANDLE;
    VkSampler externalEnvSampler_ = VK_NULL_HANDLE;
    Mode mode_ = Mode::SSFRMain;

    glm::mat4 proj_ = glm::mat4(1.0f);
    glm::mat4 view_ = glm::mat4(1.0f);

    Phantom::VKG::IGpuProfiler* gpuProfiler_ = nullptr;

    std::vector<glm::vec3> pendingPositions_;
    std::vector<glm::vec3> pendingSprayPositions_;
    std::vector<glm::vec3> pendingFoamPositions_;
    // Anisotropic kernel: latest CPU ellipsoids (guarded by mutex_) and one
    // host-visible upload slot per frame in flight.
    struct EllipsoidSlot {
        Phantom::VKG::VulkanBuffer centers;
        Phantom::VKG::VulkanBuffer axes;
        size_t   capacity   = 0;
        uint32_t count      = 0;
        uint64_t generation = 0;
    };
    std::vector<glm::vec4>         ellipsoidCenters_;
    std::vector<SSFREllipsoidAxes> ellipsoidAxes_;
    uint64_t ellipsoidGeneration_ = 0;
    std::vector<std::unique_ptr<EllipsoidSlot>> ellipsoidSlots_;
    VkBuffer extEllipsoidCenters_ = VK_NULL_HANDLE;
    VkBuffer extEllipsoidAxes_    = VK_NULL_HANDLE;
    uint32_t extEllipsoidCount_   = 0;
    bool     useExternalEllipsoids_ = false;
    bool     anisotropicKernel_       = false;
    bool     anisotropicKernelActive_ = false;
    float    maxPointSize_ = 64.0f;
    void uploadEllipsoids(uint32_t frameIndex);
    void destroyEllipsoidSlots(VkDevice device);
    bool dirty_ = false;
    bool sprayDirty_ = false;
    bool foamDirty_ = false;
    std::mutex mutex_;

    SSFROffscreenSet       targets_;
    ParticleDepthRenderer  depthPass_;
    SSThicknessRenderer    thicknessPass_;
    SSThicknessRenderer    sprayPass_;
    SSThicknessRenderer    foamPass_;
    BilateralFilter        bilateralPass_;
    BilateralFilter        bilateralDepthPass_;
    SSReflectionRenderer   reflectionPass_;
    SSRefractionRenderer   refractionPass_;

    SSFRPipeline compositePipeline_;
    float bilateralSigmaS_ = 2.0f;
    float bilateralSigmaR_ = 0.08f;
    bool  bilateralUseAnisotropic_ = true;
    float bilateralAnisotropy_ = 1.25f;
    float bilateralGradientScale_ = 8.0f;
    bool  useDepthSmoothing_ = true;
    float bilateralDepthSigmaS_ = 1.5f;
    float bilateralDepthSigmaR_ = 0.05f;
    float sprayOpacity_ = 0.6f;
    float foamOpacity_ = 0.8f;
    bool  showSpray_ = true;
    bool  showFoam_ = true;
    float particleRadius_ = 1.0f;

    VkImageView sceneColor_ = VK_NULL_HANDLE;
    VkImageView sceneDepth_ = VK_NULL_HANDLE;
    VkSampler sceneSampler_ = VK_NULL_HANDLE;
    float nearPlane_ = 0.1f;
    float farPlane_ = 1000.0f;
    glm::vec3 absorptionColor_{0.12f, 0.62f, 0.72f};
    float absorptionDistance_ = 6.0f;
    float ior_ = 1.333f;
    float roughness_ = 0.08f;
    float thicknessScale_ = 1.0f;
    float exposure_ = 1.0f;
    bool  transparentBackground_ = false;
    glm::vec3 lightDirection_{0.35f, 0.75f, 0.25f};
    glm::vec3 lightColor_{1.0f};
    float lightIntensity_ = 1.0f;

    // Environment map / skybox
    GlobalVulkanCubeMap                                envMap_;
    GlobalVulkanCubeMap                                dummyCubeMap_;
    std::unique_ptr<Phantom::VKG::VkSkyBoxRenderer>   skyBoxRenderer_;
    bool hasEnvMap_ = false;

    // Returns false (and logs to stderr via the caller) on Vulkan resource
    // creation failure.
    bool createPassResources(VkRenderPass mainRenderPass);
    void destroyPassResources(VkDevice device);
};

} // namespace VKSSFR
