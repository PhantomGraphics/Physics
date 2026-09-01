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
        float _pad[3]{};
    };
    static_assert(sizeof(CompositeUBO) == 32, "UBO layout mismatch");

    void setParticles(const std::vector<glm::vec3>& positions);
    void setSprayParticles(const std::vector<glm::vec3>& positions);
    void setFoamParticles(const std::vector<glm::vec3>& positions);

    /// For GPU_CSPH mode: pass a GPU buffer directly to the depth/thickness passes.
    /// buf must have VK_BUFFER_USAGE_VERTEX_BUFFER_BIT set.
    void setParticleBuffer(VkBuffer buf, uint32_t count);

    /// Disable GPU buffer mode and revert to CPU upload mode.
    void clearParticleBuffer();
    void setExtent(VkExtent2D ext) { extent_ = ext; }
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }

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

    void setCamera(const glm::mat4& proj, const glm::mat4& view);
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

    bool enabled_ = false;
    Mode mode_ = Mode::SSFRMain;

    glm::mat4 proj_ = glm::mat4(1.0f);
    glm::mat4 view_ = glm::mat4(1.0f);

    std::vector<glm::vec3> pendingPositions_;
    std::vector<glm::vec3> pendingSprayPositions_;
    std::vector<glm::vec3> pendingFoamPositions_;
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
