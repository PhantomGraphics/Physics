#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "../../CGLib/GltfRenderer/Gltf/GltfDocument.h"
#include "../../CGLib/GltfRenderer/Renderer/GltfSceneRenderer.h"
#include "../../CGLib/VkAppBase/IVkSubRenderer.h"

#include "BodyRenderMode.h"

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

namespace Phantom {

class SoftBodyWorld;
namespace Physics { struct ISoftBody; }

/**
 * @brief PBR ("shaded") rendering of the soft-body scene, an alternative to
 * SoftBodyWireRenderer's wireframe
 * (docs/todo/PLAN_physicsview_gltf_rendering.md Phase 3). Sibling of
 * GltfBodyRenderer.
 *
 * One Phantom::Gltf::GltfSceneRenderer + GltfDocument per ISoftBody that has a
 * triangle surface (SoftMesh::faces) -- Rope has none and stays wireframe-only.
 * The document is one shared-vertex, double-wound primitive
 * (SoftMeshGltf.h). syncFromWorld() reconciles the instance map on preset
 * switches; onUpdate() re-streams every particle position + a CPU-recomputed
 * smooth normal each frame via GltfSceneRenderer::updateMorphedGeometry() (the
 * primitive is marked setDynamic(true) so the CPU vertex mirror is kept).
 */
class GltfSoftRenderer : public ::VKG::IVkSubRenderer {
public:
    using Mode = BodyRenderMode;

    void bindWorld(SoftBodyWorld* world) { world_ = world; }
    void setShaders(std::vector<uint32_t> vertSpv, std::vector<uint32_t> fragSpv) {
        vertSpv_ = std::move(vertSpv);
        fragSpv_ = std::move(fragSpv);
    }
    void setShadowShaders(std::vector<uint32_t> vertSpv, std::vector<uint32_t> fragSpv) {
        shadowVertSpv_ = std::move(vertSpv);
        shadowFragSpv_ = std::move(fragSpv);
    }

    // ---- Shadows (Phase 4). Mirrors GltfBodyRenderer. ----
    void enableShadows(VkRenderPass shadowRenderPass, VkImageView shadowView, VkSampler shadowSampler);
    void setShadowLightVP(const glm::mat4& lightVP);
    void renderShadowCasters(VkCommandBuffer cmd, const glm::mat4& lightVP);

    void setMode(Mode m) { mode_ = m; }
    Mode mode() const    { return mode_; }

    void setEnabled(bool e) { enabled_ = e; }
    bool isEnabled() const  { return enabled_; }

    void setCamera(const glm::mat4& view, const glm::mat4& proj, const glm::vec3& eye);
    void setLight(const glm::vec4& dirW0, const glm::vec4& colorIntensityW);

    void syncFromWorld();
    int  instanceCount() const { return static_cast<int>(instances_.size()); }

    // ---- IVkSubRenderer ----
    void onInit(Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
                VkRenderPass renderPass, uint32_t framesInFlight) override;
    void onUpdate(uint32_t frameIndex) override;
    void onRender(VkCommandBuffer cmd, uint32_t frameIndex) override;
    void onCleanup(VkDevice device) override;

private:
    struct Instance {
        Phantom::Gltf::GltfDocument                        doc;
        std::unique_ptr<Phantom::Gltf::GltfSceneRenderer>  renderer;
        size_t                                            vertexCount = 0; // guards size mismatches
    };

    Phantom::Gltf::GltfSceneRenderer::Shaders makeShaders() const;
    Instance makeInstance(const Physics::ISoftBody& body) const;

    SoftBodyWorld* world_ = nullptr;
    std::vector<uint32_t> vertSpv_;
    std::vector<uint32_t> fragSpv_;
    std::vector<uint32_t> shadowVertSpv_;
    std::vector<uint32_t> shadowFragSpv_;

    bool        shadowsEnabled_ = false;
    VkRenderPass shadowRP_      = VK_NULL_HANDLE;
    VkImageView  shadowView_    = VK_NULL_HANDLE;
    VkSampler    shadowSampler_ = VK_NULL_HANDLE;
    glm::mat4    shadowVP_{1.f};

    Mode mode_    = Mode::Wireframe;
    bool enabled_ = true;

    Phantom::VKG::VulkanContext*           ctx_            = nullptr;
    const Phantom::VKG::VulkanCommandPool* pool_           = nullptr;
    VkRenderPass                           renderPass_     = VK_NULL_HANDLE;
    uint32_t                               framesInFlight_ = 0;
    bool                                   ready_          = false;

    glm::mat4 view_{1.f};
    glm::mat4 proj_{1.f};
    glm::vec3 eye_{0.f};
    glm::vec4 lightDirW0_{glm::vec4(glm::normalize(glm::vec3(-0.3f, -1.0f, -0.25f)), 0.0f)};
    glm::vec4 lightColorIntensity_{1.f, 1.f, 1.f, 3.f};

    std::vector<glm::vec3> normalScratch_; // reused each frame

    std::unordered_map<const Physics::ISoftBody*, Instance> instances_;
};

} // namespace Phantom
