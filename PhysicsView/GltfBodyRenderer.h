#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "../../CGLib/GltfRenderer/Gltf/GltfDocument.h"
#include "../../CGLib/GltfRenderer/Renderer/GltfSceneRenderer.h"
#include "../../CGLib/VkAppBase/IVkSubRenderer.h"

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace Phantom {

class RigidBodyWorld;
namespace Physics { class RigidBody; }

/**
 * @brief PBR ("shaded") rendering of the rigid-body scene, an alternative to
 * RigidBodyWireRenderer's wireframe (docs/todo/PLAN_physicsview_gltf_rendering.md
 * Phase 2). Strongly modelled on Universe's Rendering/GltfRenderer.
 *
 * One Phantom::Gltf::GltfSceneRenderer + synthesized unit-primitive GltfDocument
 * per RigidBody, keyed by the (lifetime-stable) RigidBody pointer.
 * syncFromWorld() reconciles that map with RigidBodyWorld's current body set on
 * preset switches / AddSphere / AddBox / AddFloor; onUpdate() streams each
 * body's live transform in through GltfSceneRenderer::setModelMatrix()
 * (translate * rotate * per-shape scale). Deliberately minimal: flat PBR,
 * no skybox / IBL / shadow (matches Universe Phase 2).
 *
 * Mode: Wireframe (default -- this renderer draws nothing, RigidBodyWireRenderer
 * owns the viewport), Shaded (this renderer only), or Both. FluidApp reads
 * mode() to gate the two renderers' setEnabled().
 */
class GltfBodyRenderer : public ::VKG::IVkSubRenderer {
public:
    enum class Mode { Wireframe, Shaded, Both };

    void bindWorld(RigidBodyWorld* world) { world_ = world; }
    void setShaders(std::vector<uint32_t> vertSpv, std::vector<uint32_t> fragSpv) {
        vertSpv_ = std::move(vertSpv);
        fragSpv_ = std::move(fragSpv);
    }

    void setMode(Mode m) { mode_ = m; }
    Mode mode() const    { return mode_; }
    static bool parseMode(const std::string& s, Mode& out);
    static const char* modeName(Mode m);

    // Drawn only when enabled AND mode != Wireframe (FluidApp gates enabled for
    // the Flame page, same as the wire renderers).
    void setEnabled(bool e) { enabled_ = e; }
    bool isEnabled() const  { return enabled_; }

    // Forwarded from FluidApp every frame (FluidRenderer is the camera's single
    // source of truth) alongside the shared directional light.
    void setCamera(const glm::mat4& view, const glm::mat4& proj, const glm::vec3& eye);
    void setLight(const glm::vec4& dirW0, const glm::vec4& colorIntensityW);

    // Reconcile instances_ with world_'s current bodies. Cheap no-op when the
    // body set is unchanged. Safe to call every frame; FluidApp calls it from
    // syncRigidRenderer() (fired on every rigid-world change).
    void syncFromWorld();

    int instanceCount() const { return static_cast<int>(instances_.size()); }

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
    };

    Phantom::Gltf::GltfSceneRenderer::Shaders makeShaders() const;
    static glm::mat4 bodyModelMatrix(const Physics::RigidBody& body);
    Instance makeInstance(const Physics::RigidBody& body) const;

    RigidBodyWorld* world_ = nullptr;
    std::vector<uint32_t> vertSpv_;
    std::vector<uint32_t> fragSpv_;

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

    // unordered_map so an Instance's address is stable across insert/erase of
    // *other* entries -- GltfSceneRenderer::setDocument() stores a raw pointer
    // into Instance::doc (same reason as Universe's map).
    std::unordered_map<const Physics::RigidBody*, Instance> instances_;
};

} // namespace Phantom
