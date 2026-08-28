#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "CGLib/VkAppBase/IVkSubRenderer.h"
#include "CGLib/Renderer/VkRenderer/VkTriangleRenderer.h"

#include <optional>
#include <vector>

namespace Phantom {

/**
 * @brief IVkSubRenderer that draws the triangle mesh produced by
 * FluidMeshConverter (Marching Cubes over a Volume::SparseVolumef). Wraps
 * Phantom::VKG::VkTriangleRenderer, mirroring RigidBodyWireRenderer's
 * external-MVP wrapper pattern (no owned camera).
 */
class FluidMeshRenderer : public ::VKG::IVkSubRenderer {
public:
    struct Shaders {
        std::vector<uint32_t> vertSpv;
        std::vector<uint32_t> fragSpv;
    };

    void setShaders(Shaders s)   { shaders_ = std::move(s); }
    void setEnabled(bool e)      { enabled_ = e; }
    bool isEnabled() const       { return enabled_; }

    void update(const std::vector<float>&    positions,
                const std::vector<float>&    colors,
                const std::vector<uint32_t>& indices,
                const glm::mat4&             mvp);

    // Refreshes the camera only, without re-uploading geometry. Call every
    // frame the camera moves so the mesh tracks it even between
    // world-changed events (which is when update() above gets called).
    void setMVP(const glm::mat4& mvp) { mvp_ = mvp; }

    void onInit(Phantom::VKG::VulkanContext& ctx,
                const Phantom::VKG::VulkanCommandPool& pool,
                VkRenderPass renderPass,
                uint32_t framesInFlight) override;

    void onUpdate(uint32_t frameIndex) override;
    void onRender(VkCommandBuffer cmd, uint32_t frameIndex) override;
    void onCleanup(VkDevice device) override;

private:
    Shaders shaders_;
    bool    dirty_   = false;
    bool    enabled_ = false;

    const Phantom::VKG::VulkanContext*     ctx_  = nullptr;
    const Phantom::VKG::VulkanCommandPool* pool_ = nullptr;

    std::optional<VKG::VkTriangleRenderer> triangleRenderer_;

    std::vector<float>    positions_;
    std::vector<float>    colors_;
    std::vector<uint32_t> indices_;
    glm::mat4              mvp_{1.f};
};

} // namespace Phantom
