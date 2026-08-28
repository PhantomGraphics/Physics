#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "CGLib/VkAppBase/IVkSubRenderer.h"
#include "CGLib/Renderer/VkRenderer/VkPointRenderer.h"

#include <optional>
#include <vector>

namespace Phantom {

/**
 * @brief IVkSubRenderer that draws a Volume::SparseVolumef's active-voxel
 * centers as a point cloud (FluidVolumeConverter::getVoxelPositions()).
 * Wraps Phantom::VKG::VkPointRenderer, mirroring RigidBodyWireRenderer's
 * external-MVP wrapper pattern (no owned camera -- the caller passes the
 * shared scene MVP into update() each time the data or camera changes).
 */
class VolumeRenderer : public ::VKG::IVkSubRenderer {
public:
    struct Shaders {
        std::vector<uint32_t> vertSpv;
        std::vector<uint32_t> fragSpv;
    };

    void setShaders(Shaders s)   { shaders_ = std::move(s); }
    void setEnabled(bool e)      { enabled_ = e; }
    bool isEnabled() const       { return enabled_; }

    void update(const std::vector<float>& positions,
                const std::vector<float>& colors,
                const std::vector<float>& sizes,
                const glm::mat4&          mvp);

    // Updates the camera only, keeping the last uploaded point data. Unlike
    // RigidBodyWireRenderer::setMVP(), this does NOT skip re-uploading next
    // frame -- VkPointRenderer has no incremental per-frame MVP update path
    // (see onUpdate()), so every frame re-uploads regardless.
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
    bool    enabled_ = false;

    const Phantom::VKG::VulkanContext*     ctx_  = nullptr;
    const Phantom::VKG::VulkanCommandPool* pool_ = nullptr;

    std::optional<VKG::VkPointRenderer> pointRenderer_;

    std::vector<float> positions_;
    std::vector<float> colors_;
    std::vector<float> sizes_;
    glm::mat4           mvp_{1.f};
};

} // namespace Phantom
