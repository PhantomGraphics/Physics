#pragma once

#include "CGLib/VkAppBase/IVkSubRenderer.h"
#include "CGLib/Volume/VolumeRaymarch/VolumeRaymarchGpu.h"

#include <glm/glm.hpp>

#include <vector>

namespace Phantom {

/**
 * @brief IVkSubRenderer that raymarches a cloud-water density grid into PhysicsView's HDR scene
 * (docs/todo/PLAN_cloud_sph_pbvr.md Phase 3). A thin adapter over the cloud-agnostic
 * Volume::VolumeRaymarchGpu: it owns the cloud-space -> scene-space transform and the upload
 * scheduling, nothing physical.
 *
 * Cloud space is z-up metres; the scene (FluidRenderer camera) is y-up and ~40 units across, so
 * the camera is expressed in cloud space by folding the scene transform into the inverse
 * view-projection. Density / sun changes are picked up in recordPreRender() (compute must run
 * outside the render pass), the raymarch itself happens in onRender().
 */
class CloudVolumeRenderer : public ::VKG::IVkSubRenderer
{
public:
	struct Shaders {
		std::vector<uint32_t> fullscreenVert, raymarchFrag, sunTransmittanceComp;
	};

	void setShaders(Shaders s) { shaders_ = std::move(s); }
	void setEnabled(bool e) { enabled_ = e; }
	bool isEnabled() const { return enabled_; }

	/** @brief Grid to render into (cloud-space box). Recreates the GPU images when the layout changes. */
	void setDensity(const Volume::ScalarGrid3D& density);
	void setScattering(const Volume::ScatteringParams& p);
	/** @brief cloud -> scene transform, and the camera in scene space. */
	void setCamera(const glm::mat4& proj, const glm::mat4& view, const glm::mat4& cloudToScene);

	/** @brief Uploads a changed density and refreshes the sun-transmittance grid. Outside a render pass. */
	void recordPreRender(VkCommandBuffer cmd, uint32_t frameIndex);

	void onInit(Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
	            VkRenderPass renderPass, uint32_t framesInFlight) override;
	void onRender(VkCommandBuffer cmd, uint32_t frameIndex) override;
	void onCleanup(VkDevice device) override;

	bool isReady() const { return ready_; }

private:
	Shaders shaders_;
	Phantom::VKG::VulkanContext* ctx_ = nullptr;
	const Phantom::VKG::VulkanCommandPool* pool_ = nullptr;
	Phantom::Volume::VolumeRaymarchGpu gpu_;
	bool ready_ = false;
	bool enabled_ = false;

	Volume::ScalarGrid3D pending_;
	bool densityDirty_ = false;
	bool sunDirty_ = false;
	Volume::ScatteringParams scattering_;
	Phantom::Volume::VolumeRaymarchGpu::Camera camera_;
	bool hasCamera_ = false;
};

} // namespace Phantom
