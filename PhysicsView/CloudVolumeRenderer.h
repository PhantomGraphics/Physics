#pragma once

#include "CGLib/VkAppBase/IVkSubRenderer.h"
#include "CGLib/Volume/VolumeRaymarch/VolumePbvrGpu.h"
#include "CGLib/Volume/VolumeRaymarch/VolumeRaymarchGpu.h"

#include <glm/glm.hpp>

#include <vector>

namespace Phantom {

/**
 * @brief IVkSubRenderer that draws a cloud-water density grid into PhysicsView's HDR scene
 * (docs/todo/PLAN_cloud_sph_pbvr.md Phase 3): either the reference raymarch or the ensemble PBVR
 * of the same grids. A thin adapter over the cloud-agnostic Volume::VolumeRaymarchGpu /
 * VolumePbvrGpu: it owns the cloud-space -> scene-space transform, upload scheduling and the
 * accumulation policy, nothing physical.
 *
 * Cloud space is z-up metres; the scene (FluidRenderer camera) is y-up and ~40 units across, so
 * the camera is expressed in cloud space by folding the scene transform into the view-projection.
 * Density / sun changes are picked up in recordPreRender() (compute must run outside the render
 * pass); the raymarch / PBVR composite itself is drawn in onRender().
 *
 * PBVR accumulation: while anything that changes the image moves (density, sun, camera, medium
 * parameters, viewport) the ensemble average restarts and `ensemblesPerFrame` ensembles are added
 * per frame (same-frame averaging only); once still, it keeps adding until `targetEnsembles`.
 */
class CloudVolumeRenderer : public ::VKG::IVkSubRenderer
{
public:
	enum class Mode { Raymarch, Pbvr };

	struct Shaders {
		std::vector<uint32_t> fullscreenVert, raymarchFrag, sunTransmittanceComp;
		std::vector<uint32_t> pbvrGenerateComp, pbvrPointVert, pbvrPointFrag, pbvrAccumulateComp, pbvrCompositeFrag;
	};

	struct PbvrSettings {
		uint32_t ensemblesPerFrame = 2;    ///< Added per frame (running: 1..4 is the interactive budget).
		uint32_t targetEnsembles = 64;     ///< Stop accumulating here while the image is still.
		float minDiameterPx = 1.5f;
		float maxPerCell = 16.0f;
	};

	struct Stats {
		Mode mode = Mode::Raymarch;
		uint32_t accumulated = 0;          ///< Ensembles in the current average (PBVR).
		uint32_t overflowed = 0;           ///< Particles dropped by the capacity limit (PBVR, last query).
		uint32_t generated = 0;
	};

	void setShaders(Shaders s) { shaders_ = std::move(s); }
	void setDepthFormat(VkFormat f) { depthFormat_ = f; }
	void setEnabled(bool e) { enabled_ = e; }
	bool isEnabled() const { return enabled_; }
	void setMode(Mode m);
	Mode mode() const { return mode_; }
	void setPbvrSettings(const PbvrSettings& s);
	const PbvrSettings& pbvrSettings() const { return pbvrSettings_; }

	/** @brief HDR target size in pixels (PBVR screen-sized targets). */
	void resize(uint32_t width, uint32_t height);

	/** @brief Grid to render (cloud-space box). Recreates the GPU images when the layout changes. */
	void setDensity(const Volume::ScalarGrid3D& density);
	void setScattering(const Volume::ScatteringParams& p);
	/** @brief cloud -> scene transform, and the camera in scene space. */
	void setCamera(const glm::mat4& proj, const glm::mat4& view, const glm::mat4& cloudToScene, uint32_t viewportHeight);

	/** @brief Uploads a changed density, refreshes the sun-transmittance grid, renders PBVR ensembles. Outside a render pass. */
	void recordPreRender(VkCommandBuffer cmd, uint32_t frameIndex);

	void onInit(Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
	            VkRenderPass renderPass, uint32_t framesInFlight) override;
	void onRender(VkCommandBuffer cmd, uint32_t frameIndex) override;
	void onCleanup(VkDevice device) override;

	bool isReady() const { return ready_; }
	/** @brief Counters; reading PBVR overflow stalls the GPU, so call sparingly (scenarios / stats panel). */
	Stats queryStats();

private:
	bool createPbvr();

	Shaders shaders_;
	VkFormat depthFormat_ = VK_FORMAT_D32_SFLOAT;
	VkRenderPass renderPass_ = VK_NULL_HANDLE;
	uint32_t framesInFlight_ = 2;
	Phantom::VKG::VulkanContext* ctx_ = nullptr;
	const Phantom::VKG::VulkanCommandPool* pool_ = nullptr;
	Phantom::Volume::VolumeRaymarchGpu gpu_;
	Phantom::Volume::VolumePbvrGpu pbvr_;
	bool ready_ = false;
	bool pbvrReady_ = false;
	bool enabled_ = false;
	Mode mode_ = Mode::Raymarch;
	PbvrSettings pbvrSettings_;
	uint32_t width_ = 0, height_ = 0;

	Volume::ScalarGrid3D pending_;
	bool densityDirty_ = false;
	bool sunDirty_ = false;
	bool pbvrRestart_ = true;
	Volume::ScatteringParams scattering_;
	Phantom::Volume::VolumeRaymarchGpu::Camera camera_;
	Phantom::Volume::VolumePbvrGpu::Camera pbvrCamera_;
	bool hasCamera_ = false;
};

} // namespace Phantom
