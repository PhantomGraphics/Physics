#pragma once

#include "FlameFullscreenPass.h"
#include "FlamePointPipeline.h"

#include "CGLib/VulkanGraphics/VulkanOffscreen.h"
#include "CGLib/VulkanGraphics/VulkanSampler.h"

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace Phantom { namespace VKG { class VulkanContext; } }

namespace Phantom {

/**
 * @brief Heat-haze (refraction shimmer) behind the flame, screen space.
 *
 * Three steps per frame, all owned here so FluidApp only has to route the
 * opaque scene through it:
 *  1. recordField()   -- hot particles splat (T - T_ambient) / (T_ref - T_ambient)
 *                        additively into a low-resolution field. The additive sum
 *                        along each view ray stands in for the line integral of
 *                        the refractive-index gradient. The green channel keeps
 *                        the heat-weighted depth so nearer geometry is not refracted.
 *  2. beginBackground()/endBackground() -- the opaque scene (everything except
 *                        the flame) is drawn into a full-size HDR target with
 *                        the same attachment formats as the HDR scene, hence a
 *                        compatible render pass.
 *  3. apply()         -- inside the HDR scene pass: fullscreen draw that copies
 *                        that background over, displaced by animated noise
 *                        scaled by the heat and its gradient. Only colour is
 *                        displaced; depth stays at the true scene geometry.
 */
class FlameHazePass {
public:
	struct Shaders {
		std::vector<uint32_t> fieldVert, fieldFrag; // flame_haze_field
		std::vector<uint32_t> fullscreenVert;       // flame_fullscreen
		std::vector<uint32_t> applyFrag;            // flame_haze_apply
	};

	struct Settings {
		bool  enabled = true;
		float strength = 10.0f;  ///< peak displacement in pixels (at full heat)
		float extent = 3.5f;     ///< haze sprite size relative to the flame sprite
		float frequency = 24.0f; ///< noise cells per screen height
		float riseSpeed = 0.35f; ///< screen heights per second the pattern climbs
	};

	static constexpr uint32_t kFieldDownscale = 4;

	bool create(const Phantom::VKG::VulkanContext& ctx, VkRenderPass hdrRenderPass, VkFormat depthFormat,
		uint32_t framesInFlight, const Shaders& shaders, uint32_t width, uint32_t height);
	bool resize(const Phantom::VKG::VulkanContext& ctx, uint32_t width, uint32_t height);
	void destroy(const Phantom::VKG::VulkanContext& ctx);
	bool isValid() const { return valid_; }

	/** @brief Per-frame CPU side (from onUpdate). Streams are the flame emitters. */
	void update(const Phantom::VKG::VulkanContext& ctx, uint32_t frameIndex, const FlamePointUBO& ubo,
		const Settings& settings, uint32_t count, const float* positions, const float* temperatures,
		const float* sizes, float timeSeconds);

	/** @brief Renders the haze field; outside any render pass. */
	void recordField(VkCommandBuffer cmd, uint32_t frameIndex);

	/** @brief Opens / closes the render pass the non-flame scene renderers draw into. */
	void beginBackground(VkCommandBuffer cmd, const std::array<float, 4>& clear) const;
	void endBackground(VkCommandBuffer cmd) const;

	/** @brief First draw of the HDR scene pass: displaced copy of the background. */
	void apply(VkCommandBuffer cmd) const;

private:
	bool createTargets(const Phantom::VKG::VulkanContext& ctx, uint32_t width, uint32_t height);
	void writeImageDescriptors(VkDevice device);

	bool valid_ = false;
	VkFormat depthFormat_ = VK_FORMAT_D32_SFLOAT;
	Phantom::VKG::VulkanOffscreen field_;
	Phantom::VKG::VulkanOffscreen background_;
	Phantom::VKG::VulkanSampler sampler_;
	std::optional<FlamePointPipeline> fieldPipeline_;
	FlameFullscreenPass apply_;
	std::array<float, 4> pushA_{ 0.0f, 0.0f, 10.0f, 0.35f };
};

} // namespace Phantom
