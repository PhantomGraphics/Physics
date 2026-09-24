#pragma once

#include "CGLib/VulkanGraphics/VulkanDescriptorPool.h"
#include "CGLib/VulkanGraphics/VulkanPipeline.h"

#include <vulkan/vulkan.h>

#include <cstdint>
#include <vector>

namespace Phantom { namespace VKG { class VulkanContext; } }

namespace Phantom {

/**
 * @brief Fullscreen-triangle pass sampling N images (texelFetch), with an
 * optional fragment push-constant block. Used by the Flame PBVR ensemble
 * accumulation (flame_pbvr_blend.frag) and its HDR composite
 * (flame_pbvr_composite.frag).
 *
 * `setCount` descriptor sets are allocated up front so ping-pong variants
 * (history A vs B) are selected by index at draw time -- never rewritten
 * while a recorded command buffer might still reference them. setImages() is
 * only called at creation / resize, with the device idle.
 */
class FlameFullscreenPass {
public:
	struct Config {
		std::vector<uint32_t> vertSpv;
		std::vector<uint32_t> fragSpv;
		uint32_t imageCount = 1;
		uint32_t setCount = 1;
		uint32_t pushConstantSize = 0;
		bool premultipliedBlend = false; ///< false = overwrite (no blending)
	};

	bool create(const Phantom::VKG::VulkanContext& ctx, VkRenderPass renderPass, const Config& config);
	void destroy(VkDevice device);

	/** @brief Points descriptor set `set` at `views` (bindings 0..imageCount-1). */
	void setImages(VkDevice device, uint32_t set, const std::vector<VkImageView>& views, VkSampler sampler);

	/** @brief Records the draw. Caller has begun a render pass compatible with create()'s. */
	void draw(VkCommandBuffer cmd, uint32_t set, const void* pushData = nullptr) const;

	bool isValid() const { return pipeline_.getPipeline() != VK_NULL_HANDLE; }

private:
	Config config_;
	Phantom::VKG::VulkanDescriptorSetLayout setLayout_;
	Phantom::VKG::VulkanDescriptorPool pool_;
	std::vector<VkDescriptorSet> sets_;
	Phantom::VKG::VulkanPipeline pipeline_;
};

} // namespace Phantom
