#pragma once

#include "CGLib/VulkanGraphics/VulkanBuffer.h"

#include <vulkan/vulkan.h>

#include <cstdint>
#include <cstring>
#include <vector>

namespace Phantom { namespace VKG { class VulkanContext; } }

namespace Phantom {

/**
 * @brief Persistent per-frame-in-flight host-visible buffer ring for data that
 * is rewritten every frame (the Flame page's particle vertex streams).
 *
 * Replaces the old destroy()+create() per upload (plan A6): that freed a
 * vertex buffer the previous frame's command buffer could still be reading,
 * and every create() went through a staging copy + vkQueueWaitIdle (about 7
 * GPU stalls per frame across the three flame pipelines).
 *
 * One slot per frame in flight. write(frameIndex, ...) is only called from
 * the owning sub-renderer's onUpdate(frameIndex), which VkAppBase runs after
 * waiting on that frame slot's fence -- so the slot being written (and, when
 * it has to grow, destroyed) is guaranteed idle. Capacity grows by doubling
 * and never shrinks.
 */
class FlameStreamBuffer {
public:
	void init(uint32_t framesInFlight, VkBufferUsageFlags usage)
	{
		slots_.resize(framesInFlight);
		usage_ = usage;
	}

	/**
	 * @brief Copies bytes into frame slot frameIndex, growing it if needed.
	 * @return false if the (re)allocation failed; the slot is then left empty.
	 */
	bool write(const Phantom::VKG::VulkanContext& ctx, uint32_t frameIndex, const void* data, VkDeviceSize bytes);

	/** @brief The slot's buffer (VK_NULL_HANDLE until the first successful write). */
	VkBuffer get(uint32_t frameIndex) const
	{
		return frameIndex < slots_.size() ? slots_[frameIndex].getBuffer() : VK_NULL_HANDLE;
	}

	void destroy()
	{
		for (auto& s : slots_) {
			s.destroy();
		}
		slots_.clear();
	}

private:
	std::vector<Phantom::VKG::VulkanBuffer> slots_;
	VkBufferUsageFlags usage_ = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
};

} // namespace Phantom
