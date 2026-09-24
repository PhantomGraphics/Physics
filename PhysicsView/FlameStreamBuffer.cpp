#include "FlameStreamBuffer.h"

#include "CGLib/VulkanGraphics/VulkanContext.h"

#include <vk_mem_alloc.h>

namespace Phantom {

bool FlameStreamBuffer::write(const Phantom::VKG::VulkanContext& ctx, uint32_t frameIndex, const void* data, VkDeviceSize bytes)
{
	if (frameIndex >= slots_.size() || bytes == 0) {
		return false;
	}
	auto& slot = slots_[frameIndex];
	if (!slot.isValid() || slot.getSize() < bytes) {
		VkDeviceSize capacity = slot.isValid() ? slot.getSize() : 4096;
		while (capacity < bytes) {
			capacity *= 2;
		}
		slot.destroy();
		if (!slot.createMapped(ctx, capacity, usage_)) {
			return false;
		}
	}
	slot.write(data, bytes);
	// HOST_ACCESS_SEQUENTIAL_WRITE memory may be non-coherent; a no-op when it is.
	vmaFlushAllocation(ctx.getAllocator(), slot.getVmaAllocation(), 0, bytes);
	return true;
}

} // namespace Phantom
