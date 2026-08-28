#pragma once

#include <vector>
#include <vulkan/vulkan.h>

#include "../../CGLib/Util/UnCopyable.h"
#include "../../CGLib/VulkanGraphics/VulkanContext.h"
#include "../../CGLib/VulkanGraphics/VulkanBuffer.h"
#include "../../CGLib/VulkanGraphics/VulkanCommandPool.h"

namespace Phantom {
    namespace Physics {

// GPU-side uniform grid buffers for Vulkan CSPH neighbor search.
//   binding 3: cellCount  (int[numCells])     窶・device-local (written by GPU atomics)
//   binding 4: cellStart  (int[numCells])     窶・device-local (written by GPU prefix-sum)
//   binding 5: sortedIdx  (int[numParticles]) 窶・device-local
//
// All three buffers are device-local so GPU atomics achieve full bandwidth.
// The CPU prefix-sum pass has been replaced by a compute shader (csph_prefix).
class CSPHGridBufferVk : private UnCopyable
{
public:
    CSPHGridBufferVk() = default;
    ~CSPHGridBufferVk() { remove(); }

    void create(const Phantom::VKG::VulkanContext& ctx,
                const Phantom::VKG::VulkanCommandPool& pool,
                int numCells, int numParticles);

    void remove();

    VkBuffer cellCountBuffer() const { return cellCountBuf.get(); }
    VkBuffer cellStartBuffer() const { return cellStartBuf.get(); }
    VkBuffer sortedIdxBuffer() const { return sortedIdxBuf.get(); }

    int getNumCells()     const { return numCells; }
    int getNumParticles() const { return numParticles; }

private:
    Phantom::VKG::VulkanBuffer cellCountBuf;  // device-local
    Phantom::VKG::VulkanBuffer cellStartBuf;  // device-local
    Phantom::VKG::VulkanBuffer sortedIdxBuf;  // device-local

    int numCells     = 0;
    int numParticles = 0;
};

    }
}
