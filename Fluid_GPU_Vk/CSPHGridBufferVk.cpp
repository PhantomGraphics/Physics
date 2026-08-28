#include "pch.h"

#include "CSPHGridBufferVk.h"

using namespace Phantom::Physics;

void CSPHGridBufferVk::create(const Phantom::VKG::VulkanContext& ctx,
                               const Phantom::VKG::VulkanCommandPool& pool,
                               int nCells, int nParticles)
{
    assert(nCells > 0 && nParticles > 0);
    numCells     = nCells;
    numParticles = nParticles;

    const VkDeviceSize szCells = static_cast<VkDeviceSize>(nCells)     * sizeof(int);
    const VkDeviceSize szParts = static_cast<VkDeviceSize>(nParticles) * sizeof(int);

    // All buffers are device-local so GPU atomics and shader reads/writes achieve
    // full GPU bandwidth (no HOST_VISIBLE round-trip via PCIe).
    // VK_BUFFER_USAGE_TRANSFER_DST_BIT is required by vkCmdFillBuffer for zeroing.
    const VkBufferUsageFlags gridUsage =
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
        VK_BUFFER_USAGE_TRANSFER_DST_BIT;

    cellCountBuf.create(ctx, pool, szCells, gridUsage);
    cellStartBuf.create(ctx, pool, szCells, gridUsage);
    sortedIdxBuf.create(ctx, pool, szParts, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
}

void CSPHGridBufferVk::remove()
{
    cellCountBuf.destroy();
    cellStartBuf.destroy();
    sortedIdxBuf.destroy();
    numCells = numParticles = 0;
}
