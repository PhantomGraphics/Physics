#include "pch.h"

#include "CSPHParticleBufferVk.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

void CSPHParticleBufferVk::create(const Phantom::VKG::VulkanContext& ctx,
                                   const Phantom::VKG::VulkanCommandPool& pool,
                                   int n)
{
    assert(n > 0);
    numParticles = n;

    const VkDeviceSize sz = static_cast<VkDeviceSize>(n) * sizeof(float) * 4;
    const VkBufferUsageFlags usage =
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT  |
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT   |
        VK_BUFFER_USAGE_TRANSFER_DST_BIT;

    posBuf.create(ctx, pool, sz, usage);
    velBuf.create(ctx, pool, sz, usage);
    forceBuf.create(ctx, pool, sz, usage);
}

void CSPHParticleBufferVk::remove()
{
    posBuf.destroy();
    velBuf.destroy();
    forceBuf.destroy();
    numParticles = 0;
}

void CSPHParticleBufferVk::upload(const Phantom::VKG::VulkanContext& ctx,
                                   const Phantom::VKG::VulkanCommandPool& pool,
                                   const std::vector<Vector3df>& positions,
                                   const std::vector<float>& masses,
                                   const std::vector<Vector3df>& velocities)
{
    assert(static_cast<int>(positions.size())  == numParticles);
    assert(static_cast<int>(masses.size())     == numParticles);
    assert(static_cast<int>(velocities.size()) == numParticles);

    const VkDeviceSize sz = static_cast<VkDeviceSize>(numParticles) * sizeof(float) * 4;

    std::vector<float> posData(numParticles * 4);
    std::vector<float> velData(numParticles * 4, 0.0f);

    for (int i = 0; i < numParticles; ++i) {
        posData[i * 4 + 0] = positions[i].x;
        posData[i * 4 + 1] = positions[i].y;
        posData[i * 4 + 2] = positions[i].z;
        posData[i * 4 + 3] = masses[i];

        velData[i * 4 + 0] = velocities[i].x;
        velData[i * 4 + 1] = velocities[i].y;
        velData[i * 4 + 2] = velocities[i].z;
        // velData[i*4+3] = 0: density, computed each step
    }

    std::vector<float> zeros(numParticles * 4, 0.0f);

    posBuf.upload(ctx, pool, posData.data(), sz);
    velBuf.upload(ctx, pool, velData.data(), sz);
    forceBuf.upload(ctx, pool, zeros.data(), sz);
}

void CSPHParticleBufferVk::downloadPositions(const Phantom::VKG::VulkanContext& ctx,
                                              const Phantom::VKG::VulkanCommandPool& pool,
                                              std::vector<Vector3df>& out) const
{
    const VkDeviceSize sz = static_cast<VkDeviceSize>(numParticles) * sizeof(float) * 4;

    Phantom::VKG::VulkanBuffer staging;
    staging.createMapped(ctx, sz, VK_BUFFER_USAGE_TRANSFER_DST_BIT);
    Phantom::VKG::VulkanBuffer::copyBuffer(ctx, pool, posBuf.get(), staging.get(), sz);

    const float* src = static_cast<const float*>(staging.getMapped());
    out.resize(numParticles);
    for (int i = 0; i < numParticles; ++i) {
        out[i].x = src[i * 4 + 0];
        out[i].y = src[i * 4 + 1];
        out[i].z = src[i * 4 + 2];
    }

    staging.destroy();
}
