#pragma once

#include <vector>
#include <vulkan/vulkan.h>

#include "../../CGLib/Math/Vector3d.h"
#include "../../CGLib/Util/UnCopyable.h"
#include "../../CGLib/VulkanGraphics/VulkanContext.h"
#include "../../CGLib/VulkanGraphics/VulkanBuffer.h"
#include "../../CGLib/VulkanGraphics/VulkanCommandPool.h"

namespace Phantom {
    namespace Physics {

// GPU-side particle data buffers for Vulkan CSPH simulation.
// Manages three VMA-backed device-local buffers:
//   binding 0: positions  (vec4: xyz=position, w=mass)
//   binding 1: velocities (vec4: xyz=velocity, w=density)
//   binding 2: forces     (vec4: xyz=force,    w=unused)
class CSPHParticleBufferVk : private UnCopyable
{
public:
    CSPHParticleBufferVk() = default;
    ~CSPHParticleBufferVk() { remove(); }

    void create(const Phantom::VKG::VulkanContext& ctx,
                const Phantom::VKG::VulkanCommandPool& pool,
                int numParticles);

    void remove();

    void upload(const Phantom::VKG::VulkanContext& ctx,
                const Phantom::VKG::VulkanCommandPool& pool,
                const std::vector<Math::Vector3df>& positions,
                const std::vector<float>& masses,
                const std::vector<Math::Vector3df>& velocities);

    // Downloads current positions via a temporary host-visible staging buffer.
    void downloadPositions(const Phantom::VKG::VulkanContext& ctx,
                           const Phantom::VKG::VulkanCommandPool& pool,
                           std::vector<Math::Vector3df>& out) const;

    VkBuffer posBuffer()   const { return posBuf.get(); }
    VkBuffer velBuffer()   const { return velBuf.get(); }
    VkBuffer forceBuffer() const { return forceBuf.get(); }

    int getNumParticles() const { return numParticles; }

private:
    Phantom::VKG::VulkanBuffer posBuf;
    Phantom::VKG::VulkanBuffer velBuf;
    Phantom::VKG::VulkanBuffer forceBuf;

    int numParticles = 0;
};

    }
}
