#pragma once

#include "../../CGLib/VulkanGraphics/VulkanBuffer.h"
#include "../../CGLib/VulkanGraphics/VulkanDescriptorPool.h"

#include <vulkan/vulkan.h>
#include <cstdint>
#include <vector>

namespace Phantom::VKG { class VulkanContext; }

namespace Phantom {

// Pipeline configuration for each SSFR pass.
// Fullscreen quad passes without vertex input set bindingDescs/attrDescs to empty.
struct SSFRPassConfig {
    std::vector<uint32_t>                          vertSpv;
    std::vector<uint32_t>                          fragSpv;
    std::vector<VkVertexInputBindingDescription>   bindingDescs;
    std::vector<VkVertexInputAttributeDescription> attrDescs;
    std::vector<VkDescriptorSetLayoutBinding>      descriptorBindings;
    VkPrimitiveTopology topology      = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    bool                depthTest     = true;
    bool                depthWrite    = true;
    VkCompareOp         depthCompareOp = VK_COMPARE_OP_LESS;
    bool                additiveBlend = false;  // ONE+ONE additive blend for particle pass
    uint32_t            framesInFlight = 2;
    uint32_t            uboSize        = 0;     // per-frame UBO byte size (0 = no UBO)
    // Descriptor sets (and matching UBO buffers) allocated per frame-in-flight.
    // The default of 1 matches a pass invoked once per frame. A pass invoked
    // several times per frame (e.g. a separable / ping-pong bilateral filter)
    // must set this to its max invocations per frame so each invocation gets a
    // distinct set -- otherwise updating the shared set between draws while it is
    // still bound invalidates the command buffer (and the shared UBO buffer is
    // overwritten before the earlier draw executes).
    uint32_t            setsPerFrame   = 1;
};

// Manages graphics pipeline + descriptor set + UBO for each SSFR pass.
// Similar to VkFluidPipeline, but supports additive blending and texture descriptor writes.
class SSFRPipeline {
public:
    SSFRPipeline() = default;
    SSFRPipeline(const SSFRPipeline&) = delete;
    SSFRPipeline& operator=(const SSFRPipeline&) = delete;

    // Returns false (and logs to stderr) on Vulkan resource creation failure.
    // Check isValid() before use; partially-created resources are left as-is
    // for destroy() to clean up.
    bool create(const Phantom::VKG::VulkanContext& ctx, VkRenderPass renderPass,
                const SSFRPassConfig& cfg);
    void destroy(VkDevice device);

    // Update the UBO for (frame, slot) (valid only for passes where uboSize > 0).
    // slot must be < setsPerFrame; the 2-arg form targets slot 0.
    void updateUBO(uint32_t frame, const void* data, VkDeviceSize size) { updateUBO(frame, 0, data, size); }
    void updateUBO(uint32_t frame, uint32_t slot, const void* data, VkDeviceSize size);

    // Apply WriteDescriptorSet to the (frame, slot) descriptor set.
    // Used for texture binding updates. The 3-arg form targets slot 0.
    void writeDescriptors(VkDevice device, uint32_t frame,
                          const std::vector<VkWriteDescriptorSet>& writes) {
        writeDescriptors(device, frame, 0, writes);
    }
    void writeDescriptors(VkDevice device, uint32_t frame, uint32_t slot,
                          const std::vector<VkWriteDescriptorSet>& writes);

    VkPipeline       getPipeline()                const { return pipeline_; }
    VkPipelineLayout getLayout()                  const { return layout_; }
    VkDescriptorSet  getDescriptorSet(uint32_t f) const { return getDescriptorSet(f, 0); }
    VkDescriptorSet  getDescriptorSet(uint32_t f, uint32_t slot) const {
        return descriptorSets_[f * setsPerFrame_ + slot];
    }
    bool             isValid()                    const { return pipeline_ != VK_NULL_HANDLE; }

private:
    uint32_t framesInFlight_ = 0;
    uint32_t setsPerFrame_   = 1;
    uint32_t uboSize_        = 0;

    VkPipeline            pipeline_ = VK_NULL_HANDLE;
    VkPipelineLayout      layout_   = VK_NULL_HANDLE;

    Phantom::VKG::VulkanDescriptorSetLayout descriptorSetLayout_;
    Phantom::VKG::VulkanDescriptorPool      descriptorPool_;
    std::vector<VkDescriptorSet>   descriptorSets_;
    std::vector<Phantom::VKG::VulkanBuffer> uniformBuffers_;

    VkShaderModule createShaderModule(VkDevice device,
                                      const std::vector<uint32_t>& spv) const;
};

} // namespace VKSSFR
