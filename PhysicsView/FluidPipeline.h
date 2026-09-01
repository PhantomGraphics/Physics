#pragma once

#include "../../CGLib/VulkanGraphics/VulkanPipeline.h"
#include "../../CGLib/VulkanGraphics/VulkanBuffer.h"
#include "../../CGLib/VulkanGraphics/VulkanDescriptorPool.h"

namespace Phantom {
    class VulkanContext;
class FluidPipeline {
public:
    struct Config {
        std::vector<uint32_t> vertSpv;
        std::vector<uint32_t> fragSpv;
        VkVertexInputBindingDescription               bindingDesc{};
        std::vector<VkVertexInputAttributeDescription> attrDescs;
        uint32_t framesInFlight = 2;
    };

    void create(const Phantom::VKG::VulkanContext& ctx, VkRenderPass renderPass, const Config& cfg);
    void destroy(VkDevice device);

    void updateUBO(uint32_t frame, const glm::mat4& mvp,
                   float densityMin, float densityMax, bool useDensity);

    VkPipeline       getPipeline() const                { return pipeline_.getPipeline(); }
    VkPipelineLayout getLayout() const                  { return pipeline_.getLayout(); }
    VkDescriptorSet  getDescriptorSet(uint32_t frame) const { return descriptorSets_[frame]; }

private:
    uint32_t framesInFlight_ = 0;
    Phantom::VKG::VulkanDescriptorSetLayout descriptorSetLayout_;
    Phantom::VKG::VulkanDescriptorPool      descriptorPool_;
    std::vector<VkDescriptorSet>   descriptorSets_;
    Phantom::VKG::VulkanPipeline            pipeline_;
    std::vector<Phantom::VKG::VulkanBuffer> uniformBuffers_;

    struct alignas(16) UBO {
        glm::mat4 mvp;
        glm::vec4 colorParams; // min, max, enabled, padding
    };
};

} // namespace Phantom
