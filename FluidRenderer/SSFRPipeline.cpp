#include "SSFRPipeline.h"
#include "../../CGLib/VulkanGraphics/VulkanContext.h"

#include <cstdio>

namespace Phantom {

bool SSFRPipeline::create(const Phantom::VKG::VulkanContext& ctx,
                             VkRenderPass renderPass,
                             const SSFRPassConfig& cfg)
{
    framesInFlight_ = cfg.framesInFlight;
    uboSize_        = cfg.uboSize;
    VkDevice device = ctx.getDevice();

    // --- Descriptor set layout ---
    if (!cfg.descriptorBindings.empty()) {
        descriptorSetLayout_.create(device, cfg.descriptorBindings);
    }

    // --- Shader modules ---
    VkShaderModule vertMod = createShaderModule(device, cfg.vertSpv);
    VkShaderModule fragMod = createShaderModule(device, cfg.fragSpv);
    if (vertMod == VK_NULL_HANDLE || fragMod == VK_NULL_HANDLE) {
        if (vertMod) vkDestroyShaderModule(device, vertMod, nullptr);
        if (fragMod) vkDestroyShaderModule(device, fragMod, nullptr);
        return false;
    }

    VkPipelineShaderStageCreateInfo stages[2]{};
    stages[0].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage  = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = vertMod;
    stages[0].pName  = "main";
    stages[1].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[1].stage  = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = fragMod;
    stages[1].pName  = "main";

    // --- Vertex input ---
    VkPipelineVertexInputStateCreateInfo vi{};
    vi.sType                           = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vi.vertexBindingDescriptionCount   = static_cast<uint32_t>(cfg.bindingDescs.size());
    vi.pVertexBindingDescriptions      = cfg.bindingDescs.empty() ? nullptr : cfg.bindingDescs.data();
    vi.vertexAttributeDescriptionCount = static_cast<uint32_t>(cfg.attrDescs.size());
    vi.pVertexAttributeDescriptions    = cfg.attrDescs.empty()  ? nullptr : cfg.attrDescs.data();

    VkPipelineInputAssemblyStateCreateInfo ia{};
    ia.sType    = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    ia.topology = cfg.topology;

    // --- Viewport / scissor (dynamic state) ---
    VkPipelineViewportStateCreateInfo vs{};
    vs.sType         = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    vs.viewportCount = 1;
    vs.scissorCount  = 1;

    // --- Rasterizer ---
    VkPipelineRasterizationStateCreateInfo rs{};
    rs.sType       = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode    = VK_CULL_MODE_NONE;
    rs.frontFace   = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rs.lineWidth   = 1.0f;

    // --- MSAA (disabled) ---
    VkPipelineMultisampleStateCreateInfo ms{};
    ms.sType                = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    // --- Depth / stencil ---
    VkPipelineDepthStencilStateCreateInfo ds{};
    ds.sType            = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    ds.depthTestEnable  = cfg.depthTest  ? VK_TRUE : VK_FALSE;
    ds.depthWriteEnable = cfg.depthWrite ? VK_TRUE : VK_FALSE;
    ds.depthCompareOp   = VK_COMPARE_OP_LESS;

    // --- Color blend ---
    VkPipelineColorBlendAttachmentState cba{};
    cba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                         VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    if (cfg.additiveBlend) {
        // ONE+ONE additive blend for particle pass
        cba.blendEnable         = VK_TRUE;
        cba.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
        cba.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
        cba.colorBlendOp        = VK_BLEND_OP_ADD;
        cba.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        cba.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
        cba.alphaBlendOp        = VK_BLEND_OP_ADD;
    } else {
        cba.blendEnable = VK_FALSE;
    }

    VkPipelineColorBlendStateCreateInfo cb{};
    cb.sType           = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    cb.attachmentCount = 1;
    cb.pAttachments    = &cba;

    // --- Dynamic state ---
    VkDynamicState dynStates[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    VkPipelineDynamicStateCreateInfo dyn{};
    dyn.sType             = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dyn.dynamicStateCount = 2;
    dyn.pDynamicStates    = dynStates;

    // --- Pipeline layout ---
    VkPipelineLayoutCreateInfo plci{};
    plci.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    VkDescriptorSetLayout descLayout = descriptorSetLayout_.get();
    if (descriptorSetLayout_.isValid()) {
        plci.setLayoutCount = 1;
        plci.pSetLayouts    = &descLayout;
    }
    if (vkCreatePipelineLayout(device, &plci, nullptr, &layout_) != VK_SUCCESS) {
        std::fprintf(stderr, "[VKSSFR] Failed to create pipeline layout\n");
        vkDestroyShaderModule(device, vertMod, nullptr);
        vkDestroyShaderModule(device, fragMod, nullptr);
        return false;
    }

    // --- Graphics pipeline ---
    VkGraphicsPipelineCreateInfo pci{};
    pci.sType               = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pci.stageCount          = 2;
    pci.pStages             = stages;
    pci.pVertexInputState   = &vi;
    pci.pInputAssemblyState = &ia;
    pci.pViewportState      = &vs;
    pci.pRasterizationState = &rs;
    pci.pMultisampleState   = &ms;
    pci.pDepthStencilState  = &ds;
    pci.pColorBlendState    = &cb;
    pci.pDynamicState       = &dyn;
    pci.layout              = layout_;
    pci.renderPass          = renderPass;

    if (vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pci, nullptr, &pipeline_) != VK_SUCCESS) {
        std::fprintf(stderr, "[VKSSFR] Failed to create graphics pipeline\n");
        vkDestroyShaderModule(device, vertMod, nullptr);
        vkDestroyShaderModule(device, fragMod, nullptr);
        vkDestroyPipelineLayout(device, layout_, nullptr);
        layout_ = VK_NULL_HANDLE;
        return false;
    }

    vkDestroyShaderModule(device, vertMod, nullptr);
    vkDestroyShaderModule(device, fragMod, nullptr);

    // --- per-frame UBO ---
    if (uboSize_ > 0) {
        uniformBuffers_.resize(framesInFlight_);
        for (auto& ub : uniformBuffers_)
            ub.createMapped(ctx, uboSize_, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
    }

    if (!cfg.descriptorBindings.empty()) {
        // Determine descriptor pool sizes from bindings.
        std::vector<VkDescriptorPoolSize> poolSizes;
        for (const auto& b : cfg.descriptorBindings) {
            bool found = false;
            for (auto& ps : poolSizes) {
                if (ps.type == b.descriptorType) {
                    ps.descriptorCount += framesInFlight_;
                    found = true;
                    break;
                }
            }
            if (!found)
                poolSizes.push_back({ b.descriptorType, framesInFlight_ });
        }
        descriptorPool_.create(device, poolSizes, framesInFlight_);

        std::vector<VkDescriptorSetLayout> layouts(framesInFlight_, descriptorSetLayout_.get());
        descriptorSets_ = descriptorPool_.allocateSets(device, layouts);

        // Pre-fill UBO bindings.
        if (uboSize_ > 0) {
            for (uint32_t i = 0; i < framesInFlight_; ++i) {
                // Write UBO bindings (type == UNIFORM_BUFFER).
                for (const auto& b : cfg.descriptorBindings) {
                    if (b.descriptorType != VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER) continue;

                    VkDescriptorBufferInfo bi{};
                    bi.buffer = uniformBuffers_[i].get();
                    bi.offset = 0;
                    bi.range  = uboSize_;

                    VkWriteDescriptorSet w{};
                    w.sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                    w.dstSet          = descriptorSets_[i];
                    w.dstBinding      = b.binding;
                    w.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                    w.descriptorCount = 1;
                    w.pBufferInfo     = &bi;
                    vkUpdateDescriptorSets(device, 1, &w, 0, nullptr);
                }
            }
        }
    }
    return true;
}

void SSFRPipeline::destroy(VkDevice device)
{
    for (auto& ub : uniformBuffers_)
        ub.destroy(device);
    uniformBuffers_.clear();

    if (!descriptorSets_.empty()) {
        descriptorPool_.destroy(device);
        descriptorSets_.clear();
    }
    descriptorSetLayout_.destroy(device);

    if (pipeline_) { vkDestroyPipeline(device, pipeline_, nullptr);       pipeline_ = VK_NULL_HANDLE; }
    if (layout_)   { vkDestroyPipelineLayout(device, layout_, nullptr);   layout_   = VK_NULL_HANDLE; }

    framesInFlight_ = 0;
    uboSize_        = 0;
}

void SSFRPipeline::updateUBO(uint32_t frame, const void* data, VkDeviceSize size)
{
    uniformBuffers_[frame].write(data, size);
}

void SSFRPipeline::writeDescriptors(VkDevice device, uint32_t frame,
                                       const std::vector<VkWriteDescriptorSet>& writes)
{
    // Override dstSet to the per-frame descriptor set before applying.
    std::vector<VkWriteDescriptorSet> patched = writes;
    for (auto& w : patched)
        w.dstSet = descriptorSets_[frame];
    vkUpdateDescriptorSets(device, static_cast<uint32_t>(patched.size()),
                           patched.data(), 0, nullptr);
}

VkShaderModule SSFRPipeline::createShaderModule(
    VkDevice device, const std::vector<uint32_t>& spv) const
{
    VkShaderModuleCreateInfo ci{};
    ci.sType    = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    ci.codeSize = spv.size() * sizeof(uint32_t);
    ci.pCode    = spv.data();

    VkShaderModule mod = VK_NULL_HANDLE;
    if (vkCreateShaderModule(device, &ci, nullptr, &mod) != VK_SUCCESS) {
        std::fprintf(stderr, "[VKSSFR] Failed to create shader module\n");
        return VK_NULL_HANDLE;
    }
    return mod;
}

} // namespace VKSSFR
