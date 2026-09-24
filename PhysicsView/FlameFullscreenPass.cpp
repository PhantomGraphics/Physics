#include "FlameFullscreenPass.h"

#include "CGLib/VulkanGraphics/VulkanContext.h"

using namespace Phantom::VKG;

namespace Phantom {

bool FlameFullscreenPass::create(const VulkanContext& ctx, VkRenderPass renderPass, const Config& config)
{
	config_ = config;
	VkDevice device = ctx.getDevice();

	std::vector<VkDescriptorSetLayoutBinding> bindings(config.imageCount);
	for (uint32_t i = 0; i < config.imageCount; ++i) {
		bindings[i].binding = i;
		bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		bindings[i].descriptorCount = 1;
		bindings[i].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
	}
	setLayout_.create(device, bindings);

	VkDescriptorPoolSize poolSize{ VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, config.imageCount * config.setCount };
	pool_.create(device, { poolSize }, config.setCount);
	std::vector<VkDescriptorSetLayout> layouts(config.setCount, setLayout_.get());
	sets_ = pool_.allocateSets(device, layouts);
	if (sets_.size() != config.setCount) {
		return false;
	}

	PipelineConfig p{};
	p.vertSpv = config.vertSpv;
	p.fragSpv = config.fragSpv;
	p.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
	p.descriptorSetLayout = setLayout_.get();
	p.cullMode = VK_CULL_MODE_NONE;
	p.depthTest = false;
	p.depthWrite = false;
	p.blendEnable = config.premultipliedBlend;
	p.premultipliedAlphaBlend = config.premultipliedBlend;
	if (config.pushConstantSize > 0) {
		p.pushConstantRanges.push_back({ VK_SHADER_STAGE_FRAGMENT_BIT, 0, config.pushConstantSize });
	}
	return pipeline_.create(ctx, renderPass, p);
}

void FlameFullscreenPass::destroy(VkDevice device)
{
	pipeline_.destroy(device);
	pool_.destroy(device);
	setLayout_.destroy(device);
	sets_.clear();
}

void FlameFullscreenPass::setImages(VkDevice device, uint32_t set, const std::vector<VkImageView>& views, VkSampler sampler)
{
	if (set >= sets_.size() || views.size() != config_.imageCount) {
		return;
	}
	std::vector<VkDescriptorImageInfo> infos(views.size());
	std::vector<VkWriteDescriptorSet> writes(views.size());
	for (size_t i = 0; i < views.size(); ++i) {
		infos[i] = { sampler, views[i], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
		writes[i] = {};
		writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writes[i].dstSet = sets_[set];
		writes[i].dstBinding = static_cast<uint32_t>(i);
		writes[i].descriptorCount = 1;
		writes[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		writes[i].pImageInfo = &infos[i];
	}
	vkUpdateDescriptorSets(device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void FlameFullscreenPass::draw(VkCommandBuffer cmd, uint32_t set, const void* pushData) const
{
	if (set >= sets_.size() || !isValid()) {
		return;
	}
	vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.getPipeline());
	vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.getLayout(), 0, 1, &sets_[set], 0, nullptr);
	if (pushData && config_.pushConstantSize > 0) {
		vkCmdPushConstants(cmd, pipeline_.getLayout(), VK_SHADER_STAGE_FRAGMENT_BIT, 0, config_.pushConstantSize, pushData);
	}
	vkCmdDraw(cmd, 3, 1, 0, 0);
}

} // namespace Phantom
