#include "FlamePointPipeline.h"

#include "CGLib/VulkanGraphics/VulkanContext.h"

using namespace Phantom::VKG;

namespace Phantom {

namespace {

VkFormat floatFormat(uint32_t components)
{
	switch (components) {
	case 1: return VK_FORMAT_R32_SFLOAT;
	case 2: return VK_FORMAT_R32G32_SFLOAT;
	case 3: return VK_FORMAT_R32G32B32_SFLOAT;
	default: return VK_FORMAT_R32G32B32A32_SFLOAT;
	}
}

}

bool FlamePointPipeline::create(const VulkanContext& ctx, VkRenderPass renderPass, uint32_t framesInFlight)
{
	framesInFlight_ = framesInFlight;
	counts_.assign(framesInFlight, 0);
	VkDevice device = ctx.getDevice();

	VkDescriptorSetLayoutBinding uboBinding{};
	uboBinding.binding = 0;
	uboBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	uboBinding.descriptorCount = 1;
	uboBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
	descriptorSetLayout_.create(device, { uboBinding });

	VkDescriptorPoolSize poolSize{ VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, framesInFlight };
	descriptorPool_.create(device, { poolSize }, framesInFlight);

	std::vector<VkDescriptorSetLayout> layouts(framesInFlight, descriptorSetLayout_.get());
	descriptorSets_ = descriptorPool_.allocateSets(device, layouts);

	uniformBuffers_.resize(framesInFlight);
	for (uint32_t i = 0; i < framesInFlight; ++i) {
		if (!uniformBuffers_[i].createMapped(ctx, sizeof(FlamePointUBO), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT)) {
			return false;
		}
		VkDescriptorBufferInfo bi{ uniformBuffers_[i].getBuffer(), 0, sizeof(FlamePointUBO) };
		VkWriteDescriptorSet w{};
		w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		w.dstSet = descriptorSets_[i];
		w.dstBinding = 0;
		w.descriptorCount = 1;
		w.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		w.pBufferInfo = &bi;
		vkUpdateDescriptorSets(device, 1, &w, 0, nullptr);
	}

	std::vector<VkVertexInputBindingDescription> bindings;
	std::vector<VkVertexInputAttributeDescription> attrs;
	streams_.resize(config_.streamComponents.size());
	for (uint32_t b = 0; b < config_.streamComponents.size(); ++b) {
		const uint32_t comps = config_.streamComponents[b];
		bindings.push_back({ b, static_cast<uint32_t>(sizeof(float) * comps), VK_VERTEX_INPUT_RATE_VERTEX });
		attrs.push_back({ b, b, floatFormat(comps), 0 });
		streams_[b].init(framesInFlight, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
	}

	PipelineConfig pCfg{};
	pCfg.vertSpv = config_.vertSpv;
	pCfg.fragSpv = config_.fragSpv;
	pCfg.bindingDescs = bindings;
	pCfg.attrDescs = attrs;
	pCfg.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
	pCfg.descriptorSetLayout = descriptorSetLayout_.get();
	pCfg.cullMode = VK_CULL_MODE_NONE;
	pCfg.depthTest = config_.depthTest;
	pCfg.depthWrite = config_.depthWrite;
	pCfg.blendEnable = (config_.blend != Blend::Opaque);
	pCfg.additiveBlend = (config_.blend == Blend::Additive);
	pCfg.premultipliedAlphaBlend = (config_.blend == Blend::Premultiplied);
	pCfg.samples = config_.samples;
	return pipeline_.create(ctx, renderPass, pCfg);
}

void FlamePointPipeline::destroy(VkDevice device)
{
	for (auto& ub : uniformBuffers_) {
		ub.destroy(device);
	}
	uniformBuffers_.clear();
	for (auto& s : streams_) {
		s.destroy();
	}
	streams_.clear();
	pipeline_.destroy(device);
	descriptorPool_.destroy(device);
	descriptorSetLayout_.destroy(device);
	counts_.clear();
}

void FlamePointPipeline::upload(const VulkanContext& ctx, uint32_t frameIndex, uint32_t count,
	const std::vector<const float*>& streams, const FlamePointUBO& ubo)
{
	if (frameIndex >= counts_.size()) {
		return;
	}
	counts_[frameIndex] = 0;
	if (count == 0 || streams.size() != streams_.size()) {
		return;
	}
	for (size_t b = 0; b < streams_.size(); ++b) {
		const VkDeviceSize bytes = VkDeviceSize(count) * config_.streamComponents[b] * sizeof(float);
		if (!streams[b] || !streams_[b].write(ctx, frameIndex, streams[b], bytes)) {
			return;
		}
	}
	uniformBuffers_[frameIndex].write(&ubo, sizeof(ubo));
	counts_[frameIndex] = count;
}

void FlamePointPipeline::uploadUniforms(uint32_t frameIndex, const FlamePointUBO& ubo)
{
	if (frameIndex < uniformBuffers_.size()) {
		uniformBuffers_[frameIndex].write(&ubo, sizeof(ubo));
	}
}

void FlamePointPipeline::renderIndirect(VkCommandBuffer cmd, uint32_t frameIndex, const std::vector<VkBuffer>& vertexBuffers,
	VkBuffer indirectBuffer, VkDeviceSize indirectOffset) const
{
	if (frameIndex >= descriptorSets_.size() || vertexBuffers.size() != config_.streamComponents.size() || !isValid()) {
		return;
	}
	vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.getPipeline());
	std::vector<VkDeviceSize> offsets(vertexBuffers.size(), 0);
	vkCmdBindVertexBuffers(cmd, 0, static_cast<uint32_t>(vertexBuffers.size()), vertexBuffers.data(), offsets.data());
	vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.getLayout(), 0, 1,
		&descriptorSets_[frameIndex], 0, nullptr);
	vkCmdDrawIndirect(cmd, indirectBuffer, indirectOffset, 1, sizeof(VkDrawIndirectCommand));
}

void FlamePointPipeline::render(VkCommandBuffer cmd, uint32_t frameIndex) const
{
	if (frameIndex >= counts_.size() || counts_[frameIndex] == 0) {
		return;
	}
	vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.getPipeline());

	std::vector<VkBuffer> vbufs(streams_.size());
	std::vector<VkDeviceSize> offsets(streams_.size(), 0);
	for (size_t b = 0; b < streams_.size(); ++b) {
		vbufs[b] = streams_[b].get(frameIndex);
	}
	vkCmdBindVertexBuffers(cmd, 0, static_cast<uint32_t>(vbufs.size()), vbufs.data(), offsets.data());
	vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.getLayout(), 0, 1,
		&descriptorSets_[frameIndex], 0, nullptr);
	vkCmdDraw(cmd, counts_[frameIndex], 1, 0, 0);
}

} // namespace Phantom
