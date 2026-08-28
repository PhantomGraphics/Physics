#include "FlameSmokePipeline.h"

#include "CGLib/VulkanGraphics/VulkanContext.h"
#include "CGLib/VulkanGraphics/VulkanCommandPool.h"

using namespace Phantom::VKG;

namespace FlameView {

void FlameSmokePipeline::create(const VulkanContext& ctx,
	const VulkanCommandPool& pool,
	VkRenderPass renderPass,
	uint32_t framesInFlight)
{
	(void)pool;
	framesInFlight_ = framesInFlight;
	VkDevice device = ctx.getDevice();

	// --- Descriptor Set Layout (binding 0: UBO) ---
	VkDescriptorSetLayoutBinding uboBinding{};
	uboBinding.binding = 0;
	uboBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	uboBinding.descriptorCount = 1;
	uboBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

	descriptorSetLayout_.create(device, { uboBinding });

	// --- Descriptor Pool ---
	VkDescriptorPoolSize poolSize{ VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, framesInFlight };
	descriptorPool_.create(device, { poolSize }, framesInFlight);

	// --- Descriptor Sets (one per frame) ---
	std::vector<VkDescriptorSetLayout> layouts(framesInFlight, descriptorSetLayout_.get());
	descriptorSets_ = descriptorPool_.allocateSets(device, layouts);

	// --- Uniform Buffers (one per frame, persistently mapped) ---
	uniformBuffers_.resize(framesInFlight);
	for (auto& ub : uniformBuffers_) {
		ub.createMapped(ctx, sizeof(UBOData), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
	}

	for (uint32_t i = 0; i < framesInFlight; ++i) {
		VkDescriptorBufferInfo bi{};
		bi.buffer = uniformBuffers_[i].getBuffer();
		bi.offset = 0;
		bi.range = sizeof(UBOData);

		VkWriteDescriptorSet w{};
		w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		w.dstSet = descriptorSets_[i];
		w.dstBinding = 0;
		w.descriptorCount = 1;
		w.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		w.pBufferInfo = &bi;
		vkUpdateDescriptorSets(device, 1, &w, 0, nullptr);
	}

	// --- Pipeline: 4 separate vertex buffers (position, opacity, size, temperature) ---
	std::vector<VkVertexInputBindingDescription> bindings = {
		{ 0, sizeof(float) * 3, VK_VERTEX_INPUT_RATE_VERTEX }, // position
		{ 1, sizeof(float),     VK_VERTEX_INPUT_RATE_VERTEX }, // opacity
		{ 2, sizeof(float),     VK_VERTEX_INPUT_RATE_VERTEX }, // size
		{ 3, sizeof(float),     VK_VERTEX_INPUT_RATE_VERTEX }, // temperature
	};
	std::vector<VkVertexInputAttributeDescription> attrs = {
		{ 0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0 }, // position
		{ 1, 1, VK_FORMAT_R32_SFLOAT, 0 },        // opacity
		{ 2, 2, VK_FORMAT_R32_SFLOAT, 0 },        // size
		{ 3, 3, VK_FORMAT_R32_SFLOAT, 0 },        // temperature
	};

	PipelineConfig pCfg{};
	pCfg.vertSpv = config_.vertSpv;
	pCfg.fragSpv = config_.fragSpv;
	pCfg.bindingDescs = bindings;
	pCfg.attrDescs = attrs;
	pCfg.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
	pCfg.descriptorSetLayout = descriptorSetLayout_.get();
	pCfg.cullMode = VK_CULL_MODE_NONE;
	pCfg.depthTest = true;
	pCfg.depthWrite = false; // blended particles do not occlude each other
	pCfg.blendEnable = true;
	pCfg.additiveBlend = false; // standard src-alpha / one-minus-src-alpha, unlike FlamePipeline
	pCfg.samples = config_.samples;

	pipeline_.create(ctx, renderPass, pCfg);
}

void FlameSmokePipeline::destroy(VkDevice device)
{
	for (auto& ub : uniformBuffers_) {
		ub.destroy(device);
	}
	uniformBuffers_.clear();

	positionBuffer_.destroy(device);
	opacityBuffer_.destroy(device);
	sizeBuffer_.destroy(device);
	temperatureBuffer_.destroy(device);

	pipeline_.destroy(device);
	descriptorPool_.destroy(device);
	descriptorSetLayout_.destroy(device);

	particleCount_ = 0;
}

void FlameSmokePipeline::upload(const VulkanContext& ctx,
	const VulkanCommandPool& pool,
	const Buffer& buffer)
{
	const uint32_t n = static_cast<uint32_t>(buffer.positions.size() / 3);
	if (n == 0) {
		particleCount_ = 0;
		return;
	}

	VkDevice device = ctx.getDevice();

	positionBuffer_.destroy(device);
	opacityBuffer_.destroy(device);
	sizeBuffer_.destroy(device);
	temperatureBuffer_.destroy(device);

	positionBuffer_.create(ctx, pool,
		buffer.positions.size() * sizeof(float),
		VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
		buffer.positions.data());

	opacityBuffer_.create(ctx, pool,
		buffer.opacities.size() * sizeof(float),
		VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
		buffer.opacities.data());

	sizeBuffer_.create(ctx, pool,
		buffer.sizes.size() * sizeof(float),
		VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
		buffer.sizes.data());

	// An empty temperatures array (callers that don't track it) is filled with
	// tMin, i.e. the flat-smokeColor end of the gradient (see flame_smoke.frag).
	if (buffer.temperatures.size() == n) {
		temperatureBuffer_.create(ctx, pool, n * sizeof(float), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, buffer.temperatures.data());
	} else {
		const std::vector<float> flat(n, buffer.tMin);
		temperatureBuffer_.create(ctx, pool, n * sizeof(float), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, flat.data());
	}

	particleCount_ = n;

	UBOData ubo{ buffer.mvp, glm::vec4(buffer.smokeColor, 1.0f), buffer.pointSize, buffer.tMin, buffer.tMax, 0.0f };
	for (uint32_t i = 0; i < framesInFlight_; ++i) {
		uniformBuffers_[i].write(&ubo, sizeof(ubo));
	}
}

void FlameSmokePipeline::render(VkCommandBuffer cmd, uint32_t frameIndex)
{
	if (particleCount_ == 0) {
		return;
	}

	vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.getPipeline());

	VkBuffer vbufs[] = { positionBuffer_.getBuffer(), opacityBuffer_.getBuffer(), sizeBuffer_.getBuffer(), temperatureBuffer_.getBuffer() };
	VkDeviceSize offsets[] = { 0, 0, 0, 0 };
	vkCmdBindVertexBuffers(cmd, 0, 4, vbufs, offsets);

	vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
		pipeline_.getLayout(), 0, 1,
		&descriptorSets_[frameIndex], 0, nullptr);

	vkCmdDraw(cmd, particleCount_, 1, 0, 0);
}

} // namespace FlameView
