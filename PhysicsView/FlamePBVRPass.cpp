#include "FlamePBVRPass.h"

#include "CGLib/VulkanGraphics/VulkanCommandPool.h"
#include "CGLib/VulkanGraphics/VulkanContext.h"

#include <vk_mem_alloc.h>

#include <algorithm>
#include <cmath>
#include <cstring>

using namespace Phantom::VKG;

namespace Phantom {

namespace {

constexpr VkFormat kColorFormat = VK_FORMAT_R16G16B16A16_SFLOAT; // one ensemble
// The running average needs fp32: the progressive update mix(h, e, 1/(n+1))
// adds increments of ~|e - h|/n, which fall below half an fp16 ULP (h * 2^-12)
// once n reaches the low hundreds -- the fp16 average then silently stops
// converging (measured: an RMSE floor in run_flame_pbvr_evaluation.ps1).
constexpr VkFormat kHistoryFormat = VK_FORMAT_R32G32B32A32_SFLOAT;
constexpr uint32_t kGenerateLocalSize = 64;
constexpr uint32_t kFinalizeLocalSize = 8;
constexpr uint32_t kTimestampsPerFrame = 2;

struct GeneratePush {
	uint32_t sourceCount;
	uint32_t ensembleCount;
	uint32_t seedBase;
	uint32_t capacity;
	uint32_t maxPerSource;
};

struct FinalizePush {
	uint32_t ensembleCount;
	uint32_t capacity;
};

VkDescriptorSetLayoutBinding binding(uint32_t index, VkDescriptorType type)
{
	VkDescriptorSetLayoutBinding b{};
	b.binding = index;
	b.descriptorType = type;
	b.descriptorCount = 1;
	b.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
	return b;
}

void writeBuffer(VkDevice device, VkDescriptorSet set, uint32_t index, VkDescriptorType type, VkBuffer buffer)
{
	VkDescriptorBufferInfo bi{ buffer, 0, VK_WHOLE_SIZE };
	VkWriteDescriptorSet w{};
	w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
	w.dstSet = set;
	w.dstBinding = index;
	w.descriptorCount = 1;
	w.descriptorType = type;
	w.pBufferInfo = &bi;
	vkUpdateDescriptorSets(device, 1, &w, 0, nullptr);
}

void memoryBarrier(VkCommandBuffer cmd, VkPipelineStageFlags src, VkAccessFlags srcAccess,
	VkPipelineStageFlags dst, VkAccessFlags dstAccess)
{
	VkMemoryBarrier mb{};
	mb.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
	mb.srcAccessMask = srcAccess;
	mb.dstAccessMask = dstAccess;
	vkCmdPipelineBarrier(cmd, src, dst, 0, 1, &mb, 0, nullptr, 0, nullptr);
}

}

bool FlamePBVRPass::create(const VulkanContext& ctx, const VulkanCommandPool& pool, VkRenderPass hdrRenderPass,
	uint32_t framesInFlight, const Shaders& shaders, uint32_t width, uint32_t height)
{
	framesInFlight_ = framesInFlight;
	VkDevice device = ctx.getDevice();

	if (!sampler_.create(device, VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE)) {
		return false;
	}
	if (!createTargets(ctx, std::max(1u, width), std::max(1u, height))) {
		return false;
	}

	// ---- Graphics pipelines ---------------------------------------------------
	FlamePointPipeline::Config pointCfg;
	pointCfg.vertSpv = shaders.pointVert;
	pointCfg.fragSpv = shaders.pointFrag;
	pointCfg.streamComponents = { 4, 4 }; // posSize, color (GPU-written)
	pointCfg.blend = FlamePointPipeline::Blend::Opaque;
	pointCfg.depthWrite = true;
	pointPipeline_.emplace(std::move(pointCfg));
	if (!pointPipeline_->create(ctx, ensemble_.getRenderPass(), framesInFlight)) {
		return false;
	}

	FlamePointPipeline::Config emitCfg;
	emitCfg.vertSpv = shaders.emissiveVert;
	emitCfg.fragSpv = shaders.emissiveFrag;
	emitCfg.streamComponents = { 3, 1, 1 };
	emitCfg.blend = FlamePointPipeline::Blend::Additive;
	emitCfg.depthWrite = false; // occluded by this ensemble's opaque smoke, never occluding
	emissivePipeline_.emplace(std::move(emitCfg));
	if (!emissivePipeline_->create(ctx, ensemble_.getRenderPass(), framesInFlight)) {
		return false;
	}

	FlameFullscreenPass::Config blendCfg;
	blendCfg.vertSpv = shaders.fullscreenVert;
	blendCfg.fragSpv = shaders.blendFrag;
	blendCfg.imageCount = 2;
	blendCfg.setCount = 2; // set k: (ensemble, history_[k]) -> written into history_[1-k]
	blendCfg.pushConstantSize = sizeof(float);
	// history_[0] and history_[1] have identical formats, so their render
	// passes are compatible and one pipeline serves both.
	if (!blend_.create(ctx, history_[0].getRenderPass(), blendCfg)) {
		return false;
	}

	FlameFullscreenPass::Config compCfg;
	compCfg.vertSpv = shaders.fullscreenVert;
	compCfg.fragSpv = shaders.compositeFrag;
	compCfg.imageCount = 1;
	compCfg.setCount = 2; // set k samples history_[k]
	compCfg.premultipliedBlend = true;
	if (!composite_.create(ctx, hdrRenderPass, compCfg)) {
		return false;
	}
	writeImageDescriptors(device);

	// ---- Buffers ----------------------------------------------------------------
	const VkDeviceSize outBytes = VkDeviceSize(kMaxEnsembles) * kCapacityPerEnsemble * sizeof(float) * 4;
	if (!outPos_.create(ctx, pool, outBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT) ||
		!outColor_.create(ctx, pool, outBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT)) {
		return false;
	}
	const std::vector<uint32_t> zeros(kMaxEnsembles * 4, 0u);
	if (!counters_.create(ctx, pool, kMaxEnsembles * sizeof(uint32_t), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, zeros.data()) ||
		!args_.create(ctx, pool, kMaxEnsembles * sizeof(VkDrawIndirectCommand),
			VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT, zeros.data())) {
		return false;
	}
	sources_.init(framesInFlight, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
	computeUbo_.resize(framesInFlight);
	genStats_.resize(framesInFlight);
	for (uint32_t f = 0; f < framesInFlight; ++f) {
		if (!computeUbo_[f].createMapped(ctx, sizeof(FlamePointUBO), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT) ||
			!genStats_[f].createMapped(ctx, kMaxEnsembles * sizeof(uint32_t), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT)) {
			return false;
		}
		std::memset(genStats_[f].getMapped(), 0, kMaxEnsembles * sizeof(uint32_t));
	}

	// ---- Compute: generate ----------------------------------------------------
	genLayout_.create(device, {
		binding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER),
		binding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
		binding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
		binding(3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
		binding(4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER) });
	genPool_.create(device, {
		{ VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, framesInFlight },
		{ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, framesInFlight * 4 } }, framesInFlight);
	genSets_ = genPool_.allocateSets(device, std::vector<VkDescriptorSetLayout>(framesInFlight, genLayout_.get()));
	if (genSets_.size() != framesInFlight) {
		return false;
	}
	genBoundSources_.assign(framesInFlight, VK_NULL_HANDLE);
	for (uint32_t f = 0; f < framesInFlight; ++f) {
		writeBuffer(device, genSets_[f], 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, computeUbo_[f].getBuffer());
		// Binding 1 (sources) is written in update() once the ring slot exists.
		writeBuffer(device, genSets_[f], 2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, outPos_.getBuffer());
		writeBuffer(device, genSets_[f], 3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, outColor_.getBuffer());
		writeBuffer(device, genSets_[f], 4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, counters_.getBuffer());
	}
	ComputePipelineConfig genCfg;
	genCfg.compSpv = shaders.generateComp;
	genCfg.descriptorSetLayout = genLayout_.get();
	genCfg.pushConstantRange = { VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(GeneratePush) };
	if (!genPipeline_.create(ctx, genCfg)) {
		return false;
	}

	// ---- Compute: finalize ----------------------------------------------------
	finLayout_.create(device, {
		binding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
		binding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
		binding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER) });
	finPool_.create(device, { { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, framesInFlight * 3 } }, framesInFlight);
	finSets_ = finPool_.allocateSets(device, std::vector<VkDescriptorSetLayout>(framesInFlight, finLayout_.get()));
	if (finSets_.size() != framesInFlight) {
		return false;
	}
	for (uint32_t f = 0; f < framesInFlight; ++f) {
		writeBuffer(device, finSets_[f], 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, counters_.getBuffer());
		writeBuffer(device, finSets_[f], 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, args_.getBuffer());
		writeBuffer(device, finSets_[f], 2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, genStats_[f].getBuffer());
	}
	ComputePipelineConfig finCfg;
	finCfg.compSpv = shaders.finalizeComp;
	finCfg.descriptorSetLayout = finLayout_.get();
	finCfg.pushConstantRange = { VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(FinalizePush) };
	if (!finPipeline_.create(ctx, finCfg)) {
		return false;
	}

	// ---- GPU timestamps (optional) ---------------------------------------------
	tsPeriodNs_ = ctx.getTimestampPeriodNs();
	tsWritten_.assign(framesInFlight, false);
	ensemblesRecorded_.assign(framesInFlight, 0);
	if (tsPeriodNs_ > 0.0f) {
		VkQueryPoolCreateInfo qi{ VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO };
		qi.queryType = VK_QUERY_TYPE_TIMESTAMP;
		qi.queryCount = framesInFlight * kTimestampsPerFrame;
		if (vkCreateQueryPool(device, &qi, nullptr, &queryPool_) != VK_SUCCESS) {
			queryPool_ = VK_NULL_HANDLE;
		}
	}
	stats_.timestampsSupported = (queryPool_ != VK_NULL_HANDLE);

	valid_ = true;
	return true;
}

bool FlamePBVRPass::createTargets(const VulkanContext& ctx, uint32_t width, uint32_t height)
{
	if (!ensemble_.create(ctx, width, height, kColorFormat, depthFormat_)) {
		return false;
	}
	for (auto& h : history_) {
		if (!h.create(ctx, width, height, kHistoryFormat, depthFormat_)) {
			return false;
		}
	}
	return true;
}

void FlamePBVRPass::destroyTargets(const VulkanContext& ctx)
{
	ensemble_.destroy(ctx);
	for (auto& h : history_) {
		h.destroy(ctx);
	}
}

void FlamePBVRPass::writeImageDescriptors(VkDevice device)
{
	for (uint32_t k = 0; k < 2; ++k) {
		blend_.setImages(device, k, { ensemble_.getColorImageView(), history_[k].getColorImageView() }, sampler_.get());
		composite_.setImages(device, k, { history_[k].getColorImageView() }, sampler_.get());
	}
}

bool FlamePBVRPass::resize(const VulkanContext& ctx, uint32_t width, uint32_t height)
{
	if (!valid_ || width == 0 || height == 0) {
		return false;
	}
	if (ensemble_.getExtent().width == width && ensemble_.getExtent().height == height) {
		return true;
	}
	// Device is idle (swapchain recreation). resize() keeps each render pass,
	// so every pipeline stays valid; only the image views change.
	bool ok = ensemble_.resize(ctx, width, height);
	for (auto& h : history_) {
		ok = h.resize(ctx, width, height) && ok;
	}
	if (!ok) {
		return false;
	}
	writeImageDescriptors(ctx.getDevice());
	samples_ = 0; // old history is gone
	historyNeedsInit_ = true;
	return true;
}

void FlamePBVRPass::destroy(const VulkanContext& ctx)
{
	VkDevice device = ctx.getDevice();
	if (queryPool_) {
		vkDestroyQueryPool(device, queryPool_, nullptr);
		queryPool_ = VK_NULL_HANDLE;
	}
	genPipeline_.destroy(device);
	finPipeline_.destroy(device);
	genPool_.destroy(device);
	finPool_.destroy(device);
	genLayout_.destroy(device);
	finLayout_.destroy(device);
	genSets_.clear();
	finSets_.clear();
	for (auto& b : computeUbo_) b.destroy(device);
	for (auto& b : genStats_) b.destroy(device);
	computeUbo_.clear();
	genStats_.clear();
	sources_.destroy();
	outPos_.destroy(device);
	outColor_.destroy(device);
	counters_.destroy(device);
	args_.destroy(device);
	if (pointPipeline_) { pointPipeline_->destroy(device); pointPipeline_.reset(); }
	if (emissivePipeline_) { emissivePipeline_->destroy(device); emissivePipeline_.reset(); }
	blend_.destroy(device);
	composite_.destroy(device);
	destroyTargets(ctx);
	sampler_.destroy(device);
	valid_ = false;
}

void FlamePBVRPass::update(const VulkanContext& ctx, uint32_t frameIndex, const FlamePointUBO& ubo,
	const std::vector<float>& absorbing, uint32_t emissiveCount, const float* emissivePos,
	const float* emissiveTemperature, const float* emissiveSize,
	bool resetHistory, bool animating, float dtMs)
{
	pendingR_ = 0;
	if (!valid_ || frameIndex >= framesInFlight_) {
		return;
	}

	// ---- Read back what this slot measured last time (its fence has signalled) ----
	if (tsWritten_[frameIndex] && queryPool_) {
		uint64_t ts[kTimestampsPerFrame] = {};
		if (vkGetQueryPoolResults(ctx.getDevice(), queryPool_, frameIndex * kTimestampsPerFrame, kTimestampsPerFrame,
				sizeof(ts), ts, sizeof(uint64_t), VK_QUERY_RESULT_64_BIT) == VK_SUCCESS && ts[1] >= ts[0]) {
			stats_.gpuMs = static_cast<float>(double(ts[1] - ts[0]) * tsPeriodNs_ * 1.0e-6);
		}
		tsWritten_[frameIndex] = false;
	}
	if (ensemblesRecorded_[frameIndex] > 0) {
		vmaInvalidateAllocation(ctx.getAllocator(), genStats_[frameIndex].getVmaAllocation(), 0, VK_WHOLE_SIZE);
		const auto* gen = static_cast<const uint32_t*>(genStats_[frameIndex].getMapped());
		uint32_t total = 0, over = 0;
		for (uint32_t e = 0; e < ensemblesRecorded_[frameIndex]; ++e) {
			total += gen[e];
			over += gen[e] > kCapacityPerEnsemble ? gen[e] - kCapacityPerEnsemble : 0;
		}
		stats_.generated = total;
		stats_.overflowed = over;
	}

	// ---- Upload this frame's inputs --------------------------------------------
	sourceCount_ = static_cast<uint32_t>(absorbing.size() / 8);
	if (sourceCount_ > 0) {
		if (!sources_.write(ctx, frameIndex, absorbing.data(), sourceCount_ * 8 * sizeof(float))) {
			sourceCount_ = 0;
		} else if (sources_.get(frameIndex) != genBoundSources_[frameIndex]) {
			// The ring slot grew (new VkBuffer): repoint this frame's set. Safe --
			// the set's last use was by this slot's previous, already-fenced frame.
			genBoundSources_[frameIndex] = sources_.get(frameIndex);
			writeBuffer(ctx.getDevice(), genSets_[frameIndex], 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
				genBoundSources_[frameIndex]);
		}
	}
	computeUbo_[frameIndex].write(&ubo, sizeof(ubo));
	pointPipeline_->uploadUniforms(frameIndex, ubo);
	emissivePipeline_->upload(ctx, frameIndex, emissiveCount, { emissivePos, emissiveTemperature, emissiveSize }, ubo);

	// ---- History / ensemble-count policy ---------------------------------------
	if (resetHistory) {
		samples_ = 0;
		lod_.notifyMotion();
	}

	uint32_t r = std::clamp(settings_.ensemblesPerFrame, 1u, kMaxEnsembles);
	uint32_t target = std::max(1u, settings_.targetEnsembles);
	if (settings_.lodMode == LodMode::Adaptive) {
		Phantom::Graphics::EnsembleLodController::Config cfg = lod_.config();
		cfg.rMax = kMaxEnsembles;
		cfg.targetMax = target;
		cfg.frameBudgetLowMs = settings_.budgetMs;
		cfg.frameBudgetHighMs = 2.0f * settings_.budgetMs;
		lod_.setConfig(cfg);
		// While animating, the EMA never "converges": report the effective
		// (capped, previous-frame) sample count so the controller keeps refining.
		const uint32_t shown = animating ? std::min(samples_, sampleCap_) : samples_;
		const auto req = lod_.advance(dtMs, stats_.gpuMs, stats_.timestampsSupported,
			animating && sampleCap_ != UINT32_MAX ? std::min(shown, target - 1) : shown);
		r = std::clamp(req.ensemblesPerFrame, 1u, kMaxEnsembles);
		stats_.lodState = static_cast<int>(lod_.state());
	}

	if (animating) {
		if (settings_.temporalFrames <= 0.0f) {
			samples_ = 0; // no temporal reuse: this frame's R ensembles only
			sampleCap_ = UINT32_MAX;
		} else {
			// EMA over ~temporalFrames frames: weights never drop below 1/(cap+1).
			sampleCap_ = std::max(1u, static_cast<uint32_t>(std::lround(settings_.temporalFrames * r)));
		}
	} else {
		sampleCap_ = UINT32_MAX; // exact progressive mean
		if (samples_ >= target) {
			r = 0; // converged: keep showing the history
		}
	}
	pendingR_ = r;
}

void FlamePBVRPass::record(VkCommandBuffer cmd, uint32_t frameIndex)
{
	stats_.ensemblesThisFrame = 0;
	if (!valid_ || frameIndex >= framesInFlight_) {
		return;
	}
	if (historyNeedsInit_) {
		for (auto& h : history_) {
			h.beginRenderPass(cmd, { 0.0f, 0.0f, 0.0f, 0.0f }, 1.0f);
			h.endRenderPass(cmd);
		}
		historyNeedsInit_ = false;
	}
	ensemblesRecorded_[frameIndex] = pendingR_;
	if (pendingR_ == 0) {
		stats_.displayedEnsembles = std::min(samples_, sampleCap_);
		return;
	}
	const uint32_t r = pendingR_;

	if (queryPool_) {
		vkCmdResetQueryPool(cmd, queryPool_, frameIndex * kTimestampsPerFrame, kTimestampsPerFrame);
		vkCmdWriteTimestamp(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, queryPool_, frameIndex * kTimestampsPerFrame);
	}

	// WAR: the previous frame's draws may still be reading the shared
	// sub-particle / indirect buffers this frame's compute overwrites.
	memoryBarrier(cmd,
		VK_PIPELINE_STAGE_VERTEX_INPUT_BIT | VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT,
		VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT | VK_ACCESS_INDIRECT_COMMAND_READ_BIT,
		VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT);

	if (sourceCount_ > 0 && genBoundSources_[frameIndex] != VK_NULL_HANDLE) {
		vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, genPipeline_.getPipeline());
		vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, genPipeline_.getLayout(), 0, 1,
			&genSets_[frameIndex], 0, nullptr);
		const GeneratePush push{ sourceCount_, r, seed_, kCapacityPerEnsemble, std::max(1u, settings_.maxPerSource) };
		vkCmdPushConstants(cmd, genPipeline_.getLayout(), VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(push), &push);
		vkCmdDispatch(cmd, (sourceCount_ + kGenerateLocalSize - 1) / kGenerateLocalSize, r, 1);
		memoryBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT,
			VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT);
	}
	seed_ += kMaxEnsembles; // every frame draws fresh, independent ensembles

	// Always finalize (zero-count args when there were no sources) -- it also
	// resets the counters for the next frame.
	vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, finPipeline_.getPipeline());
	vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, finPipeline_.getLayout(), 0, 1,
		&finSets_[frameIndex], 0, nullptr);
	const FinalizePush fpush{ r, kCapacityPerEnsemble };
	vkCmdPushConstants(cmd, finPipeline_.getLayout(), VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(fpush), &fpush);
	vkCmdDispatch(cmd, 1, 1, 1);
	memoryBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT,
		VK_PIPELINE_STAGE_VERTEX_INPUT_BIT | VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT | VK_PIPELINE_STAGE_HOST_BIT,
		VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT | VK_ACCESS_INDIRECT_COMMAND_READ_BIT | VK_ACCESS_HOST_READ_BIT);

	const std::vector<VkBuffer> vbufs = { outPos_.getBuffer(), outColor_.getBuffer() };
	for (uint32_t e = 0; e < r; ++e) {
		if (e > 0) {
			// The render passes' external dependencies order colour attachment
			// vs. sampling; this adds the depth WAW between consecutive clears.
			memoryBarrier(cmd,
				VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT, VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
				VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT,
				VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);
		}
		ensemble_.beginRenderPass(cmd, { 0.0f, 0.0f, 0.0f, 0.0f }, 1.0f);
		pointPipeline_->renderIndirect(cmd, frameIndex, vbufs, args_.getBuffer(), e * sizeof(VkDrawIndirectCommand));
		emissivePipeline_->render(cmd, frameIndex);
		ensemble_.endRenderPass(cmd);

		const uint32_t dst = 1u - historyIndex_;
		const float weight = 1.0f / static_cast<float>(std::min(samples_, sampleCap_) + 1u);
		history_[dst].beginRenderPass(cmd, { 0.0f, 0.0f, 0.0f, 0.0f }, 1.0f);
		blend_.draw(cmd, historyIndex_, &weight);
		history_[dst].endRenderPass(cmd);
		historyIndex_ = dst;
		if (samples_ < UINT32_MAX) {
			++samples_;
		}
	}

	if (queryPool_) {
		vkCmdWriteTimestamp(cmd, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, queryPool_, frameIndex * kTimestampsPerFrame + 1);
		tsWritten_[frameIndex] = true;
	}
	stats_.ensemblesThisFrame = r;
	stats_.displayedEnsembles = std::min(samples_, sampleCap_);
}

void FlamePBVRPass::composite(VkCommandBuffer cmd, uint32_t /*frameIndex*/) const
{
	if (!valid_ || samples_ == 0) {
		return;
	}
	composite_.draw(cmd, historyIndex_);
}

} // namespace Phantom
