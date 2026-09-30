#include "FlameHazePass.h"

#include "CGLib/VulkanGraphics/VulkanContext.h"

#include <algorithm>

using namespace Phantom::VKG;

namespace Phantom {

namespace {
constexpr VkFormat kColorFormat = VK_FORMAT_R16G16B16A16_SFLOAT; // = FluidApp's hdrScene_
}

bool FlameHazePass::create(const VulkanContext& ctx, VkRenderPass hdrRenderPass, VkFormat depthFormat,
	uint32_t framesInFlight, const Shaders& shaders, uint32_t width, uint32_t height)
{
	depthFormat_ = depthFormat;
	if (!sampler_.create(ctx.getDevice(), VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE)) {
		return false;
	}
	if (!createTargets(ctx, std::max(1u, width), std::max(1u, height))) {
		return false;
	}

	FlamePointPipeline::Config fieldCfg;
	fieldCfg.vertSpv = shaders.fieldVert;
	fieldCfg.fragSpv = shaders.fieldFrag;
	fieldCfg.streamComponents = { 3, 1, 1 }; // position, temperature, size
	fieldCfg.blend = FlamePointPipeline::Blend::Additive;
	fieldCfg.depthTest = false;
	fieldCfg.depthWrite = false;
	fieldPipeline_.emplace(std::move(fieldCfg));
	if (!fieldPipeline_->create(ctx, field_.getRenderPass(), framesInFlight)) {
		return false;
	}

	FlameFullscreenPass::Config applyCfg;
	applyCfg.vertSpv = shaders.fullscreenVert;
	applyCfg.fragSpv = shaders.applyFrag;
	applyCfg.imageCount = 3; // background colour, background depth, haze field
	applyCfg.setCount = 1;
	applyCfg.pushConstantSize = sizeof(float) * 4;
	applyCfg.writeDepth = true;
	if (!apply_.create(ctx, hdrRenderPass, applyCfg)) {
		return false;
	}
	writeImageDescriptors(ctx.getDevice());
	valid_ = true;
	return true;
}

bool FlameHazePass::createTargets(const VulkanContext& ctx, uint32_t width, uint32_t height)
{
	const uint32_t fw = std::max(1u, width / kFieldDownscale);
	const uint32_t fh = std::max(1u, height / kFieldDownscale);
	return field_.create(ctx, fw, fh, kColorFormat, depthFormat_) &&
		background_.create(ctx, width, height, kColorFormat, depthFormat_);
}

void FlameHazePass::writeImageDescriptors(VkDevice device)
{
	apply_.setImages(device, 0,
		{ background_.getColorImageView(), background_.getDepthImageView(), field_.getColorImageView() },
		sampler_.get(),
		{ VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL,
		  VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL });
}

bool FlameHazePass::resize(const VulkanContext& ctx, uint32_t width, uint32_t height)
{
	if (!valid_ || width == 0 || height == 0) {
		return false;
	}
	// Device is idle (swapchain recreation); resize() keeps the render passes.
	const bool ok = field_.resize(ctx, std::max(1u, width / kFieldDownscale), std::max(1u, height / kFieldDownscale)) &&
		background_.resize(ctx, width, height);
	if (!ok) {
		return false;
	}
	writeImageDescriptors(ctx.getDevice());
	return true;
}

void FlameHazePass::destroy(const VulkanContext& ctx)
{
	VkDevice device = ctx.getDevice();
	apply_.destroy(device);
	if (fieldPipeline_) {
		fieldPipeline_->destroy(device);
		fieldPipeline_.reset();
	}
	sampler_.destroy(device);
	field_.destroy(ctx);
	background_.destroy(ctx);
	valid_ = false;
}

void FlameHazePass::update(const VulkanContext& ctx, uint32_t frameIndex, const FlamePointUBO& ubo,
	const Settings& settings, uint32_t count, const float* positions, const float* temperatures,
	const float* sizes, float timeSeconds)
{
	if (!valid_) {
		return;
	}
	FlamePointUBO u = ubo;
	u.view.y = static_cast<float>(field_.getExtent().height); // point sizes are in field pixels
	u.smoke.w = settings.extent;
	fieldPipeline_->upload(ctx, frameIndex, count, { positions, temperatures, sizes }, u);
	pushA_ = { settings.strength, timeSeconds, settings.frequency, settings.riseSpeed };
}

void FlameHazePass::recordField(VkCommandBuffer cmd, uint32_t frameIndex)
{
	if (!valid_) {
		return;
	}
	field_.beginRenderPass(cmd, { 0.0f, 0.0f, 0.0f, 0.0f }, 1.0f);
	fieldPipeline_->render(cmd, frameIndex);
	field_.endRenderPass(cmd);
}

void FlameHazePass::beginBackground(VkCommandBuffer cmd, const std::array<float, 4>& clear) const
{
	background_.beginRenderPass(cmd, clear, 1.0f);
}

void FlameHazePass::endBackground(VkCommandBuffer cmd) const
{
	background_.endRenderPass(cmd);
}

void FlameHazePass::apply(VkCommandBuffer cmd) const
{
	apply_.draw(cmd, 0, pushA_.data());
}

} // namespace Phantom
