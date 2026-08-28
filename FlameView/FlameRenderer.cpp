#include "FlameRenderer.h"

#include "CGLib/VulkanGraphics/VulkanContext.h"
#include "CGLib/VulkanGraphics/VulkanCommandPool.h"

using namespace Phantom::VKG;

namespace FlameView {

void FlameRenderer::setShaders(std::vector<uint32_t> vertSpv, std::vector<uint32_t> fragSpv)
{
	vertSpv_ = std::move(vertSpv);
	fragSpv_ = std::move(fragSpv);
}

void FlameRenderer::setSmokeShaders(std::vector<uint32_t> vertSpv, std::vector<uint32_t> fragSpv)
{
	smokeVertSpv_ = std::move(vertSpv);
	smokeFragSpv_ = std::move(fragSpv);
}

void FlameRenderer::setPBVRShaders(std::vector<uint32_t> vertSpv, std::vector<uint32_t> fragSpv)
{
	pbvrVertSpv_ = std::move(vertSpv);
	pbvrFragSpv_ = std::move(fragSpv);
}

void FlameRenderer::setParticles(std::vector<float> positions, std::vector<float> temperatures, std::vector<float> sizes)
{
	positions_ = std::move(positions);
	temperatures_ = std::move(temperatures);
	sizes_ = std::move(sizes);
}

void FlameRenderer::setSmokeParticles(std::vector<float> positions, std::vector<float> opacities, std::vector<float> sizes,
	std::vector<float> temperatures)
{
	smokePositions_ = std::move(positions);
	smokeOpacities_ = std::move(opacities);
	smokeSizes_ = std::move(sizes);
	smokeTemperatures_ = std::move(temperatures);
}

void FlameRenderer::setPBVRParticles(std::vector<float> positions, std::vector<float> colors, std::vector<float> sizes)
{
	pbvrPositions_ = std::move(positions);
	pbvrColors_ = std::move(colors);
	pbvrSizes_ = std::move(sizes);
}

void FlameRenderer::onInit(VulkanContext& ctx, const VulkanCommandPool& pool,
	VkRenderPass renderPass, uint32_t framesInFlight)
{
	ctx_ = &ctx;
	pool_ = &pool;

	FlamePipeline::Config cfg;
	cfg.vertSpv = vertSpv_;
	cfg.fragSpv = fragSpv_;
	pipeline_.emplace(std::move(cfg));
	pipeline_->create(ctx, pool, renderPass, framesInFlight);

	FlameSmokePipeline::Config smokeCfg;
	smokeCfg.vertSpv = smokeVertSpv_;
	smokeCfg.fragSpv = smokeFragSpv_;
	smokePipeline_.emplace(std::move(smokeCfg));
	smokePipeline_->create(ctx, pool, renderPass, framesInFlight);

	FlamePBVRPipeline::Config pbvrCfg;
	pbvrCfg.vertSpv = pbvrVertSpv_;
	pbvrCfg.fragSpv = pbvrFragSpv_;
	pbvrPipeline_.emplace(std::move(pbvrCfg));
	pbvrPipeline_->create(ctx, pool, renderPass, framesInFlight);
}

void FlameRenderer::onUpdate(uint32_t /*frameIndex*/)
{
	if (!pipeline_ || !smokePipeline_ || !pbvrPipeline_ || !ctx_ || !pool_) {
		return;
	}

	if (renderMode_ == RenderMode::PBVR) {
		FlamePBVRPipeline::Buffer pbvrBuffer;
		pbvrBuffer.positions = pbvrPositions_;
		pbvrBuffer.colors = pbvrColors_;
		pbvrBuffer.sizes = pbvrSizes_;
		pbvrBuffer.mvp = proj_ * view_;
		pbvrPipeline_->upload(*ctx_, *pool_, pbvrBuffer);
		return;
	}

	FlamePipeline::Buffer buffer;
	buffer.positions = positions_;
	buffer.temperatures = temperatures_;
	buffer.sizes = sizes_;
	buffer.mvp = proj_ * view_;
	buffer.pointSize = pointSize_;
	buffer.tMin = tMin_;
	buffer.tMax = tMax_;
	pipeline_->upload(*ctx_, *pool_, buffer);

	FlameSmokePipeline::Buffer smokeBuffer;
	smokeBuffer.positions = smokePositions_;
	smokeBuffer.opacities = smokeOpacities_;
	smokeBuffer.sizes = smokeSizes_;
	smokeBuffer.temperatures = smokeTemperatures_;
	smokeBuffer.mvp = proj_ * view_;
	smokeBuffer.pointSize = smokePointSize_;
	smokeBuffer.tMin = tMin_;
	smokeBuffer.tMax = tMax_;
	smokePipeline_->upload(*ctx_, *pool_, smokeBuffer);
}

void FlameRenderer::onRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
	if (renderMode_ == RenderMode::PBVR) {
		if (pbvrPipeline_ && pbvrPipeline_->isValid()) {
			pbvrPipeline_->render(cmd, frameIndex);
		}
		return;
	}

	// Smoke first (alpha-blended, so draw order affects the result), then the
	// additive flame/sparks on top -- see the class doc comment.
	if (smokePipeline_ && smokePipeline_->isValid()) {
		smokePipeline_->render(cmd, frameIndex);
	}
	if (pipeline_ && pipeline_->isValid()) {
		pipeline_->render(cmd, frameIndex);
	}
}

void FlameRenderer::onCleanup(VkDevice device)
{
	if (pipeline_) {
		pipeline_->destroy(device);
		pipeline_.reset();
	}
	if (smokePipeline_) {
		smokePipeline_->destroy(device);
		smokePipeline_.reset();
	}
	if (pbvrPipeline_) {
		pbvrPipeline_->destroy(device);
		pbvrPipeline_.reset();
	}
}

} // namespace FlameView
