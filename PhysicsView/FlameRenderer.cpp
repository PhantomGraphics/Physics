#include "FlameRenderer.h"

#include "FlameBlackbody.h"
#include "FlameSmokeShadow.h"

#include "CGLib/VulkanGraphics/VulkanCommandPool.h"
#include "CGLib/VulkanGraphics/VulkanContext.h"

#include <cmath>
#include <cstdio>

using namespace Phantom::VKG;

namespace Phantom {

bool FlameRenderer::Shading::operator==(const Shading& o) const
{
	return ambientTemperature == o.ambientTemperature && referenceTemperature == o.referenceTemperature &&
		exposure == o.exposure && whiteBalanceTemperature == o.whiteBalanceTemperature &&
		whiteBalanceDegree == o.whiteBalanceDegree && smokeExtinction == o.smokeExtinction && smokeGlow == o.smokeGlow &&
		smokeAlbedo == o.smokeAlbedo && pbvrSubdivision == o.pbvrSubdivision &&
		pbvrMinSubPixels == o.pbvrMinSubPixels && pbvrDensityScale == o.pbvrDensityScale &&
		smokeShadowStrength == o.smokeShadowStrength && smokeShadowAmbient == o.smokeShadowAmbient &&
		smokeLightDirection == o.smokeLightDirection && smokeLightRadiance == o.smokeLightRadiance && smokeFlameLight == o.smokeFlameLight;
}

void FlameRenderer::setEmitters(std::vector<float> positions, std::vector<float> temperatures, std::vector<float> sizes)
{
	emitPositions_ = std::move(positions);
	emitTemperatures_ = std::move(temperatures);
	emitSizes_ = std::move(sizes);
}

void FlameRenderer::setAbsorbers(std::vector<float> positions, std::vector<float> densities, std::vector<float> sizes,
	std::vector<float> temperatures)
{
	if (absPositions_ != positions || absDensities_ != densities || absSizes_ != sizes) smokeShadowDirty_ = true;
	absPositions_ = std::move(positions);
	absDensities_ = std::move(densities);
	absSizes_ = std::move(sizes);
	absTemperatures_ = std::move(temperatures);
}

void FlameRenderer::onInit(VulkanContext& ctx, const VulkanCommandPool& pool,
	VkRenderPass renderPass, uint32_t framesInFlight)
{
	ctx_ = &ctx;

	FlamePointPipeline::Config flameCfg;
	flameCfg.vertSpv = shaders_.flameVert;
	flameCfg.fragSpv = shaders_.flameFrag;
	flameCfg.streamComponents = { 3, 1, 1 }; // position, temperature, size
	flameCfg.blend = FlamePointPipeline::Blend::Additive;
	flameCfg.depthWrite = false; // emission never occludes
	flamePipeline_.emplace(std::move(flameCfg));
	flamePipeline_->create(ctx, renderPass, framesInFlight);

	FlamePointPipeline::Config smokeCfg;
	smokeCfg.vertSpv = shaders_.smokeVert;
	smokeCfg.fragSpv = shaders_.smokeFrag;
	smokeCfg.streamComponents = { 3, 1, 1, 1 }; // position, density, size, temperature
	smokeCfg.blend = FlamePointPipeline::Blend::Premultiplied; // Beer-Lambert absorption + glow
	smokeCfg.depthWrite = false;
	smokePipeline_.emplace(std::move(smokeCfg));
	smokePipeline_->create(ctx, renderPass, framesInFlight);

	// Real size arrives with resize() once the HDR target exists.
	if (!pbvr_.create(ctx, pool, renderPass, framesInFlight, shaders_.pbvr, 1280, 720)) {
		std::fprintf(stderr, "[FlameRenderer] PBVR pass creation failed; PBVR mode will draw nothing\n");
	}
	if (!haze_.create(ctx, renderPass, depthFormat_, framesInFlight, shaders_.haze, 1280, 720)) {
		std::fprintf(stderr, "[FlameRenderer] heat-haze pass creation failed; haze disabled\n");
	}
}

void FlameRenderer::resize(uint32_t width, uint32_t height)
{
	if (ctx_ && pbvr_.isValid()) {
		pbvr_.resize(*ctx_, width, height);
	}
	if (ctx_ && haze_.isValid()) {
		haze_.resize(*ctx_, width, height);
	}
}

void FlameRenderer::recordHazeField(VkCommandBuffer cmd, uint32_t frameIndex)
{
	if (hazeActive_) {
		haze_.recordField(cmd, frameIndex);
	}
}

FlamePointUBO FlameRenderer::makeUBO()
{
	FlamePointUBO ubo;
	ubo.mvp = proj_ * view_;
	// Vulkan projections often flip Y (negative [1][1]); only the scale matters here.
	ubo.view = glm::vec4(std::abs(proj_[1][1]), viewportHeight_, shading_.pbvrMinSubPixels, shading_.pbvrDensityScale);
	ubo.thermal = glm::vec4(shading_.ambientTemperature, shading_.referenceTemperature, shading_.exposure, 0.0f);
	ubo.smoke = glm::vec4(shading_.smokeExtinction, shading_.smokeGlow, shading_.pbvrSubdivision, 0.0f);
	ubo.smokeAlbedo = glm::vec4(shading_.smokeAlbedo, 0.0f);
	const float lightLength = glm::length(shading_.smokeLightDirection);
	const glm::vec3 toLight = lightLength > 1.0e-6f ? -shading_.smokeLightDirection / lightLength : glm::vec3(0, 1, 0);
	ubo.smokeShadow = glm::vec4(toLight, shading_.smokeShadowStrength);
	ubo.smokeShadowLight = glm::vec4(shading_.smokeLightRadiance, shading_.smokeShadowAmbient);
	ubo.flameLightPosition.w = shading_.smokeFlameLight;
	ubo.lutRange = glm::vec4(FlameBlackbody::kLutMinT, FlameBlackbody::kLutMaxT, 0.0f, 0.0f);
	if (shading_.whiteBalanceTemperature != lutWhite_ || shading_.whiteBalanceDegree != lutDegree_) {
		lut_ = FlameBlackbody::makeLut(shading_.whiteBalanceTemperature, shading_.whiteBalanceDegree);
		lutWhite_ = shading_.whiteBalanceTemperature;
		lutDegree_ = shading_.whiteBalanceDegree;
	}
	for (int i = 0; i < FlamePointUBO::kLutSize; ++i) {
		ubo.lut[i] = lut_[i];
	}
	return ubo;
}

void FlameRenderer::onUpdate(uint32_t frameIndex)
{
	pbvrRecordedThisFrame_ = false;
	hazeActive_ = false;
	if (!enabled_ || !ctx_ || !flamePipeline_ || !smokePipeline_) {
		pbvrWasActive_ = false;
		return;
	}
	const FlamePointUBO ubo = makeUBO();
	const auto nEmit = static_cast<uint32_t>(emitPositions_.size() / 3);

	// Heat haze refracts what is behind the flame, in both render modes.
	if (hazeSettings_.enabled && haze_.isValid() && nEmit > 0 && hazeSettings_.strength > 0.0f) {
		haze_.update(*ctx_, frameIndex, ubo, hazeSettings_, nEmit, emitPositions_.data(),
			emitTemperatures_.data(), emitSizes_.data(), hazeTime_);
		hazeActive_ = true;
	}

	if (renderMode_ == RenderMode::PBVR) {
		const auto nAbs = absPositions_.size() / 3;
		if (smokeShadowDirty_) {
			smokeShadowDepth_.assign(nAbs, 0.0f);
			if (shading_.smokeShadowStrength > 0.0f) {
				std::vector<FlameSmokeShadow::Puff> puffs;
				puffs.reserve(nAbs);
				for (size_t i = 0; i < nAbs; ++i) {
					puffs.push_back({glm::vec3(absPositions_[i*3], absPositions_[i*3+1], absPositions_[i*3+2]),
						absSizes_[i], shading_.smokeExtinction * absDensities_[i] * shading_.pbvrDensityScale});
				}
				FlameSmokeShadow shadow;
				shadow.build(std::move(puffs));
				for (size_t i = 0; i < nAbs; ++i) smokeShadowDepth_[i] = shadow.opticalDepth(static_cast<unsigned>(i), glm::vec3(ubo.smokeShadow));
			}
			smokeShadowDirty_ = false;
		}
		absPacked_.resize(nAbs * 8);
		for (size_t i = 0; i < nAbs; ++i) {
			float* d = &absPacked_[i * 8];
			d[0] = absPositions_[i * 3 + 0];
			d[1] = absPositions_[i * 3 + 1];
			d[2] = absPositions_[i * 3 + 2];
			d[3] = absSizes_[i];
			d[4] = absDensities_[i];
			d[5] = absTemperatures_[i];
			d[6] = smokeShadowDepth_[i]; // external light-ray optical depth, excluding this puff
			d[7] = 0.0f;
		}

		const auto now = std::chrono::steady_clock::now();
		const float dtMs = pbvrWasActive_
			? std::chrono::duration<float, std::milli>(now - lastFrameTime_).count() : 0.0f;
		lastFrameTime_ = now;

		// Any change to what an ensemble *would* look like invalidates the average.
		// So does a jump in the sim, and pausing: a static frame should converge
		// to exactly the paused state, not keep the EMA of the frames before it.
		const bool paused = wasAnimating_ && !animating_;
		// While the sim runs, the auto reference temperature follows the flame
		// every frame -- that is content motion for the EMA to absorb, not a
		// view change, so it must not restart the history (it would pin the
		// LOD controller at R=1 in its Moving state).
		Shading compare = shading_;
		if (animating_) {
			compare.referenceTemperature = lastShading_.referenceTemperature;
			compare.whiteBalanceTemperature = lastShading_.whiteBalanceTemperature; // auto WB follows it
		}
		const bool reset = !pbvrWasActive_ || ubo.mvp != lastMvp_ || !(compare == lastShading_) ||
			viewportHeight_ != lastViewportHeight_ || jumped_ || paused;
		lastMvp_ = ubo.mvp;
		lastShading_ = shading_;
		lastViewportHeight_ = viewportHeight_;
		pbvrWasActive_ = true;

		pbvr_.update(*ctx_, frameIndex, ubo, absPacked_, nEmit, emitPositions_.data(), emitTemperatures_.data(),
			emitSizes_.data(), reset, animating_ && !jumped_, dtMs);
		wasAnimating_ = animating_ && !jumped_;
		animating_ = false;
		jumped_ = false;
		return;
	}
	pbvrWasActive_ = false;
	animating_ = false;
	jumped_ = false;
	wasAnimating_ = false;

	flamePipeline_->upload(*ctx_, frameIndex, nEmit,
		{ emitPositions_.data(), emitTemperatures_.data(), emitSizes_.data() }, ubo);
	const auto nAbs = static_cast<uint32_t>(absPositions_.size() / 3);
	smokePipeline_->upload(*ctx_, frameIndex, nAbs,
		{ absPositions_.data(), absDensities_.data(), absSizes_.data(), absTemperatures_.data() }, ubo);
}

void FlameRenderer::recordPreRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
	if (!enabled_ || renderMode_ != RenderMode::PBVR || !pbvr_.isValid()) {
		return;
	}
	pbvr_.record(cmd, frameIndex);
	pbvrRecordedThisFrame_ = true;
}

void FlameRenderer::onRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
	if (!enabled_) {
		return;
	}

	if (renderMode_ == RenderMode::PBVR) {
		if (pbvrRecordedThisFrame_) {
			pbvr_.composite(cmd, frameIndex);
		}
		return;
	}

	// Absorbers first (premultiplied over the background), then the additive
	// emitters on top -- see the class doc comment.
	if (smokePipeline_ && smokePipeline_->isValid()) {
		smokePipeline_->render(cmd, frameIndex);
	}
	if (flamePipeline_ && flamePipeline_->isValid()) {
		flamePipeline_->render(cmd, frameIndex);
	}
}

void FlameRenderer::onCleanup(VkDevice device)
{
	for (auto* p : { &flamePipeline_, &smokePipeline_ }) {
		if (*p) {
			(*p)->destroy(device);
			p->reset();
		}
	}
	if (ctx_) {
		haze_.destroy(*ctx_);
		pbvr_.destroy(*ctx_);
	}
}

} // namespace Phantom
