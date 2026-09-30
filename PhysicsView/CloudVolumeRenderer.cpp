#include "pch.h"
#include "CloudVolumeRenderer.h"

#include "CGLib/VulkanGraphics/VulkanContext.h"
#include "CGLib/VulkanGraphics/VulkanCommandPool.h"

#include <algorithm>

namespace Phantom {

bool CloudVolumeRenderer::createPbvr()
{
	Volume::VolumePbvrGpu::Config pc;
	pc.generateCompSpv = shaders_.pbvrGenerateComp;
	pc.pointVertSpv = shaders_.pbvrPointVert;
	pc.pointFragSpv = shaders_.pbvrPointFrag;
	pc.accumulateCompSpv = shaders_.pbvrAccumulateComp;
	pc.fullscreenVertSpv = shaders_.fullscreenVert;
	pc.compositeFragSpv = shaders_.pbvrCompositeFrag;
	pc.compositeRenderPass = renderPass_;
	pc.depthFormat = depthFormat_;
	pc.framesInFlight = framesInFlight_;
	pc.minDiameterPx = pbvrSettings_.minDiameterPx;
	pc.maxPerCell = pbvrSettings_.maxPerCell;
	if (!pbvr_.create(*ctx_, *pool_, pc)) return false;
	pbvrReady_ = true;
	if (width_ > 0 && height_ > 0) {
		if (!pbvr_.setViewport(*ctx_, *pool_, width_, height_)) pbvrReady_ = false;
	}
	if (pbvrReady_ && gpu_.hasGrid()) pbvr_.bindGrid(*ctx_, gpu_);
	return pbvrReady_;
}

void CloudVolumeRenderer::onInit(Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
                                 VkRenderPass renderPass, uint32_t framesInFlight)
{
	ctx_ = &ctx;
	pool_ = &pool;
	renderPass_ = renderPass;
	framesInFlight_ = framesInFlight;
	if (!Volume::Volume3DImage::isSupported(ctx)) {
		std::fprintf(stderr, "[Cloud] R32F 3D images are not supported here; the volume renderer is disabled\n");
		return;
	}
	Volume::VolumeRaymarchGpu::Config cfg;
	cfg.fullscreenVertSpv = shaders_.fullscreenVert;
	cfg.raymarchFragSpv = shaders_.raymarchFrag;
	cfg.sunTransmittanceCompSpv = shaders_.sunTransmittanceComp;
	cfg.renderPass = renderPass;
	cfg.framesInFlight = framesInFlight;
	ready_ = gpu_.create(ctx, pool, cfg);
	if (!ready_) {
		std::fprintf(stderr, "[Cloud] VolumeRaymarchGpu creation failed\n");
		return;
	}
	if (!createPbvr()) std::fprintf(stderr, "[Cloud] PBVR creation failed; only the raymarch view is available\n");
}

void CloudVolumeRenderer::onCleanup(VkDevice /*device*/)
{
	if (ctx_ && ready_) {
		vkDeviceWaitIdle(ctx_->getDevice());
		pbvr_.destroy(*ctx_);
		gpu_.destroy(*ctx_);
	}
	ready_ = false;
	pbvrReady_ = false;
}

void CloudVolumeRenderer::setMode(Mode m)
{
	if (m != mode_) pbvrRestart_ = true;
	mode_ = m;
}

void CloudVolumeRenderer::setPbvrSettings(const PbvrSettings& s)
{
	if (s.minDiameterPx != pbvrSettings_.minDiameterPx || s.maxPerCell != pbvrSettings_.maxPerCell) {
		pbvrRestart_ = true;
		if (ready_ && pbvrReady_) {
			// Diameter / per-cell limits are baked into the config: rebuild (rare, UI-driven).
			vkDeviceWaitIdle(ctx_->getDevice());
			pbvr_.destroy(*ctx_);
			pbvrSettings_ = s;
			pbvrReady_ = false;
			createPbvr();
			return;
		}
	}
	pbvrSettings_ = s;
}

void CloudVolumeRenderer::resize(uint32_t width, uint32_t height)
{
	if (width == width_ && height == height_) return;
	width_ = width;
	height_ = height;
	pbvrRestart_ = true;
	if (ready_ && pbvrReady_ && width > 0 && height > 0) {
		vkDeviceWaitIdle(ctx_->getDevice());
		pbvrReady_ = pbvr_.setViewport(*ctx_, *pool_, width, height);
		if (pbvrReady_ && gpu_.hasGrid()) pbvr_.bindGrid(*ctx_, gpu_);
	}
}

void CloudVolumeRenderer::setDensity(const Volume::ScalarGrid3D& density)
{
	pending_ = density;
	densityDirty_ = true;
}

void CloudVolumeRenderer::setScattering(const Volume::ScatteringParams& p)
{
	// Extinction / sun direction / step length change the sun-transmittance grid.
	if (p.extinction != scattering_.extinction || p.sunDirection != scattering_.sunDirection ||
	    p.stepLength != scattering_.stepLength) {
		sunDirty_ = true;
	}
	if (p.extinction != scattering_.extinction || p.albedo != scattering_.albedo || p.phaseG != scattering_.phaseG ||
	    p.sunDirection != scattering_.sunDirection || p.sunIrradiance != scattering_.sunIrradiance ||
	    p.ambient != scattering_.ambient) {
		pbvrRestart_ = true;
	}
	scattering_ = p;
}

void CloudVolumeRenderer::setCamera(const glm::mat4& proj, const glm::mat4& view, const glm::mat4& cloudToScene,
                                    uint32_t viewportHeight)
{
	camera_.invViewProj = glm::inverse(proj * view * cloudToScene);
	const glm::vec3 eyeScene = glm::vec3(glm::inverse(view)[3]);
	camera_.position = glm::vec3(glm::inverse(cloudToScene) * glm::vec4(eyeScene, 1.0f));

	Volume::VolumePbvrGpu::Camera pc;
	pc.viewProj = proj * view * cloudToScene;
	pc.position = camera_.position;
	const float h = static_cast<float>(std::max(1u, viewportHeight));
	pc.pixelAngle = 2.0f / (proj[1][1] * h);
	pc.projScalePx = proj[1][1] * h * 0.5f * glm::length(glm::vec3(cloudToScene[0]));
	if (hasCamera_ && pc.viewProj != pbvrCamera_.viewProj) pbvrRestart_ = true;
	pbvrCamera_ = pc;
	hasCamera_ = true;
}

void CloudVolumeRenderer::recordPreRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
	if (!ready_ || !enabled_) return;
	if (densityDirty_) {
		densityDirty_ = false;
		if (pending_.data().empty()) return;
		if (!gpu_.hasGrid() || !(gpu_.gridDesc() == pending_.desc())) {
			vkDeviceWaitIdle(ctx_->getDevice());   // images are replaced: nothing may still reference them
			if (!gpu_.setGrid(*ctx_, *pool_, pending_.desc())) return;
			if (pbvrReady_) pbvr_.bindGrid(*ctx_, gpu_);
		}
		gpu_.uploadDensity(*ctx_, *pool_, pending_);
		sunDirty_ = true;
		pbvrRestart_ = true;
	}
	if (sunDirty_ && gpu_.hasGrid()) {
		sunDirty_ = false;
		gpu_.recordSunTransmittance(cmd, scattering_);
	}
	if (mode_ == Mode::Pbvr && pbvrReady_ && gpu_.hasGrid() && hasCamera_) {
		if (pbvrRestart_) {
			pbvr_.resetAccumulation();
			pbvrRestart_ = false;
		}
		const uint32_t have = pbvr_.accumulatedEnsembles();
		const uint32_t target = std::max(1u, pbvrSettings_.targetEnsembles);
		if (have < target) {
			const uint32_t count = std::min(std::max(1u, pbvrSettings_.ensemblesPerFrame), target - have);
			pbvr_.recordEnsembles(cmd, frameIndex, gpu_, pbvrCamera_, scattering_, count);
		}
	}
}

void CloudVolumeRenderer::onRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
	if (!ready_ || !enabled_ || !hasCamera_ || !gpu_.hasGrid()) return;
	if (mode_ == Mode::Pbvr && pbvrReady_) {
		pbvr_.recordComposite(cmd);
	} else {
		gpu_.recordRaymarch(cmd, frameIndex, camera_, scattering_);
	}
}

CloudVolumeRenderer::Stats CloudVolumeRenderer::queryStats()
{
	Stats s;
	s.mode = mode_;
	if (ready_ && pbvrReady_) {
		s.accumulated = pbvr_.accumulatedEnsembles();
		Volume::VolumePbvrGpu::Stats gs;
		vkDeviceWaitIdle(ctx_->getDevice());
		if (pbvr_.readStats(*ctx_, *pool_, gs)) {
			s.overflowed = gs.overflowed;
			s.generated = gs.generated;
		}
	}
	return s;
}

} // namespace Phantom
