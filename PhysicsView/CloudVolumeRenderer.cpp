#include "pch.h"
#include "CloudVolumeRenderer.h"

#include "CGLib/VulkanGraphics/VulkanContext.h"
#include "CGLib/VulkanGraphics/VulkanCommandPool.h"

namespace Phantom {

void CloudVolumeRenderer::onInit(Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
                                 VkRenderPass renderPass, uint32_t framesInFlight)
{
	ctx_ = &ctx;
	pool_ = &pool;
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
	if (!ready_) std::fprintf(stderr, "[Cloud] VolumeRaymarchGpu creation failed\n");
}

void CloudVolumeRenderer::onCleanup(VkDevice /*device*/)
{
	if (ctx_ && ready_) {
		vkDeviceWaitIdle(ctx_->getDevice());
		gpu_.destroy(*ctx_);
	}
	ready_ = false;
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
	scattering_ = p;
}

void CloudVolumeRenderer::setCamera(const glm::mat4& proj, const glm::mat4& view, const glm::mat4& cloudToScene)
{
	camera_.invViewProj = glm::inverse(proj * view * cloudToScene);
	const glm::vec3 eyeScene = glm::vec3(glm::inverse(view)[3]);
	camera_.position = glm::vec3(glm::inverse(cloudToScene) * glm::vec4(eyeScene, 1.0f));
	hasCamera_ = true;
}

void CloudVolumeRenderer::recordPreRender(VkCommandBuffer cmd, uint32_t /*frameIndex*/)
{
	if (!ready_ || !enabled_) return;
	if (densityDirty_) {
		densityDirty_ = false;
		if (pending_.data().empty()) return;
		if (!gpu_.hasGrid() || !(gpu_.gridDesc() == pending_.desc())) {
			vkDeviceWaitIdle(ctx_->getDevice());   // images are replaced: nothing may still reference them
			if (!gpu_.setGrid(*ctx_, *pool_, pending_.desc())) return;
		}
		gpu_.uploadDensity(*ctx_, *pool_, pending_);
		sunDirty_ = true;
	}
	if (sunDirty_ && gpu_.hasGrid()) {
		sunDirty_ = false;
		gpu_.recordSunTransmittance(cmd, scattering_);
	}
}

void CloudVolumeRenderer::onRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
	if (!ready_ || !enabled_ || !hasCamera_ || !gpu_.hasGrid()) return;
	gpu_.recordRaymarch(cmd, frameIndex, camera_, scattering_);
}

} // namespace Phantom
