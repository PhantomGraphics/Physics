#pragma once

#include "CGLib/VkAppBase/IVkSubRenderer.h"
#include "FlamePBVRPass.h"
#include "FlamePointPipeline.h"

#include <glm/glm.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

namespace Phantom {

/**
 * @brief IVkSubRenderer for PhysicsView's Flame page: an emission-absorption
 * renderer (docs/todo/PLAN_flame_sph_pbvr_improvement.md Phases 3-4).
 *
 * Particles arrive as two streams, identical for both render modes:
 * - *emitters* (hot gas, sparks): blackbody HDR radiance, never occlude;
 * - *absorbers* (soot smoke puffs): Beer-Lambert spheres that darken what is
 *   behind them and re-emit ambient-lit albedo + their own thermal glow.
 *
 * RenderMode::Normal splats both directly into the HDR scene: absorbers with
 * premultiplied alpha, then emitters additively on top (order-dependent,
 * approximate -- smoke never hides flame in front of *or behind* it).
 *
 * RenderMode::PBVR is FlamePBVRPass: absorbers become GPU-generated opaque
 * sub-particles, R ensembles per frame are averaged (progressively while the
 * frame is static, as an EMA while the sim runs), and emitters are occluded
 * per ensemble -- order-independent and correct in expectation.
 *
 * Particle sizes are world-space diameters projected at each particle's own
 * depth. Data is pushed in by FluidApp every frame; this class never reads
 * Physics::FlameFluid.
 */
class FlameRenderer : public ::VKG::IVkSubRenderer
{
public:
	enum class RenderMode { Normal, PBVR };

	struct Shaders {
		std::vector<uint32_t> flameVert, flameFrag; // flame_point
		std::vector<uint32_t> smokeVert, smokeFrag; // flame_smoke
		FlamePBVRPass::Shaders pbvr;
	};

	/** @brief Shading parameters (FlamePointUBO::thermal/smoke/view/smokeAlbedo). */
	struct Shading {
		float ambientTemperature = 300.0f;
		float referenceTemperature = 2000.0f; ///< radiance 1 here, HDR above
		float exposure = 1.0f;
		float whiteBalanceTemperature = 0.0f; ///< adapt this blackbody's white to D65 (0 = off)
		float whiteBalanceDegree = 1.0f;      ///< 0..1, CIECAM02-style adaptation degree D
		float smokeExtinction = 4.0f;          ///< sigma: optical depth per unit density
		float smokeGlow = 0.2f;
		glm::vec3 smokeAlbedo{ 0.03f };        ///< already multiplied by the ambient light
		float pbvrSubdivision = 2.0f;
		float pbvrMinSubPixels = 1.5f;
		float pbvrDensityScale = 1.0f;
		bool operator==(const Shading& o) const;
	};

	void setShaders(Shaders shaders) { shaders_ = std::move(shaders); }
	void setRenderMode(RenderMode m) { renderMode_ = m; }
	RenderMode getRenderMode() const { return renderMode_; }

	void setEnabled(bool e) { enabled_ = e; }
	bool isEnabled() const { return enabled_; }

	/** @brief Emitters: xyz, temperature, world diameter. */
	void setEmitters(std::vector<float> positions, std::vector<float> temperatures, std::vector<float> sizes);
	/** @brief Absorbers: xyz, soot optical density, world diameter, temperature. */
	void setAbsorbers(std::vector<float> positions, std::vector<float> densities, std::vector<float> sizes,
		std::vector<float> temperatures);

	void setCamera(const glm::mat4& proj, const glm::mat4& view) { proj_ = proj; view_ = view; }
	/** @brief Render-target height in pixels (world-size -> gl_PointSize conversion). */
	void setViewportHeight(float h) { viewportHeight_ = h; }
	void setShading(const Shading& s) { shading_ = s; }
	/**
	 * @brief The simulation state changed since the last rendered frame
	 * (latched until onUpdate() consumes it). discontinuous = it jumped by
	 * more than one step (FlameStep:N, reset): the PBVR history restarts
	 * instead of blending across the jump.
	 */
	void notifySimulationAdvanced(bool discontinuous) { animating_ = true; jumped_ = jumped_ || discontinuous; }

	FlamePBVRPass::Settings& pbvrSettings() { return pbvr_.settings(); }
	const FlamePBVRPass::Stats& pbvrStats() const { return pbvr_.stats(); }

	/** @brief (Re)sizes the PBVR targets to the HDR target; call with the device idle. */
	void resize(uint32_t width, uint32_t height);
	/** @brief Records the PBVR compute/ensemble work; call outside any render pass, before the HDR pass. */
	void recordPreRender(VkCommandBuffer cmd, uint32_t frameIndex);

	void onInit(Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
		VkRenderPass renderPass, uint32_t framesInFlight) override;
	void onUpdate(uint32_t frameIndex) override;
	void onRender(VkCommandBuffer cmd, uint32_t frameIndex) override;
	void onCleanup(VkDevice device) override;

private:
	Shaders shaders_;

	std::vector<float> emitPositions_, emitTemperatures_, emitSizes_;
	std::vector<float> absPositions_, absDensities_, absSizes_, absTemperatures_;
	std::vector<float> absPacked_; // PBVR sources, 8 floats each

	glm::mat4 proj_{ 1.0f };
	glm::mat4 view_{ 1.0f };
	float viewportHeight_ = 720.0f;
	Shading shading_;
	RenderMode renderMode_ = RenderMode::Normal;
	bool enabled_ = false;
	bool animating_ = false;
	bool jumped_ = false;
	bool wasAnimating_ = false;

	// PBVR history invalidation: what the last PBVR frame was rendered with.
	glm::mat4 lastMvp_{ 0.0f };
	Shading lastShading_;
	float lastViewportHeight_ = 0.0f;
	bool pbvrWasActive_ = false;
	std::chrono::steady_clock::time_point lastFrameTime_{};

	const Phantom::VKG::VulkanContext* ctx_ = nullptr;
	std::optional<FlamePointPipeline> flamePipeline_;
	std::optional<FlamePointPipeline> smokePipeline_;
	FlamePBVRPass pbvr_;
	bool pbvrRecordedThisFrame_ = false;

	FlamePointUBO makeUBO();

	// Blackbody LUT for the current white balance (rebuilt only when it changes).
	std::array<glm::vec4, FlamePointUBO::kLutSize> lut_{};
	float lutWhite_ = -1.0f;
	float lutDegree_ = -1.0f;
};

} // namespace Phantom
