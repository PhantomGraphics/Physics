#pragma once

#include "CGLib/VkAppBase/IVkSubRenderer.h"
#include "FlamePipeline.h"
#include "FlameSmokePipeline.h"
#include "FlamePBVRPipeline.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace FlameView {

/**
 * @brief IVkSubRenderer wrapping FlamePipeline + FlameSmokePipeline + FlamePBVRPipeline.
 *
 * Two mutually exclusive render modes (see RenderMode):
 * - Normal: flame/spark particles as additive-blended point sprites (FlamePipeline), smoke
 *   particles as alpha-blended point sprites (FlameSmokePipeline), smoke drawn first so the
 *   additive flame/sparks draw on top of it. This is the original/default look.
 * - PBVR: flame/spark AND smoke particles are merged into ONE combined stream and drawn through
 *   FlamePBVRPipeline's single opaque, depth-tested, no-blend pass -- see that class's doc
 *   comment for why treating flame and smoke uniformly (rather than smoke alone getting an
 *   opaque "PBVR variant" while flame kept unconditionally drawing on top) is what makes this a
 *   real no-sort-needed PBVR technique instead of an inconsistent, incorrectly-occluding one.
 *
 * Particle data is pushed in via setParticles()/setSmokeParticles()/setPBVRParticles() (FlameApp
 * extracts/thins it from Physics::FlameFluid each frame) rather than this class reading
 * FlameFluid directly, so it stays a plain renderer with no simulation-side coupling. Sparks are
 * cosmetic secondary particles (see Phantom::Physics::FlameFluid::SecondaryParticle) but share
 * the flame's own temperature-gradient look, so FlameApp merges them into the same
 * position/temperature (Normal mode) or position/color (PBVR mode) arrays -- no renderer-side
 * distinction needed. Smoke needs a different (non-additive, tinted) look in Normal mode, so it
 * gets its own pipeline/buffers there, including its own inherited-temperature attribute (see
 * FlameSmokePipeline's doc comment) blended toward an ember-glow tint instead of the flame's
 * full blackbody gradient; in PBVR mode that same tint is pre-computed by FlameApp into the
 * shared color buffer instead (see FlamePBVRPipeline's doc comment).
 */
class FlameRenderer : public ::VKG::IVkSubRenderer
{
public:
	enum class RenderMode { Normal, PBVR };

	void setShaders(std::vector<uint32_t> vertSpv, std::vector<uint32_t> fragSpv);
	void setSmokeShaders(std::vector<uint32_t> vertSpv, std::vector<uint32_t> fragSpv);
	void setPBVRShaders(std::vector<uint32_t> vertSpv, std::vector<uint32_t> fragSpv);
	void setRenderMode(RenderMode m) { renderMode_ = m; }
	RenderMode getRenderMode() const { return renderMode_; }

	/** @brief Replaces this frame's particle data (Normal mode). Call once per frame before the render pass. */
	void setParticles(std::vector<float> positions, std::vector<float> temperatures, std::vector<float> sizes = {});

	/** @brief Replaces this frame's smoke secondary-particle data (Normal mode). */
	void setSmokeParticles(std::vector<float> positions, std::vector<float> opacities, std::vector<float> sizes,
		std::vector<float> temperatures = {});

	/** @brief Replaces this frame's combined flame+spark+smoke particle data (PBVR mode). */
	void setPBVRParticles(std::vector<float> positions, std::vector<float> colors, std::vector<float> sizes);

	void setCamera(const glm::mat4& proj, const glm::mat4& view) { proj_ = proj; view_ = view; }
	void setPointSize(float s) { pointSize_ = s; }
	void setTemperatureRange(float tMin, float tMax) { tMin_ = tMin; tMax_ = tMax; }
	void setSmokePointSize(float s) { smokePointSize_ = s; }

	void onInit(Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
		VkRenderPass renderPass, uint32_t framesInFlight) override;
	void onUpdate(uint32_t frameIndex) override;
	void onRender(VkCommandBuffer cmd, uint32_t frameIndex) override;
	void onCleanup(VkDevice device) override;

private:
	std::vector<uint32_t> vertSpv_;
	std::vector<uint32_t> fragSpv_;
	std::vector<uint32_t> smokeVertSpv_;
	std::vector<uint32_t> smokeFragSpv_;
	std::vector<uint32_t> pbvrVertSpv_;
	std::vector<uint32_t> pbvrFragSpv_;

	std::vector<float> positions_;
	std::vector<float> temperatures_;
	std::vector<float> sizes_;

	std::vector<float> smokePositions_;
	std::vector<float> smokeOpacities_;
	std::vector<float> smokeSizes_;
	std::vector<float> smokeTemperatures_;

	std::vector<float> pbvrPositions_;
	std::vector<float> pbvrColors_;
	std::vector<float> pbvrSizes_;

	glm::mat4 proj_{ 1.0f };
	glm::mat4 view_{ 1.0f };
	float pointSize_ = 14.0f;
	float tMin_ = 300.0f;
	float tMax_ = 1500.0f;
	float smokePointSize_ = 20.0f;
	RenderMode renderMode_ = RenderMode::Normal;

	const Phantom::VKG::VulkanContext* ctx_ = nullptr;
	const Phantom::VKG::VulkanCommandPool* pool_ = nullptr;

	std::optional<FlamePipeline> pipeline_;
	std::optional<FlameSmokePipeline> smokePipeline_;
	std::optional<FlamePBVRPipeline> pbvrPipeline_;
};

} // namespace FlameView
