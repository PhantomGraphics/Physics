#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "CGLib/VulkanGraphics/VulkanBuffer.h"
#include "CGLib/VulkanGraphics/VulkanDescriptorPool.h"
#include "CGLib/VulkanGraphics/VulkanPipeline.h"

#include <vector>

namespace Phantom { namespace VKG { class VulkanContext; class VulkanCommandPool; } }

namespace FlameView {

/**
 * @brief Minimal alpha-blended point-sprite pipeline for FlameView's cosmetic
 * smoke secondary particles (see Phantom::Physics::FlameFluid::SecondaryParticle).
 *
 * Structurally a sibling of FlamePipeline (same create/upload/render shape),
 * but deliberately a separate, simpler pipeline: standard src-alpha blending
 * instead of additive (smoke should be able to darken/obscure, unlike the
 * flame's glow), a per-vertex size attribute (smoke puffs grow as they age),
 * and a per-vertex temperature attribute blended toward smokeColor in the
 * fragment shader -- freshly emitted, still-hot smoke (see
 * Phantom::Physics::FlameFluid::SecondaryParticle::temperature) reads as a
 * warm ember glow that cools to the flat sooty tint as it ages, instead of
 * every puff sharing one flat color regardless of how it was spawned.
 * Independent of FlamePipeline so each stays a small, single-purpose
 * implementation. Only used for FlameRenderer::RenderMode::Normal -- PBVR mode bypasses this
 * pipeline (and FlamePipeline) entirely in favor of the unified FlamePBVRPipeline, which draws
 * flame/spark and smoke particles together through one opaque, no-sort-needed pass (see its
 * class doc comment for why treating them separately, as an earlier version of this pipeline
 * did with its own opaque "PBVR variant", produced inconsistent-looking, incorrectly-occluding
 * smoke).
 */
class FlameSmokePipeline
{
public:
	struct Config {
		std::vector<uint32_t> vertSpv;
		std::vector<uint32_t> fragSpv;
		VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT;
	};

	/** @brief Per-frame draw data. */
	struct Buffer {
		std::vector<float> positions;    ///< Interleaved x,y,z (3 floats per particle).
		std::vector<float> opacities;    ///< 1 float per particle, 0..1.
		std::vector<float> sizes;        ///< 1 float per particle, multiplies pointSize.
		std::vector<float> temperatures; ///< 1 float per particle; empty => all tMin (flat smokeColor).
		glm::mat4 mvp{ 1.0f };
		float pointSize = 20.0f;
		glm::vec3 smokeColor{ 0.22f, 0.20f, 0.18f }; // dark sooty gray
		float tMin = 300.0f;  ///< Temperature at/below which smoke is the flat smokeColor.
		float tMax = 1500.0f; ///< Temperature mapped to the fully warm ember-glow tint.
	};

	explicit FlameSmokePipeline(Config config) : config_(std::move(config)) {}

	void create(const Phantom::VKG::VulkanContext& ctx,
		const Phantom::VKG::VulkanCommandPool& pool,
		VkRenderPass renderPass,
		uint32_t framesInFlight = 2);

	void destroy(VkDevice device);

	/** @brief Uploads this frame's particle data to the GPU. Call before render(). */
	void upload(const Phantom::VKG::VulkanContext& ctx,
		const Phantom::VKG::VulkanCommandPool& pool,
		const Buffer& buffer);

	void render(VkCommandBuffer cmd, uint32_t frameIndex);

	bool isValid() const { return pipeline_.getPipeline() != VK_NULL_HANDLE; }

private:
	struct UBOData {
		glm::mat4 mvp;
		glm::vec4 smokeColor; // rgb used; alpha unused (padding).
		float pointSize;
		float tMin;
		float tMax;
		float pad0; // std140 padding to keep the struct 16-byte aligned.
	};

	Config config_;
	uint32_t framesInFlight_ = 2;
	uint32_t particleCount_ = 0;

	Phantom::VKG::VulkanDescriptorSetLayout descriptorSetLayout_;
	Phantom::VKG::VulkanDescriptorPool descriptorPool_;
	std::vector<VkDescriptorSet> descriptorSets_;

	Phantom::VKG::VulkanPipeline pipeline_;

	Phantom::VKG::VulkanBuffer positionBuffer_;
	Phantom::VKG::VulkanBuffer opacityBuffer_;
	Phantom::VKG::VulkanBuffer sizeBuffer_;
	Phantom::VKG::VulkanBuffer temperatureBuffer_;

	std::vector<Phantom::VKG::VulkanBuffer> uniformBuffers_; // one per frame in flight
};

} // namespace FlameView
