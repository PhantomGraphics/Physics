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
 * @brief Unified PBVR (Particle-Based Volume Rendering) pipeline for FlameView.
 *
 * Renders flame/spark particles AND smoke particles through the exact same opaque,
 * depth-tested, non-blended point-sprite pass -- there is deliberately no separate
 * "smoke PBVR pipeline" and "flame pipeline" anymore. FlameApp::uploadParticlesToRenderer()
 * stochastically thins BOTH particle kinds by independent Bernoulli(opacity) trials (flame/spark
 * via flameOpacityScale_, smoke via sp.opacity*smokeOpacityScale_) before handing them here, so
 * opacity becomes particle *existence* rather than a blend weight for either kind alike. Colour
 * is pre-computed on the CPU per particle (flame's blackbody-ish gradient / smoke's
 * ember-to-soot tint -- see flame_point.frag's and flame_smoke.frag's formulas, replicated in
 * FlameApp.cpp) instead of derived from a temperature attribute in-shader, precisely because
 * this pipeline does not distinguish between particle kinds: every vertex is just a
 * pos+color+size PBVR particle (same pos+color convention as Phantom::Volume::PBVRVertex).
 *
 * depthWrite=true/blendEnable=false is what makes this order-independent: whichever particle
 * (flame or smoke, in any draw order) reaches a pixel first, the depth test keeps only the
 * nearest one exactly like ordinary opaque geometry -- no back-to-front sort is ever needed,
 * and flame/smoke now correctly occlude each other via the shared depth buffer instead of smoke
 * being depth-tested-but-not-written while flame draws unconditionally on top.
 */
class FlamePBVRPipeline
{
public:
	struct Config {
		std::vector<uint32_t> vertSpv;
		std::vector<uint32_t> fragSpv;
		VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT;
	};

	/** @brief Per-frame draw data: one combined stream for flame+spark+smoke particles alike. */
	struct Buffer {
		std::vector<float> positions; ///< Interleaved x,y,z (3 floats per particle).
		std::vector<float> colors;    ///< Interleaved r,g,b,a (4 floats per particle); alpha unused (opaque).
		std::vector<float> sizes;     ///< 1 float per particle -- the actual gl_PointSize, not a multiplier.
		glm::mat4 mvp{ 1.0f };
	};

	explicit FlamePBVRPipeline(Config config) : config_(std::move(config)) {}

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
	};

	Config config_;
	uint32_t framesInFlight_ = 2;
	uint32_t particleCount_ = 0;

	Phantom::VKG::VulkanDescriptorSetLayout descriptorSetLayout_;
	Phantom::VKG::VulkanDescriptorPool descriptorPool_;
	std::vector<VkDescriptorSet> descriptorSets_;

	Phantom::VKG::VulkanPipeline pipeline_;

	Phantom::VKG::VulkanBuffer positionBuffer_;
	Phantom::VKG::VulkanBuffer colorBuffer_;
	Phantom::VKG::VulkanBuffer sizeBuffer_;

	std::vector<Phantom::VKG::VulkanBuffer> uniformBuffers_; // one per frame in flight
};

} // namespace FlameView
