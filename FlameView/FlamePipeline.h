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
 * @brief Minimal additive-blended point-sprite pipeline for FlameView.
 *
 * Modeled after Phantom::VKG::VkPointRenderer (CGLib/Renderer/VkRenderer)
 * but deliberately a separate, new implementation: three vertex bindings
 * (position vec3, temperature float, size float) instead of position/color/size,
 * and an additive-blend pipeline instead of the opaque default. Independent of
 * Physics/PhysicsView/FluidRenderer & FluidPipeline (see docs/todo/sph_flame_plan.md
 * -- that pair is referenced only, never modified or reused here). The size
 * attribute exists so cosmetic spark secondary particles (see
 * Phantom::Physics::FlameFluid::SecondaryParticle) can render smaller than the
 * primary flame particles despite sharing this pipeline/shader.
 */
class FlamePipeline
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
		std::vector<float> temperatures; ///< 1 float per particle.
		std::vector<float> sizes;        ///< 1 float per particle, multiplies pointSize (empty => all 1.0).
		glm::mat4 mvp{ 1.0f };
		float pointSize = 10.0f;
		float tMin = 300.0f;  ///< Temperature mapped to the "cold" end of the gradient.
		float tMax = 1500.0f; ///< Temperature mapped to the "hot" end of the gradient.
	};

	explicit FlamePipeline(Config config) : config_(std::move(config)) {}

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
	Phantom::VKG::VulkanBuffer temperatureBuffer_;
	Phantom::VKG::VulkanBuffer sizeBuffer_;

	std::vector<Phantom::VKG::VulkanBuffer> uniformBuffers_; // one per frame in flight
};

} // namespace FlameView
