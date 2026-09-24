#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "FlameStreamBuffer.h"

#include "CGLib/VulkanGraphics/VulkanBuffer.h"
#include "CGLib/VulkanGraphics/VulkanDescriptorPool.h"
#include "CGLib/VulkanGraphics/VulkanPipeline.h"

#include <cstdint>
#include <vector>

namespace Phantom { namespace VKG { class VulkanContext; } }

namespace Phantom {

/**
 * @brief Uniform block shared by every Flame page shader -- the C++ side of
 * `FlameUBO` in shaders/flame_common.glsl (std140; keep the two in sync).
 *
 * `view` carries what the vertex shaders need to turn a *world-space*
 * diameter into gl_PointSize with perspective (plan A7):
 *   gl_PointSize = worldSize * proj[1][1] * viewportHeight / (2 * clip.w)
 * `thermal` / `smoke` / `lut` drive the emission-absorption model (plan
 * Phase 3, FlameBlackbody.h).
 */
struct FlamePointUBO {
	static constexpr int kLutSize = 64;

	glm::mat4 mvp{ 1.0f };
	glm::vec4 view{ 1.0f, 720.0f, 1.5f, 1.0f };     ///< x=|proj[1][1]|, y=viewport height (px), z=min PBVR sub-particle px, w=PBVR density scale
	glm::vec4 thermal{ 300.0f, 2000.0f, 1.0f, 0.0f }; ///< x=T_ambient, y=T_ref (radiance 1), z=flame exposure
	glm::vec4 smoke{ 4.0f, 0.2f, 2.0f, 0.0f };      ///< x=extinction sigma, y=smoke glow scale, z=PBVR subdivision
	glm::vec4 smokeAlbedo{ 0.02f, 0.02f, 0.02f, 0.0f };
	glm::vec4 lutRange{ 500.0f, 4000.0f, 0.0f, 0.0f };
	glm::vec4 lut[kLutSize]{};
};

/**
 * @brief Point-list pipeline with N tightly-packed float vertex streams and
 * one FlamePointUBO, used for every Flame page pass.
 *
 * Replaces the former FlamePipeline / FlameSmokePipeline / FlamePBVRPipeline,
 * which were the same class three times over with different stream layouts
 * and blend states. Vertex streams and the UBO are persistent per-frame
 * buffers (FlameStreamBuffer / one mapped UBO per frame slot), written only
 * for the frame being recorded (plan A6).
 */
class FlamePointPipeline {
public:
	enum class Blend {
		Opaque,        ///< no blending (PBVR opaque particles)
		Alpha,         ///< src-alpha / one-minus-src-alpha
		Additive,      ///< ONE / ONE (emission)
		Premultiplied, ///< ONE / ONE-minus-src-alpha (emission + absorption in one sprite)
	};

	struct Config {
		std::vector<uint32_t> vertSpv;
		std::vector<uint32_t> fragSpv;
		/** Floats per vertex for each binding, e.g. {3,1,1} = vec3 + float + float at locations 0,1,2. */
		std::vector<uint32_t> streamComponents;
		Blend blend = Blend::Additive;
		bool depthTest = true;
		bool depthWrite = false;
		VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT;
	};

	explicit FlamePointPipeline(Config config) : config_(std::move(config)) {}

	bool create(const Phantom::VKG::VulkanContext& ctx, VkRenderPass renderPass, uint32_t framesInFlight);
	void destroy(VkDevice device);

	/**
	 * @brief Writes frame slot frameIndex: `count` vertices from `streams`
	 * (one pointer per Config::streamComponents entry, each holding
	 * count * components floats) plus the UBO. count == 0 draws nothing.
	 */
	void upload(const Phantom::VKG::VulkanContext& ctx, uint32_t frameIndex, uint32_t count,
		const std::vector<const float*>& streams, const FlamePointUBO& ubo);

	void render(VkCommandBuffer cmd, uint32_t frameIndex) const;

	/** @brief Writes only frame slot frameIndex's UBO (for renderIndirect() users). */
	void uploadUniforms(uint32_t frameIndex, const FlamePointUBO& ubo);

	/**
	 * @brief Draws from caller-owned vertex buffers (one per stream binding)
	 * with a VkDrawIndirectCommand at indirectOffset -- the GPU-generated PBVR
	 * sub-particles, whose count only the GPU knows.
	 */
	void renderIndirect(VkCommandBuffer cmd, uint32_t frameIndex, const std::vector<VkBuffer>& vertexBuffers,
		VkBuffer indirectBuffer, VkDeviceSize indirectOffset) const;

	bool isValid() const { return pipeline_.getPipeline() != VK_NULL_HANDLE; }

private:
	Config config_;
	uint32_t framesInFlight_ = 2;
	std::vector<uint32_t> counts_; // per frame slot

	Phantom::VKG::VulkanDescriptorSetLayout descriptorSetLayout_;
	Phantom::VKG::VulkanDescriptorPool descriptorPool_;
	std::vector<VkDescriptorSet> descriptorSets_;
	Phantom::VKG::VulkanPipeline pipeline_;

	std::vector<FlameStreamBuffer> streams_;            // one ring per binding
	std::vector<Phantom::VKG::VulkanBuffer> uniformBuffers_; // one per frame slot
};

} // namespace Phantom
