#pragma once

#include "FlameFullscreenPass.h"
#include "FlamePointPipeline.h"
#include "FlameStreamBuffer.h"

#include "CGLib/Graphics/EnsembleLodController.h"
#include "CGLib/VulkanGraphics/VulkanBuffer.h"
#include "CGLib/VulkanGraphics/VulkanComputePipeline.h"
#include "CGLib/VulkanGraphics/VulkanDescriptorPool.h"
#include "CGLib/VulkanGraphics/VulkanOffscreen.h"
#include "CGLib/VulkanGraphics/VulkanSampler.h"

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace Phantom { namespace VKG { class VulkanContext; class VulkanCommandPool; } }

namespace Phantom {

/**
 * @brief GPU particle-based volume rendering of the flame with ensemble
 * averaging (docs/todo/PLAN_flame_sph_pbvr_improvement.md Phase 4).
 *
 * Replaces the CPU Bernoulli thinning PBVR mode (one stochastic draw per
 * frame, no averaging, colours precomputed on the CPU). Per displayed frame:
 *
 *  1. flame_pbvr_generate.comp turns every *absorbing* source (smoke puff)
 *     into Poisson(lambda) opaque sub-particles, lambda derived from the
 *     puff's optical depth (see the shader's header), for R independent
 *     ensembles at once (R segments of one vertex buffer);
 *     flame_pbvr_finalize.comp turns the per-ensemble counts into indirect
 *     draw records.
 *  2. For each ensemble: clear an RGBA16F+depth target, draw its opaque
 *     sub-particles (depth write, no sort), then draw every *emissive* source
 *     (hot gas / sparks) additively with depth test but no depth write, then
 *     fold the image into the running average (flame_pbvr_blend.frag).
 *  3. Inside the HDR scene pass, composite the average with premultiplied
 *     alpha (flame_pbvr_composite.frag).
 *
 * Emission-absorption (the plan's open question, decided here): emitters are
 * drawn deterministically in every ensemble and only *occluded* by that
 * ensemble's stochastic opaque smoke. A pixel then sees emission e with
 * probability T (the smoke's transmittance in front of it), so the ensemble
 * mean is sum e_i T_i -- exactly the emission-absorption integral -- and the
 * mean alpha is 1 - T. No emitter needs to be sampled, and emission never
 * occludes anything.
 *
 * History: while the frame is static (paused sim, same camera and params) the
 * average is the exact progressive mean 1/(n+1) and stops at
 * Settings::targetEnsembles (converged). While the simulation runs, n is
 * capped at temporalFrames * R, which turns the same update into an
 * exponential moving average over ~temporalFrames frames (0 = no reuse:
 * every frame shows the mean of its own R ensembles only). Any camera /
 * parameter / viewport change resets the history.
 *
 * R per frame is fixed (LodMode::Manual) or chosen by the shared
 * Phantom::Graphics::EnsembleLodController from this pass's GPU time, measured
 * with timestamp queries (LodMode::Adaptive).
 */
class FlamePBVRPass {
public:
	static constexpr uint32_t kMaxEnsembles = 8;
	static constexpr uint32_t kCapacityPerEnsemble = 1u << 17; // sub-particles per ensemble segment

	struct Shaders {
		std::vector<uint32_t> pointVert, pointFrag;       // flame_pbvr_point
		std::vector<uint32_t> emissiveVert, emissiveFrag; // flame_point
		std::vector<uint32_t> fullscreenVert;             // flame_fullscreen
		std::vector<uint32_t> blendFrag, compositeFrag;   // flame_pbvr_blend / _composite
		std::vector<uint32_t> generateComp, finalizeComp; // flame_pbvr_generate / _finalize
	};

	enum class LodMode { Manual, Adaptive };

	struct Settings {
		LodMode  lodMode = LodMode::Adaptive;
		uint32_t ensemblesPerFrame = 4;  ///< R for LodMode::Manual (1..kMaxEnsembles)
		uint32_t targetEnsembles = 64;   ///< static-frame convergence target
		float    temporalFrames = 4.0f;  ///< EMA length while animating (frames); 0 = off
		float    budgetMs = 8.0f;        ///< Adaptive: grow R while this pass stays under it (shrink above 2x)
		uint32_t maxPerSource = 64;      ///< cap on sub-particles per source per ensemble
	};

	struct Stats {
		uint32_t displayedEnsembles = 0; ///< effective samples in the shown average (min(n, EMA cap) while running)
		uint32_t ensemblesThisFrame = 0; ///< R actually rendered by the last record()
		uint32_t generated = 0;          ///< sub-particles generated (last completed frame, all ensembles)
		uint32_t overflowed = 0;         ///< of which dropped by the segment capacity
		float    gpuMs = 0.0f;           ///< GPU time of the whole pass (last completed frame)
		bool     timestampsSupported = false;
		int      lodState = 0;           ///< Phantom::Graphics::EnsembleLodController::State
	};

	bool create(const Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
		VkRenderPass hdrRenderPass, uint32_t framesInFlight, const Shaders& shaders, uint32_t width, uint32_t height);
	bool resize(const Phantom::VKG::VulkanContext& ctx, uint32_t width, uint32_t height);
	void destroy(const Phantom::VKG::VulkanContext& ctx);
	bool isValid() const { return valid_; }

	/**
	 * @brief Per-frame CPU side, called from the owner's onUpdate(frameIndex).
	 * @param absorbing   8 floats per smoke source: x,y,z,diameter, density,temperature,0,0.
	 * @param emissive*   Emitter streams (xyz / temperature / diameter), `emissiveCount` long.
	 * @param resetHistory Camera/params/viewport changed: restart the average.
	 * @param animating   The simulation advanced since the previous frame.
	 */
	void update(const Phantom::VKG::VulkanContext& ctx, uint32_t frameIndex, const FlamePointUBO& ubo,
		const std::vector<float>& absorbing, uint32_t emissiveCount, const float* emissivePos,
		const float* emissiveTemperature, const float* emissiveSize,
		bool resetHistory, bool animating, float dtMs);

	/** @brief Compute + ensemble passes; record outside any render pass (FluidApp::onPreRender). */
	void record(VkCommandBuffer cmd, uint32_t frameIndex);

	/** @brief Composites the average into the currently open HDR render pass. */
	void composite(VkCommandBuffer cmd, uint32_t frameIndex) const;

	Settings& settings() { return settings_; }
	const Settings& settings() const { return settings_; }
	const Stats& stats() const { return stats_; }

private:
	bool createTargets(const Phantom::VKG::VulkanContext& ctx, uint32_t width, uint32_t height);
	void destroyTargets(const Phantom::VKG::VulkanContext& ctx);
	void writeImageDescriptors(VkDevice device);

	bool valid_ = false;
	uint32_t framesInFlight_ = 2;
	Settings settings_;
	Stats stats_;

	// Targets: one ensemble image, two history images (ping-pong).
	Phantom::VKG::VulkanOffscreen ensemble_;
	std::array<Phantom::VKG::VulkanOffscreen, 2> history_;
	Phantom::VKG::VulkanSampler sampler_;
	VkFormat depthFormat_ = VK_FORMAT_D32_SFLOAT;

	std::optional<FlamePointPipeline> pointPipeline_;
	std::optional<FlamePointPipeline> emissivePipeline_;
	FlameFullscreenPass blend_;
	FlameFullscreenPass composite_;

	// Compute.
	Phantom::VKG::VulkanDescriptorSetLayout genLayout_;
	Phantom::VKG::VulkanDescriptorPool genPool_;
	std::vector<VkDescriptorSet> genSets_;
	std::vector<VkBuffer> genBoundSources_; // source SSBO handle each frame set currently points at
	Phantom::VKG::VulkanComputePipeline genPipeline_;
	Phantom::VKG::VulkanDescriptorSetLayout finLayout_;
	Phantom::VKG::VulkanDescriptorPool finPool_;
	std::vector<VkDescriptorSet> finSets_;
	Phantom::VKG::VulkanComputePipeline finPipeline_;

	std::vector<Phantom::VKG::VulkanBuffer> computeUbo_;  // per frame
	FlameStreamBuffer sources_;                           // per frame
	Phantom::VKG::VulkanBuffer outPos_, outColor_;        // kMaxEnsembles * capacity each
	Phantom::VKG::VulkanBuffer counters_, args_;
	std::vector<Phantom::VKG::VulkanBuffer> genStats_;    // per frame, host-visible readback

	VkQueryPool queryPool_ = VK_NULL_HANDLE;
	float tsPeriodNs_ = 0.0f;
	std::vector<bool> tsWritten_;
	std::vector<uint32_t> ensemblesRecorded_; // R recorded into each frame slot (for stats readback)

	// Per-frame decisions made in update(), consumed by record().
	uint32_t sourceCount_ = 0;
	uint32_t pendingR_ = 0;
	uint32_t historyIndex_ = 0; // which history_ holds the current average
	uint32_t samples_ = 0;      // ensembles folded into it
	uint32_t sampleCap_ = UINT32_MAX;
	uint32_t seed_ = 0;
	// Freshly (re)created history images are VK_IMAGE_LAYOUT_UNDEFINED; the
	// blend descriptor sets reference both, so record() clears them once
	// (their render pass leaves them SHADER_READ_ONLY_OPTIMAL).
	bool historyNeedsInit_ = true;

	Phantom::Graphics::EnsembleLodController lod_;
};

} // namespace Phantom
