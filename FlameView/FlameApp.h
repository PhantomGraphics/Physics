#pragma once

#include "CGLib/VkAppBase/VkAppBase.h"
#include "FlameRenderer.h"

#include "Physics/Physics/FlameFluid.h"
#include "Physics/Physics/FlameSolver.h"

#include <glm/glm.hpp>

#include <random>
#include <string>

namespace FlameView {

/**
 * @brief Minimal standalone visualization app for Phantom::Physics::FlameFluid/FlameSolver.
 *
 * Deliberately independent of Physics/PhysicsView: owns only a FlameFluid and
 * a FlameSolver (no RigidBodyWorld/SoftBodyWorld/PhysicsSolver), and its own
 * FlameRenderer/FlamePipeline (no FluidRenderer/FluidPipeline). See
 * docs/todo/sph_flame_plan.md Phase 3.
 */
class FlameApp : public ::VKG::VkAppBase
{
public:
	FlameApp(int w, int h, const std::string& title);

protected:
	void onInit() override;
	void onUpdate(uint32_t frameIndex) override;
	void onSwapChainCreated() override;
	void onImGui() override;
	void onCleanup() override;

private:
	Phantom::Physics::FlameFluid fluid_;
	Phantom::Physics::FlameSolver solver_;
	FlameRenderer renderer_;

	bool running_ = true;
	float pointSize_ = 14.0f;
	float smokePointSize_ = 20.0f;

	// PBVR mode (see uploadParticlesToRenderer()/FlameRenderer::RenderMode): flame/spark AND
	// smoke particles alike survive with probability = their (scaled) opacity instead of being
	// blended, so overlapping particles of either kind composite correctly via plain depth
	// test/write and no back-to-front sort -- see FlamePBVRPipeline's class doc comment for why
	// treating both particle kinds through the identical mechanism (rather than smoke alone) is
	// the actual point of PBVR here. std::mt19937 (not re-seeded) so the thinning is a fresh
	// independent draw every frame, same idea as the existing volume/GS PBVR paths regenerating
	// stochastic particles from a density value.
	bool pbvrMode_ = false;
	std::mt19937 pbvrRng_{ 1337 };

	// Multiplies each flame/spark particle's (implicit, otherwise-always-1) opacity, and each
	// smoke particle's simulated opacity respectively, before either reaches the PBVR
	// keep-probability test below (both clamped back to [0,1] in
	// uploadParticlesToRenderer()) -- lets the user override how solid each substance reads,
	// same role as Phantom::Volume::PBVRRenderer's densityScale_. Flame is fully opaque (1.0) by
	// default, matching its Normal-mode look (every simulated particle is "real" gas); smoke
	// defaults the same way, but is the one users most often want to thin out.
	float flameOpacityScale_ = 1.0f;
	float smokeOpacityScale_ = 1.0f;

	// Independent Bernoulli(opacity) trials per particle per frame, flame/spark and smoke alike
	// (like Phantom::Volume::PBVRRenderer's/VolumeView MenuPanel's "Repeat Count##pbvr" slider).
	// Each surviving trial is jittered to a different sub-position within the original
	// particle's own screen footprint and drawn smaller (area scaled by 1/repeatCount) rather
	// than redrawing the exact same point -- opaque, depth-tested points at the *same* position
	// don't blend or average at all, so without this spread, repeatCount would have no visual
	// effect (see uploadParticlesToRenderer()'s pushPBVR doc comment). No cap on the resulting
	// total PBVR point count is applied, by design.
	int pbvrRepeatCount_ = 1;

	float azimuth_ = 0.0f;
	float elevation_ = 12.0f;
	float distance_ = 4.0f;
	double prevMouseX_ = 0.0;
	double prevMouseY_ = 0.0;
	bool dragging_ = false;

	void setupWindowCallbacks();
	void setupInitialScene();
	void stepSimulation(float dt);
	void uploadParticlesToRenderer();

	glm::mat4 computeViewMatrix() const;
	glm::mat4 computeProjMatrix() const;
};

} // namespace FlameView
