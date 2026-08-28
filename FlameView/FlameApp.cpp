#include "FlameApp.h"

#include "CGLib/VulkanGraphics/VulkanSPVResolver.h"
#include "CGLib/Math/Box3d.h"

#include "imgui.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace FlameView {

namespace {

// Mirrors flame_point.frag's blackbody-ish gradient -- kept in exact sync with that shader
// since PBVR mode pre-computes flame/spark colour on the CPU instead of deriving it in-shader
// (see FlamePBVRPipeline's class doc comment for why).
glm::vec3 flameColor(float temperature, float tMin, float tMax) {
	const float t = glm::clamp((temperature - tMin) / std::max(tMax - tMin, 1.0e-4f), 0.0f, 1.0f);
	const glm::vec3 cold(0.35f, 0.02f, 0.0f);
	const glm::vec3 mid(1.0f, 0.55f, 0.05f);
	const glm::vec3 hot(1.0f, 0.95f, 0.75f);
	glm::vec3 color = glm::mix(cold, mid, glm::clamp(t * 2.0f, 0.0f, 1.0f));
	color = glm::mix(color, hot, glm::clamp(t * 2.0f - 1.0f, 0.0f, 1.0f));
	return color;
}

// Mirrors flame_smoke.frag's ember-to-soot tint (same dark sooty gray as
// FlameSmokePipeline::Buffer::smokeColor's default, which Normal mode never overrides).
glm::vec3 smokeColorTint(float temperature, float tMin, float tMax) {
	const glm::vec3 smokeColor(0.22f, 0.20f, 0.18f);
	const float t = glm::clamp((temperature - tMin) / std::max(tMax - tMin, 1.0e-4f), 0.0f, 1.0f);
	const glm::vec3 emberGlow(1.0f, 0.45f, 0.12f);
	return glm::mix(smokeColor, emberGlow, t * t);
}

} // namespace

FlameApp::FlameApp(int w, int h, const std::string& title)
	: ::VKG::VkAppBase(w, h, title)
{
	add(&renderer_);
}

void FlameApp::onInit()
{
	renderer_.setShaders(
		::VKG::loadSPVRepo("shaders/flame_point.vert.spv"),
		::VKG::loadSPVRepo("shaders/flame_point.frag.spv"));
	renderer_.setSmokeShaders(
		::VKG::loadSPVRepo("shaders/flame_smoke.vert.spv"),
		::VKG::loadSPVRepo("shaders/flame_smoke.frag.spv"));
	renderer_.setPBVRShaders(
		::VKG::loadSPVRepo("shaders/flame_pbvr.vert.spv"),
		::VKG::loadSPVRepo("shaders/flame_pbvr.frag.spv"));

	::VKG::VkAppBase::onInit();

	setupInitialScene();
	renderer_.setCamera(computeProjMatrix(), computeViewMatrix());
	setupWindowCallbacks();
}

void FlameApp::setupInitialScene()
{
	fluid_.setEffectLength(0.12f);
	fluid_.setDensity(1.0f);
	fluid_.setPressureCoe(20.0f);
	fluid_.setVicosityCoe(0.001f);
	fluid_.setMaxParticles(4000);
	fluid_.setMaxSpeed(3.0f);

	FlameFluid::Emitter e;
	e.center = Vector3df(0.0f, 0.0f, 0.0f);
	e.radius = 0.12f;
	e.rate = 250.0f;
	// Co-emit ambient "air" particles in a wider halo so freshly ignited
	// particles have SPH neighbors from frame one instead of spawning into a
	// near-vacuum (see FlameFluid::updateEmitters()).
	e.airRate = 150.0f;
	fluid_.addEmitter(e);

	// Secondary particles (cosmetic, non-SPH; see FlameFluid::updateSecondaryParticles()).
	// Population targets, not flat rates: total secondary count tracks ~20x
	// the current primary particle count, dense enough that sparks/smoke
	// alone read as a complete flame without the primary points showing.
	fluid_.setSparkCountPerPrimary(6.0f);
	fluid_.setSmokeCountPerPrimary(14.0f);

	solver_.add(&fluid_);
	solver_.setEffectLength(fluid_.getEffectLength());
	solver_.setBoundary(Box3df(Vector3df(-1.5f, -0.05f, -1.5f), Vector3df(1.5f, 4.0f, 1.5f)), 0.01f);
}

void FlameApp::stepSimulation(const float dt)
{
	solver_.simulate(dt);
}

void FlameApp::uploadParticlesToRenderer()
{
	const auto& particles = fluid_.getParticles();
	const auto& secondaries = fluid_.getSecondaryParticles();
	const float tMin = fluid_.getAmbientTemperature();
	const float tMax = fluid_.getIgnitionTemperature();

	if (pbvrMode_) {
		// Unified PBVR stream (see FlamePBVRPipeline's class doc comment): flame/spark AND smoke
		// particles alike are thinned by independent Bernoulli(opacity) trials and merged into
		// one pos+color+size buffer, instead of smoke alone getting stochastic/opaque treatment
		// while flame kept unconditionally drawing on top of it.
		std::vector<float> pbvrPositions;
		std::vector<float> pbvrColors;
		std::vector<float> pbvrSizes;
		std::uniform_real_distribution<float> keepDist(0.0f, 1.0f);
		std::uniform_real_distribution<float> jitterDist(-1.0f, 1.0f);

		// A repeat count > 1 is meaningless without this: pushing N identical Bernoulli trials
		// at the exact same position/size just redraws the same pixel N times (opaque, depth
		// test/write -- no blending happens), so nothing is actually averaged and repeatCount
		// has zero visual effect. The correct PBVR technique (matching
		// Phantom::Volume::ParticleGenerator's per-voxel randomInCell()) instead spreads the
		// repeat trials that survive across DIFFERENT sub-positions within the original
		// particle's own on-screen footprint, each drawn smaller (area scaled by 1/repeatCount so
		// the *total* covered area at opacity=1 still matches the un-repeated footprint) -- the
		// resulting scatter of small opaque dots, viewed together, dithers to an average
		// covered-area fraction equal to opacity, which is what actually reads as translucency
		// in a single frame without literal framebuffer accumulation. footprintRadius converts
		// each particle's screen-space point size (gl_PointSize, in pixels) to an approximate
		// world-space jitter radius using the current camera distance, since all flame/smoke
		// particles in this scene sit at roughly the same distance from the camera.
		const auto ext = getExtent();
		const float viewportHeight = (ext.height > 0) ? static_cast<float>(ext.height) : 720.0f;
		const float worldPerPixel = (2.0f * distance_ * std::tan(glm::radians(45.0f) * 0.5f)) / viewportHeight;
		const float subSizeScale = 1.0f / std::sqrt(static_cast<float>(pbvrRepeatCount_));

		const auto pushPBVR = [&](const Vector3df& pos, float opacity, const glm::vec3& color, float sizePixels) {
			const float footprintRadius = 0.5f * sizePixels * worldPerPixel;
			const float subSize = sizePixels * subSizeScale;
			for (int r = 0; r < pbvrRepeatCount_; ++r) {
				if (keepDist(pbvrRng_) >= opacity) {
					continue;
				}
				Vector3df jittered = pos;
				if (pbvrRepeatCount_ > 1) {
					jittered.x += jitterDist(pbvrRng_) * footprintRadius;
					jittered.y += jitterDist(pbvrRng_) * footprintRadius;
					jittered.z += jitterDist(pbvrRng_) * footprintRadius;
				}
				pbvrPositions.push_back(jittered.x);
				pbvrPositions.push_back(jittered.y);
				pbvrPositions.push_back(jittered.z);
				pbvrColors.push_back(color.r);
				pbvrColors.push_back(color.g);
				pbvrColors.push_back(color.b);
				pbvrColors.push_back(1.0f);
				pbvrSizes.push_back(subSize);
			}
		};

		const float flameOpacity = std::clamp(flameOpacityScale_, 0.0f, 1.0f);
		for (size_t i = 0; i < particles.size(); ++i) {
			pushPBVR(particles.positions[i], flameOpacity, flameColor(particles.temperatures[i], tMin, tMax), pointSize_);
		}
		for (const auto& sp : secondaries) {
			if (sp.kind == FlameFluid::SecondaryKind::Spark) {
				pushPBVR(sp.position, flameOpacity, flameColor(sp.temperature, tMin, tMax), pointSize_ * sp.size);
			} else {
				const float smokeOpacity = std::clamp(sp.opacity * smokeOpacityScale_, 0.0f, 1.0f);
				pushPBVR(sp.position, smokeOpacity, smokeColorTint(sp.temperature, tMin, tMax), smokePointSize_ * sp.size);
			}
		}

		renderer_.setPBVRParticles(std::move(pbvrPositions), std::move(pbvrColors), std::move(pbvrSizes));
	} else {
		std::vector<float> positions;
		std::vector<float> temperatures;
		std::vector<float> sizes;
		positions.reserve(particles.size() * 3);
		temperatures.reserve(particles.size());
		sizes.reserve(particles.size());

		for (size_t i = 0; i < particles.size(); ++i) {
			const auto& pos = particles.positions[i];
			positions.push_back(pos.x);
			positions.push_back(pos.y);
			positions.push_back(pos.z);
			temperatures.push_back(particles.temperatures[i]);
			sizes.push_back(1.0f);
		}

		std::vector<float> smokePositions;
		std::vector<float> smokeOpacities;
		std::vector<float> smokeSizes;
		std::vector<float> smokeTemperatures;

		// Sparks reuse the primary particles' temperature-gradient look, so they
		// are simply appended to the same buffers (see FlameRenderer's doc
		// comment) with a smaller relative size; smoke needs its own
		// alpha-blended pipeline/buffers (including its own temperature attribute,
		// see FlameSmokePipeline's doc comment).
		for (const auto& sp : secondaries) {
			if (sp.kind == FlameFluid::SecondaryKind::Spark) {
				positions.push_back(sp.position.x);
				positions.push_back(sp.position.y);
				positions.push_back(sp.position.z);
				temperatures.push_back(sp.temperature);
				sizes.push_back(sp.size);
			} else {
				smokePositions.push_back(sp.position.x);
				smokePositions.push_back(sp.position.y);
				smokePositions.push_back(sp.position.z);
				smokeOpacities.push_back(std::clamp(sp.opacity * smokeOpacityScale_, 0.0f, 1.0f));
				smokeSizes.push_back(sp.size);
				smokeTemperatures.push_back(sp.temperature);
			}
		}

		renderer_.setParticles(std::move(positions), std::move(temperatures), std::move(sizes));
		renderer_.setSmokeParticles(std::move(smokePositions), std::move(smokeOpacities), std::move(smokeSizes), std::move(smokeTemperatures));
	}

	renderer_.setTemperatureRange(tMin, tMax);
	renderer_.setPointSize(pointSize_);
	renderer_.setSmokePointSize(smokePointSize_);
	renderer_.setRenderMode(pbvrMode_ ? FlameRenderer::RenderMode::PBVR : FlameRenderer::RenderMode::Normal);
}

void FlameApp::onUpdate(uint32_t frameIndex)
{
	if (running_) {
		stepSimulation(1.0f / 60.0f);
	}
	uploadParticlesToRenderer();
	renderer_.setCamera(computeProjMatrix(), computeViewMatrix());

	::VKG::VkAppBase::onUpdate(frameIndex);
}

void FlameApp::onSwapChainCreated()
{
	// FlamePipeline's viewport/scissor are dynamic state; nothing extent-dependent to redo.
}

void FlameApp::onImGui()
{
	::VKG::VkAppBase::onImGui();

	ImGui::Begin("Flame Controls");

	ImGui::Text("Particles: %d / %d", fluid_.getNumParticles(), fluid_.getMaxParticles());
	ImGui::Checkbox("Running", &running_);
	ImGui::SameLine();
	if (ImGui::Button("Step") && !running_) {
		stepSimulation(1.0f / 60.0f);
	}

	ImGui::SliderFloat("Point Size", &pointSize_, 2.0f, 40.0f);

	if (ImGui::CollapsingHeader("Render Mode", ImGuiTreeNodeFlags_DefaultOpen)) {
		if (ImGui::RadioButton("Normal", !pbvrMode_)) pbvrMode_ = false;
		ImGui::SameLine();
		if (ImGui::RadioButton("PBVR (stochastic, no sort)", pbvrMode_)) pbvrMode_ = true;

		ImGui::SliderFloat("Smoke Opacity", &smokeOpacityScale_, 0.0f, 2.0f);

		if (pbvrMode_) {
			ImGui::TextDisabled(
				"Flame/spark and smoke particles alike survive with probability = their\n"
				"(scaled) opacity, then draw fully opaque -- flame and smoke correctly\n"
				"occlude each other via one shared depth buffer, so no sort is needed.");
			ImGui::SliderFloat("Flame Opacity", &flameOpacityScale_, 0.0f, 1.0f);
			ImGui::SliderInt("Repeat Count##pbvr", &pbvrRepeatCount_, 1, 100);
		}
	}

	if (ImGui::CollapsingHeader("Emitter", ImGuiTreeNodeFlags_DefaultOpen)) {
		auto& emitters = fluid_.getEmittersMutable();
		if (!emitters.empty()) {
			ImGui::SliderFloat("Rate (particles/sec)", &emitters[0].rate, 0.0f, 2000.0f);
			ImGui::SliderFloat("Air Rate (particles/sec)", &emitters[0].airRate, 0.0f, 2000.0f);
			ImGui::SliderFloat("Emitter Radius", &emitters[0].radius, 0.01f, 0.5f);
		}
	}

	if (ImGui::CollapsingHeader("Combustion", ImGuiTreeNodeFlags_DefaultOpen)) {
		float ambient = fluid_.getAmbientTemperature();
		if (ImGui::SliderFloat("Ambient Temperature", &ambient, 0.0f, 1000.0f)) fluid_.setAmbientTemperature(ambient);

		float ignition = fluid_.getIgnitionTemperature();
		if (ImGui::SliderFloat("Ignition Temperature", &ignition, 500.0f, 3000.0f)) fluid_.setIgnitionTemperature(ignition);

		float burnRate = fluid_.getBurnRate();
		if (ImGui::SliderFloat("Burn Rate", &burnRate, 0.0f, 5.0f)) fluid_.setBurnRate(burnRate);

		float heatRelease = fluid_.getHeatRelease();
		if (ImGui::SliderFloat("Heat Release", &heatRelease, 0.0f, 5000.0f)) fluid_.setHeatRelease(heatRelease);

		float coolRate = fluid_.getCoolRate();
		if (ImGui::SliderFloat("Cool Rate", &coolRate, 0.0f, 10.0f)) fluid_.setCoolRate(coolRate);

		float sootYield = fluid_.getSootYield();
		if (ImGui::SliderFloat("Soot Yield", &sootYield, 0.0f, 1.0f)) fluid_.setSootYield(sootYield);

		float lifeMax = fluid_.getLifeMax();
		if (ImGui::SliderFloat("Life Max (s)", &lifeMax, 0.5f, 15.0f)) fluid_.setLifeMax(lifeMax);
	}

	if (ImGui::CollapsingHeader("Buoyancy / Vorticity / Noise", ImGuiTreeNodeFlags_DefaultOpen)) {
		float buoyancyCoe = fluid_.getBuoyancyCoe();
		if (ImGui::SliderFloat("Buoyancy Coe", &buoyancyCoe, 0.0f, 20.0f)) fluid_.setBuoyancyCoe(buoyancyCoe);

		float thermalExpansion = fluid_.getThermalExpansion();
		if (ImGui::SliderFloat("Thermal Expansion", &thermalExpansion, 0.0f, 0.2f)) fluid_.setThermalExpansion(thermalExpansion);

		float vorticityEps = fluid_.getVorticityEps();
		if (ImGui::SliderFloat("Vorticity Eps", &vorticityEps, 0.0f, 10.0f)) fluid_.setVorticityEps(vorticityEps);

		float curlStrength = fluid_.getCurlNoiseStrength();
		if (ImGui::SliderFloat("Curl Noise Strength", &curlStrength, 0.0f, 3.0f)) fluid_.setCurlNoiseStrength(curlStrength);

		float curlFrequency = fluid_.getCurlNoiseFrequency();
		if (ImGui::SliderFloat("Curl Noise Frequency", &curlFrequency, 0.0f, 2.0f)) fluid_.setCurlNoiseFrequency(curlFrequency);

		float maxSpeed = fluid_.getMaxSpeed();
		if (ImGui::SliderFloat("Max Speed (m/s)", &maxSpeed, 0.5f, 10.0f)) fluid_.setMaxSpeed(maxSpeed);
	}

	if (ImGui::CollapsingHeader("Secondary Particles (Sparks / Smoke)", ImGuiTreeNodeFlags_DefaultOpen)) {
		ImGui::Text("Count: %d / %d", static_cast<int>(fluid_.getSecondaryParticles().size()), fluid_.getMaxSecondaryParticles());

		int maxSecondary = fluid_.getMaxSecondaryParticles();
		if (ImGui::SliderInt("Max Secondary Particles", &maxSecondary, 1000, 300000)) fluid_.setMaxSecondaryParticles(maxSecondary);

		float sparkPerPrimary = fluid_.getSparkCountPerPrimary();
		if (ImGui::SliderFloat("Sparks per Primary", &sparkPerPrimary, 0.0f, 50.0f)) fluid_.setSparkCountPerPrimary(sparkPerPrimary);

		float sparkLifeMax = fluid_.getSparkLifeMax();
		if (ImGui::SliderFloat("Spark Life Max (s)", &sparkLifeMax, 0.1f, 3.0f)) fluid_.setSparkLifeMax(sparkLifeMax);

		float sparkSpeed = fluid_.getSparkSpeed();
		if (ImGui::SliderFloat("Spark Speed", &sparkSpeed, 0.0f, 5.0f)) fluid_.setSparkSpeed(sparkSpeed);

		float sparkSize = fluid_.getSparkSize();
		if (ImGui::SliderFloat("Spark Size", &sparkSize, 0.05f, 1.5f)) fluid_.setSparkSize(sparkSize);

		float smokePerPrimary = fluid_.getSmokeCountPerPrimary();
		if (ImGui::SliderFloat("Smoke Puffs per Primary", &smokePerPrimary, 0.0f, 50.0f)) fluid_.setSmokeCountPerPrimary(smokePerPrimary);

		float smokeLifeMax = fluid_.getSmokeLifeMax();
		if (ImGui::SliderFloat("Smoke Life Max (s)", &smokeLifeMax, 0.5f, 10.0f)) fluid_.setSmokeLifeMax(smokeLifeMax);

		float smokeRiseSpeed = fluid_.getSmokeRiseSpeed();
		if (ImGui::SliderFloat("Smoke Rise Speed", &smokeRiseSpeed, 0.0f, 2.0f)) fluid_.setSmokeRiseSpeed(smokeRiseSpeed);

		float smokePointSize = smokePointSize_;
		if (ImGui::SliderFloat("Smoke Point Size", &smokePointSize, 2.0f, 60.0f)) smokePointSize_ = smokePointSize;

		float swirlStrength = fluid_.getSecondarySwirlStrength();
		if (ImGui::SliderFloat("Vorticity Swirl Strength", &swirlStrength, 0.0f, 5.0f)) fluid_.setSecondarySwirlStrength(swirlStrength);
	}

	ImGui::End();
}

void FlameApp::onCleanup()
{
	::VKG::VkAppBase::onCleanup();
}

void FlameApp::setupWindowCallbacks()
{
	auto& win = getWindow();

	win.onMouseButton = [this](int button, int action, int) {
		if (button == 0) {
			dragging_ = (action == 1);
		}
	};

	win.onCursorPos = [this](double x, double y) {
		if (dragging_) {
			const float dx = static_cast<float>(x - prevMouseX_);
			const float dy = static_cast<float>(y - prevMouseY_);
			azimuth_ += dx * 0.4f;
			elevation_ = std::clamp(elevation_ + dy * 0.3f, -85.0f, 85.0f);
		}
		prevMouseX_ = x;
		prevMouseY_ = y;
	};

	win.onScroll = [this](double, double dy) {
		distance_ = std::clamp(distance_ - static_cast<float>(dy) * 0.25f, 0.5f, 30.0f);
	};
}

glm::mat4 FlameApp::computeViewMatrix() const
{
	const float az = glm::radians(azimuth_);
	const float el = glm::radians(elevation_);

	const glm::vec3 target(0.0f, 1.0f, 0.0f);
	const glm::vec3 eye(
		target.x + distance_ * std::cos(el) * std::sin(az),
		target.y + distance_ * std::sin(el),
		target.z + distance_ * std::cos(el) * std::cos(az));

	return glm::lookAt(eye, target, glm::vec3(0.f, 1.f, 0.f));
}

glm::mat4 FlameApp::computeProjMatrix() const
{
	const auto ext = getExtent();
	const float aspect = (ext.height > 0)
		? static_cast<float>(ext.width) / static_cast<float>(ext.height)
		: 1.0f;

	glm::mat4 proj = glm::perspective(glm::radians(45.0f), aspect, 0.01f, 100.0f);
	proj[1][1] *= -1.0f;
	return proj;
}

} // namespace FlameView
