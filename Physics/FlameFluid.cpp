#include "pch.h"

#include "FlameFluid.h"

#include "FlameParticle.h"

#include <algorithm>

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace {
// Emitter particle radius is deliberately not part of the plan's minimal
// Emitter struct (center/radius/rate); this small fixed value keeps emitted
// particles consistent with the ~thousands-of-particles CPU budget.
constexpr float kEmittedParticleRadius = 0.02f;
constexpr float kTwoPi = 6.2831853f;

// Arrhenius k(T) is normalized to burnRate at T_ign; above that it keeps
// growing exponentially, so cap the gain to keep explicit integration sane.
constexpr float kMaxArrheniusGain = 8.0f;

float smoothstep01(const float e0, const float e1, const float x)
{
	if (e1 <= e0) {
		return x >= e1 ? 1.0f : 0.0f;
	}
	const float t = std::clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
	return t * t * (3.0f - 2.0f * t);
}
}

FlameFluid::FlameFluid() :
	rng(kDefaultRandomSeed)
{
}

float FlameFluid::computeTemperatureFactor(const float temperature) const
{
	if (combustionModel == CombustionModel::Legacy) {
		return 1.0f;
	}
	if (reactionRateModel == ReactionRateModel::Arrhenius) {
		const float t = std::max(temperature, 1.0f);
		const float tIgn = std::max(ignitionTemperature, 1.0f);
		// exp(-Ta/T) / exp(-Ta/T_ign) == exp(Ta * (1/T_ign - 1/T)).
		const float exponent = activationTemperature * (1.0f / tIgn - 1.0f / t);
		return std::min(kMaxArrheniusGain, std::exp(std::min(exponent, 20.0f)));
	}
	return smoothstep01(ignitionTemperature - ignitionWidth, ignitionTemperature + ignitionWidth, temperature);
}

float FlameFluid::computeReactionRate(const float temperature, const float fuel, const float oxygen) const
{
	if (combustionModel == CombustionModel::Legacy) {
		return burnRate * fuel;
	}
	return burnRate * computeTemperatureFactor(temperature) * std::max(0.0f, fuel) * std::max(0.0f, oxygen);
}

float FlameFluid::computeRestDensity(const float temperature) const
{
	if (!thermalExpansionPressure) {
		return density;
	}
	// Ideal gas at constant pressure: rho ~ 1/T. Floor at 10% so a wildly hot
	// particle cannot drive its rest density (and thus pressure base) to zero.
	const float ratio = ambientTemperature / std::max(temperature, 1.0f);
	return density * std::clamp(ratio, 0.1f, 1.0f);
}

Box3df FlameFluid::getBoundingBox() const
{
	auto bb = Box3df::createDegeneratedBox();
	for (const auto& pos : particles.positions) {
		bb.add(pos);
	}
	return bb;
}

void FlameFluid::updateEmitters(const float dt)
{
	std::uniform_real_distribution<float> unit(0.0f, 1.0f);
	std::uniform_real_distribution<float> angleDist(0.0f, kTwoPi);
	std::uniform_real_distribution<float> jitterDist(-0.2f, 0.2f);
	std::uniform_real_distribution<float> airJitterDist(-0.05f, 0.05f);

	// Air carrier particles spawn in a somewhat wider halo than the fuel core,
	// so they surround/precede the hot particles rather than competing with
	// them for the same spot.
	constexpr float kAirRadiusScale = 1.3f;

	for (auto& e : emitters) {
		e.accumulator += e.rate * dt;
		while (e.accumulator >= 1.0f) {
			if (static_cast<int>(particles.size()) >= maxParticles) {
				break;
			}
			e.accumulator -= 1.0f;

			const float r = e.radius * std::sqrt(unit(rng));
			const float theta = angleDist(rng);
			const Vector3df offset(r * std::cos(theta), 0.0f, r * std::sin(theta));

			const float fuelT = (e.fuelTemperature >= 0.0f) ? e.fuelTemperature : ignitionTemperature;
			particles.push_back(e.center + offset, kEmittedParticleRadius, density, fuelT);
			const size_t idx = particles.size() - 1;
			particles.fuels[idx] = 1.0f;
			// Pure fuel vapor: it has to mix with air carriers (via the
			// solver's diffusion pass) before the Physical model lets it burn.
			particles.oxygens[idx] = 0.0f;
			particles.velocities[idx] = Vector3df(jitterDist(rng), 1.0f + jitterDist(rng), jitterDist(rng));
		}

		e.airAccumulator += e.airRate * dt;
		while (e.airAccumulator >= 1.0f) {
			if (static_cast<int>(particles.size()) >= maxParticles) {
				break;
			}
			e.airAccumulator -= 1.0f;

			const float r = e.radius * kAirRadiusScale * std::sqrt(unit(rng));
			const float theta = angleDist(rng);
			const Vector3df offset(r * std::cos(theta), 0.0f, r * std::sin(theta));

			particles.push_back(e.center + offset, kEmittedParticleRadius, density, ambientTemperature);
			const size_t idx = particles.size() - 1;
			particles.airs[idx] = true;
			particles.velocities[idx] = Vector3df(airJitterDist(rng), airJitterDist(rng), airJitterDist(rng));
		}
	}
}

void FlameFluid::applyPilots()
{
	for (const auto& e : emitters) {
		if (e.pilotTemperature <= 0.0f) {
			continue;
		}
		const float r2 = e.radius * e.radius;
		for (size_t i = 0; i < particles.size(); ++i) {
			const auto d = particles.positions[i] - e.center;
			if (d.y < 0.0f || d.y > e.pilotHeight || d.x * d.x + d.z * d.z > r2) {
				continue;
			}
			particles.temperatures[i] = std::max(particles.temperatures[i], e.pilotTemperature);
		}
	}
}

void FlameFluid::removeDead()
{
	for (size_t i = 0; i < particles.size();) {
		FlameParticle p(particles, i, this);
		if (p.isDead()) {
			particles.swapAndPop(i);
			continue;
		}
		++i;
	}
}

Vector3df FlameFluid::rotateBySwirl(const Vector3df& velocity, const Vector3df& vorticity, const float strength, const float dt)
{
	const float omegaLen = Math::getLength(vorticity);
	if (omegaLen <= 0.0f) {
		return velocity;
	}
	const Vector3df axis = vorticity / omegaLen;
	const float angle = omegaLen * strength * dt;
	const float c = std::cos(angle);
	const float s = std::sin(angle);
	return velocity * c + glm::cross(axis, velocity) * s + axis * (glm::dot(axis, velocity) * (1.0f - c));
}

void FlameFluid::updateSecondaryParticles(const float dt, const Vector3df& gravity)
{
	if (particles.empty()) {
		return;
	}

	std::uniform_int_distribution<size_t> sourceDist(0, particles.size() - 1);
	std::uniform_real_distribution<float> unit(0.0f, 1.0f);
	std::uniform_real_distribution<float> angleDist(0.0f, kTwoPi);
	std::uniform_real_distribution<float> elevDist(0.3f, 1.0f); // mostly-upward ejection
	std::uniform_real_distribution<float> jitterDist(-0.15f, 0.15f);

	const int numPrimaries = static_cast<int>(particles.size());
	constexpr float kSparkJitterRate = 1.1619f; // 0.15 * sqrt(60)
	const float sparkJitterScale = kSparkJitterRate * std::sqrt(std::max(dt, 0.0f)) / 0.15f;

	int sparkCount = 0;
	int smokeCount = 0;
	for (const auto& sp : secondaryParticles) {
		if (sp.kind == SecondaryKind::Spark) {
			++sparkCount;
		} else {
			++smokeCount;
		}
	}

	// ---- Spawn sparks: bright embers flung from hot primary particles -------
	// Rather than a flat rate*dt, the population is topped up toward a target
	// tied to the current primary particle count (sparkCountPerPrimary), so
	// the secondary cloud scales automatically with however many primaries
	// the SPH core has and stays dense enough to render on its own. The
	// attempt cap bounds the loop when few/no primaries currently qualify
	// (e.g. right after ignition, before anything is hot enough yet).
	int sparkDeficit = std::min(maxSecondaryParticles, static_cast<int>(sparkCountPerPrimary * numPrimaries)) - sparkCount;
	int sparkAttempts = 0;
	const int sparkAttemptCap = std::max(64, sparkDeficit * 4);
	while (sparkDeficit > 0 && sparkAttempts < sparkAttemptCap && static_cast<int>(secondaryParticles.size()) < maxSecondaryParticles) {
		++sparkAttempts;
		const size_t si = sourceDist(rng);
		if (particles.temperatures[si] < ignitionTemperature * 0.6f) {
			continue; // not hot enough to plausibly throw an ember
		}

		const float theta = angleDist(rng);
		const float elev = elevDist(rng);
		const float horiz = std::sqrt(std::max(0.0f, 1.0f - elev * elev));
		const Vector3df dir(horiz * std::cos(theta), elev, horiz * std::sin(theta));

		SecondaryParticle sp;
		sp.kind = SecondaryKind::Spark;
		sp.position = particles.positions[si];
		sp.velocity = particles.velocities[si] + dir * sparkSpeed;
		sp.vorticity = particles.vorticities[si];
		sp.temperature = particles.temperatures[si];
		sp.size = sparkSize;
		sp.lifeMax = sparkLifeMax * (0.7f + 0.6f * unit(rng));
		secondaryParticles.push_back(sp);
		--sparkDeficit;
	}

	// ---- Spawn smoke: soft puffs from sooty, cooling primary particles ------
	int smokeDeficit = std::min(maxSecondaryParticles, static_cast<int>(smokeCountPerPrimary * numPrimaries)) - smokeCount;
	int smokeAttempts = 0;
	const int smokeAttemptCap = std::max(64, smokeDeficit * 4);
	while (smokeDeficit > 0 && smokeAttempts < smokeAttemptCap && static_cast<int>(secondaryParticles.size()) < maxSecondaryParticles) {
		++smokeAttempts;
		const size_t si = sourceDist(rng);
		if (particles.soots[si] < 0.01f) {
			continue; // hasn't produced enough soot to visually read as smoke yet
		}

		SecondaryParticle sp;
		sp.kind = SecondaryKind::Smoke;
		sp.position = particles.positions[si];
		sp.velocity = particles.velocities[si] * 0.3f + Vector3df(jitterDist(rng), smokeRiseSpeed, jitterDist(rng));
		sp.vorticity = particles.vorticities[si];
		// Smoke starts at the source's own temperature (glowing soot fresh out
		// of the flame front) and cools toward ambient over its early life --
		// see the advection pass below and FlameApp's smoke shader tint.
		sp.temperature = particles.temperatures[si];
		sp.size = smokeStartSize;
		sp.opacity = 0.0f;
		sp.soot = particles.soots[si];
		sp.lifeMax = smokeLifeMax * (0.7f + 0.6f * unit(rng));
		secondaryParticles.push_back(sp);
		--smokeDeficit;
	}

	// ---- Advect + fade (pure kinematics, no SPH forces) ----------------------
	for (auto& sp : secondaryParticles) {
		sp.age += dt;

		// Swirl: rotate velocity around the inherited vorticity axis (same
		// cross-product shape as FlameParticle::applyVorticityConfinement(),
		// but purely kinematic -- no neighbor access) so secondary particles
		// keep curling after they decouple from the primary that spawned
		// them. Decays over ~0.5s so the effect fades rather than spinning
		// forever.
		//
		// Rotated exactly (Rodrigues) by angle |omega|*strength*dt about the
		// omega axis rather than the explicit v += (omega x v)*s*dt, which
		// lengthens v by a factor sqrt(1 + (|omega| s dt)^2) every step and
		// so made swirling particles accelerate without bound (plan A4).
		// To first order in dt the two are identical.
		if (secondarySwirlStrength > 0.0f && Math::getLength(sp.vorticity) > 1.0e-5f) {
			sp.velocity = rotateBySwirl(sp.velocity, sp.vorticity, secondarySwirlStrength, dt);
			sp.vorticity *= std::max(0.0f, 1.0f - 2.0f * dt);
		}

		if (sp.kind == SecondaryKind::Spark) {
			// Light "ember fall": weaker than full gravity, plus a little flicker
			// jitter. Cools toward ambient so the shared temperature-gradient
			// shader (see FlameApp) fades it from bright to dark as it dies.
			sp.velocity += gravity * 0.5f * dt;
			// Flicker as a velocity random walk: per-step kicks scaled by
			// sqrt(dt) so the accumulated spread after a given *time* does not
			// depend on the step size (plan A3). kSparkJitterRate matches the
			// old fixed +-0.15 kick at dt = 1/60 (0.15 * sqrt(60)).
			sp.velocity += Vector3df(jitterDist(rng), jitterDist(rng), jitterDist(rng)) * sparkJitterScale;
			sp.position += sp.velocity * dt;
			sp.temperature += (ambientTemperature - sp.temperature) * std::min(1.0f, 4.0f * dt);
		} else {
			sp.velocity += Vector3df(jitterDist(rng), 0.0f, jitterDist(rng)) * 0.3f * dt;
			sp.position += sp.velocity * dt;

			const float ageRatio = std::clamp(sp.age / sp.lifeMax, 0.0f, 1.0f);
			sp.size = smokeStartSize + (smokeEndSize - smokeStartSize) * ageRatio;
			// Cools over roughly its first second, independent of lifeMax, so
			// smoke reads as glowing soot near the flame before settling into
			// the shader's flat sooty tint for the rest of its (longer) life.
			sp.temperature += (ambientTemperature - sp.temperature) * std::min(1.0f, 1.0f * dt);

			// Fade in quickly, fade out slowly; capped at 0.5 so smoke stays
			// translucent even at its most opaque.
			const float envelope = (ageRatio < 0.15f)
				? ageRatio / 0.15f
				: (1.0f - (ageRatio - 0.15f) / 0.85f);
			sp.opacity = std::clamp(envelope, 0.0f, 1.0f) * 0.5f;
		}
	}

	// ---- Cull dead ------------------------------------------------------------
	for (size_t i = 0; i < secondaryParticles.size();) {
		if (secondaryParticles[i].age > secondaryParticles[i].lifeMax) {
			secondaryParticles[i] = secondaryParticles.back();
			secondaryParticles.pop_back();
			continue;
		}
		++i;
	}
}
