#include "pch.h"
#include "FlameCommandDispatcher.h"
#include "FlameWorld.h"

#include "../Physics/FlameStats.h"

#include <charconv>
#include <cmath>
#include <vector>

using namespace Phantom::Physics;

namespace Phantom {

namespace {

bool parseFloat(std::string_view sv, float& out)
{
	auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), out);
	return ec == std::errc{} && ptr == sv.data() + sv.size() && std::isfinite(out);
}

bool parseInt(std::string_view sv, int& out)
{
	auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), out);
	return ec == std::errc{} && ptr == sv.data() + sv.size();
}

std::string formatFloat(float v)
{
	return std::to_string(v);
}

struct ParamDef {
	const char* name;
	std::function<float(FlameWorld&)> get;
	std::function<bool(FlameWorld&, float)> set; // false = rejected value
};

// Setter helpers: most parameters only need "non-negative" or "positive".
template <class Fn>
std::function<bool(FlameWorld&, float)> nonNeg(Fn fn)
{
	return [fn](FlameWorld& w, float v) { if (v < 0.0f) return false; fn(w, v); return true; };
}
template <class Fn>
std::function<bool(FlameWorld&, float)> positive(Fn fn)
{
	return [fn](FlameWorld& w, float v) { if (v <= 0.0f) return false; fn(w, v); return true; };
}

FlameFluid::Emitter* firstEmitter(FlameWorld& w)
{
	auto& e = w.fluid().getEmittersMutable();
	return e.empty() ? nullptr : &e.front();
}

std::function<bool(FlameWorld&, float)> emitterSetter(float FlameFluid::Emitter::* member, bool allowNegative)
{
	return [member, allowNegative](FlameWorld& w, float v) {
		auto* e = firstEmitter(w);
		if (!e || (!allowNegative && v < 0.0f)) return false;
		e->*member = v;
		return true;
	};
}
std::function<float(FlameWorld&)> emitterGetter(float FlameFluid::Emitter::* member)
{
	return [member](FlameWorld& w) { auto* e = firstEmitter(w); return e ? e->*member : 0.0f; };
}

#define FLAME_F(NAME, GETTER) [](FlameWorld& w) { return static_cast<float>(w.fluid().GETTER()); }

const std::vector<ParamDef>& simParams()
{
	static const std::vector<ParamDef> defs = {
		{ "density", FLAME_F(density, getDensity), positive([](FlameWorld& w, float v) { w.fluid().setDensity(v); }) },
		{ "pressureCoe", FLAME_F(pressureCoe, getPressureCoe), nonNeg([](FlameWorld& w, float v) { w.fluid().setPressureCoe(v); }) },
		{ "viscosityCoe", FLAME_F(viscosityCoe, getViscosityCoe), nonNeg([](FlameWorld& w, float v) { w.fluid().setVicosityCoe(v); }) },
		{ "effectLength", FLAME_F(effectLength, getEffectLength), positive([](FlameWorld& w, float v) {
			w.fluid().setEffectLength(v);
			w.solver().setEffectLength(v);
		}) },
		{ "ambientTemperature", FLAME_F(ambientTemperature, getAmbientTemperature), nonNeg([](FlameWorld& w, float v) { w.fluid().setAmbientTemperature(v); }) },
		{ "ignitionTemperature", FLAME_F(ignitionTemperature, getIgnitionTemperature), positive([](FlameWorld& w, float v) { w.fluid().setIgnitionTemperature(v); }) },
		{ "burnRate", FLAME_F(burnRate, getBurnRate), nonNeg([](FlameWorld& w, float v) { w.fluid().setBurnRate(v); }) },
		{ "heatRelease", FLAME_F(heatRelease, getHeatRelease), nonNeg([](FlameWorld& w, float v) { w.fluid().setHeatRelease(v); }) },
		{ "coolRate", FLAME_F(coolRate, getCoolRate), nonNeg([](FlameWorld& w, float v) { w.fluid().setCoolRate(v); }) },
		{ "sootYield", FLAME_F(sootYield, getSootYield), nonNeg([](FlameWorld& w, float v) { w.fluid().setSootYield(v); }) },
		{ "buoyancyCoe", FLAME_F(buoyancyCoe, getBuoyancyCoe), nonNeg([](FlameWorld& w, float v) { w.fluid().setBuoyancyCoe(v); }) },
		{ "thermalExpansion", FLAME_F(thermalExpansion, getThermalExpansion), nonNeg([](FlameWorld& w, float v) { w.fluid().setThermalExpansion(v); }) },
		{ "vorticityEps", FLAME_F(vorticityEps, getVorticityEps), nonNeg([](FlameWorld& w, float v) { w.fluid().setVorticityEps(v); }) },
		{ "curlNoiseStrength", FLAME_F(curlNoiseStrength, getCurlNoiseStrength), nonNeg([](FlameWorld& w, float v) { w.fluid().setCurlNoiseStrength(v); }) },
		{ "curlNoiseFrequency", FLAME_F(curlNoiseFrequency, getCurlNoiseFrequency), nonNeg([](FlameWorld& w, float v) { w.fluid().setCurlNoiseFrequency(v); }) },
		{ "curlNoiseTimeScale", FLAME_F(curlNoiseTimeScale, getCurlNoiseTimeScale), nonNeg([](FlameWorld& w, float v) { w.fluid().setCurlNoiseTimeScale(v); }) },
		{ "lifeMax", FLAME_F(lifeMax, getLifeMax), positive([](FlameWorld& w, float v) { w.fluid().setLifeMax(v); }) },
		{ "maxSpeed", FLAME_F(maxSpeed, getMaxSpeed), positive([](FlameWorld& w, float v) { w.fluid().setMaxSpeed(v); }) },
		{ "maxParticles", FLAME_F(maxParticles, getMaxParticles), nonNeg([](FlameWorld& w, float v) { w.fluid().setMaxParticles(static_cast<int>(v)); }) },
		{ "sparkCountPerPrimary", FLAME_F(sparkCountPerPrimary, getSparkCountPerPrimary), nonNeg([](FlameWorld& w, float v) { w.fluid().setSparkCountPerPrimary(v); }) },
		{ "smokeCountPerPrimary", FLAME_F(smokeCountPerPrimary, getSmokeCountPerPrimary), nonNeg([](FlameWorld& w, float v) { w.fluid().setSmokeCountPerPrimary(v); }) },
		{ "secondarySwirlStrength", FLAME_F(secondarySwirlStrength, getSecondarySwirlStrength), nonNeg([](FlameWorld& w, float v) { w.fluid().setSecondarySwirlStrength(v); }) },
		{ "maxSecondaryParticles", FLAME_F(maxSecondaryParticles, getMaxSecondaryParticles), nonNeg([](FlameWorld& w, float v) { w.fluid().setMaxSecondaryParticles(static_cast<int>(v)); }) },
		// Physical combustion model (plan Phase 2).
		{ "combustionModel",
		  [](FlameWorld& w) { return w.fluid().getCombustionModel() == FlameFluid::CombustionModel::Physical ? 1.0f : 0.0f; },
		  [](FlameWorld& w, float v) {
			if (v != 0.0f && v != 1.0f) return false;
			w.fluid().setCombustionModel(v == 1.0f ? FlameFluid::CombustionModel::Physical : FlameFluid::CombustionModel::Legacy);
			return true;
		  } },
		{ "reactionRateModel",
		  [](FlameWorld& w) { return w.fluid().getReactionRateModel() == FlameFluid::ReactionRateModel::Arrhenius ? 1.0f : 0.0f; },
		  [](FlameWorld& w, float v) {
			if (v != 0.0f && v != 1.0f) return false;
			w.fluid().setReactionRateModel(v == 1.0f ? FlameFluid::ReactionRateModel::Arrhenius : FlameFluid::ReactionRateModel::Smoothstep);
			return true;
		  } },
		{ "ignitionWidth", FLAME_F(ignitionWidth, getIgnitionWidth), nonNeg([](FlameWorld& w, float v) { w.fluid().setIgnitionWidth(v); }) },
		{ "activationTemperature", FLAME_F(activationTemperature, getActivationTemperature), nonNeg([](FlameWorld& w, float v) { w.fluid().setActivationTemperature(v); }) },
		{ "oxygenPerFuel", FLAME_F(oxygenPerFuel, getOxygenPerFuel), nonNeg([](FlameWorld& w, float v) { w.fluid().setOxygenPerFuel(v); }) },
		{ "maxTemperature", FLAME_F(maxTemperature, getMaxTemperature), positive([](FlameWorld& w, float v) { w.fluid().setMaxTemperature(v); }) },
		{ "thermalDiffusivity", FLAME_F(thermalDiffusivity, getThermalDiffusivity), nonNeg([](FlameWorld& w, float v) { w.fluid().setThermalDiffusivity(v); }) },
		{ "fuelDiffusivity", FLAME_F(fuelDiffusivity, getFuelDiffusivity), nonNeg([](FlameWorld& w, float v) { w.fluid().setFuelDiffusivity(v); }) },
		{ "oxygenDiffusivity", FLAME_F(oxygenDiffusivity, getOxygenDiffusivity), nonNeg([](FlameWorld& w, float v) { w.fluid().setOxygenDiffusivity(v); }) },
		{ "sootDiffusivity", FLAME_F(sootDiffusivity, getSootDiffusivity), nonNeg([](FlameWorld& w, float v) { w.fluid().setSootDiffusivity(v); }) },
		{ "thermalExpansionPressure",
		  [](FlameWorld& w) { return w.fluid().getThermalExpansionPressure() ? 1.0f : 0.0f; },
		  [](FlameWorld& w, float v) {
			if (v != 0.0f && v != 1.0f) return false;
			w.fluid().setThermalExpansionPressure(v == 1.0f);
			return true;
		  } },
		// Emitter 0 (the default scene has exactly one).
		{ "emitterRate", emitterGetter(&FlameFluid::Emitter::rate), emitterSetter(&FlameFluid::Emitter::rate, false) },
		{ "emitterAirRate", emitterGetter(&FlameFluid::Emitter::airRate), emitterSetter(&FlameFluid::Emitter::airRate, false) },
		{ "emitterRadius", emitterGetter(&FlameFluid::Emitter::radius), emitterSetter(&FlameFluid::Emitter::radius, false) },
		{ "emitterFuelTemperature", emitterGetter(&FlameFluid::Emitter::fuelTemperature), emitterSetter(&FlameFluid::Emitter::fuelTemperature, true) },
		{ "emitterPilotTemperature", emitterGetter(&FlameFluid::Emitter::pilotTemperature), emitterSetter(&FlameFluid::Emitter::pilotTemperature, false) },
		{ "emitterPilotHeight", emitterGetter(&FlameFluid::Emitter::pilotHeight), emitterSetter(&FlameFluid::Emitter::pilotHeight, false) },
		// World level (survive FlameReset).
		{ "timeStep", [](FlameWorld& w) { return w.getTimeStep(); },
		  [](FlameWorld& w, float v) { if (v <= 0.0f || v > 0.1f) return false; w.setTimeStep(v); return true; } },
		{ "randomSeed", [](FlameWorld&) { return 0.0f; },
		  [](FlameWorld& w, float v) { if (v < 0.0f) return false; w.fluid().setRandomSeed(static_cast<unsigned int>(v)); return true; } },
	};
	return defs;
}

#undef FLAME_F

#define FLAME_R(FIELD) \
	[](FlameWorld& w) { return static_cast<float>(w.render().FIELD); }, \
	[](FlameWorld& w, float v) { if (v < 0.0f) return false; w.render().FIELD = static_cast<decltype(w.render().FIELD)>(v); return true; }

const std::vector<ParamDef>& renderParams()
{
	static const std::vector<ParamDef> defs = {
		{ "particleSize", FLAME_R(particleSize) },
		{ "smokeParticleSize", FLAME_R(smokeParticleSize) },
		{ "renderScale", FLAME_R(renderScale) },
		{ "exposure", FLAME_R(exposure) },
		{ "autoReferenceTemperature", FLAME_R(autoReferenceTemperature) },
		{ "referenceTemperature", FLAME_R(referenceTemperature) },
		{ "whiteBalance", FLAME_R(whiteBalance) },
		{ "whiteBalanceTemperature", FLAME_R(whiteBalanceTemperature) },
		{ "whiteBalanceDegree", FLAME_R(whiteBalanceDegree) },
		{ "smokeOpacityScale", FLAME_R(smokeOpacityScale) },
		{ "smokeExtinction", FLAME_R(smokeExtinction) },
		{ "smokeGlow", FLAME_R(smokeGlow) },
		{ "smokeAlbedo", FLAME_R(smokeAlbedo) },
		{ "pbvrAdaptive", FLAME_R(pbvrAdaptive) },
		{ "pbvrEnsembles", FLAME_R(pbvrEnsembles) },
		{ "pbvrTargetEnsembles", FLAME_R(pbvrTargetEnsembles) },
		{ "pbvrTemporalFrames", FLAME_R(pbvrTemporalFrames) },
		{ "pbvrBudgetMs", FLAME_R(pbvrBudgetMs) },
		{ "pbvrSubdivision", FLAME_R(pbvrSubdivision) },
		{ "pbvrMinSubPixels", FLAME_R(pbvrMinSubPixels) },
		{ "pbvrDensityScale", FLAME_R(pbvrDensityScale) },
	};
	return defs;
}

#undef FLAME_R

const ParamDef* findParam(const std::vector<ParamDef>& defs, std::string_view name)
{
	for (const auto& d : defs) {
		if (name == d.name) return &d;
	}
	return nullptr;
}

std::string listNames(const std::vector<ParamDef>& defs)
{
	std::string s;
	for (const auto& d : defs) {
		if (!s.empty()) s += ',';
		s += d.name;
	}
	return s;
}

bool startsWith(std::string_view s, std::string_view prefix)
{
	return s.substr(0, prefix.size()) == prefix;
}

// Shared "Set*Param:<name>,<value>" / "Get*Param:<name>" handling.
std::optional<std::string> handleParam(FlameWorld& w, const std::vector<ParamDef>& defs,
	std::string_view sv, std::string_view setPrefix, std::string_view getPrefix, bool& changed)
{
	if (startsWith(sv, setPrefix)) {
		const auto args = sv.substr(setPrefix.size());
		const auto comma = args.find(',');
		if (comma == std::string_view::npos) {
			return "Error:expected <name>,<value>";
		}
		const auto name = args.substr(0, comma);
		const auto* def = findParam(defs, name);
		if (!def) return "Error:unknown parameter '" + std::string(name) + "'";
		float v = 0.0f;
		if (!parseFloat(args.substr(comma + 1), v)) return "Error:bad value '" + std::string(args.substr(comma + 1)) + "'";
		if (!def->set(w, v)) return "Error:value out of range for '" + std::string(name) + "'";
		changed = true;
		return std::string("OK");
	}
	if (startsWith(sv, getPrefix)) {
		const auto name = sv.substr(getPrefix.size());
		if (name == "list") return listNames(defs);
		const auto* def = findParam(defs, name);
		if (!def) return "Error:unknown parameter '" + std::string(name) + "'";
		return formatFloat(def->get(w));
	}
	return std::nullopt;
}

} // namespace

std::optional<std::string> FlameCommandDispatcher::route(const std::string& cmd)
{
	const std::string_view sv(cmd);
	if (sv.find("Flame") == std::string_view::npos) {
		return std::nullopt;
	}
	if (!world_) return std::string("Error:flame world not set");
	auto& w = *world_;

	if (cmd == "SetFlamePage:true" || cmd == "SetFlamePage:false") {
		if (!setFlamePage_) return std::string("Error:flame page hook not set");
		setFlamePage_(cmd == "SetFlamePage:true");
		notifyChanged();
		return std::string("OK");
	}
	if (cmd == "IsFlamePage") {
		return std::string(isFlamePage_ && isFlamePage_() ? "true" : "false");
	}
	if (cmd == "FlameReset") {
		w.reset();
		notifyChanged();
		return std::string("OK");
	}
	if (cmd == "FlameStep" || startsWith(sv, "FlameStep:")) {
		int n = 1;
		if (cmd != "FlameStep" && (!parseInt(sv.substr(10), n) || n < 0)) {
			return std::string("Error:bad step count");
		}
		for (int i = 0; i < n; ++i) w.stepOnce();
		notifyChanged();
		return std::string("OK");
	}
	if (cmd == "SetFlameRunning:true" || cmd == "SetFlameRunning:false") {
		w.setRunning(cmd == "SetFlameRunning:true");
		return std::string("OK");
	}
	if (cmd == "IsFlameRunning") {
		return std::string(w.isRunning() ? "true" : "false");
	}
	if (cmd == "SetFlameRenderMode:Normal" || cmd == "SetFlameRenderMode:PBVR") {
		w.render().pbvrMode = (cmd == "SetFlameRenderMode:PBVR");
		notifyChanged();
		return std::string("OK");
	}
	if (startsWith(sv, "SetFlameRenderMode:")) {
		return std::string("Error:expected Normal or PBVR");
	}
	if (cmd == "GetFlameRenderMode") {
		return std::string(w.render().pbvrMode ? "PBVR" : "Normal");
	}
	if (cmd == "GetFlameParticleCount") {
		return "Count:" + std::to_string(w.fluid().getNumParticles());
	}
	if (cmd == "GetFlameSecondaryCount") {
		return "Count:" + std::to_string(w.fluid().getSecondaryParticles().size());
	}
	if (cmd == "GetFlameSimTime") {
		return formatFloat(w.getSimTime());
	}
	if (cmd == "GetFlamePBVRStats" || startsWith(sv, "GetFlamePBVRStat:")) {
		if (!pbvrStats_) return std::string("Error:PBVR renderer not available");
		const auto s = pbvrStats_();
		const std::pair<const char*, float> values[] = {
			{ "displayedEnsembles", static_cast<float>(s.displayedEnsembles) },
			{ "ensemblesThisFrame", static_cast<float>(s.ensemblesThisFrame) },
			{ "generated", static_cast<float>(s.generated) },
			{ "overflowed", static_cast<float>(s.overflowed) },
			{ "gpuMs", s.gpuMs },
			{ "timestampsSupported", s.timestampsSupported ? 1.0f : 0.0f },
			{ "lodState", static_cast<float>(s.lodState) },
		};
		if (cmd == "GetFlamePBVRStats") {
			std::string out;
			for (const auto& [k, v] : values) {
				if (!out.empty()) out += ';';
				out += std::string(k) + "=" + formatFloat(v);
			}
			return out;
		}
		const auto name = sv.substr(17);
		for (const auto& [k, v] : values) {
			if (name == k) return formatFloat(v);
		}
		return "Error:unknown PBVR stat '" + std::string(name) + "'";
	}
	if (cmd == "GetFlameStats") {
		return computeFlameStats(w.fluid()).toString();
	}
	if (startsWith(sv, "GetFlameStat:")) {
		const std::string name(sv.substr(13));
		float v = 0.0f;
		if (!computeFlameStats(w.fluid()).get(name, v)) {
			return "Error:unknown stat '" + name + "' (known: " + FlameStats::names() + ")";
		}
		return formatFloat(v);
	}

	bool changed = false;
	if (auto r = handleParam(w, simParams(), sv, "SetFlameParam:", "GetFlameParam:", changed)) {
		return r;
	}
	if (auto r = handleParam(w, renderParams(), sv, "SetFlameRenderParam:", "GetFlameRenderParam:", changed)) {
		if (changed) notifyChanged();
		return r;
	}
	return std::nullopt;
}

} // namespace Phantom
