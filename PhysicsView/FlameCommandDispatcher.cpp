#include "pch.h"
#include "FlameCommandDispatcher.h"

#include <chrono>
#include "FlameWorld.h"
#include "RigidBodyWorld.h"

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
		{ "fixedCarriers", [](FlameWorld& w) { return w.fluid().fixedCarriers ? 1.0f : 0.0f; },
		  [](FlameWorld&, float) { return false; } },
#define THERMAL_PARAM(name, member) { name, [](FlameWorld& w) { return w.solver().getThermalBoundary().member; }, \
		[](FlameWorld& w, float v) { auto t=w.solver().getThermalBoundary(); t.member=v; return w.solver().setThermalBoundary(t); } }
		THERMAL_PARAM("sourcePower", sourcePower),
		THERMAL_PARAM("sourceDuration", sourceDuration),
		THERMAL_PARAM("sourceRadius", sourceRadius),
		THERMAL_PARAM("wallTemperature", wallTemperature),
		THERMAL_PARAM("wallRate", wallRate),
		THERMAL_PARAM("wallThickness", wallThickness),
		THERMAL_PARAM("oxygenRecoveryRate", oxygenRecoveryRate),
		THERMAL_PARAM("smokeDecayRate", smokeDecayRate),
		THERMAL_PARAM("velocityDampingRate", velocityDampingRate),
#undef THERMAL_PARAM
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
			if ((v != 0.0f && v != 1.0f) || (v==0.0f && !w.bodies().empty())) return false;
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
		{ "carrierDebug", [](FlameWorld& w) { return static_cast<float>(w.render().carrierDebug); },
		  [](FlameWorld& w, float v) { if(v<0 || v>2 || v!=std::floor(v)) return false; w.render().carrierDebug=static_cast<int>(v); return true; } },
		{ "sphereWire", [](FlameWorld& w) { return w.render().sphereWire?1.0f:0.0f; },
		  [](FlameWorld& w, float v) { if(v!=0 && v!=1) return false; w.render().sphereWire=v!=0; return true; } },
		{ "smokeParticleSize", FLAME_R(smokeParticleSize) },
		{ "renderScale", FLAME_R(renderScale) },
		// Offsets may be negative, so they bypass FLAME_R's v >= 0 check.
		{ "renderOffsetX", [](FlameWorld& w) { return w.render().renderOffset.x; },
			[](FlameWorld& w, float v) { w.render().renderOffset.x = v; return true; } },
		{ "renderOffsetY", [](FlameWorld& w) { return w.render().renderOffset.y; },
			[](FlameWorld& w, float v) { w.render().renderOffset.y = v; return true; } },
		{ "renderOffsetZ", [](FlameWorld& w) { return w.render().renderOffset.z; },
			[](FlameWorld& w, float v) { w.render().renderOffset.z = v; return true; } },
		{ "hazeEnabled", FLAME_R(hazeEnabled) },
		{ "hazeStrength", FLAME_R(hazeStrength) },
		{ "hazeExtent", FLAME_R(hazeExtent) },
		{ "hazeFrequency", FLAME_R(hazeFrequency) },
		{ "hazeRiseSpeed", FLAME_R(hazeRiseSpeed) },
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
		{ "smokeDensityProfile", [](FlameWorld& w) { return w.render().smokeDensityProfile; },
			[](FlameWorld& w, float v) { if (v<0 || v>1) return false; w.render().smokeDensityProfile=v; return true; } },
		{ "smokeShadowStrength", FLAME_R(smokeShadowStrength) },
		{ "smokeFlameLight", FLAME_R(smokeFlameLight) },
		{ "objectFlameLight", FLAME_R(objectFlameLight) },
		{ "smokeShadowAmbient", [](FlameWorld& w) { return w.render().smokeShadowAmbient; },
			[](FlameWorld& w, float v) { if (v < 0 || v > 1) return false; w.render().smokeShadowAmbient = v; return true; } },
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

void FlameCommandDispatcher::tick(double budgetMs)
{
	if (stepsRemaining_ <= 0 || !world_) return;
	const auto start = std::chrono::steady_clock::now();
	do {
		world_->stepOnce();
		--stepsRemaining_;
	} while (stepsRemaining_ > 0 &&
	         std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() < budgetMs);
	notifyChanged();
	if (stepsRemaining_ == 0 && deferredDone_) deferredDone_("OK");
}

std::optional<std::string> FlameCommandDispatcher::route(const std::string& cmd)
{
	const std::string_view sv(cmd);
	if (sv.find("Flame") == std::string_view::npos) {
		return std::nullopt;
	}
	if (!world_) return std::string("Error:flame world not set");
	auto& w = *world_;
	if (startsWith(sv, "FlameSpherePreset:")) {
		auto args=sv.substr(18); const auto comma=args.find(',');
		const auto mode=args.substr(0,comma);
		float radius=0.6f, spacing=0.08f;
		if (comma!=std::string_view::npos) {
			args.remove_prefix(comma+1); const auto split=args.find(',');
			if (split==std::string_view::npos || !parseFloat(args.substr(0,split),radius) ||
				!parseFloat(args.substr(split+1),spacing)) return "Error:expected mode[,radius,spacing]";
		}
		if ((mode!="convection" && mode!="combustion") || !w.sphericalPreset(mode=="combustion",radius,spacing))
			return "Error:invalid sphere preset";
		notifyChanged(); return "OK";
	}
    if (startsWith(sv,"BindFlameBody:")) {
        const auto args=sv.substr(14); const auto comma=args.find(','); int id,index;
        if(comma==std::string_view::npos || !parseInt(args.substr(0,comma),id) || id<=0 ||
            !parseInt(args.substr(comma+1),index) || index<0 || !w.bindRigidBody(id,index))
            return "Error:expected body ID,rigid index with matching Box/Sphere size and no rigid-fluid coupling";
        notifyChanged(); return "OK";
    }
    if (startsWith(sv,"UnbindFlameBody:")) {
        int id; if(!parseInt(sv.substr(16),id) || id<=0 || !w.unbindRigidBody(id)) return "Error:unknown binding";
        notifyChanged(); return "OK";
    }
    if (startsWith(sv,"SetFlameRigidVelocity:")) {
        auto rest=sv.substr(22); std::vector<std::string_view> args;
        while(true) { auto pos=rest.find(','); args.push_back(rest.substr(0,pos)); if(pos==std::string_view::npos) break; rest.remove_prefix(pos+1); }
        int id; float v[6];
        if(args.size()!=7 || !parseInt(args[0],id) || id<=0 || !w.rigidWorld()) return "Error:expected bound ID,vx,vy,vz,wx,wy,wz";
        auto* rigid=w.rigidWorld()->findBody(w.rigidBodyId(id)); if(!rigid) return "Error:unknown binding";
        for(int i=0;i<6;++i) if(!parseFloat(args[i+1],v[i])) return "Error:invalid velocity";
        rigid->linearVelocity={v[0],v[1],v[2]}; rigid->angularVelocity={v[3],v[4],v[5]};
        w.syncRigidBindings(); notifyChanged(); return "OK";
    }
    if (startsWith(sv,"SetFlameBodyPose:")) {
        auto rest=sv.substr(17); std::vector<std::string_view> args;
        while(true) { auto pos=rest.find(','); args.push_back(rest.substr(0,pos)); if(pos==std::string_view::npos) break; rest.remove_prefix(pos+1); }
        int id; float v[7];
        if(args.size()!=8 || !parseInt(args[0],id) || id<=0) return "Error:expected ID,x,y,z,qw,qx,qy,qz";
        auto* body=w.findBody(id); if(!body || w.rigidBodyId(id)) return "Error:unknown or bound body";
        for(int i=0;i<7;++i) if(!parseFloat(args[i+1],v[i])) return "Error:invalid pose";
        if(!body->setMotion({v[0],v[1],v[2]},{v[3],v[4],v[5],v[6]},{0,0,0},{0,0,0})) return "Error:invalid pose";
        body->beginMotionStep(); notifyChanged(); return "OK";
    }
	if (cmd == "FlameCombustionPreset") { w.combustionPreset(); notifyChanged(); return "OK"; }
	if (cmd == "StopFlameSource") { w.stopSource(); notifyChanged(); return "OK"; }
	if (cmd == "GetFlameBodyCount") return std::to_string(w.bodies().size());
	if (cmd == "GetFlameFuelBudget") {
		const auto stats=computeFlameStats(w.fluid(),&w.coupler(),&w.solver());
		return "initial="+std::to_string(stats.initialSolidFuel)+";solid="+std::to_string(stats.solidFuel)+";pending="+std::to_string(stats.pendingFuel)+
			";residue="+std::to_string(stats.residueMass)+";gas="+std::to_string(stats.gasFuel)+
			";burned="+std::to_string(w.fluid().burnedFuelMass)+";outflow="+std::to_string(w.fluid().outflowFuelMass)+
			";source="+std::to_string(w.fluid().sourceFuelMass)+";removed="+std::to_string(stats.removedSolidMass);
	}
	if (startsWith(sv,"AddFlameBody:")) {
		std::vector<std::string_view> args; auto rest=sv.substr(13);
		while (true) { const auto pos=rest.find(','); args.push_back(rest.substr(0,pos)); if(pos==std::string_view::npos) break; rest.remove_prefix(pos+1); }
        if ((args.size()!=9 && args.size()!=10) || (args[0]!="box" && args[0]!="sphere")) return "Error:expected shape,x,y,z,hx,hy,hz,fuel,preset[,resolution] (simulation units)";
		float values[7]; for(int i=0;i<7;++i) if(!parseFloat(args[i+1],values[i])) return "Error:bad body value";
        int preset; if(!parseInt(args[8],preset)) return "Error:bad preset";
        int resolution=2; if(args.size()==10 && !parseInt(args[9],resolution)) return "Error:bad resolution";
		const auto id=w.addBody(args[0]=="box"?CombustibleBody::Shape::Box:CombustibleBody::Shape::Sphere,
            {values[0],values[1],values[2]},{values[3],values[4],values[5]},values[6],preset,resolution);
		if (!id) return "Error:invalid body";
		notifyChanged(); return "Id:"+std::to_string(id);
	}
	if (startsWith(sv,"RemoveFlameBody:")) {
		int id; if(!parseInt(sv.substr(16),id) || id<=0 || !w.removeBody(id)) return "Error:unknown body";
		notifyChanged(); return "OK";
	}
	// Stats use a collection index so deterministic scenarios can inspect a
	// freshly rebuilt preset while IDs stay monotonic across reset/deletion.
	if (startsWith(sv,"GetFlameBodyStat:") || startsWith(sv,"SetFlameBodyParam:")) {
		const bool set=startsWith(sv,"SetFlameBodyParam:"); auto args=sv.substr(set?18:17);
		const auto comma=args.find(','); int index;
		if(comma==std::string_view::npos || !parseInt(args.substr(0,comma),index) || index<0 || index>=static_cast<int>(w.bodies().size())) return "Error:unknown body index";
		auto& body=*w.bodies()[index]; args.remove_prefix(comma+1);
        w.syncRigidBindings();
        if(!set) { double v;
            if(args=="rigidId") return std::to_string(w.rigidBodyId(body.id()));
            if(args=="x" || args=="y" || args=="z") return std::to_string(body.center()[args=="x"?0:args=="y"?1:2]);
            if(args=="qw" || args=="qx" || args=="qy" || args=="qz") {
                const auto q=body.orientation(); return std::to_string(args=="qw"?q.w:args=="qx"?q.x:args=="qy"?q.y:q.z);
            }
            if(args=="id") return std::to_string(body.id());
            if(args=="particleCount") return std::to_string(body.samples().size());
            if(args=="solidSPH") return body.usesSolidParticles()?"1":"0";
			if(!body.stats().get(std::string(args).c_str(),v)) return "Error:unknown solid stat"; return std::to_string(v); }
		const auto split=args.find(','); float value;
		if(split==std::string_view::npos || !parseFloat(args.substr(split+1),value)) return "Error:bad body parameter";
		const auto name=args.substr(0,split); auto m=body.material();
		if(name=="preset") { if(value<0 || value>3 || value!=std::floor(value)) return "Error:bad preset"; m=CombustibleMaterial::preset(static_cast<int>(value)); }
		else if(name=="pyrolysisTemperature") m.pyrolysisTemperature=value;
		else if(name=="pyrolysisRate") m.pyrolysisRate=value;
		else if(name=="conductivity") m.conductivity=value;
		else if(name=="specificHeat") m.specificHeat=value;
		else if(name=="density") m.density=value;
		else if(name=="latentHeat") m.latentHeat=value;
		else if(name=="residueFraction") m.residueFraction=value;
		else if(name=="combustible") { if(value!=0 && value!=1) return "Error:expected 0 or 1"; m.combustible=value!=0; }
		else if(name=="x" || name=="y" || name=="z") { if(w.rigidBodyId(body.id())) return "Error:unbind before editing pose"; auto p=body.center(); p[name=="x"?0:name=="y"?1:2]=value; body.setCenter(p); notifyChanged(); return "OK"; }
		else return "Error:unknown body parameter";
		if(!body.setMaterial(m)) return "Error:invalid material";
		notifyChanged(); return "OK";
	}

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
		if (n == 0) return std::string("OK");
		stepsRemaining_ = n;   // advanced by tick(); the answer is sent when it finishes
		return std::string();
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
			{ "shadowGenerated", static_cast<float>(s.shadowGenerated) },
			{ "shadowOverflowed", static_cast<float>(s.shadowOverflowed) },
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
		return computeFlameStats(w.fluid(),&w.coupler(),&w.solver()).toString();
	}
	if (startsWith(sv, "GetFlameStat:")) {
		const std::string name(sv.substr(13));
		float v = 0.0f;
		if (!computeFlameStats(w.fluid(),&w.coupler(),&w.solver()).get(name, v)) {
			return "Error:unknown stat '" + name + "' (known: " + FlameStats::names() + ")";
		}
		return formatFloat(v);
	}

	bool changed = false;
	if (w.fluid().fixedCarriers && startsWith(sv,"SetFlameParam:density,"))
		return "Error:fixed carrier mass requires rebuilding the preset";
	if (!w.bodies().empty() && cmd=="SetFlameParam:combustionModel,0") return "Error:solid combustion requires Physical mode";
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
