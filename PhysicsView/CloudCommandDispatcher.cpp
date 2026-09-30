#include "pch.h"
#include "CloudCommandDispatcher.h"
#include "CloudWorld.h"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <functional>
#include <string_view>
#include <vector>

using namespace Phantom::Physics;

namespace Phantom {

namespace {

bool parseDouble(std::string_view sv, double& out)
{
	auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), out);
	return ec == std::errc{} && ptr == sv.data() + sv.size() && std::isfinite(out);
}

bool parseInt64(std::string_view sv, long long& out)
{
	auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), out);
	return ec == std::errc{} && ptr == sv.data() + sv.size();
}

std::vector<std::string_view> splitComma(std::string_view s)
{
	std::vector<std::string_view> out;
	size_t start = 0;
	while (true) {
		const size_t c = s.find(',', start);
		if (c == std::string_view::npos) { out.push_back(s.substr(start)); break; }
		out.push_back(s.substr(start, c - start));
		start = c + 1;
	}
	return out;
}

bool startsWith(std::string_view s, std::string_view p) { return s.substr(0, p.size()) == p; }

std::string fmt(double v)
{
	char buf[64];
	std::snprintf(buf, sizeof(buf), "%.9g", v);
	return buf;
}

struct ParamDef {
	const char* name;
	std::function<double(CloudWorld&)> get;
	std::function<bool(CloudWorld&, double)> set;   // false = rejected
};

// EXPR is a config member path (c = w.config()); VALID may use v (the new value).
#define CLOUD_PARAM(NAME, EXPR, VALID)                                                              \
	{ NAME, [](CloudWorld& w) { auto& c = w.config(); return static_cast<double>(EXPR); },         \
	  [](CloudWorld& w, double v) { auto& c = w.config(); if (!(VALID)) return false;               \
	                                EXPR = static_cast<std::remove_reference_t<decltype(EXPR)>>(v); return true; } }

#define CLOUD_TOGGLE(NAME, MEMBER)                                                          \
	{ NAME, [](CloudWorld& w) { return w.config().solver.MEMBER ? 1.0 : 0.0; },            \
	  [](CloudWorld& w, double v) { w.config().solver.MEMBER = v != 0.0; return true; } }

const std::vector<ParamDef>& params()
{
	static const std::vector<ParamDef> defs = {
		// Applied at the next CloudReset (lattice / environment):
		CLOUD_PARAM("spacing", c.solver.spacing, v >= 5.0),
		CLOUD_PARAM("domainX", c.solver.domainMax.x, v > 0.0),
		CLOUD_PARAM("domainY", c.solver.domainMax.y, v > 0.0),
		CLOUD_PARAM("domainZ", c.solver.domainMax.z, v > 0.0),
		CLOUD_PARAM("surfaceTemperature", c.environment.surfaceTemperature, v > 200.0 && v < 340.0),
		CLOUD_PARAM("lapseRate", c.environment.lapseRate, v > 0.0 && v < 0.02),
		CLOUD_PARAM("jitter", c.jitter, v >= 0.0 && v <= 0.3),
		// Live:
		CLOUD_PARAM("soundSpeed", c.solver.soundSpeed, v > 0.0),
		CLOUD_PARAM("viscosity", c.solver.viscosity, v >= 0.0),
		CLOUD_PARAM("windRelaxTime", c.solver.windRelaxTime, v >= 0.0),
		CLOUD_PARAM("mixRate", c.solver.mixRate, v >= 0.0),
		CLOUD_PARAM("wallRestitution", c.solver.wallRestitution, v >= 0.0 && v <= 1.0),
		CLOUD_PARAM("timeScale", c.timeScale, v > 0.0),
		CLOUD_PARAM("fixedTimeStep", c.fixedTimeStep, v >= 0.0),
		CLOUD_PARAM("maxSubstepsPerFrame", c.maxSubstepsPerFrame, v >= 1.0 && v <= 1000.0),
		CLOUD_PARAM("qcThreshold", c.qcThreshold, v >= 0.0),
		CLOUD_TOGGLE("enablePressure", enablePressure),
		CLOUD_TOGGLE("enableBuoyancy", enableBuoyancy),
		CLOUD_TOGGLE("enableAdiabatic", enableAdiabatic),
		CLOUD_TOGGLE("enableCondensation", enableCondensation),
		CLOUD_TOGGLE("enableMixing", enableMixing),
	};
	return defs;
}

std::string paramNames()
{
	std::string s;
	for (const auto& d : params()) { if (!s.empty()) s += ','; s += d.name; }
	return s;
}

struct StatDef { const char* name; std::function<double(const CloudWorld&, const CloudStats&)> get; };

const std::vector<StatDef>& statDefs()
{
	static const std::vector<StatDef> defs = {
		{ "simTime", [](const CloudWorld& w, const CloudStats&) { return w.simTime(); } },
		{ "stepCount", [](const CloudWorld& w, const CloudStats&) { return static_cast<double>(w.stepCount()); } },
		{ "particles", [](const CloudWorld&, const CloudStats& s) { return static_cast<double>(s.particleCount); } },
		{ "totalWater", [](const CloudWorld&, const CloudStats& s) { return s.totalWater; } },
		{ "totalVapor", [](const CloudWorld&, const CloudStats& s) { return s.totalVapor; } },
		{ "totalCloudWater", [](const CloudWorld&, const CloudStats& s) { return s.totalCloudWater; } },
		{ "cloudBase", [](const CloudWorld&, const CloudStats& s) { return s.cloudBase; } },
		{ "cloudTop", [](const CloudWorld&, const CloudStats& s) { return s.cloudTop; } },
		{ "cloudyParticles", [](const CloudWorld&, const CloudStats& s) { return static_cast<double>(s.cloudyParticles); } },
		{ "balanceError", [](const CloudWorld&, const CloudStats& s) { return s.balanceError; } },
		{ "nonFinite", [](const CloudWorld&, const CloudStats& s) { return static_cast<double>(s.nonFiniteCount); } },
		{ "negative", [](const CloudWorld&, const CloudStats& s) { return static_cast<double>(s.negativeCount); } },
		{ "adjustFailures", [](const CloudWorld&, const CloudStats& s) { return static_cast<double>(s.adjustFailures); } },
		{ "maxSpeed", [](const CloudWorld& w, const CloudStats&) { return w.diagnostics().maxSpeed; } },
		{ "maxDensityRatio", [](const CloudWorld& w, const CloudStats&) { return w.diagnostics().maxDensityRatio; } },
		{ "minDensityRatio", [](const CloudWorld& w, const CloudStats&) { return w.diagnostics().minDensityRatio; } },
		{ "overloadedFrames", [](const CloudWorld& w, const CloudStats&) { return static_cast<double>(w.overloadedFrames()); } },
		{ "droppedSimTime", [](const CloudWorld& w, const CloudStats&) { return w.droppedSimTime(); } },
	};
	return defs;
}

// cx,cy,cz,radius,heating,vapor,start,end starting at a[offset].
bool parseSource(const std::vector<std::string_view>& a, size_t offset, CloudSourceParams& s)
{
	if (a.size() != offset + 8) return false;
	double v[8];
	for (size_t i = 0; i < 8; ++i) {
		if (!parseDouble(a[offset + i], v[i])) return false;
	}
	if (v[3] <= 0.0 || v[7] < v[6]) return false;
	s.center = Math::Vector3dd(v[0], v[1], v[2]);
	s.radius = v[3];
	s.heatingRate = v[4];
	s.vaporRate = v[5];
	s.startTime = v[6];
	s.endTime = v[7];
	return true;
}

} // namespace

std::optional<std::string> CloudCommandDispatcher::route(const std::string& cmd)
{
	const std::string_view sv(cmd);
	if (sv.find("Cloud") == std::string_view::npos) return std::nullopt;
	if (!world_) return std::string("Error:cloud world not set");
	CloudWorld& w = *world_;
	w.ensureBuilt();

	if (cmd == "SetCloudPage:true" || cmd == "SetCloudPage:false") {
		if (!setPage_) return std::string("Error:cloud page hook not set");
		setPage_(cmd == "SetCloudPage:true");
		notifyChanged();
		return std::string("OK");
	}
	if (cmd == "IsCloudPage") return std::string(isPage_ && isPage_() ? "true" : "false");
	if (cmd == "CloudReset" || startsWith(sv, "CloudReset:")) {
		long long seed = -1;
		if (cmd != "CloudReset" && (!parseInt64(sv.substr(11), seed) || seed < 0)) return std::string("Error:bad seed");
		w.reset(seed);
		notifyChanged();
		return std::string("OK");
	}
	if (cmd == "CloudStep" || startsWith(sv, "CloudStep:")) {
		long long n = 1;
		if (cmd != "CloudStep" && (!parseInt64(sv.substr(10), n) || n < 0)) return std::string("Error:bad step count");
		for (long long i = 0; i < n; ++i) w.stepOnce();
		notifyChanged();
		return std::string("OK");
	}
	if (startsWith(sv, "CloudAdvance:")) {
		double s = 0.0;
		if (!parseDouble(sv.substr(13), s) || s < 0.0) return std::string("Error:bad seconds");
		w.advance(s);
		notifyChanged();
		return std::string("OK");
	}
	if (cmd == "SetCloudRunning:true" || cmd == "SetCloudRunning:false") {
		w.setRunning(cmd == "SetCloudRunning:true");
		return std::string("OK");
	}
	if (cmd == "IsCloudRunning") return std::string(w.isRunning() ? "true" : "false");

	if (startsWith(sv, "SetCloudParam:")) {
		const auto a = splitComma(sv.substr(14));
		if (a.size() != 2) return std::string("Error:expected <name>,<value>");
		for (const auto& d : params()) {
			if (a[0] != d.name) continue;
			double v = 0.0;
			if (!parseDouble(a[1], v)) return "Error:bad value '" + std::string(a[1]) + "'";
			if (!d.set(w, v)) return "Error:value out of range for '" + std::string(a[0]) + "'";
			return std::string("OK");
		}
		return "Error:unknown parameter '" + std::string(a[0]) + "'";
	}
	if (startsWith(sv, "GetCloudParam:")) {
		const auto name = sv.substr(14);
		if (name == "list") return paramNames();
		for (const auto& d : params()) {
			if (name == d.name) return fmt(d.get(w));
		}
		return "Error:unknown parameter '" + std::string(name) + "'";
	}

	if (startsWith(sv, "AddCloudSource:")) {
		CloudSourceParams s;
		if (!parseSource(splitComma(sv.substr(15)), 0, s)) {
			return std::string("Error:expected cx,cy,cz,radius,heatingRate,vaporRate,startTime,endTime (radius>0, end>=start)");
		}
		const size_t i = w.addSource(s);
		notifyChanged();
		return "Index:" + std::to_string(i);
	}
	if (startsWith(sv, "SetCloudSource:")) {
		const auto a = splitComma(sv.substr(15));
		long long idx = 0;
		CloudSourceParams s;
		if (a.empty() || !parseInt64(a[0], idx) || idx < 0 || !parseSource(a, 1, s)) {
			return std::string("Error:expected <i>,cx,cy,cz,radius,heatingRate,vaporRate,startTime,endTime");
		}
		if (!w.setSource(static_cast<size_t>(idx), s)) return std::string("Error:no such source");
		notifyChanged();
		return std::string("OK");
	}
	if (startsWith(sv, "RemoveCloudSource:")) {
		long long idx = 0;
		if (!parseInt64(sv.substr(18), idx) || idx < 0 || !w.removeSource(static_cast<size_t>(idx))) {
			return std::string("Error:no such source");
		}
		notifyChanged();
		return std::string("OK");
	}
	if (cmd == "ClearCloudSources") { w.clearSources(); return std::string("OK"); }
	if (cmd == "GetCloudSourceCount") return "Count:" + std::to_string(w.sources().size());

	if (startsWith(sv, "SetCloudHumidity:")) {
		double rh = 0.0;
		if (!parseDouble(sv.substr(17), rh) || rh < 0.0 || rh > 1.0) return std::string("Error:expected 0..1");
		w.setRelativeHumidity(rh);
		notifyChanged();
		return std::string("OK");
	}
	if (startsWith(sv, "SetCloudWind:")) {
		const auto a = splitComma(sv.substr(13));
		double x = 0.0, y = 0.0, z = 0.0;
		if (a.size() != 3 || !parseDouble(a[0], x) || !parseDouble(a[1], y) || !parseDouble(a[2], z)) {
			return std::string("Error:expected x,y,z");
		}
		w.setWind(Math::Vector3dd(x, y, z));
		return std::string("OK");
	}

	if (cmd == "GetCloudParticleCount") return "Count:" + std::to_string(w.particles().size());
	if (cmd == "GetCloudSimTime") return fmt(w.simTime());
	if (cmd == "GetCloudCsvHeader") return CloudWorld::csvHeader();
	if (cmd == "GetCloudStats") return w.csvRow();
	if (startsWith(sv, "GetCloudStat:")) {
		const auto name = sv.substr(13);
		const CloudStats s = w.stats();
		std::string known;
		for (const auto& d : statDefs()) {
			if (name == d.name) return fmt(d.get(w, s));
			if (!known.empty()) known += ',';
			known += d.name;
		}
		return "Error:unknown stat '" + std::string(name) + "' (known: " + known + ")";
	}
	return std::nullopt;
}

} // namespace Phantom
