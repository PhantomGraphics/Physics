#include "CloudThermodynamics.h"

#include <algorithm>
#include <cmath>

namespace Phantom {
	namespace Physics {
		namespace CloudThermodynamics {

double saturationVaporPressure(double t)
{
	const double tc = t - 273.15;
	return 611.2 * std::exp(17.67 * tc / (tc + 243.5));
}

double saturationVaporPressureDerivative(double t)
{
	const double tc = t - 273.15;
	const double d = tc + 243.5;
	return saturationVaporPressure(t) * 17.67 * 243.5 / (d * d);
}

namespace {
double pressureMinusEs(double p, double es)
{
	return std::max(p - es, 1.0e-3 * p);  // guard against p <= e_s (far outside the warm-cloud regime)
}
}

double saturationMixingRatio(const CloudThermoParams& th, double t, double p)
{
	const double es = saturationVaporPressure(t);
	return th.epsilon * es / pressureMinusEs(p, es);
}

double environmentTemperature(const CloudEnvironmentParams& env, double z)
{
	return env.surfaceTemperature - env.lapseRate * (env.altitudeOrigin + z);
}

double environmentPressure(const CloudThermoParams& th, const CloudEnvironmentParams& env, double z)
{
	const double t = environmentTemperature(env, z);
	const double exponent = th.gravity / (th.gasConstantDry * env.lapseRate);
	return env.surfacePressure * std::pow(t / env.surfaceTemperature, exponent);
}

double environmentVaporMixingRatio(const CloudThermoParams& th, const CloudEnvironmentParams& env, double z)
{
	return env.relativeHumidity * saturationMixingRatio(th, environmentTemperature(env, z),
	                                                    environmentPressure(th, env, z));
}

double adiabaticTemperature(const CloudThermoParams& th, double t, double p0, double p1)
{
	return t * std::pow(p1 / p0, th.gasConstantDry / th.cpDry);
}

CloudSaturationResult saturationAdjust(const CloudThermoParams& th, double t, double qv, double qc, double p)
{
	CloudSaturationResult r;
	r.temperature = t;
	r.qv = qv;
	r.qc = qc;
	if (!std::isfinite(t) || !std::isfinite(qv) || !std::isfinite(qc) || !std::isfinite(p) ||
	    qv < 0.0 || qc < 0.0 || p <= 0.0 || t <= 0.0) {
		r.converged = false;
		return r;
	}
	const double lc = th.latentHeat / th.cpDry;
	// f(dq) = qv - dq - qvs(T + lc*dq) is strictly decreasing in dq, so the root is unique.
	auto f = [&](double dq) { return qv - dq - saturationMixingRatio(th, t + lc * dq, p); };

	double lo = -qc;
	double hi = qv;
	double dq = 0.0;
	if (f(lo) <= 0.0) {
		dq = lo;  // supersaturated-or-exact even after evaporating all cloud water -> all evaporates
	}
	else if (f(hi) >= 0.0) {
		dq = hi;  // cannot condense more than the available vapour
	}
	else {
		const double scale = std::max({ qv, qc, 1.0e-12 });
		dq = std::clamp(0.0, lo, hi);
		r.converged = false;
		for (int i = 0; i < th.maxIterations; ++i) {
			r.iterations = i + 1;
			const double fv = f(dq);
			if (fv > 0.0) lo = dq; else hi = dq;
			const double tt = t + lc * dq;
			const double es = saturationVaporPressure(tt);
			const double dn = pressureMinusEs(p, es);
			// d qvs / d dq = lc * eps * p * des/dT / (p - es)^2
			const double dqvs = lc * th.epsilon * p * saturationVaporPressureDerivative(tt) / (dn * dn);
			double next = dq + fv / (1.0 + dqvs);
			if (!(next > lo && next < hi)) next = 0.5 * (lo + hi);
			const bool done = std::abs(next - dq) <= th.relTolerance * scale;
			dq = next;
			if (done) { r.converged = true; break; }
		}
		if (!r.converged) {  // bisection fallback keeps the result inside the bracket
			for (int i = 0; i < 200 && hi - lo > th.relTolerance * scale; ++i) {
				dq = 0.5 * (lo + hi);
				if (f(dq) > 0.0) lo = dq; else hi = dq;
			}
			dq = 0.5 * (lo + hi);
			r.converged = (hi - lo <= 10.0 * th.relTolerance * scale);
		}
	}
	dq = std::clamp(dq, -qc, qv);
	r.condensed = dq;
	r.qv = qv - dq;
	r.qc = qc + dq;
	r.temperature = t + lc * dq;
	return r;
}

double buoyancy(const CloudThermoParams& th, double t, double qv, double qc, double tEnv, double qvEnv)
{
	const double epsV = 1.0 / th.epsilon - 1.0;
	return th.gravity * ((t - tEnv) / tEnv + epsV * (qv - qvEnv) - qc);
}

		}
	}
}
