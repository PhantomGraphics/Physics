#include "CloudParticle.h"
#include "CloudThermodynamics.h"

#include <algorithm>
#include <cmath>

namespace Phantom {
	namespace Physics {

using Math::Vector3dd;

void CloudParticleSoA::clear()
{
	ids.clear();
	positions.clear();
	velocities.clear();
	mDry.clear();
	densities.clear();
	temperatures.clear();
	qv.clear();
	qc.clear();
	nextId = 0;
}

size_t CloudParticleSoA::add(const Vector3dd& position, const Vector3dd& velocity, double massDry,
                             double temperature, double vapor, double cloudWater)
{
	ids.push_back(nextId++);
	positions.push_back(position);
	velocities.push_back(velocity);
	mDry.push_back(massDry);
	densities.push_back(0.0);
	temperatures.push_back(temperature);
	qv.push_back(vapor);
	qc.push_back(cloudWater);
	return positions.size() - 1;
}

namespace CloudOps {

void fillEnvironmentLattice(CloudParticleSoA& soa, const CloudThermoParams& th,
                            const CloudEnvironmentParams& env, const Vector3dd& lo,
                            const Vector3dd& hi, double spacing, CloudWaterLedger& ledger)
{
	if (!(spacing > 0.0)) return;
	const double vol = spacing * spacing * spacing;
	for (double z = lo.z + 0.5 * spacing; z < hi.z; z += spacing) {
		const double t = CloudThermodynamics::environmentTemperature(env, z);
		const double p = CloudThermodynamics::environmentPressure(th, env, z);
		const double q = CloudThermodynamics::environmentVaporMixingRatio(th, env, z);
		const double m = p / (th.gasConstantDry * t) * vol;
		for (double y = lo.y + 0.5 * spacing; y < hi.y; y += spacing) {
			for (double x = lo.x + 0.5 * spacing; x < hi.x; x += spacing) {
				soa.add(Vector3dd(x, y, z), env.wind, m, t, q, 0.0);
				ledger.initialWater += m * q;
			}
		}
	}
}

void applySources(CloudParticleSoA& soa, const std::vector<CloudSourceParams>& sources,
                  double time, double dt, CloudWaterLedger& ledger)
{
	std::vector<double> w(soa.size());
	for (const CloudSourceParams& s : sources) {
		if (!s.enabled || !(s.radius > 0.0)) continue;
		const double t0 = std::max(time, s.startTime);
		const double t1 = std::min(time + dt, s.endTime);
		const double activeDt = t1 - t0;
		if (!(activeDt > 0.0)) continue;

		double sumW = 0.0;
		for (size_t i = 0; i < soa.size(); ++i) {
			const Vector3dd d = soa.positions[i] - s.center;
			const double r2 = (d.x * d.x + d.y * d.y + d.z * d.z) / (s.radius * s.radius);
			const double f = std::max(1.0 - r2, 0.0);
			w[i] = f * f;
			sumW += w[i];
		}
		if (!(sumW > 0.0)) continue;

		for (size_t i = 0; i < soa.size(); ++i) {
			if (w[i] <= 0.0) continue;
			soa.temperatures[i] += s.heatingRate * activeDt * w[i];
			soa.qv[i] += s.vaporRate * activeDt * (w[i] / sumW) / soa.mDry[i];
		}
		ledger.suppliedWater += s.vaporRate * activeDt;
	}
}

void mixPair(CloudParticleSoA& soa, size_t i, size_t j, double a)
{
	if (i == j || !(a > 0.0)) return;
	a = std::min(a, 1.0);
	const double mi = soa.mDry[i];
	const double mj = soa.mDry[j];
	const double wi = mj / (mi + mj);  // share of the difference applied to i
	const double wj = mi / (mi + mj);
	auto exchange = [&](std::vector<double>& v) {
		const double d = a * (v[j] - v[i]);
		v[i] += d * wi;
		v[j] -= d * wj;
	};
	exchange(soa.temperatures);
	exchange(soa.qv);
	exchange(soa.qc);
}

void mixPairRelative(CloudParticleSoA& soa, size_t i, size_t j, double a, const CloudThermoParams& th,
                     const CloudEnvironmentParams& env)
{
	if (i == j || !(a > 0.0)) return;
	a = std::min(a, 1.0);
	const double mi = soa.mDry[i];
	const double mj = soa.mDry[j];
	const double wi = mj / (mi + mj);
	const double wj = mi / (mi + mj);

	const double zi = soa.positions[i].z, zj = soa.positions[j].z;
	const double tEi = CloudThermodynamics::environmentTemperature(env, zi);
	const double tEj = CloudThermodynamics::environmentTemperature(env, zj);
	const double qEi = CloudThermodynamics::environmentVaporMixingRatio(th, env, zi);
	const double qEj = CloudThermodynamics::environmentVaporMixingRatio(th, env, zj);

	const double dT = a * ((soa.temperatures[j] - tEj) - (soa.temperatures[i] - tEi));
	soa.temperatures[i] += dT * wi;
	soa.temperatures[j] -= dT * wj;

	const double dQ = a * ((soa.qv[j] - qEj) - (soa.qv[i] - qEi));
	if (soa.qv[i] + dQ * wi >= 0.0 && soa.qv[j] - dQ * wj >= 0.0) {
		soa.qv[i] += dQ * wi;
		soa.qv[j] -= dQ * wj;
	}

	const double dC = a * (soa.qc[j] - soa.qc[i]);
	soa.qc[i] += dC * wi;
	soa.qc[j] -= dC * wj;
}

size_t saturationAdjustAll(CloudParticleSoA& soa, const CloudThermoParams& th,
                           const CloudEnvironmentParams& env, CloudStats* stats)
{
	size_t failures = 0;
	for (size_t i = 0; i < soa.size(); ++i) {
		const double p = CloudThermodynamics::environmentPressure(th, env, soa.positions[i].z);
		const CloudSaturationResult r = CloudThermodynamics::saturationAdjust(
			th, soa.temperatures[i], soa.qv[i], soa.qc[i], p);
		if (!r.converged) {
			++failures;
			continue;
		}
		soa.temperatures[i] = r.temperature;
		soa.qv[i] = r.qv;
		soa.qc[i] = r.qc;
	}
	if (stats) stats->adjustFailures += failures;
	return failures;
}

CloudStats computeStats(const CloudParticleSoA& soa, const CloudWaterLedger& ledger, double qcThreshold)
{
	CloudStats s;
	s.particleCount = soa.size();
	double base = 1.0e300;
	double top = -1.0e300;
	for (size_t i = 0; i < soa.size(); ++i) {
		const Vector3dd& p = soa.positions[i];
		const Vector3dd& v = soa.velocities[i];
		const bool finite = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
		                    std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
		                    std::isfinite(soa.mDry[i]) && std::isfinite(soa.temperatures[i]) &&
		                    std::isfinite(soa.qv[i]) && std::isfinite(soa.qc[i]);
		if (!finite) { ++s.nonFiniteCount; continue; }
		if (soa.qv[i] < 0.0 || soa.qc[i] < 0.0 || soa.mDry[i] <= 0.0 || soa.temperatures[i] <= 0.0) {
			++s.negativeCount;
		}
		s.totalVapor += soa.mDry[i] * soa.qv[i];
		s.totalCloudWater += soa.mDry[i] * soa.qc[i];
		if (soa.qc[i] >= qcThreshold) {
			++s.cloudyParticles;
			base = std::min(base, p.z);
			top = std::max(top, p.z);
		}
	}
	s.totalWater = s.totalVapor + s.totalCloudWater;
	if (s.cloudyParticles > 0) {
		s.cloudBase = base;
		s.cloudTop = top;
	}
	s.expectedWater = ledger.initialWater + ledger.suppliedWater + ledger.inflowWater - ledger.outflowWater;
	s.balanceError = std::abs(s.totalWater - s.expectedWater) / std::max(s.expectedWater, 1.0e-30);
	return s;
}

}  // namespace CloudOps

	}
}
