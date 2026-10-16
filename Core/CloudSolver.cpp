#include "CloudSolver.h"
#include "CloudThermodynamics.h"

#include <algorithm>
#include <cmath>

namespace Phantom {
	namespace Physics {

using Math::Vector3dd;
using Math::Vector3df;
namespace CT = CloudThermodynamics;

namespace {
constexpr double kPi = 3.14159265358979323846;

double dot3(const Vector3dd& a, const Vector3dd& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

std::vector<Vector3df> toFloat(const std::vector<Vector3dd>& in)
{
	std::vector<Vector3df> out(in.size());
	for (size_t i = 0; i < in.size(); ++i) {
		out[i] = Vector3df(static_cast<float>(in[i].x), static_cast<float>(in[i].y), static_cast<float>(in[i].z));
	}
	return out;
}
}

double CloudSolver::kernel(double r, double h) const
{
	const double q = r / h;
	if (q >= 1.0) return 0.0;
	const double sigma = 8.0 / (kPi * h * h * h);
	if (q <= 0.5) return sigma * (1.0 - 6.0 * q * q + 6.0 * q * q * q);
	const double o = 1.0 - q;
	return sigma * 2.0 * o * o * o;
}

double CloudSolver::kernelGradMag(double r, double h) const
{
	const double q = r / h;
	if (q >= 1.0) return 0.0;
	const double sigma = 8.0 / (kPi * h * h * h);
	if (q <= 0.5) return sigma * (-12.0 * q + 18.0 * q * q) / h;
	const double o = 1.0 - q;
	return sigma * (-6.0 * o * o) / h;
}

void CloudSolver::initialize(const CloudParticleSoA& soa)
{
	const size_t n = soa.size();
	const double h = params.effectLength();
	neighbors_.build(toFloat(soa.positions), static_cast<float>(h));
	restNumber_.assign(n, 0.0);
	for (size_t i = 0; i < n; ++i) {
		double sum = kernel(0.0, h);
		for (int j : neighbors_[i]) {
			const Vector3dd d = soa.positions[i] - soa.positions[static_cast<size_t>(j)];
			sum += kernel(std::sqrt(dot3(d, d)), h);
		}
		restNumber_[i] = sum;
	}
	number_.assign(n, 0.0);
	pressure_.assign(n, 0.0);
	acc_.assign(n, Vector3dd(0.0));
}

double CloudSolver::stableTimeStep(const CloudParticleSoA& soa) const
{
	const double h = params.effectLength();
	double dt = params.cfl * h / std::max(params.soundSpeed, 1.0e-9);
	double vmax = 0.0;
	for (const Vector3dd& v : soa.velocities) vmax = std::max(vmax, std::sqrt(dot3(v, v)));
	if (vmax > 0.0) dt = std::min(dt, params.cfl * h / vmax);
	if (params.viscosity > 0.0) dt = std::min(dt, 0.125 * h * h / params.viscosity);
	if (params.enableMixing && params.mixDiffusivity > 0.0) {
		dt = std::min(dt, params.spacing * params.spacing / (6.0 * params.mixDiffusivity));
	}
	if (diag_.maxAcceleration > 0.0) dt = std::min(dt, 0.4 * std::sqrt(h / diag_.maxAcceleration));
	return dt;
}

bool CloudSolver::step(CloudParticleSoA& soa, const std::vector<CloudSourceParams>& sources, double time,
                       double dt, CloudWaterLedger& ledger)
{
	const size_t n = soa.size();
	if (n == 0 || !(dt > 0.0)) return false;
	if (restNumber_.size() != n) initialize(soa);
	const double h = params.effectLength();
	const double h2 = h * h;
	const double c2 = params.soundSpeed * params.soundSpeed;

	// 1. Neighbors and number density.
	neighbors_.build(toFloat(soa.positions), static_cast<float>(h));
	const double w0 = kernel(0.0, h);
	for (size_t i = 0; i < n; ++i) {
		double sum = w0;
		for (int j : neighbors_[i]) {
			const Vector3dd d = soa.positions[i] - soa.positions[static_cast<size_t>(j)];
			sum += kernel(std::sqrt(dot3(d, d)), h);
		}
		number_[i] = sum;
		soa.densities[i] = soa.mDry[i] * sum;
		pressure_[i] = c2 * soa.mDry[i] * (sum - restNumber_[i]);
	}

	// 2. Accelerations: pressure, viscosity, buoyancy, wind relaxation.
	double maxRatio = 0.0, minRatio = 1.0e300, maxAcc = 0.0;
	for (size_t i = 0; i < n; ++i) {
		Vector3dd a(0.0);
		const double rhoI = soa.densities[i];
		for (int jj : neighbors_[i]) {
			const size_t j = static_cast<size_t>(jj);
			const Vector3dd d = soa.positions[i] - soa.positions[j];
			const double r2 = dot3(d, d);
			const double r = std::sqrt(r2);
			if (r < 1.0e-12) continue;
			const double dw = kernelGradMag(r, h);
			const Vector3dd gradW = d * (dw / r);
			const double rhoJ = soa.densities[j];
			if (params.enablePressure) {
				const double coef = pressure_[i] / (rhoI * rhoI) + pressure_[j] / (rhoJ * rhoJ);
				a -= gradW * (soa.mDry[j] * coef);
			}
			if (params.viscosity > 0.0) {
				const Vector3dd vij = soa.velocities[j] - soa.velocities[i];
				a += vij * (2.0 * params.viscosity * (soa.mDry[j] / rhoJ) * dot3(d, gradW) / (r2 + 0.01 * h2));
			}
		}
		const double z = soa.positions[i].z;
		if (params.enableBuoyancy) {
			const double tEnv = CT::environmentTemperature(environment, z);
			const double qvEnv = CT::environmentVaporMixingRatio(thermo, environment, z);
			a.z += CT::buoyancy(thermo, soa.temperatures[i], soa.qv[i], soa.qc[i], tEnv, qvEnv);
		}
		if (params.windRelaxTime > 0.0) {
			a -= (soa.velocities[i] - environment.wind) / params.windRelaxTime;
		}
		acc_[i] = a;
		maxAcc = std::max(maxAcc, std::sqrt(dot3(a, a)));
		const double ratio = number_[i] / restNumber_[i];
		maxRatio = std::max(maxRatio, ratio);
		minRatio = std::min(minRatio, ratio);
	}
	diag_.maxAcceleration = maxAcc;
	diag_.maxDensityRatio = maxRatio;
	diag_.minDensityRatio = minRatio;

	// 3. Integrate (symplectic Euler) and closed-wall handling.
	double maxSpeed = 0.0;
	std::vector<double> zBefore(n);
	for (size_t i = 0; i < n; ++i) {
		zBefore[i] = soa.positions[i].z;
		soa.velocities[i] += acc_[i] * dt;
		soa.positions[i] += soa.velocities[i] * dt;
		for (int k = 0; k < 3; ++k) {
			double& x = soa.positions[i][k];
			double& v = soa.velocities[i][k];
			const double lo = params.domainMin[k], hi = params.domainMax[k];
			if (x < lo) { x = lo; if (v < 0.0) v = -params.wallRestitution * v; ++diag_.wallClamps; }
			else if (x > hi) { x = hi; if (v > 0.0) v = -params.wallRestitution * v; ++diag_.wallClamps; }
		}
		maxSpeed = std::max(maxSpeed, std::sqrt(dot3(soa.velocities[i], soa.velocities[i])));
	}
	diag_.maxSpeed = maxSpeed;

	// 4. Dry-adiabatic pressure work for the height change.
	if (params.enableAdiabatic) {
		for (size_t i = 0; i < n; ++i) {
			const double p0 = CT::environmentPressure(thermo, environment, zBefore[i]);
			const double p1 = CT::environmentPressure(thermo, environment, soa.positions[i].z);
			soa.temperatures[i] = CT::adiabaticTemperature(thermo, soa.temperatures[i], p0, p1);
		}
	}

	// 5. Sources and conservative pairwise mixing (each unordered pair once, fixed order).
	CloudOps::applySources(soa, sources, time, dt, ledger);
	if (params.enableMixing && params.mixDiffusivity > 0.0) {
		for (size_t i = 0; i < n; ++i) {
			for (int jj : neighbors_[i]) {
				const size_t j = static_cast<size_t>(jj);
				if (j <= i) continue;
				const Vector3dd d = soa.positions[i] - soa.positions[j];
				// Cleary-Monaghan SPH Laplacian: dphi_i/dt = sum_j 2 kappa V_j |W'|/r (phi_j - phi_i). The pair
				// update moves i by a*wi (wi ~ 1/2 for equal masses), hence a = 4 kappa dt V |W'| / r. Being
				// per-volume, the diffusivity does not depend on the particle spacing.
				const double r = std::sqrt(dot3(d, d));
				if (r < 1.0e-12) continue;
				const double volume = 2.0 / (number_[i] + number_[j]);
				const double a = std::min(0.5, 4.0 * params.mixDiffusivity * dt * volume * std::abs(kernelGradMag(r, h)) / r);
				CloudOps::mixPairRelative(soa, i, j, a, thermo, environment);
			}
		}
	}

	// 6. Saturation adjustment (condensation / evaporation with latent heat).
	if (params.enableCondensation) {
		diag_.adjustFailures += CloudOps::saturationAdjustAll(soa, thermo, environment);
	}
	return true;
}

	}
}
