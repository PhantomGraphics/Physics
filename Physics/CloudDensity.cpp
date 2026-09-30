#include "CloudDensity.h"

#include <algorithm>
#include <cmath>

namespace Phantom {
	namespace Physics {
		namespace CloudDensity {

using Volume::ScalarGrid3D;
using Volume::ScalarGridDesc;

namespace {
constexpr double kPi = 3.14159265358979323846;

double cubicSpline(double r, double h)
{
	const double q = r / h;
	if (q >= 1.0) return 0.0;
	const double sigma = 8.0 / (kPi * h * h * h);
	if (q <= 0.5) return sigma * (1.0 - 6.0 * q * q + 6.0 * q * q * q);
	const double o = 1.0 - q;
	return sigma * 2.0 * o * o * o;
}
}

ScalarGridDesc makeGridDesc(const Math::Vector3dd& lo, const Math::Vector3dd& hi, uint32_t resolution)
{
	ScalarGridDesc d;
	const Math::Vector3dd ext = hi - lo;
	const double longest = std::max({ ext.x, ext.y, ext.z, 1.0e-9 });
	const double cell = longest / std::max(1u, resolution);
	d.cellSize = static_cast<float>(cell);
	d.nx = std::max(1u, static_cast<uint32_t>(std::ceil(ext.x / cell - 1.0e-9)));
	d.ny = std::max(1u, static_cast<uint32_t>(std::ceil(ext.y / cell - 1.0e-9)));
	d.nz = std::max(1u, static_cast<uint32_t>(std::ceil(ext.z / cell - 1.0e-9)));
	d.origin = Math::Vector3df(static_cast<float>(lo.x), static_cast<float>(lo.y), static_cast<float>(lo.z));
	return d;
}

CloudDensityStats reconstruct(const CloudParticleSoA& soa, const ScalarGridDesc& desc, double supportRadius,
                              ScalarGrid3D& out)
{
	out = ScalarGrid3D(desc, 0.0f, 0.0f);
	CloudDensityStats st;
	const double cell = desc.cellSize;
	const double h = std::max(supportRadius, 2.0 * cell);
	const int reach = static_cast<int>(std::ceil(h / cell));

	std::vector<double> acc(desc.cellCount(), 0.0);
	for (size_t p = 0; p < soa.size(); ++p) {
		const double mass = soa.mDry[p] * soa.qc[p];
		if (!(mass > 0.0) || !std::isfinite(mass)) continue;
		++st.cloudyParticles;
		st.particleMass += mass;
		const Math::Vector3dd& x = soa.positions[p];
		// Cell centres sit at origin + (i + 0.5) * cell.
		const int ci = static_cast<int>(std::floor((x.x - desc.origin.x) / cell));
		const int cj = static_cast<int>(std::floor((x.y - desc.origin.y) / cell));
		const int ck = static_cast<int>(std::floor((x.z - desc.origin.z) / cell));
		for (int k = std::max(0, ck - reach); k <= std::min(static_cast<int>(desc.nz) - 1, ck + reach); ++k) {
			const double dz = desc.origin.z + (k + 0.5) * cell - x.z;
			for (int j = std::max(0, cj - reach); j <= std::min(static_cast<int>(desc.ny) - 1, cj + reach); ++j) {
				const double dy = desc.origin.y + (j + 0.5) * cell - x.y;
				for (int i = std::max(0, ci - reach); i <= std::min(static_cast<int>(desc.nx) - 1, ci + reach); ++i) {
					const double dx = desc.origin.x + (i + 0.5) * cell - x.x;
					const double w = cubicSpline(std::sqrt(dx * dx + dy * dy + dz * dz), h);
					if (w > 0.0) acc[out.index(i, j, k)] += mass * w;
				}
			}
		}
	}
	const double cellVolume = cell * cell * cell;
	for (size_t c = 0; c < acc.size(); ++c) {
		out.data()[c] = static_cast<float>(acc[c]);
		st.gridMass += acc[c] * cellVolume;
	}
	st.relativeError = st.particleMass > 0.0 ? std::abs(st.gridMass - st.particleMass) / st.particleMass : 0.0;
	return st;
}

		}
	}
}
