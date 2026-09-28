#include "pch.h"

#include "AnisotropicKernel.h"

#include "CGLib/Numerics/Numerics/SVD3d.h"
#include "CGLib/Space/Space/NeighborList.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

AnisotropyFrame Phantom::Physics::anisotropyFromCovariance(const Matrix3dd& covariance, const std::size_t neighborCount, const AnisotropyParams& params)
{
	Phantom::Numerics::SVD3d svd;
	const auto result = svd.calculateJacobi(covariance);

	AnisotropyFrame frame;
	frame.rotation = result.eigenVectors;

	// Sigma~ in Yu & Turk (2013): the per-axis stretch of the kernel. The
	// covariance eigenvalues are squared lengths (~ spacing^2), so they are
	// normalized to unit product (volume-preserving ellipsoid) instead of being
	// multiplied by a scene-scale-dependent k_s; a uniform neighbourhood then
	// gives sigma = 1 on every axis.
	auto evs = result.eigenValues;
	if (neighborCount >= static_cast<std::size_t>(params.minNeighbors) && evs[0] > 0.0) {
		const double kr = params.maxRatio;
		evs[1] = std::max(evs[1], evs[0] / kr);
		evs[2] = std::max(evs[2], evs[0] / kr);
		evs /= std::cbrt(evs[0] * evs[1] * evs[2]);
		frame.sigma = evs;
	}
	else {
		const double kn = params.isolatedScale;
		frame.sigma = Vector3dd(kn, kn, kn);
	}
	return frame;
}

Matrix3dd Phantom::Physics::toKernelMatrix(const AnisotropyFrame& frame)
{
	const Matrix3dd invScale(
		1.0 / frame.sigma[0], 0.0, 0.0,
		0.0, 1.0 / frame.sigma[1], 0.0,
		0.0, 0.0, 1.0 / frame.sigma[2]);
	return frame.rotation * invScale * glm::transpose(frame.rotation);
}

Matrix3df Phantom::Physics::toEllipsoidAxes(const AnisotropyFrame& frame)
{
	Matrix3df axes;
	for (int c = 0; c < 3; ++c) {
		axes[c] = Vector3df(frame.rotation[c] * frame.sigma[c]);
	}
	return axes;
}

float Phantom::Physics::anisotropyWeight(const float distance, const float radius)
{
	if (distance > radius) {
		return 0.0f;
	}
	const float q = distance / radius;
	return 1.0f - q * q * q;
}

void Phantom::Physics::computeAnisotropy(const std::vector<Vector3df>& positions, const AnisotropyParams& params, AnisotropyResult& out)
{
	const int n = static_cast<int>(positions.size());
	out.centers.resize(positions.size());
	out.axes.resize(positions.size());
	out.meanStretchRatio = 1.0f;
	out.anisotropicCount = 0;
	if (n == 0) {
		return;
	}

	const float radius = params.searchRadius;
	Phantom::Space::CSRNeighborList neighbors;
	neighbors.build(positions, radius);

	std::vector<double> ratios(positions.size(), 1.0);
	std::vector<unsigned char> trusted(positions.size(), 0);

	// Reads only the input positions; the smoothed centres are written to
	// `out.centers`, never back into `positions`.
#pragma omp parallel for schedule(dynamic, 256)
	for (int i = 0; i < n; ++i) {
		const Vector3df& p = positions[i];
		const auto row = neighbors[i];

		// Weighted mean over the neighbourhood, the particle itself included
		// (weight 1 at distance 0).
		double totalWeight = 1.0;
		Vector3dd mean(p);
		for (const int j : row) {
			const float w = anisotropyWeight(glm::length(positions[j] - p), radius);
			totalWeight += w;
			mean += Vector3dd(positions[j]) * static_cast<double>(w);
		}
		mean /= totalWeight;

		Matrix3dd covariance(0.0);
		{
			const Vector3dd v = mean - Vector3dd(p);
			covariance += glm::outerProduct(v, v);
		}
		for (const int j : row) {
			const double w = anisotropyWeight(glm::length(positions[j] - p), radius);
			const Vector3dd v = mean - Vector3dd(positions[j]);
			covariance += w * glm::outerProduct(v, v);
		}
		covariance /= totalWeight;

		const std::size_t count = row.size() + 1;
		const auto frame = anisotropyFromCovariance(covariance, count, params);
		out.axes[i] = toEllipsoidAxes(frame);

		const double sMax = std::max({ frame.sigma[0], frame.sigma[1], frame.sigma[2] });
		const double sMin = std::min({ frame.sigma[0], frame.sigma[1], frame.sigma[2] });
		ratios[i] = sMin > 0.0 ? sMax / sMin : 1.0;
		trusted[i] = count >= static_cast<std::size_t>(params.minNeighbors) ? 1 : 0;

		const float lambda = params.smoothing;
		out.centers[i] = (1.0f - lambda) * p + lambda * Vector3df(mean);
	}

	double ratioSum = 0.0;
	std::size_t trustedCount = 0;
	for (int i = 0; i < n; ++i) {
		ratioSum += ratios[i];
		trustedCount += trusted[i];
	}
	out.meanStretchRatio = static_cast<float>(ratioSum / n);
	out.anisotropicCount = trustedCount;
}
