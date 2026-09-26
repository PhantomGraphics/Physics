#include "pch.h"

#include "SPHSurfaceParticle.h"

#include "WPCA.h"

#include "CGLib/Numerics/Numerics/SVD3d.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace {
	// Max ratio between the largest and the other principal stretches.
	constexpr auto kr = 4.0;
	// Isotropic stretch used for particles with too few neighbours (Yu & Turk's k_n).
	constexpr auto kn = 0.5;
}

SPHSurfaceParticle::SPHSurfaceParticle(const Vector3df& p, const float radius) :
	position(p),
	matrix(Math::identitiyMatrix3d<double>()),
	density(0.0f),
	radius(radius)
{}

void SPHSurfaceParticle::correctedPosition(const float lamda, const Vector3df& wm)
{
	this->position = (1.0f - lamda) * position + lamda * wm;
}

void SPHSurfaceParticle::calculateAnisotoropicMatrix(const std::vector<Vector3df>& neighbors, const float searchRadius)
{
	//const Matrix3dd scaleMatrix;
	WPCA wpca;
	wpca.setup(position, neighbors, searchRadius);
	this->matrix = wpca.calculateCovarianceMatrix(position, neighbors, searchRadius);

	Phantom::Numerics::SVD3d svd;
	auto result = svd.calculateJacobi(matrix);
	/*
	if (!result.isOk) {
		p->matrix = ::identitiyMatrix();
		return;
	}
	*/

	const auto rotation = result.eigenVectors;


	// Sigma~ in Yu & Turk (2013): the per-axis stretch of the kernel. G maps a
	// world-space offset v to the world-space distance |Gv| fed to the same
	// SPHKernel the isotropic path uses (support = searchRadius), so G is
	// dimensionless and a uniform neighbourhood must give G = I.
	//
	// The covariance eigenvalues are squared lengths (~ spacing^2), so they are
	// normalized to unit product (volume-preserving ellipsoid) instead of being
	// multiplied by a scene-scale-dependent ks. The previous code used ks = 1
	// and also folded 1/searchRadius into G, which made |Gv| ~1e4-1e5 times too
	// large for real particle spacings: the kernel collapsed to about one voxel
	// and det(G) reached ~1e14.
	Matrix3dd scaleMatrix = ::identitiyMatrix3d<double>() * kn;
	auto evs = result.eigenValues;
	if (neighbors.size() >= 25 && evs[0] > 0.0) {
		evs[1] = std::max(evs[1], evs[0] / kr);
		evs[2] = std::max(evs[2], evs[0] / kr);
		evs /= std::cbrt(evs[0] * evs[1] * evs[2]);

		scaleMatrix = Matrix3dd
		(
			evs[0], 0.0, 0.0,
			0.0, evs[1], 0.0,
			0.0, 0.0, evs[2]
		);
	}
	this->matrix = rotation * glm::inverse(scaleMatrix) * glm::transpose(rotation);
}

void SPHSurfaceParticle::calculateDensity(const std::vector<Vector3df>& neighbors, const float searchRadius, const SPHKernel& kernel)
{
	const auto mass = getMass();
	for (auto n : neighbors) {
		const auto distanceSquared = Math::getDistanceSquared(n, this->position);
		this->density += mass * kernel.getCubicSpline(::sqrt(distanceSquared));
	}
}

float SPHSurfaceParticle::getMass() const
{
	const auto diameter = radius * 2.0f;
	return diameter * diameter * diameter;
}