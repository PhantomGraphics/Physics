#include "pch.h"

#include "SPHSurfaceParticle.h"

#include "WPCA.h"
#include "AnisotropicKernel.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

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
	WPCA wpca;
	wpca.setup(position, neighbors, searchRadius);
	const Matrix3dd covariance = wpca.calculateCovarianceMatrix(position, neighbors, searchRadius);

	// G maps a world-space offset v to the world-space distance |Gv| fed to the
	// same SPHKernel the isotropic path uses (support = searchRadius), so G is
	// dimensionless and a uniform neighbourhood must give G = I. The stretch
	// itself (Yu & Turk 2013's Sigma~, clamped and normalized to unit product)
	// is shared with the SSFR ellipsoid splats -- see AnisotropicKernel.h.
	//
	// History: the code before the shared helper used ks = 1 and also folded
	// 1/searchRadius into G, which made |Gv| ~1e4-1e5 times too large for real
	// particle spacings: the kernel collapsed to about one voxel and det(G)
	// reached ~1e14.
	AnisotropyParams params;
	params.searchRadius = searchRadius;
	const auto frame = anisotropyFromCovariance(covariance, neighbors.size(), params);
	this->matrix = toKernelMatrix(frame);
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
