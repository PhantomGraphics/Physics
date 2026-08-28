#include "pch.h"

#include "SPHSurfaceParticle.h"

#include "WPCA.h"

#include "CGLib/Numerics/Numerics/SVD3d.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace {
	constexpr auto kr = 4.0;
	constexpr auto ks = 1.0;//1400.0;
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


	Matrix3dd scaleMatrix = ::identitiyMatrix3d<double>();
	if (neighbors.size() < 25) {
		scaleMatrix *= 0.5;
	}
	else {
		auto evs = result.eigenValues;
		evs[1] = std::max(evs[1], evs[0] / kr);
		evs[2] = std::max(evs[2], evs[0] / kr);
		evs *= ks;

		scaleMatrix = Matrix3dd
		(
			evs[0], 0.0, 0.0,
			0.0, evs[1], 0.0,
			0.0, 0.0, evs[2]
		);
	}
	this->matrix = rotation * glm::inverse(scaleMatrix) * glm::transpose(rotation) * (1.0 / searchRadius);
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