#include "pch.h"

#include "DMBoundary.h"

#include "SPHKernel.h"
#include "CGLib/Math/Plane3d.h"

#include "CGLib/Space/Space/SignedDistanceCalculator.h"

using namespace Phantom::Math;
using namespace Phantom::Space;
using namespace Phantom::Physics;

float DMBoundary::calculateWeight(const Vector3df& position, const float r, const float restDensity, const SPHKernel& kernel)
{
	const Plane3df bottom(boundary.getMin(), Vector3df(0,1,0));
	SignedDistanceCalculator<float> c;
	const auto sd = c.calculate(position, bottom);
	return calculateWeight(sd, r, restDensity, kernel);
}

float DMBoundary::calculateWeight(const float signedDistance, const float r, const float restDensity, const SPHKernel& kernel)
{
	return calculateDensity(signedDistance, r, restDensity ) * kernel.getCubicSpline(::fabs(signedDistance));
}

float DMBoundary::calculateDensity(const float x, const float r, const float restDensity)
{
	if (x >= r) {
		return 0.0f;
	}
	return restDensity * (1.0f - x / r);
}