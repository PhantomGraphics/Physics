#include "pch.h"

#include "SPHKernel.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace {
	static const float PIf = 3.14159265358979323846f;
}

SPHKernel::SPHKernel(const float effectLength) :
	effectLength(effectLength)
{
	this->poly6KernelConstant = 315.0f / (64.0f * PIf * std::pow(effectLength, 9.0f));
	this->spikyKernelGradConstant = 45.0f / (::PIf * std::pow(effectLength, 6.0f));
	this->effectLengthSquared = effectLength * effectLength;
}

float SPHKernel::getPoly6Kernel(const float distance)
{
	if (distance > effectLength) {
		return 0.0f;
	}
	return this->poly6KernelConstant * std::pow(effectLength * effectLength - distance * distance, 3.0f);
}

float SPHKernel::getPoly6Kernel2(const float distanceSquared)
{
	if (distanceSquared > effectLengthSquared) {
		return 0.0f;
	}
	const auto a = effectLengthSquared - distanceSquared;
	return this->poly6KernelConstant * a * a * a;
}

/*
float SPHKernel::getPoly6Kernel(const float distance, const float effectLength)
{
	if (distance > effectLength) {
		return 0.0f;
	}
	const auto poly6Constant = 315.0f / (64.0f * PI * pow(effectLength, 9));
	return poly6Constant * pow(effectLength * effectLength - distance * distance, 3);
}
*/

Vector3df SPHKernel::getPoly6KernelGradient(const Vector3df& distanceVector)
{
	const auto distance = glm::length(distanceVector);
	if (distance > effectLength) {
		return Vector3df(0, 0, 0);
	}
	const auto poly6ConstantGradient = 945.0f / (32.0f * PIf * std::pow(effectLength, 9.0f));
	const auto factor = poly6ConstantGradient * std::pow(effectLength * effectLength - distance * distance, 2.0f);
	return distanceVector * factor;
}

float SPHKernel::getPoly6KernelGradientCoe(const float distance)
{
	const auto poly6ConstantGradient = 945.0f / (32.0f * PIf * std::pow(effectLength, 9.0f));
	return poly6ConstantGradient * std::pow(effectLength * effectLength - distance * distance, 2.0f);
}

float SPHKernel::getPoly6KernelLaplacian(const float distance)
{
	if (distance > effectLength) {
		return 0.0f;
	}
	const auto poly6ConstantLaplacian = 945.0f / (32.0f * PIf * std::pow(effectLength, 9.0f));
	return poly6ConstantLaplacian * (effectLength * effectLength - distance * distance)
		* (42.0f * distance * distance - 18.0f * effectLength * effectLength);
}

Vector3df SPHKernel::getSpikyKernelGradient(const Vector3df& distanceVector) const
{
	const auto distance = glm::length(distanceVector);
	if (distance > effectLength) {
		return Vector3df(0, 0, 0);
	}
	// Two particles at exactly the same position make the expression below
	// 0/0 == NaN, and that NaN then spreads out of the force accumulator into
	// velocity/position and on into whatever reads them (PLY export, surface
	// reconstruction, the scenario assertions).
	//
	// Only the *direction* is undefined there, not the magnitude: since
	// |distanceVector| == distance, the quotient is a unit vector and the
	// result tends to spikyKernelGradConstant * effectLength^2 as r -> 0. A
	// zero vector is the right value to return anyway -- it is the only one
	// that does not break the pair's symmetry by picking an arbitrary
	// direction, and the pair contributes nothing to either particle.
	//
	// Coincident particles are not hypothetical. The domain-wall penalty
	// (PlaneBoundary/SphereBoundary::getBoundaryForce()) restores a
	// penetrating particle exactly onto the wall, so particles pressed into a
	// box corner -- where three planes act at once -- land on bit-identical
	// coordinates and stay there. Measured: a resting pool of 2744 particles
	// produced NaN in the two particles that had collapsed onto the same
	// corner point.
	if (distance <= 0.0f) {
		return Vector3df(0, 0, 0);
	}
	return distanceVector * this->spikyKernelGradConstant * std::pow(effectLength - distance, 2.0f) / distance;
}

float SPHKernel::getSpikyKernelGradientWeight(const float distance)
{
	if (distance > effectLength) {
		return 0.0;
	}
	// Same coincident-particle guard as getSpikyKernelGradient() above, where
	// the reasoning is spelled out. This overload returns the weight a caller
	// is expected to multiply by the (zero) separation vector, so without the
	// guard the division yields +inf and the caller's 0 * inf yields NaN.
	if (distance <= 0.0f) {
		return 0.0f;
	}
	const auto constant = 45.0f / (PIf * std::pow(effectLength, 6.0f));
	return constant * std::pow(effectLength - distance, 2.0f) / distance;
}

/*
Vector3df SPHKernel::getSpikyKernelGradient(const Vector3df& distanceVector)
{
	const auto distance = glm::length(distanceVector);
	if (distance > effectLength) {
		return Vector3df(0, 0, 0);
	}
	const auto constant = 45.0f / (PIf * pow(effectLength, 6));
	return distanceVector * constant * pow(effectLength - distance, 2) / distance;
}
*/

float SPHKernel::getViscosityKernelLaplacian(const float distance)
{
	if (distance > effectLength) {
		return 0.0f;
	}
	const auto constant = 45.0f / (PIf * std::pow(effectLength, 6.0f));
	return (effectLength - distance) * constant;
}

float SPHKernel::getCubicSpline(const float distance) const
{
	const auto q = distance * 2 / (effectLength);
	// 3D normalization: integral of W over the support must be 1, which for
	// this q = 2r/h parameterization works out to coe * pi * h^3 / 12 == 1,
	// i.e. coe = 12 / (pi * h^3) -- giving the standard M4 spline's
	// W(0) = 8 / (pi * h^3).
	//
	// This used to read `h^3 * 3 / pi`, i.e. it was off by a factor of h^6 / 4.
	// Since getCubicSplineGradient() below *is* correctly normalized, density
	// (built from W) and the DFSPH alpha/pressure corrections (built from gradW)
	// disagreed by that factor -- and because it depends on h, every DFSPH
	// quantity derived from both silently changed meaning with the scene's
	// scale. That is what made rigid-fluid coupling diverge at one scale and do
	// nothing at another (internal design notes 1.6).
	const auto coe = 12.0f / (PIf * effectLength * effectLength * effectLength);

	if (q < 1) {
		return coe * (2.0f / 3.0f - q * q + 0.5f * q * q * q);
	}
	else if (q < 2) {
		return coe * (std::pow(2.0f - q, 3.0f) / 6.0f);
	}
	else {
		return 0;
	}
}

float SPHKernel::getCubicSpline(const Vector3df& v) const
{
	// The scalar overload already normalizes by effectLength; this one used to
	// feed it a *ratio* (r/h) as if it were a distance and then divide by h^3
	// again, which is neither this kernel nor a scaled copy of it.
	return getCubicSpline(glm::length(v));
}

Vector3df SPHKernel::getCubicSplineGradient(const Vector3df& distanceVector)
{
	const auto length = glm::length(distanceVector);
	const auto l = 48.0f / (PIf * effectLength * effectLength * effectLength);
	const auto q = length / effectLength;
	const auto grad = distanceVector * (1.0f / (length * effectLength));
	if (q < 0.01f) {
		return Vector3df(0, 0, 0);
	}
	else if (q < 0.5f) {
		return l * q * (3.0f * q - 2.0f) * grad;
	}
	else if (q < 1.0f) {
		const auto f = 1.0f - q;
		return l * (-f * f) * grad;
	}
	else {
		return Vector3df(0, 0, 0);
	}
}