#include "pch.h"
#include "CylinderBoundary.h"

#include <algorithm>
#include <cmath>

using namespace Phantom::Math;
using namespace Phantom::Physics;

CylinderBoundary::CylinderBoundary(const Vector3df& center, const Vector3df& axis,
                                   const float radius, const float halfHeight,
                                   const float maxPenetration) :
	center_(center), axis_(glm::normalize(axis)), radius_(radius),
	halfHeight_(halfHeight), maxPenetration_(maxPenetration)
{
}

float CylinderBoundary::getSignedDistance(const Vector3df& pos) const
{
	const Vector3df relative = pos - center_;
	const float axial = glm::dot(relative, axis_);
	const float radial = glm::length(relative - axial * axis_);
	const float radialOutside = radial - radius_;
	const float axialOutside = std::fabs(axial) - halfHeight_;
	const float outside = std::sqrt(std::max(radialOutside, 0.f) * std::max(radialOutside, 0.f)
	                              + std::max(axialOutside, 0.f) * std::max(axialOutside, 0.f));
	const float inside = std::min(std::max(radialOutside, axialOutside), 0.f);
	return -(outside + inside);
}

bool CylinderBoundary::isActiveAt(const Vector3df& pos) const
{
	return getSignedDistance(pos) >= -maxPenetration_;
}

Vector3df CylinderBoundary::closestPoint(const Vector3df& pos) const
{
	const Vector3df relative = pos - center_;
	const float axial = glm::dot(relative, axis_);
	const Vector3df radialVector = relative - axial * axis_;
	const float radial = glm::length(radialVector);
	const float clampedAxial = std::max(-halfHeight_, std::min(axial, halfHeight_));
	Vector3df clampedRadial = radialVector;
	if (radial > radius_)
		clampedRadial *= radius_ / radial;
	return center_ + clampedAxial * axis_ + clampedRadial;
}

Vector3df CylinderBoundary::getBoundaryForce(const Vector3df& pos, const float timeStep) const
{
	const float d = getSignedDistance(pos);
	if (d >= 0.f || d < -maxPenetration_) return Vector3df(0.f);
	const Vector3df inward = closestPoint(pos) - pos;
	return inward * ((-d / (timeStep * timeStep)) / glm::length(inward));
}

Vector3df CylinderBoundary::getBoundaryForce(const Vector3df& pos, const Vector3df& velocity,
                                             const float timeStep, const float dampingRatio) const
{
	const float d = getSignedDistance(pos);
	if (d >= 0.f || d < -maxPenetration_) return Vector3df(0.f);
	const Vector3df inward = glm::normalize(closestPoint(pos) - pos);
	return inward * boundaryPenaltyAcceleration(d, glm::dot(inward, velocity), timeStep, dampingRatio);
}

Vector3df CylinderBoundary::clampPosition(const Vector3df& pos) const
{
	return getSignedDistance(pos) >= 0.f ? pos : closestPoint(pos);
}
