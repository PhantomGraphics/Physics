#include "pch.h"
#include "SphereBoundary.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

SphereBoundary::SphereBoundary(const Vector3df& center, const float radius, const float maxPenetration) :
	center_(center),
	radius_(radius),
	maxPenetration_(maxPenetration)
{
}

float SphereBoundary::getSignedDistance(const Vector3df& pos) const
{
	return radius_ - glm::length(pos - center_);
}

bool SphereBoundary::isActiveAt(const Vector3df& pos) const
{
	return getSignedDistance(pos) >= -maxPenetration_;
}

Vector3df SphereBoundary::getBoundaryForce(const Vector3df& pos, const float timeStep) const
{
	const float d = getSignedDistance(pos);
	if (d >= 0.f || d < -maxPenetration_) {
		return Vector3df(0.f, 0.f, 0.f);
	}
	const Vector3df toCenter = center_ - pos;
	const Vector3df normal = toCenter / glm::length(toCenter);
	return normal * (-d / (timeStep * timeStep));
}

Vector3df SphereBoundary::getBoundaryForce(const Vector3df& pos, const Vector3df& velocity,
                                            const float timeStep, const float dampingRatio) const
{
	const float d = getSignedDistance(pos);
	if (d >= 0.f || d < -maxPenetration_) {
		return Vector3df(0.f, 0.f, 0.f);
	}
	const Vector3df toCenter = center_ - pos;
	const Vector3df normal = toCenter / glm::length(toCenter);
	return normal * boundaryPenaltyAcceleration(d, glm::dot(normal, velocity), timeStep, dampingRatio);
}

Vector3df SphereBoundary::clampPosition(const Vector3df& pos) const
{
	const float d = getSignedDistance(pos);
	if (d >= 0.f) {
		return pos;
	}
	const Vector3df toCenter = center_ - pos;
	const Vector3df normal = toCenter / glm::length(toCenter);
	return pos - d * normal;
}
