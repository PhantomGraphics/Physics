#include "pch.h"
#include "PlaneBoundary.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

PlaneBoundary::PlaneBoundary(const Vector3df& normal, const float offset) :
	normal(normal),
	offset(offset)
{
}

float PlaneBoundary::getSignedDistance(const Vector3df& pos) const
{
	return glm::dot(normal, pos) - offset;
}

Vector3df PlaneBoundary::getBoundaryForce(const Vector3df& pos, const float timeStep) const
{
	const float d = getSignedDistance(pos);
	if (d >= 0.f) {
		return Vector3df(0.f, 0.f, 0.f);
	}
	return normal * (-d / (timeStep * timeStep));
}

Vector3df PlaneBoundary::getBoundaryForce(const Vector3df& pos, const Vector3df& velocity,
                                           const float timeStep, const float dampingRatio) const
{
	const float d = getSignedDistance(pos);
	if (d >= 0.f) {
		return Vector3df(0.f, 0.f, 0.f);
	}
	return normal * boundaryPenaltyAcceleration(d, glm::dot(normal, velocity), timeStep, dampingRatio);
}

Vector3df PlaneBoundary::clampPosition(const Vector3df& pos) const
{
	const float d = getSignedDistance(pos);
	if (d >= 0.f) {
		return pos;
	}
	return pos - d * normal;
}

std::vector<PlaneBoundary> Phantom::Physics::makeBoxPlaneBoundaries(const Box3df& box)
{
	const auto& bmin = box.getMin();
	const auto& bmax = box.getMax();

	std::vector<PlaneBoundary> planes;
	planes.reserve(6);
	planes.emplace_back(Vector3df(1.f, 0.f, 0.f), bmin.x);
	planes.emplace_back(Vector3df(-1.f, 0.f, 0.f), -bmax.x);
	planes.emplace_back(Vector3df(0.f, 1.f, 0.f), bmin.y);
	planes.emplace_back(Vector3df(0.f, -1.f, 0.f), -bmax.y);
	planes.emplace_back(Vector3df(0.f, 0.f, 1.f), bmin.z);
	planes.emplace_back(Vector3df(0.f, 0.f, -1.f), -bmax.z);
	return planes;
}
