#include "pch.h"
#include "PlateBoundary.h"

#include <algorithm>
#include <cmath>

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace {

// Outward face normal and penetration depth for a position known to be inside
// the OBB (all q components < 0). The nearest face is the axis whose q is
// largest (least negative); depth is -q on that axis.
void nearestFace(const Vector3df& local, const Vector3df& q,
                 const Vector3df& u, const Vector3df& v, const Vector3df& n,
                 Vector3df& outNormal, float& outDepth)
{
	int axis = 0;
	float best = q.x;
	if (q.y > best) { best = q.y; axis = 1; }
	if (q.z > best) { best = q.z; axis = 2; }

	const Vector3df& axisVec = (axis == 0) ? u : (axis == 1) ? v : n;
	const float sign = (local[axis] >= 0.f) ? 1.f : -1.f;
	outNormal = axisVec * sign;
	outDepth = -best;   // best < 0, so depth > 0
}

}

PlateBoundary::PlateBoundary(const Vector3df& center, const Vector3df& normal, const Vector3df& uAxis,
                             const float halfU, const float halfV, const float halfThickness) :
	center_(center),
	halfU_(halfU),
	halfV_(halfV),
	halfThickness_(halfThickness)
{
	n_ = glm::normalize(normal);
	// Remove the normal component from u, then normalize; v completes the frame.
	Vector3df u = uAxis - n_ * glm::dot(n_, uAxis);
	u_ = glm::normalize(u);
	v_ = glm::cross(n_, u_);

	// World AABB of the OBB: half-extent along each world axis is the sum of
	// the three local half-sizes projected onto it.
	const Vector3df ext(
		halfU_ * std::fabs(u_.x) + halfV_ * std::fabs(v_.x) + halfThickness_ * std::fabs(n_.x),
		halfU_ * std::fabs(u_.y) + halfV_ * std::fabs(v_.y) + halfThickness_ * std::fabs(n_.y),
		halfU_ * std::fabs(u_.z) + halfV_ * std::fabs(v_.z) + halfThickness_ * std::fabs(n_.z));
	aabbMin_ = center_ - ext;
	aabbMax_ = center_ + ext;
}

Vector3df PlateBoundary::toLocal(const Vector3df& pos) const
{
	const Vector3df d = pos - center_;
	return Vector3df(glm::dot(d, u_), glm::dot(d, v_), glm::dot(d, n_));
}

float PlateBoundary::getSignedDistance(const Vector3df& pos) const
{
	const Vector3df local = toLocal(pos);
	const Vector3df q(std::fabs(local.x) - halfU_,
	                  std::fabs(local.y) - halfV_,
	                  std::fabs(local.z) - halfThickness_);
	const Vector3df outside(std::max(q.x, 0.f), std::max(q.y, 0.f), std::max(q.z, 0.f));
	const float inside = std::min(std::max(q.x, std::max(q.y, q.z)), 0.f);
	return glm::length(outside) + inside;
}

bool PlateBoundary::isActiveAt(const Vector3df& pos, const float effectLength) const
{
	const float e = (effectLength > 0.f) ? effectLength : 0.f;
	return pos.x >= aabbMin_.x - e && pos.x <= aabbMax_.x + e
	    && pos.y >= aabbMin_.y - e && pos.y <= aabbMax_.y + e
	    && pos.z >= aabbMin_.z - e && pos.z <= aabbMax_.z + e;
}

Vector3df PlateBoundary::getBoundaryForce(const Vector3df& pos, const float timeStep) const
{
	const Vector3df local = toLocal(pos);
	const Vector3df q(std::fabs(local.x) - halfU_,
	                  std::fabs(local.y) - halfV_,
	                  std::fabs(local.z) - halfThickness_);
	if (q.x >= 0.f || q.y >= 0.f || q.z >= 0.f) {
		return Vector3df(0.f, 0.f, 0.f);   // outside the OBB
	}
	Vector3df normal;
	float depth;
	nearestFace(local, q, u_, v_, n_, normal, depth);
	return normal * (depth / (timeStep * timeStep));
}

Vector3df PlateBoundary::getBoundaryForce(const Vector3df& pos, const Vector3df& velocity,
                                          const float timeStep, const float dampingRatio) const
{
	const Vector3df local = toLocal(pos);
	const Vector3df q(std::fabs(local.x) - halfU_,
	                  std::fabs(local.y) - halfV_,
	                  std::fabs(local.z) - halfThickness_);
	if (q.x >= 0.f || q.y >= 0.f || q.z >= 0.f) {
		return Vector3df(0.f, 0.f, 0.f);
	}
	Vector3df normal;
	float depth;
	nearestFace(local, q, u_, v_, n_, normal, depth);
	return normal * boundaryPenaltyAcceleration(-depth, glm::dot(normal, velocity), timeStep, dampingRatio);
}

Vector3df PlateBoundary::clampPosition(const Vector3df& pos) const
{
	const Vector3df local = toLocal(pos);
	const Vector3df q(std::fabs(local.x) - halfU_,
	                  std::fabs(local.y) - halfV_,
	                  std::fabs(local.z) - halfThickness_);
	if (q.x >= 0.f || q.y >= 0.f || q.z >= 0.f) {
		return pos;
	}
	Vector3df normal;
	float depth;
	nearestFace(local, q, u_, v_, n_, normal, depth);
	return pos + normal * depth;
}

float PlateBoundary::getFaceDistance(const Vector3df& pos) const
{
	return std::fabs(glm::dot(pos - center_, n_)) - halfThickness_;
}

float PlateBoundary::getRimFraction(const Vector3df& pos, const float effectLength) const
{
	if (!(effectLength > 0.f)) return 0.f;
	const Vector3df local = toLocal(pos);
	const float du = halfU_ - std::fabs(local.x);
	const float dv = halfV_ - std::fabs(local.y);
	const float rim = std::min(du, dv) / effectLength + 0.5f;
	if (rim <= 0.f) return 0.f;
	if (rim >= 1.f) return 1.f;
	return rim;
}
