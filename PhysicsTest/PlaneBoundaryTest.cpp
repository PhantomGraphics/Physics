#include "pch.h"

#include "../Physics/PlaneBoundary.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace
{
constexpr float kTol = 1.0e-5f;
}

TEST(PlaneBoundaryTest, SignedDistanceIsPositiveOnNormalSide)
{
  PlaneBoundary plane(Vector3df(0.0f, 1.0f, 0.0f), 0.0f);

  EXPECT_NEAR(plane.getSignedDistance(Vector3df(0.0f, 2.0f, 0.0f)), 2.0f, kTol);
  EXPECT_NEAR(plane.getSignedDistance(Vector3df(0.0f, 0.0f, 0.0f)), 0.0f, kTol);
  EXPECT_NEAR(plane.getSignedDistance(Vector3df(0.0f, -1.5f, 0.0f)), -1.5f, kTol);
}

TEST(PlaneBoundaryTest, BoundaryForceIsZeroOnValidSide)
{
  PlaneBoundary plane(Vector3df(0.0f, 1.0f, 0.0f), 0.0f);

  const auto force = plane.getBoundaryForce(Vector3df(0.0f, 1.0f, 0.0f), 0.01f);

  EXPECT_NEAR(force.x, 0.0f, kTol);
  EXPECT_NEAR(force.y, 0.0f, kTol);
  EXPECT_NEAR(force.z, 0.0f, kTol);
}

TEST(PlaneBoundaryTest, BoundaryForcePushesBackAlongNormalWhenPenetrating)
{
  PlaneBoundary plane(Vector3df(0.0f, 1.0f, 0.0f), 0.0f);

  const auto force = plane.getBoundaryForce(Vector3df(0.0f, -0.1f, 0.0f), 0.01f);

  EXPECT_GT(force.y, 0.0f);
  EXPECT_NEAR(force.x, 0.0f, kTol);
  EXPECT_NEAR(force.z, 0.0f, kTol);
}

TEST(PlaneBoundaryTest, ClampPositionProjectsPenetratingPointOntoPlane)
{
  PlaneBoundary plane(Vector3df(0.0f, 1.0f, 0.0f), 0.0f);

  const auto clamped = plane.clampPosition(Vector3df(1.0f, -0.5f, 2.0f));
  EXPECT_NEAR(clamped.y, 0.0f, kTol);
  EXPECT_NEAR(clamped.x, 1.0f, kTol);
  EXPECT_NEAR(clamped.z, 2.0f, kTol);

  const auto unchanged = plane.clampPosition(Vector3df(1.0f, 0.5f, 2.0f));
  EXPECT_NEAR(unchanged.y, 0.5f, kTol);
}

TEST(PlaneBoundaryTest, MakeBoxPlaneBoundariesPushesOutsidePointInside)
{
  const auto planes = makeBoxPlaneBoundaries(Box3df(Vector3df(-1.0f, -1.0f, -1.0f), Vector3df(1.0f, 1.0f, 1.0f)));
  ASSERT_EQ(planes.size(), 6U);

  Vector3df insideForce(0.0f, 0.0f, 0.0f);
  Vector3df outsideXForce(0.0f, 0.0f, 0.0f);
  for (const auto& plane : planes)
  {
    insideForce += plane.getBoundaryForce(Vector3df(0.0f, 0.0f, 0.0f), 0.01f);
    outsideXForce += plane.getBoundaryForce(Vector3df(1.2f, 0.0f, 0.0f), 0.01f);
  }

  EXPECT_NEAR(insideForce.x, 0.0f, kTol);
  EXPECT_NEAR(insideForce.y, 0.0f, kTol);
  EXPECT_NEAR(insideForce.z, 0.0f, kTol);
  EXPECT_LT(outsideXForce.x, 0.0f);
}
