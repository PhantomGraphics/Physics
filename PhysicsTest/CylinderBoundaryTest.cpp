#include "pch.h"

#include "../Physics/CylinderBoundary.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

TEST(CylinderBoundaryTest, SignedDistanceIsPositiveInsideAndNegativeOutside)
{
	const CylinderBoundary cylinder(Vector3df(0.f), Vector3df(0.f, 1.f, 0.f), 2.f, 3.f);
	EXPECT_FLOAT_EQ(cylinder.getSignedDistance(Vector3df(0.f)), 2.f);
	EXPECT_FLOAT_EQ(cylinder.getSignedDistance(Vector3df(1.5f, 0.f, 0.f)), 0.5f);
	EXPECT_FLOAT_EQ(cylinder.getSignedDistance(Vector3df(0.f, 2.75f, 0.f)), 0.25f);
	EXPECT_FLOAT_EQ(cylinder.getSignedDistance(Vector3df(2.5f, 0.f, 0.f)), -0.5f);
	EXPECT_FLOAT_EQ(cylinder.getSignedDistance(Vector3df(0.f, 4.f, 0.f)), -1.f);
}

TEST(CylinderBoundaryTest, CornerDistanceIsEuclidean)
{
	const CylinderBoundary cylinder(Vector3df(0.f), Vector3df(0.f, 1.f, 0.f), 2.f, 3.f);
	EXPECT_NEAR(cylinder.getSignedDistance(Vector3df(2.3f, 3.4f, 0.f)), -0.5f, 1e-6f);
}

TEST(CylinderBoundaryTest, ForcePushesThroughSideAndCaps)
{
	const CylinderBoundary cylinder(Vector3df(0.f), Vector3df(0.f, 1.f, 0.f), 2.f, 3.f);
	const Vector3df side = cylinder.getBoundaryForce(Vector3df(2.5f, 0.f, 0.f), 0.5f);
	const Vector3df cap = cylinder.getBoundaryForce(Vector3df(0.f, -3.5f, 0.f), 0.5f);
	EXPECT_NEAR(side.x, -2.f, 1e-6f);
	EXPECT_NEAR(side.y, 0.f, 1e-6f);
	EXPECT_NEAR(cap.y, 2.f, 1e-6f);
}

TEST(CylinderBoundaryTest, ClampProjectsToClosestSurfacePoint)
{
	const CylinderBoundary cylinder(Vector3df(1.f, 2.f, 3.f), Vector3df(0.f, 0.f, 1.f), 2.f, 3.f);
	const Vector3df side = cylinder.clampPosition(Vector3df(4.f, 2.f, 3.f));
	const Vector3df corner = cylinder.clampPosition(Vector3df(4.f, 2.f, 7.f));
	EXPECT_NEAR(side.x, 3.f, 1e-6f);
	EXPECT_NEAR(side.z, 3.f, 1e-6f);
	EXPECT_NEAR(corner.x, 3.f, 1e-6f);
	EXPECT_NEAR(corner.z, 6.f, 1e-6f);
	EXPECT_GE(cylinder.getSignedDistance(side), -1e-6f);
	EXPECT_GE(cylinder.getSignedDistance(corner), -1e-6f);
}

TEST(CylinderBoundaryTest, MaxPenetrationRejectsLostParticles)
{
	const CylinderBoundary cylinder(Vector3df(0.f), Vector3df(0.f, 1.f, 0.f), 1.f, 2.f, 0.1f);
	EXPECT_TRUE(cylinder.isActiveAt(Vector3df(1.05f, 0.f, 0.f)));
	EXPECT_FALSE(cylinder.isActiveAt(Vector3df(1.2f, 0.f, 0.f)));
	EXPECT_EQ(cylinder.getBoundaryForce(Vector3df(1.2f, 0.f, 0.f), 0.01f), Vector3df(0.f));
}
