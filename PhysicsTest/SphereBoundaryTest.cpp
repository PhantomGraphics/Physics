#include "pch.h"

#include "../Physics/SphereBoundary.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace
{
constexpr float kTol = 1.0e-5f;
}

TEST(SphereBoundaryTest, SignedDistanceIsPositiveInsideAndNegativeOutside)
{
  SphereBoundary sphere(Vector3df(0.0f, 0.0f, 0.0f), 1.0f);

  EXPECT_NEAR(sphere.getSignedDistance(Vector3df(0.0f, 0.0f, 0.0f)), 1.0f, kTol);
  EXPECT_NEAR(sphere.getSignedDistance(Vector3df(1.0f, 0.0f, 0.0f)), 0.0f, kTol);
  EXPECT_NEAR(sphere.getSignedDistance(Vector3df(1.5f, 0.0f, 0.0f)), -0.5f, kTol);
}

TEST(SphereBoundaryTest, IsActiveAtIsFalseBeyondMaxPenetration)
{
  SphereBoundary sphere(Vector3df(0.0f, 0.0f, 0.0f), 1.0f, 0.1f);

  EXPECT_TRUE(sphere.isActiveAt(Vector3df(0.0f, 0.0f, 0.0f)));
  EXPECT_TRUE(sphere.isActiveAt(Vector3df(1.05f, 0.0f, 0.0f)));
  EXPECT_FALSE(sphere.isActiveAt(Vector3df(1.5f, 0.0f, 0.0f)));
}

TEST(SphereBoundaryTest, BoundaryForceIsZeroInsideSphere)
{
  SphereBoundary sphere(Vector3df(0.0f, 0.0f, 0.0f), 1.0f);

  const auto force = sphere.getBoundaryForce(Vector3df(0.5f, 0.0f, 0.0f), 0.01f);

  EXPECT_NEAR(force.x, 0.0f, kTol);
  EXPECT_NEAR(force.y, 0.0f, kTol);
  EXPECT_NEAR(force.z, 0.0f, kTol);
}

TEST(SphereBoundaryTest, BoundaryForceIsZeroBeyondMaxPenetration)
{
  SphereBoundary sphere(Vector3df(0.0f, 0.0f, 0.0f), 1.0f, 0.1f);

  const auto force = sphere.getBoundaryForce(Vector3df(1.5f, 0.0f, 0.0f), 0.01f);

  EXPECT_NEAR(force.x, 0.0f, kTol);
  EXPECT_NEAR(force.y, 0.0f, kTol);
  EXPECT_NEAR(force.z, 0.0f, kTol);
}

TEST(SphereBoundaryTest, BoundaryForcePushesTowardCenterWhenPenetrating)
{
  SphereBoundary sphere(Vector3df(0.0f, 0.0f, 0.0f), 1.0f);

  const auto force = sphere.getBoundaryForce(Vector3df(1.1f, 0.0f, 0.0f), 0.01f);

  EXPECT_LT(force.x, 0.0f);
  EXPECT_NEAR(force.y, 0.0f, kTol);
  EXPECT_NEAR(force.z, 0.0f, kTol);

  const auto forceOffAxis = sphere.getBoundaryForce(Vector3df(0.0f, 0.0f, -1.2f), 0.01f);
  EXPECT_GT(forceOffAxis.z, 0.0f);
  EXPECT_NEAR(forceOffAxis.x, 0.0f, kTol);
  EXPECT_NEAR(forceOffAxis.y, 0.0f, kTol);
}

TEST(SphereBoundaryTest, ClampPositionProjectsPenetratingPointOntoSphereSurface)
{
  SphereBoundary sphere(Vector3df(0.0f, 0.0f, 0.0f), 1.0f);

  const auto clamped = sphere.clampPosition(Vector3df(2.0f, 0.0f, 0.0f));
  EXPECT_NEAR(glm::length(clamped), 1.0f, kTol);
  EXPECT_NEAR(clamped.x, 1.0f, kTol);

  const auto unchanged = sphere.clampPosition(Vector3df(0.5f, 0.0f, 0.0f));
  EXPECT_NEAR(unchanged.x, 0.5f, kTol);
  EXPECT_NEAR(unchanged.y, 0.0f, kTol);
  EXPECT_NEAR(unchanged.z, 0.0f, kTol);
}

TEST(SphereBoundaryTest, OffCenterSphereSignedDistanceUsesCenter)
{
  SphereBoundary sphere(Vector3df(1.0f, 2.0f, 3.0f), 0.5f);

  EXPECT_NEAR(sphere.getSignedDistance(Vector3df(1.0f, 2.0f, 3.0f)), 0.5f, kTol);
  EXPECT_NEAR(sphere.getSignedDistance(Vector3df(1.5f, 2.0f, 3.0f)), 0.0f, kTol);
}

// The undamped penalty is a conservative spring: it gives back every bit of
// the penetration it absorbed, so an isolated particle bounces off the wall at
// (slightly over) the speed it arrived with. The damped overload exists to
// take that restitution off -- see boundaryPenaltyAcceleration() and
// docs/issue/water_sphere_showcase_emitter_instability.md section 3.
namespace
{
// Integrates one particle against the sphere wall exactly the way
// WCSPHSolver::addBoundaryForce() + WCSPHParticle::forwardTime() do (the
// penalty is returned as an acceleration), and reports |v_out| / |v_in|.
float measureRestitutionAt(const SphereBoundary& sphere, const float dampingRatio,
                           const float impactSpeed)
{
  const float dt = 2.0e-4f;
  Vector3df pos(0.0f, 0.0f, -sphere.getRadius() + 0.02f);
  Vector3df vel(0.0f, 0.0f, -impactSpeed);

  float outSpeed = 0.0f;
  for (int step = 0; step < 2000; ++step) {
    const auto acc = sphere.getBoundaryForce(pos, vel, dt, dampingRatio);
    vel += acc * dt;
    pos += vel * dt;
    // Left the wall again, heading back into the interior.
    if (sphere.getSignedDistance(pos) > 0.0f && vel.z > 0.0f) {
      outSpeed = glm::length(vel);
      break;
    }
  }
  return outSpeed / impactSpeed;
}

float measureRestitution(const SphereBoundary& sphere, const float dampingRatio)
{
  return measureRestitutionAt(sphere, dampingRatio, 3.3f);
}
}

TEST(SphereBoundaryTest, UndampedBoundaryForceBouncesAtRestitutionOne)
{
  SphereBoundary sphere(Vector3df(0.0f, 0.0f, 0.0f), 0.25f, 0.04f);

  const float restitution = measureRestitution(sphere, 0.0f);

  EXPECT_GT(restitution, 0.95f);
  // Zero damping must reproduce the position-only overload bit for bit.
  const auto damped = sphere.getBoundaryForce(Vector3df(0.0f, 0.0f, -0.26f),
                                               Vector3df(0.0f, 0.0f, -3.3f), 0.01f, 0.0f);
  const auto plain = sphere.getBoundaryForce(Vector3df(0.0f, 0.0f, -0.26f), 0.01f);
  EXPECT_NEAR(damped.z, plain.z, kTol);
}

TEST(SphereBoundaryTest, DampedBoundaryForceAbsorbsMostOfTheRebound)
{
  SphereBoundary sphere(Vector3df(0.0f, 0.0f, 0.0f), 0.25f, 0.04f);

  // Measured across impact speeds from 0.4 to 8 m/s (the penalty spring's
  // stiffness is 1/dt^2, so the contact dynamics are dt-independent and this
  // is the whole picture): restitution falls steeply with the damping ratio,
  // with some scatter from which sub-step the particle first registers the
  // penetration on. At 0.35 it lands between 0.21 and 0.42 whatever the
  // impact speed, which is why the showcase scenes use that value.
  for (const float impactSpeed : { 0.4f, 1.0f, 3.3f, 8.0f }) {
    EXPECT_LT(measureRestitutionAt(sphere, 0.10f, impactSpeed), 0.80f) << "v=" << impactSpeed;
    EXPECT_LT(measureRestitutionAt(sphere, 0.35f, impactSpeed), 0.50f) << "v=" << impactSpeed;
    // The wall never pulls, so nothing can come back out backwards.
    EXPECT_GE(measureRestitutionAt(sphere, 0.35f, impactSpeed), 0.0f) << "v=" << impactSpeed;
  }
}

TEST(SphereBoundaryTest, DampingRatioIsClampedToTheStableRange)
{
  SphereBoundary sphere(Vector3df(0.0f, 0.0f, 0.0f), 0.25f, 0.04f);

  // Above 0.5 the damper would remove more than the whole normal velocity in
  // one step and re-introduce a bounce, so it saturates there instead of
  // getting worse (clampBoundaryDampingRatio()).
  EXPECT_NEAR(measureRestitution(sphere, 5.0f), measureRestitution(sphere, 0.5f), 1.0e-4f);
  // Negative damping (which would *add* energy) is treated as none.
  EXPECT_NEAR(measureRestitution(sphere, -1.0f), measureRestitution(sphere, 0.0f), 1.0e-4f);
}
