#include "pch.h"

#include "../Physics/PlateBoundary.h"

#include <cmath>

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace
{
constexpr float kTol = 1.0e-5f;

// A plate whose frame is axis-aligned (n = +z, u = +x, v = +y): 0.4 x 0.2 m
// footprint, 20 mm thick. Its top face is at z = +halfThickness.
PlateBoundary makeAxisAlignedPlate()
{
  return PlateBoundary(Vector3df(0.0f, 0.0f, 0.0f), Vector3df(0.0f, 0.0f, 1.0f),
                       Vector3df(1.0f, 0.0f, 0.0f), 0.20f, 0.10f, 0.010f);
}
}

// ---- OBB signed-distance convention -------------------------------------
// getSignedDistance() >= 0 is the valid/exterior region, matching
// PlaneBoundary and SphereBoundary; the deepest interior point is exactly
// -halfThickness (docs/todo/PLAN_sph_showcase_waterfall.md section 3.1).

TEST(PlateBoundaryTest, SignedDistanceIsPositiveOutsideAndNegativeInside)
{
  const auto plate = makeAxisAlignedPlate();

  // Dead centre: inside, the deepest point of a 10 mm half-thickness slab.
  EXPECT_NEAR(plate.getSignedDistance(Vector3df(0.0f, 0.0f, 0.0f)), -0.010f, kTol);

  // 50 mm clear of the top face, over the footprint: outside by 50 mm.
  EXPECT_NEAR(plate.getSignedDistance(Vector3df(0.0f, 0.0f, 0.060f)), 0.050f, kTol);

  // Well past the +u edge at the plate's mid-plane: outside by (0.30 - 0.20).
  EXPECT_NEAR(plate.getSignedDistance(Vector3df(0.30f, 0.0f, 0.0f)), 0.10f, kTol);

  // Inside the footprint and within the thickness.
  EXPECT_LT(plate.getSignedDistance(Vector3df(0.10f, 0.05f, 0.005f)), 0.0f);
}

// ---- Water runs off the edge -------------------------------------------
// The single most important behaviour in the whole finite-plate design:
// directly beyond the footprint the plate exerts no force at all, so a
// particle there falls freely instead of being held by an invisible wall.

TEST(PlateBoundaryTest, NoForceBeyondFootprint)
{
  const auto plate = makeAxisAlignedPlate();
  const float dt = 1.0e-3f;

  // Beyond the +u edge, at the plate's mid-plane height (so it would be
  // "penetrating" a zero-thickness infinite plane): no force.
  const auto outside = plate.getBoundaryForce(Vector3df(0.50f, 0.0f, 0.0f), dt);
  EXPECT_NEAR(glm::length(outside), 0.0f, kTol);

  // Just inside the same edge and penetrating: force is non-zero, so the
  // zero above is the footprint boundary and not a dead method.
  const auto inside = plate.getBoundaryForce(Vector3df(0.19f, 0.0f, 0.0f), dt);
  EXPECT_GT(glm::length(inside), 0.0f);
}

// ---- Penalty force pushes out of the nearest face ----------------------

TEST(PlateBoundaryTest, ForcePushesOutOfNearestFace)
{
  const auto plate = makeAxisAlignedPlate();
  const float dt = 1.0e-3f;

  // 2 mm below the top face (half-thickness 10 mm): nearest face is +z.
  const auto up = plate.getBoundaryForce(Vector3df(0.0f, 0.0f, 0.008f), dt);
  EXPECT_GT(up.z, 0.0f);
  EXPECT_NEAR(up.x, 0.0f, kTol);
  EXPECT_NEAR(up.y, 0.0f, kTol);
  // Magnitude is the standard -d/dt^2 spring: penetration 2 mm.
  EXPECT_NEAR(up.z, 0.002f / (dt * dt), 1.0e-2f * (0.002f / (dt * dt)));

  // 2 mm above the bottom face: nearest face is -z.
  const auto down = plate.getBoundaryForce(Vector3df(0.0f, 0.0f, -0.008f), dt);
  EXPECT_LT(down.z, 0.0f);
}

// ---- Tilted plate (the waterfall's mid shelf, #6) ---------------------
// The force must be along the plate's OWN normal, not a world axis.

TEST(PlateBoundaryTest, TiltedPlateForceIsAlongItsOwnNormal)
{
  const Vector3df n = glm::normalize(Vector3df(0.208f, 0.0f, 0.978f));
  const Vector3df center(0.135f, 0.0f, 0.226f);
  PlateBoundary shelf(center, n, Vector3df(0.978f, 0.0f, -0.208f), 0.180f, 0.085f, 0.012f);
  const float dt = 1.8e-4f;

  // 10 mm in from the top face along -n (i.e. penetrating from the +n side).
  const Vector3df pos = center + n * 0.010f;
  const auto force = shelf.getBoundaryForce(pos, dt);

  ASSERT_GT(glm::length(force), 0.0f);
  const Vector3df dir = glm::normalize(force);
  EXPECT_NEAR(glm::dot(dir, n), 1.0f, 1.0e-4f);
}

// ---- Half-thickness is the penetration bound -------------------------
// A zero-thickness plate needs a maxPenetration safety valve; a slab does
// not, because the box SDF cannot report anything deeper than -halfThickness
// no matter where the point is.

TEST(PlateBoundaryTest, HalfThicknessBoundsPenetration)
{
  const auto plate = makeAxisAlignedPlate();
  const float halfThickness = 0.010f;

  for (float u = -0.30f; u <= 0.30f; u += 0.05f) {
    for (float v = -0.15f; v <= 0.15f; v += 0.05f) {
      for (float w = -0.03f; w <= 0.03f; w += 0.005f) {
        EXPECT_GE(plate.getSignedDistance(Vector3df(u, v, w)), -halfThickness - kTol)
            << "u=" << u << " v=" << v << " w=" << w;
      }
    }
  }

  // The deepest point is exactly -halfThickness, and clamping it lands on a face.
  const Vector3df deepest(0.0f, 0.0f, 0.0f);
  EXPECT_NEAR(plate.getSignedDistance(deepest), -halfThickness, kTol);
  EXPECT_NEAR(plate.getSignedDistance(plate.clampPosition(deepest)), 0.0f, kTol);
}

// ---- Early-rejection AABB is conservative --------------------------------

TEST(PlateBoundaryTest, IsActiveAtCoversTheInflatedFootprint)
{
  const auto plate = makeAxisAlignedPlate();
  const float h = 0.02f;

  // Inside the footprint, within one support of the top face: active.
  EXPECT_TRUE(plate.isActiveAt(Vector3df(0.0f, 0.0f, 0.015f), h));
  // Just past the +u edge but within one support of it: still active
  // (the AABB is inflated by h, so the force/density loops don't skip it).
  EXPECT_TRUE(plate.isActiveAt(Vector3df(0.20f + 0.5f * h, 0.0f, 0.011f), h));
  // Several supports clear of the plate in every direction: rejected.
  EXPECT_FALSE(plate.isActiveAt(Vector3df(0.0f, 0.0f, 0.20f), h));
  EXPECT_FALSE(plate.isActiveAt(Vector3df(0.40f, 0.0f, 0.0f), h));
}
