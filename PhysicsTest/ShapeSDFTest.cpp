#include "pch.h"
#include "../Physics/ICollisionShape.h"
#include "CGLib/ThirdParty/glm-0.9.9.8/glm/gtc/quaternion.hpp"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
constexpr float kTol = 1.0e-5f;
const Quaternion kIdentity(1.f, 0.f, 0.f, 0.f);
}

// ------------------------------------------------------------- SphereShape --

TEST(ShapeSDFTest, Sphere_Outside_PositiveDistance) {
    SphereShape sphere;
    sphere.radius = 0.5f;
    Vector3df pos(1.f, 0.f, 0.f);

    float d = sphere.getSignedDistance({ 3.f, 0.f, 0.f }, pos, kIdentity);
    EXPECT_NEAR(d, 1.5f, kTol);
}

TEST(ShapeSDFTest, Sphere_Inside_NegativeDistance) {
    SphereShape sphere;
    sphere.radius = 1.f;
    Vector3df pos(0.f, 0.f, 0.f);

    float d = sphere.getSignedDistance({ 0.3f, 0.f, 0.f }, pos, kIdentity);
    EXPECT_NEAR(d, -0.7f, kTol);
}

TEST(ShapeSDFTest, Sphere_SurfaceNormal_PointsAwayFromCenter) {
    SphereShape sphere;
    sphere.radius = 0.5f;
    Vector3df pos(0.f, 1.f, 0.f);

    Vector3df n = sphere.getSurfaceNormal({ 0.f, 4.f, 0.f }, pos, kIdentity);
    EXPECT_NEAR(n.x, 0.f, kTol);
    EXPECT_NEAR(n.y, 1.f, kTol);
    EXPECT_NEAR(n.z, 0.f, kTol);
}

// --------------------------------------------------------------- PlaneShape --

TEST(ShapeSDFTest, Plane_Above_PositiveDistance) {
    PlaneShape plane;
    plane.normal = { 0.f, 1.f, 0.f };
    plane.offset = 1.f;

    float d = plane.getSignedDistance({ 0.f, 4.f, 0.f }, {}, kIdentity);
    EXPECT_NEAR(d, 3.f, kTol);
}

TEST(ShapeSDFTest, Plane_Below_NegativeDistance) {
    PlaneShape plane;
    plane.normal = { 0.f, 1.f, 0.f };
    plane.offset = 1.f;

    float d = plane.getSignedDistance({ 0.f, -2.f, 0.f }, {}, kIdentity);
    EXPECT_NEAR(d, -3.f, kTol);
}

TEST(ShapeSDFTest, Plane_SurfaceNormal_MatchesPlaneNormal) {
    PlaneShape plane;
    plane.normal = { 0.f, 0.f, 1.f };
    plane.offset = 0.f;

    Vector3df n = plane.getSurfaceNormal({ 5.f, 5.f, 5.f }, {}, kIdentity);
    EXPECT_NEAR(n.x, 0.f, kTol);
    EXPECT_NEAR(n.y, 0.f, kTol);
    EXPECT_NEAR(n.z, 1.f, kTol);
}

// ----------------------------------------------------------------- BoxShape --

TEST(ShapeSDFTest, Box_OutsideAlongAxis_PositiveDistance) {
    BoxShape box;
    box.halfExtents = { 0.5f, 0.5f, 0.5f };
    Vector3df pos(0.f, 0.f, 0.f);

    float d = box.getSignedDistance({ 2.f, 0.f, 0.f }, pos, kIdentity);
    EXPECT_NEAR(d, 1.5f, kTol);
}

TEST(ShapeSDFTest, Box_OutsideCorner_EuclideanDistance) {
    BoxShape box;
    box.halfExtents = { 0.5f, 0.5f, 0.5f };
    Vector3df pos(0.f, 0.f, 0.f);

    // corner of the box is at (0.5,0.5,0.5); point offset by (1,1,1) beyond it
    float d = box.getSignedDistance({ 1.5f, 1.5f, 1.5f }, pos, kIdentity);
    EXPECT_NEAR(d, std::sqrt(3.f), kTol);
}

TEST(ShapeSDFTest, Box_Inside_NegativeDistanceToNearestFace) {
    BoxShape box;
    box.halfExtents = { 1.f, 2.f, 3.f };
    Vector3df pos(0.f, 0.f, 0.f);

    // nearest face is +/-x (halfExtents.x=1), point at x=0.7 => distance to face = 0.3
    float d = box.getSignedDistance({ 0.7f, 0.f, 0.f }, pos, kIdentity);
    EXPECT_NEAR(d, -0.3f, kTol);
}

TEST(ShapeSDFTest, Box_SurfaceNormal_OutsideAlongAxis) {
    BoxShape box;
    box.halfExtents = { 0.5f, 0.5f, 0.5f };
    Vector3df pos(0.f, 0.f, 0.f);

    Vector3df n = box.getSurfaceNormal({ 2.f, 0.f, 0.f }, pos, kIdentity);
    EXPECT_NEAR(n.x, 1.f, kTol);
    EXPECT_NEAR(n.y, 0.f, kTol);
    EXPECT_NEAR(n.z, 0.f, kTol);
}

TEST(ShapeSDFTest, Box_SurfaceNormal_InsideNearestFace) {
    BoxShape box;
    box.halfExtents = { 1.f, 2.f, 3.f };
    Vector3df pos(0.f, 0.f, 0.f);

    Vector3df n = box.getSurfaceNormal({ 0.7f, 0.f, 0.f }, pos, kIdentity);
    EXPECT_NEAR(n.x, 1.f, kTol);
    EXPECT_NEAR(n.y, 0.f, kTol);
    EXPECT_NEAR(n.z, 0.f, kTol);
}

TEST(ShapeSDFTest, Box_Rotated90AroundZ_DistanceUsesLocalFrame) {
    BoxShape box;
    box.halfExtents = { 0.5f, 1.f, 0.5f };   // tall along local Y
    Vector3df pos(0.f, 0.f, 0.f);
    Quaternion rot = glm::angleAxis(glm::radians(90.f), Vector3df(0.f, 0.f, 1.f));

    // Rotating the box 90 deg around Z swaps local X/Y extents in world space:
    // world +X now aligned with local +Y (halfExtent 1.0).
    float d = box.getSignedDistance({ 1.5f, 0.f, 0.f }, pos, rot);
    EXPECT_NEAR(d, 0.5f, kTol);
}

TEST(ShapeSDFTest, Box_Rotated90AroundZ_NormalRotatesWithBox) {
    BoxShape box;
    box.halfExtents = { 0.5f, 1.f, 0.5f };
    Vector3df pos(0.f, 0.f, 0.f);
    Quaternion rot = glm::angleAxis(glm::radians(90.f), Vector3df(0.f, 0.f, 1.f));

    Vector3df n = box.getSurfaceNormal({ 1.5f, 0.f, 0.f }, pos, rot);
    EXPECT_NEAR(n.x, 1.f, kTol);
    EXPECT_NEAR(n.y, 0.f, kTol);
    EXPECT_NEAR(n.z, 0.f, kTol);
}
