#include "pch.h"
#include "../Physics/RigidBody.h"
#include "../Physics/RigidBodySolver.h"
#include "../Physics/NarrowPhase.h"
#include "CGLib/ThirdParty/glm-0.9.9.8/glm/gtc/quaternion.hpp"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
constexpr float kTol = 1.0e-4f;

// Capsule lying along world X (local Y rotated -90 deg about Z).
Quaternion lyingOnX() {
    return glm::angleAxis(glm::radians(-90.f), Vector3df(0.f, 0.f, 1.f));
}
} // namespace

// ------------------------------------------------ shape queries --------

TEST(CapsuleShapeTest, SignedDistance_CapAndSide) {
    CapsuleShape c;
    c.radius = 0.25f;
    c.halfHeight = 0.5f;
    const Vector3df pos(0.f, 1.f, 0.f);
    const Quaternion id(1.f, 0.f, 0.f, 0.f);

    // Above the top cap: segment end at y=1.5, surface at y=1.75.
    EXPECT_NEAR(c.getSignedDistance({ 0.f, 2.f, 0.f }, pos, id), 0.25f, kTol);
    // Beside the cylinder.
    EXPECT_NEAR(c.getSignedDistance({ 1.f, 1.2f, 0.f }, pos, id), 0.75f, kTol);
    // On the core segment: fully inside.
    EXPECT_NEAR(c.getSignedDistance({ 0.f, 1.3f, 0.f }, pos, id), -0.25f, kTol);

    const Vector3df n = c.getSurfaceNormal({ 1.f, 1.2f, 0.f }, pos, id);
    EXPECT_NEAR(n.x, 1.f, kTol);
    EXPECT_NEAR(n.y, 0.f, kTol);
}

TEST(CapsuleShapeTest, AABB_FollowsOrientation) {
    CapsuleShape c;
    c.radius = 0.25f;
    c.halfHeight = 0.5f;
    const Box3df upright = c.getAABB({ 0.f, 0.f, 0.f }, Quaternion(1.f, 0.f, 0.f, 0.f));
    EXPECT_NEAR(upright.getMax().y, 0.75f, kTol);
    EXPECT_NEAR(upright.getMax().x, 0.25f, kTol);

    const Box3df lying = c.getAABB({ 0.f, 0.f, 0.f }, lyingOnX());
    EXPECT_NEAR(lying.getMax().x, 0.75f, kTol);
    EXPECT_NEAR(lying.getMax().y, 0.25f, kTol);
}

TEST(CapsuleShapeTest, Inertia_LongAxisIsEasiestToSpin) {
    CapsuleShape c;
    c.radius = 0.2f;
    c.halfHeight = 0.6f;
    RigidBody b;
    b.setShape(&c);
    b.setMass(2.f);
    b.updateInertiaTensor();
    const Matrix3df invI = b.getInverseInertiaTensorWorld();
    // Spinning about the long (Y) axis is easier than tumbling end over end.
    EXPECT_GT(invI[1][1], invI[0][0]);
    EXPECT_NEAR(invI[0][0], invI[2][2], 1e-6f);
    EXPECT_GT(invI[0][0], 0.f);
}

// ------------------------------------------------ narrow phase --------

TEST(CapsuleShapeTest, CapsulePlane_LyingGivesTwoContacts) {
    CapsuleShape c;
    c.radius = 0.25f;
    c.halfHeight = 0.5f;
    PlaneShape p;

    RigidBody cap, floor;
    cap.setShape(&c); cap.setMass(1.f);
    cap.position = { 0.f, 0.2f, 0.f };
    cap.orientation = lyingOnX();
    floor.setShape(&p); floor.setMass(0.f);

    ContactManifold out;
    ASSERT_TRUE(NarrowPhase::detect(cap, floor, out));
    ASSERT_EQ(out.contacts.size(), 2u);
    for (const auto& cp : out.contacts) {
        EXPECT_NEAR(cp.penetration, 0.05f, kTol);
        EXPECT_NEAR(cp.normal.y, 1.f, kTol); // from the floor (B) toward the capsule (A)
    }

    // Swapped order: normals flip so they still point from B (capsule) to A (floor).
    ContactManifold swapped;
    ASSERT_TRUE(NarrowPhase::detect(floor, cap, swapped));
    ASSERT_EQ(swapped.contacts.size(), 2u);
    EXPECT_NEAR(swapped.contacts[0].normal.y, -1.f, kTol);
}

TEST(CapsuleShapeTest, CapsuleSphere_UsesClosestSegmentPoint) {
    CapsuleShape c;
    c.radius = 0.25f;
    c.halfHeight = 0.5f;
    SphereShape s;
    s.radius = 0.25f;

    RigidBody cap, ball;
    cap.setShape(&c); cap.setMass(1.f);
    ball.setShape(&s); ball.setMass(1.f);
    ball.position = { 0.4f, 0.3f, 0.f }; // beside the cylinder, not the center

    ContactManifold out;
    ASSERT_TRUE(NarrowPhase::detect(cap, ball, out));
    ASSERT_EQ(out.contacts.size(), 1u);
    EXPECT_NEAR(out.contacts[0].penetration, 0.1f, kTol);
    EXPECT_NEAR(out.contacts[0].normal.x, -1.f, kTol);

    ball.position = { 0.6f, 0.3f, 0.f };
    ContactManifold miss;
    EXPECT_FALSE(NarrowPhase::detect(cap, ball, miss));
}

TEST(CapsuleShapeTest, CapsuleBox_LyingOnBoxTop) {
    CapsuleShape c;
    c.radius = 0.25f;
    c.halfHeight = 0.5f;
    BoxShape box;
    box.halfExtents = { 1.f, 0.5f, 1.f };

    RigidBody cap, table;
    cap.setShape(&c); cap.setMass(1.f);
    cap.orientation = lyingOnX();
    cap.position = { 0.f, 0.7f, 0.f }; // 0.05 into the box top (y=0.5)
    table.setShape(&box); table.setMass(0.f);

    ContactManifold out;
    ASSERT_TRUE(NarrowPhase::detect(cap, table, out));
    ASSERT_GE(out.contacts.size(), 2u);
    for (const auto& cp : out.contacts) {
        EXPECT_NEAR(cp.penetration, 0.05f, kTol);
        EXPECT_NEAR(cp.normal.y, 1.f, kTol);
    }
}

TEST(CapsuleShapeTest, CapsuleBox_SideHitMidSegment) {
    CapsuleShape c;
    c.radius = 0.25f;
    c.halfHeight = 1.f;
    BoxShape box;
    box.halfExtents = { 0.5f, 0.5f, 0.5f };

    RigidBody cap, block;
    cap.setShape(&c); cap.setMass(1.f);
    cap.position = { 0.7f, 0.f, 0.f }; // upright, grazing the +X face at mid height
    block.setShape(&box); block.setMass(0.f);

    ContactManifold out;
    ASSERT_TRUE(NarrowPhase::detect(cap, block, out));
    float maxPen = 0.f;
    for (const auto& cp : out.contacts) maxPen = std::max(maxPen, cp.penetration);
    EXPECT_NEAR(maxPen, 0.05f, kTol);
}

TEST(CapsuleShapeTest, CapsuleCapsule_Crossed) {
    CapsuleShape c;
    c.radius = 0.25f;
    c.halfHeight = 0.5f;
    RigidBody a, b;
    a.setShape(&c); a.setMass(1.f);
    b.setShape(&c); b.setMass(1.f);
    a.orientation = lyingOnX();
    a.position = { 0.f, 0.4f, 0.f };
    // b along world Z, below a.
    b.orientation = glm::angleAxis(glm::radians(90.f), Vector3df(1.f, 0.f, 0.f));
    b.position = { 0.f, 0.f, 0.f };

    ContactManifold out;
    ASSERT_TRUE(NarrowPhase::detect(a, b, out));
    ASSERT_EQ(out.contacts.size(), 1u);
    EXPECT_NEAR(out.contacts[0].penetration, 0.1f, kTol);
    EXPECT_NEAR(out.contacts[0].normal.y, 1.f, kTol);
}

// ------------------------------------------------ solver --------

TEST(CapsuleShapeTest, Solver_LyingCapsuleRestsOnFloor) {
    RigidBodySolver world;
    world.timeStep = 1.f / 60.f;

    PlaneShape floorShape;
    RigidBody floor;
    floor.setShape(&floorShape);
    floor.setMass(0.f);
    world.addBody(&floor);

    CapsuleShape capShape;
    capShape.radius = 0.2f;
    capShape.halfHeight = 0.4f;
    RigidBody cap;
    cap.setShape(&capShape);
    cap.setMass(1.f);
    cap.position = { 0.f, 1.f, 0.f };
    cap.orientation = lyingOnX();
    cap.updateInertiaTensor();
    cap.friction = 0.6f;
    world.addBody(&cap);

    world.setRunning(true);
    for (int i = 0; i < 150; ++i) world.stepUnconditional();
    const Vector3df settled = cap.position;
    for (int i = 0; i < 30; ++i) world.stepUnconditional();

    EXPECT_NEAR(cap.position.y, 0.2f, 0.03f);
    // Still lying: the local axis stays horizontal.
    const Vector3df axis = cap.orientation * Vector3df(0.f, 1.f, 0.f);
    EXPECT_LT(std::abs(axis.y), 0.05f);
    // At rest the solver alternates between a contact step and one g*dt free-fall step, so
    // check the drift over the last half second instead of the instantaneous velocity.
    EXPECT_LT(glm::length(cap.position - settled), 0.01f);
}
