#include "pch.h"
#include "../Physics/HairSolver.h"
#include "../Physics/HairGenerator.h"
#include "../PhysicsView/HairWorld.h"
#include <limits>

using namespace Phantom::Physics;
using Phantom::Math::Vector3df;

namespace {
HairGeneratorParams one() {
    HairGeneratorParams p;
    p.strands = 1; p.particlesPerStrand = 16; p.length = 0.6f;
    return p;
}
}
TEST(HairCollisionTest, CapsuleBarrelEndpointsAndSphereCentre) {
    HairCollider capsule{{0.f,-1.f,0.f}, {0.f,1.f,0.f}, 0.2f, 0.3f};
    auto h = sampleHairContact({0.1f,0.f,0.f}, {0.1f,0.f,0.f}, capsule, 0.01f);
    EXPECT_NEAR(h.penetration, 0.11f, 1.e-6f);
    EXPECT_NEAR(h.colliderFraction, 0.5f, 1.e-6f);
    EXPECT_NEAR(h.normal.x, 1.f, 1.e-6f);
    h = sampleHairContact({0.f,1.1f,0.f}, {0.f,1.1f,0.f}, capsule, 0.01f);
    EXPECT_NEAR(h.penetration, 0.11f, 1.e-6f);
    EXPECT_NEAR(h.colliderFraction, 1.f, 1.e-6f);
    EXPECT_NEAR(h.normal.y, 1.f, 1.e-6f);
    HairCollider sphere{{0.f,0.f,0.f}, {0.f,0.f,0.f}, 0.2f, 0.f};
    h = sampleHairContact(Vector3df(0.f), Vector3df(0.f), sphere, 0.01f);
    EXPECT_NEAR(h.penetration, 0.21f, 1.e-6f);
    EXPECT_NEAR(glm::length(h.normal), 1.f, 1.e-6f);
}
TEST(HairCollisionTest, StrandInteriorDetectsCollisionWithBothEndpointsOutside) {
    HairCollider c{{0.f,-0.5f,0.f}, {0.f,0.5f,0.f}, 0.2f, 0.f};
    const auto h = sampleHairContact({-1.f,0.f,0.f}, {1.f,0.f,0.f}, c, 0.01f);
    EXPECT_NEAR(h.strandFraction, 0.5f, 1.e-6f);
    EXPECT_NEAR(h.penetration, 0.21f, 1.e-6f);
    EXPECT_NEAR(glm::length(h.normal), 1.f, 1.e-6f);
}
TEST(HairCollisionTest, ParallelSegmentsAndDegenerateCapsuleAreFinite) {
    HairCollider c{{0.f,0.f,0.f}, {0.f,1.f,0.f}, 0.2f, 0.f};
    auto h = sampleHairContact({0.1f,0.2f,0.f}, {0.1f,0.8f,0.f}, c, 0.f);
    EXPECT_NEAR(h.penetration, 0.1f, 1.e-6f);
    c.b = c.a;
    h = sampleHairContact({-1.f,0.f,0.f}, {1.f,0.f,0.f}, c, 0.f);
    EXPECT_NEAR(h.penetration, 0.2f, 1.e-6f);
    EXPECT_TRUE(std::isfinite(glm::length(h.normal)));
}
TEST(HairCollisionTest, FrictionUsesVelocityRelativeToMovingBoundary) {
    const Vector3df normal(0.f,1.f,0.f), wall(1.f,0.f,0.f), velocity(4.f,-1.f,0.f);
    const auto slip = hairContactVelocity(velocity, normal, wall, 0.1f, 0.f, 0.1f);
    const auto friction = hairContactVelocity(velocity, normal, wall, 0.1f, 0.5f, 0.1f);
    EXPECT_NEAR(slip.x, 4.f, 1.e-6f);
    EXPECT_NEAR(friction.x, 3.f, 1.e-6f);
    EXPECT_NEAR(friction.y, 0.f, 1.e-6f);
    const auto carried = hairContactVelocity(Vector3df(0.f), normal, {2.f,0.f,0.f}, 0.2f, 1.f, 0.1f);
    EXPECT_NEAR(carried.x, 2.f, 1.e-6f);
}
TEST(HairDynamicsTest, RootTranslationIsInterpolatedAndFreeTipsLag) {
    HairStrands hair;
    ASSERT_TRUE(generateHairBundle(hair, one()));
    HairSolver solver;
    auto p = solver.params(); p.gravity = Vector3df(0.f); p.shapeEnabled = false;
    ASSERT_TRUE(solver.setParams(p)); solver.setStrands(&hair);
    const auto tip = hair.particles().positions.back();
    auto root = hair.ranges()[0].root;
    root.position.x += 0.02f;
    ASSERT_TRUE(solver.setRootPose(0, root));
    ASSERT_TRUE(solver.step());
    EXPECT_LT(glm::length(hair.particles().positions[0]-root.position), 1.e-6f);
    EXPECT_LT(hair.particles().positions.back().x-tip.x, 0.02f);
    EXPECT_EQ(solver.stats().rootResets, 0u);
    EXPECT_LT(solver.stats().maxSpeed, 10.f);
}
TEST(HairDynamicsTest, RestShapeRotatesWithRootAndResetUsesCurrentPose) {
    HairStrands hair;
    generateHairBundle(hair, one());
    HairSolver solver;
    auto p = solver.params(); p.gravity = Vector3df(0.f); p.shapeCompliance = 0.f;
    solver.setParams(p); solver.setStrands(&hair);
    auto root = hair.ranges()[0].root;
    root.rotation = glm::angleAxis(0.4f, Vector3df(0.f,0.f,1.f));
    ASSERT_TRUE(solver.setRootPose(0, root));
    ASSERT_TRUE(solver.step());
    for (size_t i = 0; i < hair.particleCount(); ++i)
        EXPECT_LT(glm::length(hair.particles().positions[i] - (root.position+root.rotation*hair.restPositions()[i])), 2.e-5f);
    solver.reset();
    EXPECT_EQ(solver.stats().steps, 0u);
    EXPECT_FLOAT_EQ(solver.stats().maxSpeed, 0.f);
    EXPECT_LT(glm::length(hair.particles().positions.back() - (root.position+root.rotation*hair.restPositions().back())), 1.e-6f);
}
TEST(HairDynamicsTest, TeleportAndLargeRotationResetWithoutInjectingVelocity) {
    HairStrands hair;
    generateHairBundle(hair, one());
    HairSolver solver; solver.setStrands(&hair); solver.step();
    const auto time = solver.stats().simulatedTime;
    auto root = hair.ranges()[0].root;
    root.position.x += 2.f;
    ASSERT_TRUE(solver.setRootPose(0, root));
    EXPECT_EQ(solver.stats().rootResets, 1u);
    EXPECT_EQ(solver.stats().simulatedTime, time);
    EXPECT_FLOAT_EQ(solver.stats().maxSpeed, 0.f);
    root.rotation = glm::angleAxis(2.8f, Vector3df(0.f,1.f,0.f));
    ASSERT_TRUE(solver.setRootPose(0, root));
    EXPECT_EQ(solver.stats().rootResets, 2u);
    root.rotation = -root.rotation; // equivalent quaternion, no jump
    ASSERT_TRUE(solver.setRootPose(0, root));
    EXPECT_EQ(solver.stats().rootResets, 2u);
    EXPECT_FLOAT_EQ(solver.stats().maxSpeed, 0.f);
}
TEST(HairDynamicsTest, SegmentCollisionResolvesSphereAndCapsuleInteriors) {
    for (bool sphere : {false,true}) {
        HairStrands hair;
        HairStrandInput input;
        input.root.position = {-1.f,0.f,0.f};
        input.restPositions = {{0.f,0.f,0.f}, {2.f,0.f,0.f}};
        ASSERT_TRUE(hair.initialize({input}));
        HairSolver solver;
        auto p = solver.params(); p.gravity = Vector3df(0.f); p.shapeEnabled = false;
        p.numIterations = 32;
        solver.setParams(p); solver.setStrands(&hair);
        HairCollider c{{0.f,-0.5f,0.f}, {0.f,0.5f,0.f}, 0.2f, 0.3f};
        if (sphere) c.a = c.b = Vector3df(0.f);
        ASSERT_TRUE(solver.setColliders({c}));
        for (int i = 0; i < 10; ++i) ASSERT_TRUE(solver.step());
        EXPECT_LT(solver.stats().maxPenetration, 1.e-4f);
        EXPECT_LT(solver.stats().maxRelativeLengthError, 0.01f);
        EXPECT_FLOAT_EQ(hair.particles().positions[0].x, -1.f);
    }
}
TEST(HairDynamicsTest, RejectsInvalidPoseCollidersAndWindAtomically) {
    HairStrands hair; generateHairBundle(hair, one());
    HairSolver solver; solver.setStrands(&hair);
    const auto old = hair.ranges()[0].root;
    auto root = old; root.rotation = {0.f,0.f,0.f,0.f};
    EXPECT_FALSE(solver.setRootPose(0, root));
    EXPECT_FALSE(solver.setRootPose(1, old));
    root = old; root.position.x = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(solver.setRootPose(0, root));
    EXPECT_EQ(hair.ranges()[0].root.position, old.position);
    HairCollider c; ASSERT_TRUE(solver.setColliders({c}));
    c.radius = -1.f;
    EXPECT_FALSE(solver.setColliders({c}));
    EXPECT_EQ(solver.colliders().size(), 1u);
    auto p = solver.params(); p.windDrag = -1.f;
    EXPECT_FALSE(solver.setParams(p));
    p = solver.params(); p.collisionRadius = -1.f;
    EXPECT_FALSE(solver.setParams(p));
    p = solver.params(); p.teleportAngle = 0.f;
    EXPECT_FALSE(solver.setParams(p));
}
TEST(HairDynamicsTest, StrongWindAndTimeStepGroupingRemainStable) {
    std::vector<Vector3df> tips;
    for (float dt : {1.f/30.f,1.f/60.f,1.f/120.f}) {
        HairStrands hair; generateHairBundle(hair, one());
        const auto initial = hair.particles().positions.back();
        HairSolver solver;
        auto p = solver.params(); p.timeStep = dt; p.numSubsteps = static_cast<int>(std::round(dt*480.f));
        p.windVelocity = {20.f,0.f,0.f}; p.windDrag = 3.f;
        solver.setParams(p); solver.setStrands(&hair);
        for (int i = 0; i < static_cast<int>(std::round(0.5f/dt)); ++i) ASSERT_TRUE(solver.step());
        EXPECT_GT(hair.particles().positions.back().x-initial.x, 0.05f);
        EXPECT_LT(solver.stats().maxSpeed, 50.f);
        EXPECT_LT(solver.stats().maxRelativeLengthError, 0.05f);
        tips.push_back(hair.particles().positions.back());
    }
    EXPECT_LT(glm::length(tips[0]-tips[1]), 5.e-4f);
    EXPECT_LT(glm::length(tips[1]-tips[2]), 5.e-4f);
}
TEST(HairDynamicsTest, MovingBoundaryAndPinnedOverlapAreReported) {
    HairStrands hair; generateHairBundle(hair, one());
    HairSolver solver; solver.setStrands(&hair);
    HairCollider c;
    c.a = c.b = hair.ranges()[0].root.position;
    solver.setColliders({c});
    EXPECT_GT(solver.stats().pinnedPenetration, 0.f);
    c.a.x += 0.01f; c.b = c.a;
    ASSERT_TRUE(solver.setColliders({c}));
    ASSERT_TRUE(solver.step());
    EXPECT_GT(solver.stats().pinnedPenetration, 0.f);
    EXPECT_TRUE(std::isfinite(solver.stats().maxSpeed));
    EXPECT_LT(glm::length(hair.particles().positions[0]-hair.ranges()[0].root.position), 1.e-6f);
}
TEST(HairDynamicsTest, HeadShakeAndBodyStayFinite) {
    Phantom::HairWorld world;
    ASSERT_TRUE(world.setPreset(Phantom::HairPreset::HeadShake));
    EXPECT_EQ(world.colliderCount(), 2u);
    for (int i = 0; i < 60; ++i) ASSERT_TRUE(world.stepOnce());
    EXPECT_LT(world.maxRootError(), 1.e-6f);
    EXPECT_EQ(world.stats().nonFiniteCount, 0u);
    EXPECT_EQ(world.stats().rootResets, 0u);
    EXPECT_LT(world.stats().maxSpeed, 50.f);
    EXPECT_LT(world.stats().maxPenetration, 0.001f);
}
