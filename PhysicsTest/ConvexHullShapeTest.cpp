#include "pch.h"
#include "../Physics/ConvexHullShape.h"
#include "../Physics/RigidBody.h"
#include "../Physics/RigidBodySolver.h"
#include "../Physics/NarrowPhase.h"
#include "CGLib/ThirdParty/glm-0.9.9.8/glm/gtc/quaternion.hpp"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
constexpr float kTol = 1.0e-4f;

// Corners of an axis-aligned box plus redundant interior / face points.
std::vector<Vector3df> boxPoints(const Vector3df& center, const Vector3df& h) {
    std::vector<Vector3df> pts;
    for (int i = 0; i < 8; ++i)
        pts.push_back(center + Vector3df((i & 1) ? h.x : -h.x, (i & 2) ? h.y : -h.y, (i & 4) ? h.z : -h.z));
    pts.push_back(center);
    pts.push_back(center + Vector3df(h.x, 0.f, 0.f));            // face center
    pts.push_back(center + Vector3df(0.f, h.y, h.z));            // edge midpoint
    pts.push_back(center + Vector3df(0.1f * h.x, -0.2f * h.y, 0.3f * h.z));
    return pts;
}

// A wedge (triangular prism): right triangle in XY extruded along Z.
std::vector<Vector3df> wedgePoints() {
    return { { -0.5f, 0.f, -0.5f }, { 0.5f, 0.f, -0.5f }, { -0.5f, 1.f, -0.5f },
             { -0.5f, 0.f,  0.5f }, { 0.5f, 0.f,  0.5f }, { -0.5f, 1.f,  0.5f } };
}

std::vector<Vector3df> spherePoints(int count, float r) {
    std::vector<Vector3df> pts;
    const float golden = 3.14159265f * (3.f - std::sqrt(5.f));
    for (int i = 0; i < count; ++i) {
        const float y = 1.f - 2.f * (i + 0.5f) / count;
        const float s = std::sqrt(1.f - y * y);
        pts.push_back(Vector3df(s * std::cos(golden * i), y, s * std::sin(golden * i)) * r);
    }
    return pts;
}

const Quaternion kIdentity(1.f, 0.f, 0.f, 0.f);
} // namespace

// ------------------------------------------------ building --------------

TEST(ConvexHullShapeTest, Build_BoxPointsGiveSixQuadFaces) {
    ConvexHullShape hull;
    ASSERT_TRUE(hull.build(boxPoints({ 1.f, 2.f, 3.f }, { 0.5f, 1.f, 1.5f })));
    EXPECT_EQ(hull.getVertices().size(), 8u);
    ASSERT_EQ(hull.getFaces().size(), 6u);
    for (const auto& f : hull.getFaces()) EXPECT_EQ(f.loop.size(), 4u);
    EXPECT_EQ(hull.getEdges().size(), 12u);
    EXPECT_EQ(hull.getEdgeDirections().size(), 3u);
    EXPECT_NEAR(hull.getVolume(), 1.f * 2.f * 3.f, 1e-3f);
    // Recentered on the center of mass; the removed offset is reported.
    EXPECT_NEAR(hull.getCenterOffset().x, 1.f, kTol);
    EXPECT_NEAR(hull.getCenterOffset().y, 2.f, kTol);
    EXPECT_NEAR(hull.getCenterOffset().z, 3.f, kTol);
    const Box3df box = hull.getAABB({ 0.f, 0.f, 0.f }, kIdentity);
    EXPECT_NEAR(box.getMax().x, 0.5f, kTol);
    EXPECT_NEAR(box.getMin().z, -1.5f, kTol);
}

TEST(ConvexHullShapeTest, Build_FaceLoopsAreCounterClockwiseAroundNormals) {
    ConvexHullShape hull;
    ASSERT_TRUE(hull.build(wedgePoints()));
    EXPECT_EQ(hull.getFaces().size(), 5u); // 2 triangles + 3 quads
    for (const auto& f : hull.getFaces()) {
        Vector3df newell(0.f);
        for (size_t i = 0; i < f.loop.size(); ++i)
            newell += glm::cross(hull.getVertices()[f.loop[i]],
                                 hull.getVertices()[f.loop[(i + 1) % f.loop.size()]]);
        EXPECT_GT(glm::dot(newell, f.normal), 0.f);
        for (int v : f.loop) // every loop vertex lies on the face plane
            EXPECT_NEAR(glm::dot(f.normal, hull.getVertices()[v]), f.d, 1e-4f);
    }
}

TEST(ConvexHullShapeTest, Build_DegenerateInputFails) {
    ConvexHullShape hull;
    EXPECT_FALSE(hull.build({ { 0.f, 0.f, 0.f }, { 1.f, 0.f, 0.f }, { 0.f, 0.f, 1.f } }));
    // Coplanar points (a flat quad) have no volume.
    EXPECT_FALSE(hull.build({ { 0.f, 0.f, 0.f }, { 1.f, 0.f, 0.f }, { 1.f, 0.f, 1.f },
                              { 0.f, 0.f, 1.f }, { 0.5f, 0.f, 0.5f } }));
    EXPECT_FALSE(hull.isValid());
}

TEST(ConvexHullShapeTest, Build_LargeInputIsCappedAtMaxVertices) {
    ConvexHullShape hull;
    ASSERT_TRUE(hull.build(spherePoints(2000, 1.f), 48));
    EXPECT_LE(hull.getVertices().size(), 48u);
    EXPECT_GE(hull.getVertices().size(), 24u);
    // The reduced hull still spans the sphere.
    const Box3df box = hull.getAABB({ 0.f, 0.f, 0.f }, kIdentity);
    EXPECT_GT(box.getMax().y, 0.9f);
    EXPECT_LT(box.getMin().x, -0.9f);
}

TEST(ConvexHullShapeTest, Inertia_MatchesBoxFormula) {
    ConvexHullShape hull;
    ASSERT_TRUE(hull.build(boxPoints({ 0.f, 0.f, 0.f }, { 0.5f, 1.f, 1.5f })));
    const Matrix3df I = hull.getUnitInertia();
    // Solid box, unit mass: Ixx = (hy^2 + hz^2) / 3.
    EXPECT_NEAR(I[0][0], (1.f + 2.25f) / 3.f, 1e-3f);
    EXPECT_NEAR(I[1][1], (0.25f + 2.25f) / 3.f, 1e-3f);
    EXPECT_NEAR(I[2][2], (0.25f + 1.f) / 3.f, 1e-3f);
    EXPECT_NEAR(I[0][1], 0.f, 1e-4f);
    EXPECT_NEAR(I[1][2], 0.f, 1e-4f);
}

TEST(ConvexHullShapeTest, SignedDistance_InsideAndOutside) {
    ConvexHullShape hull;
    ASSERT_TRUE(hull.build(boxPoints({ 0.f, 0.f, 0.f }, { 0.5f, 0.5f, 0.5f })));
    const Vector3df pos(0.f, 1.f, 0.f);
    EXPECT_NEAR(hull.getSignedDistance({ 0.f, 1.f, 0.f }, pos, kIdentity), -0.5f, kTol);
    EXPECT_NEAR(hull.getSignedDistance({ 0.f, 2.f, 0.f }, pos, kIdentity), 0.5f, kTol);
    // Outside a corner: distance to the corner, not to a face plane.
    EXPECT_NEAR(hull.getSignedDistance({ 1.f, 2.f, 0.f }, pos, kIdentity), std::sqrt(0.5f), kTol);
    const Vector3df n = hull.getSurfaceNormal({ 0.f, 2.f, 0.f }, pos, kIdentity);
    EXPECT_NEAR(n.y, 1.f, kTol);
}

// ------------------------------------------------ narrow phase ----------

TEST(ConvexHullShapeTest, HullPlane_CornersBelowGiveContacts) {
    ConvexHullShape hull;
    ASSERT_TRUE(hull.build(boxPoints({ 0.f, 0.f, 0.f }, { 0.5f, 0.5f, 0.5f })));
    PlaneShape plane;
    RigidBody a, b;
    a.setShape(&hull); a.setMass(1.f);
    a.position = { 0.f, 0.45f, 0.f };
    b.setShape(&plane); b.setMass(0.f);

    ContactManifold m;
    ASSERT_TRUE(NarrowPhase::detect(a, b, m));
    EXPECT_EQ(m.contacts.size(), 4u);
    for (const auto& cp : m.contacts) {
        EXPECT_NEAR(cp.normal.y, 1.f, kTol);
        EXPECT_NEAR(cp.penetration, 0.05f, kTol);
    }
    // Swapped order flips the normal.
    ContactManifold flipped;
    ASSERT_TRUE(NarrowPhase::detect(b, a, flipped));
    EXPECT_NEAR(flipped.contacts[0].normal.y, -1.f, kTol);
}

TEST(ConvexHullShapeTest, HullBox_FaceContactHasFourPoints) {
    ConvexHullShape hull;
    ASSERT_TRUE(hull.build(boxPoints({ 0.f, 0.f, 0.f }, { 0.25f, 0.25f, 0.25f })));
    BoxShape box;
    box.halfExtents = { 1.f, 0.5f, 1.f };
    RigidBody top, ground;
    top.setShape(&hull); top.setMass(1.f);
    top.position = { 0.2f, 0.74f, -0.1f }; // 0.01 into the box top
    ground.setShape(&box); ground.setMass(0.f);

    ContactManifold m;
    ASSERT_TRUE(NarrowPhase::detect(top, ground, m));
    ASSERT_EQ(m.contacts.size(), 4u);
    for (const auto& cp : m.contacts) {
        EXPECT_NEAR(cp.normal.y, 1.f, kTol);           // from the box toward the hull
        EXPECT_NEAR(cp.penetration, 0.01f, 1e-3f);
        EXPECT_NEAR(std::abs(cp.position.x - 0.2f), 0.25f, 1e-3f);
    }
    ContactManifold flipped;
    ASSERT_TRUE(NarrowPhase::detect(ground, top, flipped));
    EXPECT_NEAR(flipped.contacts[0].normal.y, -1.f, kTol);
}

TEST(ConvexHullShapeTest, HullHull_SeparatedGivesNoContact) {
    ConvexHullShape h1, h2;
    ASSERT_TRUE(h1.build(wedgePoints()));
    ASSERT_TRUE(h2.build(wedgePoints()));
    RigidBody a, b;
    a.setShape(&h1); a.setMass(1.f);
    b.setShape(&h2); b.setMass(1.f);
    b.position = { 1.2f, 0.f, 0.f };
    ContactManifold m;
    EXPECT_FALSE(NarrowPhase::detect(a, b, m));
    // Here the AABBs overlap but B sits beyond A's slanted face.
    b.position = { 0.9f, 0.8f, 0.f };
    EXPECT_FALSE(NarrowPhase::detect(a, b, m));
    b.position = { 0.5f, 0.2f, 0.f };
    EXPECT_TRUE(NarrowPhase::detect(a, b, m));
}

TEST(ConvexHullShapeTest, HullSphere_And_HullCapsule) {
    ConvexHullShape hull;
    ASSERT_TRUE(hull.build(boxPoints({ 0.f, 0.f, 0.f }, { 0.5f, 0.5f, 0.5f })));
    SphereShape sphere;
    sphere.radius = 0.3f;
    CapsuleShape capsule;
    capsule.radius = 0.2f;
    capsule.halfHeight = 0.5f;

    RigidBody h, s, c;
    h.setShape(&hull); h.setMass(0.f);
    s.setShape(&sphere); s.setMass(1.f);
    s.position = { 0.f, 0.75f, 0.f };
    ContactManifold m;
    ASSERT_TRUE(NarrowPhase::detect(s, h, m));
    ASSERT_EQ(m.contacts.size(), 1u);
    EXPECT_NEAR(m.contacts[0].normal.y, 1.f, kTol);      // pushes the sphere up
    EXPECT_NEAR(m.contacts[0].penetration, 0.05f, kTol);

    // Lying capsule on the hull's top: both caps touch.
    c.setShape(&capsule); c.setMass(1.f);
    c.position = { 0.f, 0.69f, 0.f };
    c.orientation = glm::angleAxis(glm::radians(-90.f), Vector3df(0.f, 0.f, 1.f));
    ASSERT_TRUE(NarrowPhase::detect(c, h, m));
    EXPECT_GE(m.contacts.size(), 2u);
    for (const auto& cp : m.contacts) EXPECT_NEAR(cp.normal.y, 1.f, kTol);
}

// ------------------------------------------------ solver ----------------

TEST(ConvexHullShapeTest, Solver_WedgeRestsOnFloorWithoutRocking) {
    RigidBodySolver world;
    world.timeStep = 1.f / 60.f;
    PlaneShape floorShape;
    RigidBody floor;
    floor.setShape(&floorShape);
    floor.setMass(0.f);
    world.addBody(&floor);

    ConvexHullShape hull;
    ASSERT_TRUE(hull.build(wedgePoints()));
    RigidBody wedge;
    wedge.setShape(&hull);
    wedge.setMass(1.f);
    // COM of the right-triangle prism is 1/3 up from its base.
    wedge.position = { 0.f, 1.f + 1.f / 3.f, 0.f };
    wedge.friction = 0.6f;
    world.addBody(&wedge);

    world.setRunning(true);
    for (int i = 0; i < 180; ++i) world.stepUnconditional();
    const Vector3df settled = wedge.position;
    for (int i = 0; i < 30; ++i) world.stepUnconditional();

    EXPECT_NEAR(wedge.position.y, 1.f / 3.f, 0.03f);
    const Vector3df up = wedge.orientation * Vector3df(0.f, 1.f, 0.f);
    EXPECT_GT(up.y, 0.995f);
    EXPECT_LT(glm::length(wedge.position - settled), 0.01f);
}

TEST(ConvexHullShapeTest, Solver_HullStacksOnHull) {
    RigidBodySolver world;
    world.timeStep = 1.f / 60.f;
    PlaneShape floorShape;
    RigidBody floor;
    floor.setShape(&floorShape);
    floor.setMass(0.f);
    world.addBody(&floor);

    ConvexHullShape cube;
    ASSERT_TRUE(cube.build(boxPoints({ 0.f, 0.f, 0.f }, { 0.5f, 0.5f, 0.5f })));
    RigidBody lower, upper;
    lower.setShape(&cube); lower.setMass(1.f);
    lower.position = { 0.f, 0.6f, 0.f };
    upper.setShape(&cube); upper.setMass(1.f);
    upper.position = { 0.1f, 1.8f, 0.05f };
    world.addBody(&lower);
    world.addBody(&upper);

    world.setRunning(true);
    for (int i = 0; i < 240; ++i) world.stepUnconditional();

    EXPECT_NEAR(lower.position.y, 0.5f, 0.03f);
    EXPECT_NEAR(upper.position.y, 1.5f, 0.05f);
    EXPECT_NEAR(upper.position.x, 0.1f, 0.05f);
    EXPECT_GT((upper.orientation * Vector3df(0.f, 1.f, 0.f)).y, 0.99f);
}
