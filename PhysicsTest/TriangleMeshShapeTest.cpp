#include "pch.h"
#include "../Physics/TriangleMeshShape.h"
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

// A flat grid in the XZ plane at height y, `n` x `n` quads, 2 triangles each.
void makeGrid(int n, float halfSize, float y,
              std::vector<Vector3df>& verts, std::vector<uint32_t>& indices) {
    verts.clear();
    indices.clear();
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i)
            verts.emplace_back(-halfSize + 2.f * halfSize * i / n, y, -halfSize + 2.f * halfSize * j / n);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const uint32_t a = j * (n + 1) + i, b = a + 1, c = a + (n + 1), d = c + 1;
            indices.insert(indices.end(), { a, c, b, b, c, d });
        }
}

struct World {
    RigidBodySolver solver;
    TriangleMeshShape meshShape;
    RigidBody mesh;
    World(int gridN, float halfSize) {
        solver.timeStep = 1.f / 60.f;
        std::vector<Vector3df> v;
        std::vector<uint32_t> idx;
        makeGrid(gridN, halfSize, 0.f, v, idx);
        meshShape.build(v, idx);
        mesh.setShape(&meshShape);
        mesh.setMass(0.f);
        solver.addBody(&mesh);
        solver.setRunning(true);
    }
    void run(int steps) { for (int i = 0; i < steps; ++i) solver.stepUnconditional(); }
};
} // namespace

TEST(TriangleMeshShapeTest, Build_DropsDegenerateAndOutOfRangeTriangles) {
    TriangleMeshShape mesh;
    const std::vector<Vector3df> v = { { 0.f, 0.f, 0.f }, { 1.f, 0.f, 0.f }, { 0.f, 0.f, 1.f }, { 2.f, 0.f, 0.f } };
    ASSERT_TRUE(mesh.build(v, { 0, 2, 1,   0, 1, 3,   0, 1, 9 }));
    EXPECT_EQ(mesh.getTriangleCount(), 1u); // collinear (0,1,3) and out-of-range (9) dropped
    EXPECT_FALSE(mesh.build(v, { 0, 1, 3 }));
}

TEST(TriangleMeshShapeTest, QueryTriangles_OnlyOverlappingOnes) {
    std::vector<Vector3df> v;
    std::vector<uint32_t> idx;
    makeGrid(10, 5.f, 0.f, v, idx);
    TriangleMeshShape mesh;
    ASSERT_TRUE(mesh.build(v, idx));
    EXPECT_EQ(mesh.getTriangleCount(), 200u);
    std::vector<int> hits;
    mesh.queryTriangles(Box3df({ 0.1f, -0.1f, 0.1f }, { 0.9f, 0.1f, 0.9f }), hits);
    EXPECT_EQ(hits.size(), 2u); // one grid cell
    mesh.queryTriangles(Box3df({ 0.1f, 0.5f, 0.1f }, { 0.9f, 1.f, 0.9f }), hits);
    EXPECT_TRUE(hits.empty());
}

TEST(TriangleMeshShapeTest, SetMass_AlwaysStatic) {
    std::vector<Vector3df> v;
    std::vector<uint32_t> idx;
    makeGrid(1, 1.f, 0.f, v, idx);
    TriangleMeshShape mesh;
    ASSERT_TRUE(mesh.build(v, idx));
    RigidBody body;
    body.setShape(&mesh);
    body.setMass(5.f);
    EXPECT_TRUE(body.isStatic());
}

TEST(TriangleMeshShapeTest, DistanceIsUnsignedAndTwoSided) {
    std::vector<Vector3df> v;
    std::vector<uint32_t> idx;
    makeGrid(2, 1.f, 0.f, v, idx);
    TriangleMeshShape mesh;
    ASSERT_TRUE(mesh.build(v, idx));
    const Quaternion id(1.f, 0.f, 0.f, 0.f);
    EXPECT_NEAR(mesh.getSignedDistance({ 0.2f, 0.5f, 0.1f }, { 0.f, 0.f, 0.f }, id), 0.5f, kTol);
    EXPECT_NEAR(mesh.getSignedDistance({ 0.2f, -0.5f, 0.1f }, { 0.f, 0.f, 0.f }, id), 0.5f, kTol);
    EXPECT_NEAR(mesh.getSurfaceNormal({ 0.2f, -0.5f, 0.1f }, { 0.f, 0.f, 0.f }, id).y, -1.f, kTol);
}

TEST(TriangleMeshShapeTest, SphereOnMesh_ContactPushesUp) {
    std::vector<Vector3df> v;
    std::vector<uint32_t> idx;
    makeGrid(4, 2.f, 0.f, v, idx);
    TriangleMeshShape meshShape;
    ASSERT_TRUE(meshShape.build(v, idx));
    SphereShape sphereShape;
    sphereShape.radius = 0.5f;
    RigidBody mesh, sphere;
    mesh.setShape(&meshShape); mesh.setMass(0.f);
    sphere.setShape(&sphereShape); sphere.setMass(1.f);
    sphere.position = { 1.f, 0.45f, 1.f }; // exactly over a grid vertex shared by 6 triangles

    ContactManifold m;
    ASSERT_TRUE(NarrowPhase::detect(sphere, mesh, m));
    ASSERT_EQ(m.contacts.size(), 1u); // the shared vertex is reported once
    EXPECT_NEAR(m.contacts[0].normal.y, 1.f, kTol);
    EXPECT_NEAR(m.contacts[0].penetration, 0.05f, kTol);

    ContactManifold flipped;
    ASSERT_TRUE(NarrowPhase::detect(mesh, sphere, flipped));
    EXPECT_NEAR(flipped.contacts[0].normal.y, -1.f, kTol);
}

TEST(TriangleMeshShapeTest, Solver_SphereRestsOnTwoTriangleFloor) {
    World w(1, 5.f);
    SphereShape s;
    s.radius = 0.3f;
    RigidBody ball;
    ball.setShape(&s); ball.setMass(1.f);
    ball.position = { 0.7f, 2.f, -0.4f };
    w.solver.addBody(&ball);
    w.run(180);
    EXPECT_NEAR(ball.position.y, 0.3f, 0.03f);
    EXPECT_NEAR(ball.position.x, 0.7f, 0.01f);
}

TEST(TriangleMeshShapeTest, Solver_BoxRestsFlatOnTessellatedFloor) {
    World w(16, 4.f); // cells of 0.5, so the box spans several triangles
    BoxShape boxShape;
    boxShape.halfExtents = { 0.6f, 0.3f, 0.45f };
    RigidBody box;
    box.setShape(&boxShape); box.setMass(2.f);
    box.position = { 0.13f, 1.f, -0.27f };
    box.friction = 0.6f;
    w.solver.addBody(&box);
    w.run(180);
    const Vector3df settled = box.position;
    w.run(30);
    EXPECT_NEAR(box.position.y, 0.3f, 0.03f);
    EXPECT_GT((box.orientation * Vector3df(0.f, 1.f, 0.f)).y, 0.995f);
    EXPECT_LT(glm::length(box.position - settled), 0.01f);
}

TEST(TriangleMeshShapeTest, Solver_HullAndCapsuleRestOnFloor) {
    World w(8, 4.f);
    ConvexHullShape hullShape;
    std::vector<Vector3df> pts;
    for (int i = 0; i < 8; ++i)
        pts.emplace_back((i & 1) ? 0.4f : -0.4f, (i & 2) ? 0.4f : -0.4f, (i & 4) ? 0.4f : -0.4f);
    ASSERT_TRUE(hullShape.build(pts));
    RigidBody hull;
    hull.setShape(&hullShape); hull.setMass(1.f);
    hull.position = { -1.3f, 1.f, 0.2f };
    w.solver.addBody(&hull);

    CapsuleShape capShape;
    capShape.radius = 0.2f;
    capShape.halfHeight = 0.5f;
    RigidBody cap;
    cap.setShape(&capShape); cap.setMass(1.f);
    cap.position = { 1.2f, 1.f, 0.3f };
    cap.orientation = glm::angleAxis(glm::radians(-90.f), Vector3df(0.f, 0.f, 1.f));
    cap.updateInertiaTensor();
    cap.friction = 0.6f;
    w.solver.addBody(&cap);

    w.run(200);
    EXPECT_NEAR(hull.position.y, 0.4f, 0.03f);
    EXPECT_GT((hull.orientation * Vector3df(0.f, 1.f, 0.f)).y, 0.995f);
    EXPECT_NEAR(cap.position.y, 0.2f, 0.03f);
    EXPECT_LT(std::abs((cap.orientation * Vector3df(0.f, 1.f, 0.f)).y), 0.05f);
}

TEST(TriangleMeshShapeTest, Solver_FlippedMeshIsStillSolid) {
    // The mesh body rotated upside down: its triangles now face -Y, but they are
    // two-sided, so a sphere dropped from above still lands on them.
    World w(2, 3.f);
    w.mesh.orientation = glm::angleAxis(glm::radians(180.f), Vector3df(1.f, 0.f, 0.f));
    w.mesh.position = { 0.f, 1.f, 0.f };
    SphereShape s;
    s.radius = 0.25f;
    RigidBody ball;
    ball.setShape(&s); ball.setMass(1.f);
    ball.position = { 0.3f, 2.5f, 0.3f };
    w.solver.addBody(&ball);
    w.run(180);
    EXPECT_NEAR(ball.position.y, 1.25f, 0.03f);
}

TEST(TriangleMeshShapeTest, Solver_SphereRollsDownRamp) {
    // A ramp rising along -X (slope 0.5): the ball ends up further along +X.
    RigidBodySolver solver;
    solver.timeStep = 1.f / 60.f;
    const std::vector<Vector3df> v = { { -3.f, 1.5f, -1.f }, { 3.f, -1.5f, -1.f },
                                       { -3.f, 1.5f,  1.f }, { 3.f, -1.5f,  1.f } };
    TriangleMeshShape rampShape;
    ASSERT_TRUE(rampShape.build(v, { 0, 2, 1, 1, 2, 3 }));
    RigidBody ramp;
    ramp.setShape(&rampShape); ramp.setMass(0.f);
    solver.addBody(&ramp);
    SphereShape s;
    s.radius = 0.2f;
    RigidBody ball;
    ball.setShape(&s); ball.setMass(1.f);
    ball.position = { -2.f, 1.3f, 0.f };
    solver.addBody(&ball);
    solver.setRunning(true);
    for (int i = 0; i < 60; ++i) solver.stepUnconditional();
    EXPECT_GT(ball.position.x, -1.5f);
    // Still on the ramp surface (not fallen through): height above the plane ~ radius.
    const Vector3df n = glm::normalize(Vector3df(0.5f, 1.f, 0.f));
    EXPECT_NEAR(glm::dot(ball.position, n), 0.2f, 0.05f);
}
