#include "pch.h"
#include "../Physics/RigidBoundary.h"
#include "../Physics/RigidBody.h"
#include "../Physics/MeshBoundaryShape.h"
#include "CGLib/ThirdParty/glm-0.9.9.8/glm/gtc/quaternion.hpp"
#include <array>
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
constexpr float kTol = 1.0e-4f;
const Quaternion kIdentity(1.f, 0.f, 0.f, 0.f);
}

TEST(RigidBoundaryTest, NoShapeSynced_ForceIsZero) {
    RigidBoundary rb;
    Vector3df force = rb.getBoundaryForce({ 0.f, 0.f, 0.f });
    EXPECT_NEAR(getLength(force), 0.f, kTol);
}

TEST(RigidBoundaryTest, Sphere_ParticleOutside_ForceIsZero) {
    SphereShape sphere;
    sphere.radius = 0.5f;

    RigidBoundary rb;
    rb.setShape(&sphere);
    rb.syncKinematic({ 0.f, 0.f, 0.f }, kIdentity);

    Vector3df force = rb.getBoundaryForce({ 2.f, 0.f, 0.f });
    EXPECT_NEAR(getLength(force), 0.f, kTol);
}

TEST(RigidBoundaryTest, Sphere_ParticlePenetrating_ForceAlongNormal) {
    SphereShape sphere;
    sphere.radius = 0.5f;

    RigidBoundary rb;
    rb.setShape(&sphere);
    rb.setPenaltyStiffness(1000.f);
    rb.syncKinematic({ 0.f, 0.f, 0.f }, kIdentity);

    // particle at x=0.3 -> penetration depth = 0.5 - 0.3 = 0.2
    Vector3df force = rb.getBoundaryForce({ 0.3f, 0.f, 0.f });

    EXPECT_NEAR(force.x, 1000.f * 0.2f, kTol);
    EXPECT_NEAR(force.y, 0.f, kTol);
    EXPECT_NEAR(force.z, 0.f, kTol);
}

TEST(RigidBoundaryTest, Sphere_ForceScalesWithStiffness) {
    SphereShape sphere;
    sphere.radius = 1.f;

    RigidBoundary rb;
    rb.setShape(&sphere);
    rb.setPenaltyStiffness(200.f);
    rb.syncKinematic({ 0.f, 0.f, 0.f }, kIdentity);

    Vector3df force = rb.getBoundaryForce({ 0.f, 0.f, 0.f });  // penetration = radius = 1
    EXPECT_NEAR(getLength(force), 200.f, kTol);
}

TEST(RigidBoundaryTest, Box_ParticlePenetratingRotatedBox_ForceMatchesLocalFrame) {
    BoxShape box;
    box.halfExtents = { 0.5f, 1.f, 0.5f };

    RigidBoundary rb;
    rb.setShape(&box);
    rb.setPenaltyStiffness(500.f);
    Quaternion rot = glm::angleAxis(glm::radians(90.f), Vector3df(0.f, 0.f, 1.f));
    rb.syncKinematic({ 0.f, 0.f, 0.f }, rot);

    // World +X aligns with the box's local Y extent (1.0) after this rotation
    // (see ShapeSDFTest.Box_Rotated90AroundZ_*): particle at x=0.8 -> penetration 0.2.
    Vector3df force = rb.getBoundaryForce({ 0.8f, 0.f, 0.f });

    EXPECT_NEAR(force.x, 500.f * 0.2f, kTol);
    EXPECT_NEAR(force.y, 0.f, kTol);
    EXPECT_NEAR(force.z, 0.f, kTol);
}

TEST(RigidBoundaryTest, Sync_CopiesPoseAndVelocityFromRigidBody) {
    SphereShape sphere;
    sphere.radius = 0.5f;

    RigidBody body;
    body.setShape(&sphere);
    body.position = { 1.f, 2.f, 3.f };
    body.linearVelocity = { 0.f, -4.f, 0.f };

    RigidBoundary rb;
    rb.sync(body);

    // particle at the body's center is fully inside -> penetration == radius
    Vector3df force = rb.getBoundaryForce(body.position);
    EXPECT_NEAR(getLength(force), rb.getPenaltyStiffness() * sphere.radius, kTol);
    EXPECT_NEAR(rb.getVelocity().y, -4.f, kTol);
}

// internal design notes Phase 2 (#1): estimateStiffness()
// should reproduce the historical fixed default (5000.f) exactly at this
// codebase's default maxTimeStep/boundaryTimeStep (0.01f), so that calling it
// is a pure opt-in with zero behavior change for existing scenes.
TEST(RigidBoundaryTest, EstimateStiffness_RecoversHistoricalDefaultAtDefaultTimeStep) {
    const float stiffness = RigidBoundary::estimateStiffness(0.01f);
    EXPECT_NEAR(stiffness, 5000.f, 5000.f * 1.0e-4f);
}

TEST(RigidBoundaryTest, EstimateStiffness_ScalesAsInverseDtSquared) {
    const float s1 = RigidBoundary::estimateStiffness(0.01f);
    const float s2 = RigidBoundary::estimateStiffness(0.001f);
    EXPECT_NEAR(s2, s1 * 100.f, s1 * 100.f * 1.0e-4f);
}

TEST(RigidBoundaryTest, SetStiffnessFromScale_AppliesEstimateStiffness) {
    RigidBoundary rb;
    rb.setStiffnessFromScale(0.01f);
    EXPECT_NEAR(rb.getPenaltyStiffness(), 5000.f, 5000.f * 1.0e-4f);
}

// The core scale-invariance claim (see the internal design notes' scale-invariance
// analysis for the failure mode this avoids): pick dt the way DFSPH's
// CFL estimate would (0.4 * diameter / characteristic velocity, with the
// characteristic velocity taken as free-fall speed through one particle
// radius under gravity), then estimateStiffness(dt) should keep the penalty
// acceleration, at a *fixed relative* penetration depth (depth = 10% of
// radius), proportional to gravity across scales -- unlike a fixed absolute
// stiffness, which would make the same relative penetration produce a
// wildly different (accel/gravity) ratio at each scale.
TEST(RigidBoundaryTest, EstimateStiffness_KeepsPenaltyAccelProportionalToGravityAcrossScales) {
    SphereShape sphere;
    const float gravity = 9.8f;
    const float depthRatio = 0.1f;
    const float cflNumber = 0.4f;

    float firstRatio = -1.0f;
    for (float radius : {1.0f, 0.1f, 0.01f}) {
        sphere.radius = radius;

        const float characteristicVelocity = std::sqrt(2.0f * gravity * radius);
        const float dt = cflNumber * (radius * 2.0f) / characteristicVelocity;
        const float stiffness = RigidBoundary::estimateStiffness(dt);

        RigidBoundary rb;
        rb.setShape(&sphere);
        rb.setPenaltyStiffness(stiffness);
        rb.syncKinematic({ 0.f, 0.f, 0.f }, kIdentity);

        const float depth = depthRatio * radius;
        Vector3df force = rb.getBoundaryForce({ radius - depth, 0.f, 0.f });
        const float ratio = getLength(force) / gravity;

        if (firstRatio < 0.0f) {
            firstRatio = ratio;
        } else {
            EXPECT_NEAR(ratio, firstRatio, firstRatio * 0.01f)
                << "radius=" << radius << " accel/gravity should stay proportional across scales";
        }
    }
}

TEST(RigidBoundaryTest, Plane_ParticleBelow_PushesUpByPenetration) {
    PlaneShape plane;
    plane.normal = { 0.f, 1.f, 0.f };
    plane.offset = 0.f;

    RigidBoundary rb;
    rb.setShape(&plane);
    rb.setPenaltyStiffness(100.f);
    rb.syncKinematic({ 0.f, 0.f, 0.f }, kIdentity);

    Vector3df force = rb.getBoundaryForce({ 0.f, -0.1f, 0.f });
    EXPECT_NEAR(force.x, 0.f, kTol);
    EXPECT_NEAR(force.y, 100.f * 0.1f, kTol);
    EXPECT_NEAR(force.z, 0.f, kTol);
}

namespace {
// Closed unit box (half-extents h, centered at the origin) as 12 triangles,
// exercising MeshBoundaryShape through the same RigidBoundary::getBoundaryForce()
// surface the DFSPH/PBSPH/WCSPH solvers call per-particle -- see
// DFSPHSolver.cpp's rigidBoundaries_ loop.
std::vector<Triangle3df> buildUnitBoxTriangles(const Vector3df& h) {
    const Vector3df c[8] = {
        { -h.x, -h.y, -h.z }, {  h.x, -h.y, -h.z }, {  h.x,  h.y, -h.z }, { -h.x,  h.y, -h.z },
        { -h.x, -h.y,  h.z }, {  h.x, -h.y,  h.z }, {  h.x,  h.y,  h.z }, { -h.x,  h.y,  h.z },
    };
    auto tri = [](const Vector3df& a, const Vector3df& b, const Vector3df& cc) {
        return Triangle3df(std::array<Vector3df, 3>{ a, b, cc });
    };
    return {
        tri(c[0], c[1], c[2]), tri(c[0], c[2], c[3]),
        tri(c[4], c[6], c[5]), tri(c[4], c[7], c[6]),
        tri(c[0], c[5], c[1]), tri(c[0], c[4], c[5]),
        tri(c[3], c[2], c[6]), tri(c[3], c[6], c[7]),
        tri(c[0], c[3], c[7]), tri(c[0], c[7], c[4]),
        tri(c[1], c[5], c[6]), tri(c[1], c[6], c[2]),
    };
}
// Voxelized SDF accuracy is bounded by voxelSize, not float epsilon.
constexpr float kMeshTol = 0.06f;
}

TEST(RigidBoundaryTest, Mesh_ParticleOutside_ForceIsZero) {
    MeshBoundaryShape mesh(buildUnitBoxTriangles({ 0.5f, 0.5f, 0.5f }), 0.1f);

    RigidBoundary rb;
    rb.setShape(&mesh);
    rb.syncKinematic({ 0.f, 0.f, 0.f }, kIdentity);

    Vector3df force = rb.getBoundaryForce({ 2.f, 0.f, 0.f });
    EXPECT_NEAR(getLength(force), 0.f, kMeshTol);
}

TEST(RigidBoundaryTest, Mesh_ParticlePenetrating_ForceAlongNormal) {
    MeshBoundaryShape mesh(buildUnitBoxTriangles({ 0.5f, 0.5f, 0.5f }), 0.1f);

    RigidBoundary rb;
    rb.setShape(&mesh);
    rb.setPenaltyStiffness(1000.f);
    rb.syncKinematic({ 0.f, 0.f, 0.f }, kIdentity);

    // particle at x=0.3 -> penetration depth = 0.5 - 0.3 = 0.2 (mirrors
    // RigidBoundaryTest.Sphere_ParticlePenetrating_ForceAlongNormal above).
    Vector3df force = rb.getBoundaryForce({ 0.3f, 0.f, 0.f });

    EXPECT_NEAR(force.x, 1000.f * 0.2f, 1000.f * kMeshTol);
    EXPECT_GT(force.x, 0.f);
    EXPECT_NEAR(force.y, 0.f, 1000.f * kMeshTol);
    EXPECT_NEAR(force.z, 0.f, 1000.f * kMeshTol);
}
