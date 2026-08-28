#include "pch.h"
#include "../Physics/RigidBoundaryParticles.h"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
constexpr float kTol = 1.0e-4f;
}

TEST(RigidBoundaryParticlesTest, Sample_Sphere_ProducesPointsOnSurface) {
    SphereShape sphere;
    sphere.radius = 1.0f;

    RigidBoundaryParticles rb;
    rb.sample(sphere, 0.2f);

    ASSERT_FALSE(rb.localPositions().empty());
    for (auto& localPos : rb.localPositions()) {
        EXPECT_NEAR(getLength(localPos), sphere.radius, 1.0e-3f);
    }
}

TEST(RigidBoundaryParticlesTest, Sample_Box_ProducesPointsOnFaces) {
    BoxShape box;
    box.halfExtents = { 0.5f, 1.f, 1.5f };

    RigidBoundaryParticles rb;
    rb.sample(box, 0.25f);

    ASSERT_FALSE(rb.localPositions().empty());
    for (auto& localPos : rb.localPositions()) {
        const float ex = std::abs(std::abs(localPos.x) - box.halfExtents.x);
        const float ey = std::abs(std::abs(localPos.y) - box.halfExtents.y);
        const float ez = std::abs(std::abs(localPos.z) - box.halfExtents.z);
        // Every sample must lie exactly on (at least) one of the 6 faces.
        EXPECT_TRUE(ex < 1.0e-4f || ey < 1.0e-4f || ez < 1.0e-4f);
    }
}

TEST(RigidBoundaryParticlesTest, Sample_Plane_ProducesNoParticles) {
    PlaneShape plane;

    RigidBoundaryParticles rb;
    rb.sample(plane, 0.2f);

    EXPECT_TRUE(rb.particles().empty());
}

TEST(RigidBoundaryParticlesTest, ComputePsi_YieldsPositiveValues) {
    SphereShape sphere;
    sphere.radius = 0.5f;

    RigidBoundaryParticles rb;
    rb.sample(sphere, 0.15f);

    SPHKernel kernel(0.3f);
    rb.computePsi(kernel, 1000.f);

    ASSERT_FALSE(rb.particles().empty());
    for (auto& bp : rb.particles()) {
        EXPECT_GT(bp.psi, 0.f);
    }
}

TEST(RigidBoundaryParticlesTest, Sync_TransformsLocalToWorld) {
    SphereShape sphere;
    sphere.radius = 0.5f;

    RigidBoundaryParticles rb;
    rb.sample(sphere, 0.2f);

    const Vector3df pos(1.f, 2.f, 3.f);
    const Quaternion identity(1.f, 0.f, 0.f, 0.f);
    rb.sync(pos, identity);

    ASSERT_EQ(rb.particles().size(), rb.localPositions().size());
    for (size_t i = 0; i < rb.particles().size(); ++i) {
        const Vector3df expected = pos + rb.localPositions()[i];
        EXPECT_NEAR(getDistance(rb.particles()[i].worldPos, expected), 0.f, kTol);
    }
}

TEST(RigidBoundaryParticlesTest, ClearAccumForce_ZeroesAllParticles) {
    SphereShape sphere;
    sphere.radius = 0.5f;

    RigidBoundaryParticles rb;
    rb.sample(sphere, 0.2f);
    for (auto& bp : rb.particles()) bp.accumForce = Vector3df(1.f, 2.f, 3.f);

    rb.clearAccumForce();

    for (auto& bp : rb.particles()) {
        EXPECT_NEAR(getLength(bp.accumForce), 0.f, kTol);
    }
}
